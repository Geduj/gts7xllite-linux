// SPDX-License-Identifier: GPL-2.0-only
/*
 * Silicon Mitus SM5714 switching charger (MFD cell "sm5714-charger")
 *
 * Step 1: enable sequence at probe + power_supply with writable limits.
 * Step 2a: optional "vbuspok"/"vbusuvlo"/"chgon"/"done" IRQs from the MFD
 *          regmap_irq chip -> re-enable after a bad handoff +
 *          power_supply_changed().
 * Step 2b: input limit = max(Type-C Rp, BC1.2), 500 mA if neither says more.
 *          Rp: supplier CURRENT_MAX (sm5714-typec). BC1.2: MUIC DEVICETYPE1,
 *          re-read on the MUIC "chgtype"/"muic-detach" IRQs.
 * Step J-3: JEITA. Battery temperature (sm5714-battery TEMP, the PM6150L
 *          NTC) every 30 s while VBUS is present, Samsung gts7xllite bands
 *          with hysteresis: stop / reduced current / reduced float voltage.
 * Charge limit: sm5714-battery charge_control_{start,end}_threshold, checked
 *          with JEITA; at the end threshold the battery path (ENQ4FET) is
 *          switched off and the tablet runs from the cable.
 * No WDT, no AFC, no SM5440 direct-charging handoff yet.
 */
#include <linux/bits.h>
#include <linux/interrupt.h>
#include <linux/mfd/sm5714.h>
#include <linux/minmax.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/workqueue.h>

#define SM5714_CHG_STATUS1		0x0d
#define  SM5714_STATUS1_VBUSPOK		BIT(0)
#define  SM5714_STATUS1_VBUSUVLO	BIT(1)
#define  SM5714_STATUS1_VBUSOVP		BIT(2)
#define  SM5714_STATUS1_VBUSLIMIT	BIT(3)
#define SM5714_CHG_STATUS2		0x0e
#define  SM5714_STATUS2_AICL		BIT(0)
#define  SM5714_STATUS2_CHGON		BIT(3)
#define  SM5714_STATUS2_Q4FULLON	BIT(4)
#define  SM5714_STATUS2_TOPOFF		BIT(5)
#define  SM5714_STATUS2_DONE		BIT(6)
#define  SM5714_STATUS2_WDT_EXP		BIT(7)
#define SM5714_CHG_CNTL1		0x13
#define  SM5714_CNTL1_ENQ4FET		BIT(3)
#define SM5714_CHG_CNTL2		0x14
#define  SM5714_CNTL2_OP_MODE		GENMASK(3, 0)
#define  SM5714_OP_MODE_USB_OTG		0x7
#define SM5714_CHG_VBUSCNTL		0x15
#define  SM5714_VBUSCNTL_INPUT_LIMIT	GENMASK(6, 0)
#define SM5714_CHG_CHGCNTL2		0x18
#define SM5714_CHG_CHGCNTL4		0x1a
#define  SM5714_CHGCNTL4_BATREG		GENMASK(5, 0)	/* float voltage */

/* MUIC (0x25) BC1.2 result */
#define SM5714_MUIC_DEVICETYPE1		0x07
#define  SM5714_DEVTYPE1_U200		BIT(4)	/* Samsung 2 A TA */
#define  SM5714_DEVTYPE1_CDP		BIT(3)
#define  SM5714_DEVTYPE1_DCP		BIT(2)
#define  SM5714_DEVTYPE1_SDP		BIT(1)
#define SM5714_BC12_CHG_UA		1500000	/* BC1.2 DCP/CDP limit */

/* Input limit: 0x15[6:0] = (uA - 100 mA) / 25 mA */
#define SM5714_ICL_MIN_UA		100000
#define SM5714_ICL_STEP_UA		25000
#define SM5714_ICL_MAX_UA		3000000	/* sec-battery max input */
#define SM5714_ICL_SAFE_UA		500000

/* Fast charge: 0x18 = 7 + (uA - 109375) / 15625, 0x07 = min */
#define SM5714_CCC_MIN_UA		109375
#define SM5714_CCC_STEP_UA		15625
#define SM5714_CCC_REG_MIN		0x07
#define SM5714_CCC_MAX_UA		3200000	/* sec-battery max charge */
#define SM5714_CCC_NO_TEMP_UA		1500000	/* cap while temp is unknown */

/* Float (BATREG) 4.05-4.62 V: offset = (mV - 4050) / 10 + 6 */
#define SM5714_FLOAT_NORMAL_MV		4380	/* sec-battery chg_float_voltage */
#define SM5714_FLOAT_SWELL_MV		4150	/* swelling (warm) float */

#define SM5714_JEITA_POLL_MS		30000

/* Test hook: pretend the battery is at this temperature (0.1 degC) */
#define SM5714_JEITA_TEMP_OFF		9999
static int jeita_temp = SM5714_JEITA_TEMP_OFF;
module_param(jeita_temp, int, 0644);
MODULE_PARM_DESC(jeita_temp, "test only: battery temperature override in 0.1 degC (9999 = off)");

enum sm5714_jeita_zone {
	SM5714_JEITA_UNKNOWN,	/* no temperature yet */
	SM5714_JEITA_COLD,	/* < 0 C: stop */
	SM5714_JEITA_COOL3,	/* 0..5 C */
	SM5714_JEITA_COOL2,	/* 5..15 C */
	SM5714_JEITA_NORMAL,	/* 15..41 C */
	SM5714_JEITA_WARM,	/* 41..50 C: swelling float */
	SM5714_JEITA_HOT,	/* > 50 C: stop */
};

static const struct {
	const char *name;
	u32 ccc_max_ua;		/* 0 = charging off */
	u32 float_mv;
	int health;
} sm5714_jeita[] = {
	[SM5714_JEITA_UNKNOWN] = { "unknown", SM5714_CCC_NO_TEMP_UA, SM5714_FLOAT_NORMAL_MV, POWER_SUPPLY_HEALTH_GOOD },
	[SM5714_JEITA_COLD]    = { "cold",    0,                     SM5714_FLOAT_NORMAL_MV, POWER_SUPPLY_HEALTH_COLD },
	[SM5714_JEITA_COOL3]   = { "cool3",   900000,                SM5714_FLOAT_NORMAL_MV, POWER_SUPPLY_HEALTH_COOL },
	[SM5714_JEITA_COOL2]   = { "cool2",   2700000,               SM5714_FLOAT_NORMAL_MV, POWER_SUPPLY_HEALTH_COOL },
	[SM5714_JEITA_NORMAL]  = { "normal",  SM5714_CCC_MAX_UA,     SM5714_FLOAT_NORMAL_MV, POWER_SUPPLY_HEALTH_GOOD },
	[SM5714_JEITA_WARM]    = { "warm",    SM5714_CCC_MAX_UA,     SM5714_FLOAT_SWELL_MV,  POWER_SUPPLY_HEALTH_WARM },
	[SM5714_JEITA_HOT]     = { "hot",     0,                     SM5714_FLOAT_NORMAL_MV, POWER_SUPPLY_HEALTH_OVERHEAT },
};

struct sm5714_charger {
	struct device *dev;
	struct regmap *regmap;
	struct regmap *regmap_muic;
	struct power_supply *psy;
	struct mutex lock;	/* enable sequence vs. input limit updates */
	u32 icl_ua;	/* requested input limit */
	u32 ccc_ua;	/* requested charge current (before JEITA) */
	struct power_supply *bat;	/* sm5714-battery, for TEMP */
	struct delayed_work jeita_work;
	enum sm5714_jeita_zone zone;
	int temp;			/* last battery temperature, 0.1 degC */
	bool limit_stop;		/* charge limit (end threshold) reached */
};

/* Charge current actually programmed: request capped by the JEITA zone */
static u32 sm5714_chg_ccc_eff(struct sm5714_charger *chg)
{
	return min(chg->ccc_ua, sm5714_jeita[chg->zone].ccc_max_ua);
}

/* Charging allowed: JEITA zone permits it and the charge limit is not hit */
static bool sm5714_chg_jeita_allows(struct sm5714_charger *chg)
{
	return sm5714_jeita[chg->zone].ccc_max_ua != 0 && !chg->limit_stop;
}

static int sm5714_chg_set_icl(struct sm5714_charger *chg, u32 ua)
{
	unsigned int val;

	ua = clamp_t(u32, ua, SM5714_ICL_MIN_UA, SM5714_ICL_MAX_UA);
	val = (ua - SM5714_ICL_MIN_UA) / SM5714_ICL_STEP_UA;

	return regmap_update_bits(chg->regmap, SM5714_CHG_VBUSCNTL,
				  SM5714_VBUSCNTL_INPUT_LIMIT, val);
}

static int sm5714_chg_get_icl(struct sm5714_charger *chg, int *ua)
{
	unsigned int val;
	int ret;

	ret = regmap_read(chg->regmap, SM5714_CHG_VBUSCNTL, &val);
	if (ret)
		return ret;

	val &= SM5714_VBUSCNTL_INPUT_LIMIT;
	*ua = SM5714_ICL_MIN_UA + val * SM5714_ICL_STEP_UA;
	return 0;
}

static int sm5714_chg_set_ccc(struct sm5714_charger *chg, u32 ua)
{
	unsigned int val;

	ua = clamp_t(u32, ua, SM5714_CCC_MIN_UA, SM5714_CCC_MAX_UA);
	val = SM5714_CCC_REG_MIN + (ua - SM5714_CCC_MIN_UA) / SM5714_CCC_STEP_UA;

	return regmap_write(chg->regmap, SM5714_CHG_CHGCNTL2, val);
}

static int sm5714_chg_set_float(struct sm5714_charger *chg, u32 mv)
{
	unsigned int val;

	mv = clamp_t(u32, mv, 4050, 4620);
	val = (mv - 4050) / 10 + 6;
	return regmap_update_bits(chg->regmap, SM5714_CHG_CHGCNTL4,
				  SM5714_CHGCNTL4_BATREG, val);
}

static int sm5714_chg_get_ccc(struct sm5714_charger *chg, int *ua)
{
	unsigned int val;
	int ret;

	ret = regmap_read(chg->regmap, SM5714_CHG_CHGCNTL2, &val);
	if (ret)
		return ret;

	if (val < SM5714_CCC_REG_MIN)
		val = SM5714_CCC_REG_MIN;
	*ua = SM5714_CCC_MIN_UA + (val - SM5714_CCC_REG_MIN) * SM5714_CCC_STEP_UA;
	return 0;
}

/*
 * Samsung order: input 500 mA -> Q4 FET on -> charge current -> input limit.
 * Keeps the adapter from seeing a step while the battery path opens.
 */
static int sm5714_chg_enable(struct sm5714_charger *chg)
{
	unsigned int st1, cntl2;
	int ret;

	ret = regmap_read(chg->regmap, SM5714_CHG_STATUS1, &st1);
	if (ret)
		return ret;
	if (!(st1 & SM5714_STATUS1_VBUSPOK)) {
		dev_info(chg->dev, "no VBUS, not enabling charging\n");
		return 0;
	}

	/* OTG boost (regulator cell) owns OP_MODE 7; stay out of its way */
	ret = regmap_read(chg->regmap, SM5714_CHG_CNTL2, &cntl2);
	if (ret)
		return ret;
	if ((cntl2 & SM5714_CNTL2_OP_MODE) == SM5714_OP_MODE_USB_OTG) {
		dev_info(chg->dev, "OTG active, not enabling charging\n");
		return 0;
	}

	if (!sm5714_chg_jeita_allows(chg)) {
		dev_info(chg->dev, "%s, not enabling charging\n",
			 chg->limit_stop ? "charge limit reached" :
			 sm5714_jeita[chg->zone].name);
		return 0;
	}

	ret = sm5714_chg_set_icl(chg, SM5714_ICL_SAFE_UA);
	if (ret)
		return ret;

	ret = sm5714_chg_set_float(chg, sm5714_jeita[chg->zone].float_mv);
	if (ret)
		return ret;

	ret = regmap_set_bits(chg->regmap, SM5714_CHG_CNTL1,
			      SM5714_CNTL1_ENQ4FET);
	if (ret)
		return ret;

	ret = sm5714_chg_set_ccc(chg, sm5714_chg_ccc_eff(chg));
	if (ret)
		return ret;

	ret = sm5714_chg_set_icl(chg, chg->icl_ua);
	if (ret)
		return ret;

	dev_info(chg->dev, "charging enabled: input %u uA, charge %u uA (JEITA %s)\n",
		 chg->icl_ua, sm5714_chg_ccc_eff(chg),
		 sm5714_jeita[chg->zone].name);
	return 0;
}

static int sm5714_chg_get_status(struct sm5714_charger *chg, int *status)
{
	unsigned int st1, st2;
	int ret;

	ret = regmap_read(chg->regmap, SM5714_CHG_STATUS1, &st1);
	if (ret)
		return ret;
	ret = regmap_read(chg->regmap, SM5714_CHG_STATUS2, &st2);
	if (ret)
		return ret;

	if (!(st1 & SM5714_STATUS1_VBUSPOK))
		*status = POWER_SUPPLY_STATUS_DISCHARGING;
	else if (st2 & SM5714_STATUS2_DONE)
		*status = POWER_SUPPLY_STATUS_FULL;
	else if (st2 & SM5714_STATUS2_CHGON)
		*status = POWER_SUPPLY_STATUS_CHARGING;
	else
		*status = POWER_SUPPLY_STATUS_NOT_CHARGING;

	return 0;
}

static int sm5714_chg_get_property(struct power_supply *psy,
				   enum power_supply_property psp,
				   union power_supply_propval *val)
{
	struct sm5714_charger *chg = power_supply_get_drvdata(psy);
	unsigned int reg;
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		ret = regmap_read(chg->regmap, SM5714_CHG_STATUS1, &reg);
		if (ret)
			return ret;
		val->intval = !!(reg & SM5714_STATUS1_VBUSPOK);
		return 0;
	case POWER_SUPPLY_PROP_STATUS:
		return sm5714_chg_get_status(chg, &val->intval);
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		ret = regmap_read(chg->regmap, SM5714_CHG_STATUS2, &reg);
		if (ret)
			return ret;
		val->intval = (reg & SM5714_STATUS2_CHGON) ?
			      POWER_SUPPLY_CHARGE_TYPE_FAST :
			      POWER_SUPPLY_CHARGE_TYPE_NONE;
		return 0;
	case POWER_SUPPLY_PROP_HEALTH:
		ret = regmap_read(chg->regmap, SM5714_CHG_STATUS1, &reg);
		if (ret)
			return ret;
		val->intval = (reg & SM5714_STATUS1_VBUSOVP) ?
			      POWER_SUPPLY_HEALTH_OVERVOLTAGE :
			      sm5714_jeita[READ_ONCE(chg->zone)].health;
		return 0;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		return sm5714_chg_get_icl(chg, &val->intval);
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		return sm5714_chg_get_ccc(chg, &val->intval);
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		val->intval = SM5714_CCC_MAX_UA;
		return 0;
	default:
		return -EINVAL;
	}
}

static int sm5714_chg_set_property(struct power_supply *psy,
				   enum power_supply_property psp,
				   const union power_supply_propval *val)
{
	struct sm5714_charger *chg = power_supply_get_drvdata(psy);
	int ret;

	if (val->intval < 0)
		return -EINVAL;

	mutex_lock(&chg->lock);
	switch (psp) {
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		/* manual override, until the next source change */
		ret = sm5714_chg_set_icl(chg, val->intval);
		if (!ret)
			chg->icl_ua = val->intval;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		/* request; the JEITA zone still caps what gets programmed */
		chg->ccc_ua = min_t(u32, val->intval, SM5714_CCC_MAX_UA);
		ret = sm5714_chg_set_ccc(chg, sm5714_chg_ccc_eff(chg));
		break;
	default:
		ret = -EINVAL;
		break;
	}
	mutex_unlock(&chg->lock);
	return ret;
}

/* BC1.2 result from the MUIC; 0 = nothing better than USB 2.0 default */
static u32 sm5714_chg_bc12_ua(struct sm5714_charger *chg, unsigned int *dt1)
{
	if (regmap_read(chg->regmap_muic, SM5714_MUIC_DEVICETYPE1, dt1)) {
		*dt1 = 0;
		return 0;
	}

	if (*dt1 & (SM5714_DEVTYPE1_DCP | SM5714_DEVTYPE1_CDP |
		    SM5714_DEVTYPE1_U200))
		return SM5714_BC12_CHG_UA;
	return 0;
}

/*
 * The one place that decides the input limit: max(Rp, BC1.2), at least the
 * safe 500 mA, at most 3 A. Called on supplier changes (Rp) and on every
 * charger/MUIC IRQ. AFC/PD and SM5440 direct charging hook in here later.
 * @psy is passed in because the first call can come before
 * devm_power_supply_register() has returned (chg->psy not set yet).
 */
static void sm5714_chg_update_icl(struct sm5714_charger *chg,
				  struct power_supply *psy)
{
	union power_supply_propval val;
	u32 rp_ua = 0, bc12_ua, ua;
	unsigned int dt1;
	int ret;

	if (!power_supply_get_property_from_supplier(psy,
			POWER_SUPPLY_PROP_CURRENT_MAX, &val) && val.intval > 0)
		rp_ua = val.intval;
	bc12_ua = sm5714_chg_bc12_ua(chg, &dt1);

	ua = clamp_t(u32, max(rp_ua, bc12_ua), SM5714_ICL_SAFE_UA,
		     SM5714_ICL_MAX_UA);

	mutex_lock(&chg->lock);
	if (ua != chg->icl_ua) {
		/*
		 * Battery path already open: a plain limit change is fine.
		 * If it is still closed, the next sm5714_chg_enable() (plug IRQ)
		 * ramps 500 mA -> icl_ua in Samsung order.
		 */
		ret = sm5714_chg_set_icl(chg, ua);
		if (!ret) {
			dev_info(chg->dev,
				 "input limit %u -> %u uA (Rp %u, BC1.2 %u, DEVTYPE1 0x%02x)\n",
				 chg->icl_ua, ua, rp_ua, bc12_ua, dt1);
			chg->icl_ua = ua;
		}
	}
	mutex_unlock(&chg->lock);
}

/* A supplier (sm5714-typec, supplied_to = "sm5714-charger") changed */
static void sm5714_chg_external_power_changed(struct power_supply *psy)
{
	struct sm5714_charger *chg = power_supply_get_drvdata(psy);

	sm5714_chg_update_icl(chg, psy);
	power_supply_changed(psy);
}

static int sm5714_chg_property_is_writeable(struct power_supply *psy,
					    enum power_supply_property psp)
{
	return psp == POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT ||
	       psp == POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT;
}

static const enum power_supply_property sm5714_chg_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
};

static const struct power_supply_desc sm5714_chg_desc = {
	.name = "sm5714-charger",
	.type = POWER_SUPPLY_TYPE_USB,
	.properties = sm5714_chg_props,
	.num_properties = ARRAY_SIZE(sm5714_chg_props),
	.get_property = sm5714_chg_get_property,
	.set_property = sm5714_chg_set_property,
	.property_is_writeable = sm5714_chg_property_is_writeable,
	.external_power_changed = sm5714_chg_external_power_changed,
};

/* Next zone with Samsung's hysteresis (0.1 degC); one step at a time */
static enum sm5714_jeita_zone sm5714_jeita_next(enum sm5714_jeita_zone z, int t)
{
	int i;

	if (z == SM5714_JEITA_UNKNOWN) {
		if (t < 0)
			return SM5714_JEITA_COLD;
		if (t < 50)
			return SM5714_JEITA_COOL3;
		if (t < 150)
			return SM5714_JEITA_COOL2;
		if (t <= 410)
			return SM5714_JEITA_NORMAL;
		if (t <= 500)
			return SM5714_JEITA_WARM;
		return SM5714_JEITA_HOT;
	}

	for (i = 0; i < ARRAY_SIZE(sm5714_jeita); i++) {
		switch (z) {
		case SM5714_JEITA_COLD:
			if (t < 20)
				return z;
			z = SM5714_JEITA_COOL3;
			break;
		case SM5714_JEITA_COOL3:
			if (t < 0)
				z = SM5714_JEITA_COLD;
			else if (t >= 70)
				z = SM5714_JEITA_COOL2;
			else
				return z;
			break;
		case SM5714_JEITA_COOL2:
			if (t < 50)
				z = SM5714_JEITA_COOL3;
			else if (t >= 170)
				z = SM5714_JEITA_NORMAL;
			else
				return z;
			break;
		case SM5714_JEITA_NORMAL:
			if (t < 150)
				z = SM5714_JEITA_COOL2;
			else if (t > 410)
				z = SM5714_JEITA_WARM;
			else
				return z;
			break;
		case SM5714_JEITA_WARM:
			if (t < 390)
				z = SM5714_JEITA_NORMAL;
			else if (t > 500)
				z = SM5714_JEITA_HOT;
			else
				return z;
			break;
		case SM5714_JEITA_HOT:
			if (t >= 480)
				return z;
			z = SM5714_JEITA_WARM;
			break;
		default:
			return z;
		}
	}
	return z;
}

static int sm5714_chg_read_temp(struct sm5714_charger *chg, int *t)
{
	union power_supply_propval val;
	int ret;

	if (jeita_temp != SM5714_JEITA_TEMP_OFF) {
		*t = jeita_temp;
		return 0;
	}

	if (!chg->bat) {
		chg->bat = power_supply_get_by_name("sm5714-battery");
		if (!chg->bat)
			return -ENODEV;
	}

	ret = power_supply_get_property(chg->bat, POWER_SUPPLY_PROP_TEMP, &val);
	if (!ret)
		*t = val.intval;
	return ret;
}

/* Called with chg->lock held: program the hardware for chg->zone */
static void sm5714_chg_jeita_apply(struct sm5714_charger *chg)
{
	unsigned int st1, cntl1;

	if (regmap_read(chg->regmap, SM5714_CHG_STATUS1, &st1) ||
	    !(st1 & SM5714_STATUS1_VBUSPOK) ||
	    regmap_read(chg->regmap, SM5714_CHG_CNTL1, &cntl1))
		return;

	if (!sm5714_chg_jeita_allows(chg)) {
		if (cntl1 & SM5714_CNTL1_ENQ4FET)
			regmap_clear_bits(chg->regmap, SM5714_CHG_CNTL1,
					  SM5714_CNTL1_ENQ4FET);
	} else if (!(cntl1 & SM5714_CNTL1_ENQ4FET)) {
		sm5714_chg_enable(chg);	/* full sequence, honours the zone */
	} else {
		sm5714_chg_set_float(chg, sm5714_jeita[chg->zone].float_mv);
		sm5714_chg_set_ccc(chg, sm5714_chg_ccc_eff(chg));
	}
}

/*
 * Charge limit from the battery's charge_control_*_threshold: stop at
 * capacity >= end (end < 100), resume at capacity <= start. Unreadable
 * values mean no limit. Called with chg->lock held.
 */
static bool sm5714_chg_limit_update(struct sm5714_charger *chg)
{
	union power_supply_propval cap, start, end;
	bool old = chg->limit_stop;

	if (!chg->bat)
		chg->bat = power_supply_get_by_name("sm5714-battery");
	if (!chg->bat ||
	    power_supply_get_property(chg->bat, POWER_SUPPLY_PROP_CAPACITY, &cap) ||
	    power_supply_get_property(chg->bat,
			POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD, &start) ||
	    power_supply_get_property(chg->bat,
			POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD, &end)) {
		chg->limit_stop = false;
	} else if (end.intval >= 100) {
		chg->limit_stop = false;
	} else if (!chg->limit_stop && cap.intval >= end.intval) {
		chg->limit_stop = true;
	} else if (chg->limit_stop && cap.intval <= start.intval) {
		chg->limit_stop = false;
	}

	if (chg->limit_stop != old)
		dev_info(chg->dev, "charge limit: %d %% -> %s (start %d, end %d)\n",
			 cap.intval, chg->limit_stop ? "hold" : "charge",
			 start.intval, end.intval);
	return chg->limit_stop != old;
}

static void sm5714_chg_jeita_work(struct work_struct *work)
{
	struct sm5714_charger *chg = container_of(work, struct sm5714_charger,
						  jeita_work.work);
	enum sm5714_jeita_zone old, new;
	unsigned int st1 = 0;
	bool changed = false;
	int t, ret;

	ret = sm5714_chg_read_temp(chg, &t);

	mutex_lock(&chg->lock);
	if (ret) {
		/* keep the last zone (a HOT tablet must not resume on a glitch) */
		dev_warn_once(chg->dev, "no battery temperature (%d)\n", ret);
	} else {
		chg->temp = t;
		old = chg->zone;
		new = sm5714_jeita_next(old, t);
		if (new != old) {
			WRITE_ONCE(chg->zone, new);
			changed = true;
			dev_info(chg->dev,
				 "JEITA %s%d.%d C: %s -> %s (charge %u uA, float %u mV)\n",
				 t < 0 ? "-" : "", abs(t) / 10, abs(t) % 10,
				 sm5714_jeita[old].name,
				 sm5714_jeita[new].name, sm5714_chg_ccc_eff(chg),
				 sm5714_jeita[new].float_mv);
		}
	}
	if (sm5714_chg_limit_update(chg))
		changed = true;
	sm5714_chg_jeita_apply(chg);
	regmap_read(chg->regmap, SM5714_CHG_STATUS1, &st1);
	mutex_unlock(&chg->lock);

	if (changed)
		power_supply_changed(chg->psy);

	/* Poll only while a cable is in; the plug IRQ restarts us */
	if (st1 & SM5714_STATUS1_VBUSPOK)
		queue_delayed_work(system_dfl_wq, &chg->jeita_work,
				      msecs_to_jiffies(SM5714_JEITA_POLL_MS));
}

/*
 * Threaded handler for the MFD's nested CHG IRQs. The MFD has already read
 * (and so cleared) INT1..INT5; we only look at live status here.
 */
static irqreturn_t sm5714_chg_isr(int irq, void *data)
{
	struct sm5714_charger *chg = data;
	unsigned int st1, cntl1;

	mutex_lock(&chg->lock);
	if (!regmap_read(chg->regmap, SM5714_CHG_STATUS1, &st1) &&
	    (st1 & SM5714_STATUS1_VBUSPOK) &&
	    !regmap_read(chg->regmap, SM5714_CHG_CNTL1, &cntl1) &&
	    !(cntl1 & SM5714_CNTL1_ENQ4FET)) {
		/* VBUS present but battery path off (e.g. after OTG): redo it */
		sm5714_chg_enable(chg);
	}
	mutex_unlock(&chg->lock);

	/* MUIC chgtype/detach, or VBUS came/went: re-decide the limit */
	sm5714_chg_update_icl(chg, chg->psy);

	/* Cable in: check the temperature now and start polling */
	mod_delayed_work(system_dfl_wq, &chg->jeita_work, 0);

	power_supply_changed(chg->psy);
	return IRQ_HANDLED;
}

/*
 * CHG: plug = VBUSPOK, unplug = VBUSUVLO (VBUSPOK does not latch on removal).
 * MUIC: chgtype = BC1.2 done, muic-detach = MUIC saw the cable go.
 */
static const char * const sm5714_chg_irq_names[] = {
	"vbuspok", "vbusuvlo", "chgon", "done", "chgtype", "muic-detach",
};

static int sm5714_chg_request_irqs(struct platform_device *pdev,
				   struct sm5714_charger *chg)
{
	struct device *dev = &pdev->dev;
	int i, irq, ret;

	for (i = 0; i < ARRAY_SIZE(sm5714_chg_irq_names); i++) {
		irq = platform_get_irq_byname_optional(pdev,
						       sm5714_chg_irq_names[i]);
		if (irq == -ENXIO) {
			/* MFD without the IRQ chip yet: run without events */
			dev_info(dev, "no %s irq, events disabled\n",
				 sm5714_chg_irq_names[i]);
			continue;
		}
		if (irq < 0)
			return irq;

		ret = devm_request_threaded_irq(dev, irq, NULL, sm5714_chg_isr,
						IRQF_ONESHOT,
						sm5714_chg_irq_names[i], chg);
		if (ret)
			return dev_err_probe(dev, ret, "failed to request %s irq\n",
					     sm5714_chg_irq_names[i]);
	}

	return 0;
}

static void sm5714_chg_cleanup(void *data)
{
	struct sm5714_charger *chg = data;

	cancel_delayed_work_sync(&chg->jeita_work);
	if (chg->bat)
		power_supply_put(chg->bat);
}

static int sm5714_chg_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct power_supply_config psy_cfg = {};
	struct sm5714 *core = dev_get_drvdata(dev->parent);
	struct sm5714_charger *chg;
	int ret;

	if (!core)
		return dev_err_probe(dev, -EPROBE_DEFER, "no MFD core data\n");

	chg = devm_kzalloc(dev, sizeof(*chg), GFP_KERNEL);
	if (!chg)
		return -ENOMEM;

	chg->dev = dev;
	/* Same as sm5714-regulator: regmaps come from the core's drvdata */
	chg->regmap = core->regmap_chg;
	chg->regmap_muic = core->regmap_muic;

	ret = devm_mutex_init(dev, &chg->lock);
	if (ret)
		return ret;

	chg->icl_ua = SM5714_ICL_SAFE_UA;
	/* JEITA caps it by temperature; sec-battery max is 3.2 A */
	chg->ccc_ua = 2500000;
	chg->zone = SM5714_JEITA_UNKNOWN;	/* capped until the first reading */
	INIT_DELAYED_WORK(&chg->jeita_work, sm5714_chg_jeita_work);

	/* probe runs before any IRQ/callback can, so no lock needed here */
	ret = sm5714_chg_enable(chg);
	if (ret)
		return dev_err_probe(dev, ret, "enable sequence failed\n");

	psy_cfg.drv_data = chg;
	chg->psy = devm_power_supply_register(dev, &sm5714_chg_desc, &psy_cfg);
	if (IS_ERR(chg->psy))
		return dev_err_probe(dev, PTR_ERR(chg->psy),
				     "failed to register power supply\n");

	/*
	 * Teardown runs in reverse: IRQs freed (no new kicks) -> work
	 * cancelled -> psy unregistered. So add the cleanup between the two.
	 */
	ret = devm_add_action_or_reset(dev, sm5714_chg_cleanup, chg);
	if (ret)
		return ret;

	/* after the psy exists: the handler calls power_supply_changed() */
	ret = sm5714_chg_request_irqs(pdev, chg);
	if (ret)
		return ret;

	/*
	 * If sm5714-typec registered first its "changed" already went by;
	 * pick up its current state now. Otherwise this finds no supplier and
	 * keeps 500 mA until typec registers and notifies us.
	 */
	sm5714_chg_update_icl(chg, chg->psy);
	power_supply_changed(chg->psy);

	queue_delayed_work(system_dfl_wq, &chg->jeita_work, 0);

	platform_set_drvdata(pdev, chg);
	return 0;
}

static const struct platform_device_id sm5714_chg_id[] = {
	{ "sm5714-charger" },
	{ }
};
MODULE_DEVICE_TABLE(platform, sm5714_chg_id);

static struct platform_driver sm5714_chg_driver = {
	.driver = {
		.name = "sm5714-charger",
	},
	.probe = sm5714_chg_probe,
	.id_table = sm5714_chg_id,
};
module_platform_driver(sm5714_chg_driver);

MODULE_DESCRIPTION("Silicon Mitus SM5714 charger driver");
MODULE_LICENSE("GPL");
