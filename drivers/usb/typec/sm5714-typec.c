// SPDX-License-Identifier: GPL-2.0-only
/*
 * Silicon Mitus SM5714 USB Type-C / USB PD port (TCPM low-level driver)
 *
 * The SM5714 PD block (own I2C address 0x33, own IRQ) has:
 *  - a hardware Type-C CC state machine (DRP toggling, attach/detach,
 *    debounce, orientation) reported in CC_STATUS,
 *  - a PD PHY with a hardware protocol layer (GoodCRC, retries) and
 *    RX/TX buffers.
 * The policy engine is mainline TCPM. Like qcom_pmic_typec, CC is not
 * driven pin by pin: get_cc() is synthesised from the hardware attach
 * result and set_cc()/start_toggling() select the hardware mode.
 *
 * TCPM's current limit (Rp level or PD contract) is published through a
 * small power_supply "sm5714-typec" (CURRENT_MAX), supplied_to
 * "sm5714-charger", which sets its input limit from it.
 *
 * Register usage from Samsung's downstream sm5714_typec.c / sm5714_policy.c
 * (GPL-2.0); behaviour verified on gts7xllite (see project notes):
 *  - RX payload is valid only until RX_BUF is acked; burst-read it first.
 *  - PD_CNTL2 bit 2 must stay set.
 *  - After any Hard Reset the protocol layer waits for "reset done".
 *  - Masked INT4 events do not latch.
 *
 * PR_Swap (CC_CNTL7 Rp/Rd flip with CC state hold) follows Samsung's
 * sequence but is not verified yet.
 */

#include <linux/bitfield.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/mfd/sm5714.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/usb/pd.h>
#include <linux/usb/tcpm.h>
#include <linux/workqueue.h>

#define SM5714_TC_REG_INT1		0x01	/* INT1..INT5, read to clear */
#define SM5714_TC_REG_INT_MASK1		0x06	/* INT_MASK1..5, 1 = masked */
#define SM5714_TC_REG_STATUS1		0x0b
#define SM5714_TC_REG_CORR_CNTL4	0x23	/* water detection */
#define SM5714_TC_REG_CORR_CNTL5	0x24	/* SBU sourcing */
#define SM5714_TC_REG_CC_STATUS		0x28
#define SM5714_TC_REG_CC_CNTL1		0x29
#define SM5714_TC_REG_CC_CNTL3		0x2b
#define SM5714_TC_REG_CC_CNTL5		0x2d	/* VCONN */
#define SM5714_TC_REG_CC_CNTL7		0x2f	/* b0: flip Rp/Rd (PR_Swap) */
#define SM5714_TC_REG_PD_CNTL1		0x38
#define SM5714_TC_REG_PD_CNTL2		0x39
#define SM5714_TC_REG_PD_CNTL4		0x3b
#define SM5714_TC_REG_RX_SRC		0x41
#define SM5714_TC_REG_RX_HEADER		0x42
#define SM5714_TC_REG_RX_PAYLOAD	0x44
#define SM5714_TC_REG_RX_BUF		0x5e
#define SM5714_TC_REG_RX_BUF_ST		0x5f
#define SM5714_TC_REG_TX_HEADER		0x60
#define SM5714_TC_REG_TX_PAYLOAD	0x62
#define SM5714_TC_REG_TX_REQ		0x7e
#define SM5714_TC_REG_PROBE0		0xd0
#define SM5714_TC_REG_PD_STATE0		0xd5	/* b3:0 CC state machine state */
#define SM5714_TC_REG_PD_STATE3		0xd8
#define SM5714_TC_MAX_REGISTER		0xda

/* INT1 / STATUS1 */
#define SM5714_TC_INT1_VBUSPOK		BIT(0)
#define SM5714_TC_INT1_ATTACH		BIT(3)
#define SM5714_TC_INT1_DETACH		BIT(4)
/* INT2 */
#define SM5714_TC_INT2_SRC_ADV_CHG	BIT(4)
#define SM5714_TC_INT2_VBUS_0V		BIT(5)
/* INT4 */
#define SM5714_TC_INT4_RX_DONE		BIT(0)
#define SM5714_TC_INT4_TX_DONE		BIT(1)
#define SM5714_TC_INT4_TX_SOP_ERR	BIT(2)
#define SM5714_TC_INT4_PRL_RST_DONE	BIT(4)
#define SM5714_TC_INT4_HRST_RCVED	BIT(5)
#define SM5714_TC_INT4_HCRST_DONE	BIT(6)
#define SM5714_TC_INT4_TX_DISCARD	BIT(7)

#define SM5714_TC_INT1_EN	(SM5714_TC_INT1_VBUSPOK | SM5714_TC_INT1_ATTACH | \
				 SM5714_TC_INT1_DETACH)
#define SM5714_TC_INT2_EN	(SM5714_TC_INT2_SRC_ADV_CHG | SM5714_TC_INT2_VBUS_0V)
#define SM5714_TC_INT4_EN	(SM5714_TC_INT4_RX_DONE | SM5714_TC_INT4_TX_DONE | \
				 SM5714_TC_INT4_TX_SOP_ERR | SM5714_TC_INT4_HRST_RCVED | \
				 SM5714_TC_INT4_HCRST_DONE | SM5714_TC_INT4_TX_DISCARD)

/* CC_STATUS */
#define SM5714_TC_CC_ATTACH_TYPE	GENMASK(2, 0)
#define SM5714_TC_CC_ATTACH_NONE	0
#define SM5714_TC_CC_ATTACH_SNK		1	/* partner is Rp: we are sink */
#define SM5714_TC_CC_ATTACH_SRC		2	/* partner is Rd: we are source */
#define SM5714_TC_CC_ATTACH_AUDIO	3
#define SM5714_TC_CC_ADV_CURR		GENMASK(4, 3)
#define SM5714_TC_CC_FLIP		BIT(5)

/* CC_CNTL1: mode (Samsung values) + Rp current as source in b5:4 */
#define SM5714_TC_CC_CNTL1_DRP		0x41
#define SM5714_TC_CC_CNTL1_SNK		0x45
#define SM5714_TC_CC_CNTL1_SRC		0x49
#define SM5714_TC_CC_CNTL1_RP_MASK	GENMASK(5, 4)
#define SM5714_TC_CC_CNTL1_RP_DEF	0x00	/* 80 uA */
#define SM5714_TC_CC_CNTL1_RP_1_5	0x10	/* 180 uA, PD3 SinkTxNG */
#define SM5714_TC_CC_CNTL1_RP_3_0	0x20	/* 330 uA, PD3 SinkTxOK */
/* CC_CNTL3 */
#define SM5714_TC_CC_CNTL3_OP_EN	0x80
#define SM5714_TC_CC_CNTL3_TO_UNATT_SNK	0x82	/* leave CC_DISABLE (Samsung) */
#define SM5714_TC_CC_CNTL3_DETACH	BIT(3)	/* force detach */
#define SM5714_TC_CC_CNTL3_HOLD_MASK	GENMASK(5, 4)
#define SM5714_TC_CC_CNTL3_HOLD		0x10	/* CC state hold (swap) */
/* CC_CNTL5 VCONN */
#define SM5714_TC_VCONN_OFF		0x18
#define SM5714_TC_VCONN_CC1		0x19
#define SM5714_TC_VCONN_CC2		0x1a

#define SM5714_TC_PD_CNTL1_PD_ON	0x08
#define SM5714_TC_PD_CNTL2_BASE		0x04	/* bit 2 must stay set */
#define SM5714_TC_PD_CNTL2_DFP		BIT(0)
#define SM5714_TC_PD_CNTL2_SRC		BIT(1)
#define SM5714_TC_PD_CNTL4_RESET_DONE	0x01
#define SM5714_TC_PD_CNTL4_CABLE_RESET	0x02
#define SM5714_TC_PD_CNTL4_HARD_RESET	0x04
#define SM5714_TC_RX_BUF_READ_DONE	0x80
#define SM5714_TC_RX_BUF_ST_FLUSH	0x10
#define SM5714_TC_TX_REQ_SOP		0x07
#define SM5714_TC_PROBE0_VBUS_OK	BIT(6)
#define SM5714_TC_PD_STATE3_RESET_WAIT	0x06
#define SM5714_TC_PD_STATE0_CC		GENMASK(3, 0)
#define SM5714_TC_CC_STATE_DISABLE	0x1

/* Release the CC state hold this long after a PR_Swap Rp/Rd flip */
#define SM5714_TC_SWAP_HOLD_MS		600

struct sm5714_typec {
	struct device *dev;
	struct regmap *regmap;
	struct sm5714 *sm5714;		/* MFD core, for the MUIC switch */
	struct regulator *vbus;
	struct tcpc_dev tcpc;
	struct tcpm_port *tcpm_port;
	struct power_supply *psy;
	struct delayed_work hold_work;
	struct mutex lock;		/* register sequences (TX, CC, VBUS) */
	enum typec_cc_status cc;	/* last set_cc() */
	bool vbus_on;
	bool muic_usb;
	bool pd_rx;			/* set_pd_rx(true): a PD session exists */
	bool try_snk;			/* hardware switched to sink-only for Try.SNK */
	u32 src_ua;			/* TCPM current limit as sink, 0 otherwise */
	u32 src_uv;			/* negotiated VBUS (5 V without PD) */
};

static inline struct sm5714_typec *tcpc_to_tc(struct tcpc_dev *tcpc)
{
	return container_of(tcpc, struct sm5714_typec, tcpc);
}

/*
 * TCPM's state machine may already call our ops from its worker while
 * tcpm_register_port() is still returning, so tc->tcpm_port can be NULL.
 * probe() kicks both events once the port pointer is set.
 */
static void sm5714_tc_cc_change(struct sm5714_typec *tc)
{
	struct tcpm_port *port = READ_ONCE(tc->tcpm_port);

	if (port)
		tcpm_cc_change(port);
}

static void sm5714_tc_vbus_change(struct sm5714_typec *tc)
{
	struct tcpm_port *port = READ_ONCE(tc->tcpm_port);

	if (port)
		tcpm_vbus_change(port);
}

static const struct regmap_config sm5714_typec_regmap_config = {
	.name = "typec",
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = SM5714_TC_MAX_REGISTER,
};

/* --- power_supply towards the charger ----------------------------------- */

static int sm5714_typec_psy_get_property(struct power_supply *psy,
					 enum power_supply_property psp,
					 union power_supply_propval *val)
{
	struct sm5714_typec *tc = power_supply_get_drvdata(psy);
	u32 ua = READ_ONCE(tc->src_ua);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = ua > 0;
		return 0;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = ua;
		return 0;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = ua ? READ_ONCE(tc->src_uv) : 0;
		return 0;
	default:
		return -EINVAL;
	}
}

static const enum power_supply_property sm5714_typec_psy_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
};

static const struct power_supply_desc sm5714_typec_psy_desc = {
	.name = "sm5714-typec",
	.type = POWER_SUPPLY_TYPE_USB,
	.properties = sm5714_typec_psy_props,
	.num_properties = ARRAY_SIZE(sm5714_typec_psy_props),
	.get_property = sm5714_typec_psy_get_property,
};

static char *sm5714_typec_supplied_to[] = { "sm5714-charger" };

/* --- tcpc_dev ops ------------------------------------------------------- */

static int sm5714_tcpc_get_vbus(struct tcpc_dev *tcpc)
{
	struct sm5714_typec *tc = tcpc_to_tc(tcpc);
	unsigned int val;

	if (READ_ONCE(tc->vbus_on))
		return 1;
	if (regmap_read(tc->regmap, SM5714_TC_REG_PROBE0, &val))
		return 0;
	return !!(val & SM5714_TC_PROBE0_VBUS_OK);
}

static int sm5714_tcpc_get_cc(struct tcpc_dev *tcpc, enum typec_cc_status *cc1,
			      enum typec_cc_status *cc2)
{
	struct sm5714_typec *tc = tcpc_to_tc(tcpc);
	enum typec_cc_status val;
	unsigned int cc;
	int ret;

	*cc1 = TYPEC_CC_OPEN;
	*cc2 = TYPEC_CC_OPEN;

	ret = regmap_read(tc->regmap, SM5714_TC_REG_CC_STATUS, &cc);
	if (ret)
		return ret;

	switch (FIELD_GET(SM5714_TC_CC_ATTACH_TYPE, cc)) {
	case SM5714_TC_CC_ATTACH_SNK:
		switch (FIELD_GET(SM5714_TC_CC_ADV_CURR, cc)) {
		case 0:
			val = TYPEC_CC_RP_DEF;
			break;
		case 1:
			val = TYPEC_CC_RP_1_5;
			break;
		default:
			val = TYPEC_CC_RP_3_0;
			break;
		}
		break;
	case SM5714_TC_CC_ATTACH_SRC:
		val = TYPEC_CC_RD;
		break;
	case SM5714_TC_CC_ATTACH_AUDIO:
		*cc1 = TYPEC_CC_RA;
		*cc2 = TYPEC_CC_RA;
		goto out;
	default:
		goto out;
	}

	if (cc & SM5714_TC_CC_FLIP)
		*cc2 = val;
	else
		*cc1 = val;
out:
	dev_dbg(tc->dev, "get_cc: CC_STATUS 0x%02x -> cc1 %d cc2 %d\n",
		cc, *cc1, *cc2);
	return 0;
}

static void sm5714_typec_hold_release(struct work_struct *work)
{
	struct sm5714_typec *tc = container_of(work, struct sm5714_typec,
					       hold_work.work);

	mutex_lock(&tc->lock);
	regmap_update_bits(tc->regmap, SM5714_TC_REG_CC_CNTL3,
			   SM5714_TC_CC_CNTL3_HOLD_MASK, 0);
	mutex_unlock(&tc->lock);
	dev_dbg(tc->dev, "CC state hold released\n");
}

static bool sm5714_cc_is_rp(enum typec_cc_status cc)
{
	return cc == TYPEC_CC_RP_DEF || cc == TYPEC_CC_RP_1_5 ||
	       cc == TYPEC_CC_RP_3_0;
}

static unsigned int sm5714_cc_rp_bits(enum typec_cc_status cc)
{
	switch (cc) {
	case TYPEC_CC_RP_1_5:
		return SM5714_TC_CC_CNTL1_RP_1_5;
	case TYPEC_CC_RP_3_0:
		return SM5714_TC_CC_CNTL1_RP_3_0;
	default:
		return SM5714_TC_CC_CNTL1_RP_DEF;
	}
}

/*
 * (Re)start the CC state machine. After a forced detach (set_cc(OPEN)) it
 * can sit in CC_DISABLE, where OP_EN alone does nothing: Samsung's
 * check_cc_state() then writes 0x82 (CC_DISABLE -> unattached sink).
 * Seen on gts7xllite: PD_STATE0 = 0x01, no attach at all until 0x82.
 */
static int sm5714_typec_restart_fsm(struct sm5714_typec *tc)
{
	unsigned int st0, val = SM5714_TC_CC_CNTL3_OP_EN;

	if (!regmap_read(tc->regmap, SM5714_TC_REG_PD_STATE0, &st0) &&
	    (st0 & SM5714_TC_PD_STATE0_CC) == SM5714_TC_CC_STATE_DISABLE) {
		dev_dbg(tc->dev, "CC state machine in CC_DISABLE, kicking\n");
		val = SM5714_TC_CC_CNTL3_TO_UNATT_SNK;
	}
	return regmap_write(tc->regmap, SM5714_TC_REG_CC_CNTL3, val);
}

/* Change the hardware mode (keeping the Rp level) and restart its CC FSM */
static int sm5714_typec_set_mode(struct sm5714_typec *tc, unsigned int mode)
{
	int ret;

	ret = regmap_update_bits(tc->regmap, SM5714_TC_REG_CC_CNTL1,
				 (u8)~SM5714_TC_CC_CNTL1_RP_MASK, mode);
	if (!ret)
		ret = sm5714_typec_restart_fsm(tc);
	return ret;
}

/* PR_Swap: hold the CC state and flip Rp/Rd (Samsung assert_rd/assert_rp) */
static int sm5714_typec_swap_flip(struct sm5714_typec *tc, bool to_rd)
{
	unsigned int v;
	int ret;

	dev_info(tc->dev, "PR_Swap: assert %s\n", to_rd ? "Rd" : "Rp");
	regmap_update_bits(tc->regmap, SM5714_TC_REG_CC_CNTL3,
			   SM5714_TC_CC_CNTL3_HOLD_MASK, SM5714_TC_CC_CNTL3_HOLD);
	ret = regmap_read(tc->regmap, SM5714_TC_REG_CC_CNTL7, &v);
	if (!ret)
		ret = regmap_write(tc->regmap, SM5714_TC_REG_CC_CNTL7, v ^ BIT(0));
	mod_delayed_work(system_dfl_wq, &tc->hold_work,
			 msecs_to_jiffies(SM5714_TC_SWAP_HOLD_MS));
	return ret;
}

/*
 * The hardware state machine owns CC. What set_cc() can mean here:
 *  - OPEN: force a detach (PORT_RESET / error recovery);
 *  - an Rp level: program the source Rp current (PD3 collision avoidance
 *    toggles 1.5 A / 3.0 A while attached as source);
 *  - Rd while attached as source:
 *      PD session up  -> PR_Swap: flip with CC_CNTL7 under a CC hold;
 *      no PD yet      -> Try.SNK: hardware to sink-only, restart, and let
 *                        TCPM watch for the partner's Rp;
 *  - Rp while attached as sink with PD up -> PR_Swap the other way;
 *  - Rp after a Try.SNK -> SRC_TRYWAIT: hardware to source-only.
 */
static int sm5714_tcpc_set_cc(struct tcpc_dev *tcpc, enum typec_cc_status cc)
{
	struct sm5714_typec *tc = tcpc_to_tc(tcpc);
	unsigned int st, attach;
	int ret = 0;

	mutex_lock(&tc->lock);

	if (cc == TYPEC_CC_OPEN) {
		ret = regmap_set_bits(tc->regmap, SM5714_TC_REG_CC_CNTL3,
				      SM5714_TC_CC_CNTL3_DETACH);
		goto out;
	}

	ret = regmap_read(tc->regmap, SM5714_TC_REG_CC_STATUS, &st);
	if (ret)
		goto out;
	attach = FIELD_GET(SM5714_TC_CC_ATTACH_TYPE, st);

	if (sm5714_cc_is_rp(cc)) {
		ret = regmap_update_bits(tc->regmap, SM5714_TC_REG_CC_CNTL1,
					 SM5714_TC_CC_CNTL1_RP_MASK,
					 sm5714_cc_rp_bits(cc));
		if (ret)
			goto out;

		if (attach == SM5714_TC_CC_ATTACH_SNK && tc->pd_rx) {
			ret = sm5714_typec_swap_flip(tc, false);
		} else if (tc->try_snk) {
			dev_info(tc->dev, "Try.SNK: no source, back to Rp\n");
			tc->try_snk = false;
			ret = sm5714_typec_set_mode(tc, SM5714_TC_CC_CNTL1_SRC);
		}
	} else if (cc == TYPEC_CC_RD && attach == SM5714_TC_CC_ATTACH_SRC) {
		if (tc->pd_rx) {
			ret = sm5714_typec_swap_flip(tc, true);
		} else {
			dev_info(tc->dev, "Try.SNK: presenting Rd only\n");
			tc->try_snk = true;
			ret = sm5714_typec_set_mode(tc, SM5714_TC_CC_CNTL1_SNK);
		}
	}
out:
	if (!ret)
		tc->cc = cc;
	mutex_unlock(&tc->lock);
	dev_dbg(tc->dev, "set_cc %d (attach %u) ret %d\n", cc, attach, ret);
	return ret;
}

static int sm5714_tcpc_set_polarity(struct tcpc_dev *tcpc,
				    enum typec_cc_polarity polarity)
{
	return 0;	/* hardware follows the attached CC pin */
}

static int sm5714_tcpc_set_vconn(struct tcpc_dev *tcpc, bool on)
{
	struct sm5714_typec *tc = tcpc_to_tc(tcpc);
	unsigned int cc, val = SM5714_TC_VCONN_OFF;
	int ret;

	mutex_lock(&tc->lock);
	if (on) {
		ret = regmap_read(tc->regmap, SM5714_TC_REG_CC_STATUS, &cc);
		if (ret)
			goto out;
		/* VCONN goes on the other pin than the attached CC */
		val = (cc & SM5714_TC_CC_FLIP) ? SM5714_TC_VCONN_CC2 :
						 SM5714_TC_VCONN_CC1;
	}
	ret = regmap_write(tc->regmap, SM5714_TC_REG_CC_CNTL5, val);
out:
	mutex_unlock(&tc->lock);
	return ret;
}

static int sm5714_tcpc_set_vbus(struct tcpc_dev *tcpc, bool on, bool charge)
{
	struct sm5714_typec *tc = tcpc_to_tc(tcpc);
	int ret = 0;

	mutex_lock(&tc->lock);
	if (on != tc->vbus_on) {
		ret = on ? regulator_enable(tc->vbus) : regulator_disable(tc->vbus);
		if (!ret)
			WRITE_ONCE(tc->vbus_on, on);
		else
			dev_err(tc->dev, "VBUS %s failed: %d\n",
				on ? "on" : "off", ret);
	}
	mutex_unlock(&tc->lock);

	if (!ret)
		sm5714_tc_vbus_change(tc);
	return ret;
}

static int sm5714_tcpc_set_current_limit(struct tcpc_dev *tcpc, u32 max_ma,
					 u32 mv)
{
	struct sm5714_typec *tc = tcpc_to_tc(tcpc);
	u32 ua = max_ma * 1000;
	u32 uv = (mv ? mv : 5000) * 1000;

	if (READ_ONCE(tc->src_ua) != ua || READ_ONCE(tc->src_uv) != uv) {
		dev_info(tc->dev, "sink current limit %u mA @ %u mV\n", max_ma, mv);
		WRITE_ONCE(tc->src_uv, uv);
		WRITE_ONCE(tc->src_ua, ua);
		power_supply_changed(tc->psy);
	}
	return 0;
}

static int sm5714_tcpc_set_pd_rx(struct tcpc_dev *tcpc, bool on)
{
	struct sm5714_typec *tc = tcpc_to_tc(tcpc);
	unsigned int st3;
	int ret;

	mutex_lock(&tc->lock);
	if (on) {
		if (!regmap_read(tc->regmap, SM5714_TC_REG_PD_STATE3, &st3) &&
		    (st3 & SM5714_TC_PD_STATE3_RESET_WAIT))
			regmap_write(tc->regmap, SM5714_TC_REG_PD_CNTL4,
				     SM5714_TC_PD_CNTL4_RESET_DONE);
		regmap_write(tc->regmap, SM5714_TC_REG_RX_BUF_ST,
			     SM5714_TC_RX_BUF_ST_FLUSH);
		ret = regmap_write(tc->regmap, SM5714_TC_REG_PD_CNTL1,
				   SM5714_TC_PD_CNTL1_PD_ON);
	} else {
		ret = regmap_write(tc->regmap, SM5714_TC_REG_PD_CNTL1, 0);
	}
	if (!ret)
		tc->pd_rx = on;
	mutex_unlock(&tc->lock);
	dev_dbg(tc->dev, "set_pd_rx %d\n", on);
	return ret;
}

static int sm5714_tcpc_set_roles(struct tcpc_dev *tcpc, bool attached,
				 enum typec_role role, enum typec_data_role data)
{
	struct sm5714_typec *tc = tcpc_to_tc(tcpc);
	unsigned int val = SM5714_TC_PD_CNTL2_BASE;
	bool usb = attached && data == TYPEC_HOST;
	int ret;

	/* Roles go into the GoodCRC/headers the protocol layer generates */
	if (data == TYPEC_HOST)
		val |= SM5714_TC_PD_CNTL2_DFP;
	if (role == TYPEC_SOURCE)
		val |= SM5714_TC_PD_CNTL2_SRC;

	mutex_lock(&tc->lock);
	ret = regmap_update_bits(tc->regmap, SM5714_TC_REG_PD_CNTL2,
				 SM5714_TC_PD_CNTL2_DFP | SM5714_TC_PD_CNTL2_SRC |
				 SM5714_TC_PD_CNTL2_BASE, val);
	/* Host mode: no BC1.2 closes the MUIC switch, do it by hand */
	if (!ret && usb != tc->muic_usb) {
		sm5714_muic_set_usb_path(tc->sm5714, usb);
		tc->muic_usb = usb;
	}
	mutex_unlock(&tc->lock);

	dev_dbg(tc->dev, "set_roles attached %d %s %s\n", attached,
		role == TYPEC_SOURCE ? "source" : "sink",
		data == TYPEC_HOST ? "host" : "device");
	return ret;
}

static int sm5714_tcpc_start_toggling(struct tcpc_dev *tcpc,
				      enum typec_port_type port_type,
				      enum typec_cc_status cc)
{
	struct sm5714_typec *tc = tcpc_to_tc(tcpc);
	unsigned int mode, st;
	int ret;

	switch (port_type) {
	case TYPEC_PORT_SNK:
		mode = SM5714_TC_CC_CNTL1_SNK;
		break;
	case TYPEC_PORT_SRC:
		mode = SM5714_TC_CC_CNTL1_SRC;
		break;
	default:
		mode = SM5714_TC_CC_CNTL1_DRP;
		break;
	}

	mutex_lock(&tc->lock);
	cancel_delayed_work(&tc->hold_work);
	tc->try_snk = false;
	/* undo a PR_Swap flip, as Samsung's detach does (&= 0xfc) */
	regmap_update_bits(tc->regmap, SM5714_TC_REG_CC_CNTL7, 0x03, 0);
	/* (Re)start the state machine; also clears a forced detach / hold */
	ret = sm5714_typec_set_mode(tc, mode);
	if (!ret)
		ret = regmap_read(tc->regmap, SM5714_TC_REG_CC_STATUS, &st);
	mutex_unlock(&tc->lock);
	if (ret)
		return ret;

	dev_dbg(tc->dev, "start_toggling type %d mode 0x%02x CC_STATUS 0x%02x\n",
		port_type, mode, st);

	/* Already attached (cable in at boot): let TCPM see it now */
	if (FIELD_GET(SM5714_TC_CC_ATTACH_TYPE, st) != SM5714_TC_CC_ATTACH_NONE)
		sm5714_tc_cc_change(tc);
	return 0;
}

static int sm5714_tcpc_pd_transmit(struct tcpc_dev *tcpc,
				   enum tcpm_transmit_type type,
				   const struct pd_message *msg,
				   unsigned int negotiated_rev)
{
	struct sm5714_typec *tc = tcpc_to_tc(tcpc);
	unsigned int cnt;
	int ret;

	mutex_lock(&tc->lock);
	switch (type) {
	case TCPC_TX_HARD_RESET:
		ret = regmap_set_bits(tc->regmap, SM5714_TC_REG_PD_CNTL4,
				      SM5714_TC_PD_CNTL4_HARD_RESET);
		break;
	case TCPC_TX_CABLE_RESET:
		ret = regmap_set_bits(tc->regmap, SM5714_TC_REG_PD_CNTL4,
				      SM5714_TC_PD_CNTL4_CABLE_RESET);
		break;
	case TCPC_TX_SOP:
		cnt = pd_header_cnt_le(msg->header);
		ret = regmap_bulk_write(tc->regmap, SM5714_TC_REG_TX_HEADER,
					&msg->header, sizeof(msg->header));
		if (!ret && cnt)
			ret = regmap_bulk_write(tc->regmap,
						SM5714_TC_REG_TX_PAYLOAD,
						msg->payload, cnt * 4);
		if (!ret)
			ret = regmap_write(tc->regmap, SM5714_TC_REG_TX_REQ,
					   SM5714_TC_TX_REQ_SOP);
		dev_dbg(tc->dev, "tx hdr 0x%04x cnt %u ret %d\n",
			le16_to_cpu(msg->header), cnt, ret);
		break;
	default:
		ret = -EOPNOTSUPP;	/* SOP'/SOP'' and BIST: not yet */
		break;
	}
	mutex_unlock(&tc->lock);
	return ret;
}

static int sm5714_tcpc_init(struct tcpc_dev *tcpc)
{
	struct sm5714_typec *tc = tcpc_to_tc(tcpc);
	u8 mask[5] = { 0xff, 0xff, 0xff, 0xff, 0xff };
	u8 junk[5];
	unsigned int val;
	int ret;

	mask[0] &= ~SM5714_TC_INT1_EN;
	mask[1] &= ~SM5714_TC_INT2_EN;
	mask[3] &= ~SM5714_TC_INT4_EN;

	ret = regmap_write(tc->regmap, SM5714_TC_REG_PD_CNTL1, 0);
	if (!ret)
		ret = regmap_write(tc->regmap, SM5714_TC_REG_PD_CNTL2,
				   SM5714_TC_PD_CNTL2_BASE);
	if (!ret)
		ret = regmap_write(tc->regmap, SM5714_TC_REG_CC_CNTL5,
				   SM5714_TC_VCONN_OFF);
	/*
	 * Samsung reg_init without water detection: CORR_CNTL4 = 0, release
	 * SBU sourcing (CORR_CNTL5 = 0). The bootloader can leave 0x03 here.
	 */
	if (!ret)
		ret = regmap_write(tc->regmap, SM5714_TC_REG_CORR_CNTL4, 0);
	if (!ret)
		ret = regmap_write(tc->regmap, SM5714_TC_REG_CORR_CNTL5, 0);
	/* Release the PD engine if it waits for a reset ack (as downstream) */
	if (!ret)
		ret = regmap_read(tc->regmap, SM5714_TC_REG_PD_STATE3, &val);
	if (!ret && (val & SM5714_TC_PD_STATE3_RESET_WAIT))
		ret = regmap_write(tc->regmap, SM5714_TC_REG_PD_CNTL4,
				   SM5714_TC_PD_CNTL4_RESET_DONE);
	/* Stale flags from the bootloader would hold the level IRQ low */
	if (!ret)
		ret = regmap_bulk_read(tc->regmap, SM5714_TC_REG_INT1,
				       junk, sizeof(junk));
	if (!ret)
		ret = regmap_bulk_write(tc->regmap, SM5714_TC_REG_INT_MASK1,
					mask, sizeof(mask));
	return ret;
}

/* --- interrupt ---------------------------------------------------------- */

static void sm5714_typec_pd_receive(struct sm5714_typec *tc)
{
	struct pd_message msg = { };
	unsigned int src = 0, cnt;
	int ret;

	mutex_lock(&tc->lock);
	/* One burst each: the payload is only valid until RX_BUF is acked */
	ret = regmap_bulk_read(tc->regmap, SM5714_TC_REG_RX_HEADER,
			       &msg.header, sizeof(msg.header));
	cnt = pd_header_cnt_le(msg.header);
	if (!ret && cnt)
		ret = regmap_bulk_read(tc->regmap, SM5714_TC_REG_RX_PAYLOAD,
				       msg.payload, cnt * 4);
	if (!ret)
		ret = regmap_read(tc->regmap, SM5714_TC_REG_RX_SRC, &src);
	regmap_write(tc->regmap, SM5714_TC_REG_RX_BUF, SM5714_TC_RX_BUF_READ_DONE);
	mutex_unlock(&tc->lock);

	if (ret) {
		dev_err(tc->dev, "PD rx read failed: %d\n", ret);
		return;
	}

	dev_dbg(tc->dev, "rx sop%u hdr 0x%04x cnt %u\n", src & 0xf,
		le16_to_cpu(msg.header), cnt);

	switch (src & 0xf) {
	case 0:
		tcpm_pd_receive(tc->tcpm_port, &msg, TCPC_TX_SOP);
		break;
	case 1:
		tcpm_pd_receive(tc->tcpm_port, &msg, TCPC_TX_SOP_PRIME);
		break;
	default:
		break;	/* SOP'' / debug: not enabled */
	}
}

static irqreturn_t sm5714_typec_irq(int irq, void *data)
{
	struct sm5714_typec *tc = data;
	u8 intr[5];
	int ret;

	/* Reading INT1..INT5 clears them and releases the (level) IRQ line */
	ret = regmap_bulk_read(tc->regmap, SM5714_TC_REG_INT1, intr, sizeof(intr));
	if (ret) {
		dev_err(tc->dev, "failed to read interrupts: %d\n", ret);
		return IRQ_NONE;
	}

	dev_dbg(tc->dev, "INT %*ph\n", (int)sizeof(intr), intr);

	/* Transmit result before any reply that came in with it */
	if (intr[3] & SM5714_TC_INT4_TX_DONE)
		tcpm_pd_transmit_complete(tc->tcpm_port, TCPC_TX_SUCCESS);
	else if (intr[3] & SM5714_TC_INT4_TX_DISCARD)
		tcpm_pd_transmit_complete(tc->tcpm_port, TCPC_TX_DISCARDED);
	else if (intr[3] & SM5714_TC_INT4_TX_SOP_ERR)
		tcpm_pd_transmit_complete(tc->tcpm_port, TCPC_TX_FAILED);

	if (intr[3] & SM5714_TC_INT4_RX_DONE)
		sm5714_typec_pd_receive(tc);

	/* After any Hard Reset the PRL waits for "reset done" */
	if (intr[3] & (SM5714_TC_INT4_HRST_RCVED | SM5714_TC_INT4_HCRST_DONE)) {
		mutex_lock(&tc->lock);
		regmap_write(tc->regmap, SM5714_TC_REG_PD_CNTL4,
			     SM5714_TC_PD_CNTL4_RESET_DONE);
		mutex_unlock(&tc->lock);
	}
	if (intr[3] & SM5714_TC_INT4_HCRST_DONE)
		tcpm_pd_transmit_complete(tc->tcpm_port, TCPC_TX_SUCCESS);
	if (intr[3] & SM5714_TC_INT4_HRST_RCVED)
		tcpm_pd_hard_reset(tc->tcpm_port);

	if ((intr[0] & (SM5714_TC_INT1_ATTACH | SM5714_TC_INT1_DETACH)) ||
	    (intr[1] & SM5714_TC_INT2_SRC_ADV_CHG))
		tcpm_cc_change(tc->tcpm_port);

	if ((intr[0] & SM5714_TC_INT1_VBUSPOK) ||
	    (intr[1] & SM5714_TC_INT2_VBUS_0V))
		tcpm_vbus_change(tc->tcpm_port);

	return IRQ_HANDLED;
}

/* --- probe -------------------------------------------------------------- */

static int sm5714_typec_get_mfd(struct sm5714_typec *tc)
{
	struct device_node *np;
	struct i2c_client *client;

	np = of_parse_phandle(tc->dev->of_node, "siliconmitus,muic", 0);
	if (!np)
		return dev_err_probe(tc->dev, -EINVAL,
				     "missing siliconmitus,muic phandle\n");

	client = of_find_i2c_device_by_node(np);
	of_node_put(np);
	if (!client)
		return -EPROBE_DEFER;

	/*
	 * vbus-supply already made us wait for the SM5714 core to finish
	 * probing (its regulator cell is added last), so drvdata is valid.
	 */
	tc->sm5714 = i2c_get_clientdata(client);
	put_device(&client->dev);
	if (!tc->sm5714)
		return -EPROBE_DEFER;

	return 0;
}

static void sm5714_typec_unregister(void *data)
{
	struct sm5714_typec *tc = data;

	tcpm_unregister_port(tc->tcpm_port);
	cancel_delayed_work_sync(&tc->hold_work);
	if (tc->vbus_on)
		regulator_disable(tc->vbus);
	if (tc->muic_usb)
		sm5714_muic_set_usb_path(tc->sm5714, false);
}

static void sm5714_typec_put_fwnode(void *data)
{
	fwnode_handle_put(data);
}

static int sm5714_typec_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct power_supply_config psy_cfg = { };
	struct tcpm_port *port;
	struct sm5714_typec *tc;
	int ret;

	if (!client->irq)
		return dev_err_probe(dev, -EINVAL, "no interrupt\n");

	tc = devm_kzalloc(dev, sizeof(*tc), GFP_KERNEL);
	if (!tc)
		return -ENOMEM;

	tc->dev = dev;
	tc->cc = TYPEC_CC_OPEN;
	INIT_DELAYED_WORK(&tc->hold_work, sm5714_typec_hold_release);
	ret = devm_mutex_init(dev, &tc->lock);
	if (ret)
		return ret;

	tc->regmap = devm_regmap_init_i2c(client, &sm5714_typec_regmap_config);
	if (IS_ERR(tc->regmap))
		return dev_err_probe(dev, PTR_ERR(tc->regmap),
				     "failed to init regmap\n");

	tc->vbus = devm_regulator_get(dev, "vbus");
	if (IS_ERR(tc->vbus))
		return dev_err_probe(dev, PTR_ERR(tc->vbus),
				     "failed to get VBUS regulator\n");

	ret = sm5714_typec_get_mfd(tc);
	if (ret)
		return ret;

	psy_cfg.drv_data = tc;
	psy_cfg.supplied_to = sm5714_typec_supplied_to;
	psy_cfg.num_supplicants = ARRAY_SIZE(sm5714_typec_supplied_to);
	tc->psy = devm_power_supply_register(dev, &sm5714_typec_psy_desc,
					     &psy_cfg);
	if (IS_ERR(tc->psy))
		return dev_err_probe(dev, PTR_ERR(tc->psy),
				     "failed to register power supply\n");

	tc->tcpc.fwnode = device_get_named_child_node(dev, "connector");
	if (!tc->tcpc.fwnode)
		return dev_err_probe(dev, -ENODEV, "missing connector node\n");
	ret = devm_add_action_or_reset(dev, sm5714_typec_put_fwnode,
				       tc->tcpc.fwnode);
	if (ret)
		return ret;

	tc->tcpc.init = sm5714_tcpc_init;
	tc->tcpc.get_vbus = sm5714_tcpc_get_vbus;
	tc->tcpc.get_cc = sm5714_tcpc_get_cc;
	tc->tcpc.set_cc = sm5714_tcpc_set_cc;
	tc->tcpc.set_polarity = sm5714_tcpc_set_polarity;
	tc->tcpc.set_vconn = sm5714_tcpc_set_vconn;
	tc->tcpc.set_vbus = sm5714_tcpc_set_vbus;
	tc->tcpc.set_current_limit = sm5714_tcpc_set_current_limit;
	tc->tcpc.set_pd_rx = sm5714_tcpc_set_pd_rx;
	tc->tcpc.set_roles = sm5714_tcpc_set_roles;
	tc->tcpc.start_toggling = sm5714_tcpc_start_toggling;
	tc->tcpc.pd_transmit = sm5714_tcpc_pd_transmit;

	port = tcpm_register_port(dev, &tc->tcpc);
	if (IS_ERR(port))
		return dev_err_probe(dev, PTR_ERR(port),
				     "failed to register TCPM port\n");
	WRITE_ONCE(tc->tcpm_port, port);
	ret = devm_add_action_or_reset(dev, sm5714_typec_unregister, tc);
	if (ret)
		return ret;

	ret = devm_request_threaded_irq(dev, client->irq, NULL, sm5714_typec_irq,
					IRQF_ONESHOT, "sm5714-typec", tc);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request IRQ\n");

	/* Anything that happened before the IRQ was ours */
	tcpm_cc_change(tc->tcpm_port);
	tcpm_vbus_change(tc->tcpm_port);
	return 0;
}

static const struct of_device_id sm5714_typec_of_match[] = {
	{ .compatible = "siliconmitus,sm5714-typec" },
	{ }
};
MODULE_DEVICE_TABLE(of, sm5714_typec_of_match);

static struct i2c_driver sm5714_typec_driver = {
	.driver = {
		.name = "sm5714-typec",
		.of_match_table = sm5714_typec_of_match,
	},
	.probe = sm5714_typec_probe,
};
module_i2c_driver(sm5714_typec_driver);

MODULE_DESCRIPTION("Silicon Mitus SM5714 USB Type-C / PD port (TCPM)");
MODULE_LICENSE("GPL");
