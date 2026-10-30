// SPDX-License-Identifier: GPL-2.0-only
/*
 * Silicon Mitus SM5714 fuel gauge driver
 *
 * Read-only for now: the gauge keeps its battery model and state across
 * reboots (it runs from the battery), so as long as SYSTEM_STATUS reports
 * INIT_DONE we only read SRAM. Loading the model on a cold gauge is not
 * implemented.
 *
 * Measurements live in an internal SRAM, read by writing the SRAM address
 * to SRAM_RADDR and reading SRAM_RDATA. Conversions are taken from
 * Samsung's downstream sm5714_fuelgauge.c (GPL-2.0).
 */

#include <linux/math64.h>
#include <linux/mfd/sm5714.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/regmap.h>

struct sm5714_fg {
	struct device *dev;
	struct regmap *regmap;		/* fuel gauge block */
	struct regmap *regmap_chg;	/* charger block, for STATUS */
	struct power_supply *psy;
	struct power_supply_battery_info *bat_info;
	struct mutex sram_lock;		/* RADDR/RDATA pair must not interleave */
};

static int sm5714_fg_read_sram(struct sm5714_fg *fg, unsigned int addr,
			       unsigned int *val)
{
	int ret;

	mutex_lock(&fg->sram_lock);
	ret = regmap_write(fg->regmap, SM5714_FG_REG_SRAM_RADDR, addr);
	if (!ret)
		ret = regmap_read(fg->regmap, SM5714_FG_REG_SRAM_RDATA, val);
	mutex_unlock(&fg->sram_lock);

	return ret;
}

/* VBAT/VBAT_AVG raw -> uV: 2700 mV + raw / 10.9, bit 15 = below 2700 mV */
static int sm5714_fg_raw_to_uv(unsigned int raw)
{
	int mv;

	if (raw & BIT(15))
		mv = 2700 - (int)(raw & 0x7fff) * 10 / 109;
	else
		mv = (int)raw * 10 / 109 + 2700;

	return mv * 1000;
}

/* CURRENT/CURRENT_AVG raw -> uA: raw / 2.044 mA, bit 15 = discharging */
static int sm5714_fg_raw_to_ua(struct sm5714_fg *fg, unsigned int raw)
{
	unsigned int aux;
	s64 ua;

	ua = div_s64((s64)(raw & 0x7fff) * 1000000, 2044);

	if (!regmap_read(fg->regmap, SM5714_FG_REG_AUX_STAT, &aux) &&
	    (aux & SM5714_FG_AUX_STAT_CURR_X2_5))
		ua = div_s64(ua * 25, 10);

	if (raw & BIT(15))
		ua = -ua;

	return ua;
}

static int sm5714_fg_get_status(struct sm5714_fg *fg, int *status)
{
	unsigned int st1, st2, raw;
	int ret;

	ret = regmap_read(fg->regmap_chg, SM5714_CHG_REG_STATUS1, &st1);
	if (ret)
		return ret;
	ret = regmap_read(fg->regmap_chg, SM5714_CHG_REG_STATUS2, &st2);
	if (ret)
		return ret;

	if (!(st1 & SM5714_CHG_STATUS1_VBUSPOK)) {
		*status = POWER_SUPPLY_STATUS_DISCHARGING;
		return 0;
	}

	if (st2 & SM5714_CHG_STATUS2_CHGON) {
		*status = POWER_SUPPLY_STATUS_CHARGING;
		return 0;
	}

	/* Cable in, charger not charging: full if the gauge says so */
	ret = sm5714_fg_read_sram(fg, SM5714_FG_SRAM_SOC, &raw);
	if (ret)
		return ret;

	*status = (raw >= 0x6400) ? POWER_SUPPLY_STATUS_FULL
				  : POWER_SUPPLY_STATUS_NOT_CHARGING;
	return 0;
}

/*
 * SOC in 1/256 % (SRAM 8.8 fixed point), clamped to 0..100 %. Used to
 * scale the design capacity into CHARGE_NOW / ENERGY_NOW with more
 * resolution than the integer CAPACITY.
 */
static int sm5714_fg_get_soc_fine(struct sm5714_fg *fg, unsigned int *soc)
{
	unsigned int raw;
	int ret;

	ret = sm5714_fg_read_sram(fg, SM5714_FG_SRAM_SOC, &raw);
	if (ret)
		return ret;

	*soc = min(raw, 100U << 8);
	return 0;
}

/* full * soc / (100 * 256), full in uAh or uWh */
static int sm5714_fg_scale_soc(int full, unsigned int soc)
{
	return div_u64((u64)full * soc, 100 * 256);
}

static int sm5714_fg_get_property(struct power_supply *psy,
				  enum power_supply_property psp,
				  union power_supply_propval *val)
{
	struct sm5714_fg *fg = power_supply_get_drvdata(psy);
	struct power_supply_battery_info *bi = fg->bat_info;
	unsigned int raw, soc;
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		return sm5714_fg_get_status(fg, &val->intval);

	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		break;

	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;

	case POWER_SUPPLY_PROP_CAPACITY:
		/* SOC: 8.8 fixed point percent */
		ret = sm5714_fg_read_sram(fg, SM5714_FG_SRAM_SOC, &raw);
		if (!ret)
			val->intval = clamp_t(int, raw >> 8, 0, 100);
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = sm5714_fg_read_sram(fg, SM5714_FG_SRAM_VBAT, &raw);
		if (!ret)
			val->intval = sm5714_fg_raw_to_uv(raw);
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_AVG:
		ret = sm5714_fg_read_sram(fg, SM5714_FG_SRAM_VBAT_AVG, &raw);
		if (!ret)
			val->intval = sm5714_fg_raw_to_uv(raw);
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_OCV:
		/* OCV: raw / 2.048 mV */
		ret = sm5714_fg_read_sram(fg, SM5714_FG_SRAM_OCV, &raw);
		if (!ret)
			val->intval = (int)(((u64)raw * 1000000) >> 11);
		break;

	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = sm5714_fg_read_sram(fg, SM5714_FG_SRAM_CURRENT, &raw);
		if (!ret)
			val->intval = sm5714_fg_raw_to_ua(fg, raw);
		break;

	case POWER_SUPPLY_PROP_CURRENT_AVG:
		ret = sm5714_fg_read_sram(fg, SM5714_FG_SRAM_CURRENT_AVG, &raw);
		if (!ret)
			val->intval = sm5714_fg_raw_to_ua(fg, raw);
		break;

	case POWER_SUPPLY_PROP_TEMP:
		/*
		 * Gauge die temperature in 0.1 degC, not the battery NTC
		 * (that is on the PM6150L ADC and not wired up yet).
		 */
		ret = sm5714_fg_read_sram(fg, SM5714_FG_SRAM_TEMPERATURE, &raw);
		if (!ret) {
			int t = ((raw & 0x7fff) * 10 * 2989) >> 19;

			val->intval = (raw & BIT(15)) ? -t : t;
		}
		break;

	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		ret = sm5714_fg_read_sram(fg, SM5714_FG_SRAM_SOC_CYCLE, &raw);
		if (!ret)
			val->intval = raw & 0xff;
		break;

	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		if (!fg->bat_info || fg->bat_info->charge_full_design_uah < 0)
			return -ENODATA;
		val->intval = fg->bat_info->charge_full_design_uah;
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		if (!fg->bat_info || fg->bat_info->voltage_max_design_uv < 0)
			return -ENODATA;
		val->intval = fg->bat_info->voltage_max_design_uv;
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		if (!fg->bat_info || fg->bat_info->voltage_min_design_uv < 0)
			return -ENODATA;
		val->intval = fg->bat_info->voltage_min_design_uv;
		break;

	/*
	 * The gauge's learned full capacity is not decoded yet, so "full"
	 * is the design value and CHARGE_NOW / ENERGY_NOW are SOC-scaled.
	 */
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		if (!bi || bi->charge_full_design_uah <= 0)
			return -ENODATA;
		val->intval = bi->charge_full_design_uah;
		break;

	case POWER_SUPPLY_PROP_CHARGE_NOW:
		if (!bi || bi->charge_full_design_uah <= 0)
			return -ENODATA;
		ret = sm5714_fg_get_soc_fine(fg, &soc);
		if (!ret)
			val->intval = sm5714_fg_scale_soc(bi->charge_full_design_uah, soc);
		break;

	case POWER_SUPPLY_PROP_ENERGY_FULL_DESIGN:
	case POWER_SUPPLY_PROP_ENERGY_FULL:
		if (!bi || bi->energy_full_design_uwh <= 0)
			return -ENODATA;
		val->intval = bi->energy_full_design_uwh;
		break;

	case POWER_SUPPLY_PROP_ENERGY_NOW:
		if (!bi || bi->energy_full_design_uwh <= 0)
			return -ENODATA;
		ret = sm5714_fg_get_soc_fine(fg, &soc);
		if (!ret)
			val->intval = sm5714_fg_scale_soc(bi->energy_full_design_uwh, soc);
		break;

	case POWER_SUPPLY_PROP_POWER_NOW: {
		/* V * I, same sign as CURRENT_NOW (negative = discharging) */
		unsigned int vraw;
		int uv, ua;

		ret = sm5714_fg_read_sram(fg, SM5714_FG_SRAM_VBAT, &vraw);
		if (!ret)
			ret = sm5714_fg_read_sram(fg, SM5714_FG_SRAM_CURRENT, &raw);
		if (ret)
			break;
		uv = sm5714_fg_raw_to_uv(vraw);
		ua = sm5714_fg_raw_to_ua(fg, raw);
		val->intval = div_s64((s64)uv * ua, 1000000);
		break;
	}

	default:
		return -EINVAL;
	}

	return ret;
}

static const enum power_supply_property sm5714_fg_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_AVG,
	POWER_SUPPLY_PROP_VOLTAGE_OCV,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_ENERGY_FULL_DESIGN,
	POWER_SUPPLY_PROP_ENERGY_FULL,
	POWER_SUPPLY_PROP_ENERGY_NOW,
	POWER_SUPPLY_PROP_POWER_NOW,
};

static const struct power_supply_desc sm5714_fg_desc = {
	.name = "sm5714-battery",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = sm5714_fg_props,
	.num_properties = ARRAY_SIZE(sm5714_fg_props),
	.get_property = sm5714_fg_get_property,
};

static int sm5714_fg_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sm5714 *sm5714 = dev_get_drvdata(dev->parent);
	struct power_supply_config psy_cfg = {};
	struct sm5714_fg *fg;
	unsigned int status;
	int ret;

	fg = devm_kzalloc(dev, sizeof(*fg), GFP_KERNEL);
	if (!fg)
		return -ENOMEM;

	fg->dev = dev;
	fg->regmap = sm5714->regmap_fg;
	fg->regmap_chg = sm5714->regmap_chg;

	ret = devm_mutex_init(dev, &fg->sram_lock);
	if (ret)
		return ret;

	ret = regmap_read(fg->regmap, SM5714_FG_REG_SYSTEM_STATUS, &status);
	if (ret)
		return dev_err_probe(dev, ret, "failed to read SYSTEM_STATUS\n");

	if (!(status & SM5714_FG_SYSTEM_STATUS_INIT_DONE))
		dev_warn(dev, "gauge not initialised (0x%04x), readings may be wrong; model loading not implemented\n",
			 status);
	if (status & SM5714_FG_SYSTEM_STATUS_JIG)
		dev_warn(dev, "jig mode: VBAT reports VSYS\n");

	psy_cfg.drv_data = fg;
	psy_cfg.fwnode = dev_fwnode(dev);

	fg->psy = devm_power_supply_register(dev, &sm5714_fg_desc, &psy_cfg);
	if (IS_ERR(fg->psy))
		return dev_err_probe(dev, PTR_ERR(fg->psy),
				     "failed to register power supply\n");

	/* Optional: only feeds the *_DESIGN properties */
	ret = power_supply_get_battery_info(fg->psy, &fg->bat_info);
	if (ret) {
		dev_warn(dev, "no monitored-battery: %d\n", ret);
		fg->bat_info = NULL;
	}

	return 0;
}

static const struct of_device_id sm5714_fg_of_match[] = {
	{ .compatible = "siliconmitus,sm5714-fuel-gauge" },
	{ }
};
MODULE_DEVICE_TABLE(of, sm5714_fg_of_match);

static struct platform_driver sm5714_fg_driver = {
	.driver = {
		.name = "sm5714-fuel-gauge",
		.of_match_table = sm5714_fg_of_match,
	},
	.probe = sm5714_fg_probe,
};
module_platform_driver(sm5714_fg_driver);

MODULE_DESCRIPTION("Silicon Mitus SM5714 fuel gauge driver");
MODULE_LICENSE("GPL");
