// SPDX-License-Identifier: GPL-2.0-only
/*
 * Silicon Mitus SM5714 USB OTG VBUS boost regulator
 *
 * In OTG mode the charger runs in reverse and boosts the battery onto VBUS.
 * Sequence taken from Samsung's sm5714_charger_oper.c (USB_OTG table row):
 * BSTOUT = 5.1 V, OTG_CURRENT = 900 mA, then OP_MODE = USB_OTG (0x7).
 * Disabling returns OP_MODE to CHG_ON_VBUS (0x5).
 *
 * Verified on gts7xllite by hand: BOOSTPOK (STATUS4 bit 1) sets, a USB
 * keyboard powers up and enumerates.
 */

#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/mfd/sm5714.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>

static int sm5714_otg_enable(struct regulator_dev *rdev)
{
	struct regmap *map = rdev_get_regmap(rdev);
	unsigned int val;
	int ret;

	/*
	 * Never fight an external source: refuse while something else
	 * (charger, PC) is already driving VBUS.
	 */
	ret = regmap_read(map, SM5714_CHG_REG_STATUS1, &val);
	if (ret)
		return ret;
	if (val & SM5714_CHG_STATUS1_VBUSPOK) {
		dev_warn(&rdev->dev, "external VBUS present, not boosting\n");
		return -EBUSY;
	}

	ret = regmap_update_bits(map, SM5714_CHG_REG_BSTCNTL1,
				 SM5714_CHG_BSTCNTL1_BSTOUT |
				 SM5714_CHG_BSTCNTL1_OTG_CURRENT,
				 FIELD_PREP(SM5714_CHG_BSTCNTL1_BSTOUT,
					    SM5714_CHG_BSTOUT_5100MV) |
				 FIELD_PREP(SM5714_CHG_BSTCNTL1_OTG_CURRENT,
					    SM5714_CHG_OTG_CURRENT_900MA));
	if (ret)
		return ret;

	return regmap_update_bits(map, SM5714_CHG_REG_CNTL2,
				  SM5714_CHG_CNTL2_OP_MODE,
				  SM5714_CHG_OP_MODE_USB_OTG);
}

static int sm5714_otg_disable(struct regulator_dev *rdev)
{
	return regmap_update_bits(rdev_get_regmap(rdev), SM5714_CHG_REG_CNTL2,
				  SM5714_CHG_CNTL2_OP_MODE,
				  SM5714_CHG_OP_MODE_CHG_ON_VBUS);
}

static int sm5714_otg_is_enabled(struct regulator_dev *rdev)
{
	unsigned int val;
	int ret;

	ret = regmap_read(rdev_get_regmap(rdev), SM5714_CHG_REG_CNTL2, &val);
	if (ret)
		return ret;

	return FIELD_GET(SM5714_CHG_CNTL2_OP_MODE, val) ==
	       SM5714_CHG_OP_MODE_USB_OTG;
}

static int sm5714_otg_get_error_flags(struct regulator_dev *rdev,
				      unsigned int *flags)
{
	struct regmap *map = rdev_get_regmap(rdev);
	unsigned int st3;
	int ret;

	*flags = 0;
	ret = regmap_read(map, SM5714_CHG_REG_STATUS3, &st3);
	if (ret)
		return ret;
	if (st3 & SM5714_CHG_STATUS3_OTGFAIL)
		*flags |= REGULATOR_ERROR_OVER_CURRENT;

	return 0;
}

static const struct regulator_ops sm5714_otg_ops = {
	.enable = sm5714_otg_enable,
	.disable = sm5714_otg_disable,
	.is_enabled = sm5714_otg_is_enabled,
	.get_error_flags = sm5714_otg_get_error_flags,
};

static const struct regulator_desc sm5714_otg_desc = {
	.name = "usb-otg-vbus",
	.of_match = "usb-otg-vbus",
	.regulators_node = "regulators",
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
	.ops = &sm5714_otg_ops,
	.n_voltages = 1,
	.fixed_uV = 5100000,
	.enable_time = 20000,
};

static int sm5714_regulator_probe(struct platform_device *pdev)
{
	struct sm5714 *sm5714 = dev_get_drvdata(pdev->dev.parent);
	struct regulator_config config = { };
	struct regulator_dev *rdev;

	/* DT lookup happens on the parent's "regulators" child node */
	config.dev = pdev->dev.parent;
	config.regmap = sm5714->regmap_chg;

	rdev = devm_regulator_register(&pdev->dev, &sm5714_otg_desc, &config);
	if (IS_ERR(rdev))
		return dev_err_probe(&pdev->dev, PTR_ERR(rdev),
				     "failed to register OTG VBUS regulator\n");

	return 0;
}

static const struct platform_device_id sm5714_regulator_id[] = {
	{ "sm5714-regulator" },
	{ }
};
MODULE_DEVICE_TABLE(platform, sm5714_regulator_id);

static struct platform_driver sm5714_regulator_driver = {
	.driver = {
		.name = "sm5714-regulator",
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
	.probe = sm5714_regulator_probe,
	.id_table = sm5714_regulator_id,
};
module_platform_driver(sm5714_regulator_driver);

MODULE_DESCRIPTION("Silicon Mitus SM5714 USB OTG VBUS regulator");
MODULE_LICENSE("GPL");
