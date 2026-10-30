// SPDX-License-Identifier: GPL-2.0-only
/*
 * Silicon Mitus SM5714 IF-PMIC core driver
 *
 * The SM5714 answers on three I2C addresses: the charger block (the main
 * address, 0x49), the fuel gauge (0x71) and the MUIC (0x25). This core
 * creates regmaps for all three and spawns the function drivers.
 *
 * Interrupts (TLMM 84, shared by all blocks) are not handled yet; the
 * function drivers poll.
 *
 * Based on Samsung's downstream sm5714_core.c (GPL-2.0).
 */

#include <linux/bitfield.h>
#include <linux/i2c.h>
#include <linux/mfd/core.h>
#include <linux/mfd/sm5714.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regmap.h>

static const struct regmap_config sm5714_chg_regmap_config = {
	.name = "chg",
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0x50,
};

static const struct regmap_config sm5714_muic_regmap_config = {
	.name = "muic",
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0x58,
};

/* Fuel gauge: 8-bit address, 16-bit little-endian value (SMBus word) */
static const struct regmap_config sm5714_fg_regmap_config = {
	.name = "fg",
	.reg_bits = 8,
	.val_bits = 16,
	.val_format_endian = REGMAP_ENDIAN_LITTLE,
	.max_register = 0x91,
};

static const struct mfd_cell sm5714_devs[] = {
	MFD_CELL_OF("sm5714-fuel-gauge", NULL, NULL, 0, 0,
		    "siliconmitus,sm5714-fuel-gauge"),
	/* OTG VBUS boost; looks up the "regulators" child of this node */
	MFD_CELL_NAME("sm5714-regulator"),
};

/**
 * sm5714_muic_set_usb_path() - switch the MUIC D+/D- path
 * @sm5714: core handle
 * @usb: true routes D+/D- to the USB controller in manual mode (needed for
 *       OTG host, where no BC1.2 detection closes the switch); false
 *       returns the switch to automatic mode.
 */
int sm5714_muic_set_usb_path(struct sm5714 *sm5714, bool usb)
{
	if (usb)
		return regmap_update_bits(sm5714->regmap_muic,
					  SM5714_MUIC_REG_MANUAL_SW,
					  SM5714_MUIC_MANUAL_SW_MANUAL |
					  SM5714_MUIC_MANUAL_SW_PATH,
					  SM5714_MUIC_MANUAL_SW_MANUAL |
					  SM5714_MUIC_MANUAL_SW_USB);

	return regmap_clear_bits(sm5714->regmap_muic, SM5714_MUIC_REG_MANUAL_SW,
				 SM5714_MUIC_MANUAL_SW_MANUAL);
}
EXPORT_SYMBOL_GPL(sm5714_muic_set_usb_path);

static int sm5714_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct i2c_client *fg_client, *muic_client;
	struct sm5714 *sm5714;
	unsigned int val;
	int ret;

	sm5714 = devm_kzalloc(dev, sizeof(*sm5714), GFP_KERNEL);
	if (!sm5714)
		return -ENOMEM;

	sm5714->dev = dev;
	i2c_set_clientdata(client, sm5714);

	sm5714->regmap_chg = devm_regmap_init_i2c(client, &sm5714_chg_regmap_config);
	if (IS_ERR(sm5714->regmap_chg))
		return dev_err_probe(dev, PTR_ERR(sm5714->regmap_chg),
				     "failed to init charger regmap\n");

	ret = regmap_read(sm5714->regmap_chg, SM5714_CHG_REG_DEVICEID, &val);
	if (ret)
		return dev_err_probe(dev, ret, "failed to read device ID\n");
	if (FIELD_GET(SM5714_CHG_DEVICEID_VENDOR, val) != 0x1)
		return dev_err_probe(dev, -ENODEV, "unknown device ID 0x%02x\n", val);
	sm5714->rev = FIELD_GET(SM5714_CHG_DEVICEID_REV, val);

	fg_client = devm_i2c_new_dummy_device(dev, client->adapter,
					      SM5714_I2C_ADDR_FG);
	if (IS_ERR(fg_client))
		return dev_err_probe(dev, PTR_ERR(fg_client),
				     "failed to create fuel gauge client\n");

	sm5714->regmap_fg = devm_regmap_init_i2c(fg_client, &sm5714_fg_regmap_config);
	if (IS_ERR(sm5714->regmap_fg))
		return dev_err_probe(dev, PTR_ERR(sm5714->regmap_fg),
				     "failed to init fuel gauge regmap\n");

	ret = regmap_read(sm5714->regmap_fg, SM5714_FG_REG_DEVICE_ID, &val);
	if (ret || val > 0xff)
		return dev_err_probe(dev, ret ?: -ENODEV,
				     "fuel gauge not responding (0x%04x)\n", val);

	muic_client = devm_i2c_new_dummy_device(dev, client->adapter,
						SM5714_I2C_ADDR_MUIC);
	if (IS_ERR(muic_client))
		return dev_err_probe(dev, PTR_ERR(muic_client),
				     "failed to create MUIC client\n");

	sm5714->regmap_muic = devm_regmap_init_i2c(muic_client,
						   &sm5714_muic_regmap_config);
	if (IS_ERR(sm5714->regmap_muic))
		return dev_err_probe(dev, PTR_ERR(sm5714->regmap_muic),
				     "failed to init MUIC regmap\n");

	dev_info(dev, "SM5714 rev %u\n", sm5714->rev);

	return devm_mfd_add_devices(dev, PLATFORM_DEVID_AUTO, sm5714_devs,
				    ARRAY_SIZE(sm5714_devs), NULL, 0, NULL);
}

static const struct of_device_id sm5714_of_match[] = {
	{ .compatible = "siliconmitus,sm5714" },
	{ }
};
MODULE_DEVICE_TABLE(of, sm5714_of_match);

static const struct i2c_device_id sm5714_id[] = {
	{ "sm5714" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sm5714_id);

static struct i2c_driver sm5714_driver = {
	.driver = {
		.name = "sm5714",
		.of_match_table = sm5714_of_match,
	},
	.probe = sm5714_probe,
	.id_table = sm5714_id,
};
module_i2c_driver(sm5714_driver);

MODULE_DESCRIPTION("Silicon Mitus SM5714 IF-PMIC core driver");
MODULE_LICENSE("GPL");
