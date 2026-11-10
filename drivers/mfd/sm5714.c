// SPDX-License-Identifier: GPL-2.0-only
/*
 * Silicon Mitus SM5714 IF-PMIC core driver
 *
 * The SM5714 answers on three I2C addresses: the charger block (the main
 * address, 0x49), the fuel gauge (0x71) and the MUIC (0x25). This core
 * creates regmaps for all three and spawns the function drivers.
 *
 * Interrupts: one open-drain, active-low line (TLMM 84, needs a pull-up)
 * shared by all three blocks. regmap_irq handles one regmap per chip, so
 * there are two chips sharing the line (IRQF_SHARED): CHG (INT1..5) and
 * MUIC (INT1..2). Each reads and clears its own registers on every edge;
 * the line goes high once both are clear.
 *
 * FG: the gauge also pulls the line, and only lets go once INTFG has been
 * read (even if it reads 0), like CHG INT1..5. It can also reset itself,
 * which restores its default INTFG_MASK (0x0001) and unmasks its events.
 * A third shared handler therefore reads INTFG on every edge and puts the
 * mask back (Samsung reads INTFG whenever INT_SOURCE says FG). Without it
 * the line stuck low and the kernel disabled the IRQ ("nobody cared").
 *
 * Measured on gts7xllite: the line releases only after CHG INT1..INT5 have
 * all been read (even when they read 0), so the CHG chip must cover all
 * five status registers.
 *
 * The charger cell gets IRQs from both chips. mfd core maps resources with
 * one irq_domain only, so the charger's resources are built at probe from
 * already-mapped virqs and the cells are added with no domain.
 *
 * Based on Samsung's downstream sm5714_core.c / sm5714-irq.c (GPL-2.0).
 */

#include <linux/bitfield.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
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

/* Interrupt registers (local names to stay clear of the shared header) */
#define SM5714_CORE_CHG_INT1		0x01	/* INT1..INT5, clear on read */
#define SM5714_CORE_CHG_INTMSK1		0x07	/* INTMSK1..5, 1 = masked */
#define SM5714_CORE_CHG_NUM_INT_REGS	5
#define SM5714_CORE_MUIC_INT1		0x01	/* INT1..INT2, clear on read */
#define SM5714_CORE_MUIC_INTMASK1	0x03
#define SM5714_CORE_MUIC_INTMASK2	0x04
#define SM5714_CORE_MUIC_NUM_INT_REGS	2
#define SM5714_CORE_FG_INTFG		0x02	/* clear on read */
#define SM5714_CORE_FG_INTFG_MASK	0x04
#define SM5714_CORE_FG_MASK_ALL		0x000f	/* as Samsung's irq_init */
#define SM5714_CORE_CHG_INT_SOURCE	0x00
#define  SM5714_CORE_INT_SOURCE_FG	BIT(2)

enum {
	SM5714_CORE_IRQ_VBUSPOK,
	SM5714_CORE_IRQ_VBUSUVLO,
	SM5714_CORE_IRQ_VBUSOVP,
	SM5714_CORE_IRQ_AICL,
	SM5714_CORE_IRQ_CHGON,
	SM5714_CORE_IRQ_TOPOFF,
	SM5714_CORE_IRQ_DONE,
	SM5714_CORE_IRQ_OTGFAIL,
};

enum {
	SM5714_CORE_MUIC_IRQ_CHGTYPE,		/* BC1.2 result in DEVICETYPE1/2 */
	SM5714_CORE_MUIC_IRQ_DETACH,		/* VBUS / RID detach */
};

/* reg_offset is relative to INT1 (0x01): 0 = INT1, 1 = INT2, 2 = INT3 */
static const struct regmap_irq sm5714_irqs[] = {
	REGMAP_IRQ_REG(SM5714_CORE_IRQ_VBUSPOK,  0, BIT(0)),
	REGMAP_IRQ_REG(SM5714_CORE_IRQ_VBUSUVLO, 0, BIT(1)),
	REGMAP_IRQ_REG(SM5714_CORE_IRQ_VBUSOVP,  0, BIT(2)),
	REGMAP_IRQ_REG(SM5714_CORE_IRQ_AICL,     1, BIT(0)),
	REGMAP_IRQ_REG(SM5714_CORE_IRQ_CHGON,    1, BIT(3)),
	REGMAP_IRQ_REG(SM5714_CORE_IRQ_TOPOFF,   1, BIT(5)),
	REGMAP_IRQ_REG(SM5714_CORE_IRQ_DONE,     1, BIT(6)),
	REGMAP_IRQ_REG(SM5714_CORE_IRQ_OTGFAIL,  2, BIT(2)),
};

static const struct regmap_irq_chip sm5714_irq_chip = {
	.name = "sm5714",
	.status_base = SM5714_CORE_CHG_INT1,
	.mask_base = SM5714_CORE_CHG_INTMSK1,
	/* all five, even if no irq lives in INT4/INT5: see header comment */
	.num_regs = SM5714_CORE_CHG_NUM_INT_REGS,
	.irqs = sm5714_irqs,
	.num_irqs = ARRAY_SIZE(sm5714_irqs),
};

/* MUIC: reg_offset 0 = INT1 (0x01), 1 = INT2 (0x02) */
static const struct regmap_irq sm5714_muic_irqs[] = {
	REGMAP_IRQ_REG(SM5714_CORE_MUIC_IRQ_CHGTYPE, 0, BIT(1)),
	REGMAP_IRQ_REG(SM5714_CORE_MUIC_IRQ_DETACH,  0, BIT(4)),
};

static const struct regmap_irq_chip sm5714_muic_irq_chip = {
	.name = "sm5714-muic",
	.domain_suffix = "muic",	/* second domain on the same fwnode */
	.status_base = SM5714_CORE_MUIC_INT1,
	.mask_base = SM5714_CORE_MUIC_INTMASK1,
	.num_regs = SM5714_CORE_MUIC_NUM_INT_REGS,
	.irqs = sm5714_muic_irqs,
	.num_irqs = ARRAY_SIZE(sm5714_muic_irqs),
};

#define SM5714_CORE_CELL_CHARGER	2
static const struct mfd_cell sm5714_devs[] = {
	MFD_CELL_OF("sm5714-fuel-gauge", NULL, NULL, 0, 0,
		    "siliconmitus,sm5714-fuel-gauge"),
	/* OTG VBUS boost; looks up the "regulators" child of this node */
	MFD_CELL_NAME("sm5714-regulator"),
	/* [SM5714_CORE_CELL_CHARGER]: IRQ resources added at probe */
	MFD_CELL_NAME("sm5714-charger"),
};

/* Charger cell IRQs: name, which chip, hwirq */
static const struct {
	const char *name;
	bool muic;
	int hwirq;
} sm5714_chg_irq_map[] = {
	{ "vbuspok",  false, SM5714_CORE_IRQ_VBUSPOK },
	{ "vbusuvlo", false, SM5714_CORE_IRQ_VBUSUVLO },
	{ "chgon",    false, SM5714_CORE_IRQ_CHGON },
	{ "done",     false, SM5714_CORE_IRQ_DONE },
	{ "chgtype",  true,  SM5714_CORE_MUIC_IRQ_CHGTYPE },
	{ "muic-detach", true, SM5714_CORE_MUIC_IRQ_DETACH },
};

/* Shared-line handler for the fuel gauge block (see header comment) */
static irqreturn_t sm5714_fg_irq(int irq, void *data)
{
	struct sm5714 *sm5714 = data;
	unsigned int src = 0, intfg = 0, mask;

	regmap_read(sm5714->regmap_chg, SM5714_CORE_CHG_INT_SOURCE, &src);
	/* the read is what releases the line */
	regmap_read(sm5714->regmap_fg, SM5714_CORE_FG_INTFG, &intfg);

	if (!regmap_read(sm5714->regmap_fg, SM5714_CORE_FG_INTFG_MASK, &mask) &&
	    mask != SM5714_CORE_FG_MASK_ALL) {
		dev_warn_ratelimited(sm5714->dev,
				     "FG INTFG_MASK was 0x%04x (gauge reset?), re-masking\n",
				     mask);
		regmap_write(sm5714->regmap_fg, SM5714_CORE_FG_INTFG_MASK,
			     SM5714_CORE_FG_MASK_ALL);
		return IRQ_HANDLED;
	}

	if ((src & SM5714_CORE_INT_SOURCE_FG) || intfg) {
		dev_dbg(sm5714->dev, "FG irq: INT_SOURCE 0x%02x INTFG 0x%04x\n",
			src, intfg);
		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}

static int sm5714_add_devices_irq(struct sm5714 *sm5714,
				  struct regmap_irq_chip_data *chg_irqs,
				  struct regmap_irq_chip_data *muic_irqs)
{
	struct device *dev = sm5714->dev;
	struct mfd_cell *cells;
	struct resource *res;
	int i, virq;

	res = devm_kcalloc(dev, ARRAY_SIZE(sm5714_chg_irq_map), sizeof(*res),
			   GFP_KERNEL);
	cells = devm_kmemdup(dev, sm5714_devs, sizeof(sm5714_devs), GFP_KERNEL);
	if (!res || !cells)
		return -ENOMEM;

	for (i = 0; i < ARRAY_SIZE(sm5714_chg_irq_map); i++) {
		virq = regmap_irq_get_virq(sm5714_chg_irq_map[i].muic ?
					   muic_irqs : chg_irqs,
					   sm5714_chg_irq_map[i].hwirq);
		if (virq <= 0)
			return dev_err_probe(dev, virq ?: -EINVAL,
					     "no virq for %s\n",
					     sm5714_chg_irq_map[i].name);
		res[i] = DEFINE_RES_IRQ_NAMED(virq, sm5714_chg_irq_map[i].name);
	}

	cells[SM5714_CORE_CELL_CHARGER].resources = res;
	cells[SM5714_CORE_CELL_CHARGER].num_resources =
		ARRAY_SIZE(sm5714_chg_irq_map);

	/* No domain: resources already hold Linux IRQ numbers */
	return devm_mfd_add_devices(dev, PLATFORM_DEVID_AUTO, cells,
				    ARRAY_SIZE(sm5714_devs), NULL, 0, NULL);
}

/*
 * Mask every interrupt source in all three blocks and clear whatever the
 * bootloader left pending. Otherwise a stale (and, for MUIC/FG, never
 * handled) flag holds the level line low and the IRQ storms.
 */
static int sm5714_irq_quiesce(struct sm5714 *sm5714)
{
	u8 buf[SM5714_CORE_CHG_NUM_INT_REGS];
	unsigned int val;
	int ret, i;

	for (i = 0; i < SM5714_CORE_CHG_NUM_INT_REGS; i++) {
		ret = regmap_write(sm5714->regmap_chg,
				   SM5714_CORE_CHG_INTMSK1 + i, 0xff);
		if (ret)
			return ret;
	}

	ret = regmap_write(sm5714->regmap_muic, SM5714_CORE_MUIC_INTMASK1, 0xff);
	if (!ret)
		ret = regmap_write(sm5714->regmap_muic, SM5714_CORE_MUIC_INTMASK2, 0xff);
	if (!ret)
		ret = regmap_write(sm5714->regmap_fg, SM5714_CORE_FG_INTFG_MASK,
				   SM5714_CORE_FG_MASK_ALL);
	if (ret)
		return ret;

	/* Read-to-clear */
	ret = regmap_bulk_read(sm5714->regmap_chg, SM5714_CORE_CHG_INT1,
			       buf, sizeof(buf));
	if (ret)
		return ret;
	for (i = 0; i < 2; i++) {
		ret = regmap_read(sm5714->regmap_muic, SM5714_CORE_MUIC_INT1 + i, &val);
		if (ret)
			return ret;
	}
	return regmap_read(sm5714->regmap_fg, SM5714_CORE_FG_INTFG, &val);
}

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
	struct regmap_irq_chip_data *chg_irqs, *muic_irqs;
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

	ret = sm5714_irq_quiesce(sm5714);
	if (ret)
		return dev_err_probe(dev, ret, "failed to mask interrupts\n");

	if (client->irq <= 0) {
		dev_info(dev, "no interrupt, function drivers poll\n");
		/* plain cells: no IRQ resources at all */
		return devm_mfd_add_devices(dev, PLATFORM_DEVID_AUTO,
					    sm5714_devs, ARRAY_SIZE(sm5714_devs),
					    NULL, 0, NULL);
	}

	/*
	 * Trigger type (level low) comes from DT. Both chips share the line;
	 * regmap_irq adds IRQF_ONESHOT itself, and both must agree on flags.
	 */
	ret = devm_regmap_add_irq_chip(dev, sm5714->regmap_chg, client->irq,
				       IRQF_ONESHOT | IRQF_SHARED, 0,
				       &sm5714_irq_chip, &chg_irqs);
	if (ret)
		return dev_err_probe(dev, ret, "failed to add CHG IRQ chip\n");

	ret = devm_regmap_add_irq_chip(dev, sm5714->regmap_muic, client->irq,
				       IRQF_ONESHOT | IRQF_SHARED, 0,
				       &sm5714_muic_irq_chip, &muic_irqs);
	if (ret)
		return dev_err_probe(dev, ret, "failed to add MUIC IRQ chip\n");

	/* Same flags as regmap_irq uses: the line is shared by all three */
	ret = devm_request_threaded_irq(dev, client->irq, NULL, sm5714_fg_irq,
					IRQF_ONESHOT | IRQF_SHARED, "sm5714-fg",
					sm5714);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request FG IRQ\n");

	return sm5714_add_devices_irq(sm5714, chg_irqs, muic_irqs);
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
