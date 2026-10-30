// SPDX-License-Identifier: GPL-2.0-only
/*
 * Silicon Mitus SM5714 USB Type-C port: CC detection only (no USB PD)
 *
 * The SM5714 PD block (own I2C address, own IRQ) runs the Type-C CC state
 * machine in hardware. In DRP mode it toggles Rp/Rd and reports the result
 * in CC_STATUS; this driver only reacts to ATTACH/DETACH:
 *
 *   partner has Rd (keyboard, hub, OTG adapter) -> we are source:
 *       VBUS boost on, MUIC D+/D- to USB, data role host
 *   partner has Rp (PC, charger)                -> we are sink:
 *       data role device (VBUS comes from the partner)
 *   detach                                      -> role none (= peripheral
 *       default), MUIC auto, VBUS off
 *
 * Register usage taken from Samsung's downstream sm5714_typec.c (GPL-2.0).
 * PD (power/data role swap, 9 V, DP alt mode) is left for a TCPM port.
 */

#include <linux/bitfield.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/mfd/sm5714.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/usb/role.h>
#include <linux/usb/typec.h>

#define SM5714_TC_REG_INT1		0x01	/* INT1..INT5, read to clear */
#define SM5714_TC_REG_INT_MASK1		0x06	/* INT_MASK1..5, 1 = masked */
#define SM5714_TC_REG_STATUS1		0x0b
#define SM5714_TC_REG_CC_STATUS		0x28
#define SM5714_TC_REG_CC_CNTL1		0x29
#define SM5714_TC_REG_CC_CNTL3		0x2b
#define SM5714_TC_REG_PD_CNTL4		0x3b
#define SM5714_TC_REG_PD_STATE3		0xd8
#define SM5714_TC_MAX_REGISTER		0xda

#define SM5714_TC_INT1_VBUSPOK		BIT(0)
#define SM5714_TC_INT1_ATTACH		BIT(3)
#define SM5714_TC_INT1_DETACH		BIT(4)

#define SM5714_TC_CC_ATTACH_TYPE	GENMASK(2, 0)
#define SM5714_TC_CC_ATTACH_NONE	0
#define SM5714_TC_CC_ATTACH_SNK		1	/* partner is Rp: we are sink */
#define SM5714_TC_CC_ATTACH_SRC		2	/* partner is Rd: we are source */
#define SM5714_TC_CC_ATTACH_AUDIO	3
#define SM5714_TC_CC_ADV_CURR		GENMASK(4, 3)
#define SM5714_TC_CC_FLIP		BIT(5)

#define SM5714_TC_CC_CNTL1_DRP		0x41	/* 0x45 = sink only (boot default) */
#define SM5714_TC_CC_CNTL3_OP_EN	0x80
#define SM5714_TC_PD_CNTL4_RESET_DONE	0x01

struct sm5714_typec {
	struct device *dev;
	struct regmap *regmap;
	struct sm5714 *sm5714;		/* MFD core, for the MUIC switch */
	struct regulator *vbus;
	struct usb_role_switch *role_sw;
	struct typec_port *port;
	struct typec_partner *partner;
	struct mutex lock;
	unsigned int state;		/* SM5714_TC_CC_ATTACH_* currently handled */
	bool vbus_on;
};

static const struct regmap_config sm5714_typec_regmap_config = {
	.name = "typec",
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = SM5714_TC_MAX_REGISTER,
};

static void sm5714_typec_set_vbus(struct sm5714_typec *tc, bool on)
{
	int ret;

	if (on == tc->vbus_on)
		return;

	ret = on ? regulator_enable(tc->vbus) : regulator_disable(tc->vbus);
	if (ret) {
		dev_err(tc->dev, "failed to turn VBUS %s: %d\n",
			on ? "on" : "off", ret);
		return;
	}
	tc->vbus_on = on;
}

static void sm5714_typec_register_partner(struct sm5714_typec *tc)
{
	struct typec_partner_desc desc = {
		.usb_pd = false,
		.accessory = TYPEC_ACCESSORY_NONE,
	};
	struct typec_partner *partner;

	if (tc->partner)
		return;

	partner = typec_register_partner(tc->port, &desc);
	if (IS_ERR(partner)) {
		dev_warn(tc->dev, "failed to register partner: %ld\n",
			 PTR_ERR(partner));
		return;
	}
	tc->partner = partner;
}

static void sm5714_typec_detach(struct sm5714_typec *tc)
{
	/* Data role first, so xHCI is gone before its port loses power */
	usb_role_switch_set_role(tc->role_sw, USB_ROLE_NONE);

	if (tc->state == SM5714_TC_CC_ATTACH_SRC) {
		sm5714_muic_set_usb_path(tc->sm5714, false);
		sm5714_typec_set_vbus(tc, false);
	}

	if (tc->partner) {
		typec_unregister_partner(tc->partner);
		tc->partner = NULL;
	}

	typec_set_orientation(tc->port, TYPEC_ORIENTATION_NONE);
	typec_set_pwr_opmode(tc->port, TYPEC_PWR_MODE_USB);
	typec_set_pwr_role(tc->port, TYPEC_SINK);
	typec_set_data_role(tc->port, TYPEC_DEVICE);

	dev_info(tc->dev, "detached\n");
	tc->state = SM5714_TC_CC_ATTACH_NONE;
}

static void sm5714_typec_attach_source(struct sm5714_typec *tc)
{
	/* VBUS before host mode: the device must be powered when xHCI scans */
	sm5714_typec_set_vbus(tc, true);
	sm5714_muic_set_usb_path(tc->sm5714, true);
	usb_role_switch_set_role(tc->role_sw, USB_ROLE_HOST);

	typec_set_pwr_role(tc->port, TYPEC_SOURCE);
	typec_set_data_role(tc->port, TYPEC_HOST);
	sm5714_typec_register_partner(tc);

	dev_info(tc->dev, "sink attached: host mode, VBUS %s\n",
		 tc->vbus_on ? "on" : "FAILED");
}

static void sm5714_typec_attach_sink(struct sm5714_typec *tc, unsigned int cc)
{
	static const char * const curr_name[] = { "default", "1.5A", "3.0A" };
	unsigned int curr = FIELD_GET(SM5714_TC_CC_ADV_CURR, cc);
	enum typec_pwr_opmode mode;

	switch (curr) {
	case 0:
		mode = TYPEC_PWR_MODE_USB;
		break;
	case 1:
		mode = TYPEC_PWR_MODE_1_5A;
		break;
	default:
		mode = TYPEC_PWR_MODE_3_0A;
		curr = 2;
		break;
	}

	usb_role_switch_set_role(tc->role_sw, USB_ROLE_DEVICE);

	typec_set_pwr_role(tc->port, TYPEC_SINK);
	typec_set_data_role(tc->port, TYPEC_DEVICE);
	typec_set_pwr_opmode(tc->port, mode);
	sm5714_typec_register_partner(tc);

	dev_info(tc->dev, "source attached (Rp %s): device mode\n",
		 curr_name[curr]);
}

/* Read the current CC state and move to it. Called with tc->lock held. */
static void sm5714_typec_update(struct sm5714_typec *tc)
{
	unsigned int st1, cc, new;
	int ret;

	ret = regmap_read(tc->regmap, SM5714_TC_REG_STATUS1, &st1);
	if (!ret)
		ret = regmap_read(tc->regmap, SM5714_TC_REG_CC_STATUS, &cc);
	if (ret) {
		dev_err(tc->dev, "failed to read CC state: %d\n", ret);
		return;
	}

	new = (st1 & SM5714_TC_INT1_ATTACH) ?
	      FIELD_GET(SM5714_TC_CC_ATTACH_TYPE, cc) : SM5714_TC_CC_ATTACH_NONE;

	dev_dbg(tc->dev, "STATUS1 0x%02x CC_STATUS 0x%02x (state %u -> %u)\n",
		st1, cc, tc->state, new);

	if (new == tc->state)
		return;

	/* Going straight from one attach type to another: detach first */
	if (tc->state != SM5714_TC_CC_ATTACH_NONE)
		sm5714_typec_detach(tc);

	switch (new) {
	case SM5714_TC_CC_ATTACH_NONE:
		return;
	case SM5714_TC_CC_ATTACH_SRC:
		sm5714_typec_attach_source(tc);
		break;
	case SM5714_TC_CC_ATTACH_SNK:
		sm5714_typec_attach_sink(tc, cc);
		break;
	default:
		/* Audio / debug accessories: not handled yet */
		dev_info(tc->dev, "unsupported attach type %u (CC_STATUS 0x%02x)\n",
			 new, cc);
		break;
	}

	typec_set_orientation(tc->port, (cc & SM5714_TC_CC_FLIP) ?
			      TYPEC_ORIENTATION_REVERSE :
			      TYPEC_ORIENTATION_NORMAL);
	tc->state = new;
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

	mutex_lock(&tc->lock);
	sm5714_typec_update(tc);
	mutex_unlock(&tc->lock);

	return IRQ_HANDLED;
}

static int sm5714_typec_hw_init(struct sm5714_typec *tc)
{
	/* Unmask only ATTACH/DETACH; everything else stays masked */
	u8 mask[5] = { 0xff, 0xff, 0xff, 0xff, 0xff };
	unsigned int val;
	int ret;

	mask[0] &= ~(SM5714_TC_INT1_ATTACH | SM5714_TC_INT1_DETACH);
	ret = regmap_bulk_write(tc->regmap, SM5714_TC_REG_INT_MASK1,
				mask, sizeof(mask));
	if (ret)
		return ret;

	/* Release the bootloader's sink-only setting: dual role */
	ret = regmap_write(tc->regmap, SM5714_TC_REG_CC_CNTL1,
			   SM5714_TC_CC_CNTL1_DRP);
	if (ret)
		return ret;

	/* Unattached: make sure the CC state machine is running */
	ret = regmap_read(tc->regmap, SM5714_TC_REG_STATUS1, &val);
	if (ret)
		return ret;
	if (!(val & SM5714_TC_INT1_ATTACH)) {
		ret = regmap_write(tc->regmap, SM5714_TC_REG_CC_CNTL3,
				   SM5714_TC_CC_CNTL3_OP_EN);
		if (ret)
			return ret;
	}

	/* As downstream: release the PD engine if it waits for a reset ack */
	ret = regmap_read(tc->regmap, SM5714_TC_REG_PD_STATE3, &val);
	if (ret)
		return ret;
	if (val & 0x06)
		ret = regmap_write(tc->regmap, SM5714_TC_REG_PD_CNTL4,
				   SM5714_TC_PD_CNTL4_RESET_DONE);

	return ret;
}

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

static void sm5714_typec_cleanup(void *data)
{
	struct sm5714_typec *tc = data;

	mutex_lock(&tc->lock);
	if (tc->state != SM5714_TC_CC_ATTACH_NONE)
		sm5714_typec_detach(tc);
	mutex_unlock(&tc->lock);
}

static void sm5714_typec_put_role_sw(void *data)
{
	usb_role_switch_put(data);
}

static void sm5714_typec_unregister_port(void *data)
{
	typec_unregister_port(data);
}

static int sm5714_typec_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct typec_capability cap = { };
	struct fwnode_handle *connector;
	struct sm5714_typec *tc;
	int ret;

	tc = devm_kzalloc(dev, sizeof(*tc), GFP_KERNEL);
	if (!tc)
		return -ENOMEM;

	tc->dev = dev;
	tc->state = SM5714_TC_CC_ATTACH_NONE;
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

	connector = device_get_named_child_node(dev, "connector");
	if (!connector)
		return dev_err_probe(dev, -ENODEV, "missing connector node\n");

	tc->role_sw = fwnode_usb_role_switch_get(connector);
	if (IS_ERR(tc->role_sw)) {
		fwnode_handle_put(connector);
		return dev_err_probe(dev, PTR_ERR(tc->role_sw),
				     "failed to get USB role switch\n");
	}
	if (!tc->role_sw) {
		fwnode_handle_put(connector);
		return dev_err_probe(dev, -EPROBE_DEFER,
				     "USB role switch not found yet\n");
	}
	ret = devm_add_action_or_reset(dev, sm5714_typec_put_role_sw, tc->role_sw);
	if (ret) {
		fwnode_handle_put(connector);
		return ret;
	}

	cap.type = TYPEC_PORT_DRP;
	cap.data = TYPEC_PORT_DRD;
	cap.revision = USB_TYPEC_REV_1_2;
	cap.prefer_role = TYPEC_NO_PREFERRED_ROLE;
	cap.fwnode = connector;
	cap.driver_data = tc;

	tc->port = typec_register_port(dev, &cap);
	fwnode_handle_put(connector);
	if (IS_ERR(tc->port))
		return dev_err_probe(dev, PTR_ERR(tc->port),
				     "failed to register Type-C port\n");
	ret = devm_add_action_or_reset(dev, sm5714_typec_unregister_port, tc->port);
	if (ret)
		return ret;

	ret = sm5714_typec_hw_init(tc);
	if (ret)
		return dev_err_probe(dev, ret, "failed to init chip\n");

	/* Runs last at remove: undo host mode / VBUS before the port goes */
	ret = devm_add_action_or_reset(dev, sm5714_typec_cleanup, tc);
	if (ret)
		return ret;

	/* Something may already be plugged in (keyboard at boot) */
	mutex_lock(&tc->lock);
	sm5714_typec_update(tc);
	mutex_unlock(&tc->lock);

	if (!client->irq)
		return dev_err_probe(dev, -EINVAL, "no interrupt\n");

	ret = devm_request_threaded_irq(dev, client->irq, NULL, sm5714_typec_irq,
					IRQF_ONESHOT, "sm5714-typec", tc);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request IRQ\n");

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

MODULE_DESCRIPTION("Silicon Mitus SM5714 USB Type-C port driver (CC only)");
MODULE_LICENSE("GPL");
