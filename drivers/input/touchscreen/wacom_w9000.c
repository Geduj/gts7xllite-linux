// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Wacom W9000-series penabled I2C touchscreen driver
 *
 * Copyright (c) 2026 Hendrik Noack <hendrik-noack@gmx.de>
 *
 * Partially based on vendor driver:
 *	Copyright (C) 2012, Samsung Electronics Co. Ltd.
 */

#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/touchscreen.h>
#include <linux/interrupt.h>
#include <linux/lockdep.h>
#include <linux/regulator/consumer.h>
#include <linux/unaligned.h>

/* Some chips have flaky firmware that requires many retries before responding. */
#define CMD_QUERY_RETRIES	8

/* Message length */
#define CMD_QUERY_NUM_MAX	9
#define MSG_COORD_NUM_MAX	16

/* Commands */
#define CMD_QUERY		0x2a

/* Packet ID in the low nibble of byte 0, on chips that send more than coordinates */
#define PACKET_ID_MASK		GENMASK(3, 0)
#define PACKET_ID_COORD		1

/**
 * struct wacom_w9000_variant - per-chip protocol differences
 * @cmd_query_num: length of the query response
 * @msg_coord_num: length of one report read per interrupt
 * @pressure_mask: valid bits of the pressure field, 0 for all 16 bits
 * @max_tilt: tilt range (+/-) reported in bytes 8 and 9, 0 if not reported
 * @has_packet_id: byte 0 carries a packet type, only type 1 is a coordinate
 * @name: input device name
 */
struct wacom_w9000_variant {
	const u8 cmd_query_num;
	const u8 msg_coord_num;
	const u16 pressure_mask;
	const u8 max_tilt;
	const bool has_packet_id;
	const char *name;
};

struct wacom_w9000_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	const struct wacom_w9000_variant *variant;
	u16 fw_version;

	struct touchscreen_properties prop;
	u16 max_pressure;

	struct regulator *regulator;

	struct gpio_desc *flash_mode_gpio;
	struct gpio_desc *reset_gpio;

	unsigned int irq;

	bool pen_proximity;
};

static int wacom_w9000_read(struct i2c_client *client, u8 command, u8 len, u8 *data)
{
	int error, res;
	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.buf = &command,
			.len = sizeof(command),
		}, {
			.addr = client->addr,
			.flags = I2C_M_RD,
			.buf = data,
			.len = len,
		}
	};

	res = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	if (res != ARRAY_SIZE(msg)) {
		error = res < 0 ? res : -EIO;
		dev_err(&client->dev, "%s: i2c transfer failed: %d (%d)\n",
			__func__, error, res);
		return error;
	}

	return 0;
}

static int wacom_w9000_query(struct wacom_w9000_data *wacom_data)
{
	struct i2c_client *client = wacom_data->client;
	struct device *dev = &wacom_data->client->dev;
	u8 data[CMD_QUERY_NUM_MAX];
	int retry;
	int error;

	for (retry = 0; retry < CMD_QUERY_RETRIES; retry++) {
		error = wacom_w9000_read(client, CMD_QUERY,
					 wacom_data->variant->cmd_query_num,
					 data);
		if (!error && data[0] == 0x0f)
			break;
	}

	if (error)
		return error;

	if (data[0] != 0x0f)
		return -EIO;

	dev_dbg(dev, "query: %*ph, %d\n",
		wacom_data->variant->cmd_query_num, data, retry);

	wacom_data->prop.max_x = get_unaligned_be16(&data[1]);
	wacom_data->prop.max_y = get_unaligned_be16(&data[3]);
	wacom_data->max_pressure = get_unaligned_be16(&data[5]);
	wacom_data->fw_version = get_unaligned_be16(&data[7]);

	dev_dbg(dev, "max_x:%d, max_y:%d, max_pressure:%d, fw:%#x\n",
		wacom_data->prop.max_x, wacom_data->prop.max_y,
		wacom_data->max_pressure, wacom_data->fw_version);

	return 0;
}

static int wacom_w9000_power_on(struct wacom_w9000_data *wacom_data)
{
	int error;

	lockdep_assert_held(&wacom_data->input_dev->mutex);

	error = regulator_enable(wacom_data->regulator);
	if (error) {
		dev_err(&wacom_data->client->dev, "Failed to enable regulators: %d\n", error);
		return error;
	}

	msleep(200);

	gpiod_set_value_cansleep(wacom_data->reset_gpio, 0);
	enable_irq(wacom_data->irq);

	return 0;
}

static int wacom_w9000_power_off(struct wacom_w9000_data *wacom_data)
{
	lockdep_assert_held(&wacom_data->input_dev->mutex);

	disable_irq(wacom_data->irq);
	gpiod_set_value_cansleep(wacom_data->reset_gpio, 1);
	regulator_disable(wacom_data->regulator);

	return 0;
}

static void wacom_w9000_report_tilt(struct wacom_w9000_data *wacom_data, const u8 *data)
{
	const struct touchscreen_properties *prop = &wacom_data->prop;
	int tilt_x = (s8)data[8];
	int tilt_y = (s8)data[9];

	/* Same transform touchscreen_report_pos() applies to x/y: invert, then swap */
	if (prop->invert_x)
		tilt_x = -tilt_x;
	if (prop->invert_y)
		tilt_y = -tilt_y;
	if (prop->swap_x_y)
		swap(tilt_x, tilt_y);

	input_report_abs(wacom_data->input_dev, ABS_TILT_X, tilt_x);
	input_report_abs(wacom_data->input_dev, ABS_TILT_Y, tilt_y);
}

static void wacom_w9000_coord(struct wacom_w9000_data *wacom_data)
{
	struct i2c_client *client = wacom_data->client;
	struct device *dev = &wacom_data->client->dev;
	const struct wacom_w9000_variant *variant = wacom_data->variant;
	u8 data[MSG_COORD_NUM_MAX];
	bool touch, rubber, side_button;
	u16 x, y, pressure;
	u8 distance = 0;
	int error;

	error = i2c_master_recv(client, data, wacom_data->variant->msg_coord_num);
	if (error != wacom_data->variant->msg_coord_num) {
		if (error >= 0)
			error = -EIO;
		dev_err_ratelimited(dev, "%s: i2c receive failed (%d)\n", __func__, error);
		return;
	}

	dev_dbg(dev, "data: %*ph\n", wacom_data->variant->msg_coord_num, data);

	/*
	 * Newer chips also send notifications (e.g. a request to pause touch
	 * scanning while the pen is near). They have bit 7 clear and must not
	 * be taken as the pen leaving proximity.
	 */
	if (variant->has_packet_id &&
	    FIELD_GET(PACKET_ID_MASK, data[0]) != PACKET_ID_COORD)
		return;

	if (data[0] & BIT(7)) {
		wacom_data->pen_proximity = true;

		touch = !!(data[0] & BIT(4));
		side_button = !!(data[0] & BIT(5));
		rubber = !!(data[0] & BIT(6));

		x = get_unaligned_be16(&data[1]);
		y = get_unaligned_be16(&data[3]);
		pressure = get_unaligned_be16(&data[5]);
		if (variant->pressure_mask)
			pressure &= variant->pressure_mask;

		if (wacom_data->variant->msg_coord_num > 7)
			distance = data[7];

		if (x > wacom_data->prop.max_x || y > wacom_data->prop.max_y) {
			dev_warn_ratelimited(dev, "Coordinates out of range x=%d, y=%d\n", x, y);
			return;
		}

		if (pressure > wacom_data->max_pressure) {
			dev_warn_ratelimited(dev, "Pressure out of range %d\n", pressure);
			return;
		}

		touchscreen_report_pos(wacom_data->input_dev, &wacom_data->prop, x, y, false);
		input_report_abs(wacom_data->input_dev, ABS_PRESSURE, pressure);

		if (wacom_data->variant->msg_coord_num > 7)
			input_report_abs(wacom_data->input_dev, ABS_DISTANCE, distance);

		if (variant->max_tilt)
			wacom_w9000_report_tilt(wacom_data, data);

		input_report_key(wacom_data->input_dev, BTN_STYLUS, side_button);
		input_report_key(wacom_data->input_dev, BTN_TOUCH, touch);
		input_report_key(wacom_data->input_dev, BTN_TOOL_PEN, !rubber);
		input_report_key(wacom_data->input_dev, BTN_TOOL_RUBBER, rubber);
		input_sync(wacom_data->input_dev);
	} else if (wacom_data->pen_proximity) {
		input_report_abs(wacom_data->input_dev, ABS_PRESSURE, 0);

		if (wacom_data->variant->msg_coord_num > 7)
			input_report_abs(wacom_data->input_dev, ABS_DISTANCE, 255);

		if (variant->max_tilt) {
			input_report_abs(wacom_data->input_dev, ABS_TILT_X, 0);
			input_report_abs(wacom_data->input_dev, ABS_TILT_Y, 0);
		}

		input_report_key(wacom_data->input_dev, BTN_STYLUS, 0);
		input_report_key(wacom_data->input_dev, BTN_TOUCH, 0);
		input_report_key(wacom_data->input_dev, BTN_TOOL_PEN, 0);
		input_report_key(wacom_data->input_dev, BTN_TOOL_RUBBER, 0);
		input_sync(wacom_data->input_dev);

		wacom_data->pen_proximity = false;
	}
}

static irqreturn_t wacom_w9000_interrupt(int irq, void *dev_id)
{
	struct wacom_w9000_data *wacom_data = dev_id;

	wacom_w9000_coord(wacom_data);

	return IRQ_HANDLED;
}

static int wacom_w9000_open(struct input_dev *dev)
{
	struct wacom_w9000_data *wacom_data = input_get_drvdata(dev);

	return wacom_w9000_power_on(wacom_data);
}

static void wacom_w9000_close(struct input_dev *dev)
{
	struct wacom_w9000_data *wacom_data = input_get_drvdata(dev);

	wacom_w9000_power_off(wacom_data);
}

static int wacom_w9000_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct wacom_w9000_data *wacom_data;
	struct input_dev *input_dev;
	int error;
	u32 val;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(dev, "i2c_check_functionality error\n");
		return -EIO;
	}

	wacom_data = devm_kzalloc(dev, sizeof(*wacom_data), GFP_KERNEL);
	if (!wacom_data)
		return -ENOMEM;

	wacom_data->variant = i2c_get_match_data(client);
	if (!wacom_data->variant) {
		dev_err(dev, "No i2c match_data available\n");
		return -EINVAL;
	}

	if (wacom_data->variant->cmd_query_num > CMD_QUERY_NUM_MAX ||
	    wacom_data->variant->msg_coord_num > MSG_COORD_NUM_MAX) {
		dev_err(dev, "Length of message for %s exceeds the maximum\n",
			wacom_data->variant->name);
		return -EINVAL;
	}

	if (wacom_data->variant->msg_coord_num < 7 ||
	    (wacom_data->variant->max_tilt && wacom_data->variant->msg_coord_num < 10)) {
		dev_err(dev, "Length of coordinates message for %s too short\n",
			wacom_data->variant->name);
		return -EINVAL;
	}

	wacom_data->client = client;
	wacom_data->irq = client->irq;
	i2c_set_clientdata(client, wacom_data);

	wacom_data->regulator = devm_regulator_get(dev, "vdd");
	if (IS_ERR(wacom_data->regulator))
		return dev_err_probe(dev, PTR_ERR(wacom_data->regulator),
				     "Failed to get regulators\n");

	wacom_data->flash_mode_gpio = devm_gpiod_get_optional(dev, "flash-mode", GPIOD_OUT_LOW);
	if (IS_ERR(wacom_data->flash_mode_gpio))
		return dev_err_probe(dev, PTR_ERR(wacom_data->flash_mode_gpio),
				     "Failed to get flash-mode gpio\n");

	wacom_data->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(wacom_data->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(wacom_data->reset_gpio),
				     "Failed to get reset gpio\n");

	error = regulator_enable(wacom_data->regulator);
	if (error)
		return dev_err_probe(dev, error, "Failed to enable regulators\n");

	msleep(200);

	gpiod_set_value_cansleep(wacom_data->reset_gpio, 0);

	error = wacom_w9000_query(wacom_data);

	gpiod_set_value_cansleep(wacom_data->reset_gpio, 1);
	regulator_disable(wacom_data->regulator);

	if (error)
		return dev_err_probe(dev, error, "Failed to query\n");

	error = devm_request_threaded_irq(dev, wacom_data->irq, NULL, wacom_w9000_interrupt,
					  IRQF_ONESHOT | IRQF_NO_AUTOEN, client->name, wacom_data);
	if (error)
		return dev_err_probe(dev, error, "Failed to register interrupt\n");

	input_dev = devm_input_allocate_device(dev);
	if (!input_dev)
		return -ENOMEM;

	wacom_data->input_dev = input_dev;
	input_set_drvdata(input_dev, wacom_data);

	input_dev->name = wacom_data->variant->name;
	input_dev->id.bustype = BUS_I2C;
	input_dev->id.vendor = 0x56a;
	input_dev->id.version = wacom_data->fw_version;
	input_dev->open = wacom_w9000_open;
	input_dev->close = wacom_w9000_close;

	input_set_capability(input_dev, EV_KEY, BTN_TOUCH);
	input_set_capability(input_dev, EV_KEY, BTN_TOOL_PEN);
	input_set_capability(input_dev, EV_KEY, BTN_TOOL_RUBBER);
	input_set_capability(input_dev, EV_KEY, BTN_STYLUS);

	input_set_abs_params(input_dev, ABS_X, 0, wacom_data->prop.max_x, 4, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, wacom_data->prop.max_y, 4, 0);
	if (wacom_data->variant->pressure_mask)
		wacom_data->max_pressure &= wacom_data->variant->pressure_mask;
	input_set_abs_params(input_dev, ABS_PRESSURE, 0, wacom_data->max_pressure, 0, 0);

	if (wacom_data->variant->msg_coord_num > 7)
		input_set_abs_params(input_dev, ABS_DISTANCE, 0, 255, 0, 0);

	if (wacom_data->variant->max_tilt) {
		input_set_abs_params(input_dev, ABS_TILT_X, -wacom_data->variant->max_tilt,
				     wacom_data->variant->max_tilt, 0, 0);
		input_set_abs_params(input_dev, ABS_TILT_Y, -wacom_data->variant->max_tilt,
				     wacom_data->variant->max_tilt, 0, 0);
	}

	touchscreen_parse_properties(input_dev, false, &wacom_data->prop);

	dev_info(dev, "%s size X%uY%u\n", wacom_data->variant->name,
		 wacom_data->prop.max_x, wacom_data->prop.max_y);

	error = device_property_read_u32(dev, "touchscreen-x-mm", &val);
	if (!error && val)
		input_abs_set_res(input_dev, wacom_data->prop.swap_x_y ? ABS_Y : ABS_X,
				  wacom_data->prop.max_x / val);
	error = device_property_read_u32(dev, "touchscreen-y-mm", &val);
	if (!error && val)
		input_abs_set_res(input_dev, wacom_data->prop.swap_x_y ? ABS_X : ABS_Y,
				  wacom_data->prop.max_y / val);

	error = input_register_device(wacom_data->input_dev);
	if (error)
		return dev_err_probe(dev, error, "Failed to register input device\n");

	return 0;
}

static int wacom_w9000_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct wacom_w9000_data *wacom_data = i2c_get_clientdata(client);

	guard(mutex)(&wacom_data->input_dev->mutex);

	if (input_device_enabled(wacom_data->input_dev))
		return wacom_w9000_power_off(wacom_data);

	return 0;
}

static int wacom_w9000_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct wacom_w9000_data *wacom_data = i2c_get_clientdata(client);

	guard(mutex)(&wacom_data->input_dev->mutex);

	if (input_device_enabled(wacom_data->input_dev))
		return wacom_w9000_power_on(wacom_data);

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(wacom_w9000_pm, wacom_w9000_suspend, wacom_w9000_resume);

static const struct wacom_w9000_variant w9002 = {
	.cmd_query_num  = 9,
	.msg_coord_num  = 7,
	.name = "Wacom W9002 Digitizer",
};

static const struct wacom_w9000_variant w9007a_lt03 = {
	.cmd_query_num	= 9,
	.msg_coord_num	= 8,
	.name = "Wacom W9007A LT03 Digitizer",
};

static const struct wacom_w9000_variant w9007a_v1 = {
	.cmd_query_num	= 9,
	.msg_coord_num	= 12,
	.name = "Wacom W9007A V1 Digitizer",
};

/*
 * W9021: 16-byte reports with a packet ID in byte 0, a flag in bit 15 of the
 * pressure field and tilt in bytes 8/9 (+/-63).
 */
static const struct wacom_w9000_variant w9021 = {
	.cmd_query_num	= 9,
	.msg_coord_num	= 16,
	.pressure_mask	= GENMASK(11, 0),
	.max_tilt	= 63,
	.has_packet_id	= true,
	.name = "Wacom W9021 Digitizer",
};

static const struct of_device_id wacom_w9000_of_match[] = {
	{ .compatible = "wacom,w9002", .data = &w9002 },
	{ .compatible = "wacom,w9007a-lt03", .data = &w9007a_lt03, },
	{ .compatible = "wacom,w9007a-v1", .data = &w9007a_v1, },
	{ .compatible = "wacom,w9021", .data = &w9021, },
	{ }
};
MODULE_DEVICE_TABLE(of, wacom_w9000_of_match);

static const struct i2c_device_id wacom_w9000_id[] = {
	{ .name = "w9002", .driver_data = (kernel_ulong_t)&w9002 },
	{ .name = "w9007a-lt03", .driver_data = (kernel_ulong_t)&w9007a_lt03 },
	{ .name = "w9007a-v1", .driver_data = (kernel_ulong_t)&w9007a_v1 },
	{ .name = "w9021", .driver_data = (kernel_ulong_t)&w9021 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, wacom_w9000_id);

static struct i2c_driver wacom_w9000_driver = {
	.driver = {
		.name	= "wacom_w9000",
		.of_match_table = wacom_w9000_of_match,
		.pm	= pm_sleep_ptr(&wacom_w9000_pm),
	},
	.probe		= wacom_w9000_probe,
	.id_table	= wacom_w9000_id,
};
module_i2c_driver(wacom_w9000_driver);

MODULE_AUTHOR("Hendrik Noack <hendrik-noack@gmx.de>");
MODULE_DESCRIPTION("Wacom W9000-series penabled touchscreen driver");
MODULE_LICENSE("GPL");
