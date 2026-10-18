// SPDX-License-Identifier: GPL-2.0-only
/*
 * FocalTech FT8203 (TDDI) SPI touchscreen driver
 *
 * The FT8203 has no flash for its touch firmware: after every reset the host
 * enters the ROM bootloader, writes the firmware image into PRAM, verifies it
 * with an ECC computed by the chip and starts it. Protocol derived from the
 * FocalTech/Samsung downstream driver (FTS V3.3.2, "SPI V2" framing).
 *
 * SPI frame:
 *   write: [cmd][0x00][len hi][len lo]([3 dummy][data...])
 *   read:  [cmd][0xa0][len hi][len lo][3 dummy][data...][crc16 le]
 * rx byte 3 is a status byte (bits 0xa0 set = error), read data is protected
 * by CRC-16 (poly 0x8408 reflected, init 0xffff = crc_ccitt()).
 */

#include <linux/crc-ccitt.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/gpio/consumer.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/interrupt.h>
#include <linux/minmax.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pm.h>
#include <linux/property.h>
#include <linux/sizes.h>
#include <linux/spi/spi.h>
#include <linux/unaligned.h>
#include <linux/workqueue.h>

#include <drm/drm_panel.h>

/* SPI framing */
#define FT8203_CTRL_WRITE		0x00
#define FT8203_CTRL_READ		0xa0	/* read | data CRC enable */
#define FT8203_HDR_LEN			4
#define FT8203_DUMMY_LEN		3
#define FT8203_DATA_OFF			(FT8203_HDR_LEN + FT8203_DUMMY_LEN)
#define FT8203_CRC_LEN			2
#define FT8203_STATUS_ERR		0xa0
#define FT8203_XFER_RETRIES		3
#define FT8203_CS_HIGH_US		150

/* application registers */
#define FT8203_REG_TOUCH		0x01
#define FT8203_REG_CHIP_ID		0xa3
#define FT8203_REG_POWER_MODE		0xa5
#define FT8203_POWER_MODE_SLEEP		0x03
#define FT8203_REG_SEC_IC_NAME		0xc3	/* c3..c6: ic, project, module, fw */

/* ROM bootloader commands */
#define FT8203_CMD_START1		0x55
#define FT8203_CMD_READ_ID		0x90
#define FT8203_CMD_SET_PRAM_ADDR	0xad
#define FT8203_CMD_WRITE_PRAM		0xae
#define FT8203_CMD_ECC_CALC		0xcc
#define FT8203_CMD_ECC_READ		0xcd
#define FT8203_CMD_ECC_FINISH		0xce
#define FT8203_CMD_START_APP		0x08

#define FT8203_ROM_ID			0x8201
#define FT8203_APP_ID_HIGH		0x82
#define FT8203_ECC_OK			0xa5
#define FT8203_ECC_POLY			0x8408

/* firmware image layout */
#define FT8203_FW_APPINFO_OFF		0x100
#define FT8203_FW_VER_OFF		0x1a0
#define FT8203_FW_MIN_LEN		0x1a4
#define FT8203_FW_APP_MAX		(96 * SZ_1K)
#define FT8203_PRAM_PKT			SZ_4K
#define FT8203_UPLOAD_RETRIES		3

/* touch report */
#define FT8203_MAX_POINTS		10
#define FT8203_POINT_LEN		6
#define FT8203_TOUCH_LEN		90	/* same read length as downstream */
#define FT8203_EVENT_UP			1

#define FT8203_BUF_LEN	(FT8203_DATA_OFF + FT8203_PRAM_PKT + FT8203_CRC_LEN)

struct ft8203 {
	struct spi_device *spi;
	struct input_dev *input;
	struct gpio_desc *reset_gpio;
	struct touchscreen_properties prop;

	const char *fw_name;
	const struct firmware *fw;
	u32 app_size;
	bool ver_logged;

	struct mutex lock;		/* bus access and power state */
	bool powered;

	struct drm_panel_follower follower;
	struct work_struct resume_work;
	bool is_follower;

	u8 *tx;
	u8 *rx;
	u8 point_buf[FT8203_TOUCH_LEN];
};

/* ------------------------------------------------------------------------ */
/* bus layer                                                                */

static int ft8203_xfer(struct ft8203 *ts, size_t len)
{
	struct spi_transfer xfer = {
		.tx_buf = ts->tx,
		.rx_buf = ts->rx,
		.len = len,
	};
	int ret;

	ret = spi_sync_transfer(ts->spi, &xfer, 1);
	usleep_range(FT8203_CS_HIGH_US, FT8203_CS_HIGH_US + 50);
	return ret;
}

static int ft8203_write(struct ft8203 *ts, u8 cmd, const u8 *data, size_t len)
{
	size_t n = FT8203_HDR_LEN;
	int ret = -EIO;
	int i;

	if (len > FT8203_PRAM_PKT)
		return -EINVAL;

	ts->tx[0] = cmd;
	ts->tx[1] = FT8203_CTRL_WRITE;
	put_unaligned_be16(len, &ts->tx[2]);
	if (len) {
		memset(&ts->tx[FT8203_HDR_LEN], 0, FT8203_DUMMY_LEN);
		memcpy(&ts->tx[FT8203_DATA_OFF], data, len);
		n = FT8203_DATA_OFF + len;
	}

	for (i = 0; i < FT8203_XFER_RETRIES; i++) {
		ret = ft8203_xfer(ts, n);
		if (ret)
			continue;
		if (!(ts->rx[3] & FT8203_STATUS_ERR))
			return 0;
		ret = -EIO;
	}

	return ret;
}

static int ft8203_write_reg(struct ft8203 *ts, u8 reg, u8 val)
{
	return ft8203_write(ts, reg, &val, 1);
}

static int ft8203_read(struct ft8203 *ts, u8 cmd, u8 *buf, size_t len)
{
	size_t n = FT8203_DATA_OFF + len + FT8203_CRC_LEN;
	int ret = -EIO;
	int i;

	if (!len || n > FT8203_BUF_LEN)
		return -EINVAL;

	memset(ts->tx, 0, n);
	ts->tx[0] = cmd;
	ts->tx[1] = FT8203_CTRL_READ;
	put_unaligned_be16(len, &ts->tx[2]);

	for (i = 0; i < FT8203_XFER_RETRIES; i++) {
		const u8 *data = &ts->rx[FT8203_DATA_OFF];

		ret = ft8203_xfer(ts, n);
		if (ret)
			continue;

		if (ts->rx[3] & FT8203_STATUS_ERR) {
			ret = -EIO;
			continue;
		}

		if (crc_ccitt(0xffff, data, len) != get_unaligned_le16(data + len)) {
			ret = -EBADMSG;
			continue;
		}

		memcpy(buf, data, len);
		return 0;
	}

	return ret;
}

/* ------------------------------------------------------------------------ */
/* firmware upload                                                          */

static void ft8203_reset(struct ft8203 *ts)
{
	gpiod_set_value_cansleep(ts->reset_gpio, 1);
	usleep_range(1000, 2000);
	gpiod_set_value_cansleep(ts->reset_gpio, 0);
}

static u16 ft8203_ecc(const u8 *data, size_t len)
{
	u16 ecc = 0;
	size_t i;
	int j;

	for (i = 0; i + 1 < len; i += 2) {
		ecc ^= (data[i] << 8) | data[i + 1];
		for (j = 0; j < 16; j++)
			ecc = (ecc & 1) ? (ecc >> 1) ^ FT8203_ECC_POLY : ecc >> 1;
	}

	return ecc;
}

static int ft8203_load_firmware(struct ft8203 *ts)
{
	struct device *dev = &ts->spi->dev;
	const u8 *d;
	u16 len, len_n;
	int ret;

	if (ts->fw)
		return 0;

	ret = request_firmware(&ts->fw, ts->fw_name, dev);
	if (ret) {
		dev_err(dev, "failed to load firmware %s: %d\n", ts->fw_name, ret);
		return ret;
	}

	d = ts->fw->data;
	if (ts->fw->size < FT8203_FW_MIN_LEN)
		goto invalid;

	len = get_unaligned_be16(d + FT8203_FW_APPINFO_OFF);
	len_n = get_unaligned_be16(d + FT8203_FW_APPINFO_OFF + 2);
	if (len + len_n != 0xffff)
		goto invalid;

	ts->app_size = (u32)len * 2;
	if (!ts->app_size || ts->app_size > ts->fw->size ||
	    ts->app_size > FT8203_FW_APP_MAX)
		goto invalid;

	dev_info(dev, "firmware %s: %u bytes, version FT%*phN\n",
		 ts->fw_name, ts->app_size, 4, d + FT8203_FW_VER_OFF);
	return 0;

invalid:
	dev_err(dev, "invalid firmware image %s\n", ts->fw_name);
	release_firmware(ts->fw);
	ts->fw = NULL;
	return -EINVAL;
}

static int ft8203_enter_boot(struct ft8203 *ts)
{
	u8 id[2];
	int i, j;

	for (i = 0; i < 30; i++) {
		ft8203_reset(ts);
		usleep_range(8000, 9000);

		for (j = 0; j < 3; j++) {
			if (ft8203_write(ts, FT8203_CMD_START1, NULL, 0))
				continue;
			usleep_range(8000, 9000);
			if (ft8203_read(ts, FT8203_CMD_READ_ID, id, sizeof(id)))
				continue;
			if (get_unaligned_be16(id) == FT8203_ROM_ID)
				return 0;
		}
	}

	return -ENODEV;
}

static int ft8203_wait_app(struct ft8203 *ts)
{
	u8 id;
	int i;

	msleep(200);
	for (i = 0; i < 5; i++) {
		if (!ft8203_read(ts, FT8203_REG_CHIP_ID, &id, 1) &&
		    id == FT8203_APP_ID_HIGH)
			return 0;
		msleep(50);
	}

	return -ETIMEDOUT;
}

static void ft8203_log_version(struct ft8203 *ts)
{
	u8 ver[4];
	int i;

	if (ts->ver_logged)
		return;

	for (i = 0; i < 4; i++)
		if (ft8203_read(ts, FT8203_REG_SEC_IC_NAME + i, &ver[i], 1))
			return;

	dev_info(&ts->spi->dev, "touch firmware running, version FT%*phN\n",
		 4, ver);
	ts->ver_logged = true;
}

static int ft8203_upload_once(struct ft8203 *ts)
{
	struct device *dev = &ts->spi->dev;
	const u8 *img = ts->fw->data;
	u32 size = ts->app_size;
	u32 off, chunk;
	u8 buf[6], val;
	u16 ecc;
	int ret, i;

	ret = ft8203_enter_boot(ts);
	if (ret) {
		dev_err(dev, "failed to enter ROM bootloader\n");
		return ret;
	}

	for (off = 0; off < size; off += chunk) {
		chunk = min_t(u32, FT8203_PRAM_PKT, size - off);

		put_unaligned_be24(off, buf);
		ret = ft8203_write(ts, FT8203_CMD_SET_PRAM_ADDR, buf, 3);
		if (ret)
			return ret;

		ret = ft8203_write(ts, FT8203_CMD_WRITE_PRAM, img + off, chunk);
		if (ret)
			return ret;
	}

	put_unaligned_be24(0, buf);
	put_unaligned_be24(size, buf + 3);
	ret = ft8203_write(ts, FT8203_CMD_ECC_CALC, buf, 6);
	if (ret)
		return ret;

	usleep_range(2000, 3000);
	for (i = 0; i < 100; i++) {
		ret = ft8203_read(ts, FT8203_CMD_ECC_FINISH, &val, 1);
		if (ret)
			return ret;
		if (val == FT8203_ECC_OK)
			break;
		usleep_range(1000, 1500);
	}
	if (i == 100) {
		dev_err(dev, "ECC calculation timeout\n");
		return -ETIMEDOUT;
	}

	ret = ft8203_read(ts, FT8203_CMD_ECC_READ, buf, 2);
	if (ret)
		return ret;

	ecc = ft8203_ecc(img, size);
	if (get_unaligned_be16(buf) != ecc) {
		dev_err(dev, "ECC mismatch: chip %04x, host %04x\n",
			get_unaligned_be16(buf), ecc);
		return -EIO;
	}

	ret = ft8203_write(ts, FT8203_CMD_START_APP, NULL, 0);
	if (ret)
		return ret;

	ret = ft8203_wait_app(ts);
	if (ret) {
		dev_err(dev, "touch firmware did not start\n");
		return ret;
	}

	ft8203_log_version(ts);
	return 0;
}

static int ft8203_upload(struct ft8203 *ts)
{
	int ret = -EIO;
	int i;

	for (i = 0; i < FT8203_UPLOAD_RETRIES; i++) {
		ret = ft8203_upload_once(ts);
		if (!ret)
			return 0;
		dev_warn(&ts->spi->dev, "firmware upload attempt %d failed: %d\n",
			 i + 1, ret);
	}

	return ret;
}

/* ------------------------------------------------------------------------ */
/* input reporting                                                          */

static void ft8203_release_contacts(struct ft8203 *ts)
{
	int i;

	for (i = 0; i < FT8203_MAX_POINTS; i++) {
		input_mt_slot(ts->input, i);
		input_mt_report_slot_inactive(ts->input);
	}
	input_mt_sync_frame(ts->input);
	input_sync(ts->input);
}

static irqreturn_t ft8203_irq(int irq, void *data)
{
	struct ft8203 *ts = data;
	struct device *dev = &ts->spi->dev;
	u8 *d = ts->point_buf;
	int i, ret;

	mutex_lock(&ts->lock);

	if (!ts->powered)
		goto out;

	ret = ft8203_read(ts, FT8203_REG_TOUCH, d, FT8203_TOUCH_LEN);
	if (ret) {
		dev_err_ratelimited(dev, "touch read failed: %d\n", ret);
		goto out;
	}

	/* firmware crashed / lost (e.g. panel reset): upload again */
	if (d[1] == 0xef && d[2] == 0xef && d[3] == 0xef) {
		dev_warn(dev, "touch firmware stopped, reloading\n");
		ft8203_release_contacts(ts);
		if (ft8203_upload(ts))
			dev_err(dev, "touch firmware reload failed\n");
		goto out;
	}

	/* 0x0f point count marks an invalid frame */
	if ((d[1] & 0x0f) > FT8203_MAX_POINTS)
		goto out;

	for (i = 0; i < FT8203_MAX_POINTS; i++) {
		const u8 *p = &d[2 + i * FT8203_POINT_LEN];
		unsigned int id = p[2] >> 4;
		bool down;

		if (id >= FT8203_MAX_POINTS)
			break;

		down = (p[0] >> 6) != FT8203_EVENT_UP;

		input_mt_slot(ts->input, id);
		input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, down);
		if (!down)
			continue;

		touchscreen_report_pos(ts->input, &ts->prop,
				       ((p[0] & 0x0f) << 8) | p[1],
				       ((p[2] & 0x0f) << 8) | p[3], true);
		input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, p[5]);
		input_report_abs(ts->input, ABS_MT_TOUCH_MINOR, p[4]);
	}

	input_mt_sync_frame(ts->input);
	input_sync(ts->input);

out:
	mutex_unlock(&ts->lock);
	return IRQ_HANDLED;
}

/* ------------------------------------------------------------------------ */
/* power handling                                                           */

static int ft8203_power_on(struct ft8203 *ts)
{
	int ret;

	mutex_lock(&ts->lock);

	if (ts->powered) {
		mutex_unlock(&ts->lock);
		return 0;
	}

	ret = ft8203_load_firmware(ts);
	if (!ret)
		ret = ft8203_upload(ts);
	if (!ret)
		ts->powered = true;

	mutex_unlock(&ts->lock);

	if (ret) {
		dev_err(&ts->spi->dev, "failed to start touch firmware: %d\n", ret);
		return ret;
	}

	enable_irq(ts->spi->irq);
	return 0;
}

static void ft8203_power_off(struct ft8203 *ts)
{
	mutex_lock(&ts->lock);
	if (!ts->powered) {
		mutex_unlock(&ts->lock);
		return;
	}
	ts->powered = false;
	mutex_unlock(&ts->lock);

	/* waits for a running IRQ thread, which bails out on !powered */
	disable_irq(ts->spi->irq);

	mutex_lock(&ts->lock);
	ft8203_write_reg(ts, FT8203_REG_POWER_MODE, FT8203_POWER_MODE_SLEEP);
	gpiod_set_value_cansleep(ts->reset_gpio, 1);
	ft8203_release_contacts(ts);
	mutex_unlock(&ts->lock);
}

static void ft8203_resume_work(struct work_struct *work)
{
	struct ft8203 *ts = container_of(work, struct ft8203, resume_work);

	ft8203_power_on(ts);
}

/*
 * TDDI: the touch part lives in the display driver IC, so touch can only run
 * while the panel is powered, and a panel reset kills the touch firmware.
 * Follow the panel: upload after it is prepared, sleep before it goes down.
 */
static int ft8203_panel_prepared(struct drm_panel_follower *follower)
{
	struct ft8203 *ts = container_of(follower, struct ft8203, follower);

	/* firmware upload takes ~0.4 s, don't block the display resume */
	schedule_work(&ts->resume_work);
	return 0;
}

static int ft8203_panel_unpreparing(struct drm_panel_follower *follower)
{
	struct ft8203 *ts = container_of(follower, struct ft8203, follower);

	cancel_work_sync(&ts->resume_work);
	ft8203_power_off(ts);
	return 0;
}

static const struct drm_panel_follower_funcs ft8203_follower_funcs = {
	.panel_prepared = ft8203_panel_prepared,
	.panel_unpreparing = ft8203_panel_unpreparing,
};

static int ft8203_suspend(struct device *dev)
{
	struct ft8203 *ts = dev_get_drvdata(dev);

	if (!ts->is_follower)
		ft8203_power_off(ts);
	return 0;
}

static int ft8203_resume(struct device *dev)
{
	struct ft8203 *ts = dev_get_drvdata(dev);

	if (ts->is_follower)
		return 0;
	return ft8203_power_on(ts);
}

static DEFINE_SIMPLE_DEV_PM_OPS(ft8203_pm_ops, ft8203_suspend, ft8203_resume);

/* ------------------------------------------------------------------------ */
/* probe                                                                    */

static void ft8203_release_fw(void *data)
{
	struct ft8203 *ts = data;

	release_firmware(ts->fw);
	ts->fw = NULL;
}

static void ft8203_stop(void *data)
{
	struct ft8203 *ts = data;

	cancel_work_sync(&ts->resume_work);
	ft8203_power_off(ts);
}

static void ft8203_remove_follower(void *data)
{
	struct ft8203 *ts = data;

	drm_panel_remove_follower(&ts->follower);
}

static int ft8203_input_init(struct ft8203 *ts)
{
	struct device *dev = &ts->spi->dev;
	struct input_dev *input;
	int ret;

	input = devm_input_allocate_device(dev);
	if (!input)
		return -ENOMEM;

	input->name = "FocalTech FT8203 Touchscreen";
	input->id.bustype = BUS_SPI;

	input_set_capability(input, EV_ABS, ABS_MT_POSITION_X);
	input_set_capability(input, EV_ABS, ABS_MT_POSITION_Y);
	input_set_abs_params(input, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(input, ABS_MT_TOUCH_MINOR, 0, 255, 0, 0);

	touchscreen_parse_properties(input, true, &ts->prop);
	if (!ts->prop.max_x || !ts->prop.max_y)
		return dev_err_probe(dev, -EINVAL,
				     "touchscreen-size-x/y are required\n");

	ret = input_mt_init_slots(input, FT8203_MAX_POINTS,
				  INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED);
	if (ret)
		return ret;

	ts->input = input;
	return input_register_device(input);
}

static int ft8203_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct ft8203 *ts;
	int ret;

	if (!spi->irq)
		return dev_err_probe(dev, -EINVAL, "no interrupt\n");

	spi->mode = SPI_MODE_0;
	spi->bits_per_word = 8;
	ret = spi_setup(spi);
	if (ret)
		return dev_err_probe(dev, ret, "SPI setup failed\n");

	ts = devm_kzalloc(dev, sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	ts->spi = spi;
	spi_set_drvdata(spi, ts);
	INIT_WORK(&ts->resume_work, ft8203_resume_work);

	ret = devm_mutex_init(dev, &ts->lock);
	if (ret)
		return ret;

	ts->tx = devm_kzalloc(dev, FT8203_BUF_LEN, GFP_KERNEL);
	ts->rx = devm_kzalloc(dev, FT8203_BUF_LEN, GFP_KERNEL);
	if (!ts->tx || !ts->rx)
		return -ENOMEM;

	ret = devm_add_action_or_reset(dev, ft8203_release_fw, ts);
	if (ret)
		return ret;

	/* deasserted = chip running; the upload resets it anyway */
	ts->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ts->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ts->reset_gpio),
				     "failed to get reset GPIO\n");

	ts->fw_name = "focaltech/ft8203.bin";
	device_property_read_string(dev, "firmware-name", &ts->fw_name);

	ret = ft8203_input_init(ts);
	if (ret)
		return ret;

	ret = devm_request_threaded_irq(dev, spi->irq, NULL, ft8203_irq,
					IRQF_ONESHOT | IRQF_NO_AUTOEN,
					"ft8203", ts);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request IRQ\n");

	ret = devm_add_action_or_reset(dev, ft8203_stop, ts);
	if (ret)
		return ret;

	if (drm_is_panel_follower(dev)) {
		ts->is_follower = true;
		ts->follower.funcs = &ft8203_follower_funcs;

		/* calls panel_prepared right away if the panel is already up */
		ret = drm_panel_add_follower(dev, &ts->follower);
		if (ret)
			return dev_err_probe(dev, ret, "failed to follow panel\n");

		return devm_add_action_or_reset(dev, ft8203_remove_follower, ts);
	}

	return ft8203_power_on(ts);
}

static const struct of_device_id ft8203_of_match[] = {
	{ .compatible = "focaltech,ft8203" },
	{ }
};
MODULE_DEVICE_TABLE(of, ft8203_of_match);

static const struct spi_device_id ft8203_spi_id[] = {
	{ "ft8203" },
	{ }
};
MODULE_DEVICE_TABLE(spi, ft8203_spi_id);

static struct spi_driver ft8203_driver = {
	.driver = {
		.name = "focaltech-ft8203",
		.of_match_table = ft8203_of_match,
		.pm = pm_sleep_ptr(&ft8203_pm_ops),
	},
	.id_table = ft8203_spi_id,
	.probe = ft8203_probe,
};
module_spi_driver(ft8203_driver);

MODULE_DESCRIPTION("FocalTech FT8203 SPI touchscreen driver");
MODULE_LICENSE("GPL");
