// SPDX-License-Identifier: GPL-2.0-only
/*
 * BOE TS124QDM 12.4" WQXGA LCD with FocalTech FT8203 TDDI controller,
 * as used in the Samsung Galaxy Tab S7 FE (gts7xllite, SM-T735).
 *
 * Timings, DSC parameters, power sequence and init sequence taken from
 * Samsung's downstream DT node ss_dsi_panel_FT8203_TS124QDM_WQXGA
 * (gts7xllite_eur_open_w00_r05 overlay).
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>
#include <linux/i2c.h>
#include <linux/of.h>

#include <drm/display/drm_dsc.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>
#include <drm/drm_probe_helper.h>

#include <video/mipi_display.h>

/*
 * DCS 0x51 takes two bytes. Downstream does not state the bit depth;
 * 12-bit is the usual for this controller family. Adjust after checking
 * Samsung's brightness code if the range turns out wrong.
 */
#define FT8203_MAX_BRIGHTNESS 	3231
#define FT8203_DEF_BRIGHTNESS	2048

static const struct regulator_bulk_data ft8203_supplies[] = {
	{ .supply = "vddi" },	/* panel_ldo_en: PM6350 L14A, 1.9 V */
	{ .supply = "vsp" },	/* panel_buck_en:  TLMM 88 -> ISL98608 */
	{ .supply = "vsn" },	/* panel_buck_en2: TLMM 89 -> ISL98608 */
	{ .supply = "boost" },	/* panel_boost_en: TLMM 72 -> MAX77816 */
};

struct ft8203 {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct drm_dsc_config dsc;
	struct regulator_bulk_data *supplies;
	struct gpio_desc *reset_gpio;
	struct i2c_client *boost;	/* MAX77816 on I2C8, optional */
};

static inline struct ft8203 *to_ft8203(struct drm_panel *panel)
{
	return container_of(panel, struct ft8203, panel);
}

static int ft8203_power_on(struct ft8203 *ctx)
{
	struct regulator_bulk_data *s = ctx->supplies;
	int ret;

	/* Order and delays from qcom,panel-supply-entries (pre/post-on-sleep) */
	usleep_range(3000, 4000);
	ret = regulator_enable(s[0].consumer);		/* vddi */
	if (ret)
		return ret;
	usleep_range(4000, 5000);

	usleep_range(3000, 4000);
	ret = regulator_enable(s[1].consumer);		/* vsp */
	if (ret)
		goto err_vddi;
	usleep_range(4000, 5000);

	ret = regulator_enable(s[2].consumer);		/* vsn */
	if (ret)
		goto err_vsp;
	usleep_range(3000, 4000);

	ret = regulator_enable(s[3].consumer);		/* boost */
	if (ret)
		goto err_vsn;
	usleep_range(1000, 2000);

	/* panel_reset: pre-on 5 ms, then qcom,mdss-dsi-reset-sequence <1 5 0 4 1 12> */
	usleep_range(5000, 6000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(5000, 6000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	usleep_range(4000, 5000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	msleep(12);

	return 0;

err_vsn:
	regulator_disable(s[2].consumer);
err_vsp:
	regulator_disable(s[1].consumer);
err_vddi:
	regulator_disable(s[0].consumer);
	return ret;
}

static void ft8203_power_off(struct ft8203 *ctx)
{
	struct regulator_bulk_data *s = ctx->supplies;

	/* Reverse order, pre/post-off-sleep from the supply entries */
	msleep(90);
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	usleep_range(1000, 2000);

	regulator_disable(s[3].consumer);		/* boost */
	usleep_range(4000, 5000);
	regulator_disable(s[2].consumer);		/* vsn */
	usleep_range(4000, 5000);
	regulator_disable(s[1].consumer);		/* vsp */
	usleep_range(8000, 9000);
	regulator_disable(s[0].consumer);		/* vddi */
}

static int ft8203_on(struct ft8203 *ctx)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	/* qcom,mdss-dsi-on-command (dsi_lp_mode), decoded 1:1 */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x82, 0x01, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0x80);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x82, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0x93);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc5, 0x16);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0x97);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc5, 0x16);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0x9e);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc5, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0x9a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc5, 0x25);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0x9c);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc5, 0x25);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0xb6);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc5, 0x07, 0x07);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0xb8);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc5, 0x1b, 0x1b);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xd8, 0xaa, 0xaa);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0x82);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc5, 0x95);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0x83);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc5, 0x07);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xe1, 0x05, 0x0e, 0x23, 0x37, 0x42, 0x4f, 0x62, 0x70, 0x73, 0x81, 0x83, 0x98, 0x6c, 0x59, 0x5a, 0x50);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0x10);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xe1, 0x49, 0x3f, 0x32, 0x29, 0x22, 0x14, 0x06, 0x02);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xe2, 0x05, 0x0e, 0x23, 0x37, 0x42, 0x4f, 0x62, 0x70, 0x73, 0x81, 0x83, 0x98, 0x6c, 0x59, 0x5a, 0x50);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0x10);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xe2, 0x49, 0x3f, 0x32, 0x29, 0x22, 0x14, 0x06, 0x02);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0x80);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xa4, 0x8c);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0xa0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf3, 0x10);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0xa1);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb3, 0x06, 0x40);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0xa3);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb3, 0x0a, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0xa5);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb3, 0x00, 0x13);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0xd0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc1, 0xb0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0x80);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xcb, 0x3f, 0x33, 0x30, 0x3f, 0x30, 0x33, 0x30);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0x87);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xcb, 0x3f);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0x88);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xcb, 0x00, 0x3f, 0x33, 0x33, 0x33, 0x30, 0x3f, 0x3f);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0x90);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xcb, 0x00, 0x33, 0x33, 0x33, 0x30, 0x30, 0x3f);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0x97);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xcb, 0x33);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0x98);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xcb, 0xd7, 0x14, 0x14, 0xd4, 0x14, 0x14, 0x14, 0xd7);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0xa0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xcb, 0x00, 0xfc, 0x14, 0x14, 0x14, 0x14, 0xeb, 0xd4);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0xa8);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xcb, 0x28, 0x14, 0x14, 0x14, 0x14, 0x14, 0xc0, 0x14);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0xb0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xcb, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0xb7);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xcb, 0xff);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0xb8);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xcb, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0xc0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xcb, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0xc7);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xcb, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0xd0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xcb, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0x80);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xcc, 0x38, 0x2d, 0x2d, 0x2d, 0x13, 0x13, 0x13, 0x07);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0x88);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xcc, 0x08, 0x2c, 0x17, 0x2b, 0x2b, 0x01, 0x23, 0x23);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0x90);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xcc, 0x11, 0x12, 0x0f, 0x10, 0x2c, 0x2c);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0x80);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xcd, 0x38, 0x2d, 0x2d, 0x2d, 0x13, 0x13, 0x13, 0x07);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0x88);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xcd, 0x08, 0x2c, 0x17, 0x2b, 0x2b, 0x01, 0x23, 0x23);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0x90);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xcd, 0x11, 0x12, 0x0f, 0x10, 0x2c, 0x2c);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0xa0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xcc, 0x38, 0x2d, 0x2d, 0x2d, 0x13, 0x13, 0x13, 0x08);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0xa8);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xcc, 0x07, 0x17, 0x2c, 0x2b, 0x2b, 0x01, 0x23, 0x23);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0xb0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xcc, 0x10, 0x0f, 0x12, 0x11, 0x2c, 0x2c);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0xa0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xcd, 0x38, 0x2d, 0x2d, 0x2d, 0x13, 0x13, 0x13, 0x08);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0xa8);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xcd, 0x07, 0x17, 0x2c, 0x2b, 0x2b, 0x01, 0x23, 0x23);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0xb0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xcd, 0x10, 0x0f, 0x12, 0x11, 0x2c, 0x2c);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0x81);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc2, 0x40);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0x90);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc2, 0x84, 0x02, 0x58, 0x93);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0x94);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc2, 0x83, 0x02, 0x58, 0x93);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0xe0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc2, 0x8b, 0x09, 0x01, 0x61, 0x93, 0x00, 0x00, 0x03);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0xe8);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc2, 0x8a, 0x0a, 0x01, 0x61, 0x93, 0x00, 0x00, 0x03);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0xf0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc2, 0x89, 0x07, 0x01, 0x61, 0x93, 0x00, 0x00, 0x03);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0xf8);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc2, 0x88, 0x08, 0x01, 0x61, 0x93, 0x00, 0x00, 0x03);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0xe0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc3, 0x36, 0x24, 0x00, 0xc2);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0xe4);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc3, 0x35, 0x24, 0x00, 0x76);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0xe8);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc3, 0x35, 0x24, 0x00, 0x76);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0xa0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc3, 0x01, 0xaa, 0x0a, 0x0f);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0xfd);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xcb, 0x82);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0x80);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc0, 0x00, 0x79, 0x00, 0x10, 0x00, 0x10);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0x90);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc0, 0x00, 0x79, 0x00, 0x10, 0x00, 0x10);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0xa0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc0, 0x00, 0xf0, 0x00, 0x10, 0x00, 0x10);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0xb0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc0, 0x00, 0x79, 0x00, 0x10, 0x10);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0xa3);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc1, 0x2d, 0x21, 0x04);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0x80);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xce, 0x01, 0x81, 0xff, 0xff, 0x00, 0x88, 0x00, 0xd0, 0x00, 0x54, 0x00, 0x68);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0x90);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xce, 0x00, 0x87, 0x0e, 0x00, 0x00, 0x87, 0x80, 0xff, 0xff, 0x00, 0x06, 0x00, 0x17, 0x0f);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0xa0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xce, 0x00, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0xb0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xce, 0x20, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0xd1);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xce, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0xe1);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xce, 0x09, 0x02, 0x30, 0x02, 0x30, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0xf0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xce, 0x80, 0x17, 0x0b, 0x01, 0x10, 0x01, 0xa0, 0x00, 0x20, 0x25);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0xb0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xcf, 0x00, 0x00, 0x46, 0x4a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0xb5);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xcf, 0x05, 0x05, 0x00, 0x04);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0xc0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xcf, 0x09, 0x09, 0xec, 0xf0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0xc5);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xcf, 0x00, 0x0a, 0x08, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0x90);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc4, 0x88);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0x92);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc4, 0xc0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0xc5);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc3, 0x00, 0x00, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0xd6);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc1, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0xd0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc1, 0x90);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0xbf);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc0, 0x04);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0xd5);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc0, 0xf1);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0x9f);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc5, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0x91);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc5, 0x4c);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0xd7);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xce, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0x94);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc5, 0x46);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0x98);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc5, 0x64);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0x9b);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc5, 0x65);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0x9d);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc5, 0x65);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0x9a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xcf, 0xff);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0x82);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xa5, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0x8c);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xcf, 0x40, 0x40);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0xa2);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf5, 0x1f);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0xc1);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc0, 0x11);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0x9a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf5, 0x3f);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0x9c);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf5, 0x1e);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0xb6);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc0, 0x02);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0xb0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xca, 0x09, 0x09, 0x04);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0xb4);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xca, 0x03);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0x80);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xca, 0xf0, 0xd9, 0xc8, 0xba, 0xaf, 0xa6, 0x9e, 0x98, 0x92, 0x8d, 0x88, 0x84);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0x90);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xca, 0xfb, 0xff, 0x33);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0xa0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xca, 0x06);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xec, 0x00, 0x04, 0x08, 0x0c, 0x00, 0x10, 0x14, 0x18, 0x1c, 0x40, 0x20, 0x28, 0x30, 0x38, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0x10);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xec, 0x40, 0x48, 0x4f, 0x57, 0xf0, 0x5f, 0x67, 0x6f, 0x77, 0xff, 0x7f, 0x87, 0x8f, 0x97, 0xff);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0x20);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xec, 0x9f, 0xa7, 0xaf, 0xb7, 0xff, 0xbf, 0xc7, 0xcf, 0xd7, 0xaf, 0xdf, 0xe7, 0xee, 0xf6, 0xfa);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0x30);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xec, 0xfa, 0xfc, 0xfd, 0x3f);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0x40);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xec, 0x00, 0x04, 0x08, 0x0c, 0x00, 0x10, 0x14, 0x18, 0x1c, 0x00, 0x20, 0x28, 0x30, 0x38, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0x50);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xec, 0x40, 0x48, 0x50, 0x58, 0x00, 0x60, 0x68, 0x70, 0x78, 0x00, 0x80, 0x88, 0x90, 0x98, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0x60);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xec, 0xa0, 0xa8, 0xb0, 0xb8, 0x00, 0xc0, 0xc8, 0xd0, 0xd8, 0x00, 0xe0, 0xe8, 0xf0, 0xf8, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0x70);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xec, 0xfc, 0xfe, 0xff, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0x80);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xec, 0x00, 0x03, 0x07, 0x0b, 0x6c, 0x0f, 0x13, 0x16, 0x1a, 0xb1, 0x1e, 0x26, 0x2e, 0x35, 0xc5);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0x90);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xec, 0x3d, 0x45, 0x4d, 0x54, 0x86, 0x5c, 0x64, 0x6b, 0x73, 0x61, 0x7b, 0x82, 0x8a, 0x91, 0xd8);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0xa0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xec, 0x99, 0xa1, 0xa8, 0xb0, 0x72, 0xb8, 0xbf, 0xc7, 0xcf, 0x6c, 0xd7, 0xde, 0xe7, 0xee, 0xcc);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0xb0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xec, 0xf2, 0xf4, 0xf5, 0x2b);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0xb0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb4, 0x00, 0x28, 0x02, 0x00, 0x04, 0x9f, 0x00, 0x0b, 0x02, 0x77, 0x01, 0xb1, 0x10, 0xd0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0xbe);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb4, 0x00, 0x80);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0xd5);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc1, 0x80);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0x80);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x92);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x1c, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0xa4);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf3, 0x0b);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfa, 0x02);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0xa8);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc5, 0x09);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0xcb);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc5, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0xb6);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc5, 0x07, 0x07);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0xb8);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc5, 0x1b, 0x1b);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0x91);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xa5, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0xb2);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xce, 0x79);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0xa4);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf3, 0x0b);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfa, 0x5a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0xa4);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf3, 0x0b);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfa, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0xa8);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc5, 0x09);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0xcb);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc5, 0x09);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0xb6);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc5, 0x05, 0x05);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0xb8);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc5, 0x19, 0x19);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0x91);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xa5, 0x40);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0xb2);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xce, 0x7a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0xa4);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf3, 0x0b);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfa, 0x5a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x00, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0x80);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x00, 0x00);
	mipi_dsi_dcs_exit_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 128);
	// mipi_dsi_dcs_set_display_on_multi(&dsi_ctx);
	// mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x35, 0x00);
	// mipi_dsi_msleep(&dsi_ctx, 32);

	// /* samsung,tft_pwm_tx_cmds: BCTRL | DD | BL on */
	// mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_WRITE_CONTROL_DISPLAY, 0x24);

	return dsi_ctx.accum_err;
}

static int ft8203_off(struct ft8203 *ctx)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	/* qcom,mdss-dsi-pre-off-command + qcom,mdss-dsi-off-command */
	mipi_dsi_dcs_set_display_off_multi(&dsi_ctx);
	mipi_dsi_dcs_enter_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 32);

	return dsi_ctx.accum_err;
}

static int ft8203_prepare(struct drm_panel *panel)
{
	struct ft8203 *ctx = to_ft8203(panel);
	struct device *dev = &ctx->dsi->dev;
	int ret;

	ret = ft8203_power_on(ctx);
	if (ret) {
		dev_err(dev, "failed to power on: %d\n", ret);
		return ret;
	}

	ret = ft8203_on(ctx);
	if (ret) {
		dev_err(dev, "failed to initialize panel: %d\n", ret);
		gpiod_set_value_cansleep(ctx->reset_gpio, 1);
		ft8203_power_off(ctx);
		return ret;
	}

	/*
	 * Downstream sets samsung,no_qcom_pps: the host does not send a PPS
	 * or compression-mode packet; the panel's own init configures its
	 * decoder. If the picture is garbage, try sending them here:
	 *
	 *	struct drm_dsc_picture_parameter_set pps;
	 *	drm_dsc_pps_payload_pack(&pps, &ctx->dsc);
	 *	mipi_dsi_picture_parameter_set(ctx->dsi, &pps);
	 *	mipi_dsi_compression_mode(ctx->dsi, true);
	 */

	if (ctx->boost) {
		/* MAX77816 VOUT = 5.0 V as stock; the 0x23 default is too low for
		 * the backlight driver to start at low PWM after power-on */
		ret = i2c_smbus_write_byte_data(ctx->boost, 0x04, 0x78);
		if (ret)
			dev_warn(&ctx->dsi->dev, "boost write failed: %d\n", ret);
		else
			dev_dbg(&ctx->dsi->dev, "boost VOUT 0x%02x\n",
				 i2c_smbus_read_byte_data(ctx->boost, 0x04));
	}

	return 0;
}

static int ft8203_unprepare(struct drm_panel *panel)
{
	struct ft8203 *ctx = to_ft8203(panel);
	int ret;

	ret = ft8203_off(ctx);
	if (ret)
		dev_err(&ctx->dsi->dev, "failed to turn off panel: %d\n", ret);

	ft8203_power_off(ctx);

	return 0;
}

static int ft8203_enable(struct drm_panel *panel)
{
	struct ft8203 *ctx = to_ft8203(panel);
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };
	u16 v = clamp_t(u16, ctx->panel.backlight->props.brightness, 32,
			FT8203_MAX_BRIGHTNESS);
	u8 pwm[3] = { MIPI_DCS_SET_DISPLAY_BRIGHTNESS, v >> 4, v & 0x0f };
	/* first BL-on after power-on must be without dimming (DD=0), or it latches dim */
	u8 ctrl[2] = { MIPI_DCS_WRITE_CONTROL_DISPLAY, 0x24 };

	mipi_dsi_dcs_set_display_on_multi(&dsi_ctx);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x35, 0x00);	/* TE on */
	mipi_dsi_dcs_write_buffer_multi(&dsi_ctx, pwm, sizeof(pwm));
	mipi_dsi_dcs_write_buffer_multi(&dsi_ctx, ctrl, sizeof(ctrl));
	mipi_dsi_msleep(&dsi_ctx, 80);	/* let the BL start before dimming is enabled */

	return dsi_ctx.accum_err;
}

static const struct drm_display_mode ft8203_mode = {
	.clock = (1600 + 90 + 8 + 89) * (2560 + 16 + 8 + 13) * 60 / 1000,
	.hdisplay = 1600,
	.hsync_start = 1600 + 90,
	.hsync_end = 1600 + 90 + 8,
	.htotal = 1600 + 90 + 8 + 89,
	.vdisplay = 2560,
	.vsync_start = 2560 + 16,
	.vsync_end = 2560 + 16 + 8,
	.vtotal = 2560 + 16 + 8 + 13,
	.width_mm = 166,
	.height_mm = 266,
	.type = DRM_MODE_TYPE_DRIVER,
};

static int ft8203_get_modes(struct drm_panel *panel,
			    struct drm_connector *connector)
{
	return drm_connector_helper_get_modes_fixed(connector, &ft8203_mode);
}

static const struct drm_panel_funcs ft8203_panel_funcs = {
	.prepare = ft8203_prepare,
	.enable = ft8203_enable,
	.unprepare = ft8203_unprepare,
	.get_modes = ft8203_get_modes,
};

static int ft8203_bl_update_status(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	u16 v = backlight_get_brightness(bl);		/* 0..3231 */
	u8 pwm[2], dim[1];
	ssize_t ret;

	if (v == 0)
   		return 0;
   	if (v < 32)
   		v = 32;

	pwm[0] = v >> 4;
	pwm[1] = v & 0x0f;
	dim[0] = v > 14 ? 0x2c : 0x24;

	dev_dbg(&dsi->dev, "bl update %u\n", v);	/* debug */

	ret = mipi_dsi_dcs_write(dsi, MIPI_DCS_SET_DISPLAY_BRIGHTNESS, pwm, 2);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_dcs_write(dsi, MIPI_DCS_WRITE_CONTROL_DISPLAY, dim, 1);
	return ret < 0 ? ret : 0;
}

static const struct backlight_ops ft8203_bl_ops = {
	.update_status = ft8203_bl_update_status,
};

static struct backlight_device *
ft8203_create_backlight(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	const struct backlight_properties props = {
		.type = BACKLIGHT_RAW,
		.brightness = FT8203_DEF_BRIGHTNESS,
		.max_brightness = FT8203_MAX_BRIGHTNESS,
	};

	return devm_backlight_device_register(dev, dev_name(dev), dev, dsi,
					      &ft8203_bl_ops, &props);
}

static void ft8203_put_boost(void *data)
{
	struct i2c_client *client = data;

	put_device(&client->dev);
}

static int ft8203_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct device_node *np;
	struct ft8203 *ctx;
	int ret;

	ctx = devm_drm_panel_alloc(dev, struct ft8203, panel,
				   &ft8203_panel_funcs,
				   DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);

	ret = devm_regulator_bulk_get_const(dev, ARRAY_SIZE(ft8203_supplies),
					    ft8203_supplies, &ctx->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get regulators\n");

	/*
	 * ABL already brought the panel up (splash). Request reset as
	 * deasserted so probing does not blank it before prepare runs.
	 */
	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->reset_gpio),
				     "failed to get reset-gpios\n");

	/* Optional MAX77816 backlight boost on I2C8 (stock sets VOUT 5 V) */
	np = of_parse_phandle(dev->of_node, "samsung,boost", 0);
	if (np) {
		ctx->boost = of_find_i2c_device_by_node(np);
		of_node_put(np);
		if (!ctx->boost)
			return dev_err_probe(dev, -EPROBE_DEFER,
					     "boost i2c device not ready\n");
		ret = devm_add_action_or_reset(dev, ft8203_put_boost, ctx->boost);
		if (ret)
			return ret;
	}

	ctx->dsi = dsi;
	mipi_dsi_set_drvdata(dsi, ctx);

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	/* non-burst sync pulse (stock default), EoT appended, init sent in LP */
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE |
			  MIPI_DSI_MODE_LPM;

	ctx->panel.prepare_prev_first = true;

	ctx->panel.backlight = ft8203_create_backlight(dsi);
	if (IS_ERR(ctx->panel.backlight))
		return dev_err_probe(dev, PTR_ERR(ctx->panel.backlight),
				     "failed to create backlight\n");

	drm_panel_add(&ctx->panel);

	/* qcom,compression-mode = "dsc", DSC 1.1, 800x40 slices, 8 bpc -> 8 bpp */
	ctx->dsc.dsc_version_major = 1;
	ctx->dsc.dsc_version_minor = 1;
	ctx->dsc.slice_height = 40;
	ctx->dsc.slice_width = 800;
	ctx->dsc.slice_count = 1600 / 800;
	ctx->dsc.bits_per_component = 8;
	ctx->dsc.bits_per_pixel = 8 << 4;	/* 4 fractional bits */
	ctx->dsc.block_pred_enable = false;	/* not set downstream for FT8203 */
	dsi->dsc = &ctx->dsc;
	dsi->mode_flags |= MIPI_DSI_MODE_DSC_ALL_SLICES_IN_PKT;	/* qcom,mdss-dsc-slice-per-pkt = <2> == slice_count */

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		dev_err_probe(dev, ret, "failed to attach to DSI host\n");
		drm_panel_remove(&ctx->panel);
		return ret;
	}

	return 0;
}

static void ft8203_remove(struct mipi_dsi_device *dsi)
{
	struct ft8203 *ctx = mipi_dsi_get_drvdata(dsi);
	int ret;

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		dev_err(&dsi->dev, "failed to detach from DSI host: %d\n", ret);

	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id ft8203_of_match[] = {
	{ .compatible = "boe,ts124qdm-ft8203" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ft8203_of_match);

static struct mipi_dsi_driver ft8203_driver = {
	.probe = ft8203_probe,
	.remove = ft8203_remove,
	.driver = {
		.name = "panel-boe-ts124qdm-ft8203",
		.of_match_table = ft8203_of_match,
	},
};
module_mipi_dsi_driver(ft8203_driver);

MODULE_DESCRIPTION("DRM driver for BOE TS124QDM FT8203 DSI panel (Galaxy Tab S7 FE)");
MODULE_LICENSE("GPL");
