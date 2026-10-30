/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Silicon Mitus SM5714 IF-PMIC (charger, fuel gauge, MUIC)
 *
 * Register names follow Samsung's downstream sm5714-private.h.
 */
#ifndef __LINUX_MFD_SM5714_H
#define __LINUX_MFD_SM5714_H

#include <linux/bits.h>
#include <linux/types.h>

struct device;
struct regmap;

/* I2C addresses; the DT node / main client is the charger block */
#define SM5714_I2C_ADDR_MUIC		0x25
#define SM5714_I2C_ADDR_CHG		0x49
#define SM5714_I2C_ADDR_FG		0x71

/* Charger block: 8-bit registers */
#define SM5714_CHG_REG_STATUS1		0x0d
#define SM5714_CHG_REG_STATUS2		0x0e
#define SM5714_CHG_REG_STATUS3		0x0f
#define SM5714_CHG_REG_STATUS4		0x10
#define SM5714_CHG_REG_STATUS5		0x11
#define SM5714_CHG_REG_CNTL1		0x13
#define SM5714_CHG_REG_CNTL2		0x14
#define SM5714_CHG_REG_VBUSCNTL		0x15
#define SM5714_CHG_REG_WDTCNTL		0x22
#define SM5714_CHG_REG_BSTCNTL1		0x23
#define SM5714_CHG_REG_DEVICEID		0x50

#define SM5714_CHG_DEVICEID_VENDOR	GENMASK(2, 0)
#define SM5714_CHG_DEVICEID_REV		GENMASK(7, 3)

/*
 * STATUS bits mirror the INT1/INT2 layout of the downstream IRQ table.
 * VBUSPOK and CHGON verified on hardware (plug/unplug); the rest inferred.
 */
#define SM5714_CHG_STATUS1_VBUSPOK	BIT(0)
#define SM5714_CHG_STATUS1_VBUSUVLO	BIT(1)
#define SM5714_CHG_STATUS1_VBUSOVP	BIT(2)
#define SM5714_CHG_STATUS1_VBUSLIMIT	BIT(3)
#define SM5714_CHG_STATUS2_AICL		BIT(0)
#define SM5714_CHG_STATUS2_CHGON	BIT(3)
#define SM5714_CHG_STATUS2_Q4FULLON	BIT(4)
#define SM5714_CHG_STATUS2_TOPOFF	BIT(5)
#define SM5714_CHG_STATUS2_DONE		BIT(6)
#define SM5714_CHG_STATUS2_WDTMROFF	BIT(7)
#define SM5714_CHG_STATUS3_OTGFAIL	BIT(2)	/* OTG over-current */
#define SM5714_CHG_STATUS4_BOOSTPOK	BIT(1)	/* verified: set with OTG on */

/* CNTL2: operating mode (downstream sm5714_charger_oper.c) */
#define SM5714_CHG_CNTL2_OP_MODE	GENMASK(3, 0)
#define SM5714_CHG_OP_MODE_SUSPEND	0x0
#define SM5714_CHG_OP_MODE_CHG_ON_VBUS	0x5	/* normal / idle */
#define SM5714_CHG_OP_MODE_USB_OTG	0x7	/* reverse boost onto VBUS */
#define SM5714_CHG_OP_MODE_FLASH_BOOST	0x8

/* BSTCNTL1: boost output voltage and OTG current limit */
#define SM5714_CHG_BSTCNTL1_BSTOUT	GENMASK(3, 0)
#define SM5714_CHG_BSTCNTL1_OTG_CURRENT	GENMASK(7, 6)
#define SM5714_CHG_BSTOUT_5100MV	0x6
#define SM5714_CHG_OTG_CURRENT_900MA	0x1

/* MUIC block: 8-bit registers */
#define SM5714_MUIC_REG_DEVICEID	0x00
#define SM5714_MUIC_REG_CNTL		0x05
#define SM5714_MUIC_REG_MANUAL_SW	0x06
#define SM5714_MUIC_REG_DEVICETYPE1	0x07
#define SM5714_MUIC_REG_DEVICETYPE2	0x08
#define SM5714_MUIC_REG_RESET		0x1e
#define SM5714_MUIC_REG_VBUS		0x3b
#define SM5714_MUIC_REG_OTP_IF_STS	0x58

/*
 * MANUAL_SW: bit 7 selects manual switching, D- path in [5:3], D+ in [2:0]
 * (0 open, 1 USB, 2 JTAG, 3 UART, 4 GND). Verified: auto mode leaves D+/D-
 * open when VBUS comes from our own OTG boost (no BC1.2 detection runs).
 */
#define SM5714_MUIC_MANUAL_SW_MANUAL	BIT(7)
#define SM5714_MUIC_MANUAL_SW_PATH	GENMASK(5, 0)
#define SM5714_MUIC_MANUAL_SW_USB	0x09

/* Fuel gauge block: 16-bit little-endian words */
#define SM5714_FG_REG_DEVICE_ID		0x00
#define SM5714_FG_REG_SYSTEM_STATUS	0x10
#define SM5714_FG_REG_AUX_STAT		0x22
#define SM5714_FG_REG_SRAM_RADDR	0x8c
#define SM5714_FG_REG_SRAM_RDATA	0x8d
#define SM5714_FG_REG_RESET		0x91

#define SM5714_FG_SYSTEM_STATUS_INIT_DONE	BIT(12)
#define SM5714_FG_SYSTEM_STATUS_JIG		BIT(15)
#define SM5714_FG_AUX_STAT_CURR_X2_5		BIT(10)

/* Fuel gauge SRAM (read via SRAM_RADDR / SRAM_RDATA) */
#define SM5714_FG_SRAM_SOC		0x00
#define SM5714_FG_SRAM_OCV		0x01
#define SM5714_FG_SRAM_VBAT		0x03
#define SM5714_FG_SRAM_CURRENT		0x05
#define SM5714_FG_SRAM_TEMPERATURE	0x07
#define SM5714_FG_SRAM_VBAT_AVG		0x08
#define SM5714_FG_SRAM_CURRENT_AVG	0x09
#define SM5714_FG_SRAM_SOC_CYCLE	0x87

struct sm5714 {
	struct device *dev;
	struct regmap *regmap_chg;
	struct regmap *regmap_fg;
	struct regmap *regmap_muic;
	unsigned int rev;
};

/* Route D+/D- to the USB controller (manual) or back to automatic */
int sm5714_muic_set_usb_path(struct sm5714 *sm5714, bool usb);

#endif /* __LINUX_MFD_SM5714_H */
