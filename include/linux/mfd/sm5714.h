/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Silicon Mitus SM5714 IF-PMIC (charger, fuel gauge, MUIC)
 *
 * Register names follow Samsung's downstream sm5714-private.h.
 */
#ifndef __LINUX_MFD_SM5714_H
#define __LINUX_MFD_SM5714_H

#include <linux/bits.h>

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

/* MUIC block: 8-bit registers */
#define SM5714_MUIC_REG_DEVICEID	0x00
#define SM5714_MUIC_REG_RESET		0x1e

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

#endif /* __LINUX_MFD_SM5714_H */
