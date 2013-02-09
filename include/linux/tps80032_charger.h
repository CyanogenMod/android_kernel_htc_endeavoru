/*
 * Copyright (C) 2012 HTC Incorporated
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef _TPS80032_CHARGER_H_
#define _TPS80032_CHARGER_H_
#include <linux/notifier.h>
#include <linux/device.h>

#ifndef	 _TPS65200_H_ /* TODO: these are defined in the tps65200.h, collect it as common.h if necessary*/
#define ENABLE_LIMITED_CHG      0x10
#define CLEAR_LIMITED_CHG       0x11
#define CHECK_CHG		0X64
#define SET_ICL500		0X65
#define SET_ICL100		0X66
#define CHECK_INT2		0X67
#define OVERTEMP_VREG		0XC8
#define NORMALTEMP_VREG		0XC9
#define CHECK_INT1		0XCA
#define CHECK_CONTROL		0xCB
#define NORMALTEMP_VREG_HV	0xCC
#define CHECK_INT3		0XD1
#endif
#define ENABLE_HIZ_CHG		0x12
#define NORMALTEMP_VREG_HV4340	0XF1
#define CHG_CHARGE_DONE		0xF2
#define CHECK_WATCHDOG		0xF3
#define NORMALTEMP_VREG_HV4320	0XF4
#define ALLTEMP_VSYS_DISABLE	0XF5
#define NORMALTEMP_VSYS_4400	0XF6
#define NORMALTEMP_VSYS_4440	0XF7
#define OVERTEMP_VSYS_4300	0XF8
#define OVERTEMP_VSYS_4340	0XF9
#define OVERTEMP_VSYS_4140	0XFA
#define SET_ICL_NORMAL		0XFB
#define SET_ICL750		0XFC
#define UNDERTEMP_VSYS_4200	0XFD
#define UNDERTEMP_VREG		0XFE

#define OVERTEMP_VREG_BOUND		4000
#define UNDERTEMP_VREG_BOUND		4100
#define NORMALTEMP_VREG_BOUND		4200
#define NORMALTEMP_VREG_HV_BOUND	4300
#define NORMALTEMP_VREG_HV4320_BOUND	4320
#define NORMALTEMP_VREG_HV4340_BOUND	4340

#define NORMALTEMP_VSYS_4400_BOUND	4400
#define NORMALTEMP_VSYS_4440_BOUND	4440
#define OVERTEMP_VSYS_4300_BOUND	4300
#define OVERTEMP_VSYS_4340_BOUND	4340
#define OVERTEMP_VSYS_4140_BOUND	4140
#define UNDERTEMP_VSYS_4200_BOUND	4200

struct tps80032_charger_platform_data {
	int linch_gated_irq;
	int fault_wdg_irq;
	int int_chg_irq;
	int use_hiz_disable;
	int sw_temp_25_gpio;
};

struct tps80032_charger_int_notifier {
	struct list_head notifier_link;
	const char *name;
	void (*func)(int int_reg, int value);
};

#ifdef CONFIG_CHARGER_TPS80032
extern int tps80032_charger_set_ctrl(u32 ctl);
extern int tps80032_charger_register_notifier(struct tps80032_charger_int_notifier *notifier);
extern int tps80032_charger_append_attr(struct device *dev);
extern void tps80032_charger_dump_status(int cond);
#else
static int tps80032_charger_set_ctrl(u32 ctl) { return 0; }
static int tps80032_charger_register_notifier(struct tps80032_charger_int_notifier *notifier) { return 0; }
static int tps80032_charger_append_attr(struct device *dev) { return 0; };
static void tps80032_charger_dump_status(int cond) {}
#endif

#endif
