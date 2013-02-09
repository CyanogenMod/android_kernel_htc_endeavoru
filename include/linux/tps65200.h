/*
 * Copyright (C) 2007 HTC Incorporated
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
#ifndef _TPS65200_H_
#define _TPS65200_H_
#include <linux/notifier.h>

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

/* These value are used in tps65200.c and supported by some power_supply.h.      */
/* It is placed here, however, due to it might be integrated with upper macros   */
/* since all these value are only used for tps65200.c                            */
#if 0 
enum {
	POWER_SUPPLY_DISABLE_CHARGE = 0,
	POWER_SUPPLY_ENABLE_SLOW_CHARGE,
	POWER_SUPPLY_ENABLE_FAST_CHARGE,
	POWER_SUPPLY_ENABLE_9VAC_CHARGE,
	POWER_SUPPLY_ENABLE_WIRELESS_CHARGE,
	POWER_SUPPLY_ENABLE_SLOW_HV_CHARGE,
	POWER_SUPPLY_ENABLE_FAST_HV_CHARGE,
	POWER_SUPPLY_ENABLE_INTERNAL,
};
#endif

enum wled_ctl_t {
	WLED_DISABLE = 0,
	WLED_ENABLE,
	WLED_STATUS
};

struct tps65200_chg_int_data {
	int gpio_chg_int;
	int tps65200_reg;
	struct delayed_work int_work;
};

struct tps65200_platform_data {
	int gpio_chg_stat;
	int gpio_chg_int;
#ifdef CONFIG_SUPPORT_DQ_BATTERY
	int dq_result;
#endif
};

struct tps65200_chg_int_notifier {
	struct list_head notifier_link;
	const char *name;
	void (*func)(int int_reg, int value);
};

#ifdef CONFIG_CHARGER_TPS65200
extern int tps_set_charger_ctrl(u32 ctl);
extern int tps_register_notifier(struct tps65200_chg_int_notifier *notifier);
extern int tps_charger_dump_status(void);
#else
static int tps_set_charger_ctrl(u32 ctl) { return 0; }
static int tps_register_notifier(struct tps65200_chg_int_notifier *notifier) { return 0; }
static int tps_charger_dump_status(void) { return 0; }
#endif

#endif
