/*
 * Copyright (C) 2010 HTC Incorporated
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
#ifndef HTC_BATTERY_CORE_H
#define HTC_BATTERY_CORE_H
//#include "board.h"
#include <linux/notifier.h>
#include <linux/power_supply.h>
#include <linux/rtc.h>
#include <mach/htc_battery_common.h>

enum {
	BATT_ID = 0,
	BATT_VOL,
	BATT_TEMP,
	BATT_CURRENT,
	CHARGING_SOURCE,
	CHARGING_ENABLED,
	FULL_BAT,
	OVER_VCHG,
	BATT_STATE,
};

struct battery_info_reply {
	u32 batt_vol;
	u32 batt_id;
	s32 batt_temp;
	s32 batt_current;
	u32 batt_discharg_current;
	u32 level;
	u32 charging_source;
	u32 charging_enabled;
	u32 full_bat;
	u32 full_level;
	u32 over_vchg;
	s32 temp_fault;
	u32 batt_state;
	u32 overload;
};

struct htc_battery_core {
	int (*func_show_batt_attr)(struct device_attribute *attr, char *buf);
	int (*func_get_battery_info)(struct battery_info_reply *buffer);
	int (*func_charger_control)(enum charger_control_flag);
	void (*func_set_full_level)(int full_level);
	void (*func_phone_call_notification)(int phone_call);
};

#ifdef CONFIG_HTC_BATT_CORE
extern int htc_battery_core_update(void);
extern int htc_battery_core_register(struct device *dev, struct htc_battery_core *htc_battery);
#else
static int htc_battery_core_update(void) { return 0; }
static int htc_battery_core_register(struct device *dev, struct htc_battery_core *htc_battery) { return 0; }
#endif
#endif
