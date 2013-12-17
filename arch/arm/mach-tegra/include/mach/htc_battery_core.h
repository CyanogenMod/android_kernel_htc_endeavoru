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

#define LIMIT_CHARGING_ANONYMOUS	(1U << 0)
#define LIMIT_CHARGING_PHONE_CALL	(1U << 1)
#define LIMIT_CHARGING_NAVIGATION	(1U << 2)
#define LIMIT_CHARGING_MEDIALINK	(1U << 3)
#define LIMIT_CHARGING_DATA_ENCRYPTION	(1U << 4)

#define KEEP_EARLY_SUSPEND_LIMIT_CHARGING	(LIMIT_CHARGING_PHONE_CALL |\
						LIMIT_CHARGING_DATA_ENCRYPTION)

enum htc_batt_rt_attr {
	HTC_BATT_RT_VOLTAGE = 0,
	HTC_BATT_RT_CURRENT,
	HTC_BATT_RT_TEMPERATURE,
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
	int (*func_get_batt_rt_attr)(enum htc_batt_rt_attr attr, int* val);
	int (*func_show_batt_attr)(struct device_attribute *attr, char *buf);
	int (*func_show_batt_power_meter)(struct device_attribute *attr, char *buf);
	int (*func_show_htc_extension_attr)(struct device_attribute *attr, char *buf);
	int (*func_get_battery_info)(struct battery_info_reply *buffer);
	int (*func_charger_control)(enum charger_control_flag);
	void (*func_set_full_level)(int full_level);
	void (*func_phone_call_notification)(int phone_call);
	void (*func_limit_charging_notification)(unsigned int limit_type, int enable);
};

#ifdef CONFIG_HTC_BATT_CORE
extern int htc_battery_core_update(void);
extern int htc_battery_is_charging_full(void);
extern int htc_battery_core_register(struct device *dev, struct htc_battery_core *htc_battery);
#else
static int htc_battery_core_update(void) { return 0; }
extern int htc_battery_is_charging_full(void) { return 0; }
static int htc_battery_core_register(struct device *dev, struct htc_battery_core *htc_battery) { return 0; }
#endif
#endif
