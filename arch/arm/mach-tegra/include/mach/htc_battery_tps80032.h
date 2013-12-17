/*
 * Copyright (C) 2011 HTC Incorporated
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
#ifndef _HTC_BATTERY_H_
#define _HTC_BATTERY_H_

#define ADC_REPLY_ARRAY_SIZE		10

/* ioctl define */
#define HTC_BATT_IOCTL_MAGIC		0xba

#define DEBUG_LOG_LENGTH		1024
#define POWER_METER_LENGTH		16

#define HTC_BATT_IOCTL_READ_SOURCE \
	_IOR(HTC_BATT_IOCTL_MAGIC, 1, unsigned int)
#define HTC_BATT_IOCTL_SET_BATT_ALARM \
	_IOW(HTC_BATT_IOCTL_MAGIC, 2, unsigned int)
#define HTC_BATT_IOCTL_GET_ADC_VREF \
	_IOR(HTC_BATT_IOCTL_MAGIC, 3, unsigned int[ADC_REPLY_ARRAY_SIZE])
#define HTC_BATT_IOCTL_GET_ADC_ALL \
	_IOR(HTC_BATT_IOCTL_MAGIC, 4, struct battery_adc_reply)
#define HTC_BATT_IOCTL_CHARGER_CONTROL \
	_IOW(HTC_BATT_IOCTL_MAGIC, 5, unsigned int)
#define HTC_BATT_IOCTL_UPDATE_BATT_INFO \
	_IOW(HTC_BATT_IOCTL_MAGIC, 6, struct battery_info_reply)
#define HTC_BATT_IOCTL_BATT_DEBUG_LOG \
	_IOW(HTC_BATT_IOCTL_MAGIC, 7, char[DEBUG_LOG_LENGTH])
#define HTC_BATT_IOCTL_SET_VOLTAGE_ALARM \
	_IOW(HTC_BATT_IOCTL_MAGIC, 8, struct battery_vol_alarm)
#define HTC_BATT_IOCTL_SET_ALARM_TIMER_FLAG \
	_IOW(HTC_BATT_IOCTL_MAGIC, 9, unsigned int)
#define HTC_BATT_IOCTL_BATT_POWER_METER \
	_IOW(HTC_BATT_IOCTL_MAGIC, 10, char[POWER_METER_LENGTH])
#define HTC_BATT_IOCTL_SET_VREG \
	_IOW(HTC_BATT_IOCTL_MAGIC, 11, unsigned int)
#define HTC_BATT_IOCTL_SET_VSYS_REG \
	_IOW(HTC_BATT_IOCTL_MAGIC, 12, unsigned int)
#define HTC_BATT_IOCTL_ADC_TRIGGER \
	_IOW(HTC_BATT_IOCTL_MAGIC, 13, unsigned int)

#define REGULAR_BATTERRY_TIMER		"regular_timer"
#define CABLE_DETECTION			"cable"
#define CHARGER_IC_INTERRUPT		"charger_int"

#define XOADC_MPP			0
#define PM_MPP_AIN_AMUX			1

#define MBAT_IN_LOW_TRIGGER		0
#define MBAT_IN_HIGH_TRIGGER		1

#define NORMAL_TEMP_COND	0
#define LOW_TEMP_COND		1
#define HIGH_TEMP_COND		2

#define BATTERY_LEVEL_SIG_SHIFT		(8)
#define BATTERY_LEVEL_SIG_MASK		(0xFFFFFF)
#define BATTERY_LEVEL_SIG		(0xBBC566)
#define BATTERY_IS_CHARGING_FULL_SHIFT	(7)
#define BATTERY_IS_CHARGING_FULL_MASK	(0x1)
#define BATTERY_LEVEL_MASK		(0x7F)
#define BATTERY_LEVEL_NO_VALUE		(0xFF)

#define ADC2TEMP_MAP_NUM (15)

struct battery_adc_reply {
	u32 adc_voltage[ADC_REPLY_ARRAY_SIZE];
	u32 adc_current[ADC_REPLY_ARRAY_SIZE];
	u32 adc_temperature[ADC_REPLY_ARRAY_SIZE];
	u32 adc_battid[ADC_REPLY_ARRAY_SIZE];
	u32 adc_vzero_clb[ADC_REPLY_ARRAY_SIZE];
};

struct battery_vol_alarm {
	int lower_threshold;
	int upper_threshold;
	int enable;
};

struct batt_adc2temp {
	unsigned int adc;
	int temperature;
};

/* information about the system we're running on */
extern unsigned int system_rev;

enum {
	GUAGE_NONE,
	GUAGE_MODEM,
	GUAGE_DS2784,
	GUAGE_DS2746,
	GUAGE_BQ27510,
	GUAGE_TPS80032,
};

enum {
	LINEAR_CHARGER,
	SWITCH_CHARGER_TPS65200,
	SWITCH_CHARGER_TPS80032,
};

enum {
	BATT_TIMER_WAKE_LOCK = 0,
	BATT_IOCTL_WAKE_LOCK,
};

enum {
	HTC_BATT_DEBUG_UEVT = 1U << 1,
	HTC_BATT_DEBUG_USER_QUERY = 1U << 2,
	HTC_BATT_DEBUG_USB_NOTIFY = 1U << 3,
	HTC_BATT_DEBUG_FULL_LOG = 1U << 4,
};

struct htc_battery_platform_data {
	int gpio_mbat_in;
	int gpio_mbat_in_trigger_level;
	int guage_driver;
	int charger;
	int vzero_clb_channel;
	int volt_adc_offset;
	int power_off_by_id;
	int sw_temp_25;
	struct batt_adc2temp adc2temp_map[ADC2TEMP_MAP_NUM];
};

#endif
