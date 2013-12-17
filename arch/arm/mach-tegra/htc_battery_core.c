/*
 *
 * Copyright (C) 2011 HTC Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/power_supply.h>
#include <linux/platform_device.h>
#include <linux/debugfs.h>
#include <linux/wakelock.h>
#include <linux/gpio.h>
#include <linux/rtc.h>
#include <linux/workqueue.h>
#include <mach/htc_battery_core.h>
#include <linux/android_alarm.h>
#include <linux/tps80032_charger.h>

static ssize_t htc_battery_show_property(struct device *dev,
					struct device_attribute *attr,
					char *buf);

static ssize_t htc_battery_rt_attr_show(struct device *dev,
					struct device_attribute *attr,
					char *buf);

static int htc_power_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val);

static int htc_battery_get_property(struct power_supply *psy,
				    enum power_supply_property psp,
				    union power_supply_propval *val);

static ssize_t htc_battery_charger_ctrl_timer(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count);

#define HTC_BATTERY_ATTR(_name)                   \
{                                                 \
	.attr = { .name = #_name, .mode = S_IRUGO },  \
	.show = htc_battery_show_property,            \
	.store = NULL,                                \
}

#define BATT_LOG(fmt, ...) do { \
	struct timespec ts; \
	struct rtc_time tm; \
	getnstimeofday(&ts); \
	rtc_time_to_tm(ts.tv_sec, &tm); \
	printk(KERN_INFO "[BATT] " fmt \
	" at %lld (%d-%02d-%02d %02d:%02d:%02d.%09lu UTC)\n", ##__VA_ARGS__, \
	ktime_to_ns(ktime_get()), tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, \
	tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec); \
} while (0)

#define BATT_ERR(fmt, ...) do { \
	struct timespec ts; \
	struct rtc_time tm; \
	getnstimeofday(&ts); \
	rtc_time_to_tm(ts.tv_sec, &tm); \
	printk(KERN_ERR "[BATT] err:" fmt \
	" at %lld (%d-%02d-%02d %02d:%02d:%02d.%09lu UTC)\n", ##__VA_ARGS__, \
	ktime_to_ns(ktime_get()), tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, \
	tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec); \
} while (0)


struct htc_battery_core_info {
	int present;
	int htc_charge_full;
	unsigned long update_time;
	struct mutex info_lock;
	struct battery_info_reply rep;
	struct htc_battery_core func;
};

static struct htc_battery_core_info battery_core_info;
static int battery_register = 1;
#if 0	/* note: old mechanism */
static int battery_over_loading;
#endif

static struct alarm batt_charger_ctrl_alarm;
static struct work_struct batt_charger_ctrl_work;
struct workqueue_struct *batt_charger_ctrl_wq;
static unsigned int charger_ctrl_stat;
static unsigned int phone_call_stat;

static enum power_supply_property htc_battery_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_OVERLOAD,
};

static enum power_supply_property htc_power_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static char *supply_list[] = {
	"battery",
};

static struct power_supply htc_power_supplies[] = {
	{
		.name = "battery",
		.type = POWER_SUPPLY_TYPE_BATTERY,
		.properties = htc_battery_properties,
		.num_properties = ARRAY_SIZE(htc_battery_properties),
		.get_property = htc_battery_get_property,
	},
	{
		.name = "usb",
		.type = POWER_SUPPLY_TYPE_USB,
		.supplied_to = supply_list,
		.num_supplicants = ARRAY_SIZE(supply_list),
		.properties = htc_power_properties,
		.num_properties = ARRAY_SIZE(htc_power_properties),
		.get_property = htc_power_get_property,
	},
	{
		.name = "ac",
		.type = POWER_SUPPLY_TYPE_MAINS,
		.supplied_to = supply_list,
		.num_supplicants = ARRAY_SIZE(supply_list),
		.properties = htc_power_properties,
		.num_properties = ARRAY_SIZE(htc_power_properties),
		.get_property = htc_power_get_property,
	},
	{
		.name = "wireless",
		.type = POWER_SUPPLY_TYPE_WIRELESS,
		.supplied_to = supply_list,
		.num_supplicants = ARRAY_SIZE(supply_list),
		.properties = htc_power_properties,
		.num_properties = ARRAY_SIZE(htc_power_properties),
		.get_property = htc_power_get_property,
	},
};

/*
 *  For Off-mode charging animation,
 *  add a function for display driver to inform the charging animation mode.
 */
static int zcharge_enabled;
int htc_battery_get_zcharge_mode(void)
{
	return zcharge_enabled;
}
static int __init enable_zcharge_setup(char *str)
{
	int cal = simple_strtol(str, NULL, 0);

	zcharge_enabled = cal;
	return 1;
}
__setup("enable_zcharge=", enable_zcharge_setup);

static int htc_battery_get_charging_status(void)
{
	enum charger_type_t charger;
	int ret;

	mutex_lock(&battery_core_info.info_lock);
	charger = battery_core_info.rep.charging_source;
	mutex_unlock(&battery_core_info.info_lock);

	if (battery_core_info.rep.batt_id == 255)
		charger = CHARGER_UNKNOWN;

	switch (charger) {
	case CHARGER_BATTERY:
		ret = POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;
	case CHARGER_USB:
	case CHARGER_AC:
	case CHARGER_2A_AC:
	case CHARGER_9V_AC:
	case CHARGER_WIRELESS:
		if (battery_core_info.htc_charge_full)
			ret = POWER_SUPPLY_STATUS_FULL;
		else {
			if (battery_core_info.rep.charging_enabled != 0)
				ret = POWER_SUPPLY_STATUS_CHARGING;
			else
				ret = POWER_SUPPLY_STATUS_DISCHARGING;
		}
		break;
	default:
		ret = POWER_SUPPLY_STATUS_UNKNOWN;
	}

	return ret;
}

static ssize_t htc_battery_show_batt_attr(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	return battery_core_info.func.func_show_batt_attr(attr, buf);
}

static ssize_t htc_battery_show_batt_power_meter(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	return battery_core_info.func.func_show_batt_power_meter(attr, buf);
}

static ssize_t htc_battery_show_htc_extension_attr(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	if (battery_core_info.func.func_show_htc_extension_attr)
		return battery_core_info.func.func_show_htc_extension_attr(attr, buf);
	return 0;
}

static ssize_t htc_battery_set_delta(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	unsigned long delta = 0;

	delta = simple_strtoul(buf, NULL, 10);

	if (delta > 100)
		return -EINVAL;

	return count;
}

enum htc_battery_debug_flag {
	HBDF_ENABLE_MAX_CHARGING_CURRENT = 22,
	HBDF_DISABLE_MAX_CHARGING_CURRENT = 23,
};

static ssize_t htc_battery_debug_flag(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	unsigned long debug_flag;
	debug_flag = simple_strtoul(buf, NULL, 10);

	if (debug_flag > 100 || debug_flag == 0)
		return -EINVAL;

	switch (debug_flag) {
	case HBDF_ENABLE_MAX_CHARGING_CURRENT:
		battery_core_info.func.func_charger_control(ENABLE_NO_LIMIT_CHARGER);
		break;
	case HBDF_DISABLE_MAX_CHARGING_CURRENT:
		battery_core_info.func.func_charger_control(DISABLE_NO_LIMIT_CHARGER);
		break;
	};

	return 0;
}

static ssize_t htc_battery_set_full_level(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int rc = 0;
	unsigned long percent = 100;

	rc = strict_strtoul(buf, 10, &percent);
	if (rc)
		return rc;

	if (percent > 100 || percent == 0)
		return -EINVAL;

	if (!battery_core_info.func.func_set_full_level) {
		BATT_ERR("No set full level function!");
		return -ENOENT;
	}

	battery_core_info.func.func_set_full_level(percent);

	return count;
}

int htc_battery_charger_disable()
{
	int rc = 0;

	if (!battery_core_info.func.func_charger_control) {
		BATT_ERR("No charger control function!");
		return -ENOENT;
	}
	rc = battery_core_info.func.func_charger_control(STOP_CHARGER);
	if (rc < 0)
		BATT_ERR("charger control failed!");

	return rc;
}

static ssize_t htc_battery_charger_stat(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	int i = 0;

	i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", charger_ctrl_stat);

	return i;
}

static ssize_t htc_battery_phone_call_stat(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	int i = 0;

	i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", phone_call_stat);

	return i;
}

static ssize_t htc_battery_charger_switch(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	unsigned long enable = 0;
	int rc = 0;

	rc = strict_strtoul(buf, 10, &enable);
	if (rc)
		return rc;

	if (enable >= END_CHARGER)
		return -EINVAL;

	if (!battery_core_info.func.func_charger_control) {
		BATT_ERR("No charger control function!");
		return -ENOENT;
	}

	if (enable == STOP_CHARGER) {
		rc = battery_core_info.func.func_charger_control(STOP_CHARGER);
		if (rc < 0) {
			BATT_ERR("charger control failed!");
			return rc;
		}
		charger_ctrl_stat = STOP_CHARGER;
	} else if (enable == ENABLE_CHARGER) {
		rc = battery_core_info.func.func_charger_control(
							    ENABLE_CHARGER);
		if (rc < 0) {
			BATT_ERR("charger control failed!");
			return rc;
		}
		charger_ctrl_stat = ENABLE_CHARGER;
	} else if (enable == ENABLE_LIMIT_CHARGER) {
		rc = battery_core_info.func.func_charger_control(
							    ENABLE_LIMIT_CHARGER);
		if (rc < 0) {
			BATT_LOG("charger control not support!");
			return rc;
		}
		charger_ctrl_stat = ENABLE_LIMIT_CHARGER;
	} else if (enable == DISABLE_LIMIT_CHARGER) {
		rc = battery_core_info.func.func_charger_control(
							    DISABLE_LIMIT_CHARGER);
		if (rc < 0) {
			BATT_LOG("charger control not support!");
			return rc;
		}
		charger_ctrl_stat = DISABLE_LIMIT_CHARGER;
	} else if (enable == ENABLE_HIZ) {
		rc = battery_core_info.func.func_charger_control(
							    ENABLE_HIZ);
		if (rc) {
			BATT_LOG("enable hiz fail!");
			return rc;
		}
	}

	alarm_cancel(&batt_charger_ctrl_alarm);

	return count;
}

static ssize_t htc_battery_phone_call_switch(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	unsigned long phone_call = 0;
	int rc = 0;

	rc = strict_strtoul(buf, 10, &phone_call);
	if (rc)
		return rc;

	if (!battery_core_info.func.func_phone_call_notification) {
		BATT_ERR("No phone call notification function!");
		return -ENOENT;
	}

	battery_core_info.func.func_phone_call_notification(phone_call);

	if (!battery_core_info.func.func_limit_charging_notification) {
		BATT_ERR("No limit charging notification function!");
		goto no_limit_charging;
	}
	battery_core_info.func.func_limit_charging_notification(LIMIT_CHARGING_PHONE_CALL, phone_call);

no_limit_charging:
	phone_call_stat = phone_call;
	return count;
}

#define LIMIT_CHARGING_FILENODE_FUNC(_name, _limit_type)					\
	static unsigned int _name##_stat;							\
	static ssize_t htc_battery_##_name##_stat(struct device *dev,				\
				struct device_attribute *attr,					\
				char *buf)							\
	{											\
		int i = 0;									\
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", _name##_stat);			\
		return i;									\
	}											\
												\
	static ssize_t htc_battery_##_name##_switch(struct device *dev,				\
					struct device_attribute *attr,				\
					const char *buf, size_t count)				\
	{											\
		unsigned long value = 0;							\
		int rc = 0;									\
		rc = strict_strtoul(buf, 10, &value);						\
		if (rc)										\
			return rc;								\
		if (!battery_core_info.func.func_limit_charging_notification) {			\
			BATT_ERR("No limit charging notification function!");			\
			return -ENOENT;								\
		}										\
		battery_core_info.func.func_limit_charging_notification(_limit_type, value);	\
		_name##_stat = value;								\
		return count;									\
	}

#define LIMIT_CHARGING_FILENODE_ATTR(_name)			\
{								\
	.attr = { .name = #_name, .mode = S_IWUSR | S_IWGRP },	\
	.show = htc_battery_##_name##_stat,			\
	.store = htc_battery_##_name##_switch,			\
}

LIMIT_CHARGING_FILENODE_FUNC(navigation, LIMIT_CHARGING_NAVIGATION)
LIMIT_CHARGING_FILENODE_FUNC(medialink, LIMIT_CHARGING_MEDIALINK)
LIMIT_CHARGING_FILENODE_FUNC(data_encryption, LIMIT_CHARGING_DATA_ENCRYPTION)

static struct device_attribute htc_battery_attrs[] = {
	HTC_BATTERY_ATTR(batt_id),
	HTC_BATTERY_ATTR(batt_vol),
	HTC_BATTERY_ATTR(batt_temp),
	HTC_BATTERY_ATTR(batt_current),
	HTC_BATTERY_ATTR(charging_source),
	HTC_BATTERY_ATTR(charging_enabled),
	HTC_BATTERY_ATTR(full_bat),
	HTC_BATTERY_ATTR(over_vchg),
	HTC_BATTERY_ATTR(batt_state),

	__ATTR(batt_attr_text, S_IRUGO, htc_battery_show_batt_attr, NULL),
	__ATTR(batt_power_meter, S_IRUGO, htc_battery_show_batt_power_meter, NULL),
	__ATTR(htc_extension, S_IRUGO, htc_battery_show_htc_extension_attr, NULL),
};

static struct device_attribute htc_set_delta_attrs[] = {
	__ATTR(delta, S_IWUSR | S_IWGRP, NULL, htc_battery_set_delta),
	__ATTR(full_level, S_IWUSR | S_IWGRP, NULL,
		htc_battery_set_full_level),
	__ATTR(batt_debug_flag, S_IWUSR | S_IWGRP, NULL,
		htc_battery_debug_flag),
	__ATTR(charger_control, S_IWUSR | S_IWGRP, htc_battery_charger_stat,
		htc_battery_charger_switch),
	__ATTR(charger_timer, S_IWUSR | S_IWGRP, NULL,
		htc_battery_charger_ctrl_timer),
	__ATTR(phone_call, S_IWUSR | S_IWGRP, htc_battery_phone_call_stat,
		htc_battery_phone_call_switch),
	LIMIT_CHARGING_FILENODE_ATTR(navigation),
	LIMIT_CHARGING_FILENODE_ATTR(medialink),
	LIMIT_CHARGING_FILENODE_ATTR(data_encryption),
};


static struct device_attribute htc_battery_rt_attrs[] = {
	__ATTR(batt_vol_now, S_IRUGO, htc_battery_rt_attr_show, NULL),
	__ATTR(batt_current_now, S_IRUGO, htc_battery_rt_attr_show, NULL),
	__ATTR(batt_temp_now, S_IRUGO, htc_battery_rt_attr_show, NULL),
};

static int htc_battery_create_attrs(struct device *dev)
{
	int i = 0, j = 0, k = 0, rc = 0;

	for (i = 0; i < ARRAY_SIZE(htc_battery_attrs); i++) {
		rc = device_create_file(dev, &htc_battery_attrs[i]);
		if (rc)
			goto htc_attrs_failed;
	}

	for (j = 0; j < ARRAY_SIZE(htc_set_delta_attrs); j++) {
		rc = device_create_file(dev, &htc_set_delta_attrs[j]);
		if (rc)
			goto htc_delta_attrs_failed;
	}

	for (k = 0; k < ARRAY_SIZE(htc_battery_rt_attrs); k++) {
		rc = device_create_file(dev, &htc_battery_rt_attrs[k]);
		if (rc)
			goto htc_rt_attrs_failed;
	}

	goto succeed;

htc_rt_attrs_failed:
	while (k--)
		device_remove_file(dev, &htc_battery_rt_attrs[k]);
htc_delta_attrs_failed:
	while (j--)
		device_remove_file(dev, &htc_set_delta_attrs[j]);
htc_attrs_failed:
	while (i--)
		device_remove_file(dev, &htc_battery_attrs[i]);
succeed:
	return rc;
}

static int htc_battery_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = htc_battery_get_charging_status();
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		/* Fix me: temperature criteria should depend on projects,
			   but not hard code. */
		val->intval = POWER_SUPPLY_HEALTH_GOOD;
		if (battery_core_info.rep.temp_fault != -1) {
			if (battery_core_info.rep.temp_fault == 1)
				val->intval =  POWER_SUPPLY_HEALTH_OVERHEAT;
		}
		else if (battery_core_info.rep.batt_temp >= 480 ||
			battery_core_info.rep.batt_temp <= 0)
			val->intval =  POWER_SUPPLY_HEALTH_OVERHEAT;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = battery_core_info.present;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		mutex_lock(&battery_core_info.info_lock);
		val->intval = battery_core_info.rep.level;
		mutex_unlock(&battery_core_info.info_lock);
		break;
	case POWER_SUPPLY_PROP_OVERLOAD:
		mutex_lock(&battery_core_info.info_lock);
		val->intval = battery_core_info.rep.overload;
		mutex_unlock(&battery_core_info.info_lock);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int htc_power_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	enum charger_type_t charger;

	mutex_lock(&battery_core_info.info_lock);

	charger = battery_core_info.rep.charging_source;

#if 0
	if (battery_core_info.rep.batt_id == 255)
		charger = CHARGER_BATTERY;
#endif

	mutex_unlock(&battery_core_info.info_lock);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if (psy->type == POWER_SUPPLY_TYPE_MAINS) {
			if (charger == CHARGER_AC ||
				charger == CHARGER_2A_AC ||
			    charger == CHARGER_9V_AC)
				val->intval = 1;
			else
				val->intval = 0;
		} else if (psy->type == POWER_SUPPLY_TYPE_USB)
			val->intval = (charger ==  CHARGER_USB ? 1 : 0);
		else if (psy->type == POWER_SUPPLY_TYPE_WIRELESS)
			val->intval = (charger ==  CHARGER_WIRELESS ? 1 : 0);
		else
			val->intval = 0;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static ssize_t htc_battery_show_property(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	int i = 0;
	const ptrdiff_t off = attr - htc_battery_attrs;

	mutex_lock(&battery_core_info.info_lock);

	switch (off) {
	case BATT_ID:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
				battery_core_info.rep.batt_id);
		break;
	case BATT_VOL:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
				battery_core_info.rep.batt_vol);
		break;
	case BATT_TEMP:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
				battery_core_info.rep.batt_temp);
		break;
	case BATT_CURRENT:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
				battery_core_info.rep.batt_current -
				battery_core_info.rep.batt_discharg_current);
		break;
	case CHARGING_SOURCE:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
				battery_core_info.rep.charging_source);
		break;
	case CHARGING_ENABLED:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
				battery_core_info.rep.charging_enabled);
		break;
	case FULL_BAT:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
				battery_core_info.rep.full_bat);
		break;
	case OVER_VCHG:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
				battery_core_info.rep.over_vchg);
		break;
	case BATT_STATE:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
				battery_core_info.rep.batt_state);
		break;
	default:
		i = -EINVAL;
	}
	mutex_unlock(&battery_core_info.info_lock);

	if (i < 0)
		BATT_ERR("%s: battery: attribute is not supported: %d",
			__func__, off);

	return i;
}

static ssize_t htc_battery_rt_attr_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	int i = 0;
	int val = 0;
	int rc = 0;
	const ptrdiff_t attr_index = attr - htc_battery_rt_attrs;

	if (!battery_core_info.func.func_get_batt_rt_attr) {
		BATT_ERR("%s: func_get_batt_rt_attr does not exist", __func__);
		return -EINVAL;
	}

	rc = battery_core_info.func.func_get_batt_rt_attr(attr_index, &val);
	if (rc) {
		BATT_ERR("%s: get_batt_rt_attrs[%d] failed", __func__, attr_index);
		return -EINVAL;
	}

	i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", val);
	return i;
}

int htc_battery_charger_hiz(void)
{
	int rc = 0;

	rc = battery_core_info.func.func_charger_control(
			ENABLE_HIZ);
	if (rc) {
		BATT_LOG("enable hiz fail!");
	}

	return rc;
}

static ssize_t htc_battery_charger_ctrl_timer(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	int rc;
	unsigned long time_out = 0;
	ktime_t interval;
	ktime_t next_alarm;

	rc = strict_strtoul(buf, 10, &time_out);
	if (rc)
		return rc;

	if (time_out > 65536)
		return -EINVAL;

	if (time_out > 0) {
		rc = battery_core_info.func.func_charger_control(STOP_CHARGER);
		if (rc < 0) {
			BATT_ERR("charger control failed!");
			return rc;
		}
		interval = ktime_set(time_out, 0);
		next_alarm = ktime_add(alarm_get_elapsed_realtime(), interval);
		alarm_start_range(&batt_charger_ctrl_alarm,
					next_alarm, next_alarm);
		charger_ctrl_stat = STOP_CHARGER;
	} else if (time_out == 0) {
		rc = battery_core_info.func.func_charger_control(
							ENABLE_CHARGER);
		if (rc < 0) {
			BATT_ERR("charger control failed!");
			return rc;
		}
		alarm_cancel(&batt_charger_ctrl_alarm);
		charger_ctrl_stat = ENABLE_CHARGER;
	}

	return count;
}

static void batt_charger_ctrl_func(struct work_struct *work)
{
	int rc;

	rc = battery_core_info.func.func_charger_control(ENABLE_CHARGER);
	if (rc) {
		BATT_ERR("charger control failed!");
		return;
	}

	charger_ctrl_stat = (unsigned int)ENABLE_CHARGER;
}

static void batt_charger_ctrl_alarm_handler(struct alarm *alarm)
{
	BATT_LOG("charger control alarm is timeout.");

	queue_work(batt_charger_ctrl_wq, &batt_charger_ctrl_work);
}

int htc_battery_core_update(void)
{
	struct battery_info_reply new_batt_info_rep;
	int is_send_batt_uevent = 0;
	int is_send_usb_uevent = 0;
	int is_send_ac_uevent = 0;

	if (battery_register) {
		BATT_ERR("No battery driver exists.");
		return -1;
	}

	mutex_lock(&battery_core_info.info_lock);
	memcpy(&new_batt_info_rep, &battery_core_info.rep, sizeof(struct battery_info_reply));
	mutex_unlock(&battery_core_info.info_lock);
	if (battery_core_info.func.func_get_battery_info) {
		battery_core_info.func.func_get_battery_info(&new_batt_info_rep);
	} else {
		BATT_ERR("no func_get_battery_info hooked.");
		return -EINVAL;
	}

	mutex_lock(&battery_core_info.info_lock);
	if (battery_core_info.rep.charging_source != new_batt_info_rep.charging_source) {
		if (CHARGER_BATTERY == battery_core_info.rep.charging_source ||
			CHARGER_BATTERY == new_batt_info_rep.charging_source)
			is_send_batt_uevent = 1;
		if (CHARGER_USB == battery_core_info.rep.charging_source ||
			CHARGER_USB == new_batt_info_rep.charging_source)
			is_send_usb_uevent = 1;
		if (CHARGER_AC <= battery_core_info.rep.charging_source ||
			CHARGER_AC <= new_batt_info_rep.charging_source)
			is_send_ac_uevent = 1;
	}
	if ((!is_send_batt_uevent) &&
		((battery_core_info.rep.level != new_batt_info_rep.level) ||
		(battery_core_info.rep.batt_vol != new_batt_info_rep.batt_vol) ||
		(battery_core_info.rep.over_vchg != new_batt_info_rep.over_vchg) ||
		(battery_core_info.rep.batt_temp != new_batt_info_rep.batt_temp))) {
		is_send_batt_uevent = 1;
	}

#if 0	/* note: old mechanism */
	/* To make sure that device is under over loading scenario, accumulate
	   variable battery_over_loading only when device has been under charging
	   and level is decreased. */
	if ((battery_core_info.rep.charging_enabled != 0) &&
		(new_batt_info_rep.charging_enabled != 0)) {
		if (battery_core_info.rep.level > new_batt_info_rep.level)
			battery_over_loading++;
		else
			battery_over_loading = 0;
	}
#endif

	memcpy(&battery_core_info.rep, &new_batt_info_rep, sizeof(struct battery_info_reply));
#if 0
	battery_core_info.rep.batt_vol = new_batt_info_rep.batt_vol;
	battery_core_info.rep.batt_id = new_batt_info_rep.batt_id;
	battery_core_info.rep.batt_temp = new_batt_info_rep.batt_temp;
	battery_core_info.rep.batt_current = new_batt_info_rep.batt_current;
	battery_core_info.rep.batt_discharg_current = new_batt_info_rep.batt_discharg_current;
	battery_core_info.rep.level = new_batt_info_rep.level;
	battery_core_info.rep.charging_source = new_batt_info_rep.charging_source;
	battery_core_info.rep.charging_enabled = new_batt_info_rep.charging_enabled;
	battery_core_info.rep.full_bat = new_batt_info_rep.full_bat;
	battery_core_info.rep.over_vchg = new_batt_info_rep.over_vchg;
	battery_core_info.rep.temp_fault = new_batt_info_rep.temp_fault;
	battery_core_info.rep.batt_state = new_batt_info_rep.batt_state;
#endif

	if (battery_core_info.rep.charging_source == CHARGER_BATTERY)
		battery_core_info.htc_charge_full = 0;
	else {
		if (battery_core_info.htc_charge_full &&
			(battery_core_info.rep.full_level == 100)) {
			if (battery_core_info.rep.level > 98) {
				battery_core_info.htc_charge_full = 1;
				battery_core_info.rep.level = 100;
			} else
				battery_core_info.htc_charge_full = 0;
		} else {
			if (battery_core_info.rep.level == 100)
				battery_core_info.htc_charge_full = 1;
			else
				battery_core_info.htc_charge_full = 0;
		}

#if 0	/* note: old mechanism */
		/* Clear htc_charge_full while over loading is happened. */
		if (battery_over_loading >= 2) {
			battery_core_info.htc_charge_full = 0;
			battery_over_loading = 0;
		}
#endif
	}

	battery_core_info.update_time = jiffies;
	mutex_unlock(&battery_core_info.info_lock);

	BATT_LOG("capacity=%d%%, ID=%d, vol=%d, temp=%d, curr=%d, "
		"dis_curr=%d, chg_src=%d, chg_en=%d, over_vchg=%d, batt_state=%d, batt_ol=%d",
			battery_core_info.rep.level,
			battery_core_info.rep.batt_id,
			battery_core_info.rep.batt_vol,
			battery_core_info.rep.batt_temp,
			battery_core_info.rep.batt_current,
			battery_core_info.rep.batt_discharg_current,
			battery_core_info.rep.charging_source,
			battery_core_info.rep.charging_enabled,
			battery_core_info.rep.over_vchg,
			battery_core_info.rep.batt_state,
			battery_core_info.rep.overload);

	/* send uevent if need */
	if (is_send_batt_uevent) {
		power_supply_changed(&htc_power_supplies[BATTERY_SUPPLY]);
		BATT_LOG("power_supply_changed: battery");
	}
	if (is_send_usb_uevent) {
		power_supply_changed(&htc_power_supplies[USB_SUPPLY]);
		BATT_LOG("power_supply_changed: usb");
	}
	if (is_send_ac_uevent) {
		power_supply_changed(&htc_power_supplies[AC_SUPPLY]);
		BATT_LOG("power_supply_changed: ac");
	}

	return 0;
}
EXPORT_SYMBOL_GPL(htc_battery_core_update);

int htc_battery_is_charging_full(void)
{
	return battery_core_info.htc_charge_full;
}
EXPORT_SYMBOL_GPL(htc_battery_is_charging_full);

int htc_battery_core_register(struct device *dev,
				struct htc_battery_core *htc_battery)
{
	int i, rc = 0;

	if (!battery_register) {
		BATT_ERR("Only one battery driver could exist.");
		return -1;
	}
	battery_register = 0;

	mutex_init(&battery_core_info.info_lock);

	if (htc_battery->func_get_batt_rt_attr)
		battery_core_info.func.func_get_batt_rt_attr =
					htc_battery->func_get_batt_rt_attr;
	if (htc_battery->func_show_batt_attr)
		battery_core_info.func.func_show_batt_attr =
					htc_battery->func_show_batt_attr;
	if (htc_battery->func_show_batt_power_meter)
		battery_core_info.func.func_show_batt_power_meter =
					htc_battery->func_show_batt_power_meter;
	if (htc_battery->func_show_htc_extension_attr)
		battery_core_info.func.func_show_htc_extension_attr =
					htc_battery->func_show_htc_extension_attr;
	if (htc_battery->func_get_battery_info)
		battery_core_info.func.func_get_battery_info =
					htc_battery->func_get_battery_info;
	if (htc_battery->func_charger_control)
		battery_core_info.func.func_charger_control =
					htc_battery->func_charger_control;

	if (htc_battery->func_set_full_level)
		battery_core_info.func.func_set_full_level =
					htc_battery->func_set_full_level;

	if (htc_battery->func_phone_call_notification)
		battery_core_info.func.func_phone_call_notification =
					htc_battery->func_phone_call_notification;

	if (htc_battery->func_limit_charging_notification)
		battery_core_info.func.func_limit_charging_notification =
					htc_battery->func_limit_charging_notification;

	/* init power supplier framework */
	for (i = 0; i < ARRAY_SIZE(htc_power_supplies); i++) {
		rc = power_supply_register(dev, &htc_power_supplies[i]);
		if (rc)
			BATT_ERR("Failed to register power supply"
				" (%d)\n", rc);
	}

	/* create htc detail attributes */
	htc_battery_create_attrs(htc_power_supplies[CHARGER_BATTERY].dev);

	/* init charger_ctrl_timer */
	charger_ctrl_stat = ENABLE_CHARGER;
	INIT_WORK(&batt_charger_ctrl_work, batt_charger_ctrl_func);
	alarm_init(&batt_charger_ctrl_alarm,
			ANDROID_ALARM_ELAPSED_REALTIME_WAKEUP,
			batt_charger_ctrl_alarm_handler);
	batt_charger_ctrl_wq =
			create_singlethread_workqueue("charger_ctrl_timer");

	/* init battery parameters. */
	battery_core_info.update_time = jiffies;
	battery_core_info.present = 1;
	battery_core_info.htc_charge_full = 0;
	battery_core_info.rep.charging_source = CHARGER_BATTERY;
	battery_core_info.rep.batt_id = 1;
	battery_core_info.rep.batt_vol = 4000;
	battery_core_info.rep.batt_temp = 285;
	battery_core_info.rep.batt_current = 162000;
	battery_core_info.rep.level = 66;
	battery_core_info.rep.full_bat = 1580000;
	battery_core_info.rep.full_level = 100;
	/* initial state = -1, valid values: 0 or 1 */
	battery_core_info.rep.temp_fault = -1;
	/* zero means battey info is not ready */
	battery_core_info.rep.batt_state = 0;

#if 0	/* note: old mechanism */
	battery_over_loading = 0;
#endif

	return 0;
}
EXPORT_SYMBOL_GPL(htc_battery_core_register);
