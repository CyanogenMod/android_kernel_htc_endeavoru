/* arch/arm/mach-msm/htc_battery_tps80032.c
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
#include <linux/platform_device.h>
#include <linux/debugfs.h>
#include <linux/wakelock.h>
#include <linux/gpio.h>
#include <mach/htc_battery_core.h>
#include <mach/htc_battery_tps80032.h>
#include <asm/mach-types.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/tps65200.h>
#include <linux/tps80032_adc.h>
#include <linux/tps80032_charger.h>
#include <linux/tps80032_vsys_alarm.h>
#include <linux/reboot.h>
#include <linux/miscdevice.h>
#include <linux/android_alarm.h>
#include <linux/suspend.h>
#include <linux/time.h>
#include <mach/cable_detect.h>
#include <linux/delay.h>
#include <mach/board_htc.h>
#include <linux/earlysuspend.h>
#include <linux/platform_data/ina230.h>
#include <mach/restart.h>
#include <linux/atomic.h>

/* used for debug if function called */
#define FUNC_CALL_CHECK 0
#define WK_MBAT_IN 0	// for enrc2u bringup
#define WK_ALARM_NOT_WORK 1
#define WRITE_PWR_SAVE_DISABLE	0x1000
#define ALL_AC_CHARGING		0x4000000
#define SET_FAKE_FULL		0x4
#define SET_FAKE_TEMP		0x4000
#define SET_FAKE_CAPACITY	0x8000

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

#ifdef PWR_DEVICE_TAG
#undef PWR_DEVICE_TAG
#endif
#define PWR_DEVICE_TAG "CHAR"

#if FUNC_CALL_CHECK
#define CHECK_LOG() BATT_LOG("%s", __func__)
#else
#define CHECK_LOG() (void)0
#endif

#define BATT_SUSPEND_CHECK_TIME			3600
#define BATT_SUSPEND_PHONE_CALL_CHECK_TIME	300
#define BATT_LATE_RESUME_CHECK_TIME		300
#define BATT_TIMER_CHECK_TIME			180
#define FIRST_ADC_READ_DELAY			5
#define TEMP_ADC_READ_RETRY			5

#define BATT_VOLT_CHANNEL	0
#define BATT_TEMP_CHANNEL	1
#define BATT_ID_CHANNEL		3
#define BATT_CURR_CHANNEL	5
#define VBUS_VOLT_CHANNEL	10

#define VOLT_ALARM_CHECK	3

#define	ADC_VBUS_BITS_TO_MV(adc) ((adc) * 27250 / 4095)
#define MV_TO_ADC_BITS(batt_vol) ((batt_vol) * 4095 / 1250)

#define QB_LPB_SHUTDOWN_VOLTAGE 3000

#define PRE_LED_USB_V	(1U << 2)
#define PRE_LED_AC_V	(1U << 1)
#define PRE_LED_100P	(1U << 0)

#define CHG_TPS80032_OFFMODE_USB_THRESHOLD	3200
#define CHG_TPS80032_OFFMODE_AC_THRESHOLD	3500

#define DEFAULT_OFFMODE_USB_THRESHOLD	3400
#define DEFAULT_OFFMODE_AC_THRESHOLD	3600

enum {
	ATTR_REBOOT_LEVEL = 0,
	ATTR_REBOOT_IS_CHARGING_FULL,
	ATTR_HBOOT_VOLT,
	ATTR_HBOOT_CURR,
	ATTR_HBOOT_TEMP,
	ATTR_QB_LOW_POWER,
	ATTR_QB_ENABLED,
	ATTR_FAKE_TEMP,
	ATTR_SUSPEND_1_PERCENT,
};

#if WK_MBAT_IN
static int is_mbat_in;
#endif

static void no_batt_func(struct work_struct *work);
static DECLARE_DELAYED_WORK(no_batt_struct, no_batt_func);
static struct kset *htc_batt_kset;

static void reverse_current_func(struct work_struct *work);
static DECLARE_DELAYED_WORK(reverse_current_struct, reverse_current_func);

static int htc_batt_phone_call = 0;
static unsigned int htc_batt_limit_charging = 0;
struct mutex htc_batt_limit_charging_lock;
#if WK_ALARM_NOT_WORK	/* fixme: no use this workaround now since solution is phased in */
static int is_alarm_not_work = 0;
#endif

static unsigned reboot_battery_value = BATTERY_LEVEL_NO_VALUE;

struct htc_battery_info {
	int device_id;

	/* lock to protect the battery info */
	struct mutex info_lock;

	spinlock_t batt_lock;
	int is_open;

	struct kobject batt_timer_kobj;
	struct kobject batt_cable_kobj;

	struct wake_lock vbus_wake_lock;
	struct wake_lock alarm_wake_lock;
	char debug_log[DEBUG_LOG_LENGTH];
	char power_meter[POWER_METER_LENGTH];

	struct battery_info_reply rep;

	struct battery_adc_reply adc_data;
	int adc_vref[ADC_REPLY_ARRAY_SIZE];

	int guage_driver;
	int charger;
	int vzero_clb_channel;
	int volt_adc_offset;
	int power_off_by_id;
	int check2_value;
	int first_level_ready;
	int online;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
	unsigned int charge_done;
	int sw_temp_25;
	int fake_temp_state;
	unsigned int ac_8hour_count;
	unsigned int pre_led_change;
	int mfg_mode;
	unsigned int over_vchg;
};
static struct htc_battery_info htc_batt_info;

struct htc_battery_timer {
	unsigned long batt_system_jiffies;
	unsigned long batt_suspend_ms;
	unsigned long total_time_ms;
	atomic_t batt_alarm_status;
	int alarm_trigger_volt;
	int batt_alarm_enabled;
	int batt_alarm_threshold;
	unsigned int alarm_timer_flag;
	unsigned int charger_flag;
	unsigned int time_out;
	struct work_struct batt_work;
	struct work_struct batt_first_work;
	struct work_struct alarm_check_work;
	struct alarm batt_check_wakeup_alarm;
	struct timer_list batt_timer;
	struct timer_list batt_first_timer;
	struct workqueue_struct *batt_wq;
	struct wake_lock battery_lock;
	struct wake_lock led_change_lock;
};
static struct htc_battery_timer htc_batt_timer;

static void usb_status_notifier_func(int online);
static struct t_usb_status_notifier usb_status_notifier = {
	.name = "htc_battery_tps80032",
	.func = usb_status_notifier_func,
};

static int htc_battery_initial;
static int htc_full_level_flag;
static int htc_battery_set_charging(int ctl);

static void tps_int_notifier_func(int int_reg, int value);
static struct tps65200_chg_int_notifier tps_int_notifier = {
	.name = "htc_battery_tps80032",
	.func = tps_int_notifier_func,
};

static void tps80032_int_notifier_func(int int_reg, int value);
static struct tps80032_charger_int_notifier tps80032_int_notifier = {
	.name = "htc_battery_tps80032",
	.func = tps80032_int_notifier_func,
};

static ssize_t tps80032_batt_show_attributes(struct device *dev,
					struct device_attribute *attr,
					char *buf);
static int tps80032_qb_store_attributes(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);

static int tps80032_fake_temp_store_attributes(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);
static unsigned long hboot_batt_volt = TPS80032_GPADC_FAKE_VALUE;
int __init batt_voltage_get(char *s)
{
	int ret = strict_strtoul(s, 10, &hboot_batt_volt);
	if (ret < 0)
		return -EINVAL;
	return 0;
}
__setup("batt_V=", batt_voltage_get);

static unsigned long hboot_batt_curr = TPS80032_GPADC_FAKE_VALUE;
int __init batt_current_get(char *s)
{
	int ret = strict_strtoul(s, 10, &hboot_batt_curr);
	if (ret < 0)
		return -EINVAL;
	return 0;
}
__setup("batt_I=", batt_current_get);

static unsigned long hboot_batt_temp = TPS80032_GPADC_FAKE_VALUE;
int __init batt_temperature_get(char *s)
{
	int ret = strict_strtoul(s, 10, &hboot_batt_temp);
	if (ret < 0)
		return -EINVAL;
	return 0;
}
__setup("batt_T=", batt_temperature_get);

static void tps_int_notifier_func(int int_reg, int value)
{
	CHECK_LOG();

	if (int_reg == CHECK_INT1) {
		htc_batt_info.rep.over_vchg = (unsigned int)value;
		htc_battery_core_update();
	} else if (int_reg == CHECK_INT2) {
		htc_batt_info.check2_value = value;
		schedule_delayed_work(&reverse_current_struct
					, msecs_to_jiffies(5000));
	}
}

static void watchdog_timeout_func(void)
{
	char message[16];
	char *envp[] = { message, NULL };

	CHECK_LOG();

	scnprintf(message, 16, "WDG_TIMEOUT");

	kobject_uevent_env(&htc_batt_info.batt_cable_kobj, KOBJ_CHANGE, envp);
}

static void tps80032_int_notifier_func(int int_reg, int value)
{
	CHECK_LOG();

	switch (int_reg) {
	case CHECK_INT1:
		mutex_lock(&htc_batt_info.info_lock);
		htc_batt_info.over_vchg = (unsigned int)value;
		mutex_unlock(&htc_batt_info.info_lock);
		htc_batt_info.rep.over_vchg = (unsigned int)value;
		if (value != 0)
			htc_batt_info.rep.charging_enabled = 0;
		htc_battery_core_update();
		break;
	case CHG_CHARGE_DONE:
		mutex_lock(&htc_batt_info.info_lock);
		htc_batt_info.charge_done = 1;
		mutex_unlock(&htc_batt_info.info_lock);

		if (!!htc_batt_info.rep.batt_state) {
			del_timer_sync(&htc_batt_timer.batt_timer);
			cancel_work_sync(&htc_batt_timer.batt_work);
			wake_lock(&htc_batt_timer.battery_lock);
			queue_work(htc_batt_timer.batt_wq, &htc_batt_timer.batt_work);
		} else
			BATT_LOG("Battery not ready, skip charge done interrupt");
		break;
	case CHECK_WATCHDOG:
		if (htc_batt_info.rep.charging_source == CHARGER_USB) {
			wake_lock(&htc_batt_info.vbus_wake_lock);
			if (!!(get_kernel_flag() & ALL_AC_CHARGING))
				tps80032_charger_set_ctrl(POWER_SUPPLY_ENABLE_FAST_CHARGE);
			else
				tps80032_charger_set_ctrl(POWER_SUPPLY_ENABLE_SLOW_CHARGE);
			wake_unlock(&htc_batt_info.vbus_wake_lock);
		} else if (!!(get_kernel_flag() & (SET_FAKE_FULL | SET_FAKE_TEMP)) || htc_batt_info.ac_8hour_count < 1) {
			if (htc_batt_info.rep.charging_source == CHARGER_AC) {
				wake_lock(&htc_batt_info.vbus_wake_lock);
				tps80032_charger_set_ctrl(POWER_SUPPLY_ENABLE_FAST_CHARGE);
				htc_batt_info.ac_8hour_count++;
				wake_unlock(&htc_batt_info.vbus_wake_lock);
			}
		} else
			watchdog_timeout_func();
		break;
	default:
		BATT_LOG("NO support such charger interrupt handling %d, %d", int_reg, value);
	}

}

static int batt_alarm_config(unsigned long threshold)
{
	int rc = 0;

	CHECK_LOG();

	rc = tps80032_vsys_alarm_state_set(0);
	if (rc) {
		BATT_ERR("state_set disabled failed, rc=%d", rc);
		goto done;
	}

	rc = tps80032_vsys_alarm_threshold_set(threshold);
	if (rc) {
		BATT_ERR("threshold_set failed, rc=%d!", rc);
		goto done;
	}

done:
	return rc;
}

static void alarm_check_work_func(struct work_struct *work)
{
	int sum = 0, i = 0;
	int rc;
	int voltage[VOLT_ALARM_CHECK];

	CHECK_LOG();

	wake_lock(&htc_batt_info.alarm_wake_lock);
	rc = tps80032_vsys_alarm_state_set(0);
	if (rc)
		BATT_ERR("%s:state_set disabled failed, rc=%d!",__func__, rc);

	if (htc_batt_info.guage_driver == GUAGE_TPS80032) {
		ina230_get_bus_voltage_multi(voltage, VOLT_ALARM_CHECK);
		do {
			sum += voltage[i];
		} while (++i < VOLT_ALARM_CHECK);

		if (sum >= htc_batt_timer.batt_alarm_threshold * VOLT_ALARM_CHECK) {
			atomic_set(&htc_batt_timer.batt_alarm_status, 0);
			htc_batt_timer.alarm_trigger_volt = 0;
		} else {
			htc_batt_timer.alarm_trigger_volt = htc_batt_timer.batt_alarm_threshold;
			BATT_LOG("%s: threshold meet, voltage alarm triggerred (%d, %d)"
					, __func__, htc_batt_timer.alarm_trigger_volt, htc_batt_timer.batt_alarm_threshold);
			del_timer_sync(&htc_batt_timer.batt_timer);
			cancel_work_sync(&htc_batt_timer.batt_work);
			wake_lock(&htc_batt_timer.battery_lock);
			queue_work(htc_batt_timer.batt_wq, &htc_batt_timer.batt_work);
		}
	} else {
		BATT_LOG("%s: voltage alarm triggerred", __func__);

		del_timer_sync(&htc_batt_timer.batt_timer);
		cancel_work_sync(&htc_batt_timer.batt_work);
		wake_lock(&htc_batt_timer.battery_lock);
		queue_work(htc_batt_timer.batt_wq, &htc_batt_timer.batt_work);
	}

	if (!!htc_batt_timer.batt_alarm_enabled) {
		rc = tps80032_vsys_alarm_state_set(1);
		if (rc)
			BATT_ERR("%s:state_set enabled failed, rc=%d!", __func__, rc);
	}
	wake_unlock(&htc_batt_info.alarm_wake_lock);
}

static int battery_alarm_notifier_func(struct notifier_block *nfb,
					unsigned long value, void *data);
static struct notifier_block battery_alarm_notifier = {
	.notifier_call = battery_alarm_notifier_func,
};

static int battery_alarm_notifier_func(struct notifier_block *nfb,
		unsigned long status, void *data)
{
	int alarm_status;

	CHECK_LOG();

	alarm_status = atomic_inc_return(&htc_batt_timer.batt_alarm_status);

	if (alarm_status == 1)
		queue_work(htc_batt_timer.batt_wq, &htc_batt_timer.alarm_check_work);

	return 0;
}


static void update_wake_lock(int status)
{
	CHECK_LOG();

	if (status == CHARGER_USB && htc_batt_info.mfg_mode != BOARD_MFG_MODE_OFFMODE_CHARGING)
		wake_lock(&htc_batt_info.vbus_wake_lock);
	else
		/* give userspace some time to see the uevent and update
		   LED state or whatnot...*/
		wake_lock_timeout(&htc_batt_info.vbus_wake_lock, HZ * 5);
}

static void reverse_current_func(struct work_struct *work)
{
	char message[16];
	char *envp[] = { message, NULL };

	CHECK_LOG();

	scnprintf(message, 16, "REVERSE_CURR=%d",
					htc_batt_info.check2_value);

	update_wake_lock(htc_batt_info.rep.charging_source);

	kobject_uevent_env(&htc_batt_info.batt_cable_kobj, KOBJ_CHANGE, envp);
}

static void usb_status_notifier_func(int online)
{
	char message[16];
	char *envp[] = { message, NULL };

	CHECK_LOG();

	if (online == htc_batt_info.online) {
		BATT_LOG("%s: charger type (%u) same return.",
			__func__, online);
		return;
	}

	mutex_lock(&htc_batt_info.info_lock);

	htc_batt_info.online = online;
	htc_batt_info.ac_8hour_count = 0;
	switch (online) {
	case CONNECT_TYPE_USB:
		BATT_LOG("cable USB");
		if ( !!(get_kernel_flag() & ALL_AC_CHARGING) ) {
			BATT_LOG("Debug flag is set to force AC charging, fake as AC");
			htc_batt_info.rep.charging_source = CHARGER_AC;
		} else
			htc_batt_info.rep.charging_source = CHARGER_USB;
		break;
	case CONNECT_TYPE_AC:
		BATT_LOG("cable AC");
		htc_batt_info.rep.charging_source = CHARGER_AC;
		break;
	case CONNECT_TYPE_UNKNOWN:
		BATT_ERR("unknown cable");
		htc_batt_info.rep.charging_source = CHARGER_USB;
		break;
	case CONNECT_TYPE_INTERNAL:
		BATT_LOG("delivers power to VBUS from battery");
		htc_battery_set_charging(POWER_SUPPLY_ENABLE_INTERNAL);
		mutex_unlock(&htc_batt_info.info_lock);
		return;
	case CONNECT_TYPE_NONE:
		if ( !!(get_kernel_flag() & WRITE_PWR_SAVE_DISABLE && !(get_kernel_flag() & ALL_AC_CHARGING)) ) {
			mutex_unlock(&htc_batt_info.info_lock);
			kernel_power_off();
			break;
		}
	/* if not writing kernel flag for usb plug-out , handled by default */
	default:
		BATT_LOG("No cable exists");
		htc_batt_info.rep.charging_source = CHARGER_BATTERY;
		break;
	}
#if WK_ALARM_NOT_WORK	/* fixme: no use this workaround now since solution is phased in */
	is_alarm_not_work = 0;
#endif
	htc_batt_timer.charger_flag =
			(unsigned int)htc_batt_info.rep.charging_source;

	scnprintf(message, 16, "CHG_SOURCE=%d",
					htc_batt_info.rep.charging_source);

	update_wake_lock(htc_batt_info.rep.charging_source);
	mutex_unlock(&htc_batt_info.info_lock);

	kobject_uevent_env(&htc_batt_info.batt_cable_kobj, KOBJ_CHANGE, envp);
}

static int htc_battery_set_charging(int ctl)
{
	int rc = 0;

	CHECK_LOG();

	if (htc_batt_info.charger == SWITCH_CHARGER_TPS65200)
		rc = tps_set_charger_ctrl(ctl);
	else if (htc_batt_info.charger == SWITCH_CHARGER_TPS80032)
		rc = tps80032_charger_set_ctrl(ctl);

	return rc;
}

static int htc_batt_charger_control(enum charger_control_flag control)
{
	char message[16] = "CHARGERSWITCH=";
	char *envp[] = { message, NULL };

	CHECK_LOG();

	BATT_LOG("%s: switch charger to mode: %u", __func__, control);

	switch (control) {
	case STOP_CHARGER:
		strncat(message, "0", 1);
		break;
	case ENABLE_CHARGER:
		strncat(message, "1", 1);
		break;
	case ENABLE_LIMIT_CHARGER:
		htc_battery_set_charging(ENABLE_LIMITED_CHG);
		goto done;
	case DISABLE_LIMIT_CHARGER:
		htc_battery_set_charging(CLEAR_LIMITED_CHG);
		goto done;
	case ENABLE_HIZ:
		if (htc_batt_info.charger == SWITCH_CHARGER_TPS80032) {
			htc_battery_set_charging(ENABLE_HIZ_CHG);
			goto done;
		} else
			strncat(message, "0", 1);
		break;
	default:
		return -1;
	};

	kobject_uevent_env(&htc_batt_info.batt_cable_kobj, KOBJ_CHANGE, envp);
done:
	return 0;
}

static void htc_batt_phone_call_notification(int phone_call)
{
	CHECK_LOG();

	BATT_LOG("%s:Phone call notified with value %d", __func__, phone_call);
	htc_batt_phone_call = phone_call;
}

static void htc_batt_limit_charging_notification(unsigned int type, int enable)
{
	CHECK_LOG();

	BATT_LOG("%s:Limit charging notified with type %d and enable %d", __func__, type, enable);
	mutex_lock(&htc_batt_limit_charging_lock);
	if (!enable)
		htc_batt_limit_charging  &= ~type;
	else
		htc_batt_limit_charging  |= type;

	if (htc_batt_info.charger == SWITCH_CHARGER_TPS80032) {
		if (!htc_batt_limit_charging)
			htc_battery_set_charging(CLEAR_LIMITED_CHG);
		else
			htc_battery_set_charging(ENABLE_LIMITED_CHG);
	}
	mutex_unlock(&htc_batt_limit_charging_lock);
}

static void htc_batt_set_full_level(int percent)
{
	char message[16];
	char *envp[] = { message, NULL };

	CHECK_LOG();

	BATT_LOG("%s: set full level as %d", __func__, percent);

	scnprintf(message, sizeof(message), "FULL_LEVEL=%d", percent);

	kobject_uevent_env(&htc_batt_info.batt_cable_kobj, KOBJ_CHANGE, envp);

	return;
}

static ssize_t htc_battery_show_batt_attr(struct device_attribute *attr,
					char *buf)
{
	int len = 0;

	CHECK_LOG();

	len += scnprintf(buf + len, PAGE_SIZE - len,
			"%s", htc_batt_info.debug_log);
	return len;
}

static ssize_t htc_battery_show_batt_power_meter(struct device_attribute *attr,
					char *buf)
{
	int len = 0;

	CHECK_LOG();

	len += scnprintf(buf + len, PAGE_SIZE - len,
			"%s", htc_batt_info.power_meter);
	return len;
}

static int htc_batt_open(struct inode *inode, struct file *filp)
{
	int ret = 0;

	CHECK_LOG();

	BATT_LOG("%s: open misc device driver.", __func__);
	spin_lock(&htc_batt_info.batt_lock);

	if (!htc_batt_info.first_level_ready) {
		ret = -EBUSY;
		goto not_ready;
	}

	if (!htc_batt_info.is_open)
		htc_batt_info.is_open = 1;
	else
		ret = -EBUSY;
not_ready:
	spin_unlock(&htc_batt_info.batt_lock);
	return ret;
}

static int htc_batt_release(struct inode *inode, struct file *filp)
{
	CHECK_LOG();

	BATT_LOG("%s: release misc device driver.", __func__);
	spin_lock(&htc_batt_info.batt_lock);
	htc_batt_info.is_open = 0;
	spin_unlock(&htc_batt_info.batt_lock);

	return 0;
}

static int htc_batt_get_battery_info(struct battery_info_reply *htc_batt_update)
{
	CHECK_LOG();

	htc_batt_update->batt_vol = htc_batt_info.rep.batt_vol;
	htc_batt_update->batt_id = htc_batt_info.rep.batt_id;
	htc_batt_update->batt_temp = htc_batt_info.rep.batt_temp;
	htc_batt_update->batt_current = htc_batt_info.rep.batt_current;
	htc_batt_update->batt_discharg_current =
				htc_batt_info.rep.batt_discharg_current;
	htc_batt_update->level = htc_batt_info.rep.level;
	htc_batt_update->charging_source =
				htc_batt_info.rep.charging_source;
	htc_batt_update->charging_enabled =
				htc_batt_info.rep.charging_enabled;
	htc_batt_update->full_bat = htc_batt_info.rep.full_bat;
	htc_batt_update->full_level = htc_batt_info.rep.full_level;
	htc_batt_update->over_vchg = htc_batt_info.rep.over_vchg;
	htc_batt_update->temp_fault = htc_batt_info.rep.temp_fault;
	htc_batt_update->batt_state = htc_batt_info.rep.batt_state;
	htc_batt_update->overload = htc_batt_info.rep.overload;

	return 0;
}

static void batt_set_check_timer(u32 seconds)
{
	CHECK_LOG();

	mod_timer(&htc_batt_timer.batt_timer,
			jiffies + msecs_to_jiffies(seconds * 1000));
}

static int32_t htc_batt_get_battery_adc(void)
{
	int ret = 0;
	struct battery_adc_reply adc;
	unsigned int i, j;
	int temp = 0, temp1 = 0;;

	CHECK_LOG();

	switch (htc_batt_info.guage_driver) {

	case GUAGE_TPS80032:
		memset(&adc, 0, sizeof(struct battery_adc_reply));
		ret = ina230_get_batt_voltage_current(&temp, &temp1);
		if (!ret) {
			memcpy(&adc.adc_voltage[0], &temp, sizeof(int));
			memcpy(&adc.adc_current[0], &temp1, sizeof(int));
		} else
			goto get_adc_failed;
#if WK_MBAT_IN
		if (is_mbat_in) {
#endif
			/* Read battery temperature adc data. */
			ret = tps80032_adc_select_and_read(
					adc.adc_temperature,
					BATT_TEMP_CHANNEL);
			if (ret)
				goto get_adc_failed;

			/* note: retry if battery less than -20 degree or over 68 degree C */
			if (adc.adc_temperature[0] > 2571 || adc.adc_temperature[0] < 990) {
				BATT_LOG("over temperature = %d", adc.adc_temperature[0]);
				/* Re-Read battery temperature adc data. */
				ret = tps80032_adc_select_and_read(
						adc.adc_temperature,
						BATT_TEMP_CHANNEL);
				if (ret)
					goto get_adc_failed;

				for (i = 1; i < TEMP_ADC_READ_RETRY; i++) {
					mdelay(1);
					ret = tps80032_adc_select_and_read(
							&temp,
							BATT_TEMP_CHANNEL);
					if (ret)
						goto get_adc_failed;

					for (j = i; j > 0; j--) {
						if (adc.adc_temperature[j - 1] <= temp)
							break;
						adc.adc_temperature[j] = adc.adc_temperature[j - 1];
					}
					adc.adc_temperature[j] = temp;
				}

				for (i = 0; i < TEMP_ADC_READ_RETRY; i++) {
					BATT_LOG("retry temperature = %d", adc.adc_temperature[i]);
				}

				adc.adc_temperature[0] = adc.adc_temperature[TEMP_ADC_READ_RETRY / 2];
			}

			/* Read battery id adc data. */
			ret = tps80032_adc_select_and_read(
					adc.adc_battid,
					BATT_ID_CHANNEL);
			if (ret)
				goto get_adc_failed;
#if WK_MBAT_IN
		} else {
			adc.adc_temperature[0] = TPS80032_GPADC_FAKE_VALUE;
			adc.adc_battid[0] = TPS80032_GPADC_FAKE_VALUE;
		}
#endif
		break;

	default:
		for (i = 0; i < ADC_REPLY_ARRAY_SIZE; i++) {
			/* Read battery voltage adc data. */
			ret = tps80032_adc_select_and_read(
					adc.adc_voltage + i,
					BATT_VOLT_CHANNEL);
			if (ret)
				goto get_adc_failed;

			/* Read battery current adc data. */
			ret = tps80032_adc_select_and_read(
					adc.adc_current + i,
					BATT_CURR_CHANNEL);
			if (ret)
				goto get_adc_failed;

			/* Read battery vzero clb adc data. */
			if (htc_batt_info.vzero_clb_channel >= 0) {
				ret = tps80032_adc_select_and_read(
						adc.adc_vzero_clb + i,
						htc_batt_info.vzero_clb_channel);
				if (ret)
					goto get_adc_failed;
			} else
				adc.adc_vzero_clb[i] = TPS80032_GPADC_FAKE_VALUE;

			mdelay(1);
		}

		for (i = 0; i < ADC_REPLY_ARRAY_SIZE; i++) {
			/* an adjustment for board no calibratioin */
			temp = ((int) adc.adc_voltage[i]) + htc_batt_info.volt_adc_offset;
			if (temp < 0)
				adc.adc_voltage[i] = 0;
			else if (temp > 4095)
				adc.adc_voltage[i] = 4095;
			else
				adc.adc_voltage[i] = temp;

#if WK_MBAT_IN
			if (is_mbat_in) {
#endif
				/* Read battery temperature adc data. */
				ret = tps80032_adc_select_and_read(
						adc.adc_temperature + i,
						BATT_TEMP_CHANNEL);
				if (ret)
					goto get_adc_failed;
				/* Read battery id adc data. */
				ret = tps80032_adc_select_and_read(
						adc.adc_battid + i,
						BATT_ID_CHANNEL);
				if (ret)
					goto get_adc_failed;
#if WK_MBAT_IN
			} else {
				adc.adc_temperature[i] = TPS80032_GPADC_FAKE_VALUE;
				adc.adc_battid[i] = TPS80032_GPADC_FAKE_VALUE;
			}
#endif
			mdelay(1);
		}
	}

	memcpy(&htc_batt_info.adc_data, &adc,
		sizeof(struct battery_adc_reply));

#if 0	/* for debug adc */
	for (i = 0; i < 1/*ADC_REPLY_ARRAY_SIZE*/; i++) {
		BATT_LOG("index@%u:volt=%u, curr=%u, temp=%u, id=%u"
				, i, adc.adc_voltage[i], adc.adc_current[i]
				, adc.adc_temperature[i], adc.adc_battid[i]);
	}
#endif

get_adc_failed:
	return ret;
}

static void batt_regular_timer_handler(unsigned long data)
{
	CHECK_LOG();

	wake_lock(&htc_batt_timer.battery_lock);
	queue_work(htc_batt_timer.batt_wq, &htc_batt_timer.batt_work);
}

static void batt_first_timer_handler(unsigned long data)
{
	CHECK_LOG();

	queue_work(htc_batt_timer.batt_wq, &htc_batt_timer.batt_first_work);
}

static void batt_check_alarm_handler(struct alarm *alarm)
{
	CHECK_LOG();

	BATT_LOG("alarm handler, but do nothing.");
	return;
}

static void batt_work_func(struct work_struct *work)
{
	int rc = 0;
	int notify_cable_gone = 0;
	int has_vbus;
	int vbus_volt = 0;
	char total_time[32];
	char battery_alarm[16];
	char *envp[] = { total_time, battery_alarm, NULL };

	CHECK_LOG();

	rc = htc_batt_get_battery_adc();
	if (rc)
		BATT_ERR("Read ADC failed!");

	tps80032_adc_select_and_read(&vbus_volt, VBUS_VOLT_CHANNEL);
	/*read tps80032 VBUS_DETECT state*/
	has_vbus = tps80032_read_vbus_detection();
	BATT_LOG("tps80032 VBUS_DETECT:%d OPA_MODE:%d BOOST_HW_PWR:%d VBUS_VOLT:%d", has_vbus,
			tps80032_read_opa_mode(),tps80032_read_boots_hw_pwr(), ADC_VBUS_BITS_TO_MV(vbus_volt));

	if (htc_batt_info.charger == SWITCH_CHARGER_TPS80032)
		tps80032_charger_dump_status(3);
	else
		tps_charger_dump_status();

	htc_batt_timer.total_time_ms += (jiffies -
				htc_batt_timer.batt_system_jiffies) * MSEC_PER_SEC / HZ;

	scnprintf(total_time, sizeof(total_time), "TOTAL_TIME=%lu",
					htc_batt_timer.total_time_ms);

	if (htc_batt_info.charger == SWITCH_CHARGER_TPS80032)
		scnprintf(battery_alarm, sizeof(battery_alarm), "BATT_ALARM=%d",
						htc_batt_timer.alarm_trigger_volt);
	else
		scnprintf(battery_alarm, sizeof(battery_alarm), "BATT_ALARM=%d",
				atomic_read(&htc_batt_timer.batt_alarm_status));

	kobject_uevent_env(&htc_batt_info.batt_timer_kobj, KOBJ_CHANGE, envp);
	htc_batt_timer.total_time_ms = 0;
	htc_batt_timer.batt_system_jiffies = jiffies;
	atomic_set(&htc_batt_timer.batt_alarm_status, 0);
	htc_batt_timer.alarm_trigger_volt = 0;
	batt_set_check_timer(htc_batt_timer.time_out);
	wake_unlock(&htc_batt_timer.battery_lock);

	if (!has_vbus) {
		/* vbus gone, check if need to send no cable event to battery  */
		mutex_lock(&htc_batt_info.info_lock);
		if (htc_batt_info.online != CONNECT_TYPE_NONE && htc_batt_info.online != CONNECT_TYPE_INTERNAL)
			notify_cable_gone = 1;
		mutex_unlock(&htc_batt_info.info_lock);

		if (!!notify_cable_gone) {
			BATT_LOG("VBUS is not present, notify cable gone!!");
			usb_status_notifier_func(CONNECT_TYPE_NONE);
		}
	}

#if WK_ALARM_NOT_WORK	/* fixme: no use this workaround now since solution is phased in */
	if (is_alarm_not_work == 1) {
		/* force wake lock once when charging*/
		mutex_lock(&htc_batt_info.info_lock);
		if (htc_batt_info.rep.charging_source != CHARGER_BATTERY) {
			is_alarm_not_work = 2;
			wake_lock(&htc_batt_info.vbus_wake_lock);
		} else
			is_alarm_not_work = 0;
		mutex_unlock(&htc_batt_info.info_lock);
	}
#endif

	return;
}

static unsigned inline tps80032_batt_get_battery_level(void)
{
	if (htc_batt_info.rep.batt_state != 0)
		return htc_batt_info.rep.level;

	return BATTERY_LEVEL_NO_VALUE;
}

static void tps80032_batt_get_first_reboot_battery_value(void)
{
	if (reboot_battery_value == BATTERY_LEVEL_NO_VALUE) {
		reboot_battery_value = get_last_reboot_params_battery_level();
	}

	printk(KERN_INFO "[BATT]%s:the reboot_battery_value:0x%x\n"
				, __func__, reboot_battery_value);
}

static unsigned int tps80032_batt_get_reboot_battery_level(void)
{
	unsigned signature;
	unsigned value, level;

	signature = (reboot_battery_value >> BATTERY_LEVEL_SIG_SHIFT)
			& BATTERY_LEVEL_SIG_MASK;

	value = reboot_battery_value
			& (~(BATTERY_LEVEL_SIG_MASK << BATTERY_LEVEL_SIG_SHIFT));

	if (signature != BATTERY_LEVEL_SIG || value == BATTERY_LEVEL_NO_VALUE)
		return BATTERY_LEVEL_NO_VALUE;

	level = value & BATTERY_LEVEL_MASK;

	if (level > 100)
		return BATTERY_LEVEL_NO_VALUE;

	return level;
}

static unsigned int tps80032_batt_get_reboot_is_charging_full(void)
{
	unsigned signature;
	unsigned value;

	signature = (reboot_battery_value >> BATTERY_LEVEL_SIG_SHIFT)
			& BATTERY_LEVEL_SIG_MASK;

	value = reboot_battery_value
			& (~(BATTERY_LEVEL_SIG_MASK << BATTERY_LEVEL_SIG_SHIFT));

	if (signature != BATTERY_LEVEL_SIG || value == BATTERY_LEVEL_NO_VALUE)
		return BATTERY_LEVEL_NO_VALUE;

	return (value >> BATTERY_IS_CHARGING_FULL_SHIFT) & BATTERY_IS_CHARGING_FULL_MASK;
}

static void batt_first_work_func(struct work_struct *work)
{
	int rc;
	unsigned long flags;

	CHECK_LOG();

	rc = htc_batt_get_battery_adc();
	if (rc) {
		BATT_ERR("Get first battery ADC value failed!");
		return;
	}

	spin_lock_irqsave(&htc_batt_info.batt_lock, flags);
	htc_batt_info.first_level_ready = 1;
	spin_unlock_irqrestore(&htc_batt_info.batt_lock, flags);
	BATT_LOG("First level read");
}

static unsigned int quickboot_low_power_boot;
static unsigned int quickboot_enabled;

static long htc_batt_ioctl(struct file *filp,
			unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	int led_change = 0;
	int offmode_usb_threshold, offmode_ac_threshold;
	CHECK_LOG();

	wake_lock(&htc_batt_timer.battery_lock);

	switch (cmd) {
	case HTC_BATT_IOCTL_READ_SOURCE: {
		if (copy_to_user((void __user *)arg,
			&htc_batt_info.rep.charging_source, sizeof(u32)))
			ret = -EFAULT;
		break;
	}
	case HTC_BATT_IOCTL_SET_BATT_ALARM: {
		u32 time_out = 0;
		if (copy_from_user(&time_out, (void *)arg, sizeof(u32))) {
			ret = -EFAULT;
			break;
		}

		htc_batt_timer.time_out = time_out;
		if (!htc_battery_initial) {
			htc_battery_initial = 1;
			batt_set_check_timer(htc_batt_timer.time_out);
		}
		break;
	}
	case HTC_BATT_IOCTL_GET_ADC_VREF: {
		if (copy_to_user((void __user *)arg, &htc_batt_info.adc_vref,
				sizeof(htc_batt_info.adc_vref))) {
			BATT_ERR("copy_to_user failed!");
			ret = -EFAULT;
		}
		break;
	}
	case HTC_BATT_IOCTL_GET_ADC_ALL: {
		if (copy_to_user((void __user *)arg, &htc_batt_info.adc_data,
					sizeof(struct battery_adc_reply))) {
			BATT_ERR("copy_to_user failed!");
			ret = -EFAULT;
		}
		break;
	}
	case HTC_BATT_IOCTL_CHARGER_CONTROL: {
		u32 charger_mode = 0;
		if (copy_from_user(&charger_mode, (void *)arg, sizeof(u32))) {
			BATT_ERR("copy_from_user failed!");
			ret = -EFAULT;
			break;
		}
		BATT_LOG("do charger control = %u", charger_mode);
		htc_battery_set_charging(charger_mode);
		break;
	}
	case HTC_BATT_IOCTL_UPDATE_BATT_INFO: {
		mutex_lock(&htc_batt_info.info_lock);
		if (copy_from_user(&htc_batt_info.rep, (void *)arg,
					sizeof(struct battery_info_reply))) {
			BATT_ERR("copy_from_user failed!");
			ret = -EFAULT;
			mutex_unlock(&htc_batt_info.info_lock);
			break;
		}

		if (htc_batt_info.mfg_mode == BOARD_MFG_MODE_OFFMODE_CHARGING) {
			if (htc_batt_info.charger == SWITCH_CHARGER_TPS80032) {
				offmode_usb_threshold = CHG_TPS80032_OFFMODE_USB_THRESHOLD;
				offmode_ac_threshold = CHG_TPS80032_OFFMODE_AC_THRESHOLD;
			} else {
				offmode_usb_threshold = DEFAULT_OFFMODE_USB_THRESHOLD;
				offmode_ac_threshold = DEFAULT_OFFMODE_AC_THRESHOLD;
			}
			if (htc_batt_info.rep.level >= 100) {
				if (!(htc_batt_info.pre_led_change & PRE_LED_100P)) {
					htc_batt_info.pre_led_change |= PRE_LED_100P;
					led_change = 1;
				}
			} else
				htc_batt_info.pre_led_change &= ~PRE_LED_100P;

			if (htc_batt_info.rep.batt_vol < offmode_usb_threshold) {
				htc_batt_info.pre_led_change &= ~(PRE_LED_USB_V | PRE_LED_AC_V);
			} else if (htc_batt_info.rep.batt_vol < offmode_ac_threshold) {
				if (!(htc_batt_info.pre_led_change & PRE_LED_USB_V)) {
					htc_batt_info.pre_led_change |= PRE_LED_USB_V;
					led_change = 1;
				}
				htc_batt_info.pre_led_change &= ~PRE_LED_AC_V;
			} else {
				if (!(htc_batt_info.pre_led_change & PRE_LED_AC_V)) {
					htc_batt_info.pre_led_change |= PRE_LED_AC_V;
					led_change = 1;
				}
			}

			if (led_change != 0)
				wake_lock_timeout(&htc_batt_timer.led_change_lock, HZ * 5);
		}

		if (htc_batt_info.guage_driver != GUAGE_TPS80032) {
			if (htc_batt_info.rep.batt_temp > 150) {
				if (htc_batt_info.rep.batt_vol <= 2700) {
					BATT_LOG("Normal temperature, critical shutdown, level 0");
					htc_batt_info.rep.charging_source = CHARGER_BATTERY;
					htc_batt_info.rep.level = 0;
				}
			}
		}

		if (!!(get_kernel_flag() & SET_FAKE_CAPACITY)) {
			htc_batt_info.rep.batt_vol = 3800;
			htc_batt_info.rep.batt_id = 1;
			htc_batt_info.rep.batt_temp = 300;
			htc_batt_info.rep.level = 77;
			htc_batt_info.rep.full_bat = 1580000;
			htc_batt_info.rep.full_level = 100;
			htc_batt_info.rep.batt_state = 1;
			htc_batt_info.rep.temp_fault = -1;
			htc_batt_info.rep.overload = 0;
		} else {
			if (htc_batt_info.over_vchg != 0)
				htc_batt_info.rep.charging_enabled = 0;
			htc_batt_info.rep.over_vchg = htc_batt_info.over_vchg;
		}

		if (quickboot_low_power_boot && htc_batt_info.rep.level >= 4)
			quickboot_low_power_boot = 0;

		if (quickboot_low_power_boot &&
			htc_batt_info.rep.batt_vol <= QB_LPB_SHUTDOWN_VOLTAGE) {
			BATT_LOG("QuickBoot once, and voltage lower than %d.  "
				"Shutdown System", QB_LPB_SHUTDOWN_VOLTAGE);
			mutex_unlock(&htc_batt_info.info_lock);
			kernel_power_off();
		}

		mutex_unlock(&htc_batt_info.info_lock);

		BATT_LOG("ioctl: battery level update: %u",
			htc_batt_info.rep.level);

		htc_battery_core_update();
		break;
	}
	case HTC_BATT_IOCTL_BATT_DEBUG_LOG:
		if (copy_from_user(htc_batt_info.debug_log, (void *)arg,
					DEBUG_LOG_LENGTH)) {
			BATT_ERR("copy debug log from user failed!");
			ret = -EFAULT;
		}
		break;
	case HTC_BATT_IOCTL_SET_VOLTAGE_ALARM: {
		struct battery_vol_alarm alarm_data;
		if (copy_from_user(&alarm_data, (void *)arg,
					sizeof(struct battery_vol_alarm))) {
			BATT_ERR("user set batt alarm failed!");
			ret = -EFAULT;
			break;
		}

		if (htc_batt_info.guage_driver == GUAGE_TPS80032) {
			atomic_set(&htc_batt_timer.batt_alarm_status, 0);
			htc_batt_timer.alarm_trigger_volt = 0;
			htc_batt_timer.batt_alarm_enabled = alarm_data.enable;
			htc_batt_timer.batt_alarm_threshold = alarm_data.lower_threshold;
			ret = batt_alarm_config(alarm_data.lower_threshold);
			if (ret)
				BATT_ERR("batt alarm config failed!");

			ret = tps80032_vsys_alarm_state_set(!!alarm_data.enable);
			if (ret)
				BATT_ERR("state_set enabled failed, rc=%d!", ret);
			BATT_LOG("Set vsys alarm threshold: %d, Enabled:%u.",
					alarm_data.lower_threshold, alarm_data.enable);
		}
		break;
	}
	case HTC_BATT_IOCTL_SET_ALARM_TIMER_FLAG: {
		/* alarm flag could be reset by cable. */
		unsigned int flag;
		if (copy_from_user(&flag, (void *)arg, sizeof(unsigned int))) {
			BATT_ERR("Set timer type into alarm failed!");
			ret = -EFAULT;
			break;
		}
		htc_batt_timer.alarm_timer_flag = flag;
		BATT_LOG("Set alarm timer flag:%u", flag);
		break;
	}
	case HTC_BATT_IOCTL_BATT_POWER_METER:
		if (copy_from_user(htc_batt_info.power_meter, (void *)arg,
					POWER_METER_LENGTH)) {
			BATT_ERR("copy power meter from user failed!");
			ret = -EFAULT;
		}
		break;
	case HTC_BATT_IOCTL_SET_VREG: {
		unsigned int voltage;
		unsigned int voltage_select = OVERTEMP_VREG_BOUND;
		if (copy_from_user(&voltage, (void *)arg, sizeof(unsigned int))) {
			BATT_ERR("Set charger regulation voltage failed!");
			ret = -EFAULT;
			break;
		}
		BATT_LOG("Set charger regulation valtage: %d", voltage);

		if (voltage == NORMALTEMP_VREG_BOUND)
			voltage_select = NORMALTEMP_VREG;
		else if (voltage == NORMALTEMP_VREG_HV4340_BOUND)
			voltage_select = NORMALTEMP_VREG_HV4340;
		else if (voltage == NORMALTEMP_VREG_HV4320_BOUND)
			voltage_select = NORMALTEMP_VREG_HV4320;
		else if (voltage == OVERTEMP_VREG_BOUND)
			voltage_select = OVERTEMP_VREG;
		else if (voltage == UNDERTEMP_VREG_BOUND)
			voltage_select = UNDERTEMP_VREG;
		else if (voltage == NORMALTEMP_VREG_HV_BOUND)
			voltage_select = NORMALTEMP_VREG_HV;
		else
			BATT_ERR("NO support voltage, use lowest regulation voltage");

		htc_battery_set_charging(voltage_select);
		break;
	}
	case HTC_BATT_IOCTL_SET_VSYS_REG: {
		unsigned int vsys_voltage;
		unsigned int vsys_select = ALLTEMP_VSYS_DISABLE;
		if (copy_from_user(&vsys_voltage, (void *)arg, sizeof(unsigned int))) {
			BATT_ERR("Charger vsys regulation set failed!");
			ret = -EFAULT;
			break;
		}
		BATT_LOG("Charger vsys regulation valtage set: %d", vsys_voltage);

		if (vsys_voltage == NORMALTEMP_VSYS_4400_BOUND)
			vsys_select = NORMALTEMP_VSYS_4400;
		else if (vsys_voltage == NORMALTEMP_VSYS_4440_BOUND)
			vsys_select = NORMALTEMP_VSYS_4440;
		else if (vsys_voltage == OVERTEMP_VSYS_4300_BOUND)
			vsys_select = OVERTEMP_VSYS_4300;
		else if (vsys_voltage == OVERTEMP_VSYS_4340_BOUND)
			vsys_select = OVERTEMP_VSYS_4340;
		else if (vsys_voltage == OVERTEMP_VSYS_4140_BOUND)
			vsys_select = OVERTEMP_VSYS_4140;
		else if (vsys_voltage == UNDERTEMP_VSYS_4200_BOUND)
			vsys_select = UNDERTEMP_VSYS_4200;

		htc_battery_set_charging(vsys_select);
		break;
	}
	case HTC_BATT_IOCTL_ADC_TRIGGER: {
		unsigned int mode;
		if (copy_from_user(&mode, (void *)arg, sizeof(unsigned int))) {
			BATT_ERR("ADC trigger failed!");
			ret = -EFAULT;
			break;
		}

		if (mode > 0) {
			ret = htc_batt_get_battery_adc();
			if (ret) {
				BATT_ERR("Trigger battery ADC value failed!");
			}
		}

		break;
	}
	default:
		BATT_ERR("%s: no matched ioctl cmd", __func__);
		break;
	}

	wake_unlock(&htc_batt_timer.battery_lock);

	return ret;
}

/*  MBAT_IN interrupt handler	*/
static void no_batt_func(struct work_struct *work)
{
	int is_power_off;
	CHECK_LOG();

	mutex_lock(&htc_batt_info.info_lock);
	is_power_off = htc_batt_info.power_off_by_id;
	mutex_unlock(&htc_batt_info.info_lock);
	if (is_power_off && !(get_kernel_flag() & SET_FAKE_CAPACITY)) {
#if WK_MBAT_IN
		BATT_LOG("MBAT_IN interrupt!! No shutdown due to fake battery id");
#else
		BATT_LOG("shut down device due to MBAT_IN interrupt");
		htc_battery_set_charging(0);
		kernel_power_off();
#endif
	} else {
		BATT_LOG("MBAT_IN interrupt!! No shutdown due to debug flag or board setting");
	}
}

static irqreturn_t mbat_int_handler(int irq, void *data)
{
	struct htc_battery_platform_data *pdata = data;

	CHECK_LOG();

#if WK_MBAT_IN
	is_mbat_in = 0;
#endif

	disable_irq_nosync(pdata->gpio_mbat_in);

	schedule_delayed_work(&no_batt_struct, msecs_to_jiffies(50));
	return IRQ_HANDLED;
}
/*  MBAT_IN interrupt handler end   */

const struct file_operations htc_batt_fops = {
	.owner = THIS_MODULE,
	.open = htc_batt_open,
	.release = htc_batt_release,
	.unlocked_ioctl = htc_batt_ioctl,
};

static struct miscdevice htc_batt_device_node = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "htc_batt",
	.fops = &htc_batt_fops,
};

static void htc_batt_kobject_release(struct kobject *kobj)
{
	BATT_ERR("htc_batt_kobject_release.");
	return;
}

static struct kobj_type htc_batt_ktype = {
	.release = htc_batt_kobject_release,
};

#if defined(CONFIG_HAS_EARLYSUSPEND)
static void htc_battery_early_suspend(struct early_suspend *h)
{
	CHECK_LOG();

	mutex_lock(&htc_batt_limit_charging_lock);
	if (htc_batt_info.charger == SWITCH_CHARGER_TPS80032) {
		if (!!htc_batt_limit_charging && !(htc_batt_limit_charging & KEEP_EARLY_SUSPEND_LIMIT_CHARGING)) {
			htc_battery_set_charging(CLEAR_LIMITED_CHG);
		}
	}
	mutex_unlock(&htc_batt_limit_charging_lock);
}

static void htc_battery_late_resume(struct early_suspend *h)
{
	CHECK_LOG();

	htc_batt_timer.total_time_ms += (jiffies -
			htc_batt_timer.batt_system_jiffies) * MSEC_PER_SEC / HZ;
	htc_batt_timer.batt_system_jiffies = jiffies;

	if (htc_batt_timer.total_time_ms >= BATT_LATE_RESUME_CHECK_TIME * MSEC_PER_SEC) {
		BATT_LOG("late resume with check time up, update battery level");
		del_timer_sync(&htc_batt_timer.batt_timer);
		cancel_work_sync(&htc_batt_timer.batt_work);
		wake_lock(&htc_batt_timer.battery_lock);
		queue_work(htc_batt_timer.batt_wq, &htc_batt_timer.batt_work);
	}

	mutex_lock(&htc_batt_limit_charging_lock);
	if (htc_batt_info.charger == SWITCH_CHARGER_TPS80032) {
		if (!!htc_batt_limit_charging && !(htc_batt_limit_charging & KEEP_EARLY_SUSPEND_LIMIT_CHARGING)) {
			htc_battery_set_charging(ENABLE_LIMITED_CHG);
		}
	}
	mutex_unlock(&htc_batt_limit_charging_lock);
}
#endif

static unsigned long target_interval_ms = 0;
static int htc_battery_prepare(struct device *dev)
{
#if 0  /* fixme: not require this now */
	int rc = 0;
#endif
	int time_diff;
	int time_out;
	int use_time_out_flag;
	struct timespec xtime;
	ktime_t interval;
	ktime_t slack = ktime_set(0, 0);
	ktime_t next_alarm;

	CHECK_LOG();

	target_interval_ms = 0;

	del_timer_sync(&htc_batt_timer.batt_timer);
	cancel_work_sync(&htc_batt_timer.batt_work);

	htc_batt_timer.total_time_ms += (jiffies -
			htc_batt_timer.batt_system_jiffies) * MSEC_PER_SEC / HZ;
	htc_batt_timer.batt_system_jiffies = jiffies;
	getnstimeofday(&xtime);
	htc_batt_timer.batt_suspend_ms = xtime.tv_sec * MSEC_PER_SEC +
					xtime.tv_nsec / NSEC_PER_MSEC;

	use_time_out_flag = (htc_batt_timer.charger_flag || htc_batt_timer.alarm_timer_flag);

	if (use_time_out_flag) {
		switch (htc_batt_info.charger) {
		case SWITCH_CHARGER_TPS80032:
			if (htc_batt_info.rep.level == 100
					&& htc_batt_info.mfg_mode == BOARD_MFG_MODE_OFFMODE_CHARGING) {
				time_out = BATT_SUSPEND_CHECK_TIME;
			} else
				time_out = htc_batt_timer.time_out;
			break;
		default:
			time_out = htc_batt_timer.time_out;
			break;
		}
	} else if (htc_batt_phone_call)
		time_out = BATT_SUSPEND_PHONE_CALL_CHECK_TIME;
	else
		time_out = BATT_SUSPEND_CHECK_TIME;

	time_diff = time_out * MSEC_PER_SEC - htc_batt_timer.total_time_ms;

	if (time_diff <= 5 * MSEC_PER_SEC) {
		wake_lock(&htc_batt_timer.battery_lock);
		queue_work(htc_batt_timer.batt_wq, &htc_batt_timer.batt_work);
		return -EBUSY;
	} else if (use_time_out_flag)
		batt_set_check_timer((time_diff / MSEC_PER_SEC) + 1);
	else
		batt_set_check_timer(htc_batt_timer.time_out);

#if 0  /* fixme: not require this now */
	if (!htc_batt_timer.alarm_timer_flag
		&& !htc_batt_phone_call
		&& htc_batt_timer.batt_alarm_enabled) {
		rc = tps80032_vsys_alarm_state_set(1);
		if (rc) {
			BATT_ERR("state_set enabled failed, rc=%d!", rc);
			return -EBUSY;
		}
	}
#endif

	interval = ktime_set(time_diff / MSEC_PER_SEC,
			(time_diff % MSEC_PER_SEC) * NSEC_PER_MSEC);

	xtime = ktime_to_timespec(interval);
	BATT_LOG("%s: passing time:%lu, status:%u%u%u, "
		"alarm will be triggered after %ld.%ld seconds",
		__func__, htc_batt_timer.total_time_ms,
		htc_batt_phone_call,
		htc_batt_timer.alarm_timer_flag,
		htc_batt_timer.charger_flag,
		xtime.tv_sec, xtime.tv_nsec);
	next_alarm = ktime_add(alarm_get_elapsed_realtime(), interval);
	alarm_start_range(&htc_batt_timer.batt_check_wakeup_alarm,
				next_alarm, ktime_add(next_alarm, slack));
	target_interval_ms = xtime.tv_sec * MSEC_PER_SEC;
	return 0;
}

int dbg_tps_showk(void); /* from tps80032.c */
static void htc_battery_complete(struct device *dev)
{
	unsigned long resume_ms;
	unsigned long check_time;
	int use_time_out_flag;
	struct timespec xtime;

	CHECK_LOG();

	del_timer_sync(&htc_batt_timer.batt_timer);
	cancel_work_sync(&htc_batt_timer.batt_work);

	getnstimeofday(&xtime);
	resume_ms = xtime.tv_sec * MSEC_PER_SEC + xtime.tv_nsec / NSEC_PER_MSEC;
	htc_batt_timer.total_time_ms += resume_ms -
					htc_batt_timer.batt_suspend_ms;
	htc_batt_timer.batt_system_jiffies = jiffies;

	BATT_LOG("%s: total suspend time:%lu, the total passing time:%lu",
			__func__, (resume_ms - htc_batt_timer.batt_suspend_ms),
			htc_batt_timer.total_time_ms);

#if WK_ALARM_NOT_WORK
        /* XXX We found rtc-alarm will have chance not work.  If rtc-alarm not
         * work, the capacity accumulate will have error, and other security
         * effect.  This check will dump RTC register in kernel.  And also set
         * flag to keep device not to suspend.
         * If suspend time over target interval 20sec twice, set the flag.
         * Clean when cable status change.
         * However if resume take too long, the workaround may also happened.
         */
        if (!is_alarm_not_work &&
                target_interval_ms != 0 &&
                target_interval_ms + 20000 <
                        (resume_ms - htc_batt_timer.batt_suspend_ms)) {
                pr_info("[BATT] Alarm not work!!\n");
                dbg_tps_showk();
                is_alarm_not_work = 1;
        }
#endif

	use_time_out_flag = (htc_batt_timer.charger_flag || htc_batt_timer.alarm_timer_flag);

	if (use_time_out_flag)
		/* 500 msecs check buffer time */
		check_time = (htc_batt_timer.time_out * MSEC_PER_SEC)
				- (MSEC_PER_SEC / 2);
	else if (htc_batt_phone_call)
		check_time = BATT_SUSPEND_PHONE_CALL_CHECK_TIME * MSEC_PER_SEC;
	else
		check_time = BATT_SUSPEND_CHECK_TIME * MSEC_PER_SEC;

	/*  - When kernel resumes, battery driver should check total time to decide if do battery algorithm or just ignore.
	    - If kernel resumes due to battery voltage alarm, do battery algorithm forcibly. */
	if (htc_batt_timer.total_time_ms >= check_time) {
		wake_lock(&htc_batt_timer.battery_lock);
		BATT_LOG("Check battery status for resume at checking time up");
		queue_work(htc_batt_timer.batt_wq, &htc_batt_timer.batt_work);
	} else if (use_time_out_flag)
		batt_set_check_timer(((check_time - htc_batt_timer.total_time_ms) / MSEC_PER_SEC) + 1);
	else
		batt_set_check_timer(htc_batt_timer.time_out);

#if 0  /* fixme: not require this now */
	tps80032_vsys_alarm_state_set(0);
#endif
}

static struct dev_pm_ops htc_battery_tps80032_pm_ops = {
	.prepare = htc_battery_prepare,
	.complete = htc_battery_complete,
};

static struct device_attribute tps80032_batt_attrs[] = {
	__ATTR(reboot_level, S_IRUGO, tps80032_batt_show_attributes, NULL),
	__ATTR(reboot_is_charging_full, S_IRUGO, tps80032_batt_show_attributes, NULL),
	__ATTR(hboot_volt, S_IRUGO, tps80032_batt_show_attributes, NULL),
	__ATTR(hboot_curr, S_IRUGO, tps80032_batt_show_attributes, NULL),
	__ATTR(hboot_temp, S_IRUGO, tps80032_batt_show_attributes, NULL),
	__ATTR(quickboot_low_power_boot, S_IWUSR, NULL, tps80032_qb_store_attributes),
	__ATTR(quickboot_enabled, S_IWUSR, NULL, tps80032_qb_store_attributes),
	__ATTR(fake_temp, S_IWUSR, NULL, tps80032_fake_temp_store_attributes),
	__ATTR(suspend_1_percent, S_IRUGO, tps80032_batt_show_attributes, NULL),
	};

static ssize_t tps80032_batt_show_attributes(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	int i = 0;
	const ptrdiff_t off = attr - tps80032_batt_attrs;
	int temp;

	if (off < sizeof(tps80032_batt_attrs)) {
		switch (htc_batt_info.guage_driver) {
		case GUAGE_TPS80032:
			if (off == ATTR_REBOOT_LEVEL)
				i += scnprintf(buf + i, PAGE_SIZE - i, "%u\n",
						tps80032_batt_get_reboot_battery_level());
			else if (off == ATTR_REBOOT_IS_CHARGING_FULL)
				i += scnprintf(buf + i, PAGE_SIZE - i, "%u\n",
						tps80032_batt_get_reboot_is_charging_full());
			else if (off == ATTR_HBOOT_VOLT) {
				if (hboot_batt_volt == TPS80032_GPADC_FAKE_VALUE)
					temp = TPS80032_GPADC_FAKE_VALUE;
				else
					temp = (hboot_batt_volt * 5) >> 2;
				BATT_LOG("volt_reg=0x%lx, volt=%d", hboot_batt_volt, temp);
				i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
						temp);
			} else if (off == ATTR_HBOOT_CURR) {
				if (hboot_batt_curr == TPS80032_GPADC_FAKE_VALUE)
					temp = TPS80032_GPADC_FAKE_VALUE;
				else {
					if (!!(hboot_batt_curr & 0x8000))
						temp = 0xFFFF0000 | hboot_batt_curr;
					else
						temp = hboot_batt_curr;
					temp = ((temp * 5) >> 2 ) / 10;
				}
				BATT_LOG("curr_reg=0x%lx, curr=%d", hboot_batt_curr, temp);
				i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
						temp);
			} else if (off == ATTR_HBOOT_TEMP)
				i += scnprintf(buf + i, PAGE_SIZE - i, "%u\n",
						(unsigned) (hboot_batt_temp & 0xFFFFFFFF));
			else if (off == ATTR_SUSPEND_1_PERCENT)
				i += scnprintf(buf + i, PAGE_SIZE - i, "%u\n",
						!!(quickboot_low_power_boot || quickboot_enabled));
			else
				i = -EINVAL;
			break;
		default:
			if (off == ATTR_REBOOT_LEVEL)
				i += scnprintf(buf + i, PAGE_SIZE - i, "%u\n",
						tps80032_batt_get_reboot_battery_level());
			else if (off == ATTR_REBOOT_IS_CHARGING_FULL)
				i += scnprintf(buf + i, PAGE_SIZE - i, "%u\n",
						tps80032_batt_get_reboot_is_charging_full());
			else if (off == ATTR_HBOOT_VOLT)
				i += scnprintf(buf + i, PAGE_SIZE - i, "%u\n",
						(unsigned) (hboot_batt_volt & 0xFFFFFFFF));
			else if (off == ATTR_HBOOT_CURR)
				i += scnprintf(buf + i, PAGE_SIZE - i, "%u\n",
						(unsigned) (hboot_batt_curr & 0xFFFFFFFF));
			else if (off == ATTR_HBOOT_TEMP)
				i += scnprintf(buf + i, PAGE_SIZE - i, "%u\n",
						(unsigned) (hboot_batt_temp & 0xFFFFFFFF));
			else
				i = -EINVAL;
			break;
		}
	} else
		i = -EINVAL;

	if (i < 0)
		pr_err("%s: attribute %d is not supported\n",
			__func__, i);

	return i;
}

static int tps80032_qb_store_attributes(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	const ptrdiff_t off = attr - tps80032_batt_attrs;

	if (off < sizeof(tps80032_batt_attrs)) {
		if (off == ATTR_QB_LOW_POWER) {
			quickboot_low_power_boot = sysfs_streq(buf, "0") ? 0 : 1;
			BATT_LOG("set quickboot_low_power_boot %d", quickboot_low_power_boot);
		} else if (off == ATTR_QB_ENABLED) {
			quickboot_enabled = sysfs_streq(buf, "0") ? 0 : 1;
			BATT_LOG("set quickboot_enabled %d", quickboot_enabled);
		} else {
			BATT_LOG("%s: No support such value store", __func__);
		}
	}
	return size;
}

static int tps80032_fake_temp_store_attributes(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int value;

	value = ((int) simple_strtoul(buf, NULL, 10));
	BATT_LOG("fake temp control: %d", value);

	mutex_lock(&htc_batt_info.info_lock);
	if (htc_batt_info.sw_temp_25 < 0) {
		BATT_LOG("NO support fake temp control!!");
		goto done;
	}

	if (value == htc_batt_info.fake_temp_state) {
		BATT_LOG("The same fake temp state: %d, do nothing", htc_batt_info.fake_temp_state);
		goto done;
	}

	htc_batt_info.fake_temp_state = value;
	switch (value) {
	case LOW_TEMP_COND:
		gpio_direction_output(htc_batt_info.sw_temp_25, 0);
		break;
	case HIGH_TEMP_COND:
		gpio_direction_output(htc_batt_info.sw_temp_25, 1);
		break;
	case NORMAL_TEMP_COND:
		/* NOTE: do nothing, let it be the default behavior */
	default:
		gpio_direction_input(htc_batt_info.sw_temp_25);
		gpio_set_pullnone(htc_batt_info.sw_temp_25);
	}

done:
	mutex_unlock(&htc_batt_info.info_lock);
	return size;
}

static int tps80032_batt_create_attrs(struct device *dev)
{
	int i = 0, rc = 0;

	for (i = 0; i < ARRAY_SIZE(tps80032_batt_attrs); i++) {
		rc = device_create_file(dev, &tps80032_batt_attrs[i]);
		if (rc)
			goto batt_attrs_failed;
	}

	goto succeed;

batt_attrs_failed:
	while (i--)
		device_remove_file(dev, &tps80032_batt_attrs[i]);
succeed:
	return rc;
}

static int htc_battery_probe(struct platform_device *pdev)
{
	int i, rc = 0;
	struct htc_battery_platform_data *pdata = pdev->dev.platform_data;
	struct htc_battery_core *htc_battery_core_ptr;

	CHECK_LOG();

	htc_battery_core_ptr = kmalloc(sizeof(struct htc_battery_core),
					GFP_KERNEL);
	if (!htc_battery_core_ptr) {
		BATT_ERR("%s: kmalloc failed for htc_battery_core_ptr.",
				__func__);
		return -ENOMEM;
	}

	mutex_lock(&htc_batt_info.info_lock);
	htc_batt_info.power_off_by_id = pdata->power_off_by_id;
	mutex_unlock(&htc_batt_info.info_lock);

	if (pdata->gpio_mbat_in >= 0) {
		INIT_DELAYED_WORK(&no_batt_struct, no_batt_func);
		if (pdata->gpio_mbat_in_trigger_level == MBAT_IN_HIGH_TRIGGER)
			rc = request_irq(pdata->gpio_mbat_in,
					mbat_int_handler, IRQF_TRIGGER_HIGH,
					"mbat_in", pdata);
		else if (pdata->gpio_mbat_in_trigger_level == MBAT_IN_LOW_TRIGGER)
			rc = request_irq(pdata->gpio_mbat_in,
					mbat_int_handler, IRQF_TRIGGER_LOW,
					"mbat_in", pdata);
		if (rc)
			BATT_ERR("request mbat_in irq failed!");
		else
			irq_set_irq_wake(pdata->gpio_mbat_in, 1);
	}

	htc_battery_core_ptr->func_show_batt_attr = htc_battery_show_batt_attr;
	htc_battery_core_ptr->func_show_batt_power_meter = htc_battery_show_batt_power_meter;
	htc_battery_core_ptr->func_get_battery_info = htc_batt_get_battery_info;
	htc_battery_core_ptr->func_charger_control = htc_batt_charger_control;
	htc_battery_core_ptr->func_set_full_level = htc_batt_set_full_level;
	htc_battery_core_ptr->func_phone_call_notification = htc_batt_phone_call_notification;
	htc_battery_core_ptr->func_limit_charging_notification = htc_batt_limit_charging_notification;
	htc_battery_core_register(&pdev->dev, htc_battery_core_ptr);

	htc_batt_info.device_id = pdev->id;
	htc_batt_info.guage_driver = pdata->guage_driver;
	htc_batt_info.charger = pdata->charger;
	htc_batt_info.vzero_clb_channel = pdata->vzero_clb_channel;
	htc_batt_info.volt_adc_offset = pdata->volt_adc_offset;
	htc_batt_info.check2_value = 0;
	htc_batt_info.first_level_ready = 0;
	htc_batt_info.charge_done = 0;
	htc_batt_info.mfg_mode = board_mfg_mode();

	if (gpio_is_valid(pdata->sw_temp_25)) {
		htc_batt_info.sw_temp_25 = pdata->sw_temp_25;
		gpio_request(pdata->sw_temp_25, "sw_temp_25");
	} else
		htc_batt_info.sw_temp_25 = -1;

	htc_batt_info.rep.full_level = 100;

	htc_batt_info.is_open = 0;

	for (i = 0; i < ADC_REPLY_ARRAY_SIZE; i++)
		htc_batt_info.adc_vref[i] = (1 << 12) - 1;

	htc_batt_info.online = -1;

	INIT_WORK(&htc_batt_timer.batt_work, batt_work_func);
	init_timer(&htc_batt_timer.batt_timer);
	htc_batt_timer.batt_timer.function = batt_regular_timer_handler;
	alarm_init(&htc_batt_timer.batt_check_wakeup_alarm,
			ANDROID_ALARM_ELAPSED_REALTIME_WAKEUP,
			batt_check_alarm_handler);
	INIT_WORK(&htc_batt_timer.batt_first_work, batt_first_work_func);
	init_timer(&htc_batt_timer.batt_first_timer);
	htc_batt_timer.batt_first_timer.function = batt_first_timer_handler;
	htc_batt_timer.batt_wq = create_singlethread_workqueue("batt_timer");

	rc = misc_register(&htc_batt_device_node);
	if (rc) {
		BATT_ERR("Unable to register misc device %d",
			MISC_DYNAMIC_MINOR);
		goto fail;
	}

	htc_batt_kset = kset_create_and_add("event_to_daemon", NULL,
			kobject_get(&htc_batt_device_node.this_device->kobj));
	if (!htc_batt_kset) {
		rc = -ENOMEM;
		goto fail;
	}

	htc_batt_info.batt_timer_kobj.kset = htc_batt_kset;
	rc = kobject_init_and_add(&htc_batt_info.batt_timer_kobj,
				&htc_batt_ktype, NULL, "htc_batt_timer");
	if (rc) {
		BATT_ERR("init kobject htc_batt_timer failed.");
		kobject_put(&htc_batt_info.batt_timer_kobj);
		goto fail;
	}

	htc_batt_info.batt_cable_kobj.kset = htc_batt_kset;
	rc = kobject_init_and_add(&htc_batt_info.batt_cable_kobj,
				&htc_batt_ktype, NULL, "htc_cable_detect");
	if (rc) {
		BATT_ERR("init kobject htc_cable_timer failed.");
		kobject_put(&htc_batt_info.batt_timer_kobj);
		goto fail;
	}

	if (pdata->charger == SWITCH_CHARGER_TPS65200)
		tps_register_notifier(&tps_int_notifier);
	else if (pdata->charger == SWITCH_CHARGER_TPS80032) {
		tps80032_charger_register_notifier(&tps80032_int_notifier);
		tps80032_charger_append_attr(&pdev->dev);
	}

	if (pdata->guage_driver == GUAGE_TPS80032)
		INIT_WORK(&htc_batt_timer.alarm_check_work, alarm_check_work_func);

	tps80032_vsys_alarm_register_notifier(&battery_alarm_notifier);

#ifdef CONFIG_HAS_EARLYSUSPEND
	htc_batt_info.early_suspend.level = EARLY_SUSPEND_LEVEL_STOP_DRAWING;
	htc_batt_info.early_suspend.suspend = htc_battery_early_suspend;
	htc_batt_info.early_suspend.resume = htc_battery_late_resume;
	register_early_suspend(&htc_batt_info.early_suspend);
#endif

#if 0	/* fixme: use a delayed work to get better timming */
	rc = htc_batt_get_battery_adc();
	if (rc) {
		BATT_ERR("Get first battery ADC value failed!");
		goto fail;
	}
#endif
	tps80032_batt_get_first_reboot_battery_value();

	tps80032_batt_create_attrs(&pdev->dev);

	mod_timer(&htc_batt_timer.batt_first_timer,
			jiffies + msecs_to_jiffies(FIRST_ADC_READ_DELAY * 1000));

	BATT_LOG("htc_battery_probe(): finish");

fail:
	kfree(htc_battery_core_ptr);
	return rc;
}

static struct platform_driver htc_battery_driver = {
	.probe	= htc_battery_probe,
	.driver	= {
		.name	= "htc_battery",
		.owner	= THIS_MODULE,
		.pm = &htc_battery_tps80032_pm_ops,
	},
};

static const char* battery_reason[] = {
	"", "offmode", "force-hard",
	"oem-11", "oem-33", "oem-77", "oem-88",
};

static int htc_battery_reboot_callback(struct notifier_block *nb,
		unsigned long event, void *unused)
{
	int i;
	char* cmd;
	int cmd_len;
	unsigned int level, is_charging_full, reboot_value;

	cmd = (char*) (unused ? unused : "");
	cmd_len = strlen(cmd);
	for (i = 0; i < ARRAY_SIZE(battery_reason); i++)
		if (0 == strncmp(battery_reason[i], cmd, cmd_len)) {
			pr_info("DEBUG: reboot: %s (%s)\n", __func__, cmd);

			level = tps80032_batt_get_battery_level();
			is_charging_full = ((unsigned int) htc_battery_is_charging_full());

			if ((level >= 0 && level <= 100) || (level == BATTERY_LEVEL_NO_VALUE)) {
				reboot_value = (BATTERY_LEVEL_SIG << BATTERY_LEVEL_SIG_SHIFT)
					| ((is_charging_full & BATTERY_IS_CHARGING_FULL_MASK) << BATTERY_IS_CHARGING_FULL_SHIFT)
					| level;
				set_reboot_params_battery_level(reboot_value);
				BATT_LOG("record reboot_battery_first_level :0x%x", reboot_value);
			}
			break;
		}

	return NOTIFY_DONE;
}

static struct notifier_block htc_battery_reboot_notifier = {
	.notifier_call = htc_battery_reboot_callback,
};

static int __init htc_battery_init(void)
{
	CHECK_LOG();

	htc_batt_phone_call = 0;
	htc_battery_initial = 0;
	htc_full_level_flag = 0;
	spin_lock_init(&htc_batt_info.batt_lock);
	wake_lock_init(&htc_batt_info.vbus_wake_lock, WAKE_LOCK_SUSPEND,
			"vbus_present");
	wake_lock_init(&htc_batt_timer.battery_lock, WAKE_LOCK_SUSPEND,
			"htc_battery_tps80032");
	wake_lock_init(&htc_batt_timer.led_change_lock, WAKE_LOCK_SUSPEND,
			"htc_battery_led_change");
	wake_lock_init(&htc_batt_info.alarm_wake_lock, WAKE_LOCK_SUSPEND,
			"vsys_alarm");
	mutex_init(&htc_batt_info.info_lock);
	mutex_init(&htc_batt_limit_charging_lock);
	
#if WK_MBAT_IN
	is_mbat_in = 0;
#endif
	usb_register_notifier(&usb_status_notifier);
	platform_driver_register(&htc_battery_driver);

	/* init battery parameters. */
	htc_batt_info.rep.batt_vol = 3300;
	htc_batt_info.rep.batt_id = 1;
	htc_batt_info.rep.batt_temp = 300;
	htc_batt_info.rep.level = 10;
	htc_batt_info.rep.full_bat = 1580000;
	htc_batt_info.rep.full_level = 100;
	htc_batt_info.rep.batt_state = 0;
	htc_batt_info.rep.temp_fault = -1;
	htc_batt_info.rep.overload = 0;
	htc_batt_info.pre_led_change = 0;
	htc_batt_timer.total_time_ms = 0;
	htc_batt_timer.batt_system_jiffies = jiffies;
	atomic_set(&htc_batt_timer.batt_alarm_status, 0);
	htc_batt_timer.alarm_trigger_volt = 0;
	htc_batt_timer.alarm_timer_flag = 0;
	htc_batt_timer.charger_flag = 0;
	htc_batt_timer.time_out = 300;

	register_reboot_notifier(&htc_battery_reboot_notifier);
	return 0;
}

module_init(htc_battery_init);
MODULE_DESCRIPTION("HTC Battery Driver");
MODULE_LICENSE("GPL");
