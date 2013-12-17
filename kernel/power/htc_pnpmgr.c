/* linux/kernel/power/htc_pnpmgr.c
 *
 * Copyright (C) 2012 HTC Corporation.
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

#include <linux/module.h>
#include <linux/string.h>
#include <linux/cpu.h>

#include "power.h"

#define MAX_BUF 128

static wait_queue_head_t sysfs_state_wq;

static struct kobject *cpufreq_kobj;
static struct kobject *hotplug_kobj;
static struct kobject *thermal_kobj;
static struct kobject *apps_kobj;
static struct kobject *pnpmgr_kobj;
static struct kobject *adaptive_policy_kobj;

#define define_string_show(_name, str_buf)				\
static ssize_t _name##_show						\
(struct kobject *kobj, struct kobj_attribute *attr, char *buf)		\
{									\
	return snprintf(buf, strnlen(str_buf, MAX_BUF) + 1, str_buf);	\
}

#define define_string_store(_name, str_buf, store_cb)		\
static ssize_t _name##_store					\
(struct kobject *kobj, struct kobj_attribute *attr,		\
 const char *buf, size_t n)					\
{								\
	strncpy(str_buf, buf, MAX_BUF);				\
	str_buf[MAX_BUF-1] = '\0';				\
	(store_cb)(#_name);					\
	sysfs_notify(kobj, NULL, #_name);			\
	return n;						\
}

#define define_int_show(_name, int_val)				\
static ssize_t _name##_show					\
(struct kobject *kobj, struct kobj_attribute *attr, char *buf)	\
{								\
	return sprintf(buf, "%d", int_val);			\
}

#define define_int_store(_name, int_val, store_cb)		\
static ssize_t _name##_store					\
(struct kobject *kobj, struct kobj_attribute *attr,		\
 const char *buf, size_t n)					\
{								\
	int val;						\
	if (sscanf(buf, "%d", &val) > 0) {			\
		int_val = val;					\
		(store_cb)(#_name);				\
		sysfs_notify(kobj, NULL, #_name);		\
		return n;					\
	}							\
	return -EINVAL;						\
}

static char activity_buf[MAX_BUF];

static void null_cb(const char *attr) {
	do { } while (0);
}

define_string_show(activity_trigger, activity_buf);
define_string_store(activity_trigger, activity_buf, null_cb);
power_attr(activity_trigger);

#ifdef CONFIG_ARCH_TEGRA
static unsigned int perflock_scaling_min_value;
static unsigned int perflock_scaling_max_value;
static int cpu_auto_hotplug_value;
static int min_on_cpus_value;
static int cpu1_online_value;
static int cpu2_online_value;
static int cpu3_online_value;
static int cpu_user_cap_value;
static int edp_ap_limit_value;
static char ap_dvcs_value[MAX_BUF] = {0};
static int data_throttling_value;

define_int_show(perflock_scaling_min, perflock_scaling_min_value);
define_int_store(perflock_scaling_min, perflock_scaling_min_value, null_cb);
power_attr(perflock_scaling_min);
define_int_show(perflock_scaling_max, perflock_scaling_max_value);
define_int_store(perflock_scaling_max, perflock_scaling_max_value, null_cb);
power_attr(perflock_scaling_max);
define_int_show(cpu_auto_hotplug, cpu_auto_hotplug_value);
define_int_store(cpu_auto_hotplug, cpu_auto_hotplug_value, null_cb);
power_attr(cpu_auto_hotplug);
define_int_show(min_on_cpus, min_on_cpus_value);
define_int_store(min_on_cpus, min_on_cpus_value, null_cb);
power_attr(min_on_cpus);

define_int_show(cpu1_online, cpu1_online_value);
define_int_store(cpu1_online, cpu1_online_value, null_cb);
power_attr(cpu1_online);
define_int_show(cpu2_online, cpu2_online_value);
define_int_store(cpu2_online, cpu2_online_value, null_cb);
power_attr(cpu2_online);
define_int_show(cpu3_online, cpu3_online_value);
define_int_store(cpu3_online, cpu3_online_value, null_cb);
power_attr(cpu3_online);

define_string_show(ap_dvcs, ap_dvcs_value);
define_string_store(ap_dvcs, ap_dvcs_value, null_cb);
power_attr(ap_dvcs);
define_int_show(cpu_user_cap, cpu_user_cap_value);
define_int_store(cpu_user_cap, cpu_user_cap_value, null_cb);
power_attr(cpu_user_cap);
define_int_show(edp_ap_limit, edp_ap_limit_value);
define_int_store(edp_ap_limit, edp_ap_limit_value, null_cb);
power_attr(edp_ap_limit);

define_int_show(pause_dt, data_throttling_value);
define_int_store(pause_dt, data_throttling_value, null_cb);
power_attr(pause_dt);
#endif


#ifdef CONFIG_PERFLOCK
extern ssize_t
perflock_scaling_max_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf);
extern ssize_t
perflock_scaling_max_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t n);
extern ssize_t
perflock_scaling_min_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf);
extern ssize_t
perflock_scaling_min_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t n);
power_attr(perflock_scaling_max);
power_attr(perflock_scaling_min);
#endif

#ifdef CONFIG_HOTPLUG_CPU
ssize_t
cpu_hotplug_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf)
{
	int ret = 0;
	ret = sprintf(buf, "%u", num_online_cpus());
	return ret;
}
ssize_t
cpu_hotplug_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t n)
{
	return 0;
}
power_attr(cpu_hotplug);
#endif

static struct attribute *cpufreq_g[] = {
#ifdef CONFIG_PERFLOCK
	&perflock_scaling_max_attr.attr,
	&perflock_scaling_min_attr.attr,
#endif
#ifdef CONFIG_ARCH_TEGRA
	&perflock_scaling_min_attr.attr,
	&perflock_scaling_max_attr.attr,
	&ap_dvcs_attr.attr,
#endif
	NULL,
};

static struct attribute *hotplug_g[] = {
#ifdef CONFIG_HOTPLUG_CPU
	&cpu_hotplug_attr.attr,
#endif
#ifdef CONFIG_ARCH_TEGRA
	&cpu_auto_hotplug_attr.attr,
	&min_on_cpus_attr.attr,
	&cpu1_online_attr.attr,
	&cpu2_online_attr.attr,
	&cpu3_online_attr.attr,
	&cpu_user_cap_attr.attr,
#endif
	NULL,
};

static struct attribute *thermal_g[] = {
#ifdef CONFIG_ARCH_TEGRA
	&edp_ap_limit_attr.attr,
	&pause_dt_attr.attr,
#endif
	NULL,
};

static struct attribute *apps_g[] = {
	&activity_trigger_attr.attr,
	NULL,
};

static struct attribute_group cpufreq_attr_group = {
	.attrs = cpufreq_g,
};

static struct attribute_group hotplug_attr_group = {
	.attrs = hotplug_g,
};

static struct attribute_group thermal_attr_group = {
	.attrs = thermal_g,
};

static struct attribute_group apps_attr_group = {
	.attrs = apps_g,
};

#ifdef CONFIG_HOTPLUG_CPU
static int __cpuinit cpu_hotplug_callback(struct notifier_block *nfb, unsigned long action, void *hcpu)
{
	switch (action) {
		/* To reduce overhead, we only notify cpu plug */
		case CPU_ONLINE:
		case CPU_ONLINE_FROZEN:
			sysfs_notify(hotplug_kobj, NULL, "cpu_hotplug");
			break;
		case CPU_DEAD:
		case CPU_DEAD_FROZEN:
			break;
	}
	return NOTIFY_OK;
}

static struct notifier_block __refdata cpu_hotplug_notifier = {
	.notifier_call = cpu_hotplug_callback,
};
#endif

static unsigned int slack_time_ms;
static unsigned int step_time_ms;
static unsigned int max_powersave_bias;
static unsigned int powersave_bias_step;
static unsigned int parameter_changed;
static unsigned int adaptive_policy_enabled = 1;

define_int_show(slack_time_ms, slack_time_ms);
define_int_store(slack_time_ms, slack_time_ms, null_cb);
power_attr(slack_time_ms);

define_int_show(step_time_ms, step_time_ms);
define_int_store(step_time_ms, step_time_ms, null_cb);
power_attr(step_time_ms);

define_int_show(max_powersave_bias, max_powersave_bias);
define_int_store(max_powersave_bias, max_powersave_bias, null_cb);
power_attr(max_powersave_bias);

define_int_show(powersave_bias_step, powersave_bias_step);
define_int_store(powersave_bias_step, powersave_bias_step, null_cb);
power_attr(powersave_bias_step);


define_int_show(parameter_changed, parameter_changed);
define_int_store(parameter_changed, parameter_changed, null_cb);
power_attr(parameter_changed);

define_int_show(enabled, adaptive_policy_enabled);
define_int_store(enabled, adaptive_policy_enabled, null_cb);
power_attr(enabled);


static struct attribute *adaptive_attr[] = {
	&slack_time_ms_attr.attr,
	&step_time_ms_attr.attr,
	&max_powersave_bias_attr.attr,
	&powersave_bias_step_attr.attr,
	&parameter_changed_attr.attr,
	&enabled_attr.attr,
	NULL,
};

static struct attribute_group adaptive_attr_group = {
	.attrs = adaptive_attr,
};

static int __init pnpmgr_init(void)
{
	int ret;

	init_waitqueue_head(&sysfs_state_wq);

	pnpmgr_kobj = kobject_create_and_add("pnpmgr", power_kobj);

	if (!pnpmgr_kobj) {
		pr_err("%s: Can not allocate enough memory for pnpmgr.\n", __func__);
		return -ENOMEM;
	}

	cpufreq_kobj = kobject_create_and_add("cpufreq", pnpmgr_kobj);
	hotplug_kobj = kobject_create_and_add("hotplug", pnpmgr_kobj);
	thermal_kobj = kobject_create_and_add("thermal", pnpmgr_kobj);
	apps_kobj = kobject_create_and_add("apps", pnpmgr_kobj);
	adaptive_policy_kobj = kobject_create_and_add("adaptive_policy", power_kobj);

	if (!cpufreq_kobj || !hotplug_kobj || !thermal_kobj || !apps_kobj || !adaptive_policy_kobj) {
		pr_err("%s: Can not allocate enough memory.\n", __func__);
		return -ENOMEM;
	}

	ret = sysfs_create_group(cpufreq_kobj, &cpufreq_attr_group);
	ret |= sysfs_create_group(hotplug_kobj, &hotplug_attr_group);
	ret |= sysfs_create_group(thermal_kobj, &thermal_attr_group);
	ret |= sysfs_create_group(apps_kobj, &apps_attr_group);
	ret |= sysfs_create_group(adaptive_policy_kobj, &adaptive_attr_group);

	if (ret) {
		pr_err("%s: sysfs_create_group failed\n", __func__);
		return ret;
	}

#ifdef CONFIG_HOTPLUG_CPU
	register_hotcpu_notifier(&cpu_hotplug_notifier);
#endif

	return 0;
}

static void  __exit pnpmgr_exit(void)
{
	sysfs_remove_group(cpufreq_kobj, &cpufreq_attr_group);
	sysfs_remove_group(hotplug_kobj, &hotplug_attr_group);
	sysfs_remove_group(thermal_kobj, &thermal_attr_group);
	sysfs_remove_group(apps_kobj, &apps_attr_group);
	sysfs_remove_group(adaptive_policy_kobj, &adaptive_attr_group);
#ifdef CONFIG_HOTPLUG_CPU
	unregister_hotcpu_notifier(&cpu_hotplug_notifier);
#endif
}

module_init(pnpmgr_init);
module_exit(pnpmgr_exit);

