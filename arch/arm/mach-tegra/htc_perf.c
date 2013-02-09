/*
 * arch/arm/mach-tegra/htc_perf.c
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

#include <linux/cpufreq.h>
#include <linux/kobject.h>
#include <linux/mutex.h>
#include <linux/pm_qos_params.h>
#include <linux/workqueue.h>
#include <linux/nct1008.h> /* for thermal temperature */
#include "htc_perf.h"
#include "cpu-tegra.h"
#include "fuse.h"

#define htc_perf_attr(attrbute) 				\
static struct kobj_attribute attrbute##_attr = {	\
	.attr	= {					\
		.name = __stringify(attrbute),		\
		.mode = 0644,				\
	},						\
	.show	= attrbute##_show,			\
	.store	= attrbute##_store,			\
}

#define DEF_TARGET_FREQ (1700000)
#define DEF_POKE_FREQ (1700000)
#define DEF_POKE_MS (100)
#define DEF_IDLE_MS (300)

#ifndef CONFIG_POWER_SAVE_FREQ
#define CONFIG_POWER_SAVE_FREQ (1300000)
#endif
unsigned int powersave_freq = CONFIG_POWER_SAVE_FREQ;

#define FUSE_CPUIDDQ 0x118

static char media_boost = 'N';
static unsigned int orig_user_cap = 0;

static int is_in_power_save = 0;
static int is_power_save_policy = 0;

DEFINE_MUTEX(poke_mutex);

static struct pm_qos_request_list poke_cpu_req;
static struct pm_qos_request_list cap_cpu_req;

struct kobject *htc_perf_kobj;

static ssize_t cpuiddq_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	u32 reg;
        reg = tegra_fuse_readl(FUSE_CPUIDDQ);
        return sprintf(buf, "0x%x\n", reg);
}

static ssize_t cpuiddq_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	pr_info("[htc_perf] cpuiddq do nothing");
	return 0;
}
htc_perf_attr(cpuiddq);

static ssize_t cpu_temp_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	long temperature;
	struct nct1008_data *thermal_data = get_pwr_data();
	nct1008_thermal_get_temp(thermal_data, &temperature);
	temperature /= 10;
	return sprintf(buf, "%d.%d\n", (int)temperature/100, (int)temperature%100);
}

static ssize_t cpu_temp_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	pr_info("[htc_perf] camera temperature do nothing");
	return 0;
}

htc_perf_attr(cpu_temp);

static ssize_t media_boost_freq_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%c\n", media_boost);
}

static ssize_t media_boost_freq_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	char value;
	struct cpufreq_policy policy;
	sscanf(buf, "%c", &value);

	if ((value == 'n' || value == 'N') &&
			(media_boost == 'n' || media_boost == 'N'))
		return 0;

	if ((value == 'y' || value == 'Y') &&
			(media_boost == 'y' || media_boost == 'Y'))
		return 0;

	if ((value == 'l' || value == 'L') &&
			(media_boost == 'l' || media_boost == 'L'))
		return 0;

	if ((value == 'v' || value == 'V') &&
			(media_boost == 'v' || media_boost == 'V'))
		return 0;

	media_boost = value;

	if (media_boost == 'y' || media_boost == 'Y') {

		/* release user cap */
		htc_get_cpu_user_cap(&orig_user_cap);
		htc_set_cpu_user_cap(0);

		/* To get policy of current cpu */
		cpufreq_get_policy(&policy, smp_processor_id());

		/* update frequency qos request */
		pm_qos_update_request(&poke_cpu_req, (s32)1700000);

		/* update frequency request right now */
		cpufreq_driver_target(&policy,
				1700000, CPUFREQ_RELATION_L);

		pr_info("[htc_perf] Orig user cap is %d,, media_boost is %c",
				orig_user_cap, media_boost);

	} else if (media_boost == 'n' || media_boost == 'N' ) {

		htc_set_cpu_user_cap(orig_user_cap);

		/* restore default cpu frequency request */
		pm_qos_update_request(&poke_cpu_req,
				(s32)PM_QOS_CPU_FREQ_MIN_DEFAULT_VALUE);

		pr_info("[htc_perf] Restore user cap to %d, media_boost is %c",
				orig_user_cap, media_boost);

	} else if (media_boost == 'l' || media_boost == 'L') {
		/* release user cap */
		htc_get_cpu_user_cap(&orig_user_cap);
		htc_set_cpu_user_cap(640000);
		pr_info("[htc_perf] lowP orig user cap is %d, media_boost is %c",
				orig_user_cap, value );
	} else if (media_boost == 'v' || media_boost == 'V') {
		/* release user cap */
		htc_get_cpu_user_cap(&orig_user_cap);
		htc_set_cpu_user_cap(760000);
		pr_info("[htc_perf] video orig user cap is %d, media_boost is %c",
				orig_user_cap, value );
	} else {
		media_boost = 'N';
	}

	return count;
}

htc_perf_attr(media_boost_freq);

static ssize_t power_save_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	char value = 'N';

	if(is_in_power_save)
		value = 'Y';

	if(is_power_save_policy)
		value = 'T';

	return sprintf(buf, "%c\n", value);
}

static ssize_t power_save_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	char value;
	sscanf(buf, "%c", &value);

	switch(value) {
	case 'n':
	case 'N':
		if (is_in_power_save) {
			pr_info("[htc_perf] restore user_cap");
			pm_qos_update_request(&cap_cpu_req,
					(s32)PM_QOS_CPU_FREQ_MAX_DEFAULT_VALUE);
			is_in_power_save = 0;
		}
		break;

	case 'y':
	case 'Y':
		if (!is_in_power_save) {
			pr_info("[htc_perf] set user_cap");
			/* enable user cap */
			pm_qos_update_request(&cap_cpu_req, (s32)powersave_freq);
			is_in_power_save = 1;
		}
		break;
	case 't':
	case 'T':
		if(!is_power_save_policy) {
			pr_info("[htc_perf] set policy cap");
			pm_qos_update_request(&cap_cpu_req, (s32)640000);
			is_power_save_policy = 1;
		}
		break;
	default:
		pr_info("[htc_perf] Default, return;");
		break;
	}

	return count;
}

htc_perf_attr(power_save);

/* For JNI power save policy */
static struct kobj_attribute power_save_policy_attr = {
	.attr	= {
		.name = __stringify(power_save_policy),
		.mode = 0644,
	},
	.show	= power_save_show,
	.store	= power_save_store,
};

static unsigned int cpu_debug_on = 0;

static ssize_t cpu_debug_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", cpu_debug_on);
}

static ssize_t cpu_debug_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	unsigned int value;

	sscanf(buf, "%u", &value);
	cpu_debug_on = value;

	return count;
}

htc_perf_attr(cpu_debug);

unsigned int get_cpu_debug(void)
{
	return cpu_debug_on;
}
EXPORT_SYMBOL(get_cpu_debug);

void restoreCap(int on)
{
	if (is_power_save_policy) {
		if (is_in_power_save) {
			pm_qos_update_request(&cap_cpu_req, (s32)1000000);
		} else {
			pm_qos_update_request(&cap_cpu_req,
						(s32)PM_QOS_CPU_FREQ_MAX_DEFAULT_VALUE);
		}
		is_power_save_policy = 0;
	}
}
EXPORT_SYMBOL(restoreCap);

unsigned int get_powersave_freq(){

    if (is_in_power_save)
        return powersave_freq;

    return 0;
}
EXPORT_SYMBOL(get_powersave_freq);

static struct attribute * g[] = {
	&media_boost_freq_attr.attr,
	&cpu_temp_attr.attr,
	&power_save_attr.attr,
	&cpu_debug_attr.attr,
	&power_save_policy_attr.attr,
        &cpuiddq_attr.attr,
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = g,
};

static int __init htc_perf_init(void)
{
	pr_info("[htc_perf] htc_perf_init\n");
        htc_perf_kobj = kobject_create_and_add("htc", NULL);

	pm_qos_add_request(&poke_cpu_req,
			PM_QOS_CPU_FREQ_MIN,
			(s32)PM_QOS_CPU_FREQ_MIN_DEFAULT_VALUE);
	pm_qos_add_request(&cap_cpu_req,
			PM_QOS_CPU_FREQ_MAX,
			(s32)PM_QOS_CPU_FREQ_MAX_DEFAULT_VALUE);

        if (!htc_perf_kobj)
		return -ENOMEM;

	return sysfs_create_group(htc_perf_kobj, &attr_group);
}
late_initcall(htc_perf_init);
