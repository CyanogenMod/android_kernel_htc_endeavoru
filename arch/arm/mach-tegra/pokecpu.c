/*
 *  Steven C. 2012/2 initial version
 *  HTC Corporation
 */

#include <linux/cpufreq.h>
#include <linux/kobject.h>
#include <linux/mutex.h>
#include <linux/pm_qos_params.h>
#include <linux/workqueue.h>
#include <linux/nct1008.h> /* for thermal temperature */
#include "pokecpu.h"
#include "cpu-tegra.h"

#define POKE_DEBUG_ENABLE 1

#define poke_attr(attrbute) 				\
static struct kobj_attribute attrbute##_attr = {	\
	.attr	= {					\
		.name = __stringify(attrbute),		\
		.mode = 0644,				\
	},						\
	.show	= attrbute##_show,			\
	.store	= attrbute##_store,			\
}

#define DEF_TARGET_FREQ (1500000)
#define DEF_POKE_FREQ (1500000)
#define DEF_POKE_MS (100)
#define DEF_IDLE_MS (300)

static char media_boost = 'N';
static unsigned int orig_user_cap = 0;

static int is_in_power_save = 0;

DEFINE_MUTEX(poke_mutex);

static struct pm_qos_request_list poke_cpu_req;
static struct pm_qos_request_list cap_cpu_req;
struct delayed_work poke_work;
static struct workqueue_struct *poke_work_q;

struct poke_cpu {
	struct kobject kobj;
	unsigned int threshold_freq; /* The desired cpu poke frequency */
	unsigned int poke_freq;      /* The threshold that will poke */
	int continuous;              /* If we will do continuous poke */
	unsigned int poke_period;    /* The period that CPU will be poked*/
	unsigned int idle_period;    /* The poeiro that CPU will be low */
	int is_poking;               /* Flag to check if we are poking */
};

struct poke_cpu pokecpu = {
	.threshold_freq = DEF_TARGET_FREQ,
	.poke_freq   = DEF_POKE_FREQ,
	.continuous  = false,
	.poke_period = DEF_POKE_MS,
	.idle_period = DEF_POKE_MS,
	.is_poking   = false,
};

struct kobject *poke_kobj;

static ssize_t threshold_freq_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", pokecpu.threshold_freq);
}

static ssize_t threshold_freq_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	unsigned int value;
	/*needs to check if we should keep a global variable*/
	sscanf(buf, "%u", &value);
	pokecpu.threshold_freq = value;
	return count;
}

poke_attr(threshold_freq);

static ssize_t poke_freq_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", pokecpu.poke_freq);
}

static ssize_t poke_freq_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	unsigned int value;
	sscanf(buf, "%u", &value);
	pokecpu.poke_freq = value;
	return count;
}

poke_attr(poke_freq);

static ssize_t poke_period_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", pokecpu.poke_period);
}

static ssize_t poke_period_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	unsigned int value;
	sscanf(buf, "%u", &value);
	pokecpu.poke_period = value;
	return count;
}

poke_attr(poke_period);


static ssize_t cpu_temp_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	long temperature;
	struct nct_1008data *thermal_data = get_pwr_data();
	nct1008_thermal_get_temp(thermal_data, &temperature);
	temperature /= 10;
	return sprintf(buf, "%d.%d\n", temperature/100, temperature%100);
}

static ssize_t cpu_temp_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	pr_info("[Poke] camera temperature do nothing");
	return 0;
}

poke_attr(cpu_temp);

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

	media_boost = value;

	if (media_boost == 'y' || media_boost == 'Y') {

		/* release user cap */
		htc_get_cpu_user_cap(&orig_user_cap);
		htc_set_cpu_user_cap(0);

		/* To get policy of current cpu */
		cpufreq_get_policy(&policy, smp_processor_id());

		/* update frequency qos request */
		pm_qos_update_request(&poke_cpu_req, (s32)1500000);

		/* update frequency request right now */
		cpufreq_driver_target(&policy,
				1500000, CPUFREQ_RELATION_L);

		pr_info("[Poke]Orig user cap is %d,, media_boost is %c",
				orig_user_cap, media_boost);

	} else if (media_boost == 'n' || media_boost == 'N' ) {

		htc_set_cpu_user_cap(orig_user_cap);

		/* restore default cpu frequency request */
		pm_qos_update_request(&poke_cpu_req,
				(s32)PM_QOS_CPU_FREQ_MIN_DEFAULT_VALUE);

		pr_info("[Poke]Restore user cap to %d, media_boost is %c",
				orig_user_cap, media_boost);

	} else if (media_boost == 'l' || media_boost == 'L') {
		/* release user cap */
		htc_get_cpu_user_cap(&orig_user_cap);
		htc_set_cpu_user_cap(640000);
		pr_info("[Poke]lowP orig user cap is %d, media_boost is %c",
				orig_user_cap, value );
	} else {
		media_boost = 'N';
	}

	return count;
}

poke_attr(media_boost_freq);


static ssize_t power_save_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	char value = 'N';

	if(is_in_power_save)
		value = 'Y';

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
			pr_info("[Poke] restore user_cap");
			pm_qos_update_request(&cap_cpu_req,
					(s32)PM_QOS_CPU_FREQ_MAX_DEFAULT_VALUE);
			is_in_power_save = 0;
		}
		break;

	case 'y':
	case 'Y':
		if (!is_in_power_save) {
			pr_info("[Poke] set user_cap");
			/* enable user cap */
			pm_qos_update_request(&cap_cpu_req, (s32)1000000);
			is_in_power_save = 1;
		}
		break;
	default:
		pr_info("[Poke] Default, return;");
		break;
	}

	return count;
}

poke_attr(power_save);


static void validatefreq(unsigned int *freq)
{
	int ret;
	struct cpufreq_policy policy;

	ret = cpufreq_get_policy(&policy, 0);

	if(ret) {
		pr_err("unable to get CPU0 policy, errno %d", ret);
		goto error;
	}

	if (*freq > policy.max)
		*freq = policy.max;

	if (*freq < policy.min)
		*freq = policy.min;
error:
	return;
}

/*Call back to touch notifyFinger()*/
static void pokeCPU(void)
{
	int ret;
	int freq, threshold;
	struct cpufreq_policy policy;

	/* To get policy of CPU0 */
	freq = pokecpu.poke_freq;
	threshold = pokecpu.threshold_freq;
	ret = cpufreq_get_policy(&policy, 0);

	if (ret) {
		pr_err("unable to get CPU0 policy, errno %d", ret);
		goto error;
	}

	/* check range of frequency to set*/
	validatefreq(&freq);
	/* check range of threshold of frequency to set*/
	validatefreq(&threshold);

	/* Set frequency based on attributes      */
	/* poke only it's lower than threshold freq. */
	if (policy.cur < threshold) {

		/* Ensure CPU frequency will not drop due to governor check */
		pm_qos_update_request(&poke_cpu_req, (s32)freq);

		pokecpu.is_poking = true;

		ret = cpufreq_driver_target(&policy,
				freq, CPUFREQ_RELATION_L);

		queue_delayed_work(poke_work_q, &poke_work,
			msecs_to_jiffies(pokecpu.poke_period));

#ifdef POKE_DEBUG_ENABLE
		pr_info("[Poke]policy.cur is %d, pokecpu.threshold_freq is %d"
			" pokecpu.poke_freq is %d, freq is %d",
			policy.cur,
			pokecpu.threshold_freq,
			pokecpu.poke_freq,
			freq);
#endif
	}

	if (ret)
		pr_err("unable to set threshold frequency, errno %d", ret);

error:
	return;
}

static void poke_work_fn(struct work_struct *poke_work)
{
	mutex_lock(&poke_mutex);

	/* poke_userspace_min(); */
	pr_info("[Poke] release poke frequency");
	pm_qos_update_request(&poke_cpu_req,
			(s32)PM_QOS_CPU_FREQ_MIN_DEFAULT_VALUE);

	pokecpu.is_poking = false;

	mutex_unlock(&poke_mutex);
}

void restoreCap(int on)
{
	if (!on) {
		pr_info("[Poke] Not first touch!");
		goto finish;
	}
	if (is_in_power_save) {
		pr_info("[Poke] restore user_cap");
		htc_set_cpu_user_cap(orig_user_cap);
		is_in_power_save = 0;
	}
finish:
	return;

}

void speedupCPU(int on)
{
	/*to ensure this function won't be switched.*/
	unsigned int nr_cpus;

	mutex_lock(&poke_mutex);

	if (!on) {
		pr_info("[Poke] Not first touch!");
		goto finish;
	}

	if (pokecpu.is_poking) {
		pr_info("[Poke] Already poked!");
		goto finish;
	}

	nr_cpus = num_online_cpus();
	if ( nr_cpus > 1) {
		pr_info("[Poke] Too many cpus!");
		goto finish;
	}

	pokeCPU();

finish:
	mutex_unlock(&poke_mutex);
	return;
}
EXPORT_SYMBOL(speedupCPU);

static struct attribute * g[] = {
	/* &poke_freq_attr.attr, */
	/* &threshold_freq_attr.attr, */
	/* &poke_period_attr.attr, */
	&media_boost_freq_attr.attr,
	&cpu_temp_attr.attr,
	&power_save_attr.attr,
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = g,
};

static int __init poke_init(void)
{
	pr_info("[Poke] poke init.\n");
        poke_kobj = kobject_create_and_add("htc", NULL);

	poke_work_q = create_singlethread_workqueue("poke_queue");
	INIT_DELAYED_WORK_DEFERRABLE(&poke_work, poke_work_fn);
	pm_qos_add_request(&poke_cpu_req,
			PM_QOS_CPU_FREQ_MIN,
			(s32)PM_QOS_CPU_FREQ_MIN_DEFAULT_VALUE);
	pm_qos_add_request(&cap_cpu_req,
			PM_QOS_CPU_FREQ_MAX,
			(s32)PM_QOS_CPU_FREQ_MAX_DEFAULT_VALUE);

        if (!poke_kobj)
		return -ENOMEM;

	return sysfs_create_group(poke_kobj, &attr_group);
}
late_initcall(poke_init);
