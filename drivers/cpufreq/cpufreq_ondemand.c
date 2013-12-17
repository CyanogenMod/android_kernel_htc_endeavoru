/*
 *  drivers/cpufreq/cpufreq_ondemand.c
 *
 *  Copyright (C)  2001 Russell King
 *            (C)  2003 Venkatesh Pallipadi <venkatesh.pallipadi@intel.com>.
 *                      Jun Nakajima <jun.nakajima@intel.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/cpufreq.h>
#include <linux/cpu.h>
#include <linux/jiffies.h>
#include <linux/kernel_stat.h>
#include <linux/mutex.h>
#include <linux/hrtimer.h>
#include <linux/tick.h>
#include <linux/ktime.h>
#include <linux/sched.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/syscalls.h>
#include <linux/highuid.h>
#include <linux/cpu_debug.h>
#include <linux/kthread.h>

/* Google systrace just supports Interactive governor (option -l)
 * Just backport Interactive trace points for Ondemand governor use
 */
#define CREATE_TRACE_POINTS
#include <trace/events/cpufreq_interactive.h>

/*
 * dbs is used in this file as a shortform for demandbased switching
 * It helps to keep variable names smaller, simpler
 */

#define DEF_FREQUENCY_DOWN_DIFFERENTIAL		(10)
#define DEF_FREQUENCY_UP_THRESHOLD		(80)
#define DEF_SAMPLING_DOWN_FACTOR		(1)
#define MAX_SAMPLING_DOWN_FACTOR		(100000)
#define MICRO_FREQUENCY_DOWN_DIFFERENTIAL	(3)
#define MICRO_FREQUENCY_UP_THRESHOLD		(95)
#define MICRO_FREQUENCY_MIN_SAMPLE_RATE		(10000)
#define MIN_FREQUENCY_UP_THRESHOLD		(11)
#define MAX_FREQUENCY_UP_THRESHOLD		(100)
#define DEF_SAMPLING_RATE			(50000)
#define DEF_IO_IS_BUSY				(1)
#define DEF_UI_DYNAMIC_SAMPLING_RATE		(30000)
#define DEF_UI_COUNTER				(5)
#define DEF_TWO_PHASE_FREQ			(1000000)
#define DEF_TWO_PHASE_BOTTOM_FREQ   (340000)
#define DEF_TWO_PHASE_GO_MAX_LOAD   (95)
#define DEF_UX_LOADING              (30)
#define DEF_UX_FREQ                 (0)
#define DEF_UX_BOOST_THRESHOLD      (0)
#define DEF_INPUT_BOOST_DURATION    (100000000)

/*
 * The polling frequency of this governor depends on the capability of
 * the processor. Default polling frequency is 1000 times the transition
 * latency of the processor. The governor will work on any processor with
 * transition latency <= 10mS, using appropriate sampling
 * rate.
 * For CPUs with transition latency > 10mS (mostly drivers with CPUFREQ_ETERNAL)
 * this governor will not work.
 * All times here are in uS.
 */
#define MIN_SAMPLING_RATE_RATIO			(2)

static unsigned int min_sampling_rate;
static unsigned int def_sampling_rate;

#define LATENCY_MULTIPLIER			(1000)
#define MIN_LATENCY_MULTIPLIER			(100)
#define TRANSITION_LATENCY_LIMIT		(10 * 1000 * 1000)

static void do_dbs_timer(struct work_struct *work);
static int cpufreq_governor_dbs(struct cpufreq_policy *policy,
				unsigned int event);

#ifndef CONFIG_CPU_FREQ_DEFAULT_GOV_ONDEMAND
static
#endif
struct cpufreq_governor cpufreq_gov_ondemand = {
       .name                   = "ondemand",
       .governor               = cpufreq_governor_dbs,
       .max_transition_latency = TRANSITION_LATENCY_LIMIT,
       .owner                  = THIS_MODULE,
};

/* Sampling types */
enum {DBS_NORMAL_SAMPLE, DBS_SUB_SAMPLE};

struct cpu_dbs_info_s {
	cputime64_t prev_cpu_idle;
	cputime64_t prev_cpu_iowait;
	cputime64_t prev_cpu_wall;
	cputime64_t prev_cpu_nice;
	struct cpufreq_policy *cur_policy;
	struct delayed_work work;
	struct cpufreq_frequency_table *freq_table;
	unsigned int freq_lo;
	unsigned int freq_lo_jiffies;
	unsigned int freq_hi_jiffies;
	unsigned int rate_mult;
	int cpu;
	unsigned int sample_type:1;
	/*
	 * percpu mutex that serializes governor limit change with
	 * do_dbs_timer invocation. We do not want do_dbs_timer to run
	 * when user is changing the governor or limits.
	 */
	struct mutex timer_mutex;
};
static DEFINE_PER_CPU(struct cpu_dbs_info_s, od_cpu_dbs_info);

static unsigned int dbs_enable;	/* number of CPUs using this policy */
static unsigned int g_ui_counter = 0;

/*
 * dbs_mutex protects dbs_enable in governor start/stop.
 */
static DEFINE_MUTEX(dbs_mutex);

static struct dbs_tuners {
	unsigned int sampling_rate;
	unsigned int up_threshold;
	unsigned int down_differential;
	unsigned int ignore_nice;
	unsigned int sampling_down_factor;
	unsigned int powersave_bias;
	unsigned int io_is_busy;
#ifdef CONFIG_CPU_FREQ_GOV_ONDEMAND_2_PHASE
	unsigned int two_phase_freq;
    unsigned int two_phase_dynamic;
    unsigned int two_phase_bottom_freq;
#endif
	unsigned int touch_poke;
    unsigned int floor_freq;
	cputime64_t floor_valid_time;
    unsigned int input_boost_duration;
	unsigned int origin_sampling_rate;
	unsigned int ui_sampling_rate;
	unsigned int ui_counter;
    struct {
        unsigned int freq;
        unsigned int loading;
        unsigned int boost_threshold;
    } ux;
} dbs_tuners_ins = {
	.up_threshold = DEF_FREQUENCY_UP_THRESHOLD,
	.sampling_down_factor = DEF_SAMPLING_DOWN_FACTOR,
	.down_differential = DEF_FREQUENCY_DOWN_DIFFERENTIAL,
	.ignore_nice = 0,
	.powersave_bias = 0,
#ifdef CONFIG_CPU_FREQ_GOV_ONDEMAND_2_PHASE
	.two_phase_freq = DEF_TWO_PHASE_FREQ,
    .two_phase_dynamic = 1,
    .two_phase_bottom_freq = DEF_TWO_PHASE_BOTTOM_FREQ,
#endif
	.touch_poke = 1,
    .floor_freq = 0UL,
    .floor_valid_time = 0ULL,
    .input_boost_duration = DEF_INPUT_BOOST_DURATION,
	.ui_sampling_rate = DEF_UI_DYNAMIC_SAMPLING_RATE,
	.ui_counter = DEF_UI_COUNTER,
    .ux = {
        .freq = DEF_UX_FREQ,
        .loading = DEF_UX_LOADING,
        .boost_threshold = DEF_UX_BOOST_THRESHOLD
    },
};

static inline cputime64_t get_cpu_idle_time_jiffy(unsigned int cpu,
							cputime64_t *wall)
{
	cputime64_t idle_time;
	cputime64_t cur_wall_time;
	cputime64_t busy_time;

	cur_wall_time = jiffies64_to_cputime64(get_jiffies_64());
	busy_time = cputime64_add(kstat_cpu(cpu).cpustat.user,
			kstat_cpu(cpu).cpustat.system);

	busy_time = cputime64_add(busy_time, kstat_cpu(cpu).cpustat.irq);
	busy_time = cputime64_add(busy_time, kstat_cpu(cpu).cpustat.softirq);
	busy_time = cputime64_add(busy_time, kstat_cpu(cpu).cpustat.steal);
	busy_time = cputime64_add(busy_time, kstat_cpu(cpu).cpustat.nice);

	idle_time = cputime64_sub(cur_wall_time, busy_time);
	if (wall)
		*wall = (cputime64_t)jiffies_to_usecs(cur_wall_time);

	return (cputime64_t)jiffies_to_usecs(idle_time);
}

static inline cputime64_t get_cpu_idle_time(unsigned int cpu, cputime64_t *wall)
{
	u64 idle_time = get_cpu_idle_time_us(cpu, wall);

	if (idle_time == -1ULL)
		return get_cpu_idle_time_jiffy(cpu, wall);

	return idle_time;
}

static inline cputime64_t get_cpu_iowait_time(unsigned int cpu, cputime64_t *wall)
{
	u64 iowait_time = get_cpu_iowait_time_us(cpu, wall);

	if (iowait_time == -1ULL)
		return 0;

	return iowait_time;
}

/*
 * Find right freq to be set now with powersave_bias on.
 * Returns the freq_hi to be used right now and will set freq_hi_jiffies,
 * freq_lo, and freq_lo_jiffies in percpu area for averaging freqs.
 */
static unsigned int powersave_bias_target(struct cpufreq_policy *policy,
					  unsigned int freq_next,
					  unsigned int relation)
{
	unsigned int freq_req, freq_reduc, freq_avg;
	unsigned int freq_hi, freq_lo;
	unsigned int index = 0;
	unsigned int jiffies_total, jiffies_hi, jiffies_lo;
	struct cpu_dbs_info_s *dbs_info = &per_cpu(od_cpu_dbs_info,
						   policy->cpu);

	if (!dbs_info->freq_table) {
		dbs_info->freq_lo = 0;
		dbs_info->freq_lo_jiffies = 0;
		return freq_next;
	}

	cpufreq_frequency_table_target(policy, dbs_info->freq_table, freq_next,
			relation, &index);
	freq_req = dbs_info->freq_table[index].frequency;
	freq_reduc = freq_req * dbs_tuners_ins.powersave_bias / 1000;
	freq_avg = freq_req - freq_reduc;

	/* Find freq bounds for freq_avg in freq_table */
	index = 0;
	cpufreq_frequency_table_target(policy, dbs_info->freq_table, freq_avg,
			CPUFREQ_RELATION_H, &index);
	freq_lo = dbs_info->freq_table[index].frequency;
	index = 0;
	cpufreq_frequency_table_target(policy, dbs_info->freq_table, freq_avg,
			CPUFREQ_RELATION_L, &index);
	freq_hi = dbs_info->freq_table[index].frequency;

	/* Find out how long we have to be in hi and lo freqs */
	if (freq_hi == freq_lo) {
		dbs_info->freq_lo = 0;
		dbs_info->freq_lo_jiffies = 0;
		return freq_lo;
	}
	jiffies_total = usecs_to_jiffies(dbs_tuners_ins.sampling_rate);
	jiffies_hi = (freq_avg - freq_lo) * jiffies_total;
	jiffies_hi += ((freq_hi - freq_lo) / 2);
	jiffies_hi /= (freq_hi - freq_lo);
	jiffies_lo = jiffies_total - jiffies_hi;
	dbs_info->freq_lo = freq_lo;
	dbs_info->freq_lo_jiffies = jiffies_lo;
	dbs_info->freq_hi_jiffies = jiffies_hi;
	return freq_hi;
}

static void ondemand_powersave_bias_init_cpu(int cpu)
{
	struct cpu_dbs_info_s *dbs_info = &per_cpu(od_cpu_dbs_info, cpu);
	dbs_info->freq_table = cpufreq_frequency_get_table(cpu);
	dbs_info->freq_lo = 0;
}

static void ondemand_powersave_bias_init(void)
{
	int i;
	for_each_online_cpu(i) {
		ondemand_powersave_bias_init_cpu(i);
	}
}

/************************** sysfs interface ************************/

static ssize_t show_sampling_rate_min(struct kobject *kobj,
				      struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", min_sampling_rate);
}

define_one_global_ro(sampling_rate_min);

/* cpufreq_ondemand Governor Tunables */
#define show_one(file_name, object)					\
static ssize_t show_##file_name						\
(struct kobject *kobj, struct attribute *attr, char *buf)              \
{									\
	return sprintf(buf, "%u\n", dbs_tuners_ins.object);		\
}
show_one(sampling_rate, sampling_rate);
show_one(io_is_busy, io_is_busy);
show_one(up_threshold, up_threshold);
show_one(sampling_down_factor, sampling_down_factor);
show_one(down_differential, down_differential);
show_one(ignore_nice_load, ignore_nice);
show_one(powersave_bias, powersave_bias);
#ifdef CONFIG_CPU_FREQ_GOV_ONDEMAND_2_PHASE
show_one(two_phase_freq, two_phase_freq);
show_one(two_phase_dynamic, two_phase_dynamic);
show_one(two_phase_bottom_freq, two_phase_bottom_freq);
#endif
show_one(touch_poke, touch_poke);
show_one(input_boost_duration, input_boost_duration);
show_one(ui_sampling_rate, ui_sampling_rate);
show_one(ui_counter, ui_counter);
show_one(ux_freq, ux.freq);
show_one(ux_loading, ux.loading);
show_one(ux_boost_threshold, ux.boost_threshold);
static ssize_t store_sampling_rate(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;
	dbs_tuners_ins.sampling_rate = max(input, min_sampling_rate);
	dbs_tuners_ins.origin_sampling_rate = dbs_tuners_ins.sampling_rate;
	return count;
}
#ifdef CONFIG_CPU_FREQ_GOV_ONDEMAND_2_PHASE
static ssize_t store_two_phase_freq(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	dbs_tuners_ins.two_phase_freq = input;

	return count;
}

static ssize_t store_two_phase_dynamic (
   struct kobject *a,
   struct attribute *b,
   const char *buf,
   size_t count
   )
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	dbs_tuners_ins.two_phase_dynamic = input;

	return count;
}

static ssize_t store_two_phase_bottom_freq (
   struct kobject *a,
   struct attribute *b,
   const char *buf,
   size_t count
   )
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	dbs_tuners_ins.two_phase_bottom_freq = input;

	return count;
}
#endif

static unsigned int Touch_poke_attr[4] = {1500000, 880000, 0, 0};

static ssize_t store_touch_poke(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	int ret;
	ret = sscanf(buf, "%u,%u,%u,%u", &Touch_poke_attr[0], &Touch_poke_attr[1],
		&Touch_poke_attr[2], &Touch_poke_attr[3]);
	if (ret < 4)
		return -EINVAL;

	if(Touch_poke_attr[0] == 0)
		dbs_tuners_ins.touch_poke = 0;
	else
		dbs_tuners_ins.touch_poke = 1;

	return count;
}

static ssize_t store_input_boost_duration(
   struct kobject *a,
   struct attribute *b,
   const char *buf,
   size_t count
   ) {
    unsigned int input;
    int ret;
    ret = sscanf(buf, "%u", &input);
    if (ret != 1)
        return -EINVAL;

    dbs_tuners_ins.input_boost_duration = input;

    return count;
}

static ssize_t store_ui_sampling_rate(struct kobject *a, struct attribute *b,
					const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	dbs_tuners_ins.ui_sampling_rate = max(input, min_sampling_rate);

	return count;
}

static ssize_t store_io_is_busy(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	unsigned int input;
	int ret;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;
	dbs_tuners_ins.io_is_busy = !!input;
	return count;
}

static ssize_t store_up_threshold(struct kobject *a, struct attribute *b,
				  const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1 || input > MAX_FREQUENCY_UP_THRESHOLD ||
			input < MIN_FREQUENCY_UP_THRESHOLD) {
		return -EINVAL;
	}
	dbs_tuners_ins.up_threshold = input;
	return count;
}

static ssize_t store_down_differential(struct kobject *a, struct attribute *b,
					const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if(ret != 1 || input > DEF_FREQUENCY_DOWN_DIFFERENTIAL ||
			input < MICRO_FREQUENCY_DOWN_DIFFERENTIAL) {
		return -EINVAL;
	}
	dbs_tuners_ins.down_differential = input;
	return count;
}

static ssize_t store_sampling_down_factor(struct kobject *a,
			struct attribute *b, const char *buf, size_t count)
{
	unsigned int input, j;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1 || input > MAX_SAMPLING_DOWN_FACTOR || input < 1)
		return -EINVAL;
	dbs_tuners_ins.sampling_down_factor = input;

	/* Reset down sampling multiplier in case it was active */
	for_each_online_cpu(j) {
		struct cpu_dbs_info_s *dbs_info;
		dbs_info = &per_cpu(od_cpu_dbs_info, j);
		dbs_info->rate_mult = 1;
	}
	return count;
}

static ssize_t store_ignore_nice_load(struct kobject *a, struct attribute *b,
				      const char *buf, size_t count)
{
	unsigned int input;
	int ret;

	unsigned int j;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	if (input > 1)
		input = 1;

	if (input == dbs_tuners_ins.ignore_nice) { /* nothing to do */
		return count;
	}
	dbs_tuners_ins.ignore_nice = input;

	/* we need to re-evaluate prev_cpu_idle */
	for_each_online_cpu(j) {
		struct cpu_dbs_info_s *dbs_info;
		dbs_info = &per_cpu(od_cpu_dbs_info, j);
		dbs_info->prev_cpu_idle = get_cpu_idle_time(j,
						&dbs_info->prev_cpu_wall);
		if (dbs_tuners_ins.ignore_nice)
			dbs_info->prev_cpu_nice = kstat_cpu(j).cpustat.nice;

	}
	return count;
}

static ssize_t store_powersave_bias(struct kobject *a, struct attribute *b,
				    const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1)
		return -EINVAL;

	if (input > 1000)
		input = 1000;

	dbs_tuners_ins.powersave_bias = input;
	ondemand_powersave_bias_init();
	return count;
}

static ssize_t store_ui_counter(struct kobject *a, struct attribute *b,
				const char *buf, size_t count)
{
	unsigned int input;
	int ret;

	ret = sscanf(buf, "%u", &input);
	if(ret != 1)
		return -EINVAL;

	dbs_tuners_ins.ui_counter = input;
	return count;
}

static ssize_t store_ux_freq (
   struct kobject *a,
   struct attribute *b,
   const char *buf,
   size_t count
   )
{
	unsigned int input;
	int ret;

	ret = sscanf(buf, "%u", &input);
	if(ret != 1)
		return -EINVAL;

	dbs_tuners_ins.ux.freq = input;
	return count;
}

static ssize_t store_ux_loading (
   struct kobject *a,
   struct attribute *b,
   const char *buf,
   size_t count
   )
{
	unsigned int input;
	int ret;

	ret = sscanf(buf, "%u", &input);
	if(ret != 1)
		return -EINVAL;

	dbs_tuners_ins.ux.loading = input;
	return count;
}

static ssize_t store_ux_boost_threshold (
   struct kobject *a,
   struct attribute *b,
   const char *buf,
   size_t count
   )
{
	unsigned int input;
	int ret;

	ret = sscanf(buf, "%u", &input);
	if(ret != 1)
		return -EINVAL;

	dbs_tuners_ins.ux.boost_threshold = input;
	return count;
}

define_one_global_rw(sampling_rate);
define_one_global_rw(io_is_busy);
define_one_global_rw(up_threshold);
define_one_global_rw(down_differential);
define_one_global_rw(sampling_down_factor);
define_one_global_rw(ignore_nice_load);
define_one_global_rw(powersave_bias);
#ifdef CONFIG_CPU_FREQ_GOV_ONDEMAND_2_PHASE
define_one_global_rw(two_phase_freq);
define_one_global_rw(two_phase_dynamic);
define_one_global_rw(two_phase_bottom_freq);
#endif
define_one_global_rw(touch_poke);
define_one_global_rw(input_boost_duration);
define_one_global_rw(ui_sampling_rate);
define_one_global_rw(ui_counter);
define_one_global_rw(ux_freq);
define_one_global_rw(ux_loading);
define_one_global_rw(ux_boost_threshold);

static struct attribute *dbs_attributes[] = {
	&sampling_rate_min.attr,
	&sampling_rate.attr,
	&up_threshold.attr,
	&down_differential.attr,
	&sampling_down_factor.attr,
	&ignore_nice_load.attr,
	&powersave_bias.attr,
	&io_is_busy.attr,
#ifdef CONFIG_CPU_FREQ_GOV_ONDEMAND_2_PHASE
	&two_phase_freq.attr,
    &two_phase_dynamic.attr,
    &two_phase_bottom_freq.attr,
#endif
	&touch_poke.attr,
    &input_boost_duration.attr,
	&ui_sampling_rate.attr,
	&ui_counter.attr,
    &ux_freq.attr,
    &ux_loading.attr,
    &ux_boost_threshold.attr,
	NULL
};

static struct attribute_group dbs_attr_group = {
	.attrs = dbs_attributes,
	.name = "ondemand",
};

/************************** sysfs end ************************/

static void dbs_freq_increase(struct cpufreq_policy *p, unsigned int load, unsigned int freq)
{
	if (dbs_tuners_ins.powersave_bias && freq > 475000 )
		freq = powersave_bias_target(p, freq, CPUFREQ_RELATION_H);
	//else if (p->cur == p->max)
	//	return;

    trace_cpufreq_interactive_target (p->cpu, load, p->cur, freq);

	__cpufreq_driver_target(p, freq, dbs_tuners_ins.powersave_bias ?
			CPUFREQ_RELATION_L : CPUFREQ_RELATION_H);

    trace_cpufreq_interactive_up (p->cpu, freq, p->cur);
}

static void dbs_check_cpu(struct cpu_dbs_info_s *this_dbs_info)
{
	unsigned int max_load_freq;
	unsigned int debug_freq;
	unsigned int debug_load;
	unsigned int debug_iowait;

	struct cpufreq_policy *policy;
	unsigned int j;
#ifdef CONFIG_CPU_FREQ_GOV_ONDEMAND_2_PHASE
	static unsigned int phase = 0;
	static unsigned int counter = 0;
    bool mid_idle_busy = false;
#endif
    unsigned int final_up_threshold = dbs_tuners_ins.up_threshold;
    cputime64_t now = ktime_to_ns (ktime_get ());

	this_dbs_info->freq_lo = 0;
	policy = this_dbs_info->cur_policy;

	/*
	 * Every sampling_rate, we check, if current idle time is less
	 * than 20% (default), then we try to increase frequency
	 * Every sampling_rate, we look for a the lowest
	 * frequency which can sustain the load while keeping idle time over
	 * 30%. If such a frequency exist, we try to decrease to this frequency.
	 *
	 * Any frequency increase takes it to the maximum frequency.
	 * Frequency reduction happens at minimum steps of
	 * 5% (default) of current frequency
	 */

	/* Get Absolute Load - in terms of freq */
	max_load_freq = 0;

	for_each_cpu(j, policy->cpus) {
		struct cpu_dbs_info_s *j_dbs_info;
		cputime64_t cur_wall_time, cur_idle_time, cur_iowait_time;
		unsigned int idle_time, wall_time, iowait_time;
		unsigned int load, load_freq;
		int freq_avg;

		j_dbs_info = &per_cpu(od_cpu_dbs_info, j);

		cur_idle_time = get_cpu_idle_time(j, &cur_wall_time);
		cur_iowait_time = get_cpu_iowait_time(j, &cur_wall_time);

		wall_time = (unsigned int) cputime64_sub(cur_wall_time,
				j_dbs_info->prev_cpu_wall);
		j_dbs_info->prev_cpu_wall = cur_wall_time;

		idle_time = (unsigned int) cputime64_sub(cur_idle_time,
				j_dbs_info->prev_cpu_idle);
		j_dbs_info->prev_cpu_idle = cur_idle_time;

		iowait_time = (unsigned int) cputime64_sub(cur_iowait_time,
				j_dbs_info->prev_cpu_iowait);
		j_dbs_info->prev_cpu_iowait = cur_iowait_time;

		if (dbs_tuners_ins.ignore_nice) {
			cputime64_t cur_nice;
			unsigned long cur_nice_jiffies;

			cur_nice = cputime64_sub(kstat_cpu(j).cpustat.nice,
					 j_dbs_info->prev_cpu_nice);
			/*
			 * Assumption: nice time between sampling periods will
			 * be less than 2^32 jiffies for 32 bit sys
			 */
			cur_nice_jiffies = (unsigned long)
					cputime64_to_jiffies64(cur_nice);

			j_dbs_info->prev_cpu_nice = kstat_cpu(j).cpustat.nice;
			idle_time += jiffies_to_usecs(cur_nice_jiffies);
		}

		/*
		 * For the purpose of ondemand, waiting for disk IO is an
		 * indication that you're performance critical, and not that
		 * the system is actually idle. So subtract the iowait time
		 * from the cpu idle time.
		 */

		if (dbs_tuners_ins.io_is_busy && idle_time >= iowait_time)
			idle_time -= iowait_time;

		if (unlikely(!wall_time || wall_time < idle_time))
			continue;

		load = 100 * (wall_time - idle_time) / wall_time;

		freq_avg = __cpufreq_driver_getavg(policy, j);
		if (freq_avg <= 0)
			freq_avg = policy->cur;

		load_freq = load * freq_avg;
		if (load_freq > max_load_freq) {
			max_load_freq = load_freq;
			debug_load = load;
			debug_iowait = 100 * iowait_time / wall_time;
		}
	}

	if (g_ui_counter > 0){
		g_ui_counter--;
		if(g_ui_counter == 0)
			dbs_tuners_ins.sampling_rate = dbs_tuners_ins.origin_sampling_rate;
	}

    /* catch up with min. freq asap */
    if (policy->cur < policy->min) {
        dbs_freq_increase(policy, debug_load, policy->min);

        CPU_DEBUG_PRINTK(CPU_DEBUG_GOVERNOR,
                         " cpu%d,"
                         " load=%3u, iowait=%3u,"
                         " freq=%7u(%7u), counter=%d, phase=%d, min_freq=%7u",
                         policy->cpu,
                         debug_load, debug_iowait,
                         policy->min, policy->cur, counter, phase, policy->min);
        return;
    }

    /* make boost up to phase 1 from quite low speed more easier */
    if (policy->cur < dbs_tuners_ins.ux.freq &&
        dbs_tuners_ins.ux.boost_threshold > 0)
    {
        final_up_threshold = dbs_tuners_ins.ux.boost_threshold;
    }

	/* Check for frequency increase */
	if (max_load_freq > final_up_threshold * policy->cur) {
		/* If switching to max speed, apply sampling_down_factor */
#ifndef CONFIG_CPU_FREQ_GOV_ONDEMAND_2_PHASE
		if (policy->cur < policy->max)
			this_dbs_info->rate_mult =
				dbs_tuners_ins.sampling_down_factor;
		debug_freq = policy->max;

		dbs_freq_increase(policy, debug_load, policy->max);

        CPU_DEBUG_PRINTK(CPU_DEBUG_GOVERNOR,
                         " cpu%d,"
                         " load=%3u, iowait=%3u,"
                         " freq=%7u(%7u), min_freq=%7u",
                         policy->cpu,
                         debug_load, debug_iowait,
                         debug_freq, policy->cur, policy->min);

#else
		if (counter < 5) {
			counter++;
			if (counter > 2) {
                if (!phase && dbs_tuners_ins.two_phase_dynamic)
                    mid_idle_busy = true;

				/* change to busy phase */
				phase = 1;

            } else {
                /* set/re-set to idle phase */
                phase = 0;
            }
		}

        /* idle phase */
		if (dbs_tuners_ins.two_phase_freq != 0 && phase == 0) {
			debug_freq = dbs_tuners_ins.two_phase_freq;

            if (dbs_tuners_ins.two_phase_dynamic) {
                /* scale UP by 2 */
                unsigned int scaled_freq = policy->cur << 1;
                unsigned int idx = 0;

                /* align to bottom freq. first */
                if (scaled_freq < dbs_tuners_ins.two_phase_bottom_freq)
                    scaled_freq = dbs_tuners_ins.two_phase_bottom_freq;

				/* aligned with low relation and see if
                 * it's worth in replace of the original two phase speed
                 */
                if (!cpufreq_frequency_table_target (
                        policy,
                        this_dbs_info->freq_table,
                        scaled_freq,
                        CPUFREQ_RELATION_L,
                        &idx) &&
                    this_dbs_info->freq_table[idx].frequency < debug_freq)
                {
                    /* Good, get power saved even a bit for a short while */
                    debug_freq = this_dbs_info->freq_table[idx].frequency;
                }
            }

            /* NEVER less than current speed */
            if (debug_freq < policy->cur)
                if (debug_load > DEF_TWO_PHASE_GO_MAX_LOAD)
                    debug_freq = policy->max;
                else
                    debug_freq = policy->cur;

        /* busy phase */
		} else {
            debug_freq = policy->max;

            if (dbs_tuners_ins.two_phase_dynamic && mid_idle_busy) {
                /* mid of two phases */
                unsigned int mid_freq =
                    (dbs_tuners_ins.two_phase_freq + policy->max) >> 1;
                unsigned int idx = 0;

                if (mid_freq > policy->cur &&
                    !cpufreq_frequency_table_target (
                       policy,
                       this_dbs_info->freq_table,
                       mid_freq,
                       CPUFREQ_RELATION_L,
                       &idx))
                {
                    /* Good, get power saved even a bit for a short while */
                    debug_freq = this_dbs_info->freq_table[idx].frequency;
                }

            } else {
                if (policy->cur < policy->max)
                    this_dbs_info->rate_mult =
					    dbs_tuners_ins.sampling_down_factor;
            }
		}

        dbs_freq_increase(policy, debug_load, debug_freq);

        CPU_DEBUG_PRINTK(CPU_DEBUG_GOVERNOR,
                         " cpu%d,"
                         " load=%3u, iowait=%3u,"
                         " freq=%7u(%7u), counter=%d, phase=%d, min_freq=%7u",
                         policy->cpu,
                         debug_load, debug_iowait,
                         debug_freq, policy->cur, counter, phase, policy->min);

#endif
		return;
	}

    if (time_before64 (now, dbs_tuners_ins.floor_valid_time)) {
        trace_cpufreq_interactive_notyet (policy->cpu,
                                          debug_load,
                                          policy->cur,
                                          policy->cur);
        return;
    }

#ifdef CONFIG_CPU_FREQ_GOV_ONDEMAND_2_PHASE
	if (counter > 0) {
		counter--;
		if (counter == 0) {
			/* change to idle phase */
			phase = 0;
		}
	}
#endif
	/* Check for frequency decrease */
	/* if we cannot reduce the frequency anymore, break out early */
	if (policy->cur == policy->min) {
        trace_cpufreq_interactive_already (policy->cpu,
                                           debug_load,
                                           policy->cur,
                                           policy->cur);
		return;
    }

	/*
	 * The optimal frequency is the frequency that is the lowest that
	 * can support the current CPU usage without triggering the up
	 * policy. To be safe, we focus 10 points under the threshold.
	 */
	if (max_load_freq <
	    (dbs_tuners_ins.up_threshold - dbs_tuners_ins.down_differential) *
	     policy->cur) {
		unsigned int freq_next;
		freq_next = max_load_freq /
				(dbs_tuners_ins.up_threshold -
				 dbs_tuners_ins.down_differential);

		/* No longer fully busy, reset rate_mult */
		this_dbs_info->rate_mult = 1;

		if (freq_next < policy->min)
			freq_next = policy->min;

        /* NEVER go below ux_freq if current loading > ux_loading for UX sake */
        if (freq_next < dbs_tuners_ins.ux.freq &&
            debug_load > dbs_tuners_ins.ux.loading)
            freq_next = dbs_tuners_ins.ux.freq;

		if (!dbs_tuners_ins.powersave_bias) {
			debug_freq = freq_next;

            trace_cpufreq_interactive_target (policy->cpu,
                                              debug_load,
                                              policy->cur,
                                              freq_next);

			__cpufreq_driver_target(policy, freq_next,
					CPUFREQ_RELATION_L);
		} else {
			int freq = 0;
			if( freq_next > 475000 )
			{
				freq = powersave_bias_target(policy, freq_next,
					CPUFREQ_RELATION_L);
				debug_freq = freq;
			}
			else
				debug_freq = freq = freq_next;

            trace_cpufreq_interactive_target (policy->cpu,
                                              debug_load,
                                              policy->cur,
                                              freq);

			__cpufreq_driver_target(policy, freq,
				CPUFREQ_RELATION_L);
		}

        trace_cpufreq_interactive_down (policy->cpu, debug_freq, policy->cur);

#ifdef CONFIG_CPU_FREQ_GOV_ONDEMAND_2_PHASE
        CPU_DEBUG_PRINTK(CPU_DEBUG_GOVERNOR,
                         " cpu%d,"
                         " load=%3u, iowait=%3u,"
                         " freq=%7u, %7u(%7u), counter=%d, phase=%d, min_freq=%7u",
                         policy->cpu,
                         debug_load, debug_iowait,
                         freq_next, debug_freq, policy->cur, counter, phase, policy->min);
#else
        CPU_DEBUG_PRINTK(CPU_DEBUG_GOVERNOR,
                         " cpu%d,"
                         " load=%3u, iowait=%3u,"
                         " freq=%7u(%7u), min_freq=%7u",
                         policy->cpu,
                         debug_load, debug_iowait,
                         debug_freq, policy->cur, policy->min);

#endif
	}
}

static void do_dbs_timer(struct work_struct *work)
{
	struct cpu_dbs_info_s *dbs_info =
		container_of(work, struct cpu_dbs_info_s, work.work);
	unsigned int cpu = dbs_info->cpu;
	int sample_type = dbs_info->sample_type;

	int delay;

	mutex_lock(&dbs_info->timer_mutex);

	/* Common NORMAL_SAMPLE setup */
	dbs_info->sample_type = DBS_NORMAL_SAMPLE;
	if (!dbs_tuners_ins.powersave_bias ||
	    sample_type == DBS_NORMAL_SAMPLE) {
		dbs_check_cpu(dbs_info);
		if (dbs_info->freq_lo) {
			/* Setup timer for SUB_SAMPLE */
			dbs_info->sample_type = DBS_SUB_SAMPLE;
			delay = dbs_info->freq_hi_jiffies;
		} else {
			/* We want all CPUs to do sampling nearly on
			 * same jiffy
			 */
			delay = usecs_to_jiffies(dbs_tuners_ins.sampling_rate
				* dbs_info->rate_mult);

			if (num_online_cpus() > 1)
				delay -= jiffies % delay;
		}
	} else {
		__cpufreq_driver_target(dbs_info->cur_policy,
			dbs_info->freq_lo, CPUFREQ_RELATION_H);
		delay = dbs_info->freq_lo_jiffies;
	}
	schedule_delayed_work_on(cpu, &dbs_info->work, delay);
	mutex_unlock(&dbs_info->timer_mutex);
}

static inline void dbs_timer_init(struct cpu_dbs_info_s *dbs_info)
{
	/* We want all CPUs to do sampling nearly on same jiffy */
	int delay = usecs_to_jiffies(dbs_tuners_ins.sampling_rate);

	if (num_online_cpus() > 1)
		delay -= jiffies % delay;

	dbs_info->sample_type = DBS_NORMAL_SAMPLE;
	INIT_DELAYED_WORK_DEFERRABLE(&dbs_info->work, do_dbs_timer);
	schedule_delayed_work_on(dbs_info->cpu, &dbs_info->work, delay);
}

static inline void dbs_timer_exit(struct cpu_dbs_info_s *dbs_info)
{
	cancel_delayed_work_sync(&dbs_info->work);
}

/*
 * Not all CPUs want IO time to be accounted as busy; this dependson how
 * efficient idling at a higher frequency/voltage is.
 * Pavel Machek says this is not so for various generations of AMD and old
 * Intel systems.
 * Mike Chan (androidlcom) calis this is also not true for ARM.
 * Because of this, whitelist specific known (series) of CPUs by default, and
 * leave all others up to the user.
 */
static int should_io_be_busy(void)
{
#if defined(CONFIG_X86)
	/*
	 * For Intel, Core 2 (model 15) andl later have an efficient idle.
	 */
	if (boot_cpu_data.x86_vendor == X86_VENDOR_INTEL &&
	    boot_cpu_data.x86 == 6 &&
	    boot_cpu_data.x86_model >= 15)
		return 1;
#endif
	return DEF_IO_IS_BUSY;
}

#define	AID_SYSTEM	(1000)
static void dbs_chown(void)
{
	int ret;

	ret = sys_chown("/sys/devices/system/cpu/cpufreq/ondemand/ignore_nice_load", low2highuid(AID_SYSTEM), low2highgid(0));
	if (ret)
		pr_warn("sys_chown ignore_nice_load returns: %d", ret);

	ret = sys_chown("/sys/devices/system/cpu/cpufreq/ondemand/io_is_busy", low2highuid(AID_SYSTEM), low2highgid(0));
	if (ret)
		pr_warn("sys_chown io_is_busy returns: %d", ret);

	ret = sys_chown("/sys/devices/system/cpu/cpufreq/ondemand/powersave_bias", low2highuid(AID_SYSTEM), low2highgid(0));
	if (ret)
		pr_warn("sys_chown powersave_bias returns: %d", ret);

	ret = sys_chown("/sys/devices/system/cpu/cpufreq/ondemand/sampling_down_factor", low2highuid(AID_SYSTEM), low2highgid(0));
	if (ret)
		pr_warn("sys_chown sampling_down_factor returns: %d", ret);

	ret = sys_chown("/sys/devices/system/cpu/cpufreq/ondemand/sampling_rate", low2highuid(AID_SYSTEM), low2highgid(0));
	if (ret)
		pr_warn("sys_chown sampling_rate returns: %d", ret);

	ret = sys_chown("/sys/devices/system/cpu/cpufreq/ondemand/two_phase_freq", low2highuid(AID_SYSTEM), low2highgid(0));
	if (ret)
		pr_warn("sys_chown two_phase_freq returns: %d", ret);

    ret = sys_chown("/sys/devices/system/cpu/cpufreq/ondemand/two_phase_dynamic", low2highuid(AID_SYSTEM), low2highgid(0));
	if (ret)
		pr_warn("sys_chown two_phase_dynamic returns: %d", ret);

    ret = sys_chown("/sys/devices/system/cpu/cpufreq/ondemand/two_phase_bottom_freq", low2highuid(AID_SYSTEM), low2highgid(0));
	if (ret)
		pr_warn("sys_chown two_phase_bottom_freq returns: %d", ret);

	ret = sys_chown("/sys/devices/system/cpu/cpufreq/ondemand/up_threshold", low2highuid(AID_SYSTEM), low2highgid(0));
	if (ret)
		pr_warn("sys_chown up_threshold returns: %d", ret);

	ret = sys_chown("/sys/devices/system/cpu/cpufreq/ondemand/down_differential", low2highuid(AID_SYSTEM), low2highgid(0));
	if (ret)
		pr_warn("sys_chown down_differential returns: %d", ret);

	ret = sys_chown("/sys/devices/system/cpu/cpufreq/ondemand/touch_poke", low2highuid(AID_SYSTEM), low2highgid(0));
	if (ret)
		pr_warn("sys_chown touch_poke returns: %d", ret);

    ret = sys_chown("/sys/devices/system/cpu/cpufreq/ondemand/input_boost_duration", low2highuid(AID_SYSTEM), low2highgid(0));
	if (ret)
		pr_warn("sys_chown input_boost_duration returns: %d", ret);

	ret = sys_chown("/sys/devices/system/cpu/cpufreq/ondemand/ui_sampling_rate", low2highuid(AID_SYSTEM), low2highgid(0));
	if (ret)
		pr_warn("sys_chown ui_sampling_rate returns: %d", ret);

	ret = sys_chown("/sys/devices/system/cpu/cpufreq/ondemand/ui_counter", low2highuid(AID_SYSTEM), low2highgid(0));
	if (ret)
		pr_warn("sys_chown ui_counter returns: %d", ret);

    ret = sys_chown("/sys/devices/system/cpu/cpufreq/ondemand/ux_freq", low2highuid(AID_SYSTEM), low2highgid(0));
	if (ret)
		pr_err("sys_chown ux_freq error: %d", ret);

    ret = sys_chown("/sys/devices/system/cpu/cpufreq/ondemand/ux_loading", low2highuid(AID_SYSTEM), low2highgid(0));
	if (ret)
		pr_err("sys_chown ux_loading error: %d", ret);

    ret = sys_chown("/sys/devices/system/cpu/cpufreq/ondemand/ux_boost_threshold", low2highuid(AID_SYSTEM), low2highgid(0));
	if (ret)
		pr_err("sys_chown ux_boost_threshold error: %d", ret);
}

static void dbs_refresh_callback_ondemand(struct work_struct *unused)
{
	struct cpufreq_policy *policy;
	struct cpu_dbs_info_s *this_dbs_info;
	unsigned int nr_cpus;
	unsigned int touch_poke_freq;
	unsigned int cpu = smp_processor_id();

	if (lock_policy_rwsem_write(cpu) < 0)
		return;

	this_dbs_info = &per_cpu(od_cpu_dbs_info, cpu);
	policy = this_dbs_info->cur_policy;

	g_ui_counter = dbs_tuners_ins.ui_counter;
	if(dbs_tuners_ins.ui_counter > 0)
		dbs_tuners_ins.sampling_rate = dbs_tuners_ins.ui_sampling_rate;

	/* We poke the frequency base on the online cpu number */
	nr_cpus = num_online_cpus();

	touch_poke_freq = Touch_poke_attr[nr_cpus-1];

	if(touch_poke_freq == 0 || (policy && policy->cur >= touch_poke_freq)) {
		unlock_policy_rwsem_write(cpu);
		return;
	}

    if (policy) {
        __cpufreq_driver_target(policy, touch_poke_freq,
            CPUFREQ_RELATION_L);
        this_dbs_info->prev_cpu_idle = get_cpu_idle_time(cpu,
            &this_dbs_info->prev_cpu_wall);
    }

	unlock_policy_rwsem_write(cpu);
}

#if defined(CONFIG_BEST_TRADE_HOTPLUG)
extern void bthp_set_floor_cap (unsigned int floor_freq,
                                cputime64_t floor_time
                                );
#endif

static DECLARE_WORK(dbs_refresh_work, dbs_refresh_callback_ondemand);

extern
int tegra_input_boost (
   int cpu,
   unsigned int target_freq
   );

static bool boost_task_alive = false;
static struct task_struct *input_boost_task;

static int cpufreq_ondemand_input_boost_task (
   void *data
   )
{
    struct cpufreq_policy *policy;
    struct cpu_dbs_info_s *this_dbs_info;
    unsigned int nr_cpus;
    unsigned int touch_poke_freq;
    unsigned int cpu;

    while (1) {
        set_current_state(TASK_INTERRUPTIBLE);
        schedule();

        if (kthread_should_stop())
            break;

        set_current_state(TASK_RUNNING);
        cpu = smp_processor_id();

        /* We poke the frequency base on the online cpu number */
        nr_cpus = num_online_cpus();
        touch_poke_freq = Touch_poke_attr[nr_cpus - 1];

        /* boost ASAP */
        if (!touch_poke_freq ||
            tegra_input_boost(cpu, touch_poke_freq) < 0)
            continue;

        dbs_tuners_ins.floor_freq = touch_poke_freq;
        dbs_tuners_ins.floor_valid_time =
            ktime_to_ns(ktime_get()) + dbs_tuners_ins.input_boost_duration;

#if defined(CONFIG_BEST_TRADE_HOTPLUG)
        bthp_set_floor_cap (dbs_tuners_ins.floor_freq,
                            dbs_tuners_ins.floor_valid_time);
#endif

        if (lock_policy_rwsem_write(cpu) < 0)
            continue;

        this_dbs_info = &per_cpu(od_cpu_dbs_info, cpu);
        if (this_dbs_info) {
            policy = this_dbs_info->cur_policy;

            g_ui_counter = dbs_tuners_ins.ui_counter;
            if (dbs_tuners_ins.ui_counter > 0)
                dbs_tuners_ins.sampling_rate = dbs_tuners_ins.ui_sampling_rate;

            if (policy) {
                this_dbs_info->prev_cpu_idle =
                   get_cpu_idle_time(cpu,
                                     &this_dbs_info->prev_cpu_wall);
            }
        }

        unlock_policy_rwsem_write(cpu);
    }

    return 0;
}

static void dbs_input_event(struct input_handle *handle, unsigned int type,
		unsigned int code, int value)
{
	if (dbs_tuners_ins.touch_poke && type == EV_SYN && code == SYN_REPORT) {
		/*schedule_work(&dbs_refresh_work);*/

        if (boost_task_alive)
            wake_up_process (input_boost_task);
    }
}

static int input_dev_filter(const char* input_dev_name)
{
	int ret = 0;
	if (strstr(input_dev_name, "touchscreen") ||
		strstr(input_dev_name, "-keypad") ||
		strstr(input_dev_name, "-nav") ||
		strstr(input_dev_name, "-oj")) {
	}
	else {
		ret = 1;
	}
	return ret;
}


static int dbs_input_connect(struct input_handler *handler,
		struct input_dev *dev, const struct input_device_id *id)
{
	struct input_handle *handle;
	int error;

	/* filter out those input_dev that we don't care */
	if (input_dev_filter(dev->name))
		return 0;

	handle = kzalloc(sizeof(struct input_handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = "cpufreq";

	error = input_register_handle(handle);
	if (error)
		goto err2;

	error = input_open_device(handle);
	if (error)
		goto err1;

	return 0;
err1:
	input_unregister_handle(handle);
err2:
	kfree(handle);
	return error;
}

static void dbs_input_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static const struct input_device_id dbs_ids[] = {
	{ .driver_info = 1 },
	{ },
};
static struct input_handler dbs_input_handler = {
	.event          = dbs_input_event,
	.connect        = dbs_input_connect,
	.disconnect     = dbs_input_disconnect,
	.name           = "cpufreq_ond",
	.id_table       = dbs_ids,
};

static int cpufreq_governor_dbs(struct cpufreq_policy *policy,
				   unsigned int event)
{
	unsigned int cpu = policy->cpu;
	struct cpu_dbs_info_s *this_dbs_info;
	unsigned int j;
	int rc;
    struct sched_param param = { .sched_priority = 1 };

	this_dbs_info = &per_cpu(od_cpu_dbs_info, cpu);

	switch (event) {
	case CPUFREQ_GOV_START:
		if ((!cpu_online(cpu)) || (!policy->cur))
			return -EINVAL;

		mutex_lock(&dbs_mutex);

        if (!cpu) {
            if (!boost_task_alive) {
                input_boost_task = kthread_create (
                   cpufreq_ondemand_input_boost_task,
                   NULL,
                   "kinputboostd"
                   );

                if (IS_ERR(input_boost_task)) {
                    mutex_unlock(&dbs_mutex);
                    return PTR_ERR(input_boost_task);
                }

                sched_setscheduler_nocheck(input_boost_task, SCHED_RR, &param);
                get_task_struct(input_boost_task);
                boost_task_alive = true;
            }
        }

		dbs_enable++;
		for_each_cpu(j, policy->cpus) {
			struct cpu_dbs_info_s *j_dbs_info;
			j_dbs_info = &per_cpu(od_cpu_dbs_info, j);
			j_dbs_info->cur_policy = policy;

			j_dbs_info->prev_cpu_idle = get_cpu_idle_time(j,
						&j_dbs_info->prev_cpu_wall);
			if (dbs_tuners_ins.ignore_nice) {
				j_dbs_info->prev_cpu_nice =
						kstat_cpu(j).cpustat.nice;
			}
		}
		this_dbs_info->cpu = cpu;
		this_dbs_info->rate_mult = 1;
		ondemand_powersave_bias_init_cpu(cpu);
		/*
		 * Start the timerschedule work, when this governor
		 * is used for first time
		 */
		if (dbs_enable == 1) {
			unsigned int latency;

			rc = sysfs_create_group(cpufreq_global_kobject,
						&dbs_attr_group);
			if (rc) {
				mutex_unlock(&dbs_mutex);
				return rc;
			}

			dbs_chown();

			/* policy latency is in nS. Convert it to uS first */
			latency = policy->cpuinfo.transition_latency / 1000;
			if (latency == 0)
				latency = 1;
			/* Bring kernel and HW constraints together */
			min_sampling_rate = max(min_sampling_rate,
					MIN_LATENCY_MULTIPLIER * latency);
			dbs_tuners_ins.sampling_rate =
				max(min_sampling_rate,
				    latency * LATENCY_MULTIPLIER);
			if (def_sampling_rate)
				dbs_tuners_ins.sampling_rate = def_sampling_rate;
			dbs_tuners_ins.origin_sampling_rate = dbs_tuners_ins.sampling_rate;
			dbs_tuners_ins.io_is_busy = should_io_be_busy();
		}
		if (!cpu)
			rc = input_register_handler(&dbs_input_handler);

		mutex_unlock(&dbs_mutex);

		mutex_init(&this_dbs_info->timer_mutex);
		dbs_timer_init(this_dbs_info);
		break;

	case CPUFREQ_GOV_STOP:
		dbs_timer_exit(this_dbs_info);

		mutex_lock(&dbs_mutex);
		mutex_destroy(&this_dbs_info->timer_mutex);
		dbs_enable--;

		if (!cpu)
			input_unregister_handler(&dbs_input_handler);

		if (!dbs_enable)
			sysfs_remove_group(cpufreq_global_kobject,
					   &dbs_attr_group);
		mutex_unlock(&dbs_mutex);
		break;

	case CPUFREQ_GOV_LIMITS:
		mutex_lock(&this_dbs_info->timer_mutex);
		if (policy->max < this_dbs_info->cur_policy->cur)
			__cpufreq_driver_target(this_dbs_info->cur_policy,
				policy->max, CPUFREQ_RELATION_H);
		else if (policy->min > this_dbs_info->cur_policy->cur)
			__cpufreq_driver_target(this_dbs_info->cur_policy,
				policy->min, CPUFREQ_RELATION_L);
		mutex_unlock(&this_dbs_info->timer_mutex);
		break;
	}
	return 0;
}

static int __init cpufreq_gov_dbs_init(void)
{
	cputime64_t wall;
	u64 idle_time;
	int cpu = get_cpu();

	idle_time = get_cpu_idle_time_us(cpu, &wall);
	put_cpu();
	if (idle_time != -1ULL) {
		/* Idle micro accounting is supported. Use finer thresholds */
		dbs_tuners_ins.up_threshold = MICRO_FREQUENCY_UP_THRESHOLD;
		dbs_tuners_ins.down_differential =
					MICRO_FREQUENCY_DOWN_DIFFERENTIAL;
		/*
		 * In no_hz/micro accounting case we set the minimum frequency
		 * not depending on HZ, but fixed (very low). The deferred
		 * timer might skip some samples if idle/sleeping as needed.
		*/
		min_sampling_rate = MICRO_FREQUENCY_MIN_SAMPLE_RATE;
	} else {
		/* For correct statistics, we need 10 ticks for each measure */
		min_sampling_rate =
			MIN_SAMPLING_RATE_RATIO * jiffies_to_usecs(10);
	}
	def_sampling_rate = DEF_SAMPLING_RATE;

	return cpufreq_register_governor(&cpufreq_gov_ondemand);
}

static void __exit cpufreq_gov_dbs_exit(void)
{
	cpufreq_unregister_governor(&cpufreq_gov_ondemand);
}


MODULE_AUTHOR("Venkatesh Pallipadi <venkatesh.pallipadi@intel.com>");
MODULE_AUTHOR("Alexey Starikovskiy <alexey.y.starikovskiy@intel.com>");
MODULE_DESCRIPTION("'cpufreq_ondemand' - A dynamic cpufreq governor for "
	"Low Latency Frequency Transition capable processors");
MODULE_LICENSE("GPL");

#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_ONDEMAND
fs_initcall(cpufreq_gov_dbs_init);
#else
module_init(cpufreq_gov_dbs_init);
#endif
module_exit(cpufreq_gov_dbs_exit);
