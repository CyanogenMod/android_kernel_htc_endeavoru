/*
 * arch/arm/mach-tegra/cpu-tegra3.c
 *
 * CPU auto-hotplug for Tegra3 CPUs
 *
 * Copyright (c) 2011-2012, NVIDIA Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/cpufreq.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/cpu.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/pm_qos_params.h>
#include <linux/cpu_debug.h>
#include <mach/mfootprint.h>

#include "pm.h"
#include "cpu-tegra.h"
#include "clock.h"

#define INITIAL_STATE		TEGRA_HP_DISABLED
#define UP2G0_DELAY_MS		70
#define UP2Gn_DELAY_MS		100
#define DOWN_DELAY_MS		2000

static struct mutex *tegra3_cpu_lock;

static struct workqueue_struct *hotplug_wq;
static struct delayed_work hotplug_work;

static bool is_plugging;

#if defined(CONFIG_BEST_TRADE_HOTPLUG)
extern void bthp_cpuup_standalone (unsigned int freq);
extern void lp_ticket_reset (cputime64_t new_time_ticket);
extern bool lp_ticket_claim (cputime64_t claim_time);

struct workqueue_struct *bthp_hotplug_wq;
static struct work_struct bthp_hotplug_work;

static bool cpu_num_catchup = false;

struct {
    int core_num_diff;
    int dest_core;

    struct {
        unsigned int *prev_loc;
        unsigned int prev;
        unsigned int next;
    } ref_speeds;

	struct {
		cputime64_t *fallback_time_loc;
		cputime64_t fallback_time;
	} fallback;

} bthp_wq_params = {
    .core_num_diff = 0,
    .dest_core = NR_CPUS,
    .ref_speeds.prev_loc = NULL,
    .ref_speeds.prev = 0UL,
    .ref_speeds.next = 0UL,
	.fallback.fallback_time_loc = NULL,
	.fallback.fallback_time = 0ULL
};
#endif

/* BTHP sysfs ctrl node */
static bool bthp_en = 1;
bool is_bthp_en (void) {
    return ACCESS_ONCE(bthp_en);
}
EXPORT_SYMBOL(is_bthp_en);

static bool no_lp;
module_param(no_lp, bool, 0644);

static unsigned long up2gn_delay;
static unsigned long up2g0_delay;
static unsigned long down_delay;
module_param(up2gn_delay, ulong, 0644);
module_param(up2g0_delay, ulong, 0644);
module_param(down_delay, ulong, 0644);

static unsigned int idle_top_freq;
static unsigned int idle_bottom_freq;
module_param(idle_top_freq, uint, 0644);
module_param(idle_bottom_freq, uint, 0644);

static int mp_overhead = 10;
module_param(mp_overhead, int, 0644);

static int balance_level = 60;
module_param(balance_level, int, 0644);

static int up_time = 100;
module_param(up_time, int, 0644);
static int down_time = 200;
module_param(down_time, int, 0644);

static struct clk *cpu_clk;
static struct clk *cpu_g_clk;
static struct clk *cpu_lp_clk;

static unsigned long last_change_time;

static struct {
	cputime64_t time_up_total;
	u64 last_update;
	unsigned int up_down_count;
} hp_stats[CONFIG_NR_CPUS + 1];	/* Append LP CPU entry at the end */

#if defined(CONFIG_BEST_TRADE_HOTPLUG)
#define P(policy)           __mA_POLICY_##policy##__

enum {
    /* new-added policy here!!*/

    /* the last two policies, NEVER change or move them!! */
    __mA_POLICY_MAX_POWER__    = ~0UL>>2,
    __mA_POLICY_NOT_USED__     = ~0UL>>1 /* MUST set to max integer */
};

enum {
    __SPEED_340_MHZ__,
    __SPEED_475_MHZ__,
    __SPEED_640_MHZ__,
    __SPEED_760_MHZ__,
    __SPEED_910_MHZ__,
    __SPEED_880_MHZ__,
    __SPEED_1000_MHZ__,
    __SPEED_1100_MHZ__,
    __SPEED_1150_MHZ__,
    __SPEED_1200_MHZ__,
    __SPEED_1300_MHZ__,
    __SPEED_1400_MHZ__,
    __SPEED_1500_MHZ__,
    __SPEED_1600_MHZ__,
    __SPEED_1700_MHZ__

} _CPU_SPEED_LEVEL_;

#define _OFFSET(strct, elem)	((long)&(((struct strct *)NULL)->elem))
#define _POWER_ENTRY(freq) \
    (struct list_head *)((long)&_pe_tbl[__SPEED_##freq##_MHZ__]+\
    _OFFSET(pe_tbl, lv_link))

struct pe_tbl {
    unsigned long speed;            /* in KHz */
    struct list_head lv_link;
    unsigned long pwrref[NR_CPUS];  /* 1core ~ NR_CPUS cores,
                                     * each entry value MUST <= P(MAX_POWER)
                                     */
} _pe_tbl[] = {
#if defined(CONFIG_TEGRA_CPU_AP33)
    [__SPEED_340_MHZ__] =
    {
        .speed = 340000,
        .lv_link = { .prev = NULL, .next = _POWER_ENTRY(475) },
        .pwrref = { 74, 100, 125, 146 }
    },
    [__SPEED_475_MHZ__] =
    {
        .speed = 475000,
        .lv_link = { .prev = _POWER_ENTRY(340), .next = _POWER_ENTRY(640) },
        .pwrref = { 140, 179, 216, 257 }
    },
    [__SPEED_640_MHZ__] =
    {
        .speed = 640000,
        .lv_link = { .prev = _POWER_ENTRY(475), .next = _POWER_ENTRY(760) },
        .pwrref = { 156, 202, 249, 302 }
    },
    [__SPEED_760_MHZ__] =
    {
        .speed = 760000,
        .lv_link = { .prev = _POWER_ENTRY(640), .next = _POWER_ENTRY(880) },
        .pwrref = { 183, 248, 318, 397 }
    },
    [__SPEED_880_MHZ__] =
    {
        .speed = 880000,
        .lv_link = { .prev = _POWER_ENTRY(760), .next = _POWER_ENTRY(1000) },
        .pwrref = { 200, 275, 360, 457 }
    },
    [__SPEED_1000_MHZ__] =
    {
        .speed = 1000000,
        .lv_link = { .prev = _POWER_ENTRY(880), .next = _POWER_ENTRY(1100) },
        .pwrref = { 220, 312, 415, 537 }
    },
    [__SPEED_1100_MHZ__] =
    {
        .speed = 1100000,
        .lv_link = { .prev = _POWER_ENTRY(1000), .next = _POWER_ENTRY(1200) },
        .pwrref = { 249, 366, 500, 659 }
    },
    [__SPEED_1200_MHZ__] =
    {
        .speed = 1200000,
        .lv_link = { .prev = _POWER_ENTRY(1100), .next = _POWER_ENTRY(1300) },
        .pwrref = { 272, 409, 567, 761 }
    },
    [__SPEED_1300_MHZ__] =
    {
        .speed = 1300000,
        .lv_link = { .prev = _POWER_ENTRY(1200), .next = _POWER_ENTRY(1400) },
        .pwrref = { 323, 506, 726, 1017 }
    },
    [__SPEED_1400_MHZ__] =
    {
        .speed = 1400000,
        .lv_link = { .prev = _POWER_ENTRY(1300), .next = _POWER_ENTRY(1500) },
        .pwrref = { 365, 591, 909, 1323 }
    },
    [__SPEED_1500_MHZ__] =
    {
        .speed = 1500000,
        .lv_link = { .prev = _POWER_ENTRY(1400), .next = NULL },
        .pwrref = { 398, P(NOT_USED), P(NOT_USED), P(NOT_USED) }
    },

#elif defined(CONFIG_TEGRA_CPU_AP37)
    [__SPEED_340_MHZ__] =
    {
        .speed = 340000,
        .lv_link = { .prev = NULL, .next = _POWER_ENTRY(475) },
        .pwrref = { 84, 109, 134, 161 }
    },
    [__SPEED_475_MHZ__] =
    {
        .speed = 475000,
        .lv_link = { .prev = _POWER_ENTRY(340), .next = _POWER_ENTRY(640) },
        .pwrref = { 105, 136, 168, 204 }
    },
    [__SPEED_640_MHZ__] =
    {
        .speed = 640000,
        .lv_link = { .prev = _POWER_ENTRY(475), .next = _POWER_ENTRY(760) },
        .pwrref = { 134, 195, 245, 299 }
    },
    [__SPEED_760_MHZ__] =
    {
        .speed = 760000,
        .lv_link = { .prev = _POWER_ENTRY(640), .next = _POWER_ENTRY(910) },
        .pwrref = { 140, 212, 273, 340 }
    },
    [__SPEED_910_MHZ__] =
    {
        .speed = 910000,
        .lv_link = { .prev = _POWER_ENTRY(760), .next = _POWER_ENTRY(1000) },
        .pwrref = { 165, 256, 337, 423 }
    },
    [__SPEED_1000_MHZ__] =
    {
        .speed = 1000000,
        .lv_link = { .prev = _POWER_ENTRY(910), .next = _POWER_ENTRY(1150) },
        .pwrref = { 178, 298, 398, 508 }
    },
    [__SPEED_1150_MHZ__] =
    {
        .speed = 1150000,
        .lv_link = { .prev = _POWER_ENTRY(1000), .next = _POWER_ENTRY(1300) },
        .pwrref = { 237, 349, 475, 615 }
    },
    [__SPEED_1300_MHZ__] =
    {
        .speed = 1300000,
        .lv_link = { .prev = _POWER_ENTRY(1150), .next = _POWER_ENTRY(1400) },
        .pwrref = { 282, 431, 601, 795 }
    },
    [__SPEED_1400_MHZ__] =
    {
        .speed = 1400000,
        .lv_link = { .prev = _POWER_ENTRY(1300), .next = _POWER_ENTRY(1500) },
        .pwrref = { 334, 524, 746, 1036 }
    },
    [__SPEED_1500_MHZ__] =
    {
        .speed = 1500000,
        .lv_link = { .prev = _POWER_ENTRY(1400), .next = _POWER_ENTRY(1600) },
        .pwrref = { 368, 593, 873, 1198 }
    },
    [__SPEED_1600_MHZ__] =
    {
        .speed = 1600000,
        .lv_link = { .prev = _POWER_ENTRY(1500), .next = _POWER_ENTRY(1700) },
        .pwrref = { 458, 793, 1185, 1688 }
    },
    [__SPEED_1700_MHZ__] =
    {
        .speed = 1700000,
        .lv_link = { .prev = _POWER_ENTRY(1600), .next = NULL },
        .pwrref = { 467, P(NOT_USED), P(NOT_USED), P(NOT_USED) }
    },
#endif
};

static struct list_head pe_tbl_housekeeper = {
    .prev = NULL,
    .next = _POWER_ENTRY(340)
};

unsigned int bthp_supported_min_speed (void) {
    return container_of (pe_tbl_housekeeper.next,
                         struct pe_tbl, lv_link)->speed;
}
EXPORT_SYMBOL (bthp_supported_min_speed);

bool valid_power_diff (
    int diff_val
    )
{
    return (diff_val <= P(MAX_POWER) && diff_val >= -P(MAX_POWER));
}
EXPORT_SYMBOL (valid_power_diff);

bool valid_power_value (
    unsigned int power
    )
{
    return (power <= P(MAX_POWER));
}
EXPORT_SYMBOL (valid_power_value);

unsigned int valid_max_power (void) {
    return P(MAX_POWER);
}
EXPORT_SYMBOL (valid_max_power);

bool valid_bargain_speed (
    unsigned int freq
    )
{
    struct list_head *lh;

    for (lh = pe_tbl_housekeeper.next; lh; lh = lh->next) {
        if (!lh->next)
            break;
    }

    return (freq >= container_of (pe_tbl_housekeeper.next,
                                  struct pe_tbl, lv_link)->speed) &&
           (freq <= container_of (lh, struct pe_tbl,
                                  lv_link)->speed);
}
EXPORT_SYMBOL(valid_bargain_speed);

unsigned int round_or_level_up_speed (
    unsigned int speed
    )
{
    struct list_head *lh_round_up;

    for (lh_round_up = pe_tbl_housekeeper.next;
         lh_round_up;
         lh_round_up = lh_round_up->next) {

        /* level up */
        if (container_of (lh_round_up,
                          struct pe_tbl, lv_link)->speed == speed)
        {
            if (lh_round_up->next) {
                return container_of (lh_round_up->next,
                                     struct pe_tbl, lv_link)->speed;
            }

        /* round up */
        } else if (container_of (lh_round_up,
                                 struct pe_tbl, lv_link)->speed > speed) {
            return container_of (lh_round_up,
                                 struct pe_tbl, lv_link)->speed;
        }
    }

    return 0;
}
EXPORT_SYMBOL (round_or_level_up_speed);

unsigned long p2pconv (
    unsigned int speed,
    int active_cpus
    )
{
    struct list_head *lh_round_down, *lh_round_up;

    if (!speed || !active_cpus)
        return P(NOT_USED);

    for (lh_round_down = NULL, lh_round_up = pe_tbl_housekeeper.next;
         lh_round_up;
         lh_round_up = lh_round_up->next) {

        /* exact one */
        if (container_of (lh_round_up,
                          struct pe_tbl, lv_link)->speed == speed) {
            return container_of (lh_round_up,
                              struct pe_tbl, lv_link)->pwrref[active_cpus-1];

        /* need to be scaled */
        } else if (container_of (lh_round_up,
                                 struct pe_tbl, lv_link)->speed > speed) {
            lh_round_down = lh_round_up->prev;
            break;
        }
    }

    if (likely(lh_round_up) && likely(lh_round_down)) {
        /* come out % of speed distance */
        int percentage =
            ((speed - container_of (lh_round_down,
                                    struct pe_tbl, lv_link)->speed) * 100) /
            (container_of (lh_round_up, struct pe_tbl, lv_link)->speed -
             container_of (lh_round_down, struct pe_tbl, lv_link)->speed);

        if (container_of (lh_round_up,
                          struct pe_tbl, lv_link)->pwrref[active_cpus-1] ==
                          P(NOT_USED)
            ||
            container_of (lh_round_down,
                          struct pe_tbl, lv_link)->pwrref[active_cpus-1] ==
                          P(NOT_USED))
        {
            return P(NOT_USED);
        }

        /* scale power up to align with speed distance */
        return
            (((container_of (lh_round_up,
                             struct pe_tbl, lv_link)->pwrref[active_cpus-1] -
               container_of (lh_round_down,
                             struct pe_tbl, lv_link)->pwrref[active_cpus-1]) *
                percentage) / 100) +
            container_of (lh_round_down,
                          struct pe_tbl, lv_link)->pwrref[active_cpus-1];
    }

    return P(NOT_USED);
}
EXPORT_SYMBOL (p2pconv);
#endif

static void hp_init_stats(void)
{
	int i;
	u64 cur_jiffies = get_jiffies_64();

	for (i = 0; i <= CONFIG_NR_CPUS; i++) {
		hp_stats[i].time_up_total = 0;
		hp_stats[i].last_update = cur_jiffies;

		hp_stats[i].up_down_count = 0;
		if (is_lp_cluster()) {
			if (i == CONFIG_NR_CPUS)
				hp_stats[i].up_down_count = 1;
		} else {
			if ((i < nr_cpu_ids) && cpu_online(i))
				hp_stats[i].up_down_count = 1;
		}
	}

}

static void hp_stats_update(unsigned int cpu, bool up)
{
	u64 cur_jiffies = get_jiffies_64();
	bool was_up = hp_stats[cpu].up_down_count & 0x1;

	if (was_up)
		hp_stats[cpu].time_up_total = cputime64_add(
			hp_stats[cpu].time_up_total, cputime64_sub(
				cur_jiffies, hp_stats[cpu].last_update));

	if (was_up != up) {
		hp_stats[cpu].up_down_count++;
		if ((hp_stats[cpu].up_down_count & 0x1) != up) {
			/* FIXME: sysfs user space CPU control breaks stats */
			pr_err("tegra hotplug stats out of sync with %s CPU%d",
			       (cpu < CONFIG_NR_CPUS) ? "G" : "LP",
			       (cpu < CONFIG_NR_CPUS) ?  cpu : 0);
			hp_stats[cpu].up_down_count ^=  0x1;
		}
	}
	hp_stats[cpu].last_update = cur_jiffies;
}


enum {
	TEGRA_HP_DISABLED = 0,
	TEGRA_HP_IDLE,
	TEGRA_HP_DOWN,
	TEGRA_HP_UP,
};
static int hp_state;

static int hp_state_set(const char *arg, const struct kernel_param *kp)
{
	int ret = 0;
	int old_state;

	if (!tegra3_cpu_lock)
		return ret;

	mutex_lock(tegra3_cpu_lock);

	old_state = hp_state;
	ret = param_set_bool(arg, kp);	/* set idle or disabled only */

	if (ret == 0) {
		if ((hp_state == TEGRA_HP_DISABLED) &&
		    (old_state != TEGRA_HP_DISABLED)) {
			mutex_unlock(tegra3_cpu_lock);
			cancel_delayed_work_sync(&hotplug_work);
#if defined(CONFIG_BEST_TRADE_HOTPLUG)
            cancel_work_sync(&bthp_hotplug_work);
#endif
			mutex_lock(tegra3_cpu_lock);
#if defined(CONFIG_BEST_TRADE_HOTPLUG)
            if (cpu_num_catchup)
                cpu_num_catchup = false;
#endif

			if (is_plugging) {
				pr_info("Tegra auto-hotplug: is_plugging is true, set to false\n");
				is_plugging = false;
			}
			pr_info("Tegra auto-hotplug disabled\n");
		} else if (hp_state != TEGRA_HP_DISABLED) {
			if (old_state == TEGRA_HP_DISABLED) {
				pr_info("Tegra auto-hotplug enabled\n");
				hp_init_stats();
			}
			/* catch-up with governor target speed */
			tegra_cpu_set_speed_cap(NULL);
		}
	} else
		pr_warn("%s: unable to set tegra hotplug state %s\n",
				__func__, arg);

	mutex_unlock(tegra3_cpu_lock);
	return ret;
}

static int hp_state_get(char *buffer, const struct kernel_param *kp)
{
	return param_get_int(buffer, kp);
}

static struct kernel_param_ops tegra_hp_state_ops = {
	.set = hp_state_set,
	.get = hp_state_get,
};
module_param_cb(auto_hotplug, &tegra_hp_state_ops, &hp_state, 0644);

enum {
	TEGRA_CPU_SPEED_BALANCED,
	TEGRA_CPU_SPEED_BIASED,
	TEGRA_CPU_SPEED_SKEWED,
};

#define NR_FSHIFT	2

static unsigned int rt_profile_sel;

/* avg run threads * 4 (e.g., 9 = 2.25 threads) */

static unsigned int rt_profile_default[] = {
/*      1,  2,  3,  4 - on-line cpus target */
	5,  9, 10, UINT_MAX
};

static unsigned int rt_profile_1[] = {
/*      1,  2,  3,  4 - on-line cpus target */
	8,  9, 10, UINT_MAX
};

static unsigned int rt_profile_2[] = {
/*      1,  2,  3,  4 - on-line cpus target */
	5,  13, 14, UINT_MAX
};

static unsigned int rt_profile_off[] = { /* disables runable thread */
	0,  0,  0, UINT_MAX
};

static unsigned int *rt_profiles[] = {
	rt_profile_default,
	rt_profile_1,
	rt_profile_2,
	rt_profile_off
};


static unsigned int nr_run_hysteresis = 2;	/* 0.5 thread */
static unsigned int nr_run_last;

static noinline int tegra_cpu_speed_balance(void)
{
	unsigned long highest_speed = tegra_cpu_highest_speed();
	unsigned long balanced_speed = highest_speed * balance_level / 100;
	unsigned long skewed_speed = balanced_speed / 2;
	unsigned int nr_cpus = num_online_cpus();
	unsigned int max_cpus = pm_qos_request(PM_QOS_MAX_ONLINE_CPUS) ? : 4;
	unsigned int min_cpus = pm_qos_request(PM_QOS_MIN_ONLINE_CPUS);
	unsigned int avg_nr_run = avg_nr_running();
	unsigned int nr_run;

	/* Evaluate:
	 * - distribution of freq targets for already on-lined CPUs
	 * - average number of runnable threads
	 * - effective MIPS available within EDP frequency limits,
	 * and return:
	 * TEGRA_CPU_SPEED_BALANCED to bring one more CPU core on-line
	 * TEGRA_CPU_SPEED_BIASED to keep CPU core composition unchanged
	 * TEGRA_CPU_SPEED_SKEWED to remove CPU core off-line
	 */

	unsigned int *current_profile = rt_profiles[rt_profile_sel];
	for (nr_run = 1; nr_run < ARRAY_SIZE(rt_profile_default); nr_run++) {
		unsigned int nr_threshold = current_profile[nr_run - 1];
		if (nr_run_last <= nr_run)
			nr_threshold += nr_run_hysteresis;
		if (avg_nr_run <= (nr_threshold << (FSHIFT - NR_FSHIFT)))
			break;
	}
	nr_run_last = nr_run;

	if (((tegra_count_slow_cpus(skewed_speed) >= 2) ||
	     (nr_run < nr_cpus) ||
	     tegra_cpu_edp_favor_down(nr_cpus, mp_overhead) ||
	     (highest_speed <= idle_bottom_freq) || (nr_cpus > max_cpus)) &&
	    (nr_cpus > min_cpus))
		return TEGRA_CPU_SPEED_SKEWED;

	if (((tegra_count_slow_cpus(balanced_speed) >= 1) ||
	     (nr_run <= nr_cpus) ||
	     (!tegra_cpu_edp_favor_up(nr_cpus, mp_overhead)) ||
	     (highest_speed <= idle_bottom_freq) || (nr_cpus == max_cpus)) &&
	    (nr_cpus >= min_cpus))
		return TEGRA_CPU_SPEED_BIASED;

	return TEGRA_CPU_SPEED_BALANCED;
}

static void tegra_auto_hotplug_work_func(struct work_struct *work)
{
	bool up = false;
	unsigned int cpu = nr_cpu_ids;
	unsigned long now = jiffies;

	mutex_lock(tegra3_cpu_lock);

	switch (hp_state) {
	case TEGRA_HP_DISABLED:
	case TEGRA_HP_IDLE:
		break;
	case TEGRA_HP_DOWN:
		cpu = tegra_get_slowest_cpu_n();
		if (cpu < nr_cpu_ids) {
			up = false;
		} else if (!is_lp_cluster() && !no_lp &&
			   !pm_qos_request(PM_QOS_MIN_ONLINE_CPUS) &&
			   ((now - last_change_time) >= down_delay)) {
			if(!clk_set_parent(cpu_clk, cpu_lp_clk)) {
				CPU_DEBUG_PRINTK(CPU_DEBUG_HOTPLUG, " enter LPCPU");
				hp_stats_update(CONFIG_NR_CPUS, true);
				hp_stats_update(0, false);
				/* catch-up with governor target speed */
				tegra_cpu_set_speed_cap(NULL);
				break;
			}
		}
		queue_delayed_work(
			hotplug_wq, &hotplug_work, up2gn_delay);
		break;
	case TEGRA_HP_UP:
		if (is_lp_cluster() && !no_lp) {
			if(!clk_set_parent(cpu_clk, cpu_g_clk)) {
				CPU_DEBUG_PRINTK(CPU_DEBUG_HOTPLUG,
						 " leave LPCPU (%s)", __func__);
				last_change_time = now;
				hp_stats_update(CONFIG_NR_CPUS, false);
				hp_stats_update(0, true);

#if defined(CONFIG_BEST_TRADE_HOTPLUG)
                lp_ticket_reset (ktime_to_ns (ktime_get ()));
#endif
				/* catch-up with governor target speed */
				tegra_cpu_set_speed_cap(NULL);
			}
		} else {
			switch (tegra_cpu_speed_balance()) {
			/* cpu speed is up and balanced - one more on-line */
			case TEGRA_CPU_SPEED_BALANCED:
				cpu = cpumask_next_zero(0, cpu_online_mask);
				if (cpu < nr_cpu_ids)
					up = true;
				break;
			/* cpu speed is up, but skewed - remove one core */
			case TEGRA_CPU_SPEED_SKEWED:
				cpu = tegra_get_slowest_cpu_n();
				if (cpu < nr_cpu_ids)
					up = false;
				break;
			/* cpu speed is up, but under-utilized - do nothing */
			case TEGRA_CPU_SPEED_BIASED:
			default:
				break;
			}
		}
		queue_delayed_work(
			hotplug_wq, &hotplug_work, up2gn_delay);
		break;
	default:
		pr_err("%s: invalid tegra hotplug state %d\n",
		       __func__, hp_state);
	}

	if (!up && ((now - last_change_time) < down_delay))
			cpu = nr_cpu_ids;

	if (cpu < nr_cpu_ids) {
		last_change_time = now;
		hp_stats_update(cpu, up);
	}
	mutex_unlock(tegra3_cpu_lock);

	if (system_state > SYSTEM_RUNNING) {
		CPU_DEBUG_PRINTK(CPU_DEBUG_HOTPLUG, " system is not running\n");
	} else if (cpu < nr_cpu_ids) {
		if (up) {
			cpu_up(cpu);
			CPU_DEBUG_PRINTK(CPU_DEBUG_HOTPLUG, " turn on CPU %d, online CPU 0-3=[%d%d%d%d]\n",
					cpu, cpu_online(0), cpu_online(1), cpu_online(2), cpu_online(3));
		} else {
			cpu_down(cpu);
			CPU_DEBUG_PRINTK(CPU_DEBUG_HOTPLUG, " turn off CPU %d, online CPU 0-3=[%d%d%d%d]\n",
					cpu, cpu_online(0), cpu_online(1), cpu_online(2), cpu_online(3));
		}
	}
}

#if defined(CONFIG_BEST_TRADE_HOTPLUG)
extern unsigned int best_core_to_turn_up (void);

unsigned int g2lp_bottom_freq (void) {
    return idle_bottom_freq;
}
EXPORT_SYMBOL (g2lp_bottom_freq);

unsigned int lp2g_top_freq (void) {
    return idle_top_freq;
}
EXPORT_SYMBOL (lp2g_top_freq);

bool on_plugging (void) {
    return ACCESS_ONCE (is_plugging);
}
EXPORT_SYMBOL (on_plugging);

bool can_do_bthp (void) {
    return (!ACCESS_ONCE (is_plugging) && hp_state != TEGRA_HP_DISABLED);
}
EXPORT_SYMBOL (can_do_bthp);

static void init_bthp_params (void) {
    bthp_wq_params.core_num_diff = 0;
    bthp_wq_params.dest_core = NR_CPUS;
    bthp_wq_params.ref_speeds.prev_loc = NULL;
    bthp_wq_params.ref_speeds.prev = 0UL;
    bthp_wq_params.ref_speeds.next = 0UL;

	if (likely(bthp_wq_params.fallback.fallback_time_loc)) {
		*bthp_wq_params.fallback.fallback_time_loc =
            bthp_wq_params.fallback.fallback_time;
	}
}

void bthp_auto_hotplug_work_func (
    struct work_struct *work
    )
{
    unsigned int curr_speed = 0;
    unsigned int ret = 0;

    MF_DEBUG("00UP0000");
    mutex_lock (tegra3_cpu_lock);

    /* final chance to turn around to bring required cores up */
    if (!cpu_num_catchup &&
        num_online_cpus () < pm_qos_request (PM_QOS_MIN_ONLINE_CPUS) &&
        hp_state != TEGRA_HP_DISABLED)
    {
        cpu_num_catchup = true;
        hp_state = TEGRA_HP_UP;
    }

    if (unlikely(cpu_num_catchup)) {
        if (is_plugging &&
            hp_state == TEGRA_HP_UP &&
            system_state <= SYSTEM_RUNNING)
        {
            int num_cores_to_online =
                pm_qos_request (PM_QOS_MIN_ONLINE_CPUS) -
                num_online_cpus ();
            unsigned int hp_stat_cpu;
            struct cpumask awake_cores = { CPU_BITS_NONE };

            mutex_unlock (tegra3_cpu_lock);

            /* not queue a work repeatedly to bring up core one-by-one;
             * instead, get it done directly ASAP
             */
            for (; num_cores_to_online > 0 &&
                   num_cores_to_online < nr_cpu_ids;
                   num_cores_to_online--)
            {
                unsigned int core_to_online = best_core_to_turn_up ();

                if (core_to_online < nr_cpu_ids) {
                    MF_DEBUG("00UP0001");
		    if (!cpu_up (core_to_online)) {
                        cpumask_set_cpu (core_to_online, &awake_cores);
                        CPU_DEBUG_PRINTK (
                           CPU_DEBUG_HOTPLUG,
                           " TURN ON CPU %d, online CPU 0-3=[%d%d%d%d]\n",
                           core_to_online,
                           cpu_online(0),
                           cpu_online(1),
                           cpu_online(2),
                           cpu_online(3));
                    }
                }
            }

            mutex_lock (tegra3_cpu_lock);

            for_each_cpu (hp_stat_cpu, &awake_cores)
                hp_stats_update (hp_stat_cpu, true);
        }

        cpu_num_catchup = false;

        if (hp_state != TEGRA_HP_DISABLED)
            hp_state = TEGRA_HP_IDLE;

        if (is_plugging)
            is_plugging = false;

        mutex_unlock (tegra3_cpu_lock);

        return;
    }

    do {
         /* hot-plug is disabled at runtime? */
        if (!is_plugging ||

            /* LP -> G, take NV's original design */
            is_lp_cluster () ||

            /* SHOULD NOT have happene!! */
            !bthp_wq_params.ref_speeds.prev_loc ||
            !bthp_wq_params.ref_speeds.prev ||
            !bthp_wq_params.ref_speeds.next ||

            /* performance demand had changed,
             * don't do the too-late hotplug decision
             */
            (bthp_wq_params.dest_core != 0 &&
             *bthp_wq_params.ref_speeds.prev_loc !=
             bthp_wq_params.ref_speeds.prev)
            )
        {
            init_bthp_params ();
            mutex_unlock (tegra3_cpu_lock);
            break;
        }

        /* update target speed record first and then
         * catch up with this speed after hotplug
         */
        if (bthp_wq_params.dest_core != 0) {
            *bthp_wq_params.ref_speeds.prev_loc =
                bthp_wq_params.ref_speeds.next;
        }

        /* CPU UP */
        if (bthp_wq_params.core_num_diff > 0 &&
            hp_state == TEGRA_HP_UP &&
            bthp_wq_params.dest_core < NR_CPUS &&
            !cpu_online (bthp_wq_params.dest_core))
        {
            hp_stats_update (bthp_wq_params.dest_core, true);
            mutex_unlock (tegra3_cpu_lock);
            break;
        }
        /* CPU DOWN */
        else if (bthp_wq_params.core_num_diff < 0 &&
                 hp_state == TEGRA_HP_DOWN &&
                 bthp_wq_params.dest_core < NR_CPUS &&
                 cpu_online (bthp_wq_params.dest_core))
        {
            /* G -> LP */
            if (!bthp_wq_params.dest_core) {
                if (!no_lp) {
                    /* scale down to max of lp clk */
                    curr_speed = tegra_getspeed (0);
                    if (likely(curr_speed) && likely(idle_top_freq) &&
                        curr_speed > idle_top_freq)
                    {
                        ret = tegra_update_cpu_speed (idle_top_freq);
                    }

                    if (likely(!ret) &&
                        !clk_set_parent (cpu_clk, cpu_lp_clk))
                    {
                        CPU_DEBUG_PRINTK (CPU_DEBUG_HOTPLUG, " ENTER LPCPU");
                        hp_stats_update (CONFIG_NR_CPUS, true);
                        hp_stats_update (0, false);

                    } else
                        pr_err ("%s: scale_down_to_max_of_lp_clk or "\
                                "clk_set_parent fail\n",
                                __func__
                                );
                }

            } else {
                hp_stats_update (bthp_wq_params.dest_core, false);
            }

            mutex_unlock (tegra3_cpu_lock);
            break;
        }

        mutex_unlock(tegra3_cpu_lock);

    } while (0);

    if (system_state > SYSTEM_RUNNING) {
        CPU_DEBUG_PRINTK (CPU_DEBUG_HOTPLUG, " SYSTEM is not running\n");

    } else if (bthp_wq_params.dest_core < NR_CPUS &&
               bthp_wq_params.dest_core != 0)
    {
        if (bthp_wq_params.core_num_diff > 0) {
	    MF_DEBUG("00UP0002");
            cpu_up (bthp_wq_params.dest_core);
            CPU_DEBUG_PRINTK (CPU_DEBUG_HOTPLUG,
                              " TURN ON CPU %d, online CPU 0-3=[%d%d%d%d]\n",
                              bthp_wq_params.dest_core,
                              cpu_online(0),
                              cpu_online(1),
                              cpu_online(2),
                              cpu_online(3));
        } else {
            cpu_down (bthp_wq_params.dest_core);
            CPU_DEBUG_PRINTK (CPU_DEBUG_HOTPLUG,
                              " TURN OFF CPU %d, online CPU 0-3=[%d%d%d%d]\n",
                              bthp_wq_params.dest_core,
                              cpu_online(0),
                              cpu_online(1),
                              cpu_online(2),
                              cpu_online(3));
        }
    }

    mutex_lock (tegra3_cpu_lock);
    if (hp_state != TEGRA_HP_DISABLED)
        hp_state = TEGRA_HP_IDLE;

    MF_DEBUG("00UP0029");
    if (is_plugging) {
        /* catch-up with up-to-date governor target speed */
        tegra_cpu_set_speed_cap (NULL);

        is_plugging = false;
    }
    MF_DEBUG("00UP0051");
	mutex_unlock(tegra3_cpu_lock);
    MF_DEBUG("00UP0052");
}

bool bthp_do_hotplug (
    int core_num_diff,
    int dest_core,
    cputime64_t *last_time,
    unsigned int *debounce_interval,
    unsigned int target_speed,
    unsigned int *decision_cpu_speed_loc
    )
{
    cputime64_t now = ktime_to_ns (ktime_get ());

    if (core_num_diff != 0 &&
        hp_state != TEGRA_HP_DISABLED &&
        !is_plugging &&
        time_after_eq64 (now, *last_time + *debounce_interval))
    {
        if (core_num_diff == -1 && dest_core == 0 && !lp_ticket_claim (now))
            return false;

        is_plugging = true;
        bthp_wq_params.fallback.fallback_time_loc = last_time;
        bthp_wq_params.fallback.fallback_time = *last_time;
        *last_time = now;
        hp_state = core_num_diff > 0? TEGRA_HP_UP: TEGRA_HP_DOWN;
        bthp_wq_params.core_num_diff = core_num_diff;
        bthp_wq_params.dest_core = dest_core;
        bthp_wq_params.ref_speeds.prev_loc = decision_cpu_speed_loc;

        /* Hotplug always takes some while, said the overhead is observable
         * We MUST secure performance during hotplug processing
         *
         * if speed goes UP, UP before hotplug
         * if speed goes DOWN, DOWN after hotplug
         */
        if (target_speed > *decision_cpu_speed_loc) {
            *decision_cpu_speed_loc = target_speed;
        }

        bthp_wq_params.ref_speeds.prev = *decision_cpu_speed_loc;
        bthp_wq_params.ref_speeds.next = target_speed;
        queue_work (bthp_hotplug_wq, &bthp_hotplug_work);

        return true;
    }

    return false;
}
EXPORT_SYMBOL (bthp_do_hotplug);

bool bthp_cpu_num_catchup (void)
{
    if (hp_state != TEGRA_HP_DISABLED && !is_plugging) {
        is_plugging = true;
        cpu_num_catchup = true;
        hp_state = TEGRA_HP_UP;
        queue_work (bthp_hotplug_wq, &bthp_hotplug_work);

        return true;
    }

    return false;
}
EXPORT_SYMBOL (bthp_cpu_num_catchup);
#endif

static int min_cpus_notify(struct notifier_block *nb, unsigned long n, void *p)
{
	mutex_lock(tegra3_cpu_lock);

	if ((n >= 1) && is_lp_cluster()) {
		/* make sure cpu rate is within g-mode range before switching */
		unsigned int speed = max((unsigned long)tegra_getspeed(0),
			clk_get_min_rate(cpu_g_clk) / 1000);
		tegra_update_cpu_speed(speed);

		if (!clk_set_parent(cpu_clk, cpu_g_clk)) {
			CPU_DEBUG_PRINTK(CPU_DEBUG_HOTPLUG,
					 " leave LPCPU (%s)", __func__);
			last_change_time = jiffies;
			hp_stats_update(CONFIG_NR_CPUS, false);
			hp_stats_update(0, true);
		}
	}
	/* update governor state machine */
	tegra_cpu_set_speed_cap(NULL);
	mutex_unlock(tegra3_cpu_lock);
	return NOTIFY_OK;
}

static struct notifier_block min_cpus_notifier = {
	.notifier_call = min_cpus_notify,
};

void tegra_auto_hotplug_governor(unsigned int cpu_freq, bool suspend)
{
	unsigned long up_delay, top_freq, bottom_freq;

	if (!is_g_cluster_present())
		return;

	if (hp_state == TEGRA_HP_DISABLED)
		return;

	if (suspend) {
		hp_state = TEGRA_HP_IDLE;

		/* Switch to G-mode if suspend rate is high enough */
		if (is_lp_cluster() && (cpu_freq >= idle_bottom_freq)) {
			if (!clk_set_parent(cpu_clk, cpu_g_clk)) {
				CPU_DEBUG_PRINTK(CPU_DEBUG_HOTPLUG,
						 " leave LPCPU (%s)", __func__);
				hp_stats_update(CONFIG_NR_CPUS, false);
				hp_stats_update(0, true);
			}
		}
		return;
	}

	if (is_lp_cluster()) {
		up_delay = up2g0_delay;
		top_freq = idle_top_freq;
		bottom_freq = 0;
	} else {
		up_delay = up2gn_delay;
		top_freq = idle_bottom_freq;
		bottom_freq = idle_bottom_freq;

#if defined(CONFIG_BEST_TRADE_HOTPLUG)
	        if (likely(bthp_en)) {
			bthp_cpuup_standalone (cpu_freq);
			return;
	        }
#endif
	}

	if (pm_qos_request(PM_QOS_MIN_ONLINE_CPUS) >= 2) {
		if (hp_state != TEGRA_HP_UP) {
			hp_state = TEGRA_HP_UP;
			queue_delayed_work(
				hotplug_wq, &hotplug_work, up_delay);
		}
		return;
	}

	switch (hp_state) {
	case TEGRA_HP_IDLE:
		if (cpu_freq > top_freq) {
			hp_state = TEGRA_HP_UP;
			queue_delayed_work(
				hotplug_wq, &hotplug_work, up_delay);
		} else if (cpu_freq <= bottom_freq) {
			hp_state = TEGRA_HP_DOWN;
			queue_delayed_work(
				hotplug_wq, &hotplug_work, up_delay);
		}
		break;
	case TEGRA_HP_DOWN:
		if (cpu_freq > top_freq) {
			hp_state = TEGRA_HP_UP;
			queue_delayed_work(
				hotplug_wq, &hotplug_work, up_delay);
		} else if (cpu_freq > bottom_freq) {
			hp_state = TEGRA_HP_IDLE;
		}
		break;
	case TEGRA_HP_UP:
		if (cpu_freq <= bottom_freq) {
			hp_state = TEGRA_HP_DOWN;
			queue_delayed_work(
				hotplug_wq, &hotplug_work, up_delay);
		} else if (cpu_freq <= top_freq) {
			hp_state = TEGRA_HP_IDLE;
		}
		break;
	default:
		pr_err("%s: invalid tegra hotplug state %d\n",
		       __func__, hp_state);
		BUG();
	}
}

int tegra_auto_hotplug_init(struct mutex *cpu_lock)
{
	/*
	 * Not bound to the issuer CPU (=> high-priority), has rescue worker
	 * task, single-threaded, freezable.
	 */
	hotplug_wq = alloc_workqueue(
		"cpu-tegra3", WQ_UNBOUND | WQ_RESCUER | WQ_FREEZABLE, 1);
	if (!hotplug_wq)
		return -ENOMEM;
	INIT_DELAYED_WORK(&hotplug_work, tegra_auto_hotplug_work_func);

#if defined(CONFIG_BEST_TRADE_HOTPLUG)
    bthp_hotplug_wq = alloc_workqueue (
        "bthp-tegra3", WQ_UNBOUND | WQ_RESCUER | WQ_FREEZABLE, 1);
    if (!bthp_hotplug_wq)
        return -ENOMEM;
    INIT_WORK(&bthp_hotplug_work, bthp_auto_hotplug_work_func);
#endif

	cpu_clk = clk_get_sys(NULL, "cpu");
	cpu_g_clk = clk_get_sys(NULL, "cpu_g");
	cpu_lp_clk = clk_get_sys(NULL, "cpu_lp");
	if (IS_ERR(cpu_clk) || IS_ERR(cpu_g_clk) || IS_ERR(cpu_lp_clk))
		return -ENOENT;

	idle_top_freq = clk_get_max_rate(cpu_lp_clk) / 1000;
	idle_bottom_freq = clk_get_min_rate(cpu_g_clk) / 1000;

	up2g0_delay = msecs_to_jiffies(UP2G0_DELAY_MS);
	up2gn_delay = msecs_to_jiffies(UP2Gn_DELAY_MS);
	down_delay = msecs_to_jiffies(DOWN_DELAY_MS);

	is_plugging = false;

	tegra3_cpu_lock = cpu_lock;
	hp_state = INITIAL_STATE;
	hp_init_stats();
	pr_info("Tegra auto-hotplug initialized: %s\n",
		(hp_state == TEGRA_HP_DISABLED) ? "disabled" : "enabled");

	if (pm_qos_add_notifier(PM_QOS_MIN_ONLINE_CPUS, &min_cpus_notifier))
		pr_err("%s: Failed to register min cpus PM QoS notifier\n",
			__func__);

	return 0;
}

#ifdef CONFIG_DEBUG_FS

static struct dentry *hp_debugfs_root;

struct pm_qos_request_list min_cpu_req;
struct pm_qos_request_list max_cpu_req;

static int hp_stats_show(struct seq_file *s, void *data)
{
	int i;
	u64 cur_jiffies = get_jiffies_64();

	mutex_lock(tegra3_cpu_lock);
	if (hp_state != TEGRA_HP_DISABLED) {
		for (i = 0; i <= CONFIG_NR_CPUS; i++) {
			bool was_up = (hp_stats[i].up_down_count & 0x1);
			hp_stats_update(i, was_up);
		}
	}
	mutex_unlock(tegra3_cpu_lock);

	seq_printf(s, "%-15s ", "cpu:");
	for (i = 0; i < CONFIG_NR_CPUS; i++) {
		seq_printf(s, "G%-9d ", i);
	}
	seq_printf(s, "LP\n");

	seq_printf(s, "%-15s ", "transitions:");
	for (i = 0; i <= CONFIG_NR_CPUS; i++) {
		seq_printf(s, "%-10u ", hp_stats[i].up_down_count);
	}
	seq_printf(s, "\n");

	seq_printf(s, "%-15s ", "time plugged:");
	for (i = 0; i <= CONFIG_NR_CPUS; i++) {
		seq_printf(s, "%-10llu ",
			   cputime64_to_clock_t(hp_stats[i].time_up_total));
	}
	seq_printf(s, "\n");

	seq_printf(s, "%-15s %llu\n", "time-stamp:",
		   cputime64_to_clock_t(cur_jiffies));

	return 0;
}

static int hp_stats_open(struct inode *inode, struct file *file)
{
	return single_open(file, hp_stats_show, inode->i_private);
}

static const struct file_operations hp_stats_fops = {
	.open		= hp_stats_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int rt_bias_get(void *data, u64 *val)
{
	*val = rt_profile_sel;
	return 0;
}
static int rt_bias_set(void *data, u64 val)
{
	if (val < ARRAY_SIZE(rt_profiles))
		rt_profile_sel = (u32)val;

	pr_debug("rt_profile_sel set to %d\nthresholds are now [%d, %d, %d]\n",
		rt_profile_sel,
		rt_profiles[rt_profile_sel][0],
		rt_profiles[rt_profile_sel][1],
		rt_profiles[rt_profile_sel][2]);
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(rt_bias_fops, rt_bias_get, rt_bias_set, "%llu\n");

static int min_cpus_get(void *data, u64 *val)
{
	*val = pm_qos_request(PM_QOS_MIN_ONLINE_CPUS);
	return 0;
}
static int min_cpus_set(void *data, u64 val)
{
	pm_qos_update_request(&min_cpu_req, (s32)val);

#if defined(CONFIG_BEST_TRADE_HOTPLUG)
    if (likely(bthp_en) &&
        num_online_cpus() < pm_qos_request(PM_QOS_MIN_ONLINE_CPUS))
    {
        if (!bthp_cpu_num_catchup())
            CPU_DEBUG_PRINTK(CPU_DEBUG_BTHP,
                             " %s: cannot bring up # of cpus required",
                             __func__
                             );
    }
#endif

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(min_cpus_fops, min_cpus_get, min_cpus_set, "%llu\n");

static int max_cpus_get(void *data, u64 *val)
{
	*val = pm_qos_request(PM_QOS_MAX_ONLINE_CPUS);
	return 0;
}
static int max_cpus_set(void *data, u64 val)
{
	pm_qos_update_request(&max_cpu_req, (s32)val);
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(max_cpus_fops, max_cpus_get, max_cpus_set, "%llu\n");

static int bthp_ctrl_set (
   const char *arg,
   const struct kernel_param *kp
   )
{
	int ret = 0;

	ret = param_set_int (arg, kp);
	if (!ret) {
#if defined(CONFIG_BEST_TRADE_HOTPLUG)
        extern void cleanup_bthp_rqinfo (void);

        /* To support dynamic BTHP on/off, we MUST clean up stale rq info.
         * so that the accurate prophet can be given
         */
        cleanup_bthp_rqinfo ();
#endif
        pr_bthp_info ("bthp_en = %d\n", bthp_en);
	}

	return ret;
}

static int bthp_ctrl_get (
   char *buffer,
   const struct kernel_param *kp
   )
{
	return param_get_int (buffer, kp);
}

static struct kernel_param_ops bthp_ctrl_ops = {
	.set = bthp_ctrl_set,
	.get = bthp_ctrl_get,
};
module_param_cb(bthp_en, &bthp_ctrl_ops, &bthp_en, 0444);

/* controller for activity trigger */
static bool at_en = 1;

static int at_ctrl_set (
   const char *arg,
   const struct kernel_param *kp
   )
{
	return param_set_int (arg, kp);
}

static int at_ctrl_get (
   char *buffer,
   const struct kernel_param *kp
   )
{
	return param_get_int (buffer, kp);
}

static struct kernel_param_ops at_ctrl_ops = {
	.set = at_ctrl_set,
	.get = at_ctrl_get,
};
module_param_cb(at_en, &at_ctrl_ops, &at_en, 0664);

static int __init tegra_auto_hotplug_debug_init(void)
{
	if (!tegra3_cpu_lock)
		return -ENOENT;

	hp_debugfs_root = debugfs_create_dir("tegra_hotplug", NULL);
	if (!hp_debugfs_root)
		return -ENOMEM;

	pm_qos_add_request(&min_cpu_req, PM_QOS_MIN_ONLINE_CPUS,
			   PM_QOS_DEFAULT_VALUE);
	pm_qos_add_request(&max_cpu_req, PM_QOS_MAX_ONLINE_CPUS,
			   PM_QOS_DEFAULT_VALUE);

	if (!debugfs_create_file(
		"min_cpus", S_IRUGO, hp_debugfs_root, NULL, &min_cpus_fops))
		goto err_out;

	if (!debugfs_create_file(
		"max_cpus", S_IRUGO, hp_debugfs_root, NULL, &max_cpus_fops))
		goto err_out;

	if (!debugfs_create_file(
		"stats", S_IRUGO, hp_debugfs_root, NULL, &hp_stats_fops))
		goto err_out;

	if (!debugfs_create_file(
		"core_bias", S_IRUGO, hp_debugfs_root, NULL, &rt_bias_fops))
		goto err_out;

	return 0;

err_out:
	debugfs_remove_recursive(hp_debugfs_root);
	pm_qos_remove_request(&min_cpu_req);
	pm_qos_remove_request(&max_cpu_req);
	return -ENOMEM;
}

late_initcall(tegra_auto_hotplug_debug_init);
#endif

void tegra_auto_hotplug_exit(void)
{
	destroy_workqueue(hotplug_wq);
#if defined(CONFIG_BEST_TRADE_HOTPLUG)
    destroy_workqueue(bthp_hotplug_wq);
#endif
#ifdef CONFIG_DEBUG_FS
	debugfs_remove_recursive(hp_debugfs_root);
	pm_qos_remove_request(&min_cpu_req);
	pm_qos_remove_request(&max_cpu_req);
#endif
}
