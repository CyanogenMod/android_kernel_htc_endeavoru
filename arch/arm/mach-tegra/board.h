/*
 * arch/arm/mach-tegra/board.h
 *
 * Copyright (C) 2010 Google, Inc.
 * Copyright (C) 2011-2012 NVIDIA Corporation.
 *
 * Author:
 *	Colin Cross <ccross@google.com>
 *	Erik Gilling <konkers@google.com>
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

#ifndef __MACH_TEGRA_BOARD_H
#define __MACH_TEGRA_BOARD_H

#include <linux/types.h>
#include <linux/power_supply.h>
#include <linux/memory.h>

#define ADD_FIXED_VOLTAGE_REG(_name)	(&_name##_fixed_voltage_device)

/* Macro for defining fixed voltage regulator */
#define FIXED_VOLTAGE_REG_INIT(_id, _name, _microvolts, _gpio,		\
		_startup_delay, _enable_high, _enabled_at_boot,		\
		_valid_ops_mask, _always_on)				\
	static struct regulator_init_data _name##_initdata = {		\
		.consumer_supplies = _name##_consumer_supply,		\
		.num_consumer_supplies =				\
				ARRAY_SIZE(_name##_consumer_supply),	\
		.constraints = {					\
			.valid_ops_mask = _valid_ops_mask ,		\
			.always_on = _always_on,			\
		},							\
	};								\
	static struct fixed_voltage_config _name##_config = {		\
		.supply_name		= #_name,			\
		.microvolts		= _microvolts,			\
		.gpio			= _gpio,			\
		.startup_delay		= _startup_delay,		\
		.enable_high		= _enable_high,			\
		.enabled_at_boot	= _enabled_at_boot,		\
		.init_data		= &_name##_initdata,		\
	};								\
	static struct platform_device _name##_fixed_voltage_device = {	\
		.name			= "reg-fixed-voltage",		\
		.id			= _id,				\
		.dev			= {				\
			.platform_data	= &_name##_config,		\
		},							\
	}

#if defined(CONFIG_TEGRA_NVMAP)
#define NVMAP_HEAP_CARVEOUT_IRAM_INIT	\
	{	.name		= "iram",					\
		.usage_mask	= NVMAP_HEAP_CARVEOUT_IRAM,			\
		.base		= TEGRA_IRAM_BASE + TEGRA_RESET_HANDLER_SIZE,	\
		.size		= TEGRA_IRAM_SIZE - TEGRA_RESET_HANDLER_SIZE,	\
		.buddy_size	= 0, /* no buddy allocation for IRAM */		\
	}
#endif

/* This information is passed by bootloader */
#define COMMCHIP_UNKNOWN		0
#define COMMCHIP_NOCHIP			1
#define COMMCHIP_BROADCOM_BCM4329	2
#define COMMCHIP BROADCOM_BCM4330	3
#define COMMCHIP_MARVELL_SD8797		4


struct memory_accessor;

void tegra_assert_system_reset(char mode, const char *cmd);
void get_mac_addr(struct memory_accessor *, void *);

void __init tegra_init_early(void);
void __init tegra_mc_init(void);
void __init tegra_map_common_io(void);
void __init tegra_init_irq(void);
void __init tegra_init_clock(void);
void __init tegra_reserve(unsigned long carveout_size, unsigned long fb_size,
	unsigned long fb2_size);
/* FIXME: The following needs to move common.h when arm_soc_desc is
	  introduced in a future version of the kernel */
#ifdef CONFIG_CACHE_L2X0
void tegra_init_cache(bool init);
#else
static inline void tegra_init_cache(bool init) {}
#endif
void __init tegra_ram_console_debug_reserve(unsigned long ram_console_size);
void __init tegra_ram_console_debug_init(void);
void __init htc_memory_footprint_space_reserve(unsigned long size);
void __init htc_memory_footprint_init(void);
void __init fiq_dumper_space_reserve(unsigned long size);
void __init fiq_dumper_init(void);
void __init tegra_release_bootloader_fb(void);
void __init tegra_protected_aperture_init(unsigned long aperture);
int  __init tegra_init_board_info(void);
void tegra_move_framebuffer(unsigned long to, unsigned long from,
	unsigned long size);
bool is_tegra_debug_uartport_hs(void);
int get_tegra_uart_debug_port_id(void);
int arb_lost_recovery(int scl_gpio, int sda_gpio);

extern unsigned long tegra_bootloader_fb_start;
extern unsigned long tegra_bootloader_fb_size;
extern unsigned long tegra_fb_start;
extern unsigned long tegra_fb_size;
extern unsigned long tegra_fb2_start;
extern unsigned long tegra_fb2_size;
extern unsigned long tegra_carveout_start;
extern unsigned long tegra_carveout_size;
extern unsigned long tegra_vpr_start;
extern unsigned long tegra_vpr_size;
extern unsigned long tegra_lp0_vec_start;
extern unsigned long tegra_lp0_vec_size;
extern bool tegra_lp0_vec_relocate;
extern unsigned long tegra_grhost_aperture;
extern unsigned long g_panel_id;

extern struct sys_timer tegra_timer;

enum board_fab {
	BOARD_FAB_A = 0,
	BOARD_FAB_B,
	BOARD_FAB_C,
	BOARD_FAB_D,
};

struct board_info {
	u16 board_id;
	u16 sku;
	u8  fab;
	u8  major_revision;
	u8  minor_revision;
};

enum panel_type {
	panel_type_lvds = 0,
	panel_type_dsi,
};

enum audio_codec_type {
	audio_codec_none,
	audio_codec_wm8903,
};

enum image_type {
	system_image = 0,
	rck_image,
};

void tegra_get_board_info(struct board_info *);
void tegra_get_pmu_board_info(struct board_info *bi);
void tegra_get_display_board_info(struct board_info *bi);
void tegra_get_camera_board_info(struct board_info *bi);

#if defined CONFIG_TEGRA_CONSERVATIVE_GOV_ON_EARLY_SUSPEND \
	|| defined CONFIG_TEGRA_INTERACTIVE_GOV_ON_EARLY_SUSPEND
#define CONSERVATIVE_GOVERNOR	"conservative"
#define UP_THRESHOLD		"up_threshold"
#define DOWN_THRESHOLD		"down_threshold"
#define FREQ_STEP		"FREQ_STEP"
#define UP_THRESHOLD_VALUE	95
#define DOWN_THRESHOLD_VALUE	50
#define FREQ_STEP_VALUE		3

#define INTERACTIVE_GOVERNOR	"interactive"
#define BOOST_FACTOR 		"boost_factor"
#define GO_MAXSPEED_LOAD 	"go_maxspeed_load"
#define MAX_BOOST		"max_boost"
#define MIN_SAMPLE_TIME		"min_sample_time"
#define SUSTAIN_LOAD 		"sustain_load"
#define BOOST_FACTOR_VALUE	2
#define GO_MAXSPEED_LOAD_VALUE 	97
#define MAX_BOOST_VALUE		180000
#define MIN_SAMPLE_TIME_VALUE	20000
#define SUSTAIN_LOAD_VALUE 	95

#define CPUFREQ_SYSFS_PLACE_HOLDER \
		"/sys/devices/system/cpu/cpu%i/cpufreq/scaling_governor"
#define CPUFREQ_GOV_PARAM "/sys/devices/system/cpu/cpufreq/%s/%s"

void cpufreq_save_governor(void);
void cpufreq_restore_governor(void);
void cpufreq_set_governor(char *governor);
void cpufreq_set_governor_param(char *governor, char *name, int value);
#endif

int get_core_edp(void);
enum panel_type get_panel_type(void);
int tegra_get_modem_id(void);
int tegra_get_commchip_id(void);
enum power_supply_type get_power_supply_type(void);
enum audio_codec_type get_audio_codec_type(void);
int get_maximum_cpu_current_supported(void);
enum image_type get_tegra_image_type(void);

void nvdumper_reserve_init(void);
void* get_reboot_params(void);
void* get_last_reboot_params(void);
extern int dying_processors_read_proc(char *page, char **start, off_t off,
		int count, int *eof, void *data);
#endif
