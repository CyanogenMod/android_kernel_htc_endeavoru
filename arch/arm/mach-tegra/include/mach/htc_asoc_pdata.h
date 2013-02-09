/*
 * arch/arm/mach-tegra/include/mach/htc_asoc_pdata.h
 *
 * Copyright 2012 HTC, Inc.
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
#include <linux/spinlock_types.h>

struct aic3008_power {
	spinlock_t spin_lock;
	bool isPowerOn;
	bool mic_switch;
	bool amp_switch;
	bool i2s_switch;
	bool hs_vol_control;
	void (*powerinit)(void);
	void (*resume)(void);
	void (*suspend)(void);
	void (*mic_powerup)(void);
	void (*mic_powerdown)(void);
	void (*amp_powerup)(int);
	void (*amp_powerdown)(int);
	void (*i2s_control)(int);
	void (*headset_vol_control)(int);
	void (*modem_coredump)(void);
};

struct htc_asoc_platform_data {
	struct aic3008_power aic3008_power;
};
