/*
 * arch/arm/mach-tegra/touch.h
 *
 * Copyright (C) 2010 Google, Inc.
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

#ifndef _MACH_TEGRA_TOUCH_H
#define _MACH_TEGRA_TOUCH_H

struct gpio_setvalue {
	bool valid;
	unsigned gpio;
	int value;
	int delay;
};

struct i2c_touch_info {
	int busnum;
	struct i2c_board_info const *info;
	unsigned n;
};

struct tegra_touchscreen_init {
	int	irq_gpio;
	int	rst_gpio;
	struct gpio_setvalue	sv_gpio1;
	struct gpio_setvalue	sv_gpio2;
	struct i2c_touch_info	ts_boardinfo;
};

#endif	/* _MACH_TEGRA_TOUCH_H	*/
