/*
 * include/asm-arm/arch-tegra/include/mach/sdhci.h
 *
 * Copyright (C) 2009 Palm, Inc.
 * Author: Yvonne Yip <y@palm.com>
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
#ifndef __ASM_ARM_ARCH_TEGRA_SDHCI_H
#define __ASM_ARM_ARCH_TEGRA_SDHCI_H

#include <linux/mmc/host.h>
#include <asm/mach/mmc.h>

struct tegra_sdhci_platform_data {
	int cd_gpio;
	int wp_gpio;
	int power_gpio;
	int is_8bit;
	unsigned int max_clk_limit;
	unsigned int tap_delay;
	struct mmc_platform_data mmc_data;
	int (*suspend_gpiocfg)(void);
	void (*resume_gpiocfg)(void);
};

#define ENABLE_GPIO(_pg, _pin, _name, _direction, _state, _pupd) \
    enable_gpio_config(TEGRA_PINGROUP_##_pg, TEGRA_GPIO_P##_pin, _name, _direction, _state, TEGRA_PUPD_##_pupd);

#define DISABLE_GPIO(_pg, _pin, _pupd) \
    disable_gpio_config(TEGRA_PINGROUP_##_pg, TEGRA_GPIO_P##_pin, TEGRA_PUPD_##_pupd);

static int inline enable_gpio_config(enum tegra_pingroup pg, int pin, const char *name,
    int direction, int state, enum tegra_pullupdown pupd)
{
	int ret;

	ret = gpio_request(pin, name);
	if(ret < 0) {
		pr_err("%s: gpio (%s) request failed (return=%d) \r\n", __func__, name, ret);
		return -1;
	}

	if (direction)
		ret = gpio_direction_input(pin);
	else
		ret = gpio_direction_output(pin, state);

	if(ret < 0) {
		pr_err("%s: gpio (%s) direction  failed (return=%d) \r\n", __func__, name, ret);
		gpio_free(pin);
		return -1;
	}
	tegra_pinmux_set_pullupdown(pg, pupd);
	tegra_gpio_enable(pin);
	return 0;
}

static void inline disable_gpio_config(enum tegra_pingroup pg, int pin, enum tegra_pullupdown pupd)
{
	tegra_pinmux_set_pullupdown(pg, pupd);
	tegra_gpio_disable(pin);
	gpio_free(pin);
}
#endif
