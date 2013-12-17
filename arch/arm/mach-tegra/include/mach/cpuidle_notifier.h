/*
 * arch/arm/mach-tegra/cpuidle_notifier.h
 *
 * Declarations for cpu idle notifier
 *
 * Copyright (c) 2012, hTC Corporation.
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
#pragma once

#include <linux/notifier.h>
#include <linux/types.h>
#include <linux/cpuidle.h>

enum {
	TEGRA_CPUIDLE_STATE_ENTER,
	TEGRA_CPUIDLE_STATE_EXIT
};

int tegra_cpuidle_register_notifier(unsigned int cpu,
		struct notifier_block *nb);
int tegra_cpuidle_unregister_notifier(unsigned int cpu,
		struct notifier_block *nb);

void tegra_cpuidle_enter(void);
void tegra_cpuidle_exit(void);


