/*
 * arch/arm/mach-tegra/cpuidle_notifier.c
 *
 * Implementations of cpu idle notifier
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/cpuidle.h>
#include <mach/cpuidle_notifier.h>"

static DEFINE_PER_CPU(struct atomic_notifier_head, tegra_cpuidle_notifiers);

int tegra_cpuidle_register_notifier(unsigned int cpu, struct notifier_block *nb)
{
	struct atomic_notifier_head *head =
		&per_cpu(tegra_cpuidle_notifiers, cpu);

	return atomic_notifier_chain_register(head, nb);
}
EXPORT_SYMBOL(tegra_cpuidle_register_notifier);

int tegra_cpuidle_unregister_notifier(unsigned int cpu, struct notifier_block *nb)
{
	struct atomic_notifier_head *head =
		&per_cpu(tegra_cpuidle_notifiers, cpu);

	return atomic_notifier_chain_unregister(head, nb);
}
EXPORT_SYMBOL(tegra_cpuidle_unregister_notifier);

void tegra_cpuidle_enter(void) {
    unsigned long flags;
    struct atomic_notifier_head *head =
			&__get_cpu_var(tegra_cpuidle_notifiers);

    local_irq_save(flags);
    atomic_notifier_call_chain(head, TEGRA_CPUIDLE_STATE_ENTER, NULL);
    local_irq_restore(flags);
}
EXPORT_SYMBOL(tegra_cpuidle_enter);

void tegra_cpuidle_exit(void) {
    struct atomic_notifier_head *head =
			&__get_cpu_var(tegra_cpuidle_notifiers);

    local_irq_disable();
    atomic_notifier_call_chain(head, TEGRA_CPUIDLE_STATE_EXIT, NULL);
    local_irq_enable();
}
EXPORT_SYMBOL(tegra_cpuidle_exit);

static int __init tegra_cpuidle_early_init(void)
{
	unsigned int cpu;

	for_each_possible_cpu(cpu)
		ATOMIC_INIT_NOTIFIER_HEAD(&per_cpu(tegra_cpuidle_notifiers, cpu));

	return 0;
}
early_initcall(tegra_cpuidle_early_init);

