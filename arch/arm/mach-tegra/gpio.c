/*
 * arch/arm/mach-tegra/gpio.c
 *
 * Copyright (c) 2010 Google, Inc
 *
 * Author:
 *	Erik Gilling <konkers@google.com>
 *
 * Copyright (c) 2011 NVIDIA Corporation.
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

#include <linux/init.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/module.h>

#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/syscore_ops.h>

#include <asm/mach/irq.h>

#include <mach/iomap.h>
#include "pm-irq.h"
#include <mach/pinmux.h>

#include "gpio-names.h"
#include "htc-gpio.h"
#include <mach/board_htc.h>

#define GPIO_BANK(x)		((x) >> 5)
#define GPIO_PORT(x)		(((x) >> 3) & 0x3)
#define GPIO_BIT(x)		((x) & 0x7)

#define GPIO_CNF(x)		(GPIO_REG(x) + 0x00)
#define GPIO_OE(x)		(GPIO_REG(x) + 0x10)
#define GPIO_OUT(x)		(GPIO_REG(x) + 0X20)
#define GPIO_IN(x)		(GPIO_REG(x) + 0x30)
#define GPIO_INT_STA(x)		(GPIO_REG(x) + 0x40)
#define GPIO_INT_ENB(x)		(GPIO_REG(x) + 0x50)
#define GPIO_INT_LVL(x)		(GPIO_REG(x) + 0x60)
#define GPIO_INT_CLR(x)		(GPIO_REG(x) + 0x70)

#if defined(CONFIG_ARCH_TEGRA_2x_SOC)
#define GPIO_REG(x)		(IO_TO_VIRT(TEGRA_GPIO_BASE) +	\
				 GPIO_BANK(x) * 0x80 +		\
				 GPIO_PORT(x) * 4)

#define GPIO_MSK_CNF(x)		(GPIO_REG(x) + 0x800)
#define GPIO_MSK_OE(x)		(GPIO_REG(x) + 0x810)
#define GPIO_MSK_OUT(x)		(GPIO_REG(x) + 0X820)
#define GPIO_MSK_INT_STA(x)	(GPIO_REG(x) + 0x840)
#define GPIO_MSK_INT_ENB(x)	(GPIO_REG(x) + 0x850)
#define GPIO_MSK_INT_LVL(x)	(GPIO_REG(x) + 0x860)
#else
#define GPIO_REG(x)		(IO_TO_VIRT(TEGRA_GPIO_BASE) +	\
				 GPIO_BANK(x) * 0x100 +		\
				 GPIO_PORT(x) * 4)

#define GPIO_MSK_CNF(x)		(GPIO_REG(x) + 0x80)
#define GPIO_MSK_OE(x)		(GPIO_REG(x) + 0x90)
#define GPIO_MSK_OUT(x)		(GPIO_REG(x) + 0XA0)
#define GPIO_MSK_INT_STA(x)	(GPIO_REG(x) + 0xC0)
#define GPIO_MSK_INT_ENB(x)	(GPIO_REG(x) + 0xD0)
#define GPIO_MSK_INT_LVL(x)	(GPIO_REG(x) + 0xE0)
#endif

#define GPIO_INT_LVL_MASK		0x010101
#define GPIO_INT_LVL_EDGE_RISING	0x000101
#define GPIO_INT_LVL_EDGE_FALLING	0x000100
#define GPIO_INT_LVL_EDGE_BOTH		0x010100
#define GPIO_INT_LVL_LEVEL_HIGH		0x000001
#define GPIO_INT_LVL_LEVEL_LOW		0x000000
#define GPIO_DUMP_ENABLE_BIT 0x01

extern const char* enr_td_suspend_gpio_config_xc[TEGRA_NR_GPIOS];

extern const char* quo_suspend_gpio_config_xa[TEGRA_NR_GPIOS];
extern const char* quo_suspend_gpio_config_xb[TEGRA_NR_GPIOS];
extern const char* quo_suspend_gpio_config_xc[TEGRA_NR_GPIOS];
extern const char* quo_suspend_gpio_config_xd[TEGRA_NR_GPIOS];

static int gpio_dump_enable = 0;

struct tegra_gpio_bank {
	int bank;
	int irq;
	spinlock_t lvl_lock[4];
#ifdef CONFIG_PM_SLEEP
	u32 cnf[4];
	u32 out[4];
	u32 oe[4];
	u32 int_enb[4];
	u32 int_lvl[4];
#endif
};

static struct tegra_gpio_bank tegra_gpio_banks[] = {
	{.bank = 0, .irq = INT_GPIO1},
	{.bank = 1, .irq = INT_GPIO2},
	{.bank = 2, .irq = INT_GPIO3},
	{.bank = 3, .irq = INT_GPIO4},
	{.bank = 4, .irq = INT_GPIO5},
	{.bank = 5, .irq = INT_GPIO6},
	{.bank = 6, .irq = INT_GPIO7},
#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	{.bank = 7, .irq = INT_GPIO8},
#endif
};

static int tegra_gpio_compose(int bank, int port, int bit)
{
	return (bank << 5) | ((port & 0x3) << 3) | (bit & 0x7);
}

void tegra_gpio_set_tristate(int gpio_nr, enum tegra_tristate ts)
{
	int pin_group  =  tegra_pinmux_get_pingroup(gpio_nr);
	tegra_pinmux_set_tristate(pin_group, ts);
}

static void tegra_gpio_mask_write(u32 reg, int gpio, int value)
{
	u32 val;

	val = 0x100 << GPIO_BIT(gpio);
	if (value)
		val |= 1 << GPIO_BIT(gpio);
	__raw_writel(val, reg);
}

int tegra_gpio_get_bank_int_nr(int gpio)
{
	int bank;
	int irq;
	if (gpio >= TEGRA_NR_GPIOS) {
		pr_warn("%s : Invalid gpio ID - %d\n", __func__, gpio);
		return -EINVAL;
	}
	bank = gpio >> 5;
	irq = tegra_gpio_banks[bank].irq;
	return irq;
}

void tegra_gpio_enable(int gpio)
{
	if (gpio >= TEGRA_NR_GPIOS) {
		pr_warn("%s : Invalid gpio ID - %d\n", __func__, gpio);
		return;
	}
	tegra_gpio_mask_write(GPIO_MSK_CNF(gpio), gpio, 1);
}
EXPORT_SYMBOL_GPL(tegra_gpio_enable);

void tegra_gpio_disable(int gpio)
{
	if (gpio >= TEGRA_NR_GPIOS) {
		pr_warn("%s : Invalid gpio ID - %d\n", __func__, gpio);
		return;
	}
	tegra_gpio_mask_write(GPIO_MSK_CNF(gpio), gpio, 0);
}
EXPORT_SYMBOL_GPL(tegra_gpio_disable);

void tegra_gpio_init_configure(unsigned gpio, bool is_input, int value)
{
	if (gpio >= TEGRA_NR_GPIOS) {
		pr_warn("%s : Invalid gpio ID - %d\n", __func__, gpio);
		return;
	}
	if (is_input) {
		tegra_gpio_mask_write(GPIO_MSK_OE(gpio), gpio, 0);
	} else {
		tegra_gpio_mask_write(GPIO_MSK_OUT(gpio), gpio, value);
		tegra_gpio_mask_write(GPIO_MSK_OE(gpio), gpio, 1);
	}
	tegra_gpio_mask_write(GPIO_MSK_CNF(gpio), gpio, 1);
}

static void tegra_gpio_set(struct gpio_chip *chip, unsigned offset, int value)
{
	tegra_gpio_mask_write(GPIO_MSK_OUT(offset), offset, value);
}

static int tegra_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	if ((__raw_readl(GPIO_OE(offset)) >> GPIO_BIT(offset)) & 0x1)
		return (__raw_readl(GPIO_OUT(offset)) >>
			GPIO_BIT(offset)) & 0x1;
	return (__raw_readl(GPIO_IN(offset)) >> GPIO_BIT(offset)) & 0x1;
}

static int tegra_gpio_direction_input(struct gpio_chip *chip, unsigned offset)
{
	tegra_gpio_mask_write(GPIO_MSK_OE(offset), offset, 0);
	return 0;
}

static int tegra_gpio_direction_output(struct gpio_chip *chip, unsigned offset,
					int value)
{
	tegra_gpio_set(chip, offset, value);
	tegra_gpio_mask_write(GPIO_MSK_OE(offset), offset, 1);
	return 0;
}

int tegra_gpio_to_int_pin(int gpio)
{
	if (gpio < TEGRA_NR_GPIOS)
		return tegra_gpio_banks[gpio >> 5].irq;

	return -EIO;
}


static struct gpio_chip tegra_gpio_chip = {
	.label			= "tegra-gpio",
	.direction_input	= tegra_gpio_direction_input,
	.get			= tegra_gpio_get,
	.direction_output	= tegra_gpio_direction_output,
	.set			= tegra_gpio_set,
	.base			= 0,
	.ngpio			= TEGRA_NR_GPIOS,
};

static void tegra_gpio_irq_ack(struct irq_data *d)
{
	int gpio = d->irq - INT_GPIO_BASE;

	__raw_writel(1 << GPIO_BIT(gpio), GPIO_INT_CLR(gpio));

#ifdef CONFIG_TEGRA_FPGA_PLATFORM
	/* FPGA platforms have a serializer between the GPIO
	   block and interrupt controller. Allow time for
	   clearing of the GPIO interrupt to propagate to the
	   interrupt controller before re-enabling the IRQ
	   to prevent double interrupts. */
	udelay(15);
#endif
}

static void tegra_gpio_irq_mask(struct irq_data *d)
{
	int gpio = d->irq - INT_GPIO_BASE;

	tegra_gpio_mask_write(GPIO_MSK_INT_ENB(gpio), gpio, 0);
}

static void tegra_gpio_irq_unmask(struct irq_data *d)
{
	int gpio = d->irq - INT_GPIO_BASE;

	tegra_gpio_mask_write(GPIO_MSK_INT_ENB(gpio), gpio, 1);
}

static int tegra_gpio_irq_set_type(struct irq_data *d, unsigned int type)
{
	int gpio = d->irq - INT_GPIO_BASE;
	struct tegra_gpio_bank *bank = irq_data_get_irq_chip_data(d);
	int port = GPIO_PORT(gpio);
	int lvl_type;
	int val;
	unsigned long flags;

	switch (type & IRQ_TYPE_SENSE_MASK) {
	case IRQ_TYPE_EDGE_RISING:
		lvl_type = GPIO_INT_LVL_EDGE_RISING;
		break;

	case IRQ_TYPE_EDGE_FALLING:
		lvl_type = GPIO_INT_LVL_EDGE_FALLING;
		break;

	case IRQ_TYPE_EDGE_BOTH:
		lvl_type = GPIO_INT_LVL_EDGE_BOTH;
		break;

	case IRQ_TYPE_LEVEL_HIGH:
		lvl_type = GPIO_INT_LVL_LEVEL_HIGH;
		break;

	case IRQ_TYPE_LEVEL_LOW:
		lvl_type = GPIO_INT_LVL_LEVEL_LOW;
		break;

	default:
		return -EINVAL;
	}

	spin_lock_irqsave(&bank->lvl_lock[port], flags);

	val = __raw_readl(GPIO_INT_LVL(gpio));
	val &= ~(GPIO_INT_LVL_MASK << GPIO_BIT(gpio));
	val |= lvl_type << GPIO_BIT(gpio);
	__raw_writel(val, GPIO_INT_LVL(gpio));

	spin_unlock_irqrestore(&bank->lvl_lock[port], flags);

	if (type & (IRQ_TYPE_LEVEL_LOW | IRQ_TYPE_LEVEL_HIGH))
		__irq_set_handler_locked(d->irq, handle_level_irq);
	else if (type & (IRQ_TYPE_EDGE_FALLING | IRQ_TYPE_EDGE_RISING))
		__irq_set_handler_locked(d->irq, handle_edge_irq);

	tegra_pm_irq_set_wake_type(d->irq, type);

	return 0;
}

static void tegra_gpio_irq_handler(unsigned int irq, struct irq_desc *desc)
{
	struct tegra_gpio_bank *bank;
	int port;
	int pin;
	struct irq_chip *chip = irq_desc_get_chip(desc);

	chained_irq_enter(chip, desc);

	bank = irq_get_handler_data(irq);

	for (port = 0; port < 4; port++) {
		int gpio = tegra_gpio_compose(bank->bank, port, 0);
		unsigned long sta = __raw_readl(GPIO_INT_STA(gpio)) &
			__raw_readl(GPIO_INT_ENB(gpio));

		for_each_set_bit(pin, &sta, 8)
			generic_handle_irq(gpio_to_irq(gpio + pin));
	}

	chained_irq_exit(chip, desc);

}

#ifdef CONFIG_PM_SLEEP
static void tegra_gpio_resume(void)
{
	unsigned long flags;
	int b;
	int p;

	local_irq_save(flags);

	for (b = 0; b < ARRAY_SIZE(tegra_gpio_banks); b++) {
		struct tegra_gpio_bank *bank = &tegra_gpio_banks[b];

		for (p = 0; p < ARRAY_SIZE(bank->oe); p++) {
			unsigned int gpio = (b<<5) | (p<<3);
			__raw_writel(bank->cnf[p], GPIO_CNF(gpio));
			__raw_writel(bank->out[p], GPIO_OUT(gpio));
			__raw_writel(bank->oe[p], GPIO_OE(gpio));
			__raw_writel(bank->int_lvl[p], GPIO_INT_LVL(gpio));
			__raw_writel(bank->int_enb[p], GPIO_INT_ENB(gpio));
		}
	}
#if defined(CONFIG_MACH_ENDEAVORTD)
	int projectPhase = htc_get_pcbid_info();
	if (projectPhase == PROJECT_PHASE_XC){
		enr_xc_no_owner_gpio_resume();
	}
#endif
	local_irq_restore(flags);
}

static char GPIO_STATE[7][10] =
{
	"A",
	"O(H)",
	"O(L)",
	"I(NP)",
	"I(PD)",
	"I(PU)",
	"A(PU)"
};

static int gpio_config_state(int cnf, int oe, int out, int pupd)
{
	if ((cnf & 0x01) ==  1 && ( oe & 0x01) == 1) {
		if ((out & 0x01) == 1)
			return 1;
		else
			return 2;
	} else if ((cnf & 0x01) ==  1 && (oe & 0x01) == 0) {
		if (pupd == TEGRA_PUPD_NORMAL)
			return 3;
		else if (pupd == TEGRA_PUPD_PULL_DOWN)
			return 4;
		else if ( pupd == TEGRA_PUPD_PULL_UP)
			return 5;
	} else {
		if ( pupd == TEGRA_PUPD_PULL_UP)
			return 6;
	}
	return 0;
}

void gpio_dump(void)
{
    int b, p, i;
    unsigned long flags;

    for (b = 0; b < ARRAY_SIZE(tegra_gpio_banks); b++) {
	struct tegra_gpio_bank *bank = &tegra_gpio_banks[b];

		const char** expected_gpio = enr_td_suspend_gpio_config_xc;

#if defined(CONFIG_MACH_VERTEXF)
			int projectPhase = htc_get_pcbid_info();

			if (projectPhase == PROJECT_PHASE_XA) /* EVT XA */
				expected_gpio = quo_suspend_gpio_config_xa;
			else if (projectPhase == PROJECT_PHASE_XB)
				expected_gpio = quo_suspend_gpio_config_xb;
			else if (projectPhase == PROJECT_PHASE_XC)
				expected_gpio = quo_suspend_gpio_config_xc;
			else if (projectPhase >= PROJECT_PHASE_XD)
				expected_gpio = quo_suspend_gpio_config_xd;

#endif

		for (p = 0; p < ARRAY_SIZE(bank->oe); p++) {
			unsigned int gpio = (b<<5) | (p<<3);
			char port[2];
			if((b*4+p) < 26)
				sprintf(port, "%c", (b*4+p)+'A');
			else
				sprintf(port, "%c%c", ((b*4+p)%26)+'A', ((b*4+p)%26)+'A');

			int gpio_cnf = __raw_readl(GPIO_CNF(gpio));
			int gpio_out = __raw_readl(GPIO_OUT(gpio));
			int gpio_oe = __raw_readl(GPIO_OE(gpio));

			int i;

			for (i = 0; i < 8; i++) {
				enum tegra_pingroup ball = gpio_to_pingroup[gpio+i];
				int reg = tegra_pinmux_get_pullupdown(ball);
				int tristate = tegra_pinmux_get_tristate(ball);
				int io_enable = tegra_pinmux_get_io(ball);

				char* reg_gpio_config = GPIO_STATE[gpio_config_state(gpio_cnf, gpio_oe, gpio_out, reg %3)];
				printk("%s%d %s", port, i, reg_gpio_config);

				if(gpio+i < TEGRA_GPIO_INVALID){
					if (strcmp(reg_gpio_config, expected_gpio[gpio+i]) != 0){
						printk(" *%s %s", expected_gpio[gpio+i], tegra_soc_pingroups[ball].name);
					}
				}

				if(gpio_oe==1&&tristate==1)
					printk(" !tristate");
				if(reg_gpio_config[0]!='I' && reg == 1)
					printk(" pd");
				if(reg_gpio_config[0]!='I' && reg == 2)
					printk(" pu");

				if(reg_gpio_config[0]=='I' && io_enable==0)
					printk(" !inputDisable");

				printk("\n");
				gpio_cnf = gpio_cnf >> 1;
				gpio_out = gpio_out >> 1;
				gpio_oe = gpio_oe >> 1;
			}
		}

	}
}

static int tegra_gpio_suspend(void)
{
	unsigned long flags;
	int b;
	int p;

	local_irq_save(flags);
	for (b = 0; b < ARRAY_SIZE(tegra_gpio_banks); b++) {
		struct tegra_gpio_bank *bank = &tegra_gpio_banks[b];

		for (p = 0; p < ARRAY_SIZE(bank->oe); p++) {
			unsigned int gpio = (b<<5) | (p<<3);
			bank->cnf[p] = __raw_readl(GPIO_CNF(gpio));
			bank->out[p] = __raw_readl(GPIO_OUT(gpio));
			bank->oe[p] = __raw_readl(GPIO_OE(gpio));
			bank->int_enb[p] = __raw_readl(GPIO_INT_ENB(gpio));
			bank->int_lvl[p] = __raw_readl(GPIO_INT_LVL(gpio));
		}
	}


	local_irq_restore(flags);
   /*local_gpio_config_test(quo_suspend_gpio_config_xb);*/

	// TODO gpio_basic_during_suspend
	gpio_basic_during_suspend();

	if (gpio_dump_enable)
		gpio_dump();

	return 0;
}

static int tegra_gpio_irq_set_wake(struct irq_data *d, unsigned int enable)
{
	struct tegra_gpio_bank *bank = irq_data_get_irq_chip_data(d);
	int ret = 0;

	ret = tegra_pm_irq_set_wake(d->irq, enable);

	if (ret)
		return ret;

	ret = irq_set_irq_wake(bank->irq, enable);

	if (ret)
		tegra_pm_irq_set_wake(d->irq, !enable);

	return ret;
}
#else
#define tegra_gpio_irq_set_wake NULL
#define tegra_gpio_suspend NULL
#define tegra_gpio_resume NULL
#endif

static struct syscore_ops tegra_gpio_syscore_ops = {
	.suspend = tegra_gpio_suspend,
	.resume = tegra_gpio_resume,
};

int tegra_gpio_resume_init(void)
{
	register_syscore_ops(&tegra_gpio_syscore_ops);

	return 0;
}

static struct irq_chip tegra_gpio_irq_chip = {
	.name		= "GPIO",
	.irq_ack	= tegra_gpio_irq_ack,
	.irq_mask	= tegra_gpio_irq_mask,
	.irq_unmask	= tegra_gpio_irq_unmask,
	.irq_set_type	= tegra_gpio_irq_set_type,
	.irq_set_wake	= tegra_gpio_irq_set_wake,
	.flags		= IRQCHIP_MASK_ON_SUSPEND,
};


/* This lock class tells lockdep that GPIO irqs are in a different
 * category than their parents, so it won't report false recursion.
 */
static struct lock_class_key gpio_lock_class;

static int __init tegra_gpio_init(void)
{
	struct tegra_gpio_bank *bank;
	int i;
	int j;

	for (i = 0; i < ARRAY_SIZE(tegra_gpio_banks); i++) {
		for (j = 0; j < 4; j++) {
			int gpio = tegra_gpio_compose(i, j, 0);
			__raw_writel(0x00, GPIO_INT_ENB(gpio));
			__raw_writel(0x00, GPIO_INT_STA(gpio));
		}
	}

	gpiochip_add(&tegra_gpio_chip);

	for (i = INT_GPIO_BASE; i < (INT_GPIO_BASE + TEGRA_NR_GPIOS); i++) {
		bank = &tegra_gpio_banks[GPIO_BANK(irq_to_gpio(i))];

		irq_set_lockdep_class(i, &gpio_lock_class);
		irq_set_chip_data(i, bank);
		irq_set_chip_and_handler(i, &tegra_gpio_irq_chip,
					 handle_simple_irq);
		set_irq_flags(i, IRQF_VALID);
	}

	for (i = 0; i < ARRAY_SIZE(tegra_gpio_banks); i++) {
		bank = &tegra_gpio_banks[i];

		for (j = 0; j < 4; j++)
			spin_lock_init(&bank->lvl_lock[j]);

		irq_set_handler_data(bank->irq, bank);
		irq_set_chained_handler(bank->irq, tegra_gpio_irq_handler);

	}

	if (get_extra_kernel_flag() & GPIO_DUMP_ENABLE_BIT)
		gpio_dump_enable = 1;

	return 0;
}

postcore_initcall(tegra_gpio_init);

void __init tegra_gpio_config(struct tegra_gpio_table *table, int num)
{
	int i;

	for (i = 0; i < num; i++) {
		int gpio = table[i].gpio;

		if (table[i].enable)
			tegra_gpio_enable(gpio);
		else
			tegra_gpio_disable(gpio);
	}
}

#ifdef	CONFIG_DEBUG_FS

#include <linux/debugfs.h>
#include <linux/seq_file.h>

static int dbg_gpio_show(struct seq_file *s, void *unused)
{
	int i;
	int j;

	seq_printf(s, "Bank:Port CNF OE OUT IN INT_STA INT_ENB INT_LVL\n");
	for (i = 0; i < ARRAY_SIZE(tegra_gpio_banks); i++) {
		for (j = 0; j < 4; j++) {
			int gpio = tegra_gpio_compose(i, j, 0);
			seq_printf(s,
				"%d:%d %02x %02x %02x %02x %02x %02x %06x\n",
				i, j,
				__raw_readl(GPIO_CNF(gpio)),
				__raw_readl(GPIO_OE(gpio)),
				__raw_readl(GPIO_OUT(gpio)),
				__raw_readl(GPIO_IN(gpio)),
				__raw_readl(GPIO_INT_STA(gpio)),
				__raw_readl(GPIO_INT_ENB(gpio)),
				__raw_readl(GPIO_INT_LVL(gpio)));
		}
	}
	return 0;
}

static int htc_dbg_gpio_show(struct seq_file *s, void *unused)
{
#define MSG_DEL "\t"
	extern const int gpio_to_pingroup[TEGRA_MAX_GPIO];
	int i;
	int j;
	int b;

	for (i = 0; i < ARRAY_SIZE(tegra_gpio_banks); i++) {
		for (j = 0; j < 4; j++) {
			int gpio, gpio_cnf, gpio_out, gpio_oe;
			char port[2];
			if ((i*4+j) < 26)
				sprintf(port, "%c", (i*4+j)+'A');
			else
				sprintf(port, "%c%c", ((i*4+j)%26)+'A', ((i*4+j)%26)+'A');

			gpio     = tegra_gpio_compose(i, j, 0);
			gpio_cnf = __raw_readl(GPIO_CNF(gpio));
			gpio_out = __raw_readl(GPIO_OUT(gpio));
			gpio_oe  = __raw_readl(GPIO_OE(gpio));
			for (b = 0; b < 8; b++) {
				char msg[100];
				enum tegra_pingroup pin = gpio_to_pingroup[gpio+b];
				int mux = tegra_pinmux_get_func(pin);
				int reg = tegra_pinmux_get_pullupdown(pin);
				int tristate = tegra_pinmux_get_tristate(pin);
				int io_enable = tegra_pinmux_get_io(pin);

				sprintf(msg, "%s%d", port, b);

				if ((gpio_cnf & 0x01) == 1)
				{
					if ((gpio_oe & 0x01) == 1)
					{
						if ((gpio_out & 0x01) == 1)
							strcat(msg, MSG_DEL "O(H)");
						else
							strcat(msg, MSG_DEL "O(L)");

						if (tristate)
							strcat(msg, MSG_DEL "tristate!");

						if (reg == TEGRA_PUPD_PULL_DOWN)
							strcat(msg, MSG_DEL "pull-down!");
						else if (reg == TEGRA_PUPD_PULL_UP)
							strcat(msg, MSG_DEL "pull-up!");
					}
					else
					{
						if (reg % 3 == TEGRA_PUPD_NORMAL)
							strcat(msg, MSG_DEL "I(NP)");
						else if (reg % 3 == TEGRA_PUPD_PULL_DOWN)
							strcat(msg, MSG_DEL "I(PD)");
						else if (reg % 3 == TEGRA_PUPD_PULL_UP)
							strcat(msg, MSG_DEL "I(PU)");

						if (io_enable == 0)
							strcat(msg, MSG_DEL "input-disabled!");
					}
				} else if((gpio_cnf & 0x01) ==  0) {

					char buf[10];
					sprintf(buf, MSG_DEL "A" MSG_DEL "SFIO%d", mux);
					strcat(msg, buf);
					// TODO display function name

					if ((!tristate) & (!io_enable))
						strcat(msg, MSG_DEL "output");
					else if (tristate & io_enable)
						strcat(msg, MSG_DEL "input");
					else if ((!tristate) & io_enable)
						strcat(msg, MSG_DEL "bi-direction");
					else if (tristate & (!io_enable))
						strcat(msg, MSG_DEL "disabled");

					if (reg == TEGRA_PUPD_PULL_DOWN)
						strcat(msg, MSG_DEL "pull-down");
					else if (reg == TEGRA_PUPD_PULL_UP)
						strcat(msg, MSG_DEL "pull-up");
				}

				seq_printf(s, "%s\n", msg);

				gpio_cnf = gpio_cnf >> 1;
				gpio_out = gpio_out >> 1;
				gpio_oe = gpio_oe >> 1;
			}
			seq_printf(s, "\n");
		}
	}
	return 0;
}


static int dbg_gpio_open(struct inode *inode, struct file *file)
{
	return single_open(file, dbg_gpio_show, &inode->i_private);
}

static int htc_dbg_gpio_open(struct inode *inode, struct file *file)
{
	return single_open(file, htc_dbg_gpio_show, &inode->i_private);
}


static const struct file_operations debug_fops = {
	.open		= dbg_gpio_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};
static const struct file_operations htc_debug_fops = {
	.open		= htc_dbg_gpio_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};


static int __init tegra_gpio_debuginit(void)
{
	(void) debugfs_create_file("tegra_gpio", S_IRUGO,
					NULL, NULL, &debug_fops);
	(void) debugfs_create_file("htc_tegra_gpio", S_IRUGO,
					NULL, NULL, &htc_debug_fops);
	return 0;
}
late_initcall(tegra_gpio_debuginit);
#endif
