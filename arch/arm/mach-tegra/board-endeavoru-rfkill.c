/*
 * Copyright (C) 2009 Google, Inc.
 * Copyright (C) 2009-2011 HTC Corporation.
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

#include <linux/err.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/rfkill.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <asm/mach-types.h>
#include "board-endeavoru.h"
#include "gpio-names.h"

static struct rfkill *bt_rfk;
static const char bt_name[] = "bcm4334";

#ifdef PWR_DEVICE_TAG
#undef PWR_DEVICE_TAG
#endif
#define PWR_DEVICE_TAG "BT"

static void endeavoru_config_bt_on(void)
{
	printk(KERN_INFO "[BT]== R ON ==\n");

	tegra_gpio_disable(ENDEAVORU_GPIO_BT_UART3_CTS);
	tegra_pinmux_set_pullupdown(TEGRA_PINGROUP_UART3_CTS_N, TEGRA_PUPD_NORMAL);

	tegra_gpio_disable(ENDEAVORU_GPIO_BT_UART3_RTS);
	tegra_pinmux_set_pullupdown(TEGRA_PINGROUP_UART3_RTS_N, TEGRA_PUPD_NORMAL);

	tegra_gpio_disable(ENDEAVORU_GPIO_BT_UART3_TX);
	tegra_pinmux_set_pullupdown(TEGRA_PINGROUP_UART3_TXD, TEGRA_PUPD_NORMAL);

	tegra_gpio_disable(ENDEAVORU_GPIO_BT_UART3_RX);
	tegra_pinmux_set_pullupdown(TEGRA_PINGROUP_UART3_RXD, TEGRA_PUPD_NORMAL);

	/* BT_RTS */

	/* BT_CTS */

	/* BT_TX */

	/* BT_RX */

	/* BT_HOST_WAKE */
	tegra_pinmux_set_pullupdown(TEGRA_PINGROUP_ULPI_DATA4, TEGRA_PUPD_PULL_UP);

	/* BT_CHIP_WAKE */
	gpio_set_value(ENDEAVORU_GPIO_BT_WAKE, 1);

	/* BT_SHUTDOWN_N */
	gpio_set_value(ENDEAVORU_GPIO_BT_SHUTDOWN_N, 0);
	mdelay(5);

	/* BT_SHUTDOWN_N */
	gpio_set_value(ENDEAVORU_GPIO_BT_SHUTDOWN_N, 1);
	mdelay(1);

}

static void endeavoru_config_bt_off(void)
{

	/* BT_SHUTDOWN_N */
	gpio_set_value(ENDEAVORU_GPIO_BT_SHUTDOWN_N, 0);
	mdelay(1);

	/* UART3_CTS_N GPIO-A.01 I(PU) */
	tegra_gpio_enable(ENDEAVORU_GPIO_BT_UART3_CTS);
	gpio_direction_input(ENDEAVORU_GPIO_BT_UART3_CTS);

	tegra_pinmux_set_pullupdown(TEGRA_PINGROUP_UART3_CTS_N, TEGRA_PUPD_PULL_UP);

	/* UART3_RTS_N GPIO-C.00 O(H) */
	tegra_gpio_enable(ENDEAVORU_GPIO_BT_UART3_RTS);
	gpio_direction_output(ENDEAVORU_GPIO_BT_UART3_RTS, 1);

	tegra_pinmux_set_pullupdown(TEGRA_PINGROUP_UART3_RTS_N, TEGRA_PUPD_PULL_UP);

	/* UART3_TXD GPIO-W.06 O(H) */
	tegra_gpio_enable(ENDEAVORU_GPIO_BT_UART3_TX);
	gpio_direction_output(ENDEAVORU_GPIO_BT_UART3_TX, 1);

	tegra_pinmux_set_pullupdown(TEGRA_PINGROUP_UART3_TXD, TEGRA_PUPD_PULL_UP);

	/* UART3_RXD GPIO-W.07 I(PU) */
	tegra_gpio_enable(ENDEAVORU_GPIO_BT_UART3_RX);
	gpio_direction_input(ENDEAVORU_GPIO_BT_UART3_RX);

	tegra_pinmux_set_pullupdown(TEGRA_PINGROUP_UART3_RXD, TEGRA_PUPD_PULL_UP);

	/* BT_RTS */
	gpio_set_value(ENDEAVORU_GPIO_BT_UART3_RTS, 1);

	/* BT_CTS */

	/* BT_TX */
	gpio_set_value(ENDEAVORU_GPIO_BT_UART3_TX, 1);

	/* BT_RX */

	/* BT_HOST_WAKE */
	tegra_pinmux_set_pullupdown(TEGRA_PINGROUP_ULPI_DATA4, TEGRA_PUPD_PULL_DOWN);

	/* BT_CHIP_WAKE */
	gpio_set_value(ENDEAVORU_GPIO_BT_WAKE, 0);

	printk(KERN_INFO "[BT]== R OFF ==\n");
}

static int bluetooth_set_power(void *data, bool blocked)
{
	if (!blocked)
		endeavoru_config_bt_on();
	else
		endeavoru_config_bt_off();

	return 0;
}

static const struct rfkill_ops endeavoru_rfkill_ops = {
	.set_block = bluetooth_set_power,
};

static int endeavoru_rfkill_probe(struct platform_device *pdev)
{
	int ret = 0;
	int err = 0;
	bool default_state = true; /* off */

	pr_info("[BT]%s: starting initialize endeavoru_rfkill\n" , __func__);

	gpio_request(ENDEAVORU_GPIO_BT_SHUTDOWN_N, "bcm4334_nshutdown_gpio");
	if (err)
		pr_err("BT_SHUTDOWN_N gpio request failed:%d\n", err);
	gpio_request(ENDEAVORU_GPIO_BT_UART3_CTS, "bcm4334_uart3_cts");
	if (err)
		pr_err("BT_CTS_N gpio request failed:%d\n", err);
	gpio_request(ENDEAVORU_GPIO_BT_UART3_RTS, "bcm4334_uart3_rts");
	if (err)
		pr_err("BT_RTS_N gpio request failed:%d\n", err);
	gpio_request(ENDEAVORU_GPIO_BT_UART3_TX, "bcm4334_uart3_tx");
	if (err)
		pr_err("BT_TXD gpio request failed:%d\n", err);
	gpio_request(ENDEAVORU_GPIO_BT_UART3_RX, "bcm4334_uart3_rx");
	if (err)
		pr_err("BT_RXD gpio request failed:%d\n", err);

	tegra_gpio_enable(ENDEAVORU_GPIO_BT_SHUTDOWN_N);
	gpio_direction_output(ENDEAVORU_GPIO_BT_SHUTDOWN_N, 0);

	bluetooth_set_power(NULL, default_state);

	bt_rfk = rfkill_alloc(bt_name, &pdev->dev, RFKILL_TYPE_BLUETOOTH,
		&endeavoru_rfkill_ops, NULL);

	if (!bt_rfk) {
		ret = -ENOMEM;
		goto err_rfkill_alloc;
	}

	rfkill_set_states(bt_rfk, default_state, false);

	/* userspace cannot take exclusive control */

	ret = rfkill_register(bt_rfk);
	if (ret)
		goto err_rfkill_reg;

	return 0;

err_rfkill_reg:
	rfkill_destroy(bt_rfk);
err_rfkill_alloc:
	return ret;

}

static int endeavoru_rfkill_remove(struct platform_device *pdev)
{
	rfkill_unregister(bt_rfk);
	rfkill_destroy(bt_rfk);

	return 0;
}

static struct platform_driver endeavoru_rfkill_driver = {
	.probe = endeavoru_rfkill_probe,
	.remove = endeavoru_rfkill_remove,
	.driver = {
		   .name = "endeavoru_rfkill",
		   .owner = THIS_MODULE,
	},
};

static int __init endeavoru_rfkill_init(void)
{
	if(!machine_is_endeavoru())
		return 0;
	return platform_driver_register(&endeavoru_rfkill_driver);
}

static void __exit endeavoru_rfkill_exit(void)
{
	platform_driver_unregister(&endeavoru_rfkill_driver);
}

module_init(endeavoru_rfkill_init);
module_exit(endeavoru_rfkill_exit);
MODULE_DESCRIPTION("endeavoru rfkill");
MODULE_AUTHOR("htc_ssdbt <htc_ssdbt@htc.com>");
MODULE_LICENSE("GPL");
