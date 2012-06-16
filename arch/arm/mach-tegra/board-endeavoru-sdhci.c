/*
 * arch/arm/mach-tegra/board-endeavoru-sdhci.c
 *
 * Copyright (C) 2011 NVIDIA Corporation.
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

#include <linux/resource.h>
#include <linux/platform_device.h>
#include <linux/wlan_plat.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/mmc/host.h>

#include <asm/mach-types.h>
#include <mach/irqs.h>
#include <mach/iomap.h>
#include <mach/sdhci.h>

#include "gpio-names.h"
#include "board.h"

/* HTC_WIFI_START */
#include <linux/wl12xx.h>
/* HTC_WIFI_END */

#define ENTERPRISE_WLAN_PWR	TEGRA_GPIO_PV2
#define ENTERPRISE_WLAN_RST	TEGRA_GPIO_PV3
#define ENTERPRISE_WLAN_WOW	TEGRA_GPIO_PO4

//#define ENTERPRISE_SD_CD TEGRA_GPIO_PI5

static void (*wifi_status_cb)(int card_present, void *dev_id);
static void *wifi_status_cb_devid;
static int enterprise_wifi_status_register(void (*callback)(int , void *), void *);

static int enterprise_wifi_reset(int on);
/* HTC_WIFI_START */
//static int enterprise_wifi_power(int on);
//static int enterprise_wifi_set_carddetect(int val);
int enterprise_wifi_power(int on);
int enterprise_wifi_set_carddetect(int val);
int enterprise_wifi_status(struct device *dev);
static int enterprise_wifi_cd;		/* WIFI virtual 'card detect' status */
/* HTC_WIFI_END */


/* HTC_WIFI_START */
static struct wl12xx_platform_data enterprise_wlan_data __initdata = {
	.irq = TEGRA_GPIO_TO_IRQ(ENTERPRISE_WLAN_WOW),
	.board_ref_clock = WL12XX_REFCLOCK_26,
	.board_tcxo_clock = 1,
//	.platform_quirks = WL12XX_PLATFORM_QUIRK_EDGE_IRQ,
};
/* HTC_WIFI_END */

static struct wifi_platform_data enterprise_wifi_control = {
	.set_power      = enterprise_wifi_power,
	.set_reset      = enterprise_wifi_reset,
	.set_carddetect = enterprise_wifi_set_carddetect,
};

static struct resource wifi_resource[] = {
	[0] = {
		.name	= "bcm4329_wlan_irq",
		.start	= TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PU6),
		.end	= TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PU6),
		.flags	= IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL | IORESOURCE_IRQ_SHAREABLE,
	},
};

static struct platform_device enterprise_wifi_device = {
	.name           = "bcm4329_wlan",
	.id             = 1,
	.num_resources	= 1,
	.resource	= wifi_resource,
	.dev            = {
		.platform_data = &enterprise_wifi_control,
	},
};

static int emmc_suspend_gpiocfg(void)
{
	ENABLE_GPIO(SDMMC4_CLK, CC4, "SDMMC4_CLK", 0, 0, NORMAL);
    return 0;
}

static void emmc_resume_gpiocfg(void)
{
	DISABLE_GPIO(SDMMC4_CLK, CC4, NORMAL);
}

// No uSD
#if 0
static struct resource sdhci_resource0[] = {
	[0] = {
		.start  = INT_SDMMC1,
		.end    = INT_SDMMC1,
		.flags  = IORESOURCE_IRQ,
	},
	[1] = {
		.start	= TEGRA_SDMMC1_BASE,
		.end	= TEGRA_SDMMC1_BASE + TEGRA_SDMMC1_SIZE-1,
		.flags	= IORESOURCE_MEM,
	},
};
#endif

static struct resource sdhci_resource2[] = {
	[0] = {
		.start  = INT_SDMMC3,
		.end    = INT_SDMMC3,
		.flags  = IORESOURCE_IRQ,
	},
	[1] = {
		.start	= TEGRA_SDMMC3_BASE,
		.end	= TEGRA_SDMMC3_BASE + TEGRA_SDMMC3_SIZE-1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct resource sdhci_resource3[] = {
	[0] = {
		.start  = INT_SDMMC4,
		.end    = INT_SDMMC4,
		.flags  = IORESOURCE_IRQ,
	},
	[1] = {
		.start	= TEGRA_SDMMC4_BASE,
		.end	= TEGRA_SDMMC4_BASE + TEGRA_SDMMC4_SIZE-1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct embedded_sdio_data embedded_sdio_data0 = {
	.cccr   = {
		.sdio_vsn       = 2,
		.multi_block    = 1,
		.low_speed      = 0,
		.wide_bus       = 0,
		.high_power     = 1,
		.high_speed     = 1,
	},
	.cis  = {
		.vendor         = 0x02d0,
		.device         = 0x4329,
	},
};

// No uSD
#if 0
static struct tegra_sdhci_platform_data tegra_sdhci_platform_data0 = {
	.mmc_data = {
		.register_status_notify	= enterprise_wifi_status_register,
		.embedded_sdio = &embedded_sdio_data0,
		/* FIXME need to revert the built_in change
		once we use get the signal strength fix of
		bcmdhd driver from broadcom for bcm4329 chipset*/
		.built_in = 0,
	},
	.cd_gpio = -1,
	.wp_gpio = -1,
	.power_gpio = -1,
	.max_clk_limit = 45000000,
};
#endif

static struct tegra_sdhci_platform_data tegra_sdhci_platform_data2 = {
	.mmc_data = {
		.status = enterprise_wifi_status,
		.register_status_notify	= enterprise_wifi_status_register,
		/* HTC_WIFI_START */
		//.embedded_sdio = &embedded_sdio_data0,
		/* HTC_WIFI_END */
		.built_in = 1,
	},
	.cd_gpio = -1,
	.wp_gpio = -1,
	.power_gpio = -1,
};

static struct tegra_sdhci_platform_data tegra_sdhci_platform_data3 = {
	.cd_gpio = -1,
	.wp_gpio = -1,
	.power_gpio = -1,
	.is_8bit = 1,
	.mmc_data = {
		.built_in = 1,
	},
	.suspend_gpiocfg = emmc_suspend_gpiocfg,
	.resume_gpiocfg = emmc_resume_gpiocfg,
};

// No uSD
#if 0
static struct platform_device tegra_sdhci_device0 = {
	.name		= "sdhci-tegra",
	.id		= 0,
	.resource	= sdhci_resource0,
	.num_resources	= ARRAY_SIZE(sdhci_resource0),
	.dev = {
		.platform_data = &tegra_sdhci_platform_data0,
	},
};
#endif

static struct platform_device tegra_sdhci_device2 = {
	.name		= "sdhci-tegra",
	.id		= 2,
	.resource	= sdhci_resource2,
	.num_resources	= ARRAY_SIZE(sdhci_resource2),
	.dev = {
		.platform_data = &tegra_sdhci_platform_data2,
	},
};

static struct platform_device tegra_sdhci_device3 = {
	.name		= "sdhci-tegra",
	.id		= 3,
	.resource	= sdhci_resource3,
	.num_resources	= ARRAY_SIZE(sdhci_resource3),
	.dev = {
		.platform_data = &tegra_sdhci_platform_data3,
	},
};

static int enterprise_wifi_status_register(
		void (*callback)(int card_present, void *dev_id),
		void *dev_id)
{
	if (wifi_status_cb)
		return -EAGAIN;
	wifi_status_cb = callback;
	wifi_status_cb_devid = dev_id;
	return 0;
}

/* HTC_WIFI_START */
int enterprise_wifi_status(struct device *dev)
{
	return enterprise_wifi_cd;
}

//static int enterprise_wifi_set_carddetect(int val)
int enterprise_wifi_set_carddetect(int val)
{
	printk("%s: %d\n", __func__, val);
	enterprise_wifi_cd = val;
	if (wifi_status_cb)
		wifi_status_cb(val, wifi_status_cb_devid);
	else
		pr_warning("%s: Nobody to notify\n", __func__);
	return 0;
}
EXPORT_SYMBOL(enterprise_wifi_set_carddetect);

//static int enterprise_wifi_power(int on)
int enterprise_wifi_power(int on)
{
	static int power_state;

	if (on == power_state)
		return 0;

	printk("%s: Powering %s wifi\n", __func__, (on ? "on" : "off"));

	power_state = on;
	if (on) {
		gpio_set_value(ENTERPRISE_WLAN_PWR, 1);
		mdelay(20);
	} else {
		gpio_set_value(ENTERPRISE_WLAN_PWR, 0);
	}

	return 0;
/*	
	pr_debug("%s: %d\n", __func__, on);
	gpio_set_value(ENTERPRISE_WLAN_PWR, on);
	mdelay(100);
	gpio_set_value(ENTERPRISE_WLAN_RST, on);
	mdelay(200);

	return 0;
*/
}
EXPORT_SYMBOL(enterprise_wifi_power);
/* HTC_WIFI_END */

static int enterprise_wifi_reset(int on)
{
	pr_debug("%s: do nothing\n", __func__);
	return 0;
}

static int __init enterprise_wifi_init(void)
{
	int rc;

	rc = gpio_request(ENTERPRISE_WLAN_PWR, "wlan_power");
	if (rc)
		pr_err("WLAN_PWR gpio request failed:%d\n", rc);
	rc = gpio_request(ENTERPRISE_WLAN_RST, "wlan_rst");
	if (rc)
		pr_err("WLAN_RST gpio request failed:%d\n", rc);
	rc = gpio_request(ENTERPRISE_WLAN_WOW, "bcmsdh_sdmmc");
	if (rc)
		pr_err("WLAN_WOW gpio request failed:%d\n", rc);

	tegra_gpio_enable(ENTERPRISE_WLAN_PWR);
	tegra_gpio_enable(ENTERPRISE_WLAN_RST);
	tegra_gpio_enable(ENTERPRISE_WLAN_WOW);

	rc = gpio_direction_output(ENTERPRISE_WLAN_PWR, 0);
	if (rc)
		pr_err("WLAN_PWR gpio direction configuration failed:%d\n", rc);
	gpio_direction_output(ENTERPRISE_WLAN_RST, 0);
	if (rc)
		pr_err("WLAN_RST gpio direction configuration failed:%d\n", rc);
	rc = gpio_direction_input(ENTERPRISE_WLAN_WOW);
	if (rc)
		pr_err("WLAN_WOW gpio direction configuration failed:%d\n", rc);

	/* HTC_WIFI_START */
	// platform_device_register(&enterprise_wifi_device);
	if (wl12xx_set_platform_data(&enterprise_wlan_data))
		pr_err("Error setting wl12xx_data\n");
	/* HTC_WIFI_END */
	return 0;
}

int __init enterprise_sdhci_init(void)
{
	platform_device_register(&tegra_sdhci_device3);
	platform_device_register(&tegra_sdhci_device2);
	enterprise_wifi_init();
	return 0;
}

void blue_pincfg_uartc_resume(void) {
	tegra_pinmux_set_pullupdown(TEGRA_PINGROUP_UART3_CTS_N, TEGRA_PUPD_NORMAL);
        tegra_gpio_disable(TEGRA_GPIO_PA1);
        tegra_gpio_disable(TEGRA_GPIO_PC0);
        tegra_gpio_disable(TEGRA_GPIO_PW6);
	tegra_pinmux_set_pullupdown(TEGRA_PINGROUP_UART3_RXD, TEGRA_PUPD_NORMAL);
        tegra_gpio_disable(TEGRA_GPIO_PW7);
}

EXPORT_SYMBOL(blue_pincfg_uartc_resume);

void blue_pincfg_uartc_suspend(void) {
        /* BT_EN GPIO-U.00 O(L) */

        gpio_direction_output(TEGRA_GPIO_PU0, 0);

        /* UART3_CTS_N GPIO-A.01 I(PU) */
        tegra_gpio_enable(TEGRA_GPIO_PA1);
        gpio_direction_input(TEGRA_GPIO_PA1);
        tegra_pinmux_set_pullupdown(TEGRA_PINGROUP_UART3_CTS_N, TEGRA_PUPD_PULL_UP);

        /* UART3_RTS_N GPIO-C.00 O(H) */
        tegra_gpio_enable(TEGRA_GPIO_PC0);
        gpio_direction_output(TEGRA_GPIO_PC0, 1);

        /* UART3_TXD GPIO-W.06 O(H) */
        tegra_gpio_enable(TEGRA_GPIO_PW6);
        gpio_direction_output(TEGRA_GPIO_PW6, 1);

        /* UART3_RXD GPIO-W.07 I(PU) */
        tegra_gpio_enable(TEGRA_GPIO_PW7);
        gpio_direction_input(TEGRA_GPIO_PW7);
        tegra_pinmux_set_pullupdown(TEGRA_PINGROUP_UART3_RXD, TEGRA_PUPD_PULL_UP);

	/* UART3 CTS_N WAKEUP GPIO-O.05 I(NP) */
	tegra_gpio_enable(TEGRA_GPIO_PO5);
	gpio_direction_input(TEGRA_GPIO_PO5);
	tegra_pinmux_set_pullupdown(TEGRA_PINGROUP_ULPI_DATA4, TEGRA_PUPD_NORMAL);
}

EXPORT_SYMBOL(blue_pincfg_uartc_suspend);

void blue_pincfg_uartc_gpio_request(void) {

        /* BT_EN GPIO-U.00 O(L) */
        long err = 0;

	/* UART3_CTS_N GPIO-A.01 */
        err = gpio_request(TEGRA_GPIO_PA1, "bt");
        if (err)
                pr_err("BT_CTS_N gpio request failed:%d\n", err);

	/* UART3_RTS_N GPIO-C.00 */
        err = gpio_request(TEGRA_GPIO_PC0, "bt");
        if (err)
                pr_err("BT_RTS_N gpio request failed:%d\n", err);

        /* UART3_TXD GPIO-W.06  */
        err = gpio_request(TEGRA_GPIO_PW6, "bt");
        if (err)
                pr_err("BT_TXD gpio request failed:%d\n", err);

        /* UART3_RXD GPIO-W.07  */
        err = gpio_request(TEGRA_GPIO_PW7, "bt");
        if (err)
                pr_err("BT_RXD gpio request failed:%d\n", err);

        /* BT_CTS#WAKE_UPGPIO-O.05_W  */
        err = gpio_request(TEGRA_GPIO_PO5, "bt");
        if (err)
                pr_err("BT_WAKEUP gpio request failed:%d\n", err);
        tegra_gpio_enable(TEGRA_GPIO_PO5);
        gpio_direction_input(TEGRA_GPIO_PO5);
	tegra_pinmux_set_pullupdown(TEGRA_PINGROUP_ULPI_DATA4, TEGRA_PUPD_NORMAL);


}

EXPORT_SYMBOL(blue_pincfg_uartc_gpio_request);
