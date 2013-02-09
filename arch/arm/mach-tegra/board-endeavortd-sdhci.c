/*
 * arch/arm/mach-tegra/board-endeavortd-sdhci.c
 *
 * Copyright (C) 2011-2012 NVIDIA Corporation.
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

/* HTC_WIFI_START */
#include <linux/wl12xx.h>
/* HTC_WIFI_END */

#include "gpio-names.h"
#include "board.h"


#define ENDEAVOR_WLAN_PWR	TEGRA_GPIO_PV2
#define ENDEAVOR_WLAN_RST	TEGRA_GPIO_PV3
#define ENDEAVOR_WLAN_WOW	TEGRA_GPIO_PO4
#define ENDEAVORTD_SD_CD TEGRA_GPIO_PI5

#define SDIO_CLK TEGRA_GPIO_PA6

static void (*wifi_status_cb)(int card_present, void *dev_id);
static void *wifi_status_cb_devid;
static int endeavor_wifi_status_register(void (*callback)(int , void *), void *);

static int endeavor_wifi_reset(int on);
/* HTC_WIFI_START */
//static int endeavor_wifi_power(int on);
//static int endeavor_wifi_set_carddetect(int val);
int endeavor_wifi_power(int on);
int endeavor_wifi_set_carddetect(int val);
int endeavor_wifi_status(struct device *dev);
static int endeavor_wifi_cd;		/* WIFI virtual 'card detect' status */
/* HTC_WIFI_END */

static struct wifi_platform_data endeavor_wifi_control = {
	.set_power      = endeavor_wifi_power,
	.set_reset      = endeavor_wifi_reset,
	.set_carddetect = endeavor_wifi_set_carddetect,
};

/* HTC_WIFI_START */
static struct wl12xx_platform_data endeavor_wlan_data __initdata = {
	.irq = TEGRA_GPIO_TO_IRQ(ENDEAVOR_WLAN_WOW),
	.board_ref_clock = WL12XX_REFCLOCK_26,
	.board_tcxo_clock = 1,
//	.platform_quirks = WL12XX_PLATFORM_QUIRK_EDGE_IRQ,
};
/* HTC_WIFI_END */

static struct resource wifi_resource[] = {
	[0] = {
		.name	= "bcm4329_wlan_irq",
		.start	= TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PU6),
		.end	= TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PU6),
		.flags	= IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL | IORESOURCE_IRQ_SHAREABLE,
	},
};

static struct platform_device endeavor_wifi_device = {
	.name           = "bcm4329_wlan",
	.id             = 1,
	.num_resources	= 1,
	.resource	= wifi_resource,
	.dev            = {
		.platform_data = &endeavor_wifi_control,
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

#ifdef CONFIG_MMC_EMBEDDED_SDIO
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
#endif

// No uSD
#if 0
static struct tegra_sdhci_platform_data tegra_sdhci_platform_data0 = {
	.mmc_data = {
		.register_status_notify	= endeavor_wifi_status_register,
#ifdef CONFIG_MMC_EMBEDDED_SDIO
		.embedded_sdio = &embedded_sdio_data0,
#endif
		/* FIXME need to revert the built_in change
		once we use get the signal strength fix of
		bcmdhd driver from broadcom for bcm4329 chipset*/
		.built_in = 0,
	},
#ifndef CONFIG_MMC_EMBEDDED_SDIO
	.pm_flags = MMC_PM_KEEP_POWER,
#endif
	.cd_gpio = -1,
	.wp_gpio = -1,
	.power_gpio = -1,
	.tap_delay = 0x0F,
	.max_clk_limit = 45000000,
	.ddr_clk_limit = 41000000,
};
#endif

static struct tegra_sdhci_platform_data tegra_sdhci_platform_data2 = {
	.mmc_data = {
		.status = endeavor_wifi_status,
		.register_status_notify	= endeavor_wifi_status_register,
		/* HTC_WIFI_START */
		//.embedded_sdio = &embedded_sdio_data0,
		/* HTC_WIFI_END */
		.built_in = 1,
	},
	.cd_gpio = -1,
	.wp_gpio = -1,
	.power_gpio = -1,
	.tap_delay = 0x0F,
	.ddr_clk_limit = 41000000,
};

static struct tegra_sdhci_platform_data tegra_sdhci_platform_data3 = {
	.cd_gpio = -1,
	.wp_gpio = -1,
	.power_gpio = -1,
	.is_8bit = 1,
	.tap_delay = 0x0F,
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

static int endeavor_wifi_status_register(
		void (*callback)(int card_present, void *dev_id),
		void *dev_id)
{
	if (wifi_status_cb)
		return -EAGAIN;
	wifi_status_cb = callback;
	wifi_status_cb_devid = dev_id;
	return 0;
}


static int endeavor_wifi_reset(int on)
{
	pr_debug("%s: do nothing\n", __func__);
	return 0;
}

#ifdef CONFIG_TEGRA_PREPOWER_WIFI
static int __init endeavortd_wifi_prepower(void)
{
	if (!machine_is_endeavortd())
		return 0;

	endeavor_wifi_power(1);

	return 0;
}

subsys_initcall_sync(endeavortd_wifi_prepower);
#endif

static int __init endeavor_wifi_init(void)
{
	int rc;

	rc = gpio_request(ENDEAVOR_WLAN_PWR, "wlan_power");
	if (rc)
		pr_err("WLAN_PWR gpio request failed:%d\n", rc);
	rc = gpio_request(ENDEAVOR_WLAN_RST, "wlan_rst");
	if (rc)
		pr_err("WLAN_RST gpio request failed:%d\n", rc);
	rc = gpio_request(ENDEAVOR_WLAN_WOW, "bcmsdh_sdmmc");
	if (rc)
		pr_err("WLAN_WOW gpio request failed:%d\n", rc);

	tegra_gpio_enable(ENDEAVOR_WLAN_PWR);
	tegra_gpio_enable(ENDEAVOR_WLAN_RST);
	tegra_gpio_enable(ENDEAVOR_WLAN_WOW);

	rc = gpio_direction_output(ENDEAVOR_WLAN_PWR, 0);
	if (rc)
		pr_err("WLAN_PWR gpio direction configuration failed:%d\n", rc);
	gpio_direction_output(ENDEAVOR_WLAN_RST, 0);
	if (rc)
		pr_err("WLAN_RST gpio direction configuration failed:%d\n", rc);
	rc = gpio_direction_input(ENDEAVOR_WLAN_WOW);
	if (rc)
		pr_err("WLAN_WOW gpio direction configuration failed:%d\n", rc);

	/* HTC_WIFI_START */
	// platform_device_register(&endeavor_wifi_device);
	if (wl12xx_set_platform_data(&endeavor_wlan_data))
		pr_err("Error setting wl12xx_data\n");
	/* HTC_WIFI_END */

    printk("[AUS] endeavor_wlan_data.irq=%d\n",endeavor_wlan_data.irq);
	return 0;
}

int __init endeavortd_sdhci_init(void)
{
	platform_device_register(&tegra_sdhci_device3);
	platform_device_register(&tegra_sdhci_device2);
	endeavor_wifi_init();
	return 0;
}




