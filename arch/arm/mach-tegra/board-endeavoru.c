/*
 * arch/arm/mach-tegra/board-endeavoru.c
 *
 * Copyright (c) 2011, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/serial_8250.h>
#include <linux/i2c.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/i2c-tegra.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/platform_data/tegra_usb.h>
#ifdef CONFIG_USB_G_ANDROID
#include <linux/usb/android_composite.h>
#endif
#include <linux/spi/spi.h>
#include <linux/tegra_uart.h>
#include <linux/fsl_devices.h>
#include <linux/i2c/atmel_mxt_ts.h>
#include <linux/memblock.h>
#include <linux/gpio_keys.h>
#include <linux/synaptics_i2c_rmi.h>
#include <linux/spi-tegra.h>

#include <mach/clk.h>
#include <mach/iomap.h>
#include <mach/irqs.h>
#include <mach/pinmux.h>
#include <mach/iomap.h>
#include <mach/io.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/usb_phy.h>
#include <mach/htc_usb.h>
#include <mach/i2s.h>
#include <mach/htc_headset_mgr.h>
#include <mach/htc_headset_gpio.h>
#include <mach/htc_headset_microp.h>
#include <mach/htc_headset_pmic.h>
#include <mach/htc_headset_misc.h>
#include <mach/tegra_flashlight.h>
#include <mach/board_htc.h>
#include <linux/proc_fs.h>
#include <mach/thermal.h>
#include <mach/mhl.h>
#include <mach/cable_detect.h>
#include <linux/tps80032_adc.h>
#include <linux/disp_debug.h>
#include <linux/keyreset.h>

#include "board.h"
#include "clock.h"
#include "board-endeavoru.h"
#include "baseband-xmm-power.h"
#include "devices.h"
#include "gpio-names.h"
#include "fuse.h"
#include "wakeups-t3.h"
#include "pm.h"
#include "htc-gpio.h"
#include <mach/htc_util.h>
#include <media/rawchip/rawchip.h>

#include "pokecpu.h"

#include "touch.h"
#ifdef CONFIG_TEGRA_HAPTIC2
#include <linux/pm8xxx-haptic2.h>
#endif
#ifdef CONFIG_TEGRA_VIBRATOR_ENR
#include <linux/tegra_vibrator_enr.h>
#endif
#include <linux/leds.h>
#include <linux/leds-lp5521_htc.h>


#include <linux/skbuff.h>
#include <linux/ti_wilink_st.h>
#include <linux/wakelock.h>

#define PMC_WAKE_STATUS         0x14
#define POWER_WAKEUP_ENR 7
extern global_wakeup_state;
extern int resume_from_deep_suspend;
/* TI 128x Bluetooth begin */
#include <linux/ti_wilink_st.h>
/* TI 128x Bluetooth end */

extern unsigned engineer_id;

#ifdef CONFIG_CPU_FREQ_GOV_ONDEMAND_2_PHASE
int set_two_phase_freq(int cpufreq);
#endif


/* All units are in millicelsius */
static struct tegra_thermal_data thermal_data = {
	.temp_throttle = 85000,
	.temp_shutdown = 90000,
	.temp_offset = TDIODE_OFFSET, /* temps based on tdiode */
#ifdef CONFIG_TEGRA_EDP_LIMITS
	.edp_offset = TDIODE_OFFSET,  /* edp based on tdiode */
	.hysteresis_edp = 3000,
#endif
#ifdef CONFIG_TEGRA_THERMAL_SYSFS
	.tc1 = 2,
	.tc2 = 1,
	.passive_delay = 500,
#else
	.hysteresis_throttle = 1000,
#endif
};


#if 1    // for A project bring up
#define GPIO_KEY(_id, _gpio, _iswake)		\
	{					\
		.code = _id,			\
		.gpio = TEGRA_GPIO_##_gpio,	\
		.active_low = 1,		\
		.desc = #_id,			\
		.type = EV_KEY,			\
		.wakeup = _iswake,		\
		.debounce_interval = 10,	\
	}

#define BOOT_DEBUG_LOG_ENTER(fn) \
	printk(KERN_NOTICE "[BOOT_LOG] Entering %s\n", fn);
#define BOOT_DEBUG_LOG_LEAVE(fn) \
	printk(KERN_NOTICE "[BOOT_LOG] Leaving %s\n", fn);

static int enrkey_wakeup()
{
	if (resume_from_deep_suspend) {
		unsigned long status =
			readl(IO_ADDRESS(TEGRA_PMC_BASE) + PMC_WAKE_STATUS);
		return status & TEGRA_WAKE_GPIO_PU6 ? KEY_POWER : KEY_RESERVED;
	} else
		return KEY_RESERVED;
}

static struct gpio_keys_button A_PROJECT_keys[] = {
	[0] = GPIO_KEY(KEY_POWER, PU6, 1),
	[1] = GPIO_KEY(KEY_VOLUMEUP, PS0, 0),
	[2] = GPIO_KEY(KEY_VOLUMEDOWN, PW3, 0),
 };

static struct gpio_keys_platform_data A_PROJECT_keys_platform_data = {
	.buttons	= A_PROJECT_keys,
	.nbuttons	= ARRAY_SIZE(A_PROJECT_keys),
	.wakeup_key     = enrkey_wakeup,
 };

static struct platform_device A_PROJECT_keys_device = {
	.name   = "gpio-keys",
	.id     = 0,
	.dev    = {
		.platform_data  = &A_PROJECT_keys_platform_data,
	},
};

int __init A_PROJECT_keys_init(void)
{
	int i;
/*
	struct board_info board_info;

	tegra_get_board_info(&board_info);
	if (!((board_info.board_id == BOARD_E1198) ||
		(board_info.board_id == BOARD_E1291) ||
		(board_info.board_id == BOARD_PM269)))
		return 0;
*/
	pr_info("Registering gpio keys\n");

	for (i = 0; i < ARRAY_SIZE(A_PROJECT_keys); i++)
		tegra_gpio_enable(A_PROJECT_keys[i].gpio);

	platform_device_register(&A_PROJECT_keys_device);

	return 0;
}

#endif

//MHL
#ifdef	CONFIG_TEGRA_HDMI_MHL

#define EDGE_GPIO_MHL_INT       TEGRA_GPIO_PC7
#define EDGE_GPIO_MHL_USB_SEL   TEGRA_GPIO_PE0
#define EDGE_GPIO_MHL_1V2       TEGRA_GPIO_PE4
#define EDGE_GPIO_MHL_RESET     TEGRA_GPIO_PE6
#define EDGE_GPIO_MHL_DDC_CLK   TEGRA_GPIO_PV4
#define EDGE_GPIO_MHL_DDC_DATA  TEGRA_GPIO_PV5
#define EDGE_GPIO_MHL_3V3       TEGRA_GPIO_PY2

static int mhl_sii_power(int on)
{
	pr_info("[DISP]%s(%d) IN\n", __func__, __LINE__);

	int rc = 0;
	int err = 0;

	switch (on) {
		case 0:
			//mhl_sii9234_1v2_power(false);

			/*Turn off MHL_3V3*/
			gpio_set_value(EDGE_GPIO_MHL_3V3, 0);
			mdelay(10);

			/*Turn off MHL_1V2*/
			gpio_set_value(EDGE_GPIO_MHL_1V2, 0);
			mdelay(5);

			break;
		case 1:
			//mhl_sii9234_all_power(true);

			/*Turn on MHL_3V3*/
			gpio_set_value(EDGE_GPIO_MHL_3V3, 1);
			mdelay(10);

			/*Turn on MHL_1V2*/
			gpio_set_value(EDGE_GPIO_MHL_1V2, 1);
			mdelay(5);

			break;

		default:
			pr_info("[DISP]%s(%d) got unsupport parameter %d!\n", __func__, __LINE__, on);
			break;
	}

	pr_info("[DISP]%s(%d) OUT\n", __func__, __LINE__);

	return rc;
}

static T_MHL_PLATFORM_DATA mhl_sii_device_data = {
	.gpio_intr        = EDGE_GPIO_MHL_INT,
	.gpio_usb_sel     = EDGE_GPIO_MHL_USB_SEL,
	.gpio_reset       = EDGE_GPIO_MHL_RESET,
	.gpio_ddc_clk     = EDGE_GPIO_MHL_DDC_CLK,
	.gpio_ddc_data    = EDGE_GPIO_MHL_DDC_DATA,
	.ci2ca            = 0,
	//.mhl_usb_switch = pyramid_usb_dpdn_switch,
	//.mhl_1v2_power  = mhl_sii9234_1v2_power,
	.power            = mhl_sii_power,
	.enMhlD3Guard     = false,
#ifdef CONFIG_TEGRA_HDMI_MHL_SUPERDEMO
	.abs_x_min = 941,/* 0 */
	.abs_x_max = 31664,/* 32767 */
	.abs_y_min = 417,/* 0 */
	.abs_y_max = 32053,/* 32767 */
	.abs_pressure_min = 0,
	.abs_pressure_max = 255,
	.abs_width_min = 0,
	.abs_width_max = 20,
#endif
};

static struct i2c_board_info i2c_mhl_sii_info[] =
{
	{
		I2C_BOARD_INFO(MHL_SII9234_I2C_NAME, 0x72 >> 1),
		.platform_data = &mhl_sii_device_data,
		.irq = TEGRA_GPIO_TO_IRQ(EDGE_GPIO_MHL_INT)
	}
};
#endif

//haptic vibrator
#ifdef CONFIG_TEGRA_HAPTIC2
static struct pm8xxx_haptic_data haptic_data[] ={
		{
		.pwm_data={
					.name = "haptic",
					.bank = 0,
				  },
		}, 
};
static struct pm8xxx_haptic_data_group haptic_data_group={
		.p_haptic_data=haptic_data,
		.size=ARRAY_SIZE(haptic_data),
};
static struct platform_device pm8xxx_haptic = {
	.name= PM8XXX_HAPTIC_NAME,
	.id=-1,
	.dev = {
		.platform_data=&haptic_data_group,
	},
};
static struct platform_device *haptic_vibrator[] __initdata = {
	&pm8xxx_haptic,
};
static void haptic_vibrator_init(int board_id)
{
	if (board_id == PROJECT_PHASE_XA) {
		haptic_data->ena_gpio = TEGRA_GPIO_PR3;
		haptic_data->pwm_sfio = TEGRA_GPIO_PH0;
	}
	else if (board_id >= PROJECT_PHASE_XB) {
		haptic_data->ena_gpio = TEGRA_GPIO_PF1;
		haptic_data->pwm_sfio = TEGRA_GPIO_PH0;
	}
	platform_add_devices(haptic_vibrator, ARRAY_SIZE(haptic_vibrator));
}
#endif

//vibrator
#ifdef CONFIG_TEGRA_VIBRATOR_ENR
static struct vibrator_platform_data vibrator_data = {
	.pwm_data={
		.name = "vibrator",
		.bank = 0,
	},
	.pwm_gpio = TEGRA_GPIO_PH0,
	.ena_gpio = TEGRA_GPIO_PF1,
	.pwr_gpio = TEGRA_GPIO_PE7,
};
static struct platform_device tegra_vibrator = {
	.name= VIBRATOR_NAME,
	.id=-1,
	.dev = {
		.platform_data=&vibrator_data,
	},
};
static void tegra_vibrator_init(void)
{
	platform_device_register(&tegra_vibrator);
}
#endif

//leds-lp5521
static struct led_i2c_config lp5521_led_config[] = {
	{
		.name = "amber",
	},
	{
		.name = "green",
	},
	{
		.name = "button-backlight",
	},
};
static struct led_i2c_platform_data led_data = {
	.num_leds	= ARRAY_SIZE(lp5521_led_config),
	.led_config	= lp5521_led_config,
	.ena_gpio = TEGRA_GPIO_PY0,
	.tri_gpio = TEGRA_GPIO_PY1,
};
static struct i2c_board_info i2c_led_devices[] = {
	{
		I2C_BOARD_INFO(LED_I2C_NAME, 0x32),
		.platform_data = &led_data,
		.irq = -1,
	},
};
static void leds_lp5521_init(void)
{
	i2c_register_board_info(1, i2c_led_devices,
		ARRAY_SIZE(i2c_led_devices));
}

//TODO: USB VERIFY
#if 0
static struct usb_mass_storage_platform_data tegra_usb_fsg_platform = {
	.vendor = "HTC",
	.product = "Android USB Device",
#ifdef CONFIG_USB_CDROM
	.nluns = 2,
	.cdrom_lun = 2,
#else
	.nluns = 1,
#endif
};

static struct platform_device tegra_usb_fsg_device = {
	.name = "usb_mass_storage",
	.id = -1,
	.dev = {
		.platform_data = &tegra_usb_fsg_platform,
	},
};
#endif

//flashlight
static void config_enterprise_flashlight_gpios(void)
{
	int ret;
	printk("%s: start...", __func__);
	// FL_TORCH_EN
	ret = gpio_request(FL_TORCH_EN, "fl_torch_en");
	if (ret < 0)
		pr_err("[FLT] %s: gpio_request failed for gpio %s\n",
			__func__, "FL_TORCH_EN");
	ret = gpio_direction_output(FL_TORCH_EN, 0);
	if (ret < 0) {
		pr_err("[FLT] %s: gpio_direction_output failed %d\n", __func__, ret);
		gpio_free(FL_TORCH_EN);
		return;
	}
	tegra_gpio_enable(FL_TORCH_EN);
	gpio_export(FL_TORCH_EN, false);

	// FL_FLASH_EN
	ret = gpio_request(FL_FLASH_EN, "fl_flash_en");
	if (ret < 0)
		pr_err("[FLT] %s: gpio_request failed for gpio %s\n",
			__func__, "FL_FLASH_EN");
	ret = gpio_direction_output(FL_FLASH_EN, 0);
	if (ret < 0) {
		pr_err("[FLT] %s: gpio_direction_output failed %d\n", __func__, ret);
		gpio_free(FL_FLASH_EN);
		return;
	}
	tegra_gpio_enable(FL_FLASH_EN);
	gpio_export(FL_FLASH_EN, false);
	printk("%s: end...", __func__);
}

static struct flashlight_platform_data enterprise_flashlight_data = {
	.gpio_init  = config_enterprise_flashlight_gpios,
	.torch = FL_TORCH_EN,
	.flash = FL_FLASH_EN,
	.flash_duration_ms = 600
};

static struct platform_device enterprise_flashlight_device = {
	.name = FLASHLIGHT_NAME,
	.dev		= {
		.platform_data	= &enterprise_flashlight_data,
	},
};

static void enterprise_flashlight_init(void)
{
	platform_device_register(&enterprise_flashlight_device);
}

/* !!!TODO: Change for enterprise (Taken from Cardhu) */
static struct tegra_utmip_config utmi_phy_config[] = {
	[0] = {
			.hssync_start_delay = 0,
			.idle_wait_delay = 17,
			.elastic_limit = 16,
			.term_range_adj = 6,
			.xcvr_setup = 15,
			.xcvr_setup_offset = 2,
			.xcvr_use_fuses = 1,
			.xcvr_lsfslew = 2,
			.xcvr_lsrslew = 2,
	},
	[1] = {
			.hssync_start_delay = 0,
			.idle_wait_delay = 17,
			.elastic_limit = 16,
			.term_range_adj = 6,
			.xcvr_setup = 15,
			.xcvr_setup_offset = 0,
			.xcvr_use_fuses = 1,
			.xcvr_lsfslew = 2,
			.xcvr_lsrslew = 2,
	},
	[2] = {
			.hssync_start_delay = 0,
			.idle_wait_delay = 17,
			.elastic_limit = 16,
			.term_range_adj = 6,
			.xcvr_setup = 8,
			.xcvr_setup_offset = 0,
			.xcvr_use_fuses = 1,
			.xcvr_lsfslew = 2,
			.xcvr_lsrslew = 2,
	},
};

static struct resource enterprise_bcm4329_rfkill_resources[] = {
	{
		.name   = "bcm4329_nshutdown_gpio",
		.start  = TEGRA_GPIO_PE6,
		.end    = TEGRA_GPIO_PE6,
		.flags  = IORESOURCE_IO,
	},
};

static struct platform_device enterprise_bcm4329_rfkill_device = {
	.name = "bcm4329_rfkill",
	.id		= -1,
	.num_resources  = ARRAY_SIZE(enterprise_bcm4329_rfkill_resources),
	.resource       = enterprise_bcm4329_rfkill_resources,
};

static unsigned long retry_suspend;

/* TI 128x Bluetooth begin */
int plat_kim_suspend(struct platform_device *pdev, pm_message_t state)
{
        /* TODO: wait for HCI-LL sleep */
	pr_info("plat_kim_suspend\n");
        return 0;
}
int plat_kim_resume(struct platform_device *pdev)
{
	retry_suspend = 0;
        return 0;
}

static struct wake_lock st_wk_lock;

/* Release the wakelock when chip is asleep */
static int plat_chip_asleep(void)
{
	pr_info("plat_chip_asleep\n");
	wake_unlock(&st_wk_lock);
	return 1;
}

/* Aquire the wakelock when chip is awake */
static int plat_chip_awake(void)
{

	pr_info("plat_chip_awake\n");
	wake_lock(&st_wk_lock);
	return 1;
}

struct ti_st_plat_data wilink_pdata = {
                .nshutdown_gpio = 160,
                .dev_name = "/dev/ttyHS2",
                .flow_cntrl = 1,
                .baud_rate = 3000000,
                /*.baud_rate = 115200,*/
                .suspend = plat_kim_suspend,
                .resume = plat_kim_resume,
		.chip_asleep = plat_chip_asleep,
		.chip_awake  = plat_chip_awake,
		.chip_enable = plat_chip_awake,
		.chip_disable = plat_chip_asleep,
};

static struct platform_device btwilink_device = {
        .name = "btwilink",
        .id = -1,
};

static struct platform_device wl128x_device = {
        .name           = "kim",
        .id             = -1,
        .dev.platform_data = &wilink_pdata,
};

static noinline void __init enterprise_bt_wl128x(void)
{
	wake_lock_init(&st_wk_lock, WAKE_LOCK_SUSPEND, "st_wake_lock");

        platform_device_register(&wl128x_device);
	platform_device_register(&btwilink_device);
        tegra_gpio_enable(TEGRA_GPIO_PU0);

        return;
}
/* TI 128x Bluetooth end */

static __initdata struct tegra_clk_init_table enterprise_clk_init_table[] = {
	/* name		parent		rate		enabled */
	{ "pll_m",	NULL,		0,		false},
	{ "hda",	"pll_p",	108000000,	false},
	{ "hda2codec_2x","pll_p",	48000000,	false},
	{ "pwm",	"pll_p",	108000000,		false},
	{ "blink",	"clk_32k",	32768,		true},
	{ "pll_a",	NULL,		564480000,	false},
	{ "pll_a_out0",	NULL,		11289600,	false},
	{ "i2s1",	"pll_a_out0",	0,		false},
	{ "audio1",	"i2s1_sync",	0,		false},
	{ "spdif_out",	"pll_a_out0",	0,		false},
	{ "d_audio",	"pll_a_out0",	0,		false},
        { "vi_sensor",  "pll_p",        68000000,   false},
	{ "dam0",	"pll_a_out0",	0,		false},
	{ "dam1",	"pll_a_out0",	0,		false},
	{ "dam2",	"pll_a_out0",	0,		false},
	{ "i2c1",	"pll_p",	3200000,	false},
	{ "i2c2",	"pll_p",	3200000,	false},
	{ "i2c3",	"pll_p",	3200000,	false},
	{ "i2c4",	"pll_p",	3200000,	false},
	{ "i2c5",	"pll_p",	3200000,	false},
	{ NULL,		NULL,		0,		0},
};

#define USB_MANUFACTURER_NAME	"HTC"
#define USB_PRODUCT_NAME		"Android USB Device"
#define BLUE_PID				0x0CD9
#define USB_VENDOR_ID			0x0BB4

#if 0	//k30
/* standard android USB platform data */
static struct android_usb_platform_data andusb_plat = {
	.vendor_id              = USB_VENDOR_ID,
	.product_id             = BLUE_PID,
	.manufacturer_name      = USB_MANUFACTURER_NAME,
	.product_name           = USB_PRODUCT_NAME,
	.serial_number          = NULL,
	.num_products = ARRAY_SIZE(usb_products),
	.products = usb_products,
	.num_functions = ARRAY_SIZE(usb_functions_all),
	.functions = usb_functions_all,
};

static struct platform_device androidusb_device = {
	.name   = "android_usb",
	.id     = -1,
	.dev    = {
		.platform_data  = &andusb_plat,
	},
};

static struct func_switch_platform_data func_switch_pdata = {
	.vendor = "HTC",
	.product = "Android USB Device",
//	.num_functions = ARRAY_SIZE(usb_functions_all),
	.usb_id_pin_gpio = TEGRA_GPIO_PS2,
};
static struct platform_device func_switch_device = {
	.name = "msm_hsusb",
	.id = -1,
	.dev = {
		.platform_data = &func_switch_pdata,
	},
};
#else

static struct android_usb_platform_data android_usb_pdata = {
	.vendor_id	= 0x0BB4,
	.product_id	= 0x0cd6,
	.version	= 0x0100,
	.product_name		= "Android Phone",
	.manufacturer_name	= "HTC",
	.num_products = ARRAY_SIZE(usb_products),
	.products = usb_products,
	.num_functions = ARRAY_SIZE(usb_functions_all),
	.functions = usb_functions_all,
//k30-htc	.enable_fast_charge = NULL,
//k30-htc	.update_pid_and_serial_num = usb_diag_update_pid_and_serial_num,
	.fserial_init_string = "tty:modem,tty:autobot,tty:serial,tty:autobot",
	.usb_id_pin_gpio = TEGRA_GPIO_PS2,
	.RndisDisableMPDecision = true,
	.nluns = 1,
};

static struct platform_device android_usb_device = {
	.name	= "android_usb",
	.id		= -1,
	.dev		= {
		.platform_data = &android_usb_pdata,
	},
};
#endif	//end of #if 0 //k30

#ifdef CONFIG_USB_ANDROID_RNDIS
static struct usb_ether_platform_data rndis_pdata = {
	.ethaddr = {0, 0, 0, 0, 0, 0},
	.vendorID = USB_VENDOR_ID,
	.vendorDescr = USB_MANUFACTURER_NAME,
};

static struct platform_device rndis_device = {
	.name   = "rndis",
	.id     = -1,
	.dev    = {
		.platform_data  = &rndis_pdata,
	},
};
#endif

static struct tegra_i2c_platform_data enterprise_i2c1_platform_data = {
	.adapter_nr	= 0,
	.bus_count	= 1,
	.bus_clk_rate	= { 384000, 0 },
	.scl_gpio		= {TEGRA_GPIO_PC4, 0},
	.sda_gpio		= {TEGRA_GPIO_PC5, 0},
	.arb_recovery = arb_lost_recovery,
};

static struct tegra_i2c_platform_data enterprise_i2c2_platform_data = {
	.adapter_nr	= 1,
	.bus_count	= 1,
	.bus_clk_rate	= { 384000, 0 },
	.is_clkon_always = true,
	.scl_gpio		= {TEGRA_GPIO_PT5, 0},
	.sda_gpio		= {TEGRA_GPIO_PT6, 0},
	.arb_recovery = arb_lost_recovery,
};

static struct tegra_i2c_platform_data enterprise_i2c3_platform_data = {
	.adapter_nr	= 2,
	.bus_count	= 1,
	.bus_clk_rate	= { 384000, 0 },
	.scl_gpio		= {TEGRA_GPIO_PBB1, 0},
	.sda_gpio		= {TEGRA_GPIO_PBB2, 0},
	.arb_recovery = arb_lost_recovery,
};

static struct tegra_i2c_platform_data enterprise_i2c4_platform_data = {
	.adapter_nr	= 3,
	.bus_count	= 1,
	.bus_clk_rate	= { 100000, 0 },
	.scl_gpio		= {TEGRA_GPIO_PV4, 0},
	.sda_gpio		= {TEGRA_GPIO_PV5, 0},
	.arb_recovery = arb_lost_recovery,
};

static struct tegra_i2c_platform_data enterprise_i2c5_platform_data = {
	.adapter_nr	= 4,
	.bus_count	= 1,
	.bus_clk_rate	= { 100000, 0 },
	.scl_gpio		= {TEGRA_GPIO_PZ6, 0},
	.sda_gpio		= {TEGRA_GPIO_PZ7, 0},
	.arb_recovery = arb_lost_recovery,
};

// +++ Audio tlv320aic3008 +++
static struct tegra_spi_device_controller_data dev_cdata_audio = {
	.is_hw_based_cs = false, /* bool is_hw_based_cs */
	.cs_setup_clk_count = 4, /* int cs_setup_clk_count */
	.cs_hold_clk_count = 2, /* int cs_hold_clk_count */
};

static struct spi_board_info spi_board_info_audio[] __initdata = {
	{
		.modalias	= "aic3008",
		.mode           = SPI_MODE_1,
		.bus_num        = 1,
		.chip_select    = 0,
		.max_speed_hz   = 10800000,
		.controller_data = &dev_cdata_audio,
	},
};

static struct platform_device enterprise_audio_device = {
	.name	= "tegra-snd-aic3008",
	.id	= 0,
	.dev	= {
		.platform_data  = NULL,
	},
};
// --- Audio tlv320aic3008 ---

/*HTC rawchip SPI4 ++ */
static struct tegra_spi_device_controller_data dev_cdata_rawchip = {
       .is_hw_based_cs = true, /* bool is_hw_based_cs */
       .cs_setup_clk_count = 1, /* int cs_setup_clk_count */
       .cs_hold_clk_count = 2, /* int cs_hold_clk_count */
};
static struct spi_board_info enterprise_spi_board_info_rawchip[] __initdata = {
       {
               .modalias       = "spi_rawchip",
               .mode           = SPI_MODE_0,
               .bus_num        = 3,
               .chip_select    = 1,
               .max_speed_hz   = 25000000,
               .controller_data = &dev_cdata_rawchip,
       },
};
+// Riemer 07-05-2012: How can we have a symbol exported which is declared as __initdata? 
//EXPORT_SYMBOL_GPL(enterprise_spi_board_info_rawchip);

static struct tegra_camera_rawchip_info tegra_rawchip_board_info = {
	.rawchip_intr0  = TEGRA_GPIO_PR0,
	.rawchip_intr1  = TEGRA_GPIO_PEE1,
};

static struct platform_device tegra_rawchip_device = {
	.name = "rawchip",
	.id	= -1,
	.dev	= {
		.platform_data = &tegra_rawchip_board_info,
	},
};

/*HTC rawchip SPI4 -- */

/* HTC_HEADSET_GPIO Driver */
static struct htc_headset_gpio_platform_data htc_headset_gpio_data = {
	.eng_cfg		= HS_EDE_U,
	.hpin_gpio		= TEGRA_GPIO_PW2,
	.key_gpio		= TEGRA_GPIO_PBB6,
	.key_enable_gpio	= 0,
	.mic_select_gpio	= 0,
};

static struct platform_device htc_headset_gpio = {
	.name	= "HTC_HEADSET_GPIO",
	.id	= -1,
	.dev	= {
		.platform_data	= &htc_headset_gpio_data,
	},
};

/* HTC_HEADSET_MICROP Driver */
static struct htc_headset_microp_platform_data htc_headset_microp_data = {
	.eng_cfg			= HS_EDE_U,
	.remote_int		= 1 << 13,
	.remote_irq		= TEGRA_uP_TO_INT(13),
	.remote_enable_pin	= 0,
	.adc_channel		= 0x01,
	.adc_remote		= {0, 33, 38, 85, 95, 180},
};

static struct platform_device htc_headset_microp = {
	.name	= "HTC_HEADSET_MICROP",
	.id	= -1,
	.dev	= {
		.platform_data	= &htc_headset_microp_data,
	},
};

/* HTC_HEADSET_PMIC Driver */
static struct htc_headset_pmic_platform_data htc_headset_pmic_data = {
	.eng_cfg		= HS_EDE_U,
	.driver_flag	= DRIVER_HS_PMIC_RPC_KEY,
	.adc_mic_bias	= {HS_DEF_MIC_ADC_12_BIT_MIN,
				   HS_DEF_MIC_ADC_12_BIT_MAX},
	.adc_remote	= {0, 125, 126, 330, 331, 710},
};

static struct htc_headset_pmic_platform_data htc_headset_pmic_data_xe = {
	.eng_cfg		= HS_EDE_U,
	.driver_flag	= DRIVER_HS_PMIC_RPC_KEY,
	.adc_mic_bias	= {HS_DEF_MIC_ADC_12_BIT_MIN,
				   HS_DEF_MIC_ADC_12_BIT_MAX},
	.adc_remote	= {0, 164, 165, 379, 380, 830},
};


static struct platform_device htc_headset_pmic = {
	.name	= "HTC_HEADSET_PMIC",
	.id	= -1,
	.dev	= {
		.platform_data	= &htc_headset_pmic_data,
	},
};

static struct platform_device htc_headset_pmic_xe = {
	.name	= "HTC_HEADSET_PMIC",
	.id	= -1,
	.dev	= {
		.platform_data	= &htc_headset_pmic_data_xe,
	},
};


static struct headset_adc_config htc_headset_mgr_config[] = {
	{
		.type = HEADSET_UNPLUG,
		.adc_max = 4095,
		.adc_min = 3804,
	},
	{
		.type = HEADSET_MIC,
		.adc_max = 3803,
		.adc_min = 2500,
	},
	{
		.type = HEADSET_BEATS,
		.adc_max = 2499,
		.adc_min = 1800,
	},
	{
		.type = HEADSET_BEATS_SOLO,
		.adc_max = 1799,
		.adc_min = 1175,
	},
	{
		.type = HEADSET_NO_MIC,
		.adc_max = 1174,
		.adc_min = 0,
	},
};

static struct headset_adc_config htc_headset_mgr_config_xe[] = {
	{
		.type = HEADSET_UNPLUG,
		.adc_max = 4095,
		.adc_min = 3601,
	},
	{
		.type = HEADSET_MIC,
		.adc_max = 3600,
		.adc_min = 2951,
	},
	{
		.type = HEADSET_BEATS,
		.adc_max = 2950,
		.adc_min = 2101,
	},
	{
		.type = HEADSET_BEATS_SOLO,
		.adc_max = 2100,
		.adc_min = 1500,
	},
	{
		.type = HEADSET_NO_MIC,
		.adc_max = 1499,
		.adc_min = 0,
	},
};



/* HTC_HEADSET_MISC Driver */
static struct htc_headset_misc_platform_data htc_headset_misc_data = {
/*
	.driver_flag		= DRIVER_HS_MISC_EXT_HP_DET,
	.ext_hpin_gpio		= PM8058_GPIO_PM_TO_SYS(FLYER_H2W_CABLE_IN1),
	.ext_accessory_type	= USB_AUDIO_OUT,
*/
};

static struct platform_device htc_headset_misc = {
	.name	= "HTC_HEADSET_MISC",
	.id	= -1,
	.dev	= {
		.platform_data	= &htc_headset_misc_data,
	},
};

/* HTC_HEADSET_MGR Driver */
static struct platform_device *headset_devices[] = {
//	&htc_headset_microp,
	&htc_headset_pmic,
	&htc_headset_gpio,
//	&htc_headset_misc,
	/* Please put the headset detection driver on the last */
};

static struct platform_device *headset_devices_xe[] = {
//	&htc_headset_microp,
	&htc_headset_pmic_xe,
	&htc_headset_gpio,
//	&htc_headset_misc,
	/* Please put the headset detection driver on the last */
};


static struct htc_headset_mgr_platform_data htc_headset_mgr_data = {
	.eng_cfg				= HS_EDE_U,
	.headset_devices_num	= ARRAY_SIZE(headset_devices),
	.headset_devices	= headset_devices,
	.headset_config_num	= ARRAY_SIZE(htc_headset_mgr_config),
	.headset_config		= htc_headset_mgr_config,
};

static struct htc_headset_mgr_platform_data htc_headset_mgr_data_xe = {
	.eng_cfg				= HS_EDE_U,
	.headset_devices_num	= ARRAY_SIZE(headset_devices_xe),
	.headset_devices	= headset_devices_xe,
	.headset_config_num	= ARRAY_SIZE(htc_headset_mgr_config_xe),
	.headset_config		= htc_headset_mgr_config_xe,
};


static struct platform_device htc_headset_mgr = {
	.name	= "HTC_HEADSET_MGR",
	.id	= -1,
	.dev	= {
		.platform_data	= &htc_headset_mgr_data,
	},
};

static struct platform_device htc_headset_mgr_xe = {
	.name	= "HTC_HEADSET_MGR",
	.id	= -1,
	.dev	= {
		.platform_data	= &htc_headset_mgr_data_xe,
	},
};


static void enterprise_i2c_init(void)
{
	tegra_i2c_device1.dev.platform_data = &enterprise_i2c1_platform_data;
	tegra_i2c_device2.dev.platform_data = &enterprise_i2c2_platform_data;
	tegra_i2c_device3.dev.platform_data = &enterprise_i2c3_platform_data;
	tegra_i2c_device4.dev.platform_data = &enterprise_i2c4_platform_data;
	tegra_i2c_device5.dev.platform_data = &enterprise_i2c5_platform_data;

	platform_device_register(&tegra_i2c_device5);
	platform_device_register(&tegra_i2c_device4);
	platform_device_register(&tegra_i2c_device3);
	platform_device_register(&tegra_i2c_device2);
	platform_device_register(&tegra_i2c_device1);
}

static struct platform_device *enterprise_uart_devices[] __initdata = {
	&tegra_uarta_device,
	&tegra_uartb_device,
	&tegra_uartc_device,
	&tegra_uartd_device,
	&tegra_uarte_device,
};

struct uart_clk_parent uart_parent_clk[] = {
	[0] = {.name = "pll_p"},
	[1] = {.name = "pll_m"},
#ifndef CONFIG_TEGRA_PLLM_RESTRICTED
	[2] = {.name = "clk_m"},
#endif
};
static struct tegra_uart_platform_data enterprise_uart_pdata;

#ifdef CONFIG_BT_CTS_WAKEUP
static struct tegra_uart_platform_data enterprise_bt_uart_pdata;
#endif

static void __init uart_debug_init(void)
{
	unsigned long rate;
	struct clk *c;

	/* UARTA is the debug port. */
	pr_info("Selecting UARTA as the debug console\n");
	enterprise_uart_devices[0] = &debug_uarta_device;
	debug_uart_port_base = ((struct plat_serial8250_port *)(
			debug_uarta_device.dev.platform_data))->mapbase;
	debug_uart_clk = clk_get_sys("serial8250.0", "uarta");

	/* Clock enable for the debug channel */
	if (!IS_ERR_OR_NULL(debug_uart_clk)) {
		rate = ((struct plat_serial8250_port *)(
			debug_uarta_device.dev.platform_data))->uartclk;
		pr_info("The debug console clock name is %s\n",
						debug_uart_clk->name);
		c = tegra_get_clock_by_name("pll_p");
		if (IS_ERR_OR_NULL(c))
			pr_err("Not getting the parent clock pll_p\n");
		else
			clk_set_parent(debug_uart_clk, c);

		clk_enable(debug_uart_clk);
		clk_set_rate(debug_uart_clk, rate);
	} else {
		pr_err("Not getting the clock %s for debug console\n",
				debug_uart_clk->name);
	}
}

static void headset_uart_init(void)
{
int ret = 0;

ret = gpio_request(TEGRA_GPIO_PY4,"headset_uart_TX");
if (ret < 0) {
	printk(KERN_WARNING "[HS_UART_GPIO] %s: gpio_request failed %d\n", __func__, ret);
	gpio_free(TEGRA_GPIO_PY4);
	return;
}
ret = gpio_direction_output(TEGRA_GPIO_PY4,0);
if (ret < 0) {
	printk(KERN_WARNING "[HS_UART_GPIO] %s: gpio_direction_output failed %d\n", __func__, ret);
	gpio_free(TEGRA_GPIO_PY4);
	return;
}
tegra_gpio_enable(TEGRA_GPIO_PY4);


ret = gpio_request(TEGRA_GPIO_PY5,"headset_uart_RX");
if (ret < 0) {
	printk(KERN_WARNING "[HS_UART_GPIO] %s: gpio_request failed %d\n", __func__, ret);
	gpio_free(TEGRA_GPIO_PY5);
	return;
}
ret = gpio_direction_input(TEGRA_GPIO_PY5);
if (ret < 0) {
	printk(KERN_WARNING "[HS_UART_GPIO] %s: gpio_direction_input failed %d\n", __func__, ret);
	gpio_free(TEGRA_GPIO_PY5);
	return;
}
tegra_gpio_enable(TEGRA_GPIO_PY5);

ret = gpio_request(TEGRA_GPIO_PZ0,"headset_uart_switch");
if (ret < 0) {
	printk(KERN_WARNING "[HS_UART_GPIO] %s: gpio_request failed %d\n", __func__, ret);
	gpio_free(TEGRA_GPIO_PZ0);
	return;
}
ret = gpio_direction_output(TEGRA_GPIO_PZ0,1);
if (ret < 0) {
	printk(KERN_WARNING "[HS_UART_GPIO] %s: gpio_direction_input failed %d\n", __func__, ret);
	gpio_free(TEGRA_GPIO_PZ0);
	return;
}
tegra_gpio_enable(TEGRA_GPIO_PZ0);

}


static void __init enterprise_uart_init(void)
{
	int i;
	struct clk *c;

	for (i = 0; i < ARRAY_SIZE(uart_parent_clk); ++i) {
		c = tegra_get_clock_by_name(uart_parent_clk[i].name);
		if (IS_ERR_OR_NULL(c)) {
			pr_err("Not able to get the clock for %s\n",
						uart_parent_clk[i].name);
			continue;
		}
		uart_parent_clk[i].parent_clk = c;
		uart_parent_clk[i].fixed_clk_rate = clk_get_rate(c);
	}
	enterprise_uart_pdata.parent_clk_list = uart_parent_clk;
	enterprise_uart_pdata.parent_clk_count = ARRAY_SIZE(uart_parent_clk);
	tegra_uarta_device.dev.platform_data = &enterprise_uart_pdata;
	tegra_uartb_device.dev.platform_data = &enterprise_uart_pdata;
	tegra_uartc_device.dev.platform_data = &enterprise_uart_pdata;
	tegra_uartd_device.dev.platform_data = &enterprise_uart_pdata;
	tegra_uarte_device.dev.platform_data = &enterprise_uart_pdata;

#ifdef CONFIG_BT_CTS_WAKEUP
	int board_id = htc_get_pcbid_info();

	enterprise_bt_uart_pdata = enterprise_uart_pdata;
	if (board_id >= PROJECT_PHASE_XC) {// XC
		enterprise_bt_uart_pdata.uart_bt = (1 == 1); /* true */
	}
	else {
		enterprise_bt_uart_pdata.uart_bt = (1 == 0); /* false */
	}
	enterprise_bt_uart_pdata.bt_en = BT_GPIO_EN;
	enterprise_bt_uart_pdata.bt_cts_irq = BT_GPIO_CTS_IRQ;
	tegra_uartc_device.dev.platform_data = &enterprise_bt_uart_pdata;
#endif

	/* Register low speed only if it is selected */
	if (!is_tegra_debug_uartport_hs())
		uart_debug_init();

	platform_add_devices(enterprise_uart_devices,
				ARRAY_SIZE(enterprise_uart_devices));
}

struct spi_clk_parent spi_parent_clk[] = {
	[0] = {.name = "pll_p"},
#ifndef CONFIG_TEGRA_PLLM_RESTRICTED
	[1] = {.name = "pll_m"},
	[2] = {.name = "clk_m"},
#else
	[1] = {.name = "clk_m"},
#endif
};

static struct tegra_spi_platform_data cardhu_spi_pdata = {
	.is_dma_based		= true,
	.max_dma_buffer		= (16 * 1024),
	.is_clkon_always	= false,
	.max_rate		= 100000000,
};

static void __init enterprise_spi_init(void)
{
	int i;
	struct clk *c;

	for (i = 0; i < ARRAY_SIZE(spi_parent_clk); ++i) {
		c = tegra_get_clock_by_name(spi_parent_clk[i].name);
		if (IS_ERR_OR_NULL(c)) {
			pr_err("Not able to get the clock for %s\n",
						spi_parent_clk[i].name);
			continue;
		}
		spi_parent_clk[i].parent_clk = c;
		spi_parent_clk[i].fixed_clk_rate = clk_get_rate(c);
	}
	cardhu_spi_pdata.parent_clk_list = spi_parent_clk;
	cardhu_spi_pdata.parent_clk_count = ARRAY_SIZE(spi_parent_clk);

	spi_register_board_info(spi_board_info_audio, ARRAY_SIZE(spi_board_info_audio));
        spi_register_board_info(enterprise_spi_board_info_rawchip, ARRAY_SIZE(enterprise_spi_board_info_rawchip));
	platform_device_register(&tegra_spi_device2);
	tegra_spi_device4.dev.platform_data = &cardhu_spi_pdata;
        platform_device_register(&tegra_spi_device4);
}

static struct resource tegra_rtc_resources[] = {
	[0] = {
		.start = TEGRA_RTC_BASE,
		.end = TEGRA_RTC_BASE + TEGRA_RTC_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = INT_RTC,
		.end = INT_RTC,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device tegra_rtc_device = {
	.name = "tegra_rtc",
	.id   = -1,
	.resource = tegra_rtc_resources,
	.num_resources = ARRAY_SIZE(tegra_rtc_resources),
};

static struct platform_device tegra_camera = {
	.name = "tegra_camera",
	.id = -1,
};

static struct resource ram_console_resources[] = {
	{
		.flags = IORESOURCE_MEM,
	},
};

static struct platform_device ram_console_device = {
	.name 		= "ram_console",
	.id 		= -1,
	.num_resources	= ARRAY_SIZE(ram_console_resources),
	.resource	= ram_console_resources,
};

static struct platform_device *enterprise_devices[] __initdata = {
	&tegra_pmu_device,
	&tegra_rtc_device,
	&tegra_udc_device,
#if defined(CONFIG_TEGRA_IOVMM_SMMU)
	&tegra_smmu_device,
#endif
	&tegra_wdt_device,
#if defined(CONFIG_TEGRA_AVP)
	&tegra_avp_device,
#endif
	&tegra_camera,
	&tegra_ahub_device,
	&tegra_dam_device0,
	&tegra_dam_device1,
	&tegra_dam_device2,
	&tegra_i2s_device1,
	&tegra_spdif_device,
	&spdif_dit_device,
	&baseband_dit_device,
	&tegra_pcm_device,
	&enterprise_audio_device,
	&tegra_rawchip_device,
	&tegra_hda_device,
//	&htc_headset_mgr,
#if defined(CONFIG_CRYPTO_DEV_TEGRA_SE)
	&tegra_se_device,
#endif
#if defined(CONFIG_CRYPTO_DEV_TEGRA_AES)
	&tegra_aes_device,
#endif
	&ram_console_device,
};

/* Touchscreen GPIO addresses   */
#define TOUCH_GPIO_IRQ TEGRA_GPIO_PV1
#define TOUCH_GPIO_RST TEGRA_GPIO_PF3
#define TOUCH_GPIO_PWD TEGRA_GPIO_PY2

static struct regulator *srio_1v8_en;

int powerfun(int enable)
{
	int ret = 0;
	if(enable >= 2){
	}
	else
	{
		if (srio_1v8_en == NULL) {
			srio_1v8_en = regulator_get(NULL, "v_srio_1v8");
			if (WARN_ON(IS_ERR(srio_1v8_en))) {
				pr_err("[srio_1v8] %s: couldn't get regulator srio_1v8_en: %ld\n",
						__func__, PTR_ERR(srio_1v8_en));
				ret = -1;
				goto exit;
			}
		}
		if(enable){
			ret = regulator_enable(srio_1v8_en);
			mdelay(10);
		}else{
			ret = regulator_disable(srio_1v8_en);
		}
	}
exit:
	return ret;

}

static struct synaptics_i2c_rmi_platform_data edge_ts_3k_data[] = {
	{
		.version = 0x0100,
		.abs_x_min = 0,
		.abs_x_max = 1100,
		.abs_y_min = 0,
		.abs_y_max = 1755,
		//.flags = SYNAPTICS_FLIP_Y,
		.gpio_irq = TOUCH_GPIO_IRQ,
		.default_config = 1,
		.power = powerfun,
		.source = 1, 
		.config = {
			0x00 , 0x00 , 0x00 , 0x00 , //config version
			0x84 , 0x0F , 0x03 , 0x1E , 0x05 , 0x20 , 
			0xB1 , 0x00 , 0x0B , 0x19 , 0x19 , 0x00 , 0x00 , 0x4C , 0x04 , 0x6C , 
			0x07 , 0x1E , 0x05 , 0x28 , 0xF5 , 0x28 , 0x1E , 0x05 , 0x01 , 0x48 , 
			0xFD , 0x41 , 0xFE , 0x00 , 0x48 , 0x00 , 0x48 , 0xF1 , 0xC5 , 0x79 , 
			0xC8 , 0x00 , 0x70 , 0x00 , 0x00 , 0x00 , 0x00 , 0x0A , 0x04 , 0xC0 , 
			0x00 , 0x02 , 0xF3 , 0x00 , 0x80 , 0x03 , 0x0D , 0x1E , 0x00 , 0x32 , 
			0x00 , 0x19 , 0x04 , 0x1E , 0x00 , 0x10 , 0x0A , 0x00 , 0x19 , 0x11 , 
			0x1B , 0x14 , 0x1A , 0x12 , 0x18 , 0x0F , 0x17 , 0x16 , 0x0D , 0x0A , 
			0x0E , 0x08 , 0x09 , 0x15 , 0x07 , 0x02 , 0x01 , 0x0B , 0x00 , 0x0C , 
			0xFF , 0xFF , 0xFF , 0xFF , 0xFF , 0x07 , 0x04 , 0x0E , 0x05 , 0x0C , 
			0x02 , 0x0F , 0x06 , 0x12 , 0x01 , 0x10 , 0x08 , 0xFF , 0xFF , 0xFF , 
			0xFF , 0xC0 , 0xC0 , 0xC0 , 0xC0 , 0xA0 , 0xA0 , 0xA0 , 0xA0 , 0x4B , 
			0x4A , 0x48 , 0x47 , 0x45 , 0x44 , 0x42 , 0x40 , 0x00 , 0x02 , 0x04 , 
			0x06 , 0x08 , 0x0A , 0x0D , 0x10 , 0x00 , 0xFF , 0xFF , 0xFF , 0xFF , 
			0xFF , 0xFF , 0xFF , 0xFF , 0xFF , 0x00 , 0xFF , 0xFF , 0x00 , 0xC0 , 
			0x80 , 0x00 , 0x10 , 0x00 , 0x10 , 0x00 , 0x10 , 0x00 , 0x10 , 0x80 , 
			0x80 , 0x80 , 0x80 , 0x80 , 0x80 , 0x80 , 0x80 , 0x80 , 0x80 , 0x80 , 
			0x80 , 0x80 , 0x80 , 0x80 , 0x80 , 0x80 , 0x80 , 0x80 , 0x80 , 0x80 , 
			0x80 , 0x80 , 0x80 , 0x80 , 0x80 , 0x80 , 0x02 , 0x02 , 0x02 , 0x02 , 
			0x02 , 0x02 , 0x02 , 0x02 , 0x20 , 0x20 , 0x20 , 0x20 , 0x20 , 0x20 , 
			0x20 , 0x20 , 0x58 , 0x5B , 0x5D , 0x5F , 0x61 , 0x63 , 0x66 , 0x69 , 
			0x48 , 0x41 , 0x00 , 0x1E , 0x19 , 0x05 , 0xFD , 0xFE , 0x3D , 0x08 
		}
		//.sensitivity_adjust = 0,
		//.finger_support = 10,
		//.display_height = 960,
	},
	{
		.version = 0x0100,
		.abs_x_min = 0,
		.abs_x_max = 1100,
		.abs_y_min = 0,
		.abs_y_max = 1755,
		//.flags = SYNAPTICS_FLIP_Y,
		.gpio_irq = TOUCH_GPIO_IRQ,
		.power = powerfun,
		.source = 0, 
		.config = {
			0x00 , 0x00 , 0x00 , 0x01 , //config version
			0x84 , 0x0F , 0x03 , 0x1E , 0x05 , 0x20 , 
			0xB1 , 0x00 , 0x0B , 0x19 , 0x19 , 0x00 , 0x00 , 0x4C , 0x04 , 0x6C , 
			0x07 , 0x1E , 0x05 , 0x28 , 0xF5 , 0x28 , 0x1E , 0x05 , 0x01 , 0x48 , 
			0xFD , 0x41 , 0xFE , 0x00 , 0x48 , 0x00 , 0x48 , 0xF1 , 0xC5 , 0x79 , 
			0xC8 , 0x00 , 0x70 , 0x00 , 0x00 , 0x00 , 0x00 , 0x0A , 0x04 , 0xC0 , 
			0x00 , 0x02 , 0xF3 , 0x00 , 0x80 , 0x03 , 0x0D , 0x1E , 0x00 , 0x32 , 
			0x00 , 0x19 , 0x04 , 0x1E , 0x00 , 0x10 , 0x0A , 0x00 , 0x19 , 0x11 , 
			0x1B , 0x14 , 0x1A , 0x12 , 0x18 , 0x0F , 0x17 , 0x16 , 0x0D , 0x0A , 
			0x0E , 0x08 , 0x09 , 0x15 , 0x07 , 0x02 , 0x01 , 0x0B , 0x00 , 0x0C , 
			0xFF , 0xFF , 0xFF , 0xFF , 0xFF , 0x07 , 0x04 , 0x0E , 0x05 , 0x0C , 
			0x02 , 0x0F , 0x06 , 0x12 , 0x01 , 0x10 , 0x08 , 0xFF , 0xFF , 0xFF , 
			0xFF , 0xC0 , 0xC0 , 0xC0 , 0xC0 , 0xA0 , 0xA0 , 0xA0 , 0xA0 , 0x4B , 
			0x4A , 0x48 , 0x47 , 0x45 , 0x44 , 0x42 , 0x40 , 0x00 , 0x02 , 0x04 , 
			0x06 , 0x08 , 0x0A , 0x0D , 0x10 , 0x00 , 0xFF , 0xFF , 0xFF , 0xFF , 
			0xFF , 0xFF , 0xFF , 0xFF , 0xFF , 0x00 , 0xFF , 0xFF , 0x00 , 0xC0 , 
			0x80 , 0x00 , 0x10 , 0x00 , 0x10 , 0x00 , 0x10 , 0x00 , 0x10 , 0x80 , 
			0x80 , 0x80 , 0x80 , 0x80 , 0x80 , 0x80 , 0x80 , 0x80 , 0x80 , 0x80 , 
			0x80 , 0x80 , 0x80 , 0x80 , 0x80 , 0x80 , 0x80 , 0x80 , 0x80 , 0x80 , 
			0x80 , 0x80 , 0x80 , 0x80 , 0x80 , 0x80 , 0x02 , 0x02 , 0x02 , 0x02 , 
			0x02 , 0x02 , 0x02 , 0x02 , 0x20 , 0x20 , 0x20 , 0x20 , 0x20 , 0x20 , 
			0x20 , 0x20 , 0x58 , 0x5B , 0x5D , 0x5F , 0x61 , 0x63 , 0x66 , 0x69 , 
			0x48 , 0x41 , 0x00 , 0x1E , 0x19 , 0x05 , 0xFD , 0xFE , 0x3D , 0x08 
		}
		//.sensitivity_adjust = 0,
		//.finger_support = 10,
		//.display_height = 960,
	}
};

static struct synaptics_i2c_rmi_platform_data edge_ts_3k_data_XB[] = {
	{
		.version = 0x3330, // fw30
		.abs_x_min = 0,
		.abs_x_max = 1100,
		.abs_y_min = 0,
		.abs_y_max = 1770,
		.notifyFinger = NULL, /* speedupCPU, */
		//.flags = SYNAPTICS_FLIP_Y,
		.gpio_irq = TOUCH_GPIO_IRQ,
		.default_config = 1,
		.power = powerfun,
		.cable_support = 1,
		.source = 1, //YFO
		.customer_register = {0xF9,0x64,0x74,0x32},
		.config = {
			0x35,0x44,0x30,0x38,
			0x00,0x3F,0x03,0x1E,0x05,0xB1,
			0x08,0x0B,0x19,0x19,0x00,0x00,0x4C,0x04,0x75,0x07,
			0x02,0x14,0x1E,0x05,0x37,0xA5,0x16,0xE8,0x03,0x01,
			0x3C,0x17,0x02,0x17,0x01,0xEC,0x4D,0x71,0x51,0xF8,
			0xA7,0xC8,0xAF,0x00,0x50,0x00,0x00,0x00,0x00,0x0A,
			0x04,0xC0,0x00,0x00,0x00,0x00,0x00,0x00,0x19,0x01,
			0x00,0x0A,0x70,0x32,0xA2,0x02,0x28,0x0A,0x0A,0x64,
			0x16,0x0C,0x00,0x02,0xEE,0x00,0x80,0x03,0x0E,0x1F,
			0x11,0x38,0x00,0x13,0x08,0x1B,0x00,0x08,0xFF,0x00,
			0x06,0x0C,0x0D,0x0B,0x15,0x17,0x16,0x18,0x19,0x1A,
			0x1B,0x11,0x14,0x12,0x0F,0x0E,0x09,0x0A,0x07,0x02,
			0x01,0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0x04,0x05,0x02,
			0x06,0x01,0x0C,0x07,0x08,0x0E,0x10,0x0F,0x12,0xFF,
			0xFF,0xFF,0xFF,0xC0,0xC0,0xC0,0xC0,0xC0,0xC8,0xC8,
			0xC8,0x59,0x57,0x55,0x53,0x52,0x50,0x4E,0x4D,0x00,
			0x02,0x04,0x06,0x08,0x0A,0x0C,0x0E,0x00,0xA0,0x0F,
			0xFF,0x28,0x00,0xC8,0x00,0xB3,0xC8,0xCD,0xA0,0x0F,
			0x00,0xC0,0x80,0x00,0x10,0x00,0x10,0x00,0x10,0x00,
			0x10,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
			0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
			0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
			0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
			0x80,0x80,0x80,0x80,0x02,0x02,0x02,0x02,0x02,0x02,
			0x02,0x02,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,
			0x58,0x5B,0x5D,0x5F,0x61,0x63,0x65,0x67,0x00,0x64,
			0x00,0x10,0x0A,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
			0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0xFF,0xFF,
			0xFF,0xFF,0xFF,0xFF,0x51,0x51,0x51,0x51,0xCD,0x0D,
			0x04
		}
	},
	{
		.version = 0x3330, // fw30
		.abs_x_min = 0,
		.abs_x_max = 1100,
		.abs_y_min = 0,
		.abs_y_max = 1770,
		.notifyFinger = NULL, /* speedupCPU,*/
		//.flags = SYNAPTICS_FLIP_Y,
		.gpio_irq = TOUCH_GPIO_IRQ,
		.power = powerfun,
		.cable_support = 1,
		.source = 0, //second source
		.customer_register = {0xF9,0x64,0x74,0x32},
		.config = {
			0x35,0x4A,0x30,0x39,
			0x00,0x3F,0x03,0x1E,0x05,0xB1,
			0x08,0x0B,0x19,0x19,0x00,0x00,0x4C,0x04,0x75,0x07,
			0x02,0x14,0x1E,0x05,0x3A,0x57,0x1F,0xAF,0x02,0x01,
			0x3C,0x17,0x02,0x17,0x01,0xEC,0x4D,0x71,0x51,0xF8,
			0xA7,0xC8,0xAF,0x00,0x50,0x13,0x00,0x00,0x00,0x0A,
			0x04,0xC0,0x00,0x00,0x00,0x00,0x00,0x00,0x19,0x01,
			0x00,0x0A,0x70,0x32,0xA2,0x02,0x28,0x0A,0x0A,0x64,
			0x16,0x0C,0x00,0x02,0xF0,0x00,0x80,0x03,0x0E,0x1F,
			0x11,0x38,0x00,0x13,0x08,0x1B,0x00,0x08,0xFF,0x00,
			0x06,0x0C,0x0D,0x0B,0x15,0x17,0x16,0x18,0x19,0x1A,
			0x1B,0x11,0x14,0x12,0x0F,0x0E,0x09,0x0A,0x07,0x02,
			0x01,0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0x04,0x05,0x02,
			0x06,0x01,0x0C,0x07,0x08,0x0E,0x10,0x0F,0x12,0xFF,
			0xFF,0xFF,0xFF,0xC0,0xC0,0xC0,0xC0,0xC0,0xC8,0xC8,
			0xC8,0x62,0x60,0x5E,0x5C,0x5A,0x58,0x57,0x55,0x00,
			0x02,0x04,0x06,0x08,0x0A,0x0C,0x0E,0x00,0xB8,0x0B,
			0xCD,0x28,0x00,0xC8,0x00,0x80,0xC8,0xCD,0xB8,0x0B,
			0x00,0xC0,0x80,0x00,0x10,0x00,0x10,0x00,0x10,0x00,
			0x10,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
			0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
			0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
			0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
			0x80,0x80,0x80,0x80,0x02,0x02,0x02,0x02,0x02,0x02,
			0x02,0x02,0x30,0x20,0x20,0x20,0x20,0x20,0x20,0x20,
			0x7E,0x57,0x59,0x5B,0x5D,0x5F,0x61,0x63,0x00,0x64,
			0x00,0x10,0x0A,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
			0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0xFF,0xFF,
			0xFF,0xFF,0xFF,0xFF,0x51,0x51,0x51,0x51,0xCD,0x0D,
			0x04
		}
	},
	{
		.version = 0x3230,  //fw20
		.abs_x_min = 0,
		.abs_x_max = 1100,
		.abs_y_min = 0,
		.abs_y_max = 1770,
		.flags = SYNAPTICS_FLIP_Y,
		.gpio_irq = TOUCH_GPIO_IRQ,
		.default_config = 1,
		.power = powerfun,
		.source = 1, //YFO
		.config = {
			0x30, 0x30, 0x30, 0x31, 
			0x84, 0x0F, 0x03, 0x1E, 0x05, 0x20, 
			0xB1, 0x00, 0x0B, 0x19, 0x19, 0x00, 0x00, 0x4C, 0x04, 0x6C, 
			0x07, 0x1E, 0x05, 0x2D, 0x08, 0x0A, 0xA9, 0x02, 0x01, 0x3C, 
			0xFE, 0x43, 0xFE, 0x08, 0x4F, 0x65, 0x52, 0x49, 0xB5, 0xC1, 
			0x9C, 0x00, 0xB8, 0x00, 0x00, 0x00, 0x00, 0x0A, 0x04, 0xAD, 
			0x00, 0x02, 0xF2, 0x00, 0x80, 0x02, 0x0D, 0x1E, 0x00, 0x2E, 
			0x00, 0x19, 0x04, 0x1E, 0x00, 0x10, 0xFF, 0x00, 0x00, 0x01, 
			0x02, 0x07, 0x0A, 0x09, 0x0E, 0x0F, 0x12, 0x14, 0x11, 0x1B, 
			0x1A, 0x19, 0x18, 0x16, 0x17, 0x15, 0x0B, 0x0D, 0x0C, 0x06, 
			0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x04, 0x05, 0x02, 0x06, 0x01, 
			0x0C, 0x07, 0x08, 0x0E, 0x10, 0x0F, 0x12, 0xFF, 0xFF, 0xFF, 
			0xFF, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0x4F, 
			0x4F, 0x4F, 0x4F, 0x4F, 0x4F, 0x4F, 0x4F, 0x00, 0x03, 0x06, 
			0x09, 0x0C, 0x0F, 0x12, 0x15, 0x00, 0xB8, 0x0B, 0xB3, 0xE8, 
			0x03, 0xB8, 0x0B, 0x33, 0x28, 0x80, 0xE8, 0x03, 0x00, 0xC0, 
			0x80, 0x00, 0x10, 0x00, 0x10, 0x00, 0x10, 0x00, 0x10, 0x80, 
			0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 
			0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 
			0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x02, 0x02, 0x02, 0x02, 
			0x02, 0x02, 0x02, 0x02, 0x30, 0x20, 0x20, 0x20, 0x20, 0x20, 
			0x20, 0x20, 0x7E, 0x57, 0x5A, 0x5D, 0x60, 0x63, 0x66, 0x69, 
			0x3C, 0x43, 0x00, 0x1E, 0x19, 0x05, 0xFE, 0xFE, 0x3D, 0x08
		}
	},
	{
		.version = 0x3230, //fw20
		.abs_x_min = 0,
		.abs_x_max = 1100,
		.abs_y_min = 0,
		.abs_y_max = 1770,
		//.flags = SYNAPTICS_FLIP_Y,
		.gpio_irq = TOUCH_GPIO_IRQ,
		.power = powerfun,
		.source = 0, //wintek
		.config = {
			0x30, 0x30, 0x30, 0x30, 
			0x84, 0xFF, 0x03, 0x1E, 0x05, 0x20, 0xB1, 0x00, 0x0B, 0x19, 
			0x19, 0x00, 0x00, 0x4C, 0x04, 0x6C, 0x07, 0x1E, 0x05, 0x2D, 
			0x16, 0x0F, 0x70, 0x03, 0x01, 0x37, 0xFF, 0x3D, 0xFE, 0x08, 
			0x4F, 0x99, 0x51, 0x6B, 0xCC, 0x34, 0xD4, 0x00, 0x70, 0x00, 
			0x00, 0x00, 0x00, 0x0A, 0x04, 0xC0, 0x00, 0x02, 0x02, 0x01, 
			0x80, 0x03, 0x0D, 0x1F, 0x00, 0x50, 0x00, 0x19, 0x04, 0x1E, 
			0x00, 0x10, 0xFF, 0x00, 0x06, 0x0C, 0x0D, 0x0B, 0x15, 0x17, 
			0x16, 0x18, 0x19, 0x1A, 0x1B, 0x11, 0x14, 0x12, 0x0F, 0x0E, 
			0x09, 0x0A, 0x07, 0x02, 0x01, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 
			0xFF, 0x04, 0x05, 0x02, 0x06, 0x01, 0x0C, 0x07, 0x08, 0x0E, 
			0x10, 0x0F, 0x12, 0xFF, 0xFF, 0xFF, 0xFF, 0x80, 0x80, 0x80, 
			0x80, 0x80, 0x80, 0x80, 0x88, 0x38, 0x38, 0x38, 0x38, 0x38, 
			0x38, 0x38, 0x38, 0x00, 0x04, 0x09, 0x0E, 0x14, 0x19, 0x1E, 
			0x23, 0x00, 0x68, 0x04, 0x80, 0x68, 0x04, 0xDE, 0x2F, 0xC0, 
			0x14, 0xCC, 0x0D, 0x26, 0x00, 0xC0, 0x80, 0x00, 0x10, 0x00, 
			0x10, 0x00, 0x10, 0x00, 0x10, 0x80, 0x80, 0x80, 0x80, 0x80, 
			0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 
			0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 
			0x80, 0x80, 0x02, 0x02, 0x04, 0x07, 0x03, 0x0B, 0x04, 0x07, 
			0x20, 0x20, 0x30, 0x50, 0x20, 0x70, 0x30, 0x50, 0x76, 0x7B, 
			0x60, 0x5F, 0x5C, 0x5B, 0x6F, 0x6E
		}
	},

};

static struct i2c_board_info __initdata synaptics_i2c_info[] = {
	{
		I2C_BOARD_INFO(SYNAPTICS_3200_NAME, 0x20),
		.platform_data = &edge_ts_3k_data,
		.irq = TEGRA_GPIO_TO_IRQ(TOUCH_GPIO_IRQ),
	},
};

static struct i2c_board_info __initdata synaptics_i2c_info_XB[] = {
	{
		I2C_BOARD_INFO(SYNAPTICS_3200_NAME, 0x20),
		.platform_data = &edge_ts_3k_data_XB,
		.irq = TEGRA_GPIO_TO_IRQ(TOUCH_GPIO_IRQ),
	},

};

struct tegra_touchscreen_init __initdata synaptics_init_data = {
	.irq_gpio = TOUCH_GPIO_IRQ,                    /* GPIO1 Value for IRQ */
	.rst_gpio = TOUCH_GPIO_RST,                    /* GPIO2 Value for RST */
	.sv_gpio1 = {1, TOUCH_GPIO_RST, 0, 1},
	.sv_gpio2 = {1, TOUCH_GPIO_RST, 1, 100},       /* Valid, GPIOx, Set value, Delay      */
	.ts_boardinfo = {1, synaptics_i2c_info, 1}      /* BusNum, BoardInfo, Value     */
};

struct tegra_touchscreen_init __initdata synaptics_init_data_XB = {
	.irq_gpio = TOUCH_GPIO_IRQ,                    /* GPIO1 Value for IRQ */
	.rst_gpio = TOUCH_GPIO_RST,                    /* GPIO2 Value for RST */
	.sv_gpio1 = {1, TOUCH_GPIO_RST, 0, 1},
	.sv_gpio2 = {1, TOUCH_GPIO_RST, 1, 100},       /* Valid, GPIOx, Set value, Delay      */
	.ts_boardinfo = {1, synaptics_i2c_info_XB, 1}      /* BusNum, BoardInfo, Value     */
};

int __init generic_touch_init(struct tegra_touchscreen_init *tsdata)
{
	int ret;

	ret = gpio_request(tsdata->rst_gpio, "touch-reset");
	if (ret < 0) {
		pr_err("%s(): gpio_request() fails for gpio %d (touch-reset)\n",
			__func__, tsdata->rst_gpio);
		return ret;
	} else {
                gpio_direction_output(tsdata->rst_gpio,1);
                tegra_gpio_enable(tsdata->rst_gpio);
                gpio_set_value(tsdata->rst_gpio, 1);
                msleep(1);
	}

	ret = gpio_request(tsdata->irq_gpio, "touch-irq");
	if (ret < 0) {
		pr_err("%s(): gpio_request() fails for gpio %d (touch-irq)\n",
			__func__, tsdata->irq_gpio);
		gpio_free(tsdata->irq_gpio);
		return ret;
	} else {
		gpio_direction_input(tsdata->irq_gpio);
		tegra_gpio_enable(tsdata->irq_gpio);
	}

	tegra_pinmux_set_pullupdown(TEGRA_PINGROUP_GPIO_PV1, TEGRA_PUPD_NORMAL);

	i2c_register_board_info(tsdata->ts_boardinfo.busnum,
		tsdata->ts_boardinfo.info,
		tsdata->ts_boardinfo.n);

	return 0;
}

static int __init enterprise_touch_init(void)
{
	int retval = 0;
	struct board_info BoardInfo;

	tegra_get_board_info(&BoardInfo);
	
	if (htc_get_pcbid_info() == PROJECT_PHASE_XA)
		retval = generic_touch_init(&synaptics_init_data);
	else
		retval = generic_touch_init(&synaptics_init_data_XB);

	return retval;
}

static struct usb_phy_plat_data tegra_usb_phy_pdata[] = {
	[0] = {
			.instance = 0,
			.vbus_gpio = -1,
			.vbus_reg_supply = "usb_vbus",
			.vbus_irq = ENT_TPS80031_IRQ_BASE +
							TPS80031_INT_VBUS_DET,
			.drive_strength = 0x7F,
			.drive_slew = 0x0,
	},
	[1] = {
			.instance = 1,
			.vbus_gpio = -1,
	},
	[2] = {
			.instance = 2,
			.vbus_gpio = -1,
	},
};

static struct tegra_uhsic_config uhsic_phy_config = {
	.enable_gpio = -1,
	.reset_gpio = -1,
	.sync_start_delay = 9,
	.idle_wait_delay = 17,
	.term_range_adj = 0,
	.elastic_underrun_limit = 16,
	.elastic_overrun_limit = 16,
};

static struct tegra_ehci_platform_data tegra_ehci_uhsic_pdata = {
	.phy_type = TEGRA_USB_PHY_TYPE_HSIC,
	.phy_config = &uhsic_phy_config,
	.operating_mode = TEGRA_USB_HOST,
	.power_down_on_bus_suspend = 0,
	.default_enable = true,
};


static struct tegra_ehci_platform_data tegra_ehci_pdata[] = {
	[0] = {
			.phy_config = &utmi_phy_config[0],
			.operating_mode = TEGRA_USB_HOST,
			.power_down_on_bus_suspend = 1,
			.default_enable = false,
	},
	[1] = {
			.phy_config = &utmi_phy_config[1],
			.operating_mode = TEGRA_USB_HOST,
			.power_down_on_bus_suspend = 1,
			.default_enable = false,
	},
	[2] = {
			.phy_config = &utmi_phy_config[2],
			.operating_mode = TEGRA_USB_HOST,
			.power_down_on_bus_suspend = 1,
			.default_enable = false,
	},
};
struct platform_device *tegra_usb_hsic_host_register(void)
{
	struct platform_device *pdev;
	int val;

	pdev = platform_device_alloc(tegra_ehci2_device.name,
		tegra_ehci2_device.id);
	if (!pdev)
		return NULL;

	val = platform_device_add_resources(pdev, tegra_ehci2_device.resource,
		tegra_ehci2_device.num_resources);
	if (val)
		goto error;

	pdev->dev.dma_mask =  tegra_ehci2_device.dev.dma_mask;
	pdev->dev.coherent_dma_mask = tegra_ehci2_device.dev.coherent_dma_mask;

/* 77544-1 patch */
	val = platform_device_add_data(pdev, &tegra_ehci_uhsic_pdata,
		sizeof(struct tegra_ehci_platform_data));
	if (val)
/* 77544-1 patch */
		goto error;

	val = platform_device_add(pdev);
	if (val)
/* 77544-1 patch */
		goto error;
/* 77544-1 patch */

	return pdev;

error:
	pr_err("%s: failed to add the host contoller device\n", __func__);
	platform_device_put(pdev);
	return NULL;
}

void tegra_usb_hsic_host_unregister(struct platform_device *pdev)
{
	platform_device_unregister(pdev);
}


static struct tegra_otg_platform_data tegra_otg_pdata = {
	.ehci_device = &tegra_ehci1_device,
	.ehci_pdata = &tegra_ehci_pdata[0],
	.rcv_host_en = 1,
};


static int cardu_usb_hsic_postsupend(void)
{
	printk(KERN_INFO"%s\n",__func__);
	baseband_xmm_set_power_status(BBXMM_PS_L2);
	return 0;
}

static int cardu_usb_hsic_preresume(void)
{
	printk(KERN_INFO"%s\n",__func__);
	baseband_xmm_set_power_status(BBXMM_PS_L2TOL0);
	return 0;
}

static int cardu_usb_hsic_phy_ready(void)
{
	printk(KERN_INFO"%s\n",__func__);
	baseband_xmm_set_power_status(BBXMM_PS_L0);
	return 0;
}

static int cardu_usb_hsic_phy_off(void)
{
	printk(KERN_INFO"%s\n",__func__);
	baseband_xmm_set_power_status(BBXMM_PS_L3);
	return 0;
}

static void enterprise_usb_init(void)
{
	struct	fsl_usb2_platform_data *udc_pdata;

	tegra_usb_phy_init(tegra_usb_phy_pdata, ARRAY_SIZE(tegra_usb_phy_pdata));

	tegra_otg_device.dev.platform_data = &tegra_otg_pdata;
	platform_device_register(&tegra_otg_device);

	udc_pdata = tegra_udc_device.dev.platform_data;
	android_usb_pdata.serial_number = board_serialno();
	android_usb_pdata.products[0].product_id =
	android_usb_pdata.product_id;
#if defined(CONFIG_USB_CDROM)
	if (board_mfg_mode() == 0 /* normal mode */) {
		android_usb_pdata.nluns = 2;
		android_usb_pdata.cdrom_lun = 0x2;
	}
#endif

	uhsic_phy_config.postsuspend = cardu_usb_hsic_postsupend;
	uhsic_phy_config.preresume = cardu_usb_hsic_preresume;
	uhsic_phy_config.usb_phy_ready = cardu_usb_hsic_phy_ready;
	uhsic_phy_config.post_phy_off = cardu_usb_hsic_phy_off;

//	platform_device_register(&func_switch_device);
	platform_device_register(&android_usb_device);
}

static int32_t get_tegra_adc_cb(void)
{
	int32_t adc = ~0;

	tps80032_adc_select_and_read(&adc, 6);
	adc = TPS80032ADC_12BIT(adc);
	return adc;
}

static void config_tegra_usb_id_gpios(bool output)
{
	if (output) {
		if (gpio_direction_output(TEGRA_GPIO_USB_ID, 1) < 0) {
			pr_info("[CABLE] %s: TEGRA_GPIO_USB_ID dir NG\n", __func__);
			return;
		}
		tegra_gpio_enable(TEGRA_GPIO_USB_ID);
	}
	else {
		if (gpio_direction_input(TEGRA_GPIO_USB_ID) < 0) {
			pr_info("[CABLE] %s: TEGRA_GPIO_USB_ID dir setup failed\n", __func__);
			return;
		}
		tegra_gpio_enable(TEGRA_GPIO_USB_ID);
	}
}

void config_tegra_desk_aud_gpios(bool output, bool out_val)
{
	if (output) {
		if (gpio_direction_output(TEGRA_GPIO_DESK_AUD, out_val) < 0) {
			pr_info("[CABLE] %s: TEGRA_GPIO_DESK_AUD dir NG\n", __func__);
			return;
		}
		tegra_gpio_enable(TEGRA_GPIO_DESK_AUD);
	}
	else {
		if (gpio_direction_input(TEGRA_GPIO_DESK_AUD) < 0) {
			pr_info("[CABLE] %s: TEGRA_GPIO_DESK_AUD dir setup failed\n", __func__);
			return;
		}
		tegra_gpio_enable(TEGRA_GPIO_DESK_AUD);
	}
}
EXPORT_SYMBOL(config_tegra_desk_aud_gpios);

static void tegra_usb_dpdn_switch(int path)
{
	int polarity = 1; /* high = mhl */
	int mhl = (path == PATH_MHL);

	switch (path) {
		case PATH_USB:
		case PATH_MHL:
			pr_info("[CABLE] %s: Set %s path\n", __func__, mhl ? "MHL" : "USB");
			gpio_set_value(TEGRA_GPIO_MHL_USB_SEL, (mhl ^ !polarity) ? 1 : 0);
			break;
	}

#ifdef CONFIG_TEGRA_HDMI_MHL
	sii9234_change_usb_owner((path == PATH_MHL) ? 1 : 0);
#endif
}

static void cable_tegra_gpio_init(void);
static struct cable_detect_platform_data cable_detect_pdata = {
	/*.vbus_mpp_gpio	= PM8058_MPP_PM_TO_SYS(10),*/
	/*.vbus_mpp_config 	= pm8058_usb_config,*/
	/*.vbus_mpp_irq		= PM8058_CBLPWR_IRQ(PM8058_IRQ_BASE),*/
	.detect_type		= CABLE_TYPE_PMIC_ADC,
	.usb_id_pin_gpio	= TEGRA_GPIO_USB_ID,
	.mhl_reset_gpio		= TEGRA_GPIO_MHL_RST,
	.usb_dpdn_switch	= tegra_usb_dpdn_switch,
	/*.mpp_data = {
		.usbid_mpp	= XOADC_MPP_4,
		.usbid_amux	= PM_MPP_AIN_AMUX_CH5,
	},*/
	.config_usb_id_gpios	= config_tegra_usb_id_gpios,
	.config_desk_aud_gpios  = config_tegra_desk_aud_gpios,
	.get_adc_cb		= get_tegra_adc_cb,
	.cable_gpio_init	= cable_tegra_gpio_init,
#ifdef CONFIG_TEGRA_HDMI_MHL
	/*.mhl_1v2_power = mhl_sii9234_1v2_power,*/
#endif
};

static struct platform_device cable_detect_device = {
	.name	= "cable_detect",
	.id	= -1,
	.dev	= {
		.platform_data = &cable_detect_pdata,
	},
};

static void cable_tegra_gpio_init(void)
{
	if (cable_detect_pdata.usb_id_pin_gpio >= 0) {
		if (gpio_request(cable_detect_pdata.usb_id_pin_gpio, "USB_ID_WAKE") < 0) {
			pr_err("[CABLE:ERR] %s: cable_detect_pdata.usb_id_pin_gpio req NG\n", __func__);
			return;
		}
		if (gpio_direction_input(cable_detect_pdata.usb_id_pin_gpio) < 0) {
			pr_err("[CABLE:ERR] %s: cable_detect_pdata.usb_id_pin_gpio dir setup failed\n", __func__);
			return;
		}
		tegra_gpio_enable(cable_detect_pdata.usb_id_pin_gpio);
	}

	if (TEGRA_GPIO_DESK_AUD >= 0) {
		if (gpio_request(TEGRA_GPIO_DESK_AUD, "AUD_DOCK_OUT_EN") < 0) {
			pr_err("[CABLE:ERR] %s: TEGRA_GPIO_DESK_AUD req NG\n", __func__);
			return;
		}
		if (gpio_direction_input(TEGRA_GPIO_DESK_AUD) < 0) {
			pr_err("[CABLE:ERR] %s: TEGRA_GPIO_DESK_AUD dir setup failed\n", __func__);
			return;
		}
		tegra_gpio_enable(TEGRA_GPIO_DESK_AUD);
	}
}

static void enterprise_cable_detect_init(void)
{
	platform_device_register(&cable_detect_device);
}

static void enterprise_gps_init(void)
{
	//tegra_gpio_enable(TEGRA_GPIO_PE4);
	//tegra_gpio_enable(TEGRA_GPIO_PE5);
}

//#if CONFIG_IMC_FLASHLESS
static struct baseband_power_platform_data tegra_baseband_power_data = {
	.baseband_type = BASEBAND_XMM,
	.modem = {
	.xmm = {
			.bb_rst = XMM6260_GPIO_BB_RST,
			.bb_on = XMM6260_GPIO_BB_ON,
			.ipc_bb_wake = XMM6260_GPIO_IPC_BB_WAKE,
			.ipc_ap_wake = XMM6260_GPIO_IPC_AP_WAKE,
			.ipc_hsic_active = XMM6260_GPIO_IPC_HSIC_ACTIVE,
			.ipc_hsic_sus_req = XMM6260_GPIO_IPC_HSIC_SUS_REQ,
		},
	},
};

static struct platform_device tegra_baseband_power_device = {
	.name = "baseband_xmm_power",
	.id = -1,
	.dev = {
		.platform_data = &tegra_baseband_power_data,
	},
};

static struct platform_device tegra_baseband_power2_device = {
	.name = "baseband_xmm_power2",
	.id = -1,
	.dev = {
		.platform_data = &tegra_baseband_power_data,
	},
};

static struct platform_device simhotswap_device = {
	.name	= "htc_simhotswap",
	.id = -1,
};

//#else
/*
static struct baseband_power_platform_data tegra_baseband_power_data = {
	.baseband_type = BASEBAND_XMM,
	.modem = {
	.xmm = {
			.bb_rst = XMM_GPIO_BB_RST,
			.bb_on = XMM_GPIO_BB_ON,
			.ipc_bb_wake = XMM_GPIO_IPC_BB_WAKE,
			.ipc_ap_wake = XMM_GPIO_IPC_AP_WAKE,
			.ipc_hsic_active = XMM_GPIO_IPC_HSIC_ACTIVE,
			.ipc_hsic_sus_req = XMM_GPIO_IPC_HSIC_SUS_REQ,
			.hsic_device = &tegra_ehci2_device,
		},
	},
};

static struct platform_device tegra_baseband_power_device = {
	.name = "baseband_xmm_power",
	.id = -1,
	.dev = {
		.platform_data = &tegra_baseband_power_data,
	},
};
*/
//#endif

static void enterprise_modem_init(void)
{
	struct board_info board_info;
//	int w_disable_gpio;


//	tegra_get_board_info(&board_info);
//	switch (board_info.board_id) {
//	case BOARD_E1291:
//	{
		pr_info("%s: enable baseband gpio(s)\n", __func__);
		/* enable baseband gpio(s) */
		tegra_gpio_enable(BB_VDD_EN);
//#if CONFIG_IMC_FLASHLESS
		tegra_gpio_enable(AP2BB_RST_PWRDWNn);
		tegra_gpio_enable(AP2BB_RSTn);
		tegra_gpio_enable(AP2BB_PWRON);
		tegra_gpio_enable(BB2AP_RADIO_FATAL);
		tegra_gpio_enable(tegra_baseband_power_data.modem.generic.mdm_reset);
		tegra_gpio_enable(tegra_baseband_power_data.modem.generic.mdm_on);
		tegra_gpio_enable(tegra_baseband_power_data.modem.generic.ap2mdm_ack);
		tegra_gpio_enable(tegra_baseband_power_data.modem.generic.mdm2ap_ack);
		tegra_gpio_enable(tegra_baseband_power_data.modem.generic.ap2mdm_ack2);
		tegra_gpio_enable(tegra_baseband_power_data.modem.generic.mdm2ap_ack2);
		/* HTC: Seshendra patch 1117 */
		tegra_baseband_power_data.hsic_register = &tegra_usb_hsic_host_register;
		tegra_baseband_power_data.hsic_unregister = &tegra_usb_hsic_host_unregister;

		platform_device_register(&tegra_baseband_power_device);
		platform_device_register(&tegra_baseband_power2_device);
		platform_device_register(&simhotswap_device);
		
//#else
/*
			tegra_gpio_enable(
				tegra_baseband_power_data.modem.xmm.bb_rst);
			tegra_gpio_enable(
				tegra_baseband_power_data.modem.xmm.bb_on);
			tegra_gpio_enable(
				tegra_baseband_power_data.modem.xmm.ipc_bb_wake);
			tegra_gpio_enable(
				tegra_baseband_power_data.modem.xmm.ipc_ap_wake);
			tegra_gpio_enable(
				tegra_baseband_power_data.modem.xmm.ipc_hsic_active);
			tegra_gpio_enable(
				tegra_baseband_power_data.modem.xmm.ipc_hsic_sus_req);
			platform_device_register(&tegra_baseband_power_device);
			printk(KERN_INFO "[Xmm]  %s tegra_baseband_power_device\n",__func__);
*/
//#endif

		
#if 1		
		// TEGRA_GPIO_PI5
		printk(KERN_INFO"%s: gpio config for sim_det#.", __func__);
                int ret;
		ret = gpio_request(TEGRA_GPIO_PI5, "sim_det#");
		if (ret < 0)
			pr_err("[FLT] %s: gpio_request failed for gpio %s\n",
				__func__, "sin_init");
		ret = gpio_direction_input(TEGRA_GPIO_PI5);


		if (ret < 0) {
			pr_err("[FLT] %s: gpio_direction_output failed %d\n", __func__, ret);
			gpio_free(TEGRA_GPIO_PI5);
			return;
		}
		tegra_gpio_enable(TEGRA_GPIO_PI5);
		gpio_export(TEGRA_GPIO_PI5, true);
#endif
		//jerry.pj--
		
/*enable core dumo dectect++*/
			printk(KERN_INFO"%s: gpio config for core dump when radio fatal error.", __func__);
			/* TEGRA_GPIO_PN2*/
				ret = gpio_request(TEGRA_GPIO_PN2, "core_dump");
				if (ret < 0)
					pr_err("[FLT] %s: gpio_request failed for gpio %s\n",
						__func__, "core_dump");
				ret = gpio_direction_input(TEGRA_GPIO_PN2);
				if (ret < 0) {
					pr_err("[FLT] %s: gpio_direction_input failed %d\n", __func__, ret);
					gpio_free(TEGRA_GPIO_PN2);
					return;
				}
				tegra_gpio_enable(TEGRA_GPIO_PN2);
				gpio_export(TEGRA_GPIO_PN2, true);
/*enable core dumo dectect--*/

		//platform_device_register(&tegra_baseband_power_device);
//		platform_device_register(&tegra_baseband_power2_device);
//	} break;
//	default:
//		break;
//	}
}

static void gpio_o_l(int gpio, char* name)
{
	int ret = gpio_request(gpio, name);
	if (ret < 0)
	{
		pr_err("[KW] %s: gpio_request failed for gpio %s\n",
			__func__, name);
		//return;
	}
	ret = gpio_direction_output(gpio, 0);
	if (ret < 0) {
		pr_err("[KW] %s: gpio_direction_output failed %d\n", __func__, ret);
		gpio_free(gpio);
		return;
	}
	tegra_gpio_enable(gpio);
	gpio_export(gpio, true);
}


static void modem_not_init(void)
{
	pr_info("%s: disable gpio\n", __func__);

	gpio_o_l(TEGRA_GPIO_PM4, "TEGRA_GPIO_PM4");
	gpio_o_l(TEGRA_GPIO_PC1, "TEGRA_GPIO_PC1");
	gpio_o_l(TEGRA_GPIO_PN0, "TEGRA_GPIO_PN0");
	gpio_o_l(TEGRA_GPIO_PN3, "TEGRA_GPIO_PN3");
	gpio_o_l(TEGRA_GPIO_PC6, "TEGRA_GPIO_PC6");
	gpio_o_l(TEGRA_GPIO_PJ0, "TEGRA_GPIO_PJ0");
	gpio_o_l(TEGRA_GPIO_PV0, "TEGRA_GPIO_PV0");
	gpio_o_l(TEGRA_GPIO_PN1, "TEGRA_GPIO_PN1");
	gpio_o_l(TEGRA_GPIO_PN2, "TEGRA_GPIO_PN2");
	gpio_o_l(TEGRA_GPIO_PJ7, "TEGRA_GPIO_PJ7");
	gpio_o_l(TEGRA_GPIO_PK7, "TEGRA_GPIO_PK7");
	gpio_o_l(TEGRA_GPIO_PB0, "TEGRA_GPIO_PB0");
	gpio_o_l(TEGRA_GPIO_PB1, "TEGRA_GPIO_PB1");

}


static void enterprise_baseband_init(void)
{
//	modem_not_init();
//	return;
	enterprise_modem_init();
#if 0
	int modem_id = tegra_get_modem_id();

	switch (modem_id) {
	case 1: /* PH450 ULPI */
		enterprise_modem_init();
		break;
		case 2: /* 6260 HSIC */
		break;
	}
#endif
}

//virtual key 
static ssize_t Aproj_virtual_keys_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf,
			__stringify(EV_KEY) ":" __stringify(KEY_HOME)       ":111:1340:40:120"
			":" __stringify(EV_KEY) ":" __stringify(KEY_MENU)   ":267:1340:40:120"
			":" __stringify(EV_KEY) ":" __stringify(KEY_BACK)   ":432:1340:54:120"
			":" __stringify(EV_KEY) ":" __stringify(KEY_SEARCH) ":594:1340:40:120"
			"\n");
}
static struct kobj_attribute Aproj_virtual_keys_attr = {
	.attr = {
		.name = "virtualkeys.synaptics-rmi-touchscreen",
		.mode = S_IRUGO,
	},
	.show = &Aproj_virtual_keys_show,
};

static struct attribute *Aproj_properties_attrs[] = {
	&Aproj_virtual_keys_attr.attr,
	NULL
};

static struct attribute_group Aproj_properties_attr_group = {
	.attrs = Aproj_properties_attrs,
};

//virtual key for XC board and later (3 virtual keys)
static ssize_t Aproj_virtual_keys_show_XC(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf,
			__stringify(EV_KEY) ":" __stringify(KEY_BACK)       ":113:1345:124:86"
			":" __stringify(EV_KEY) ":" __stringify(KEY_HOME)   ":360:1345:124:86"
			":" __stringify(EV_KEY) ":" __stringify(KEY_F12) ":610:1345:124:86"
			"\n");
}
static struct kobj_attribute Aproj_virtual_keys_attr_XC = {
	.attr = {
		.name = "virtualkeys.synaptics-rmi-touchscreen",
		.mode = S_IRUGO,
	},
	.show = &Aproj_virtual_keys_show_XC,
};

static struct attribute *Aproj_properties_attrs_XC[] = {
	&Aproj_virtual_keys_attr_XC.attr,
	NULL
};

static struct attribute_group Aproj_properties_attr_group_XC = {
	.attrs = Aproj_properties_attrs_XC,
};

static struct keyreset_platform_data enr_reset_keys_pdata = {
	.keys_down = {
		KEY_POWER,
		KEY_VOLUMEDOWN,
		KEY_VOLUMEUP,
		0
	},
};

static struct platform_device enr_reset_keys_device = {
	.name = KEYRESET_NAME,
	.dev.platform_data = &enr_reset_keys_pdata,
};

extern int emmc_partition_read_proc(char *page, char **start, off_t off,
			   int count, int *eof, void *data);

static void enr_u_basic_gpio_setup(void)
{
	int pcbid = htc_get_pcbid_info();
	static struct gpio_callbacks callbacks = {NULL};

	switch (pcbid) {
		case PROJECT_PHASE_XA:
		case PROJECT_PHASE_XB:
			break;
		case PROJECT_PHASE_XC:
			callbacks.init           = enr_u_xc_no_owner_gpio_init;
			break;
		case PROJECT_PHASE_XD:
			callbacks.init           = enr_u_xd_no_owner_gpio_init;
			break;
		case PROJECT_PHASE_XE:
		default:
			callbacks.init           = enr_u_xe_no_owner_gpio_init;
			break;
	}

	register_gpio_basic_callbacks(&callbacks);

}

static void __init tegra_enterprise_init(void)
{
	int board_id = 0;
	struct kobject *properties_kobj;  	

	tegra_thermal_init(&thermal_data);
	BOOT_DEBUG_LOG_ENTER("<machine>.init_machine");
	board_id = htc_get_pcbid_info();
	tegra_clk_init_from_table(enterprise_clk_init_table);
	endeavoru_pinmux_init();
	enterprise_i2c_init();
	enterprise_uart_init();
	enterprise_spi_init();
	enterprise_usb_init();
//	andusb_plat.serial_number = board_serialno();
	enterprise_tsensor_init();
	platform_add_devices(enterprise_devices, ARRAY_SIZE(enterprise_devices));
	if (board_id <= PROJECT_PHASE_XD)
		platform_device_register(&htc_headset_mgr);
	else
		platform_device_register(&htc_headset_mgr_xe);

	if (board_id >= PROJECT_PHASE_A && engineer_id != 1) {
		mhl_sii_device_data.ci2ca = 1;
		mhl_sii_device_data.enMhlD3Guard = true;
	}

	enterprise_regulator_init();
	enterprise_sdhci_init();
	headset_uart_init();
#ifdef CONFIG_CPU_FREQ_GOV_ONDEMAND_2_PHASE
		set_two_phase_freq(1000000);
#endif

#ifdef CONFIG_TEGRA_EDP_LIMITS
	enterprise_edp_init();
#endif
	//enterprise_kbc_init();
#if 1    // for A project bring up
	A_PROJECT_keys_init();
#endif
#ifdef CONFIG_TEGRA_HDMI_MHL
	i2c_register_board_info(4, i2c_mhl_sii_info,
			ARRAY_SIZE(i2c_mhl_sii_info));
#endif
	enterprise_touch_init();
	//enterprise_gps_init();
	enterprise_baseband_init();
	enterprise_panel_init();
	enterprise_bt_wl128x();
	enterprise_emc_init();
	enterprise_sensors_init();
	if (platform_device_register(&enr_reset_keys_device))
		printk(KERN_WARNING "%s: register reset key fail\n", __func__);
        properties_kobj = kobject_create_and_add("board_properties", NULL);
	if (properties_kobj) {
		if (htc_get_pcbid_info() >= PROJECT_PHASE_XC) {
			sysfs_create_group(properties_kobj, &Aproj_properties_attr_group_XC);
		} else {
			sysfs_create_group(properties_kobj, &Aproj_properties_attr_group);
		}
	}
	enterprise_suspend_init();
	tegra_release_bootloader_fb();
#ifdef CONFIG_TEGRA_HAPTIC2
	haptic_vibrator_init(board_id);
#endif
#ifdef CONFIG_TEGRA_VIBRATOR_ENR
	tegra_vibrator_init();
#endif
	leds_lp5521_init();
	enterprise_flashlight_init();
	enr_u_basic_gpio_setup();
#if defined(CONFIG_CABLE_DETECT_ACCESSORY)
	enterprise_cable_detect_init();
#endif
	struct proc_dir_entry* proc;
	proc = create_proc_read_entry("emmc", 0, NULL, emmc_partition_read_proc, NULL);
	if (proc) {
		printk(KERN_ALERT "[mtd] mount /proc/emmc successfully\n");
	} else {
		printk(KERN_ALERT "[mtd] mount /proc/emmc failed\n");
	}

	proc = create_proc_read_entry("dying_processes", 0, NULL, dying_processors_read_proc, NULL);
	if (!proc)
		printk(KERN_ERR"Create /proc/dying_processes FAILED!\n");

	BOOT_DEBUG_LOG_LEAVE("<machine>.init_machine");
}

static void __init tegra_enterprise_ramconsole_reserve(unsigned long size)
{
	struct resource *res;
	long ret;

	res = platform_get_resource(&ram_console_device, IORESOURCE_MEM, 0);
	if (!res) {
		pr_err("Failed to find memory resource for ram console\n");
		return;
	}
	res->start = memblock_end_of_DRAM() - size;
	res->end = res->start + size - 1;

	ret = memblock_remove(res->start, size);
	if (ret) {
		ram_console_device.resource = NULL;
		ram_console_device.num_resources = 0;
		pr_err("Failed to reserve memory block for ram console\n");
	}
}

static void __init tegra_enterprise_reserve(void)
{
#if defined(CONFIG_NVMAP_CONVERT_CARVEOUT_TO_IOVMM)
	tegra_reserve(0, SZ_4M, SZ_8M);
#else
	tegra_reserve(SZ_128M, SZ_4M, SZ_8M);
#endif
	tegra_enterprise_ramconsole_reserve(SZ_1M);
}

MACHINE_START(ENDEAVORU, "endeavoru")
	.boot_params    = 0x80000100,
	.map_io         = tegra_map_common_io,
	.reserve        = tegra_enterprise_reserve,
	.init_early	= tegra_init_early,
	.init_irq       = tegra_init_irq,
	.timer          = &tegra_timer,
	.init_machine   = tegra_enterprise_init,
MACHINE_END

/* XXX for transition period only, will be removed soon */
MACHINE_START(TEGRA_ENTERPRISE, "endeavoru")
	.boot_params    = 0x80000100,
	.map_io         = tegra_map_common_io,
	.reserve        = tegra_enterprise_reserve,
	.init_early	= tegra_init_early,
	.init_irq       = tegra_init_irq,
	.timer          = &tegra_timer,
	.init_machine   = tegra_enterprise_init,
MACHINE_END
