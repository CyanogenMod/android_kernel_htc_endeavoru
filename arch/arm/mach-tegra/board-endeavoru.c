/*
 * arch/arm/mach-tegra/board-endeavoru.c
 *
 * Copyright (c) 2011-2012, NVIDIA Corporation.
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
#include <linux/spi/spi.h>
#include <linux/tegra_uart.h>
#include <linux/fsl_devices.h>
#include <linux/i2c/atmel_mxt_ts.h>
#include <linux/memblock.h>
#include <linux/gpio_keys.h>
#include <linux/synaptics_i2c_rmi.h>
#include <linux/keyreset.h>
/*#include <linux/nfc/pn544.h>*/
#include <linux/spi-tegra.h>
#include <linux/i2c/tfa9887.h>
#include <linux/leds.h>
#include <linux/leds-lp5521_htc.h>
#include <linux/regulator/consumer.h>
#include <linux/proc_fs.h>

#ifdef CONFIG_TEGRA_VIBRATOR_ENR
#include <linux/tegra_vibrator_enr.h>
#endif

#include <mach/clk.h>
#include <mach/iomap.h>
#include <mach/irqs.h>
#include <mach/pinmux.h>
#include <mach/iomap.h>
#include <mach/io.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/usb_phy.h>
#include <mach/i2s.h>
#include <mach/htc_headset_mgr.h>
#include <mach/htc_headset_gpio.h>
#include <mach/htc_headset_microp.h>
#include <mach/htc_headset_pmic.h>
#include <mach/htc_headset_misc.h>
#include <mach/htc_headset_one_wire.h>
#include <mach/tegra_asoc_pdata.h>
#include <mach/thermal.h>
#include <mach/mhl.h>
#include <mach/tegra-bb-power.h>
#include <mach/htc_bdaddress.h>
#include <mach/htc_util.h>
#include "board.h"
#include "clock.h"
#include "board-endeavoru.h"
#include "baseband-xmm-power.h"
#include "devices.h"
#include "gpio-names.h"
#include "fuse.h"
#include "pm.h"
#include "touch.h"
#include "htc_perf.h"
#include "wakeups-t3.h"
#include <mach/tegra_flashlight.h>

#include <linux/skbuff.h>
#include <linux/ti_wilink_st.h>
#include <linux/wakelock.h>
/* TI 128x Bluetooth begin */
#include <linux/ti_wilink_st.h>
/* TI 128x Bluetooth end */

#define PMC_WAKE_STATUS         0x14
extern bool is_resume_from_deep_suspend(void);
extern void aic3008_set_mic_bias(int en);

#ifdef CONFIG_USB_G_ANDROID
#include <linux/usb/android_composite.h>
#endif
#include <mach/htc_usb.h>
#include <mach/board_htc.h>
#include <linux/tps80032_adc.h>
#include <mach/cable_detect.h>

#include <media/rawchip/rawchip.h>
#include <media/rawchip/Yushan_HTC_Functions.h>

static struct balanced_throttle throttle_list[] = {
	{
		.id = BALANCED_THROTTLE_ID_TJ,
		.throt_tab_size = 10,
		.throt_tab = {
			{      0, 1000 },
			{ 640000, 1000 },
			{ 640000, 1000 },
			{ 640000, 1000 },
			{ 640000, 1000 },
			{ 640000, 1000 },
			{ 760000, 1000 },
			{ 760000, 1050 },
			{1000000, 1050 },
			{1000000, 1100 },
		},
	},
#ifdef CONFIG_TEGRA_SKIN_THROTTLE
	{
		.id = BALANCED_THROTTLE_ID_SKIN,
		.throt_tab_size = 6,
		.throt_tab = {
			{ 640000, 1200 },
			{ 640000, 1200 },
			{ 760000, 1200 },
			{ 760000, 1200 },
			{1000000, 1200 },
			{1000000, 1200 },
		},
	},
#endif
};

/* All units are in millicelsius */
static struct tegra_thermal_data thermal_data = {
	.shutdown_device_id = THERMAL_DEVICE_ID_NCT_EXT,
	.temp_shutdown = 90000,
#if defined(CONFIG_TEGRA_EDP_LIMITS) || defined(CONFIG_TEGRA_THERMAL_THROTTLE)
	.throttle_edp_device_id = THERMAL_DEVICE_ID_NCT_EXT,
#endif
#ifdef CONFIG_TEGRA_EDP_LIMITS
	.edp_offset = TDIODE_OFFSET,  /* edp based on tdiode */
	.hysteresis_edp = 3000,
#endif
#ifdef CONFIG_TEGRA_THERMAL_THROTTLE
	.temp_throttle = 85000,
	.tc1 = 2,
	.tc2 = 1,
	.passive_delay = 500,
#endif
#ifdef CONFIG_TEGRA_SKIN_THROTTLE
	.skin_device_id = THERMAL_DEVICE_ID_SKIN,
	.temp_throttle_skin = 43000,
#endif
};


//leds-lp5521
static struct led_i2c_config lp5521_led_config[] = {
	{
		.name = "amber",
                .led_cur = 95,
                .led_lux = 100
	},
	{
		.name = "green",
                .led_cur = 95,
                .led_lux = 100
	},
	{
		.name = "button-backlight",
                .led_cur = 2,
                .led_lux = 55,

	},
};
static struct led_i2c_platform_data led_data = {
	.num_leds	= ARRAY_SIZE(lp5521_led_config),
	.led_config	= lp5521_led_config,
	.ena_gpio = TEGRA_GPIO_PY0,
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
	int ret ;
	ret = gpio_request(TEGRA_GPIO_PY1, "led_trigger");
        if (ret < 0){
                pr_err("[LED] %s: gpio_request failed for gpio %s\n",
                        __func__, "LED TRIGGER");
	}
        ret = gpio_direction_output(TEGRA_GPIO_PY1, 0);
        if (ret < 0) {
                pr_err("[LED] %s: gpio_direction_output failed %d\n", __func__, ret);
                gpio_free(TEGRA_GPIO_PY1);
                return;
        }
	tegra_gpio_enable(TEGRA_GPIO_PY1);

	i2c_register_board_info(1, i2c_led_devices,
		ARRAY_SIZE(i2c_led_devices));
}

//flashlight
static void config_endeavor_flashlight_gpios(void)
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

static struct flashlight_platform_data endeavor_flashlight_data = {
	.gpio_init  = config_endeavor_flashlight_gpios,
	.torch = FL_TORCH_EN,
	.flash = FL_FLASH_EN,
	.flash_duration_ms = 600
};

static struct platform_device endeavor_flashlight_device = {
	.name = FLASHLIGHT_NAME,
	.dev		= {
		.platform_data	= &endeavor_flashlight_data,
	},
};

static void endeavor_flashlight_init(void)
{
	platform_device_register(&endeavor_flashlight_device);
}

//vibrator
#ifdef CONFIG_TEGRA_VIBRATOR_ENR
static struct vibrator_platform_data vibrator_data = {
	.pwm_data={
		.name = "vibrator",
		.bank = 0,
	},
	.pwm_gpio = TEGRA_GPIO_PH0,
	.ena_gpio = TEGRA_GPIO_PF1,
	.duty     = 38000,
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


static struct platform_device endeavoru_rfkill = {
	.name = "endeavoru_rfkill",
	.id = -1,
};


/* TI 128x Bluetooth begin */
static unsigned long retry_suspend;

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
extern void blue_pincfg_uartc_suspend(void);
extern void blue_pincfg_uartc_resume(void);
extern void blue_pincfg_uartc_gpio_request(void);

/* Release the wakelock when chip is asleep */
static int plat_chip_asleep(struct kim_data_s * data)
{
	pr_info("plat_chip_asleep\n");
	wake_unlock(&st_wk_lock);
	return 1;
}

/* Aquire the wakelock when chip is awake */
static int plat_chip_awake(struct kim_data_s * data)
{

	pr_info("plat_chip_awake\n");
	wake_lock(&st_wk_lock);
	return 1;
}

static int plat_chip_disable(struct kim_data_s * data)
{
	pr_info("plat_chip_disable\n");
	blue_pincfg_uartc_suspend();
	wake_unlock(&st_wk_lock);
	return 0;
}

static int plat_chip_enable(struct kim_data_s * data)
{
	pr_info("plat_chip_enable\n");
	wake_lock(&st_wk_lock);
	blue_pincfg_uartc_resume();
	return 0;
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
		.chip_enable = plat_chip_enable,
		.chip_disable = plat_chip_disable,
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

static noinline void __init endeavor_bt_wl128x(void)
{
	pr_info("%s\n", __func__);
	wake_lock_init(&st_wk_lock, WAKE_LOCK_SUSPEND, "st_wake_lock");

        platform_device_register(&wl128x_device);
	platform_device_register(&btwilink_device);
        tegra_gpio_enable(TEGRA_GPIO_PU0);
	blue_pincfg_uartc_gpio_request();

        return;
}
/* TI 128x Bluetooth end */

static __initdata struct tegra_clk_init_table endeavoru_clk_init_table[] = {
	/* name		parent		rate		enabled */
	{ "pll_m",	NULL,		0,		false},
	{ "hda",	"pll_p",	108000000,	false},
	{ "hda2codec_2x","pll_p",	48000000,	false},
	{ "pwm",	"clk_32k",	32768,		false},
	{ "blink",	"clk_32k",	32768,		true},
	{ "i2s1",	"pll_a_out0",	0,		false},
	{ "audio1",	"i2s1_sync",	0,		false},
	{ "spdif_out",	"pll_a_out0",	0,		false},
	{ "d_audio",	"clk_m",	12000000,	false},
	{ "dam0",	"clk_m",	12000000,	false},
	{ "dam1",	"clk_m",	12000000,	false},
	{ "dam2",	"clk_m",	12000000,	false},
	{ "vi",		"pll_p",	0,		false},
	{ "vi_sensor",	"pll_p",	0,		false},
	{ NULL,		NULL,		0,		0},
};

static struct tegra_i2c_platform_data endeavoru_i2c1_platform_data = {
	.adapter_nr	= 0,
	.bus_count	= 1,
	.bus_clk_rate	= { 384000, 0 },
	.scl_gpio		= {TEGRA_GPIO_PC4, 0},
	.sda_gpio		= {TEGRA_GPIO_PC5, 0},
	.arb_recovery = arb_lost_recovery,
};

static struct tegra_i2c_platform_data endeavoru_i2c2_platform_data = {
	.adapter_nr	= 1,
	.bus_count	= 1,
	.bus_clk_rate	= { 384000, 0 },
	.is_clkon_always = true,
	.scl_gpio		= {TEGRA_GPIO_PT5, 0},
	.sda_gpio		= {TEGRA_GPIO_PT6, 0},
	.arb_recovery = arb_lost_recovery,
};

static struct tegra_i2c_platform_data endeavoru_i2c3_platform_data = {
	.adapter_nr	= 2,
	.bus_count	= 1,
	.bus_clk_rate	= { 384000, 0 },
	.scl_gpio		= {TEGRA_GPIO_PBB1, 0},
	.sda_gpio		= {TEGRA_GPIO_PBB2, 0},
	.arb_recovery = arb_lost_recovery,
};

static struct tegra_i2c_platform_data endeavoru_i2c4_platform_data = {
	.adapter_nr	= 3,
	.bus_count	= 1,
	.bus_clk_rate	= { 10000, 0 },
	.scl_gpio		= {TEGRA_GPIO_PV4, 0},
	.sda_gpio		= {TEGRA_GPIO_PV5, 0},
	.arb_recovery = arb_lost_recovery,
};

static struct tegra_i2c_platform_data endeavoru_i2c5_platform_data = {
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

static struct htc_asoc_platform_data endeavoru_audio_pdata;

static struct platform_device endeavoru_audio_device = {
	.name	= "tegra-snd-aic3008",
	.id	= 0,
	.dev	= {
		.platform_data  = &endeavoru_audio_pdata,
	},
};

#if (defined(CONFIG_SND_AMP_TFA9887))
static struct tfa9887_platform_data endeavoru_speaker_amp_data = {
	.gpio_tfa9887_spk_en = -1,	// not used
};

static struct i2c_board_info i2c_speaker_amp_devices[] = {
	{
		I2C_BOARD_INFO("tfa9887", 0x68 >> 1),
		.platform_data = &endeavoru_speaker_amp_data,
		.irq = -1,
	},
};
#endif
// --- Audio tlv320aic3008 ---

/*HTC rawchip SPI4 ++ */
static struct tegra_spi_device_controller_data dev_cdata_rawchip = {
       .is_hw_based_cs = true, /* bool is_hw_based_cs */
       .cs_setup_clk_count = 1, /* int cs_setup_clk_count */
       .cs_hold_clk_count = 2, /* int cs_hold_clk_count */
};
static struct spi_board_info endeavoru_spi_board_info_rawchip[] __initdata = {
       {
               .modalias       = "spi_rawchip",
               .mode           = SPI_MODE_0,
               .bus_num        = 3,
               .chip_select    = 1,
               .max_speed_hz   = 25000000,
               .controller_data = &dev_cdata_rawchip,
       },
};
EXPORT_SYMBOL_GPL(endeavoru_spi_board_info_rawchip);

static int eva_use_ext_1v2(void)
{
	return 1;
}

static int eva_rawchip_vreg_on(void)
{
	int ret;

	pr_info("[CAM] rawchip power on ++\n");

	/* enable main clock */
	struct clk *csus_clk = NULL;
	struct clk *sensor_clk = NULL;
	csus_clk = clk_get(NULL, "csus");
	if (IS_ERR_OR_NULL(csus_clk)) {
		pr_err("%s: couldn't get csus clock\n", __func__);
		csus_clk = NULL;
		return -1;
	}
	sensor_clk = clk_get(NULL, "vi_sensor");
	if (IS_ERR_OR_NULL(sensor_clk)) {
		pr_err("%s: couldn't get sensor clock\n", __func__);
		sensor_clk = NULL;
		return -1;
	}
	clk_enable(csus_clk);
	clk_enable(sensor_clk);
	clk_set_rate(sensor_clk, 24000000);  /* 24MHz */

	/* rawchip power on sequence */
	tegra_gpio_disable(MCAM_SPI_CLK);
	tegra_gpio_disable(MCAM_SPI_CS0);
	tegra_gpio_disable(MCAM_SPI_DI);
	tegra_gpio_disable(MCAM_SPI_DO);
	tegra_gpio_disable(CAM_MCLK);

	gpio_direction_output(RAW_RSTN, 0);

	/* RAW_1V8_EN */
	gpio_direction_output(RAW_1V8_EN, 1);
	msleep(1);

	/* core */
	gpio_direction_output(CAM_D1V2_EN, 1);
	msleep(1);

	/* CAM SEL */
	gpio_direction_output(CAM_SEL, 0);
	msleep(1);

	/* RAW_RSTN */
	ret = gpio_direction_output(RAW_RSTN, 1);
	msleep(3);

	/* SPI send command to configure RAWCHIP here! */
	yushan_spi_write(0x0008, 0x7f);
	msleep(1);

	return 0;
}

static int eva_rawchip_vreg_off(void)
{
	pr_info("[CAM] rawchip power off ++\n");

	/* disable main clock */
	struct clk *csus_clk = NULL;
	struct clk *sensor_clk = NULL;
	csus_clk = clk_get(NULL, "csus");
	if (IS_ERR_OR_NULL(csus_clk)) {
		pr_err("%s: couldn't get csus clock\n", __func__);
		csus_clk = NULL;
		return -1;
	}
	sensor_clk = clk_get(NULL, "vi_sensor");
	if (IS_ERR_OR_NULL(sensor_clk)) {
		pr_err("%s: couldn't get sensor clock\n", __func__);
		sensor_clk = NULL;
		return -1;
	}
	clk_disable(csus_clk);
	clk_disable(sensor_clk);

	/* rawchip power off sequence */
	/* RAW RSTN */
	gpio_direction_output(RAW_RSTN, 0);
	msleep(3);

	/* digital */
	gpio_direction_output(CAM_D1V2_EN, 0);
	msleep(1);

	/* RAW_1V8_EN */
	gpio_direction_output(RAW_1V8_EN, 0);
	msleep(1);

	/* set gpio output low : O(L) */
	tegra_gpio_enable(MCAM_SPI_CLK);
	tegra_gpio_enable(MCAM_SPI_CS0);
	tegra_gpio_enable(MCAM_SPI_DI);
	tegra_gpio_enable(MCAM_SPI_DO);
	tegra_gpio_enable(CAM_MCLK);

	return 0;
}

static struct tegra_camera_rawchip_info tegra_rawchip_board_info = {
	.rawchip_reset  = RAW_RSTN,
	.rawchip_intr0  = TEGRA_GPIO_PR0,
	.rawchip_intr1  = TEGRA_GPIO_PEE1,
	.rawchip_spi_freq = 25,
	.rawchip_mclk_freq = 24,
	.camera_rawchip_power_on = eva_rawchip_vreg_on,
	.camera_rawchip_power_off = eva_rawchip_vreg_off,
	.rawchip_use_ext_1v2 = eva_use_ext_1v2,
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

/* HTC_HEADSET_PMIC Driver */
static struct htc_headset_pmic_platform_data htc_headset_pmic_data_xe = {
	.driver_flag	= DRIVER_HS_PMIC_ADC,
	.adc_mic_bias	= {HS_DEF_MIC_ADC_16_BIT_MIN,
				   HS_DEF_MIC_ADC_16_BIT_MAX},
	.adc_channel	= 4,
	.adc_remote	= {0, 164, 165, 379, 380, 830},
};

static struct platform_device htc_headset_pmic_xe = {
	.name	= "HTC_HEADSET_PMIC",
	.id	= -1,
	.dev	= {
		.platform_data	= &htc_headset_pmic_data_xe,
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
		.type = HEADSET_MIC,
		.adc_max = 1499,
		.adc_min = 1150,
	},
	{
		.type = HEADSET_NO_MIC,
		.adc_max = 1149,
		.adc_min = 0,
	},
};

static struct htc_headset_1wire_platform_data htc_headset_1wire_data = {
	.tx_level_shift_en	= TEGRA_GPIO_PZ0,
	.uart_sw		= 0,
	.one_wire_remote	={0x7E, 0x7F, 0x7D, 0x7F, 0x7B, 0x7F},
	.remote_press		= TEGRA_GPIO_PBB6,
	.onewire_tty_dev	= "/dev/ttyHS4",
};

static struct platform_device htc_headset_one_wire = {
	.name	= "HTC_HEADSET_1WIRE",
	.id	= -1,
	.dev	= {
		.platform_data	= &htc_headset_1wire_data,
	},
};

/* HTC_HEADSET_MGR Driver */
static struct platform_device *headset_devices_xe[] = {
//	&htc_headset_microp,
	&htc_headset_pmic_xe,
	&htc_headset_one_wire,
	&htc_headset_gpio,
//	&htc_headset_misc,
	/* Please put the headset detection driver on the last */
};

static void headset_power(int hs_enable)
{
	pr_info("[HS_BOARD]hs_enable = %d\n", hs_enable);
	aic3008_set_mic_bias(hs_enable);
}

static void headset_init(void)
{
	tegra_gpio_disable(TEGRA_GPIO_PY4);
	tegra_gpio_disable(TEGRA_GPIO_PY5);
}

static void uart_tx_gpo(int mode)
{
	switch (mode) {
		case 0:
			tegra_gpio_enable(TEGRA_GPIO_PY4);
			gpio_direction_output(TEGRA_GPIO_PY4, 0);
			break;
		case 1:
			tegra_gpio_enable(TEGRA_GPIO_PY4);
			gpio_direction_output(TEGRA_GPIO_PY4, 1);
			break;
		case 2:
			tegra_gpio_disable(TEGRA_GPIO_PY4);
			break;
	}
}

static void uart_lv_shift_en(int enable)
{
	gpio_direction_output(TEGRA_GPIO_PZ0, enable);
}

static struct htc_headset_mgr_platform_data htc_headset_mgr_data_xe = {
	.driver_flag		= DRIVER_HS_MGR_FLOAT_DET,
	.headset_devices_num	= ARRAY_SIZE(headset_devices_xe),
	.headset_devices	= headset_devices_xe,
	.headset_config_num	= ARRAY_SIZE(htc_headset_mgr_config_xe),
	.headset_config		= htc_headset_mgr_config_xe,
	.headset_init		= headset_init,
	.headset_power		= headset_power,
	.uart_tx_gpo		= uart_tx_gpo,
	.uart_lv_shift_en	= uart_lv_shift_en,
};

static struct platform_device htc_headset_mgr_xe = {
	.name	= "HTC_HEADSET_MGR",
	.id	= -1,
	.dev	= {
		.platform_data	= &htc_headset_mgr_data_xe,
	},
};

static void endeavoru_i2c_init(void)
{
	tegra_i2c_device1.dev.platform_data = &endeavoru_i2c1_platform_data;
	tegra_i2c_device2.dev.platform_data = &endeavoru_i2c2_platform_data;
	tegra_i2c_device3.dev.platform_data = &endeavoru_i2c3_platform_data;
	tegra_i2c_device4.dev.platform_data = &endeavoru_i2c4_platform_data;
	tegra_i2c_device5.dev.platform_data = &endeavoru_i2c5_platform_data;

	platform_device_register(&tegra_i2c_device5);
	platform_device_register(&tegra_i2c_device4);
	platform_device_register(&tegra_i2c_device3);
	platform_device_register(&tegra_i2c_device2);
	platform_device_register(&tegra_i2c_device1);

#if (defined(CONFIG_SND_AMP_TFA9887))
	i2c_register_board_info(0, i2c_speaker_amp_devices,
		ARRAY_SIZE(i2c_speaker_amp_devices));
#endif
}

static struct platform_device *endeavoru_uart_devices[] __initdata = {
	&tegra_uarta_device,
	&tegra_uartb_device,
	&tegra_uartc_device,
	&tegra_uartd_device,
	&tegra_uarte_device,
};

static struct uart_clk_parent uart_parent_clk[] = {
	[0] = {.name = "clk_m"},
	[1] = {.name = "pll_p"},
#ifndef CONFIG_TEGRA_PLLM_RESTRICTED
	[2] = {.name = "pll_m"},
#endif
};
static struct tegra_uart_platform_data endeavoru_uart_pdata;
static struct tegra_uart_platform_data endeavoru_loopback_uart_pdata;

#ifdef CONFIG_SERIAL_TEGRA_BRCM
static struct tegra_uart_platform_data endeavoru_brcm_uart_pdata;
#endif

static struct tegra_uart_platform_data endeavor_uart_pdata;

#ifdef CONFIG_BT_CTS_WAKEUP
static struct tegra_uart_platform_data endeavor_bt_uart_pdata;
#endif

static void __init uart_debug_init(void)
{
	unsigned long rate;
	struct clk *c;

	/* UARTA is the debug port. */
	pr_info("Selecting UARTA as the debug console\n");
	endeavoru_uart_devices[0] = &debug_uarta_device;
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

static void __init endeavoru_uart_init(void)
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
	endeavoru_uart_pdata.parent_clk_list = uart_parent_clk;
	endeavoru_uart_pdata.parent_clk_count = ARRAY_SIZE(uart_parent_clk);

/* GPS use UARTE
	endeavoru_loopback_uart_pdata.parent_clk_list = uart_parent_clk;
	endeavoru_loopback_uart_pdata.parent_clk_count =
						ARRAY_SIZE(uart_parent_clk);
	endeavoru_loopback_uart_pdata.is_loopback = true;
*/

	tegra_uarta_device.dev.platform_data = &endeavoru_uart_pdata;
	tegra_uartb_device.dev.platform_data = &endeavoru_uart_pdata;
	tegra_uartc_device.dev.platform_data = &endeavoru_uart_pdata;
	tegra_uartd_device.dev.platform_data = &endeavoru_uart_pdata;
	tegra_uarte_device.dev.platform_data = &endeavoru_uart_pdata; // Used for BRCM GPS.

#ifdef CONFIG_BT_CTS_WAKEUP
	int board_id = htc_get_pcbid_info();

	endeavor_bt_uart_pdata = endeavoru_uart_pdata;
	if ((machine_is_endeavoru() && (board_id >= PROJECT_PHASE_XC)))
	{
		endeavor_bt_uart_pdata.uart_bt = (1 == 1); /* true */
	}
	else {
		endeavor_bt_uart_pdata.uart_bt = (1 == 0); /* false */
	}
	endeavor_bt_uart_pdata.bt_en = BT_GPIO_EN;
	endeavor_bt_uart_pdata.bt_cts_irq = BT_GPIO_CTS_IRQ;
	tegra_uartc_device.dev.platform_data = &endeavor_bt_uart_pdata;
#endif

#ifdef CONFIG_SERIAL_TEGRA_BRCM
	endeavoru_brcm_uart_pdata = endeavoru_uart_pdata;
	endeavoru_brcm_uart_pdata.bt_wakeup_pin_supported = 1;
	endeavoru_brcm_uart_pdata.bt_wakeup_pin = ENDEAVORU_GPIO_BT_WAKE;
	endeavoru_brcm_uart_pdata.host_wakeup_pin = ENDEAVORU_GPIO_BT_HOST_WAKE;
	tegra_uartc_device.dev.platform_data = &endeavoru_brcm_uart_pdata;
	tegra_uartc_device.name = "tegra_uart_brcm"; /* for brcm */
#endif

/* GPS uses UARTE
	// UARTE is used for loopback test purpose
	tegra_uarte_device.dev.platform_data = &endeavoru_loopback_uart_pdata;
*/

	/* Register low speed only if it is selected */
	if (!is_tegra_debug_uartport_hs())
		uart_debug_init();

	platform_add_devices(endeavoru_uart_devices,
				ARRAY_SIZE(endeavoru_uart_devices));
}

static struct spi_clk_parent spi_parent_clk[] = {
	[0] = {.name = "pll_p"},
#ifndef CONFIG_TEGRA_PLLM_RESTRICTED
	[1] = {.name = "pll_m"},
	[2] = {.name = "clk_m"},
#else
	[1] = {.name = "clk_m"},
#endif
};

static struct tegra_spi_platform_data endeavoru_spi_pdata = {
	.is_dma_based		= true,
	.max_dma_buffer		= (16 * 1024),
	.is_clkon_always	= false,
	.max_rate		= 100000000,
	.disable_runtime_pm	= true,
};

static void __init endeavoru_spi_init(void)
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
	endeavoru_spi_pdata.parent_clk_list = spi_parent_clk;
	endeavoru_spi_pdata.parent_clk_count = ARRAY_SIZE(spi_parent_clk);

	spi_register_board_info(spi_board_info_audio, ARRAY_SIZE(spi_board_info_audio));
	spi_register_board_info(endeavoru_spi_board_info_rawchip, ARRAY_SIZE(endeavoru_spi_board_info_rawchip));
	platform_device_register(&tegra_spi_device2);
	tegra_spi_device4.dev.platform_data = &endeavoru_spi_pdata;
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

static struct platform_device *endeavoru_devices[] __initdata = {
	&tegra_pmu_device,
	&tegra_rtc_device,
	&tegra_udc_device,
#if defined(CONFIG_TEGRA_IOVMM_SMMU) || defined(CONFIG_TEGRA_IOMMU_SMMU)
	&tegra_smmu_device,
#endif
	&tegra_wdt0_device,
	&tegra_wdt1_device,
	&tegra_wdt2_device,
#if defined(CONFIG_TEGRA_AVP)
	&tegra_avp_device,
#endif
	&tegra_camera,
	//&tegra_spi_device4, spi_init function
	&tegra_rawchip_device,
	&tegra_hda_device,
#if defined(CONFIG_CRYPTO_DEV_TEGRA_SE)
	&tegra_se_device,
#endif
#if defined(CONFIG_CRYPTO_DEV_TEGRA_AES)
	&tegra_aes_device,
#endif
};

#define MXT_CONFIG_CRC 0x62F903
/*
 * Config converted from memory-mapped cfg-file with
 * following version information:
 *
 *
 *
 *      FAMILY_ID=128
 *      VARIANT=1
 *      VERSION=32
 *      BUILD=170
 *      VENDOR_ID=255
 *      PRODUCT_ID=TBD
 *      CHECKSUM=0xC189B6
 *
 *
 */

static const u8 config[] = {
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0xFF, 0xFF, 0x32, 0x0A, 0x00, 0x05, 0x01, 0x00,
        0x00, 0x1E, 0x0A, 0x8B, 0x00, 0x00, 0x13, 0x0B,
        0x00, 0x10, 0x32, 0x03, 0x03, 0x00, 0x03, 0x01,
        0x00, 0x0A, 0x0A, 0x0A, 0x0A, 0xBF, 0x03, 0x1B,
        0x02, 0x00, 0x00, 0x37, 0x37, 0x00, 0x00, 0x00,
        0x00, 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0xA9, 0x7F, 0x9A, 0x0E, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x03, 0x23, 0x00, 0x00, 0x00, 0x0A,
        0x0F, 0x14, 0x19, 0x03, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x01, 0x00, 0x03, 0x08, 0x10,
        0x00
};

static struct mxt_platform_data atmel_mxt_info = {
        .x_line         = 19,
        .y_line         = 11,
        .x_size         = 960,
        .y_size         = 540,
        .blen           = 0x10,
        .threshold      = 0x32,
        .voltage        = 3300000,              /* 3.3V */
        .orient         = 3,
        .config         = config,
        .config_length  = 168,
        .config_crc     = MXT_CONFIG_CRC,
        .irqflags       = IRQF_TRIGGER_FALLING,
/*      .read_chg       = &read_chg, */
        .read_chg       = NULL,
};

static struct i2c_board_info __initdata atmel_i2c_info[] = {
	{
		I2C_BOARD_INFO("atmel_mxt_ts", MXT224_I2C_ADDR1),
		.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PH6),
		.platform_data = &atmel_mxt_info,
	}
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

/* Touchscreen GPIO addresses   */
#define TOUCH_GPIO_IRQ TEGRA_GPIO_PV1
#define TOUCH_GPIO_RST TEGRA_GPIO_PF3
#define TOUCH_GPIO_PWD TEGRA_GPIO_PY2

static struct regulator *srio_1v8_en;

static int powerfun(int enable)
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

static struct synaptics_i2c_rmi_platform_data edge_ts_3k_data_XB[] = {
	{
		.version = 0x3332, /* fw32 */
		.packrat_number = 1293981,
		.abs_x_min = 0,
		.abs_x_max = 1100,
		.abs_y_min = 0,
		.abs_y_max = 1770,
		.display_width = 720,
		.display_height = 1280,
		.notifyFinger = restoreCap, /* restore browser cap, */
		.gpio_irq = TOUCH_GPIO_IRQ,
		.power = powerfun,
		.report_type = SYN_AND_REPORT_TYPE_B,
		.reduce_report_level = {60, 60, 50, 0, 0},
		.default_config = 2,
		.config = {
			0x35, 0x4A, 0x31, 0x32, 0x80, 0x7F, 0x03, 0x14,
			0x14, 0x08, 0x00, 0x19, 0x19, 0x00, 0x10, 0x4C,
			0x04, 0x6C, 0x07, 0x02, 0x14, 0x1E, 0x05, 0x41,
			0xF2, 0x27, 0x8B, 0x02, 0x01, 0x3C, 0x0C, 0x03,
			0x10, 0x03, 0xEC, 0x4D, 0x71, 0x51, 0x04, 0xBF,
			0xD4, 0xC6, 0x00, 0xC8, 0x00, 0x00, 0x00, 0x00,
			0x0A, 0x04, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x19, 0x01, 0x00, 0x0A, 0x16, 0x0C, 0x0A,
			0x00, 0x14, 0x0A, 0x40, 0x64, 0x07, 0xF3, 0xC8,
			0xBE, 0x43, 0x2A, 0x05, 0x00, 0x00, 0x00, 0x00,
			0x4C, 0x6C, 0x74, 0x3C, 0x32, 0x00, 0x00, 0x00,
			0x4C, 0x6C, 0x74, 0x1E, 0x05, 0x00, 0x02, 0xFA,
			0x00, 0x80, 0x03, 0x0E, 0x1F, 0x11, 0x38, 0x00,
			0x13, 0x04, 0x1B, 0x00, 0x10, 0x0A, 0xC0, 0xC0,
			0xC0, 0xC0, 0xA8, 0xA0, 0xA8, 0xA0, 0x4B, 0x4A,
			0x48, 0x47, 0x45, 0x44, 0x42, 0x40, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x88,
			0x13, 0x00, 0x64, 0x00, 0xC8, 0x00, 0x80, 0x0A,
			0x80, 0xB8, 0x0B, 0x00, 0xC0, 0x19, 0x02, 0x02,
			0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x20, 0x20,
			0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x58, 0x5B,
			0x5D, 0x5F, 0x61, 0x63, 0x66, 0x69, 0x00, 0x8C,
			0x00, 0x10, 0x28, 0x00, 0x00, 0x00, 0x02, 0x04,
			0x06, 0x08, 0x0A, 0x0D, 0x0E, 0x04, 0x31, 0x04,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF,
			0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
			0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x51, 0x51, 0x51,
			0x51, 0x51, 0x51, 0x51, 0x51, 0xCD, 0x0D, 0x04,
			0x00, 0x06, 0x0C, 0x0D, 0x0B, 0x15, 0x17, 0x16,
			0x18, 0x19, 0x1A, 0x1B, 0x11, 0x14, 0x12, 0x0F,
			0x0E, 0x09, 0x0A, 0x07, 0x02, 0x01, 0x00, 0xFF,
			0xFF, 0xFF, 0xFF, 0xFF, 0x04, 0x05, 0x02, 0x06,
			0x01, 0x0C, 0x07, 0x08, 0x0E, 0x10, 0x0F, 0x12,
			0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x10, 0x00, 0x10,
			0x00, 0x10, 0x00, 0x10, 0x80, 0x80, 0x80, 0x80,
			0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80,
			0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80,
			0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80,
			0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80,
			0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x00,
			0x0F, 0x00, 0x4F, 0x53
		},
	},
	{
		.version = 0x3330, /* fw30 */
		.packrat_number = 1100755,
		.abs_x_min = 0,
		.abs_x_max = 1100,
		.abs_y_min = 0,
		.abs_y_max = 1770,
		.notifyFinger = restoreCap, /* restore browser cap, */
		.gpio_irq = TOUCH_GPIO_IRQ,
		.power = powerfun,
		.default_config = 2,
		.large_obj_check = 1,
		.tw_pin_mask = 0x0080,
		.sensor_id = 0x0080 | SENSOR_ID_CHECKING_EN, /* YFO */
		.report_type = SYN_AND_REPORT_TYPE_B,
		.segmentation_bef_unlock = 0x50,
		.customer_register = {0xF9,0x64,0x74,0x32},
		.multitouch_calibration = 1,
		.config = {
			0x59,0x31,0x32,0x30,0x00,0x3F,0x03,0x1E,0x05,0xB1,
			0x08,0x0B,0x19,0x19,0x00,0x00,0x4C,0x04,0x75,0x07,
			0x02,0x14,0x1E,0x05,0x37,0xA5,0x16,0xE8,0x03,0x01,
			0x3C,0x17,0x02,0x17,0x01,0xEC,0x4D,0x71,0x51,0x04,
			0xBF,0xD4,0xC6,0x00,0x50,0x00,0x00,0x00,0x00,0x0A,
			0x04,0xC0,0x00,0x00,0x00,0x00,0x00,0x00,0x19,0x01,
			0x00,0x0A,0x70,0x32,0xA2,0x02,0x28,0x0A,0x0A,0x64,
			0x16,0x0C,0x00,0x02,0xEE,0x00,0x80,0x03,0x0E,0x1F,
			0x11,0x38,0x00,0x13,0x08,0x1B,0x00,0x08,0xFF,0x00,
			0x06,0x0C,0x0D,0x0B,0x15,0x17,0x16,0x18,0x19,0x1A,
			0x1B,0x11,0x14,0x12,0x0F,0x0E,0x09,0x0A,0x07,0x02,
			0x01,0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0x04,0x05,0x02,
			0x06,0x01,0x0C,0x07,0x08,0x0E,0x10,0x0F,0x12,0xFF,
			0xFF,0xFF,0xFF,0xA0,0xA0,0xA0,0xA0,0xA0,0x80,0x80,
			0x80,0x42,0x41,0x40,0x3F,0x3D,0x3C,0x3A,0x39,0x00,
			0x02,0x04,0x06,0x08,0x0A,0x0D,0x10,0x00,0xA0,0x0F,
			0xFF,0x28,0x00,0xC8,0x00,0xB3,0xC8,0xCD,0xA0,0x0F,
			0x00,0xC0,0x80,0x00,0x10,0x00,0x10,0x00,0x10,0x00,
			0x10,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
			0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
			0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
			0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
			0x80,0x80,0x80,0x80,0x02,0x02,0x02,0x02,0x02,0x02,
			0x02,0x02,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,
			0x58,0x5B,0x5D,0x5F,0x61,0x63,0x66,0x69,0x00,0x64,
			0x00,0x10,0x0A,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
			0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0xFF,0xFF,
			0xFF,0xFF,0xFF,0xFF,0x51,0x51,0x51,0x51,0xCD,0x0D,
			0x04
		}
	},
	{
		.version = 0x3330, /* fw30 */
		.packrat_number = 1100755,
		.abs_x_min = 0,
		.abs_x_max = 1100,
		.abs_y_min = 0,
		.abs_y_max = 1770,
		.notifyFinger = restoreCap, /* restore browser cap, */
		.gpio_irq = TOUCH_GPIO_IRQ,
		.power = powerfun,
		.default_config = 2,
		.large_obj_check = 1,
		.tw_pin_mask = 0x0080,
		.sensor_id = 0x0000 | SENSOR_ID_CHECKING_EN, /* J-Touch */
		.report_type = SYN_AND_REPORT_TYPE_B,
		.segmentation_bef_unlock = 0x50,
		.customer_register = {0xF9,0x64,0x74,0x32},
		.multitouch_calibration = 1,
		.config = {
			0x4A,0x31,0x32,0x30,0x00,0x3F,0x03,0x1E,0x05,0xB1,
			0x08,0x0B,0x19,0x19,0x00,0x00,0x4C,0x04,0x75,0x07,
			0x02,0x14,0x1E,0x05,0x3A,0x57,0x1F,0xAF,0x02,0x01,
			0x3C,0x17,0x02,0x17,0x01,0xEC,0x4D,0x71,0x51,0x04,
			0xBF,0xD4,0xC6,0x00,0x50,0x13,0x00,0x00,0x00,0x0A,
			0x04,0xC0,0x00,0x00,0x00,0x00,0x00,0x00,0x19,0x01,
			0x00,0x0A,0x90,0x32,0xA2,0x02,0x28,0x0A,0x0A,0x64,
			0x16,0x0C,0x00,0x02,0xF0,0x00,0x80,0x03,0x0E,0x1F,
			0x11,0x38,0x00,0x13,0x08,0x1B,0x00,0x08,0x00,0x00,
			0x06,0x0C,0x0D,0x0B,0x15,0x17,0x16,0x18,0x19,0x1A,
			0x1B,0x11,0x14,0x12,0x0F,0x0E,0x09,0x0A,0x07,0x02,
			0x01,0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0x04,0x05,0x02,
			0x06,0x01,0x0C,0x07,0x08,0x0E,0x10,0x0F,0x12,0xFF,
			0xFF,0xFF,0xFF,0xA0,0xA0,0xA0,0xA0,0xA0,0x80,0x80,
			0x80,0x42,0x41,0x40,0x3F,0x3D,0x3C,0x3A,0x39,0x00,
			0x02,0x04,0x06,0x08,0x0A,0x0D,0x10,0x00,0xB8,0x0B,
			0xCD,0x28,0x00,0xC8,0x00,0x80,0xC8,0xCD,0xB8,0x0B,
			0x00,0xC0,0x80,0x00,0x10,0x00,0x10,0x00,0x10,0x00,
			0x10,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
			0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
			0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
			0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
			0x80,0x80,0x80,0x80,0x02,0x02,0x02,0x02,0x02,0x02,
			0x02,0x02,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,
			0x58,0x5B,0x5D,0x5F,0x61,0x63,0x66,0x69,0x00,0x64,
			0x00,0x10,0x0A,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
			0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0xFF,0xFF,
			0xFF,0xFF,0xFF,0xFF,0x51,0x51,0x51,0x51,0xCD,0x0D,
			0x04
		}
	},
	{
		.version = 0x3230,  /* fw20 */
		.abs_x_min = 0,
		.abs_x_max = 1100,
		.abs_y_min = 0,
		.abs_y_max = 1770,
		.flags = SYNAPTICS_FLIP_Y,
		.gpio_irq = TOUCH_GPIO_IRQ,
		.default_config = 1,
		.power = powerfun,
		.tw_pin_mask = 0x0080,
		.sensor_id = 0x0080 | SENSOR_ID_CHECKING_EN, /* YFO */
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
		.version = 0x3230, /* fw20 */
		.abs_x_min = 0,
		.abs_x_max = 1100,
		.abs_y_min = 0,
		.abs_y_max = 1770,
		.gpio_irq = TOUCH_GPIO_IRQ,
		.power = powerfun,
		.tw_pin_mask = 0x0080,
		.sensor_id = 0x0000 | SENSOR_ID_CHECKING_EN, /* wintek */
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
	{
		.version = 0x0000
	},
};

static struct i2c_board_info __initdata synaptics_i2c_info_XB[] = {
	{
		I2C_BOARD_INFO(SYNAPTICS_3200_NAME, 0x20),
		.platform_data = &edge_ts_3k_data_XB,
		.irq = TEGRA_GPIO_TO_IRQ(TOUCH_GPIO_IRQ),
	},

};

static struct tegra_touchscreen_init __initdata synaptics_init_data_XB = {
	.irq_gpio = TOUCH_GPIO_IRQ,                    /* GPIO1 Value for IRQ */
	.rst_gpio = TOUCH_GPIO_RST,                    /* GPIO2 Value for RST */
	.sv_gpio1 = {1, TOUCH_GPIO_RST, 0, 1},
	.sv_gpio2 = {1, TOUCH_GPIO_RST, 1, 100},       /* Valid, GPIOx, Set value, Delay      */
	.ts_boardinfo = {1, synaptics_i2c_info_XB, 1}      /* BusNum, BoardInfo, Value     */
};

static int __init generic_touch_init(struct tegra_touchscreen_init *tsdata)
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

static void nv_touch_mfgcheck(void)
{
	int i;
	if (board_mfg_mode() == BOARD_MFG_MODE_FACTORY2 ) {/*MFG mode */
		for (i = 0; i < ARRAY_SIZE(edge_ts_3k_data_XB);  i++)
			edge_ts_3k_data_XB[i].mfg_flag = 1;
	} else if (board_mfg_mode() != BOARD_MFG_MODE_NORMAL ) /*not normal mode */
		pr_info("[TP]board_mfg_mode() == %d",board_mfg_mode());

	pr_info("[TP]Enter nv_touch_mfgcheck\n");
	if ( ARRAY_SIZE(edge_ts_3k_data_XB)>0 ) {
		pr_info("[TP]edge_ts_3k_data_XB[0].mfg_flag = %d\n",edge_ts_3k_data_XB[0].mfg_flag);
	}
}

static int __init endeavoru_touch_init(void)
{
	int retval = 0;
	struct board_info BoardInfo;
	pr_info("[TP]Enter endeavoru_touch_init\n");
	tegra_get_board_info(&BoardInfo);
	nv_touch_mfgcheck();
	retval = generic_touch_init(&synaptics_init_data_XB);
	return retval;
}

static int endeavoru_usb_hsic_postsupend(void)
{
	pr_debug("%s\n", __func__);
#ifdef CONFIG_TEGRA_BB_XMM_POWER
	baseband_xmm_set_power_status(BBXMM_PS_L2);
#endif
	return 0;
}

static int endeavoru_usb_hsic_preresume(void)
{
	pr_debug("%s\n", __func__);
#ifdef CONFIG_TEGRA_BB_XMM_POWER
	baseband_xmm_set_power_status(BBXMM_PS_L2TOL0);
#endif
	return 0;
}

static int endeavoru_usb_hsic_phy_ready(void)
{
	pr_debug("%s\n", __func__);
#ifdef CONFIG_TEGRA_BB_XMM_POWER
	baseband_xmm_set_power_status(BBXMM_PS_L0);
#endif
	return 0;
}

static int endeavoru_usb_hsic_phy_off(void)
{
	pr_debug("%s\n", __func__);
#ifdef CONFIG_TEGRA_BB_XMM_POWER
#ifdef CONFIG_REMOVE_HSIC_L3_STATE
	baseband_xmm_set_power_status(BBXMM_PS_L2);
#else
	baseband_xmm_set_power_status(BBXMM_PS_L3);
#endif
#endif
	return 0;
}

static struct tegra_usb_phy_platform_ops hsic_xmm_plat_ops = {
	.post_suspend = endeavoru_usb_hsic_postsupend,
	.pre_resume = endeavoru_usb_hsic_preresume,
	.port_power = endeavoru_usb_hsic_phy_ready,
	.post_phy_off = endeavoru_usb_hsic_phy_off,
};

static struct tegra_usb_platform_data tegra_ehci2_hsic_xmm_pdata = {
	.port_otg = false,
	.has_hostpc = true,
	.phy_intf = TEGRA_USB_PHY_INTF_HSIC,
	.op_mode	= TEGRA_USB_OPMODE_HOST,
	.u_data.host = {
		.vbus_gpio = -1,
		.hot_plug = false,
		.remote_wakeup_supported = false,
		.power_off_on_suspend = true,
	},
	.u_cfg.hsic = {
		.sync_start_delay = 9,
		.idle_wait_delay = 17,
		.term_range_adj = 0,
		.elastic_underrun_limit = 16,
		.elastic_overrun_limit = 16,
	},
	.ops = &hsic_xmm_plat_ops,
};



static struct tegra_usb_platform_data tegra_udc_pdata = {
	.port_otg = true,
	.has_hostpc = true,
	.phy_intf = TEGRA_USB_PHY_INTF_UTMI,
	.op_mode = TEGRA_USB_OPMODE_DEVICE,
	.u_data.dev = {
		.vbus_pmu_irq = ENT_TPS80031_IRQ_BASE +
				TPS80031_INT_VBUS_DET,
		.vbus_gpio = -1,
		.charging_supported = false,
		.remote_wakeup_supported = false,
	},
	.u_cfg.utmi = {
		.hssync_start_delay = 0,
		.elastic_limit = 16,
		.idle_wait_delay = 17,
		.term_range_adj = 7,
		.xcvr_setup = 14,
		.xcvr_lsfslew = 2,
		.xcvr_lsrslew = 2,
		.xcvr_setup_offset = 0,
		.xcvr_use_fuses = 1,
	},
};

static struct tegra_usb_platform_data tegra_ehci1_utmi_pdata = {
	.port_otg = true,
	.has_hostpc = true,
	.builtin_host_disabled = true,
	.phy_intf = TEGRA_USB_PHY_INTF_UTMI,
	.op_mode = TEGRA_USB_OPMODE_HOST,
	.u_data.host = {
		.vbus_gpio = -1,
		.vbus_reg = "usb_vbus",
		.hot_plug = true,
		.remote_wakeup_supported = true,
		.power_off_on_suspend = true,
	},
	.u_cfg.utmi = {
		.hssync_start_delay = 0,
		.elastic_limit = 16,
		.idle_wait_delay = 17,
		.term_range_adj = 6,
		.xcvr_setup = 15,
		.xcvr_lsfslew = 2,
		.xcvr_lsrslew = 2,
		.xcvr_setup_offset = 0,
		.xcvr_use_fuses = 1,
	},
};

static struct tegra_usb_otg_data tegra_otg_pdata = {
	.ehci_device = &tegra_ehci1_device,
	.ehci_pdata = &tegra_ehci1_utmi_pdata,
};

static struct platform_device *tegra_usb_hsic_host_register(void)
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

	val = platform_device_add_data(pdev, &tegra_ehci2_hsic_xmm_pdata,
			sizeof(struct tegra_usb_platform_data));
	if (val)
		goto error;

	val = platform_device_add(pdev);
	if (val)
		goto error;

	return pdev;

error:
	pr_err("%s: failed to add the host contoller device\n", __func__);
	platform_device_put(pdev);
	return NULL;
}

static void tegra_usb_hsic_host_unregister(struct platform_device *pdev)
{
	platform_device_unregister(pdev);
}

static struct android_usb_platform_data android_usb_pdata = {
	.vendor_id	= 0x0BB4,
#ifdef CONFIG_SENSE_4_PLUS
	.product_id	= 0x0dfb,
#else
	.product_id	= 0x0cd6,
#endif
	.version	= 0x0100,
	.product_name		= "Android Phone",
	.manufacturer_name	= "HTC",
	.num_products = ARRAY_SIZE(usb_products),
	.products = usb_products,
	.num_functions = ARRAY_SIZE(usb_functions_all),
	.functions = usb_functions_all,
	.fserial_init_string = "tty,tty:autobot,tty:serial,tty:autobot",
	.usb_id_pin_gpio = TEGRA_GPIO_PS2,
	.RndisDisableMPDecision = true,
	.nluns = 1,
	.support_modem = false,
};

static struct platform_device android_usb_device = {
	.name	= "android_usb",
	.id		= -1,
	.dev		= {
		.platform_data = &android_usb_pdata,
	},
};

static void endeavoru_usb_init(void)
{
	tegra_udc_device.dev.platform_data = &tegra_udc_pdata;

	tegra_otg_device.dev.platform_data = &tegra_otg_pdata;
	platform_device_register(&tegra_otg_device);

	android_usb_pdata.serial_number = board_serialno();
	android_usb_pdata.products[0].product_id = android_usb_pdata.product_id;

#if 0
	if (board_mfg_mode() == BOARD_MFG_MODE_NORMAL /* normal mode */) {
		android_usb_pdata.cdrom_lun = 0x1;
	}
#endif

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
	/* TEGRA_GPIO_PS2:USB_ID */
	if (output) {
		if (gpio_direction_output(TEGRA_GPIO_PS2, 1) < 0) {
			pr_info("[CABLE] %s: TEGRA_GPIO_USB_ID dir NG\n", __func__);
			return;
		}
		tegra_gpio_enable(TEGRA_GPIO_PS2);
	}
	else {
		if (gpio_direction_input(TEGRA_GPIO_PS2) < 0) {
			pr_info("[CABLE] %s: TEGRA_GPIO_USB_ID dir setup failed\n", __func__);
			return;
		}
		tegra_gpio_enable(TEGRA_GPIO_PS2);
	}
}

static void tegra_usb_dpdn_switch(int path)
{
	int polarity = 1; /* high = mhl */
	int mhl = (path == PATH_MHL);

	switch (path) {
		case PATH_USB:
		case PATH_MHL:
			pr_info("[CABLE] %s: Set %s path\n", __func__, mhl ? "MHL" : "USB");
			/* TEGRA_GPIO_PE0: MHL_USB_SEL */
			gpio_set_value(TEGRA_GPIO_PE0, (mhl ^ !polarity) ? 1 : 0);
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
	.usb_id_pin_gpio	= TEGRA_GPIO_PS2,
	.mhl_reset_gpio		= TEGRA_GPIO_PE6,
	.usb_dpdn_switch	= tegra_usb_dpdn_switch,
	/*.mpp_data = {
		.usbid_mpp	= XOADC_MPP_4,
		.usbid_amux	= PM_MPP_AIN_AMUX_CH5,
	},*/
	.config_usb_id_gpios	= config_tegra_usb_id_gpios,
	.get_adc_cb		= get_tegra_adc_cb,
	.cable_gpio_init	= cable_tegra_gpio_init,
#ifdef CONFIG_TEGRA_HDMI_MHL
	/*.mhl_1v2_power = mhl_sii9234_1v2_power,*/
	.mhl_internal_3v3 	= 1,
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
	int ret;
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
/*
	ret = gpio_request(TEGRA_GPIO_PB3, "USB_HOST");
	if (ret < 0) {
		pr_err("%s: gpio_request failed %d\n", __func__, ret);
		return;
	}

	if (gpio_direction_output(TEGRA_GPIO_PB3, 0) < 0) {
		pr_info("[CABLE] %s: TEGRA_GPIO_PB3(USB_HOST) dir NG\n", __func__);
		return;
	}

	tegra_gpio_enable(TEGRA_GPIO_PB3);
*/
}

static void endeavoru_cable_detect_init(void)
{
	platform_device_register(&cable_detect_device);
}


static struct platform_device *endeavoru_audio_devices[] __initdata = {
	&tegra_ahub_device,
	&tegra_dam_device0,
	&tegra_dam_device1,
	&tegra_dam_device2,
	&tegra_i2s_device1,
	&tegra_spdif_device,
	&spdif_dit_device,
	&tegra_pcm_device,
	&endeavoru_audio_device,
};

static void endeavoru_audio_init(void)
{
	endeavoru_audio_codec_init(&endeavoru_audio_pdata);

	platform_add_devices(endeavoru_audio_devices,
			ARRAY_SIZE(endeavoru_audio_devices));
}

static void endeavoru_gps_init(void)
{
	int rc;

	tegra_gpio_enable(TEGRA_GPIO_PBB7);	// GPS_RESET_N gpio223
	tegra_gpio_enable(TEGRA_GPIO_PP5);	// GPS_ON_OFF gpio125

	rc = gpio_request(TEGRA_GPIO_PP5, "GPS_ON_OFF");
	if (rc)
		pr_err("GPS_ON_OFF gpio request failed:%d\n", rc);
	rc = gpio_direction_output(TEGRA_GPIO_PP5, 0);
	if (rc)
		pr_err("GPS_ON_OFF gpio direction configuration failed:%d\n", rc);
	gpio_export(TEGRA_GPIO_PP5, false);

	rc = gpio_request(TEGRA_GPIO_PBB7, "GPS_RESET_N");
	if (rc)
		pr_err("GPS_RESET_N gpio request failed:%d\n", rc);
	rc = gpio_direction_output(TEGRA_GPIO_PBB7, 1);
	if (rc)
		pr_err("GPS_RESET_N gpio direction configuration failed:%d\n", rc);
	gpio_export(TEGRA_GPIO_PBB7, false);
}

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
			.bb_vdd_en = BB_VDD_EN,
			.bb_rst_pwrdn = AP2BB_RST_PWRDWNn,
			.bb_rst2 = BB2AP_RST2,
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
    .name   = "htc_simhotswap",
    .id = -1,
};

#ifdef CONFIG_TEGRA_BB_M7400
static union tegra_bb_gpio_id m7400_gpio_id = {
	.m7400 = {
		.pwr_status = GPIO_BB_RESET,
		.pwr_on = GPIO_BB_PWRON,
		.uart_awr = GPIO_BB_APACK,
		.uart_cwr = GPIO_BB_CPACK,
		.usb_awr = GPIO_BB_APACK2,
		.usb_cwr = GPIO_BB_CPACK2,
		.service = GPIO_BB_RSVD2,
		.resout2 = GPIO_BB_RSVD1,
	},
};

static struct tegra_bb_pdata m7400_pdata = {
	.id = &m7400_gpio_id,
	.device = &tegra_ehci2_device,
	.ehci_register = tegra_usb_hsic_host_register,
	.ehci_unregister = tegra_usb_hsic_host_unregister,
	.bb_id = TEGRA_BB_M7400,
};

static struct platform_device tegra_baseband_m7400_device = {
	.name = "tegra_baseband_power",
	.id = -1,
	.dev = {
		.platform_data = &m7400_pdata,
	},
};
#endif

static void endeavoru_modem_init(void)
{
        struct board_info board_info;

        pr_info("%s: enable baseband gpio(s)\n", __func__);
        /* enable baseband gpio(s) */
        tegra_gpio_enable(BB_VDD_EN);
        tegra_gpio_enable(AP2BB_RST_PWRDWNn);
        tegra_gpio_enable(AP2BB_RSTn);
        tegra_gpio_enable(AP2BB_PWRON);
        tegra_gpio_enable(BB2AP_RADIO_FATAL);
        tegra_gpio_enable(tegra_baseband_power_data.modem.xmm.bb_rst);
        tegra_gpio_enable(tegra_baseband_power_data.modem.xmm.bb_on);
        tegra_gpio_enable(tegra_baseband_power_data.modem.xmm.ipc_bb_wake);
        tegra_gpio_enable(tegra_baseband_power_data.modem.xmm.ipc_ap_wake);
        tegra_gpio_enable(tegra_baseband_power_data.modem.xmm.ipc_hsic_active);
        tegra_gpio_enable(tegra_baseband_power_data.modem.xmm.ipc_hsic_sus_req);
        /* HTC: Seshendra patch 1117 */
        tegra_baseband_power_data.hsic_register = &tegra_usb_hsic_host_register;
        tegra_baseband_power_data.hsic_unregister = &tegra_usb_hsic_host_unregister;

        platform_device_register(&tegra_baseband_power_device);
        platform_device_register(&tegra_baseband_power2_device);
        platform_device_register(&simhotswap_device);

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

static void endeavoru_baseband_init(void)
{
        endeavoru_modem_init();

//        uhsic_phy_config.postsuspend = endeavoru_usb_hsic_postsupend;
//        uhsic_phy_config.preresume = endeavoru_usb_hsic_preresume;
//        uhsic_phy_config.usb_phy_ready = endeavoru_usb_hsic_phy_ready;
//        uhsic_phy_config.post_phy_off = endeavoru_usb_hsic_phy_off;
}


static struct keyreset_platform_data enr_reset_keys_pdata = {
	.keys_down = {
		KEY_POWER,
		KEY_VOLUMEDOWN,
		KEY_VOLUMEUP,
		0
	},
	.swResetCheck = NULL,
};

static struct platform_device enr_reset_keys_device = {
	.name = KEYRESET_NAME,
	.dev.platform_data = &enr_reset_keys_pdata,
};

// for ENRC2 project bring up
#define GPIO_KEY(_id, _gpio, _iswake)		\
	{					\
		.code = _id,			\
		.gpio = TEGRA_GPIO_##_gpio,	\
		.active_low = 1,		\
		.desc = #_id,			\
		.type = EV_KEY,			\
		.wakeup = _iswake,		\
		.debounce_interval = 20,	\
	}

#define BOOT_DEBUG_LOG_ENTER(fn) \
	printk(KERN_NOTICE "[BOOT_LOG] Entering %s\n", fn);
#define BOOT_DEBUG_LOG_LEAVE(fn) \
	printk(KERN_NOTICE "[BOOT_LOG] Leaving %s\n", fn);

static int enrkey_wakeup() {
	if ( is_resume_from_deep_suspend() ) {
		unsigned long status =
			readl(IO_ADDRESS(TEGRA_PMC_BASE) + PMC_WAKE_STATUS);
		return status & TEGRA_WAKE_GPIO_PU6 ? KEY_POWER : KEY_RESERVED;
	} else
		return KEY_RESERVED;
}

static struct gpio_keys_button ENDEAVORU_PROJECT_keys[] = {
	[0] = GPIO_KEY(KEY_POWER, PU6, 1),
	[1] = GPIO_KEY(KEY_VOLUMEUP, PS0, 0),
	[2] = GPIO_KEY(KEY_VOLUMEDOWN, PW3, 0),
 };

static struct gpio_keys_platform_data ENDEAVORU_PROJECT_keys_platform_data = {
	.buttons	= ENDEAVORU_PROJECT_keys,
	.nbuttons	= ARRAY_SIZE(ENDEAVORU_PROJECT_keys),
	.wakeup_key     = enrkey_wakeup,
 };

static struct platform_device ENDEAVORU_PROJECT_keys_device = {
	.name   = "gpio-keys",
	.id     = 0,
	.dev    = {
		.platform_data  = &ENDEAVORU_PROJECT_keys_platform_data,
	},
};

int __init ENDEAVORU_PROJECT_keys_init(void)
{
	int i;
	pr_info("[KEY]Enter ENDEAVORU_PROJECT_keys_init\n");
	for (i = 0; i < ARRAY_SIZE(ENDEAVORU_PROJECT_keys); i++)
		tegra_gpio_enable(ENDEAVORU_PROJECT_keys[i].gpio);

	platform_device_register(&ENDEAVORU_PROJECT_keys_device);

	return 0;
}

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

static void __init endeavoru_init(void)
{
	struct kobject *properties_kobj;
	int board_id = 0;
	tegra_thermal_init(&thermal_data,
				throttle_list,
				ARRAY_SIZE(throttle_list));
	tegra_clk_init_from_table(endeavoru_clk_init_table);
	endeavoru_pinmux_init();
	endeavoru_i2c_init();
	endeavoru_spi_init();
	endeavoru_uart_init();
	endeavoru_usb_init();
	bt_export_bd_address();
	platform_add_devices(endeavoru_devices, ARRAY_SIZE(endeavoru_devices));
	platform_device_register(&htc_headset_mgr_xe);
	board_id = htc_get_pcbid_info();
	if (machine_is_endeavoru()) {
		if ((board_id > PROJECT_PHASE_A) ||
				(board_id == PROJECT_PHASE_A && engineer_id != 1)) {
			mhl_sii_device_data.ci2ca = 1;
			mhl_sii_device_data.enMhlD3Guard = true;
		}
	}

#if defined(CONFIG_HTC_FIQ_DUMPER)
	fiq_dumper_init();
#endif
#if defined(CONFIG_MEMORY_FOOTPRINT_DEBUGGING)
	htc_memory_footprint_init();
#endif
	tegra_ram_console_debug_init();
	endeavoru_regulator_init();
	ENDEAVORU_PROJECT_keys_init();
	endeavoru_sdhci_init();
#ifdef CONFIG_TEGRA_EDP_LIMITS
	endeavoru_edp_init();
#endif
#ifdef CONFIG_TEGRA_HDMI_MHL
	i2c_register_board_info(4, i2c_mhl_sii_info,
			ARRAY_SIZE(i2c_mhl_sii_info));
#endif
	endeavoru_touch_init();
	endeavor_bt_wl128x();
	if (platform_device_register(&enr_reset_keys_device))
		printk(KERN_WARNING "[KEY]%s: register reset key fail\n", __func__);
        properties_kobj = kobject_create_and_add("board_properties", NULL);
	if (properties_kobj) {
		sysfs_create_group(properties_kobj, &Aproj_properties_attr_group_XC);
	}
	endeavoru_audio_init();
	//endeavoru_gps_init();
	endeavoru_baseband_init();
	endeavor_panel_init();
	endeavoru_emc_init();
	endeavoru_sensors_init();
	endeavoru_cam_init();
	endeavoru_suspend_init();
	tegra_release_bootloader_fb();
	tegra_vibrator_init();
	leds_lp5521_init();
	endeavor_flashlight_init();
#if defined(CONFIG_CABLE_DETECT_ACCESSORY)
	endeavoru_cable_detect_init();
#endif
	//endeavoru_nfc_init();
	
	struct proc_dir_entry* proc;
	proc = create_proc_read_entry("dying_processes", 0, NULL, dying_processors_read_proc, NULL);
	if (!proc)
		printk(KERN_ERR"Create /proc/dying_processes FAILED!\n");

	if (board_mfg_mode() != BOARD_MFG_MODE_OFFMODE_CHARGING)
#if ! defined(CONFIG_HTC_PERFORMANCE_MONITOR_ALWAYS_ON)
		if (get_kernel_flag() & KERNEL_FLAG_PM_MONITOR)
#endif
		{
			htc_monitor_init();
			htc_pm_monitor_init();
		}
}

static void __init endeavoru_reserve(void)
{
#if defined(CONFIG_NVMAP_CONVERT_CARVEOUT_TO_IOVMM)
	tegra_reserve(0, SZ_4M, 0);
#else
	tegra_reserve(SZ_128M, SZ_4M, SZ_8M);
#endif
	tegra_ram_console_debug_reserve(SZ_1M);
#if defined(CONFIG_MEMORY_FOOTPRINT_DEBUGGING)
	htc_memory_footprint_space_reserve(SZ_4K);
#endif
#if defined(CONFIG_HTC_FIQ_DUMPER)
	fiq_dumper_space_reserve(SZ_4K);
#endif
}

MACHINE_START(ENDEAVORU, "endeavoru")
	.boot_params    = 0x80000100,
	.map_io         = tegra_map_common_io,
	.reserve        = endeavoru_reserve,
	.init_early	= tegra_init_early,
	.init_irq       = tegra_init_irq,
	.timer          = &tegra_timer,
	.init_machine   = endeavoru_init,
MACHINE_END
