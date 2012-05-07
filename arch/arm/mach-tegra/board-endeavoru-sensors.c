/*
 * arch/arm/mach-tegra/board-endeavoru-sensors.c
 *
 * Copyright (c) 2011, NVIDIA CORPORATION, All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * Neither the name of NVIDIA CORPORATION nor the names of its contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/i2c/pca954x.h>
#include <linux/err.h>
#include <linux/mpu_htc.h>
#include <linux/nct1008.h>
#include <linux/err.h>
#include <linux/mpu.h>
#include <linux/platform_data/ina230.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/tps65200.h>
#include <mach/htc_battery_tps80032.h>
#include <linux/pn544.h>
#include <mach/gpio.h>
#include <media/ar0832_main.h>
#include <media/s5k3h2y.h>
#include <media/s5k6a1gx03.h>
#include <media/ad5823.h>
#include <media/tps61050.h>
#include <media/ov9726.h>
#include <mach/edp.h>
#include <mach/thermal.h>
#include "cpu-tegra.h"
#include "gpio-names.h"
#include "board-endeavoru.h"
#include "board.h"
#include <linux/akm8975.h>
#include <linux/bma250.h>
#include <linux/ewtzmu2.h>
#include <mach/board_htc.h>
#include <linux/isl29028.h>
#include <linux/cm3628.h>
#include <linux/cm3629.h>

#define RAWCHIP 1
#include <media/rawchip/Yushan_API.h>
//#define CAMERA_REGULATOR

static struct regulator *v_sdmmc_2v85_en ;
static struct regulator *v_srio_1v8_en ;
void cm3629_enable_power(int enable)
{
	if(htc_get_pcbid_info() >= PROJECT_PHASE_XD) {
		if(enable == 1) {
			if (v_sdmmc_2v85_en == NULL) {
		  		v_sdmmc_2v85_en = regulator_get(NULL, "v_sdmmc_2v85");
		  		if (WARN_ON(IS_ERR(v_sdmmc_2v85_en))) {
		   			pr_err("[v_sdmmc_2v85] %s: couldn't get regulator v_sdmmc_2v85_en: %ld\n", __func__, PTR_ERR(v_sdmmc_2v85_en));
				}
			}
		 	regulator_enable(v_sdmmc_2v85_en);

			if (v_srio_1v8_en == NULL) {
		  		v_srio_1v8_en = regulator_get(NULL, "v_srio_1v8");
		  		if (WARN_ON(IS_ERR(v_srio_1v8_en))) {
		   			pr_err("[v_srio_1v8] %s: couldn't get regulator v_srio_1v8_en: %ld\n", __func__, PTR_ERR(v_srio_1v8_en));
				}
			}
		 	regulator_enable(v_srio_1v8_en);
		}else if(enable == 0) {
			if(regulator_is_enabled(v_srio_1v8_en)) {
				regulator_disable(v_srio_1v8_en);
			}
			if(regulator_is_enabled(v_sdmmc_2v85_en)) {
				regulator_disable(v_sdmmc_2v85_en);
			}	
		}
	}else {
		if(enable == 1) {
		 	if (v_srio_1v8_en == NULL) {
		  		v_srio_1v8_en = regulator_get(NULL, "v_srio_1v8");
		  		if (WARN_ON(IS_ERR(v_srio_1v8_en))) {
		   			pr_err("[v_srio_1v8] %s: couldn't get regulator v_srio_1v8_en: %ld\n", __func__, PTR_ERR(v_srio_1v8_en));
				}
			}
		 	regulator_enable(v_srio_1v8_en);
		}else if(enable == 0) {
			if(regulator_is_enabled(v_srio_1v8_en)) {
				regulator_disable(v_srio_1v8_en);
			}	
		}
	}
}

/*void cm3629_enable_power(int enable)
{
	int ret = 0;
	int gpio = TEGRA_GPIO_PM3;

	if(htc_get_pcbid_info() >= PROJECT_PHASE_XD) {
		ret = gpio_request(gpio, "PLSensor_EN");
		if (ret < 0) {
			pr_err("[PS][cm3629] Requesting GPIO %d failes\n", gpio);
			return;
		}
		ret = gpio_direction_output(gpio, enable);
		if (ret < 0) {
			pr_err("[PS][cm3629] Requesting GPIO %d failes\n", gpio);
			gpio_free(gpio);
			return;
		}
		tegra_gpio_enable(gpio);
	}
}*/

static struct cm3628_platform_data cm3628_pdata = {
	/*.intr = PSNENOR_INTz,*/
	.pwr = NULL,
	.intr = TEGRA_GPIO_PK2,
	.levels = { 12, 14, 16, 41, 83, 3561, 6082, 6625, 7168, 65535},
	.golden_adc = 0x1145,
	.power = NULL,
	.ALS_slave_address = 0xC0>>1,
	.PS_slave_address = 0xC2>>1,
	.check_interrupt_add = 0x2C>>1	,
	.is_cmd = CM3628_ALS_IT_400ms | CM3628_ALS_PERS_2,
	.ps_thd_set = 0x4,
	.ps_conf2_val = 0,
	.ps_calibration_rule = 1,
	.ps_reset_thd = 1,
	.ps_conf1_val = CM3628_PS_DR_1_80 | CM3628_PS_IT_2T |
			CM3628_PS_PERS_4,
	.ps_thd_no_cal = 0x10,
	.ps_thd_with_cal = 0x4,
};

static struct cm3629_platform_data cm3629_pdata = {
	.model = CAPELLA_CM36282,
	.ps_select = CM3629_PS1_ONLY,
	.intr = TEGRA_GPIO_PK2,
	.levels = { 12, 14, 16, 176, 361, 4169, 6891, 9662, 12433, 65535},
	.golden_adc = 0x13EF,
	.power = cm3629_enable_power,
	.cm3629_slave_address = 0xC0>>1,
	.ps_calibration_rule = 1,
	.ps1_thd_set = 0x3,
	.ps1_thd_no_cal = 0x3,
	.ps1_thd_with_cal = 0x3,
	.ps_conf1_val = CM3629_PS_DR_1_80 | CM3629_PS_IT_2T |
			CM3629_PS1_PERS_4,
	.ps_conf2_val = CM3629_PS_ITB_1 | CM3629_PS_ITR_1 |
			CM3629_PS2_INT_DIS | CM3629_PS1_INT_DIS,
	.ps_conf3_val = CM3629_PS2_PROL_32,
};

static struct i2c_board_info i2c_CM3628_devices[] = {
	{
		I2C_BOARD_INFO("cm3628", 0xc0 >> 1),
		.platform_data = &cm3628_pdata,
		.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PK2),
	}
};

static struct i2c_board_info i2c_CM3629_devices[] = {
	{
		I2C_BOARD_INFO(CM3629_I2C_NAME, 0xC0 >> 1),
		.platform_data = &cm3629_pdata,
			.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PK2),
	},
};

static void psensor_init(void)
{
#if 0
		i2c_register_board_info(0,
				i2c_CM3628_devices, ARRAY_SIZE(i2c_CM3628_devices));
		pr_info("[PS][cm3628]%s\n", __func__);
#endif
#if 1
	if(ps_type) {
		i2c_register_board_info(0,
				i2c_CM3629_devices, ARRAY_SIZE(i2c_CM3629_devices));
		pr_info("[PS][cm3629]%s ps_type = %d\n", __func__, ps_type);
	}
	else {
		i2c_register_board_info(0,
				i2c_CM3628_devices, ARRAY_SIZE(i2c_CM3628_devices));
		pr_info("[PS][cm3628]%s ps_type = %d\n", __func__, ps_type);
	}
#endif
}
static struct regulator *cam_vcm_2v85_en = NULL;
static struct regulator *cam_vddio_1v8_en = NULL;
static struct regulator *cam_a2v85_en = NULL;
static struct regulator *cam_d1v2_en = NULL;
static struct regulator *cam2_d1v2_en = NULL;

#ifndef CONFIG_TEGRA_INTERNAL_TSENSOR_EDP_SUPPORT
static int nct_get_temp(void *_data, long *temp)
{
	struct nct1008_data *data = _data;
	return nct1008_thermal_get_temp(data, temp);
}

static int nct_get_temp_low(void *_data, long *temp)
{
	struct nct1008_data *data = _data;
	return nct1008_thermal_get_temp_low(data, temp);
}

static int nct_set_limits(void *_data,
			long lo_limit_milli,
			long hi_limit_milli)
{
	struct nct1008_data *data = _data;
	return nct1008_thermal_set_limits(data,
					lo_limit_milli,
					hi_limit_milli);
}

static int nct_set_alert(void *_data,
				void (*alert_func)(void *),
				void *alert_data)
{
	struct nct1008_data *data = _data;
	return nct1008_thermal_set_alert(data, alert_func, alert_data);
}

static int nct_set_shutdown_temp(void *_data, long shutdown_temp)
{
	struct nct1008_data *data = _data;
	return nct1008_thermal_set_shutdown_temp(data,
						shutdown_temp);
}

static void nct1008_probe_callback(struct nct1008_data *data)
{
	struct tegra_thermal_device *thermal_device;

	thermal_device = kzalloc(sizeof(struct tegra_thermal_device),
					GFP_KERNEL);
	if (!thermal_device) {
		pr_err("unable to allocate thermal device\n");
		return;
	}

	thermal_device->name = "nct1008";
	thermal_device->data = data;
	thermal_device->offset = TDIODE_OFFSET;
	thermal_device->get_temp = nct_get_temp;
	thermal_device->get_temp_low = nct_get_temp_low;
	thermal_device->set_limits = nct_set_limits;
	thermal_device->set_alert = nct_set_alert;
	thermal_device->set_shutdown_temp = nct_set_shutdown_temp;

	tegra_thermal_set_device(thermal_device);
}
#endif

static struct nct1008_platform_data enterprise_nct1008_pdata = {
	.supported_hwrev = true,
	.ext_range = true,
	.conv_rate = 0x08,
	.offset = 8, /* 4 * 2C. Bug 844025 - 1C for device accuracies */
#ifndef CONFIG_TEGRA_INTERNAL_TSENSOR_EDP_SUPPORT
	.probe_callback = nct1008_probe_callback,
#endif
	.reg_name = "v_usb_3v3",
};

static struct i2c_board_info enterprise_i2c4_nct1008_board_info[] = {
	{
		I2C_BOARD_INFO("nct1008", 0x4C),
		//.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PH7),
		.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PCC2),
		.platform_data = &enterprise_nct1008_pdata,
	}
};

static void enterprise_nct1008_init(void)
{
	int ret;

#if 0
	tegra_gpio_enable(TEGRA_GPIO_PH7);
	ret = gpio_request(TEGRA_GPIO_PH7, "temp_alert");
	if (ret < 0) {
		pr_err("%s: gpio_request failed %d\n", __func__, ret);
		return;
	}

	ret = gpio_direction_input(TEGRA_GPIO_PH7);
	if (ret < 0) {
		pr_err("%s: gpio_direction_input failed %d\n", __func__, ret);
		gpio_free(TEGRA_GPIO_PH7);
		return;
	}
#endif

tegra_gpio_enable(TEGRA_GPIO_PCC2);
	ret = gpio_request(TEGRA_GPIO_PCC2, "temp_alert");
	if (ret < 0) {
		pr_err("%s: gpio_request failed %d\n", __func__, ret);
		return;
	}

	ret = gpio_direction_input(TEGRA_GPIO_PCC2);
	if (ret < 0) {
		pr_err("%s: gpio_direction_input failed %d\n", __func__, ret);
		gpio_free(TEGRA_GPIO_PCC2);
		return;
	}

	i2c_register_board_info(4, enterprise_i2c4_nct1008_board_info,
				ARRAY_SIZE(enterprise_i2c4_nct1008_board_info));
}
void config_ruby_gyro_diag_gpios(bool pulldown)
{
/*
	if (pulldown) {
		config_gpio_table(gyro_DIAG_PIN_pull_down, ARRAY_SIZE(gyro_DIAG_PIN_pull_down));
		printk(KERN_INFO "%s %d pull down\n",  __func__, RUBY_GPIO_GYRO_DIAG);
	} else {
		config_gpio_table(gyro_DIAG_PIN_no_pull, ARRAY_SIZE(gyro_DIAG_PIN_no_pull));
		printk(KERN_INFO "%s %d input none pull\n",  __func__, RUBY_GPIO_GYRO_DIAG);
	}
*/
}

static struct pana_gyro_platform_data pana_gyro_pdata = {
	.acc_dir = 0x06,
	.acc_polarity = 0x07,
	.gyro_dir = 0x06,
	.gyro_polarity = 0x02,
	.mag_dir = 0x06,
	.mag_polarity = 0x07,
	.sleep_pin = TEGRA_GPIO_PR2,//RUBY_GPIO_PANA_GYRO_SLEEP,
	.config_gyro_diag_gpios = config_ruby_gyro_diag_gpios,
};




static struct i2c_board_info __initdata pana_gyro_GSBI12_boardinfo[] = {
	{
		I2C_BOARD_INFO("ewtzmu2", 0x69),
		.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PI6),
		.platform_data = &pana_gyro_pdata,
	},
};

static struct bma250_platform_data gsensor_bma250_platform_data = {
	.intr = TEGRA_GPIO_PO5,//RUBY_GPIO_GSENSOR_INT_N,
	.chip_layout = 1,
};
static struct i2c_board_info i2c_bma250_devices[] = {
	{
		I2C_BOARD_INFO(BMA250_I2C_NAME, 0x19),
		.platform_data = &gsensor_bma250_platform_data,
		.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PO5),
	},
};

static struct akm8975_platform_data compass_platform_data_xb = {
	.layouts = RUBY_LAYOUTS_XB,
	.use_pana_gyro = 1,
};

static struct i2c_board_info i2c_akm8975_devices_xb[] = {
	{
		I2C_BOARD_INFO(AKM8975_I2C_NAME, 0x0D),
		.platform_data = &compass_platform_data_xb,
		.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PJ2),
	},
};

static struct akm8975_platform_data compass_platform_data_xc = {
	.layouts = RUBY_LAYOUTS_XC,
	.use_pana_gyro = 1,
};

static struct i2c_board_info i2c_akm8975_devices_xc[] = {
	{
		I2C_BOARD_INFO(AKM8975_I2C_NAME, 0x0D),
		.platform_data = &compass_platform_data_xc,
		.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PJ2),
	},
};

#define SENSOR_MPU_NAME "mpu3050"
static struct mpu3050_platform_data mpu3050_data = {
	.int_config  = 0x10,
	/* Orientation matrix for MPU on enterprise */
	#if defined(CONFIG_BOARD_EVT)
	#warning "EVT board"
	  .en_1v8 = 1,
	  .orientation = { 1, 0, 0, 0, -1, 0, 0, 0, -1 }, // EVT
	#else
	  .en_1v8 = 1,
	  .orientation = { -1, 0, 0, 0, -1, 0, 0, 0, 1 }, // EVM
	#endif
	.level_shifter = 0,

	.accel = {
		.get_slave_descr = get_accel_slave_descr,
		.adapt_num   = 0,
		.bus         = EXT_SLAVE_BUS_SECONDARY,
		//.address     = 0x0F,
		.address     = 0x19, //for A-project
		/* Orientation matrix for Kionix on enterprise */
		#if defined(CONFIG_BOARD_EVT)
		#warning "EVT board"
		  .orientation = { -1, 0, 0, 0, 1, 0, 0, 0,-1 }, // EVT .orientation = { -1, 0, 0, 0, 1, 0, 0, 0,-1 }, // EVT
		#else
		  .orientation = { 1, 0, 0, 0, 1, 0, 0, 0, 1 }, // EVM
		#endif

	},

	.compass = {
		.get_slave_descr = get_compass_slave_descr,
		.adapt_num   = 0,
		.bus         = EXT_SLAVE_BUS_PRIMARY,
		//.address     = 0x0C,
		.address     = 0x0D,//for A-project
		/* Orientation matrix for AKM on enterprise */
		#if defined(CONFIG_BOARD_EVT)
		#warning "EVT board"
		  .orientation = { 1, 0, 0, 0, 1, 0, 0, 0, 1 }, // EVT
		#else
		  .orientation = { -1, 0, 0, 0, 1, 0, 0, 0, -1 }, // EVM
		#endif
	},
};

static struct i2c_board_info __initdata mpu3050_i2c0_boardinfo[] = {
	{
		I2C_BOARD_INFO(SENSOR_MPU_NAME, 0x68),
		//.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PH4),
		.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PI6),
		.platform_data = &mpu3050_data,
	},
};

static void config_nfc_gpios(void)
{
    int ret = 0;

    ret = gpio_direction_output(RUBY_GPIO_NFC_VEN, 0);
    if (ret < 0) {
        pr_err("TEGRA_GPIO_PM5 output failed\n");
        pr_err("[NFC] %s: gpio_direction_output failed %d\n", __func__, ret);
        gpio_free(RUBY_GPIO_NFC_VEN);
        return;
    }
    tegra_gpio_enable(RUBY_GPIO_NFC_VEN);

    ret = gpio_direction_output(RUBY_GPIO_NFC_DL, 0);
    if (ret < 0) {
        pr_err("TEGRA_GPIO_PM6 output failed\n");
        pr_err("[NFC] %s: gpio_direction_output failed %d\n", __func__, ret);
        gpio_free(RUBY_GPIO_NFC_DL);
        return;
    }
    tegra_gpio_enable(RUBY_GPIO_NFC_DL);

    ret = gpio_direction_input(RUBY_GPIO_NFC_INT);
    if (ret < 0) {
        pr_err("TEGRA_GPIO_PY6 output failed\n");
        pr_err("[NFC] %s: gpio_direction_output failed %d\n", __func__, ret);
        gpio_free(RUBY_GPIO_NFC_INT);
        return;
    }
    tegra_gpio_enable(RUBY_GPIO_NFC_INT);

    gpio_set_value(RUBY_GPIO_NFC_VEN, 1);
    pr_info("%s\n", __func__);

}

static struct pn544_i2c_platform_data nfc_platform_data = {
    .gpio_init  = config_nfc_gpios,
    .irq_gpio = RUBY_GPIO_NFC_INT,
    .ven_gpio = RUBY_GPIO_NFC_VEN,
    .firm_gpio = RUBY_GPIO_NFC_DL,
    .ven_isinvert = 1,
};

static struct i2c_board_info pn544_i2c_boardinfo[] = {
    {
        I2C_BOARD_INFO(PN544_I2C_NAME, 0x50 >> 1),
        .platform_data = &nfc_platform_data,
        .irq = TEGRA_GPIO_TO_IRQ(RUBY_GPIO_NFC_INT),
    },
};

static void edge_nfc_init(void)
{
    i2c_register_board_info(0, pn544_i2c_boardinfo,
            ARRAY_SIZE(pn544_i2c_boardinfo));
}


static inline void ENR_msleep(u32 t)
{
	/*
	If timer value is between ( 10us - 20ms),
	ENR_usleep_range() is recommended.
	Please read Documentation/timers/timers-howto.txt.
	*/
	usleep_range(t*1000, t*1000 + 500);
}

static inline void ENR_usleep(u32 t)
{
        usleep_range(t, t + 500);
}

static void enterprise_gsensor_irq_init(void)
{
	int ret = 0;

	//tegra_gpio_enable(TEGRA_GPIO_PH4);
	//ret = gpio_request(TEGRA_GPIO_PH4, SENSOR_MPU_NAME);
	pr_info("[GSNR] g-sensor irq_start...\n");
	if(htc_get_pcbid_info() <= PROJECT_PHASE_XB){
		ret = gpio_request(TEGRA_GPIO_PO5, "GSNR_INT");
		if (ret < 0) {
			pr_err("%s: gpio_request failed %d\n", __func__, ret);
			return;
		}

		ret = gpio_direction_input(TEGRA_GPIO_PO5);
		if (ret < 0) {
			pr_err("%s: gpio_direction_input failed %d\n", __func__, ret);
			gpio_free(TEGRA_GPIO_PO5);
			return;
		}
		tegra_gpio_enable(TEGRA_GPIO_PO5);
		tegra_pinmux_set_pullupdown(TEGRA_PINGROUP_ULPI_DATA4, TEGRA_PUPD_NORMAL);
	}
	else{
		ret = gpio_request(TEGRA_GPIO_PN5, "GSNR_INT");
		if (ret < 0) {
			pr_err("%s: gpio_request failed %d\n", __func__, ret);
			return;
		}

		ret = gpio_direction_input(TEGRA_GPIO_PN5);
		if (ret < 0) {
			pr_err("%s: gpio_direction_input failed %d\n", __func__, ret);
			gpio_free(TEGRA_GPIO_PN5);
			return;
		}
		tegra_gpio_enable(TEGRA_GPIO_PN5);
		tegra_pinmux_set_pullupdown(TEGRA_PINGROUP_LCD_SDOUT, TEGRA_PUPD_NORMAL);

	}
		
	pr_info("[GSNR] g-sensor irq end...\n");

}

static void enterprise_gyro_diag_init(void)
{
	int ret = 0;


	pr_info("[GYRO] gyro diag_start...\n");
		ret = gpio_request(TEGRA_GPIO_PH3, "GYRO_DIAG");
		if (ret < 0) {
			pr_err("%s: gpio_request failed %d\n", __func__, ret);
			return;
		}

		ret = gpio_direction_input(TEGRA_GPIO_PH3);
		if (ret < 0) {
			pr_err("%s: gpio_direction_input failed %d\n", __func__, ret);
			gpio_free(TEGRA_GPIO_PH3);
			return;
		}
		tegra_gpio_enable(TEGRA_GPIO_PH3);
		tegra_pinmux_set_pullupdown(TEGRA_PINGROUP_GMI_AD11, TEGRA_PUPD_NORMAL);

		
	pr_info("[GYRO] gyro diag irq end...\n");

}

// Riemer 07-05-2012: Added __init
static void __init enterprise_mpuirq_init(void)
{
	int ret = 0;

	tegra_gpio_enable(TEGRA_GPIO_PI6);
	ret = gpio_request(TEGRA_GPIO_PI6, SENSOR_MPU_NAME);
	if (ret < 0) {
		pr_err("%s: gpio_request failed %d\n", __func__, ret);
		return;
	}

	ret = gpio_direction_input(TEGRA_GPIO_PI6);
	if (ret < 0) {
		pr_err("%s: gpio_direction_input failed %d\n", __func__, ret);
		gpio_free(TEGRA_GPIO_PI6);
		return;
	}
	tegra_pinmux_set_pullupdown(TEGRA_PINGROUP_GMI_CS7_N, TEGRA_PUPD_NORMAL);

	if(htc_get_pcbid_info() == PROJECT_PHASE_XA){
		i2c_register_board_info(0, mpu3050_i2c0_boardinfo,
					ARRAY_SIZE(mpu3050_i2c0_boardinfo));
	}
}
static void enterprise_gyro_sleep_pin(void)
{

	int ret = 0;
	ret = gpio_request(TEGRA_GPIO_PR2, "sleep_pin");
	pr_info("[GYRO] mog sleep pin...\n");
	if (ret < 0) {
	pr_err("TEGRA_GPIO_PR2 request failes\n");
			pr_err("%s: gpio_request failed %d\n", __func__, ret);
			return;
	}
	ret = gpio_direction_output(TEGRA_GPIO_PR2, 1);
	if (ret < 0) {
	pr_err("TEGRA_GPIOPR2, output failed\n");
			pr_err("[sleep_pin] %s: gpio_direction_output failed %d\n", __func__, ret);
			gpio_free(TEGRA_GPIO_PR2);
			return;
	}
	tegra_gpio_enable(TEGRA_GPIO_PR2);


}

static struct i2c_board_info enterprise_i2c0_isl_board_info[] = {
	{
		I2C_BOARD_INFO("isl29028", 0x44),
	}
};

static void enterprise_comp_irq_init(void)
{
	int ret = 0;

	// comp int
	ret = gpio_request(TEGRA_GPIO_PJ2, "COMP_INT");
	if (ret < 0) {
		pr_err("[COMP] %s: gpio_request failed %d\n", __func__, ret);
		return;
	}

	ret = gpio_direction_input(TEGRA_GPIO_PJ2);
	if (ret < 0) {
		pr_err("[COMP] %s: gpio_direction_input failed %d\n", __func__, ret);
		gpio_free(TEGRA_GPIO_PJ2);
		return;
	}

	tegra_gpio_enable(TEGRA_GPIO_PJ2);
	tegra_pinmux_set_pullupdown(TEGRA_PINGROUP_GMI_CS1_N, TEGRA_PUPD_NORMAL);
	gpio_free(TEGRA_GPIO_PJ2);
}

static void enterprise_isl_init(void)
{
	i2c_register_board_info(0, enterprise_i2c0_isl_board_info,
				ARRAY_SIZE(enterprise_i2c0_isl_board_info));
}

static int endeavor_s5k3h2y_power_state = 0;
static int endeavor_s5k3h2y_get_power(void)
{
	return endeavor_s5k3h2y_power_state;
}

static int endeavor_s5k3h2y_power_on(void)
{
    int ret;

    pr_info("[CAM] s5k3h2y power on ++\n");

	if (endeavor_s5k3h2y_power_state)
		return 0;

	tegra_gpio_disable(RAW_SPI_CLK);
	tegra_gpio_disable(RAW_SPI_CS);
	tegra_gpio_disable(RAW_SPI_DI);
	tegra_gpio_disable(RAW_SPI_DO);
	//tegra_gpio_disable(RAW_INTR0);
	//tegra_gpio_disable(RAW_INTR1);
	tegra_gpio_disable(CAM_I2C_SCL_GPIO);
	tegra_gpio_disable(CAM_I2C_SDA_GPIO);
	tegra_gpio_disable(CAM_MCLK_GPIO);

    gpio_direction_output(CAM_PWDN, 0);
    gpio_direction_output(CAM1_VCM_PD_GPIO, 0);
    #if RAWCHIP
    gpio_direction_output(RAW_RSTN, 0);
    #endif
    //ENR_msleep(1); //TODO

#ifdef CAMERA_REGULATOR
    //pr_info("[CAM] use regurator to get power\n");

	#if RAWCHIP
	/*RAW_1V8_EN */
	gpio_direction_output(RAW_1V8_EN, 1);
	ENR_usleep(200);
	#endif

	/* VCM */
	ret = regulator_enable(cam_vcm_2v85_en);
	if (ret < 0) {
		pr_err("[CAM] couldn't enable regulator v_cam1_vcm_2v85\n");
		regulator_put(cam_vcm_2v85_en);
		cam_vcm_2v85_en = NULL;
		return ret;
	}

    /* main/front cam analog*/
	ret = regulator_enable(cam_a2v85_en);
	if (ret < 0) {
		pr_err("[CAM] couldn't enable regulator cam_a2v85_en\n");
		regulator_put(cam_a2v85_en);
		cam_a2v85_en = NULL;
		return ret;
	}
    ENR_usleep(200);
 
    /*main cam core 1v2 & rawchip external 1v2 */
	ret = regulator_enable(cam_d1v2_en);
	if (ret < 0) {
		pr_err("[CAM] couldn't enable regulator cam_d1v2_en\n");
		regulator_put(cam_d1v2_en);
		cam_d1v2_en = NULL;
		return ret;
	}
	ENR_usleep(200);
 
	#if RAWCHIP
	/*RAW_1V2_EN */
    if (htc_get_pcbid_info() < PROJECT_PHASE_XE) {
        gpio_direction_output(RAW_1V2_EN, 1);
	    ENR_usleep(200);
    }
	#endif

    /* IO */
	ret = regulator_enable(cam_vddio_1v8_en);
	if (ret < 0) {
		pr_err("[CAM] couldn't enable regulator cam_vddio_1v8_en\n");
		regulator_put(cam_vddio_1v8_en);
		cam_vddio_1v8_en = NULL;
		return ret;
	}
    ENR_usleep(100);

    /*CAM SEL */
    gpio_direction_output(CAM_SEL_GPIO, 0);
    ENR_usleep(100);

    #if RAWCHIP
    /*RAW_RSTN */
    gpio_direction_output(RAW_RSTN, 1);
    ENR_msleep(3);
    /*SPI send command to configure RAWCHIP here!*/
	yushan_spi_write(0x0008, 0x7f);
	ENR_msleep(1);
    #endif

    /* XSHUTDOWM */
    gpio_direction_output(CAM_PWDN, 1);
    ENR_usleep(100);

     /* VCM PD*/
    gpio_direction_output(CAM1_VCM_PD_GPIO, 1);
    ENR_usleep(100);

#else/* use gpio pull up to get power*/
      #if RAWCHIP
	/*RAW_1V8_EN */
	gpio_direction_output(RAW_1V8_EN, 1);
	ENR_usleep(200);
      #endif

	/* VCM */
	gpio_direction_output(CAM_VCM2V85, 1);
	ENR_usleep(200);

	/* analog */
	gpio_direction_output(CAM_A2V85_EN, 1);
	ENR_usleep(200);

	/*core*/
	gpio_direction_output(CAM_D1V2_EN, 1);
	ENR_usleep(200);

	#if RAWCHIP
	/*RAW_1V2_EN */
    if (htc_get_pcbid_info() < PROJECT_PHASE_XE) {
	    ret = gpio_direction_output(RAW_1V2_EN, 1);
	    ENR_usleep(200);
    }
    #endif

	/* IO */
	gpio_direction_output(CAMIO_1V8_EN, 1);
	ENR_usleep(100);

	/*CAM SEL */
	gpio_direction_output(CAM_SEL_GPIO, 0);
	ENR_usleep(100);

      #if RAWCHIP
	/*RAW_RSTN */
	ret = gpio_direction_output(RAW_RSTN, 1);
	ENR_msleep(3);

      /*SPI send command to configure RAWCHIP here!*/
	  yushan_spi_write(0x0008, 0x7f);
		ENR_msleep(1);
      #endif

	/* XSHUTDOWM */
	gpio_direction_output(CAM_PWDN, 1);
	ENR_usleep(100);
	/* VCM PD*/
	gpio_direction_output(CAM1_VCM_PD_GPIO, 1);
	ENR_usleep(100);
#endif
	endeavor_s5k3h2y_power_state = 1;
    //pr_info("[CAM] s5k3h2y power on --\n");
    return 0;
}

static int endeavor_s5k3h2y_power_off(void)
{
    pr_info("[CAM] s5k3h2y power off ++\n");
    if (!endeavor_s5k3h2y_power_state)
	return 0;
    endeavor_s5k3h2y_power_state = 0;
#ifdef CAMERA_REGULATOR
    /* VCM PD*/
    gpio_direction_output(CAM1_VCM_PD_GPIO, 0);
    ENR_msleep(1);

    /* XSHUTDOWN */
    gpio_direction_output(CAM_PWDN, 0);
    ENR_msleep(1);

    #if RAWCHIP
    /* RAW RSTN*/
    gpio_direction_output(RAW_RSTN, 0);
    ENR_msleep(3);
    #endif

   /* VCM */
   regulator_disable(cam_vcm_2v85_en);
   ENR_msleep(1);

    #if RAWCHIP
    /*RAW_1V2_EN */
    if (htc_get_pcbid_info() < PROJECT_PHASE_XE) {
        gpio_direction_output(RAW_1V2_EN, 0);
        ENR_msleep(5);
    }
    #endif

   /* digital */
   regulator_disable(cam_d1v2_en);
   ENR_msleep(1);

   #if RAWCHIP
   /*RAW_1V8_EN */
   gpio_direction_output(RAW_1V8_EN, 0);
   ENR_msleep(1);
   #endif

   /* analog */
   regulator_disable(cam_a2v85_en);
   ENR_msleep(5);
   /* IO */
   regulator_disable(cam_vddio_1v8_en);
   ENR_msleep(10);

#else/* use gpio pull down to disable power */
    /* VCM PD*/
    gpio_direction_output(CAM1_VCM_PD_GPIO, 0);
    ENR_msleep(1);
    /* TODO: Set 0x0100[0] = 0 (Enter SW Standby mode)*/

    /* XSHUTDOWN */
    gpio_direction_output(CAM_PWDN, 0);
    ENR_msleep(1);

    #if RAWCHIP
    /* RAW RSTN*/
    gpio_direction_output(RAW_RSTN, 0);
    ENR_msleep(3);
    #endif

    /* VCM */
    gpio_direction_output(CAM_VCM2V85, 0);
    ENR_msleep(1);

    #if RAWCHIP
    /*RAW_1V2_EN */
    if (htc_get_pcbid_info() < PROJECT_PHASE_XE) {
        gpio_direction_output(RAW_1V2_EN, 0);
        ENR_msleep(5);
    }
    #endif

    /* digital */
    gpio_direction_output(CAM_D1V2_EN, 0);
	ENR_msleep(1);

   #if RAWCHIP
   /*RAW_1V8_EN */
   gpio_direction_output(RAW_1V8_EN, 0);
   ENR_msleep(1);
   #endif

    /* analog */
    gpio_direction_output(CAM_A2V85_EN, 0);
	ENR_msleep(5);
    /* IO */
    gpio_direction_output(CAMIO_1V8_EN, 0);
    ENR_msleep(10);
#endif

	/* set gpio output low : O(L) */
	tegra_gpio_enable(RAW_SPI_CLK);
	tegra_gpio_enable(RAW_SPI_CS);
	tegra_gpio_enable(RAW_SPI_DI);
	tegra_gpio_enable(RAW_SPI_DO);
	//tegra_gpio_enable(RAW_INTR0);
	//tegra_gpio_enable(RAW_INTR1);
	tegra_gpio_enable(CAM_I2C_SCL_GPIO);
	tegra_gpio_enable(CAM_I2C_SDA_GPIO);
	tegra_gpio_enable(CAM_MCLK_GPIO);

    return 0;
}

struct s5k3h2y_platform_data endeavor_s5k3h2y_data = {
	.get_power_state = endeavor_s5k3h2y_get_power,
	.power_on = endeavor_s5k3h2y_power_on,
	.power_off = endeavor_s5k3h2y_power_off,
};

struct ad5823_platform_data endeavor_ad5823_data = {
	.get_power_state = endeavor_s5k3h2y_get_power,
};

static int endeavor_s5k6a1gx03_power_on(void)
{
    int ret;
    pr_info("[CAM] s5k6a1g power on ++\n");
    gpio_direction_output(FRONT_CAM_RST_GPIO, 0);
    gpio_direction_output(CAM_SEL_GPIO, 0);

	tegra_gpio_disable(CAM_I2C_SCL_GPIO);
	tegra_gpio_disable(CAM_I2C_SDA_GPIO);
	tegra_gpio_disable(CAM_MCLK_GPIO);

#ifdef CAMERA_REGULATOR
    pr_info("[CAM] use regurator to get power\n");
    /* analog */
	ret = regulator_enable(cam_a2v85_en);
	if (ret < 0) {
		pr_err("[CAM] couldn't enable regulator cam_a2v85_en\n");
		regulator_put(cam_a2v85_en);
		cam_a2v85_en = NULL;
		return ret;
	}
    ENR_usleep(200);
    /*vcm*/
	ret = regulator_enable(cam_vcm_2v85_en);
	if (ret < 0) {
		pr_err("[CAM] couldn't enable regulator cam_vcm_2v85_en\n");
		regulator_put(cam_vcm_2v85_en);
		cam_vcm_2v85_en = NULL;
		return ret;
	}
    ENR_usleep(200);
    /*IO*/
	ret = regulator_enable(cam_vddio_1v8_en);
	if (ret < 0) {
		pr_err("[CAM] couldn't enable regulator cam_vddio_1v8_en\n");
		regulator_put(cam_vddio_1v8_en);
		cam_vddio_1v8_en = NULL;
		return ret;
	}
    ENR_usleep(200);
    /*RSTN */
    gpio_direction_output(FRONT_CAM_RST_GPIO, 1);
    /* digital */
	ret = regulator_enable(cam2_d1v2_en);
	if (ret < 0) {
		pr_err("[CAM] couldn't enable regulator cam_d1v2_en\n");
		regulator_put(cam_d1v2_en);
		cam_d1v2_en = NULL;
		return ret;
	}
    /*CAM SEL */
    gpio_direction_output(CAM_SEL_GPIO, 1);
    ENR_msleep(1);
#else /* use gpio pull up to get power */
    /* vcm */
    gpio_direction_output(CAM_VCM2V85, 1);
    ENR_usleep(200);
    /* analog */
    gpio_direction_output(CAM_A2V85_EN, 1);
    ENR_usleep(200);
    /*IO*/
    gpio_direction_output(CAMIO_1V8_EN, 1);
    ENR_usleep(200);
    /*RSTN */
    gpio_direction_output(FRONT_CAM_RST_GPIO, 1);
    /* digital */
    gpio_direction_output(CAM2_D1V2_EN, 1);
    /*CAM SEL */
    gpio_direction_output(CAM_SEL_GPIO, 1);
    ENR_msleep(1);
#endif
    return 0;
}

static int endeavor_s5k6a1gx03_power_off(void)
{
    pr_info("[CAM] s5k6a1g power off ++\n");
#ifdef CAMERA_REGULATOR
    /*CAM SEL */
    gpio_direction_output(CAM_SEL_GPIO, 0);
    ENR_msleep(1);
    /* vcm */
    regulator_disable(cam_vcm_2v85_en);
    ENR_msleep(1);
    /* analog */
    regulator_disable(cam_a2v85_en);
    ENR_msleep(5);
    /*RSTN */
    gpio_direction_output(FRONT_CAM_RST_GPIO, 0);
    /* digital */
    regulator_disable(cam2_d1v2_en);
    ENR_msleep(1);
      /* IO */
    regulator_disable(cam_vddio_1v8_en);
    ENR_msleep(10);

#else/* use gpio pull down to disable power*/
    /*CAM SEL */
    gpio_direction_output(CAM_SEL_GPIO, 0);
    ENR_msleep(1);
    /* vcm */
    gpio_direction_output(CAM_VCM2V85, 0);
    ENR_msleep(5);
    /* analog */
    gpio_direction_output(CAM_A2V85_EN, 0);
    ENR_msleep(5);
    /*RSTN */
    gpio_direction_output(FRONT_CAM_RST_GPIO, 0);
    /* digital */
    gpio_direction_output(CAM2_D1V2_EN, 0);
    ENR_msleep(1);
      /* IO */
    gpio_direction_output(CAMIO_1V8_EN, 0);
    ENR_msleep(10);
#endif

	tegra_gpio_enable(CAM_I2C_SCL_GPIO);
	tegra_gpio_enable(CAM_I2C_SDA_GPIO);
	tegra_gpio_enable(CAM_MCLK_GPIO);

    return 0;
}

struct s5k6a1gx03_platform_data endeavor_s5k6a1gx03_data = {
	.power_on = endeavor_s5k6a1gx03_power_on,
	.power_off = endeavor_s5k6a1gx03_power_off,
};

static struct i2c_board_info endeavor_i2c3_board_info[] = {
	{
		I2C_BOARD_INFO("s5k3h2y", 0x10),
		.platform_data = &endeavor_s5k3h2y_data,
	},
	{
		I2C_BOARD_INFO("ad5823", 0x0C),
		.platform_data = &endeavor_ad5823_data,
	}
};

static struct i2c_board_info endeavor_i2c4_board_info[] = {
	{
		I2C_BOARD_INFO("s5k6a1gx03", 0x36),
		.platform_data = &endeavor_s5k6a1gx03_data,
	},
};

struct endeavor_cam_gpio {
	int gpio;
	const char *label;
	int value;
};

/*static struct tps61050_pin_state enterprise_tps61050_pinstate = {
	.mask		= 0x0008,*/ /*VGP3*/
/*	.values		= 0x0008,
};*/

/* I2C bus becomes active when vdd_1v8_cam is enabled */
/*static int enterprise_tps61050_pm(int pwr)
{
	static struct regulator *enterprise_flash_reg = NULL;
	int ret = 0;

	pr_info("%s: ++%d\n", __func__, pwr);
	switch (pwr) {
	case TPS61050_PWR_OFF:
		if (enterprise_flash_reg)
			regulator_disable(enterprise_flash_reg);
		break;
	case TPS61050_PWR_STDBY:
	case TPS61050_PWR_COMM:
	case TPS61050_PWR_ON:
		if (!enterprise_flash_reg) {
			enterprise_flash_reg = regulator_get(NULL, "vdd_1v8_cam");
			if (IS_ERR_OR_NULL(enterprise_flash_reg)) {
				pr_err("%s: failed to get flash pwr\n", __func__);
				return PTR_ERR(enterprise_flash_reg);
			}
		}
		ret = regulator_enable(enterprise_flash_reg);
		if (ret) {
			pr_err("%s: failed to enable flash pwr\n", __func__);
			goto fail_regulator_flash_reg;
		}
		ENR_msleep(1);
		break;
	default:
		ret = -1;
	}
	return ret;

fail_regulator_flash_reg:
	regulator_put(enterprise_flash_reg);
	enterprise_flash_reg = NULL;
	return ret;
}*/

#define TEGRA_CAMERA_GPIO(_gpio, _label, _value)	\
	{						\
		.gpio = _gpio,				\
		.label = _label,			\
		.value = _value,			\
	}
#ifndef CAMERA_REGULATOR
static struct endeavor_cam_gpio endeavor_cam_gpio_output_data[] = {
	[0] = TEGRA_CAMERA_GPIO(CAM_SEL_GPIO, "cam_sel_gpio", 0),
	[1] = TEGRA_CAMERA_GPIO(FRONT_CAM_RST_GPIO, "front_cam_rst_gpio", 0),
	[2] = TEGRA_CAMERA_GPIO(CAM_A2V85_EN, "cam_a2v85_en", 0),
	[3] = TEGRA_CAMERA_GPIO(CAM_PWDN, "cam_pwdn", 0),
	[4] = TEGRA_CAMERA_GPIO(CAM_D1V2_EN, "cam_d1v2_en", 0),
	[5] = TEGRA_CAMERA_GPIO(CAM2_D1V2_EN, "cam2_d1v2_en", 0),
	[6] = TEGRA_CAMERA_GPIO(CAM_VCM2V85, "cam_vcm2v85", 0),
	[7] = TEGRA_CAMERA_GPIO(CAMIO_1V8_EN, "camio_1v8_en", 0),
	[8] = TEGRA_CAMERA_GPIO(CAM1_VCM_PD_GPIO, "cam1_vcm_pd", 0),
	[9] = TEGRA_CAMERA_GPIO(CAM_I2C_SCL_GPIO, "CAM_I2C_SCL_GPIO", 0),
	[10] = TEGRA_CAMERA_GPIO(CAM_I2C_SDA_GPIO, "CAM_I2C_SDA_GPIO", 0),
	[11] = TEGRA_CAMERA_GPIO(CAM_MCLK_GPIO, "CAM_MCLK_GPIO", 0),
	/*for rawchip */
	[12] = TEGRA_CAMERA_GPIO(RAW_1V8_EN, "RAW_1V8_EN", 0),
	[13] = TEGRA_CAMERA_GPIO(RAW_1V2_EN, "RAW_1V2_EN", 0),
	[14] = TEGRA_CAMERA_GPIO(RAW_RSTN, "RAW_RSTN", 0),
	[15] = TEGRA_CAMERA_GPIO(RAW_SPI_CLK, "RAW_SPI_CLK", 0),
	[16] = TEGRA_CAMERA_GPIO(RAW_SPI_CS, "RAW_SPI_CS", 0),
	[17] = TEGRA_CAMERA_GPIO(RAW_SPI_DI, "RAW_SPI_DI", 0),
	[18] = TEGRA_CAMERA_GPIO(RAW_SPI_DO, "RAW_SPI_DO", 0),
};
#else
static struct endeavor_cam_gpio endeavor_cam_gpio_output_data[] = {
	[0] = TEGRA_CAMERA_GPIO(CAM_SEL_GPIO, "cam_sel_gpio", 0),
	[1] = TEGRA_CAMERA_GPIO(FRONT_CAM_RST_GPIO, "front_cam_rst_gpio", 0),
	[2] = TEGRA_CAMERA_GPIO(CAM_PWDN, "cam_pwdn", 0),
	[3] = TEGRA_CAMERA_GPIO(CAM1_VCM_PD_GPIO, "cam1_vcm_pd", 0),
	[4] = TEGRA_CAMERA_GPIO(CAM_I2C_SCL_GPIO, "CAM_I2C_SCL_GPIO", 0),
	[5] = TEGRA_CAMERA_GPIO(CAM_I2C_SDA_GPIO, "CAM_I2C_SDA_GPIO", 0),
	[6] = TEGRA_CAMERA_GPIO(CAM_MCLK_GPIO, "CAM_MCLK_GPIO", 0),
	/*for rawchip */
	[7] = TEGRA_CAMERA_GPIO(RAW_1V8_EN, "RAW_1V8_EN", 0),
	[8] = TEGRA_CAMERA_GPIO(RAW_1V2_EN, "RAW_1V2_EN", 0),
	[9] = TEGRA_CAMERA_GPIO(RAW_RSTN, "RAW_RSTN", 0),
	[10] = TEGRA_CAMERA_GPIO(RAW_SPI_CLK, "RAW_SPI_CLK", 0),
	[11] = TEGRA_CAMERA_GPIO(RAW_SPI_CS, "RAW_SPI_CS", 0),
	[12] = TEGRA_CAMERA_GPIO(RAW_SPI_DI, "RAW_SPI_DI", 0),
	[13] = TEGRA_CAMERA_GPIO(RAW_SPI_DO, "RAW_SPI_DO", 0),
};
#endif
static struct endeavor_cam_gpio endeavor_cam_gpio_input_data[] = {
	[0] = TEGRA_CAMERA_GPIO(CAM1_ID_GPIO, "cam1_id_gpio", 0),
	[1] = TEGRA_CAMERA_GPIO(FRONT_CAM_ID_GPIO, "front_cam_id_gpio", 0),
	[2] = TEGRA_CAMERA_GPIO(RAW_INTR0, "RAW_INTR0", 0),
	[3] = TEGRA_CAMERA_GPIO(RAW_INTR1, "RAW_INTR1", 0),
};

static struct pca954x_platform_mode enterprise_pca954x_modes[] = {
	{ .adap_id = PCA954x_I2C_BUS0, .deselect_on_exit = true, },
	{ .adap_id = PCA954x_I2C_BUS1, .deselect_on_exit = true, },
	{ .adap_id = PCA954x_I2C_BUS2, .deselect_on_exit = true, },
	{ .adap_id = PCA954x_I2C_BUS3, .deselect_on_exit = true, },
};

static struct pca954x_platform_data enterprise_pca954x_data = {
	.modes    = enterprise_pca954x_modes,
	.num_modes      = ARRAY_SIZE(enterprise_pca954x_modes),
};

	
/*
static struct tps61050_platform_data enterprise_tps61050_data = {
	.cfg		= 0,
	.num		= 1,
	.max_amp_torch	= CAM_FLASH_MAX_TORCH_AMP,
	.max_amp_flash	= CAM_FLASH_MAX_FLASH_AMP,
	.pinstate	= &enterprise_tps61050_pinstate,
	.init		= NULL,
	.exit		= NULL,
	.pm		= &enterprise_tps61050_pm,
	.gpio_envm	= NULL,
	.gpio_sync	= NULL,
};
*/

static int endeavor_cam_init(void)
{
    int ret;
    int i = 0, j = 0;

    for (i = 0; i < ARRAY_SIZE(endeavor_cam_gpio_output_data); i++) {
        /* for XE, raw_1v2_en is removed */
        if (htc_get_pcbid_info() >= PROJECT_PHASE_XE && endeavor_cam_gpio_output_data[i].gpio == RAW_1V2_EN) {
            continue;
        }
        ret = gpio_request(endeavor_cam_gpio_output_data[i].gpio,
                   endeavor_cam_gpio_output_data[i].label);
        if (ret < 0) {
            pr_err("[CAM] %s: gpio_request failed for gpio #%d\n",
                __func__, i);
            goto fail_free_gpio;
        }
        gpio_direction_output(endeavor_cam_gpio_output_data[i].gpio,
                      endeavor_cam_gpio_output_data[i].value);
        gpio_export(endeavor_cam_gpio_output_data[i].gpio, false);
        tegra_gpio_enable(endeavor_cam_gpio_output_data[i].gpio);
    }

    for (j = 0; j < ARRAY_SIZE(endeavor_cam_gpio_input_data); j++) {
        ret = gpio_request(endeavor_cam_gpio_input_data[j].gpio,
                   endeavor_cam_gpio_input_data[j].label);
        if (ret < 0) {
            pr_err("[CAM] %s: gpio_request failed for gpio #%d\n",
                __func__, j);
            goto fail_free_gpio;
        }
        gpio_direction_input(endeavor_cam_gpio_input_data[j].gpio);
        gpio_export(endeavor_cam_gpio_input_data[j].gpio, false);
        tegra_gpio_enable(endeavor_cam_gpio_input_data[j].gpio);
    }
/* set gpio input no pull: I(NP) */
    tegra_pinmux_set_pullupdown(TEGRA_PINGROUP_KB_ROW6, TEGRA_PUPD_NORMAL);/* CAM1_ID_GPIO */
    tegra_pinmux_set_pullupdown(TEGRA_PINGROUP_KB_ROW0, TEGRA_PUPD_NORMAL);/* RAW_INTR0 */
    tegra_pinmux_set_pullupdown(TEGRA_PINGROUP_CLK3_REQ, TEGRA_PUPD_NORMAL);/* RAW_INTR1 */
/* set gpio input no pull: I(PU) */
    tegra_pinmux_set_pullupdown(TEGRA_PINGROUP_KB_ROW7, TEGRA_PUPD_PULL_UP);/* FRONT_CAM_ID_GPIO */

    i2c_register_board_info(2, endeavor_i2c3_board_info,
        ARRAY_SIZE(endeavor_i2c3_board_info));

    i2c_register_board_info(2, endeavor_i2c4_board_info,
        ARRAY_SIZE(endeavor_i2c4_board_info));

    return 0;

fail_free_gpio:
    pr_err("[CAM] %s endeavor_cam_init failed!\n", __func__);
    while (i--)
        gpio_free(endeavor_cam_gpio_output_data[i].gpio);
    while (j--)
        gpio_free(endeavor_cam_gpio_input_data[j].gpio);
    return ret;
}

struct enterprise_battery_gpio {
	int gpio;
	const char *label;
};

#define TEGRA_BATTERY_GPIO(_gpio, _label)	\
	{					\
		.gpio = _gpio,			\
		.label = _label,		\
	}

struct enterprise_battery_gpio enterprise_battery_gpio_data[] ={
	[0] = TEGRA_BATTERY_GPIO(TEGRA_GPIO_PU4, "mbat_in"),
	[1] = TEGRA_BATTERY_GPIO(TEGRA_GPIO_PW0, "chg_stat"),
	[2] = TEGRA_BATTERY_GPIO(TEGRA_GPIO_PX5, "chg_int"),
};

static struct htc_battery_platform_data htc_battery_pdev_data = {
	.gpio_mbat_in = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PU4),
	.gpio_mbat_in_trigger_level = MBAT_IN_LOW_TRIGGER,
	.guage_driver = GUAGE_NONE,
	.charger = SWITCH_CHARGER_TPS65200,
	.vzero_clb_channel = -1,
	.volt_adc_offset = 0,
	.power_off_by_id = 1,
};

static struct platform_device htc_battery_pdev = {
	.name	= "htc_battery",
	.id	= -1,
	.dev	= {
	        .platform_data = &htc_battery_pdev_data,
	},
};

static struct tps65200_platform_data tps65200_data = {
	.gpio_chg_stat = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PW0),
	.gpio_chg_int  = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PX5),
};

static struct i2c_board_info tps_65200_boardinfo[] = {
	{
		I2C_BOARD_INFO("tps65200", 0xD4 >> 1),
		.platform_data = &tps65200_data,
	},
};

#if 1	/* fixme: for MFG build to disable mbat_in check */
static int __init check_mbat_in_tag(char *get_mbat_in)
{
	if (strlen(get_mbat_in) && !strcmp(get_mbat_in, "false")) {
		htc_battery_pdev_data.power_off_by_id = 0;
	}
	return 1;
}
__setup("mbat_in_check=", check_mbat_in_tag);
#endif

static void enterprise_battery_init(void)
{
	int ret;
	int i;

	for (i = 0; i < ARRAY_SIZE(enterprise_battery_gpio_data); i++) {
		ret = gpio_request(enterprise_battery_gpio_data[i].gpio,
				   enterprise_battery_gpio_data[i].label);
		if (ret < 0) {
			pr_err("%s: gpio_request failed for gpio #%d\n",
				__func__, i);
			goto bat_fail_free_gpio;
		}

		ret = gpio_direction_input(enterprise_battery_gpio_data[i].gpio);
		if (ret < 0) {
			pr_err("%s: gpio_direction_input failed for gpio #%d\n",
				__func__, i);
			goto bat_fail_free_gpio;
		}

		tegra_gpio_enable(enterprise_battery_gpio_data[i].gpio);
	}

	if(htc_get_pcbid_info() <= PROJECT_PHASE_XC)
		htc_battery_pdev_data.volt_adc_offset = -17;

	platform_device_register(&htc_battery_pdev);

	i2c_register_board_info(4, tps_65200_boardinfo,
					ARRAY_SIZE(tps_65200_boardinfo));

	return;

bat_fail_free_gpio:
	pr_err("%s enterprise_battery_init failed!\n", __func__);
	while (i--)
		gpio_free(enterprise_battery_gpio_data[i].gpio);
}

#define ENTERPRISE_INA230_ENABLED 0

#if ENTERPRISE_INA230_ENABLED
static struct ina230_platform_data ina230_platform = {
	.rail_name = "VDD_AC_BAT",
	.current_threshold = TEGRA_CUR_MON_THRESHOLD,
	.resistor = TEGRA_CUR_MON_RESISTOR,
	.min_cores_online = TEGRA_CUR_MON_MIN_CORES,
};

static struct i2c_board_info enterprise_i2c0_ina230_info[] = {
	{
		I2C_BOARD_INFO("ina230", 0x42),
		.platform_data = &ina230_platform,
		.irq = -1,
	},
};

static int __init enterprise_ina230_init(void)
{
	return i2c_register_board_info(0, enterprise_i2c0_ina230_info,
				       ARRAY_SIZE(enterprise_i2c0_ina230_info));
}
#endif

int __init enterprise_sensors_init(void)
{
	psensor_init();
	
	int ret;
	if (htc_get_pcbid_info() == PROJECT_PHASE_XA){
		pr_info("[GYRO]Use Invensense solution");
		enterprise_mpuirq_init();
	}
	enterprise_comp_irq_init();
	//enterprise_srio_1v8_en();
	enterprise_gsensor_irq_init(); 
	if (htc_get_pcbid_info() != PROJECT_PHASE_XA ){
		enterprise_mpuirq_init();
		enterprise_gyro_diag_init();
		i2c_register_board_info(0,
			pana_gyro_GSBI12_boardinfo, ARRAY_SIZE(pana_gyro_GSBI12_boardinfo));
		i2c_register_board_info(0,
			i2c_bma250_devices, ARRAY_SIZE(i2c_bma250_devices));
		if (htc_get_pcbid_info() < PROJECT_PHASE_XC )
			i2c_register_board_info(0,
				i2c_akm8975_devices_xb, ARRAY_SIZE(i2c_akm8975_devices_xb));
		else
			i2c_register_board_info(0,
				i2c_akm8975_devices_xc, ARRAY_SIZE(i2c_akm8975_devices_xc));
	
		enterprise_gyro_sleep_pin();
	}

#if 0	/* fixme: require owner to check these */
	enterprise_isl_init();
#endif
	enterprise_battery_init();
	ret = endeavor_cam_init();
	enterprise_nct1008_init();
#if ENTERPRISE_INA230_ENABLED
	enterprise_ina230_init();
#endif
	edge_nfc_init();
	return ret;
}

int __init endeavor_cam_late_init(void)
{
	int ret = 0;
	printk("%s: \n", __func__);
/* vcm */
      cam_vcm_2v85_en = regulator_get(NULL, "v_cam_vcm_2v85");
		if (IS_ERR_OR_NULL(cam_vcm_2v85_en)) {
			ret = PTR_ERR(cam_vcm_2v85_en);
			pr_err("[CAM] couldn't get regulator v_cam_vcm_2v85\n");
			cam_vcm_2v85_en = NULL;
			return ret;
		}
/* io */
      cam_vddio_1v8_en = regulator_get(NULL, "v_camio_1v8");
		if (IS_ERR_OR_NULL(cam_vddio_1v8_en)) {
			ret = PTR_ERR(cam_vddio_1v8_en);
			pr_err("[CAM] couldn't get regulator v_camio_1v8\n");
			cam_vddio_1v8_en = NULL;
			return ret;
		}
/* analog */
      cam_a2v85_en = regulator_get(NULL, "v_cam_a2v85");
		if (IS_ERR_OR_NULL(cam_a2v85_en)) {
			ret = PTR_ERR(cam_a2v85_en);
			pr_err("[CAM] couldn't get regulator v_cam_a2v85\n");
			cam_a2v85_en = NULL;
			return ret;
		}
/* cam_d1v2 */
      cam_d1v2_en = regulator_get(NULL, "v_cam_d1v2");
		if (IS_ERR_OR_NULL(cam_d1v2_en)) {
			ret = PTR_ERR(cam_d1v2_en);
			pr_err("[CAM] couldn't get regulator v_cam_d1v2\n");
			cam_d1v2_en = NULL;
			return ret;
		}
/* cam2_d1v2 */
      cam2_d1v2_en = regulator_get(NULL, "v_cam2_d1v2");
		if (IS_ERR_OR_NULL(cam2_d1v2_en)) {
			ret = PTR_ERR(cam2_d1v2_en);
			pr_err("[CAM] couldn't get regulator v_cam2_d1v2\n");
			cam2_d1v2_en = NULL;
			return ret;
		}
	return ret;
}

#ifdef CAMERA_REGULATOR
late_initcall(endeavor_cam_late_init);
#endif
