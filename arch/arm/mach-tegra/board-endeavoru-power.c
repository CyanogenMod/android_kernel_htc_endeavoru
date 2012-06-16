/*
 * arch/arm/mach-tegra/board-endeavoru-power.c
 *
 * Copyright (C) 2011 NVIDIA, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */
#include <linux/i2c.h>
#include <linux/pda_power.h>
#include <linux/platform_device.h>
#include <linux/resource.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/gpio-switch-regulator.h>
#include <linux/mfd/tps80031.h>
#include <linux/regulator/tps80031-regulator.h>
#include <linux/tps80031-charger.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/regulator/fan53555-regulator.h>
#include <linux/cpumask.h>
#include <linux/platform_data/tegra_bpc_mgmt.h>

#include <mach/edp.h>
#include <mach/iomap.h>
#include <mach/irqs.h>
#include <mach/pinmux.h>
#include <mach/board_htc.h>

#include "gpio-names.h"
#include "board.h"
#include "board-endeavoru.h"
#include "pm.h"
#include "wakeups-t3.h"
#include "tegra3_tsensor.h"
#ifdef CONFIG_ADC_TPS80032
#include <linux/tps80032_adc.h>
#endif
#ifdef CONFIG_VSYS_ALARM_TPS80032
#include <linux/tps80032_vsys_alarm.h>
#endif

#define PMC_CTRL		0x0
#define PMC_CTRL_INTR_LOW	(1 << 17)

#define PMC_DPD_PADS_ORIDE		0x01c
#define PMC_DPD_PADS_ORIDE_BLINK	(1 << 20)

/************************ TPS80031 based regulator ****************/
static struct regulator_consumer_supply tps80031_vio_supply[] = {
	REGULATOR_SUPPLY("vio_1v8", NULL),
	REGULATOR_SUPPLY("vddio_gmi", NULL),
	REGULATOR_SUPPLY("vddio_lcd", NULL),
	REGULATOR_SUPPLY("pwrdet_lcd", NULL),
	REGULATOR_SUPPLY("vddio_sdmmc1", NULL),
	REGULATOR_SUPPLY("pwrdet_sdmmc1", NULL),
	REGULATOR_SUPPLY("vddio_sdmmc4", NULL),
	REGULATOR_SUPPLY("pwrdet_sdmmc4", NULL),
	REGULATOR_SUPPLY("vddio_sys", NULL),
	REGULATOR_SUPPLY("vddio_cam", NULL),
	REGULATOR_SUPPLY("pwrdet_cam", NULL),
	REGULATOR_SUPPLY("vddio_bb", NULL),
	REGULATOR_SUPPLY("pwrdet_bb", NULL),
	REGULATOR_SUPPLY("vddio_uart", NULL),
	REGULATOR_SUPPLY("pwrdet_uart", NULL),
	REGULATOR_SUPPLY("vddio_audio", NULL),
	REGULATOR_SUPPLY("pwrdet_audio", NULL),
	REGULATOR_SUPPLY("avdd_osc", NULL),
	REGULATOR_SUPPLY("avdd_usb_pll", NULL),
	REGULATOR_SUPPLY("avdd_hdmi_pll", NULL), //NC
	REGULATOR_SUPPLY("vddio_sdmmc3", NULL),
};

static struct regulator_consumer_supply tps80031_smps1_supply[] = {
	REGULATOR_SUPPLY("v_core_1v2", NULL),
	REGULATOR_SUPPLY("vdd_core", NULL),
};

static struct regulator_consumer_supply tps80031_smps2_supply[] = {
	REGULATOR_SUPPLY("v_prereg_2v6", NULL),
};

/* board XD */
static struct regulator_consumer_supply tps80031_smps1_xd_supply[] = {
	REGULATOR_SUPPLY("vdd_cpu", NULL),
};

/* board XD */
static struct regulator_consumer_supply tps80031_smps2_xd_supply[] = {
	REGULATOR_SUPPLY("vdd_core", NULL),
};

static struct regulator_consumer_supply tps80031_smps3_supply[] = {
	REGULATOR_SUPPLY("v_lpddr2_1v2", NULL),
	REGULATOR_SUPPLY("vddio_ddr", NULL),
	REGULATOR_SUPPLY("ddr_comp_pu", NULL),
};

static struct regulator_consumer_supply tps80031_smps4_supply[] = {
	REGULATOR_SUPPLY("v_sdmmc_2v85", NULL),
};

/* board XD */
static struct regulator_consumer_supply tps80031_smps4_xd_supply[] = {
        REGULATOR_SUPPLY("v_prereg_2v6", NULL),
};

static struct regulator_consumer_supply tps80031_vana_supply[] = {
	REGULATOR_SUPPLY("unused_vana", NULL),
};

static struct regulator_consumer_supply tps80031_ldo1_supply[] = {
	REGULATOR_SUPPLY("v_dsi_csi_1v2", NULL),
	REGULATOR_SUPPLY("avdd_dsi_csi", NULL),
	REGULATOR_SUPPLY("vddio_hsic", NULL),
};

static struct regulator_consumer_supply tps80031_ldo2_supply[] = {
	REGULATOR_SUPPLY("v_vrtc_1v2", NULL),
	REGULATOR_SUPPLY("vdd_rtc", NULL),
};

static struct regulator_consumer_supply tps80031_ldo3_supply[] = {
	REGULATOR_SUPPLY("v_fuse_src_3v3", NULL),
	REGULATOR_SUPPLY("vpp_fuse", NULL),
};

static struct regulator_consumer_supply tps80031_ldo4_supply[] = {
	REGULATOR_SUPPLY("v_lcm_3v", NULL), //NC
};

static struct regulator_consumer_supply tps80031_ldo5_supply[] = {
	REGULATOR_SUPPLY("v_sr_2v85", NULL),
};

static struct regulator_consumer_supply tps80031_ldo6_supply[] = {
	REGULATOR_SUPPLY("v_mmc_rx_2v85", NULL),
	REGULATOR_SUPPLY("vdd_ddr_rx", NULL),
};

static struct regulator_consumer_supply tps80031_ldo7_supply[] = {
	REGULATOR_SUPPLY("v_pllmeux_1v2", NULL),
	REGULATOR_SUPPLY("vdd_plla_p_c", NULL),
	REGULATOR_SUPPLY("vdd_pllm", NULL),
	REGULATOR_SUPPLY("vdd_pllu_d", NULL),
	REGULATOR_SUPPLY("vdd_pllx", NULL),
};

static struct regulator_consumer_supply tps80031_ldo7_xe_supply[] = {
	REGULATOR_SUPPLY("v_pllmeux_1v2", NULL),
	REGULATOR_SUPPLY("vdd_plla_p_c", NULL),
	REGULATOR_SUPPLY("vdd_pllm", NULL),
	REGULATOR_SUPPLY("vdd_pllu_d", NULL),
	REGULATOR_SUPPLY("vdd_pllx", NULL),
};

static struct regulator_consumer_supply tps80031_ldoln_supply[] = {
	REGULATOR_SUPPLY("v_ddr_hs_1v0", NULL),
	REGULATOR_SUPPLY("vdd_ddr_hs", NULL),
};

static struct regulator_consumer_supply tps80031_ldousb_supply[] = {
	REGULATOR_SUPPLY("v_usb_3v3", NULL),
	REGULATOR_SUPPLY("avdd_usb", NULL),
	REGULATOR_SUPPLY("avdd_hdmi", NULL),
};

static struct regulator_consumer_supply tps80031_vbus_supply[] = {
	REGULATOR_SUPPLY("usb_vbus", NULL),
};

#define TPS_PDATA_INIT(_id, _minmv, _maxmv, _supply_reg, _always_on,	\
	_boot_on, _apply_uv, _init_uV, _init_enable, _init_apply, 	\
	_flags, _ectrl, _delay)						\
	static struct tps80031_regulator_platform_data pdata_##_id = {	\
		.regulator = {						\
			.constraints = {				\
				.min_uV = (_minmv)*1000,		\
				.max_uV = (_maxmv)*1000,		\
				.valid_modes_mask = (REGULATOR_MODE_NORMAL |  \
						REGULATOR_MODE_STANDBY),      \
				.valid_ops_mask = (REGULATOR_CHANGE_MODE |    \
						REGULATOR_CHANGE_STATUS |     \
						REGULATOR_CHANGE_VOLTAGE),    \
				.always_on = _always_on,		\
				.boot_on = _boot_on,			\
				.apply_uV = _apply_uv,			\
			},						\
			.num_consumer_supplies =			\
				ARRAY_SIZE(tps80031_##_id##_supply),	\
			.consumer_supplies = tps80031_##_id##_supply,	\
			.supply_regulator = _supply_reg,		\
		},							\
		.init_uV =  _init_uV * 1000,				\
		.init_enable = _init_enable,				\
		.init_apply = _init_apply,				\
		.flags = _flags,					\
		.ext_ctrl_flag = _ectrl,				\
		.delay_us = _delay,					\
	}

TPS_PDATA_INIT(vio,   600, 2100, 0, 1, 0, 0, -1, 0, 0, 0, 0, 0);
TPS_PDATA_INIT(smps1, 600, 2600, 0, 0, 0, 0, -1, 0, 0, 0, PWR_REQ_INPUT_PREQ1, 0);
TPS_PDATA_INIT(smps2, 600, 2600, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0);
TPS_PDATA_INIT(smps1_xd, 600, 2100, 0, 0, 0, 0, -1, 0, 0, 0, PWR_REQ_INPUT_PREQ2 | PWR_OFF_ON_SLEEP, 0);
TPS_PDATA_INIT(smps2_xd, 600, 2600, 0, 0, 0, 0, -1, 0, 0, 0, PWR_REQ_INPUT_PREQ1, 0);
TPS_PDATA_INIT(smps3, 600, 2100, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0);
TPS_PDATA_INIT(smps4, 600, 2100, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0);
TPS_PDATA_INIT(smps4_xd, 600, 2200, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0);
TPS_PDATA_INIT(ldo1, 1000, 3300, tps80031_rails(VIO), 0, 0, 0, -1, 0, 0, 0, 0, 0);
TPS_PDATA_INIT(ldo2, 1000, 3300, 0, 1, 1, 1, 1000, 1, 1, 0, 0, 0);
TPS_PDATA_INIT(ldo3, 1000, 3300, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0);
TPS_PDATA_INIT(ldo4, 1000, 3300, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0);
TPS_PDATA_INIT(ldo5, 1000, 3300, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0);
TPS_PDATA_INIT(ldo6, 1000, 3300, 0, 0, 0, 0, -1, 0, 0, 0, PWR_REQ_INPUT_PREQ1, 0);
TPS_PDATA_INIT(ldo7, 1000, 3300, tps80031_rails(VIO), 0, 0, 0, -1, 0, 0, 0, 0, 0);
TPS_PDATA_INIT(ldo7_xe, 1000, 3300, tps80031_rails(VIO), 0, 0, 0, -1, 0, 0, 0, PWR_REQ_INPUT_PREQ1, 0);
TPS_PDATA_INIT(ldoln, 1000, 3300, tps80031_rails(SMPS3), 0, 0, 0, -1, 0, 0, 0, PWR_REQ_INPUT_PREQ1, 0);
TPS_PDATA_INIT(ldousb, 1000, 3300, 0, 0, 0, 0, -1, 0, 0, USBLDO_INPUT_VSYS, PWR_OFF_ON_SLEEP,0);
TPS_PDATA_INIT(vana,  1000, 3300, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0);
TPS_PDATA_INIT(vbus,  0, 5000, 0, 0, 0, 0, -1, 0, 0, (VBUS_SW_ONLY | VBUS_DISCHRG_EN_PDN), 0, 100000);

static struct tps80031_rtc_platform_data rtc_data = {
	.irq = ENT_TPS80031_IRQ_BASE + TPS80031_INT_RTC_ALARM,
	.time = {
		.tm_year = 2011,
		.tm_mon = 0,
		.tm_mday = 1,
		.tm_hour = 1,
		.tm_min = 2,
		.tm_sec = 3,
	},
};

#define TPS_RTC()				\
	{						\
		.id	= 0,		\
		.name	= "rtc_tps80031",	\
		.platform_data = &rtc_data,	\
	}

#ifdef CONFIG_ADC_TPS80032
static struct tps80032_adc_platform_data pdata_adc = {
	.adc_vref = 1250,
	.gpadc_rt_irq = ENT_TPS80031_IRQ_BASE + TPS80031_INT_GPADC_RT,
	.gpadc_sw_irq = ENT_TPS80031_IRQ_BASE + TPS80031_INT_GPADC_SW2_EOC,
	.calib_bit_map = 0x7F,
};
#endif

#ifdef CONFIG_VSYS_ALARM_TPS80032
struct tps80032_vsys_alarm_platform_data pdata_vsys_alarm = {
	.vsys_low_irq = ENT_TPS80031_IRQ_BASE + TPS80031_INT_SYS_VLOW,
};
#endif

#define TPS_DEV(_data)					\
	{						\
		.id	 = -1,				\
		.name   = "tps80032-"#_data,		\
		.platform_data  = &pdata_##_data,	\
	}

#define TPS_REG(_id, _data)				\
	{						\
		.id	 = TPS80031_ID_##_id,		\
		.name   = "tps80031-regulator",		\
		.platform_data  = &pdata_##_data,	\
	}

static struct tps80031_subdev_info tps80031_devs[] = {
	TPS_REG(VIO, vio),
	TPS_REG(SMPS1, smps1),
	TPS_REG(SMPS2, smps2),
	TPS_REG(SMPS3, smps3),
	TPS_REG(SMPS4, smps4),
	TPS_REG(LDO1, ldo1),
	TPS_REG(LDO2, ldo2),
	TPS_REG(LDO3, ldo3),
	TPS_REG(LDO4, ldo4),
	TPS_REG(LDO5, ldo5),
	TPS_REG(LDO6, ldo6),
	TPS_REG(LDO7, ldo7),
	TPS_REG(LDOLN, ldoln),
	TPS_REG(LDOUSB, ldousb),
	TPS_REG(VANA, vana),
	TPS_REG(VBUS, vbus),
	TPS_RTC(),
#ifdef CONFIG_ADC_TPS80032
	TPS_DEV(adc),
#endif
#ifdef CONFIG_VSYS_ALARM_TPS80032
	TPS_DEV(vsys_alarm),
#endif
};

static struct tps80031_subdev_info tps80031_devs_xd[] = {
	TPS_REG(VIO, vio),
	TPS_REG(SMPS1, smps1_xd),
	TPS_REG(SMPS2, smps2_xd),
	TPS_REG(SMPS3, smps3),
	TPS_REG(SMPS4, smps4_xd),
	TPS_REG(LDO1, ldo1),
	TPS_REG(LDO2, ldo2),
	TPS_REG(LDO3, ldo3),
	TPS_REG(LDO4, ldo4),
	TPS_REG(LDO5, ldo5),
	TPS_REG(LDO6, ldo6),
	TPS_REG(LDO7, ldo7),
	TPS_REG(LDOLN, ldoln),
	TPS_REG(LDOUSB, ldousb),
	TPS_REG(VANA, vana),
	TPS_REG(VBUS, vbus),
	TPS_RTC(),
#ifdef CONFIG_ADC_TPS80032
	TPS_DEV(adc),
#endif
#ifdef CONFIG_VSYS_ALARM_TPS80032
	TPS_DEV(vsys_alarm),
#endif
};

static struct tps80031_subdev_info tps80031_devs_xe[] = {
	TPS_REG(VIO, vio),
	TPS_REG(SMPS1, smps1_xd),
	TPS_REG(SMPS2, smps2_xd),
	TPS_REG(SMPS3, smps3),
	TPS_REG(SMPS4, smps4_xd),
	TPS_REG(LDO1, ldo1),
	TPS_REG(LDO2, ldo2),
	TPS_REG(LDO3, ldo3),
	TPS_REG(LDO4, ldo4),
	TPS_REG(LDO5, ldo5),
	TPS_REG(LDO6, ldo6),
	TPS_REG(LDO7, ldo7_xe),
	TPS_REG(LDOLN, ldoln),
	TPS_REG(LDOUSB, ldousb),
	TPS_REG(VANA, vana),
	TPS_REG(VBUS, vbus),
	TPS_RTC(),
#ifdef CONFIG_ADC_TPS80032
	TPS_DEV(adc),
#endif
#ifdef CONFIG_VSYS_ALARM_TPS80032
	TPS_DEV(vsys_alarm),
#endif
};

struct tps80031_clk32k_init_data clk32k_idata[] = {
	{
		.clk32k_nr = TPS80031_CLOCK32K_G,
		.enable = true,
		.ext_ctrl_flag = 0,//PWR_REQ_INPUT_PREQ1,
	},
	{
		.clk32k_nr = TPS80031_CLOCK32K_AUDIO,
		.enable = true,
		.ext_ctrl_flag = PWR_REQ_INPUT_PREQ1,
	},
};

static struct tps80031_platform_data tps_platform = {
	.num_subdevs	= ARRAY_SIZE(tps80031_devs),
	.subdevs	= tps80031_devs,
	.irq_base	= ENT_TPS80031_IRQ_BASE,
	.gpio_base	= ENT_TPS80031_GPIO_BASE,
	.clk32k_init_data	= clk32k_idata,
	.clk32k_init_data_size	= ARRAY_SIZE(clk32k_idata),
};

static struct i2c_board_info __initdata enterprise_regulators[] = {
	{
		I2C_BOARD_INFO("tps80031", 0x4A),
		.irq		= INT_EXTERNAL_PMU,
		.platform_data	= &tps_platform,
	},
};

/* TPS62361B DC-DC converter */
static struct regulator_consumer_supply fan53555_dcdc_supply[] = {
	REGULATOR_SUPPLY("vdd_cpu", NULL),
};

static struct fan53555_regulator_platform_data fan53555_pdata = {
	.reg_init_data = {					\
		.constraints = {				\
			.min_uV = 500000,			\
			.max_uV = 1770000,			\
			.valid_modes_mask = (REGULATOR_MODE_NORMAL |  \
					     REGULATOR_MODE_STANDBY), \
			.valid_ops_mask = (REGULATOR_CHANGE_MODE |    \
					   REGULATOR_CHANGE_STATUS |  \
					   REGULATOR_CHANGE_VOLTAGE), \
			.always_on = 1,				\
			.boot_on =  1,				\
			.apply_uV = 0,				\
		},						\
		.num_consumer_supplies = ARRAY_SIZE(fan53555_dcdc_supply), \
		.consumer_supplies = fan53555_dcdc_supply,		\
		},							\
	.vsel_id = 0,							\
};
static struct i2c_board_info __initdata fan53555_boardinfo[] = {
	{
		I2C_BOARD_INFO("fan53555", 0x60),
		.platform_data	= &fan53555_pdata,
	},
};
/************************ GPIO based switch regulator ****************/

/* REGEN2 from PMU*/
static struct regulator_consumer_supply gpio_switch_led_3v3_en_supply[] = {
	REGULATOR_SUPPLY("v_led_3v3", NULL),
};
static int gpio_switch_led_3v3_en_voltages[] = {3300};

static struct regulator_consumer_supply gpio_switch_aud_a1v8_en_supply[] = {
        REGULATOR_SUPPLY("v_aud_a1v8", NULL),
};
static int gpio_switch_aud_a1v8_en_voltages[] = {1800};

static struct regulator_consumer_supply gpio_switch_vib_3v_en_supply[] = {
        REGULATOR_SUPPLY("v_vib_3v", NULL),
};
static int gpio_switch_vib_3v_en_voltages[] = {3000};

static struct regulator_consumer_supply gpio_switch_mhl_3v3_en_supply[] = {
        REGULATOR_SUPPLY("v_tp_3v3", NULL),  //v_mhl_3v3
};
static int gpio_switch_mhl_3v3_en_voltages[] = {3300};

static struct regulator_consumer_supply gpio_switch_aud_3v3_en_supply[] = {
        REGULATOR_SUPPLY("v_aud_3v3", NULL),
};
static int gpio_switch_aud_3v3_en_voltages[] = {3300};

static struct regulator_consumer_supply gpio_switch_cam_vcm_2v85_en_supply[] = {
        REGULATOR_SUPPLY("v_cam_vcm_2v85", NULL),
};
static int gpio_switch_cam_vcm_2v85_en_voltages[] = {2850};

static struct regulator_consumer_supply gpio_switch_lcmio_1v8_en_supply[] = {
        REGULATOR_SUPPLY("v_lcmio_1v8", NULL),
};
static int gpio_switch_lcmio_1v8_en_voltages[] = {1800};

static struct regulator_consumer_supply gpio_switch_lcm_3v3_en_supply[] = {
        REGULATOR_SUPPLY("v_lcm_3v3", NULL),
};
static int gpio_switch_lcm_3v3_en_voltages[] = {3300};

static struct regulator_consumer_supply gpio_switch_srio_1v8_en_supply[] = {
        REGULATOR_SUPPLY("v_srio_1v8", NULL),
};
static int gpio_switch_srio_1v8_en_voltages[] = {1800};

static struct regulator_consumer_supply gpio_switch_cam2_d1v2_en_supply[] = {
        REGULATOR_SUPPLY("v_cam2_d1v2", NULL),
};
static int gpio_switch_cam2_d1v2_en_voltages[] = {1800};

static struct regulator_consumer_supply gpio_switch_cam_d1v2_en_supply[] = {
        REGULATOR_SUPPLY("v_cam_d1v2", NULL),
};
static int gpio_switch_cam_d1v2_en_voltages[] = {1800};

static struct regulator_consumer_supply gpio_switch_camio_1v8_en_supply[] = {
        REGULATOR_SUPPLY("v_camio_1v8", NULL),
};
static int gpio_switch_camio_1v8_en_voltages[] = {1800};

static struct regulator_consumer_supply gpio_switch_cam_a2v85_en_supply[] = {
        REGULATOR_SUPPLY("v_cam_a2v85", NULL),
};
static int gpio_switch_cam_a2v85_en_voltages[] = {2850};

static struct regulator_consumer_supply gpio_switch_mhl_1v2_en_supply[] = {
        REGULATOR_SUPPLY("v_mhl_1v2", NULL),
};
static int gpio_switch_mhl_1v2_en_voltages[] = {1200};


static struct regulator_consumer_supply gpio_switch_sdmmc_2v85_en_supply[] = {
        REGULATOR_SUPPLY("v_sdmmc_2v85", NULL),
};
static int gpio_switch_sdmmc_2v85_en_voltages[] = {2850};

/* Macro for defining gpio switch regulator sub device data */
#define GREG_INIT(_id, _name, _input_supply, _gpio_nr, _active_low, \
			_init_state, _pg, _enable, _disable)		\
	static struct gpio_switch_regulator_subdev_data gpio_pdata_##_name =  \
	{								\
		.regulator_name	= "gpio-switch-"#_name,			\
		.input_supply	= _input_supply,			\
		.id		= _id,					\
		.gpio_nr	= _gpio_nr,				\
		.pin_group	= _pg,					\
		.active_low	= _active_low,				\
		.init_state	= _init_state,				\
		.voltages	= gpio_switch_##_name##_voltages,	\
		.n_voltages	= ARRAY_SIZE(gpio_switch_##_name##_voltages), \
		.num_consumer_supplies =				\
				ARRAY_SIZE(gpio_switch_##_name##_supply), \
		.consumer_supplies = gpio_switch_##_name##_supply,	\
		.constraints = {					\
			.valid_modes_mask = (REGULATOR_MODE_NORMAL |	\
					REGULATOR_MODE_STANDBY),	\
			.valid_ops_mask = (REGULATOR_CHANGE_MODE |	\
					REGULATOR_CHANGE_STATUS |	\
					REGULATOR_CHANGE_VOLTAGE),	\
		},							\
		.enable_rail = _enable,					\
		.disable_rail = _disable,				\
	}


GREG_INIT(0, led_3v3_en,      NULL, ENT_TPS80031_GPIO_REGEN2, false, 1, 0, 0, 0);
GREG_INIT(1, aud_a1v8_en,       NULL, TEGRA_GPIO_PD2, false, 0, 0, 0, 0);
GREG_INIT(2, vib_3v_en, 	NULL, TEGRA_GPIO_PE7, false, 0, 0, 0, 0);
GREG_INIT(3, mhl_3v3_en,        NULL, TEGRA_GPIO_PY2, false, 0, 0, 0, 0);
GREG_INIT(4, aud_3v3_en,       NULL, TEGRA_GPIO_PB2, false, 0, 0, 0, 0);
GREG_INIT(5, cam_vcm_2v85_en, NULL, TEGRA_GPIO_PM7, false, 0, 0, 0, 0);
GREG_INIT(6, lcm_3v3_en,     NULL, TEGRA_GPIO_PE2, false, 1, 0, 0, 0);
GREG_INIT(7, lcmio_1v8_en,     "vio_1v8", TEGRA_GPIO_PE5, false, 1, 0, 0, 0);
GREG_INIT(8, srio_1v8_en,      "vio_1v8", TEGRA_GPIO_PY3, false, 1, 0, 0, 0);
GREG_INIT(9, cam2_d1v2_en,      "vio_1v8", TEGRA_GPIO_PF6, false, 0, 0, 0, 0);
GREG_INIT(10, cam_d1v2_en,      "vio_1v8", TEGRA_GPIO_PF5, false, 0, 0, 0, 0);
GREG_INIT(11, camio_1v8_en,      NULL, TEGRA_GPIO_PBB4, false, 0, 0, 0, 0);
GREG_INIT(12, cam_a2v85_en,      NULL, TEGRA_GPIO_PE3, false, 0, 0, 0, 0);
GREG_INIT(13, mhl_1v2_en,     NULL, TEGRA_GPIO_PE4, false, 0, 0, 0, 0);
GREG_INIT(14, sdmmc_2v85_en,     NULL, TEGRA_GPIO_PM3, false, 1, 0, 0, 0);


#define ADD_GPIO_REG(_name)	(&gpio_pdata_##_name)
static struct gpio_switch_regulator_subdev_data *gswitch_subdevs[] = {
	ADD_GPIO_REG(led_3v3_en),
	ADD_GPIO_REG(vib_3v_en),
	ADD_GPIO_REG(srio_1v8_en),
	ADD_GPIO_REG(aud_a1v8_en),
	ADD_GPIO_REG(aud_3v3_en),
	ADD_GPIO_REG(lcmio_1v8_en),
	ADD_GPIO_REG(lcm_3v3_en),
	ADD_GPIO_REG(mhl_3v3_en),
	ADD_GPIO_REG(mhl_1v2_en),
	ADD_GPIO_REG(cam_vcm_2v85_en),
	ADD_GPIO_REG(cam2_d1v2_en),
	ADD_GPIO_REG(cam_d1v2_en),
	ADD_GPIO_REG(camio_1v8_en),
	ADD_GPIO_REG(cam_a2v85_en),
};

static struct gpio_switch_regulator_subdev_data *gswitch_subdevs_xd[] = {
	ADD_GPIO_REG(led_3v3_en),
	ADD_GPIO_REG(vib_3v_en),
	ADD_GPIO_REG(srio_1v8_en),
	ADD_GPIO_REG(aud_a1v8_en),
	ADD_GPIO_REG(aud_3v3_en),
	ADD_GPIO_REG(lcmio_1v8_en),
	ADD_GPIO_REG(lcm_3v3_en),
	ADD_GPIO_REG(mhl_3v3_en),
	ADD_GPIO_REG(mhl_1v2_en),
	ADD_GPIO_REG(cam_vcm_2v85_en),
	ADD_GPIO_REG(cam2_d1v2_en),
	ADD_GPIO_REG(cam_d1v2_en),
	ADD_GPIO_REG(camio_1v8_en),
	ADD_GPIO_REG(cam_a2v85_en),
	ADD_GPIO_REG(sdmmc_2v85_en),
};

static struct gpio_switch_regulator_platform_data  gswitch_pdata = {
	.num_subdevs = ARRAY_SIZE(gswitch_subdevs),
	.subdevs = gswitch_subdevs,
};

static struct platform_device gswitch_regulator_pdata = {
	.name	= "gpio-switch-regulator",
	.id	= -1,
	.dev	= {
		.platform_data = &gswitch_pdata,
	},
};

static int __init enterprise_gpio_switch_regulator_init(void)
{
	int i;
	if(htc_get_pcbid_info() >= PROJECT_PHASE_XD) {
		gswitch_pdata.num_subdevs = ARRAY_SIZE(gswitch_subdevs_xd);
		gswitch_pdata.subdevs = gswitch_subdevs_xd;
	}
	for (i = 0; i < gswitch_pdata.num_subdevs; ++i) {
		struct gpio_switch_regulator_subdev_data *gswitch_data =
						gswitch_pdata.subdevs[i];
		if (gswitch_data->gpio_nr <= TEGRA_NR_GPIOS)
			tegra_gpio_enable(gswitch_data->gpio_nr);
	}
	return platform_device_register(&gswitch_regulator_pdata);
}

#ifdef CONFIG_PM
static void endeavor_suspend_work(void)
{
	int ret = gpio_direction_output(TEGRA_GPIO_PF7, 0);
	if (ret)
		pr_err("%s: fail to output low gpio PF7\n", __func__);
}

static void endeavor_resume_work(void)
{
	int ret = gpio_direction_output(TEGRA_GPIO_PF7, 1);
	if (ret)
		pr_err("%s: fail to output high gpio PF7\n", __func__);
}
#endif

static int __init endeavor_gpio_rtc_init(void)
{
	int ret = gpio_request(TEGRA_GPIO_PF7, "pmu_msecure");
	if (ret) {
		pr_err("%s unable to request gpio PF7\n", __func__);
		goto fail;
	}
	ret = gpio_direction_output(TEGRA_GPIO_PF7, 1);
	if (ret) {
		pr_err("%s unable to set output gpio PF7\n", __func__);
		goto fail;
	}
	tegra_gpio_enable(TEGRA_GPIO_PF7);
#ifdef CONFIG_PM
	tps_platform.suspend_work = endeavor_suspend_work;
	tps_platform.resume_work = endeavor_resume_work;
#endif
	return 0;
fail:
	gpio_free(TEGRA_GPIO_PF7);
	return ret;
}

static void enterprise_power_off(void)
{
	int ret;
#if 0
        pr_info("enterprise: Powering off the device\n");
        ret = tps80031_power_off();
#else
        pr_info("enterprise: Powering off the device or"
                " enter offmode charging\n");
        tps80031_power_off_or_reboot();
#endif
	if (ret)
		pr_err("enterprise: failed to power off\n");
	while(1);
}

void __init enterprise_tsensor_init(void)
{
	tegra3_tsensor_init(NULL);
}

int __init enterprise_regulator_init(void)
{
	void __iomem *pmc = IO_ADDRESS(TEGRA_PMC_BASE);
	u32 pmc_ctrl;
	u32 pmc_dpd_pads;

	/* configure the power management controller to trigger PMU
	 * interrupts when low */

	pmc_ctrl = readl(pmc + PMC_CTRL);
	writel(pmc_ctrl | PMC_CTRL_INTR_LOW, pmc + PMC_CTRL);

	pmc_dpd_pads = readl(pmc + PMC_DPD_PADS_ORIDE);
	writel(pmc_dpd_pads & ~PMC_DPD_PADS_ORIDE_BLINK , pmc + PMC_DPD_PADS_ORIDE);

	endeavor_gpio_rtc_init();

	int projectPhase = htc_get_pcbid_info();

	if (projectPhase == PROJECT_PHASE_XD){
		tps_platform.num_subdevs = ARRAY_SIZE(tps80031_devs_xd);
		tps_platform.subdevs = tps80031_devs_xd;
	} else if (projectPhase >= PROJECT_PHASE_XE){
		tps_platform.num_subdevs = ARRAY_SIZE(tps80031_devs_xe);
		tps_platform.subdevs = tps80031_devs_xe;
	}
	i2c_register_board_info(4, enterprise_regulators, 1);
	enterprise_gpio_switch_regulator_init();
	pm_power_off = enterprise_power_off;


	if (htc_get_pcbid_info() <= PROJECT_PHASE_XC) {
		pr_info("[PMIC]Registering the device FAN53555\n");
		i2c_register_board_info(4, fan53555_boardinfo, 1);
	}

	return 0;
}

static void enterprise_board_suspend(int lp_state, enum suspend_stage stg)
{
	if ((lp_state == TEGRA_SUSPEND_LP1) && (stg == TEGRA_SUSPEND_BEFORE_CPU))
		tegra_console_uart_suspend();
}

static void enterprise_board_resume(int lp_state, enum resume_stage stg)
{
	if ((lp_state == TEGRA_SUSPEND_LP1) && (stg == TEGRA_RESUME_AFTER_CPU))
		tegra_console_uart_resume();
}

static struct tegra_suspend_platform_data enterprise_suspend_data = {
	.cpu_timer	= 2000,
	.cpu_off_timer	= 200,
	.suspend_mode	= TEGRA_SUSPEND_LP0,
	.core_timer	= 0x7e7e,
	.core_off_timer = 0,
	.corereq_high	= true,
	.sysclkreq_high	= true,
	.board_suspend = enterprise_board_suspend,
	.board_resume = enterprise_board_resume,
	.cpu_resume_boost	= 1500000,
	.boost_resume_reason	= 0x80,
};

static void enterprise_init_deep_sleep_mode(void)
{
	struct board_info bi;
	tegra_get_board_info(&bi);

	if (bi.board_id == BOARD_E1205 && bi.fab == BOARD_FAB_A01)
		enterprise_suspend_data.suspend_mode = TEGRA_SUSPEND_LP1;

	if ((bi.board_id == BOARD_E1205 && (bi.sku & BOARD_SKU_VF_BIT) == 0) ||
	    (bi.board_id == BOARD_E1197 && (bi.sku & BOARD_SKU_VF_BIT)))
		enterprise_suspend_data.cpu_timer = 8000;
}

int __init enterprise_suspend_init(void)
{
	enterprise_init_deep_sleep_mode();
	tegra_init_suspend(&enterprise_suspend_data);
	return 0;
}

#ifdef CONFIG_TEGRA_EDP_LIMITS

int __init enterprise_edp_init(void)
{
	unsigned int regulator_mA;

	regulator_mA = get_maximum_cpu_current_supported();
	pr_info("%s: CPU regulator %d mA\n", __func__, regulator_mA);
//	if (!regulator_mA) {
	regulator_mA = 5000; /* regular AP30 */
//	}
//	pr_info("%s: CPU regulator %d mA\n", __func__, regulator_mA);

	tegra_init_cpu_edp_limits(regulator_mA);
	tegra_init_system_edp_limits(TEGRA_BPC_CPU_PWR_LIMIT);
	return 0;
}
#endif

static struct tegra_bpc_mgmt_platform_data bpc_mgmt_platform_data = {
	.gpio_trigger = TEGRA_BPC_TRIGGER,
	.bpc_mgmt_timeout = TEGRA_BPC_TIMEOUT,
};

static struct platform_device enterprise_bpc_mgmt_device = {
	.name		= "tegra-bpc-mgmt",
	.id		= -1,
	.dev		= {
		.platform_data = &bpc_mgmt_platform_data,
	},
};

void __init enterprise_bpc_mgmt_init(void)
{
	int int_gpio;

	tegra_gpio_enable(TEGRA_BPC_TRIGGER);

	int_gpio = tegra_gpio_to_int_pin(TEGRA_BPC_TRIGGER);

#ifdef CONFIG_SMP
	cpumask_setall(&(bpc_mgmt_platform_data.affinity_mask));
	irq_set_affinity_hint(int_gpio,
				&(bpc_mgmt_platform_data.affinity_mask));
	irq_set_affinity(int_gpio, &(bpc_mgmt_platform_data.affinity_mask));
#endif
	platform_device_register(&enterprise_bpc_mgmt_device);

	return;
}
