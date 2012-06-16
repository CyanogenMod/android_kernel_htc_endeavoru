/*
 * arch/arm/mach-tegra/board-endeavoru.h
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

#ifndef _MACH_TEGRA_BOARD_ENDEAVORU_H
#define _MACH_TEGRA_BOARD_ENDEAVORU_H

#include <mach/gpio.h>
#include <mach/irqs.h>
#include <linux/mfd/tps80031.h>

/* Processor Board  ID */
#define BOARD_E1205		0x0C05
#define BOARD_E1197		0x0B61
#define SKU_BATTERY_SUPPORT	0x1

/* Board Fab version */
#define BOARD_FAB_A00		0x0
#define BOARD_FAB_A01		0x1
#define BOARD_FAB_A02		0x2

/* vdd_cpu voltage follower */
#define BOARD_SKU_VF_BIT	0x0400

int enterprise_charge_init(void);
int enterprise_sdhci_init(void);
int endeavoru_pinmux_init(void);
int enterprise_panel_init(void);
int enterprise_sensors_init(void);
int touch_init(void);
int enterprise_kbc_init(void);
int enterprise_emc_init(void);
int enterprise_regulator_init(void);
//int enterprise_modem_init(void);
int enterprise_suspend_init(void);
int enterprise_edp_init(void);
void __init enterprise_tsensor_init(void);
void enterprise_bpc_mgmt_init(void);

/* Invensense MPU Definitions */
#define MPU_GYRO_NAME		"mpu3050"
#define MPU_GYRO_IRQ_GPIO	TEGRA_GPIO_PH4
#define MPU_GYRO_ADDR		0x68
#define MPU_GYRO_BUS_NUM	0
#define MPU_GYRO_ORIENTATION	{ -1, 0, 0, 0, -1, 0, 0, 0, 1 }
#define MPU_ACCEL_NAME		"kxtf9"
#define MPU_ACCEL_IRQ_GPIO	0 /* DISABLE ACCELIRQ:  TEGRA_GPIO_PJ2 */
#define MPU_ACCEL_ADDR		0x0F
#define MPU_ACCEL_BUS_NUM	0
#define MPU_ACCEL_ORIENTATION	{ 0, 1, 0, -1, 0, 0, 0, 0, 1 }
#define MPU_COMPASS_NAME	"ak8975"
#define MPU_COMPASS_IRQ_GPIO	0
#define MPU_COMPASS_ADDR	0x0C
#define MPU_COMPASS_BUS_NUM	0
#define MPU_COMPASS_ORIENTATION	{ 0, 1, 0, -1, 0, 0, 0, 0, 1 }

/* PCA954x I2C bus expander bus addresses */
#define PCA954x_I2C_BUS_BASE	6
#define PCA954x_I2C_BUS0	(PCA954x_I2C_BUS_BASE + 0)
#define PCA954x_I2C_BUS1	(PCA954x_I2C_BUS_BASE + 1)
#define PCA954x_I2C_BUS2	(PCA954x_I2C_BUS_BASE + 2)
#define PCA954x_I2C_BUS3	(PCA954x_I2C_BUS_BASE + 3)

/*Gyro*/

#define RUBY_GPIO_PANA_GYRO_SLEEP		(70)
#define RUBY_GPIO_GYRO_ID		(130)
#define RUBY_GPIO_GYRO_DIAG	(41)

#define RUBY_LAYOUTS_XB			{ \
			{ { -1,  0, 0}, {0,  -1,  0}, {0, 0, 1} }, \
			{ { 0, -1, 0}, { 1,  0,  0}, {0, 0, -1} }, \
			{ { 0,  -1, 0}, { 1,  0,  0}, {0, 0,  1} }, \
			{ {-1,  0, 0}, { 0,  0, -1}, {0, 1,  0} }   \
				}

#define RUBY_LAYOUTS_XC			{ \
			{ { 0,  1, 0}, {-1,  0,  0}, {0, 0, 1} }, \
			{ { 0, -1, 0}, { 1,  0,  0}, {0, 0, -1} }, \
			{ { -1,  0, 0}, { 0,  -1,  0}, {0, 0,  1} }, \
			{ {-1,  0, 0}, { 0,  0, -1}, {0, 1,  0} }   \
				}

/*****************External GPIO tables ******************/
/* External peripheral gpio base. */
#define ENT_TPS80031_GPIO_BASE	   TEGRA_NR_GPIOS
#define ENT_TPS80031_GPIO_REGEN1 (ENT_TPS80031_GPIO_BASE + TPS80031_GPIO_REGEN1)
#define ENT_TPS80031_GPIO_REGEN2 (ENT_TPS80031_GPIO_BASE + TPS80031_GPIO_REGEN2)
#define ENT_TPS80031_GPIO_SYSEN	 (ENT_TPS80031_GPIO_BASE + TPS80031_GPIO_SYSEN)
#define ENT_TPS80031_GPIO_END	(ENT_TPS80031_GPIO_BASE + TPS80031_GPIO_NR)

/*****************External Interrupt tables ******************/
/* External peripheral irq base */
#define ENT_TPS80031_IRQ_BASE	TEGRA_NR_IRQS
#define ENT_TPS80031_IRQ_END  (ENT_TPS80031_IRQ_BASE + TPS80031_INT_NR)

/***************** Camera GPIOs ******************/
#define CAM_SEL_GPIO	TEGRA_GPIO_PCC1

#define CAM_PWDN TEGRA_GPIO_PF4
#define FRONT_CAM_RST_GPIO	TEGRA_GPIO_PM2
#define CAM_D1V2_EN TEGRA_GPIO_PF5
#define CAM2_D1V2_EN TEGRA_GPIO_PF6
#define CAMIO_1V8_EN TEGRA_GPIO_PBB4
#define CAM_A2V85_EN TEGRA_GPIO_PE3

#define CAM1_VCM_PD_GPIO	TEGRA_GPIO_PBB5
#define CAM_VCM2V85 TEGRA_GPIO_PM7

#define CAM1_ID_GPIO	TEGRA_GPIO_PR6
#define FRONT_CAM_ID_GPIO	TEGRA_GPIO_PR7

#define CAM_I2C_SCL_GPIO	TEGRA_GPIO_PBB1
#define CAM_I2C_SDA_GPIO	TEGRA_GPIO_PBB2
#define CAM_MCLK_GPIO	TEGRA_GPIO_PCC0

#define RAW_1V8_EN TEGRA_GPIO_PR3
#define RAW_1V2_EN TEGRA_GPIO_PR5
#define RAW_RSTN TEGRA_GPIO_PR4
#define RAW_INTR0 TEGRA_GPIO_PR0
#define RAW_INTR1 TEGRA_GPIO_PEE1

#define RAW_SPI_CLK TEGRA_GPIO_PC2
#define RAW_SPI_DO TEGRA_GPIO_PC3
#define RAW_SPI_CS TEGRA_GPIO_PJ5
#define RAW_SPI_DI TEGRA_GPIO_PJ6
/***************** end of Camera GPIOs ******************/

/* Audio-related GPIOs */
#define TEGRA_GPIO_HP_DET	TEGRA_GPIO_PW3

/* Baseband GPIO addresses */
#define BB_VDD_EN                       TEGRA_GPIO_PM4
#define AP2BB_RST_PWRDWNn               TEGRA_GPIO_PC1
#define AP2BB_RSTn                      TEGRA_GPIO_PN0
#define AP2BB_PWRON                     TEGRA_GPIO_PN3
#define BB2AP_RADIO_FATAL               TEGRA_GPIO_PN2

#define BB_GPIO_BB_EN                   AP2BB_PWRON
#define BB_GPIO_BB_RST                  AP2BB_RSTn
#define BB_GPIO_SPI_INT                 TEGRA_GPIO_PN1
#define BB_GPIO_SPI_SS                  TEGRA_GPIO_PV0 //wakeup
#define BB_GPIO_AWR                     TEGRA_GPIO_PC6
#define BB_GPIO_CWR                     TEGRA_GPIO_PJ0

#define XMM6260_GPIO_BB_ON              BB_GPIO_BB_EN
#define XMM6260_GPIO_BB_RST             BB_GPIO_BB_RST
#define XMM6260_GPIO_IPC_HSIC_ACTIVE    BB_GPIO_SPI_INT
#define XMM6260_GPIO_IPC_HSIC_SUS_REQ   BB_GPIO_SPI_SS
#define XMM6260_GPIO_IPC_BB_WAKE        BB_GPIO_AWR
#define XMM6260_GPIO_IPC_AP_WAKE        BB_GPIO_CWR

/* NFC GPIO */
#define RUBY_GPIO_NFC_INT       TEGRA_GPIO_PY6
#define RUBY_GPIO_NFC_VEN       TEGRA_GPIO_PM5
#define RUBY_GPIO_NFC_DL        TEGRA_GPIO_PM6

/* flashlight, FL_TORCH_EN, FL_FLASH_EN */
#define FL_TORCH_EN		TEGRA_GPIO_PR1
#define FL_FLASH_EN		TEGRA_GPIO_PBB3

/* Proximity sensor, PS_INT*/
#define PS_INT			TEGRA_GPIO_PK2

#ifdef CONFIG_BT_CTS_WAKEUP
/* Bluetooth BT_EN*/
#define BT_GPIO_EN	TEGRA_GPIO_PU0
#define BT_GPIO_CTS_IRQ	TEGRA_GPIO_PO5
#endif

#define TDIODE_OFFSET	(9000)	/* in millicelsius */

/* Battery Peak Current Management */
#define TEGRA_BPC_TRIGGER		TEGRA_GPIO_PR3
#define TEGRA_BPC_TIMEOUT		100 /* ms */
#define TEGRA_BPC_CPU_PWR_LIMIT	0 /* in mW, (0 disables) */

#define TEGRA_CUR_MON_THRESHOLD		-2000
#define TEGRA_CUR_MON_RESISTOR		20
#define TEGRA_CUR_MON_MIN_CORES		2

/* Baseband IDs */

enum tegra_bb_type {
	TEGRA_BB_PH450 = 1,
	TEGRA_BB_XMM6260,
	TEGRA_BB_M7400,
};

/* Cable Detect */
#define TEGRA_GPIO_DESK_AUD	TEGRA_GPIO_PCC5
#define TEGRA_GPIO_MHL_RST	TEGRA_GPIO_PE6
#define TEGRA_GPIO_MHL_USB_SEL	TEGRA_GPIO_PE0
#define TEGRA_GPIO_USB_ID	TEGRA_GPIO_PS2

#endif /*_MACH_TEGRA_BOARD_ENDEAVORU_H */
