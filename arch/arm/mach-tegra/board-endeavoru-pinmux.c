/*
 * arch/arm/mach-tegra/board-endeavoru-pinmux.c
 *
 * Copyright (C) 2011 NVIDIA Corporation
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <mach/pinmux.h>
#include "board.h"
#include "board-endeavoru.h"
#include "gpio-names.h"
#include <mach/board_htc.h>

#define DEFAULT_DRIVE(_name)					\
	{							\
		.pingroup = TEGRA_DRIVE_PINGROUP_##_name,	\
		.hsm = TEGRA_HSM_DISABLE,			\
		.schmitt = TEGRA_SCHMITT_ENABLE,		\
		.drive = TEGRA_DRIVE_DIV_1,			\
		.pull_down = TEGRA_PULL_31,			\
		.pull_up = TEGRA_PULL_31,			\
		.slew_rising = TEGRA_SLEW_SLOWEST,		\
		.slew_falling = TEGRA_SLEW_SLOWEST,		\
	}
/* Setting the drive strength of pins
 * hsm: Enable High speed mode (ENABLE/DISABLE)
 * Schimit: Enable/disable schimit (ENABLE/DISABLE)
 * drive: low power mode (DIV_1, DIV_2, DIV_4, DIV_8)
 * pulldn_drive - drive down (falling edge) - Driver Output Pull-Down drive
 *                strength code. Value from 0 to 31.
 * pullup_drive - drive up (rising edge)  - Driver Output Pull-Up drive
 *                strength code. Value from 0 to 31.
 * pulldn_slew -  Driver Output Pull-Up slew control code  - 2bit code
 *                code 11 is least slewing of signal. code 00 is highest
 *                slewing of the signal.
 *                Value - FASTEST, FAST, SLOW, SLOWEST
 * pullup_slew -  Driver Output Pull-Down slew control code -
 *                code 11 is least slewing of signal. code 00 is highest
 *                slewing of the signal.
 *                Value - FASTEST, FAST, SLOW, SLOWEST
 */
#define SET_DRIVE(_name, _hsm, _schmitt, _drive, _pulldn_drive, _pullup_drive, _pulldn_slew, _pullup_slew) \
	{                                               \
		.pingroup = TEGRA_DRIVE_PINGROUP_##_name,   \
		.hsm = TEGRA_HSM_##_hsm,                    \
		.schmitt = TEGRA_SCHMITT_##_schmitt,        \
		.drive = TEGRA_DRIVE_##_drive,              \
		.pull_down = TEGRA_PULL_##_pulldn_drive,    \
		.pull_up = TEGRA_PULL_##_pullup_drive,		\
		.slew_rising = TEGRA_SLEW_##_pulldn_slew,   \
		.slew_falling = TEGRA_SLEW_##_pullup_slew,	\
	}

/* !!!FIXME!!!! POPULATE THIS TABLE */
static __initdata struct tegra_drive_pingroup_config endeavoru_drive_pinmux[] = {
	/* DEFAULT_DRIVE(<pin_group>), */
	/* SET_DRIVE(ATA, DISABLE, DISABLE, DIV_1, 31, 31, FAST, FAST) */
	SET_DRIVE(DAP2, 	DISABLE, ENABLE, DIV_1, 31, 31, FASTEST, FASTEST),
	SET_DRIVE(SPI, 	        DISABLE, ENABLE, DIV_1, 31, 31, FASTEST, FASTEST),

	/* All I2C pins are driven to maximum drive strength */
	/* GEN1 I2C */
	SET_DRIVE(DBG,		DISABLE, ENABLE, DIV_1, 31, 31, FASTEST, FASTEST),

	/* GEN2 I2C */
	SET_DRIVE(AT5,		DISABLE, ENABLE, DIV_1, 31, 31, FASTEST, FASTEST),

	/* CAM I2C */
	SET_DRIVE(GME,		DISABLE, ENABLE, DIV_1, 31, 31, FASTEST, FASTEST),

	/* DDC I2C */
	SET_DRIVE(DDC,		DISABLE, ENABLE, DIV_1, 31, 31, FASTEST, FASTEST),

	/* PWR_I2C */
	SET_DRIVE(AO1,		DISABLE, ENABLE, DIV_1, 31, 31, FASTEST, FASTEST),

	/* UART3 */
	/*SET_DRIVE(UART3,	DISABLE, ENABLE, DIV_1, 31, 31, FASTEST, FASTEST),*/
};

#define DEFAULT_PINMUX(_pingroup, _mux, _pupd, _tri, _io)	\
	{							\
		.pingroup	= TEGRA_PINGROUP_##_pingroup,	\
		.func		= TEGRA_MUX_##_mux,		\
		.pupd		= TEGRA_PUPD_##_pupd,		\
		.tristate	= TEGRA_TRI_##_tri,		\
		.io		= TEGRA_PIN_##_io,		\
		.lock		= TEGRA_PIN_LOCK_DEFAULT,	\
		.od		= TEGRA_PIN_OD_DEFAULT,		\
		.ioreset	= TEGRA_PIN_IO_RESET_DEFAULT,	\
	}

#define I2C_PINMUX(_pingroup, _mux, _pupd, _tri, _io, _lock, _od) \
	{							\
		.pingroup	= TEGRA_PINGROUP_##_pingroup,	\
		.func		= TEGRA_MUX_##_mux,		\
		.pupd		= TEGRA_PUPD_##_pupd,		\
		.tristate	= TEGRA_TRI_##_tri,		\
		.io		= TEGRA_PIN_##_io,		\
		.lock		= TEGRA_PIN_LOCK_##_lock,	\
		.od		= TEGRA_PIN_OD_##_od,		\
		.ioreset	= TEGRA_PIN_IO_RESET_DEFAULT,	\
	}

#define VI_PINMUX(_pingroup, _mux, _pupd, _tri, _io, _lock, _ioreset) \
	{							\
		.pingroup	= TEGRA_PINGROUP_##_pingroup,	\
		.func		= TEGRA_MUX_##_mux,		\
		.pupd		= TEGRA_PUPD_##_pupd,		\
		.tristate	= TEGRA_TRI_##_tri,		\
		.io		= TEGRA_PIN_##_io,		\
		.lock		= TEGRA_PIN_LOCK_##_lock,	\
		.od		= TEGRA_PIN_OD_DEFAULT,		\
		.ioreset	= TEGRA_PIN_IO_RESET_##_ioreset	\
	}

static __initdata struct tegra_pingroup_config endeavoru_pinmux_common[] = {
    // For A project
    //Port A
    DEFAULT_PINMUX(CLK_32K_OUT,     BLINK,           NORMAL,    NORMAL,     OUTPUT),                        //NC
    DEFAULT_PINMUX(UART3_CTS_N,     UARTC,           PULL_UP,   NORMAL,     INPUT),                         //BT_UART_CTS
    DEFAULT_PINMUX(DAP2_FS,         I2S1,            PULL_DOWN,    NORMAL,     INPUT),                         //AUD_AIC3008_I2S_LRCK
    DEFAULT_PINMUX(DAP2_SCLK,       I2S1,            PULL_DOWN,    NORMAL,     INPUT),                         //AUD_AIC3008_I2S_SCK
    DEFAULT_PINMUX(DAP2_DIN,        I2S1,            PULL_DOWN,    NORMAL,     INPUT),                         //AUD_AIC3008_I2S_DIN
    DEFAULT_PINMUX(DAP2_DOUT,       I2S1,            PULL_DOWN,    NORMAL,     INPUT),                         //AUD_AIC3008_I2S_DOUT
    DEFAULT_PINMUX(SDMMC3_CLK,      SDMMC3,          NORMAL,    NORMAL,     INPUT),                         //WIFI_SDIO_CLOCK
    DEFAULT_PINMUX(SDMMC3_CMD,      SDMMC3,          PULL_UP,   NORMAL,     INPUT),                         //WIFI_SDIO_COMMAND

    //Port B
    DEFAULT_PINMUX(GMI_A17,         UARTD,           NORMAL,    NORMAL,     INPUT),                         //MDM_IMC_UART_RX
    DEFAULT_PINMUX(GMI_A18,         UARTD,           NORMAL,    NORMAL,     INPUT),                         //MDM_IMC_UART_CTS
    DEFAULT_PINMUX(LCD_PWR0,        RSVD,            NORMAL,    NORMAL,     INPUT),                         //AUD_3V3_EN
    DEFAULT_PINMUX(LCD_PCLK,        DISPLAYA,        NORMAL,    NORMAL,     INPUT),                         //NC
    DEFAULT_PINMUX(SDMMC3_DAT3,     SDMMC3,          PULL_UP,   NORMAL,     INPUT),                         //WIFI_SDIO_DATA3
    DEFAULT_PINMUX(SDMMC3_DAT2,     SDMMC3,          PULL_UP,   NORMAL,     INPUT),                         //WIFI_SDIO_DATA2
    DEFAULT_PINMUX(SDMMC3_DAT1,     SDMMC3,          PULL_UP,   NORMAL,     INPUT),                         //WIFI_SDIO_DATA1
    DEFAULT_PINMUX(SDMMC3_DAT0,     SDMMC3,          PULL_UP,   NORMAL,     INPUT),                         //WIFI_SDIO_DATA0

    //Port C
    DEFAULT_PINMUX(UART3_RTS_N,     UARTC,           NORMAL,    NORMAL,     OUTPUT),                        //BT_UART_RTS
    DEFAULT_PINMUX(LCD_PWR1,        RSVD,            NORMAL,   NORMAL,     OUTPUT),                         //MDM_AP2BB_RST_PWRDWN#
    DEFAULT_PINMUX(UART2_TXD,       IRDA,            NORMAL,    NORMAL,     OUTPUT),                        //
    DEFAULT_PINMUX(UART2_RXD,       IRDA,            NORMAL,    NORMAL,     INPUT),                         //
    I2C_PINMUX(GEN1_I2C_SCL,        I2C1,        NORMAL,    NORMAL,     INPUT,  DISABLE, DISABLE),    //PER_SENSOR_I2C_SCL
    I2C_PINMUX(GEN1_I2C_SDA,    I2C1,        NORMAL,    NORMAL,     INPUT,  DISABLE, DISABLE),    //PER_SENSOR_I2C_SDA
    DEFAULT_PINMUX(LCD_PWR2,        DISPLAYA,         NORMAL, NORMAL,     OUTPUT),                         //MDM_AP2BB_SLAVE_WAKEUP_1
    DEFAULT_PINMUX(GMI_WP_N,        RSVD1,           PULL_UP,   NORMAL,     INPUT),                         //MHL_INT#

    //Port D
    DEFAULT_PINMUX(SDMMC3_DAT5,     SDMMC3,          PULL_UP,   NORMAL,   INPUT),                         //NC
    DEFAULT_PINMUX(SDMMC3_DAT4,     SDMMC3,          PULL_UP,   NORMAL,   INPUT),                         //NC
    DEFAULT_PINMUX(LCD_DC1,         RSVD,            NORMAL,    NORMAL,   INPUT),                         //AUD_A1V8_EN
    DEFAULT_PINMUX(SDMMC3_DAT6,     RSVD1,           NORMAL,    NORMAL,   INPUT),                         //NC
    DEFAULT_PINMUX(SDMMC3_DAT7,     RSVD1,           NORMAL,    NORMAL,   INPUT),                         //NC

    //Port E
//    DEFAULT_PINMUX(LCD_D0,          RSVD1,           PULL_DOWN, NORMAL,     OUTPUT),                        //MDM_SIM_INIT
    DEFAULT_PINMUX(LCD_D0,          DISPLAYA,        NORMAL,    NORMAL,     OUTPUT),                        //MHL_USB_SEL#
    DEFAULT_PINMUX(LCD_D1,          DISPLAYA,        NORMAL,    NORMAL,     INPUT),                         //NC
    DEFAULT_PINMUX(LCD_D2,          RSVD1,           NORMAL,    NORMAL,     OUTPUT),                        //PEH_CAP_INT
    DEFAULT_PINMUX(LCD_D3,          DISPLAYA,        NORMAL,    NORMAL,     INPUT),                         //NC
    DEFAULT_PINMUX(LCD_D4,          DISPLAYA,        NORMAL,    NORMAL,     INPUT),                         //MHL_1V2_EN
    DEFAULT_PINMUX(LCD_D5,          DISPLAYA,        NORMAL,    NORMAL,     INPUT),                         //DSP_LCM_1V8_EN
    DEFAULT_PINMUX(LCD_D6,          RSVD1,           NORMAL,    NORMAL,     INPUT),                         //MHL_RST#
    DEFAULT_PINMUX(LCD_D7,          DISPLAYA,        NORMAL,    NORMAL,     INPUT),                         //PEH_VIBRATOR_ON

    //Port F
    DEFAULT_PINMUX(LCD_D8,          RSVD,            PULL_DOWN, NORMAL,     INPUT),                         //CAM_VCM_2V85_PWR
    DEFAULT_PINMUX(LCD_D9,          DISPLAYA,        NORMAL,    NORMAL,     INPUT),                         //
    DEFAULT_PINMUX(LCD_D10,         DISPLAYA,        NORMAL,    NORMAL,   INPUT),                         //NC
    DEFAULT_PINMUX(LCD_D11,         DISPLAYA,        NORMAL,    NORMAL,   INPUT),                         //NC
    DEFAULT_PINMUX(LCD_D12,         DISPLAYA,        NORMAL,    NORMAL,   INPUT),                         //NC
    DEFAULT_PINMUX(LCD_D13,         DISPLAYA,        NORMAL,    NORMAL,   INPUT),                         //NC
    DEFAULT_PINMUX(LCD_D14,         RSVD,            NORMAL, NORMAL,     INPUT),                         //CAM_CAM2_CORE_1V8_EN
    DEFAULT_PINMUX(LCD_D15,         RSVD,        NORMAL,    NORMAL,     OUTPUT),                         //SYS_PMU_MSECURE

    //Port G
    DEFAULT_PINMUX(GMI_AD0,         RSVD,            NORMAL,    NORMAL,     INPUT),                         //SYS_BOOT_SEL_0
    DEFAULT_PINMUX(GMI_AD1,         RSVD,            NORMAL,    NORMAL,     INPUT),                         //SYS_BOOT_SEL_1
    DEFAULT_PINMUX(GMI_AD2,         RSVD,            NORMAL,    NORMAL,     INPUT),                         //SYS_BOOT_SEL_2
    DEFAULT_PINMUX(GMI_AD3,         RSVD,            NORMAL,    NORMAL,     INPUT),                         //SYS_BOOT_SEL_3
    DEFAULT_PINMUX(GMI_AD4,         RSVD,            NORMAL,    NORMAL,     INPUT),                         //SYS_RAM_CODE_0
    DEFAULT_PINMUX(GMI_AD5,         RSVD,            NORMAL,    NORMAL,     INPUT),                         //SYS_RAM_CODE_1
    DEFAULT_PINMUX(GMI_AD6,         RSVD,            NORMAL,    NORMAL,     INPUT),                         //SYS_RAM_CODE_2
    DEFAULT_PINMUX(GMI_AD7,         RSVD,            NORMAL,    NORMAL,     INPUT),                         //SYS_RAM_CODE_3

    //Port H
    DEFAULT_PINMUX(GMI_AD8,         PWM0,            NORMAL,    NORMAL ,   OUTPUT),                         //NC
    DEFAULT_PINMUX(GMI_AD9,         RSVD,            NORMAL,    NORMAL,   INPUT),                         //NC
    DEFAULT_PINMUX(GMI_AD10,        NAND,            NORMAL,    NORMAL,   OUTPUT),                         //NC
    DEFAULT_PINMUX(GMI_AD11,        RSVD,            NORMAL,    NORMAL,   INPUT),                         //NC
    DEFAULT_PINMUX(GMI_AD12,        RSVD,            NORMAL,    NORMAL,   INPUT),                         //NC
    DEFAULT_PINMUX(GMI_AD13,        RSVD,            NORMAL,    NORMAL,   INPUT),                         //NC
    DEFAULT_PINMUX(GMI_AD14,        RSVD,            NORMAL,    NORMAL,   INPUT),                         //DSP_TP_RST#
    DEFAULT_PINMUX(GMI_AD15,        NAND,            PULL_UP,   TRISTATE, INPUT),                         //NC

    //Port I
    DEFAULT_PINMUX(GMI_WR_N,        RSVD,            NORMAL,    NORMAL,     INPUT),                         //external pull high
    DEFAULT_PINMUX(GMI_OE_N,        RSVD,            NORMAL,    NORMAL,     INPUT),                         //external pull high
    DEFAULT_PINMUX(GMI_DQS,         RSVD,            NORMAL,    NORMAL,   INPUT),                         //NC
    DEFAULT_PINMUX(GMI_CS6_N,       RSVD,            NORMAL,    NORMAL,   INPUT),                         //NC
    DEFAULT_PINMUX(GMI_RST_N,       RSVD3,           PULL_UP,   TRISTATE,   INPUT),                         //NC
    //DEFAULT_PINMUX(GMI_IORDY,       RSVD1,           PULL_DOWN, NORMAL,     OUTPUT),                        //MDM_SIM_DET
    DEFAULT_PINMUX(GMI_IORDY,       RSVD1,           NORMAL,   TRISTATE,     INPUT),  //SIM_DET#  //should use this setting after EVM
    DEFAULT_PINMUX(GMI_CS7_N,       RSVD,            PULL_UP,    NORMAL,     INPUT),                         //PEH_GYR_INT
    DEFAULT_PINMUX(GMI_WAIT,        RSVD,            NORMAL,    NORMAL,   INPUT),                         //NC

    //Port J
    DEFAULT_PINMUX(GMI_CS0_N,       GMI,             NORMAL, NORMAL,     INPUT),                         //MDM_BB2AP_HOST_WAKEUP
    DEFAULT_PINMUX(LCD_DE,          DISPLAYA,        NORMAL, NORMAL,     INPUT),                         //DSP_LCM_DE
    DEFAULT_PINMUX(GMI_CS1_N,       RSVD,            PULL_DOWN, NORMAL,     INPUT),                         //PEH_COMP_INT
    DEFAULT_PINMUX(LCD_HSYNC,       DISPLAYA,        NORMAL,    NORMAL,     INPUT),             //NC
    DEFAULT_PINMUX(LCD_VSYNC,       DISPLAYA,        PULL_DOWN,    NORMAL,     INPUT),              //MDM_AP_USB_UART_OE#
    // Have been config in hboot, no need config in kernel
    //DEFAULT_PINMUX(UART2_CTS_N,     RSVD,        PULL_UP,    NORMAL,     INPUT),                //NC
    //DEFAULT_PINMUX(UART2_RTS_N,     RSVD,            NORMAL,    NORMAL,     INPUT),             //NC
    DEFAULT_PINMUX(GMI_A16,         UARTD,           NORMAL,      NORMAL,     OUTPUT), //MDM_TX

    //Port K
    DEFAULT_PINMUX(GMI_ADV_N,       RSVD,            NORMAL,    NORMAL,     INPUT),             //external pull down
    DEFAULT_PINMUX(GMI_CLK,         RSVD,            NORMAL,    NORMAL,     INPUT),             //NC
    DEFAULT_PINMUX(GMI_CS4_N,       RSVD,            PULL_UP,   NORMAL,     INPUT),             //PER_PS_INT#
    DEFAULT_PINMUX(GMI_CS2_N,       RSVD,            NORMAL,    NORMAL,     INPUT),             //NC
    DEFAULT_PINMUX(GMI_CS3_N,       RSVD,            NORMAL,    NORMAL,     INPUT),             //NC
    DEFAULT_PINMUX(SPDIF_OUT,       SPDIF,            NORMAL,    NORMAL,    OUTPUT),                //NC
    DEFAULT_PINMUX(SPDIF_IN,        SPDIF,            NORMAL,    NORMAL,     INPUT),                //NC
    DEFAULT_PINMUX(GMI_A19,         UARTD,           NORMAL,    NORMAL,     OUTPUT),            //MDM_RTS

    //Port L
    //Port M
    DEFAULT_PINMUX(LCD_D16,         DISPLAYA,        NORMAL,    NORMAL,     INPUT),             //DSP_LCD_ID0
    DEFAULT_PINMUX(LCD_D17,         DISPLAYA,        NORMAL,    NORMAL,     INPUT),             //DSP_LCD_ID1
    DEFAULT_PINMUX(LCD_D18,         RSVD,        NORMAL,    NORMAL,     INPUT),             //CAM_FRONT_CAM_RST#
    DEFAULT_PINMUX(LCD_D19,         DISPLAYA,        NORMAL,    NORMAL,     INPUT),             //NC
    DEFAULT_PINMUX(LCD_D20,         DISPLAYA,        NORMAL, NORMAL,     INPUT),             //MDM_V_DCIN_MODEM_EN
    DEFAULT_PINMUX(LCD_D21,         RSVD,            NORMAL,    NORMAL,     OUTPUT),            //NFC_VEN
    DEFAULT_PINMUX(LCD_D22,         RSVD,            NORMAL,    NORMAL,     OUTPUT),            //NFC_DL_MODE
    DEFAULT_PINMUX(LCD_D23,         RSVD,        NORMAL, NORMAL,     INPUT),             //CAM_V_CAM_VAA_2V85_EN

    //Port N
    DEFAULT_PINMUX(DAP1_FS,         I2S0,            NORMAL,    NORMAL,     OUTPUT),            //MDM_AP2BB_RST#
    DEFAULT_PINMUX(DAP1_DIN,        I2S0,            NORMAL,    NORMAL,     OUTPUT),              //MDM_AP2BB_HOST_ACTIVE
    DEFAULT_PINMUX(DAP1_DOUT,       I2S0,            NORMAL, 	NORMAL,     INPUT),             //MDM_BB_FATAL_INT
    DEFAULT_PINMUX(DAP1_SCLK,       I2S0,            NORMAL,    NORMAL,     OUTPUT),              //MDM_AP2BB_PWRON
    DEFAULT_PINMUX(LCD_CS0_N,       RSVD,            NORMAL,    NORMAL,     INPUT),             //NC
    DEFAULT_PINMUX(LCD_SDOUT,       DISPLAYA,        NORMAL,    NORMAL,     INPUT),             //NC
    DEFAULT_PINMUX(LCD_DC0,         DISPLAYA,        NORMAL,    NORMAL,     INPUT),             //DSP_LCD_RST#
    //DEFAULT_PINMUX(HDMI_INT,        RSVD,            PULL_UP,   TRISTATE,   INPUT),             //AUD_REMO_PRES#
    DEFAULT_PINMUX(HDMI_INT,        RSVD,            NORMAL,    TRISTATE,    INPUT),            //MHL_HPD

    //Port O
    DEFAULT_PINMUX(ULPI_DATA7,      RSVD,            NORMAL,    NORMAL,     INPUT),             //AP_USB_UART_SEL
    DEFAULT_PINMUX(ULPI_DATA0,      UARTA,           NORMAL,    NORMAL,     OUTPUT),            //BSP_AP_DEBUG_TX
    DEFAULT_PINMUX(ULPI_DATA1,      UARTA,           NORMAL,    NORMAL,     INPUT),             //BSP_AP_DEBUG_RX
    DEFAULT_PINMUX(ULPI_DATA2,      RSVD,            NORMAL,    NORMAL,     INPUT),             //NC
    DEFAULT_PINMUX(ULPI_DATA3,      RSVD1,           NORMAL,    NORMAL,     INPUT),             //CON_WIFI_IRQ
    DEFAULT_PINMUX(ULPI_DATA4,      ULPI,            PULL_DOWN, NORMAL,     INPUT),             //PER_GSENSOR_INT
    DEFAULT_PINMUX(ULPI_DATA5,      ULPI,            NORMAL,    NORMAL,     INPUT),             //NC
    DEFAULT_PINMUX(ULPI_DATA6,      ULPI,            NORMAL,    NORMAL,     INPUT),             //NC

    //Port P
    DEFAULT_PINMUX(DAP3_FS,         I2S2,            NORMAL,    NORMAL,     INPUT),             //AUD_AP_PCM_SYNC
    DEFAULT_PINMUX(DAP3_DIN,        I2S2,            NORMAL,    NORMAL,     INPUT),
    DEFAULT_PINMUX(DAP3_DOUT,       I2S2,            NORMAL,    NORMAL,     INPUT),             //AUD_AP_PCM_DOUT
    DEFAULT_PINMUX(DAP3_SCLK,       I2S2,            NORMAL,    NORMAL,     INPUT),
    DEFAULT_PINMUX(DAP4_FS,         I2S3,            PULL_DOWN,    NORMAL,     INPUT),             //AUD_BTPCM_SYNC
    DEFAULT_PINMUX(DAP4_DIN,        I2S3,            PULL_DOWN,    NORMAL,     INPUT),             //AUD_BTPCM_DOUT
    DEFAULT_PINMUX(DAP4_DOUT,       RSVD,            NORMAL,    NORMAL,     OUTPUT),             //AUD_SPK_EN
    DEFAULT_PINMUX(DAP4_SCLK,       RSVD,            NORMAL,    NORMAL,     OUTPUT),             //AUD_LINEOUT_EN

    //Port Q
    DEFAULT_PINMUX(KB_COL0,         KBC,             PULL_UP,    NORMAL,     INPUT),                //NC
    DEFAULT_PINMUX(KB_COL1,         KBC,             PULL_UP,    NORMAL,     INPUT),                //NC
    DEFAULT_PINMUX(KB_COL2,         KBC,             PULL_UP,    NORMAL,     INPUT),                //NC
    DEFAULT_PINMUX(KB_COL3,         KBC,             PULL_UP,    NORMAL,     INPUT),                //NC
    DEFAULT_PINMUX(KB_COL4,         KBC,             PULL_UP,    NORMAL,     INPUT),                //NC
    DEFAULT_PINMUX(KB_COL5,         KBC,             PULL_UP,    NORMAL,     INPUT),                //NC

    //Port R
    DEFAULT_PINMUX(KB_ROW0,         KBC,             PULL_UP,    NORMAL,     INPUT),                //NC
    DEFAULT_PINMUX(KB_ROW1,         KBC,             NORMAL,    NORMAL,     OUTPUT),                //PER_TORCH_EN
    DEFAULT_PINMUX(KB_ROW2,         RSVD,             NORMAL,    NORMAL,     OUTPUT),                //GYRO SLEEP
    DEFAULT_PINMUX(KB_ROW3,         RSVD,             NORMAL,    NORMAL,     OUTPUT),                //HAPTIC_EN
    DEFAULT_PINMUX(KB_ROW4,         RSVD,            NORMAL,    NORMAL,     INPUT),             //NC
    DEFAULT_PINMUX(KB_ROW5,         RSVD,            NORMAL,    NORMAL,     INPUT),             //NC
    DEFAULT_PINMUX(KB_ROW6,         RSVD,        NORMAL,    TRISTATE,     INPUT),             //CAM_CAM1_ID
    DEFAULT_PINMUX(KB_ROW7,         RSVD,        NORMAL,    TRISTATE,     INPUT),             //CAM_FRONT_CAM_ID

    //Port S
    DEFAULT_PINMUX(KB_ROW8,         KBC,            PULL_UP,   NORMAL,     INPUT),              //DSP_VOL_UP
    DEFAULT_PINMUX(KB_ROW9,         RSVD,            NORMAL,    NORMAL,     INPUT),             //NC
    DEFAULT_PINMUX(KB_ROW10,        RSVD,            NORMAL,    NORMAL,     INPUT),             //CON_USB_ID_1

    //Port T
    I2C_PINMUX(GEN2_I2C_SCL,    I2C2,        NORMAL,    NORMAL, INPUT,  DISABLE, DISABLE),   //DSP_TW_I2C_SCL
    I2C_PINMUX(GEN2_I2C_SDA,    I2C2,        NORMAL,    NORMAL, INPUT,  DISABLE, DISABLE),   //DSP_TW_I2C_SDA
    DEFAULT_PINMUX(SDMMC4_CMD,      SDMMC4,          PULL_UP,   NORMAL,     INPUT),             //PER_EMMC_CMD

    //Port U
    DEFAULT_PINMUX(GPIO_PU0,        RSVD,            NORMAL,    NORMAL,     INPUT),             //CON_BT_EN
    DEFAULT_PINMUX(GPIO_PU1,        RSVD,            NORMAL,    NORMAL,     INPUT),             //NC
    DEFAULT_PINMUX(GPIO_PU2,        RSVD,            NORMAL,    NORMAL,     INPUT),             //NC
    DEFAULT_PINMUX(GPIO_PU3,        RSVD,            NORMAL,    NORMAL,     INPUT),             //NC
    DEFAULT_PINMUX(GPIO_PU4,        RSVD1,           NORMAL,    NORMAL,     INPUT),             //PWR_MBAT_IN
    DEFAULT_PINMUX(GPIO_PU5,        RSVD1,       PULL_UP, NORMAL,     INPUT),             //PER_CAPSENSOR_INT_CPU
    DEFAULT_PINMUX(GPIO_PU6,        PWM3,            PULL_UP,   TRISTATE,     INPUT),               //DSP_AP_KPDPWR#

    //Port V
    DEFAULT_PINMUX(GPIO_PV0,        RSVD,            NORMAL, NORMAL,     INPUT),             //MDM_BB2AP_SUSPEND_REQ
    DEFAULT_PINMUX(GPIO_PV1,        RSVD,            PULL_UP,   NORMAL,     INPUT),             //DSP_TP_ATT#
    DEFAULT_PINMUX(GPIO_PV2,        RSVD1,           NORMAL,    NORMAL,     OUTPUT),            //CON_WIFI_EN
    DEFAULT_PINMUX(GPIO_PV3,        RSVD1,           NORMAL,    NORMAL,     OUTPUT),            //NC
    //I2C_PINMUX(DDC_SCL,           RSVD,        NORMAL, NORMAL, INPUT, DISABLE, ENABLE),       //NC
    //I2C_PINMUX(DDC_SDA,           RSVD,        NORMAL, NORMAL, INPUT, DISABLE, ENABLE),       //NC
    I2C_PINMUX(DDC_SCL,             I2C4,        NORMAL, NORMAL, INPUT, DISABLE, ENABLE),       //MHL_DDC_CLK
    I2C_PINMUX(DDC_SDA,             I2C4,        NORMAL, NORMAL, INPUT, DISABLE, ENABLE),       //MHL_DDC_DATA
    DEFAULT_PINMUX(CRT_VSYNC,       RSVD,             NORMAL,    NORMAL,     OUTPUT),

    //Port W
    DEFAULT_PINMUX(LCD_CS1_N,       RSVD1,            PULL_UP,   NORMAL,     INPUT),            //PWR_CHG_STAT
    DEFAULT_PINMUX(LCD_M1,          DISPLAYA,        NORMAL,    NORMAL,     OUTPUT),             //DSP_BL_PWM_CPU
    DEFAULT_PINMUX(SPI2_CS1_N,      RSVD,            PULL_UP,   NORMAL,     INPUT),             //AUD_HP_DET#
    DEFAULT_PINMUX(SPI2_CS2_N,      RSVD,            PULL_UP,   TRISTATE,     INPUT),               //DSP_VOL_DOWN
    DEFAULT_PINMUX(CLK1_OUT,        RSVD,      NORMAL,    NORMAL,     INPUT),             //AUD_MCLK
    DEFAULT_PINMUX(CLK2_OUT,        RSVD,        NORMAL,    NORMAL,     INPUT),             //AUD_AIC3008_RST#
    DEFAULT_PINMUX(UART3_TXD,       UARTC,           NORMAL,    NORMAL,     OUTPUT),            //CON_BT_TX
    DEFAULT_PINMUX(UART3_RXD,       UARTC,           PULL_UP,   NORMAL,     INPUT),             //CON_BT_RX

    //Port X
    DEFAULT_PINMUX(SPI2_MOSI,   SPI2,        NORMAL,    NORMAL, OUTPUT),             //AUD_SPI_DO
    DEFAULT_PINMUX(SPI2_MISO,   SPI2,        NORMAL,    NORMAL, INPUT),             //AUD_SPI_DI
    DEFAULT_PINMUX(SPI2_SCK,    SPI2,        NORMAL,    NORMAL, OUTPUT),             //AUD_SPI_CLK
    DEFAULT_PINMUX(SPI2_CS0_N,  SPI2,        NORMAL,    NORMAL, OUTPUT),             //AUD_SPI_CS#
    DEFAULT_PINMUX(SPI1_MOSI,       SPI1,            NORMAL,    NORMAL,     INPUT),             //NC
    DEFAULT_PINMUX(SPI1_SCK,        RSVD,            PULL_UP,   NORMAL,     INPUT),             //PWR_CHG_INT
    DEFAULT_PINMUX(SPI1_CS0_N,      SPI1,            NORMAL,    NORMAL,     INPUT),             //NC
    DEFAULT_PINMUX(SPI1_MISO,       RSVD,            NORMAL,    NORMAL,     OUTPUT),             //AUDIO_MCLK_EN

    //Port Y
    DEFAULT_PINMUX(ULPI_CLK,        RSVD,            NORMAL,    NORMAL,     OUTPUT),             //LED_DRV_EN
    DEFAULT_PINMUX(ULPI_DIR,        RSVD,            NORMAL,    NORMAL,     OUTPUT),             //LED_DRV_TRIG
    DEFAULT_PINMUX(ULPI_NXT,        ULPI,            NORMAL,    NORMAL,     OUTPUT),            //MHL_3V3_EN
    DEFAULT_PINMUX(ULPI_STP,        ULPI,            NORMAL,    NORMAL,     INPUT),             //PEH_V_SRIO_1V8_EN
    DEFAULT_PINMUX(SDMMC1_DAT1,     RSVD,            NORMAL,	TRISTATE,   INPUT),             //NFC_IRQ
    DEFAULT_PINMUX(SDMMC1_DAT3,     UARTE,           NORMAL,    NORMAL,     OUTPUT),            //AUD_REMO_TX
    DEFAULT_PINMUX(SDMMC1_DAT2,     UARTE,           NORMAL,    NORMAL,     INPUT),             //AUD_REMO_RX
    DEFAULT_PINMUX(SDMMC1_DAT0,     SDMMC1,          PULL_UP,   NORMAL,     INPUT),             //test point

    //Port Z
    DEFAULT_PINMUX(SDMMC1_CLK,      SDMMC1,          NORMAL,    NORMAL,     OUTPUT),             //AUD_REMO_OE#
    DEFAULT_PINMUX(SDMMC1_CMD,      SDMMC1,          PULL_UP,    NORMAL,     INPUT),                //test point
//    DEFAULT_PINMUX(LCD_SDIN,        DISPLAYA,        NORMAL,    NORMAL,     INPUT),         //MDM_USB_UART_OE#
    DEFAULT_PINMUX(LCD_WR_N,        DISPLAYA,        NORMAL,    NORMAL,     INPUT),             //NC
    DEFAULT_PINMUX(LCD_SCK,         RSVD,            PULL_UP,   TRISTATE,   OUTPUT),            //NC
    DEFAULT_PINMUX(SYS_CLK_REQ,     SYSCLK,          NORMAL,    NORMAL,     OUTPUT),            //SYS
    I2C_PINMUX(PWR_I2C_SCL,     I2CPWR,      NORMAL,    NORMAL, INPUT,  DISABLE,    ENABLE),    //SYS_PWR_I2C_SCL
    I2C_PINMUX(PWR_I2C_SDA,     I2CPWR,      NORMAL,    NORMAL, INPUT,  DISABLE,    ENABLE),    //SYS_PWR_I2C_SDA

    //Port AA
    DEFAULT_PINMUX(SDMMC4_DAT0,     SDMMC4,          PULL_UP,   NORMAL,     INPUT),             //BSP_EMMC_D0
    DEFAULT_PINMUX(SDMMC4_DAT1,     SDMMC4,          PULL_UP,   NORMAL,     INPUT),             //BSP_EMMC_D1
    DEFAULT_PINMUX(SDMMC4_DAT2,     SDMMC4,          PULL_UP,   NORMAL,     INPUT),             //BSP_EMMC_D2
    DEFAULT_PINMUX(SDMMC4_DAT3,     SDMMC4,          PULL_UP,   NORMAL,     INPUT),             //BSP_EMMC_D3
    DEFAULT_PINMUX(SDMMC4_DAT4,     SDMMC4,          PULL_UP,   NORMAL,     INPUT),             //BSP_EMMC_D4
    DEFAULT_PINMUX(SDMMC4_DAT5,     SDMMC4,          PULL_UP,   NORMAL,     INPUT),             //BSP_EMMC_D5
    DEFAULT_PINMUX(SDMMC4_DAT6,     SDMMC4,          PULL_UP,   NORMAL,     INPUT),             //BSP_EMMC_D6
    DEFAULT_PINMUX(SDMMC4_DAT7,     SDMMC4,          PULL_UP,   NORMAL,     INPUT),             //BSP_EMMC_D7

    //Port BB
    DEFAULT_PINMUX(GPIO_PBB0,       RSVD,        NORMAL,    NORMAL,     INPUT),             //CAM_CAM1_RST#
    I2C_PINMUX(CAM_I2C_SCL,         I2C3,    NORMAL,     NORMAL,    INPUT,    DISABLE,    DISABLE),      //CAM_I2C_SCL
    I2C_PINMUX(CAM_I2C_SDA,         I2C3,    NORMAL,     NORMAL,    INPUT,    DISABLE,    DISABLE),      //CAM_I2C_SDA
    DEFAULT_PINMUX(GPIO_PBB3,       VGP3,        NORMAL,    NORMAL,     OUTPUT),             //PER_FLASH_EN
    DEFAULT_PINMUX(GPIO_PBB4,       RSVD,        NORMAL, NORMAL,     INPUT),             //CAM_VCAM_VDDIO_1V8_EN
    DEFAULT_PINMUX(GPIO_PBB5,       RSVD,            NORMAL,    NORMAL,     INPUT),             //CAM_CAM1_VCM_PD
    DEFAULT_PINMUX(GPIO_PBB6,       RSVD,            NORMAL,    NORMAL,     INPUT),          //AUD_REMO_PRES#
    DEFAULT_PINMUX(GPIO_PBB7,       RSVD,            NORMAL,    NORMAL,     INPUT),             //CAM_FRONT_CAM_STANDBY

    //Port CC
    DEFAULT_PINMUX(CAM_MCLK,        VI_ALT2,     NORMAL,   NORMAL,    INPUT),              //CAM_MCLK
    DEFAULT_PINMUX(GPIO_PCC1,       RSVD,        NORMAL,    NORMAL,    INPUT),              //CAM_SEL
    DEFAULT_PINMUX(GPIO_PCC2,       RSVD2,           PULL_UP,   NORMAL,    INPUT),              //PWR_THEMP_ALERT_INT#
    DEFAULT_PINMUX(SDMMC4_RST_N,    RSVD1,			NORMAL, NORMAL,    INPUT),             //BSP_EMMC_RESOUT#
    DEFAULT_PINMUX(SDMMC4_CLK,      SDMMC4,          NORMAL,    NORMAL,    INPUT),              //BSP_EMMC_CLK
    DEFAULT_PINMUX(CLK2_REQ,        RSVD,             NORMAL,    NORMAL,    OUTPUT),              //AUD_DOCK_OUT_EN

    //Port DD
    //Port EE
    DEFAULT_PINMUX(CLK3_OUT,        EXTPERIPH3,  NORMAL,    NORMAL,     OUTPUT),            //NC
    DEFAULT_PINMUX(CLK3_REQ,        DEV3,            NORMAL,    NORMAL,     INPUT),             //NC
    DEFAULT_PINMUX(HDMI_CEC,        CEC,            NORMAL,    NORMAL,     INPUT),              //NC
};

static __initdata struct tegra_pingroup_config endeavoru_pinmux_EVM[] = {
    DEFAULT_PINMUX(LCD_D8,       RSVD,    PULL_DOWN,    NORMAL,    INPUT), // camera VCM power
	/* add SFIO config for specific board here */
};

static __initdata struct tegra_pingroup_config endeavoru_pinmux_EVT_XA[] = {
	/* add SFIO config for specific board here */
	DEFAULT_PINMUX(GMI_AD8,         PWM0,            NORMAL,    NORMAL ,   OUTPUT), //HAPTIC_PWM_XA
};

static __initdata struct tegra_pingroup_config endeavoru_pinmux_EVT_XB[] = {
	/* add SFIO config for specific board here */
	DEFAULT_PINMUX(UART2_TXD,       SPI4,            NORMAL,    NORMAL,     INPUT),  //MAMCAM_SPI_CLK
	DEFAULT_PINMUX(UART2_RXD,       SPI4,            NORMAL,    NORMAL,     INPUT),     //MAMCAM_SPI_DO
	DEFAULT_PINMUX(UART2_RTS_N,       SPI4,            NORMAL,    NORMAL,     INPUT),     //MCAM_SPI_DI
	DEFAULT_PINMUX(UART2_CTS_N,       SPI4,            NORMAL,    NORMAL,     INPUT),     //MCAM_SPI_CS0
	DEFAULT_PINMUX(KB_ROW0,       RSVD,            NORMAL,    TRISTATE,     INPUT),     //RAW_INTR0(input only)
	DEFAULT_PINMUX(KB_ROW3,       RSVD,            NORMAL, NORMAL,     INPUT),     //RAW_1V8_EN
	DEFAULT_PINMUX(KB_ROW4,       RSVD,            NORMAL,    NORMAL,     INPUT),     //RAW_RSTN
	DEFAULT_PINMUX(KB_ROW5,       RSVD,            NORMAL, NORMAL,     INPUT),     //RAW_1V2_EN
	DEFAULT_PINMUX(CLK3_REQ,       RSVD,            NORMAL,    TRISTATE,     INPUT),     //RAW_INTR1(input only)
	DEFAULT_PINMUX(GMI_AD8,         PWM0,            NORMAL,    NORMAL ,   OUTPUT), //HAPTIC_PWM_XB
};

static __initdata struct tegra_pingroup_config endeavoru_pinmux_EVT_XC[] = {
	/* add SFIO config for specific board here */
	DEFAULT_PINMUX(GMI_AD8,         PWM0,            NORMAL,    NORMAL ,   OUTPUT), //HAPTIC_PWM_XC

	/* clone from endeavoru_pinmux_EVT_XB */
	DEFAULT_PINMUX(UART2_TXD,       SPI4,            NORMAL,    NORMAL,     INPUT),  //MAMCAM_SPI_CLK
	DEFAULT_PINMUX(UART2_RXD,       SPI4,            NORMAL,    NORMAL,     INPUT),     //MAMCAM_SPI_DO
	DEFAULT_PINMUX(UART2_RTS_N,       SPI4,            NORMAL,    NORMAL,     INPUT),     //MCAM_SPI_DI
	DEFAULT_PINMUX(UART2_CTS_N,       SPI4,            NORMAL,    NORMAL,     INPUT),     //MCAM_SPI_CS0
	DEFAULT_PINMUX(KB_ROW0,       RSVD,            NORMAL,    TRISTATE,     INPUT),     //RAW_INTR0(input only)
	DEFAULT_PINMUX(KB_ROW3,       RSVD,            NORMAL, NORMAL,     INPUT),     //RAW_1V8_EN
	DEFAULT_PINMUX(KB_ROW4,       RSVD,            NORMAL,    NORMAL,     INPUT),     //RAW_RSTN
	DEFAULT_PINMUX(KB_ROW5,       RSVD,            NORMAL, NORMAL,     INPUT),     //RAW_1V2_EN
	DEFAULT_PINMUX(CLK3_REQ,       RSVD,            NORMAL,    TRISTATE,     INPUT),     //RAW_INTR1(input only)
   DEFAULT_PINMUX(ULPI_DATA4,     RSVD,           NORMAL,   NORMAL,     INPUT),   //BT UART CTS wake up source
};

static __initdata struct tegra_pingroup_config endeavoru_pinmux_EVT_XD[] = {
	/* add SFIO config for specific board here */
	DEFAULT_PINMUX(GMI_AD8,         PWM0,            NORMAL,    NORMAL ,   OUTPUT), //HAPTIC_PWM_XC

	/* clone from endeavoru_pinmux_EVT_XB */
	DEFAULT_PINMUX(UART2_TXD,       SPI4,            NORMAL,    NORMAL,     INPUT),  //MAMCAM_SPI_CLK
	DEFAULT_PINMUX(UART2_RXD,       SPI4,            NORMAL,    NORMAL,     INPUT),     //MAMCAM_SPI_DO
	DEFAULT_PINMUX(UART2_RTS_N,       SPI4,            NORMAL,    NORMAL,     INPUT),     //MCAM_SPI_DI
	DEFAULT_PINMUX(UART2_CTS_N,       SPI4,            NORMAL,    NORMAL,     INPUT),     //MCAM_SPI_CS0
	DEFAULT_PINMUX(KB_ROW0,       RSVD,            NORMAL,    TRISTATE,     INPUT),     //RAW_INTR0(input only)
	DEFAULT_PINMUX(KB_ROW3,       RSVD,            NORMAL, NORMAL,     INPUT),     //RAW_1V8_EN
	DEFAULT_PINMUX(KB_ROW4,       RSVD,            NORMAL,    NORMAL,     INPUT),     //RAW_RSTN
	DEFAULT_PINMUX(KB_ROW5,       RSVD,            NORMAL, NORMAL,     INPUT),     //RAW_1V2_EN
	DEFAULT_PINMUX(CLK3_REQ,       RSVD,            NORMAL,    TRISTATE,     INPUT),     //RAW_INTR1(input only)
   DEFAULT_PINMUX(ULPI_DATA4,     RSVD,           NORMAL,   NORMAL,     INPUT),   //BT UART CTS wake up source

	/* PLSensor sdmmc_2v85_en */
	DEFAULT_PINMUX(LCD_D19,  RSVD, NORMAL, NORMAL, OUTPUT),
};

static __initdata struct tegra_pingroup_config endeavoru_pinmux_EVT_XE[] = {
	DEFAULT_PINMUX(GMI_AD8,         PWM0,            NORMAL,    NORMAL ,   OUTPUT), //HAPTIC_PWM_XC
	DEFAULT_PINMUX(UART2_TXD,       SPI4,            NORMAL,    NORMAL,     INPUT),  //MAMCAM_SPI_CLK
	DEFAULT_PINMUX(UART2_RXD,       SPI4,            NORMAL,    NORMAL,     INPUT),     //MAMCAM_SPI_DO
	DEFAULT_PINMUX(UART2_RTS_N,       SPI4,            NORMAL,    NORMAL,     INPUT),     //MCAM_SPI_DI
	DEFAULT_PINMUX(UART2_CTS_N,       SPI4,            NORMAL,    NORMAL,     INPUT),     //MCAM_SPI_CS0
	DEFAULT_PINMUX(KB_ROW0,       RSVD,            NORMAL,    TRISTATE,     INPUT),     //RAW_INTR0(input only)
	DEFAULT_PINMUX(KB_ROW3,       RSVD,            NORMAL, NORMAL,     INPUT),     //RAW_1V8_EN
	DEFAULT_PINMUX(KB_ROW4,       RSVD,            NORMAL,    NORMAL,     INPUT),     //RAW_RSTN
	DEFAULT_PINMUX(KB_ROW5,       RSVD,            NORMAL, NORMAL,     OUTPUT),     //NC
	DEFAULT_PINMUX(CLK3_REQ,       RSVD,            NORMAL,    TRISTATE,     INPUT),     //RAW_INTR1(input only)
	DEFAULT_PINMUX(ULPI_DATA4,     RSVD,           NORMAL,   NORMAL,     INPUT),   //BT UART CTS wake up source

	/* PLSensor sdmmc_2v85_en */
	DEFAULT_PINMUX(LCD_D19,  RSVD, NORMAL, NORMAL, OUTPUT),
};

int __init endeavoru_pinmux_init(void)
{
	int board_id;
	tegra_pinmux_config_table(endeavoru_pinmux_common, ARRAY_SIZE(endeavoru_pinmux_common));
	tegra_drive_pinmux_config_table(endeavoru_drive_pinmux,
					ARRAY_SIZE(endeavoru_drive_pinmux));

	board_id = htc_get_pcbid_info();
	if (board_id == PROJECT_PHASE_EVM) // EVM
		tegra_pinmux_config_table(endeavoru_pinmux_EVM,
			ARRAY_SIZE(endeavoru_pinmux_EVM));
	else if (board_id == PROJECT_PHASE_XA) // EVT XA
		tegra_pinmux_config_table(endeavoru_pinmux_EVT_XA,
			ARRAY_SIZE(endeavoru_pinmux_EVT_XA));
	else if (board_id == PROJECT_PHASE_XB) // EVT XB
		tegra_pinmux_config_table(endeavoru_pinmux_EVT_XB,
			ARRAY_SIZE(endeavoru_pinmux_EVT_XB));
	else if (board_id == PROJECT_PHASE_XC) // XC
		tegra_pinmux_config_table(endeavoru_pinmux_EVT_XC,
			ARRAY_SIZE(endeavoru_pinmux_EVT_XC));
	else if (board_id == PROJECT_PHASE_XD) // XD
		tegra_pinmux_config_table(endeavoru_pinmux_EVT_XD,
			ARRAY_SIZE(endeavoru_pinmux_EVT_XD));
	else // latest project phase
		tegra_pinmux_config_table(endeavoru_pinmux_EVT_XE,
			ARRAY_SIZE(endeavoru_pinmux_EVT_XE));

	return 0;
}
