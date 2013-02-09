/*
 * linux/arch/arm/mach-tegra/include/mach/board_htc.c
 *
 * Copyright (C) 2012 HTC Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef __ARCH_ARM_MACH_TEGRA_BOARD_HTC_H
#define __ARCH_ARM_MACH_TEGRA_BOARD_HTC_H

#include <linux/types.h>
#include <linux/list.h>
#include <asm/setup.h>

#define SHIP_BUILD  0
#define MFG_BUILD   1
#define ENG_BUILD   2

enum {
	BOARD_MFG_MODE_NORMAL = 0,
	BOARD_MFG_MODE_FACTORY2,
	BOARD_MFG_MODE_RECOVERY,
	BOARD_MFG_MODE_CHARGE,
	BOARD_MFG_MODE_POWERTEST,
	BOARD_MFG_MODE_OFFMODE_CHARGING,
	BOARD_MFG_MODE_MFGKERNEL,
	BOARD_MFG_MODE_MODEM_CALIBRATION,
};

/* HTC_START_Simon.Ti_Liu_20120702_Enhance_bypass */
enum rawchip_enable_type {
	RAWCHIP_DISABLE,
	RAWCHIP_ENABLE,
	RAWCHIP_DXO_BYPASS,
	RAWCHIP_MIPI_BYPASS,
};
/* HTC_END*/

int board_mfg_mode(void);
int board_zchg_mode(void);
void htc_gpio_set_diag_gpio_table(unsigned char* dwMFG_gpio_table);

int __init board_mcp_monodie(void);
int __init parse_tag_smi(const struct tag *tags);
int __init parse_tag_hwid(const struct tag *tags);
int __init parse_tag_monodie(const struct tag *tags);

int board_get_sku_tag(void);
void board_get_keycaps_tag(char **);
void board_get_cid_tag(char **);
void board_get_carrier_tag(char **);
void board_get_mid_tag(char **);
int board_emmc_boot(void);

void notify_usb_connected(int online);

char *board_serialno(void);

unsigned int get_kernel_flag(void);
unsigned int get_extra_kernel_flag(void);
unsigned int get_bl_ac_in_flag(void);
/*
 * Obviously, we need these in all project.
 * To export a function to get these is too lousy.
 * Each BSP can include board.h to get these.
 *
 * Jay, 15/May/09'
 * */
extern int panel_type;
extern unsigned engineer_id;
extern int usb_phy_error;

extern unsigned long tegra_bootloader_panel_lsb;
extern unsigned long tegra_bootloader_panel_msb;

extern const int htc_get_pcbid_info(void);

enum {
	PROJECT_PHASE_INVALID = -2,
	PROJECT_PHASE_EVM =  0x99,
	PROJECT_PHASE_XA  =  0,
	PROJECT_PHASE_XB  =  1,
	PROJECT_PHASE_XC  =  2,
	PROJECT_PHASE_XD  =  3,
	PROJECT_PHASE_XE  =  4,
	PROJECT_PHASE_XF  =  5,
	PROJECT_PHASE_XG  =  6,
	PROJECT_PHASE_XH  =  7,
	PROJECT_PHASE_A   =  0x80,
	PROJECT_PHASE_B   =  0x81,
	PROJECT_PHASE_C   =  0x82,
	PROJECT_PHASE_D   =  0x83,
	PROJECT_PHASE_E   =  0x84,
	PROJECT_PHASE_F   =  0x85,
	PROJECT_PHASE_G   =  0x86,
	PROJECT_PHASE_H   =  0x87,
	PROJECT_PHASE_LATEST = 0xFF,
};

enum {
	KERNEL_FLAG_WATCHDOG_ENABLE = BIT(0),
	KERNEL_FLAG_SERIAL_HSL_ENABLE = BIT(1),
	KERNEL_FLAG_KEEP_CHARG_ON = BIT(2),
	KERNEL_FLAG_APPSBARK = BIT(3),
	KERNEL_FLAG_RESERVED_4 = BIT(4),
	KERNEL_FLAG_RESERVED_5 = BIT(5),
	KERNEL_FLAG_RESERVED_6 = BIT(6),
	KERNEL_FLAG_RESERVED_7 = BIT(7),
	KERNEL_FLAG_PVS_SLOW_CPU = BIT(8),
	KERNEL_FLAG_PVS_NOM_CPU = BIT(9),
	KERNEL_FLAG_PVS_FAST_CPU = BIT(10),
	KERNEL_FLAG_RESERVED_11 = BIT(11),
	KERNEL_FLAG_RESERVED_12 = BIT(12),
	KERNEL_FLAG_DISABLE_WAKELOCK = BIT(13),
	KERNEL_FLAG_PA_RECHARG_TEST = BIT(14),
	KERNEL_FLAG_TEST_PWR_SUPPLY = BIT(15),
	KERNEL_FLAG_RESERVED_16 = BIT(16),
	KERNEL_FLAG_RIL_DBG_RMNET = BIT(17),
	KERNEL_FLAG_RESERVED_18 = BIT(18),
	KERNEL_FLAG_RESERVED_19 = BIT(19),
	KERNEL_FLAG_RESERVED_20 = BIT(20),
	KERNEL_FLAG_RESERVED_21 = BIT(21),
	KERNEL_FLAG_RESERVED_22 = BIT(22),
	KERNEL_FLAG_RESERVED_23 = BIT(23),
	KERNEL_FLAG_RESERVED_24 = BIT(24),
	KERNEL_FLAG_PM_MONITOR = BIT(25),
	KERNEL_FLAG_RESERVED_26 = BIT(26),
	KERNEL_FLAG_RESERVED_27 = BIT(27),
	KERNEL_FLAG_RESERVED_28 = BIT(28),
	KERNEL_FLAG_ENABLE_BMS_CHARGER_LOG = BIT(29),
	KERNEL_FLAG_RESERVED_30 = BIT(30),
	KERNEL_FLAG_GPIO_DUMP = BIT(31),
};

enum {
	RADIO_FLAG_USB_UPLOAD = BIT(3),
};

unsigned int get_radio_flag(void);

#endif
