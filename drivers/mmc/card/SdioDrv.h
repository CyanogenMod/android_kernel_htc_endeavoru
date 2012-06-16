/***************************************************************************
**+----------------------------------------------------------------------+**
**|                                ****                                  |**
**|                                ****                                  |**
**|                                ******o***                            |**
**|                          ********_///_****                           |**
**|                           ***** /_//_/ ****                          |**
**|                            ** ** (__/ ****                           |**
**|                                *********                             |**
**|                                 ****                                 |**
**|                                  ***                                 |**
**|                                                                      |**
**|     Copyright (c) 1998 - 2008 Texas Instruments Incorporated         |**
**|                        ALL RIGHTS RESERVED                           |**
**|                                                                      |**
**| Permission is hereby granted to licensees of Texas Instruments       |**
**| Incorporated (TI) products to use this computer program for the sole |**
**| purpose of implementing a licensee product based on TI products.     |**
**| No other rights to reproduce, use, or disseminate this computer      |**
**| program, whether in part or in whole, are granted.                   |**
**|                                                                      |**
**| TI makes no representation or warranties with respect to the         |**
**| performance of this computer program, and specifically disclaims     |**
**| any responsibility for any damages, special or consequential,        |**
**| connected with the use of this program.                              |**
**|                                                                      |**
**+----------------------------------------------------------------------+**
***************************************************************************/

#ifndef __MSM_SDIODRV_API_H
#define __MSM_SDIODRV_API_H

#include <asm/types.h>
#include <linux/mmc/mmc.h>


/*#define SDIO_1_BIT*/
#define CLK_RATE 5000000

#ifdef SDIO_1_BIT
#define TIWLAN_SDIO_BUSWIDE MMC_BUS_WIDTH_1
#define SDIO_BITS_CODE   0x80 /* 1 bits */
#else
#define TIWLAN_SDIO_BUSWIDE MMC_BUS_WIDTH_4
#define SDIO_BITS_CODE   0x82 /* 4 bits */
#endif

#define MAX_RETRIES                 10

/* For block mode configuration */
#define FN0_FBR2_REG_108                    0x210
#define FN0_FBR2_REG_108_BIT_MASK           0xFFF

#define TIWLAN_SDIO_INITIAL_CLOCK (400 * 1000)
#define TIWLAN_SDIO_CLOCK (25 * 1000 * 1000)

/* Card Common Control Registers (CCCR) */

#define CCCR_SDIO_REVISION                  0x00
#define CCCR_SD_SPECIFICATION_REVISION      0x01
#define CCCR_IO_ENABLE                      0x02
#define CCCR_IO_READY                       0x03
#define CCCR_INT_ENABLE                     0x04
#define CCCR_INT_PENDING                    0x05
#define CCCR_IO_ABORT                       0x06
#define CCCR_BUS_INTERFACE_CONTOROL         0x07
#define CCCR_CARD_CAPABILITY	            0x08
#define CCCR_COMMON_CIS_POINTER             0x09 /*0x09-0x0B*/
#define CCCR_FNO_BLOCK_SIZE	                0x10 /*0x10-0x11*/
#define FN0_CCCR_REG_32	                    0x64


/* Pprotocol defined constants */

#define SD_IO_GO_IDLE_STATE		  		    0
#define SD_IO_SEND_RELATIVE_ADDR	  	    3
#define SDIO_CMD5			  			    5
#define SD_IO_SELECT_CARD		  		    7
#define SDIO_CMD52		 	 			    52
#define SDIO_CMD53		 	 			    53
#define SDIO_SHIFT(v, n)                     (v<<n)
#define SDIO_RWFLAG(v)                      (SDIO_SHIFT(v, 31))
#define SDIO_FUNCN(v)                       (SDIO_SHIFT(v, 28))
#define SDIO_RAWFLAG(v)                     (SDIO_SHIFT(v, 27))
#define SDIO_BLKM(v)                        (SDIO_SHIFT(v, 27))
#define SDIO_OPCODE(v)                      (SDIO_SHIFT(v, 26))
#define SDIO_ADDRREG(v)                     (SDIO_SHIFT(v, 9))


#define VDD_VOLTAGE_WINDOW                  0xffffc0

#define MMC_RSP_PRESENT (1 << 0)
#define MMC_RSP_136     (1 << 1)                /* 136 bit response */
#define MMC_RSP_CRC     (1 << 2)                /* expect valid crc */
#define MMC_RSP_BUSY    (1 << 3)                /* card may send busy */
#define MMC_RSP_OPCODE  (1 << 4)                /* response contains opcode */
#define MMC_CMD_AC      (0 << 5)      			/* Addressed Command, no data */

#define MMC_RSP_SPI_S1  (1 << 7)                /* one status byte */



#define MMC_RSP_NONE    (0)
#define MMC_RSP_R1      (MMC_RSP_PRESENT|MMC_RSP_CRC|MMC_RSP_OPCODE)
#define MMC_RSP_R1b	(MMC_RSP_PRESENT|MMC_RSP_CRC|MMC_RSP_OPCODE| \
			MMC_RSP_BUSY)
#define MMC_RSP_R2      (MMC_RSP_PRESENT|MMC_RSP_136|MMC_RSP_CRC)
#define MMC_RSP_R3      (MMC_RSP_PRESENT)
#define MMC_RSP_R4      (MMC_RSP_PRESENT)
#define MMC_RSP_R5      (MMC_RSP_PRESENT|MMC_RSP_CRC|MMC_RSP_OPCODE)
#define MMC_RSP_R6      (MMC_RSP_PRESENT|MMC_RSP_CRC|MMC_RSP_OPCODE)
#define MMC_RSP_R7      (MMC_RSP_PRESENT|MMC_RSP_CRC|MMC_RSP_OPCODE)
/********************************************************************/
/*	SDIO driver functions prototypes			    */
/********************************************************************/
int sdioDrv_ConnectBus(void *fCbFunc,
			void *hCbArg,
			unsigned int uBlkSizeShift,
			unsigned int uSdioThreadPriority);

int sdioDrv_DisconnectBus(void);

int sdioDrv_ExecuteCmd(unsigned int uCmd,
			unsigned int uArg,
			unsigned int uRespType,
			void *pResponse,
			unsigned int uLen);

int sdioDrv_ReadSync(unsigned int uFunc,
			unsigned int uHwAddr,
			void *pData,
			unsigned int uLen,
			unsigned int bIncAddr,
			unsigned int bMore);

int sdioDrv_ReadAsync(unsigned int uFunc,
			unsigned int uHwAddr,
			void *pData,
			unsigned int uLen,
			unsigned int bBlkMode,
			unsigned int bIncAddr,
			unsigned int bMore);

int sdioDrv_WriteSync(unsigned int uFunc,
			unsigned int uHwAddr,
			void *pData,
			unsigned int uLen,
			unsigned int bIncAddr,
			unsigned int bMore);

int sdioDrv_WriteAsync(unsigned int uFunc,
			unsigned int uHwAddr,
			void *pData,
			unsigned int uLen,
			unsigned int bBlkMode,
			unsigned int bIncAddr,
			unsigned int bMore);

int sdioDrv_ReadSyncBytes(unsigned int  uFunc,
			unsigned int  uHwAddr,
			unsigned char *pData,
			unsigned int  uLen,
			unsigned int  bMore);

int sdioDrv_WriteSyncBytes(unsigned int  uFunc,
			unsigned int  uHwAddr,
			unsigned char *pData,
			unsigned int  uLen,
			unsigned int bMore);

#ifndef SDIO_KERNEL_MODULE
int  sdioDrv_init(void);
void  sdioDrv_exit(void);
#endif

int  sdioDrv_clk_enable(void);
void sdioDrv_clk_disable(void);
/*void sdioDrv_start_inact_timer(void);  //austin disabled first, beacuse we will insmod fail(unknow symbol sdioDrv_start_inact_timer)
void sdioDrv_cancel_inact_timer(void); //austin disabled first, beacuse we will insmod fail(unknow symbol sdioDrv_cancel_inact_timer)*/

#define SDIO_MAX_PACKET_LEN 512
#define INCREMENT_REG_ADDR	1
#define FIX_REG_ADDR		0

#define TXN_FUNC_ID_CTRL         0
#define TXN_FUNC_ID_BT           1
#define TXN_FUNC_ID_WLAN         2

#define WLAN_EN			170

typedef unsigned long pin_cfg_t;

#define NMK_GPIO_ALT_GPIO	0
#define NMK_GPIO_ALT_A	1
#define NMK_GPIO_ALT_B	2
#define NMK_GPIO_ALT_C	(NMK_GPIO_ALT_A | NMK_GPIO_ALT_B)

/*enum nmk_gpio_pull {
	NMK_GPIO_PULL_NONE,
	NMK_GPIO_PULL_UP,
	NMK_GPIO_PULL_DOWN,
};*/

#define PIN_NUM_MASK		0x1ff
#define PIN_NUM(x)		((x) & PIN_NUM_MASK)

#define PIN_ALT_SHIFT		9
#define PIN_ALT_MASK		(0x3 << PIN_ALT_SHIFT)
#define PIN_ALT(x)		(((x) & PIN_ALT_MASK) >> PIN_ALT_SHIFT)
#define PIN_GPIO		(NMK_GPIO_ALT_GPIO << PIN_ALT_SHIFT)
#define PIN_ALT_A		(NMK_GPIO_ALT_A << PIN_ALT_SHIFT)
#define PIN_ALT_B		(NMK_GPIO_ALT_B << PIN_ALT_SHIFT)
#define PIN_ALT_C		(NMK_GPIO_ALT_C << PIN_ALT_SHIFT)

#define PIN_PULL_SHIFT		11
#define PIN_PULL_MASK		(0x3 << PIN_PULL_SHIFT)
#define PIN_PULL(x)		(((x) & PIN_PULL_MASK) >> PIN_PULL_SHIFT)
#define PIN_PULL_NONE		(NMK_GPIO_PULL_NONE << PIN_PULL_SHIFT)
#define PIN_PULL_UP		(1/*NMK_GPIO_PULL_UP*/ << PIN_PULL_SHIFT)
#define PIN_PULL_DOWN		(NMK_GPIO_PULL_DOWN << PIN_PULL_SHIFT)

#define PIN_SLPM_SHIFT		13
#define PIN_SLPM_MASK		(0x1 << PIN_SLPM_SHIFT)
#define PIN_SLPM(x)		(((x) & PIN_SLPM_MASK) >> PIN_SLPM_SHIFT)
#define PIN_SLPM_MAKE_INPUT	(NMK_GPIO_SLPM_INPUT << PIN_SLPM_SHIFT)
#define PIN_SLPM_NOCHANGE	(NMK_GPIO_SLPM_NOCHANGE << PIN_SLPM_SHIFT)
/* These two replace the above in DB8500v2+ */
#define PIN_SLPM_WAKEUP_ENABLE	(NMK_GPIO_SLPM_WAKEUP_ENABLE << PIN_SLPM_SHIFT)
#define PIN_SLPM_WAKEUP_DISABLE	(NMK_GPIO_SLPM_WAKEUP_DISABLE << PIN_SLPM_SHIFT)

#define PIN_DIR_SHIFT		14
#define PIN_DIR_MASK		(0x1 << PIN_DIR_SHIFT)
#define PIN_DIR(x)		(((x) & PIN_DIR_MASK) >> PIN_DIR_SHIFT)
#define PIN_DIR_INPUT		(0 << PIN_DIR_SHIFT)
#define PIN_DIR_OUTPUT		(1 << PIN_DIR_SHIFT)

#define PIN_VAL_SHIFT		15
#define PIN_VAL_MASK		(0x1 << PIN_VAL_SHIFT)
#define PIN_VAL(x)		(((x) & PIN_VAL_MASK) >> PIN_VAL_SHIFT)
#define PIN_VAL_LOW		(0 << PIN_VAL_SHIFT)
#define PIN_VAL_HIGH		(1 << PIN_VAL_SHIFT)

#define PIN_SLPM_PULL_SHIFT	16
#define PIN_SLPM_PULL_MASK	(0x7 << PIN_SLPM_PULL_SHIFT)
#define PIN_SLPM_PULL(x)	\
	(((x) & PIN_SLPM_PULL_MASK) >> PIN_SLPM_PULL_SHIFT)
#define PIN_SLPM_PULL_NONE	\
	((1 + NMK_GPIO_PULL_NONE) << PIN_SLPM_PULL_SHIFT)
#define PIN_SLPM_PULL_UP	\
	((1 + 1/*NMK_GPIO_PULL_UP*/) << PIN_SLPM_PULL_SHIFT)
#define PIN_SLPM_PULL_DOWN	\
	((1 + NMK_GPIO_PULL_DOWN) << PIN_SLPM_PULL_SHIFT)

#define PIN_SLPM_DIR_SHIFT	19
#define PIN_SLPM_DIR_MASK	(0x3 << PIN_SLPM_DIR_SHIFT)
#define PIN_SLPM_DIR(x)		\
	(((x) & PIN_SLPM_DIR_MASK) >> PIN_SLPM_DIR_SHIFT)
#define PIN_SLPM_DIR_INPUT	((1 + 0) << PIN_SLPM_DIR_SHIFT)
#define PIN_SLPM_DIR_OUTPUT	((1 + 1) << PIN_SLPM_DIR_SHIFT)

#define PIN_SLPM_VAL_SHIFT	21
#define PIN_SLPM_VAL_MASK	(0x3 << PIN_SLPM_VAL_SHIFT)
#define PIN_SLPM_VAL(x)		\
	(((x) & PIN_SLPM_VAL_MASK) >> PIN_SLPM_VAL_SHIFT)
#define PIN_SLPM_VAL_LOW	((1 + 0) << PIN_SLPM_VAL_SHIFT)
#define PIN_SLPM_VAL_HIGH	((1 + 1) << PIN_SLPM_VAL_SHIFT)

/* Shortcuts.  Use these instead of separate DIR, PULL, and VAL.  */
#define PIN_INPUT_PULLDOWN	(PIN_DIR_INPUT | PIN_PULL_DOWN)
#define PIN_INPUT_PULLUP	(PIN_DIR_INPUT | PIN_PULL_UP)
#define PIN_INPUT_NOPULL	(PIN_DIR_INPUT | 0/*PIN_PULL_NONE*/)
#define PIN_OUTPUT_LOW		(PIN_DIR_OUTPUT | PIN_VAL_LOW)
#define PIN_OUTPUT_HIGH		(PIN_DIR_OUTPUT | PIN_VAL_HIGH)

#define PIN_SLPM_INPUT_PULLDOWN	(PIN_SLPM_DIR_INPUT | PIN_SLPM_PULL_DOWN)
#define PIN_SLPM_INPUT_PULLUP	(PIN_SLPM_DIR_INPUT | PIN_SLPM_PULL_UP)
#define PIN_SLPM_INPUT_NOPULL	(PIN_SLPM_DIR_INPUT | PIN_SLPM_PULL_NONE)
#define PIN_SLPM_OUTPUT_LOW	(PIN_SLPM_DIR_OUTPUT | PIN_SLPM_VAL_LOW)
#define PIN_SLPM_OUTPUT_HIGH	(PIN_SLPM_DIR_OUTPUT | PIN_SLPM_VAL_HIGH)

#define PIN_CFG_DEFAULT		(0)

#define PIN_CFG(num, alt)		\
	(PIN_CFG_DEFAULT |\
	 (PIN_NUM(num) | PIN_##alt))

#define PIN_CFG_INPUT(num, alt, pull)		\
	(PIN_CFG_DEFAULT |\
	 (PIN_NUM(num) | PIN_##alt | PIN_INPUT_##pull))

#define PIN_CFG_OUTPUT(num, alt, val)		\
	(PIN_CFG_DEFAULT |\
	 (PIN_NUM(num) | PIN_##alt | PIN_OUTPUT_##val))

#define PIN_CFG_PULL(num, alt, pull)	\
	((PIN_CFG_DEFAULT & ~PIN_PULL_MASK) |\
	 (PIN_NUM(num) | PIN_##alt | PIN_PULL_##pull))

#define GPIO155_GPIO		PIN_CFG(155, GPIO)
#define GPIO156_GPIO		PIN_CFG(156, GPIO)
#define GPIO208_MC1_CLK		PIN_CFG(208, ALT_A)
#define GPIO210_MC1_CMD		PIN_CFG(210, ALT_A)
#define GPIO211_MC1_DAT0	PIN_CFG(211, ALT_A)
#define GPIO212_MC1_DAT1	PIN_CFG(212, ALT_A)
#define GPIO213_MC1_DAT2	PIN_CFG(213, ALT_A)
#define GPIO214_MC1_DAT3	PIN_CFG(214, ALT_A)

extern int ENABLE_IRQ_LOG; /*austin added for debug*/
extern struct platform_device *mmci_get_platform_device(void);
extern struct mmc_host *mmci_get_mmc(void);
extern int nmk_config_pin(pin_cfg_t cfg, bool sleep);
/*extern int mmc_add_host(struct mmc_host *host); //austin added it for EXPORT_SYMBOL test*/

#endif
