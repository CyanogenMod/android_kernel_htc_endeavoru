/*
 * SdioDrv.c - MSM Linux SDIO driver (platform and OS dependent)
 *
 * The lower SDIO driver (BSP) for MSM on Linux OS.
 * Provides all SDIO commands and read/write operation methods.
 *
 * Copyright (c) 1998 - 2008 Texas Instruments Incorporated
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2
 *  as published by the Free Software Foundation
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <mach/io.h>
#include <linux/types.h>
#include <linux/dma-mapping.h>
#include <mach/hardware.h>
#include <linux/platform_device.h>
/*#include <mach/board.h>*/
#include <linux/errno.h>
#include <linux/clk.h>
/*#include <mach/dma.h>*/
#include <asm/io.h>
#include <linux/mmc/sdio_ids.h>
#include <linux/mmc/sdio_func.h>
#include <linux/mmc/card.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/sd.h>
#include <linux/mmc/sdio.h>
/*#include <linux/mmc/debug.h>*/
/*#include <asm/mach/mmc.h>*/
#include <linux/delay.h>
#include <mach/gpio.h> /*austin added*/

#include "SdioDrvDbg.h"
#include "SdioDrv.h"

/*HTC_CSP_START*/
#include <linux/mmc/sdio_ops.h>
#include <linux/i2c.h>
#include <asm/io.h>
#include <linux/miscdevice.h>
#include <linux/proc_fs.h>
#include <asm/mach-types.h>
#include <asm/setup.h>
//TODO extern void pincfg_sdio_power_on_wifi(void);
//TODO extern void pincfg_sdio_power_off_wifi(void);
//TODO extern void config_clk32k_on(int, int);
/*HTC_CSP_END*/

/* Add by daniel, provide sdio driver for emapi
 */
#define EXPORT_TI_SDIO_DRIVER 0
#if EXPORT_TI_SDIO_DRIVER
struct tisdio_platform_data {
	int (*sdio_connect_bus)(void *fCbFunc,
				void *hCbArg,
				unsigned int uBlkSizeShift,
				unsigned int uSdioThreadPriority);
	int (*sdio_disconnect_bus)(void);
	int (*sdio_execute_cmd)(unsigned int uCmd,
				unsigned int uArg,
				unsigned int uRespType,
				void *pResponse,
				unsigned int uLen);
	int (*sdio_read)(unsigned int uFunc,
			unsigned int uHwAddr,
			void *pData,
			unsigned int uLen,
			unsigned int bBlkMode,
			unsigned int bIncAddr,
			unsigned int bMore);
	int (*sdio_write)(unsigned int uFunc,
			unsigned int uHwAddr,
			void *pData,
			unsigned int uLen,
			unsigned int bBlkMode,
			unsigned int bIncAddr,
			unsigned int bMore);
	int (*sdio_read_direct_bytes)(unsigned int uFunc,
					unsigned int  uHwAddr,
					unsigned char *pData,
					unsigned int  uLen,
					unsigned int  bMore);
	int (*sdio_write_direct_bytes)(unsigned int uFunc,
				unsigned int  uHwAddr,
				unsigned char *pData,
				unsigned int  uLen,
				unsigned int  bMore);
};

struct tisdio_platform_data ti_sdio_dev_data = {
	sdioDrv_ConnectBus,
	sdioDrv_DisconnectBus,
	sdioDrv_ExecuteCmd,
	/*sdioDrv_ReadSync,  //austin change to sdioDrv_ReadAsync*/
	/*sdioDrv_WriteSync, //austin change to sdioDrv_WriteAsync*/
	sdioDrv_ReadAsync,
	sdioDrv_WriteAsync,
	sdioDrv_ReadSyncBytes,
	sdioDrv_WriteSyncBytes,
};

static void tisdio_dev_release(struct device *dev)
{
	return;
}

static struct platform_device tisdio_dev = {
	.name		= "ti_sdio_driver",
	.id		= 1,
	.num_resources  = 0,
	.resource	= NULL,
	.dev		= {
		.release = tisdio_dev_release,
	},
};

#endif


#if 0 /*mmc_io_rw_extended is defined at sdio_ops.c*/
int mmc_io_rw_extended(struct mmc_card *card, int write, unsigned fn,
		unsigned addr, int incr_addr, u8 *buf, unsigned blocks, unsigned blksz)
{
	struct mmc_request mrq;
	struct mmc_command cmd;
	struct mmc_data data;
	struct scatterlist sg;

	BUG_ON(!card);
	BUG_ON(fn > 7);
	BUG_ON(blocks == 1 && blksz > 512);
	WARN_ON(blocks == 0);
	WARN_ON(blksz == 0);

	memset(&mrq, 0, sizeof(struct mmc_request));
	memset(&cmd, 0, sizeof(struct mmc_command));
	memset(&data, 0, sizeof(struct mmc_data));

	mrq.cmd = &cmd;
	mrq.data = &data;

	cmd.opcode = SD_IO_RW_EXTENDED;
	cmd.arg = write ? 0x80000000 : 0x00000000;
	cmd.arg |= fn << 28;
	cmd.arg |= incr_addr ? 0x04000000 : 0x00000000;
	cmd.arg |= addr << 9;
	if (blocks == 1 && blksz <= 512)
		cmd.arg |= (blksz == 512) ? 0 : blksz;	/* byte mode */
	else
		cmd.arg |= 0x08000000 | blocks;		/* block mode */
	cmd.flags = MMC_RSP_SPI_R5 | MMC_RSP_R5 | MMC_CMD_ADTC;

	data.blksz = blksz;
	data.blocks = blocks;
	data.flags = write ? MMC_DATA_WRITE : MMC_DATA_READ;
	data.sg = &sg;
	data.sg_len = 1;

	sg_init_one(&sg, buf, blksz * blocks);

	mmc_set_data_timeout(&data, card);

	/*LOGD("mmc_io_rw_extended  cmd.arg=0x%x  data.blksz=%d  data.blocks=%d\n",cmd.arg,data.blksz,data.blocks);*/
	mmc_wait_for_req(card->host, &mrq);

	if (cmd.error)
		return cmd.error;
	if (data.error)
		return data.error;

	if (mmc_host_is_spi(card->host)) {
		/* host driver already reported errors */
	} else {
		if (cmd.resp[0] & R5_ERROR)
			return -EIO;
		if (cmd.resp[0] & R5_FUNCTION_NUMBER)
			return -EINVAL;
		if (cmd.resp[0] & R5_OUT_OF_RANGE)
			return -ERANGE;
	}

	return 0;

}
#endif

/* This definition is very important. It should match the platform device
   definition. E.g., in msm's devices file, the following has been added
   to the second mmc device:

   static struct platform_device msm_sdc1_device = {
   .name		= "TIWLAN_SDIO",
   .id		= 1,
   .num_resources	= ARRAY_SIZE(msm_sdc1_resources),
   .resource	= msm_sdc1_resources,
   .dev		= {
   .coherent_dma_mask	= 0xffffffff,
   },
   };
   */
#define SDIO_DRIVER_NAME 			"TIWLAN_SDIO"

/* TODO: Platform specific */
#define TIWLAN_MMC_CONTROLLER	1

typedef struct ST_sdiodrv {
	struct mmc_host *mmc;
	struct platform_device *pdev;
	void (*BusTxnCB)(void *BusTxnHandle, int status);
	void *BusTxnHandle;
	unsigned int  uBlkSize;
	unsigned int  uBlkSizeShift;
	int (*wlanDrvIf_pm_resume)(void);
	int (*wlanDrvIf_pm_suspend)(void);
	struct device *dev;
/*HTC_CSP_START austin added for early suspend*/
	void (*wlanDrvIf_setResumeConfig)(void);  /*austin sync Dragon*/
	void (*wlanDrvIf_setSuspendConfig)(void); /*austin sync Dragon*/
/*HTC_CSP_END*/

} ST_sdiodrv_t;

ST_sdiodrv_t g_drv;

module_param(g_sdio_debug_level1, int, SDIO_DEBUGLEVEL_ERR);
MODULE_PARM_DESC(g_sdio_debug_level1, "TIWLAN SDIO debug level");
/*module_param(g_sdio_debug_level1, int, 0644);*/
/*MODULE_PARM_DESC(g_sdio_debug_level1, "debug level");*/
int g_sdio_debug_level1 = SDIO_DEBUGLEVEL_ERR;
EXPORT_SYMBOL(g_sdio_debug_level1);

struct platform_device *sdioDrv_get_platform_device(void)
{
	return g_drv.pdev;
}

int sdioDrv_ReadSync(unsigned int uFunc,
			unsigned int uHwAddr,
			void *pData,
			unsigned int uLen,
			unsigned int bIncAddr,
			unsigned int bMore)
{
	printk(KERN_INFO "We don't use sdioDrv_ReadSync, please call sdioDrv_ReadAsync\n");
	return 0;
}

int sdioDrv_WriteSync(unsigned int uFunc,
			unsigned int uHwAddr,
			void *pData,
			unsigned int uLen,
			unsigned int bIncAddr,
			unsigned int bMore)
{
	printk(KERN_INFO "We don't use sdioDrv_WriteSync, please call sdioDrv_WriteAsync\n");
	return 0;
}

#ifdef CMD53_ALL_BLOCK_MODE
#define FN0_FBR2_REG_108                    0x210
#define FN0_FBR2_REG_108_BIT_MASK           0xFFF
int change_block_size(unsigned long size)
{
	unsigned long  uLong;
	unsigned long  uCount = 0;
		int iStatus;

		/*LOGD("change_block_size:%u\n", size);*/

	/* set block size for SDIO block mode */
	do {
	uLong = size;
	iStatus = sdioDrv_WriteSyncBytes(TXN_FUNC_ID_CTRL, FN0_FBR2_REG_108, (unsigned char *)&uLong, 2, 1);
	if (iStatus)
		return -1;

	iStatus = sdioDrv_ReadSyncBytes(TXN_FUNC_ID_CTRL, FN0_FBR2_REG_108, (unsigned char *)&uLong, 2, 1);
	if (iStatus)
		return -2;

	/*iStatus = sdioDrv_WriteSyncBytes(TXN_FUNC_ID_CTRL, 0xC8, (unsigned char*)&uLong, 2, 1);
	if (iStatus)
		return -3;*/

	uCount++;

	} while (((uLong & FN0_FBR2_REG_108_BIT_MASK) != size) && (uCount < 10));

	if (uCount >= 10)
		return uCount;
	return 0;
}
#endif

int sdioDrv_ExecuteCmd(unsigned int uCmd,
		unsigned int uArg,
		unsigned int uRespType,
		void *pResponse,
		unsigned int uLen)
{
	int read, i = 0;
	int err;
	struct mmc_command cmd;
	unsigned char *ret = ((unsigned char *)pResponse);

	memset(&cmd, 0, sizeof(struct mmc_command));
	cmd.opcode = uCmd;
	cmd.arg = uArg;
	cmd.flags = uRespType;

	PDEBUG("sdioDrv_ExecuteCmd() starting cmd %02x arg %08x\n", (int)uCmd, (int)uArg)
	printk(KERN_INFO "sdioDrv_ExecuteCmd() starting cmd %02x arg %08x\n", (int)uCmd, (int)uArg);

	err = mmc_wait_for_cmd(g_drv.mmc, &cmd, 3);
	if (err) {
		PERR("sdioDrv_ExecuteCmd fail: %d\n", err)
		return 1;
	}

	/* Copy the result back to the argument*/
	i = 0;
	while ((uLen > 0) && (ret)) {
		read = (uLen < 4) ? uLen : 4;
		memcpy(ret, &cmd.resp[i], uLen);
		ret += read;
		uLen -= read;
		i++;
	}

	return 0;
}

int sdioDrv_ReadAsync(unsigned int uFunc,
		unsigned int uHwAddr,
		void *pData,
		unsigned int uLen,
		unsigned int bBlkMode,
		unsigned int bIncAddr,
		unsigned int bMore)
{
	int iStatus;
	struct mmc_card scard;
	int num_blocks;
	int i, status = 0;
	/*int i=0; //austin added for debug*/

	/*printk("sdioDrv_ReadAsync  blk_size:%d  bBlkMode:%d  uLen:%d    ++++++\n",g_drv.uBlkSize,bBlkMode,uLen);*/
	if (uLen <= 512) {
	/*if(uLen == 8 || uLen == 4) {*/
		for (i = 0; i < uLen ; ++i) {
			status = sdioDrv_ReadSyncBytes(uFunc, uHwAddr, pData, 1, bMore);
			uHwAddr += 1;
			pData = (void *)((unsigned int)pData+1);
			if (status)
				PERR("%s FAILED(%d)!!\n", __func__, status)
		}
		return status;
	}

	i = 0;
	if (bBlkMode) {
		memset(&scard, 0, sizeof(struct mmc_card));
		scard.cccr.multi_block = 1;
		scard.type = MMC_TYPE_SDIO;
		scard.host = g_drv.mmc;

		num_blocks = uLen / g_drv.uBlkSize;
		iStatus = mmc_io_rw_extended(&scard, 0, uFunc, uHwAddr, bIncAddr, pData, num_blocks, g_drv.uBlkSize);

	} else {
		memset(&scard, 0, sizeof(struct mmc_card));
		scard.cccr.multi_block = 0; /*don't use block mode for now*/
		scard.type = MMC_TYPE_SDIO;
		scard.host = g_drv.mmc;

		iStatus = mmc_io_rw_extended(&scard, 0, uFunc, uHwAddr, bIncAddr, pData, 1, uLen);

		/*for(i=0;i<uLen;i++) { //austin added for debug
			LOGD("pData[%d] = 0x%x\n",i,(unsigned *)pData[i]);
		}*/
	}

	if (iStatus != 0)
		PERR("%s FAILED(%d)!!\n", __func__, iStatus)

	/*if(ENABLE_IRQ_LOG)LOGD("sdioDrv_ReadAsync  iStatus:%d                           ------\n",iStatus);*/
	/*printk("sdioDrv_ReadAsync  iStatus:%d                           ------\n",iStatus);*/
	return iStatus;


}


/*--------------------------------------------------------------------------------------*/

int sdioDrv_WriteAsync(unsigned int uFunc,
			unsigned int uHwAddr,
			void *pData,
			unsigned int uLen,
			unsigned int bBlkMode,
			unsigned int bIncAddr,
			unsigned int bMore)
{

	int iStatus;
	struct mmc_card scard;
	int num_blocks;
	int i = 0, status = 0;

	/*printk("sdioDrv_WriteAsync  blk_size:%d  bBlkMode:%d  uLen:%d   ++++++\n",g_drv.uBlkSize,bBlkMode,uLen);
	work arround for CMD 53 8 bytes data be blocked
	if (uLen == 8 || uLen == 4) {*/
	if (uLen <= 512) {
		for (i = 0; i < uLen ; ++i) {
			status = sdioDrv_WriteSyncBytes(uFunc, uHwAddr, pData, 1, bMore);
			uHwAddr += 1;
			pData = (void *)((unsigned int)pData+1);
			if (status)
				PERR("%s FAILED(%d)!!\n", __func__, status)
		}
		return status;
	}

	if (bBlkMode) {
		memset(&scard, 0, sizeof(struct mmc_card));
		scard.cccr.multi_block = 1;
		scard.type = MMC_TYPE_SDIO;
		scard.host = g_drv.mmc;

		num_blocks = uLen / g_drv.uBlkSize;
		iStatus = mmc_io_rw_extended(&scard, 1, uFunc, uHwAddr, bIncAddr, pData, num_blocks, g_drv.uBlkSize);

	} else {
		memset(&scard, 0, sizeof(struct mmc_card));
		scard.cccr.multi_block = 0;
		scard.type = MMC_TYPE_SDIO;
		scard.host = g_drv.mmc;

		iStatus = mmc_io_rw_extended(&scard, 1, uFunc, uHwAddr, bIncAddr, pData, 1, uLen);
	}

	if (iStatus != 0)
		PERR(KERN_INFO "%s FAILED(%d)!!\n", __func__, iStatus)

	/*if(ENABLE_IRQ_LOG) LOGD("sdioDrv_WriteAsync  iStatus:%d                           ------\n",iStatus);
	printk(KERN_INFO "sdioDrv_WriteAsync  iStatus:%d                           ------\n",iStatus);
	*/return iStatus;


}

/*--------------------------------------------------------------------------------------*/

int sdioDrv_ReadSyncBytes(unsigned int  uFunc,
		unsigned int  uHwAddr,
		unsigned char *pData,
		unsigned int  uLen,
		unsigned int  bMore)
{
	unsigned int i;
	int iStatus;
	struct mmc_command cmd;

	for (i = 0; i < uLen; i++) {
		memset(&cmd, 0, sizeof(struct mmc_command));

		cmd.opcode = SD_IO_RW_DIRECT;
		cmd.arg = 0x00000000; /*read*/
		cmd.arg |= uFunc << 28;
		cmd.arg |= 0x00000000; /*not write*/
		cmd.arg |= uHwAddr << 9;
		cmd.arg |= 0; /*no in*/
		cmd.flags = MMC_RSP_PRESENT;

		iStatus = mmc_wait_for_cmd(g_drv.mmc, &cmd, 0);
		if (iStatus) {
			PERR("sdioDrv_WriteSyncBytes() SDIO Command error status = %d\n", iStatus)
			return -1;
		}

		*pData = cmd.resp[0] & 0xFF;

		uHwAddr++;
		pData++;
	}

	return 0;
}

/*--------------------------------------------------------------------------------------*/

int sdioDrv_WriteSyncBytes(unsigned int  uFunc,
		unsigned int  uHwAddr,
		unsigned char *pData,
		unsigned int  uLen,
		unsigned int  bMore)
{
	unsigned int i;
	int iStatus;
	struct mmc_command cmd;

	for (i = 0; i < uLen; i++) {
		memset(&cmd, 0, sizeof(struct mmc_command));

		cmd.opcode = SD_IO_RW_DIRECT;
		cmd.arg = 0x80000000; /*write*/
		cmd.arg |= uFunc << 28;
		cmd.arg |= 0x00000000; /*no out*/
		cmd.arg |= uHwAddr << 9;
		cmd.arg |= *pData; /*in*/
		cmd.flags = MMC_RSP_PRESENT;

		iStatus = mmc_wait_for_cmd(g_drv.mmc, &cmd, 3);
		if (iStatus) {
			PERR("sdioDrv_WriteSyncBytes() SDIO Command error status = %d\n", iStatus)
			return -1;
		}

		uHwAddr++;
		pData++;
	}

	return 0;
}

int emapi_sdio_read(char *out_data_buffer, unsigned long in_source_address, unsigned long data_length)
{
	int status;
	unsigned long bytes_count;
	unsigned long extra_size;
	unsigned int transferSize;
	unsigned int incr_fix = INCREMENT_REG_ADDR; /*address will be inc by HW*/
	/*unsigned int ublkmode = 0;*/

	/*austin +++ start*/
	/*work around for data_length%4 neq 0 */
	if ((data_length%4) != 0) {
		for (bytes_count = 0; bytes_count < data_length; bytes_count++)
			sdioDrv_ReadSyncBytes(TXN_FUNC_ID_WLAN, in_source_address++, out_data_buffer++, 1, 1);
	} /*austin +++ end*/
	else{
		extra_size = data_length;

	/*if (extra_size < g_drv.uBlkSize)
		ublkmode = 0;
	else
		ublkmode = 1;*/

		while (extra_size > 0) {
			transferSize = (extra_size > SDIO_MAX_PACKET_LEN) ? SDIO_MAX_PACKET_LEN:extra_size;
			/*status=SDIO_SyncReadBlock(out_data_buffer, in_source_address, transferSize);*/
			status = sdioDrv_ReadAsync(TXN_FUNC_ID_WLAN, in_source_address, out_data_buffer, transferSize, 1, incr_fix, 0);
			/*status = sdioDrv_ReadAsync(TXN_FUNC_ID_WLAN, in_source_address, out_data_buffer, transferSize,ublkmode, incr_fix, 0);*/
			if (status != 0) {
				printk(KERN_INFO "emapi_sdio_read fail (addr:0x%x, size:%d) \r\n", (unsigned int)in_source_address, transferSize);
				return status;
			}
			out_data_buffer += transferSize;
			in_source_address += ((incr_fix == INCREMENT_REG_ADDR) ? transferSize : 0);
			extra_size -= transferSize;
		}
	}

	return 0;
}
EXPORT_SYMBOL(emapi_sdio_read);

int emapi_sdio_write(char *in_data_buffer,  unsigned long out_target_address, unsigned long out_length)
{
	int status;
	unsigned long bytes_count;
	unsigned long extra_size;
	unsigned int transferSize;
	unsigned int incr_fix = INCREMENT_REG_ADDR; /*address will be inc by HW*/
	/*unsigned int ublkmode = 0;*/

	/*austin +++ start*/
	/*work around for data_length%4 neq 0 */
	if ((out_length%4) != 0) {
		for (bytes_count = 0; bytes_count < out_length; bytes_count++)
			sdioDrv_WriteSyncBytes(TXN_FUNC_ID_WLAN, out_target_address++, in_data_buffer++, 1, 1);
	} /*austin +++ end*/
	else{
		extra_size = out_length;

		/*if (extra_size < g_drv.uBlkSize)
		ublkmode = 0;
	else
		ublkmode = 1;*/

		while (extra_size > 0) {
			transferSize = (extra_size > SDIO_MAX_PACKET_LEN) ? SDIO_MAX_PACKET_LEN:extra_size;
			/*status=SDIO_SyncWriteBlock(in_data_buffer, out_target_address, transferSize);*/
			status = sdioDrv_WriteAsync(TXN_FUNC_ID_WLAN, out_target_address, in_data_buffer, transferSize, 1, incr_fix, 0);
			/*status= sdioDrv_WriteAsync(TXN_FUNC_ID_WLAN, out_target_address, in_data_buffer, transferSize, ublkmode,incr_fix, 0);*/
			if (status != 0) {
				printk(KERN_INFO "emapi_sdio_write fail (addr:0x%x, size:%d) \r\n", (unsigned int)out_target_address, transferSize);
				return status;
			}
			in_data_buffer += transferSize;
			out_target_address += ((incr_fix == INCREMENT_REG_ADDR) ? transferSize : 0);
			extra_size -= transferSize;
		}
	}

	return 0;
}
EXPORT_SYMBOL(emapi_sdio_write);

unsigned int sdioDrv_status(struct device *dev)
{
	return 1;
}

int sdioDrv_ConnectBus(void *fCbFunc,
			void *hCbArg,
			unsigned int uBlkSizeShift,
			unsigned int  uSdioThreadPriority)
{
	g_drv.BusTxnCB      = fCbFunc;
	g_drv.BusTxnHandle  = hCbArg;
	g_drv.uBlkSizeShift = uBlkSizeShift;
	g_drv.uBlkSize      = 1 << uBlkSizeShift;

	return 0;
}

int sdioDrv_DisconnectBus(void)
{
	g_drv.BusTxnCB      = NULL;
	g_drv.BusTxnHandle  = 0;
	g_drv.uBlkSizeShift = 0;
	g_drv.uBlkSize      = 0;

	return 0;
}

int sdioDrv_set_clock(unsigned int clock)
{
	struct mmc_ios ios;
	struct mmc_host *mmc = 0;


	mmc = g_drv.mmc;
/*TODO	mmc->index = TIWLAN_MMC_CONTROLLER;*/

	printk(KERN_INFO "[ %s ] Settin clock %lu Hz\n", __func__, (unsigned long)clock);
	memset(&ios, 0, sizeof(struct mmc_ios));
	ios.bus_width = TIWLAN_SDIO_BUSWIDE;
	ios.power_mode = MMC_POWER_ON;
	ios.clock =  clock;
	mmc->ops->set_ios(mmc, &ios);
	return 0;
}

/*HTC_CSP_START for emapiTest*/
typedef char	*PUCHAR;

unsigned long g_uMemAddr;
unsigned long g_uMemSize;
unsigned long g_uRegAddr;
unsigned long g_uRegSize;

typedef enum {
	SDIO_SUCCESS = 0,
	SDIO_FAILURE,
	SDIO_BUSY,
	SDIO_BAD_ALIGNMENT,
	SDIO_TIMEOUT,
	SDIO_BAD_INPUT_PARAMS,
	SDIO_BAD_PERIPHERAL_ADDRESS,
	SDIO_PACKET_SIZE_AND_MODE_INCONSISTENCY,
	SDIO_CONTEXT_AND_MODE_INCONSISTENCY,
	SDIO_NO_RESOURCES,
	SDIO_NON_AFFECTED,
} SDIO_Status;

int sdioDrv_SyncReadMethod(char *out_data_buffer,
			unsigned long in_source_address,
			unsigned long data_length)
{
	/*SDIO_BufferLength bytes_count;*/
	/*unsigned long extra_size;
	unsigned int transferSize;
	SDIO_Status status;
	unsigned int incr_fix=1;/*INCREMENT_REG_ADDR; //address will be inc by HW automatically*/
#if 0/*USE_SIGNAL_READ_WRITE*/
/*use cmd 52*/
	unsigned long bytes_count;
	for (bytes_count = 0; bytes_count < data_length; bytes_count++)
		sdioDrv_ReadSyncBytes(TXN_FUNC_ID_WLAN, in_source_address++, out_data_buffer++, 1, 1);
#else
/*use cmd 53*/
	unsigned long extra_size;
	unsigned int transferSize;
	int block_mode = 0;
	SDIO_Status status;
	unsigned int incr_fix = 1;/*INCREMENT_REG_ADDR; //address will be inc by HW automatically*/
	extra_size = data_length;
	if (data_length < g_drv.uBlkSize)
		block_mode = 0;
	else
		block_mode = 1;
/* we don't support multi-block mode now
	if (gWiFi_SDIOBlockMode) {
		transferSize = extra_size / SDIO_MAX_PACKET_LEN;
		if (transferSize > 1) {
			status=SDIO_SyncReadBlockF(out_data_buffer,in_source_address,1,transferSize, SDIO_MAX_PACKET_LEN, FUNCTION_2, incr_fix, 0);
			if (status != SDIO_SUCCESS) {
				OutputFormatString("SDIO_SyncReadBlockF fail (addr:0x%x, blknum:%d, blksize:%d) \r\n", in_source_address, transferSize,SDIO_MAX_PACKET_LEN);
					return status;
			}
			out_data_buffer += (transferSize * SDIO_MAX_PACKET_LEN);
			in_source_address += ((incr_fix == INCREMENT_REG_ADDR) ? transferSize * SDIO_MAX_PACKET_LEN : 0);
			extra_size = transferSize % SDIO_MAX_PACKET_LEN;
		}
	}
*/
	while (extra_size > 0) {
		transferSize = (extra_size > g_drv.uBlkSize)? g_drv.uBlkSize :extra_size;
		/*status=SDIO_SyncReadBlock(out_data_buffer, in_source_address, transferSize);	*/
		printk(KERN_INFO "Aus: readsync  transferSize:%d\n", transferSize);
		status = sdioDrv_ReadAsync(TXN_FUNC_ID_WLAN, in_source_address, out_data_buffer, transferSize, block_mode, incr_fix, 0);
		if (status != SDIO_SUCCESS) {
			printk(KERN_INFO "sdioDrv_SyncReadMethod fail (addr:0x%x, size:%d) \r\n", (unsigned int)in_source_address, transferSize);
			return status;
		}
		out_data_buffer += transferSize;
		in_source_address += ((incr_fix == 1/*INCREMENT_REG_ADDR*/) ? transferSize : 0);
		extra_size -= transferSize;
	}
#endif
	return 0;/*SDIO_SUCCESS;*/
}

int sdioDrv_SyncWriteMethod(char *in_data_buffer,
				unsigned long out_target_address,
				unsigned long out_length)
{
	/*SDIO_BufferLength bytes_count;*/
	/*unsigned long extra_size;
	unsigned int transferSize;
	SDIO_Status status;
	unsigned int incr_fix=1; INCREMENT_REG_ADDR; address will be inc by HW automatically*/

#if 0 /*USE_SIGNAL_READ_WRITE*/
	/*use cmd 52*/
	unsigned long bytes_count;
	for (bytes_count = 0; bytes_count < out_length; bytes_count++)
		sdioDrv_WriteSyncBytes(TXN_FUNC_ID_WLAN, out_target_address++, in_data_buffer++, 1, 1);
	/*Austin in_source_address -> out_target_address*/
#else
	unsigned int extra_size;
	unsigned int transferSize;
	int block_mode = 0;
	SDIO_Status status;
	unsigned int incr_fix = 1;/*INCREMENT_REG_ADDR;//address will be inc by HW automatically*/
	extra_size = out_length;

	if (out_length < g_drv.uBlkSize)
		block_mode = 0;
	else
		block_mode = 1;

	/* we don't suport block mode now
	if(gWiFi_SDIOBlockMode)
	{
		transferSize = extra_size / SDIO_MAX_PACKET_LEN;
		if (transferSize > 1)
		{
			status=SDIO_SyncWriteBlockF(in_data_buffer,out_target_address, 1, transferSize, SDIO_MAX_PACKET_LEN, FUNCTION_2, incr_fix, 0);
			if(status!=SDIO_SUCCESS)
			{
				OutputFormatString("SDIO_SyncWriteBlockF fail (addr:0x%x, blknum:%d, blksize:%d) \r\n", out_target_address, transferSize, SDIO_MAX_PACKET_LEN);
				return status;
			}
			in_data_buffer += (transferSize * SDIO_MAX_PACKET_LEN);
			out_target_address += ((incr_fix == INCREMENT_REG_ADDR) ? transferSize * SDIO_MAX_PACKET_LEN : 0);
			extra_size = transferSize % SDIO_MAX_PACKET_LEN;
		}
	}*/

	while (extra_size > 0) {
		int i = 0;
		transferSize = (extra_size > g_drv.uBlkSize)? g_drv.uBlkSize :extra_size;
		/*status=SDIO_SyncWriteBlock(in_data_buffer, out_target_address, transferSize);*/
		printk(KERN_INFO "i= %d -> Aus: writesync  transferSize:%d,  extra_size:%d  g_drv.uBlkSize:%d\n", i, transferSize, extra_size, g_drv.uBlkSize);
		status = sdioDrv_WriteAsync(TXN_FUNC_ID_WLAN, out_target_address, in_data_buffer, transferSize, block_mode, incr_fix, 0);
		if (status != SDIO_SUCCESS) {
			printk(KERN_INFO "sdioDrv_SyncWriteMethod fail (addr:0x%x, size:%d) \r\n", (unsigned int)out_target_address, transferSize);
			return status;
		}
		in_data_buffer += transferSize;
		out_target_address += ((incr_fix == 1/*INCREMENT_REG_ADDR*/) ? transferSize : 0);
		extra_size -= transferSize;
		i++;
	}
#endif
	return 0;/*SDIO_SUCCESS;*/
}

int g_start_tnetw_address;
int g_end_tnetw_address;
int g_start_reg_address;

void sdioDrv_Configure_Partition(unsigned long start_addr, unsigned long size)
{
	char data;

	data = (char)(size & 0xFF);
	sdioDrv_WriteSyncBytes(TXN_FUNC_ID_WLAN, 0x1FFC0, &data, 1, 1);

	data = (char)((size>>8) & 0xFF);
	sdioDrv_WriteSyncBytes(TXN_FUNC_ID_WLAN, 0x1FFC1, &data, 1, 1);

	data = (char)((size>>16) & 0xFF);
	sdioDrv_WriteSyncBytes(TXN_FUNC_ID_WLAN, 0x1FFC2, &data, 1, 1);

	data = (char)((size>>24) & 0xFF);
	sdioDrv_WriteSyncBytes(TXN_FUNC_ID_WLAN, 0x1FFC3, &data, 1, 1);

	/* Set offset */
	data = (char)(start_addr & 0xFF);
	sdioDrv_WriteSyncBytes(TXN_FUNC_ID_WLAN, 0x1FFC4, &data, 1, 1);

	data = (char)((start_addr>>8) & 0xFF);
	sdioDrv_WriteSyncBytes(TXN_FUNC_ID_WLAN, 0x1FFC5, &data, 1, 1);

	data = (char)((start_addr>>16) & 0xFF);
	sdioDrv_WriteSyncBytes(TXN_FUNC_ID_WLAN, 0x1FFC6, &data, 1, 1);

	data = (char)((start_addr>>24) & 0xFF);
	sdioDrv_WriteSyncBytes(TXN_FUNC_ID_WLAN, 0x1FFC7, &data, 1, 1);

	g_start_tnetw_address = start_addr;
	g_end_tnetw_address = start_addr + size;

}

void sdioDrv_Configure_Partition2(unsigned long start_addr, unsigned long size)
{
	char data;

	/* Configure 17-bits address range in TNETW: */
	/* Set size */
	data = (char)(size & 0xFF);
	sdioDrv_WriteSyncBytes(TXN_FUNC_ID_WLAN, 0x1FFC8, &data, 1, 1);

	data = (char)((size>>8) & 0xFF);
	sdioDrv_WriteSyncBytes(TXN_FUNC_ID_WLAN, 0x1FFC9, &data, 1, 1);

	data = (char)((size>>16) & 0xFF);
	sdioDrv_WriteSyncBytes(TXN_FUNC_ID_WLAN, 0x1FFCA, &data, 1, 1);

	data = (char)((size>>24) & 0xFF);
	sdioDrv_WriteSyncBytes(TXN_FUNC_ID_WLAN, 0x1FFCB, &data, 1, 1);

	/* Set offset */
	data = (char)(start_addr & 0xFF);
	sdioDrv_WriteSyncBytes(TXN_FUNC_ID_WLAN, 0x1FFCC, &data, 1, 1);

	data = (char)((start_addr>>8) & 0xFF);
	sdioDrv_WriteSyncBytes(TXN_FUNC_ID_WLAN, 0x1FFCD, &data, 1, 1);

	data = (char)((start_addr>>16) & 0xFF);
	sdioDrv_WriteSyncBytes(TXN_FUNC_ID_WLAN, 0x1FFCE, &data, 1, 1);

	data = (char)((start_addr>>24) & 0xFF);
	sdioDrv_WriteSyncBytes(TXN_FUNC_ID_WLAN, 0x1FFCF, &data, 1, 1);

	g_start_reg_address = start_addr;

}

void eMapiSetPartition(unsigned long uMemAddr, unsigned long uMemSize, unsigned long uRegAddr, unsigned long uRegSize)
{
	g_uRegAddr = uRegAddr;
	g_uRegSize = uRegSize;
	g_uMemAddr = uMemAddr;
	g_uMemSize = uMemSize;

	sdioDrv_Configure_Partition(uMemAddr, uMemSize);
	sdioDrv_Configure_Partition2(uRegAddr, uRegSize);

}
void eMapiConvertSDIOAddress(unsigned hostAddr, unsigned *SDIOAddr)
{
	unsigned uHwAddr = hostAddr;

	/*HTC_PrintMsg(ZONE_NKDBGMSG, "SDIO_ConvertAddress+\r\n");*/
	/* Translate HW address for registers region */
	if ((uHwAddr >= g_uRegAddr) && (uHwAddr <= g_uRegAddr + g_uRegSize))
		uHwAddr = uHwAddr - g_uRegAddr + g_uMemSize;

	/* Translate HW address for memory region */
	else
		uHwAddr = uHwAddr - g_uMemAddr;

	*SDIOAddr = uHwAddr;

}
unsigned eMapiWriteMemory(unsigned Addr, unsigned Length, PUCHAR pData)
{
	unsigned tnetw_addr;
	int istatus;

	eMapiConvertSDIOAddress(Addr, &tnetw_addr);

	istatus = sdioDrv_SyncWriteMethod((char *)pData, tnetw_addr, Length);

	return 0;
}
unsigned eMapiReadMemory(unsigned Addr, unsigned Length, PUCHAR pData)
{
	unsigned tnetw_addr;
	int istatus;

	eMapiConvertSDIOAddress(Addr, &tnetw_addr);

	istatus = sdioDrv_SyncReadMethod((char *)pData, tnetw_addr, Length);

	return 0;
}

void eMapiSdInterfaceTest2(unsigned testsize)
{
	const int size = testsize;
	int default_size = 4;
	unsigned char aTestPattern[size];
	unsigned char aTempBuf[size];
	unsigned int ii = 0;
	unsigned int kk = 0;
	const unsigned int NumberOfTest = 1;

	if (testsize > 2048)
		return ;

	/*init test block*/
	for (ii = 0; ii < testsize; ++ii) {
	if (1 == (ii%4))
		aTestPattern[ii] = 0xB1;
	else if (2 == (ii%4))
		aTestPattern[ii] = 0xC2;
	else if (3 == (ii%4))
		aTestPattern[ii] = 0xD3;
	else
		aTestPattern[ii] = 0xA0;
	}
	ii = 0;
	for (kk = 0; kk < testsize; ++kk)
		aTempBuf[kk] = 0x0;


	eMapiSetPartition(0x40000,
			0x14FC0,
			0x00300000,
			0xB000);

	printk(KERN_INFO "Set Partition Over\n\n");

	for (default_size = 4; default_size <= testsize; default_size += 4) {
		do {
			int bMatch = 0;
			unsigned int jj;
			++ii;

			if (eMapiWriteMemory(0x40000, default_size, (PUCHAR) &aTestPattern) != 0) {
				printk(KERN_INFO "[eMapiSdInterfaceTest] Can't write data to memory!!!\r\n");
				break;
			}
			mdelay(20);

			if (eMapiReadMemory(0x40000, default_size, (PUCHAR) &aTempBuf) != 0) {
				printk(KERN_INFO "[eMapiSdInterfaceTest] Can't read data from memory!!!\r\n");
				break;
			}

			for (jj = 0; jj < default_size; ++jj) {
				if (aTempBuf[jj] != aTestPattern[jj]) {
					printk(KERN_INFO "No j=%d, aTempBuf=0x%x, aTestPattern=0x%x\r\n", jj, aTempBuf[jj], aTestPattern[jj]);
					break;
				} else{
					printk(KERN_INFO "Yes the same j=%d, aTempBuf=0x%x, aTestPattern=0x%x  the same!!\r\n", jj, aTempBuf[jj], aTestPattern[jj]);
				}
			}

			/*clear aTempBuf*/
			for (kk = 0; kk < default_size; ++kk)
				aTempBuf[kk] = 0x0;

			printk(KERN_INFO "clear done ii=%d\n", ii);

			if (jj == default_size) {
				bMatch = 1;
				printk(KERN_INFO "Match ii=%d\n", ii);
			}
			if (!bMatch) {
				printk(KERN_INFO "[eMapiSdInterfaceTest] mismatch at %d!!!\r\n", ii);
				/*break;*/
			}
		} while (ii < NumberOfTest);
	}

	if (ii < NumberOfTest) {
		printk(KERN_INFO "eMapiSdInterfaceTest NG!\r\n");
		return ;
	} else {
		printk(KERN_INFO "eMapiSdInterfaceTest OK!\r\n");
		return ;
	}
}

void eMapiSdInterfaceTest(unsigned testsize)
{
	const int size = testsize;
	unsigned char aTestPattern[size];
	unsigned char aTempBuf[size];
	unsigned int ii = 0;
	unsigned int kk = 0;
	const unsigned int NumberOfTest = 1;

	if (testsize > 2048)
		return ;

	/*init test block*/
	for (ii = 0; ii < testsize; ++ii) {
		if (1 == (ii%4))
			aTestPattern[ii] = 0xB1;
		else if (2 == (ii%4))
			aTestPattern[ii] = 0xC2;
		else if (3 == (ii%4))
			aTestPattern[ii] = 0xD3;
		else
			aTestPattern[ii] = 0xA0;
	}
	ii = 0;
	for (kk = 0; kk < testsize; ++kk)
		aTempBuf[kk] = 0x0;

	eMapiSetPartition(0x40000,
			0x14FC0,
			0x00300000,
			0xB000);

	printk(KERN_INFO "Set Partition Over\n\n");

	do {
		int bMatch = 0;
		unsigned int jj;
		++ii;

		if (eMapiWriteMemory(0x40000, testsize, (PUCHAR) &aTestPattern) != 0) {
			printk(KERN_INFO "[eMapiSdInterfaceTest] Can't write data to memory!!!\r\n");
			break;
		}
		mdelay(20);

		if (eMapiReadMemory(0x40000, testsize, (PUCHAR) &aTempBuf) != 0) {
			printk(KERN_INFO "[eMapiSdInterfaceTest] Can't read data from memory!!!\r\n");
			break;
		}

		for (jj = 0; jj < testsize; ++jj) {
			if (aTempBuf[jj] != aTestPattern[jj]) {
				printk(KERN_INFO "No j=%d, aTempBuf=0x%x, aTestPattern=0x%x\r\n", jj, aTempBuf[jj], aTestPattern[jj]);
				break;
			} else {
				printk(KERN_INFO "Yes the same j=%d, aTempBuf=0x%x, aTestPattern=0x%x  the same!!\r\n", jj, aTempBuf[jj], aTestPattern[jj]);
			}
		}

		/*clear aTempBuf*/
		for (kk = 0; kk < testsize; ++kk)
			aTempBuf[kk] = 0x0;

		printk(KERN_INFO "clear done ii=%d\n", ii);

		if (jj == testsize) {
			bMatch = 1;
			printk(KERN_INFO "Match ii=%d\n", ii);
		}
		if (!bMatch) {
			printk(KERN_INFO "[eMapiSdInterfaceTest] mismatch at %d!!!\r\n", ii);
			/*break;*/
		}
	} while (ii < NumberOfTest);

	if (ii < NumberOfTest) {
		printk(KERN_INFO "eMapiSdInterfaceTest NG!\r\n");
		return ;
	} else {
		printk(KERN_INFO "eMapiSdInterfaceTest OK!\r\n");
		return ;
	}
}
/*HTC_CSP_END for emapiTest*/

/*HTC_CSP_START */
int wifi_power(int on)
{
	gpio_set_value(WLAN_EN, on);
	return 0;
}

int hPlatform_DevicePowerOff(void)
{
	int err = 0;

	printk(KERN_INFO "[%s]\n", __func__);
	err = wifi_power(0);
	msleep(10);

	return err;
}

int hPlatform_DevicePowerOn(void)
{
	int err = 0;
	int gpio_value = 0;

	printk(KERN_INFO "[%s]\n", __func__);
	gpio_value = gpio_get_value(WLAN_EN);
	printk(KERN_INFO "WLAN_EN GPIO value:%d (before enable)\n", gpio_value);

	err = wifi_power(1);
	msleep(15);

/*	gpio_value = gpio_get_value(WLAN_EN);*/
/*	printk("WLAN_EN GPIO value:%d ..(2)\n",gpio_value);*/

	err = wifi_power(0);
	msleep(1);

/*	gpio_value = gpio_get_value(WLAN_EN);*/
/*	printk("WLAN_EN GPIO value:%d ..(3)\n",gpio_value);*/

	err = wifi_power(1);
	msleep(70);

	gpio_value = gpio_get_value(WLAN_EN);
	printk(KERN_INFO "WLAN_EN GPIO value:%d (after enable)\n", gpio_value);
	return err;
}

int sdio_command_init(void)
{
	unsigned char  uByte;
	unsigned long  uLong;
	unsigned long  uCount = 0;
	unsigned int   uBlkSize = 1 << 9;
	int iStatus;
	g_drv.uBlkSize = uBlkSize;
	g_drv.uBlkSizeShift = 9;

	/* Init SDIO driver and HW */

	/* Send commands sequence: 0, 5, 3, 7 */
	printk(KERN_INFO "sdioAdapt_ConnectBus  CMD 0 5 3 7\n");
	iStatus = sdioDrv_ExecuteCmd(SD_IO_GO_IDLE_STATE, 0, MMC_RSP_NONE, &uByte, sizeof(uByte));
	if (iStatus) {
	printk(KERN_INFO "%s %d command number: %d failed\n", __func__, __LINE__, SD_IO_GO_IDLE_STATE);
	return iStatus;
	}

	iStatus = sdioDrv_ExecuteCmd(SDIO_CMD5, VDD_VOLTAGE_WINDOW, MMC_RSP_R4/*MMC_RSP_R4 austin change to MMC_RSP_PRESENT*/, &uByte, sizeof(uByte));
	if (iStatus) {
	printk(KERN_INFO "%s %d command number: %d failed\n", __func__, __LINE__, SDIO_CMD5);
	return iStatus;
	}

	iStatus = sdioDrv_ExecuteCmd(SD_IO_SEND_RELATIVE_ADDR, 0, MMC_RSP_R6, &uLong, sizeof(uLong));
	if (iStatus) {
	printk(KERN_INFO "%s %d command number: %d failed\n", __func__, __LINE__, SD_IO_SEND_RELATIVE_ADDR);
	return iStatus;
	}

	iStatus = sdioDrv_ExecuteCmd(SD_IO_SELECT_CARD, uLong, MMC_RSP_R6, &uByte, sizeof(uByte));
	if (iStatus) {
	printk(KERN_INFO "%s %d command number: %d failed\n", __func__, __LINE__, SD_IO_SELECT_CARD);
	return iStatus;
	}

	printk(KERN_INFO "sdioAdapt_ConnectBus  CMD 0 5 3 7 over\n");

	/* NOTE:
	* =====
	* Each of the following loops is a workaround for a HW bug that will be solved in PG1.1 !!
	* Each write of CMD-52 to function-0 should use it as follows:
	* 1) Write the desired byte using CMD-52
	* 2) Read back the byte using CMD-52
	* 3) Write two dummy bytes to address 0xC8 using CMD-53
	* 4) If the byte read in step 2 is different than the written byte repeat the sequence
	*/

#if 1/* TNETW1283*/
#ifdef SDIO_HIGH_SPEED
	PERR("sdioAdapt_ConnectBus: Set HIGH_SPEED bit on register 0x13\n")
	/* CCCR 13 bit EHS(1) */
	iStatus = sdioDrv_ReadSyncBytes(TXN_FUNC_ID_CTRL, 0x13, &uByte, 1, 1)
	PERR2("After r 0x%x, iStatus=%d \n", uByte, iStatus);
	if (iStatus)
		return iStatus;

	uByte |= 0x2; /* set bit #1 EHS */
	iStatus = sdioDrv_WriteSyncBytes(TXN_FUNC_ID_CTRL, 0x13, &uByte, 1, 1)
	PERR2("After w 0x%x, iStatus=%d \n", uByte, iStatus);
	if (iStatus)
		return iStatus;

	iStatus = sdioDrv_ReadSyncBytes(TXN_FUNC_ID_CTRL, 0x13, &uByte, 1, 1)
	PERR2("After r 0x%x, iStatus=%d \n", uByte, iStatus)
	if (iStatus)
		return iStatus;

	PERR1("After CCCR 0x13, uByte=%d \n", (int)uByte)
#endif
#endif

	/* set device side bus width to 4 bit (for 1 bit write 0x80 instead of 0x82) */
	do {
	uByte = SDIO_BITS_CODE;
	iStatus = sdioDrv_WriteSyncBytes(TXN_FUNC_ID_CTRL, CCCR_BUS_INTERFACE_CONTOROL, &uByte, 1, 1);
	if (iStatus)
		return iStatus;

	iStatus = sdioDrv_ReadSyncBytes(TXN_FUNC_ID_CTRL, CCCR_BUS_INTERFACE_CONTOROL, &uByte, 1, 1);
	if (iStatus)
		return iStatus;

#if 1/*TNETW1283*/
	printk(KERN_INFO "Skip  w 0xC8, iStatus=%d \n", iStatus);
#else
	iStatus = sdioDrv_WriteSync(TXN_FUNC_ID_CTRL, 0xC8, &uLong, 2, 1, 1)
	PERR1("After w 0xC8, after HIGH_SPEED iStatus=%d \n", iStatus);
	if (iStatus)
		return iStatus;
#endif
	uCount++;

	} while ((uByte != SDIO_BITS_CODE) && (uCount < MAX_RETRIES));

	uCount = 0;

	/* allow function 2 */
	do {
	uByte = 4;
	iStatus = sdioDrv_WriteSyncBytes(TXN_FUNC_ID_CTRL, CCCR_IO_ENABLE, &uByte, 1, 1);
	if (iStatus)
		return iStatus;

	iStatus = sdioDrv_ReadSyncBytes(TXN_FUNC_ID_CTRL, CCCR_IO_ENABLE, &uByte, 1, 1);
	if (iStatus)
		return iStatus;

#if 1/*TNETW1283*/
	printk(KERN_INFO "Skip  w 0xC8, after CCCR_IO_ENABLE iStatus=%d \n", iStatus);
#else
	iStatus = sdioDrv_WriteSync(TXN_FUNC_ID_CTRL, 0xC8, &uLong, 2, 1, 1);
	if (iStatus)
		return iStatus;
#endif
	uCount++;

	} while ((uByte != 4) && (uCount < MAX_RETRIES));



	uCount = 0;

	/* set block size for SDIO block mode */
	do {
		uLong = uBlkSize;/*SDIO_BLOCK_SIZE;*/

		iStatus = sdioDrv_WriteSyncBytes(TXN_FUNC_ID_CTRL, FN0_FBR2_REG_108, (unsigned char *)&uLong, 2, 1);
		if (iStatus)
			return iStatus;

		iStatus = sdioDrv_ReadSyncBytes(TXN_FUNC_ID_CTRL, FN0_FBR2_REG_108, (unsigned char *)&uLong, 2, 1);
		if (iStatus)
			return iStatus;

	/*iStatus = sdioDrv_WriteSyncBytes (TXN_FUNC_ID_CTRL, 0xC8, (unsigned char*)&uLong, 2, 1); austin added it for test
	if (iStatus) { return iStatus; } //austin added it for test*/
		/*OutputFormatString("Aus (while test 3) sdioDrv_WriteSync (%d)\n",__LINE__);*/
	/*iStatus = sdioDrv_WriteSync (TXN_FUNC_ID_CTRL, 0xC8, &uLong, 2, 1, 1);*/
	/*iStatus = sdioDrv_WriteSyncBytes (TXN_FUNC_ID_CTRL, 0xC8, (unsigned char*)&uLong, 2, 1);  austin disabled for 1281*/

	uCount++;

	} while (((uLong & FN0_FBR2_REG_108_BIT_MASK) != uBlkSize /*SDIO_BLOCK_SIZE*/) && (uCount < MAX_RETRIES));

	if (uCount >= MAX_RETRIES) {
		/* Failed to write CMD52_WRITE to function 0 */
		return (int)uCount;
	}

	return iStatus;
}

static void wlan_gpio_init(void){
/*	int ret;
	ret = gpio_request(WLAN_EN, "wifi_power");
	if (ret < 0) {
		pr_err("[WLAN] %s: can't reserve GPIO: %d\n", __func__,
			WLAN_EN);
		return;
	}
	tegra_gpio_enable(WLAN_EN);
	gpio_direction_output(WLAN_EN, 0);
*/

/*
	nmk_config_pin(GPIO155_GPIO | PIN_OUTPUT_LOW,false);
	nmk_config_pin(GPIO156_GPIO | PIN_INPUT_PULLUP, false);
	nmk_config_pin(GPIO208_MC1_CLK,false);
	nmk_config_pin(GPIO210_MC1_CMD | PIN_PULL_UP,false);
	nmk_config_pin(GPIO211_MC1_DAT0 | PIN_PULL_UP,false);
	nmk_config_pin(GPIO212_MC1_DAT1 | PIN_PULL_UP,false);
	nmk_config_pin(GPIO213_MC1_DAT2 | PIN_PULL_UP,false);
	nmk_config_pin(GPIO214_MC1_DAT3 | PIN_PULL_UP,false);
*/
}

static void wlan_gpio_deinit(void){
/*	nmk_config_pin(GPIO208_MC1_CLK,false);
	nmk_config_pin(GPIO210_MC1_CMD | PIN_SLPM_INPUT_PULLUP,false);
	nmk_config_pin(GPIO211_MC1_DAT0 | PIN_SLPM_INPUT_PULLUP,false);
	nmk_config_pin(GPIO212_MC1_DAT1 | PIN_SLPM_INPUT_PULLUP,false);
	nmk_config_pin(GPIO213_MC1_DAT2 | PIN_SLPM_INPUT_PULLUP,false);
	nmk_config_pin(GPIO214_MC1_DAT3 | PIN_SLPM_INPUT_PULLUP,false);
*/
}
/*HTC_CSP_END*/

static int sdioDrv_probe(void)
{
	int rc = 0;
/*TODO	struct mmc_ios ios;*/
	/*struct mmc_platform_data* plat = pdev->dev.platform_data;  //austin disabled for build break*/
	struct mmc_host *mmc = 0;

	wlan_gpio_init();

	printk(KERN_INFO "[ %s ] BEGIN\n", __func__);
	printk(KERN_INFO "[ %s ] Probing the controller\n", __func__);
	printk(KERN_INFO "[ %s ] Calling pincfg_sdio_power_on_wifi\n", __func__);

/*TODO 
	config_clk32k_on(1,0);
	msleep(1);	//Wait 3 clock cycle to enable wifi_en

	pincfg_sdio_power_on_wifi();
*/
	/*plat->ocr_mask = TIWLAN_MMC_CONTROLLER; //austin disabled for build break start*/
	/*plat->status = sdioDrv_status;*/
	/*rc = msmsdcc_probe(pdev);
	if (rc)
		return rc;*/	/*austin disabled for build break end*/

	g_drv.pdev = mmci_get_platform_device();
	g_drv.mmc = mmci_get_mmc();

	mmc_claim_host(g_drv.mmc);

	printk(KERN_INFO "SdioDrv_probe  g_drv.pdev:0x%x  g_drv.mmc:0x%x  mmc->index=%d\n",
		   (int)g_drv.pdev, (int)g_drv.mmc, g_drv.mmc->index);

	mmc = g_drv.mmc;
/*TODO	mmc->index = TIWLAN_MMC_CONTROLLER;*/

	printk(KERN_INFO "[ %s ] Setting bus width to %d bits, power on, clock %dHz\n", __func__,
			(TIWLAN_SDIO_BUSWIDE == MMC_BUS_WIDTH_4 ? 4 : 1), TIWLAN_SDIO_CLOCK);
/*TODO	memset(&ios, 0, sizeof(struct mmc_ios));
	ios.bus_width = MMC_BUS_WIDTH_1;//TIWLAN_SDIO_BUSWIDE; bus width 1
	ios.power_mode = MMC_POWER_UP;
	ios.clock = TIWLAN_SDIO_INITIAL_CLOCK;//TIWLAN_SDIO_CLOCK; austin changed it to TIWLAN_SDIO_INITIAL_CLOCK
	ios.vdd = 21;
	ios.bus_mode = MMC_BUSMODE_PUSHPULL;
	ios.chip_select = MMC_CS_DONTCARE;
	ios.timing = MMC_TIMING_LEGACY;

	memcpy(&mmc->ios,&ios,sizeof(struct mmc_ios));
	mmc->ops->set_ios(mmc, &ios);
*/

	int bit;
	mmc->ios.bus_width = MMC_BUS_WIDTH_1;
	mmc->ios.power_mode = MMC_POWER_UP;
	/* If ocr is set, we use it */
	if (mmc->ocr)
		bit = ffs(mmc->ocr) - 1;
	else
		bit = fls(mmc->ocr_avail) - 1;
	mmc->ios.vdd = bit;
	mmc->ios.bus_mode = MMC_BUSMODE_OPENDRAIN;
	mmc->ios.chip_select = MMC_CS_DONTCARE;
	mmc->ios.timing = MMC_TIMING_LEGACY;
	struct mmc_ios *ios;
	ios = &mmc->ios;
	pr_info("[SD] %s: clock %uHz busmode %u powermode %u cs %u Vdd %u "
		"width %u timing %u\n",
		 mmc_hostname(mmc), ios->clock, ios->bus_mode,
		 ios->power_mode, ios->chip_select, ios->vdd,
		 ios->bus_width, ios->timing);
	mmc->ops->set_ios(mmc, ios);

	msleep(100);

/*TODO	memset(&ios, 0, sizeof(struct mmc_ios));
	ios.bus_width = MMC_BUS_WIDTH_1;//TIWLAN_SDIO_BUSWIDE; bus width 1
	ios.power_mode = MMC_POWER_ON;
	ios.clock = TIWLAN_SDIO_INITIAL_CLOCK;//TIWLAN_SDIO_CLOCK; austin changed it to TIWLAN_SDIO_INITIAL_CLOCK
	ios.vdd = 21;
	ios.bus_mode = MMC_BUSMODE_PUSHPULL;
	ios.chip_select = MMC_CS_DONTCARE;
	ios.timing = MMC_TIMING_LEGACY;

	memcpy(&mmc->ios,&ios,sizeof(struct mmc_ios));
	mmc->ops->set_ios(mmc, &ios);
*/

	mmc->ios.clock = mmc->f_min;
	mmc->ios.power_mode = MMC_POWER_ON;
	ios = &mmc->ios;
	pr_info("[SD] %s: clock %uHz busmode %u powermode %u cs %u Vdd %u "
		"width %u timing %u\n",
		 mmc_hostname(mmc), ios->clock, ios->bus_mode,
		 ios->power_mode, ios->chip_select, ios->vdd,
		 ios->bus_width, ios->timing);
	mmc->ops->set_ios(mmc, ios);


	hPlatform_DevicePowerOn();

	rc = sdio_command_init();

	msleep(100);
/*TODO
	memset(&ios, 0, sizeof(struct mmc_ios));
#ifdef SDIO_1_BIT
	ios.bus_width = MMC_BUS_WIDTH_1;
#else
	ios.bus_width = MMC_BUS_WIDTH_4;
#endif
	ios.power_mode = MMC_POWER_ON;
	ios.clock =  20000000;
	ios.vdd = 21;
	ios.bus_mode = MMC_BUSMODE_PUSHPULL;
	ios.chip_select = MMC_CS_DONTCARE;
	ios.timing = MMC_TIMING_LEGACY;

	memcpy(&mmc->ios,&ios,sizeof(struct mmc_ios));
	mmc->ops->set_ios(mmc, &ios);
*/

#ifdef SDIO_1_BIT
	mmc->ios.bus_width = MMC_BUS_WIDTH_1;
#else
	mmc->ios.bus_width = MMC_BUS_WIDTH_4;
#endif
	mmc->ios.power_mode = MMC_POWER_ON;
	mmc->ios.clock =  20000000;
	mmc->ios.vdd = 21;
	mmc->ios.bus_mode = MMC_BUSMODE_PUSHPULL;
	mmc->ios.chip_select = MMC_CS_DONTCARE;
	mmc->ios.timing = MMC_TIMING_LEGACY;
	ios = &mmc->ios;
	pr_info("[SD] %s: clock %uHz busmode %u powermode %u cs %u Vdd %u "
		"width %u timing %u\n",
		 mmc_hostname(mmc), ios->clock, ios->bus_mode,
		 ios->power_mode, ios->chip_select, ios->vdd,
		 ios->bus_width, ios->timing);
	mmc->ops->set_ios(mmc, ios);

	return rc;
}

static int sdioDrv_remove(void)
{

	hPlatform_DevicePowerOff();
/*TODO
	printk(KERN_INFO "[ %s ] Calling pincfg_sdio_power_off_wifi\n", __func__);
	pincfg_sdio_power_off_wifi();
	config_clk32k_on(0,0);
*/
	wlan_gpio_deinit();
	mmc_release_host(g_drv.mmc);

	return 0;
}

void sdioDrv_set_clock_rate(int clock_rate)
{
	struct mmc_host *mmc  = mmci_get_mmc();
/*TODO	struct mmc_ios ios;

	memset(&ios, 0, sizeof(struct mmc_ios));

#ifdef SDIO_1_BIT
	ios.bus_width = MMC_BUS_WIDTH_1;
#else
	ios.bus_width = MMC_BUS_WIDTH_4;
#endif
	printk("sdioDrv_set_clock_rate: clk=%d  buswidth=%d\n",clock_rate,ios.bus_width);

	ios.power_mode = MMC_POWER_ON;

	ios.clock = clock_rate;

	ios.vdd = 21;

	mmc->ops->set_ios(mmc, &ios);
*/

#ifdef SDIO_1_BIT
	mmc->ios.bus_width = MMC_BUS_WIDTH_1;
#else
	mmc->ios.bus_width = MMC_BUS_WIDTH_4;
#endif
	printk(KERN_INFO "sdioDrv_set_clock_rate: clk=%d  buswidth=%d\n", clock_rate, mmc->ios.bus_width);

	mmc->ios.power_mode = MMC_POWER_ON;

	mmc->ios.clock = clock_rate;

	mmc->ios.vdd = 21;
	mmc->ios.timing = MMC_TIMING_LEGACY;
	struct mmc_ios *ios;
	ios = &mmc->ios;
	pr_info("[SD] %s: clock %uHz busmode %u powermode %u cs %u Vdd %u "
		"width %u timing %u\n",
		 mmc_hostname(mmc), ios->clock, ios->bus_mode,
		 ios->power_mode, ios->chip_select, ios->vdd,
		 ios->bus_width, ios->timing);
	mmc->ops->set_ios(mmc, ios);

	msleep(50);
}

int sdiodrv_open(void)
{
	int rc;
	printk(KERN_INFO"sdiodrv_open+\n");
	rc = sdioDrv_probe();

	return rc;
}

int sdiodrv_close(void)
{
	int rc;
	printk(KERN_INFO "sdiodrv close+\n");
	rc = sdioDrv_remove();

	return rc;
}

static struct file_operations sdiodrv_fops = {
	.owner		= THIS_MODULE,
	.open		= sdiodrv_open,
	.release	= sdiodrv_close,
};

static struct miscdevice sdiodrv_device = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "sdiodrv",
	.fops	= &sdiodrv_fops,
};

static int sdiodrv_init(void)
{
	int rc;

	rc = misc_register(&sdiodrv_device);
	if (rc)
		printk(KERN_ERR "sdiodrv: register fail\n");
	return rc;
}

static void sdiodrv_exit(void)
{
	if (misc_deregister(&sdiodrv_device))
		printk(KERN_ERR "sdiodrv: deregister fail\n");
}

module_init(sdiodrv_init);
module_exit(sdiodrv_exit);

MODULE_DESCRIPTION("TI WLAN SDIO driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS(SDIO_DRIVER_NAME);
MODULE_AUTHOR("Texas Instruments Inc");
