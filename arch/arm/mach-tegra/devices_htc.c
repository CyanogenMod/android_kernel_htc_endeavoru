/* linux/arch/arm/mach-msm/devices.c
 *
 * Copyright (C) 2008 Google, Inc.
 * Copyright (C) 2007-2009 HTC Corporation.
 * Author: Thomas Tsai <thomas_tsai@htc.com>
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

/*  ATAG LIST
 * #define ATAG_SMI 0x4d534D71
 * #define ATAG_HWID 0x4d534D72
 * #define ATAG_GS         0x5441001d
 * #define ATAG_PS         0x5441001c
 * #define ATAG_CSA	0x5441001f
 * #define ATAG_CSA	0x5441001f
 * #define ATAG_SKUID 0x4d534D73
 * #define ATAG_HERO_PANEL_TYPE 0x4d534D74
 * #define ATAG_PS_TYPE 0x4d534D77
 * #define ATAG_TP_TYPE 0x4d534D78
 * #define ATAG_ENGINEERID 0x4d534D75
 * #define ATAG_MFG_GPIO_TABLE 0x59504551
 * #define ATAG_MEMSIZE 0x5441001e
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>

#include <linux/dma-mapping.h>
/* #include <mach/msm_iomap.h>
 */
#include <mach/dma.h>
/* #include "gpio_chip.h"
 */
#include "devices.h"
/* #include <mach/board.h>
 */
#include "board.h"
#include <mach/board_htc.h>
/* #include <mach/msm_hsusb.h>
 * #include <linux/usb/mass_storage_function.h>
 */
//#include <linux/usb/android_composite.h>

#include <asm/mach/flash.h>
#include <asm/setup.h>
#if 0
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#endif
#include <linux/delay.h>
#include <linux/android_pmem.h>
#include <linux/proc_fs.h>
/* #include <mach/msm_rpcrouter.h>
 * #include <asm/mach/mmc.h>
 */

#include <mach/gpio.h>
#include "gpio-names.h"

#if defined(CONFIG_MACH_QUATTRO_U) || defined(CONFIG_MACH_VERTEXF)
#define ENGID_XB 0xFFEE
#endif

unsigned long tegra_bootloader_panel_lsb;
unsigned long tegra_bootloader_panel_msb;

static char *df_serialno = "000000000000";
static char *board_sn;
static char *df_mb_serialno = "000000000000";
static char *board_mb_sn;
#define MFG_GPIO_TABLE_MAX_SIZE        0x400
static unsigned char mfg_gpio_table[MFG_GPIO_TABLE_MAX_SIZE];

#define EMMC_FREQ_533 533
#define EMMC_FREQ_400 400


#define ATAG_SMI 0x4d534D71
/* setup calls mach->fixup, then parse_tags, parse_cmdline
 * We need to setup meminfo in mach->fixup, so this function
 * will need to traverse each tag to find smi tag.
 */
int __init parse_tag_smi(const struct tag *tags)
{
	int smi_sz = 0, find = 0;
	struct tag *t = (struct tag *)tags;

	for (; t->hdr.size; t = tag_next(t)) {
		if (t->hdr.tag == ATAG_SMI) {
			printk(KERN_DEBUG "find the smi tag\n");
			find = 1;
			break;
		}
	}
	if (!find)
		return -1;

	printk(KERN_DEBUG "parse_tag_smi: smi size = %d\n", t->u.mem.size);
	smi_sz = t->u.mem.size;
	return smi_sz;
}
__tagtable(ATAG_SMI, parse_tag_smi);

#if defined(CONFIG_MACH_BLUE)
static char logger_status[10];
#define ATAG_LOGGER_STATUS  0x54410100
int __init parse_tag_loggerstatus(const struct tag *tags)
{
	int find = 0;
	char status[10];
	struct tag *t = (struct tag *)tags;

	for (; t->hdr.size; t = tag_next(t)) {
		if (t->hdr.tag == ATAG_LOGGER_STATUS) {
			printk(KERN_DEBUG "find the logger status tag\n");
			find = 1;
			break;
		}
	}
	if (!find)
		return -1;
	else {
		unsigned char *dptr = (unsigned char *)(&t->u);
		memcpy(logger_status, dptr, 10);
		memcpy(status, dptr, 10);
	}
	printk(KERN_DEBUG "parse_tag_loggerstatus: logger status = %s\r\n", status);
	return 0;
}
__tagtable(ATAG_LOGGER_STATUS, parse_tag_loggerstatus);

static ssize_t logger_status_read(struct file *file, char __user *buf,
		size_t len, loff_t *offset)
{
	loff_t pos = *offset;
	ssize_t count;

	int slen = strlen(logger_status);
	if (pos >= slen)
		return 0;
	// 	char lb[1];
	//	sprintf(lb, "%d", logger_status);

	count = min(len, (size_t)(slen - pos));
	if (copy_to_user(buf, logger_status + pos, count))
		return -EFAULT;

	*offset += count;
	return count;
}

static const struct file_operations logger_status_file_ops = {
	.owner = "system",
	.read = logger_status_read,
};

static int __init logger_setting_status(void)
{
	struct proc_dir_entry *entry;
	entry = create_proc_entry("logger_status", S_IFREG | S_IRUGO, NULL);
	if (!entry) {
		printk(KERN_ERR "logger_setting: failed to create proc entry\n");
		return 0;
	}

	entry->proc_fops = &logger_status_file_ops;
	entry->size = 1;
	return 0;
}
late_initcall(logger_setting_status);

static char logger_path[10];
#define ATAG_LOGGER_PATH  0x54410110
int __init parse_tag_loggerpath(const struct tag *tags)
{
	int find = 0;
	char path[10];
	struct tag *t = (struct tag *)tags;

	for (; t->hdr.size; t = tag_next(t)) {
		if (t->hdr.tag == ATAG_LOGGER_PATH) {
			printk(KERN_DEBUG "find the logger path tag\n");
			find = 1;
			break;
		}
	}
	if (!find)
		return -1;
	else {
		unsigned char *dptr = (unsigned char *)(&t->u);
		memcpy(logger_path, dptr, 10);
		memcpy(path, dptr, 10);
	}
	printk(KERN_DEBUG "parse_tag_loggerstatus: logger path = %s\r\n", path);
	return 0;
}
__tagtable(ATAG_LOGGER_PATH, parse_tag_loggerpath);

static ssize_t logger_path_read(struct file *file, char __user *buf,
		size_t len, loff_t *offset)
{
	loff_t pos = *offset;
	ssize_t count;

	int slen = strlen(logger_path);
	if (pos >= slen)
		return 0;

	//	char lb[8];
	//	sprintf(lb, "%d", logger_path);

	count = min(len, (size_t)(slen - pos));
	if (copy_to_user(buf, logger_path + pos, count))
		return -EFAULT;

	*offset += count;
	return count;
}

static const struct file_operations logger_path_file_ops = {
	.owner = "system",
	.read = logger_path_read,
};

static int __init logger_setting_path(void)
{
	struct proc_dir_entry *entry;
	entry = create_proc_entry("logger_path", S_IFREG | S_IRUGO, NULL);
	if (!entry) {
		printk(KERN_ERR "logger_setting: failed to create proc entry\n");
		return 0;
	}

	entry->proc_fops = &logger_path_file_ops;
	entry->size = 1;
	return 0;
}
late_initcall(logger_setting_path);
#endif

#define ATAG_HWID 0x4d534D72
int __init parse_tag_hwid(const struct tag *tags)
{
	int hwid = 0, find = 0;
	struct tag *t = (struct tag *)tags;

	for (; t->hdr.size; t = tag_next(t)) {
		if (t->hdr.tag == ATAG_HWID) {
			printk(KERN_DEBUG "find the hwid tag\n");
			find = 1;
			break;
		}
	}

	if (find)
		hwid = t->u.revision.rev;
	printk(KERN_DEBUG "parse_tag_hwid: hwid = 0x%x\n", hwid);
	return hwid;
}
__tagtable(ATAG_HWID, parse_tag_hwid);

static char *keycap_tag = NULL;
static int __init board_keycaps_tag(char *get_keypads)
{
	if (strlen(get_keypads))
		keycap_tag = get_keypads;
	else
		keycap_tag = NULL;
	return 1;
}
__setup("androidboot.keycaps=", board_keycaps_tag);

void board_get_keycaps_tag(char **ret_data)
{
	*ret_data = keycap_tag;
}
EXPORT_SYMBOL(board_get_keycaps_tag);

static char *cid_tag = NULL;
static int __init board_set_cid_tag(char *get_hboot_cid)
{
	if (strlen(get_hboot_cid))
		cid_tag = get_hboot_cid;
	else
		cid_tag = NULL;
	return 1;
}
__setup("androidboot.cid=", board_set_cid_tag);

void board_get_cid_tag(char **ret_data)
{
	*ret_data = cid_tag;
}
EXPORT_SYMBOL(board_get_cid_tag);

static char *carrier_tag = NULL;
static int __init board_set_carrier_tag(char *get_hboot_carrier)
{
	if (strlen(get_hboot_carrier))
		carrier_tag = get_hboot_carrier;
	else
		carrier_tag = NULL;
	return 1;
}
__setup("androidboot.carrier=", board_set_carrier_tag);

void board_get_carrier_tag(char **ret_data)
{
	*ret_data = carrier_tag;
}
EXPORT_SYMBOL(board_get_carrier_tag);

/* G-Sensor calibration value */
#define ATAG_GS         0x5441001d

unsigned int gs_kvalue;
EXPORT_SYMBOL(gs_kvalue);

static int __init parse_tag_gs_calibration(const struct tag *tag)
{
	gs_kvalue = tag->u.revision.rev;
	printk(KERN_DEBUG "%s: gs_kvalue = 0x%x\n", __func__, gs_kvalue);
	return 0;
}

__tagtable(ATAG_GS, parse_tag_gs_calibration);

/* Proximity sensor calibration values */
#define ATAG_PS         0x5441001c

unsigned int ps_kparam1;
EXPORT_SYMBOL(ps_kparam1);

unsigned int ps_kparam2;
EXPORT_SYMBOL(ps_kparam2);

static int __init parse_tag_ps_calibration(const struct tag *tag)
{
	ps_kparam1 = tag->u.serialnr.low;
	ps_kparam2 = tag->u.serialnr.high;

	printk(KERN_INFO "%s: ps_kparam1 = 0x%x, ps_kparam2 = 0x%x\n",
		__func__, ps_kparam1, ps_kparam2);

	return 0;
}

__tagtable(ATAG_PS, parse_tag_ps_calibration);

#if 0 
defined(CONFIG_MACH_QUATTRO_U) || defined(CONFIG_MACH_VERTEXF)
unsigned int als_kadc_htc;
EXPORT_SYMBOL(als_kadc_htc);

static int __init parse_tag_als_calibration(const struct tag *tag)
{
	als_kadc_htc = tag->u.als_kadc.kadc;

	printk(KERN_INFO "%s: als_kadc = 0x%x\n",
		__func__, als_kadc_htc);

	return 0;
}
__tagtable(ATAG_ALS, parse_tag_als_calibration);
#endif

#if defined(CONFIG_MACH_ENDEAVORU) || defined(CONFIG_MACH_ENDEAVORTD) || defined(CONFIG_MACH_BLUE) || defined(CONFIG_MACH_QUATTRO_U) || defined(CONFIG_MACH_VERTEXF)
unsigned int als_kadc;
EXPORT_SYMBOL(als_kadc);

static int __init parse_tag_als_calibration(const struct tag *tag)
{
	als_kadc = tag->u.als_kadc.kadc;

	printk(KERN_INFO "%s: als_kadc = 0x%x\n",
		__func__, als_kadc);

	return 0;
}
__tagtable(ATAG_ALS, parse_tag_als_calibration);
#endif



/* CSA sensor calibration values */
#define ATAG_CSA	0x5441001f

unsigned int csa_kvalue1;
EXPORT_SYMBOL(csa_kvalue1);

unsigned int csa_kvalue2;
EXPORT_SYMBOL(csa_kvalue2);

unsigned int csa_kvalue3;
EXPORT_SYMBOL(csa_kvalue3);

unsigned int csa_kvalue4;
EXPORT_SYMBOL(csa_kvalue4);

unsigned int csa_kvalue5;
EXPORT_SYMBOL(csa_kvalue5);

static int __init parse_tag_csa_calibration(const struct tag *tag)
{
	unsigned int *ptr = (unsigned int *)&tag->u;
	csa_kvalue1 = ptr[0];
	csa_kvalue2 = ptr[1];
	csa_kvalue3 = ptr[2];
	csa_kvalue4 = ptr[3];
	csa_kvalue5 = ptr[4];

	printk(KERN_DEBUG "csa_kvalue1 = 0x%x, csa_kvalue2 = 0x%x, "
		"csa_kvalue3 = 0x%x, csa_kvalue4 = 0x%x, csa_kvalue5 = 0x%x\n", 
		csa_kvalue1, csa_kvalue2, csa_kvalue3, csa_kvalue4, csa_kvalue5);

	return 0;
}
__tagtable(ATAG_CSA, parse_tag_csa_calibration);

#if defined(CONFIG_MACH_BLUE)
/* Hand sensor calibration values */
#define ATAG_HSNR	0x4e540000
#define PAD_CNT		9

uint8_t hsnr_kidacs[PAD_CNT];
EXPORT_SYMBOL(hsnr_kidacs);

uint8_t hsnr_kthds[PAD_CNT];
EXPORT_SYMBOL(hsnr_kthds);

uint16_t hsnr_kwoods[PAD_CNT];
EXPORT_SYMBOL(hsnr_kwoods);

uint8_t hsnr_kthd_done;
EXPORT_SYMBOL(hsnr_kthd_done);

uint8_t hsnr_kidac_done;
EXPORT_SYMBOL(hsnr_kidac_done);

static int __init parse_tag_hsnr_calibration(const struct tag *tag)
{
	int i;
	unsigned char hsnr_kvalues[96];
	unsigned char *ptr = (unsigned char *)&tag->u;
	memcpy(&hsnr_kvalues[0], ptr, sizeof(hsnr_kvalues));

	memcpy(&hsnr_kidacs[0], hsnr_kvalues, sizeof(hsnr_kidacs));
	memcpy(&hsnr_kthds[0], hsnr_kvalues + 46, sizeof(hsnr_kthds));
	memcpy(&hsnr_kthd_done, hsnr_kvalues + 60, sizeof(hsnr_kthd_done)); 
	memcpy(&hsnr_kidac_done, hsnr_kvalues + 61, sizeof(hsnr_kidac_done));
	memcpy(&hsnr_kwoods[0], hsnr_kvalues + 64, sizeof (hsnr_kwoods));
#if 0
	printk(KERN_DEBUG "hsnr kidac\n");

	for (i = 0; i < PAD_CNT; i++)
		printk(KERN_DEBUG "[%d]:0x%02x", i, hsnr_kidacs[i]);

	printk(KERN_DEBUG "hsnr kthd\n");
	for (i = 0; i < PAD_CNT; i++)
		printk(KERN_DEBUG "[%d]:0x%02x", i, hsnr_kthds[i]);

	printk(KERN_DEBUG "hsnr kwood\n");
	for (i = 0; i < PAD_CNT; i++)
		printk(KERN_DEBUG "[%d]:0x%04x", i, hsnr_kwoods[i]);


	printk(KERN_DEBUG "hsnr_kthd_done: 0x%x\n", hsnr_kthd_done);

	printk(KERN_DEBUG "hsnr_kidac_done: 0x%x\n", hsnr_kidac_done);

	
#endif
	return 0;
}
__tagtable(ATAG_HSNR, parse_tag_hsnr_calibration);
#endif

#ifdef CAMERA_CALIBRATION
/* camera AWB calibration values */
#define ATAG_CAM_AWB    0x59504550
unsigned char awb_kvalues[2048];
EXPORT_SYMBOL(awb_kvalues);

static int __init parse_tag_awb_calibration(const struct tag *tag)
{
    printk(KERN_INFO "[CAM] %s: read MFG calibration data\n", __func__);
    unsigned char *ptr = (unsigned char *)&tag->u;

    memcpy(&awb_kvalues[0], ptr, sizeof(awb_kvalues));

    return 0;
}
__tagtable(ATAG_CAM_AWB, parse_tag_awb_calibration);
#endif

/* Gyro/G-senosr calibration values */
#define ATAG_GRYO_GSENSOR	0x54410020
unsigned char gyro_gsensor_kvalue[37];
EXPORT_SYMBOL(gyro_gsensor_kvalue);

static int __init parse_tag_gyro_gsensor_calibration(const struct tag *tag)
{
	int i;
	unsigned char *ptr = (unsigned char *)&tag->u;
	memcpy(&gyro_gsensor_kvalue[0], ptr, sizeof(gyro_gsensor_kvalue));
#if 0
	printk(KERN_DEBUG "gyro_gs data\n");
	for (i = 0; i < sizeof(gyro_gsensor_kvalue); i++)
		printk(KERN_DEBUG "[%d]:0x%x", i, gyro_gsensor_kvalue[i]);
#endif
	return 0;
}
__tagtable(ATAG_GRYO_GSENSOR, parse_tag_gyro_gsensor_calibration);

static int mfg_mode;
int __init board_mfg_mode_init(char *s)
{
	if (!strcmp(s, "normal"))
		mfg_mode = 0;
	else if (!strcmp(s, "factory2"))
		mfg_mode = 1;
	else if (!strcmp(s, "recovery"))
		mfg_mode = 2;
	else if (!strcmp(s, "charge"))
		mfg_mode = 3;
	else if (!strcmp(s, "power_test"))
		mfg_mode = 4;
	else if (!strcmp(s, "offmode_charging"))
		mfg_mode = 5;

	return 1;
}

int board_mfg_mode(void)
{
	return mfg_mode;
}

EXPORT_SYMBOL(board_mfg_mode);

__setup("androidboot.mode=", board_mfg_mode_init);

static int zchg_mode = 0;
int __init board_zchg_mode_init(char *s)
{
	if (!strcmp(s, "1"))
		zchg_mode = 1;
	else if (!strcmp(s, "2"))
		zchg_mode = 2;
	else if (!strcmp(s, "3"))
		zchg_mode = 3;

	return 1;
}

int board_zchg_mode(void)
{
	return zchg_mode;
}

EXPORT_SYMBOL(board_zchg_mode);
__setup("enable_zcharge=", board_zchg_mode_init);

static int build_flag;

static int __init board_bootloader_setup(char *str)
{
	char temp[strlen(str) + 1];
	char *p = NULL;
	char *build = NULL;
	char *args = temp;

	printk(KERN_INFO "%s: %s\n", __func__, str);

	strcpy(temp, str);

	/*parse the last parameter*/
	while ((p = strsep(&args, ".")) != NULL) build = p;

	if (build) {
		if (strcmp(build, "0000") == 0) {
			printk(KERN_INFO "%s: SHIP BUILD\n", __func__);
			build_flag = SHIP_BUILD;
		} else if (strcmp(build, "2000") == 0) {
			printk(KERN_INFO "%s: ENG BUILD\n", __func__);
			build_flag = ENG_BUILD;
		} else {
			printk(KERN_INFO "%s: default ENG BUILD\n", __func__);
			build_flag = ENG_BUILD;
		}
	}
	return 1;
}
__setup("androidboot.bootloader=", board_bootloader_setup);

int board_build_flag(void)
{
	return build_flag;
}

EXPORT_SYMBOL(board_build_flag);

static int __init board_serialno_setup(char *serialno)
{
	char *str;

	/* use default serial number when mode is factory2 */
	if (board_mfg_mode() == 1 || !strlen(serialno))
		str = df_serialno;
	else
		str = serialno;
#ifdef CONFIG_USB_FUNCTION
	msm_hsusb_pdata.serial_number = str;
#endif
	board_sn = str;
	return 1;
}
__setup("androidboot.serialno=", board_serialno_setup);

static int __init board_mb_serialno_setup(char *serialno)
{
	char *str;

	/* use default serial number when mode is factory2 */
	if (board_mfg_mode() == 1 || !strlen(serialno))
		str = df_mb_serialno;
	else
		str = serialno;
	board_mb_sn = str;
	return 1;
}
__setup("androidboot.mb_serialno=", board_mb_serialno_setup);

char *board_serialno(void)
{
	return board_sn;
}

char *board_mb_serialno(void)
{
	return board_mb_sn;
}

static int sku_id;
int board_get_sku_tag()
{
	return sku_id;
}

#define ATAG_SKUID 0x4d534D73
int __init parse_tag_skuid(const struct tag *tags)
{
	int skuid = 0, find = 0;
	struct tag *t = (struct tag *)tags;

	for (; t->hdr.size; t = tag_next(t)) {
		if (t->hdr.tag == ATAG_SKUID) {
			printk(KERN_DEBUG "find the skuid tag\n");
			find = 1;
			break;
		}
	}

	if (find) {
		skuid = t->u.revision.rev;
		sku_id = skuid;
	}
	printk(KERN_DEBUG "parse_tag_skuid: hwid = 0x%x\n", skuid);
	return skuid;
}
__tagtable(ATAG_SKUID, parse_tag_skuid);

#define ATAG_HERO_PANEL_TYPE 0x4d534D74
int panel_type;
int __init tag_panel_parsing(const struct tag *tags)
{
	panel_type = tags->u.revision.rev;

	printk(KERN_DEBUG "%s: panel type = %d\n", __func__,
		panel_type);

	return panel_type;
}
__tagtable(ATAG_HERO_PANEL_TYPE, tag_panel_parsing);

/* ISL29028 ID values */
//#if defined(CONFIG_MACH_ENDEAVORU) || defined(CONFIG_MACH_ENDEAVORTD)
#define ATAG_PS_TYPE 0x4d534D77
int ps_type;
EXPORT_SYMBOL(ps_type);
int __init tag_ps_parsing(const struct tag *tags)
{
	ps_type = tags->u.revision.rev;

	printk(KERN_DEBUG "%s: PS type = 0x%x\n", __func__,
		ps_type);

	return ps_type;
}
__tagtable(ATAG_PS_TYPE, tag_ps_parsing);
//#endif

 /* Touch Controller ID values */
#define ATAG_TP_TYPE 0x4d534D78
int tp_type;
EXPORT_SYMBOL(tp_type);
int __init tag_tp_parsing(const struct tag *tags)
{
	tp_type = tags->u.revision.rev;

	printk(KERN_DEBUG "%s: TS type = 0x%x\n", __func__,
		tp_type);

	return tp_type;
}
__tagtable(ATAG_TP_TYPE, tag_tp_parsing);


#define ATAG_ENGINEERID 0x4d534D75
unsigned engineer_id;
EXPORT_SYMBOL(engineer_id);
int __init parse_tag_engineerid(const struct tag *tags)
{
	int engineerid = 0, find = 0;
	struct tag *t = (struct tag *)tags;

	for (; t->hdr.size; t = tag_next(t)) {
		if (t->hdr.tag == ATAG_ENGINEERID) {
			printk(KERN_DEBUG "find the engineer tag\n");
			find = 1;
			break;
		}
	}

	if (find) {
		engineer_id = t->u.revision.rev;
		engineerid = t->u.revision.rev;
	}
	printk(KERN_DEBUG "parse_tag_engineerid: 0x%x\n", engineerid);
	return engineerid;
}
__tagtable(ATAG_ENGINEERID, parse_tag_engineerid);

#define ATAG_PCBID 0x4d534D76
unsigned char pcbid = PROJECT_PHASE_INVALID;
int __init parse_tag_pcbid(const struct tag *tags)
{
	int find = 0;
	struct tag *t = (struct tag *)tags;

	for (; t->hdr.size; t = tag_next(t)) {
		if (t->hdr.tag == ATAG_PCBID) {
			printk(KERN_DEBUG "found the pcbid tag\n");
			find = 1;
			break;
		}
	}

	if (find) {
		pcbid = t->u.revision.rev;
	}
	printk(KERN_DEBUG "parse_tag_pcbid: 0x%x\n", pcbid);
	return pcbid;
}
__tagtable(ATAG_PCBID, parse_tag_pcbid);

#define ATAG_MFG_GPIO_TABLE 0x59504551
int __init parse_tag_mfg_gpio_table(const struct tag *tags)
{
	   unsigned char *dptr = (unsigned char *)(&tags->u);
	   __u32 size;

	   size = min((__u32)(tags->hdr.size - 2) * sizeof(__u32), (__u32)MFG_GPIO_TABLE_MAX_SIZE);
	   memcpy(mfg_gpio_table, dptr, size);
	   return 0;
}
__tagtable(ATAG_MFG_GPIO_TABLE, parse_tag_mfg_gpio_table);

char *board_get_mfg_sleep_gpio_table(void)
{
		return mfg_gpio_table;
}
EXPORT_SYMBOL(board_get_mfg_sleep_gpio_table);

static char *emmc_tag;
static int __init board_set_emmc_tag(char *get_hboot_emmc)
{
	if (strlen(get_hboot_emmc))
		emmc_tag = get_hboot_emmc;
	else
		emmc_tag = NULL;
	return 1;
}
__setup("androidboot.emmc=", board_set_emmc_tag);

int board_emmc_boot(void)
{
	if (emmc_tag) {
		if (!strcmp(emmc_tag, "true"))
	return 1;
}

	return 0;
}

#define ATAG_MEMSIZE 0x5441001e
unsigned memory_size;
int __init parse_tag_memsize(const struct tag *tags)
{
	int mem_size = 0, find = 0;
	struct tag *t = (struct tag *)tags;

	for (; t->hdr.size; t = tag_next(t)) {
		if (t->hdr.tag == ATAG_MEMSIZE) {
			printk(KERN_DEBUG "find the memsize tag\n");
			find = 1;
			break;
		}
	}

	if (find) {
		memory_size = t->u.revision.rev;
		mem_size = t->u.revision.rev;
	}
	printk(KERN_DEBUG "parse_tag_memsize: %d\n", memory_size);
	return mem_size;
}
__tagtable(ATAG_MEMSIZE, parse_tag_memsize);

int __init parse_tag_extdiag(const struct tag *tags)
{
	const struct tag *t = tags;

	for (; t->hdr.size; t = tag_next(t)) {
		if (t->hdr.tag == 0x54410021)
			return t->u.revision.rev;
	}
	return 0;
}

#if defined(CONFIG_ARCH_MSM8X60)
static struct msm_mem_settings *board_find_mem_settings(unsigned mem_size_mb)
{
	int index;
	for (index = 0; index < sizeof(mem_settings) / sizeof(mem_settings[0]); index++) {
		if (mem_settings[index].mem_size_mb == mem_size_mb) {
			pr_info("%s: %d MB settings is found.\n", __func__, mem_size_mb);
			return &mem_settings[index];
		}
	}
	pr_info("%s: use default mem bank settigs.\n", __func__);
	return &mem_settings[0];
}

int msm_fixup(struct tag *tags, struct meminfo *mi)
{
	unsigned mem_size_mb = parse_tag_memsize((const struct tag *)tags);
	struct msm_mem_settings *settings = board_find_mem_settings(mem_size_mb);
	int index = 0;

	pr_info("%s: mem size = %d\n", __func__, mem_size_mb);

	mi->nr_banks = settings->mem_info.nr_banks;
	for (index = 0; index < settings->mem_info.nr_banks; index++) {
		mi->bank[index].start = settings->mem_info.bank[index].start;
		mi->bank[index].node = settings->mem_info.bank[index].node;
		mi->bank[index].size = settings->mem_info.bank[index].size;
	}
	return 0;
}
#endif

static unsigned int radio_flag = 0;
int __init radio_flag_init(char *s)
{
	strict_strtoul(s, 16, &radio_flag);
	return 1;
}
__setup("radioflag=", radio_flag_init);

unsigned int get_radio_flag(void)
{
	return radio_flag;
}

static unsigned int kernel_flag = 0;
int __init kernel_flag_init(char *s)
{
	strict_strtoul(s, 16, &kernel_flag);
	return 1;
}
__setup("kernelflag=", kernel_flag_init);

unsigned int get_kernel_flag(void)
{
	return kernel_flag;
}

static unsigned int extra_kernel_flag = 0;
int __init extra_kernel_flag_init(char *s)
{
	strict_strtoul(s, 16, &extra_kernel_flag);
	return 1;
}
__setup("kernelflagex=", extra_kernel_flag_init);

unsigned int get_extra_kernel_flag(void)
{
	return extra_kernel_flag;
}

BLOCKING_NOTIFIER_HEAD(psensor_notifier_list);

int register_notifier_by_psensor(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&psensor_notifier_list, nb);
}

int unregister_notifier_by_psensor(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&psensor_notifier_list, nb);
}

static int __init tegra_bootloader_panel_arg(char *options)
{
	char *p = options;

	tegra_bootloader_panel_lsb = memparse(p, &p);
	if (*p == '@')
		tegra_bootloader_panel_msb = memparse(p+1, &p);

	pr_info("Found panel_vendor: %08lx@%08lx\n",
		tegra_bootloader_panel_lsb, tegra_bootloader_panel_msb);

	return 0;
}
early_param("panel_vendor", tegra_bootloader_panel_arg);

/* should call only one time */
static int __htc_get_pcbid_info(void)
{
#if defined(CONFIG_MACH_ENDEAVORU) || defined(CONFIG_MACH_ENDEAVORTD)
	switch (pcbid)
	{
		case 0:
			return PROJECT_PHASE_XA;
		case 1:
			return PROJECT_PHASE_XB;
		case 2:
			return PROJECT_PHASE_XC;
		case 3:
			return PROJECT_PHASE_XD;
		case 4:
			return PROJECT_PHASE_XE;
		case 5:
			return PROJECT_PHASE_XF;
		case 6:
			return PROJECT_PHASE_XG;
		case 7:
			return PROJECT_PHASE_XH;
		default:
			return pcbid;
	}

#elif defined(CONFIG_MACH_QUATTRO_U) || defined(CONFIG_MACH_VERTEXF)
	int err;
	int pinSixValue   = 0;
	int pinSevenValue = 0;

	switch (pcbid)
	{
		case 0:
		case 1:
			break;
		case 2:
			return PROJECT_PHASE_XC;
		case 3:
			return PROJECT_PHASE_XD;
		case 4:
			return PROJECT_PHASE_XE;
		case 5:
			return PROJECT_PHASE_XF;
		case 6:
			return PROJECT_PHASE_XG;
		case 7:
			return PROJECT_PHASE_XH;
		default:
			return pcbid;
	}

	static const struct gpio pcbid_info_gpios[] = {
		{ TEGRA_GPIO_PG6, GPIOF_IN, "TEGRA_GPIO_PG6" },
		{ TEGRA_GPIO_PG7, GPIOF_IN, "TEGRA_GPIO_PG7" },
	};
	err = gpio_request_array(pcbid_info_gpios,
		ARRAY_SIZE(pcbid_info_gpios));
	if (err < 0) {
		pr_err("%s - gpio_request_array failed\n", __func__);
		return PROJECT_PHASE_INVALID;
	}
	tegra_gpio_enable(TEGRA_GPIO_PG6);
	tegra_gpio_enable(TEGRA_GPIO_PG7);

	pinSixValue   = gpio_get_value(TEGRA_GPIO_PG6);
	pinSevenValue = gpio_get_value(TEGRA_GPIO_PG7);

#ifdef DEBUG_GET_PROJ_PHASE
	pr_info("pinSixValue: %d\n", pinSixValue);
	pr_info("pinSevenValue: %d\n", pinSevenValue);
	pr_info("engineer_id: %d\n", engineer_id);
#endif

	switch (engineer_id) {
		case 0x0:
			if (pinSixValue & pinSevenValue)
				return PROJECT_PHASE_XA;
			else
				return PROJECT_PHASE_EVM;
		case ENGID_XB:
			return PROJECT_PHASE_XB;
		default:
			return PROJECT_PHASE_INVALID;
	}
#endif
}

static char* __pcbid_to_name(signed int id)
{
	switch (id)
	{
	case PROJECT_PHASE_INVALID: return "INVALID";
	case PROJECT_PHASE_EVM:     return "EVM";
	case PROJECT_PHASE_XA:      return "XA";
	case PROJECT_PHASE_XB:      return "XB";
	case PROJECT_PHASE_XC:      return "XC";
	case PROJECT_PHASE_XD:      return "XD";
	case PROJECT_PHASE_XE:      return "XE";
	case PROJECT_PHASE_XF:      return "XF";
	case PROJECT_PHASE_XG:      return "XG";
	case PROJECT_PHASE_XH:      return "XH";
	default:
		return "<Latest HW phase>";
	}
}

const int htc_get_pcbid_info(void)
{
	static int __pcbid = PROJECT_PHASE_INVALID;
	if (__pcbid == PROJECT_PHASE_INVALID)
	{
		__pcbid = __htc_get_pcbid_info();
		pr_info("[hTC info] project phase: %s (id=%d)\n",
				__pcbid_to_name(__pcbid), __pcbid);
	}
	return __pcbid;
}

#define ENG_ID_MODEM_REWORK 0xCA0F
const bool is_modem_rework_phase()
{
    return (htc_get_pcbid_info() == PROJECT_PHASE_XC) &&
        (engineer_id == ENG_ID_MODEM_REWORK);
}

#ifdef CONFIG_DEBUG_LL_DYNAMIC
bool enable_debug_ll = false;
static int __init board_set_debug_ll(char *val)
{
       pr_debug("%s: low level debug: on\n", __func__);
       enable_debug_ll = true;
       return 1;
}
__setup("debug_ll", board_set_debug_ll);
#endif

static unsigned int bl_ac_flag = 0;
int __init bl_ac_flag_init(char *s)
{
	bl_ac_flag=1;
	return 1;
}
__setup("bl_ac_in", bl_ac_flag_init);

unsigned int get_bl_ac_in_flag(void)
{
	return bl_ac_flag;
}
