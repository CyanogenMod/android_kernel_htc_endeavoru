/*
 * linux/arch/arm/mach-tegra/devices_htc.c
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

/*
 * ATAG LIST
 */
#define ATAG_SMI             0x4d534D71
#define ATAG_HWID            0x4d534D72
#define ATAG_GS              0x5441001d
#define ATAG_PS              0x5441001c
#define ATAG_CSA             0x5441001f
#define ATAG_CSA             0x5441001f
#define ATAG_SKUID           0x4d534D73
#define ATAG_PS_TYPE         0x4d534D77
#define ATAG_TP_TYPE         0x4d534D78
#define ATAG_ENGINEERID      0x4d534D75
#define ATAG_MFG_GPIO_TABLE  0x59504551
#define ATAG_MEMSIZE         0x5441001e
#define ATAG_CAM_AWB         0x59504550
#define ATAG_GRYO_GSENSOR    0x54410020
#define ATAG_PCBID           0x4d534D76

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/reboot.h>
#include <linux/cpu.h>
#include <linux/sched.h>

#include <mach/dma.h>
#include <mach/board_htc.h>
#include <mach/gpio.h>
#include <mach/restart.h>
#include <mach/mfootprint.h>

#include <asm/mach/flash.h>
#include <asm/setup.h>
#include <asm/mach-types.h>

#include "gpio-names.h"
#include "devices.h"
#include "board.h"
#include <asm/uaccess.h>

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

/* +FIXME: workaround for imc radio secbin code not ready */
static char IMEI[16];
#define ATAG_IMEI  0x54410120
int __init parse_tag_IMEI(const struct tag *tags)
{
	int find = 0;
	struct tag *t = (struct tag *)tags;

	for (; t->hdr.size; t = tag_next(t)) {
		if (t->hdr.tag == ATAG_IMEI) {
			printk(KERN_DEBUG "[IMEI] find the IMEI tag\n");
			find = 1;
			break;
		}
	}
	if (!find) {
		printk(KERN_ERR "[IMEI] parse_tag_IMEI(): error: IMEI ATAG not found\n", IMEI);
		return -1;
	}
	else {
		unsigned char *dptr = (unsigned char *)(&t->u);
		memcpy(IMEI, dptr, 16);
	}
	printk(KERN_DEBUG "[IMEI] parse_tag_IMEI(): IMEI = %s\n", IMEI);
	return 0;
}
__tagtable(ATAG_IMEI, parse_tag_IMEI);

static ssize_t IMEI_read(struct file *file, char __user *buf, size_t len, loff_t *offset)
{
	loff_t pos = *offset;
	ssize_t count;

	int slen = strlen(IMEI);
	if (pos >= slen)
		return 0;

	count = min(len, (size_t)(slen - pos));
	if (copy_to_user(buf, IMEI + pos, count)) {
		printk(KERN_ERR "[IMEI] IMEI_read(): error: copy_to_user() failed\n");
		return -EFAULT;
	}

	*offset += count;
	return count;
}

static const struct file_operations IMEI_file_ops = {
	.owner = "system",
	.read = IMEI_read,
};

static int __init IMEI_setting(void)
{
	struct proc_dir_entry *entry;
	entry = create_proc_entry("IMEI", S_IFREG | S_IRUGO, NULL);
	if (!entry) {
		printk(KERN_ERR "[IMEI] IMEI_setting(): error: failed to create proc entry\n");
		return 0;
	}

	entry->proc_fops = &IMEI_file_ops;
	entry->size = 1;
	return 0;
}
late_initcall(IMEI_setting);
/* -FIXME: workaround for imc radio secbin code not ready */


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

BLOCKING_NOTIFIER_HEAD(psensor_notifier_list);

int register_notifier_by_psensor(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&psensor_notifier_list, nb);
}

int unregister_notifier_by_psensor(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&psensor_notifier_list, nb);
}

/* CSA sensor calibration values */

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

#if defined(CONFIG_MACH_OPERAUL)
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
unsigned char gyro_gsensor_kvalue[37];
EXPORT_SYMBOL(gyro_gsensor_kvalue);

static int __init parse_tag_gyro_gsensor_calibration(const struct tag *tag)
{
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
		mfg_mode = BOARD_MFG_MODE_NORMAL;
	else if (!strcmp(s, "factory2"))
		mfg_mode = BOARD_MFG_MODE_FACTORY2;
	else if (!strcmp(s, "recovery"))
		mfg_mode = BOARD_MFG_MODE_RECOVERY;
	else if (!strcmp(s, "charge"))
		mfg_mode = BOARD_MFG_MODE_CHARGE;
	else if (!strcmp(s, "power_test"))
		mfg_mode = BOARD_MFG_MODE_POWERTEST;
	else if (!strcmp(s, "offmode_charging"))
		mfg_mode = BOARD_MFG_MODE_OFFMODE_CHARGING;
	else if (!strcmp(s, "mfgkernel"))
		mfg_mode = BOARD_MFG_MODE_MFGKERNEL;
	else if (!strcmp(s, "modem_calibration"))
		mfg_mode = BOARD_MFG_MODE_MODEM_CALIBRATION;

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

static unsigned int sf = 0;
int __init board_sf_init(char *s)
{
	sf = simple_strtoul(s, 0, 10);
	return 1;
}

int get_tamper_sf(void)
{
	return sf;
}

EXPORT_SYMBOL(get_tamper_sf);
__setup("androidboot.sf=", board_sf_init);

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
	if (board_mfg_mode() == BOARD_MFG_MODE_FACTORY2 || !strlen(serialno))
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
	if (board_mfg_mode() == BOARD_MFG_MODE_FACTORY2 || !strlen(serialno))
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

static unsigned int sku_id;
static char SKUID[16];
int board_get_sku_tag()
{
	return sku_id;
}
EXPORT_SYMBOL(board_get_sku_tag);
int __init parse_tag_skuid(const struct tag *tags)
{
	int find = 0;
	struct tag *t = (struct tag *)tags;

	for (; t->hdr.size; t = tag_next(t)) {
		if (t->hdr.tag == ATAG_SKUID) {
			printk(KERN_DEBUG "find the skuid tag\n");
			find = 1;
			break;
		}
	}

	if (find) {
		unsigned char *dptr = (unsigned char *)(&t->u);
		memcpy(SKUID, dptr, 16);

		unsigned int sku_id_int = 0;
		char *ptr = SKUID + sizeof(SKUID) - 1;
		int rate = 1;
		int index= 0;
		for (index = 0;index < sizeof(SKUID);index++) {
			if (NULL == *ptr) {
				ptr--;
				continue;
			}
			if (*ptr >= '0' && *ptr <= '9')
				sku_id_int += (*ptr - '0') * rate;
			else if (*ptr >= 'a' && *ptr <= 'f')
				sku_id_int += ((*ptr - 'a') + 10) * rate;
			else if (*ptr >= 'A' && *ptr <= 'F')
				sku_id_int += ((*ptr - 'A') + 10) * rate;
			ptr--;
			rate *= 16;
		}
		sku_id = sku_id_int;
	}
	printk(KERN_INFO "parse_tag_skuid: 0x%s\n", SKUID);
	printk(KERN_INFO "parse_tag_skuid: 0x%x\n", sku_id);
	return 0;
}
__tagtable(ATAG_SKUID, parse_tag_skuid);
static ssize_t SKUID_read(struct file *file, char __user *buf,
		size_t len, loff_t *offset)
{
	loff_t pos = *offset;
	ssize_t count;
	int slen = strlen(SKUID);
	if (pos >= slen)
		return 0;

	count = min(len, (size_t)(slen - pos));
	if (copy_to_user(buf, SKUID + pos, count)) {
		printk(KERN_ERR "[SKUID] SKUID_read(): error: copy_to_user() failed\n");
		return -EFAULT;
	}

	*offset += count;
	return count;
}

static const struct file_operations SKUID_file_ops = {
	.owner = "system",
	.read = SKUID_read,
};

static int __init SKUID_setting(void)
{
	struct proc_dir_entry *entry;
	entry = create_proc_entry("SKUID", S_IFREG | S_IRUGO, NULL);
	if (!entry) {
		printk(KERN_ERR "[SKUID] SKUID_setting(): error: failed to create proc entry\n");
		return 0;
	}

	entry->proc_fops = &SKUID_file_ops;
	entry->size = 1;
	return 0;
}
late_initcall(SKUID_setting);

/* ISL29028 ID values */
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

 /* Touch Controller ID values */
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
	pr_info("parse_tag_engineerid: 0x%x\n", engineerid);
	return engineerid;
}
__tagtable(ATAG_ENGINEERID, parse_tag_engineerid);

unsigned char pcbid = PROJECT_PHASE_LATEST;
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
	pr_info("parse_tag_pcbid: 0x%x\n", pcbid);
	return pcbid;
}
__tagtable(ATAG_PCBID, parse_tag_pcbid);

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
	if (emmc_tag)
		if (!strcmp(emmc_tag, "true"))
			return 1;
	return 0;
}

/* defined in nand_partitions.c */
extern int emmc_partition_read_proc(char *page, char **start, off_t off,
		int count, int *eof, void *data);

static int __init emmc_create_proc_entry(void)
{
	struct proc_dir_entry* proc;

	proc = create_proc_read_entry("emmc", 0, NULL, emmc_partition_read_proc, NULL);
	if (proc) {
		printk(KERN_INFO "[emmc] mount /proc/emmc successfully\n");
	} else {
		printk(KERN_INFO "[emmc] mount /proc/emmc failed\n");
	}

	return 0;
}
late_initcall(emmc_create_proc_entry);

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

static unsigned long radio_flag = 0;
int __init radio_flag_init(char *s)
{
	if (strict_strtoul(s, 16, &radio_flag))
		return -EINVAL;
	else
		return 1;
}
__setup("radioflag=", radio_flag_init);

unsigned int get_radio_flag(void)
{
	return radio_flag;
}

static unsigned long kernel_flag = 0;
int __init kernel_flag_init(char *s)
{
	if (strict_strtoul(s, 16, &kernel_flag))
		return -EINVAL;
	else
		return 1;
}
__setup("kernelflag=", kernel_flag_init);

unsigned int get_kernel_flag(void)
{
	return kernel_flag;
}

static unsigned long extra_kernel_flag = 0;
int __init extra_kernel_flag_init(char *s)
{
	if (strict_strtoul(s, 16, &extra_kernel_flag))
		return -EINVAL;
	else
		return 1;
}
__setup("kernelflagex=", extra_kernel_flag_init);

unsigned int get_extra_kernel_flag(void)
{
	return extra_kernel_flag;
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

const int htc_get_pcbid_info(void)
{
	return pcbid;
}
EXPORT_SYMBOL(htc_get_pcbid_info);

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

static long unsigned usb_ats;
int __init board_ats_init(char *s)
{
	if (strict_strtoul(s, 16, &usb_ats))
		return -EINVAL;
	else
		return 1;
}
__setup("ats=", board_ats_init);

int board_get_usb_ats(void)
{
	return usb_ats;
}

/*
	Those function should not be called outsite
	to ensure that others do no change restart reason.
	Use mode & cmd to set reason & msg in arch_reset().

	TODO: decouple get_htc_reboot_params with nvdumper
*/
#define get_htc_reboot_params() ((struct htc_reboot_params*) get_reboot_params())
#define get_last_htc_reboot_params() ((struct htc_reboot_params*) get_last_reboot_params())

static void set_restart_reason(unsigned int reason)
{
	get_htc_reboot_params()->reboot_reason = reason;
}

static void set_restart_msg(const char *msg)
{
	char* buf;
	size_t msg_len, buf_len;
	if (unlikely(!msg))
	{
		WARN(1, "%s: argument msg is NULL\n", __func__);
		msg = "";
	}

	buf = get_htc_reboot_params()->msg;
	msg_len = strlen(msg);
	buf_len = sizeof(get_htc_reboot_params()->msg);

	memset(buf, 0, buf_len);
	strncpy(buf, msg, min(msg_len, buf_len - 1));
}

static struct cmd_reason_map
{
	char* cmd;
	unsigned long reason;
} _crmap[] = {
	{ .cmd = "",           .reason = RESTART_REASON_REBOOT },
	{ .cmd = "bootloader", .reason = RESTART_REASON_BOOTLOADER },
	{ .cmd = "recovery",   .reason = RESTART_REASON_RECOVERY },
	{ .cmd = "eraseflash", .reason = RESTART_REASON_ERASE_FLASH },
	{ .cmd = "offmode",    .reason = RESTART_REASON_OFFMODE },
	{ .cmd = "poweroff",   .reason = RESTART_REASON_POWEROFF },
	{ .cmd = "force-hard", .reason = RESTART_REASON_RAMDUMP },
};
#define OEM_CMD_PREFIX ("oem-")

static void set_restart_command(const char* command)
{
	int i;

	if (unlikely(!command))
	{
		WARN(1, "%s: argument command is NULL\n", __func__);
		command = "";
	}

	/* standard reboot command */
	for (i = 0; i < ARRAY_SIZE(_crmap); i++)
		if (!strncmp(command, _crmap[i].cmd, strlen(command)))
		{
			set_restart_msg(_crmap[i].cmd);
			set_restart_reason(_crmap[i].reason);
			return;
		}

	/* oem reboot command */
	if (!strncmp(command, OEM_CMD_PREFIX, strlen(OEM_CMD_PREFIX)))
	{
		unsigned long code;
		code = simple_strtoul(command + strlen(OEM_CMD_PREFIX), NULL, 16) & 0xff;

		/* oem-97, 98, 99 are RIL fatal */
		if ((code == 0x97) || (code == 0x98))
			code = 0x99;

		set_restart_msg(command);
		set_restart_reason(RESTART_REASON_OEM_BASE | code);
		return;
	}

	WARN(1, "Unknown restart command: %s\n", command);
}

static int reboot_callback(struct notifier_block *nb,
		unsigned long event, void *data)
{
	/*
	 * NOTE: data is NULL when reboot w/o command or shutdown
	 */
	struct task_struct* t;
	char* cmd;

	cmd = (char*) (data ? data : "");
	pr_info("kernel_restart(cmd=%s) - triggered with task: %s (%d:%d)\n",
			data ? data : "<null>",
			current->comm, current->tgid, current->pid);
	pr_info("parents of %s:\n", current->comm);
	t = current->parent;
	do {
		pr_info("    %s (%d:%d)\n", t->comm, t->tgid, t->pid);
		t = t->parent;
	} while (t->parent != t);
	dump_stack();

	switch (event)
	{
	case SYS_RESTART:
		set_restart_command(cmd);
		break;
	case SYS_HALT:
	case SYS_POWER_OFF:
	default:
		/*
		 * clear reboot_params to prevent unnessary RAM issue
		 * set it to 'offmode' instead of 'poweroff' since
		 * it is required to make device enter offmode charging
		 * if cable attached
		 */
		set_restart_command("offmode");
		break;
	}
	return NOTIFY_DONE;
}

static struct notifier_block reboot_notifier = {
	.notifier_call = reboot_callback,
};

static int __init htc_reset_reason_init(void)
{
	WARN(sizeof(struct htc_reboot_params) > (4096),
		    "size of htc_reboot_params exceeds limitation.");
	register_reboot_notifier(&reboot_notifier);
	return 0;
}
arch_initcall(htc_reset_reason_init);

static int __cpuinit debug_cpu_toggle_notify(struct notifier_block *self,
		unsigned long action, void *hcpu)
{
	MF_DEBUG("00UP0007");
	switch (action) {
	case CPU_ONLINE:
	case CPU_DEAD:
		pr_info("[CPUHP] current online: %d%d%d%d\n",
				cpu_online(0), cpu_online(1), cpu_online(2), cpu_online(3));
	}
	return NOTIFY_OK;
}
static int __init htc_debug_cpu_toggle_init(void)
{
	hotcpu_notifier(debug_cpu_toggle_notify, 0);
	return 0;
}
late_initcall(htc_debug_cpu_toggle_init);

unsigned get_last_reboot_params_battery_level(void)
{
	return get_last_htc_reboot_params()->battery_level;
}
void set_reboot_params_battery_level(unsigned level)
{
	pr_debug("battery_level: from 0x%08x to 0x%08x\n",
			get_htc_reboot_params()->battery_level, level);
	get_htc_reboot_params()->battery_level = level;
}
