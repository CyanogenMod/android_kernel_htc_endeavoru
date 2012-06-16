/*
 * arch/arm/mach-tegra/baseband-xmm-power2.c
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
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/wakelock.h>
#include <linux/pm_qos_params.h>
#include <mach/usb_phy.h>
#include "baseband-xmm-power.h"
#include "board.h"
#include "devices.h"

/*
 *	HTC: version history
 *
 *		v01 - bert_lin - 20111027
 *			1. item 14 easy to panic after flight mode on/off
 *				plugin the usb cable and try to use adb shell
 *			root cause: wake lock doesn't released after module exit
 *				wake_lock_init name=htc_modem_6260 bf018f60
 *				wakelock_stats_show 137 bf018f60
 *				wakelock_stats_show 138 (null) (null)
 *			2. re-arrange the kset_create_and_add flow
 *				original one doesn't take care the error handling
 *				kernel panic when we see [FLS] can not allocate modem_kset0
 *		       20111110-
 * 			1. modify sim detec kset object name to modem_kset_sim from modem_kset_sim
 *			2. Add radio core dump
 *
 */
#define MODULE_NAME "[XMM2_v1]"

MODULE_LICENSE("GPL");

#if defined(CONFIG_MACH_EDGE) || defined(CONFIG_MACH_ENDEAVORU) || defined(CONFIG_MACH_EDGE_TD) || defined(CONFIG_MACH_BLUE)
#if 0//sim move to other driver
    #define SIM_DETECT_LOW_ACTIVE TEGRA_GPIO_PI5
#endif//sim move to other driver
    #define SIM_INIT_LOW_ACTIVE TEGRA_GPIO_PE0
#if 0//sim move to other driver
    #define SIM_DETECT SIM_DETECT_LOW_ACTIVE
#endif//sim move to other driver
    #define SIM_INIT SIM_INIT_LOW_ACTIVE

#elif defined(CONFIG_MACH_QUATTRO_U)
#if 0//sim move to other driver
    #define SIM_DETECT_HIGH_ACTIVE TEGRA_GPIO_PN2
#endif//sim move to other driver
    #define SIM_INIT_HIGH_ACTIVE TEGRA_GPIO_PK2
#if 0//sim move to other driver
    #define SIM_DETECT SIM_DETECT_HIGH_ACTIVE
#endif//sim move to other driver

    #define SIM_INIT SIM_INIT_HIGH_ACTIVE

#endif


#if defined(CONFIG_MACH_EDGE) || defined(CONFIG_MACH_ENDEAVORU) || defined(CONFIG_MACH_EDGE_TD) || defined(CONFIG_MACH_BLUE)   
	#define CORE_DUMP_DETECT TEGRA_GPIO_PN2
	//radio fatal

static enum {
	RADIO_STATUS_UNKNOWN,
	RADIO_STATUS_READY,
	RADIO_STATUS_FATAL,
	RADIO_STATUS_MAX,
} radio_detect_status;

static struct work_struct radio_detect_work_struct;

#endif


#ifdef BB_XMM_OEM1

#define DEBUG_LOG_LENGTH 1024
#if 0//sim move to other driver
static struct kset *modem_kset_sim;
#endif//sim move to other driver
static struct kset *modem_kset_radio;
static struct kobject* kobj_hsic_device;

static void htc_modem_kobject_release(struct kobject *kobj)
{
    pr_err("htc_modem_kobject_release.\n");
    return;
}

static struct kobj_type htc_modem_ktype = {
    .release = htc_modem_kobject_release,
};

#endif /* BB_XMM_OEM1 */

static unsigned long XYZ = 1000 * 1000000 + 800 * 1000 + 500;

module_param(modem_ver, ulong, 0644);
MODULE_PARM_DESC(modem_ver,
	"baseband xmm power2 - modem software version");
module_param(modem_flash, ulong, 0644);
MODULE_PARM_DESC(modem_flash,
	"baseband xmm power2 - modem flash (1 = flash, 0 = flashless)");
module_param(modem_pm, ulong, 0644);
MODULE_PARM_DESC(modem_pm,
	"baseband xmm power2 - modem power management (1 = pm, 0 = no pm)");
module_param(XYZ, ulong, 0644);
MODULE_PARM_DESC(XYZ,
	"baseband xmm power2 - timing parameters X/Y/Z delay in ms");

static struct baseband_power_platform_data *baseband_power2_driver_data;
static struct workqueue_struct *workqueue;
static struct baseband_xmm_power_work_t *baseband_xmm_power2_work;
extern struct pm_qos_request_list modem_boost_cpu_freq_req;

static enum {
	IPC_AP_WAKE_UNINIT,
	IPC_AP_WAKE_IRQ_READY,
	IPC_AP_WAKE_INIT1,
	IPC_AP_WAKE_INIT2,
	IPC_AP_WAKE_L,
	IPC_AP_WAKE_H,
} ipc_ap_wake_state;

#ifdef BB_XMM_OEM1
#if 0//sim move to other driver
static enum {
	SIM_STATUS_UNKNOWN,
	SIM_STATUS_READY,
	SIM_STATUS_INSERTED,
	SIM_STATUS_UNPLUGGED,
	SIM_STATUS_MAX,
} sim_detect_status;

static struct work_struct sim_detect_work_struct;
#endif//sim move to other driver

struct htc_modem_info {

    int device_id;
    bool is_flashless;

    /* lock to protect modem info */
    struct mutex info_lock;

    int is_open;
#if 0//sim move to other driver
    struct kobject modem_sim_det_kobj;
#endif//sim move to other driver

    struct kobject modem_core_dump_kobj;

    struct wake_lock modem_wake_lock;
    char debug_log[DEBUG_LOG_LENGTH];

#if 0
    struct battery_info_reply rep;

    struct battery_adc_reply adc_data;
    int adc_vref[ADC_REPLY_ARRAY_SIZE];

    int guage_driver;
    int charger;
    int read_current;
#endif
};

static struct htc_modem_info modem_info;

#endif /* BB_XMM_OEM1 */

static irqreturn_t baseband_xmm_power2_ver_lt_1130_ipc_ap_wake_irq2
	(int irq, void *dev_id)
{
	int value;

	pr_debug("%s\n", __func__);

	/* check for platform data */
	if (!baseband_power2_driver_data)
		return IRQ_HANDLED;

	value = gpio_get_value(baseband_power2_driver_data->
		modem.xmm.ipc_ap_wake);

	/* IPC_AP_WAKE state machine */
	if (ipc_ap_wake_state < IPC_AP_WAKE_IRQ_READY) {
		pr_err("%s - spurious irq\n", __func__);
	} else if (ipc_ap_wake_state == IPC_AP_WAKE_IRQ_READY) {
		if (!value) {
			pr_debug("%s - IPC_AP_WAKE_INIT1"
				" - got falling edge\n",
				__func__);
			/* go to IPC_AP_WAKE_INIT1 state */
			ipc_ap_wake_state = IPC_AP_WAKE_INIT1;
			/* queue work */
			baseband_xmm_power2_work->state =
				BBXMM_WORK_INIT_FLASHLESS_PM_VER_LT_1130_STEP1;
			queue_work(workqueue, (struct work_struct *)
				baseband_xmm_power2_work);
		} else {
			pr_debug("%s - IPC_AP_WAKE_INIT1"
				" - wait for falling edge\n",
				__func__);
		}
	} else if (ipc_ap_wake_state == IPC_AP_WAKE_INIT1) {
		if (!value) {
			pr_debug("%s - IPC_AP_WAKE_INIT2"
				" - wait for rising edge\n",
				__func__);
		} else {
			pr_debug("%s - IPC_AP_WAKE_INIT2"
				" - got rising edge\n",
				__func__);
			/* go to IPC_AP_WAKE_INIT2 state */
			ipc_ap_wake_state = IPC_AP_WAKE_INIT2;
			/* queue work */
			baseband_xmm_power2_work->state =
				BBXMM_WORK_INIT_FLASHLESS_PM_VER_LT_1130_STEP2;
			queue_work(workqueue, (struct work_struct *)
				baseband_xmm_power2_work);
		}
	} else {
		value = gpio_get_value(baseband_power2_driver_data->
			modem.xmm.ipc_ap_wake);
		if (!value) {
			pr_debug("%s - falling\n", __func__);
			ipc_ap_wake_state = IPC_AP_WAKE_L;
		} else {
			pr_debug("%s - rising\n", __func__);
			ipc_ap_wake_state = IPC_AP_WAKE_H;
		}
		return baseband_xmm_power_ipc_ap_wake_irq(irq, dev_id);
	}

	return IRQ_HANDLED;
}

static irqreturn_t baseband_xmm_power2_ver_ge_1130_ipc_ap_wake_irq2
	(int irq, void *dev_id)
{
	int value;

	/* pr_debug("%s\n", __func__); */

	/* check for platform data */
	if (!baseband_power2_driver_data) {
		pr_err(MODULE_NAME "%s - !baseband_power2_driver_data\n", __func__);
		return IRQ_HANDLED;
	}

	value = gpio_get_value(baseband_power2_driver_data->
		modem.xmm.ipc_ap_wake);

	/* IPC_AP_WAKE state machine */
	if (ipc_ap_wake_state < IPC_AP_WAKE_IRQ_READY) {
		pr_err("%s - spurious irq\n", __func__);
	} else if (ipc_ap_wake_state == IPC_AP_WAKE_IRQ_READY) {
		if (!value) {
			pr_debug("%s - IPC_AP_WAKE_INIT1"
				" - got falling edge\n",
				__func__);
			/* go to IPC_AP_WAKE_INIT2 state */
			ipc_ap_wake_state = IPC_AP_WAKE_INIT2;
			/* queue work */
			baseband_xmm_power2_work->state =
				BBXMM_WORK_INIT_FLASHLESS_PM_VER_GE_1130_STEP2;
			queue_work(workqueue, (struct work_struct *)
				baseband_xmm_power2_work);
		} else {
			pr_debug("%s - IPC_AP_WAKE_INIT1"
				" - wait for falling edge\n",
				__func__);
		}
	} else {
		if (!value) {
			pr_debug("%s - falling\n", __func__);
			ipc_ap_wake_state = IPC_AP_WAKE_L;
		} else {
			pr_debug("%s - rising\n", __func__);
			ipc_ap_wake_state = IPC_AP_WAKE_H;
		}
		return baseband_xmm_power_ipc_ap_wake_irq(irq, dev_id);
	}

	return IRQ_HANDLED;
}

#ifdef BB_XMM_OEM1
#if 0//sim move to other driver
static void sim_detect_work_handler(struct work_struct *work)
{
    if (!baseband_power2_driver_data)
    {
	pr_err("baseband_power2_driver_data is null\n");
	return ;
    }

	char message[20] = "SIMHOTSWAP=";
		char *envp[] = { message, NULL };
    int status = gpio_get_value(SIM_DETECT);

    pr_info("SIM_DETECT = %d\n", status);

		if (status){
			strncat(message, "REMOVE", 6);
			pr_info("SIM CARD REMOVE\n");
			}
		else{
			strncat(message, "INSERT", 6);			
			pr_info("SIM CARD INSERT\n");
			}


    pr_info("[FLS] issue SIMHOTSWAP uevent\n");
    //kobject_uevent(&baseband_power2_driver_data->modem.xmm6260.hsic_device->dev.kobj, KOBJ_ADD);
    //kobject_uevent(  &modem_info.modem_sim_det_kobj, KOBJ_ADD);
    kobject_uevent_env(&modem_info.modem_sim_det_kobj, KOBJ_ADD,envp);

}
#endif//sim move to other driver

/*SIM detection IRQ*/
#if defined(CONFIG_MACH_EDGE) || defined(CONFIG_MACH_ENDEAVORU) || defined(CONFIG_MACH_EDGE_TD) || defined(CONFIG_MACH_BLUE)
#if 0//sim move to other driver
static irqreturn_t sim_det_irq(int irq, void *dev_id)
{

    int value;

    if (!baseband_power2_driver_data)
    {
	pr_info("no baseband_power2_driver_data\n");
	return IRQ_HANDLED;
    }

    if (sim_detect_status < SIM_STATUS_READY) {
	pr_err("%s - spurious irq\n", __func__);
    } else {
	value = gpio_get_value(SIM_DETECT/* 69*/);
	if (!value) {
	    pr_info("%s - falling\n", __func__);
	    sim_detect_status = SIM_STATUS_INSERTED;
	} else {
	    pr_info("%s - rising\n", __func__);
	    sim_detect_status = SIM_STATUS_UNPLUGGED;
	}
    }

    queue_work(workqueue, &sim_detect_work_struct);

    return IRQ_HANDLED;
}
#endif//sim move to other driver
static void radio_detect_work_handler(struct work_struct *work)
{
		int radiopower=0;

		pr_info("Enter radio_detect_work_handler\n");

		/* Sleep 30 ms and then check if radio is turn off */
		msleep(30);
		radiopower =gpio_get_value(TEGRA_GPIO_PM4);

		if (!radiopower) {
			pr_info("radio is off, it's not coredump interrupt\n");
			return;
		}

    if (!baseband_power2_driver_data)
    {
			pr_info("baseband_power2_driver_data is null\n");
			return ;
    }

	char message[20] = "RADIO=";
	char *envp[] = { message, NULL };
	int status = gpio_get_value(CORE_DUMP_DETECT);

	pr_info("CORE_DUMP_DETECT = %d\n", status);
		if (status) {
			pr_info("CORE_DUMP_DETECT=High, Normal\n");
			return ;
		}
		else {
			strncat(message, "FATAL", 5);
			pr_info("CORE_DUMP_DETECT=Low, radio fatal!!\n");
		}
		/*

		if (status)
			strncat(message, "FATAL", 5);
		else
			strncat(message, "READY", 5);
			*/

    pr_info("[FLS] coredump uevent\n");
    //kobject_uevent(&baseband_power2_driver_data->modem.xmm6260.hsic_device->dev.kobj, KOBJ_ADD);
    //kobject_uevent(  &modem_info.modem_core_dump_kobj, KOBJ_ADD);
    kobject_uevent_env(&modem_info.modem_core_dump_kobj, KOBJ_ADD,envp);

}

/*radio detection IRQ*/

static irqreturn_t radio_det_irq(int irq, void *dev_id)
{
   int value = 0;

		if (!baseband_power2_driver_data)
    {
			pr_info("no baseband_power2_driver_data\n");
			return IRQ_HANDLED;
		}

    if (radio_detect_status < RADIO_STATUS_READY) {
	pr_err("%s - spurious irq\n", __func__);
    } else {
	value = gpio_get_value(CORE_DUMP_DETECT/* 69*/);
	if (!value) {
	    pr_info("%s - falling\n", __func__);
	    radio_detect_status = RADIO_STATUS_FATAL;
	} else {
	    pr_info("%s - rising\n", __func__);
	    radio_detect_status = RADIO_STATUS_READY;
	}
    }
    if (!value) {
     queue_work(workqueue, &radio_detect_work_struct);
   } else {
    pr_info("rising is ignored");
	}
    return IRQ_HANDLED;
}
#endif
#endif /* BB_XMM_OEM1 */

static void baseband_xmm_power2_flashless_pm_ver_lt_1130_step1
	(struct work_struct *work)
{
	int value;

	pr_debug("%s {\n", __func__);

	/* check for platform data */
	if (!baseband_power2_driver_data)
		return;

	/* check if IPC_HSIC_ACTIVE high */
	value = gpio_get_value(baseband_power2_driver_data->
		modem.xmm.ipc_hsic_active);
	if (value != 1) {
		pr_err("%s - expected IPC_HSIC_ACTIVE high!\n", __func__);
		return;
	}

	/* wait 30 ms */
	msleep(30);

	/* set IPC_HSIC_ACTIVE low */
	gpio_set_value(baseband_power2_driver_data->
		modem.xmm.ipc_hsic_active, 0);

	pr_debug("%s }\n", __func__);
}

static void baseband_xmm_power2_flashless_pm_ver_lt_1130_step2
	(struct work_struct *work)
{
	int value;

	pr_debug("%s {\n", __func__);

	/* check for platform data */
	if (!baseband_power2_driver_data)
		return;

	/* check if IPC_HSIC_ACTIVE low */
	value = gpio_get_value(baseband_power2_driver_data->
		modem.xmm.ipc_hsic_active);
	if (value != 0) {
		pr_err("%s - expected IPC_HSIC_ACTIVE low!\n", __func__);
		return;
	}

	/* wait 1 ms */
	msleep(1);

	/* unregister usb host controller */
	if (baseband_power2_driver_data->hsic_unregister)
		baseband_power2_driver_data->hsic_unregister(
			baseband_power2_driver_data->modem.xmm.hsic_device);
	else
		pr_err("%s: hsic_unregister is missing\n", __func__);

	/* set IPC_HSIC_ACTIVE high */
	gpio_set_value(baseband_power2_driver_data->
		modem.xmm.ipc_hsic_active, 1);

	/* wait 20 ms */
	msleep(20);

	/* set IPC_HSIC_ACTIVE low */
	gpio_set_value(baseband_power2_driver_data->
		modem.xmm.ipc_hsic_active, 0);

	/* wait 20 ms */
	msleep(20);

	/* set IPC_HSIC_ACTIVE high */
	gpio_set_value(baseband_power2_driver_data->
		modem.xmm.ipc_hsic_active, 1);

	pr_debug("%s }\n", __func__);
}

static void baseband_xmm_power2_flashless_pm_ver_ge_1130_step1
	(struct work_struct *work)
{
	int X = XYZ / 1000000;
	int Y = XYZ / 1000 - X * 1000;
	int Z = XYZ % 1000;

	pr_info("%s {\n", __func__);

	pr_info("XYZ=%ld X=%d Y=%d Z=%d\n", XYZ, X, Y, Z);

	/* check for platform data */
	if (!baseband_power2_driver_data)
		return;

	/* unregister usb host controller */
	if (baseband_power2_driver_data->hsic_unregister)
		baseband_power2_driver_data->hsic_unregister(
			baseband_power2_driver_data->modem.xmm.hsic_device);
	else
		pr_err("%s: hsic_unregister is missing\n", __func__);

	/* wait X ms */
	msleep(X);

	/* set IPC_HSIC_ACTIVE low */
	gpio_set_value(baseband_power2_driver_data->
		modem.xmm.ipc_hsic_active, 0);

	pr_info("%s }\n", __func__);
}

static void baseband_xmm_power2_flashless_pm_ver_ge_1130_step2
	(struct work_struct *work)
{
	int X = XYZ / 1000000;
	int Y = XYZ / 1000 - X * 1000;
	int Z = XYZ % 1000;

	pr_info("%s {\n", __func__);

	pr_info("XYZ=%ld X=%d Y=%d Z=%d\n", XYZ, X, Y, Z);

	/* check for platform data */
	if (!baseband_power2_driver_data)
		return;

	/* wait Y ms */
	msleep(Y);

	/* register usb host controller */
	if (baseband_power2_driver_data->hsic_register)
		baseband_power2_driver_data->modem.xmm.hsic_device =
			baseband_power2_driver_data->hsic_register();
	else
		pr_err("%s: hsic_register is missing\n", __func__);

	/* wait Z ms */
	msleep(Z);

	/* set IPC_HSIC_ACTIVE high */
	gpio_set_value(baseband_power2_driver_data->
		modem.xmm.ipc_hsic_active, 1);

	/* printk(KERN_INFO"Mark recovery STEP3 %s \n", __func__); */

	/* queue work function to check if enumeration succeeded */
	/* recovery behavior */
	baseband_xmm_power2_work->state =
		BBXMM_WORK_INIT_FLASHLESS_PM_VER_GE_1130_STEP3;
	queue_work(workqueue, (struct work_struct *)
		baseband_xmm_power2_work);
	pr_info("%s }\n", __func__);
}

#define FILE_EXIST 0
#define FILE_NOT_EXIST -1
static int file_open_check(const char *file)
{
	mm_segment_t oldfs;
	struct file *filp;
	int ret = FILE_EXIST;

	oldfs = get_fs();
	set_fs(KERNEL_DS);

	filp = filp_open(file,
		O_RDONLY, 0);
	if (IS_ERR(filp) || (filp == NULL)) {
		pr_info("open %s %ld\n", file, PTR_ERR(filp));
		ret = FILE_NOT_EXIST;
		goto open_fail;
	}

	/* open success */
	filp_close(filp, NULL);

open_fail:
	set_fs(oldfs);
	return ret;
}

static void baseband_xmm_power2_flashless_pm_ver_ge_1130_step3
	(struct work_struct *work)
{
	int X = XYZ / 1000000;
	int Y = XYZ / 1000 - X * 1000;
	int Z = XYZ % 1000;
	int enum_success = 0;

	pr_info("%s {\n", __func__);

	pr_info("XYZ=%ld X=%d Y=%d Z=%d\n", XYZ, X, Y, Z);

	/* check for platform data */
	if (!baseband_power2_driver_data)
		return;

	/* wait 1000 ms */
	msleep(1000);

	/* check if enumeration succeeded */
	{
		mm_segment_t oldfs;
		struct file *filp;
		oldfs = get_fs();
		set_fs(KERNEL_DS);
#ifdef BB_XMM_OEM1
        filp = filp_open("/dev/ttyACM0",
            O_RDONLY, 0);
#else
        filp = filp_open("/sys/bus/usb/devices/usb2/2-1/manufacturer",
            O_RDONLY, 0);
#endif
        if (IS_ERR(filp) || (filp == NULL)) {
#ifdef BB_XMM_OEM1
            pr_err("open /dev/ttyACM0 failed %ld\n",
                PTR_ERR(filp));
#else
            pr_err("open /sys/bus/usb/devices"
                "/usb2/2-1/manufacturer failed %ld\n",
                PTR_ERR(filp));
#endif
		} else {
			filp_close(filp, NULL);
			enum_success = 1;
		}
		set_fs(oldfs);
	}

	/* if enumeration failed, attempt recovery pulse */
	if (!enum_success) {
		pr_info("attempting recovery pulse...\n");
		/* wait 20 ms */
		msleep(20);
		/* set IPC_HSIC_ACTIVE low */
		gpio_set_value(baseband_power2_driver_data->
			modem.xmm.ipc_hsic_active, 0);
		/* wait 20 ms */
		msleep(20);
		/* set IPC_HSIC_ACTIVE high */
		gpio_set_value(baseband_power2_driver_data->
			modem.xmm.ipc_hsic_active, 1);
		/* check if recovery pulse worked */
		baseband_xmm_power2_work->state =
			BBXMM_WORK_INIT_FLASHLESS_PM_VER_GE_1130_STEP4;
		queue_work(workqueue, (struct work_struct *)
			baseband_xmm_power2_work);
	} else
		pr_info("%s - enum success\n", __func__);

	pr_info("VP:%s - pm qos CPU back to normal\n", __func__);
	pm_qos_update_request(&modem_boost_cpu_freq_req,
		(s32)PM_QOS_CPU_FREQ_MIN_DEFAULT_VALUE);

	pr_info("%s }\n", __func__);
}

static void baseband_xmm_power2_flashless_pm_ver_ge_1130_step4
	(struct work_struct *work)
{
	int X = XYZ / 1000000;
	int Y = XYZ / 1000 - X * 1000;
	int Z = XYZ % 1000;
	int enum_success = 0;

	pr_info("%s {\n", __func__);

	pr_info("XYZ=%ld X=%d Y=%d Z=%d\n", XYZ, X, Y, Z);

	/* check for platform data */
	if (!baseband_power2_driver_data)
		return;

	/* wait 1000 ms */
	msleep(1000);

	/* check if enumeration succeeded */
	{
		mm_segment_t oldfs;
		struct file *filp;
		oldfs = get_fs();
		set_fs(KERNEL_DS);
#ifdef BB_XMM_OEM1
		filp = filp_open("/dev/ttyACM0",
			O_RDONLY, 0);
#else
		filp = filp_open("/sys/bus/usb/devices/usb2/2-1/manufacturer",
			O_RDONLY, 0);
#endif
		if (IS_ERR(filp) || (filp == NULL)) {
#ifdef BB_XMM_OEM1
			pr_err("open /dev/ttyACM0 failed %ld\n",
				PTR_ERR(filp));
#else
			pr_err("open /sys/bus/usb/devices"
				"/usb2/2-1/manufacturer failed %ld\n",
				PTR_ERR(filp));
#endif
		} else {
			filp_close(filp, NULL);
			enum_success = 1;
		}
		set_fs(oldfs);
	}

	/* if recovery pulse did not fix enumeration, retry from beginning */
	if (!enum_success) {
		static int retry = 3;
		if (!retry) {
			pr_info("failed to enumerate modem software"
				" - too many retry attempts\n");
		} else {
			pr_info("recovery pulse failed to fix modem"
				" enumeration..."
				" restarting from beginning"
				" - attempt #%d\n",
				retry);
			--retry;
			ipc_ap_wake_state = IPC_AP_WAKE_IRQ_READY;
			baseband_xmm_power2_work->state =
				BBXMM_WORK_INIT_FLASHLESS_PM_VER_GE_1130_STEP1;
			queue_work(workqueue, (struct work_struct *)
				baseband_xmm_power2_work);
		}
	} else
		pr_info("%s - enum success\n", __func__);

	pr_info("%s }\n", __func__);
}

static int free_ipc_ap_wake_irq;

static void baseband_xmm_power2_work_func(struct work_struct *work)
{
	struct baseband_xmm_power_work_t *bbxmm_work
		= (struct baseband_xmm_power_work_t *) work;
	int err;

	pr_debug("%s bbxmm_work->state=%d\n", __func__, bbxmm_work->state);

	switch (bbxmm_work->state) {
	case BBXMM_WORK_UNINIT:
		pr_debug("BBXMM_WORK_UNINIT\n");
		/* free baseband irq(s) */
		if (free_ipc_ap_wake_irq) {
			free_irq(gpio_to_irq(baseband_power2_driver_data
				->modem.xmm.ipc_ap_wake), NULL);
			free_ipc_ap_wake_irq = 0;
		}
		break;
	case BBXMM_WORK_INIT:
		pr_debug("BBXMM_WORK_INIT\n");
		/* request baseband irq(s) */
		ipc_ap_wake_state = IPC_AP_WAKE_UNINIT;
			err = request_threaded_irq(
			gpio_to_irq(baseband_power2_driver_data->modem.xmm.ipc_ap_wake),
			NULL,
			(modem_ver < XMM_MODEM_VER_1130)
			? baseband_xmm_power2_ver_lt_1130_ipc_ap_wake_irq2
			: baseband_xmm_power2_ver_ge_1130_ipc_ap_wake_irq2,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			"BBXMM_POWER2_IPC_AP_WAKE_IRQ",
			NULL);
		if (err < 0) {
			pr_err("%s - request irq IPC_AP_WAKE_IRQ failed\n",
				__func__);
			return;
		}
		free_ipc_ap_wake_irq = 1;
		ipc_ap_wake_state = IPC_AP_WAKE_IRQ_READY;
		/* go to next state */
		bbxmm_work->state = (modem_flash && !modem_pm)
			? BBXMM_WORK_INIT_FLASH_STEP1
			: (modem_flash && modem_pm)
			? BBXMM_WORK_INIT_FLASH_PM_STEP1
			: (!modem_flash && modem_pm)
			? BBXMM_WORK_INIT_FLASHLESS_PM_STEP1
			: BBXMM_WORK_UNINIT;
		queue_work(workqueue, work);
		break;
	case BBXMM_WORK_INIT_FLASH_STEP1:
		pr_debug("BBXMM_WORK_INIT_FLASH_STEP1\n");
		break;
	case BBXMM_WORK_INIT_FLASH_PM_STEP1:
		pr_debug("BBXMM_WORK_INIT_FLASH_PM_STEP1\n");
		/* go to next state */
		bbxmm_work->state = (modem_ver < XMM_MODEM_VER_1130)
			? BBXMM_WORK_INIT_FLASH_PM_VER_LT_1130_STEP1
			: BBXMM_WORK_INIT_FLASH_PM_VER_GE_1130_STEP1;
		queue_work(workqueue, work);
		break;
	case BBXMM_WORK_INIT_FLASH_PM_VER_LT_1130_STEP1:
		pr_debug("BBXMM_WORK_INIT_FLASH_PM_VER_LT_1130_STEP1\n");
		break;
	case BBXMM_WORK_INIT_FLASH_PM_VER_GE_1130_STEP1:
		pr_debug("BBXMM_WORK_INIT_FLASH_PM_VER_GE_1130_STEP1\n");
		break;
	case BBXMM_WORK_INIT_FLASHLESS_PM_STEP1:
		pr_debug("BBXMM_WORK_INIT_FLASHLESS_PM_STEP1\n");
		/* go to next state */
		bbxmm_work->state = (modem_ver < XMM_MODEM_VER_1130)
			? BBXMM_WORK_INIT_FLASHLESS_PM_VER_LT_1130_WAIT_IRQ
			: BBXMM_WORK_INIT_FLASHLESS_PM_VER_GE_1130_STEP1;
		queue_work(workqueue, work);
		break;
	case BBXMM_WORK_INIT_FLASHLESS_PM_VER_LT_1130_WAIT_IRQ:
		pr_debug("BBXMM_WORK_INIT_FLASHLESS_PM_VER_LT_1130_WAIT_IRQ"
			" - waiting for IPC_AP_WAKE_IRQ to trigger step1\n");
		break;
	case BBXMM_WORK_INIT_FLASHLESS_PM_VER_LT_1130_STEP1:
		pr_debug("BBXMM_WORK_INIT_FLASHLESS_PM_VER_LT_1130_STEP1\n");
		baseband_xmm_power2_flashless_pm_ver_lt_1130_step1(work);
		break;
	case BBXMM_WORK_INIT_FLASHLESS_PM_VER_LT_1130_STEP2:
		pr_debug("BBXMM_WORK_INIT_FLASHLESS_PM_VER_LT_1130_STEP2\n");
		baseband_xmm_power2_flashless_pm_ver_lt_1130_step2(work);
		break;
	case BBXMM_WORK_INIT_FLASHLESS_PM_VER_GE_1130_STEP1:
		pr_debug("BBXMM_WORK_INIT_FLASHLESS_PM_VER_GE_1130_STEP1\n");
		baseband_xmm_power2_flashless_pm_ver_ge_1130_step1(work);
		break;
	case BBXMM_WORK_INIT_FLASHLESS_PM_VER_GE_1130_STEP2:
		pr_debug("BBXMM_WORK_INIT_FLASHLESS_PM_VER_GE_1130_STEP2\n");
		baseband_xmm_power2_flashless_pm_ver_ge_1130_step2(work);
		break;
	case BBXMM_WORK_INIT_FLASHLESS_PM_VER_GE_1130_STEP3:
		pr_debug("BBXMM_WORK_INIT_FLASHLESS_PM_VER_GE_1130_STEP3\n");
		baseband_xmm_power2_flashless_pm_ver_ge_1130_step3(work);
		break;
	case BBXMM_WORK_INIT_FLASHLESS_PM_VER_GE_1130_STEP4:
		pr_debug("BBXMM_WORK_INIT_FLASHLESS_PM_VER_GE_1130_STEP4\n");
		baseband_xmm_power2_flashless_pm_ver_ge_1130_step4(work);
		break;
	}

}

static int baseband_xmm_power2_driver_probe(struct platform_device *device)
{
	struct baseband_power_platform_data *data
		= (struct baseband_power_platform_data *)
			device->dev.platform_data;

	int err=0;
	pr_debug("%s 0309 - CPU Freq with data protect.\n", __func__);

	if (data == NULL) {
		pr_err("%s: no platform data\n", __func__);
		return -EINVAL;
	}
	if (data->modem.xmm.hsic_device == NULL) {
		pr_err("%s: no hsic device\n", __func__);
		return -EINVAL;
	}

	/* save platform data */
	baseband_power2_driver_data = data;

	/* OEM specific initialization */
#ifdef BB_XMM_OEM1
#if defined(CONFIG_MACH_EDGE) || defined(CONFIG_MACH_ENDEAVORU) || defined(CONFIG_MACH_EDGE_TD) || defined(CONFIG_MACH_BLUE)
#if 0//sim move to other driver
		/* may be better to put this in init2()??*/
		sim_detect_status = SIM_STATUS_UNKNOWN;
		int err;
		err = request_irq(gpio_to_irq(SIM_DETECT),
			sim_det_irq,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			"SIM_DETECT",
		&modem_info);

		if (err < 0) {
			pr_err("%s - request irq SIM_DETECT failed\n",
				__func__);
			//return err;
		}

		sim_detect_status = SIM_STATUS_READY;
		pr_err("enable sim detection irq here\n");
		err = gpio_get_value(SIM_DETECT/* 69*/);
		pr_err("gpio 69 value is %d\n", err);
#endif//sim move to other driver

#if 0 //only open for XA,XB device

		int err = gpio_get_value(SIM_INIT);
		pr_info("gpio 32 value is %d\n", err);


#if defined(CONFIG_MACH_EDGE) || defined(CONFIG_MACH_ENDEAVORU) || defined(CONFIG_MACH_EDGE_TD) || defined(CONFIG_MACH_BLUE)
	if(err) /*if GPIO_PI5 is high, set as high*/
	{
	    gpio_set_value(SIM_INIT, 1);
	    pr_info("SIM_INIT value is 1\n");
	}
	else  /*if GPIO_PI5 is low, keep 50ms high, then pull low*/
	{
	    gpio_set_value(SIM_INIT, 1);
	    msleep(50);
	    gpio_set_value(SIM_INIT, 0);
	    pr_info("SIM_INIT value is 1 and then set to low\n");
	}
#endif
#endif
		kobj_hsic_device =
			 kobject_get(&baseband_power2_driver_data->modem.xmm.hsic_device->dev.kobj);
		if (!kobj_hsic_device) {
			pr_err("[FLS] can not get modem_kobject\n");
			goto fail;
		}
#if 0//sim move to other driver
		modem_kset_sim = kset_create_and_add("modem", NULL, kobj_hsic_device);
		if (!modem_kset_sim) {
			kobject_put(kobj_hsic_device);
			pr_err("[FLS] can not allocate modem_kset_sim%d\n", err);
			goto fail;
		}

		modem_info.modem_sim_det_kobj.kset = modem_kset_sim;
		err = kobject_init_and_add(&modem_info.modem_sim_det_kobj,
			&htc_modem_ktype, NULL, "htc_modem_sim_det");
		if (err) {
			pr_err("init kobject modem_kset_sim failed.");
			kobject_put(&modem_info.modem_sim_det_kobj);

			kset_unregister(modem_kset_sim);
			modem_kset_sim = NULL;

			kobject_put(kobj_hsic_device);

			goto fail;
		}
#endif//sim move to other driver

		/* radio detect*/
		radio_detect_status = RADIO_STATUS_UNKNOWN;
		int err_radio;
		err_radio = request_irq(gpio_to_irq(CORE_DUMP_DETECT),
			radio_det_irq,
			/*IRQF_TRIGGER_RISING |*/ IRQF_TRIGGER_FALLING,
			"RADIO_DETECT",
			&modem_info);

		if (err_radio < 0) {
			pr_err("%s - request irq RADIO_DETECT failed\n",
				__func__);
			//return err;
		}
		radio_detect_status = RADIO_STATUS_READY;
		err_radio = gpio_get_value(CORE_DUMP_DETECT/* 106*/);
		pr_info("gpio CORE_DUMP_DETECT value is %d\n", err_radio);
		modem_kset_radio = kset_create_and_add("modem_coreDump", NULL, kobj_hsic_device);
		if (!modem_kset_radio) {
			kobject_put(kobj_hsic_device);
			pr_err("[FLS] can not allocate modem_kset_radiomodem_kset_radio%d\n", err);
			goto fail;
		}
		pr_info("init and add core dump into kobject\n");
		modem_info.modem_core_dump_kobj.kset = modem_kset_radio;
		err = kobject_init_and_add(&modem_info.modem_core_dump_kobj,
			&htc_modem_ktype, NULL, "htc_modem_radioio_det");
		if (err) {
			pr_err("init kobject modem_kset_radio failed.");
			kobject_put(&modem_info.modem_core_dump_kobj);
			kset_unregister(modem_kset_radio);
			modem_kset_radio = NULL;
			kobject_put(kobj_hsic_device);
			goto fail;

		}
#endif
#endif /* BB_XMM_OEM1 */

	/* init work queue */
	pr_debug("%s: init work queue\n", __func__);
	workqueue = create_singlethread_workqueue
		("baseband_xmm_power2_workqueue");
	if (!workqueue) {
		pr_err("cannot create workqueue\n");
		return -1;
	}
	baseband_xmm_power2_work = (struct baseband_xmm_power_work_t *)
		kmalloc(sizeof(struct baseband_xmm_power_work_t), GFP_KERNEL);
	if (!baseband_xmm_power2_work) {
		pr_err("cannot allocate baseband_xmm_power2_work\n");
		return -1;
	}
	pr_debug("%s: BBXMM_WORK_INIT\n", __func__);
	INIT_WORK((struct work_struct *) baseband_xmm_power2_work,
		baseband_xmm_power2_work_func);
	baseband_xmm_power2_work->state = BBXMM_WORK_INIT;
	queue_work(workqueue,
		(struct work_struct *) baseband_xmm_power2_work);

	/* OEM specific - init work queue */
#ifdef BB_XMM_OEM1
#if defined(CONFIG_MACH_EDGE) || defined(CONFIG_MACH_ENDEAVORU) || defined(CONFIG_MACH_EDGE_TD) || defined(CONFIG_MACH_BLUE)
#if 0//sim move to other driver
	INIT_WORK(&sim_detect_work_struct, sim_detect_work_handler);
#endif//sim move to other driver
	INIT_WORK(&radio_detect_work_struct, radio_detect_work_handler);
fail:
#endif
#endif /* BB_XMM_OEM1 */

	return 0;
}

static int baseband_xmm_power2_driver_remove(struct platform_device *device)
{
	struct baseband_power_platform_data *data
		= (struct baseband_power_platform_data *)
			device->dev.platform_data;

	pr_debug("%s\n", __func__);

	/* check for platform data */
	if (!data)
		return 0;

	/* free irq */
	if (free_ipc_ap_wake_irq) {
		free_irq(gpio_to_irq(data->modem.xmm.ipc_ap_wake), NULL);
		free_ipc_ap_wake_irq = 0;
	}

	/* OEM specific - free sim detect irq */
#ifdef BB_XMM_OEM1
#if defined(CONFIG_MACH_EDGE) || defined(CONFIG_MACH_ENDEAVORU) || defined(CONFIG_MACH_EDGE_TD) || defined(CONFIG_MACH_BLUE)
#if 0//sim move to other driver
    free_irq(gpio_to_irq(SIM_DETECT), &modem_info);
#endif//sim move to other driver
    free_irq(gpio_to_irq(CORE_DUMP_DETECT), &modem_info);
#endif
#if 0//sim move to other driver
	/* HTC: 20111027 free kobj & kset */
	if (modem_kset_sim) {
		kobject_put(&modem_info.modem_sim_det_kobj);
		kset_unregister(modem_kset_sim);
		modem_kset_sim = NULL;
	}
#endif//sim move to other driver

	if (modem_kset_radio) {
		kobject_put(&modem_info.modem_core_dump_kobj);
		kset_unregister(modem_kset_radio);
		modem_kset_radio = NULL;
	}

if (kobj_hsic_device) {
	pr_debug("free kobj_hsic_device");
	kobject_put(kobj_hsic_device);
}

#endif /* BB_XMM_OEM1 */

	/* free work structure */
	if (workqueue) {
		cancel_work_sync(baseband_xmm_power2_work);
		destroy_workqueue(workqueue);
	}
	kfree(baseband_xmm_power2_work);
	baseband_xmm_power2_work = (struct baseband_xmm_power_work_t *) 0;

	return 0;
}

#ifdef CONFIG_PM
static int baseband_xmm_power2_driver_suspend(struct platform_device *device,
	pm_message_t state)
{
	struct baseband_power_platform_data *data
		= (struct baseband_power_platform_data *)
			device->dev.platform_data;

	pr_debug("%s - nop\n", __func__);

	/* check for platform data */
	if (!data)
		return 0;

	return 0;
}

static int baseband_xmm_power2_driver_resume(struct platform_device *device)
{
	struct baseband_power_platform_data *data
		= (struct baseband_power_platform_data *)
			device->dev.platform_data;

	pr_debug("%s - nop\n", __func__);

	/* check for platform data */
	if (!data)
		return 0;

	return 0;
}
#endif

static struct platform_driver baseband_power2_driver = {
	.probe = baseband_xmm_power2_driver_probe,
	.remove = baseband_xmm_power2_driver_remove,
#ifdef CONFIG_PM
	.suspend = baseband_xmm_power2_driver_suspend,
	.resume = baseband_xmm_power2_driver_resume,
#endif
	.driver = {
		.name = "baseband_xmm_power2",
	},
};

static int __init baseband_xmm_power2_init(void)
{
	pr_info(MODULE_NAME "%s 0219 - \n", __func__);

#ifdef BB_XMM_OEM1
    modem_info.device_id = -1;
    modem_info.is_flashless = 0;

    mutex_init(&modem_info.info_lock);
    modem_info.is_open = 0;
#if 0
    modem_info.modem_sim_det_kobj;
    modem_info.modem_core_dump_kobj;
#endif

    wake_lock_init(&modem_info.modem_wake_lock, WAKE_LOCK_SUSPEND,
	                            "htc_modem_6260");
#endif /* BB_XMM_OEM1 */

	return platform_driver_register(&baseband_power2_driver);
}

static void __exit baseband_xmm_power2_exit(void)
{
	pr_info(MODULE_NAME "%s\n", __func__);

#ifdef BB_XMM_OEM1
	pr_info("%s wake_lock_destroy", __func__);
	wake_lock_destroy(&modem_info.modem_wake_lock);
#endif /* BB_XMM_OEM1 */

	platform_driver_unregister(&baseband_power2_driver);
}

module_init(baseband_xmm_power2_init)
module_exit(baseband_xmm_power2_exit)
