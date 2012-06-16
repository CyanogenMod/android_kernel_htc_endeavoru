/*
 * arch/arm/mach-tegra/baseband-xmm-power.c
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
#include <linux/spinlock.h>
#include <linux/usb.h>
#include <linux/pm_runtime.h>
#include <linux/suspend.h>
#include <mach/usb_phy.h>
#include "board.h"
#include "devices.h"
#include <mach/board_htc.h>
#include <linux/pm_qos_params.h>

#if defined(CONFIG_MACH_QUATTRO_U)
#include "board-quattro.h"
#elif defined(CONFIG_MACH_BLUE)
#include "board-blue.h"
#elif defined(CONFIG_MACH_EDGE)
#include "board-edge.h"
#elif defined(CONFIG_MACH_EDGE_TD)
#include "board-edgetd.h"
#elif defined(CONFIG_MACH_ENDEAVORU)
#include "board-endeavoru.h"
#endif
#include "gpio-names.h"
#include "baseband-xmm-power.h"
#include <htc/log.h>

MODULE_LICENSE("GPL");

unsigned long modem_ver = XMM_MODEM_VER_1121;


/*
 * HTC: version history
 *
 *	v04 - bert_lin - 20111025
 *		1. remove completion & wait for probe race, use nv solution instead
 *		2. add a attribute for host usb debugging
 *	v05 - bert_lin - 20111026
 *		1. sync patch from nv michael. re-arrange the first_time var
 *			after flight off, device cant goes to L2 suspend
 *		2. modify the files to meet the coding style
 *	v06 - bert_lin - 20111026
 *		1. item 12: L0 -> flight -> suspend fail because of wakelock holding
 *			check wakelock in L3 and release it if neccessary
 *	v07 - bert_lin - 20111104
 *		workaround for item 18, AP L2->L0 fail! submit urb return -113
 *		add more logs on usb_chr for modem download issue
 *	v08 - bert_lin - 20111125
 *		workaround, origin l3 -> host_wake -> deepsleep
 *			after: L3 -> host_wakeup -> noirq suspend fail -> resume
 *	v09 - bert_lin - 20111214
 *		autopm
 *	v10 - bert_lin - 20111226
 *		log reduce
 */

/* HTC: macro, variables */
#include <mach/htc_hostdbg.h>
#define MODULE_NAME "[XMM_v10]"
unsigned int host_dbg_flag = 0;
EXPORT_SYMBOL(host_dbg_flag);

/* HTC: provide interface for user space to enable usb host debugging */
static ssize_t host_dbg_show(struct device *dev,
	struct device_attribute *attr, char *buf);
static ssize_t host_dbg_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size);
static DEVICE_ATTR(host_dbg,  0664, host_dbg_show,  host_dbg_store);

/* HTC: Create attribute for host debug purpose */
static ssize_t host_dbg_show(struct device *dev,
            struct device_attribute *attr,
            char *buf)
{
    int ret = -EINVAL;
    ret = sprintf(buf, "%x\n", host_dbg_flag);
    return ret;
}

/**
 * HTC: get the runtime debug flags from user.
 *
 * @buf: user strings
 * @size: user strings plus one 0x0a char
 */
static ssize_t host_dbg_store(struct device *dev,
            struct device_attribute *attr,
            const char *buf, size_t size)
{
    const int hedge = 2 + 8 + 1;
    /* hedge: "0x" 2 chars, max: 8 chars, plus one 0x0a char */

    pr_info(MODULE_NAME "%s size = %d\n", __func__, size);
    if (size > hedge) {
        pr_info(MODULE_NAME "%s size > hedge:%d, return\n",
                __func__, hedge);
        return size;
    }
    host_dbg_flag  = simple_strtoul(buf, NULL, 16);
    pr_info(MODULE_NAME "%s set host_dbg_flag as 0x%08x\n",
            __func__, host_dbg_flag);

    return size;
}
/*============================================================*/
struct pm_qos_request_list modem_boost_cpu_freq_req;
EXPORT_SYMBOL_GPL(modem_boost_cpu_freq_req);
#define BOOST_CPU_FREQ_MIN	1500000

EXPORT_SYMBOL(modem_ver);

unsigned long modem_flash;
EXPORT_SYMBOL(modem_flash);

unsigned long modem_pm = 1;
EXPORT_SYMBOL(modem_pm);

unsigned long autosuspend_delay = 5000; /* 5000 msec */
EXPORT_SYMBOL(autosuspend_delay);

unsigned long enum_delay_ms = 1000; /* ignored if !modem_flash */

module_param(modem_ver, ulong, 0644);
MODULE_PARM_DESC(modem_ver,
	"baseband xmm power - modem software version");
module_param(modem_flash, ulong, 0644);
MODULE_PARM_DESC(modem_flash,
	"baseband xmm power - modem flash (1 = flash, 0 = flashless)");
module_param(modem_pm, ulong, 0644);
MODULE_PARM_DESC(modem_pm,
	"baseband xmm power - modem power management (1 = pm, 0 = no pm)");
module_param(enum_delay_ms, ulong, 0644);
MODULE_PARM_DESC(enum_delay_ms,
	"baseband xmm power - delay in ms between modem on and enumeration");
module_param(autosuspend_delay, ulong, 0644);
MODULE_PARM_DESC(autosuspend_delay,
	"baseband xmm power - autosuspend delay for autopm");

#define DEFAULT_AUTOSUSPEND_DELAY 3000
#define SHORT_AUTOSUSPEND_DELAY 100

#define auto_sleep(x)	\
	if (in_interrupt() || in_atomic())\
		mdelay(x);\
	else\
		msleep(x);

static bool short_autosuspend;

#ifdef BB_XMM_OEM1

#define pr_debug pr_info


#endif /* BB_XMM_OEM1 */

static struct usb_device_id xmm_pm_ids[] = {
	{ USB_DEVICE(VENDOR_ID, PRODUCT_ID),
	.driver_info = 0 },
	{}
};
//for power on modem
static struct gpio tegra_baseband_gpios[] = {
	{ -1, GPIOF_OUT_INIT_LOW,  "BB_RSTn" },
	{ -1, GPIOF_OUT_INIT_LOW,  "BB_ON"   },
	{ -1, GPIOF_OUT_INIT_LOW,  "IPC_BB_WAKE" },
	{ -1, GPIOF_IN,            "IPC_AP_WAKE" },
	{ -1, GPIOF_OUT_INIT_HIGH, "IPC_HSIC_ACTIVE" },
	{ -1, GPIOF_IN,            "IPC_HSIC_SUS_REQ" },
#ifdef BB_XMM_OEM1
	{ BB_VDD_EN, GPIOF_OUT_INIT_LOW, "BB_VDD_EN" },
	{ AP2BB_RST_PWRDWNn, GPIOF_OUT_INIT_LOW, "AP2BB_RST_PWRDWNn" },
#endif BB_XMM_OEM1
};
/*HTC*/
//for power consumation , power off modem
static struct gpio tegra_baseband_gpios_power_off_modem[] = {
	{ -1, GPIOF_OUT_INIT_LOW,  "BB_RSTn" },
	{ -1, GPIOF_OUT_INIT_LOW,  "BB_ON"   },
	{ -1, GPIOF_OUT_INIT_LOW,  "IPC_BB_WAKE" },
	{ -1, GPIOF_OUT_INIT_LOW,   "IPC_AP_WAKE" },
	{ -1, GPIOF_OUT_INIT_LOW, "IPC_HSIC_ACTIVE" },
	{ -1, GPIOF_IN,            "IPC_HSIC_SUS_REQ" },
#ifdef BB_XMM_OEM1
	{ BB_VDD_EN, GPIOF_OUT_INIT_LOW, "BB_VDD_EN" },
	{ AP2BB_RST_PWRDWNn, GPIOF_OUT_INIT_LOW, "AP2BB_RST_PWRDWNn" },
#endif BB_XMM_OEM1
};


static enum {
	IPC_AP_WAKE_UNINIT,
	IPC_AP_WAKE_IRQ_READY,
	IPC_AP_WAKE_INIT1,
	IPC_AP_WAKE_INIT2,
	IPC_AP_WAKE_L,
	IPC_AP_WAKE_H,
} ipc_ap_wake_state = IPC_AP_WAKE_INIT2;

enum baseband_xmm_powerstate_t baseband_xmm_powerstate;
static struct workqueue_struct *workqueue;
static struct work_struct init1_work;
static struct work_struct init2_work;
static struct work_struct L2_resume_work;
static struct delayed_work init4_work;
static struct baseband_power_platform_data *baseband_power_driver_data;
static bool register_hsic_device;
static struct wake_lock wakelock;
static struct usb_device *usbdev;
static bool CP_initiated_L2toL0;
static bool modem_power_on;
static bool first_time = true;
static int power_onoff;
static void baseband_xmm_power_L2_resume(void);
static DEFINE_MUTEX(baseband_xmm_onoff_lock);
static int baseband_xmm_power_driver_handle_resume(
			struct baseband_power_platform_data *data);

static bool wakeup_pending;
static int uart_pin_pull_state=1; // 1 for UART, 0 for GPIO
static bool modem_sleep_flag = false;
static struct regulator *enterprise_dsi_reg = NULL;//for avdd_csi_dsi
static spinlock_t xmm_lock;
static bool system_suspending;

static int reenable_autosuspend; //ICS only
static int htcpcbid=0;

static struct workqueue_struct *workqueue_susp;
static struct work_struct work_shortsusp, work_defaultsusp;

/*HTC++*/
static ssize_t debug_gpio_dump(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{	
	int rValue=0;
	
	pr_info("**********************\n");
	int value = gpio_get_value(TEGRA_GPIO_PM4);
	pr_info("BB_VDD_EN=%d\n", value);
	value = gpio_get_value(TEGRA_GPIO_PC1);
	pr_info("AP2BB_RST_PWRDWNn=%d\n", value);
	value = gpio_get_value(TEGRA_GPIO_PN0);
	pr_info("AP2BB_RSTn=%d\n", value);	
	value = gpio_get_value(TEGRA_GPIO_PN3);
	pr_info("AP2BB_PWRON=%d\n", value);
	value = gpio_get_value(TEGRA_GPIO_PN2);
	pr_info("BB2AP_RADIO_FATAL=%d\n", value);
	value = gpio_get_value(TEGRA_GPIO_PN1);
	pr_info("IPC_HSIC_ACTIVE =%d\n", value);
	value = gpio_get_value(TEGRA_GPIO_PV0);
	pr_info("HSIC_SUS_REQ=%d\n", value);
	value = gpio_get_value(TEGRA_GPIO_PC6);
	pr_info("IPC_BB_WAKE=%d\n", value);
	value = gpio_get_value(TEGRA_GPIO_PJ0);
	pr_info("IPC_AP_WAKE=%d\n", value);

	
	/*set BB2AP_SUSPEND_REQ Pin (TEGRA_GPIO_PV0) to OutPut High to trigger Modem fatal*/
	int ret=gpio_direction_output(TEGRA_GPIO_PV0,1);
	pr_info("set BB2AP_SUSPEND_REQ Pin (TEGRA_GPIO_PV0) to OutPut High to trigger Modem fatal\n");
	if (ret < 0)
		pr_err("%s: set BB2AP_SUSPEND_REQ Pin to Output error\n", __func__);
	value = gpio_get_value(TEGRA_GPIO_PV0);
	pr_info("HSIC_SUS_REQ=%d\n", value);

	/*set host_active for interrupet modem*/
	value = gpio_get_value(TEGRA_GPIO_PN1);
	pr_info("Oringial IPC_HSIC_ACTIVE =%d\n", value);
	if(value==1)rValue=0;
	else if(value==0)rValue=1;

	gpio_set_value(TEGRA_GPIO_PN1,rValue);
	msleep(100);
	gpio_set_value(TEGRA_GPIO_PN1,value);
	
	pr_info("**********************\n");
	

return count;
}
static DEVICE_ATTR(debug_gpio_dump, S_IRUSR | S_IWUSR | S_IRGRP,
		NULL, debug_gpio_dump);

int enable_avdd_dsi_csi_power()
{
	 pr_info(MODULE_NAME "[xmm]%s\n",__func__);
	int ret=0;
	if (enterprise_dsi_reg == NULL) {
		enterprise_dsi_reg = regulator_get(NULL, "avdd_dsi_csi");
		pr_info(MODULE_NAME "[xmm]%s regulator_getED\n",__func__);
		if (IS_ERR_OR_NULL(enterprise_dsi_reg)) {
			pr_err("dsi: Could not get regulator avdd_dsi_csi\n");
				enterprise_dsi_reg = NULL;
				return PTR_ERR(enterprise_dsi_reg);
		}
	}
	ret = regulator_enable(enterprise_dsi_reg);
	if (ret < 0) {
		printk(KERN_ERR
			"DSI regulator avdd_dsi_csi couldn't be enabled\n",ret);
		
	}
		return ret;

}
int disable_avdd_dsi_csi_power()
{
	 pr_info(MODULE_NAME "[xmm]%s\n",__func__);
	int ret=0;
	if (enterprise_dsi_reg == NULL) {
		enterprise_dsi_reg = regulator_get(NULL, "avdd_dsi_csi");
		pr_info(MODULE_NAME "[xmm]%s regulator_getED\n",__func__);
		if (IS_ERR_OR_NULL(enterprise_dsi_reg)) {
			pr_err("dsi: Could not get regulator avdd_dsi_csi\n");
				enterprise_dsi_reg = NULL;
				return PTR_ERR(enterprise_dsi_reg);
		}
	}
	ret = regulator_disable(enterprise_dsi_reg);
	if (ret < 0) {
		printk(KERN_ERR
			"DSI regulator avdd_dsi_csi couldn't be disabled\n",ret);

	}
	enterprise_dsi_reg=NULL;
	return ret;
}

int gpio_config_only_one(unsigned gpio, unsigned long flags, const char *label)
{
	int err=0;


	if (flags & GPIOF_DIR_IN)
		err = gpio_direction_input(gpio);
	else
		err = gpio_direction_output(gpio,
				(flags & GPIOF_INIT_HIGH) ? 1 : 0);

	return err;
}

int gpio_config_only_array(struct gpio *array, size_t num)
{
	int i, err=0;

	for (i = 0; i < num; i++, array++) {
		err = gpio_config_only_one(array->gpio, array->flags, array->label);
		if (err)
			goto err_free;
	}
	return 0;

err_free:
	//while (i--)
		//gpio_free((--array)->gpio);
	return err;
}


int gpio_request_only_one(unsigned gpio,const char *label)
{
	int err=0;

	err = gpio_request(gpio, label);
	if (err)
		return err;
}


int gpio_request_only_array(struct gpio *array, size_t num)
{
	int i, err=0;

	for (i = 0; i < num; i++, array++) {
		err = gpio_request_only_one(array->gpio, array->label);
		if (err)
			goto err_free;
	}
	return 0;

err_free:
	while (i--)
		gpio_free((--array)->gpio);
	return err;
}


static int gpio_o_l_uart(int gpio, char* name)
{
	int ret=0;
	pr_debug(MODULE_NAME "%s ,name=%s gpio=%d\n", __func__,name,gpio);
	ret = gpio_direction_output(gpio, 0);
	if (ret < 0) {
		pr_err(" %s: gpio_direction_output failed %d\n", __func__, ret);
		gpio_free(gpio);
		return ret;
	}
	tegra_gpio_enable(gpio);
	gpio_export(gpio, true);
}

void modem_on_for_uart_config()
{


	pr_debug(MODULE_NAME "%s ,first_time=%s uart_pin_pull_low=%d\n", __func__,first_time?"true":"false",uart_pin_pull_state);
	if(uart_pin_pull_state==0){
	//if uart pin pull low, then we put back to normal
	pr_debug(MODULE_NAME "%s tegra_gpio_disable for UART\n", __func__);
	tegra_gpio_disable(TEGRA_GPIO_PJ7);
	tegra_gpio_disable(TEGRA_GPIO_PK7);
	tegra_gpio_disable(TEGRA_GPIO_PB0);
	tegra_gpio_disable(TEGRA_GPIO_PB1);
	uart_pin_pull_state=1;//set back to UART
	}


}

int modem_off_for_uart_config()
{
	int err=0;

	pr_debug(MODULE_NAME "%s uart_pin_pull_low=%d\n", __func__,uart_pin_pull_state);
	if(uart_pin_pull_state==1){
	//if uart pin not pull low yet, then we pull them low+enable
	err=gpio_o_l_uart(TEGRA_GPIO_PJ7, "IMC_UART_TX");
	err=gpio_o_l_uart(TEGRA_GPIO_PK7, "IMC_UART_RTS");
	err=gpio_o_l_uart(TEGRA_GPIO_PB0  ,"IMC_UART_RX");
	err=gpio_o_l_uart(TEGRA_GPIO_PB1, "IMC_UART_CTS");
	uart_pin_pull_state=0;//chagne to gpio
	}

	return err;
}

int modem_off_for_usb_config(struct gpio *array, size_t num)
{
	pr_debug(MODULE_NAME "%s 1219_01\n", __func__);

	int err=0;
	err = gpio_config_only_array(tegra_baseband_gpios_power_off_modem,
		ARRAY_SIZE(tegra_baseband_gpios_power_off_modem));
	if (err < 0) {
		pr_err("%s - gpio_config_array gpio(s) for modem off failed\n", __func__);
		return -ENODEV;
	}
	return err;
}

int modem_on_for_usb_config(struct gpio *array, size_t num)
{
	pr_debug(MODULE_NAME "%s \n", __func__);

	int err=0;
	err = gpio_config_only_array(tegra_baseband_gpios,
		ARRAY_SIZE(tegra_baseband_gpios));
	if (err < 0) {
		pr_err("%s - gpio_config_array gpio(s) for modem off failed\n", __func__);
		return -ENODEV;
	}

	return err;

}

int config_gpio_for_power_off()
{
	int err=0;

	pr_debug(MODULE_NAME "%s for power consumation 4st \n", __func__);
#if 1
		/* config  baseband gpio(s) for modem off */

		err = modem_off_for_usb_config(tegra_baseband_gpios_power_off_modem,
			ARRAY_SIZE(tegra_baseband_gpios_power_off_modem));
		if (err < 0) {
			pr_err("%s - gpio_config_array gpio(s) for modem off failed\n", __func__);
			return -ENODEV;
		}
#endif
		/* config  uart gpio(s) for modem off */
		err=modem_off_for_uart_config();
		if (err < 0) {
			pr_err("%s - modem_off_for_uart_config gpio(s)\n", __func__);
			return -ENODEV;
		}


	return err;
}

int config_gpio_for_power_on()
{
	int err=0;

	pr_debug(MODULE_NAME "%s for power consumation 4st \n", __func__);
#if 1
		/* config  baseband gpio(s) for modem off */

		err = modem_on_for_usb_config(tegra_baseband_gpios,
			ARRAY_SIZE(tegra_baseband_gpios));
		if (err < 0) {
			pr_err("%s - gpio_config_array gpio(s) for modem off failed\n", __func__);
			return -ENODEV;
		}
#endif
		/* config  uart gpio(s) for modem off */
		modem_on_for_uart_config();

	return err;
}
/*HTC--*/

extern void platfrom_set_flight_mode_onoff(bool mode_on);


static int baseband_modem_power_on(struct baseband_power_platform_data *data)
{
#ifdef BB_XMM_OEM1
	/* HTC: called in atomic context */
	int ret=0, i=0;

	pr_info("%s VP: 03/08 22.52{\n", __func__);
	if (!data) {
		pr_err("%s: data is NULL\n", __func__);
		return -1;
	}

	/* reset / power on sequence */
	gpio_set_value(BB_VDD_EN, 1); /* give modem power */
	auto_sleep(1);
	gpio_set_value(data->modem.xmm.bb_rst, 0); /* set to low first */
	//pr_debug("%s(%d)\n", __func__, __LINE__);

	for (i = 0; i < 7; i++) /* 5 ms BB_RST low */
		udelay(1000);

	ret = gpio_get_value(AP2BB_RST_PWRDWNn);
	//pr_debug("%s(%d) get AP2BB_RST_PWRDWNn=%d \n", __func__, __LINE__, ret);
	//pr_debug("%s(%d) set AP2BB_RST_PWRDWNn=1\n", __func__, __LINE__);
	gpio_set_value(AP2BB_RST_PWRDWNn, 1); /* 20 ms RST_PWRDWNn high */
	auto_sleep(25); /* need 20 but 40 is more safe */ //steven markded

	//pr_debug("%s(%d) set modem.xmm.bb_rst=1\n", __func__, __LINE__);
	gpio_set_value(data->modem.xmm.bb_rst, 1); /* 1 ms BB_RST high */
	auto_sleep(40); /* need 20 but 40 is more safe */

	//pr_debug("%s(%d) set modem.xmm.bb_on=1 duration is 60us\n", __func__, __LINE__);
	gpio_set_value(data->modem.xmm.bb_on, 1); /* power on sequence */
	udelay(60);

	gpio_set_value(data->modem.xmm.bb_on, 0);
	//pr_debug("%s(%d) set modem.xmm.bb_on=0\n", __func__, __LINE__);
	auto_sleep(10);
	pr_info("%s }\n", __func__);

	pr_info("%s:VP pm qos request CPU 1.5GHz\n", __func__);
	pm_qos_update_request(&modem_boost_cpu_freq_req, (s32)BOOST_CPU_FREQ_MIN);

#else  /* !BB_XMM_OEM1 */

	/* set IPC_HSIC_ACTIVE active */
	gpio_set_value(baseband_power_driver_data->
		modem.xmm.ipc_hsic_active, 1);

	/* wait 20 ms */
	msleep(20);

	/* reset / power on sequence */
	msleep(40);
	gpio_set_value(data->modem.xmm.bb_rst, 1);
	msleep(1);
	gpio_set_value(data->modem.xmm.bb_on, 1);
	udelay(40);
	gpio_set_value(data->modem.xmm.bb_on, 0);

#endif /* !BB_XMM_OEM1 */
	return 0;
}

static int baseband_xmm_power_on(struct platform_device *device)
{
	struct baseband_power_platform_data *data
		= (struct baseband_power_platform_data *)
			device->dev.platform_data;
	int ret; /* HTC: ENR#U wakeup src fix */
	int value;

	pr_debug(MODULE_NAME "%s{\n", __func__);

	/* check for platform data */
	if (!data) {
		pr_err("%s: !data\n", __func__);
		return -EINVAL;
	}

	if (baseband_xmm_powerstate != BBXMM_PS_UNINIT) {
		pr_err("%s: baseband_xmm_powerstate != BBXMM_PS_UNINIT\n",
			__func__);
		return -EINVAL;
	}
#if 1 /*HTC*/	
	pr_debug(MODULE_NAME " htc_get_pcbid_info= %d\n",htcpcbid );
	if(htcpcbid < PROJECT_PHASE_XE) {
		enable_avdd_dsi_csi_power();
	}
#endif

	///*HTC*/
	/*set Radio fatal Pin to Iput*/
	ret=gpio_direction_input(TEGRA_GPIO_PN2);
	if (ret < 0)
				pr_err("%s: set Radio fatal Pin to Iput error\n", __func__);

	/*set BB2AP_SUSPEND_REQ Pin (TEGRA_GPIO_PV0) to Iput*/
	ret=gpio_direction_input(TEGRA_GPIO_PV0);
	if (ret < 0)
				pr_err("%s: set BB2AP_SUSPEND_REQ Pin to Iput error\n", __func__);

	/*config back the uart pin*/
	config_gpio_for_power_on();

	/* reset the state machine */
	baseband_xmm_powerstate = BBXMM_PS_INIT;
	first_time = true;
	modem_sleep_flag = false;

	/* HTC use IPC_AP_WAKE_INIT2 */
	if (modem_ver < XMM_MODEM_VER_1130)
		ipc_ap_wake_state = IPC_AP_WAKE_INIT1;
	else
		ipc_ap_wake_state = IPC_AP_WAKE_INIT2;

	/* pr_info("%s - %d\n", __func__, __LINE__); */

	/* register usb host controller */
	if (!modem_flash) {
		/* pr_info("%s - %d\n", __func__, __LINE__); */
		/* register usb host controller only once */
		if (register_hsic_device) {
			pr_debug("%s(%d)register usb host controller\n", __func__, __LINE__);
			modem_power_on = true;
			if (data->hsic_register)
				data->modem.xmm.hsic_device =
					data->hsic_register();
			else
				pr_err("%s: hsic_register is missing\n",
					__func__);
			register_hsic_device = false;
		} else {
			/* register usb host controller */
			if (data->hsic_register)
				data->modem.xmm.hsic_device =
						data->hsic_register();
			/* turn on modem */
			pr_info("%s call baseband_modem_power_on\n", __func__);
			baseband_modem_power_on(data);
		}
	}

	/* HTC: ENR#U wakeup src fix */
	pr_info("%s: before enable irq wake \n", __func__);
	ret = enable_irq_wake(gpio_to_irq(data->modem.xmm.ipc_ap_wake));
	if (ret < 0)
		pr_err("%s: enable_irq_wake error\n", __func__);
	/* HTC: remove platfrom_set_flight_mode_onoff(false); for ENR */
#if 1
	/*For SIM det*/
	//pr_info("%s: before enable irq wake SIM det \n", __func__);
	//ret = enable_irq_wake(gpio_to_irq(TEGRA_GPIO_PI5));
	//if (ret < 0)
	//	pr_err("%s: enable_irq_wake error\n", __func__);
	/*For Radio fatal*/
	pr_info("%s: before enable irq wake Radio fatal \n", __func__);
	ret = enable_irq_wake(gpio_to_irq(TEGRA_GPIO_PN2));
	if (ret < 0)
		pr_err("%s: enable_irq_wake error\n", __func__);
#endif
	pr_debug("%s }\n", __func__);

	return 0;
}

static int baseband_xmm_power_off(struct platform_device *device)
{
	struct baseband_power_platform_data *data;
	int ret; /* HTC: ENR#U wakeup src fix */
	unsigned long flags;

	pr_debug("%s {\n", __func__);

	if (baseband_xmm_powerstate == BBXMM_PS_UNINIT) {
		pr_err("%s: baseband_xmm_powerstate != BBXMM_PS_UNINIT\n",
			__func__);
		return -EINVAL;
	}

	/* check for device / platform data */
	if (!device) {
		pr_err("%s: !device\n", __func__);
		return -EINVAL;
	}
	data = (struct baseband_power_platform_data *)
		device->dev.platform_data;
	if (!data) {
		pr_err("%s: !data\n", __func__);
		return -EINVAL;
	}
	
	ipc_ap_wake_state = IPC_AP_WAKE_UNINIT;
	/* Set this flag to have proper flash-less first enumearation */
	register_hsic_device = true;


	/* HTC: ENR#U wakeup src fix */
	pr_info("%s: Disable SIM DET ,before disable irq wake \n", __func__);
	ret = disable_irq_wake(gpio_to_irq(data->modem.xmm.ipc_ap_wake));
	if (ret < 0)
		pr_err("%s: disable_irq_wake error\n", __func__);
#if 1
	/*HTC for SIM DET*/
	//pr_info("%s: before disable irq wake SIM DET\n", __func__);
	//ret = disable_irq_wake(gpio_to_irq(TEGRA_GPIO_PI5));
	//if (ret < 0)
	//	pr_err("%s: disable_irq_wake error\n", __func__);

	/*HTC for RADIO FATAL*/
	pr_info("%s: before disable irq wake RADIO FATAL\n", __func__);
	ret = disable_irq_wake(gpio_to_irq(TEGRA_GPIO_PN2));
	if (ret < 0)
		pr_err("%s: disable_irq_wake error\n", __func__);
#endif
	/* unregister usb host controller */
	pr_info("%s: hsic device: %x\n", __func__, data->modem.xmm.hsic_device);
	if (data->hsic_unregister)
		data->hsic_unregister(data->modem.xmm.hsic_device);
	else
		pr_err("%s: hsic_unregister is missing\n", __func__);

	/* set IPC_HSIC_ACTIVE low */
	gpio_set_value(baseband_power_driver_data->
		modem.xmm.ipc_hsic_active, 0);

	/* wait 20 ms */
	msleep(20);

	/* drive bb_rst low */
	//Sophia:0118:modem power down sequence: don't need to clear BB_RST
	//gpio_set_value(data->modem.xmm.bb_rst, 0);
#ifdef BB_XMM_OEM1
	//msleep(1);
	msleep(20);

	gpio_set_value(BB_VDD_EN, 0); /* turn off the modem power */
	msleep(68);//for IMC Modem discharge.

#else  /* !BB_XMM_OEM1 */
	msleep(1);
#endif /* !BB_XMM_OEM1 */

#if 1/*HTC*/

	//for power consumation
	int err=0;
	pr_debug("%s config_gpio_for_power_off\n", __func__);
	config_gpio_for_power_off();
	//err=config_gpio_for_power_off();
	//if (err < 0) {
	//	pr_err("%s - config_gpio_for_power_off gpio(s)\n", __func__);
	//	return -ENODEV;
	//}
#endif
	/* HTC: remove platfrom_set_flight_mode_onoff for ENR */
	/* platfrom_set_flight_mode_onoff(true); */
	baseband_xmm_powerstate = BBXMM_PS_UNINIT;
	modem_sleep_flag = false;
	CP_initiated_L2toL0 = false;
	spin_lock_irqsave(&xmm_lock, flags);
	wakeup_pending = false;
	system_suspending = false;
	spin_unlock_irqrestore(&xmm_lock, flags);
#if 1 /*HTC*/
	pr_debug(MODULE_NAME " htc_get_pcbid_info= %d\n", htcpcbid);
	if(htcpcbid< PROJECT_PHASE_XE) {
		disable_avdd_dsi_csi_power();
	}
#endif
	/*set Radio fatal Pin to OutPut Low*/
	ret=gpio_direction_output(TEGRA_GPIO_PN2,0);
	if (ret < 0)
			pr_err("%s: set Radio fatal Pin to Output error\n", __func__);

	/*set BB2AP_SUSPEND_REQ Pin (TEGRA_GPIO_PV0) to OutPut Low*/
	ret=gpio_direction_output(TEGRA_GPIO_PV0,0);
	if (ret < 0)
			pr_err("%s: set BB2AP_SUSPEND_REQ Pin to Output error\n", __func__);	

	pr_debug("%s }\n", __func__);

	return 0;
}

static ssize_t baseband_xmm_onoff(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	int size;
	struct platform_device *device = to_platform_device(dev);

	mutex_lock(&baseband_xmm_onoff_lock);

	/* check input */
	if (buf == NULL) {
		pr_err("%s: buf NULL\n", __func__);
		return -EINVAL;
	}
	/* pr_debug("%s: count=%d\n", __func__, count); */

	/* parse input */
#ifdef BB_XMM_OEM1
	if (buf[0] == 0x01 || buf[0] == '1') {
		/* pr_info("%s: buf[0] = 0x%x\n", __func__, buf[0]); */
		power_onoff = 1;
	} else
		power_onoff = 0;
#else /* !BB_XMM_OEM1 */
	size = sscanf(buf, "%d", &power_onoff);
	if (size != 1) {
		pr_err("%s: size=%d -EINVAL\n", __func__, size);
		return -EINVAL;
	}
#endif /* !BB_XMM_OEM1 */

	pr_debug("%s power_onoff=%d count=%d, buf[0]=0x%x\n",
		__func__, power_onoff, count, buf[0]);

	if (power_onoff == 0)
		baseband_xmm_power_off(device);
	else if (power_onoff == 1)
		baseband_xmm_power_on(device);

	mutex_unlock(&baseband_xmm_onoff_lock);

	return count;
}

static DEVICE_ATTR(xmm_onoff, S_IRUSR | S_IWUSR | S_IRGRP,
		NULL, baseband_xmm_onoff);


void baseband_xmm_set_power_status(unsigned int status)
{
	struct baseband_power_platform_data *data = baseband_power_driver_data;
	int value = 0;
	unsigned long flags;

	if (baseband_xmm_powerstate == status)
		return;
	
	pr_info(MODULE_NAME"%s{ status=%d\n", __func__,status);

	switch (status) {
	case BBXMM_PS_L0:
		if (modem_sleep_flag) {
			pr_info("%s Resume from L3 without calling resume function\n",  __func__);
			baseband_xmm_power_driver_handle_resume(data);
		}
		pr_info("L0\n");
		baseband_xmm_powerstate = status;

		/* HTC: don't hold the wakelock multiple times */
		if (!wake_lock_active(&wakelock)) {
			pr_info("%s: wake_lock [%s] in L0\n",
				__func__, wakelock.name);
			wake_lock(&wakelock);
			//wake_lock_timeout(&wakelock, HZ * 5);
		}

		value = gpio_get_value(data->modem.xmm.ipc_hsic_active);
		pr_info("L0 Get current gpio_get_value(ipc_hsic_active)=%d\n", value);
		if (!value) {
			gpio_set_value(data->modem.xmm.ipc_hsic_active, 1);
			pr_info("L0 gpio set ipc_hsic_active=1 ->\n");
		}
		if (modem_power_on) {
			modem_power_on = false;
			baseband_modem_power_on(data);
		}
		//pr_info("gpio host active high->\n");
		/* hack to restart autosuspend after exiting LP0
		* (aka re-entering L0 from L3)
		*/
#if 0
//remove on 0305
		if (usbdev) {
				struct usb_interface *intf;
				intf = usb_ifnum_to_if(usbdev, 0);
				//pr_info("%s - autopm_get - usbdev = %d - %d {\n", __func__, usbdev, __LINE__);
				//pr_info("%s: cnt %d intf=%p &intf->dev=%p kobje=%s\n",
					//__func__, atomic_read(&intf->dev.power.usage_count),intf,&intf->dev,kobject_name(&intf->dev.kobj));
				if (usb_autopm_get_interface_async(intf) >= 0) {
					pr_info("get_interface_async succeeded"
							" - call put_interface\n");
					//pr_info("%s - usb_put - usbdev = %d - %d {\n", __func__, usbdev, __LINE__);
					usb_autopm_put_interface_async(intf);
					//pr_info("%s - usb_put - usbdev = %d - %d {\n", __func__, usbdev, __LINE__);
				} else {
					pr_info("get_interface_async failed"
							" - do not call put_interface\n");
				}
		}
#endif
		break;
	case BBXMM_PS_L2:
		pr_info("L2 wake_unlock[%s]\n", wakelock.name);
		baseband_xmm_powerstate = status;
		spin_lock_irqsave(&xmm_lock, flags);
		if (wakeup_pending) {
			spin_unlock_irqrestore(&xmm_lock, flags);
			baseband_xmm_power_L2_resume();
		} else {
			spin_unlock_irqrestore(&xmm_lock, flags);
			wake_unlock(&wakelock);
			modem_sleep_flag = true;
		}
		if (short_autosuspend && (&usbdev->dev)) {
				pr_debug("autosuspend delay %d ms,disable short_autosuspend\n", DEFAULT_AUTOSUSPEND_DELAY);
				queue_work(workqueue_susp, &work_defaultsusp);
				short_autosuspend = false;
		}
#if 0
		if (usbdev) {
			struct usb_interface *intf;
			intf = usb_ifnum_to_if(usbdev, 0);
			pr_info("%s: cnt %d intf=%p &intf->dev=%p kobje=%s\n",
					__func__, atomic_read(&intf->dev.power.usage_count),intf,&intf->dev,kobject_name(&intf->dev.kobj));
			}
#endif
		break;
	case BBXMM_PS_L3:
		if (baseband_xmm_powerstate == BBXMM_PS_L2TOL0) {
                       pr_info("%s: baseband_xmm_powerstate == BBXMM_PS_L2TOL0\n", __func__);
                       if (!data->modem.xmm.ipc_ap_wake) {
				spin_lock_irqsave(&xmm_lock, flags);
				wakeup_pending = true;
				spin_unlock_irqrestore(&xmm_lock, flags);
				pr_info("%s: L2 race condition-CP wakeup pending\n", __func__);
			}
		}
		pr_info("L3\n");
		baseband_xmm_powerstate = status;
		spin_lock_irqsave(&xmm_lock, flags);
		system_suspending = false;
		spin_unlock_irqrestore(&xmm_lock, flags);
		if (wake_lock_active(&wakelock)) {
			pr_info("L3 --- wake_unlock[%s]\n", wakelock.name);
			wake_unlock(&wakelock);
		}
		gpio_set_value(data->modem.xmm.ipc_hsic_active, 0);
		pr_info("Set gpio host active low->\n");
		break;
	case BBXMM_PS_L2TOL0:
		pr_info("L2->L0\n");
		spin_lock_irqsave(&xmm_lock, flags);
		system_suspending = false;
		wakeup_pending = false;
		spin_unlock_irqrestore(&xmm_lock, flags);
		/* do this only from L2 state */
		if (baseband_xmm_powerstate == BBXMM_PS_L2) {
			baseband_xmm_powerstate = status;
			//pr_debug("BB XMM POWER STATE = %d\n", status);
			baseband_xmm_power_L2_resume();
		}else{
			baseband_xmm_powerstate = status;
		}
	default:
		baseband_xmm_powerstate = status;
		break;
	}

	pr_info(MODULE_NAME "%s } baseband_xmm_powerstate = %d\n", __func__, baseband_xmm_powerstate);
}
EXPORT_SYMBOL_GPL(baseband_xmm_set_power_status);

irqreturn_t baseband_xmm_power_ipc_ap_wake_irq(int irq, void *dev_id)
{
	int value;
	struct baseband_power_platform_data *data = baseband_power_driver_data;

	/* pr_debug("%s\n", __func__); */

	value = gpio_get_value(data->modem.xmm.ipc_ap_wake);

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
			queue_work(workqueue, &init1_work);
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
			queue_work(workqueue, &init2_work);
		}
	} else {
		if (!value) {
			pr_debug("%s - falling\n", __func__);
			/* First check it a CP ack or CP wake  */
			value = gpio_get_value
				(data->modem.xmm.ipc_bb_wake);
			if (value) {
				pr_debug("cp ack for bb_wake\n");
				ipc_ap_wake_state = IPC_AP_WAKE_L;
				return IRQ_HANDLED;
			}
			spin_lock(&xmm_lock);
			wakeup_pending = true;
			if (system_suspending) {
				spin_unlock(&xmm_lock);
				pr_info("system_suspending=1, Just set wakup_pending flag=true\n");
			} else {
				if (baseband_xmm_powerstate ==
							BBXMM_PS_L3) {
					spin_unlock(&xmm_lock);
					pr_info(" CP L3 -> L0\n");
					pr_info("set wakeup_pending=true, wait for no-irq-resuem if you are not under LP0 yet !.\n");
					pr_info("set wakeup_pending=true, wait for system resume if you already under LP0.\n");
				} else if (baseband_xmm_powerstate ==
							BBXMM_PS_L2) {
					CP_initiated_L2toL0 = true;
					spin_unlock(&xmm_lock);
					baseband_xmm_set_power_status
					(BBXMM_PS_L2TOL0);
				} else {
					CP_initiated_L2toL0 = true;
					spin_unlock(&xmm_lock);
					pr_info(" CP wakeup pending- new race condition");
				}
			}
			/* save gpio state */
			ipc_ap_wake_state = IPC_AP_WAKE_L;
		} else {
			pr_debug("%s - rising\n", __func__);
			value = gpio_get_value
				(data->modem.xmm.ipc_hsic_active);
			if (!value) {
				pr_info("host active low: ignore request\n");
				ipc_ap_wake_state = IPC_AP_WAKE_H;
				return IRQ_HANDLED;
			}
			value = gpio_get_value
				(data->modem.xmm.ipc_bb_wake);
			if (value) {
				/* Clear the slave wakeup request */
				gpio_set_value
					(data->modem.xmm.ipc_bb_wake, 0);
				pr_info("set gpio slave wakeup low done ->\n");
				
			}
			if (reenable_autosuspend && usbdev) {
                               pr_info("set reenable_autosuspend false\n");
                               reenable_autosuspend = false;
                               struct usb_interface *intf;
                               intf = usb_ifnum_to_if(usbdev, 0);
                               if (usb_autopm_get_interface_async(intf) >= 0) {
                                       pr_info("get_interface_async succeeded"
                                               " - call put_interface\n");
                                       usb_autopm_put_interface_async(intf);
                               } else {
                                       pr_info("get_interface_async failed"
                                               " - do not call put_interface\n");
                               }

			}
			if (short_autosuspend&& (&usbdev->dev)) {
				 pr_debug("set autosuspend delay %d ms\n", SHORT_AUTOSUSPEND_DELAY);
				 queue_work(workqueue_susp, &work_shortsusp);
			}
			modem_sleep_flag = false;
			baseband_xmm_set_power_status(BBXMM_PS_L0);

		/* save gpio state */
		ipc_ap_wake_state = IPC_AP_WAKE_H;
		}

	}

	return IRQ_HANDLED;
}

EXPORT_SYMBOL(baseband_xmm_power_ipc_ap_wake_irq);

static void baseband_xmm_power_init1_work(struct work_struct *work)
{
	int value;

	pr_debug("%s {\n", __func__);

	/* check if IPC_HSIC_ACTIVE high */
	value = gpio_get_value(baseband_power_driver_data->
		modem.xmm.ipc_hsic_active);
	if (value != 1) {
		pr_err("%s - expected IPC_HSIC_ACTIVE high!\n", __func__);
		return;
	}

	/* wait 100 ms */
	msleep(100);

	/* set IPC_HSIC_ACTIVE low */
	gpio_set_value(baseband_power_driver_data->
		modem.xmm.ipc_hsic_active, 0);

	/* wait 10 ms */
	msleep(10);

	/* set IPC_HSIC_ACTIVE high */
	gpio_set_value(baseband_power_driver_data->
		modem.xmm.ipc_hsic_active, 1);

	/* wait 20 ms */
	msleep(20);

#ifdef BB_XMM_OEM1
	/* set IPC_HSIC_ACTIVE low */
	gpio_set_value(baseband_power_driver_data->
		modem.xmm.ipc_hsic_active, 0);
	printk(KERN_INFO"%s merge need check set IPC_HSIC_ACTIVE low\n", __func__);
#endif /* BB_XMM_OEM1 */

	pr_debug("%s }\n", __func__);
}

static void baseband_xmm_power_init2_work(struct work_struct *work)
{
	struct baseband_power_platform_data *data = baseband_power_driver_data;

	pr_debug("%s\n", __func__);

	/* check input */
	if (!data)
		return;

	/* register usb host controller only once */
	if (register_hsic_device) {
		if (data->hsic_register)
			data->modem.xmm.hsic_device = data->hsic_register();
		else
			pr_err("%s: hsic_register is missing\n", __func__);
		register_hsic_device = false;
	}
}

/* Do the work for AP/CP initiated L2->L0 */
static void baseband_xmm_power_L2_resume(void)
{
	struct baseband_power_platform_data *data = baseband_power_driver_data;
	int value;
	int delay = 10000; /* maxmum delay in msec */
	unsigned long flags;


	pr_debug("%s\n", __func__);

	if (!baseband_power_driver_data)
		return;
	/* claim the wakelock here to avoid any system suspend */
	if (!wake_lock_active(&wakelock))
		wake_lock(&wakelock);

	modem_sleep_flag = false;
	spin_lock_irqsave(&xmm_lock, flags);
	wakeup_pending = false;
	spin_unlock_irqrestore(&xmm_lock, flags);

	if (CP_initiated_L2toL0)  {
		pr_info("CP L2->L0\n");
		CP_initiated_L2toL0 = false;
		queue_work(workqueue, &L2_resume_work);
#if 0
		if (usbdev) {
			struct usb_interface *intf;
			intf = usb_ifnum_to_if(usbdev, 0);
			pr_info("%s: cnt %d intf=%p &intf->dev=%p kobje=%s\n",
					__func__, atomic_read(&intf->dev.power.usage_count),intf,&intf->dev,kobject_name(&intf->dev.kobj));
			}
#endif
	} else {
		/* set the slave wakeup request */
		pr_info("AP L2->L0\n");
		value = gpio_get_value(data->modem.xmm.ipc_ap_wake);
		if (value) {
			pr_debug("waiting for host wakeup from CP...\n");
			/* wake bb */
			gpio_set_value(data->modem.xmm.ipc_bb_wake, 1);
			do {
				hr_msleep(1);
				value = gpio_get_value(
					data->modem.xmm.ipc_ap_wake);
				delay--;
			} while ((value) && (delay));
			if (delay)
				pr_debug("Get gpio host wakeup low <-\n");
			else
				pr_info("!!AP L2->L0 Failed\n");
		} else {
			pr_info("CP already ready\n");
		}
	}
}

static void baseband_xmm_power_shortsusp(struct work_struct *work)
{
	  pr_debug("%s {\n", __func__);
	 if (!usbdev || !&usbdev->dev) {
		pr_err("%s usbdev is invalid\n", __func__);
		return;
}
	 pm_runtime_set_autosuspend_delay(&usbdev->dev, SHORT_AUTOSUSPEND_DELAY);
	pr_debug("} %s\n", __func__);
	}

static void baseband_xmm_power_defaultsusp(struct work_struct *work)
{
		  pr_debug("%s {\n", __func__);
		  if (!usbdev || !&usbdev->dev) {
			 pr_err("%s usbdev is invalid\n", __func__);
			return;
	 	}
		pm_runtime_set_autosuspend_delay(&usbdev->dev, DEFAULT_AUTOSUSPEND_DELAY);
	 	pr_debug("} %s\n", __func__);	
	
	 
}

/* Do the work for CP initiated L2->L0 */
static void baseband_xmm_power_L2_resume_work(struct work_struct *work)
{
	struct usb_interface *intf;

	pr_info("%s {\n", __func__);

	if (!usbdev) {
		pr_info("%s - !usbdev\n", __func__);
		return;
	}

	usb_lock_device(usbdev);
	intf = usb_ifnum_to_if(usbdev, 0);
	if (usb_autopm_get_interface(intf) == 0)
		usb_autopm_put_interface(intf);
	usb_unlock_device(usbdev);

	pr_info("} %s\n", __func__);
}

static void baseband_xmm_power_reset_on(void)
{
	/* reset / power on sequence */
	msleep(40);
	gpio_set_value(baseband_power_driver_data->modem.xmm.bb_rst, 1);
	msleep(1);
	gpio_set_value(baseband_power_driver_data->modem.xmm.bb_on, 1);
	udelay(40);
	gpio_set_value(baseband_power_driver_data->modem.xmm.bb_on, 0);
}

static struct baseband_xmm_power_work_t *baseband_xmm_power_work;

static void baseband_xmm_power_work_func(struct work_struct *work)
{
	struct baseband_xmm_power_work_t *bbxmm_work
		= (struct baseband_xmm_power_work_t *) work;

	pr_debug("%s - work->sate=%d\n", __func__, bbxmm_work->state);

	switch (bbxmm_work->state) {
	case BBXMM_WORK_UNINIT:
		pr_debug("BBXMM_WORK_UNINIT\n");
		break;
	case BBXMM_WORK_INIT:
		pr_debug("BBXMM_WORK_INIT\n");
		/* go to next state */
		bbxmm_work->state = (modem_flash && !modem_pm)
			? BBXMM_WORK_INIT_FLASH_STEP1
			: (modem_flash && modem_pm)
			? BBXMM_WORK_INIT_FLASH_PM_STEP1
			: (!modem_flash && modem_pm)
			? BBXMM_WORK_INIT_FLASHLESS_PM_STEP1
			: BBXMM_WORK_UNINIT;
		pr_debug("Go to next state %d\n", bbxmm_work->state);
		queue_work(workqueue, work);
		break;
	case BBXMM_WORK_INIT_FLASH_STEP1:
		//pr_debug("BBXMM_WORK_INIT_FLASH_STEP1\n");
		/* register usb host controller */
		pr_debug("%s: register usb host controller\n", __func__);
		if (baseband_power_driver_data->hsic_register)
			baseband_power_driver_data->modem.xmm.hsic_device =
				baseband_power_driver_data->hsic_register();
		else
			pr_err("%s: hsic_register is missing\n", __func__);
		break;
	case BBXMM_WORK_INIT_FLASH_PM_STEP1:
		//pr_debug("BBXMM_WORK_INIT_FLASH_PM_STEP1\n");
		/* [modem ver >= 1130] start with IPC_HSIC_ACTIVE low */
		if (modem_ver >= XMM_MODEM_VER_1130) {
			pr_debug("%s: ver > 1130:"
				" ipc_hsic_active -> 0\n", __func__);
			gpio_set_value(baseband_power_driver_data->
				modem.xmm.ipc_hsic_active, 0);
		}
		/* reset / power on sequence */
		baseband_xmm_power_reset_on();
		/* set power status as on */
		power_onoff = 1;
		/* optional delay
		 * 0 = flashless
		 *   ==> causes next step to enumerate modem boot rom
		 *       (058b / 0041)
		 * some delay > boot rom timeout
		 *   ==> causes next step to enumerate modem software
		 *       (1519 / 0020)
		 *       (requires modem to be flash version, not flashless
		 *       version)
		 */
		if (enum_delay_ms)
			msleep(enum_delay_ms);
		/* register usb host controller */
		pr_debug("%s: register usb host controller\n", __func__);
		if (baseband_power_driver_data->hsic_register)
			baseband_power_driver_data->modem.xmm.hsic_device =
				baseband_power_driver_data->hsic_register();
		else
			pr_err("%s: hsic_register is missing\n", __func__);
		/* go to next state */
		bbxmm_work->state = (modem_ver < XMM_MODEM_VER_1130)
			? BBXMM_WORK_INIT_FLASH_PM_VER_LT_1130_STEP1
			: BBXMM_WORK_INIT_FLASH_PM_VER_GE_1130_STEP1;
		queue_work(workqueue, work);
		pr_debug("Go to next state %d\n", bbxmm_work->state);
		break;
	case BBXMM_WORK_INIT_FLASH_PM_VER_LT_1130_STEP1:
		pr_debug("BBXMM_WORK_INIT_FLASH_PM_VER_LT_1130_STEP1\n");
		break;
	case BBXMM_WORK_INIT_FLASH_PM_VER_GE_1130_STEP1:
		pr_debug("BBXMM_WORK_INIT_FLASH_PM_VER_GE_1130_STEP1\n");
		break;
	case BBXMM_WORK_INIT_FLASHLESS_PM_STEP1:
		//pr_debug("BBXMM_WORK_INIT_FLASHLESS_PM_STEP1\n");
		/* go to next state */
		bbxmm_work->state = (modem_ver < XMM_MODEM_VER_1130)
			? BBXMM_WORK_INIT_FLASHLESS_PM_VER_LT_1130_WAIT_IRQ
			: BBXMM_WORK_INIT_FLASHLESS_PM_VER_GE_1130_STEP1;
		queue_work(workqueue, work);
		break;
	case BBXMM_WORK_INIT_FLASHLESS_PM_VER_LT_1130_STEP1:
		pr_debug("BBXMM_WORK_INIT_FLASHLESS_PM_VER_LT_1130_STEP1\n");
		break;
	case BBXMM_WORK_INIT_FLASHLESS_PM_VER_GE_1130_STEP1:
		//pr_debug("BBXMM_WORK_INIT_FLASHLESS_PM_VER_GE_1130_STEP1\n");
		break;
	default:
		break;
	}

}

static void baseband_xmm_device_add_handler(struct usb_device *udev)
{
	struct usb_interface *intf = usb_ifnum_to_if(udev, 0);
	const struct usb_device_id *id;

	pr_info("%s \n",__func__);

	if (intf == NULL)
		return;

	id = usb_match_id(intf, xmm_pm_ids);

	if (id) {
		pr_debug("persist_enabled: %u\n", udev->persist_enabled);
		pr_info("Add device %d <%s %s>\n", udev->devnum,
			udev->manufacturer, udev->product);
		usbdev = udev;

		/* HTC: autosuspend_delay is not exist in k39 */
		//usbdev->autosuspend_delay = msecs_to_jiffies(autosuspend_delay);
		pm_runtime_set_autosuspend_delay(&udev->dev, DEFAULT_AUTOSUSPEND_DELAY);//for ICS 39kernel

		usb_enable_autosuspend(udev);
		pr_info("enable autosuspend\n");
	}
}

static void baseband_xmm_device_remove_handler(struct usb_device *udev)
{
	if (usbdev == udev) {
		pr_info("Remove device %d <%s %s>\n", udev->devnum,
			udev->manufacturer, udev->product);
		usbdev = 0;
	}

}

static int usb_xmm_notify(struct notifier_block *self, unsigned long action,
			void *blob)
{
	switch (action) {
	case USB_DEVICE_ADD:
		baseband_xmm_device_add_handler(blob);
		break;
	case USB_DEVICE_REMOVE:
		baseband_xmm_device_remove_handler(blob);
		break;
	}

	return NOTIFY_OK;
}


static struct notifier_block usb_xmm_nb = {
	.notifier_call = usb_xmm_notify,
};

static int baseband_xmm_power_pm_notifier_event(struct notifier_block *this,
					unsigned long event, void *ptr)
{
    struct baseband_power_platform_data *data = baseband_power_driver_data;
	unsigned long flags;

	if (!data)
		return NOTIFY_DONE;

	pr_debug("%s: event %ld\n", __func__, event);
	switch (event) {
	case PM_SUSPEND_PREPARE:
		pr_debug("%s : PM_SUSPEND_PREPARE\n", __func__);
		if (wake_lock_active(&wakelock)) {
			pr_info("%s: wakelock was active, aborting suspend\n",__func__);
			return NOTIFY_STOP;
		}
		spin_lock_irqsave(&xmm_lock, flags);
		if (wakeup_pending) {
			wakeup_pending = false;
			spin_unlock_irqrestore(&xmm_lock, flags);
			pr_info("%s : XMM busy : Abort system suspend\n",
				 __func__);
			return NOTIFY_STOP;
		}
		system_suspending = true;
		spin_unlock_irqrestore(&xmm_lock, flags);
		return NOTIFY_OK;
	case PM_POST_SUSPEND:
			pr_debug("%s : PM_POST_SUSPEND\n", __func__);
			spin_lock_irqsave(&xmm_lock, flags);
			system_suspending = false;
			if (wakeup_pending &&
			(baseband_xmm_powerstate == BBXMM_PS_L2)) {
			wakeup_pending = false;
			spin_unlock_irqrestore(&xmm_lock, flags);
			pr_info("%s : Service Pending CP wakeup\n",
				__func__);
			CP_initiated_L2toL0 = true;
			baseband_xmm_set_power_status
				(BBXMM_PS_L2TOL0);
			return NOTIFY_OK;
		}
		wakeup_pending = false;
		spin_unlock_irqrestore(&xmm_lock, flags);
		return NOTIFY_OK;
	}
	return NOTIFY_DONE;
}

static struct notifier_block baseband_xmm_power_pm_notifier = {
	.notifier_call = baseband_xmm_power_pm_notifier_event,
};

static int baseband_xmm_power_driver_probe(struct platform_device *device)
{
	struct baseband_power_platform_data *data
		= (struct baseband_power_platform_data *)
			device->dev.platform_data;
	struct device *dev = &device->dev;
	unsigned long flags;
	int err, ret=0;

	 pr_info(MODULE_NAME"%s 0316 hr_sleep and jiffe - CPU Freq. \n", __func__);
	 pr_info(MODULE_NAME"enum_delay_ms=%d\n", enum_delay_ms);
	 htcpcbid=htc_get_pcbid_info();
	 pr_info(MODULE_NAME"htcpcbid=%d\n", htcpcbid);

	/* check for platform data */
	if (!data)
		return -ENODEV;

	/* check if supported modem */
	if (data->baseband_type != BASEBAND_XMM) {
		pr_err("unsuppported modem\n");
		return -ENODEV;
	}

	/* save platform data */
	baseband_power_driver_data = data;

	/* create device file */
	err = device_create_file(dev, &dev_attr_xmm_onoff);
	if (err < 0) {
		pr_err("%s - device_create_file failed\n", __func__);
		return -ENODEV;
	}

	err = device_create_file(dev, &dev_attr_debug_gpio_dump);
	if (err < 0) {
		pr_err("%s - device_create_file failed\n", __func__);
		return -ENODEV;
	}

	/* HTC: create device file for host debugging */
	if (device_create_file(dev,&dev_attr_host_dbg))
		pr_info(MODULE_NAME"Warning: host attribute can't be created\n");

	/* init wake lock */
	wake_lock_init(&wakelock, WAKE_LOCK_SUSPEND, "baseband_xmm_power");

	/* init spin lock */
	spin_lock_init(&xmm_lock);
	/* request baseband gpio(s) */
	tegra_baseband_gpios[0].gpio = baseband_power_driver_data
		->modem.xmm.bb_rst;
	tegra_baseband_gpios[1].gpio = baseband_power_driver_data
		->modem.xmm.bb_on;
	tegra_baseband_gpios[2].gpio = baseband_power_driver_data
		->modem.xmm.ipc_bb_wake;
	tegra_baseband_gpios[3].gpio = baseband_power_driver_data
		->modem.xmm.ipc_ap_wake;
	tegra_baseband_gpios[4].gpio = baseband_power_driver_data
		->modem.xmm.ipc_hsic_active;
	tegra_baseband_gpios[5].gpio = baseband_power_driver_data
		->modem.xmm.ipc_hsic_sus_req;
	/*HTC request these gpio on probe only, config them when running power_on/off function*/
	err = gpio_request_only_array(tegra_baseband_gpios,
		ARRAY_SIZE(tegra_baseband_gpios));
	if (err < 0) {
		pr_err("%s - request gpio(s) failed\n", __func__);
		return -ENODEV;
	}
#if 1/*HTC*/
	//assing for usb
	tegra_baseband_gpios_power_off_modem[0].gpio = baseband_power_driver_data
		->modem.xmm.bb_rst;
	tegra_baseband_gpios_power_off_modem[1].gpio = baseband_power_driver_data
		->modem.xmm.bb_on;
	tegra_baseband_gpios_power_off_modem[2].gpio = baseband_power_driver_data
		->modem.xmm.ipc_bb_wake;
	tegra_baseband_gpios_power_off_modem[3].gpio = baseband_power_driver_data
		->modem.xmm.ipc_ap_wake;
	tegra_baseband_gpios_power_off_modem[4].gpio = baseband_power_driver_data
		->modem.xmm.ipc_hsic_active;
	tegra_baseband_gpios_power_off_modem[5].gpio = baseband_power_driver_data
		->modem.xmm.ipc_hsic_sus_req;
	//request UART
	pr_debug("%s request UART\n", __func__);
	err =gpio_request(TEGRA_GPIO_PJ7, "IMC_UART_TX");
	err =gpio_request(TEGRA_GPIO_PK7, "IMC_UART_RTS");
	err =gpio_request(TEGRA_GPIO_PB0  ,"IMC_UART_RX");
	err =gpio_request(TEGRA_GPIO_PB1, "IMC_UART_CTS");

	pr_debug("%s pull UART o d\n", __func__);
	//for power consumation
	//all the needed config put on power_on function
	pr_debug("%s config_gpio_for_power_off\n", __func__);
	err=config_gpio_for_power_off();
	if (err < 0) {
		pr_err("%s - config_gpio_for_power_off gpio(s)\n", __func__);
		return -ENODEV;
	}
#endif/*HTC*/



	/* request baseband irq(s) */
	if (modem_flash && modem_pm) {
		pr_info("%s: request_irq IPC_AP_WAKE_IRQ\n", __func__);
		ipc_ap_wake_state = IPC_AP_WAKE_UNINIT;
		err = request_threaded_irq(
			gpio_to_irq(data->modem.xmm.ipc_ap_wake),
			baseband_xmm_power_ipc_ap_wake_irq,
			NULL,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			"IPC_AP_WAKE_IRQ",
			NULL);
		if (err < 0) {
			pr_err("%s - request irq IPC_AP_WAKE_IRQ failed\n",
				__func__);
			return err;
		}
		ipc_ap_wake_state = IPC_AP_WAKE_IRQ_READY;
		if (modem_ver >= XMM_MODEM_VER_1130) {
			pr_debug("%s: ver > 1130: AP_WAKE_INIT1\n", __func__);
			/* ver 1130 or later starts in INIT1 state */
			ipc_ap_wake_state = IPC_AP_WAKE_INIT1;
		}
	}

	/* init work queue */
	workqueue = create_singlethread_workqueue
		("baseband_xmm_power_workqueue");
	if (!workqueue) {
		pr_err("cannot create workqueue\n");
		return -1;
	}
	 workqueue_susp = alloc_workqueue("baseband_xmm_power_autosusp", WQ_UNBOUND | WQ_HIGHPRI | WQ_NON_REENTRANT, 1);
	if (!workqueue_susp) {
	  pr_err("cannot create workqueue_susp\n");
	  return -1;
      }
	baseband_xmm_power_work = (struct baseband_xmm_power_work_t *)
		kmalloc(sizeof(struct baseband_xmm_power_work_t), GFP_KERNEL);
	if (!baseband_xmm_power_work) {
		pr_err("cannot allocate baseband_xmm_power_work\n");
		return -1;
	}
	INIT_WORK((struct work_struct *) baseband_xmm_power_work,
		baseband_xmm_power_work_func);
	baseband_xmm_power_work->state = BBXMM_WORK_INIT;
	queue_work(workqueue,
		(struct work_struct *) baseband_xmm_power_work);

	/* init work objects */
	INIT_WORK(&init1_work, baseband_xmm_power_init1_work);
	INIT_WORK(&init2_work, baseband_xmm_power_init2_work);
	INIT_WORK(&L2_resume_work, baseband_xmm_power_L2_resume_work);
	INIT_WORK(&work_shortsusp, baseband_xmm_power_shortsusp);
	INIT_WORK(&work_defaultsusp, baseband_xmm_power_defaultsusp);

	/* init state variables */
	register_hsic_device = true;
	CP_initiated_L2toL0 = false;
	baseband_xmm_powerstate = BBXMM_PS_UNINIT;
	spin_lock_irqsave(&xmm_lock, flags);
	wakeup_pending = false;
	system_suspending = false;
	spin_unlock_irqrestore(&xmm_lock, flags);

	usb_register_notify(&usb_xmm_nb);
	register_pm_notifier(&baseband_xmm_power_pm_notifier);



	/*HTC*/
	/*set Radio fatal Pin PN2 to OutPut Low*/
	ret=gpio_direction_output(TEGRA_GPIO_PN2,0);
	if (ret < 0)
		pr_err("%s: set Radio fatal Pin to Output error\n", __func__);
	/*set BB2AP_SUSPEND_REQ Pin (TEGRA_GPIO_PV0) to OutPut Low*/
	ret=gpio_direction_output(TEGRA_GPIO_PV0,0);
	if (ret < 0)
		pr_err("%s: set BB2AP_SUSPEND_REQ Pin to Output error\n", __func__);

	//Request SIM det to wakeup Source wahtever in flight mode on/off
	/*For SIM det*/
	pr_info("%s: request enable irq wake SIM det to wakeup source\n", __func__);
	ret = enable_irq_wake(gpio_to_irq(TEGRA_GPIO_PI5));
	if (ret < 0)
		pr_err("%s: enable_irq_wake error\n", __func__);

	pr_debug("%s }\n", __func__);
	return 0;
}

static int baseband_xmm_power_driver_remove(struct platform_device *device)
{
	struct baseband_power_platform_data *data
		= (struct baseband_power_platform_data *)
			device->dev.platform_data;
	struct device *dev = &device->dev;

	pr_debug("%s\n", __func__);

	/* check for platform data */
	if (!data)
		return 0;

	unregister_pm_notifier(&baseband_xmm_power_pm_notifier);
	usb_unregister_notify(&usb_xmm_nb);

	/* free work structure */
	kfree(baseband_xmm_power_work);
	baseband_xmm_power_work = (struct baseband_xmm_power_work_t *) 0;

	/* free baseband irq(s) */
	if (modem_flash && modem_pm) {
		free_irq(gpio_to_irq(baseband_power_driver_data
			->modem.xmm.ipc_ap_wake), NULL);
	}

	/* free baseband gpio(s) */
	gpio_free_array(tegra_baseband_gpios,
		ARRAY_SIZE(tegra_baseband_gpios));

	/* destroy wake lock */
	wake_lock_destroy(&wakelock);

	/* delete device file */
	device_remove_file(dev, &dev_attr_xmm_onoff);
	device_remove_file(dev, &dev_attr_debug_gpio_dump);

	/* HTC: delete device file */
	device_remove_file(dev, &dev_attr_host_dbg);

	 /* destroy wake lock */
	  destroy_workqueue(workqueue_susp);
	  destroy_workqueue(workqueue);

	/* unregister usb host controller */
	if (data->hsic_unregister)
		data->hsic_unregister(data->modem.xmm.hsic_device);
	else
		pr_err("%s: hsic_unregister is missing\n", __func__);

	return 0;
}

static int baseband_xmm_power_driver_handle_resume(
			struct baseband_power_platform_data *data)
{
	int value;
	unsigned long flags;
	unsigned long timeout;
	int delay = 10000; /* maxmum delay in msec */


	//pr_debug("%s\n", __func__);

	/* check for platform data */
	if (!data)
		return 0;

	/* check if modem is on */
	if (power_onoff == 0) {
		pr_info("%s - flight mode - nop\n", __func__);
		return 0;
	}

	modem_sleep_flag = false;
	spin_lock_irqsave(&xmm_lock, flags);
	/* Clear wakeup pending flag */
	wakeup_pending = false;
	spin_unlock_irqrestore(&xmm_lock, flags);

	/* L3->L0 */
	baseband_xmm_set_power_status(BBXMM_PS_L3TOL0);
	value = gpio_get_value(data->modem.xmm.ipc_ap_wake);
	if (value) {
		pr_info("AP L3 -> L0\n");
		pr_debug("waiting for host wakeup...\n");
		timeout = jiffies + msecs_to_jiffies(delay);
		/* wake bb */
		gpio_set_value(data->modem.xmm.ipc_bb_wake, 1);
		pr_debug("Set bb_wake high ->\n");
		do {
			udelay(100);
			value = gpio_get_value(data->modem.xmm.ipc_ap_wake);
			if (!value)
				break;
		} while (time_before(jiffies, timeout));
		if (!value) {
			pr_debug("gpio host wakeup low <-\n");
			pr_debug("%s enable short_autosuspend\n", __func__);
			short_autosuspend = true;
		}
		else
			pr_info("!!AP L3->L0 Failed\n");
	} else {
		pr_info("CP L3 -> L0\n");
	}
	reenable_autosuspend = true;
	
	return 0;

}

#ifdef CONFIG_PM
static int baseband_xmm_power_driver_suspend(struct device *dev)
{
	pr_debug("%s\n", __func__);
	return 0;
}

static int baseband_xmm_power_driver_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct baseband_power_platform_data *data
		= (struct baseband_power_platform_data *)
			pdev->dev.platform_data;

	pr_debug("%s\n", __func__);
	baseband_xmm_power_driver_handle_resume(data);

	return 0;
}

static int baseband_xmm_power_suspend_noirq(struct device *dev)
{
	unsigned long flags;

	pr_debug("%s\n", __func__);
	spin_lock_irqsave(&xmm_lock, flags);
	system_suspending = false;
	if (wakeup_pending) {
		wakeup_pending = false;
		spin_unlock_irqrestore(&xmm_lock, flags);
		pr_info("%s:**Abort Suspend: reason CP WAKEUP**\n", __func__);
		return -EBUSY;
	}
	spin_unlock_irqrestore(&xmm_lock, flags);
	return 0;
}

static int baseband_xmm_power_resume_noirq(struct device *dev)
{
	pr_debug("%s\n", __func__);
	return 0;
}

static const struct dev_pm_ops baseband_xmm_power_dev_pm_ops = {
	.suspend_noirq = baseband_xmm_power_suspend_noirq,
	.resume_noirq = baseband_xmm_power_resume_noirq,
	.suspend = baseband_xmm_power_driver_suspend,
	.resume = baseband_xmm_power_driver_resume,
};
#endif

static struct platform_driver baseband_power_driver = {
	.probe = baseband_xmm_power_driver_probe,
	.remove = baseband_xmm_power_driver_remove,
	.driver = {
		.name = "baseband_xmm_power",
#ifdef CONFIG_PM
	.pm   = &baseband_xmm_power_dev_pm_ops,
#endif
	},
};

static int __init baseband_xmm_power_init(void)
{
	/* HTC */
	host_dbg_flag = 0;
	pr_info("%s - host_dbg_flag=0x%x\n",
		__func__, host_dbg_flag);

	pr_debug(MODULE_NAME "%s 0x%lx\n", __func__, modem_ver);

	/* HTC: disable wakeup src */
	pr_debug(MODULE_NAME "%s - platfrom_set_flight_mode_onoff - on\n",
		__func__);
	/* HTC remove platfrom_set_flight_mode_onoff(true); for ENR */

	printk("%s:VP adding pm qos request \n", __func__);
	pm_qos_add_request(&modem_boost_cpu_freq_req, PM_QOS_CPU_FREQ_MIN, (s32)PM_QOS_CPU_FREQ_MIN_DEFAULT_VALUE);

	return platform_driver_register(&baseband_power_driver);
}

static void __exit baseband_xmm_power_exit(void)
{
	pr_debug("%s\n", __func__);
	platform_driver_unregister(&baseband_power_driver);
	pm_qos_remove_request(&modem_boost_cpu_freq_req);
}

module_init(baseband_xmm_power_init)
module_exit(baseband_xmm_power_exit)
