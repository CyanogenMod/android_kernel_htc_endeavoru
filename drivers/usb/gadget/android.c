/*
 * Gadget Driver for Android
 *
 * Copyright (C) 2008 Google, Inc.
 * Copyright (c) 2011-2012, Code Aurora Forum. All rights reserved.
 * Author: Mike Lockwood <lockwood@android.com>
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

/* #define DEBUG */
/* #define VERBOSE_DEBUG */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>

#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/utsname.h>
#include <linux/platform_device.h>
#include <linux/pm_qos_params.h>

#include <linux/usb/ch9.h>
#include <linux/usb/composite.h>
#include <linux/usb/gadget.h>
#include <linux/usb/android_composite.h>//htc
#include <linux/wakelock.h>
#include "gadget_chips.h"


#define PM_QOS_USB_TP_CPU_FREQ 910
static struct pm_qos_request_list pm_qos_req_tp;

enum {
	OS_NOT_YET,
	OS_MAC,
	OS_LINUX,
	OS_WINDOWS,
};

static int mac_mtp_mode;
static int os_type;

/*
 * Kbuild is not very cooperative with respect to linking separately
 * compiled library objects into one module.  So for now we won't use
 * separate compilation ... ensuring init/exit sections work to shrink
 * the runtime footprint, and giving us at least some parts of what
 * a "gcc --combine ... part1.c part2.c part3.c ... " build would.
 */
#include "usbstring.c"
#include "config.c"
#include "epautoconf.c"
#include "composite.c"


#include "f_diag.c"
#include "f_rmnet.c"
#include "f_mass_storage.c"
#include "u_serial.c"
#include "f_serial.c"
#include "u_ctrl_hsic.c"
#include "u_data_hsic.c"

#ifdef CONFIG_USB_ANDROID_ACM
#include "f_acm.c"
#endif
#include "f_adb.c"
#ifdef CONFIG_USB_ANDROID_MTP
#include "f_mtp.c"
#endif
#include "f_accessory.c"
#define USB_ETH_RNDIS y
#include "f_rndis.c"
#include "rndis.c"

#ifdef CONFIG_USB_ANDROID_ECM
#include "f_ecm.c"
#endif
#ifdef CONFIG_USB_ETH_PASS_FW
#include "passthru.c"
#endif
#include "u_ether.c"
#ifdef CONFIG_USB_ANDROID_USBNET
#include "f_usbnet.c"
#endif

#include <linux/usb/htc_info.h>

MODULE_AUTHOR("Mike Lockwood");
MODULE_DESCRIPTION("Android Composite USB Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");

static const char longname[] = "Gadget Android";

/* Default vendor and product IDs, overridden by userspace */
#define VENDOR_ID		0x18D1
#define PRODUCT_ID		0x0001

static atomic_t connect2pc = ATOMIC_INIT(false);

struct android_usb_function {
	char *name;
	void *config;

	struct device *dev;
	char *dev_name;
	struct device_attribute **attributes;

	/* for android_dev.enabled_functions */
	struct list_head enabled_list;

	/* Optional: initialization during gadget bind */
	int (*init)(struct android_usb_function *, struct usb_composite_dev *);
	/* Optional: cleanup during gadget unbind */
	void (*cleanup)(struct android_usb_function *);

	int (*bind_config)(struct android_usb_function *, struct usb_configuration *);

	/* Optional: called when the configuration is removed */
	void (*unbind_config)(struct android_usb_function *, struct usb_configuration *);
	/* Optional: handle ctrl requests before the device is configured */
	int (*ctrlrequest)(struct android_usb_function *,
					struct usb_composite_dev *,
					const struct usb_ctrlrequest *);
};

struct android_dev {
	struct android_usb_function **functions;
	struct list_head enabled_functions;
	struct usb_composite_dev *cdev;
	struct device *dev;
	struct android_usb_platform_data *pdata;//htc

	bool enabled;
	struct mutex mutex;
	bool connected;
	bool sw_connected;
	char pm_qos[5];
	struct pm_qos_request_list pm_qos_req_dma;
	struct work_struct work;
//++htc++
	struct delayed_work init_work;
	/* waiting for enabling functions */
	struct list_head function_list;

	int num_products;
	struct android_usb_product *products;
	int num_functions;
	char **in_house_functions;

	int product_id;
	void (*enable_fast_charge)(bool enable);
	bool RndisDisableMPDecision;
	int (*match)(int product_id, int intrsharing);
	bool bEnablePerfLock;
	int autobot_mode;
};

static struct class *android_class;
static struct android_dev *_android_dev;
static int android_bind_config(struct usb_configuration *c);
static void android_unbind_config(struct usb_configuration *c);

#ifdef CONFIG_USB_ANDROID_PROJECTOR
#include "f_projector.c"
#endif

/* string IDs are assigned dynamically */
#define STRING_MANUFACTURER_IDX		0
#define STRING_PRODUCT_IDX		1
#define STRING_SERIAL_IDX		2

static char manufacturer_string[256];
static char product_string[256];
static char serial_string[256];

/* String Table */
static struct usb_string strings_dev[] = {
	[STRING_MANUFACTURER_IDX].s = manufacturer_string,
	[STRING_PRODUCT_IDX].s = product_string,
	[STRING_SERIAL_IDX].s = serial_string,
	{  }			/* end of list */
};

static struct usb_gadget_strings stringtab_dev = {
	.language	= 0x0409,	/* en-us */
	.strings	= strings_dev,
};

static struct usb_gadget_strings *dev_strings[] = {
	&stringtab_dev,
	NULL,
};

static struct usb_device_descriptor device_desc = {
	.bLength              = sizeof(device_desc),
	.bDescriptorType      = USB_DT_DEVICE,
	.bcdUSB               = __constant_cpu_to_le16(0x0200),
	.bDeviceClass         = USB_CLASS_PER_INTERFACE,
	.idVendor             = __constant_cpu_to_le16(VENDOR_ID),
	.idProduct            = __constant_cpu_to_le16(PRODUCT_ID),
	.bcdDevice            = __constant_cpu_to_le16(0xffff),
	.bNumConfigurations   = 1,
};

static struct usb_otg_descriptor otg_descriptor = {
	.bLength =		sizeof otg_descriptor,
	.bDescriptorType =	USB_DT_OTG,
	.bmAttributes =		USB_OTG_SRP | USB_OTG_HNP,
/*	.bcdOTG               = __constant_cpu_to_le16(0x0200), */ /* TODO */
};

static const struct usb_descriptor_header *otg_desc[] = {
	(struct usb_descriptor_header *) &otg_descriptor,
	NULL,
};

static struct usb_configuration android_config_driver = {
	.label		= "android",
	.unbind		= android_unbind_config,
	.bConfigurationValue = 1,
	.bmAttributes	= USB_CONFIG_ATT_ONE | USB_CONFIG_ATT_SELFPOWER,
	.bMaxPower	= 0xFA, /* 500ma */
};

enum android_device_state {
       USB_DISCONNECTED,
       USB_CONNECTED,
       USB_CONFIGURED,
};

void network_pm_qos_update_latency(int vote)
{
	static int last_vote = -1;

	if (vote == last_vote)
		return;

	if (vote) {
		pm_qos_update_request(&pm_qos_req_tp, (s32)PM_QOS_USB_TP_CPU_FREQ * 1000);
	} else {
		pm_qos_update_request(&pm_qos_req_tp, (s32)PM_QOS_CPU_FREQ_MIN_DEFAULT_VALUE);
	}
	last_vote = vote;
}
static void android_pm_qos_update_latency(struct android_dev *dev, int vote)
{
	struct android_usb_platform_data *pdata = dev->pdata;
	u32 swfi_latency = 0;
	static int last_vote = -1;

	if (!pdata || vote == last_vote)
		return;

	swfi_latency = pdata->swfi_latency + 1;
	if (vote)
		pm_qos_update_request(&dev->pm_qos_req_dma,
				swfi_latency);
	else
		pm_qos_update_request(&dev->pm_qos_req_dma,
				PM_QOS_DEFAULT_VALUE);
	last_vote = vote;
}

static void android_work(struct work_struct *data)
{
	struct android_dev *dev = container_of(data, struct android_dev, work);
	struct usb_composite_dev *cdev = dev->cdev;
	char *disconnected[2] = { "USB_STATE=DISCONNECTED", NULL };
	char *connected[2]    = { "USB_STATE=CONNECTED", NULL };
	char *configured[2]   = { "USB_STATE=CONFIGURED", NULL };
	char **uevent_envp = NULL;
	static enum android_device_state last_uevent, next_state;
	unsigned long flags;
	int ctl_flag = -1;

	spin_lock_irqsave(&cdev->lock, flags);

		if (cdev->config) {
			uevent_envp = configured;
			next_state = USB_CONFIGURED;
		} else if (dev->connected != dev->sw_connected) {
			uevent_envp = dev->connected ? connected : disconnected ;
			next_state = dev->connected ? USB_CONNECTED : USB_DISCONNECTED;
			if (dev->connected && strncmp(dev->pm_qos, "low", 3))
				ctl_flag = 1;
			else if (!dev->connected || !strncmp(dev->pm_qos, "low", 3))
				ctl_flag = 0;
		}
/*
	if (cdev->config)
		uevent_envp = configured;
	else if (dev->connected != dev->sw_connected)
		uevent_envp = dev->connected ? connected : disconnected;
*/
	dev->sw_connected = dev->connected;
	spin_unlock_irqrestore(&cdev->lock, flags);
/*	if (ctl_flag == 1)
		android_pm_qos_update_latency(dev, 1);
	else if (ctl_flag == 0)
		android_pm_qos_update_latency(dev, 0);
*/
	if (uevent_envp) {
		/*
		* Some userspace modules, e.g. MTP, work correctly only if
		* CONFIGURED uevent is preceded by DISCONNECT uevent.
		* Check if we missed sending out a DISCONNECT uevent. This can
		* happen if host PC resets and configures device really quick.
		*/
		if (((uevent_envp == connected) &&
				(last_uevent != USB_DISCONNECTED)) ||
				((uevent_envp == configured) &&
				(last_uevent == USB_CONFIGURED))) {
			pr_info("%s: sent missed DISCONNECT event\n", __func__);
			kobject_uevent_env(&dev->dev->kobj, KOBJ_CHANGE, disconnected);
			msleep(20);
		}
		/*
		* Before sending out CONFIGURED uevent give function drivers
		* a chance to wakeup userspace threads and notify disconnect
		*/
		if (uevent_envp == configured)
			msleep(50);

		kobject_uevent_env(&dev->dev->kobj, KOBJ_CHANGE, uevent_envp);
		last_uevent = next_state;
		pr_info("%s", uevent_envp[0]);
		/* will implement perf lock here*/
	} else {
		pr_info("%s: did not send uevent (%d %d %p)\n", __func__,
			 dev->connected, dev->sw_connected, cdev->config);
	}

	if (atomic_read(&connect2pc) != dev->sw_connected) {
		atomic_set(&connect2pc, dev->sw_connected);
		switch_set_state(&cdev->sw_connect2pc, atomic_read(&connect2pc) ? 1 : 0);
		USB_INFO("set usb_connect2pc = %d\n", connect2pc);
		if (!atomic_read(&connect2pc)) {
			USB_INFO("%s: OS_NOT_YET\n", __func__);
			os_type = OS_NOT_YET;
			mtp_update_mode(0);
			fsg_update_mode(0);
		}
	}

}

bool read_connect2pc(void)
{
	return atomic_read(&connect2pc);
}
EXPORT_SYMBOL(read_connect2pc);

/*-------------------------------------------------------------------------*/
/* Supported functions initialization */
static ssize_t func_en_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct android_usb_function *func = dev_get_drvdata(dev);
	struct android_usb_function *f;
	int ebl = 0;

	list_for_each_entry(f, &_android_dev->enabled_functions, enabled_list) {
		if (!strcmp(func->name, f->name)) {
			ebl = 1;
			break;
		}
	}
	return sprintf(buf, "%d", ebl);
}

static ssize_t func_en_store(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct android_usb_function *func = dev_get_drvdata(dev);
	struct android_usb_function *f;
	int ebl = 0;
	int value;

	sscanf(buf, "%d", &value);
	list_for_each_entry(f, &_android_dev->enabled_functions, enabled_list) {
		if (!strcmp(func->name, f->name)) {
			ebl = 1;
			break;
		}
	}
	if (!!value == ebl) {
		pr_info("%s function is already %s\n", func->name
			, ebl ? "enable" : "disable");
		return size;
	}

	if (value)
		htc_usb_enable_function(func->name, 1);
	else
		htc_usb_enable_function(func->name, 0);

	return size;
}
static DEVICE_ATTR(on, S_IRUGO | S_IWUSR | S_IWGRP, func_en_show, func_en_store);


#if defined(CONFIG_USB_ANDROID_RMNET_SMD)
/* RMNET_SMD */
static int rmnet_smd_function_bind_config(struct android_usb_function *f,
					  struct usb_configuration *c)
{
	return rmnet_smd_bind_config(c);
}

static struct android_usb_function rmnet_smd_function = {
	.name		= "rmnet_smd",
	.bind_config	= rmnet_smd_function_bind_config,
};
#elif defined(CONFIG_USB_ANDROID_RMNET_SDIO)

/* RMNET_SDIO */
static int rmnet_sdio_function_bind_config(struct android_usb_function *f,
					  struct usb_configuration *c)
{
	return rmnet_sdio_function_add(c);
}

static struct android_usb_function rmnet_sdio_function = {
	.name		= "rmnet_sdio",
	.bind_config	= rmnet_sdio_function_bind_config,
};

#elif defined(CONFIG_USB_ANDROID_RMNET_SMD_SDIO)
/* RMNET_SMD_SDIO */
static int rmnet_smd_sdio_function_init(struct android_usb_function *f,
				 struct usb_composite_dev *cdev)
{
	return rmnet_smd_sdio_init();
}

static void rmnet_smd_sdio_function_cleanup(struct android_usb_function *f)
{
	rmnet_smd_sdio_cleanup();
}

static int rmnet_smd_sdio_bind_config(struct android_usb_function *f,
					  struct usb_configuration *c)
{
	return rmnet_smd_sdio_function_add(c);
}

static struct device_attribute *rmnet_smd_sdio_attributes[] = {
					&dev_attr_transport, NULL };

static struct android_usb_function rmnet_smd_sdio_function = {
	.name		= "rmnet_smd_sdio",
	.init		= rmnet_smd_sdio_function_init,
	.cleanup	= rmnet_smd_sdio_function_cleanup,
	.bind_config	= rmnet_smd_sdio_bind_config,
	.attributes	= rmnet_smd_sdio_attributes,
};
#endif

/*rmnet transport string format(per port):"ctrl0,data0,ctrl1,data1..." */
#define MAX_XPORT_STR_LEN 50
static char rmnet_transports[MAX_XPORT_STR_LEN];

static void rmnet_function_cleanup(struct android_usb_function *f)
{
	frmnet_cleanup();
}

static int rmnet_function_bind_config(struct android_usb_function *f,
					 struct usb_configuration *c)
{
	int i;
	int err = 0;
	char *ctrl_name;
	char *data_name;
	char buf[MAX_XPORT_STR_LEN], *b;
	static int rmnet_initialized, ports;

	if (!rmnet_initialized) {
		rmnet_initialized = 1;
		strlcpy(buf, rmnet_transports, sizeof(buf));
		b = strim(buf);
		while (b) {
			ctrl_name = strsep(&b, ",");
			data_name = strsep(&b, ",");
			if (ctrl_name && data_name) {
				err = frmnet_init_port(ctrl_name, data_name);
				if (err) {
					pr_err("rmnet: Cannot open ctrl port:"
						"'%s' data port:'%s'\n",
						ctrl_name, data_name);
					goto out;
				}
				ports++;
			}
		}

		err = rmnet_gport_setup();
		if (err) {
			pr_err("rmnet: Cannot setup transports");
			goto out;
		}
	}

	for (i = 0; i < ports; i++) {
		err = frmnet_bind_config(c, i);
		if (err) {
			pr_err("Could not bind rmnet%u config\n", i);
			break;
		}
	}
out:
	return err;
}

static ssize_t rmnet_transports_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", rmnet_transports);
}

static ssize_t rmnet_transports_store(
		struct device *device, struct device_attribute *attr,
		const char *buff, size_t size)
{
	strlcpy(rmnet_transports, buff, sizeof(rmnet_transports));

	return size;
}

static struct device_attribute dev_attr_rmnet_transports =
					__ATTR(transports, S_IRUGO | S_IWUSR,
						rmnet_transports_show,
						rmnet_transports_store);
static struct device_attribute *rmnet_function_attributes[] = {
					&dev_attr_rmnet_transports,
					NULL };

static struct android_usb_function rmnet_function = {
	.name		= "rmnet",
	.cleanup	= rmnet_function_cleanup,
	.bind_config	= rmnet_function_bind_config,
	.attributes	= rmnet_function_attributes,
};

/* DIAG */
static char diag_clients[32];	    /*enabled DIAG clients- "diag[,diag_mdm]" */
static ssize_t clients_store(
		struct device *device, struct device_attribute *attr,
		const char *buff, size_t size)
{
	strlcpy(diag_clients, buff, sizeof(diag_clients));

	return size;
}

static DEVICE_ATTR(clients, S_IWUSR, NULL, clients_store);
static struct device_attribute *diag_function_attributes[] =
					 { &dev_attr_clients, NULL };

static int diag_function_init(struct android_usb_function *f,
				 struct usb_composite_dev *cdev)
{
	return diag_setup();
}

static void diag_function_cleanup(struct android_usb_function *f)
{
	diag_cleanup();
}

static int diag_function_bind_config(struct android_usb_function *f,
					struct usb_configuration *c)
{
#if 1
	int err;
	int (*notify)(uint32_t, const char *);

	notify = _android_dev->pdata->update_pid_and_serial_num;

	err = diag_function_add(c, DIAG_MDM, notify);/* Note:DIAG_LEGACY->DIAG_MDM */
	if (err)
		pr_err("diag: Cannot open channel '%s'", DIAG_MDM);/* Note:DIAG_LEGACY->DIAG_MDM */

#else
	char *name;
	char buf[32], *b;
	int once = 0, err = -1;
	int (*notify)(uint32_t, const char *);

	strlcpy(buf, diag_clients, sizeof(buf));
	b = strim(buf);

	while (b) {
		notify = NULL;
		name = strsep(&b, ",");
		/* Allow only first diag channel to update pid and serial no */
		if (_android_dev->pdata && !once++)
			notify = _android_dev->pdata->update_pid_and_serial_num;

		if (name) {
			if (strcmp(name, f->name) != 0)
				continue;
			err = diag_function_add(c, name, notify);
			if (err)
				pr_err("diag: Cannot open channel '%s'", name);
		}
	}
#endif
	return err;
}

static struct android_usb_function diag_function = {
	.name		= DIAG_LEGACY,
	.init		= diag_function_init,
	.cleanup	= diag_function_cleanup,
	.bind_config	= diag_function_bind_config,
	.attributes	= diag_function_attributes,
};


static int diag_mdm_function_bind_config(struct android_usb_function *f,
					struct usb_configuration *c)
{
	int err;
	int (*notify)(uint32_t, const char *);

	notify = NULL;

	err = diag_function_add(c, DIAG_MDM, notify);
	if (err)
		pr_err("diag: Cannot open channel '%s'",DIAG_MDM);

	return 0;
}

static struct android_usb_function diag_mdm_function = {
	.name		= DIAG_MDM,
	.bind_config	= diag_mdm_function_bind_config,
};


/* SERIAL, MODEM */
static char serial_transports[64];	/*enabled FSERIAL ports - "tty[,sdio]"*/

/* Not allow user space to modify serial combination */
#if 0
static ssize_t serial_transports_store(
		struct device *device, struct device_attribute *attr,
		const char *buff, size_t size)
{
	strlcpy(serial_transports, buff, sizeof(serial_transports));

	return size;
}

static DEVICE_ATTR(transports, S_IWUSR, NULL, serial_transports_store);
static struct device_attribute *serial_function_attributes[] =
					 { &dev_attr_transports, NULL };
#endif
static int serial_driver_initial(struct usb_configuration *c)
{
	char *name, *str[2];
	char buf[80], *b;
	int err = -1;
	static int serial_initialized = 0, ports = 0;

	if (serial_initialized) {
		pr_info("%s: already initial\n", __func__);
		return ports;
	}
	serial_initialized = 1;

	if (_android_dev->pdata->fserial_init_string)
		strcpy(serial_transports, _android_dev->pdata->fserial_init_string);
	else
		strcpy(serial_transports, "HSIC:modem,tty,tty,tty:serial");

	strncpy(buf, serial_transports, sizeof(buf));
	buf[79] = 0;
	pr_info("%s: init string: %s\n", __func__, buf);

	b = strim(buf);

	while (b) {
		str[0] = str[1] = 0;
		name = strsep(&b, ",");
		if (name) {
			str[0] = strsep(&name, ":");
			if (str[0])
				str[1] = strsep(&name, ":");
		}
		err = gserial_init_port(ports, str[0], str[1]);
		if (err) {
			pr_err("serial: Cannot open port '%s'\n", str[0]);
			goto out;
		}
		ports++;
	}

	err = gport_setup(c);
	if (err) {
		pr_err("serial: Cannot setup transports");
		goto out;
	}
	return ports;

out:
	return err;
}

/* Serial */

static void serial_function_cleanup(struct android_usb_function *f)
{
	gserial_cleanup();
}

static int serial_function_bind_config(struct android_usb_function *f,
					struct usb_configuration *c)
{
	int err = -1;
	int i, ports, car_mode = _android_dev->autobot_mode;

	ports = serial_driver_initial(c);
	if (ports < 0)
		goto out;
	for (i = 0; i < ports; i++) {
		if ((gserial_ports[i].func_type == USB_FSER_FUNC_SERIAL) ||
			(car_mode && gserial_ports[i].func_type == USB_FSER_FUNC_AUTOBOT)) {
			err = gser_bind_config(c, i);
			if (err) {
				pr_err("serial: bind_config failed for port %d", i);
				goto out;
			}
		}
	}
out:
	return err;
}

static struct android_usb_function serial_function = {
	.name		= "serial",
	.cleanup	= serial_function_cleanup,
	.bind_config	= serial_function_bind_config,
};

/* Modem */
static int modem_function_bind_config(struct android_usb_function *f,
					struct usb_configuration *c)
{
	int err = -1;
	int i, ports;

	ports = serial_driver_initial(c);
	if (ports < 0)
		goto out;


	for (i = 0; i < ports; i++) {
		if (gserial_ports[i].func_type == USB_FSER_FUNC_MODEM) {
			err = gser_bind_config(c, i);
			if (err) {
				pr_err("serial: bind_config failed for port %d", i);
				goto out;
			}
		}
	}

out:
	return err;
}

static struct android_usb_function modem_function = {
	.name		= "modem",
	.cleanup		= serial_function_cleanup,
	.bind_config	= modem_function_bind_config,
};
#ifdef CONFIG_USB_ANDROID_ACM
/* ACM */
static char acm_transports[32];	/*enabled ACM ports - "tty[,sdio]"*/
static ssize_t acm_transports_store(
		struct device *device, struct device_attribute *attr,
		const char *buff, size_t size)
{
	strlcpy(acm_transports, buff, sizeof(acm_transports));

	return size;
}

static DEVICE_ATTR(acm_transports, S_IWUSR, NULL, acm_transports_store);
static struct device_attribute *acm_function_attributes[] = {
		&dev_attr_acm_transports, NULL };

static void acm_function_cleanup(struct android_usb_function *f)
{
	gserial_cleanup();
}

static int acm_function_bind_config(struct android_usb_function *f,
					struct usb_configuration *c)
{
	int err = -1;
	int i, ports;

	ports = serial_driver_initial(c);
	if (ports < 0)
		goto out;
	for (i = 0; i < ports; i++) {
		if (gserial_ports[i].func_type == USB_FSER_FUNC_ACM) {
			err = acm_bind_config(c, i);
			if (err) {
				pr_err("acm: bind_config failed for port %d", i);
				goto out;
			}
		}
	}
out:
	return err;
#if 0
	char *name;
	char buf[32], *b;
	int err = -1, i;
	static int acm_initialized, ports;

	if (acm_initialized)
		goto bind_config;

	acm_initialized = 1;
	strlcpy(buf, acm_transports, sizeof(buf));
	b = strim(buf);

	while (b) {
		name = strsep(&b, ",");

		if (name) {
			err = acm_init_port(ports, name);
			if (err) {
				pr_err("acm: Cannot open port '%s'", name);
				goto out;
			}
			ports++;
		}
	}
	err = acm_port_setup(c);
	if (err) {
		pr_err("acm: Cannot setup transports");
		goto out;
	}

bind_config:
	for (i = 0; i < ports; i++) {
		err = acm_bind_config(c, i);
		if (err) {
			pr_err("acm: bind_config failed for port %d", i);
			goto out;
		}
	}

out:
	return err;
#endif
}
static struct android_usb_function acm_function = {
	.name		= "acm",
	.cleanup	= acm_function_cleanup,
	.bind_config	= acm_function_bind_config,
	.attributes	= acm_function_attributes,
};
#endif	/* CONFIG_USB_ANDROID_ACM */

/* ADB */
static int adb_function_init(struct android_usb_function *f, struct usb_composite_dev *cdev)
{
	return adb_setup();
}

static void adb_function_cleanup(struct android_usb_function *f)
{
	adb_cleanup();
}

static int adb_function_bind_config(struct android_usb_function *f, struct usb_configuration *c)
{
	return adb_bind_config(c);
}

static struct android_usb_function adb_function = {
	.name		= "adb",
	.init		= adb_function_init,
	.cleanup	= adb_function_cleanup,
	.bind_config	= adb_function_bind_config,
};
#if 0
/* CCID */
static int ccid_function_init(struct android_usb_function *f,
					struct usb_composite_dev *cdev)
{
	return ccid_setup();
}

static void ccid_function_cleanup(struct android_usb_function *f)
{
	ccid_cleanup();
}

static int ccid_function_bind_config(struct android_usb_function *f,
						struct usb_configuration *c)
{
	return ccid_bind_config(c);
}

static struct android_usb_function ccid_function = {
	.name		= "ccid",
	.init		= ccid_function_init,
	.cleanup	= ccid_function_cleanup,
	.bind_config	= ccid_function_bind_config,
};
#endif
static int mtp_function_init(struct android_usb_function *f, struct usb_composite_dev *cdev)
{
	struct android_dev *dev = _android_dev;
	int ret;
	ret = mtp_setup();
	mtp_setup_perflock(dev->pdata->mtp_perf_lock_on?true:false);
	return ret;
}

static void mtp_function_cleanup(struct android_usb_function *f)
{
	mtp_cleanup();
}

static int mtp_function_bind_config(struct android_usb_function *f, struct usb_configuration *c)
{
	return mtp_bind_config(c, false);
}

static int ptp_function_init(struct android_usb_function *f, struct usb_composite_dev *cdev)
{
	/* nothing to do - initialization is handled by mtp_function_init */
	return 0;
}

static void ptp_function_cleanup(struct android_usb_function *f)
{
	/* nothing to do - cleanup is handled by mtp_function_cleanup */
}

static int ptp_function_bind_config(struct android_usb_function *f, struct usb_configuration *c)
{
	return mtp_bind_config(c, true);
}

static int mtp_function_ctrlrequest(struct android_usb_function *f,
						struct usb_composite_dev *cdev,
						const struct usb_ctrlrequest *c)
{
	return mtp_ctrlrequest(cdev, c);
}

static ssize_t mtp_debug_level_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", htc_mtp_performance_debug);
}

static ssize_t mtp_debug_level_store(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	if (buf[0] >= '0' && buf[0] <= '9' && buf[1] == '\n')
		htc_mtp_performance_debug = buf[0] - '0';
	return size;
}

static ssize_t mtp_iobusy_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	/* We will hold perf lock during I/O jobs */
	return sprintf(buf, "%d\n", _mtp_dev->mtp_perf_lock_on?1:0);
}
static ssize_t mtp_open_state_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", htc_mtp_open_state);
}

static DEVICE_ATTR(mtp_debug_level, S_IRUGO | S_IWUSR, mtp_debug_level_show,
						    mtp_debug_level_store);
static DEVICE_ATTR(iobusy, S_IRUGO, mtp_iobusy_show, NULL);
static DEVICE_ATTR(mtp_open_state, S_IRUGO, mtp_open_state_show, NULL);
static struct device_attribute *mtp_function_attributes[] = {
	&dev_attr_mtp_debug_level,
	&dev_attr_iobusy,
	&dev_attr_mtp_open_state,
	NULL
};

static struct android_usb_function mtp_function = {
	.name		= "mtp",
	.init		= mtp_function_init,
	.cleanup	= mtp_function_cleanup,
	.bind_config	= mtp_function_bind_config,
	.ctrlrequest	= mtp_function_ctrlrequest,
	.attributes 	= mtp_function_attributes,
};

/* PTP function is same as MTP with slightly different interface descriptor */
static struct android_usb_function ptp_function = {
	.name		= "ptp",
	.init		= ptp_function_init,
	.cleanup	= ptp_function_cleanup,
	.bind_config	= ptp_function_bind_config,
};

/* ECM */
struct ecm_function_config {
	u8      ethaddr[ETH_ALEN];
};

static int ecm_function_init(struct android_usb_function *f, struct usb_composite_dev *cdev)
{
	struct ecm_function_config *ecm;
	f->config = kzalloc(sizeof(struct ecm_function_config), GFP_KERNEL);
	if (!f->config)
		return -ENOMEM;

	ecm = f->config;
	return 0;
}

static void ecm_function_cleanup(struct android_usb_function *f)
{
	kfree(f->config);
	f->config = NULL;
}

static int ecm_function_bind_config(struct android_usb_function *f,
					struct usb_configuration *c)
{
	int ret;
	struct ecm_function_config *ecm = f->config;

	if (!ecm) {
		pr_err("%s: ecm_pdata\n", __func__);
		return -1;
	}


	pr_info("%s MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", __func__,
		ecm->ethaddr[0], ecm->ethaddr[1], ecm->ethaddr[2],
		ecm->ethaddr[3], ecm->ethaddr[4], ecm->ethaddr[5]);

	ret = gether_setup_name(c->cdev->gadget, ecm->ethaddr, "usb");
	if (ret) {
		pr_err("%s: gether_setup failed\n", __func__);
		return ret;
	}


	return ecm_bind_config(c, ecm->ethaddr);
}

static void ecm_function_unbind_config(struct android_usb_function *f,
						struct usb_configuration *c)
{
	gether_cleanup();
}

static ssize_t ecm_ethaddr_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct ecm_function_config *ecm = f->config;
	return sprintf(buf, "%02x:%02x:%02x:%02x:%02x:%02x\n",
		ecm->ethaddr[0], ecm->ethaddr[1], ecm->ethaddr[2],
		ecm->ethaddr[3], ecm->ethaddr[4], ecm->ethaddr[5]);
}

static ssize_t ecm_ethaddr_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct ecm_function_config *ecm = f->config;

	if (sscanf(buf, "%02x:%02x:%02x:%02x:%02x:%02x\n",
		    (int *)&ecm->ethaddr[0], (int *)&ecm->ethaddr[1],
		    (int *)&ecm->ethaddr[2], (int *)&ecm->ethaddr[3],
		    (int *)&ecm->ethaddr[4], (int *)&ecm->ethaddr[5]) == 6)
		return size;
	return -EINVAL;
}

static DEVICE_ATTR(ecm_ethaddr, S_IRUGO | S_IWUSR, ecm_ethaddr_show,
					       ecm_ethaddr_store);

static struct device_attribute *ecm_function_attributes[] = {
	&dev_attr_ecm_ethaddr,
	NULL
};

static struct android_usb_function ecm_function = {
	.name		= "cdc_ethernet",
	.init		= ecm_function_init,
	.cleanup	= ecm_function_cleanup,
	.bind_config	= ecm_function_bind_config,
	.unbind_config	= ecm_function_unbind_config,
	.attributes	= ecm_function_attributes,
};
struct rndis_function_config {
	u8      ethaddr[ETH_ALEN];
	u32     vendorID;
	char	manufacturer[256];
	bool	wceis;
};

static int rndis_function_init(struct android_usb_function *f, struct usb_composite_dev *cdev)
{
	struct rndis_function_config *rndis;
	struct android_dev *dev = _android_dev;

	f->config = kzalloc(sizeof(struct rndis_function_config), GFP_KERNEL);
	if (!f->config)
		return -ENOMEM;

	rndis = f->config;

	strncpy(rndis->manufacturer, dev->pdata->manufacturer_name, sizeof(rndis->manufacturer));
	rndis->vendorID = dev->pdata->vendor_id;

	return 0;
}

static void rndis_function_cleanup(struct android_usb_function *f)
{
	kfree(f->config);
	f->config = NULL;
}

static int rndis_function_bind_config(struct android_usb_function *f,
					struct usb_configuration *c)
{
	int ret;
	struct rndis_function_config *rndis = f->config;

	if (!rndis) {
		pr_err("%s: rndis_pdata\n", __func__);
		return -1;
	}

	pr_info("%s MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", __func__,
		rndis->ethaddr[0], rndis->ethaddr[1], rndis->ethaddr[2],
		rndis->ethaddr[3], rndis->ethaddr[4], rndis->ethaddr[5]);

	ret = gether_setup_name(c->cdev->gadget, rndis->ethaddr, "usb");
	if (ret) {
		pr_err("%s: gether_setup failed\n", __func__);
		return ret;
	}

	if (rndis->wceis) {
		/* "Wireless" RNDIS; auto-detected by Windows */
		rndis_iad_descriptor.bFunctionClass =
						USB_CLASS_WIRELESS_CONTROLLER;
		rndis_iad_descriptor.bFunctionSubClass = 0x01;
		rndis_iad_descriptor.bFunctionProtocol = 0x03;
		rndis_control_intf.bInterfaceClass =
						USB_CLASS_WIRELESS_CONTROLLER;
		rndis_control_intf.bInterfaceSubClass =	 0x01;
		rndis_control_intf.bInterfaceProtocol =	 0x03;
	}

	return rndis_bind_config(c, rndis->ethaddr, rndis->vendorID,
				    rndis->manufacturer);
}

static void rndis_function_unbind_config(struct android_usb_function *f,
						struct usb_configuration *c)
{
	gether_cleanup();
}

static ssize_t rndis_manufacturer_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *config = f->config;
	return snprintf(buf, PAGE_SIZE, "%s\n", config->manufacturer);
}

static ssize_t rndis_manufacturer_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *config = f->config;

	if (size >= sizeof(config->manufacturer))
		return -EINVAL;
	if (sscanf(buf, "%255s", config->manufacturer) == 1)
		return size;
	return -1;
}

static DEVICE_ATTR(manufacturer, S_IRUGO | S_IWUSR, rndis_manufacturer_show,
						    rndis_manufacturer_store);

static ssize_t rndis_wceis_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *config = f->config;
	return snprintf(buf, PAGE_SIZE, "%d\n", config->wceis);
}

static ssize_t rndis_wceis_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *config = f->config;
	int value;

	if (sscanf(buf, "%d", &value) == 1) {
		config->wceis = value;
		return size;
	}
	return -EINVAL;
}

static DEVICE_ATTR(wceis, S_IRUGO | S_IWUSR, rndis_wceis_show,
					     rndis_wceis_store);

static ssize_t rndis_ethaddr_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *rndis = f->config;
	return snprintf(buf, PAGE_SIZE, "%02x:%02x:%02x:%02x:%02x:%02x\n",
		rndis->ethaddr[0], rndis->ethaddr[1], rndis->ethaddr[2],
		rndis->ethaddr[3], rndis->ethaddr[4], rndis->ethaddr[5]);
}

static ssize_t rndis_ethaddr_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *rndis = f->config;

	if (sscanf(buf, "%02x:%02x:%02x:%02x:%02x:%02x\n",
		    (int *)&rndis->ethaddr[0], (int *)&rndis->ethaddr[1],
		    (int *)&rndis->ethaddr[2], (int *)&rndis->ethaddr[3],
		    (int *)&rndis->ethaddr[4], (int *)&rndis->ethaddr[5]) == 6)
		return size;
	return -EINVAL;
}

static DEVICE_ATTR(ethaddr, S_IRUGO | S_IWUSR, rndis_ethaddr_show,
					       rndis_ethaddr_store);

static ssize_t rndis_vendorID_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *config = f->config;
	return snprintf(buf, PAGE_SIZE, "%04x\n", config->vendorID);
}

static ssize_t rndis_vendorID_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *config = f->config;
	int value;

	if (sscanf(buf, "%04x", &value) == 1) {
		config->vendorID = value;
		return size;
	}
	return -EINVAL;
}

static DEVICE_ATTR(vendorID, S_IRUGO | S_IWUSR, rndis_vendorID_show,
						rndis_vendorID_store);

static struct device_attribute *rndis_function_attributes[] = {
	&dev_attr_manufacturer,
	&dev_attr_wceis,
	&dev_attr_ethaddr,
	&dev_attr_vendorID,
	NULL
};

static struct android_usb_function rndis_function = {
	.name		= "rndis",
	.init		= rndis_function_init,
	.cleanup	= rndis_function_cleanup,
	.bind_config	= rndis_function_bind_config,
	.unbind_config	= rndis_function_unbind_config,
	.attributes	= rndis_function_attributes,
};


struct mass_storage_function_config {
	struct fsg_config fsg;
	struct fsg_common *common;
};

static int mass_storage_function_init(struct android_usb_function *f,
					struct usb_composite_dev *cdev)
{
	struct mass_storage_function_config *config;
	struct fsg_common *common;
	int err;
	struct android_dev *dev = _android_dev;
	int i;

	config = kzalloc(sizeof(struct mass_storage_function_config),
								GFP_KERNEL);
	if (!config)
		return -ENOMEM;


	if (dev->pdata->nluns) {
		config->fsg.nluns = dev->pdata->nluns;
		if (config->fsg.nluns > FSG_MAX_LUNS)
			config->fsg.nluns = FSG_MAX_LUNS;
		for (i = 0; i < config->fsg.nluns; i++) {
			if (dev->pdata->cdrom_lun & (1 << i)) {
				config->fsg.luns[i].cdrom = 1;
				config->fsg.luns[i].removable = 1;
				config->fsg.luns[i].ro = 1;
			} else {
				config->fsg.luns[i].cdrom = 0;
				config->fsg.luns[i].removable = 1;
				config->fsg.luns[i].ro = 0;
			}
		}
	} else {
		/* default value */
		config->fsg.nluns = 1;
		config->fsg.luns[0].removable = 1;
	}

	config->fsg.vendor_name = dev->pdata->manufacturer_name;
	config->fsg.product_name= dev->pdata->product_name;

	common = fsg_common_init(NULL, cdev, &config->fsg);
	if (IS_ERR(common)) {
		kfree(config);
		return PTR_ERR(common);
	}

	for (i = 0; i < config->fsg.nluns; i++) {
		err = sysfs_create_link(&f->dev->kobj,
					&common->luns[i].dev.kobj,
					common->luns[i].dev.kobj.name);
		if (err) {
			fsg_common_release(&common->ref);
			kfree(config);
			return err;
		}
	}

	config->common = common;
	f->config = config;
	return 0;
}

static void mass_storage_function_cleanup(struct android_usb_function *f)
{
	kfree(f->config);
	f->config = NULL;
}

static int mass_storage_function_bind_config(struct android_usb_function *f,
						struct usb_configuration *c)
{
	struct mass_storage_function_config *config = f->config;
	return fsg_bind_config(c->cdev, c, config->common);
}

static ssize_t mass_storage_inquiry_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct mass_storage_function_config *config = f->config;
	return snprintf(buf, PAGE_SIZE, "%s\n", config->common->inquiry_string);
}

static ssize_t mass_storage_inquiry_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct mass_storage_function_config *config = f->config;
	if (size >= sizeof(config->common->inquiry_string))
		return -EINVAL;
	if (sscanf(buf, "%28s", config->common->inquiry_string) != 1)
		return -EINVAL;
	return size;
}

static DEVICE_ATTR(inquiry_string, S_IRUGO | S_IWUSR,
					mass_storage_inquiry_show,
					mass_storage_inquiry_store);

static struct device_attribute *mass_storage_function_attributes[] = {
	&dev_attr_inquiry_string,
	NULL
};

static struct android_usb_function mass_storage_function = {
	.name		= "mass_storage",
	.init		= mass_storage_function_init,
	.cleanup	= mass_storage_function_cleanup,
	.bind_config	= mass_storage_function_bind_config,
	.attributes	= mass_storage_function_attributes,
};


static int accessory_function_init(struct android_usb_function *f,
					struct usb_composite_dev *cdev)
{
	return acc_setup();
}

static void accessory_function_cleanup(struct android_usb_function *f)
{
	acc_cleanup();
}

static int accessory_function_bind_config(struct android_usb_function *f,
						struct usb_configuration *c)
{
	return acc_bind_config(c);
}

static int accessory_function_ctrlrequest(struct android_usb_function *f,
						struct usb_composite_dev *cdev,
						const struct usb_ctrlrequest *c)
{
	return acc_ctrlrequest(cdev, c);
}

static struct android_usb_function accessory_function = {
	.name		= "accessory",
	.init		= accessory_function_init,
	.cleanup	= accessory_function_cleanup,
	.bind_config	= accessory_function_bind_config,
	.ctrlrequest	= accessory_function_ctrlrequest,
};


#ifdef CONFIG_USB_ANDROID_PROJECTOR
static int projector_function_init(struct android_usb_function *f,
		struct usb_composite_dev *cdev)
{
	f->config = kzalloc(sizeof(struct htcmode_protocol), GFP_KERNEL);
	if (!f->config)
		return -ENOMEM;

	return projector_setup(f->config);
}

static void projector_function_cleanup(struct android_usb_function *f)
{

	projector_cleanup();

	if (f->config) {
		kfree(f->config);
		f->config = NULL;
	}
}

static int projector_function_bind_config(struct android_usb_function *f,
		struct usb_configuration *c)
{
	return projector_bind_config(c);
}


static ssize_t projector_width_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct htcmode_protocol *config = f->config;
	return snprintf(buf, PAGE_SIZE, "%d\n", config->server_info.width);
}

static DEVICE_ATTR(width, S_IRUGO | S_IWUSR, projector_width_show,
						    NULL);

static ssize_t projector_height_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct htcmode_protocol *config = f->config;
	return snprintf(buf, PAGE_SIZE, "%d\n", config->server_info.height);
}

static DEVICE_ATTR(height, S_IRUGO | S_IWUSR, projector_height_show,
						    NULL);

static ssize_t projector_rotation_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct htcmode_protocol *config = f->config;
	return snprintf(buf, PAGE_SIZE, "%d\n", (config->client_info.display_conf & CLIENT_INFO_SERVER_ROTATE_USED));
}

static DEVICE_ATTR(rotation, S_IRUGO | S_IWUSR, projector_rotation_show,
						    NULL);

static ssize_t projector_version_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct htcmode_protocol *config = f->config;
	return snprintf(buf, PAGE_SIZE, "%d\n", config->version);
}

static DEVICE_ATTR(version, S_IRUGO | S_IWUSR, projector_version_show,
						    NULL);

static ssize_t projector_vendor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct htcmode_protocol *config = f->config;
	return snprintf(buf, PAGE_SIZE, "%d\n", config->vendor);
}

static DEVICE_ATTR(vendor, S_IRUGO | S_IWUSR, projector_vendor_show,
						    NULL);

static ssize_t projector_server_nonce_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct htcmode_protocol *config = f->config;
	memcpy(buf, config->nonce, HSML_SERVER_NONCE_SIZE);
	return HSML_SERVER_NONCE_SIZE;
}

static DEVICE_ATTR(server_nonce, S_IRUGO | S_IWUSR, projector_server_nonce_show,
						    NULL);

static ssize_t projector_client_sig_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct htcmode_protocol *config = f->config;
	memcpy(buf, config->client_sig, HSML_CLIENT_SIG_SIZE);
	return HSML_CLIENT_SIG_SIZE;
}

static DEVICE_ATTR(client_sig, S_IRUGO | S_IWUSR, projector_client_sig_show,
						    NULL);

static ssize_t projector_server_sig_store(
		struct device *dev, struct device_attribute *attr,
		const char *buff, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct htcmode_protocol *config = f->config;
	memcpy(config->server_sig, buff, HSML_SERVER_SIG_SIZE);
	return HSML_SERVER_SIG_SIZE;
}

static DEVICE_ATTR(server_sig, S_IWUSR, NULL,
		projector_server_sig_store);

static ssize_t projector_auth_store(
		struct device *dev, struct device_attribute *attr,
		const char *buff, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct htcmode_protocol *config = f->config;
	memcpy(&config->auth_result, buff, sizeof(config->auth_result));
	config->auth_in_progress = 0;
	return sizeof(config->auth_result);
}

static DEVICE_ATTR(auth, S_IWUSR, NULL,
		projector_auth_store);

static ssize_t projector_debug_mode_store(
		struct device *dev, struct device_attribute *attr,
		const char *buff, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct htcmode_protocol *config = f->config;
	int value, i;
	int framesize = DEFAULT_PROJ_HEIGHT * DEFAULT_PROJ_WIDTH;

	if (sscanf(buff, "%d", &value) == 1) {

		if (!test_frame)
			test_frame = kzalloc(framesize * 2, GFP_KERNEL);

		if (test_frame)
			for (i = 0 ; i < framesize ; i++)
				if (i < framesize/4)
					test_frame[i] = 0xF800;
				else if (i < framesize*2/4)
					test_frame[i] = 0x7E0;
				else if (i < framesize*3/4)
					test_frame[i] = 0x1F;
				else
					test_frame[i] = 0xFFFF;

		config->debug_mode = value;
		return size;
	}
	return -EINVAL;
}

static DEVICE_ATTR(debug_mode, S_IWUSR, NULL,
		projector_debug_mode_store);

static struct device_attribute *projector_function_attributes[] = {
	&dev_attr_width,
	&dev_attr_height,
	&dev_attr_rotation,
	&dev_attr_version,
	&dev_attr_vendor,
	&dev_attr_server_nonce,
	&dev_attr_client_sig,
	&dev_attr_server_sig,
	&dev_attr_auth,
	&dev_attr_debug_mode,
	NULL
};


struct android_usb_function projector_function = {
	.name		= "projector",
	.init		= projector_function_init,
	.cleanup	= projector_function_cleanup,
	.bind_config	= projector_function_bind_config,
	.attributes = projector_function_attributes
};
#endif

static struct android_usb_function *supported_functions[] = {
	&rndis_function,
	&accessory_function,
	&mtp_function,
	&ptp_function,
	&adb_function,
	&mass_storage_function,
	&ecm_function,
#ifdef CONFIG_USB_ANDROID_DIAG
	&diag_function,
#endif
#ifdef CONFIG_MODEM_SUPPORT
	&modem_function,
#endif
	&serial_function,
#ifdef CONFIG_USB_ANDROID_RMNET
	&rmnet_function,
#endif
#ifdef CONFIG_USB_ANDROID_PROJECTOR
	&projector_function,
#endif
#ifdef CONFIG_USB_ANDROID_ACM
	&acm_function,
#endif
#if 0
	&ccid_function,
#endif
#ifdef CONFIG_USB_ANDROID_USBNET
	&usbnet_function,
#endif
	NULL
};


static void android_cleanup_functions(struct android_usb_function **functions)
{
	struct android_usb_function *f;
	struct device_attribute **attrs;
	struct device_attribute *attr;

	while (*functions) {
		f = *functions++;

		if (f->dev) {
			device_destroy(android_class, f->dev->devt);
			kfree(f->dev_name);
		} else
			continue;

		if (f->cleanup)
			f->cleanup(f);

		attrs = f->attributes;
		if (attrs) {
			while ((attr = *attrs++))
				device_remove_file(f->dev, attr);
		}
	}
}

static int android_init_functions(struct android_usb_function **functions,
				  struct usb_composite_dev *cdev)
{
	struct android_dev *dev = _android_dev;
	struct android_usb_function *f;
	struct device_attribute **attrs;
	struct device_attribute *attr;
	int err = 0;
	int index = 1; /* index 0 is for android0 device */

	for (; (f = *functions++); index++) {
		f->dev_name = kasprintf(GFP_KERNEL, "f_%s", f->name);
		if (!f->dev_name) {
			err = -ENOMEM;
			goto err_out;
		}
		f->dev = device_create(android_class, dev->dev,
				MKDEV(0, index), f, f->dev_name);
		if (IS_ERR(f->dev)) {
			pr_err("%s: Failed to create dev %s", __func__,
							f->dev_name);
			err = PTR_ERR(f->dev);
			f->dev = NULL;
			goto err_create;
		}

		if (device_create_file(f->dev, &dev_attr_on) < 0) {
			pr_err("%s: Failed to create dev file %s", __func__,
							f->dev_name);
			goto err_create;
		}

		if (f->init) {
			err = f->init(f, cdev);
			if (err) {
				pr_err("%s: Failed to init %s", __func__,
								f->name);
				goto err_init;
			}
		}

		attrs = f->attributes;
		if (attrs) {
			while ((attr = *attrs++) && !err)
				err = device_create_file(f->dev, attr);
		}
		if (err) {
			pr_err("%s: Failed to create function %s attributes",
					__func__, f->name);
			goto err_attrs;
		}
		pr_info("%s %s init\n", __func__, f->name);
	}
	return 0;

err_attrs:
	for (attr = *(attrs -= 2); attrs != f->attributes; attr = *(attrs--))
		device_remove_file(f->dev, attr);
	if (f->cleanup)
		f->cleanup(f);
err_init:
	device_destroy(android_class, f->dev->devt);
err_create:
	f->dev = NULL;
	kfree(f->dev_name);
err_out:
	android_cleanup_functions(dev->functions);
	return err;
}

	static int
android_bind_enabled_functions(struct android_dev *dev,
		struct usb_configuration *c)
{
	struct android_usb_function *f;
	int ret;

	list_for_each_entry(f, &dev->enabled_functions, enabled_list) {
		pr_info("%s bind name: %s\n", __func__, f->name);
		ret = f->bind_config(f, c);
		if (ret) {
			pr_err("%s: %s failed", __func__, f->name);
			return ret;
		}
	}
	return 0;
}

static void
android_unbind_enabled_functions(struct android_dev *dev,
			       struct usb_configuration *c)
{
	struct android_usb_function *f;

	list_for_each_entry(f, &dev->enabled_functions, enabled_list) {
		if (f->unbind_config)
			f->unbind_config(f, c);
	}
}

static int android_enable_function(struct android_dev *dev, char *name)
{
	struct android_usb_function **functions = dev->functions;
	struct android_usb_function *f;
	while ((f = *functions++)) {
		if (!strcmp(name, f->name)) {
			pr_info("%s: %s enabled\n", __func__, name);
			list_add_tail(&f->enabled_list, &dev->enabled_functions);
			return 0;
		}
	}
	pr_info("%s: %s failed\n", __func__, name);
	return -EINVAL;
}

/*-------------------------------------------------------------------------*/
/* /sys/class/android_usb/android%d/ interface */

static ssize_t remote_wakeup_show(struct device *pdev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n",
			!!(android_config_driver.bmAttributes &
				USB_CONFIG_ATT_WAKEUP));
}

static ssize_t remote_wakeup_store(struct device *pdev,
		struct device_attribute *attr, const char *buff, size_t size)
{
	int enable = 0;

	sscanf(buff, "%d", &enable);

	pr_debug("android_usb: %s remote wakeup\n",
			enable ? "enabling" : "disabling");

	if (enable)
		android_config_driver.bmAttributes |= USB_CONFIG_ATT_WAKEUP;
	else
		android_config_driver.bmAttributes &= ~USB_CONFIG_ATT_WAKEUP;

	return size;
}

static ssize_t
functions_show(struct device *pdev, struct device_attribute *attr, char *buf)
{
	struct android_dev *dev = dev_get_drvdata(pdev);
	struct android_usb_function *f;
	char *buff = buf;

	list_for_each_entry(f, &dev->enabled_functions, enabled_list)
		buff += snprintf(buff, PAGE_SIZE, "%s,", f->name);
	if (buff != buf)
		*(buff-1) = '\n';
	return buff - buf;
}

static ssize_t
functions_store(struct device *pdev, struct device_attribute *attr,
			       const char *buff, size_t size)
{
	struct android_dev *dev = dev_get_drvdata(pdev);
	char *name;
	char buf[256], *b;
	int err;

	pr_info("%s: buff: %s\n", __func__, buff);
	return size;

	INIT_LIST_HEAD(&dev->enabled_functions);

	strlcpy(buf, buff, sizeof(buf));//htc
	b = strim(buf);

	while (b) {
		name = strsep(&b, ",");
		if (name) {
			err = android_enable_function(dev, name);
			if (err)
				pr_err("android_usb: Cannot enable '%s'", name);
		}
	}

	return size;
}

static ssize_t enable_show(struct device *pdev, struct device_attribute *attr,
			   char *buf)
{
	struct android_dev *dev = dev_get_drvdata(pdev);
	return snprintf(buf, PAGE_SIZE, "%d\n", dev->enabled);
}

static ssize_t enable_store(struct device *pdev, struct device_attribute *attr,
			    const char *buff, size_t size)
{
	struct android_dev *dev = dev_get_drvdata(pdev);
	struct usb_composite_dev *cdev = dev->cdev;
	int enabled = 0;

	sscanf(buff, "%d", &enabled);
//++htc++
	if (enabled) {
		htc_usb_enable_function("adb", 1);
		pr_info("%s, buff: %s\n", __func__, buff);
	}

	/* temporaily return immediately to prevent framework change usb behavior
	 */
	return size;
//--htc--
	if (enabled && !dev->enabled) {
		/* update values in composite driver's copy of device descriptor */
		cdev->desc.idVendor = device_desc.idVendor;
		cdev->desc.idProduct = device_desc.idProduct;
		cdev->desc.bcdDevice = device_desc.bcdDevice;
		cdev->desc.bDeviceClass = device_desc.bDeviceClass;
		cdev->desc.bDeviceSubClass = device_desc.bDeviceSubClass;
		cdev->desc.bDeviceProtocol = device_desc.bDeviceProtocol;
		if (usb_add_config(cdev, &android_config_driver,
							android_bind_config))
			return size;//htc

		usb_gadget_connect(cdev->gadget);
		dev->enabled = true;
	} else if (!enabled && dev->enabled) {
		usb_gadget_disconnect(cdev->gadget);
		usb_remove_config(cdev, &android_config_driver);
		dev->enabled = false;
	} else {
		pr_err("android_usb: already %s\n",
				dev->enabled ? "enabled" : "disabled");
	}
	return size;
}

static ssize_t pm_qos_show(struct device *pdev,
			   struct device_attribute *attr, char *buf)
{
	struct android_dev *dev = dev_get_drvdata(pdev);

	return snprintf(buf, PAGE_SIZE, "%s\n", dev->pm_qos);
}

static ssize_t pm_qos_store(struct device *pdev,
			   struct device_attribute *attr,
			   const char *buff, size_t size)
{
	struct android_dev *dev = dev_get_drvdata(pdev);

	strlcpy(dev->pm_qos, buff, sizeof(dev->pm_qos));

	return size;
}

static ssize_t state_show(struct device *pdev, struct device_attribute *attr,
			   char *buf)
{
	struct android_dev *dev = dev_get_drvdata(pdev);
	struct usb_composite_dev *cdev = dev->cdev;
	char *state = "DISCONNECTED";
	unsigned long flags;

	if (!cdev)
		goto out;

	spin_lock_irqsave(&cdev->lock, flags);
        if (cdev->config)
		state = "CONFIGURED";
	else if (dev->connected)
		state = "CONNECTED";
	spin_unlock_irqrestore(&cdev->lock, flags);
out:
	return snprintf(buf, PAGE_SIZE, "%s\n", state);
}

#define DESCRIPTOR_ATTR(field, format_string)				\
static ssize_t								\
field ## _show(struct device *dev, struct device_attribute *attr,	\
		char *buf)						\
{									\
	return snprintf(buf, PAGE_SIZE,					\
			format_string, device_desc.field);		\
}									\
static ssize_t								\
field ## _store(struct device *dev, struct device_attribute *attr,	\
		const char *buf, size_t size)		       		\
{									\
	int value;					       		\
	if (sscanf(buf, format_string, &value) == 1) {			\
		device_desc.field = value;				\
		return size;						\
	}								\
	return -1;							\
}									\
static DEVICE_ATTR(field, S_IRUGO | S_IWUSR, field ## _show, field ## _store);

#define DESCRIPTOR_STRING_ATTR(field, buffer)				\
static ssize_t								\
field ## _show(struct device *dev, struct device_attribute *attr,	\
		char *buf)						\
{									\
	return snprintf(buf, PAGE_SIZE, "%s", buffer);			\
}									\
static ssize_t								\
field ## _store(struct device *dev, struct device_attribute *attr,	\
		const char *buf, size_t size)		       		\
{									\
	if (size >= sizeof(buffer)) return -EINVAL;			\
	if (sscanf(buf, "%255s", buffer) == 1) {			\
		return size;						\
	}								\
	return -1;							\
}									\
static DEVICE_ATTR(field, S_IRUGO | S_IWUSR, field ## _show, field ## _store);

/*
DESCRIPTOR_ATTR(idVendor, "%04x\n")
DESCRIPTOR_ATTR(idProduct, "%04x\n")
*/
DESCRIPTOR_ATTR(bcdDevice, "%04x\n")
DESCRIPTOR_ATTR(bDeviceClass, "%d\n")
DESCRIPTOR_ATTR(bDeviceSubClass, "%d\n")
DESCRIPTOR_ATTR(bDeviceProtocol, "%d\n")
DESCRIPTOR_STRING_ATTR(iManufacturer, manufacturer_string)
DESCRIPTOR_STRING_ATTR(iProduct, product_string)
DESCRIPTOR_STRING_ATTR(iSerial, serial_string)

static DEVICE_ATTR(functions, S_IRUGO | S_IWUSR, functions_show, functions_store);
static DEVICE_ATTR(enable, S_IRUGO | S_IWUSR, enable_show, enable_store);
static DEVICE_ATTR(pm_qos, S_IRUGO | S_IWUSR,
		pm_qos_show, pm_qos_store);
static DEVICE_ATTR(state, S_IRUGO, state_show, NULL);
static DEVICE_ATTR(remote_wakeup, S_IRUGO | S_IWUSR,
		remote_wakeup_show, remote_wakeup_store);

static ssize_t enable_diag_store(struct device *pdev, struct device_attribute *attr,
			    const char *buff, size_t size)
{
	struct android_dev *dev = dev_get_drvdata(pdev);
	struct usb_composite_dev *cdev = dev->cdev;
	int enable_diag = 0;

	sscanf(buff, "%d", &enable_diag);
	if (enable_diag) {
		usb_gadget_disconnect(cdev->gadget);
		usb_remove_config(cdev, &android_config_driver);
		dev->enabled = false;

		INIT_LIST_HEAD(&dev->enabled_functions);
		strlcpy(diag_clients, "diag_mdm", sizeof(diag_clients));

		android_enable_function(dev, "mass_storage");
		android_enable_function(dev, "adb");
		android_enable_function(dev, "diag");

		/* update values in composite driver's copy of device descriptor */
		cdev->desc.idVendor = __constant_cpu_to_le16(0xbb4);
		cdev->desc.idProduct = __constant_cpu_to_le16(0xc07);
		cdev->desc.bcdDevice = device_desc.bcdDevice;
		cdev->desc.bDeviceClass = device_desc.bDeviceClass;
		cdev->desc.bDeviceSubClass = device_desc.bDeviceSubClass;
		cdev->desc.bDeviceProtocol = device_desc.bDeviceProtocol;
		if (usb_add_config(cdev, &android_config_driver, android_bind_config))
			return size;

		usb_gadget_connect(cdev->gadget);
		dev->enabled = true;
	}
	return size;
}
static DEVICE_ATTR(enable_diag, S_IRUGO | S_IWUSR, NULL, enable_diag_store);

static ssize_t enable_diag_mdm_rmnet_store(struct device *pdev, struct device_attribute *attr,
				const char *buff, size_t size)
{
	struct android_dev *dev = dev_get_drvdata(pdev);
	struct usb_composite_dev *cdev = dev->cdev;
	int enable_diag = 0;

	sscanf(buff, "%d", &enable_diag);
	if (enable_diag) {
		usb_gadget_disconnect(cdev->gadget);
		usb_remove_config(cdev, &android_config_driver);
		dev->enabled = false;

		INIT_LIST_HEAD(&dev->enabled_functions);
		strlcpy(diag_clients, "diag_mdm", sizeof(diag_clients));
		strlcpy(rmnet_transports, "HSIC,HSIC", sizeof(rmnet_transports));
		strlcpy(serial_transports, "HSIC", sizeof(serial_transports));

		android_enable_function(dev, "mass_storage");
		android_enable_function(dev, "adb");
		android_enable_function(dev, "diag");
		android_enable_function(dev, "modem");
		android_enable_function(dev, "rmnet");

		/* update values in composite driver's copy of device descriptor */
		cdev->desc.idVendor = __constant_cpu_to_le16(0xbb4);
		cdev->desc.idProduct = __constant_cpu_to_le16(0xfd8);
		cdev->desc.bcdDevice = device_desc.bcdDevice;
		cdev->desc.bDeviceClass = device_desc.bDeviceClass;
		cdev->desc.bDeviceSubClass = device_desc.bDeviceSubClass;
		cdev->desc.bDeviceProtocol = device_desc.bDeviceProtocol;
		if (usb_add_config(cdev, &android_config_driver, android_bind_config))
			return size;

		usb_gadget_connect(cdev->gadget);
		dev->enabled = true;
	}
	return size;
}


static DEVICE_ATTR(enable_diag_mdm_rmnet, S_IRUGO | S_IWUSR, NULL, enable_diag_mdm_rmnet_store);

static struct device_attribute *android_usb_attributes[] = {
/*	&dev_attr_idVendor,*/
/*	&dev_attr_idProduct,*/
	&dev_attr_bcdDevice,
	&dev_attr_bDeviceClass,
	&dev_attr_bDeviceSubClass,
	&dev_attr_bDeviceProtocol,
	&dev_attr_iManufacturer,
	&dev_attr_iProduct,
	&dev_attr_iSerial,
	&dev_attr_functions,
	&dev_attr_enable,
	&dev_attr_pm_qos,
	&dev_attr_state,
	&dev_attr_remote_wakeup,
	&dev_attr_enable_diag,
	&dev_attr_enable_diag_mdm_rmnet,
	NULL
};

#include "htc_attr.c"

/*-------------------------------------------------------------------------*/
/* Composite driver */

static int android_bind_config(struct usb_configuration *c)
{
	struct android_dev *dev = _android_dev;
	int ret = 0;

	ret = android_bind_enabled_functions(dev, c);
	if (ret)
		return ret;

	return 0;
}

static void android_unbind_config(struct usb_configuration *c)
{
	struct android_dev *dev = _android_dev;

	android_unbind_enabled_functions(dev, c);
}

static int android_bind(struct usb_composite_dev *cdev)
{
	struct android_dev *dev = _android_dev;
	struct android_usb_platform_data *pdata = _android_dev->pdata;
	struct usb_gadget	*gadget = cdev->gadget;
	int			gcnum, id, ret;

	usb_gadget_disconnect(gadget);

	ret = android_init_functions(dev->functions, cdev);
	if (ret)
		return ret;

	/* Allocate string descriptor numbers ... note that string
	 * contents can be overridden by the composite_dev glue.
	 */
	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	strings_dev[STRING_MANUFACTURER_IDX].id = id;
	device_desc.iManufacturer = id;

	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	strings_dev[STRING_PRODUCT_IDX].id = id;
	device_desc.iProduct = id;

	dev->products = pdata->products;
	dev->num_products = pdata->num_products;
	dev->in_house_functions = pdata->functions;
	dev->num_functions = pdata->num_functions;
	dev->match = pdata->match;

	/* default String */
	if (pdata->product_name)
		strlcpy(product_string, pdata->product_name,
			sizeof(product_string) - 1);
	if (pdata->manufacturer_name)
		strlcpy(manufacturer_string, pdata->manufacturer_name,
			sizeof(manufacturer_string) - 1);
	if (pdata->serial_number)
		strlcpy(serial_string, pdata->serial_number,
			sizeof(serial_string) - 1);

	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	strings_dev[STRING_SERIAL_IDX].id = id;
	device_desc.iSerialNumber = id;

	gcnum = usb_gadget_controller_number(gadget);
	if (gcnum >= 0)
		device_desc.bcdDevice = cpu_to_le16(0x0200 + gcnum);
	else {
		/* gadget zero is so simple (for now, no altsettings) that
		 * it SHOULD NOT have problems with bulk-capable hardware.
		 * so just warn about unrcognized controllers -- don't panic.
		 *
		 * things like configuration and altsetting numbering
		 * can need hardware-specific attention though.
		 */
		pr_warning("%s: controller '%s' not recognized\n",
			longname, gadget->name);
		device_desc.bcdDevice = __constant_cpu_to_le16(0x9999);
	}

	/*
	 * As per USB compliance update, a device that is actively drawing
	 * more than 100mA from USB must report itself as bus-powered in
	 * the GetStatus(DEVICE) call.
	 */
	if (android_config_driver.bMaxPower <= (USB_SELF_POWER_VBUS_MAX_DRAW / 2)) {
		android_config_driver.bmAttributes = USB_CONFIG_ATT_ONE | USB_CONFIG_ATT_SELFPOWER;
		usb_gadget_set_selfpowered(gadget);
	} else
		android_config_driver.bmAttributes = USB_CONFIG_ATT_ONE;
	dev->cdev = cdev;

//++htc++

	cdev->sw_connect2pc.name = "usb_connect2pc";
	ret = switch_dev_register(&cdev->sw_connect2pc);
	if (ret < 0)
		pr_err("switch_dev_register fail:usb_connect2pc\n");


	schedule_delayed_work(&dev->init_work, HZ);
//--htc--
	return 0;
}

static int android_usb_unbind(struct usb_composite_dev *cdev)
{
	struct android_dev *dev = _android_dev;

	manufacturer_string[0] = '\0';
	product_string[0] = '\0';
	serial_string[0] = '0';
	cancel_work_sync(&dev->work);
	android_cleanup_functions(dev->functions);
	switch_dev_unregister(&cdev->sw_connect2pc);
	return 0;
}

static struct usb_composite_driver android_usb_driver = {
	.name		= "android_usb",
	.dev		= &device_desc,
	.strings	= dev_strings,
	.unbind		= android_usb_unbind,
	.max_speed	= USB_SPEED_HIGH,
};

static int
android_setup(struct usb_gadget *gadget, const struct usb_ctrlrequest *c)
{
	struct android_dev		*dev = _android_dev;
	struct usb_composite_dev	*cdev = get_gadget_data(gadget);
	struct usb_request		*req = cdev->req;
	struct android_usb_function	*f;
	int value = -EOPNOTSUPP;
	unsigned long flags;

	req->zero = 0;
	req->complete = composite_setup_complete;
	req->length = 0;
	gadget->ep0->driver_data = cdev;

	value = android_switch_setup(gadget, c);
	if (value >= 0)
		return value;

	list_for_each_entry(f, &dev->enabled_functions, enabled_list) {
		if (f->ctrlrequest) {
			value = f->ctrlrequest(f, cdev, c);
			if (value >= 0)
				break;
		}
	}

	/* Special case the accessory function.
	 * It needs to handle control requests before it is enabled.
	 */
	if (value < 0)
		value = acc_ctrlrequest(cdev, c);

	/*
	 * The projector needs to handle control requests before it's enabled.
	 */
#ifdef CONFIG_USB_ANDROID_PROJECTOR
	if (value < 0)
		value = projector_ctrlrequest(cdev, c);
#endif

	if (value < 0)
		value = composite_setup(gadget, c);

	spin_lock_irqsave(&cdev->lock, flags);
	if (!dev->connected) {
		dev->connected = 1;
		schedule_work(&dev->work);
	}
	else if (c->bRequest == USB_REQ_SET_CONFIGURATION && cdev->config) {
		schedule_work(&dev->work);
	}
	spin_unlock_irqrestore(&cdev->lock, flags);

	return value;
}

static void android_disconnect(struct usb_gadget *gadget)
{
	struct android_dev *dev = _android_dev;
	struct usb_composite_dev *cdev = get_gadget_data(gadget);
	unsigned long flags;

	composite_disconnect(gadget);

	spin_lock_irqsave(&cdev->lock, flags);
	dev->connected = 0;
	schedule_work(&dev->work);
	spin_unlock_irqrestore(&cdev->lock, flags);

	/*android_switch_function is not called if removing usb cable. Without it, connect2pc may be blocked by is_mtp_enabled*/
	is_mtp_enabled = false;
}

static void android_mute_disconnect(struct usb_gadget *gadget)
{
	struct android_dev *dev = _android_dev;
	struct usb_composite_dev *cdev = get_gadget_data(gadget);
	unsigned long flags;

	composite_disconnect(gadget);

	/*changes USB_STATE only for MTP*/
	if (is_mtp_enabled) {
		spin_lock_irqsave(&cdev->lock, flags);
		dev->connected = 0;
		schedule_work(&dev->work);
		spin_unlock_irqrestore(&cdev->lock, flags);
	}
}

static int android_create_device(struct android_dev *dev)
{
	struct device_attribute **attrs = android_usb_attributes;
	struct device_attribute *attr;
	int err;

	dev->dev = device_create(android_class, NULL,
					MKDEV(0, 0), NULL, "android0");
	if (IS_ERR(dev->dev))
		return PTR_ERR(dev->dev);

	dev_set_drvdata(dev->dev, dev);

	while ((attr = *attrs++)) {
		err = device_create_file(dev->dev, attr);
		if (err) {
			device_destroy(android_class, dev->dev->devt);
			return err;
		}
	}
	return 0;
}

static void android_destroy_device(struct android_dev *dev)
{
	struct device_attribute **attrs = android_usb_attributes;
	struct device_attribute *attr;

	while ((attr = *attrs++))
		device_remove_file(dev->dev, attr);
	device_destroy(android_class, dev->dev->devt);
}

static int __devinit android_probe(struct platform_device *pdev)
{
	struct android_usb_platform_data *pdata = pdev->dev.platform_data;
	struct android_dev *dev = _android_dev;
	int ret = 0;

	dev->pdata = pdata;

	init_mfg_serialno();
	if (sysfs_create_group(&pdev->dev.kobj, &android_usb_attr_group))
		pr_err("%s: fail to create sysfs\n", __func__);

	if (dev->pdata->usb_diag_interface)
		strlcpy(diag_clients, dev->pdata->usb_diag_interface, sizeof(diag_clients));
	if (dev->pdata->usb_rmnet_interface)
		strlcpy(rmnet_transports, dev->pdata->usb_rmnet_interface, sizeof(rmnet_transports));

	android_class = class_create(THIS_MODULE, "android_usb");
	if (IS_ERR(android_class))
		return PTR_ERR(android_class);

	ret = android_create_device(dev);
	if (ret) {
		pr_err("%s(): android_create_device failed\n", __func__);
		goto err_dev;
	}

	ret = usb_composite_probe(&android_usb_driver, android_bind);
	if (ret) {
		pr_err("%s(): Failed to register android "
					"composite driver\n", __func__);
		goto err_probe;
	}

	/* pm qos request to prevent apps idle power collapse */
	if (pdata && pdata->swfi_latency)
		pm_qos_add_request(&dev->pm_qos_req_dma,
			PM_QOS_CPU_DMA_LATENCY, PM_QOS_DEFAULT_VALUE);
	strlcpy(dev->pm_qos, "high", sizeof(dev->pm_qos));

	pm_qos_add_request(&pm_qos_req_tp, PM_QOS_CPU_FREQ_MIN, (s32)PM_QOS_CPU_FREQ_MIN_DEFAULT_VALUE);

	return ret;
err_probe:
	android_destroy_device(dev);
err_dev:
	class_destroy(android_class);
	return ret;
}

static int android_remove(struct platform_device *pdev)
{
	struct android_dev *dev = _android_dev;
	struct android_usb_platform_data *pdata = pdev->dev.platform_data;

	android_destroy_device(dev);
	class_destroy(android_class);
	usb_composite_unregister(&android_usb_driver);
	if (pdata && pdata->swfi_latency)
		pm_qos_remove_request(&dev->pm_qos_req_dma);
	pm_qos_remove_request(&pm_qos_req_tp);
	return 0;
}

static struct platform_driver android_platform_driver = {
	.driver = { .name = "android_usb"},
	.probe = android_probe,
	.remove = android_remove,
};

static void android_usb_init_work(struct work_struct *data)
{
	struct android_dev *dev = _android_dev;
	struct android_usb_platform_data *pdata = dev->pdata;
	struct usb_composite_dev *cdev = dev->cdev;
	int ret = 0;
	__u16 product_id;

#ifdef CONFIG_SENSE_4_PLUS
	/* initial mtp+ums+adb by default */
	ret = android_enable_function(dev, "mtp");
	if (ret)
		pr_err("android_usb: Cannot enable '%s'", "mtp");
#endif

	ret = android_enable_function(dev, "mass_storage");
	if (ret)
		pr_err("android_usb: Cannot enable '%s'", "mass_storage");

#if 0
	ret = android_enable_function(dev, "adb");
	if (ret)
		pr_err("android_usb: Cannot enable '%s'", "adb");
#endif

	/* initial function depends on radio flag */
	if (pdata->diag_init) {
		ret = android_enable_function(dev, DIAG_LEGACY);
		if (ret)
			pr_err("android_usb: Cannot enable '%s'", DIAG_LEGACY);
	}

	if (pdata->modem_init) {
		ret = android_enable_function(dev, "modem");
		if (ret)
			pr_err("android_usb: Cannot enable '%s'", "modem");
#if defined(CONFIG_USB_ANDROID_MDM9K_MODEM)
		ret = android_enable_function(dev, "modem_mdm");
		if (ret)
			pr_err("android_usb: Cannot enable '%s'", "modem_mdm");
#endif
	}


#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
	if (pdata->diag_init) {
		ret = android_enable_function(dev, DIAG_MDM);
		if (ret)
			pr_err("android_usb: Cannot enable '%s'", DIAG_MDM);
	}
#endif //CONFIG_USB_ANDROID_MDM9K_DIAG

	if (pdata->rmnet_init) {
		ret = android_enable_function(dev, "rmnet");
		if (ret)
			pr_err("android_usb: Cannot enable '%s'", "rmnet");
	}


	cdev->desc.idVendor = __constant_cpu_to_le16(pdata->vendor_id),
	product_id = get_product_id(dev, &dev->enabled_functions);

	if (dev->match)
		product_id = dev->match(product_id, intrsharing);

	cdev->desc.idProduct = __constant_cpu_to_le16(product_id),
	cdev->desc.bcdDevice = device_desc.bcdDevice;
	cdev->desc.bDeviceClass = device_desc.bDeviceClass;
	cdev->desc.bDeviceSubClass = device_desc.bDeviceSubClass;
	cdev->desc.bDeviceProtocol = device_desc.bDeviceProtocol;

	device_desc.idVendor = cdev->desc.idVendor;
	device_desc.idProduct = cdev->desc.idProduct;

	ret = usb_add_config(cdev, &android_config_driver,
				android_bind_config);

	usb_gadget_connect(cdev->gadget);
	dev->enabled = true;
	pr_info("%s: ret: %d\n", __func__, ret);
}
//--htc--

static int __init init(void)
{
	struct android_dev *dev;
	int ret;

	atomic_set(&connect2pc, false);

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		pr_err("%s(): Failed to alloc memory for android_dev\n",
				__func__);
		return -ENOMEM;
	}

	dev->functions = supported_functions;
	INIT_LIST_HEAD(&dev->enabled_functions);
	INIT_WORK(&dev->work, android_work);
	INIT_DELAYED_WORK(&dev->init_work, android_usb_init_work);
	INIT_WORK(&switch_adb_work, do_switch_adb_work);

	_android_dev = dev;

	/* Override composite driver functions */
	composite_driver.setup = android_setup;
	composite_driver.disconnect = android_disconnect;
	composite_driver.mute_disconnect = android_mute_disconnect;

	ret = platform_driver_register(&android_platform_driver);
	if (ret) {
		pr_err("%s(): Failed to register android"
				 "platform driver\n", __func__);
		kfree(dev);
	}

	return ret;
}
module_init(init);

static void __exit cleanup(void)
{
	platform_driver_unregister(&android_platform_driver);
	kfree(_android_dev);
	_android_dev = NULL;
}
module_exit(cleanup);
