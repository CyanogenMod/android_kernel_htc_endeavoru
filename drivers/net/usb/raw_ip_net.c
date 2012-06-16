/*
 * raw_ip_net.c
 *
 * USB network driver for RAW-IP modems.
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

/*
 * HTC: version history
 *	v01 - bert_lin - 20111026
 *		1. support runtime debugging
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/etherdevice.h>
#include <linux/usb.h>

/* HTC include file */
#include <mach/htc_hostdbg.h>

#define BASEBAND_USB_NET_DEV_NAME		"rmnet%d"

/* ethernet packet ethertype for IP packets */
#define NET_IP_ETHERTYPE		0x08, 0x00

#define	TX_TIMEOUT		10
//#ifdef VERBOSE_DEBUG
//#define verbose	1
//#else
//#define verbose	0
//#endif
static int verbose = 0;
module_param(verbose, int, 0644);
MODULE_PARM_DESC(verbose, "raw ip (cdc-acm) - verbose debug messages");


/* HTC: debugging flag & macro */
/*
#define RAW_IP_DBG
*/
#define MODULE_NAME "[RAWIPv1] "
#ifdef pr_debug
#undef pr_debug
#endif
#ifdef RAW_IP_DBG
static int raw_dbg = 1; /* debugging flag, 1: print out pr_debug */
#else
static int raw_dbg = 0;
#endif
static int dbg_flag = 0; /* tmp flag */

/* HTC: usage cnt checking */
static int autopm_refcnt = 0;

#define pr_debug(x...) do {\
	if (raw_dbg || host_dbg_flag & DBG_RAWIP_L1) \
		pr_info(x); \
} while (0)
#define save_dbg(save) do {\
	save = raw_dbg;\
	raw_dbg = 1;\
} while (0)
#define restore_dbg(restore) do {\
	raw_dbg = restore;\
} while (0)
#define rawlog1(x...) do {\
	if (host_dbg_flag & DBG_RAWIP_L1) \
		pr_info(x);\
} while (0)
#define rawlog2(x...) do {\
	if (host_dbg_flag & DBG_RAWIP_L2) {\
		pr_info(x);\
} while (0)
#define rawlog3(x...) do {\
	if (host_dbg_flag & DBG_RAWIP_L3) \
		pr_info(x);\
} while (0)
#define rawlog4(x...) do {\
	if (host_dbg_flag & DBG_RAWIP_L4) \
		pr_info(x);\
} while (0)

/* HTC: variables */
extern unsigned int host_dbg_flag;


//+Sophia:0112
struct rmnet_private
{
	struct net_device_stats stats;
};
//-Sophia:0112


#ifndef USB_NET_BUFSIZ
#define USB_NET_BUFSIZ				8192
#endif  /* USB_NET_BUFSIZ */

/* maximum interface number supported */
//#define MAX_INTFS	2 /* for offline trace first */
#define MAX_INTFS	3 

MODULE_LICENSE("GPL");

int g_i;

extern void usb_register_dump(void);

int max_intfs = MAX_INTFS;
unsigned long usb_net_raw_ip_vid = 0x1519;
unsigned long usb_net_raw_ip_pid = 0x0020;
unsigned long usb_net_raw_ip_intf[MAX_INTFS] = { 0x03, 0x05, 0x07 };
//unsigned long usb_net_raw_ip_intf[MAX_INTFS] = { 0x03, 0x05 };//for offline trace first
unsigned long usb_net_raw_ip_rx_debug;
unsigned long usb_net_raw_ip_tx_debug;

module_param(max_intfs, int, 0644);
MODULE_PARM_DESC(max_intfs, "usb net (raw-ip) - max. interfaces supported");
module_param(usb_net_raw_ip_vid, ulong, 0644);
MODULE_PARM_DESC(usb_net_raw_ip_vid, "usb net (raw-ip) - USB VID");
module_param(usb_net_raw_ip_pid, ulong, 0644);
MODULE_PARM_DESC(usb_net_raw_ip_pid, "usb net (raw-ip) - USB PID");
module_param(usb_net_raw_ip_rx_debug, ulong, 0644);
MODULE_PARM_DESC(usb_net_raw_ip_rx_debug, "usb net (raw-ip) - rx debug");
module_param(usb_net_raw_ip_tx_debug, ulong, 0644);
MODULE_PARM_DESC(usb_net_raw_ip_tx_debug, "usb net (raw-ip) - tx debug");

struct baseband_usb {
	int baseband_index;
	struct {
		struct usb_driver *driver;
		struct usb_device *device;
		struct usb_interface *interface;
		struct {
			struct {
				unsigned int in;
				unsigned int out;
			} isoch, bulk, interrupt;
		} pipe;
		/* currently active rx urb */
		struct urb *rx_urb;
		/* currently active tx urb */
		/*77228-5 patch*/
		struct urb *tx_urb;struct usb_anchor tx_urb_deferred;
		struct workqueue_struct *tx_workqueue;
		struct work_struct tx_work;
		/*77228-5 patch*/
	} usb;
	/* 77969-7 patch */
	struct urb *urb_r;
	void *buff;
	/* 77969-7 patch */
	int susp_count;
};

static struct baseband_usb *baseband_usb_net[MAX_INTFS] = { 0, 0, 0};
//static struct baseband_usb *baseband_usb_net[MAX_INTFS] = { 0, 0 };//for offline trace first

static struct net_device *usb_net_raw_ip_dev[MAX_INTFS] = { 0, 0, 0};
//static struct net_device *usb_net_raw_ip_dev[MAX_INTFS] = { 0, 0 };//for offline trace first


static unsigned int g_usb_interface_index[MAX_INTFS];
static struct usb_interface *g_usb_interface[MAX_INTFS];

static int usb_net_raw_ip_rx_urb_submit(struct baseband_usb *usb);
static void usb_net_raw_ip_rx_urb_comp(struct urb *urb);
/* 77228-5 patch */
static int usb_net_raw_ip_tx_urb_submit(struct baseband_usb *usb,
struct sk_buff *skb);
static void usb_net_raw_ip_tx_urb_work(struct work_struct *work);
/* 77228-5 patch */
static void usb_net_raw_ip_tx_urb_comp(struct urb *urb);
//htc++
static void rawip_print_buffdata(unsigned char *buf, int len,int txrx)
{

	//printk("%s(%d)\n", __func__, __LINE__);
	int i=0;
	//txrx=0->tx

	printk("RAW_IP");
	printk("Len=%d ",len);
	if(len>=20){
		len=20;
	}
	if(txrx){
		printk("TX-");
	}else{

		printk("RX-");
	}

	printk("DATA[%d]=",len);

	for(i=0;i<len;i++){
		printk("%x ",buf[i]);
	}
	//pr_debug("RAW_IP DATA[%d]: %x %x %x %x %x %x %x %x %x %x\n",len,
		//	buf[0], buf[1], buf[2], buf[3], buf[4],
			//buf[5], buf[6], buf[7], buf[8], buf[9]);

	printk("\n");

}
//htc--
static int baseband_usb_driver_probe(struct usb_interface *intf,
	const struct usb_device_id *id)
{
	int i = g_i;
	/* 77969-7 patch */
	struct urb *urb;
	/* 77969-7 patch */

	pr_debug("%s(%d) with suspend conunt fix (do not return BUSY) { intf %p id %p g_i=%d max_intfs=%d\n", __func__, __LINE__, intf, id, i,max_intfs);

	rawlog1("intf->cur_altsetting->desc.bInterfaceNumber %02x\n",
		intf->cur_altsetting->desc.bInterfaceNumber);
	rawlog1("intf->cur_altsetting->desc.bAlternateSetting %02x\n",
		intf->cur_altsetting->desc.bAlternateSetting);
	rawlog1("intf->cur_altsetting->desc.bNumEndpoints %02x\n",
		intf->cur_altsetting->desc.bNumEndpoints);
	rawlog1("intf->cur_altsetting->desc.bInterfaceClass %02x\n",
		intf->cur_altsetting->desc.bInterfaceClass);
	rawlog1("intf->cur_altsetting->desc.bInterfaceSubClass %02x\n",
		intf->cur_altsetting->desc.bInterfaceSubClass);
	rawlog1("intf->cur_altsetting->desc.bInterfaceProtocol %02x\n",
		intf->cur_altsetting->desc.bInterfaceProtocol);
	rawlog1("intf->cur_altsetting->desc.iInterface %02x\n",
		intf->cur_altsetting->desc.iInterface);

	if (g_usb_interface_index[i] !=
		intf->cur_altsetting->desc.bInterfaceNumber) {
		pr_debug("%s(%d) } -ENODEV\n", __func__, __LINE__);
		return -ENODEV;
	} else {
		g_usb_interface[i] = intf;
	}

	pr_debug("%s(%d) }\n", __func__, __LINE__);
	return 0;
}

static void baseband_usb_driver_disconnect(struct usb_interface *intf)
{
	int i;
	/* 77969-7 patch*/
	struct urb *urb;
	/* 77969-7 patch*/

	pr_info("%s intf %p\n", __func__, intf);

	for (i = 0; i < max_intfs; i++) {
		pr_debug("[%d]\n", i);
		if (!baseband_usb_net[i])
		continue;
		if (baseband_usb_net[i]->usb.interface != intf) {
				pr_debug("%p != %p\n",
			baseband_usb_net[i]->usb.interface, intf);
			continue;
		}
		/* 77969-7 patch */
		/* kill usb tx */
		while ((urb = usb_get_from_anchor(&baseband_usb_net[i]->
			usb.tx_urb_deferred)) != (struct urb *) 0) {
			pr_info("%s: kill deferred tx urb %p\n",
				__func__, urb);
			/* decrement count from usb_get_from_anchor() */
			usb_free_urb(urb);
			/* kill tx urb */
			usb_kill_urb(urb);
			/* free tx urb + tx urb transfer buffer */
			if (urb->transfer_buffer) {
				kfree(urb->transfer_buffer);
				urb->transfer_buffer = (void *) 0;
			}
			usb_free_urb(urb);
		}
		if (baseband_usb_net[i]->usb.tx_workqueue) {
			flush_workqueue(baseband_usb_net[i]
				->usb.tx_workqueue);
		}
		if (baseband_usb_net[i]->usb.tx_urb) {
			usb_kill_urb(baseband_usb_net[i]->usb.tx_urb);
			baseband_usb_net[i]->usb.tx_urb
				= (struct urb *) 0;
		}
		/* kill usb rx */
		if (baseband_usb_net[i]->usb.rx_urb) {
			usb_kill_urb(baseband_usb_net[i]->usb.rx_urb);
			baseband_usb_net[i]->usb.rx_urb
				= (struct urb *) 0;
		}
		/* 77969-7 patch */
		/* mark interface as disconnected */
		baseband_usb_net[i]->usb.interface
			= (struct usb_interface *) 0;
	}
}

#ifdef CONFIG_PM
static int baseband_usb_driver_suspend(struct usb_interface *intf,
	pm_message_t message)
{
	int i, err, susp_count;

	save_dbg(dbg_flag);/* HTC */
	pr_debug("%s intf %p\n", __func__, intf);


	//pr_info("%s: cnt %d intf=%p &intf->dev=%p kobje=%s\n",
		//	__func__, atomic_read(&intf->dev.power.usage_count),intf,&intf->dev,kobject_name(&intf->dev.kobj));

	for (i = 0; i < max_intfs; i++) {
		//pr_debug("[%d]\n", i);
		if (!baseband_usb_net[i])
			continue;
		if (baseband_usb_net[i]->usb.interface != intf) {
			//pr_debug("%p != %p\n",
				//baseband_usb_net[i]->usb.interface, intf);
			continue;
		}
		/* increment suspend count */
		susp_count = (baseband_usb_net[i]->susp_count)++;
		if (susp_count > 0) {
			pr_info("%s: susp_count %d > 0 (already suspended)\n",
				__func__, susp_count);
			continue;
		}
		if (susp_count < 0) {
			pr_info("%s: susp_count %d < 0 (ILLEGAL VALUE)\n",
				__func__, susp_count);
			baseband_usb_net[i]->susp_count = 0;
			continue;
		}
		pr_info("%s: susp_count = %d (suspending...)\n",
			__func__, susp_count);
		/* kill usb rx */
		if (!baseband_usb_net[i]->usb.rx_urb) {
			pr_debug("rx_usb already killed\n");
			continue;
		}
		
		//pr_info("%s: try to kill rx_urb...\n",__func__);
		usb_kill_urb(baseband_usb_net[i]->usb.rx_urb);
		//pr_info("%s: try to kill rx_urb.......Done!\n",__func__);
		baseband_usb_net[i]->usb.rx_urb = (struct urb *) 0;

		
		/* cancel tx urb work (will restart after resume) */
		if (!baseband_usb_net[i]->usb.tx_workqueue) {
			pr_err("%s: !tx_workqueue\n", __func__);
			continue;
		}
		
		//pr_info("%s: try to cancel_work_sync...\n",__func__);
		cancel_work_sync(&baseband_usb_net[i]->usb.tx_work);
		//pr_info("%s: try to cancel_work_sync...Done!\n",__func__);
	}
	restore_dbg(dbg_flag);/* HTC */

	return 0;
}

static int baseband_usb_driver_resume(struct usb_interface *intf)
{
	int i, err, susp_count;

	save_dbg(dbg_flag);/* HTC */
	pr_debug("%s intf %p \n", __func__, intf);


	//pr_info("%s: cnt with put async %d intf=%p &intf->dev=%p kobje=%s\n",
		//	__func__, atomic_read(&intf->dev.power.usage_count),intf,&intf->dev,kobject_name(&intf->dev.kobj));

	for (i = 0; i < max_intfs; i++) {
		//pr_debug("[%d]\n", i);
		if (!baseband_usb_net[i])
			continue;
		if (baseband_usb_net[i]->usb.interface != intf) {
			//pr_debug("[%d] %p != %p DebugOnly\n",i,
				//baseband_usb_net[i]->usb.interface, intf);
			continue;
		}
		/* decrement suspend count */
		susp_count = --(baseband_usb_net[i]->susp_count);
		if (susp_count > 0) {
			pr_info("%s: susp_count %d > 0 (not resuming yet)\n",
				__func__, susp_count);
			continue;
		}
		if (susp_count < 0) {
			pr_info("%s: susp_count %d < 0 (ILLEGAL VALUE)\n",
				__func__, susp_count);
			baseband_usb_net[i]->susp_count = 0;
			continue;
		}
		pr_info("%s: susp_count = %d (resuming...)\n",
			__func__, susp_count);
		/* start usb rx */
		if (baseband_usb_net[i]->usb.rx_urb) {
			pr_debug("rx_usb already exists\n");
			continue;
		}
		err = usb_net_raw_ip_rx_urb_submit(baseband_usb_net[i]);
		if (err < 0) {
			pr_err("submit rx failed - err %d\n", err);
			continue;
		}
		/* restart tx urb work (cancelled in suspend) */
		if (!baseband_usb_net[i]->usb.tx_workqueue) {
			pr_err("%s: !tx_workqueue\n", __func__);
			continue;
		}
		queue_work(baseband_usb_net[i]->usb.tx_workqueue,
			&baseband_usb_net[i]->usb.tx_work);
	}
	restore_dbg(dbg_flag);/* HTC */

	return 0;
}
static int baseband_usb_driver_reset_resume(struct usb_interface *intf)
{
	pr_info("%s intf %p\n", __func__, intf);
	return baseband_usb_driver_resume(intf);
}
#endif /* CONFIG_PM */

#if 1
static struct usb_device_id baseband_usb_driver_id_table0[] = {
	{ USB_DEVICE(0x1519, 0x0020), },
	{ }
};
MODULE_DEVICE_TABLE(usb, baseband_usb_driver_id_table0);
#endif

static struct usb_device_id baseband_usb_driver_id_table[MAX_INTFS][2];

static char baseband_usb_driver_name[MAX_INTFS][32];

static struct usb_driver baseband_usb_driver[MAX_INTFS] = {
	{
		.name = baseband_usb_driver_name[0],
		.probe = baseband_usb_driver_probe,
		.disconnect = baseband_usb_driver_disconnect,
		.id_table = baseband_usb_driver_id_table[0],
#ifdef CONFIG_PM
		.suspend = baseband_usb_driver_suspend,
		.resume = baseband_usb_driver_resume,
		.reset_resume = baseband_usb_driver_reset_resume,
		.supports_autosuspend = 1,
#endif
	},
	{
		.name = baseband_usb_driver_name[1],
		.probe = baseband_usb_driver_probe,
		.disconnect = baseband_usb_driver_disconnect,
		.id_table = baseband_usb_driver_id_table[1],
#ifdef CONFIG_PM
		.suspend = baseband_usb_driver_suspend,
		.resume = baseband_usb_driver_resume,
		.reset_resume = baseband_usb_driver_reset_resume,
		.supports_autosuspend = 1,
#endif
	},

	{
		.name = baseband_usb_driver_name[2],
		.probe = baseband_usb_driver_probe,
		.disconnect = baseband_usb_driver_disconnect,
		.id_table = baseband_usb_driver_id_table[2],
#ifdef CONFIG_PM
		.suspend = baseband_usb_driver_suspend,
		.resume = baseband_usb_driver_resume,
		.reset_resume = baseband_usb_driver_reset_resume,
		.supports_autosuspend = 1,
#endif
	},

};

static void find_usb_pipe(struct baseband_usb *usb)
{
	struct usb_device *usbdev = usb->usb.device;
	struct usb_interface *intf = usb->usb.interface;
	unsigned char numendpoint = intf->cur_altsetting->desc.bNumEndpoints;
	struct usb_host_endpoint *endpoint = intf->cur_altsetting->endpoint;
	unsigned char n;

	for (n = 0; n < numendpoint; n++) {
		if (usb_endpoint_is_isoc_in(&endpoint[n].desc)) {
			pr_debug("endpoint[%d] isochronous in\n", n);
			usb->usb.pipe.isoch.in = usb_rcvisocpipe(usbdev,
				endpoint[n].desc.bEndpointAddress);
		} else if (usb_endpoint_is_isoc_out(&endpoint[n].desc)) {
			pr_debug("endpoint[%d] isochronous out\n", n);
			usb->usb.pipe.isoch.out = usb_sndisocpipe(usbdev,
				endpoint[n].desc.bEndpointAddress);
		} else if (usb_endpoint_is_bulk_in(&endpoint[n].desc)) {
			pr_debug("endpoint[%d] bulk in\n", n);
			usb->usb.pipe.bulk.in = usb_rcvbulkpipe(usbdev,
				endpoint[n].desc.bEndpointAddress);
		} else if (usb_endpoint_is_bulk_out(&endpoint[n].desc)) {
			pr_debug("endpoint[%d] bulk out\n", n);
			usb->usb.pipe.bulk.out = usb_sndbulkpipe(usbdev,
				endpoint[n].desc.bEndpointAddress);
		} else if (usb_endpoint_is_int_in(&endpoint[n].desc)) {
			pr_debug("endpoint[%d] interrupt in\n", n);
			usb->usb.pipe.interrupt.in = usb_rcvintpipe(usbdev,
				endpoint[n].desc.bEndpointAddress);
		} else if (usb_endpoint_is_int_out(&endpoint[n].desc)) {
			pr_debug("endpoint[%d] interrupt out\n", n);
			usb->usb.pipe.interrupt.out = usb_sndintpipe(usbdev,
				endpoint[n].desc.bEndpointAddress);
		} else {
			pr_debug("endpoint[%d] skipped\n", n);
		}
	}
}

void baseband_usb_close(struct baseband_usb *usb);

struct baseband_usb *baseband_usb_open(int index,
	unsigned int vid,
	unsigned int pid,
	unsigned int intf)
{
	struct baseband_usb *usb;
	int err;

	save_dbg(dbg_flag);/* HTC */
	pr_debug("baseband_usb_open {\n");

	/* allocate baseband usb structure */
	usb = kzalloc(sizeof(struct baseband_usb),
		GFP_KERNEL);
	if (!usb) {
		restore_dbg(dbg_flag);/* HTC */
		return (struct baseband_usb *) 0;
	}

	/* open usb driver */
	sprintf(baseband_usb_driver_name[index],
		"baseband_usb_%x_%x_%x",
		vid, pid, intf);
	baseband_usb_driver_id_table[index][0].match_flags =
		USB_DEVICE_ID_MATCH_DEVICE;
	baseband_usb_driver_id_table[index][0].idVendor = vid;
	baseband_usb_driver_id_table[index][0].idProduct = pid;
	g_usb_interface_index[index] = intf;
	g_usb_interface[index] = (struct usb_interface *) 0;
	err = usb_register(&baseband_usb_driver[index]);
	if (err < 0) {
		pr_err("cannot open usb driver - err %d\n", err);
		kfree(usb);
		restore_dbg(dbg_flag);/* HTC */
		return (struct baseband_usb *) 0;
	}
	usb->baseband_index = index;
	usb->usb.driver = &baseband_usb_driver[index];
	if (!g_usb_interface[index]) {
		pr_err("cannot open usb driver - !g_usb_interface[%d]\n",
			index);
		usb_deregister(usb->usb.driver);
		kfree(usb);
		restore_dbg(dbg_flag);/* HTC */
		return (struct baseband_usb *) 0;
	}
	usb->usb.device = interface_to_usbdev(g_usb_interface[index]);
	usb->usb.interface = g_usb_interface[index];
	find_usb_pipe(usb);
	usb->usb.rx_urb = (struct urb *) 0;
	usb->usb.tx_urb = (struct urb *) 0;
	g_usb_interface_index[index] = ~0U;
	g_usb_interface[index] = (struct usb_interface *) 0;
	rawlog1("usb->usb.driver->name %s\n", usb->usb.driver->name);
	rawlog1("usb->usb.device %p\n", usb->usb.device);
	rawlog1("usb->usb.interface %p\n", usb->usb.interface);
	rawlog1("usb->usb.pipe.isoch.in %x\n", usb->usb.pipe.isoch.in);
	rawlog1("usb->usb.pipe.isoch.out %x\n", usb->usb.pipe.isoch.out);
	rawlog1("usb->usb.pipe.bulk.in %x\n", usb->usb.pipe.bulk.in);
	rawlog1("usb->usb.pipe.bulk.out %x\n", usb->usb.pipe.bulk.out);
	rawlog1("usb->usb.pipe.interrupt.in %x\n", usb->usb.pipe.interrupt.in);
	rawlog1("usb->usb.pipe.interrupt.out %x\n",
		usb->usb.pipe.interrupt.out);

	pr_debug("baseband_usb_open }\n");
	restore_dbg(dbg_flag);/* HTC */
	return usb;
}

void baseband_usb_close(struct baseband_usb *usb)
{
	save_dbg(dbg_flag);/* HTC */
	pr_debug("baseband_usb_close {\n");

	/* check input */
	if (!usb) {
		restore_dbg(dbg_flag);/* HTC */
		return;
	}

	/* close usb driver */
	if (usb->usb.driver) {
		pr_debug("close usb driver {\n");
		usb_deregister(usb->usb.driver);
		usb->usb.driver = (struct usb_driver *) 0;
		pr_debug("close usb driver }\n");
	}

	/* free baseband usb structure */
	kfree(usb);

	pr_debug("baseband_usb_close }\n");
	restore_dbg(dbg_flag);/* HTC */
}

static int baseband_usb_netdev_init(struct net_device *dev)
{
	pr_debug("baseband_usb_netdev_init\n");
	return 0;
}

static void baseband_usb_netdev_uninit(struct net_device *dev)
{
	pr_debug("baseband_usb_netdev_uninit\n");
}

static int baseband_usb_netdev_open(struct net_device *dev)
{
	pr_debug("baseband_usb_netdev_open\n");
	netif_start_queue(dev);
	return 0;
}

static int baseband_usb_netdev_stop(struct net_device *dev)
{
	pr_debug("baseband_usb_netdev_stop\n");
	netif_stop_queue(dev);
	return 0;
}

static netdev_tx_t baseband_usb_netdev_start_xmit(
	struct sk_buff *skb, struct net_device *dev)
{
	/* 77228-5 patch */
	int i;
	struct baseband_usb *usb;
	/* 77228-5 patch */
	int err;

	//+Sophia:0112
	struct rmnet_private *p = netdev_priv(dev);
	//-Sophia:0112

	pr_debug("baseband_usb_netdev_start_xmit\n");

	/* check input */
	if (!skb) {
		pr_err("no skb\n");
		/* 77969-7 patch */
		return NETDEV_TX_BUSY;
		/* 77969-7 patch */
	}

	/* 77228-5 patch */
	if (!dev) {
		pr_err("no net dev\n");
		/* 77969-7 patch */
		return NETDEV_TX_BUSY;
		/* 77969-7 patch */
	/* 77228-5 patch */
	}
	/* 77228-5 patch */

	/* find index of network device which is transmitting */
	for (i = 0; i < max_intfs; i++) {
		if (usb_net_raw_ip_dev[i] == dev)
			break;
	/* 77228-5 patch */
	}
	/* 77228-5 patch */
	if (i >= max_intfs) {
		pr_err("unknown net dev %p\n", dev);
		/* 77969-7 patch */
		return NETDEV_TX_BUSY;
		/* 77969-7 patch */
	/* 77228-5 patch */
	}
	/* 77228-5 patch */
	usb = baseband_usb_net[i];
	/* 77228-5 patch */

	//+Sophia:0112
  p->stats.tx_packets++;
	p->stats.tx_bytes += skb->len - 14;
	//-Sophia:0112

	if (usb->usb.interface) {
		/* autoresume if suspended */
		usb_autopm_get_interface_async(usb->usb.interface);
	} else{
		pr_err("tx urb submit error\n");
		netif_stop_queue(dev);

		return NETDEV_TX_BUSY;
	}

#if 0
	if (usb->susp_count > 0) {
		//pr_info("%s: usb->susp_count %d > 0 (suspended -> autoresume)\n",__func__, usb->susp_count);
		if (usb_autopm_get_interface_async(usb->usb.interface) >= 0) {
			usb_autopm_put_interface_async(usb->usb.interface);
		}else{
			pr_debug("usb_autopm_get_interface_async failed!\n");

		}
		//return NETDEV_TX_BUSY;
	}
#endif
	/* submit tx urb */
	/* 77228-5 patch */
	err = usb_net_raw_ip_tx_urb_submit(usb, skb);
	/* 77228-5 patch */
	
	if (err < 0) {
		/* 77228-5 patch */
		pr_err("tx urb submit error\n");
		/* 77228-5 patch */
		rawlog4("[ref] -- %s(%d) %d\n", __func__, __LINE__, --autopm_refcnt);
		/*83290-1*/
		netif_stop_queue(dev);
		/*83290-1*/
		/* 77969-5 patch */
		return NETDEV_TX_BUSY;
		/* 77969-5 patch */
	}
	
	return NETDEV_TX_OK;
}

//+Sophia:0112
static struct net_device_stats *rmnet_get_stats(struct net_device *dev)
{
	struct rmnet_private *p = netdev_priv(dev);
	return &p->stats;
}
//-Sophia:0112

static struct net_device_ops usb_net_raw_ip_ops = {
	.ndo_init =		baseband_usb_netdev_init,
	.ndo_uninit =		baseband_usb_netdev_uninit,
	.ndo_open =		baseband_usb_netdev_open,
	.ndo_stop =		baseband_usb_netdev_stop,
	//+Sophia:0112
	.ndo_get_stats = rmnet_get_stats,
	//-Sophia:0112
	.ndo_start_xmit =	baseband_usb_netdev_start_xmit,
};

static int usb_net_raw_ip_rx_urb_submit(struct baseband_usb *usb)
{
	struct urb *urb;
	void *buf;
	int err;

	if (verbose) pr_info("usb_net_raw_ip_rx_urb_submit { usb %p\n", usb);

	/* check input */
	/* 77228-5 patch */
	if (!usb) {
		pr_err("%s: !usb\n", __func__);
		return -EINVAL;
	}
	/* 77228-5 patch */
	
	if (!usb->usb.interface) {
	pr_err("usb interface disconnected - not submitting rx urb\n");
	/* 77969-7 patch */
	return -EINVAL;;
	/* 77969-7 patch */
	}
	/* 77969-7 patch*/
	if (usb->usb.rx_urb) {
		pr_err("previous urb still active\n");
		return -EBUSY;
	/* 77969-7 patch */
	}
	/* 77969-7 patch */
	if (!usb->urb_r || !usb->buff) {
		pr_err("no reusable rx urb found\n");
	/* 77969-7 patch */
		return -ENOMEM;
	}
	
	/* 77969-7 patch */
	/* reuse rx urb */
	urb = usb->urb_r;
	buf = usb->buff;
	/* 77969-7 patch */
	usb_fill_bulk_urb(urb, usb->usb.device, usb->usb.pipe.bulk.in,
		buf, USB_NET_BUFSIZ,
		usb_net_raw_ip_rx_urb_comp,
		usb);
	urb->transfer_flags = 0;

	/* submit rx urb */
	usb_mark_last_busy(usb->usb.device);
	usb->usb.rx_urb = urb;
	err = usb_submit_urb(urb, GFP_ATOMIC);
	if (err < 0) {
		pr_err("usb_submit_urb() failed - err %d\n", err);
		usb->usb.rx_urb = (struct urb *) 0;
		return err;
	}

	if (verbose) pr_info("usb_net_raw_ip_rx_urb_submit }\n");
	return err;
}

static void usb_net_raw_ip_rx_urb_comp(struct urb *urb)
{
	struct baseband_usb *usb = (struct baseband_usb *) urb->context;
	int i = usb->baseband_index;

	//+Sophia:0112
	struct net_device *dev = (struct net_device *) usb_net_raw_ip_dev[i];
	struct rmnet_private *p = netdev_priv(dev);
	//-Sophia:0112


	struct sk_buff *skb;
	unsigned char *dst;
	unsigned char ethernet_header[14] = {
		/* Destination MAC */
		0x00, 0x00,
		0x00, 0x00,
		0x00, 0x00,
		/* Source MAC */
		0x00, 0x00,
		0x00, 0x00,
		0x00, 0x00,
		/* EtherType */
		NET_IP_ETHERTYPE,
	};

	if (verbose) pr_info("usb_net_raw_ip_rx_urb_comp { urb %p\n", urb);

	/* check input */
	if (!urb) {
		pr_err("no urb\n");
		return;
	}
	/* 77969-8 patch */
	switch (urb->status) {
	case 0:
		break;
	case -ESHUTDOWN:
		/* fall through */
		pr_info("%s: rx urb %p - link shutdown %d\n",
			__func__, urb, urb->status);
		goto err_exit;
	case -EPROTO:
		pr_info("%s: rx urb %p - link shutdown %d EPROTO\n",
			__func__, urb, urb->status);
		usb_register_dump();
		goto err_exit;
	default:
		pr_info("%s: rx urb %p - status %d\n",
			__func__, urb, urb->status);
		break;
 	}
	/* 77969-8 patch */
	/* put rx urb data in rx buffer */
	if (urb->actual_length) {
		pr_debug("usb_net_raw_ip_rx_urb_comp - "
			"urb->actual_length %d\n", urb->actual_length);
		/* allocate skb with space for
		 * - dummy ethernet header
		 * - rx IP packet from modem
		 */
		skb = netdev_alloc_skb(usb_net_raw_ip_dev[i],
			NET_IP_ALIGN + 14 + urb->actual_length);
		if (skb) {
			/* generate a dummy ethernet header
			 * since modem sends IP packets without
			 * any ethernet headers
			 */
			memcpy(ethernet_header + 0,
				usb_net_raw_ip_dev[i]->dev_addr, 6);
			memcpy(ethernet_header + 6,
				"0x01\0x02\0x03\0x04\0x05\0x06", 6);
			/* fill skb with
			 * - dummy ethernet header
			 * - rx IP packet from modem
			 */
			skb_reserve(skb, NET_IP_ALIGN);
			dst = skb_put(skb, 14);
			memcpy(dst, ethernet_header, 14);
			dst = skb_put(skb, urb->actual_length);
			memcpy(dst, urb->transfer_buffer, urb->actual_length);
			//htc
			if (host_dbg_flag & DBG_RAWIP_L1){
				rawip_print_buffdata(urb->transfer_buffer, urb->actual_length,1);
			}

			//+Sophia:0112
			p->stats.rx_packets++;
			p->stats.rx_bytes += urb->actual_length;
			//-Sophia:0112

			skb->protocol = eth_type_trans(skb,
				usb_net_raw_ip_dev[i]);
			/* pass skb to network stack */
			if (netif_rx(skb) < 0) {
				pr_err("usb_net_raw_ip_rx_urb_comp_work - "
					"netif_rx(%p) failed\n", skb);
				kfree_skb(skb);
			}
		} else {
			pr_err("usb_net_raw_ip_rx_urb_comp_work - "
				"netdev_alloc_skb() failed\n");
		}
	}

	/* mark rx urb complete */
	usb->usb.rx_urb = (struct urb *) 0;

	/* submit next rx urb */
	usb_net_raw_ip_rx_urb_submit(usb);
/* 77969-7 patch */
	return;

err_exit:
	/* mark rx urb complete */
	usb->usb.rx_urb = (struct urb *) 0;

	if (verbose) pr_info("usb_net_raw_ip_rx_urb_comp }\n");
	return;
/* 77969-7 patch */
}

/* 77969-7 patch */
static int usb_net_raw_ip_setup_rx_urb( struct baseband_usb *usb)
{
	pr_debug("usb_net_raw_ip_setup_rx_urb {\n");

	/* check input */
	if (!usb) {
		pr_err("%s: !usb\n", __func__);
		return -EINVAL;
	}
	if (usb->urb_r) {
		pr_err("%s: reusable rx urb already allocated\n", __func__);
		return -EINVAL;
	}

	/* allocate reusable rx urb */
	usb->urb_r = usb_alloc_urb(0, GFP_ATOMIC);
	if (!usb->urb_r) {
		pr_err("usb_alloc_urb() failed\n");
		return -ENOMEM;
	}
	usb->buff = kzalloc(USB_NET_BUFSIZ, GFP_ATOMIC);
	if (!usb->buff) {
		pr_err("usb buffer kzalloc() failed\n");
		usb_free_urb(usb->urb_r);
		usb->urb_r = (struct urb *) 0;
		return -ENOMEM;
	}

	pr_debug("usb_net_raw_setup_ip_rx_urb }\n");
	return 0;
}

static void usb_net_raw_ip_free_rx_urb(struct baseband_usb *usb)
{
	pr_debug("usb_net_raw_ip_free_rx_urb {\n");

	/* check input */
	if (!usb) {
		pr_err("%s: !usb\n", __func__);
		return;
	}

	/* free reusable rx urb */
	if (usb->urb_r) {
		usb_free_urb(usb->urb_r);
		usb->urb_r = (struct urb *) 0;
	}
	if (usb->buff) {
		kfree(usb->buff);
		usb->buff = (void *) 0;
	}

	pr_debug("usb_net_raw_ip_free_rx_urb }\n");
}
/* 77969-7 patch */
 
/* 77228-5 patch */
static int usb_net_raw_ip_tx_urb_submit(struct baseband_usb *usb,
	struct sk_buff *skb)
{
	struct urb *urb;
	unsigned char *buf;
	int err;
	
	if (verbose) pr_info("usb_net_raw_ip_tx_urb_submit {\n");
	
	/* check input */
	if (!usb) {
		pr_err("%s: !usb\n", __func__);
		return -EINVAL;
	}

	/* 77969-7 patch */
	if (!usb->usb.interface) {
		pr_err("usb interface disconnected - not submitting tx urb\n");
		return -EINVAL;
	}
	/* 77969-7 patch */
	
	if (!skb) {
		 pr_err("%s: !skb\n", __func__);
		 usb_autopm_put_interface_async(usb->usb.interface);
		return -EINVAL;
	}
	/* allocate urb */
	urb = usb_alloc_urb(0, GFP_ATOMIC);
	if (!urb) {
		pr_err("usb_alloc_urb() failed\n");
		usb_autopm_put_interface_async(usb->usb.interface);
		return -ENOMEM;
	}
	buf = kzalloc(skb->len - 14, GFP_ATOMIC);
	if (!buf) {
		pr_err("usb buffer kzalloc() failed\n");
		usb_free_urb(urb);
		usb_autopm_put_interface_async(usb->usb.interface);
		return -ENOMEM;
	}
	err = skb_copy_bits(skb, 14, buf, skb->len - 14);
	if (err < 0) {
		pr_err("skb_copy_bits() failed - %d\n", err);
		kfree(buf);
		usb_free_urb(urb);
		usb_autopm_put_interface_async(usb->usb.interface);
		return err;
	}
	usb_fill_bulk_urb(urb, usb->usb.device, usb->usb.pipe.bulk.out,
		buf, skb->len - 14,
		usb_net_raw_ip_tx_urb_comp,
		usb);

	urb->transfer_flags = URB_ZERO_PACKET;

	/* queue tx urb work */
	usb_anchor_urb(urb, &usb->usb.tx_urb_deferred);
	queue_work(usb->usb.tx_workqueue, &usb->usb.tx_work);

	/* free skb */
	consume_skb(skb);
	
	if (verbose) pr_info("usb_net_raw_ip_tx_urb_submit }\n");
	return 0;
}
	
static void usb_net_raw_ip_tx_urb_work(struct work_struct *work)
{
	struct baseband_usb *usb
		 = container_of(work, struct baseband_usb, usb.tx_work);
	struct urb *urb;
	int err;
	
	if (verbose) pr_info("usb_net_raw_ip_tx_urb_work {\n");
	
	/* check if tx urb(s) queued */
	if (!usb->usb.tx_urb && usb_anchor_empty(&usb->usb.tx_urb_deferred)) {
		pr_debug("%s: nothing to do!\n", __func__);
		return;
	}

	/* check if suspended */
	if (usb->susp_count > 0) {
		pr_info("%s: usb->susp_count %d > 0 (suspended)\n",
			__func__, usb->susp_count);
		return;
	}

	/* submit queued tx urb(s) */
	while ((urb = usb_get_from_anchor(&usb->usb.tx_urb_deferred))
		!= (struct urb *) 0) {
		/* 77969-7 patch */
		/* decrement count from usb_get_from_anchor() */
		usb_free_urb(urb);
		/* check if usb interface disconnected */
		if (!usb->usb.interface) {
			pr_err("%s: not submitting tx urb %p"
				" - interface disconnected\n",
				__func__, urb);
			if (urb->transfer_buffer) {
				kfree(urb->transfer_buffer);
				urb->transfer_buffer = (void *) 0;
			}
			usb_free_urb(urb);
			continue;
		}
		/* 77969-7 patch */
		/* autoresume before tx */
		usb_mark_last_busy(usb->usb.device);
#if 0
		err = usb_autopm_get_interface(usb->usb.interface);
		if (err < 0) {
			pr_err("%s: usb_autopm_get_interface(%p) failed %d\n",
				__func__, usb->usb.interface, err);
		/* 77969-7 patch */
			if (urb->transfer_buffer) {
				kfree(urb->transfer_buffer);
				urb->transfer_buffer = (void *) 0;
			}

		/* 77969-7 patch */
			usb_free_urb(urb);
			continue;
		}
#endif

		/* submit tx urb */
		err = usb_submit_urb(urb, GFP_ATOMIC);
		if (err < 0) {
			/* 77969-7 patch */
			pr_err("%s: usb_submit_urb(%p) failed - err %d\n",
				__func__, urb, err);
			/* 77969-7 patch */
			//usb_autopm_put_interface(usb->usb.interface);
			usb_autopm_put_interface_async(usb->usb.interface);
			/* 77969-7 patch */
			if (urb->transfer_buffer) {
				kfree(urb->transfer_buffer);
				urb->transfer_buffer = (void *) 0;
			}
			/* 77969-7 patch */
			usb_free_urb(urb);
			continue;
		}
		/* 77969-7 patch  */
		/* free tx urb
		 * - actual urb free occurs when refcnt which was incremented
		 *   in usb_submit_urb is decremented to 0 (usually after urb
		 *   completion function returns)
		 * - tx urb transfer buffer will be freed in urb completion
		 *   function
		*/
		usb_free_urb(urb);
		/* 77969-7 patch */
	}
	
	if (verbose) pr_info("usb_net_raw_ip_tx_urb_work }\n");
}
/* 77228-5 patch */

static void usb_net_raw_ip_tx_urb_comp(struct urb *urb)
{
	struct baseband_usb *usb = (struct baseband_usb *) urb->context;

	if (verbose) pr_debug("usb_net_raw_ip_tx_urb_comp {\n");
	/* 77969-7 patch */
	/* check input */
	if (!urb) {
		pr_err("no urb\n");
		return;
	}
	/* 77969-8 patch */
	switch (urb->status) {
	case 0:
		break;
	case -ENOENT:
		/* fall through */
	case -ESHUTDOWN:
		/* fall through */
	case -EPROTO:
		pr_info("%s: rx urb %p - link shutdown %d\n",
			__func__, urb, urb->status);
		usb_autopm_put_interface_async(usb->usb.interface);
		goto err_exit;
	default:
		pr_info("%s: rx urb %p - status %d\n",
			__func__, urb, urb->status);
		break;
 	}
	/* 77969-8 patch */
	if (urb->status) {
		pr_info("tx urb status %d\n", urb->status);
	/* 77969-7 patch */
	}
	
	/* autosuspend after tx completed */
	/* 77969-7 patch */
	if (!usb->usb.interface) {
		pr_err("%s: usb interface disconnected"
			" before tx urb completed!\n",
			__func__);
		goto err_exit;
	}
	/* 77969-7 patch */
	rawlog4("[ref] -- %s(%d) %d\n", __func__, __LINE__, --autopm_refcnt);
	/* 77228-5 patch */
	//usb_autopm_put_interface(usb->usb.interface);
	usb_autopm_put_interface_async(usb->usb.interface);
	/* 77228-5 patch */

	/* 77969-7 patch */
err_exit:
	/* free tx urb transfer buffer */
	if (urb->transfer_buffer) {
		kfree(urb->transfer_buffer);
		urb->transfer_buffer = (void *) 0;
	}
	/* 77969-7 patch */
	rawlog4("[ref] -- %s(%d) %d\n", __func__, __LINE__, --autopm_refcnt);
	if (verbose) pr_info("usb_net_raw_ip_tx_urb_comp }\n");
}

static int usb_net_raw_ip_init(void)
{
	int i;
	int err;
	/* 77228-5 patch */
	char name[32];
	/* 77228-5 patch */
	pr_info("usb_net_raw_ip_init {\n");

	/* create multiple raw-ip network devices */
	for (i = 0; i < max_intfs; i++) {
		/* open baseband usb */
		g_i = i;
		baseband_usb_net[i] = baseband_usb_open(i, usb_net_raw_ip_vid,
			usb_net_raw_ip_pid, usb_net_raw_ip_intf[i]);
		if (!baseband_usb_net[i]) {
			pr_err("cannot open baseband usb net\n");
			err = -1;
			goto error_exit;
		}
		init_usb_anchor(&baseband_usb_net[i]->usb.tx_urb_deferred);
		/* register network device */
		usb_net_raw_ip_dev[i] = alloc_netdev(0,
			BASEBAND_USB_NET_DEV_NAME,
			ether_setup);
		if (!usb_net_raw_ip_dev[i]) {
			pr_err("alloc_netdev() failed\n");
			err = -ENOMEM;
			goto error_exit;
		}
		usb_net_raw_ip_dev[i]->netdev_ops = &usb_net_raw_ip_ops;
		usb_net_raw_ip_dev[i]->watchdog_timeo = TX_TIMEOUT;
		random_ether_addr(usb_net_raw_ip_dev[i]->dev_addr);
		err = register_netdev(usb_net_raw_ip_dev[i]);
		if (err < 0) {
			pr_err("cannot register network device - %d\n", err);
			goto error_exit;
		}
		pr_info("registered baseband usb network device"
				" - dev %p name %s\n", usb_net_raw_ip_dev[i],
				 BASEBAND_USB_NET_DEV_NAME);
		/* start usb rx */
		/* 77969-7 patch */
		err = usb_net_raw_ip_setup_rx_urb(baseband_usb_net[i]);
		if (err < 0) {
			pr_err("setup reusable rx urb failed - err %d\n", err);
			goto error_exit;
		}
		/* 77969-7 patch */
		err = usb_net_raw_ip_rx_urb_submit(baseband_usb_net[i]);
		if (err < 0) {
			pr_err("submit rx failed - err %d\n", err);
			goto error_exit;
		}
		/* 77228-5 patch */
		/* start usb tx */
		/* 77969-7 patch */
		sprintf(name, "raw_ip_tx_wq-%d",
		/* 77969-7 patch */
			baseband_usb_net[i]->baseband_index);
		baseband_usb_net[i]->usb.tx_workqueue
			= create_singlethread_workqueue(name);
		if (!baseband_usb_net[i]->usb.tx_workqueue) {
			pr_err("cannot create workqueue\n");
			goto error_exit;
		}
		INIT_WORK(&baseband_usb_net[i]->usb.tx_work,
			usb_net_raw_ip_tx_urb_work);
		/* 77228-5 patch */
	}

	/* HTC */
	pr_info(MODULE_NAME ": %s - host_dbg_flag=0x%x\n",
		__func__, host_dbg_flag);

	pr_info("usb_net_raw_ip_init }\n");
	return 0;

error_exit:
	/* destroy multiple raw-ip network devices */
	for (i = 0; i < max_intfs; i++) {
		/* unregister network device */
		if (usb_net_raw_ip_dev[i]) {
			unregister_netdev(usb_net_raw_ip_dev[i]);
			free_netdev(usb_net_raw_ip_dev[i]);
			usb_net_raw_ip_dev[i] = (struct net_device *) 0;
		}
		/* close baseband usb */
		if (baseband_usb_net[i]) {
			/* 77228-5 patch */
			/* stop usb tx */
			if (baseband_usb_net[i]->usb.tx_workqueue) {
				destroy_workqueue(baseband_usb_net[i]
					->usb.tx_workqueue);
				baseband_usb_net[i]->usb.tx_workqueue
					= (struct workqueue_struct *) 0;
			}
			if (baseband_usb_net[i]->usb.tx_urb) {
				usb_kill_urb(baseband_usb_net[i]->usb.tx_urb);
				baseband_usb_net[i]->usb.tx_urb
					= (struct urb *) 0;
			}
			/* stop usb rx */
			if (baseband_usb_net[i]->usb.rx_urb) {
				usb_kill_urb(baseband_usb_net[i]->usb.rx_urb);
				baseband_usb_net[i]->usb.rx_urb
					= (struct urb *) 0;
			}
			/* 77969-7 patch */
			usb_net_raw_ip_free_rx_urb(baseband_usb_net[i]);
			/* 77969-7 patch */
			/* close usb */
			/* 77228-5 patch */
			baseband_usb_close(baseband_usb_net[i]);
			baseband_usb_net[i] = (struct baseband_usb *) 0;
		}
	}

	return err;
}

static void usb_net_raw_ip_exit(void)
{
	int i;

	pr_info("usb_net_raw_ip_exit {\n");

	/* destroy multiple raw-ip network devices */
	for (i = 0; i < max_intfs; i++) {
		/* unregister network device */
		if (usb_net_raw_ip_dev[i]) {
			unregister_netdev(usb_net_raw_ip_dev[i]);
			free_netdev(usb_net_raw_ip_dev[i]);
			usb_net_raw_ip_dev[i] = (struct net_device *) 0;
		}
		/* close baseband usb */
		if (baseband_usb_net[i]) {
			/* 77228-5 patch */
			/* stop usb tx */
			if (baseband_usb_net[i]->usb.tx_workqueue) {
				destroy_workqueue(baseband_usb_net[i]
					->usb.tx_workqueue);
				baseband_usb_net[i]->usb.tx_workqueue
					= (struct workqueue_struct *) 0;
			}
			if (baseband_usb_net[i]->usb.tx_urb) {
				usb_kill_urb(baseband_usb_net[i]->usb.tx_urb);
				baseband_usb_net[i]->usb.tx_urb
					= (struct urb *) 0;
			}
			/* stop usb rx */
			if (baseband_usb_net[i]->usb.rx_urb) {
				usb_kill_urb(baseband_usb_net[i]->usb.rx_urb);
				baseband_usb_net[i]->usb.rx_urb
					= (struct urb *) 0;
			}
			/* 77969-7 patch*/
			usb_net_raw_ip_free_rx_urb(baseband_usb_net[i]);
			/* 77969-7 patch*/
			/* close usb */
			/* 77228-5 patch */
			baseband_usb_close(baseband_usb_net[i]);
			baseband_usb_net[i] = (struct baseband_usb *) 0;
		}
	}

	pr_info("usb_net_raw_ip_exit }\n");
}

module_init(usb_net_raw_ip_init)
module_exit(usb_net_raw_ip_exit)

