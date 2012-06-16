/*
 * cdc-acm.c
 *
 * Copyright (c) 1999 Armin Fuerst	<fuerst@in.tum.de>
 * Copyright (c) 1999 Pavel Machek	<pavel@ucw.cz>
 * Copyright (c) 1999 Johannes Erdfelt	<johannes@erdfelt.com>
 * Copyright (c) 2000 Vojtech Pavlik	<vojtech@suse.cz>
 * Copyright (c) 2004 Oliver Neukum	<oliver@neukum.name>
 * Copyright (c) 2005 David Kubicek	<dave@awk.cz>
 *
 * USB Abstract Control Model driver for USB modems and ISDN adapters
 *
 * Sponsored by SuSE
 *
 * ChangeLog:
 *	v0.9  - thorough cleaning, URBification, almost a rewrite
 *	v0.10 - some more cleanups
 *	v0.11 - fixed flow control, read error doesn't stop reads
 *	v0.12 - added TIOCM ioctls, added break handling, made struct acm
 *		kmalloced
 *	v0.13 - added termios, added hangup
 *	v0.14 - sized down struct acm
 *	v0.15 - fixed flow control again - characters could be lost
 *	v0.16 - added code for modems with swapped data and control interfaces
 *	v0.17 - added new style probing
 *	v0.18 - fixed new style probing for devices with more configurations
 *	v0.19 - fixed CLOCAL handling (thanks to Richard Shih-Ping Chan)
 *	v0.20 - switched to probing on interface (rather than device) class
 *	v0.21 - revert to probing on device for devices with multiple configs
 *	v0.22 - probe only the control interface. if usbcore doesn't choose the
 *		config we want, sysadmin changes bConfigurationValue in sysfs.
 *	v0.23 - use softirq for rx processing, as needed by tty layer
 *	v0.24 - change probe method to evaluate CDC union descriptor
 *	v0.25 - downstream tasks paralelized to maximize throughput
 *	v0.26 - multiple write urbs, writesize increased
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

/** HTC: Modification History
 *	02: 2011-10-07 02:15 bert_lin - fix modem download issue
 *	03: 2011-10-08 14:15 bert_lin - add timer to print out statistics
 *		support ble flashless only. (flash not supported yet)
 *	04: 2011-10-12 20:55 bert_lin - add timer to print out statistics
 *		project support: edge
 *	05: 2011-10-14 03:44 bert_lin - add READ_WRITE_LOG debugging flag
 *		enable read/write logs by this definition
 *	06: 2011-10-20 21:51 bert_lin - delete timer when acm disconnect.
 *	07: 2011-10-20 23:10 bert_lin - add runtime debugging mechanism
 *	08: 2011-12-06 12:04 bert_lin - solve the MO call issue
 *		AP L2->L0 fail - runtime pm workaround
 *	09: 2011-12-14 15:10 bert_lin - 69327-2 patch
 *	10: 2011-12-21 bert_lin - add acm rw logs
 *	11: 2011-12-22 bert_lin - add acm susp_count logs
 *	12: 2011-12-28 bert_lin - add DBG_ACM1_RW flag for ttyACM1 debugging
 *	13: 2011-01-03 bert_lin - at timeout issue, prefix was sent twice
 *		rerrange usb anchor flow. 
 *	14:	2012-01-18 	at-command timeout due to process buffer sequence wrong when L2->L0
 */
/*
#define RIL_DBG
*/
#undef DEBUG
#undef VERBOSE_DEBUG
#define MODULE_NAME "[ACM14]"
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/tty.h>
#include <linux/serial.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/uaccess.h>
#include <linux/usb.h>
#include <linux/usb/cdc.h>
#include <asm/byteorder.h>
#include <asm/unaligned.h>
#include <asm/cacheflush.h>
#include <linux/list.h>

/* HTC include file */
#include <mach/htc_hostdbg.h>
#include <asm/cacheflush.h>
#include <htc/log.h>

#include "cdc-acm.h"
/* HTC: PRINTRPT */
#include <mach/htc_ril_misc.h>
#include <linux/ctype.h>

#define ACM_CLOSE_TIMEOUT	15	/* seconds to let writes drain */

/* HTC: usage cnt checking */
static int autopm_refcnt = 0;

/* HTC: TP statistic variables */
#define MAX_ACM_NUM 1 /* bert: currently support for ttyACM0 only */
static int is_mach_ble = 0;
static int is_flashless = 0;
static int is_mach_edge = 0;
static int mach_ble_flashless = 0;
static int mach_ble_flash = 0;
const int ENDIDX = 16; /* the size when we try to print out raw data */
const int timer_value_ms = 6000;
struct acm_tp {
	int pre_tx_total;
	int pre_rx_total;
	int tx_total;
	int rx_total;
};
static struct acm_tp acm_tp[MAX_ACM_NUM];
struct timer_list acm_timer;
static int timer_on = 0;
#define is_minor_valid(m) (m >= 0 && m < MAX_ACM_NUM)? 1 : 0

/* HTC: avoid QUO build break,
	extern symbol because QUO_F doesn't have xmm-power */
#ifdef CONFIG_MACH_QUATTRO_F
unsigned int host_dbg_flag = 0;
EXPORT_SYMBOL(host_dbg_flag);
#else
extern unsigned int host_dbg_flag;
#endif

/* HTC: TP statistic functions*/
static void timer_handler(unsigned long data);
static void start_timer(void);

static int max_intfs = 2;
unsigned static int txbyte=0,rxbyte=0;
char gpr_buf[512];
static int pcount=0;
static int debugtestcoung=0; 
module_param(max_intfs, int, 0644);
MODULE_PARM_DESC(max_intfs, "usb class (cdc-acm) - Number of TTYACMs");

module_param(txbyte, uint, 0644);
MODULE_PARM_DESC(txbyte, "txbyte");
module_param(rxbyte, uint, 0644);
MODULE_PARM_DESC(rxbyte, "rxbyte");


/* HTC: debug flag */
#define rwlog(x...) do {\
	if (host_dbg_flag & DBG_ACM0_RW) \
		pr_info(x);\
} while (0)

#define acm1log(x...) do {\
	if (host_dbg_flag & DBG_ACM1_RW) \
		pr_info(x);\
} while (0)

#define reflog(x...) do {\
    if (host_dbg_flag & DBG_ACM_REFCNT) \
        pr_info(x);\
} while (0)

/*
 * Version Information
 */
#define DRIVER_VERSION "v0.26"
#define DRIVER_AUTHOR "Armin Fuerst, Pavel Machek, Johannes Erdfelt, Vojtech Pavlik, David Kubicek"
#define DRIVER_DESC "USB Abstract Control Model driver for USB modems and ISDN adapters"

static struct usb_driver acm_driver;
static struct tty_driver *acm_tty_driver;
static struct acm *acm_table[ACM_TTY_MINORS];
static bool acm_ready_table[ACM_TTY_MINORS];

static DEFINE_MUTEX(open_mutex);

#define ACM_READY(acm)	(acm && acm->dev && acm->port.count)

static void acm_write_bulk(struct urb *urb);
static const struct tty_port_operations acm_port_ops = {
};

//#ifdef VERBOSE_DEBUG
//#define verbose	1
//#else
//#define verbose	0
//#endif
static int verbose = 0;
static int errstatuscount=0;
module_param(verbose, int, 0644);
MODULE_PARM_DESC(verbose, "usb class (cdc-acm) - verbose debug messages");
extern void usb_register_dump(void);


/* HTC: timer_handler - print out statistic for TP */
static void timer_handler(unsigned long data)
{
	int diff_tx=0, diff_rx=0;
	int i;

	/* loop all channels of ttyACM */
	for (i=0; i<MAX_ACM_NUM; i++) {

		diff_tx = acm_tp[i].tx_total - acm_tp[i].pre_tx_total;
		diff_rx = acm_tp[i].rx_total - acm_tp[i].pre_rx_total;

		/* 1. print out total packets and TP in every n mseconds,
			don't update if no changes */
		if (0 != diff_tx && 0 !=diff_rx) {
			PRINTRTC_PRE;
			printk("ttyACM[%d] tx_total=%d, rx_total=%d, "
				"TP in %dmseconds tx=%d, rx=%d\n",
				i, acm_tp[i].tx_total, acm_tp[i].rx_total,
				timer_value_ms, diff_tx, diff_rx);
		}

		/* 2. update pre_tx and pre_rx */
		acm_tp[i].pre_tx_total = acm_tp[i].tx_total;
		acm_tp[i].pre_rx_total = acm_tp[i].rx_total;
	}

	/* 3. start another timer */
	start_timer();
}

/* HTC: start timer every timer_value_ms m_seconds */
static void start_timer(void)
{
	acm_timer.expires = jiffies + msecs_to_jiffies(timer_value_ms);
	add_timer(&acm_timer);
}

/*
 * Functions for ACM control messages.
 */

static int acm_ctrl_msg(struct acm *acm, int request, int value,
							void *buf, int len)
{
	int retval = usb_control_msg(acm->dev, usb_sndctrlpipe(acm->dev, 0),
		request, USB_RT_ACM, value,
		acm->control->altsetting[0].desc.bInterfaceNumber,
		buf, len, 5000);
	if (verbose) pr_info("acm_control_msg: rq: 0x%02x val: %#x len: %#x result: %d",
						request, value, len, retval);
	return retval < 0 ? retval : 0;
}

/* devices aren't required to support these requests.
 * the cdc acm descriptor tells whether they do...
 */
#define acm_set_control(acm, control) \
	acm_ctrl_msg(acm, USB_CDC_REQ_SET_CONTROL_LINE_STATE, control, NULL, 0)
#define acm_set_line(acm, line) \
	acm_ctrl_msg(acm, USB_CDC_REQ_SET_LINE_CODING, 0, line, sizeof *(line))
#define acm_send_break(acm, ms) \
	acm_ctrl_msg(acm, USB_CDC_REQ_SEND_BREAK, ms, NULL, 0)

/*
 * Write buffer management.
 * All of these assume proper locks taken by the caller.
 */

static int acm_wb_alloc(struct acm *acm)
{
	int i, wbn;
	struct acm_wb *wb;

	wbn = 0;
	i = 0;
	for (;;) {
		wb = &acm->wb[wbn];
		if (!wb->use) {
			wb->use = 1;
			return wbn;
		}
		wbn = (wbn + 1) % ACM_NW;
		if (++i >= ACM_NW) {
			pr_err("%s: ttyACM%d out of WB buffers\n", __func__,acm->minor);
			return -1;
			}
	}
}

static int acm_wb_is_avail(struct acm *acm)
{
	int i, n;
	unsigned long flags;

	n = ACM_NW;
	spin_lock_irqsave(&acm->write_lock, flags);
	for (i = 0; i < ACM_NW; i++)
		n -= acm->wb[i].use;
	spin_unlock_irqrestore(&acm->write_lock, flags);
	if (verbose) pr_info("%s: n %d\n", __func__, n);
	return n;
}

/*
 * Finish write. Caller must hold acm->write_lock
 */
static void acm_write_done(struct acm *acm, struct acm_wb *wb)
{
	wb->use = 0;
	acm->transmitting--;
	if (verbose) ("[ref] - %s(%d) %d\n", __func__, __LINE__, --autopm_refcnt);
	reflog("[ref] - %s(%d) %d\n", __func__, __LINE__, --autopm_refcnt);
	usb_autopm_put_interface_async(acm->control);
}

/*
 * Poke write.
 *
 * the caller is responsible for locking
 */

static int acm_start_wb(struct acm *acm, struct acm_wb *wb, char *func_name)
{
	int rc;

	if (verbose) pr_info("%s: func_name %s\n", __func__, func_name);


	acm->transmitting++;

	wb->urb->transfer_buffer = wb->buf;
	wb->urb->transfer_dma = wb->dmah;
	wb->urb->transfer_buffer_length = wb->len;
	wb->urb->dev = acm->dev;

	/* HTC */
	if (unlikely(0 == strcmp("acm_resume", func_name)))
		pr_info(MODULE_NAME "%s - acm_start_wb %s %p len=%d [%02x]\n",
			__func__, func_name, wb->buf, wb->len, wb->buf[0]);

	/* flush tx buffer from cache */
	if (wb->len)
		dmac_flush_range(wb->buf, wb->buf + wb->len -1);

	rc = usb_submit_urb(wb->urb, GFP_ATOMIC);
	if (rc < 0) {
		pr_info("usb_submit_urb(write bulk) rc: %d", rc); /* HTC: change log */
		acm_write_done(acm, wb);
	}
	return rc;
}

static int acm_write_start(struct acm *acm, int wbn)
{
	unsigned long flags;
	struct acm_wb *wb = &acm->wb[wbn];
	int rc;
#ifdef CONFIG_PM
    struct urb *res;
#endif

	if (verbose) pr_info("%s: wbn %d\n", __func__, wbn);

	spin_lock_irqsave(&acm->write_lock, flags);
	if (!acm->dev) {
		wb->use = 0;
		spin_unlock_irqrestore(&acm->write_lock, flags);
		pr_info(MODULE_NAME "%s fail - !acm->dev\n", __func__);
		return -ENODEV;
	}

	if (verbose) pr_info("%s susp_count: %d", __func__, acm->susp_count);
	if (verbose) pr_info("[ref] + %s(%d) %d\n", __func__, __LINE__, ++autopm_refcnt);
	dbg("%s susp_count: %d", __func__, acm->susp_count);
	reflog("[ref] + %s(%d) %d\n", __func__, __LINE__, ++autopm_refcnt);
	usb_autopm_get_interface_async(acm->control);

	if (acm->susp_count) {
#ifdef CONFIG_PM
		sp_pr_debug("%s buffer urb %p len=%d, [%02x] anchor urb=%p\n",
			__func__, wb->buf, wb->len, wb->buf[0], wb->urb);
		acm->transmitting++;
		wb->urb->transfer_buffer = wb->buf;
		wb->urb->transfer_dma = wb->dmah;
		wb->urb->transfer_buffer_length = wb->len;
		wb->urb->dev = acm->dev;
		//printk("len %d\n", wb->urb->transfer_buffer_length);
		usb_anchor_urb(wb->urb, &acm->deferred);
#else
		if (!acm->delayed_wb) {
			acm->delayed_wb = wb;
			/* HTC */
			sp_pr_debug(MODULE_NAME "%s - delayed wb, ttyACM%d, susp_cnt=%d, "
				"%p len=%d, [%02x]\n",
				__func__, acm->minor, acm->susp_count, wb->buf, wb->len, wb->buf[0]);
		} else {
			if (verbose) pr_info("[ref] - %s(%d) %d\n", __func__, __LINE__, --autopm_refcnt);
			reflog("[ref] - %s(%d) %d\n", __func__, __LINE__, --autopm_refcnt);
			usb_autopm_put_interface_async(acm->control);
		}
#endif

		spin_unlock_irqrestore(&acm->write_lock, flags);
		return 0;	/* A white lie */
	}
	usb_mark_last_busy(acm->dev);

#ifdef CONFIG_PM
	while ((res = usb_get_from_anchor(&acm->deferred))) {
		/* decrement ref count*/
		usb_put_urb(res);
		if (verbose) pr_info("%s: usb_get_from_anchor %p\n", __func__, res);
		rc = usb_submit_urb(res, GFP_ATOMIC);
		if (rc < 0) {
			dbg("usb_submit_urb(pending request) failed: %d", rc);
			usb_unanchor_urb(res);
			acm_write_done(acm, res->context);
		}
	}
#endif
	rc = acm_start_wb(acm, wb, __func__);
	spin_unlock_irqrestore(&acm->write_lock, flags);

	return rc;

}

/*
 * attributes exported through sysfs
 */
static ssize_t show_caps
(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct usb_interface *intf = to_usb_interface(dev);
	struct acm *acm = usb_get_intfdata(intf);

	return sprintf(buf, "%d", acm->ctrl_caps);
}
static DEVICE_ATTR(bmCapabilities, S_IRUGO, show_caps, NULL);

static ssize_t show_country_codes
(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct usb_interface *intf = to_usb_interface(dev);
	struct acm *acm = usb_get_intfdata(intf);

	memcpy(buf, acm->country_codes, acm->country_code_size);
	return acm->country_code_size;
}

static DEVICE_ATTR(wCountryCodes, S_IRUGO, show_country_codes, NULL);

static ssize_t show_country_rel_date
(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct usb_interface *intf = to_usb_interface(dev);
	struct acm *acm = usb_get_intfdata(intf);

	return sprintf(buf, "%d", acm->country_rel_date);
}

static DEVICE_ATTR(iCountryCodeRelDate, S_IRUGO, show_country_rel_date, NULL);

/*
 * Interrupt handlers for various ACM device responses
 */

/* control interface reports status changes with "interrupt" transfers */
static void acm_ctrl_irq(struct urb *urb)
{
	struct acm *acm = urb->context;
	struct usb_cdc_notification *dr = urb->transfer_buffer;
	struct tty_struct *tty;
	unsigned char *data;
	int newctrl;
	int retval;
	int status = urb->status;

	switch (status) {
	case 0:
		/* success */
		break;
	case -ECONNRESET:
	case -ENOENT:
	case -ESHUTDOWN:
		/* this urb is terminated, clean up */
		if (verbose) pr_info("%s - urb shutting down with status: %d", __func__, status);
		dbg("%s - urb shutting down with status: %d", __func__, status);
		return;
	default:
		if (verbose) pr_info("%s - nonzero urb status received: %d", __func__, status);
		dbg("%s - nonzero urb status received: %d", __func__, status);
		goto exit;
	}

	if (!ACM_READY(acm))
		goto exit;

	data = (unsigned char *)(dr + 1);
	switch (dr->bNotificationType) {
	case USB_CDC_NOTIFY_NETWORK_CONNECTION:
		dbg("%s network", dr->wValue ?
					"connected to" : "disconnected from");
		break;

	case USB_CDC_NOTIFY_SERIAL_STATE:
		tty = tty_port_tty_get(&acm->port);
		newctrl = get_unaligned_le16(data);

		if (tty) {
			if (!acm->clocal &&
				(acm->ctrlin & ~newctrl & ACM_CTRL_DCD)) {
				dbg("calling hangup");
				tty_hangup(tty);
			}
			tty_kref_put(tty);
		}

		acm->ctrlin = newctrl;

		dbg("input control lines: dcd%c dsr%c break%c ring%c framing%c parity%c overrun%c",
			acm->ctrlin & ACM_CTRL_DCD ? '+' : '-',
			acm->ctrlin & ACM_CTRL_DSR ? '+' : '-',
			acm->ctrlin & ACM_CTRL_BRK ? '+' : '-',
			acm->ctrlin & ACM_CTRL_RI  ? '+' : '-',
			acm->ctrlin & ACM_CTRL_FRAMING ? '+' : '-',
			acm->ctrlin & ACM_CTRL_PARITY ? '+' : '-',
			acm->ctrlin & ACM_CTRL_OVERRUN ? '+' : '-');
			break;

	default:
		dbg("unknown notification %d received: index %d len %d data0 %d data1 %d",
			dr->bNotificationType, dr->wIndex,
			dr->wLength, data[0], data[1]);
		break;
	}
exit:
	usb_mark_last_busy(acm->dev);
	retval = usb_submit_urb(urb, GFP_ATOMIC);
	if (retval)
		dev_err(&urb->dev->dev, "%s - usb_submit_urb failed with "
			"result %d", __func__, retval);
}

/* data interface returns incoming bytes, or we got unthrottled */
static void acm_read_bulk(struct urb *urb)
{
	struct acm_rb *buf;
	struct acm_ru *rcv = urb->context;
	struct acm *acm = rcv->instance;
	int status = urb->status;

	if (verbose) pr_info("Entering acm_read_bulk with status %d", status);

	dbg("Entering acm_read_bulk with status %d", status);

	if (!ACM_READY(acm)) {
		if (verbose) pr_info("%s: Aborting, acm not ready\n", __func__);
		dev_dbg(&acm->data->dev, "Aborting, acm not ready");
		return;
	}
	usb_mark_last_busy(acm->dev);


	if (status) {
		//pr_info("%s: bulk rx status %d, acm->susp_count=%d\n", __func__, status, acm->susp_count);
		dev_dbg(&acm->data->dev, "bulk rx status %d\n", status);
		if (status == -EPROTO)	{
			usb_register_dump();
			//kobject_uevent(&acm->dev->dev.kobj,KOBJ_CHANGE);
			//printk(KERN_INFO "-EPROTO HIT !!\n");

		}
	}

	buf = rcv->buffer;
	buf->size = urb->actual_length;

	if (unlikely(status != 0) && urb->actual_length > 0) {
		/* we force urb->status to success, dont drop the buffer */
		pr_info("%s: VP urb was canceled but received a buffer %d\n",
			__func__, urb->actual_length);
		status = 0;
	}

	if (likely(status == 0)) {
		spin_lock(&acm->read_lock);
		acm->processing++;
		list_add_tail(&rcv->list, &acm->spare_read_urbs);
		list_add_tail(&buf->list, &acm->filled_read_bufs);
		spin_unlock(&acm->read_lock);
	} else {
		if (verbose) pr_info("%s: we drop the buffer due to an error %d\n", __func__, status);
		/* we drop the buffer due to an error */
		spin_lock(&acm->read_lock);
		list_add_tail(&rcv->list, &acm->spare_read_urbs);
		list_add(&buf->list, &acm->spare_read_bufs);
		spin_unlock(&acm->read_lock);
		/* nevertheless the tasklet must be kicked unconditionally
		so the queue cannot dry up */
	}
	if (likely(!acm->susp_count) && (errstatuscount<200)){
		tasklet_schedule(&acm->urb_task);
		
		if(status!=0){
			errstatuscount++;
		}else if(status==0){
			if(errstatuscount!=0){
				pr_info("%s: errstatuscount=%d\n", __func__, errstatuscount);
				errstatuscount=0;
			}	
		}
	}
	
}

static void acm_rx_tasklet(unsigned long _acm)
{
	struct acm *acm = (void *)_acm;
	struct acm_rb *buf;
	struct tty_struct *tty;
	struct acm_ru *rcv;
	unsigned long flags;
	unsigned char throttled;
	int copied;

	if (verbose) pr_info("%s: acm %p\n", __func__, acm);

	if (!ACM_READY(acm)) {
		pr_info(MODULE_NAME "acm_rx_tasklet: ACM not ready\n");
		return;
	} else
		rwlog(MODULE_NAME "Entering acm_rx_tasklet ttyACM%d\n", acm->minor);

	spin_lock_irqsave(&acm->throttle_lock, flags);
	throttled = acm->throttle;
	spin_unlock_irqrestore(&acm->throttle_lock, flags);
	if (throttled) {
		static unsigned int times = 0;
		static unsigned int count = 0;
		if (0 == times++ % 3000)
			pr_info(MODULE_NAME "%s: ttyACM%d throttled - %d\n",
				__func__, acm->minor, count++); /* HTC */
		rwlog(MODULE_NAME "%s(%d) ttyACM%d - exit\n", __func__, __LINE__, acm->minor);
		return;
	}

	tty = tty_port_tty_get(&acm->port);

next_buffer:
	spin_lock_irqsave(&acm->read_lock, flags);
	/* HTC c: get the control packets, not data */
	if (list_empty(&acm->filled_read_bufs)) {
		rwlog(MODULE_NAME "%s ttyACM%d list_empty(&acm->filled_read_bufs)\n",
			__func__, acm->minor);
		spin_unlock_irqrestore(&acm->read_lock, flags);
		goto urbs;
	}
	buf = list_entry(acm->filled_read_bufs.next,
			 struct acm_rb, list);
	list_del(&buf->list);
	spin_unlock_irqrestore(&acm->read_lock, flags);

	rwlog(MODULE_NAME "acm_rx_tasklet: ttyACM%d procesing buf 0x%p, size = %d",
		acm->minor, buf, buf->size);

	copied = 0;
	if (tty) {
		spin_lock_irqsave(&acm->throttle_lock, flags);
		throttled = acm->throttle;
		spin_unlock_irqrestore(&acm->throttle_lock, flags);
		if (!throttled) {
			copied = tty_insert_flip_string(tty, buf->base, buf->size);
			tty_flip_buffer_push(tty);
			if (copied != buf->size && printk_ratelimit())
				pr_info(MODULE_NAME "%s: ttyACM%d - copied %d != buf->size %d\n",
					__func__, acm->minor, copied, buf->size);
			else
				rwlog(MODULE_NAME "%s: ttyACM%d - copied %d == buf->size %d\n",
					__func__, acm->minor, copied, buf->size);
			/* HTC: calculate the TP statistic */
			if (is_minor_valid(acm->minor))
				acm_tp[acm->minor].rx_total += buf->size;
			
			/*HTC: rxbyt for acm0*/
			if(acm->minor==0)
				rxbyte=rxbyte+buf->size;

		} else {
			tty_kref_put(tty);
			pr_info(MODULE_NAME "%s - ttyACM%d Throttling noticed\n", __func__, acm->minor);
			spin_lock_irqsave(&acm->read_lock, flags);
			list_add(&buf->list, &acm->filled_read_bufs);
			spin_unlock_irqrestore(&acm->read_lock, flags);
			return;
		}
	}

	/* HTC: log */
	/* printk(MODULE_NAME":%s ttyACM%d len=%3d\n",
		__func__, acm->minor, buf->size); */
	if ((host_dbg_flag & DBG_ACM0_RW && acm->minor == 0) ||
		(host_dbg_flag & DBG_ACM1_RW && acm->minor == 1)) {
		int i = 0, size = buf->size, rc = 0;
		char *data_buf = buf->base;
		char pr_buf[512];
#if 0
		size = size > 16 ? 16 : size;
		for (i=0; i < size; i++) {
			unsigned char c = data_buf[i];
			if (!isprint(c)) {
				if (c == '\r')
					rc += sprintf(pr_buf + rc, "\\r");
				else if (c == '\n')
					rc += sprintf(pr_buf + rc, "\\n");
				else
					rc += sprintf(pr_buf + rc, ".");
			} else
				rc += sprintf(pr_buf + rc, "%c", c);
		}
		pr_buf[rc] = '\0';
		pr_info(MODULE_NAME ":[%03d] ttyACM%d RX << [%s]\n",
			buf->size, acm->minor, pr_buf);
#endif
		/* print out binary */
		size = size > 16 ? 16 : size;
		rc = 0;
		for (i = 0; i < size; i++)
			rc += sprintf(pr_buf + rc, "%02x ", data_buf[i]);
		pr_buf[rc] = '\0';
		pr_info(MODULE_NAME ":[%03d] ttyACM%d RX bin << [%s]\n",
			buf->size, acm->minor, pr_buf);
	}

	
	/*Buffer latest Rx for debug*/
	if(buf->size>0&&(acm->minor==0)){
	int gsize = buf->size;
	char *gdata_buf = buf->base;
	int gi=0;
	gsize = gsize > 16 ? 16 : gsize;
	int grc = 0;
	for (gi = 0; gi < gsize; gi++)
		grc += sprintf(gpr_buf + grc, "%02x ", gdata_buf[gi]);
	gpr_buf[grc] = '\0';
	pcount=1;
	}
	
	if (copied == buf->size || !tty) {

		//fill free buffer with pattern.
		memset(buf->base,0xac,buf->size);
		spin_lock_irqsave(&acm->read_lock, flags);
		list_add(&buf->list, &acm->spare_read_bufs);
		spin_unlock_irqrestore(&acm->read_lock, flags);
	} else {
		tty_kref_put(tty);
		rwlog(MODULE_NAME "%s - ttyACM%d Partial buffer fill %d", __func__, acm->minor, copied);
		if (copied > 0) {
			memmove(buf->base,
				buf->base + copied,
				buf->size - copied);
			//fill free buffer with pattern.
			memset(buf->base+buf->size-copied,0xad,copied);
			buf->size -= copied;
		}
		spin_lock_irqsave(&acm->read_lock, flags);
		list_add(&buf->list, &acm->filled_read_bufs);
		spin_unlock_irqrestore(&acm->read_lock, flags);
		rwlog(MODULE_NAME "%s(%d) ttyACM%d - exit\n", __func__, __LINE__, acm->minor);
		return;
	}
	goto next_buffer;

urbs:
	tty_kref_put(tty);

	/* HTC c: spare is not empty -> deal with it */
	while (!list_empty(&acm->spare_read_bufs)) {
		spin_lock_irqsave(&acm->read_lock, flags);
		if (list_empty(&acm->spare_read_urbs)) {
			acm->processing = 0;
			rwlog(MODULE_NAME "list_empty(&acm->spare_read_urbs) return\n");
			spin_unlock_irqrestore(&acm->read_lock, flags);
			rwlog(MODULE_NAME "%s(%d) ttyACM%d - exit\n", __func__, __LINE__, acm->minor);
			return;
		}
		rcv = list_entry(acm->spare_read_urbs.next,
				 struct acm_ru, list);
		list_del(&rcv->list);
		spin_unlock_irqrestore(&acm->read_lock, flags);

		buf = list_entry(acm->spare_read_bufs.next,
				 struct acm_rb, list);
		list_del(&buf->list);

		rcv->buffer = buf;

		if (acm->is_int_ep)
			usb_fill_int_urb(rcv->urb, acm->dev,
					 acm->rx_endpoint,
					 buf->base,
					 acm->readsize,
					 acm_read_bulk, rcv, acm->bInterval);
		else
			usb_fill_bulk_urb(rcv->urb, acm->dev,
					  acm->rx_endpoint,
					  buf->base,
					  acm->readsize,
					  acm_read_bulk, rcv);
		rcv->urb->transfer_dma = buf->dma;
		rcv->urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
		rcv->urb->actual_length = 0;
		/*for debug*/
		memset(buf->base,0xaa,acm->readsize);
		memset(buf->base,0xab,acm->readsize);
		dmac_flush_range(buf->base,buf->base+acm->readsize-1);
		//_range(current->,buf->base,buf->base+acm->readsize-1);
		//memset(buf->base,0xaf,acm->readsize);

		/* This shouldn't kill the driver as unsuccessful URBs are
		   returned to the free-urbs-pool and resubmited ASAP */
		spin_lock_irqsave(&acm->read_lock, flags);

	
		if (acm->susp_count ||
				usb_submit_urb(rcv->urb, GFP_ATOMIC) < 0) {
			list_add(&buf->list, &acm->spare_read_bufs);
			list_add(&rcv->list, &acm->spare_read_urbs);
			acm->processing = 0;
			spin_unlock_irqrestore(&acm->read_lock, flags);
			rwlog(MODULE_NAME "%s(%d) ttyACM%d - exit\n", __func__, __LINE__, acm->minor);
			return;
		} else {
			/* HTC c: submit ok */
			spin_unlock_irqrestore(&acm->read_lock, flags);
			/* HTC: change the log level */
			rwlog(MODULE_NAME "acm_rx_tasklet: ttyACM%d sending urb 0x%p, rcv 0x%p, buf 0x%p",
				acm->minor, rcv->urb, rcv, buf);
		}
	}
	spin_lock_irqsave(&acm->read_lock, flags);
	acm->processing = 0;
	spin_unlock_irqrestore(&acm->read_lock, flags);
}

/* data interface wrote those outgoing bytes */
static void acm_write_bulk(struct urb *urb)
{
	struct acm_wb *wb = urb->context;
	struct acm *acm = wb->instance;
	unsigned long flags;

	if (verbose || urb->status
			|| (urb->actual_length != urb->transfer_buffer_length))
		dev_dbg(&acm->data->dev, "tx %d/%d bytes -- > %d\n",
			urb->actual_length,
			urb->transfer_buffer_length,
			urb->status);
	
	/*HTC: txbyt for acm0*/
	if(acm->minor==0 && urb->status==0 )
		txbyte=txbyte+urb->transfer_buffer_length;
	
	spin_lock_irqsave(&acm->write_lock, flags);
	acm_write_done(acm, wb);
	spin_unlock_irqrestore(&acm->write_lock, flags);
	if (ACM_READY(acm))
		schedule_work(&acm->work);
	else
		wake_up_interruptible(&acm->drain_wait);
}

static void acm_softint(struct work_struct *work)
{
	struct acm *acm = container_of(work, struct acm, work);
	struct tty_struct *tty;

	dev_vdbg(&acm->data->dev, "tx work\n");
	if (!ACM_READY(acm))
		return;
	tty = tty_port_tty_get(&acm->port);
	tty_wakeup(tty);
	tty_kref_put(tty);
}

/*
 * TTY handlers
 */

static int acm_tty_open(struct tty_struct *tty, struct file *filp)
{
	struct acm *acm;
	int rv = -ENODEV;
	int i;
	dbg("Entering acm_tty_open.");

	mutex_lock(&open_mutex);

	if (!tty) {
		pr_err("%s: tty NULL\n", __func__);
		goto out;
	}
	if (tty->index >ACM_TTY_MINORS ) {
		pr_err("%s: tty > ACM_TTY_MINORS\n", __func__);
		goto out;
	}

	acm = acm_table[tty->index];

	if (!acm || !acm->dev)
		goto out;
	else
		rv = 0;

#if 1 //HTC_CSP_START
	printk(MODULE_NAME":%s ttyACM%d +\n",__FUNCTION__,acm->minor);
#endif //HTC_CSP_END

	/* Current acm implementation does not have acm->disconnected
	   feature to know whether acm is probed. Adding a check to
	   see if atleast ACM0 is up.
	*/
	for (i=0; i < MAX_ACM_NUM; i++) {
		if (acm_ready_table[i] == false) {
			printk("%s: acm is not ready\n", __func__);
			rv = -ENODEV;
		goto out;
		}
	}

	set_bit(TTY_NO_WRITE_SPLIT, &tty->flags);

	tty->driver_data = acm;
	tty_port_tty_set(&acm->port, tty);

	reflog("[ref] + %s(%d) %d\n", __func__, __LINE__, ++autopm_refcnt);
	if (usb_autopm_get_interface(acm->control) < 0)
		goto early_bail;
	else
		acm->control->needs_remote_wakeup = 0;


	mutex_lock(&acm->mutex);
	if (acm->port.count++) {
		mutex_unlock(&acm->mutex);
		reflog("[ref] - %s(%d) %d\n", __func__, __LINE__, --autopm_refcnt);
		usb_autopm_put_interface(acm->control);
		goto out;
	}

	acm->ctrlurb->dev = acm->dev;
	if (usb_submit_urb(acm->ctrlurb, GFP_KERNEL)) {
		dbg("usb_submit_urb(ctrl irq) failed");
		goto bail_out;
	}

	if (0 > acm_set_control(acm, acm->ctrlout = ACM_CTRL_DTR | ACM_CTRL_RTS) &&
	    (acm->ctrl_caps & USB_CDC_CAP_LINE))
		goto full_bailout;

	reflog("[ref] - %s(%d) %d\n", __func__, __LINE__, --autopm_refcnt);
	usb_autopm_put_interface(acm->control);

	INIT_LIST_HEAD(&acm->spare_read_urbs);
	INIT_LIST_HEAD(&acm->spare_read_bufs);
	INIT_LIST_HEAD(&acm->filled_read_bufs);

	for (i = 0; i < acm->rx_buflimit; i++)
		list_add(&(acm->ru[i].list), &acm->spare_read_urbs);
	for (i = 0; i < acm->rx_buflimit; i++)
		list_add(&(acm->rb[i].list), &acm->spare_read_bufs);

	acm->throttle = 0;

	set_bit(ASYNCB_INITIALIZED, &acm->port.flags);
	rv = tty_port_block_til_ready(&acm->port, tty, filp);
	tasklet_schedule(&acm->urb_task);

	mutex_unlock(&acm->mutex);

	/* HTC: start timer */
	printk(MODULE_NAME":%s ttyACM%d -\n",__FUNCTION__,acm->minor);
	if (!timer_on && acm->minor == 0 &&
		(mach_ble_flashless || is_mach_edge || mach_ble_flash)) {
		pr_info(MODULE_NAME ":%s mod acm_timer\n", __func__);
		timer_on = 1;
		mod_timer(&acm_timer, jiffies + msecs_to_jiffies(timer_value_ms));
	}

out:
	mutex_unlock(&open_mutex);
	return rv;

full_bailout:
	usb_kill_urb(acm->ctrlurb);
bail_out:
	acm->port.count--;
	mutex_unlock(&acm->mutex);
	reflog("[ref] - %s(%d) %d\n", __func__, __LINE__, --autopm_refcnt);
	usb_autopm_put_interface(acm->control);
early_bail:
	mutex_unlock(&open_mutex);
	tty_port_tty_set(&acm->port, NULL);
	return -EIO;
}

static void acm_tty_unregister(struct acm *acm)
{
	int i, nr;

	nr = acm->rx_buflimit;
	tty_unregister_device(acm_tty_driver, acm->minor);
	usb_put_intf(acm->control);
	acm_table[acm->minor] = NULL;
	usb_free_urb(acm->ctrlurb);
	for (i = 0; i < ACM_NW; i++)
		usb_free_urb(acm->wb[i].urb);
	for (i = 0; i < nr; i++)
		usb_free_urb(acm->ru[i].urb);
	kfree(acm->country_codes);
	kfree(acm);
}

static int acm_tty_chars_in_buffer(struct tty_struct *tty);

static void acm_port_down(struct acm *acm)
{
#if 1 //HTC_CSP_START
	printk(MODULE_NAME":%s ttyACM%d +\n",__FUNCTION__,acm->minor);
#endif //HTC_CSP_END
	int i, nr = acm->rx_buflimit;
	//mutex_lock(&open_mutex);
	if (acm->dev) {
		reflog("[ref] + %s(%d) %d\n", __func__, __LINE__, ++autopm_refcnt);
		usb_autopm_get_interface(acm->control);
		acm_set_control(acm, acm->ctrlout = 0);
		usb_kill_urb(acm->ctrlurb);
		for (i = 0; i < ACM_NW; i++)
			usb_kill_urb(acm->wb[i].urb);
		tasklet_disable(&acm->urb_task);
		for (i = 0; i < nr; i++)
			usb_kill_urb(acm->ru[i].urb);
		tasklet_enable(&acm->urb_task);
		acm->control->needs_remote_wakeup = 0;
		reflog("[ref] - %s(%d) %d\n", __func__, __LINE__, --autopm_refcnt);
		usb_autopm_put_interface(acm->control);
	}
	//mutex_unlock(&open_mutex);
#if 1 //HTC_CSP_START
	printk(MODULE_NAME":%s ttyACM%d -\n",__FUNCTION__,acm->minor);
#endif //HTC_CSP_END
}

static void acm_tty_hangup(struct tty_struct *tty)
{
	struct acm *acm;

	if(!tty)
	{
		printk(MODULE_NAME":%s !tty\n",__FUNCTION__);
		return;
	}
	mutex_lock(&open_mutex);
	acm = tty->driver_data;
	if(!acm)
	{
		printk(MODULE_NAME":%s !acm\n",__FUNCTION__);
		goto out;
	}


#if 1 //HTC_CSP_START
			printk(MODULE_NAME":%s ttyACM%d +\n",__FUNCTION__,acm->minor);
#endif //HTC_CSP_END

	tty_port_hangup(&acm->port);
	acm_port_down(acm);
#if 1 //HTC_CSP_START
		printk(MODULE_NAME":%s ttyACM%d -\n",__FUNCTION__,acm->minor);
#endif //HTC_CSP_END

out:
	mutex_unlock(&open_mutex);
	pr_info("%s: ttyACM%d out: -\n", __func__, acm->minor);


}



static void acm_tty_close(struct tty_struct *tty, struct file *filp)
{
	struct acm *acm = tty->driver_data;

	/* Perform the closing process and see if we need to do the hardware
	   shutdown */
	if (!acm) {
		pr_err("%s: !acm\n", __func__);
		return;
		}
#if 1 //HTC_CSP_START
	printk(MODULE_NAME":%s ttyACM%d +\n",__FUNCTION__,acm->minor);
#endif //HTC_CSP_END	
	mutex_lock(&open_mutex);
	if (tty_port_close_start(&acm->port, tty, filp) == 0) {
		if (!acm->dev) {
			tty_port_tty_set(&acm->port, NULL);
			acm_tty_unregister(acm);
			tty->driver_data = NULL;
		}
		mutex_unlock(&open_mutex);
		pr_info("%s: exit\n", __func__);
		return;
	}

	/* HTC: delete timer, support FLASHLESS first */
/*
	if (acm->minor == 0 && (mach_ble_flashless || is_mach_edge))
		del_timer_sync(&acm_timer);
*/
	acm_port_down(acm);
	tty_port_close_end(&acm->port, tty);
	tty_port_tty_set(&acm->port, NULL);
	mutex_unlock(&open_mutex);
#if 1 //HTC_CSP_START
	printk(MODULE_NAME":%s ttyACM%d -\n",__FUNCTION__,acm->minor);
#endif //HTC_CSP_END
}

static int acm_tty_write(struct tty_struct *tty,
					const unsigned char *buf, int count)
{
	struct acm *acm = tty->driver_data;
	int stat;
	unsigned long flags;
	int wbn;
	struct acm_wb *wb;

	if (verbose) pr_info("%s: buf %p count %d\n", __func__, buf, count);

	if (!ACM_READY(acm)) {
		pr_err("%s - !acm\n", __func__); /* HTC */
		return -EINVAL;
	}
	if (!count) {
		pr_info("%s - !count\n", __func__); /* HTC */
		return 0;
	}


	spin_lock_irqsave(&acm->write_lock, flags);
	wbn = acm_wb_alloc(acm);
	if (wbn < 0) {
		if (verbose) pr_err("%s: acm %p wbn %d < 0!\n", __func__, acm, wbn);
		spin_unlock_irqrestore(&acm->write_lock, flags);
		return 0;
	}
	wb = &acm->wb[wbn];

	count = (count > acm->writesize) ? acm->writesize : count;

	/* HTC: log */
	/* pr_info("Get %d bytes for ttyACM%d, alloc wbn=%d\n",
		count, acm->minor, wbn); */
	if (verbose) pr_info("Get %d bytes for ttyACM%d, alloc wbn=%d\n",
		count, acm->minor, wbn);
	if ((host_dbg_flag & DBG_ACM0_RW && acm->minor == 0) ||
		(host_dbg_flag & DBG_ACM1_RW && acm->minor == 1)) {
		/*Print the latest Rx data*/
		if(pcount==1&&(acm->minor==0)){
		pr_info(MODULE_NAME ":Debug Latest RX bin << [%s]\n", gpr_buf);
		pcount=0;
		}
		
				
		int i = 0, size = count, rc = 0;
		char *data_buf = buf;
		char pr_buf[512];
#if 0
		size = size > 16 ? 16 : size;
		for (i=0; i < size; i++) {
			unsigned char c = data_buf[i];
			if (!isprint(c)) {
				if (c == '\r')
					rc += sprintf(pr_buf + rc, "\\r");
				else if (c == '\n')
					rc += sprintf(pr_buf + rc, "\\n");
				else
					rc += sprintf(pr_buf + rc, ".");
			} else
				rc += sprintf(pr_buf + rc, "%c", c);
		}

		pr_buf[rc] = '\0';
		pr_info(MODULE_NAME ":[%03d] TX >> [%s]", count, pr_buf);
#endif
		/* print out binary */
		size = size > 16 ? 16 : size;
		rc = 0;
		for (i = 0; i < size; i++)
			rc += sprintf(pr_buf + rc, "%02x ", data_buf[i]);
		pr_buf[rc] = '\0';
		pr_info(MODULE_NAME ":[%03d] TX bin >> [%s]\n", count, pr_buf);
	}

	memcpy(wb->buf, buf, count);
	wb->len = count;
	spin_unlock_irqrestore(&acm->write_lock, flags);

	stat = acm_write_start(acm, wbn);
	if (stat < 0)
		return stat;

	/* HTC: get tx statistic */
	if (is_minor_valid(acm->minor) && count > 0)
		acm_tp[acm->minor].tx_total += count;
	return count;
}

static int acm_tty_write_room(struct tty_struct *tty)
{
	struct acm *acm = tty->driver_data;
	if (!ACM_READY(acm)) {
		if (verbose) pr_info("%s: !ACM_READY\n", __func__);
		return -EINVAL;
	}
	/*
	 * Do not let the line discipline to know that we have a reserve,
	 * or it might get too enthusiastic.
	 */
	return acm_wb_is_avail(acm) ? acm->writesize : 0;
}

static int acm_tty_chars_in_buffer(struct tty_struct *tty)
{
	struct acm *acm = tty->driver_data;
	if (!ACM_READY(acm))
		return 0;
	/*
	 * This is inaccurate (overcounts), but it works.
	 */
	return (ACM_NW - acm_wb_is_avail(acm)) * acm->writesize;
}

static void acm_tty_throttle(struct tty_struct *tty)
{
	struct acm *acm = tty->driver_data;
#if 1 //HTC_CSP_START
	printk(MODULE_NAME":%s ttyACM%d +\n",__FUNCTION__,acm->minor);
#endif //HTC_CSP_END
	if (!ACM_READY(acm))
		return;
	spin_lock_bh(&acm->throttle_lock);
	acm->throttle = 1;
	spin_unlock_bh(&acm->throttle_lock);
#if 1 //HTC_CSP_START
	printk(MODULE_NAME":%s ttyACM%d -\n",__FUNCTION__,acm->minor);
#endif //HTC_CSP_END

}

static void acm_tty_unthrottle(struct tty_struct *tty)
{
	struct acm *acm = tty->driver_data;
#if 1 //HTC_CSP_START
	printk(MODULE_NAME":%s ttyACM%d +\n",__FUNCTION__,acm->minor);
#endif //HTC_CSP_END

	if (!ACM_READY(acm))
		return;
	spin_lock_bh(&acm->throttle_lock);
	acm->throttle = 0;
	spin_unlock_bh(&acm->throttle_lock);
	tasklet_schedule(&acm->urb_task);
#if 1 //HTC_CSP_START
		printk(MODULE_NAME":%s ttyACM%d -\n",__FUNCTION__,acm->minor);
#endif //HTC_CSP_END

}

static int acm_tty_break_ctl(struct tty_struct *tty, int state)
{
	struct acm *acm = tty->driver_data;
	int retval;
	if (!ACM_READY(acm))
		return -EINVAL;
	retval = acm_send_break(acm, state ? 0xffff : 0);
	if (retval < 0)
		dbg("send break failed");
	return retval;
}

static int acm_tty_tiocmget(struct tty_struct *tty)
{
	struct acm *acm = tty->driver_data;

	if (!ACM_READY(acm))
		return -EINVAL;

	return (acm->ctrlout & ACM_CTRL_DTR ? TIOCM_DTR : 0) |
	       (acm->ctrlout & ACM_CTRL_RTS ? TIOCM_RTS : 0) |
	       (acm->ctrlin  & ACM_CTRL_DSR ? TIOCM_DSR : 0) |
	       (acm->ctrlin  & ACM_CTRL_RI  ? TIOCM_RI  : 0) |
	       (acm->ctrlin  & ACM_CTRL_DCD ? TIOCM_CD  : 0) |
	       TIOCM_CTS;
}

static int acm_tty_tiocmset(struct tty_struct *tty,
			    unsigned int set, unsigned int clear)
{
	struct acm *acm = tty->driver_data;
	unsigned int newctrl;

	/* pr_info(MODULE_NAME "%s\n", __func__); */

	if (!ACM_READY(acm))
		return -EINVAL;

	newctrl = acm->ctrlout;
	set = (set & TIOCM_DTR ? ACM_CTRL_DTR : 0) |
					(set & TIOCM_RTS ? ACM_CTRL_RTS : 0);
	clear = (clear & TIOCM_DTR ? ACM_CTRL_DTR : 0) |
					(clear & TIOCM_RTS ? ACM_CTRL_RTS : 0);

	newctrl = (newctrl & ~clear) | set;

	if (acm->ctrlout == newctrl)
		return 0;
	return acm_set_control(acm, acm->ctrlout = newctrl);
}

static int acm_tty_ioctl(struct tty_struct *tty,
					unsigned int cmd, unsigned long arg)
{
	struct acm *acm = tty->driver_data;

	if (!ACM_READY(acm))
		return -EINVAL;

	return -ENOIOCTLCMD;
}

static const __u32 acm_tty_speed[] = {
	0, 50, 75, 110, 134, 150, 200, 300, 600,
	1200, 1800, 2400, 4800, 9600, 19200, 38400,
	57600, 115200, 230400, 460800, 500000, 576000,
	921600, 1000000, 1152000, 1500000, 2000000,
	2500000, 3000000, 3500000, 4000000
};

static const __u8 acm_tty_size[] = {
	5, 6, 7, 8
};

static void acm_tty_set_termios(struct tty_struct *tty,
						struct ktermios *termios_old)
{
	struct acm *acm = tty->driver_data;
	struct ktermios *termios = tty->termios;
	struct usb_cdc_line_coding newline;
	int newctrl = acm->ctrlout;

	/* pr_info(MODULE_NAME "%s\n", __func__); */

	if (!ACM_READY(acm))
		return;

	newline.dwDTERate = cpu_to_le32(tty_get_baud_rate(tty));
	newline.bCharFormat = termios->c_cflag & CSTOPB ? 2 : 0;
	newline.bParityType = termios->c_cflag & PARENB ?
				(termios->c_cflag & PARODD ? 1 : 2) +
				(termios->c_cflag & CMSPAR ? 2 : 0) : 0;
	newline.bDataBits = acm_tty_size[(termios->c_cflag & CSIZE) >> 4];
	/* FIXME: Needs to clear unsupported bits in the termios */
	acm->clocal = ((termios->c_cflag & CLOCAL) != 0);

	if (!newline.dwDTERate) {
		newline.dwDTERate = acm->line.dwDTERate;
		newctrl &= ~ACM_CTRL_DTR;
	} else
		newctrl |=  ACM_CTRL_DTR;

	if (newctrl != acm->ctrlout)
		acm_set_control(acm, acm->ctrlout = newctrl);

	if (memcmp(&acm->line, &newline, sizeof newline)) {
		memcpy(&acm->line, &newline, sizeof newline);
		dbg("set line: %d %d %d %d", le32_to_cpu(newline.dwDTERate),
			newline.bCharFormat, newline.bParityType,
			newline.bDataBits);
		acm_set_line(acm, &acm->line);
	}
}

/*
 * USB probe and disconnect routines.
 */

/* Little helpers: write/read buffers free */
static void acm_write_buffers_free(struct acm *acm)
{
	int i;
	struct acm_wb *wb;
	struct usb_device *usb_dev = interface_to_usbdev(acm->control);

	for (wb = &acm->wb[0], i = 0; i < ACM_NW; i++, wb++)
		usb_free_coherent(usb_dev, acm->writesize, wb->buf, wb->dmah);
}

static void acm_read_buffers_free(struct acm *acm)
{
	struct usb_device *usb_dev = interface_to_usbdev(acm->control);
	int i, n = acm->rx_buflimit;

	for (i = 0; i < n; i++)
		usb_free_coherent(usb_dev, acm->readsize,
				  acm->rb[i].base, acm->rb[i].dma);
}

/* Little helper: write buffers allocate */
static int acm_write_buffers_alloc(struct acm *acm)
{
	int i;
	struct acm_wb *wb;

	for (wb = &acm->wb[0], i = 0; i < ACM_NW; i++, wb++) {
		wb->buf = usb_alloc_coherent(acm->dev, acm->writesize, GFP_DMA,
		    &wb->dmah);
		if (!wb->buf) {
			while (i != 0) {
				--i;
				--wb;
				usb_free_coherent(acm->dev, acm->writesize,
				    wb->buf, wb->dmah);
			}
			return -ENOMEM;
		}
	}
	return 0;
}

static int acm_probe(struct usb_interface *intf,
		     const struct usb_device_id *id)
{
	struct usb_cdc_union_desc *union_header = NULL;
	struct usb_cdc_country_functional_desc *cfd = NULL;
	unsigned char *buffer = intf->altsetting->extra;
	int buflen = intf->altsetting->extralen;
	struct usb_interface *control_interface;
	struct usb_interface *data_interface;
	struct usb_endpoint_descriptor *epctrl = NULL;
	struct usb_endpoint_descriptor *epread = NULL;
	struct usb_endpoint_descriptor *epwrite = NULL;
	struct usb_device *usb_dev = interface_to_usbdev(intf);
	struct acm *acm;
	int minor;
	int ctrlsize, readsize;
	u8 *buf;
	u8 ac_management_function = 0;
	u8 call_management_function = 0;
	int call_interface_num = -1;
	int data_interface_num;
	unsigned long quirks;
	int num_rx_buf;
	int i;
	int combined_interfaces = 0;


	pr_info("%s: 0311 - cdc drop urb fix intf->cur_altsetting->desc.bInterfaceNumber %d max_intfs=%d\n",
		__func__, intf->cur_altsetting->desc.bInterfaceNumber,max_intfs);

	/* normal quirks */
	quirks = (unsigned long)id->driver_info;
	num_rx_buf = (quirks == SINGLE_RX_URB) ? 1 : ACM_NR;

	/* not a real CDC ACM device: ignore modem boot rom */
	if (quirks & NOT_REAL_ACM) {
			if (is_minor_valid(intf->cur_altsetting->desc.bInterfaceNumber))
				acm_ready_table[intf->cur_altsetting->desc.bInterfaceNumber] = false;
			pr_info(" 0320 quirks & NOT_REAL_ACM acm_ready_table[intf->cur_altsetting->desc.bInterfaceNumber=%d]=false\n",intf->cur_altsetting->desc.bInterfaceNumber);
		return -ENODEV;
		}


	if (max_intfs == 1) {
		/* XMM HACK - use ACM for interfaces 0/1 only */
		switch (intf->cur_altsetting->desc.bInterfaceNumber)
		{
		case 0: case 1:
				break;
		default:
			pr_info("XMM HACK -"
				" interface #%d reserved for RAW-IP network driver\n",
				intf->cur_altsetting->desc.bInterfaceNumber);
			return -EINVAL;
		}
	} else if (max_intfs == 2) {
		/* XMM HACK - use ACM for interfaces 0/1 & 6/7 only */
		switch (intf->cur_altsetting->desc.bInterfaceNumber)
		{
		case 0: case 1:
		case 6: case 7:
				break;
		default:
			pr_info("XMM HACK -"
				" interface #%d reserved for RAW-IP network driver\n",
				intf->cur_altsetting->desc.bInterfaceNumber);
			return -EINVAL;
		}
	}

	/* handle quirks deadly to normal probing*/
	if (quirks & NO_UNION_NORMAL) {
		data_interface = usb_ifnum_to_if(usb_dev, 1);
		control_interface = usb_ifnum_to_if(usb_dev, 0);
		goto skip_normal_probe;
	}

	/* normal probing*/
	if (!buffer) {
		dev_err(&intf->dev, "Weird descriptor references\n");
		return -EINVAL;
	}

	if (!buflen) {
		if (intf->cur_altsetting->endpoint &&
				intf->cur_altsetting->endpoint->extralen &&
				intf->cur_altsetting->endpoint->extra) {
			dev_dbg(&intf->dev,
				"Seeking extra descriptors on endpoint\n");
			buflen = intf->cur_altsetting->endpoint->extralen;
			buffer = intf->cur_altsetting->endpoint->extra;
		} else {
			dev_err(&intf->dev,
				"Zero length descriptor references\n");
			return -EINVAL;
		}
	}

	while (buflen > 0) {
		if (buffer[1] != USB_DT_CS_INTERFACE) {
			dev_err(&intf->dev, "skipping garbage\n");
			goto next_desc;
		}

		switch (buffer[2]) {
		case USB_CDC_UNION_TYPE: /* we've found it */
			if (union_header) {
				dev_err(&intf->dev, "More than one "
					"union descriptor, skipping ...\n");
				goto next_desc;
			}
			union_header = (struct usb_cdc_union_desc *)buffer;
			break;
		case USB_CDC_COUNTRY_TYPE: /* export through sysfs*/
			cfd = (struct usb_cdc_country_functional_desc *)buffer;
			break;
		case USB_CDC_HEADER_TYPE: /* maybe check version */
			break; /* for now we ignore it */
		case USB_CDC_ACM_TYPE:
			ac_management_function = buffer[3];
			break;
		case USB_CDC_CALL_MANAGEMENT_TYPE:
			call_management_function = buffer[3];
			call_interface_num = buffer[4];
			if ( (quirks & NOT_A_MODEM) == 0 && (call_management_function & 3) != 3)
				dev_err(&intf->dev, "This device cannot do calls on its own. It is not a modem.\n");
			break;
		default:
			/* there are LOTS more CDC descriptors that
			 * could legitimately be found here.
			 */
			dev_dbg(&intf->dev, "Ignoring descriptor: "
					"type %02x, length %d\n",
					buffer[2], buffer[0]);
			break;
		}
next_desc:
		buflen -= buffer[0];
		buffer += buffer[0];
	}

	if (!union_header) {
		if (call_interface_num > 0) {
			dev_dbg(&intf->dev, "No union descriptor, using call management descriptor\n");
			data_interface = usb_ifnum_to_if(usb_dev, (data_interface_num = call_interface_num));
			control_interface = intf;
		} else {
			if (intf->cur_altsetting->desc.bNumEndpoints != 3) {
				dev_dbg(&intf->dev,"No union descriptor, giving up\n");
				return -ENODEV;
			} else {
				dev_warn(&intf->dev,"No union descriptor, testing for castrated device\n");
				combined_interfaces = 1;
				control_interface = data_interface = intf;
				goto look_for_collapsed_interface;
			}
		}
	} else {
		control_interface = usb_ifnum_to_if(usb_dev, union_header->bMasterInterface0);
		data_interface = usb_ifnum_to_if(usb_dev, (data_interface_num = union_header->bSlaveInterface0));
		if (!control_interface || !data_interface) {
			dev_dbg(&intf->dev, "no interfaces\n");
			return -ENODEV;
		}
	}

	if (data_interface_num != call_interface_num)
		dev_dbg(&intf->dev, "Separate call control interface. That is not fully supported.\n");

	if (control_interface == data_interface) {
		/* some broken devices designed for windows work this way */
		dev_warn(&intf->dev,"Control and data interfaces are not separated!\n");
		combined_interfaces = 1;
		/* a popular other OS doesn't use it */
		quirks |= NO_CAP_LINE;
		if (data_interface->cur_altsetting->desc.bNumEndpoints != 3) {
			dev_err(&intf->dev, "This needs exactly 3 endpoints\n");
			return -EINVAL;
		}
look_for_collapsed_interface:
		for (i = 0; i < 3; i++) {
			struct usb_endpoint_descriptor *ep;
			ep = &data_interface->cur_altsetting->endpoint[i].desc;

			if (usb_endpoint_is_int_in(ep))
				epctrl = ep;
			else if (usb_endpoint_is_bulk_out(ep))
				epwrite = ep;
			else if (usb_endpoint_is_bulk_in(ep))
				epread = ep;
			else
				return -EINVAL;
		}
		if (!epctrl || !epread || !epwrite)
			return -ENODEV;
		else
			goto made_compressed_probe;
	}

skip_normal_probe:

	/*workaround for switched interfaces */
	if (data_interface->cur_altsetting->desc.bInterfaceClass
						!= CDC_DATA_INTERFACE_TYPE) {
		if (control_interface->cur_altsetting->desc.bInterfaceClass
						== CDC_DATA_INTERFACE_TYPE) {
			struct usb_interface *t;
			dev_dbg(&intf->dev,
				"Your device has switched interfaces.\n");
			t = control_interface;
			control_interface = data_interface;
			data_interface = t;
		} else {
			return -EINVAL;
		}
	}

	/* Accept probe requests only for the control interface */
	if (!combined_interfaces && intf != control_interface)
		return -ENODEV;

	if (!combined_interfaces && usb_interface_claimed(data_interface)) {
		/* valid in this context */
		dev_dbg(&intf->dev, "The data interface isn't available\n");
		return -EBUSY;
	}


	if (data_interface->cur_altsetting->desc.bNumEndpoints < 2)
		return -EINVAL;

	epctrl = &control_interface->cur_altsetting->endpoint[0].desc;
	epread = &data_interface->cur_altsetting->endpoint[0].desc;
	epwrite = &data_interface->cur_altsetting->endpoint[1].desc;


	/* workaround for switched endpoints */
	if (!usb_endpoint_dir_in(epread)) {
		/* descriptors are swapped */
		struct usb_endpoint_descriptor *t;
		dev_dbg(&intf->dev,
			"The data interface has switched endpoints\n");
		t = epread;
		epread = epwrite;
		epwrite = t;
	}
made_compressed_probe:
	dbg("interfaces are valid");
	for (minor = 0; minor < ACM_TTY_MINORS && acm_table[minor]; minor++);

	if (minor == ACM_TTY_MINORS) {
		dev_err(&intf->dev, "no more free acm devices\n");
		return -ENODEV;
	}

	acm = kzalloc(sizeof(struct acm), GFP_KERNEL);
	if (acm == NULL) {
		dev_dbg(&intf->dev, "out of memory (acm kzalloc)\n");
		goto alloc_fail;
	}

	ctrlsize = le16_to_cpu(epctrl->wMaxPacketSize);
	readsize = le16_to_cpu(epread->wMaxPacketSize) *
				(quirks == SINGLE_RX_URB ? 1 : 2);
	acm->combined_interfaces = combined_interfaces;
	acm->writesize = le16_to_cpu(epwrite->wMaxPacketSize) * 20;
	acm->control = control_interface;
	acm->data = data_interface;
	acm->minor = minor;
	acm->dev = usb_dev;
	acm->ctrl_caps = ac_management_function;
	if (quirks & NO_CAP_LINE)
		acm->ctrl_caps &= ~USB_CDC_CAP_LINE;
	acm->ctrlsize = ctrlsize;
	acm->readsize = readsize;
	acm->rx_buflimit = num_rx_buf;
	acm->urb_task.func = acm_rx_tasklet;
	acm->urb_task.data = (unsigned long) acm;
	INIT_WORK(&acm->work, acm_softint);
	init_usb_anchor(&acm->deferred);
	init_waitqueue_head(&acm->drain_wait);
	spin_lock_init(&acm->throttle_lock);
	spin_lock_init(&acm->write_lock);
	spin_lock_init(&acm->read_lock);
	mutex_init(&acm->mutex);
	acm->rx_endpoint = usb_rcvbulkpipe(usb_dev, epread->bEndpointAddress);
	acm->is_int_ep = usb_endpoint_xfer_int(epread);
	if (acm->is_int_ep)
		acm->bInterval = epread->bInterval;
	if (quirks & NO_HANGUP_IN_RESET_RESUME)
		acm->no_hangup_in_reset_resume = 1;
	tty_port_init(&acm->port);
	acm->port.ops = &acm_port_ops;

	buf = usb_alloc_coherent(usb_dev, ctrlsize, GFP_DMA, &acm->ctrl_dma);
	if (!buf) {
		dev_dbg(&intf->dev, "out of memory (ctrl buffer alloc)\n");
		goto alloc_fail2;
	}
	acm->ctrl_buffer = buf;

	if (acm_write_buffers_alloc(acm) < 0) {
		dev_dbg(&intf->dev, "out of memory (write buffer alloc)\n");
		goto alloc_fail4;
	}

	acm->ctrlurb = usb_alloc_urb(0, GFP_KERNEL);
	if (!acm->ctrlurb) {
		dev_dbg(&intf->dev, "out of memory (ctrlurb kmalloc)\n");
		goto alloc_fail5;
	}
	for (i = 0; i < num_rx_buf; i++) {
		struct acm_ru *rcv = &(acm->ru[i]);

		rcv->urb = usb_alloc_urb(0, GFP_KERNEL);
		if (rcv->urb == NULL) {
			dev_dbg(&intf->dev,
				"out of memory (read urbs usb_alloc_urb)\n");
			goto alloc_fail6;
		}

		rcv->urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
		rcv->instance = acm;
	}
	for (i = 0; i < num_rx_buf; i++) {
		struct acm_rb *rb = &(acm->rb[i]);

		rb->base = usb_alloc_coherent(acm->dev, readsize,
				GFP_DMA, &rb->dma);
		if (!rb->base) {
			dev_dbg(&intf->dev,
				"out of memory (read bufs usb_alloc_coherent)\n");
			goto alloc_fail7;
		}
	}
	for (i = 0; i < ACM_NW; i++) {
		struct acm_wb *snd = &(acm->wb[i]);

		snd->urb = usb_alloc_urb(0, GFP_KERNEL);
		if (snd->urb == NULL) {
			dev_dbg(&intf->dev,
				"out of memory (write urbs usb_alloc_urb)");
			goto alloc_fail8;
		}

		if (usb_endpoint_xfer_int(epwrite))
			usb_fill_int_urb(snd->urb, usb_dev,
				usb_sndbulkpipe(usb_dev, epwrite->bEndpointAddress),
				NULL, acm->writesize, acm_write_bulk, snd, epwrite->bInterval);
		else
			usb_fill_bulk_urb(snd->urb, usb_dev,
				usb_sndbulkpipe(usb_dev, epwrite->bEndpointAddress),
				NULL, acm->writesize, acm_write_bulk, snd);
		snd->urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
		snd->instance = acm;
	}

	usb_set_intfdata(intf, acm);

	i = device_create_file(&intf->dev, &dev_attr_bmCapabilities);
	if (i < 0)
		goto alloc_fail8;

	if (cfd) { /* export the country data */
		acm->country_codes = kmalloc(cfd->bLength - 4, GFP_KERNEL);
		if (!acm->country_codes)
			goto skip_countries;
		acm->country_code_size = cfd->bLength - 4;
		memcpy(acm->country_codes, (u8 *)&cfd->wCountyCode0,
							cfd->bLength - 4);
		acm->country_rel_date = cfd->iCountryCodeRelDate;

		i = device_create_file(&intf->dev, &dev_attr_wCountryCodes);
		if (i < 0) {
			kfree(acm->country_codes);
			goto skip_countries;
		}

		i = device_create_file(&intf->dev,
						&dev_attr_iCountryCodeRelDate);
		if (i < 0) {
			device_remove_file(&intf->dev, &dev_attr_wCountryCodes);
			kfree(acm->country_codes);
			goto skip_countries;
		}
	}

skip_countries:
	usb_fill_int_urb(acm->ctrlurb, usb_dev,
			 usb_rcvintpipe(usb_dev, epctrl->bEndpointAddress),
			 acm->ctrl_buffer, ctrlsize, acm_ctrl_irq, acm,
			 /* works around buggy devices */
			 epctrl->bInterval ? epctrl->bInterval : 0xff);
	acm->ctrlurb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
	acm->ctrlurb->transfer_dma = acm->ctrl_dma;

	dev_info(&intf->dev, "ttyACM%d: USB ACM device\n", minor);

	acm_set_control(acm, acm->ctrlout);

	acm->line.dwDTERate = cpu_to_le32(9600);
	acm->line.bDataBits = 8;
	acm_set_line(acm, &acm->line);

	usb_driver_claim_interface(&acm_driver, data_interface, acm);
	usb_set_intfdata(data_interface, acm);

	usb_get_intf(control_interface);

	acm_table[minor] = acm;

	tty_register_device(acm_tty_driver, minor, &control_interface->dev);
	
	printk("%s: acm_ready_table[minor=%d]=ture\n", __func__,minor);
	acm_ready_table[minor] = true;

	return 0;
alloc_fail8:
	for (i = 0; i < ACM_NW; i++)
		usb_free_urb(acm->wb[i].urb);
alloc_fail7:
	acm_read_buffers_free(acm);
alloc_fail6:
	for (i = 0; i < num_rx_buf; i++)
		usb_free_urb(acm->ru[i].urb);
	usb_free_urb(acm->ctrlurb);
alloc_fail5:
	acm_write_buffers_free(acm);
alloc_fail4:
	usb_free_coherent(usb_dev, ctrlsize, acm->ctrl_buffer, acm->ctrl_dma);
alloc_fail2:
	kfree(acm);
alloc_fail:
	return -ENOMEM;
}

static void stop_data_traffic(struct acm *acm)
{
	int i;
	//dbg("Entering stop_data_traffic");

	if (!acm) {
		pr_err("%s: !acm\n", __func__);
		return;
	}

#if 1 //HTC_CSP_START
	//printk(MODULE_NAME "%s ttyACM%d +\n",__FUNCTION__,acm->minor);
#endif //HTC_CSP_END

	tasklet_disable(&acm->urb_task);

	usb_kill_urb(acm->ctrlurb);
	for (i = 0; i < ACM_NW; i++)
		usb_kill_urb(acm->wb[i].urb);
	for (i = 0; i < acm->rx_buflimit; i++)
		usb_kill_urb(acm->ru[i].urb);

	tasklet_enable(&acm->urb_task);

	cancel_work_sync(&acm->work);


#if 1 //HTC_CSP_START
	//printk(MODULE_NAME "%s ttyACM%d -\n",__FUNCTION__,acm->minor);
#endif //HTC_CSP_END
}

static void acm_disconnect(struct usb_interface *intf)
{
	struct acm *acm = usb_get_intfdata(intf);
	struct usb_device *usb_dev = interface_to_usbdev(intf);
	struct tty_struct *tty;
	struct urb *res;

	int i;
#if 1 //HTC_CSP_START
	if (acm){
		pr_info(MODULE_NAME "%s ttyACM%d + {\n", __func__, acm->minor);
		}else{
		pr_info(MODULE_NAME "%s ttyACM + acm!=1\n", __func__);
		}
#endif //HTC_CSP_END

	for (i=0; i < MAX_ACM_NUM; i++){
		acm_ready_table[i] = false;
	printk("%s: acm_ready_table[%d]=false\n", __func__, i);
}

	/* sibling interface is already cleaning up */
	if (!acm) {
		pr_info(MODULE_NAME "%s: !acm\n", __func__);
		return;
	}

	/* HTC: delete timer, support FLASHLESS first */
	if (timer_on && acm->minor == 0 &&
		(mach_ble_flashless || is_mach_edge || mach_ble_flash)) {
		pr_info(MODULE_NAME ": %s delete acm_timer\n", __func__);
		del_timer(&acm_timer);
		timer_on = 0;
	}

	mutex_lock(&open_mutex);
	if (acm->country_codes) {
		device_remove_file(&acm->control->dev,
				&dev_attr_wCountryCodes);
		device_remove_file(&acm->control->dev,
				&dev_attr_iCountryCodeRelDate);
	}
	device_remove_file(&acm->control->dev, &dev_attr_bmCapabilities);
	acm->dev = NULL;
	usb_set_intfdata(acm->control, NULL);
	usb_set_intfdata(acm->data, NULL);

	stop_data_traffic(acm);

	/* decrement ref count for anchored urbs*/
	while ((res = usb_get_from_anchor(&acm->deferred)))
		usb_put_urb(res);

	acm_write_buffers_free(acm);
	usb_free_coherent(usb_dev, acm->ctrlsize, acm->ctrl_buffer,
			  acm->ctrl_dma);
	acm_read_buffers_free(acm);

	if (!acm->combined_interfaces)
		usb_driver_release_interface(&acm_driver, intf == acm->control ?
					acm->data : acm->control);

	if (acm->port.count == 0) {
		acm_tty_unregister(acm);
		mutex_unlock(&open_mutex);
		return;
	}

	mutex_unlock(&open_mutex);
	tty = tty_port_tty_get(&acm->port);
	if (tty) {
		tty_hangup(tty);
		tty_kref_put(tty);
	}
	
	pr_info(MODULE_NAME "%s ttyACM%d -}\n", __func__, acm->minor);
}

#ifdef CONFIG_PM
static int acm_suspend(struct usb_interface *intf, pm_message_t message)
{
	struct acm *acm = usb_get_intfdata(intf);
	int cnt;
#if 1 //HTC_CSP_START
	printk(MODULE_NAME": %s ttyACM%d +\n",__FUNCTION__,acm->minor);
#endif //HTC_CSP_END

	if (!acm) {
		pr_err("%s: !acm\n", __func__);
		return -EBUSY;
	}

	//pr_info("%s: cnt %d intf=%p &intf->dev=%p kobje=%s\n",
		//	__func__, atomic_read(&intf->dev.power.usage_count),intf,&intf->dev,kobject_name(&intf->dev.kobj));

	if (message.event & PM_EVENT_AUTO) {
		int b;

		spin_lock_irq(&acm->read_lock);
		spin_lock(&acm->write_lock);
		b = acm->processing + acm->transmitting;
		spin_unlock(&acm->write_lock);
		spin_unlock_irq(&acm->read_lock);
		if (b)
#if 1 //HTC_CSP_START
		{
			printk(MODULE_NAME": %s ttyACM%d BUSY -> retry later %d %d\n",
				__func__, acm->minor, acm->processing, acm->transmitting);
			pr_info("%s: ttyACM%d processing %d transmitting %d -EBUSY\n", __func__, acm->minor, acm->processing, acm->transmitting);
			return -EBUSY;
		}
#endif //HTC_CSP_END
	}

	spin_lock_irq(&acm->read_lock);
	spin_lock(&acm->write_lock);
	cnt = acm->susp_count++;
	spin_unlock(&acm->write_lock);
	spin_unlock_irq(&acm->read_lock);

	if (cnt) {
		printk(MODULE_NAME": %s ttyACM%d susp_count=%d (already suspend)\n",
			__func__, acm->minor, acm->susp_count);
		return 0;
	} else
		printk(MODULE_NAME": %s ttyACM%d suspending.... port.count=%d\n",
			__func__, acm->minor, acm->port.count);

	/*
	we treat opened interfaces differently,
	we must guard against open
	*/
	mutex_lock(&acm->mutex);

	if (acm->port.count)
		stop_data_traffic(acm);

	mutex_unlock(&acm->mutex);

#if 1 //HTC_CSP_START
	printk(MODULE_NAME": %s ttyACM%d -\n",__FUNCTION__,acm->minor);
#endif //HTC_CSP_END

	return 0;
}

static int acm_resume(struct usb_interface *intf)
{
	struct acm *acm = usb_get_intfdata(intf);
	struct acm_wb *wb;
	int rv = 0;
	int cnt;
#ifdef CONFIG_PM
	struct urb *res;
#endif

#if 1 //HTC_CSP_START
	pr_info(MODULE_NAME "%s, intf %p, acm %p\n", __func__, intf, acm);
#endif //HTC_CSP_END

	if (!acm) {
		pr_err("%s: !acm\n", __func__);
		return -EBUSY;
	}

	
	//pr_info("%s: cnt %d intf=%p &intf->dev=%p kobje=%s\n",
		//	__func__, atomic_read(&intf->dev.power.usage_count),intf,&intf->dev,kobject_name(&intf->dev.kobj));

	spin_lock_irq(&acm->read_lock);
	if(acm->susp_count > 0) {
		acm->susp_count -= 1;
		cnt = acm->susp_count;
	}else{
		spin_unlock_irq(&acm->read_lock);
		pr_info(MODULE_NAME "%s, workaround hit\n", __func__);
		return 0;
	}
	spin_unlock_irq(&acm->read_lock);
	//add
	//printk(KERN_INFO"jerry usb_mark_last_busy %s\n",__func__);
	//usb_mark_last_busy(acm->dev);

	if (cnt) {
		printk(MODULE_NAME": %s ttyACM%d susp_count=%d (already resumed)\n",
			__func__, acm->minor, acm->susp_count);
		return 0;
	} else
		printk(MODULE_NAME": %s ttyACM%d resuming.....\n",
			__func__, acm->minor, acm->susp_count);

	mutex_lock(&acm->mutex);

	if (acm->port.count) {
		rv = usb_submit_urb(acm->ctrlurb, GFP_NOIO);
		spin_lock_irq(&acm->write_lock);
#ifdef CONFIG_PM
		while ((res = usb_get_from_anchor(&acm->deferred))) {
			/* decrement ref count*/
			usb_put_urb(res);
			pr_info(MODULE_NAME "%s - process buffered request "
				"anchor urb=%p len=%d\n",
				__func__, res, res->transfer_buffer_length);
			rv = usb_submit_urb(res, GFP_ATOMIC);
			if (rv < 0) {
				dbg("usb_submit_urb(pending request)"
					" failed: %d", rv);
				usb_unanchor_urb(res);
				acm_write_done(acm, res->context);
			}
		}
		spin_unlock_irq(&acm->write_lock);
#else
		if (acm->delayed_wb) {
			wb = acm->delayed_wb;
			acm->delayed_wb = NULL;
			spin_unlock_irq(&acm->write_lock);
			acm_start_wb(acm, wb, __func__);
		} else {
			spin_unlock_irq(&acm->write_lock);
		}
#endif

		/*
		 * delayed error checking because we must
		 * do the write path at all cost
		 */
		if (rv < 0)
			goto err_out;

		tasklet_schedule(&acm->urb_task);
	}

err_out:
	mutex_unlock(&acm->mutex);

#if 1 //HTC_CSP_START
	printk(MODULE_NAME": %s ttyACM%d -\n",__FUNCTION__,acm->minor);
#endif //HTC_CSP_END

#if 0
	//test only
	if(debugtestcoung==10){
	kobject_uevent(&acm->dev->dev.kobj,KOBJ_CHANGE);
	printk(KERN_INFO "debugtestcoung uevent test test test !!\n");
	debugtestcoung =0;
	}
	debugtestcoung ++;
	printk(KERN_INFO "debugtestcoung=%d uevent test test test !!\n",debugtestcoung);
#endif
	return rv;
}

static int acm_reset_resume(struct usb_interface *intf)
{
	struct acm *acm = usb_get_intfdata(intf);
	struct tty_struct *tty = NULL;

	printk(KERN_INFO"%s acm->port.count=%d\n",__func__,acm->port.count);
	pr_info(MODULE_NAME "%s - don't hangup ttyacm\n", __func__);

	if (!acm) {
		pr_err("%s: !acm\n", __func__);
		return -EBUSY;
	}
/*
	mutex_lock(&acm->mutex);
	if (acm->port.count) {
		tty = tty_port_tty_get(&acm->port);
		if (tty) {
			pr_info(MODULE_NAME "%s - ttyacm hangup\n", __func__);
			if (!acm->no_hangup_in_reset_resume)
				tty_hangup(tty);
			tty_kref_put(tty);
		}
	}
	mutex_unlock(&acm->mutex);
*/
	return acm_resume(intf);
}

#endif /* CONFIG_PM */

#define NOKIA_PCSUITE_ACM_INFO(x) \
		USB_DEVICE_AND_INTERFACE_INFO(0x0421, x, \
		USB_CLASS_COMM, USB_CDC_SUBCLASS_ACM, \
		USB_CDC_ACM_PROTO_VENDOR)

#define SAMSUNG_PCSUITE_ACM_INFO(x) \
		USB_DEVICE_AND_INTERFACE_INFO(0x04e7, x, \
		USB_CLASS_COMM, USB_CDC_SUBCLASS_ACM, \
		USB_CDC_ACM_PROTO_VENDOR)

/*
 * USB driver structure.
 */

static const struct usb_device_id acm_ids[] = {
	/* quirky and broken devices */
	{ USB_DEVICE(0x0870, 0x0001), /* Metricom GS Modem */
	.driver_info = NO_UNION_NORMAL, /* has no union descriptor */
	},
	{ USB_DEVICE(0x0e8d, 0x0003), /* FIREFLY, MediaTek Inc; andrey.arapov@gmail.com */
	.driver_info = NO_UNION_NORMAL, /* has no union descriptor */
	},
	{ USB_DEVICE(0x0e8d, 0x3329), /* MediaTek Inc GPS */
	.driver_info = NO_UNION_NORMAL, /* has no union descriptor */
	},
	{ USB_DEVICE(0x0482, 0x0203), /* KYOCERA AH-K3001V */
	.driver_info = NO_UNION_NORMAL, /* has no union descriptor */
	},
	{ USB_DEVICE(0x079b, 0x000f), /* BT On-Air USB MODEM */
	.driver_info = NO_UNION_NORMAL, /* has no union descriptor */
	},
	{ USB_DEVICE(0x0ace, 0x1602), /* ZyDAS 56K USB MODEM */
	.driver_info = SINGLE_RX_URB,
	},
	{ USB_DEVICE(0x0ace, 0x1608), /* ZyDAS 56K USB MODEM */
	.driver_info = SINGLE_RX_URB, /* firmware bug */
	},
	{ USB_DEVICE(0x0ace, 0x1611), /* ZyDAS 56K USB MODEM - new version */
	.driver_info = SINGLE_RX_URB, /* firmware bug */
	},
	{ USB_DEVICE(0x22b8, 0x7000), /* Motorola Q Phone */
	.driver_info = NO_UNION_NORMAL, /* has no union descriptor */
	},
	{ USB_DEVICE(0x0803, 0x3095), /* Zoom Telephonics Model 3095F USB MODEM */
	.driver_info = NO_UNION_NORMAL, /* has no union descriptor */
	},
	{ USB_DEVICE(0x0572, 0x1321), /* Conexant USB MODEM CX93010 */
	.driver_info = NO_UNION_NORMAL, /* has no union descriptor */
	},
	{ USB_DEVICE(0x0572, 0x1324), /* Conexant USB MODEM RD02-D400 */
	.driver_info = NO_UNION_NORMAL, /* has no union descriptor */
	},
	{ USB_DEVICE(0x0572, 0x1328), /* Shiro / Aztech USB MODEM UM-3100 */
	.driver_info = NO_UNION_NORMAL, /* has no union descriptor */
	},
	{ USB_DEVICE(0x22b8, 0x6425), /* Motorola MOTOMAGX phones */
	},
	{ USB_DEVICE(0x0572, 0x1329), /* Hummingbird huc56s (Conexant) */
	.driver_info = NO_UNION_NORMAL, /* union descriptor misplaced on
					   data interface instead of
					   communications interface.
					   Maybe we should define a new
					   quirk for this. */
	},
	{ USB_DEVICE(0x1bbb, 0x0003), /* Alcatel OT-I650 */
	.driver_info = NO_UNION_NORMAL, /* reports zero length descriptor */
	},
	{ USB_DEVICE(0x1576, 0x03b1), /* Maretron USB100 */
	.driver_info = NO_UNION_NORMAL, /* reports zero length descriptor */
	},
        //for offline trace first ++
	//{ USB_DEVICE(0x1519, 0x0020),
	//.driver_info = NO_UNION_NORMAL, /* has no union descriptor */
	//},
        //for offline trace first--
	{ USB_DEVICE(0x1519, 0x0020),
	.driver_info = /*NO_UNION_NORMAL |*/ NO_HANGUP_IN_RESET_RESUME, /* has no union descriptor */
	},

	/* Nokia S60 phones expose two ACM channels. The first is
	 * a modem and is picked up by the standard AT-command
	 * information below. The second is 'vendor-specific' but
	 * is treated as a serial device at the S60 end, so we want
	 * to expose it on Linux too. */
	{ NOKIA_PCSUITE_ACM_INFO(0x042D), }, /* Nokia 3250 */
	{ NOKIA_PCSUITE_ACM_INFO(0x04D8), }, /* Nokia 5500 Sport */
	{ NOKIA_PCSUITE_ACM_INFO(0x04C9), }, /* Nokia E50 */
	{ NOKIA_PCSUITE_ACM_INFO(0x0419), }, /* Nokia E60 */
	{ NOKIA_PCSUITE_ACM_INFO(0x044D), }, /* Nokia E61 */
	{ NOKIA_PCSUITE_ACM_INFO(0x0001), }, /* Nokia E61i */
	{ NOKIA_PCSUITE_ACM_INFO(0x0475), }, /* Nokia E62 */
	{ NOKIA_PCSUITE_ACM_INFO(0x0508), }, /* Nokia E65 */
	{ NOKIA_PCSUITE_ACM_INFO(0x0418), }, /* Nokia E70 */
	{ NOKIA_PCSUITE_ACM_INFO(0x0425), }, /* Nokia N71 */
	{ NOKIA_PCSUITE_ACM_INFO(0x0486), }, /* Nokia N73 */
	{ NOKIA_PCSUITE_ACM_INFO(0x04DF), }, /* Nokia N75 */
	{ NOKIA_PCSUITE_ACM_INFO(0x000e), }, /* Nokia N77 */
	{ NOKIA_PCSUITE_ACM_INFO(0x0445), }, /* Nokia N80 */
	{ NOKIA_PCSUITE_ACM_INFO(0x042F), }, /* Nokia N91 & N91 8GB */
	{ NOKIA_PCSUITE_ACM_INFO(0x048E), }, /* Nokia N92 */
	{ NOKIA_PCSUITE_ACM_INFO(0x0420), }, /* Nokia N93 */
	{ NOKIA_PCSUITE_ACM_INFO(0x04E6), }, /* Nokia N93i  */
	{ NOKIA_PCSUITE_ACM_INFO(0x04B2), }, /* Nokia 5700 XpressMusic */
	{ NOKIA_PCSUITE_ACM_INFO(0x0134), }, /* Nokia 6110 Navigator (China) */
	{ NOKIA_PCSUITE_ACM_INFO(0x046E), }, /* Nokia 6110 Navigator */
	{ NOKIA_PCSUITE_ACM_INFO(0x002f), }, /* Nokia 6120 classic &  */
	{ NOKIA_PCSUITE_ACM_INFO(0x0088), }, /* Nokia 6121 classic */
	{ NOKIA_PCSUITE_ACM_INFO(0x00fc), }, /* Nokia 6124 classic */
	{ NOKIA_PCSUITE_ACM_INFO(0x0042), }, /* Nokia E51 */
	{ NOKIA_PCSUITE_ACM_INFO(0x00b0), }, /* Nokia E66 */
	{ NOKIA_PCSUITE_ACM_INFO(0x00ab), }, /* Nokia E71 */
	{ NOKIA_PCSUITE_ACM_INFO(0x0481), }, /* Nokia N76 */
	{ NOKIA_PCSUITE_ACM_INFO(0x0007), }, /* Nokia N81 & N81 8GB */
	{ NOKIA_PCSUITE_ACM_INFO(0x0071), }, /* Nokia N82 */
	{ NOKIA_PCSUITE_ACM_INFO(0x04F0), }, /* Nokia N95 & N95-3 NAM */
	{ NOKIA_PCSUITE_ACM_INFO(0x0070), }, /* Nokia N95 8GB  */
	{ NOKIA_PCSUITE_ACM_INFO(0x00e9), }, /* Nokia 5320 XpressMusic */
	{ NOKIA_PCSUITE_ACM_INFO(0x0099), }, /* Nokia 6210 Navigator, RM-367 */
	{ NOKIA_PCSUITE_ACM_INFO(0x0128), }, /* Nokia 6210 Navigator, RM-419 */
	{ NOKIA_PCSUITE_ACM_INFO(0x008f), }, /* Nokia 6220 Classic */
	{ NOKIA_PCSUITE_ACM_INFO(0x00a0), }, /* Nokia 6650 */
	{ NOKIA_PCSUITE_ACM_INFO(0x007b), }, /* Nokia N78 */
	{ NOKIA_PCSUITE_ACM_INFO(0x0094), }, /* Nokia N85 */
	{ NOKIA_PCSUITE_ACM_INFO(0x003a), }, /* Nokia N96 & N96-3  */
	{ NOKIA_PCSUITE_ACM_INFO(0x00e9), }, /* Nokia 5320 XpressMusic */
	{ NOKIA_PCSUITE_ACM_INFO(0x0108), }, /* Nokia 5320 XpressMusic 2G */
	{ NOKIA_PCSUITE_ACM_INFO(0x01f5), }, /* Nokia N97, RM-505 */
	{ NOKIA_PCSUITE_ACM_INFO(0x02e3), }, /* Nokia 5230, RM-588 */
	{ NOKIA_PCSUITE_ACM_INFO(0x0178), }, /* Nokia E63 */
	{ NOKIA_PCSUITE_ACM_INFO(0x010e), }, /* Nokia E75 */
	{ NOKIA_PCSUITE_ACM_INFO(0x02d9), }, /* Nokia 6760 Slide */
	{ NOKIA_PCSUITE_ACM_INFO(0x01d0), }, /* Nokia E52 */
	{ NOKIA_PCSUITE_ACM_INFO(0x0223), }, /* Nokia E72 */
	{ NOKIA_PCSUITE_ACM_INFO(0x0275), }, /* Nokia X6 */
	{ NOKIA_PCSUITE_ACM_INFO(0x026c), }, /* Nokia N97 Mini */
	{ NOKIA_PCSUITE_ACM_INFO(0x0154), }, /* Nokia 5800 XpressMusic */
	{ NOKIA_PCSUITE_ACM_INFO(0x04ce), }, /* Nokia E90 */
	{ NOKIA_PCSUITE_ACM_INFO(0x01d4), }, /* Nokia E55 */
	{ SAMSUNG_PCSUITE_ACM_INFO(0x6651), }, /* Samsung GTi8510 (INNOV8) */

	/* NOTE: non-Nokia COMM/ACM/0xff is likely MSFT RNDIS... NOT a modem! */

	/* Support Lego NXT using pbLua firmware */
	{ USB_DEVICE(0x0694, 0xff00),
	.driver_info = NOT_A_MODEM,
	},

	/* Exclude XMM6260 boot rom (not running modem software yet) */
	{ USB_DEVICE(0x058b, 0x0041),
	.driver_info = NOT_REAL_ACM,
	},

	/* Icera 450 */
	{ USB_DEVICE(0x1983, 0x0321),
	.driver_info = NO_HANGUP_IN_RESET_RESUME,
	},

	/* control interfaces without any protocol set */
	{ USB_INTERFACE_INFO(USB_CLASS_COMM, USB_CDC_SUBCLASS_ACM,
		USB_CDC_PROTO_NONE) },

	/* control interfaces with various AT-command sets */
	{ USB_INTERFACE_INFO(USB_CLASS_COMM, USB_CDC_SUBCLASS_ACM,
		USB_CDC_ACM_PROTO_AT_V25TER) },
	{ USB_INTERFACE_INFO(USB_CLASS_COMM, USB_CDC_SUBCLASS_ACM,
		USB_CDC_ACM_PROTO_AT_PCCA101) },
	{ USB_INTERFACE_INFO(USB_CLASS_COMM, USB_CDC_SUBCLASS_ACM,
		USB_CDC_ACM_PROTO_AT_PCCA101_WAKE) },
	{ USB_INTERFACE_INFO(USB_CLASS_COMM, USB_CDC_SUBCLASS_ACM,
		USB_CDC_ACM_PROTO_AT_GSM) },
	{ USB_INTERFACE_INFO(USB_CLASS_COMM, USB_CDC_SUBCLASS_ACM,
		USB_CDC_ACM_PROTO_AT_3G) },
	{ USB_INTERFACE_INFO(USB_CLASS_COMM, USB_CDC_SUBCLASS_ACM,
		USB_CDC_ACM_PROTO_AT_CDMA) },

	{ }
};

MODULE_DEVICE_TABLE(usb, acm_ids);

static struct usb_driver acm_driver = {
	.name =		"cdc_acm",
	.probe =	acm_probe,
	.disconnect =	acm_disconnect,
#ifdef CONFIG_PM
	.suspend =	acm_suspend,
	.resume =	acm_resume,
	.reset_resume =	acm_reset_resume,
#endif
	.id_table =	acm_ids,
#ifdef CONFIG_PM
	.supports_autosuspend = 1,
#endif
};

/*
 * TTY driver structures.
 */

static const struct tty_operations acm_ops = {
	.open =			acm_tty_open,
	.close =		acm_tty_close,
	.hangup =		acm_tty_hangup,
	.write =		acm_tty_write,
	.write_room =		acm_tty_write_room,
	.ioctl =		acm_tty_ioctl,
	.throttle =		acm_tty_throttle,
	.unthrottle =		acm_tty_unthrottle,
	.chars_in_buffer =	acm_tty_chars_in_buffer,
	.break_ctl =		acm_tty_break_ctl,
	.set_termios =		acm_tty_set_termios,
	.tiocmget =		acm_tty_tiocmget,
	.tiocmset =		acm_tty_tiocmset,
};

/*
 * Init / exit.
 */

static int __init acm_init(void)
{
	int retval;

#ifdef RIL_DBG
	host_dbg_flag |= DBG_ACM0_RW; /* HTC */
#endif

#if defined(CONFIG_MACH_EDGE) || defined(CONFIG_MACH_ENDEARVORU)
	is_mach_edge = 1;
#endif
#if defined(CONFIG_MACH_BLUE)
	is_mach_ble = 1;
#endif
#if defined(CONFIG_IMC_FLASHLESS)
	is_flashless = 1;
#endif
	mach_ble_flashless = (is_mach_ble && is_flashless);
	mach_ble_flash = (is_mach_ble && !is_flashless);

	pr_info(MODULE_NAME "%s\n", __func__);

	acm_tty_driver = alloc_tty_driver(ACM_TTY_MINORS);
	if (!acm_tty_driver) {
		pr_info(MODULE_NAME "%s - no memory!!\n", __func__);
		return -ENOMEM;
	}
	acm_tty_driver->owner = THIS_MODULE,
	acm_tty_driver->driver_name = "acm",
	acm_tty_driver->name = "ttyACM",
	acm_tty_driver->major = ACM_TTY_MAJOR,
	acm_tty_driver->minor_start = 0,
	acm_tty_driver->type = TTY_DRIVER_TYPE_SERIAL,
	acm_tty_driver->subtype = SERIAL_TYPE_NORMAL,
	acm_tty_driver->flags = TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV;
	acm_tty_driver->init_termios = tty_std_termios;
	acm_tty_driver->init_termios.c_cflag = B9600 | CS8 | CREAD |
								HUPCL | CLOCAL;
	tty_set_operations(acm_tty_driver, &acm_ops);

	retval = tty_register_driver(acm_tty_driver);
	if (retval) {
		put_tty_driver(acm_tty_driver);
		pr_info(MODULE_NAME "%s: tty_register_driver return %d\n", __func__, retval);
		return retval;
	}

	retval = usb_register(&acm_driver);
	if (retval) {
		tty_unregister_driver(acm_tty_driver);
		put_tty_driver(acm_tty_driver);
		pr_info(MODULE_NAME "%s: usb_register return %d\n", __func__, retval);
		return retval;
	}


	/* HTC: timer initial by bert */
	{
		int i;
		for (i=0; i<MAX_ACM_NUM; i++) {
			acm_tp[i].pre_tx_total = acm_tp[i].pre_rx_total =
			acm_tp[i].tx_total = acm_tp[i].rx_total = 0;
		}
	}
	init_timer(&acm_timer);
	acm_timer.data = (unsigned long)NULL;
	acm_timer.function = timer_handler;
	acm_timer.expires = jiffies + msecs_to_jiffies(timer_value_ms);

	printk(KERN_INFO KBUILD_MODNAME ": " DRIVER_VERSION ":"
	       DRIVER_DESC "\n");

	/* HTC */
	pr_info(MODULE_NAME ": %s - host_dbg_flag=0x%x\n",
		__func__, host_dbg_flag);

	return 0;
}

static void __exit acm_exit(void)
{
	usb_deregister(&acm_driver);
	tty_unregister_driver(acm_tty_driver);
	put_tty_driver(acm_tty_driver);
}

module_init(acm_init);
module_exit(acm_exit);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
MODULE_ALIAS_CHARDEV_MAJOR(ACM_TTY_MAJOR);
