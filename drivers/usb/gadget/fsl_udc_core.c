/*
 * Copyright (C) 2004-2007 Freescale Semicondutor, Inc. All rights reserved.
 *
 * Author: Li Yang <leoli@freescale.com>
 *         Jiang Bo <tanya.jiang@freescale.com>
 *
 * Copyright (C) 2010-2011 NVIDIA Corporation
 *
 * Description:
 * Freescale high-speed USB SOC DR module device controller driver.
 * This can be found on MPC8349E/MPC8313E cpus.
 * The driver is previously named as mpc_udc.  Based on bare board
 * code from Dave Liu and Shlomi Gridish.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#undef VERBOSE

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/ioport.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/proc_fs.h>
#include <linux/mm.h>
#include <linux/moduleparam.h>
#include <linux/device.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/usb/otg.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>
#include <linux/dmapool.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <linux/workqueue.h>

#include <linux/gpio.h>
#include <asm/byteorder.h>
#include <asm/io.h>
#include <asm/system.h>
#include <asm/unaligned.h>
#include <asm/dma.h>
#include <mach/cable_detect.h>
#include <linux/wakelock.h>
#include "fsl_usb2_udc.h"
#include <mach/atmega_microp.h>
#include <mach/board_htc.h>
#include <mach/usb_phy.h>
#include <linux/tps80032_adc.h>
#include <mach/iomap.h>

#define VOL_LEVEL_5PIN_UPPER 715
#define VOL_LEVEL_5PIN_LOWER 585
#define VOL_LEVEL_37PIN_UPPER 140
#define VOL_LEVEL_37PIN_LOWER 97
#define VOL_LEVEL_DONGLE_UPPER 46
#define VOL_LEVEL_DONGLE_LOWER 31

#define TEGRA_GPIO_PC7         23

#define TEGRA_GPIO_PO1         113
#define TEGRA_GPIO_PO2         114
#define TEGRA_GPIO_PH3         59

#define UART_USB_SW             TEGRA_GPIO_PH3
#define UART1_DEBUG_TX          TEGRA_GPIO_PO1
#define UART1_DEBUG_RX          TEGRA_GPIO_PO2
#define CHARGER_PIN_REC		TEGRA_GPIO_PC7
#define D_PLUS_BIT 0x0400
#define D_MINUS_BIT 0x0800

#define  USB_ID_INT_EN			(1 << 0)
#define  USB_ID_INT_STATUS		(1 << 1)
#define  USB_ID_STATUS			(1 << 2)
#define  USB_ID_PIN_WAKEUP_EN	(1 << 6)
#define  USB_VBUS_WAKEUP_EN		(1 << 30)
#define  USB_VBUS_INT_EN		(1 << 8)
#define  USB_VBUS_INT_STATUS	(1 << 9)
#define  USB_VBUS_STATUS		(1 << 10)

#define VUBS_IRQ -22
#define VBUS_WAKEUP_ENR 19
extern global_wakeup_state;

static int irq_udc_debug;
int irq_otg_debug;

enum charger_pin_type {
	CHARGER_TYPE_5,
	CHARGER_TYPE_32_5,
	CHARGER_TYPE_HDMI_5,
	CHARGER_TYPE_HDMI_32_5,
};
//#define DEBUG_MSG
#ifdef DEBUG_MSG
#define USB_DEBUG(fmt, args...) \
	printk(KERN_DEBUG "[USBD] " fmt, ## args)
#else
#define USB_DEBUG(fmt,args...) \
	do { } while (0)
#endif	/* RDEBUG */

#define USB_INFO(fmt, args...) \
	printk(KERN_INFO "[USBUDC] " fmt, ## args)
#define USB_WARNING(fmt, args...) \
	printk(KERN_WARNING "[USBUDC] " fmt, ## args)
#define USB_ERR(fmt, args...) \
	pr_err("[USBUDC] " fmt, ## args)

#ifdef CONFIG_ARCH_TEGRA
#define	DRIVER_DESC	"NVidia Tegra High-Speed USB SOC Device Controller driver"
#else
#define	DRIVER_DESC	"Freescale High-Speed USB SOC Device Controller driver"
#endif
#define	DRIVER_AUTHOR	"Li Yang/Jiang Bo"
#define	DRIVER_VERSION	"Apr 20, 2007"

#define	DMA_ADDR_INVALID	(~(dma_addr_t)0)
#define	STATUS_BUFFER_SIZE	8
#define USB1_PREFETCH_ID       6

#ifdef CONFIG_ARCH_TEGRA
static const char driver_name[] = "fsl-tegra-udc";
#else
static const char driver_name[] = "fsl-usb2-udc";
#endif
static const char driver_desc[] = DRIVER_DESC;

static struct usb_dr_device *dr_regs;
#ifndef CONFIG_ARCH_MXC
static struct usb_sys_interface *usb_sys_regs;
#endif
#define UTMIP_HSRX_CFG1		0x814
#define UTMIP_HS_SYNC_START_DLY(x)	(((x) & 0x1f) << 1)


/* Charger current limit=1800mA, as per the USB charger spec */
#define USB_CHARGING_CURRENT_LIMIT_MA 1800
/* 1 sec wait time for charger detection after vbus is detected */
#define USB_CHARGER_DETECTION_WAIT_TIME_MS 1000

static struct wake_lock udc_wake_lock;
static struct wake_lock udc_wake_lock2;
struct wake_lock udc_resume_wake_lock;

/* it is initialized in probe()  */
struct fsl_udc *udc_controller = NULL;

static const struct usb_endpoint_descriptor
fsl_ep0_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	0,
	.bmAttributes =		USB_ENDPOINT_XFER_CONTROL,
	.wMaxPacketSize =	USB_MAX_CTRL_PAYLOAD,
};

static u32 *control_reg = NULL;
static void fsl_ep_fifo_flush(struct usb_ep *_ep);
static int reset_queues(struct fsl_udc *udc);

#ifdef CONFIG_PPC32
#define fsl_readl(addr)		in_le32(addr)
#define fsl_writel(val32, addr) out_le32(addr, val32)
#else
#define fsl_readl(addr)		readl(addr)
#define fsl_writel(val32, addr) writel(val32, addr)
#endif

static int usb_check_count;
static int first_online;
static void usb_vbus_state_work(struct work_struct *w);

#define USB_STATE_IDLE    0
#define USB_STATE_ONLINE  1
#define USB_STATE_OFFLINE 2

#define USB_FLAG_START          0x0001
#define USB_FLAG_VBUS_ONLINE    0x0002
#define USB_FLAG_VBUS_OFFLINE   0x0004
#define USB_FLAG_RESET          0x0008
#define USB_FLAG_CONFIGURED     0x0020

#define PHY_STATUS_CHECK_DELAY	(jiffies + msecs_to_jiffies(1000))

/*
 * High speed test mode packet(53 bytes).
 * See USB 2.0 spec, section 7.1.20.
 */
static const u8 fsl_udc_test_packet[53] = {
	/* JKJKJKJK x9 */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	/* JJKKJJKK x8 */
	0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa,
	/* JJJJKKKK x8 */
	0xee, 0xee, 0xee, 0xee, 0xee, 0xee, 0xee, 0xee,
	/* JJJJJJJKKKKKKK x8 */
	0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	/* JJJJJJJK x8 */
	0x7f, 0xbf, 0xdf, 0xef, 0xf7, 0xfb, 0xfd,
	/* JKKKKKKK x10, JK */
	0xfc, 0x7e, 0xbf, 0xdf, 0xef, 0xf7, 0xfb, 0xfd, 0x7e
};


static void usb_start(struct fsl_udc *udc);
static void update_wake_lock(int status);

/********************************************************************
 *	Internal Used Function
********************************************************************/
/*-----------------------------------------------------------------
 * vbus_enabled() - checks vbus status
 *--------------------------------------------------------------*/
static inline bool vbus_enabled(void)
{
	bool status = false;
#ifdef CONFIG_TEGRA_SILICON_PLATFORM
	status = (fsl_readl(&usb_sys_regs->vbus_wakeup) & USB_SYS_VBUS_STATUS);
#else
	/*On FPGA VBUS is detected through VBUS A Session instead of VBUS status. */
	status = (fsl_readl(&usb_sys_regs->vbus_sensors) & USB_SYS_VBUS_ASESSION);
#endif
	return status;
}
#ifdef CONFIG_USB_ID_WORKAROUND
int usb_get_id_status(void)
{

	unsigned long val;
	unsigned long status;
	val =fsl_readl(&usb_sys_regs->vbus_wakeup);

	val |= (USB_VBUS_INT_EN | USB_VBUS_WAKEUP_EN);
	val |= (USB_ID_INT_EN | USB_ID_PIN_WAKEUP_EN);

	if ((val & USB_ID_STATUS) && (val & USB_VBUS_STATUS)) {
		val |= USB_VBUS_INT_STATUS;
	} else if (!(val & USB_ID_STATUS)) {
		val |= USB_ID_INT_STATUS;
	} else {
		val &= ~(USB_ID_INT_STATUS | USB_VBUS_INT_STATUS);
	}

	if ((val & USB_ID_INT_STATUS) || (val & USB_VBUS_INT_STATUS)) {
		status = val;
		if (status & USB_ID_STATUS)
			return 1;
	}
	return 0;

}
EXPORT_SYMBOL(usb_get_id_status);

#endif

/*-----------------------------------------------------------------
 * done() - retire a request; caller blocked irqs
 * @status : request status to be set, only works when
 *	request is still in progress.
 *--------------------------------------------------------------*/
static void done(struct fsl_ep *ep, struct fsl_req *req, int status)
{
	struct fsl_udc *udc = NULL;
	unsigned char stopped = ep->stopped;
	struct ep_td_struct *curr_td, *next_td;
	int j;

	udc = (struct fsl_udc *)ep->udc;
	/* Removed the req from fsl_ep->queue */
	list_del_init(&req->queue);

	/* req.status should be set as -EINPROGRESS in ep_queue() */
	if (req->req.status == -EINPROGRESS)
		req->req.status = status;
	else
		status = req->req.status;

	/* Free dtd for the request */
	next_td = req->head;
	for (j = 0; j < req->dtd_count; j++) {
		curr_td = next_td;
		if (j != req->dtd_count - 1) {
			next_td = curr_td->next_td_virt;
		}
		dma_pool_free(udc->td_pool, curr_td, curr_td->td_dma);
	}

	if (req->mapped) {
		dma_unmap_single(ep->udc->gadget.dev.parent,
			req->req.dma, req->req.length,
			ep_is_in(ep)
				? DMA_TO_DEVICE
				: DMA_FROM_DEVICE);
		req->req.dma = DMA_ADDR_INVALID;
		req->mapped = 0;
	} else
		dma_sync_single_for_cpu(ep->udc->gadget.dev.parent,
			req->req.dma, req->req.length,
			ep_is_in(ep)
				? DMA_TO_DEVICE
				: DMA_FROM_DEVICE);

	if (status && (status != -ESHUTDOWN))
		VDBG("complete %s req %p stat %d len %u/%u",
			ep->ep.name, &req->req, status,
			req->req.actual, req->req.length);

	ep->stopped = 1;

	spin_unlock(&ep->udc->lock);
	/* complete() is from gadget layer,
	 * eg fsg->bulk_in_complete() */
	if (req->req.complete)
		req->req.complete(&ep->ep, &req->req);

	spin_lock(&ep->udc->lock);
	ep->stopped = stopped;
}

/*-----------------------------------------------------------------
 * nuke(): delete all requests related to this ep
 * called with spinlock held
 *--------------------------------------------------------------*/
static void nuke(struct fsl_ep *ep, int status)
{
	ep->stopped = 1;

	/* Flush fifo */
	fsl_ep_fifo_flush(&ep->ep);

	/* Whether this eq has request linked */
	while (!list_empty(&ep->queue)) {
		struct fsl_req *req = NULL;

		req = list_entry(ep->queue.next, struct fsl_req, queue);
		done(ep, req, status);
	}
}

/*------------------------------------------------------------------
	Internal Hardware related function
 ------------------------------------------------------------------*/

#define FSL_UDC_RESET_TIMEOUT 1000
static int dr_controller_reset(struct fsl_udc *udc)
{
	unsigned int tmp;
	unsigned long timeout;
USB_DEBUG("## %s",__func__);
	/* Stop and reset the usb controller */
	tmp = fsl_readl(&dr_regs->usbcmd);
	tmp &= ~USB_CMD_RUN_STOP;
	fsl_writel(tmp, &dr_regs->usbcmd);

	tmp = fsl_readl(&dr_regs->usbcmd);
	tmp |= USB_CMD_CTRL_RESET;
	fsl_writel(tmp, &dr_regs->usbcmd);

	/* Wait for reset to complete */
	timeout = jiffies + FSL_UDC_RESET_TIMEOUT;
	while (fsl_readl(&dr_regs->usbcmd) & USB_CMD_CTRL_RESET) {
		if (time_after(jiffies, timeout)) {
			ERR("udc reset timeout!\n");
			return -ETIMEDOUT;
		}
		cpu_relax();
	}
	return 0;
}

static int dr_controller_setup(struct fsl_udc *udc)
{
	unsigned int tmp, portctrl;
#if !defined(CONFIG_ARCH_MXC) && !defined(CONFIG_ARCH_TEGRA)
	unsigned int ctrl;
#endif
#ifdef CONFIG_ARCH_TEGRA
	unsigned long timeout;
#endif
	int status;
USB_DEBUG("## %s",__func__);
	/* Config PHY interface */
	portctrl = fsl_readl(control_reg);
	portctrl &= ~(PORTSCX_PHY_TYPE_SEL | PORTSCX_PORT_WIDTH);
	switch (udc->phy_mode) {
	case FSL_USB2_PHY_ULPI:
		portctrl |= PORTSCX_PTS_ULPI;
		break;
	case FSL_USB2_PHY_UTMI_WIDE:
		portctrl |= PORTSCX_PTW_16BIT;
		/* fall through */
	case FSL_USB2_PHY_UTMI:
		portctrl |= PORTSCX_PTS_UTMI;
		break;
	case FSL_USB2_PHY_SERIAL:
		portctrl |= PORTSCX_PTS_FSLS;
		break;
	default:
		return -EINVAL;
	}
	fsl_writel(portctrl, control_reg);

	status = dr_controller_reset(udc);
	if (status)
		return status;

	/* Set the controller as device mode */
	tmp = fsl_readl(&dr_regs->usbmode);
	tmp |= USB_MODE_CTRL_MODE_DEVICE;
	/* Disable Setup Lockout */
	tmp |= USB_MODE_SETUP_LOCK_OFF;
	tmp |= USB_MODE_STREAM_DISABLE;
	fsl_writel(tmp, &dr_regs->usbmode);

#ifdef CONFIG_ARCH_TEGRA
	/* Wait for controller to switch to device mode */
	timeout = jiffies + FSL_UDC_RESET_TIMEOUT;
	while ((fsl_readl(&dr_regs->usbmode) & USB_MODE_CTRL_MODE_DEVICE) !=
	       USB_MODE_CTRL_MODE_DEVICE) {
		if (time_after(jiffies, timeout)) {
			ERR("udc device mode setup timeout!\n");
			return -ETIMEDOUT;
		}
		cpu_relax();
	}
#endif

	/* Clear the setup status */
	fsl_writel(0, &dr_regs->usbsts);

	tmp = udc->ep_qh_dma;
	tmp &= USB_EP_LIST_ADDRESS_MASK;
	fsl_writel(tmp, &dr_regs->endpointlistaddr);

	VDBG("vir[qh_base] is %p phy[qh_base] is 0x%8x reg is 0x%8x",
		udc->ep_qh, (int)tmp,
		fsl_readl(&dr_regs->endpointlistaddr));

	/* Config control enable i/o output, cpu endian register */
#if !defined(CONFIG_ARCH_MXC) && !defined(CONFIG_ARCH_TEGRA)
	ctrl = __raw_readl(&usb_sys_regs->control);
	ctrl |= USB_CTRL_IOENB;
	__raw_writel(ctrl, &usb_sys_regs->control);
#endif

#if defined(CONFIG_PPC32) && !defined(CONFIG_NOT_COHERENT_CACHE)
	/* Turn on cache snooping hardware, since some PowerPC platforms
	 * wholly rely on hardware to deal with cache coherent. */

	/* Setup Snooping for all the 4GB space */
	tmp = SNOOP_SIZE_2GB;	/* starts from 0x0, size 2G */
	__raw_writel(tmp, &usb_sys_regs->snoop1);
	tmp |= 0x80000000;	/* starts from 0x8000000, size 2G */
	__raw_writel(tmp, &usb_sys_regs->snoop2);
#endif

	return 0;
}

/* Enable DR irq and set controller to run state */
static void dr_controller_run(struct fsl_udc *udc)
{
	u32 temp;
#ifdef CONFIG_ARCH_TEGRA
	unsigned long timeout;
#define FSL_UDC_RUN_TIMEOUT 1000
#endif
	/* Clear stopped bit */
	udc->stopped = 0;
USB_DEBUG("## %s",__func__);
/* If OTG transceiver is available, then it handles the VBUS detection */
	if (!udc_controller->transceiver) {
#ifdef CONFIG_TEGRA_SILICON_PLATFORM
		/* Enable cable detection interrupt, without setting the
		 * USB_SYS_VBUS_WAKEUP_INT bit. USB_SYS_VBUS_WAKEUP_INT is
		 * clear on write */
		temp = fsl_readl(&usb_sys_regs->vbus_wakeup);
		temp |= (USB_SYS_VBUS_WAKEUP_INT_ENABLE | USB_SYS_VBUS_WAKEUP_ENABLE);
		temp &= ~USB_SYS_VBUS_WAKEUP_INT_STATUS;
		fsl_writel(temp, &usb_sys_regs->vbus_wakeup);
#else
		/*On FPGA VBUS is detected through VBUS A Session instead of VBUS
		 * status. */
		temp = fsl_readl(&usb_sys_regs->vbus_sensors);
		temp |= USB_SYS_VBUS_ASESSION_INT_EN;
		temp &= ~USB_SYS_VBUS_ASESSION_CHANGED;
		fsl_writel(temp, &usb_sys_regs->vbus_sensors);
#endif
	}
	/* Enable DR irq reg */
	temp = USB_INTR_INT_EN | USB_INTR_ERR_INT_EN
		| USB_INTR_PTC_DETECT_EN | USB_INTR_RESET_EN
		| USB_INTR_DEVICE_SUSPEND | USB_INTR_SYS_ERR_EN;

	fsl_writel(temp, &dr_regs->usbintr);

	/* Set the controller as device mode */
	temp = fsl_readl(&dr_regs->usbmode);
	temp |= USB_MODE_CTRL_MODE_DEVICE;
	temp |= USB_MODE_STREAM_DISABLE;
	fsl_writel(temp, &dr_regs->usbmode);

	/* Set controller to Run */
	temp = fsl_readl(&dr_regs->usbcmd);
	temp |= USB_CMD_RUN_STOP;
	fsl_writel(temp, &dr_regs->usbcmd);

#ifdef CONFIG_ARCH_TEGRA
	/* Wait for controller to start */
	timeout = jiffies + FSL_UDC_RUN_TIMEOUT;
	while ((fsl_readl(&dr_regs->usbcmd) & USB_CMD_RUN_STOP) !=
	       USB_CMD_RUN_STOP) {
		if (time_after(jiffies, timeout)) {
			ERR("udc start timeout!\n");
			return;
		}
		cpu_relax();
	}
#endif

	return;
}

static void dr_controller_stop(struct fsl_udc *udc)
{
	unsigned int tmp;
USB_DEBUG("## %s",__func__);
	/* Clear pending interrupt status bits */
	tmp = fsl_readl(&dr_regs->usbsts);
	fsl_writel(tmp, &dr_regs->usbsts);

	/* disable all INTR */
	fsl_writel(0, &dr_regs->usbintr);

	/* Set stopped bit for isr */
	udc->stopped = 1;

	/* disable IO output */
/*	usb_sys_regs->control = 0; */

	/* set controller to Stop */
	tmp = fsl_readl(&dr_regs->usbcmd);
	tmp &= ~USB_CMD_RUN_STOP;
	fsl_writel(tmp, &dr_regs->usbcmd);
}

static void dr_ep_setup(unsigned char ep_num, unsigned char dir,
			unsigned char ep_type)
{
	unsigned int tmp_epctrl = 0;

	tmp_epctrl = fsl_readl(&dr_regs->endptctrl[ep_num]);
	if (dir) {
		if (ep_num)
			tmp_epctrl |= EPCTRL_TX_DATA_TOGGLE_RST;
		tmp_epctrl |= EPCTRL_TX_ENABLE;
		tmp_epctrl |= ((unsigned int)(ep_type)
				<< EPCTRL_TX_EP_TYPE_SHIFT);
	} else {
		if (ep_num)
			tmp_epctrl |= EPCTRL_RX_DATA_TOGGLE_RST;
		tmp_epctrl |= EPCTRL_RX_ENABLE;
		tmp_epctrl |= ((unsigned int)(ep_type)
				<< EPCTRL_RX_EP_TYPE_SHIFT);
	}

	fsl_writel(tmp_epctrl, &dr_regs->endptctrl[ep_num]);
}

static void
dr_ep_change_stall(unsigned char ep_num, unsigned char dir, int value)
{
	u32 tmp_epctrl = 0;

	tmp_epctrl = fsl_readl(&dr_regs->endptctrl[ep_num]);

	if (value) {
		/* set the stall bit */
		if (dir)
			tmp_epctrl |= EPCTRL_TX_EP_STALL;
		else
			tmp_epctrl |= EPCTRL_RX_EP_STALL;
	} else {
		/* clear the stall bit and reset data toggle */
		if (dir) {
			tmp_epctrl &= ~EPCTRL_TX_EP_STALL;
			tmp_epctrl |= EPCTRL_TX_DATA_TOGGLE_RST;
		} else {
			tmp_epctrl &= ~EPCTRL_RX_EP_STALL;
			tmp_epctrl |= EPCTRL_RX_DATA_TOGGLE_RST;
		}
	}
	fsl_writel(tmp_epctrl, &dr_regs->endptctrl[ep_num]);
}

/* Get stall status of a specific ep
   Return: 0: not stalled; 1:stalled */
static int dr_ep_get_stall(unsigned char ep_num, unsigned char dir)
{
	u32 epctrl;

	epctrl = fsl_readl(&dr_regs->endptctrl[ep_num]);
	if (dir)
		return (epctrl & EPCTRL_TX_EP_STALL) ? 1 : 0;
	else
		return (epctrl & EPCTRL_RX_EP_STALL) ? 1 : 0;
}

/********************************************************************
	Internal Structure Build up functions
********************************************************************/

/*------------------------------------------------------------------
* struct_ep_qh_setup(): set the Endpoint Capabilites field of QH
 * @zlt: Zero Length Termination Select (1: disable; 0: enable)
 * @mult: Mult field
 ------------------------------------------------------------------*/
static void struct_ep_qh_setup(struct fsl_udc *udc, unsigned char ep_num,
		unsigned char dir, unsigned char ep_type,
		unsigned int max_pkt_len,
		unsigned int zlt, unsigned char mult)
{
	struct ep_queue_head *p_QH = &udc->ep_qh[2 * ep_num + dir];
	unsigned int tmp = 0;

	/* set the Endpoint Capabilites in QH */
	switch (ep_type) {
	case USB_ENDPOINT_XFER_CONTROL:
		/* Interrupt On Setup (IOS). for control ep  */
		tmp = (max_pkt_len << EP_QUEUE_HEAD_MAX_PKT_LEN_POS)
			| EP_QUEUE_HEAD_IOS;
		break;
	case USB_ENDPOINT_XFER_ISOC:
		tmp = (max_pkt_len << EP_QUEUE_HEAD_MAX_PKT_LEN_POS)
			| (mult << EP_QUEUE_HEAD_MULT_POS);
		break;
	case USB_ENDPOINT_XFER_BULK:
	case USB_ENDPOINT_XFER_INT:
		tmp = max_pkt_len << EP_QUEUE_HEAD_MAX_PKT_LEN_POS;
		break;
	default:
		VDBG("error ep type is %d", ep_type);
		return;
	}
	if (zlt)
		tmp |= EP_QUEUE_HEAD_ZLT_SEL;

	p_QH->max_pkt_length = cpu_to_le32(tmp);
	p_QH->next_dtd_ptr = 1;
	p_QH->size_ioc_int_sts = 0;
}

/* Setup qh structure and ep register for ep0. */
static void ep0_setup(struct fsl_udc *udc)
{
	/* the intialization of an ep includes: fields in QH, Regs,
	 * fsl_ep struct */
	struct_ep_qh_setup(udc, 0, USB_RECV, USB_ENDPOINT_XFER_CONTROL,
			USB_MAX_CTRL_PAYLOAD, 1, 0);
	struct_ep_qh_setup(udc, 0, USB_SEND, USB_ENDPOINT_XFER_CONTROL,
			USB_MAX_CTRL_PAYLOAD, 1, 0);
	dr_ep_setup(0, USB_RECV, USB_ENDPOINT_XFER_CONTROL);
	dr_ep_setup(0, USB_SEND, USB_ENDPOINT_XFER_CONTROL);

	return;

}

/***********************************************************************
		Endpoint Management Functions
***********************************************************************/

/*-------------------------------------------------------------------------
 * when configurations are set, or when interface settings change
 * for example the do_set_interface() in gadget layer,
 * the driver will enable or disable the relevant endpoints
 * ep0 doesn't use this routine. It is always enabled.
-------------------------------------------------------------------------*/
static int fsl_ep_enable(struct usb_ep *_ep,
		const struct usb_endpoint_descriptor *desc)
{
	struct fsl_udc *udc = NULL;
	struct fsl_ep *ep = NULL;
	unsigned short max = 0;
	unsigned char mult = 0, zlt;
	int retval = -EINVAL;
	unsigned long flags = 0;

	ep = container_of(_ep, struct fsl_ep, ep);

	/* catch various bogus parameters */
	if (!_ep || !desc || ep->desc
			|| (desc->bDescriptorType != USB_DT_ENDPOINT))
		return -EINVAL;

	udc = ep->udc;

	if (!udc->driver || (udc->gadget.speed == USB_SPEED_UNKNOWN))
		return -ESHUTDOWN;

	max = le16_to_cpu(desc->wMaxPacketSize);

	/* Disable automatic zlp generation.  Driver is responsible to indicate
	 * explicitly through req->req.zero.  This is needed to enable multi-td
	 * request. */
	zlt = 1;

	/* Assume the max packet size from gadget is always correct */
	switch (desc->bmAttributes & 0x03) {
	case USB_ENDPOINT_XFER_CONTROL:
	case USB_ENDPOINT_XFER_BULK:
	case USB_ENDPOINT_XFER_INT:
		/* mult = 0.  Execute N Transactions as demonstrated by
		 * the USB variable length packet protocol where N is
		 * computed using the Maximum Packet Length (dQH) and
		 * the Total Bytes field (dTD) */
		mult = 0;
		break;
	case USB_ENDPOINT_XFER_ISOC:
		/* Calculate transactions needed for high bandwidth iso */
		mult = (unsigned char)(1 + ((max >> 11) & 0x03));
		max = max & 0x7ff;	/* bit 0~10 */
		/* 3 transactions at most */
		if (mult > 3)
			goto en_done;
		break;
	default:
		goto en_done;
	}

	spin_lock_irqsave(&udc->lock, flags);
	ep->ep.maxpacket = max;
	ep->desc = desc;
	ep->stopped = 0;

	/* Controller related setup */
	/* Init EPx Queue Head (Ep Capabilites field in QH
	 * according to max, zlt, mult) */
	struct_ep_qh_setup(udc, (unsigned char) ep_index(ep),
			(unsigned char) ((desc->bEndpointAddress & USB_DIR_IN)
					?  USB_SEND : USB_RECV),
			(unsigned char) (desc->bmAttributes
					& USB_ENDPOINT_XFERTYPE_MASK),
			max, zlt, mult);

	/* Init endpoint ctrl register */
	dr_ep_setup((unsigned char) ep_index(ep),
			(unsigned char) ((desc->bEndpointAddress & USB_DIR_IN)
					? USB_SEND : USB_RECV),
			(unsigned char) (desc->bmAttributes
					& USB_ENDPOINT_XFERTYPE_MASK));

	spin_unlock_irqrestore(&udc->lock, flags);
	retval = 0;

	VDBG("enabled %s (ep%d%s) maxpacket %d",ep->ep.name,
			ep->desc->bEndpointAddress & 0x0f,
			(desc->bEndpointAddress & USB_DIR_IN)
				? "in" : "out", max);
en_done:
	return retval;
}

/*---------------------------------------------------------------------
 * @ep : the ep being unconfigured. May not be ep0
 * Any pending and uncomplete req will complete with status (-ESHUTDOWN)
*---------------------------------------------------------------------*/
static int fsl_ep_disable(struct usb_ep *_ep)
{
	struct fsl_udc *udc = NULL;
	struct fsl_ep *ep = NULL;
	unsigned long flags = 0;
	u32 epctrl;
	int ep_num;

	ep = container_of(_ep, struct fsl_ep, ep);
	if (!_ep || !ep->desc) {
		VDBG("%s not enabled", _ep ? ep->ep.name : NULL);
		return -EINVAL;
	}

	/* disable ep on controller */
	ep_num = ep_index(ep);
#if defined(CONFIG_ARCH_TEGRA)
	/* Touch the registers if cable is connected and phy is on */
	if (vbus_enabled())
#endif
	{
		epctrl = fsl_readl(&dr_regs->endptctrl[ep_num]);
		if (ep_is_in(ep))
			epctrl &= ~EPCTRL_TX_ENABLE;
		else
			epctrl &= ~EPCTRL_RX_ENABLE;
		fsl_writel(epctrl, &dr_regs->endptctrl[ep_num]);
	}

	udc = (struct fsl_udc *)ep->udc;
	spin_lock_irqsave(&udc->lock, flags);

	/* nuke all pending requests (does flush) */
	nuke(ep, -ESHUTDOWN);

	ep->desc = NULL;
	ep->stopped = 1;
	spin_unlock_irqrestore(&udc->lock, flags);

	VDBG("disabled %s OK", _ep->name);
	return 0;
}

/*---------------------------------------------------------------------
 * allocate a request object used by this endpoint
 * the main operation is to insert the req->queue to the eq->queue
 * Returns the request, or null if one could not be allocated
*---------------------------------------------------------------------*/
static struct usb_request *
fsl_alloc_request(struct usb_ep *_ep, gfp_t gfp_flags)
{
	struct fsl_req *req = NULL;

	req = kzalloc(sizeof *req, gfp_flags);
	if (!req)
		return NULL;

	req->req.dma = DMA_ADDR_INVALID;
	INIT_LIST_HEAD(&req->queue);

	return &req->req;
}

static void fsl_free_request(struct usb_ep *_ep, struct usb_request *_req)
{
	struct fsl_req *req = NULL;

	req = container_of(_req, struct fsl_req, req);

	if (_req)
		kfree(req);
}

/*-------------------------------------------------------------------------*/
static void fsl_queue_td(struct fsl_ep *ep, struct fsl_req *req)
{
	int i = ep_index(ep) * 2 + ep_is_in(ep);
	u32 temp, bitmask, tmp_stat;
	struct ep_queue_head *dQH = &ep->udc->ep_qh[i];

	/* VDBG("QH addr Register 0x%8x", dr_regs->endpointlistaddr);
	VDBG("ep_qh[%d] addr is 0x%8x", i, (u32)&(ep->udc->ep_qh[i])); */

	bitmask = ep_is_in(ep)
		? (1 << (ep_index(ep) + 16))
		: (1 << (ep_index(ep)));

	/* Flush all the dTD structs out to memory */
	wmb();

	/* check if the pipe is empty */
	if (!(list_empty(&ep->queue))) {
		/* Add td to the end */
		struct fsl_req *lastreq;
		lastreq = list_entry(ep->queue.prev, struct fsl_req, queue);
		lastreq->tail->next_td_ptr =
			cpu_to_le32(req->head->td_dma & DTD_ADDR_MASK);
		wmb();
		/* Read prime bit, if 1 goto done */
		if (fsl_readl(&dr_regs->endpointprime) & bitmask)
			goto out;

		do {
			/* Set ATDTW bit in USBCMD */
			temp = fsl_readl(&dr_regs->usbcmd);
			fsl_writel(temp | USB_CMD_ATDTW, &dr_regs->usbcmd);

			/* Read correct status bit */
			tmp_stat = fsl_readl(&dr_regs->endptstatus) & bitmask;

		} while (!(fsl_readl(&dr_regs->usbcmd) & USB_CMD_ATDTW));

		/* Write ATDTW bit to 0 */
		temp = fsl_readl(&dr_regs->usbcmd);
		fsl_writel(temp & ~USB_CMD_ATDTW, &dr_regs->usbcmd);

		if (tmp_stat)
			goto out;
		else {
			if(!(dQH->next_dtd_ptr &
			    cpu_to_le32(DTD_NEXT_TERMINATE)))
				goto prime;
		}
	}

	/* Write dQH next pointer and terminate bit to 0 */
	temp = req->head->td_dma & EP_QUEUE_HEAD_NEXT_POINTER_MASK;
	dQH->next_dtd_ptr = cpu_to_le32(temp);
prime:
	/* Clear active and halt bit */
	temp = cpu_to_le32(~(EP_QUEUE_HEAD_STATUS_ACTIVE
			| EP_QUEUE_HEAD_STATUS_HALT));
	dQH->size_ioc_int_sts &= temp;

	/* Ensure that updates to the QH will occur before priming. */
	wmb();

	/* Prime endpoint by writing 1 to ENDPTPRIME */
	temp = ep_is_in(ep)
		? (1 << (ep_index(ep) + 16))
		: (1 << (ep_index(ep)));
	fsl_writel(temp, &dr_regs->endpointprime);
out:
	return;
}

/* Fill in the dTD structure
 * @req: request that the transfer belongs to
 * @length: return actually data length of the dTD
 * @dma: return dma address of the dTD
 * @is_last: return flag if it is the last dTD of the request
 * return: pointer to the built dTD */
static struct ep_td_struct *fsl_build_dtd(struct fsl_req *req, unsigned *length,
		dma_addr_t *dma, int *is_last, gfp_t gfp_flags)
{
	u32 swap_temp;
	struct ep_td_struct *dtd;

	/* how big will this transfer be? */
	*length = min(req->req.length - req->req.actual,
			(unsigned)EP_MAX_LENGTH_TRANSFER);

	dtd = dma_pool_alloc(udc_controller->td_pool, gfp_flags, dma);
	if (dtd == NULL)
		return dtd;

	dtd->td_dma = *dma;
	/* Clear reserved field */
	swap_temp = cpu_to_le32(dtd->size_ioc_sts);
	swap_temp &= ~DTD_RESERVED_FIELDS;
	dtd->size_ioc_sts = cpu_to_le32(swap_temp);

	/* Init all of buffer page pointers */
	swap_temp = (u32) (req->req.dma + req->req.actual);
	dtd->buff_ptr0 = cpu_to_le32(swap_temp);
	dtd->buff_ptr1 = cpu_to_le32(swap_temp + 0x1000);
	dtd->buff_ptr2 = cpu_to_le32(swap_temp + 0x2000);
	dtd->buff_ptr3 = cpu_to_le32(swap_temp + 0x3000);
	dtd->buff_ptr4 = cpu_to_le32(swap_temp + 0x4000);

	req->req.actual += *length;

	/* zlp is needed if req->req.zero is set */
	if (req->req.zero) {
		if (*length == 0 || (*length % req->ep->ep.maxpacket) != 0)
			*is_last = 1;
		else
			*is_last = 0;
	} else if (req->req.length == req->req.actual)
		*is_last = 1;
	else
		*is_last = 0;

	if ((*is_last) == 0)
		VDBG("multi-dtd request!");
	/* Fill in the transfer size; set active bit */
	swap_temp = ((*length << DTD_LENGTH_BIT_POS) | DTD_STATUS_ACTIVE);

	/* Enable interrupt for the last dtd of a request */
	if (*is_last && !req->req.no_interrupt)
		swap_temp |= DTD_IOC;

	dtd->size_ioc_sts = cpu_to_le32(swap_temp);

	mb();

	VDBG("length = %d address= 0x%x", *length, (int)*dma);

	return dtd;
}

/* Generate dtd chain for a request */
static int fsl_req_to_dtd(struct fsl_req *req, gfp_t gfp_flags)
{
	unsigned	count;
	int		is_last;
	int		is_first =1;
	struct ep_td_struct	*last_dtd = NULL, *dtd;
	dma_addr_t dma;

	do {
		dtd = fsl_build_dtd(req, &count, &dma, &is_last, gfp_flags);
		if (dtd == NULL)
			return -ENOMEM;

		if (is_first) {
			is_first = 0;
			req->head = dtd;
		} else {
			last_dtd->next_td_ptr = cpu_to_le32(dma);
			last_dtd->next_td_virt = dtd;
		}
		last_dtd = dtd;

		req->dtd_count++;
	} while (!is_last);

	dtd->next_td_ptr = cpu_to_le32(DTD_NEXT_TERMINATE);

	req->tail = dtd;

	return 0;
}

/* queues (submits) an I/O request to an endpoint */
static int
fsl_ep_queue(struct usb_ep *_ep, struct usb_request *_req, gfp_t gfp_flags)
{
	struct fsl_ep *ep = container_of(_ep, struct fsl_ep, ep);
	struct fsl_req *req = container_of(_req, struct fsl_req, req);
	struct fsl_udc *udc = ep->udc;
	unsigned long flags;
	enum dma_data_direction dir;
	int is_iso = 0;
	int status;

	/* catch various bogus parameters */
	if (!_req || !req->req.complete || !req->req.buf
			|| !list_empty(&req->queue)) {
		VDBG("%s, bad params", __func__);
		return -EINVAL;
	}

	spin_lock_irqsave(&udc->lock, flags);

	if (unlikely(!ep->desc)) {
		VDBG("%s, bad ep", __func__);
		spin_unlock_irqrestore(&udc->lock, flags);
		return -EINVAL;
	}

	if (ep->desc->bmAttributes == USB_ENDPOINT_XFER_ISOC) {
		if (req->req.length > ep->ep.maxpacket) {
			spin_unlock_irqrestore(&udc->lock, flags);
			return -EMSGSIZE;
		}
		is_iso = 1;
	}

	dir = ep_is_in(ep) ? DMA_TO_DEVICE : DMA_FROM_DEVICE;

	spin_unlock_irqrestore(&udc->lock, flags);

	if (!udc->driver || udc->gadget.speed == USB_SPEED_UNKNOWN)
		return -ESHUTDOWN;

	req->ep = ep;

	/* map virtual address to hardware */
	if (req->req.dma == DMA_ADDR_INVALID) {
		req->req.dma = dma_map_single(udc->gadget.dev.parent,
					req->req.buf, req->req.length, dir);
		req->mapped = 1;
	} else {
		dma_sync_single_for_device(udc->gadget.dev.parent,
					req->req.dma, req->req.length, dir);
		req->mapped = 0;
	}

	req->req.status = -EINPROGRESS;
	req->req.actual = 0;
	req->dtd_count = 0;


	/* build dtds and push them to device queue */
	status = fsl_req_to_dtd(req, gfp_flags);
	if (status)
		goto err_unmap;

	spin_lock_irqsave(&udc->lock, flags);

	/* re-check if the ep has not been disabled */
	if (unlikely(!ep->desc)) {
		spin_unlock_irqrestore(&udc->lock, flags);
		status = -EINVAL;
		goto err_unmap;
	}

	fsl_queue_td(ep, req);

	/* Update ep0 state */
	if ((ep_index(ep) == 0))
		udc->ep0_state = DATA_STATE_XMIT;

	/* irq handler advances the queue */
	if (req != NULL)
		list_add_tail(&req->queue, &ep->queue);
	spin_unlock_irqrestore(&udc->lock, flags);

	return 0;

err_unmap:
	if (req->mapped) {
		dma_unmap_single(udc->gadget.dev.parent,
			req->req.dma, req->req.length, dir);
		req->req.dma = DMA_ADDR_INVALID;
		req->mapped = 0;
	}
	return status;
}

/* dequeues (cancels, unlinks) an I/O request from an endpoint */
static int fsl_ep_dequeue(struct usb_ep *_ep, struct usb_request *_req)
{
	struct fsl_ep *ep = container_of(_ep, struct fsl_ep, ep);
	struct fsl_req *req;
	unsigned long flags;
	int ep_num, stopped, ret = 0;
	u32 epctrl;

	if (!_ep || !_req || !ep || !req || !ep->udc ||!ep->desc)
		return -EINVAL;

	spin_lock_irqsave(&ep->udc->lock, flags);
	stopped = ep->stopped;

	/* Stop the ep before we deal with the queue */
	ep->stopped = 1;
	ep_num = ep_index(ep);

#if defined(CONFIG_ARCH_TEGRA)
	/* Touch the registers if cable is connected and phy is on */
	if(vbus_enabled())
#endif
	{
		epctrl = fsl_readl(&dr_regs->endptctrl[ep_num]);
		if (ep_is_in(ep))
			epctrl &= ~EPCTRL_TX_ENABLE;
		else
			epctrl &= ~EPCTRL_RX_ENABLE;
		fsl_writel(epctrl, &dr_regs->endptctrl[ep_num]);
	}

	/* make sure it's actually queued on this endpoint */
	list_for_each_entry(req, &ep->queue, queue) {
		if (&req->req == _req)
			break;
	}
	if (&req->req != _req) {
		ret = -EINVAL;
		goto out;
	}

	/* The request is in progress, or completed but not dequeued */
	if (ep->queue.next == &req->queue) {
		_req->status = -ECONNRESET;
		fsl_ep_fifo_flush(_ep);	/* flush current transfer */

		/* The request isn't the last request in this ep queue */
		if (req->queue.next != &ep->queue) {
			struct ep_queue_head *qh;
			struct fsl_req *next_req;

			qh = ep->qh;
			next_req = list_entry(req->queue.next, struct fsl_req,
					queue);

			/* Point the QH to the first TD of next request */
			fsl_writel((u32) next_req->head, &qh->curr_dtd_ptr);
		}

		/* The request hasn't been processed, patch up the TD chain */
	} else {
		struct fsl_req *prev_req;

		prev_req = list_entry(req->queue.prev, struct fsl_req, queue);
		fsl_writel(fsl_readl(&req->tail->next_td_ptr),
				&prev_req->tail->next_td_ptr);

	}

	done(ep, req, -ECONNRESET);

	/* Enable EP */
out:
#if defined(CONFIG_ARCH_TEGRA)
	/* Touch the registers if cable is connected and phy is on */
	if(vbus_enabled())
#endif
	{
		epctrl = fsl_readl(&dr_regs->endptctrl[ep_num]);
		if (ep_is_in(ep))
			epctrl |= EPCTRL_TX_ENABLE;
		else
			epctrl |= EPCTRL_RX_ENABLE;
		fsl_writel(epctrl, &dr_regs->endptctrl[ep_num]);
	}
	ep->stopped = stopped;

	spin_unlock_irqrestore(&ep->udc->lock, flags);
	return ret;
}

/*-------------------------------------------------------------------------*/

/*-----------------------------------------------------------------
 * modify the endpoint halt feature
 * @ep: the non-isochronous endpoint being stalled
 * @value: 1--set halt  0--clear halt
 * Returns zero, or a negative error code.
*----------------------------------------------------------------*/
static int fsl_ep_set_halt(struct usb_ep *_ep, int value)
{
	struct fsl_ep *ep = NULL;
	unsigned long flags = 0;
	int status = -EOPNOTSUPP;	/* operation not supported */
	unsigned char ep_dir = 0, ep_num = 0;
	struct fsl_udc *udc = NULL;

	ep = container_of(_ep, struct fsl_ep, ep);
	udc = ep->udc;
	if (!_ep || !ep->desc) {
		status = -EINVAL;
		goto out;
	}

	if (ep->desc->bmAttributes == USB_ENDPOINT_XFER_ISOC) {
		status = -EOPNOTSUPP;
		goto out;
	}

	/* Attempt to halt IN ep will fail if any transfer requests
	 * are still queue */
	if (value && ep_is_in(ep) && !list_empty(&ep->queue)) {
		status = -EAGAIN;
		goto out;
	}

	status = 0;
	ep_dir = ep_is_in(ep) ? USB_SEND : USB_RECV;
	ep_num = (unsigned char)(ep_index(ep));
	spin_lock_irqsave(&ep->udc->lock, flags);
	dr_ep_change_stall(ep_num, ep_dir, value);
	spin_unlock_irqrestore(&ep->udc->lock, flags);

	if (ep_index(ep) == 0) {
		udc->ep0_state = WAIT_FOR_SETUP;
		udc->ep0_dir = 0;
	}
out:
	VDBG(" %s %s halt stat %d", ep->ep.name,
			value ?  "set" : "clear", status);

	return status;
}

static void fsl_ep_fifo_flush(struct usb_ep *_ep)
{
	struct fsl_ep *ep;
	int ep_num, ep_dir;
	u32 bits;
	unsigned long timeout;
#define FSL_UDC_FLUSH_TIMEOUT 1000

#if defined(CONFIG_ARCH_TEGRA)
	/* Touch the registers if cable is connected and phy is on */
	if (!vbus_enabled())
		return;
#endif

	if (!_ep) {
		return;
	} else {
		ep = container_of(_ep, struct fsl_ep, ep);
		if (!ep->desc)
			return;
	}
	ep_num = ep_index(ep);
	ep_dir = ep_is_in(ep) ? USB_SEND : USB_RECV;

	if (ep_num == 0)
		bits = (1 << 16) | 1;
	else if (ep_dir == USB_SEND)
		bits = 1 << (16 + ep_num);
	else
		bits = 1 << ep_num;

	timeout = jiffies + FSL_UDC_FLUSH_TIMEOUT;
	do {
		fsl_writel(bits, &dr_regs->endptflush);

		/* Wait until flush complete */
		while (fsl_readl(&dr_regs->endptflush)) {
			if (time_after(jiffies, timeout)) {
				ERR("ep flush timeout\n");
				return;
			}
			cpu_relax();
		}
		/* See if we need to flush again */
	} while (fsl_readl(&dr_regs->endptstatus) & bits);
}

static struct usb_ep_ops fsl_ep_ops = {
	.enable = fsl_ep_enable,
	.disable = fsl_ep_disable,

	.alloc_request = fsl_alloc_request,
	.free_request = fsl_free_request,

	.queue = fsl_ep_queue,
	.dequeue = fsl_ep_dequeue,

	.set_halt = fsl_ep_set_halt,
	.fifo_flush = fsl_ep_fifo_flush,	/* flush fifo */
};

/*-------------------------------------------------------------------------
		Gadget Driver Layer Operations
-------------------------------------------------------------------------*/

/*----------------------------------------------------------------------
 * Get the current frame number (from DR frame_index Reg )
 *----------------------------------------------------------------------*/
static int fsl_get_frame(struct usb_gadget *gadget)
{
	return (int)(fsl_readl(&dr_regs->frindex) & USB_FRINDEX_MASKS);
}

/*-----------------------------------------------------------------------
 * Tries to wake up the host connected to this gadget
 -----------------------------------------------------------------------*/
#ifndef CONFIG_USB_ANDROID
static int fsl_wakeup(struct usb_gadget *gadget)
{
	struct fsl_udc *udc = container_of(gadget, struct fsl_udc, gadget);
	u32 portsc;

	/* Remote wakeup feature not enabled by host */
	if (!udc->remote_wakeup)
		return -ENOTSUPP;

	portsc = fsl_readl(&dr_regs->portsc1);
	/* not suspended? */
	if (!(portsc & PORTSCX_PORT_SUSPEND))
		return 0;
	/* trigger force resume */
	portsc |= PORTSCX_PORT_FORCE_RESUME;
	fsl_writel(portsc, &dr_regs->portsc1);
	return 0;
}
#endif

static int can_pullup(struct fsl_udc *udc)
{
	return udc->driver && udc->softconnect && udc->vbus_active;
}

static int fsl_set_selfpowered(struct usb_gadget * gadget, int is_on)
{
	struct fsl_udc *udc;
	udc = container_of(gadget, struct fsl_udc, gadget);
	udc->selfpowered = (is_on != 0);
	return 0;
}

/* Notify controller that VBUS is powered, Called by whatever
   detects VBUS sessions */
static int fsl_vbus_session(struct usb_gadget *gadget, int is_active)
{
	USB_DEBUG("fsl_vbus_session %d " , is_active);
#if 1//defined(CONFIG_MACH_BLUE) || defined(CONFIG_MACH_VERTEXF) || defined(CONFIG_MACH_VERTEXF)
	tegra_usb_set_vbus_state(is_active);
	return 0;
#else
	struct fsl_udc	*udc;
	unsigned long	flags;

	udc = container_of(gadget, struct fsl_udc, gadget);

	VDBG("VBUS %s", is_active ? "on" : "off");

	if (udc->transceiver) {
		if (udc->vbus_active && !is_active) {
			/* If cable disconnected, cancel any delayed work */
			cancel_delayed_work(&udc->work);
			spin_lock_irqsave(&udc->lock, flags);
			/* reset all internal Queues and inform client driver */
			reset_queues(udc);
			/* stop the controller and turn off the clocks */
			dr_controller_stop(udc);
			dr_controller_reset(udc);
			udc->vbus_active = 0;
			udc->usb_state = USB_STATE_DEFAULT;
			spin_unlock_irqrestore(&udc->lock, flags);
			fsl_udc_clk_suspend(false);
			if (udc->vbus_regulator) {
				/* set the current limit to 0mA */
				regulator_set_current_limit(
					udc->vbus_regulator, 0, 0);
			}
		} else if (!udc->vbus_active && is_active) {
			fsl_udc_clk_resume(false);
			/* setup the controller in the device mode */
			dr_controller_setup(udc);
			/* setup EP0 for setup packet */
			ep0_setup(udc);
			/* initialize the USB and EP states */
			udc->usb_state = USB_STATE_ATTACHED;
			udc->ep0_state = WAIT_FOR_SETUP;
			udc->ep0_dir = 0;
			udc->vbus_active = 1;
			/* start the controller */
			dr_controller_run(udc);
			if (udc->vbus_regulator) {
				/* set the current limit to 100mA */
				regulator_set_current_limit(
					udc->vbus_regulator, 0, 100);
			}
			/* Schedule work to wait for 1000 msec and check for
			 * charger if setup packet is not received */
			schedule_delayed_work(&udc->work,
				USB_CHARGER_DETECTION_WAIT_TIME_MS);
		}
	}

	spin_lock_irqsave(&udc->lock, flags);
	udc->vbus_active = (is_active != 0);
	if (can_pullup(udc))
		fsl_writel((fsl_readl(&dr_regs->usbcmd) | USB_CMD_RUN_STOP),
				&dr_regs->usbcmd);
	else
		fsl_writel((fsl_readl(&dr_regs->usbcmd) & ~USB_CMD_RUN_STOP),
				&dr_regs->usbcmd);
	spin_unlock_irqrestore(&udc->lock, flags);
	return 0;
#endif
}

/* constrain controller's VBUS power usage
 * This call is used by gadget drivers during SET_CONFIGURATION calls,
 * reporting how much power the device may consume.  For example, this
 * could affect how quickly batteries are recharged.
 *
 * Returns zero on success, else negative errno.
 */
static int fsl_vbus_draw(struct usb_gadget *gadget, unsigned mA)
{
	struct fsl_udc *udc;

	udc = container_of(gadget, struct fsl_udc, gadget);
	/* check udc regulator is available for drawing the vbus current */
	if (udc->vbus_regulator) {
		udc->current_limit = mA;
		schedule_work(&udc->charger_work);
	}

	if (udc->transceiver)
		return otg_set_power(udc->transceiver, mA);
	return -ENOTSUPP;
}
//++ htc ++
//SW workarounds
static int fsl_pullup_internal(struct usb_gadget *gadget, int is_on)
{
	struct fsl_udc *udc;

	udc = container_of(gadget, struct fsl_udc, gadget);
	udc->softconnect = (is_on != 0);
	if (can_pullup(udc))
		fsl_writel((fsl_readl(&dr_regs->usbcmd) | USB_CMD_RUN_STOP),
				&dr_regs->usbcmd);
	else {
		fsl_writel((fsl_readl(&dr_regs->usbcmd) & ~USB_CMD_RUN_STOP),
				&dr_regs->usbcmd);
		/* S/W workaround, Issue#1 */
		//TODO:otg_io_write(udc->transceiver, 0x48, 0x04);
	}
	return 0;
}
//-- htc --
/* Change Data+ pullup status
 * this func is used by usb_gadget_connect/disconnet
 */
static int fsl_pullup(struct usb_gadget *gadget, int is_on)
{
	struct fsl_udc *udc;

	udc = container_of(gadget, struct fsl_udc, gadget);
	udc->softconnect = (is_on != 0);
	if (udc_controller->transceiver) {
		if (udc_controller->transceiver->state == OTG_STATE_B_PERIPHERAL) {
			if (can_pullup(udc))
				fsl_writel((fsl_readl(&dr_regs->usbcmd) | USB_CMD_RUN_STOP),
						&dr_regs->usbcmd);
			else
				fsl_writel((fsl_readl(&dr_regs->usbcmd) & ~USB_CMD_RUN_STOP),
						&dr_regs->usbcmd);
		}
	}
	return 0;
}

/* defined in gadget.h */
static struct usb_gadget_ops fsl_gadget_ops = {
	.get_frame = fsl_get_frame,
#ifndef CONFIG_USB_ANDROID
	.wakeup = fsl_wakeup,
#endif
	.set_selfpowered = fsl_set_selfpowered,
	.vbus_session = fsl_vbus_session,
	.vbus_draw = fsl_vbus_draw,
	.pullup = fsl_pullup,
};

/* Set protocol stall on ep0, protocol stall will automatically be cleared
   on new transaction */
static void ep0stall(struct fsl_udc *udc)
{
	u32 tmp;

	/* must set tx and rx to stall at the same time */
	tmp = fsl_readl(&dr_regs->endptctrl[0]);
	tmp |= EPCTRL_TX_EP_STALL | EPCTRL_RX_EP_STALL;
	fsl_writel(tmp, &dr_regs->endptctrl[0]);
	udc->ep0_state = WAIT_FOR_SETUP;
	udc->ep0_dir = 0;
}

/* Prime a status phase for ep0 */
static int ep0_prime_status(struct fsl_udc *udc, int direction)
{
	struct fsl_req *req = udc->status_req;
	struct fsl_ep *ep;

	if (direction == EP_DIR_IN)
		udc->ep0_dir = USB_DIR_IN;
	else
		udc->ep0_dir = USB_DIR_OUT;

	ep = &udc->eps[0];
	udc->ep0_state = WAIT_FOR_OUT_STATUS;

	req->ep = ep;
	req->req.length = 0;
	req->req.status = -EINPROGRESS;
	req->req.actual = 0;
	req->req.complete = NULL;
	req->dtd_count = 0;

	if (fsl_req_to_dtd(req, GFP_ATOMIC) == 0)
		fsl_queue_td(ep, req);
	else
		return -ENOMEM;

	list_add_tail(&req->queue, &ep->queue);

	return 0;
}

static void udc_reset_ep_queue(struct fsl_udc *udc, u8 pipe)
{
	struct fsl_ep *ep = get_ep_by_pipe(udc, pipe);

	if (ep->name)
		nuke(ep, -ESHUTDOWN);
}

/*
 * ch9 Set address
 */
static void ch9setaddress(struct fsl_udc *udc, u16 value, u16 index, u16 length)
{
	/* Save the new address to device struct */
	udc->device_address = (u8) value;
	/* Update usb state */
	udc->usb_state = USB_STATE_ADDRESS;
	/* Status phase */
	if (ep0_prime_status(udc, EP_DIR_IN))
		ep0stall(udc);
}

/*
 * ch9 Get status
 */
static void ch9getstatus(struct fsl_udc *udc, u8 request_type, u16 value,
		u16 index, u16 length)
{
	u16 tmp = 0;		/* Status, cpu endian */
	struct fsl_req *req;
	struct fsl_ep *ep;

	ep = &udc->eps[0];

	if ((request_type & USB_RECIP_MASK) == USB_RECIP_DEVICE) {
		/* Get device status */
		if (udc->selfpowered)
			tmp = 1 << USB_DEVICE_SELF_POWERED;
		tmp |= udc->remote_wakeup << USB_DEVICE_REMOTE_WAKEUP;
	} else if ((request_type & USB_RECIP_MASK) == USB_RECIP_INTERFACE) {
		/* Get interface status */
		/* We don't have interface information in udc driver */
		tmp = 0;
	} else if ((request_type & USB_RECIP_MASK) == USB_RECIP_ENDPOINT) {
		/* Get endpoint status */
		struct fsl_ep *target_ep;

		target_ep = get_ep_by_pipe(udc, get_pipe_by_windex(index));

		/* stall if endpoint doesn't exist */
		if (!target_ep->desc)
			goto stall;
		tmp = dr_ep_get_stall(ep_index(target_ep), ep_is_in(target_ep))
				<< USB_ENDPOINT_HALT;
	}

	udc->ep0_dir = USB_DIR_IN;
	/* Borrow the per device status_req */
	req = udc->status_req;
	/* Fill in the reqest structure */
	*((u16 *) req->req.buf) = cpu_to_le16(tmp);
	req->ep = ep;
	req->req.length = 2;
	req->req.status = -EINPROGRESS;
	req->req.actual = 0;
	req->req.complete = NULL;
	req->dtd_count = 0;

	/* map virtual address to hardware */
	if (req->req.dma == DMA_ADDR_INVALID) {
		req->req.dma = dma_map_single(ep->udc->gadget.dev.parent,
					req->req.buf,
					req->req.length, ep_is_in(ep)
						? DMA_TO_DEVICE
						: DMA_FROM_DEVICE);
		req->mapped = 1;
	} else {
		dma_sync_single_for_device(ep->udc->gadget.dev.parent,
					req->req.dma, req->req.length,
					ep_is_in(ep)
						? DMA_TO_DEVICE
						: DMA_FROM_DEVICE);
		req->mapped = 0;
	}

	/* prime the data phase */
	if ((fsl_req_to_dtd(req, GFP_ATOMIC) == 0))
		fsl_queue_td(ep, req);
	else			/* no mem */
		goto stall;

	list_add_tail(&req->queue, &ep->queue);
	udc->ep0_state = DATA_STATE_XMIT;
	return;
stall:
	ep0stall(udc);
}

static void udc_test_mode(struct fsl_udc *udc, u32 test_mode)
{
	struct fsl_req *req;
	struct fsl_ep *ep;
	u32 portsc, bitmask;
	unsigned long timeout;
	void __iomem *base = 0x7D000000;
	u32 val;

	/* Ack the ep0 IN */
	if (ep0_prime_status(udc, EP_DIR_IN))
		ep0stall(udc);

	/* get the ep0 */
	ep = &udc->eps[0];
	bitmask = ep_is_in(ep)
		? (1 << (ep_index(ep) + 16))
		: (1 << (ep_index(ep)));

	timeout = jiffies + HZ;
	/* Wait until ep0 IN endpoint txfr is complete */
	while (!(fsl_readl(&dr_regs->endptcomplete) & bitmask)) {
		if (time_after(jiffies, timeout)) {
			USB_ERR("Timeout for Ep0 IN Ack\n");
			break;
		}
		cpu_relax();
	}

	switch (test_mode << PORTSCX_PTC_BIT_POS) {
	case PORTSCX_PTC_JSTATE:
		VDBG("TEST_J\n");
		break;
	case PORTSCX_PTC_KSTATE:
		VDBG("TEST_K\n");
		break;
	case PORTSCX_PTC_SEQNAK:
		val = readl(IO_ADDRESS(base + UTMIP_HSRX_CFG1));
		val &= ~UTMIP_HS_SYNC_START_DLY(~0);
		val |= UTMIP_HS_SYNC_START_DLY(0x2);
		writel(val, IO_ADDRESS(base + UTMIP_HSRX_CFG1));
		VDBG("TEST_SE0_NAK\n");
		break;
	case PORTSCX_PTC_PACKET:
		VDBG("TEST_PACKET\n");

		/* get the ep and configure for IN direction */
		ep = &udc->eps[0];
		udc->ep0_dir = USB_DIR_IN;

		/* Initialize ep0 status request structure */
		req = container_of(fsl_alloc_request(NULL, GFP_ATOMIC),
				struct fsl_req, req);
		/* allocate a small amount of memory to get valid address */
		req->req.buf = kmalloc(sizeof(fsl_udc_test_packet), GFP_ATOMIC);
		req->req.dma = virt_to_phys(req->req.buf);

		/* Fill in the reqest structure */
		memcpy(req->req.buf, fsl_udc_test_packet, sizeof(fsl_udc_test_packet));
		req->ep = ep;
		req->req.length = sizeof(fsl_udc_test_packet);
		req->req.status = -EINPROGRESS;
		req->req.actual = 0;
		req->req.complete = NULL;
		req->dtd_count = 0;
		req->mapped = 0;

		dma_sync_single_for_device(ep->udc->gadget.dev.parent,
					req->req.dma, req->req.length,
					ep_is_in(ep)
						? DMA_TO_DEVICE
						: DMA_FROM_DEVICE);

		/* prime the data phase */
		if ((fsl_req_to_dtd(req, GFP_ATOMIC) == 0))
			fsl_queue_td(ep, req);
		else			/* no mem */
			goto stall;

		list_add_tail(&req->queue, &ep->queue);
		udc->ep0_state = DATA_STATE_XMIT;
		break;
	case PORTSCX_PTC_FORCE_EN:
		VDBG("TEST_FORCE_EN\n");
		break;
	default:
		ERR("udc unknown test mode[%d]!\n", test_mode);
		goto stall;
	}

	/* read the portsc register */
	portsc = fsl_readl(&dr_regs->portsc1);
	/* set the test mode selector */
	portsc |= test_mode << PORTSCX_PTC_BIT_POS;
	fsl_writel(portsc, &dr_regs->portsc1);

	/*
	 * The device must have its power cycled to exit test mode.
	 * See USB 2.0 spec, section 9.4.9 for test modes operation in "Set Feature"
	 * See USB 2.0 spec, section 7.1.20 for test modes.
	 */
	USB_INFO("udc entering the test mode, power cycle to exit test mode\n");
	return;
stall:
	ep0stall(udc);
}

static void setup_received_irq(struct fsl_udc *udc,
		struct usb_ctrlrequest *setup)
{
	u16 wValue = le16_to_cpu(setup->wValue);
	u16 wIndex = le16_to_cpu(setup->wIndex);
	u16 wLength = le16_to_cpu(setup->wLength);

	udc_reset_ep_queue(udc, 0);

	/* We process some stardard setup requests here */
	switch (setup->bRequest) {
	case USB_REQ_GET_STATUS:
		/* Data+Status phase from udc */
		if ((setup->bRequestType & (USB_DIR_IN | USB_TYPE_MASK))
					!= (USB_DIR_IN | USB_TYPE_STANDARD))
			break;
		ch9getstatus(udc, setup->bRequestType, wValue, wIndex, wLength);
		return;

	case USB_REQ_SET_ADDRESS:
		/* Status phase from udc */
		if (setup->bRequestType != (USB_DIR_OUT | USB_TYPE_STANDARD
						| USB_RECIP_DEVICE))
			break;
#ifdef CONFIG_ARCH_TEGRA
		/* This delay is necessary for some windows drivers to
		 * properly recognize the device */
		mdelay(1);
#endif
		ch9setaddress(udc, wValue, wIndex, wLength);
		return;

	case USB_REQ_CLEAR_FEATURE:
	case USB_REQ_SET_FEATURE:
		/* Status phase from udc */
	{
		int rc = -EOPNOTSUPP;

		if (setup->bRequestType == USB_RECIP_DEVICE &&
				 wValue == USB_DEVICE_TEST_MODE) {
			/*
			 * If the feature selector is TEST_MODE, then the most
			 * significant byte of wIndex is used to specify the specific
			 * test mode and the lower byte of wIndex must be zero.
			 */
			udc_test_mode(udc, wIndex >> 8);
			return;

		} else if ((setup->bRequestType & (USB_RECIP_MASK | USB_TYPE_MASK))
				== (USB_RECIP_ENDPOINT | USB_TYPE_STANDARD)) {
			int pipe = get_pipe_by_windex(wIndex);
			struct fsl_ep *ep;

			if (wValue != 0 || wLength != 0 || pipe > udc->max_ep)
				break;
			ep = get_ep_by_pipe(udc, pipe);

			spin_unlock(&udc->lock);
			rc = fsl_ep_set_halt(&ep->ep,
					(setup->bRequest == USB_REQ_SET_FEATURE)
						? 1 : 0);
			spin_lock(&udc->lock);

		} else if ((setup->bRequestType & (USB_RECIP_MASK
				| USB_TYPE_MASK)) == (USB_RECIP_DEVICE
				| USB_TYPE_STANDARD)) {
			/* Note: The driver has not include OTG support yet.
			 * This will be set when OTG support is added */
			if (!gadget_is_otg(&udc->gadget))
				break;
			else if (setup->bRequest == USB_DEVICE_B_HNP_ENABLE)
				udc->gadget.b_hnp_enable = 1;
			else if (setup->bRequest == USB_DEVICE_A_HNP_SUPPORT)
				udc->gadget.a_hnp_support = 1;
			else if (setup->bRequest ==
					USB_DEVICE_A_ALT_HNP_SUPPORT)
				udc->gadget.a_alt_hnp_support = 1;
			else
				break;
			rc = 0;
		} else
			break;

		if (rc == 0) {
			if (ep0_prime_status(udc, EP_DIR_IN))
				ep0stall(udc);
		}
		return;
	}

	default:
		break;
	}

	/* Requests handled by gadget */
	if (wLength) {
		/* Data phase from gadget, status phase from udc */
		udc->ep0_dir = (setup->bRequestType & USB_DIR_IN)
				?  USB_DIR_IN : USB_DIR_OUT;
		spin_unlock(&udc->lock);
		if (udc->driver && udc->driver->setup(&udc->gadget,
				&udc->local_setup_buff) < 0)
			ep0stall(udc);
		spin_lock(&udc->lock);
		udc->ep0_state = (setup->bRequestType & USB_DIR_IN)
				?  DATA_STATE_XMIT : DATA_STATE_RECV;
	} else {
		/* No data phase, IN status from gadget */
		udc->ep0_dir = USB_DIR_IN;
		spin_unlock(&udc->lock);
		if (udc->driver && udc->driver->setup(&udc->gadget,
				&udc->local_setup_buff) < 0)
			ep0stall(udc);
		spin_lock(&udc->lock);
		udc->ep0_state = WAIT_FOR_OUT_STATUS;
	}
}

/* Process request for Data or Status phase of ep0
 * prime status phase if needed */
static void ep0_req_complete(struct fsl_udc *udc, struct fsl_ep *ep0,
		struct fsl_req *req)
{
	if (udc->usb_state == USB_STATE_ADDRESS) {
		/* Set the new address */
		u32 new_address = (u32) udc->device_address;
		fsl_writel(new_address << USB_DEVICE_ADDRESS_BIT_POS,
				&dr_regs->deviceaddr);
	}

	done(ep0, req, 0);

	switch (udc->ep0_state) {
	case DATA_STATE_XMIT:
		/* receive status phase */
		if (ep0_prime_status(udc, EP_DIR_OUT))
			ep0stall(udc);
		break;
	case DATA_STATE_RECV:
		/* send status phase */
		if (ep0_prime_status(udc, EP_DIR_IN))
			ep0stall(udc);
		break;
	case WAIT_FOR_OUT_STATUS:
		udc->ep0_state = WAIT_FOR_SETUP;
		break;
	case WAIT_FOR_SETUP:
		ERR("Unexpect ep0 packets\n");
		break;
	default:
		ep0stall(udc);
		break;
	}
}

/* Tripwire mechanism to ensure a setup packet payload is extracted without
 * being corrupted by another incoming setup packet */
static void tripwire_handler(struct fsl_udc *udc, u8 ep_num, u8 *buffer_ptr)
{
	u32 temp;
	struct ep_queue_head *qh;

	qh = &udc->ep_qh[ep_num * 2 + EP_DIR_OUT];

	/* Clear bit in ENDPTSETUPSTAT */
	temp = fsl_readl(&dr_regs->endptsetupstat);
	fsl_writel(temp | (1 << ep_num), &dr_regs->endptsetupstat);

	/* while a hazard exists when setup package arrives */
	do {
		/* Set Setup Tripwire */
		temp = fsl_readl(&dr_regs->usbcmd);
		fsl_writel(temp | USB_CMD_SUTW, &dr_regs->usbcmd);

		/* Copy the setup packet to local buffer */
		memcpy(buffer_ptr, (u8 *) qh->setup_buffer, 8);
	} while (!(fsl_readl(&dr_regs->usbcmd) & USB_CMD_SUTW));

	/* Clear Setup Tripwire */
	temp = fsl_readl(&dr_regs->usbcmd);
	fsl_writel(temp & ~USB_CMD_SUTW, &dr_regs->usbcmd);
}

/* process-ep_req(): free the completed Tds for this req */
static int process_ep_req(struct fsl_udc *udc, int pipe,
		struct fsl_req *curr_req)
{
	struct ep_td_struct *curr_td;
	int	td_complete, actual, remaining_length, j, tmp;
	int	status = 0;
	int	errors = 0;
	struct  ep_queue_head *curr_qh = &udc->ep_qh[pipe];
	int direction = pipe % 2;

	curr_td = curr_req->head;
	td_complete = 0;
	actual = curr_req->req.length;

	for (j = 0; j < curr_req->dtd_count; j++) {
		remaining_length = (le32_to_cpu(curr_td->size_ioc_sts)
					& DTD_PACKET_SIZE)
				>> DTD_LENGTH_BIT_POS;
		actual -= remaining_length;

		if ((errors = le32_to_cpu(curr_td->size_ioc_sts) &
						DTD_ERROR_MASK)) {
			if (errors & DTD_STATUS_HALTED) {
				ERR("dTD error %08x QH=%d\n", errors, pipe);
				/* Clear the errors and Halt condition */
				tmp = le32_to_cpu(curr_qh->size_ioc_int_sts);
				tmp &= ~errors;
				curr_qh->size_ioc_int_sts = cpu_to_le32(tmp);
				status = -EPIPE;
				/* FIXME: continue with next queued TD? */

				break;
			}
			if (errors & DTD_STATUS_DATA_BUFF_ERR) {
				VDBG("Transfer overflow");
				status = -EPROTO;
				break;
			} else if (errors & DTD_STATUS_TRANSACTION_ERR) {
				VDBG("ISO error");
				status = -EILSEQ;
				break;
			} else
				ERR("Unknown error has occured (0x%x)!\n",
					errors);

		} else if (le32_to_cpu(curr_td->size_ioc_sts)
				& DTD_STATUS_ACTIVE) {
			VDBG("Request not complete");
			status = REQ_UNCOMPLETE;
			return status;
		} else if (remaining_length) {
			if (direction) {
				VDBG("Transmit dTD remaining length not zero");
				status = -EPROTO;
				break;
			} else {
				td_complete++;
				break;
			}
		} else {
			td_complete++;
			VDBG("dTD transmitted successful");
		}

		if (j != curr_req->dtd_count - 1)
			curr_td = (struct ep_td_struct *)curr_td->next_td_virt;
	}

	if (status)
		return status;

	curr_req->req.actual = actual;

	return 0;
}

/* Process a DTD completion interrupt */
static void dtd_complete_irq(struct fsl_udc *udc)
{
	u32 bit_pos;
	int i, ep_num, direction, bit_mask, status;
	struct fsl_ep *curr_ep;
	struct fsl_req *curr_req, *temp_req;

	/* Clear the bits in the register */
	bit_pos = fsl_readl(&dr_regs->endptcomplete);
	fsl_writel(bit_pos, &dr_regs->endptcomplete);

	if (!bit_pos)
		return;

#ifdef CONFIG_ARCH_TEGRA
	/* XXX what's going on here */
	for (i = 0; i < udc->max_ep; i++) {
#else
	for (i = 0; i < udc->max_ep * 2; i++) {
#endif
		ep_num = i >> 1;
		direction = i % 2;

		bit_mask = 1 << (ep_num + 16 * direction);

		if (!(bit_pos & bit_mask))
			continue;

		curr_ep = get_ep_by_pipe(udc, i);

		/* If the ep is configured */
		if (curr_ep->name == NULL) {
			WARNING("Invalid EP?");
			continue;
		}

		/* process the req queue until an uncomplete request */
		list_for_each_entry_safe(curr_req, temp_req, &curr_ep->queue,
				queue) {
			status = process_ep_req(udc, i, curr_req);

			VDBG("status of process_ep_req= %d, ep = %d",
					status, ep_num);
			if (status == REQ_UNCOMPLETE)
				break;
			/* write back status to req */
			curr_req->req.status = status;

			if (ep_num == 0) {
				ep0_req_complete(udc, curr_ep, curr_req);
				break;
			} else
				done(curr_ep, curr_req, status);
		}
	}
}

/* Process a port change interrupt */
static void port_change_irq(struct fsl_udc *udc)
{
	u32 speed;

	/* Bus resetting is finished */
	if (!(fsl_readl(&dr_regs->portsc1) & PORTSCX_PORT_RESET)) {
		/* Get the speed */
		speed = (fsl_readl(control_reg)
				& PORTSCX_PORT_SPEED_MASK);
		switch (speed) {
		case PORTSCX_PORT_SPEED_HIGH:
			udc->gadget.speed = USB_SPEED_HIGH;
			break;
		case PORTSCX_PORT_SPEED_FULL:
			udc->gadget.speed = USB_SPEED_FULL;
			break;
		case PORTSCX_PORT_SPEED_LOW:
			udc->gadget.speed = USB_SPEED_LOW;
			break;
		default:
			udc->gadget.speed = USB_SPEED_UNKNOWN;
			break;
		}
	}

	/* Update USB state */
	if (!udc->resume_state)
		udc->usb_state = USB_STATE_DEFAULT;
}

/* Process suspend interrupt */
static void suspend_irq(struct fsl_udc *udc)
{
	udc->resume_state = udc->usb_state;
	udc->usb_state = USB_STATE_SUSPENDED;

	/* report suspend to the driver, serial.c does not support this */
	if (udc->driver && udc->driver->suspend)
		udc->driver->suspend(&udc->gadget);
}

static void bus_resume(struct fsl_udc *udc)
{
	udc->usb_state = udc->resume_state;
	udc->resume_state = 0;
USB_DEBUG("## %s",__func__);
	/* report resume to the driver, serial.c does not support this */
	if (udc->driver && udc->driver->resume)
		udc->driver->resume(&udc->gadget);
}

/* Clear up all ep queues */
static int reset_queues(struct fsl_udc *udc)
{
	u8 pipe;
USB_DEBUG("## %s",__func__);
	for (pipe = 0; pipe < udc->max_pipes; pipe++)
		udc_reset_ep_queue(udc, pipe);

	/* report disconnect; the driver is already quiesced */
	spin_unlock(&udc->lock);
	if (udc->driver && udc->driver->disconnect)
		udc->driver->disconnect(&udc->gadget);
	spin_lock(&udc->lock);

	return 0;
}

//++ htc ++
/* Clear up all ep queues */
static int reset_queues_mute(struct fsl_udc *udc)
{
	u8 pipe;
USB_DEBUG("## %s",__func__);
	for (pipe = 0; pipe < udc->max_pipes; pipe++)
		udc_reset_ep_queue(udc, pipe);

	/* report disconnect; the driver is already quiesced */
	spin_unlock(&udc->lock);
	if (udc->driver && udc->driver->mute_disconnect)
		udc->driver->mute_disconnect(&udc->gadget);
	spin_lock(&udc->lock);

	return 0;
}
//-- htc --
/* Process reset interrupt */
static void reset_irq(struct fsl_udc *udc)
{
	u32 temp;
	unsigned long timeout;

	/* Clear the device address */
	temp = fsl_readl(&dr_regs->deviceaddr);
	fsl_writel(temp & ~USB_DEVICE_ADDRESS_MASK, &dr_regs->deviceaddr);

	udc->device_address = 0;

	/* Clear usb state */
	udc->resume_state = 0;
	udc->ep0_dir = 0;
	udc->ep0_state = WAIT_FOR_SETUP;
	udc->remote_wakeup = 0;	/* default to 0 on reset */
	udc->gadget.b_hnp_enable = 0;
	udc->gadget.a_hnp_support = 0;
	udc->gadget.a_alt_hnp_support = 0;

	/* Clear all the setup token semaphores */
	temp = fsl_readl(&dr_regs->endptsetupstat);
	fsl_writel(temp, &dr_regs->endptsetupstat);

	/* Clear all the endpoint complete status bits */
	temp = fsl_readl(&dr_regs->endptcomplete);
	fsl_writel(temp, &dr_regs->endptcomplete);

	timeout = jiffies + 100;
	while (fsl_readl(&dr_regs->endpointprime)) {
		/* Wait until all endptprime bits cleared */
		if (time_after(jiffies, timeout)) {
			ERR("Timeout for reset\n");
			break;
		}
		cpu_relax();
	}

	/* Write 1s to the flush register */
	fsl_writel(0xffffffff, &dr_regs->endptflush);

#if defined(CONFIG_ARCH_TEGRA)
	/* When the bus reset is seen on Tegra, the PORTSCX_PORT_RESET bit
	 * is not set */
	VDBG("Bus reset");
	/* Reset all the queues, include XD, dTD, EP queue
	 * head and TR Queue */
	//reset_queues(udc);
	reset_queues_mute(udc);//htc
	udc->usb_state = USB_STATE_DEFAULT;
#else
	if (fsl_readl(&dr_regs->portsc1) & PORTSCX_PORT_RESET) {
		VDBG("Bus reset");
		/* Reset all the queues, include XD, dTD, EP queue
		 * head and TR Queue */
		reset_queues(udc);
		udc->usb_state = USB_STATE_DEFAULT;
	} else {
		VDBG("Controller reset");
		/* initialize usb hw reg except for regs for EP, not
		 * touch usbintr reg */
		dr_controller_setup(udc);

		/* Reset all internal used Queues */
		reset_queues(udc);

		ep0_setup(udc);

		/* Enable DR IRQ reg, Set Run bit, change udc state */
		dr_controller_run(udc);
		udc->usb_state = USB_STATE_ATTACHED;
	}
#endif
}

static void fsl_udc_set_current_limit_work(struct work_struct* work)
{
	struct fsl_udc *udc = container_of (work, struct fsl_udc, charger_work);

	/* check udc regulator is available for drawing vbus current*/
	if (udc->vbus_regulator) {
		/* set the current limit in uA */
		regulator_set_current_limit(
			udc->vbus_regulator, 0,
			udc->current_limit *1000);
	}
}

/*
 * If VBUS is detected and setup packet is not received in 100ms then
 * work thread starts and checks for the USB charger detection.
 */
static void fsl_udc_charger_detect_work(struct work_struct* work)
{
	struct fsl_udc *udc = container_of (work, struct fsl_udc, work.work);

	/* check for the platform charger detection */
	if (fsl_udc_charger_detect()) {
		printk(KERN_INFO "[USBUDC] USB compliant charger detected\n");
		udc->connect_type = CONNECT_TYPE_AC;
		queue_work(udc->usb_wq, &udc->notifier_work);
		/* check udc regulator is available for drawing vbus current*/
		if (udc->vbus_regulator) {
			/* set the current limit in uA */
			regulator_set_current_limit(
				udc->vbus_regulator, 0,
				USB_CHARGING_CURRENT_LIMIT_MA*1000);
		}
	}
}

#if defined(CONFIG_ARCH_TEGRA)
/*
 * Restart device controller in the OTG mode on VBUS detection
 */
static void fsl_udc_restart(struct fsl_udc *udc)
{
	unsigned long flags = 0;
	USB_INFO("fsl_udc_restart");
	/* setup the controller in the device mode */
	dr_controller_setup(udc);
	/* setup EP0 for setup packet */
	ep0_setup(udc);
	/* start the controller */
	dr_controller_run(udc);
	/* initialize the USB and EP states */
	udc->usb_state = USB_STATE_ATTACHED;
	udc->ep0_state = WAIT_FOR_SETUP;
	udc->ep0_dir = 0;
	udc->vbus_active = 1;
	/*spin_lock_irqsave(&udc->lock, flags);*/ /* htc */
	udc->flags |= USB_FLAG_START;
	//udc->state = USB_STATE_IDLE;
	if (udc->vbus_active) {
			udc->flags |= USB_FLAG_VBUS_ONLINE;

	} else {
		udc->flags |= USB_FLAG_VBUS_OFFLINE;
	}

	queue_work(udc->usb_wq, &udc->detect_work);
	/*spin_unlock_irqrestore(&udc->lock, flags);*/ /* htc */
}
#endif

/*
 * USB device controller interrupt handler
 */
static irqreturn_t fsl_udc_irq(int irq, void *_udc)
{
	struct fsl_udc *udc = _udc;
	u32 irq_src;
	irqreturn_t status = IRQ_NONE;
	unsigned long flags;
	if (irq_udc_debug==1) {
		USB_INFO("udc_irq");
		irq_udc_debug = 0;
	}
	spin_lock_irqsave(&udc->lock, flags);

	/* Disable ISR for OTG host mode */
	if (udc->stopped) {
		spin_unlock_irqrestore(&udc->lock, flags);
		return IRQ_NONE;
	}

#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	/* Fence read for coherency of AHB master intiated writes */
	readl(IO_ADDRESS(IO_PPCS_PHYS + USB1_PREFETCH_ID));
#endif

#ifndef CONFIG_TEGRA_SILICON_PLATFORM
	{
		u32 temp = fsl_readl(&usb_sys_regs->vbus_sensors);
		udc->vbus_active = (temp & USB_SYS_VBUS_ASESSION) ? true : false;
		/* write back the register to clear the interrupt */
		fsl_writel(temp, &usb_sys_regs->vbus_sensors);
	}
#endif
	irq_src = fsl_readl(&dr_regs->usbsts) & fsl_readl(&dr_regs->usbintr);
	/* Clear notification bits */
	fsl_writel(irq_src, &dr_regs->usbsts);

	/* VDBG("irq_src [0x%8x]", irq_src); */

	/* Need to resume? */
	if (udc->usb_state == USB_STATE_SUSPENDED)
		if ((fsl_readl(&dr_regs->portsc1) & PORTSCX_PORT_SUSPEND) == 0)
			bus_resume(udc);

	/* USB Interrupt */
	if (irq_src & USB_STS_INT) {
		VDBG("Packet int");
		/* Setup package, we only support ep0 as control ep */
		if (fsl_readl(&dr_regs->endptsetupstat) & EP_SETUP_STATUS_EP0) {
			/* Setup packet received, we are connected to host and
			 * not charger. Cancel any delayed work */
			__cancel_delayed_work(&udc->work);
			tripwire_handler(udc, 0,
					(u8 *) (&udc->local_setup_buff));
			setup_received_irq(udc, &udc->local_setup_buff);
			status = IRQ_HANDLED;
		}

		/* completion of dtd */
		if (fsl_readl(&dr_regs->endptcomplete)) {
			dtd_complete_irq(udc);
			status = IRQ_HANDLED;
		}
	}

	/* SOF (for ISO transfer) */
	if (irq_src & USB_STS_SOF) {
		status = IRQ_HANDLED;
	}

	/* Port Change */
	if (irq_src & USB_STS_PORT_CHANGE) {
		port_change_irq(udc);
		status = IRQ_HANDLED;
	}

	/* Reset Received */
	if (irq_src & USB_STS_RESET) {
		USB_INFO("RESET packet\n"); /* PC confirmed */
		del_timer(&udc->ac_detect_timer);

		reset_irq(udc);
		status = IRQ_HANDLED;
	}

	/* Sleep Enable (Suspend) */
	if (irq_src & USB_STS_SUSPEND) {
		USB_INFO("SUSPEND packet\n"); /* PC confirmed */
		del_timer(&udc->ac_detect_timer);

		suspend_irq(udc);
		status = IRQ_HANDLED;
	}

	if (irq_src & (USB_STS_ERR | USB_STS_SYS_ERR)) {
		VDBG("Error IRQ %x", irq_src);
	}

	spin_unlock_irqrestore(&udc->lock, flags);
	return status;
}

/*----------------------------------------------------------------*
 * Hook to gadget drivers
 * Called by initialization code of gadget drivers
*----------------------------------------------------------------*/
int usb_gadget_probe_driver(struct usb_gadget_driver *driver,
		int (*bind)(struct usb_gadget *))
{
	int retval = -ENODEV;
	unsigned long flags = 0;

	if (!udc_controller)
		return -ENODEV;

	if (!driver || (driver->speed != USB_SPEED_FULL
				&& driver->speed != USB_SPEED_HIGH)
			|| !bind || !driver->disconnect || !driver->setup)
		return -EINVAL;

	if (udc_controller->driver)
		return -EBUSY;

	/* lock is needed but whether should use this lock or another */
	spin_lock_irqsave(&udc_controller->lock, flags);

	driver->driver.bus = NULL;
	/* hook up the driver */
	udc_controller->driver = driver;
	udc_controller->gadget.dev.driver = &driver->driver;
	spin_unlock_irqrestore(&udc_controller->lock, flags);

	/* bind udc driver to gadget driver */
	retval = bind(&udc_controller->gadget);
	if (retval) {
		VDBG("bind to %s --> %d", driver->driver.name, retval);
		udc_controller->gadget.dev.driver = NULL;
		udc_controller->driver = NULL;
		goto out;
	}

	/* Enable DR IRQ reg and Set usbcmd reg  Run bit */
	if (!udc_controller->transceiver) {
		dr_controller_run(udc_controller);
		udc_controller->usb_state = USB_STATE_ATTACHED;
		udc_controller->ep0_state = WAIT_FOR_SETUP;
		udc_controller->ep0_dir = 0;
	}

	USB_INFO("%s: bind to driver %s\n",
			udc_controller->gadget.name, driver->driver.name);

	usb_start(udc_controller);
out:
	if (retval)
		USB_WARNING("gadget driver register failed %d\n",
		       retval);
	return retval;
}
EXPORT_SYMBOL(usb_gadget_probe_driver);

/* Disconnect from gadget driver */
int usb_gadget_unregister_driver(struct usb_gadget_driver *driver)
{
	struct fsl_ep *loop_ep;
	unsigned long flags;

	if (!udc_controller)
		return -ENODEV;

	if (!driver || driver != udc_controller->driver || !driver->unbind)
		return -EINVAL;

	/* stop DR, disable intr */
	dr_controller_stop(udc_controller);

	/* in fact, no needed */
	udc_controller->usb_state = USB_STATE_ATTACHED;
	udc_controller->ep0_state = WAIT_FOR_SETUP;
	udc_controller->ep0_dir = 0;

	/* stand operation */
	spin_lock_irqsave(&udc_controller->lock, flags);
	udc_controller->gadget.speed = USB_SPEED_UNKNOWN;
	nuke(&udc_controller->eps[0], -ESHUTDOWN);
	list_for_each_entry(loop_ep, &udc_controller->gadget.ep_list,
			ep.ep_list)
		nuke(loop_ep, -ESHUTDOWN);
	spin_unlock_irqrestore(&udc_controller->lock, flags);

	/* report disconnect; the controller is already quiesced */
	driver->disconnect(&udc_controller->gadget);

	/* unbind gadget and unhook driver. */
	driver->unbind(&udc_controller->gadget);
	udc_controller->gadget.dev.driver = NULL;
	udc_controller->driver = NULL;

	USB_WARNING("unregistered gadget driver '%s'\n",
	       driver->driver.name);
	return 0;
}
EXPORT_SYMBOL(usb_gadget_unregister_driver);

//======================= start porting ===============================


static DEFINE_MUTEX(notify_sem);
static void send_usb_connect_notify(struct work_struct *w)
{
	static struct t_usb_status_notifier *notifier;
	struct fsl_udc *udc = container_of(w, struct fsl_udc,notifier_work);
	USB_DEBUG("send_usb_connect_notify\n");
	if (!udc)
		return;

	USB_INFO("send connect type %d\n", udc->connect_type);
	update_wake_lock(udc->connect_type);
	mutex_lock(&notify_sem);
	list_for_each_entry(notifier,
		&g_lh_usb_notifier_list,
		notifier_link) {
			if (notifier->func != NULL) {
				/* Notify other drivers about connect type. */
				/* use slow charging for unknown type*/
				if (udc->connect_type == CONNECT_TYPE_UNKNOWN)
					notifier->func(CONNECT_TYPE_USB);
				else
					notifier->func(udc->connect_type);
			}
		}
	mutex_unlock(&notify_sem);
	//USB_INFO("send connect type finish\n");
}
#ifdef CONFIG_HTC_USB_NOTIFIER
static void charger_detect(struct fsl_udc *udc);
int usb_register_notifier(struct t_usb_status_notifier *notifier)
{

	if (!notifier || !notifier->name || !notifier->func)
		return -EINVAL;

	mutex_lock(&notify_sem);
	list_add(&notifier->notifier_link,
		&g_lh_usb_notifier_list);
#if 1
	if(udc_controller){
		if(vbus_enabled())
			charger_detect(udc_controller);
		else
			notifier->func(CONNECT_TYPE_NONE);
	}
#endif
	mutex_unlock(&notify_sem);
	return 0;
}
#endif

static void charger_detect_gpio(struct fsl_udc *udc)
{
	printk("charger_detect_gpio \n");
	int val, val1, val2;
	int charger_type;
	int board_id = 0;
	bool is5Pin;
	int voltage;
	u32 portsc;
	int ret;
	uint8_t command[2]={0};
	mdelay(10);
	board_id = htc_get_pcbid_info();
	portsc = fsl_readl(&dr_regs->portsc1);
	ret = (portsc & PORTSCX_LINE_STATUS_BITS);
	if (ret != PORTSCX_LINE_STATUS_BITS) {
		USB_INFO("USB charger\n");
		udc->connect_type = CONNECT_TYPE_UNKNOWN;
		return;

	} else {
		USB_INFO("AC charger\n");
		udc->connect_type = CONNECT_TYPE_AC;
	}

	/* UART_USB_SW */
	/* 1. Set GPIO PH3 output low, switch to UART bus */

	ret = gpio_direction_output(UART_USB_SW, 0);
	if (ret < 0) {
		USB_WARNING("%s: gpio_direction_output failed %d\n", __func__, ret);
		gpio_free(UART_USB_SW);
		return;
	}
	tegra_gpio_enable(UART_USB_SW);

	/* 2. Set GPIO PO2 (UART1_DEBUG_RX) input D-*/
	ret = gpio_direction_input(UART1_DEBUG_RX);
	if (ret < 0) {
		USB_WARNING("%s: gpio_direction_input failed %d\n", __func__, ret);
		gpio_free(UART1_DEBUG_RX);
		return;
	}
	tegra_gpio_enable(UART1_DEBUG_RX);

	/* 3. Set GPIO PO1 (UART1_DEBUG_TX) output high D+ (0)*/
	

	ret = gpio_direction_output(UART1_DEBUG_TX, 0);

	if (ret < 0) {

		USB_WARNING("%s: gpio_direction_output failed %d\n", __func__, ret);
		gpio_free(UART1_DEBUG_TX);
		return;
	}
	tegra_gpio_enable(UART1_DEBUG_TX);


	mdelay(100);

	/* 4. Set GPIO PO1 (UART1_DEBUG_TX) output high D+ (1) */
	ret = gpio_direction_output(UART1_DEBUG_TX, 1);

	if (ret < 0) {

		USB_WARNING("%s: gpio_direction_output failed %d\n", __func__, ret);
		gpio_free(UART1_DEBUG_TX);
		return;
	}

	tegra_gpio_enable(UART1_DEBUG_TX);


	/* 5. Read GPIO PO1(D-) */

	val1 = gpio_get_value(UART1_DEBUG_RX);
	mdelay(5);
	val2 = gpio_get_value(UART1_DEBUG_RX);



	/* 6. Set GPIO PO2 (UART1_DEBUG_RX) as SFIO2 (UART1 RX) */
	tegra_gpio_disable(UART1_DEBUG_RX);

	/* 7. Set GPIO PO1 (UART1_DEBUG_TX) as SFIO2 (UART1 TX) */
	tegra_gpio_disable(UART1_DEBUG_TX);

	/* 8. Set GPIO PH3 (UART/USB#SW) input */
#ifndef CONFIG_USB_STRESS_TEST
	
	ret = gpio_direction_input(UART_USB_SW);
	if (ret < 0) {
		USB_WARNING("%s: gpio_direction_output failed %d\n", __func__, ret);
		gpio_free(UART_USB_SW);
		return;
	}
	tegra_gpio_enable(UART_USB_SW);
#else
	/* when stress test, we disable the uart in case hang the PC */
	ret = gpio_direction_output(UART_USB_SW, 1);
	if (ret < 0) {
		USB_WARNING("%s: gpio_direction_output failed %d\n", __func__, ret);
		gpio_free(UART_USB_SW);
		return;
	}
	tegra_gpio_enable(UART_USB_SW);
#endif
	/* read the int state */
	val = gpio_get_value(CHARGER_PIN_REC);

	microp_read_adc(command);
	voltage =  ((command[0]<<8) | command[1])&0xffff;
	USB_INFO("voltage = %d \n",voltage);
	if(voltage >  VOL_LEVEL_5PIN_LOWER && voltage < VOL_LEVEL_5PIN_UPPER ){
		USB_INFO("5 Pin Adaptor\n");
		charger_type = CHARGER_TYPE_5;
	}
	else if(voltage > VOL_LEVEL_37PIN_LOWER && voltage < VOL_LEVEL_37PIN_UPPER){
		USB_INFO("32+5 Pin Adaptor\n");
		charger_type = CHARGER_TYPE_32_5;
	}
	else if(voltage > VOL_LEVEL_DONGLE_LOWER && voltage < VOL_LEVEL_DONGLE_UPPER){
		USB_INFO("HDMI Dongle Adaptor\n");
		if(val == 1){
			printk("HDMI_5\n");
			charger_type = CHARGER_TYPE_HDMI_5;
		}
		else{
			printk("HDMI_32_5\n");
			charger_type = CHARGER_TYPE_HDMI_32_5;
		}
	}
	else{
		USB_INFO("undefined adaptor\n");
		charger_type = CHARGER_TYPE_5;
	}

	voltage = ((2850 * voltage) >> 20)  ;//((voltage * 2.6) /1023);
	USB_INFO("command = %x %x\n", command[1], command[0]);
	is5Pin = (voltage > 0) ? true : false;
	
	if (board_id == PROJECT_PHASE_XA || board_id == PROJECT_PHASE_XB) {
		if (val1 && val2) {
			USB_INFO("5V/1A AC charger\n");
		} else if (val1 == 0 && val2 == 1) {
			if (is5Pin)
				udc->connect_type = CONNECT_TYPE_1A2_AC;
			else
				udc->connect_type = CONNECT_TYPE_2A_AC;
			USB_INFO("5V/2A AC charger\n");

		} else if (val1 == 0 && val2 == 0) {
			USB_INFO("USB charger\n");

		} else{
			USB_WARNING("Unknown Type\n");
		}
	}
	else if(board_id >= PROJECT_PHASE_XC){
		if (val1 && val2) { // 1A

			if(charger_type == CHARGER_TYPE_5){
				USB_INFO("1A CHARGER_TYPE_5\n");
				udc->connect_type = CONNECT_TYPE_AC;
			}
			else if(charger_type == CHARGER_TYPE_32_5){
				USB_INFO("1A CHARGER_TYPE_32\n");
				udc->connect_type = CONNECT_TYPE_AC;
			}
			else if(charger_type == CHARGER_TYPE_HDMI_5){
				USB_INFO("1A CHARGER_TYPE_HDMI_5\n");
				udc->connect_type = CONNECT_TYPE_AC;
			}
			else if(charger_type == CHARGER_TYPE_HDMI_32_5){
				USB_INFO("1A CHARGER_TYPE_HDMI_32_5\n");
				udc->connect_type = CONNECT_TYPE_AC;
			}
		} else if (val1 == 0 && val2 == 1) { // 2A

			if(charger_type == CHARGER_TYPE_5){
				USB_INFO("1.25A CHARGER_TYPE_5\n");
				udc->connect_type = CONNECT_TYPE_1A2_AC;
			}
			else if(charger_type == CHARGER_TYPE_32_5){
				USB_INFO("2A CHARGER_TYPE_32\n");
				udc->connect_type = CONNECT_TYPE_2A_AC;
			}
			else if(charger_type == CHARGER_TYPE_HDMI_5){
				USB_INFO("1.1A CHARGER_TYPE_HDMI_5\n");
				udc->connect_type = CONNECT_TYPE_1A1_AC;
			}
			else if(charger_type == CHARGER_TYPE_HDMI_32_5){
				USB_INFO("1.6A CHARGER_TYPE_HDMI_32_5\n");
				udc->connect_type = CONNECT_TYPE_1A6_AC;
			}

		} else if (val1 == 0 && val2 == 0) {
			USB_INFO("USB charger\n");

		} else{
			USB_WARNING("Unknown Type\n");
		}

	}
}
static void update_wake_lock(int status)
{
	if (status == CONNECT_TYPE_USB || status == CONNECT_TYPE_UNKNOWN)
	{
		wake_lock(&udc_wake_lock);
		USB_INFO("wake_lock");
	}
	else
	{
		wake_lock_timeout(&udc_wake_lock, 5*HZ);
		USB_INFO("wake_lock_timeout 5*HZ");
	}
}

#define DELAY_FOR_CHECK_CHG msecs_to_jiffies(300)

#ifdef CONFIG_USB_2A_CHARGER_DETECT
static void charger_detect(struct fsl_udc *udc)
{
	USB_INFO("2A charger detect\n");
	charger_detect_gpio(udc);
	queue_work(udc->usb_wq, &udc->notifier_work);

}
#else
static void charger_detect(struct fsl_udc *udc)
{

	//USB_INFO("No 2A charger detect\n");
	//if (!vbus)
	//	return;
	u32 portsc;
	u32 ret;
	unsigned long flags;

	msleep(10);
	/* detect shorted D+/D-, indicating AC power */
	spin_lock_irqsave(&udc->lock, flags);
	portsc = fsl_readl(&dr_regs->portsc1);
	ret = (portsc & PORTSCX_LINE_STATUS_BITS);
	spin_unlock_irqrestore(&udc->lock, flags);

#if defined(CONFIG_CABLE_DETECT_ACCESSORY)
	if (ret != PORTSCX_LINE_STATUS_BITS && !cable_detection_ac_only())
#else
	if (ret != PORTSCX_LINE_STATUS_BITS)
#endif
	{
		USB_INFO("Charger :USB [portsc:%d]\n",portsc);
		/*spin_lock_irqsave(&udc->lock, flags);*/ /* htc */
		udc->connect_type = CONNECT_TYPE_UNKNOWN;
		/*spin_unlock_irqrestore(&udc->lock, flags);*/ /* htc */
		queue_delayed_work(udc->usb_wq, &udc->chg_work,
			DELAY_FOR_CHECK_CHG);
		if(first_online)
		{
			mod_timer(&udc->ac_detect_timer, jiffies + (6 * HZ));
			spin_lock_irqsave(&udc->lock, flags);
			first_online =0;
			spin_unlock_irqrestore(&udc->lock, flags);
		}
		else
			mod_timer(&udc->ac_detect_timer, jiffies + (3 * HZ));
	}
	else
	{
		USB_INFO("Charger :AC [portsc:%d]\n",portsc);
		/*spin_lock_irqsave(&udc->lock, flags);*/ /* htc */
		udc->connect_type = CONNECT_TYPE_AC;
		/*spin_unlock_irqrestore(&udc->lock, flags);*/ /* htc */
		queue_work(udc->usb_wq, &udc->notifier_work);
	}

}
#endif

static void check_charger(struct work_struct *w)
{
	struct fsl_udc *udc = container_of(w, struct fsl_udc, chg_work.work);
	/* unknown charger */
	if (udc->vbus_active && udc->connect_type == CONNECT_TYPE_UNKNOWN)
		queue_work(udc->usb_wq, &udc->notifier_work);
}

static void usb_do_work(struct work_struct *w)
{

	struct fsl_udc *udc = container_of(w, struct fsl_udc, detect_work);
	unsigned long iflags;
	unsigned flags, _vbus;
	USB_INFO("%s %d %d\n", __func__, udc->state, udc->flags);
	mutex_lock(&notify_sem);
	for (;;) {
		spin_lock_irqsave(&udc->lock, iflags);
		flags = udc->flags;
		udc->flags = 0;
                udc->myflags = 0;
		_vbus = udc->vbus_active;
		spin_unlock_irqrestore(&udc->lock, iflags);

		/* give up if we have nothing to do */
		if (flags == 0)
		{
			break;
		}
		//charger_detect(udc);
#if 1
		switch (udc->state) {
		case USB_STATE_IDLE:
			if (flags & USB_FLAG_START)
			{
				//USB_INFO("hsusb: IDLE -> ONLINE\n");
				USB_INFO("tegra udc: IDLE -> ONLINE vbus:%d\n",vbus_enabled());
				first_online =1;
				//reset_irq(udc);
				if(vbus_enabled())
				{
					if (get_bl_ac_in_flag()) {
						udc->state = USB_STATE_ONLINE;
						udc->connect_type = CONNECT_TYPE_AC;
						queue_work(udc->usb_wq, &udc->notifier_work);
					}
					else {
						charger_detect(udc);
						udc->state = USB_STATE_ONLINE;
					}

				}
				else
				{
					udc->state = USB_STATE_OFFLINE;
					udc->connect_type = CONNECT_TYPE_NONE;
					queue_work(udc->usb_wq, &udc->notifier_work);
				}

			}

			break;

		case USB_STATE_ONLINE:
			/* If at any point when we were online, we received
			 * the signal to go offline, we must honor it
			 */
			if (flags & USB_FLAG_VBUS_OFFLINE)
			{
				USB_INFO("tegra udc: ONLINE -> OFFLINE\n");
				udc->state = USB_STATE_OFFLINE;
				if (udc->connect_type != CONNECT_TYPE_NONE) {
					udc->connect_type = CONNECT_TYPE_NONE;
					queue_work(udc->usb_wq, &udc->notifier_work);
				}
				udc->ac_detect_count = 0;
				del_timer_sync(&udc->ac_detect_timer);
				break;
			}

			break;

		case USB_STATE_OFFLINE:
			/* If we were signaled to go online and vbus is stillren
			 * present when we received the signal, go online.
			 */
			if ((flags & USB_FLAG_VBUS_ONLINE) && _vbus) {
				//USB_INFO("hsusb: OFFLINE -> ONLINE\n");
				USB_INFO("tegra udc: OFFLINE -> ONLINE\n");
				msleep(200);
				//reset_irq(udc);
				charger_detect(udc);
				udc->state = USB_STATE_ONLINE;
			}
			break;
		}
#endif
	}
	mutex_unlock(&notify_sem);
}
static void usb_start(struct fsl_udc *udc)
{
	unsigned long flags;
	USB_DEBUG("usb_start\n");
	/*spin_lock_irqsave(&udc->lock, flags);*/ /* htc */
	udc->flags |= USB_FLAG_START;
	queue_work(udc->usb_wq, &udc->detect_work);
	/*spin_unlock_irqrestore(&udc->lock, flags);*/ /* htc */
}

#if defined(CONFIG_CABLE_DETECT_ACCESSORY)
void cable_status_notifier_func(int cable_type)
{
	if (vbus_enabled() && cable_type != CONNECT_TYPE_NONE) {
		USB_INFO("%s: cable=%d\n", __func__, cable_type);
		if (cable_type == CONNECT_TYPE_AC) {
			udc_controller->connect_type = CONNECT_TYPE_AC;
			queue_work(udc_controller->usb_wq, &udc_controller->notifier_work);
		}
		else {
			udc_controller->connect_type = CONNECT_TYPE_USB;
			queue_work(udc_controller->usb_wq, &udc_controller->notifier_work);
		}
	}
}

static struct t_cable_status_notifier cable_status_notifier = {
	.name = "cable_charge",
	.func = cable_status_notifier_func,
};
#endif

static void usb_prepare(struct fsl_udc *udc)
{
	//int ret;
	spin_lock_init(&udc->lock);
	udc->usb_wq = create_singlethread_workqueue("fsl_tegra_udc");
	if (udc->usb_wq == 0) {
		USB_ERR("usb_prepare fail to create workqueue\n");
		return;
	}
	INIT_WORK(&udc->detect_work, usb_do_work);
	INIT_WORK(&udc->notifier_work, send_usb_connect_notify);
	INIT_DELAYED_WORK(&udc->chg_work, check_charger);
	INIT_DELAYED_WORK(&udc->check_vbus_work, usb_vbus_state_work);
#if defined(CONFIG_CABLE_DETECT_ACCESSORY)
	cable_detect_register_notifier(&cable_status_notifier);
#endif
}

/* SW workaround
 * check vbus state
 * if vbus is on but connect_type is no cable , send notifier to battery
 * else vbus is off but connect_type is charger,call charger_detect
 * */
static void usb_vbus_state_work(struct work_struct *w)
{
	struct fsl_udc *udc = container_of(w, struct fsl_udc, check_vbus_work);
	int _vbus;
	unsigned long flags = 0;
	if(!udc)
		return;
	_vbus = vbus_enabled();
	USB_INFO("state_check() vbus:%d type:%d",_vbus,udc->connect_type);
	if (_vbus && udc->connect_type ==0 ) {
		USB_ERR("error!! connect_type=0 vbus=1");
		/*spin_lock_irqsave(&udc->lock, flags);*/ /* htc */
		udc->vbus_active = 1;
		udc->flags |= USB_FLAG_VBUS_ONLINE;
		/*spin_unlock_irqrestore(&udc->lock, flags);*/ /* htc */
		charger_detect(udc);
	}
	else if(!_vbus && udc->connect_type !=0 ) {
		USB_ERR("error!! connect_type=%d vbus=0",udc->connect_type);
		/*spin_lock_irqsave(&udc->lock, flags);*/ /* htc */
		udc->vbus_active = 0;
		udc->flags |= USB_FLAG_VBUS_OFFLINE;
		udc->connect_type = CONNECT_TYPE_NONE;
		/*spin_unlock_irqrestore(&udc->lock, flags);*/ /* htc */
		queue_work(udc->usb_wq, &udc->notifier_work);
	}

	return;
}
/*-------------------------------------------------------------------------
 * FIXME - the callers of this function should use a gadget API instead.
 * This is called from htc_battery.c and board-halibut.c
 * WARNING - this can get called before this driver is initialized.
-------------------------------------------------------------------------*/
void tegra_usb_set_vbus_state(int online)
{
	unsigned long flags = 0;
	struct fsl_udc *udc = udc_controller;
	VDBG("VBUS %s", online ? "on" : "off");
	USB_INFO("tegra_usb_set_vbus_state %s \n", online ? "on" : "off");
	wake_lock_timeout(&udc_wake_lock2, 1*HZ);
	usb_check_count--;
        int count = 0;

	if (udc && udc->transceiver) {
		if (udc->vbus_active && !online) {
			/* If cable disconnected, cancel any delayed work */
			cancel_delayed_work(&udc->work);
			spin_lock_irqsave(&udc->lock, flags);
			/* reset all internal Queues and inform client driver */
			reset_queues(udc);
			/* stop the controller and turn off the clocks */
			dr_controller_stop(udc);
			dr_controller_reset(udc);
			udc->vbus_active = 0;
			udc->usb_state = USB_STATE_DEFAULT;
			spin_unlock_irqrestore(&udc->lock, flags);
			fsl_udc_clk_suspend(false);
			if (udc->vbus_regulator) {
				/* set the current limit to 0mA */
				regulator_set_current_limit(
					udc->vbus_regulator, 0, 0);
			}
		}
		else if (!udc->vbus_active && online) {
			fsl_udc_clk_resume(false);
			/* setup the controller in the device mode */
			dr_controller_setup(udc);
			/* setup EP0 for setup packet */
			ep0_setup(udc);
			/* initialize the USB and EP states */
			udc->usb_state = USB_STATE_ATTACHED;
			udc->ep0_state = WAIT_FOR_SETUP;
			udc->ep0_dir = 0;
			udc->vbus_active = 1;
			/* start the controller */
			dr_controller_run(udc);
			if (udc->vbus_regulator) {
				/* set the current limit to 100mA */
				regulator_set_current_limit(
					udc->vbus_regulator, 0, 100);
			}
			/* Schedule work to wait for 1000 msec and check for
			 * charger if setup packet is not received */
			schedule_delayed_work(&udc->work,
				USB_CHARGER_DETECTION_WAIT_TIME_MS);

		}

		//2010/04/13 william to fix USB state machine messup due to AC plug in/out interrupt 3 times when 
                //user plug in AC just once
                //BUGID ENR_U#17344
		while (udc->myflags)
		{
			msleep(4);
                        if (count++ > 250)
			{
			    USB_INFO("BUGID ENR_U#17344 timeout\n");
                            break;
			}
		}

		spin_lock_irqsave(&udc->lock, flags); /* htc */
		if (online) {
			udc->flags |= USB_FLAG_VBUS_ONLINE;
                        udc->myflags |= USB_FLAG_VBUS_ONLINE;
		} else {
			udc->flags |= USB_FLAG_VBUS_OFFLINE;
			udc->myflags |= USB_FLAG_VBUS_OFFLINE;
		}
		USB_INFO("online = %s udc->flags %d \n", online ? "on" : "off", udc->flags);
		queue_work(udc->usb_wq, &udc->detect_work);
		spin_unlock_irqrestore(&udc->lock, flags); /* htc */

	}

	spin_lock_irqsave(&udc->lock, flags);
	udc->vbus_active = (online != 0);
	if (can_pullup(udc))
		fsl_writel((fsl_readl(&dr_regs->usbcmd) | USB_CMD_RUN_STOP),
				&dr_regs->usbcmd);
	else
		fsl_writel((fsl_readl(&dr_regs->usbcmd) & ~USB_CMD_RUN_STOP),
				&dr_regs->usbcmd);
	spin_unlock_irqrestore(&udc->lock, flags);

	return;
}



int usb_get_connect_type(void)
{
	if (!udc_controller)
		return 0;
	return udc_controller->connect_type;
}
EXPORT_SYMBOL(usb_get_connect_type);


static void ac_detect_expired(unsigned long _data)
{
	struct fsl_udc *ui = (struct fsl_udc *) _data;
	u32 delay = 0;
	u32 portsc;
	u32 ret;

	USB_INFO("%s: count = %d, connect_type = 0x%04x\n", __func__,
			ui->ac_detect_count, ui->connect_type);

	if (ui->connect_type == CONNECT_TYPE_USB || ui->ac_detect_count >= 3)
		return;
#if defined(CONFIG_CABLE_DETECT_ACCESSORY)
	if (cable_detection_det() || cable_detection_ac_only())
		return;
#endif
	/* detect shorted D+/D-, indicating AC power */
	portsc = fsl_readl(&dr_regs->portsc1);
	ret = (portsc & PORTSCX_LINE_STATUS_BITS);
	if ( ret != PORTSCX_LINE_STATUS_BITS) {

		/* Some carkit can't be recognized as AC mode.
		 * Add SW solution here to notify battery driver should
		 * work as AC charger when car mode activated.
		 */
		ui->ac_detect_count++;
		/* detect delay: 3 sec, 5 sec, 10 sec */
		if (ui->ac_detect_count == 1)
			delay = 5 * HZ;
		else if (ui->ac_detect_count == 2)
			delay = 10 * HZ;

		mod_timer(&ui->ac_detect_timer, jiffies + delay);
	} else {
		USB_INFO("USB -> AC charger\n");
		ui->connect_type = CONNECT_TYPE_AC;
		queue_work(ui->usb_wq, &ui->notifier_work);
	}
}

void usb_check_vbus_detection(void)
{
	unsigned long val;
	u32 tmp_reg;
	u32 tmp_reg2;
	int type;

	val =fsl_readl(&usb_sys_regs->vbus_wakeup);
	type = usb_get_connect_type();
	tmp_reg = fsl_readl(&dr_regs->portsc1);
	tmp_reg2 = fsl_readl(&dr_regs->hostpc1devlc);
	usb_check_count++;
	USB_DEBUG("check##(%d) reg:%lx vbus:%d udc_connetc_type:%d port_type:%d(utmi:0) connect_status:%d port_reset:%d \n"
			,usb_check_count,val,vbus_enabled(),type,tmp_reg2 & PORTSCX_PTS_FSLS,
	tmp_reg & PORTSCX_CURRENT_CONNECT_STATUS,tmp_reg & PORTSCX_PORT_RESET);

	queue_delayed_work(udc_controller->usb_wq, &udc_controller->check_vbus_work,
			DELAY_FOR_CHECK_CHG);
}
EXPORT_SYMBOL(usb_check_vbus_detection);

static ssize_t show_check_count(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned length;
	length = sprintf(buf, "%d\n", usb_check_count);
	return length;
}
static ssize_t store_check_count(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	sscanf(buf, "%d", &(usb_check_count));
	if(usb_check_count>2)
	{
		//reset_queues(udc_controller);
		if(&udc_controller->gadget){
		USB_INFO("store_check_count() usb_check_count>2 call reset udc");
		fsl_udc_irq(0,udc_controller);
		usb_check_count = 0;
		}
	}
	return count;
}
static DEVICE_ATTR(check_count, 0644,
		show_check_count, store_check_count);

static ssize_t show_tps_vbus(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned length;
#ifndef CONFIG_ADC_TPS80032
	length = sprintf(buf, "-1\n");
#else
	length = sprintf(buf, "%d\n", tps80032_read_vbus_detection());
#endif
	return length;
}
static DEVICE_ATTR(tps_vbus, 0444, show_tps_vbus, NULL);

static ssize_t show_charger(struct device *dev, struct device_attribute *attr, char *buf)
{
	USB_INFO("show_charger\n");
	unsigned length;
	if(udc_controller->connect_type == CONNECT_TYPE_USB || udc_controller->connect_type == CONNECT_TYPE_UNKNOWN){
		length = sprintf(buf, "%d\n", 1);
	}
	else if(udc_controller->connect_type == CONNECT_TYPE_AC){
		length = sprintf(buf, "%d\n", 2);
	}
	else{
		length = sprintf(buf, "%d\n", 0);
	}
	return length;
}


static ssize_t store_charger(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	USB_INFO("store_charger\n");
	int state;
	sscanf(buf, "%d", &(state));
	if(state)
	{
		udc_controller->connect_type = CONNECT_TYPE_USB;
		queue_work(udc_controller->usb_wq, &udc_controller->notifier_work);
	}
	else{
		udc_controller->connect_type = CONNECT_TYPE_NONE;
		queue_work(udc_controller->usb_wq, &udc_controller->notifier_work);
	}
	return count;
}
static DEVICE_ATTR(turn_on_off_charger, 0644, show_charger, store_charger);

//================ htc porting end =======================================
/*-------------------------------------------------------------------------
		PROC File System Support
-------------------------------------------------------------------------*/
#define CONFIG_USB_GADGET_DEBUG_FILES
#ifdef CONFIG_USB_GADGET_DEBUG_FILES

#include <linux/seq_file.h>

#ifdef CONFIG_ARCH_TEGRA
static const char proc_filename[] = "driver/fsl_tegra_udc";
#else
static const char proc_filename[] = "driver/fsl_usb2_udc";
#endif

static int fsl_proc_read(char *page, char **start, off_t off, int count,
		int *eof, void *_dev)
{
	char *buf = page;
	char *next = buf;
	unsigned size = count;
	unsigned long flags;
	int t, i;
	u32 tmp_reg;
	u32 tmp_reg2;
	struct fsl_ep *ep = NULL;
	struct fsl_req *req;

	struct fsl_udc *udc = udc_controller;
	if (off != 0)
		return 0;

	spin_lock_irqsave(&udc->lock, flags);

	/* ------basic driver information ---- */
	t = scnprintf(next, size,
			DRIVER_DESC "\n"
			"%s version: %s\n"
			"Gadget driver: %s\n\n",
			driver_name, DRIVER_VERSION,
			udc->driver ? udc->driver->driver.name : "(none)");
	size -= t;
	next += t;

	/* ------ DR Registers ----- */
	tmp_reg = fsl_readl(&dr_regs->usbcmd);
	t = scnprintf(next, size,
			"USBCMD reg:\n"
			"SetupTW: %d\n"
			"Run/Stop: %s\n\n",
			(tmp_reg & USB_CMD_SUTW) ? 1 : 0,
			(tmp_reg & USB_CMD_RUN_STOP) ? "Run" : "Stop");
	size -= t;
	next += t;

	tmp_reg = fsl_readl(&dr_regs->usbsts);
	t = scnprintf(next, size,
			"USB Status Reg:\n"
			"Dr Suspend: %d Reset Received: %d System Error: %s "
			"USB Error Interrupt: %s\n\n",
			(tmp_reg & USB_STS_SUSPEND) ? 1 : 0,
			(tmp_reg & USB_STS_RESET) ? 1 : 0,
			(tmp_reg & USB_STS_SYS_ERR) ? "Err" : "Normal",
			(tmp_reg & USB_STS_ERR) ? "Err detected" : "No err");
	size -= t;
	next += t;

	tmp_reg = fsl_readl(&dr_regs->usbintr);
	t = scnprintf(next, size,
			"USB Intrrupt Enable Reg:\n"
			"Sleep Enable: %d SOF Received Enable: %d "
			"Reset Enable: %d\n"
			"System Error Enable: %d "
			"Port Change Dectected Enable: %d\n"
			"USB Error Intr Enable: %d USB Intr Enable: %d\n\n",
			(tmp_reg & USB_INTR_DEVICE_SUSPEND) ? 1 : 0,
			(tmp_reg & USB_INTR_SOF_EN) ? 1 : 0,
			(tmp_reg & USB_INTR_RESET_EN) ? 1 : 0,
			(tmp_reg & USB_INTR_SYS_ERR_EN) ? 1 : 0,
			(tmp_reg & USB_INTR_PTC_DETECT_EN) ? 1 : 0,
			(tmp_reg & USB_INTR_ERR_INT_EN) ? 1 : 0,
			(tmp_reg & USB_INTR_INT_EN) ? 1 : 0);
	size -= t;
	next += t;

	tmp_reg = fsl_readl(&dr_regs->frindex);
	t = scnprintf(next, size,
			"USB Frame Index Reg: Frame Number is 0x%x\n\n",
			(tmp_reg & USB_FRINDEX_MASKS));
	size -= t;
	next += t;

	tmp_reg = fsl_readl(&dr_regs->deviceaddr);
	t = scnprintf(next, size,
			"USB Device Address Reg: Device Addr is 0x%x\n\n",
			(tmp_reg & USB_DEVICE_ADDRESS_MASK));
	size -= t;
	next += t;

	tmp_reg = fsl_readl(&dr_regs->endpointlistaddr);
	t = scnprintf(next, size,
			"USB Endpoint List Address Reg: "
			"Device Addr is 0x%x\n\n",
			(tmp_reg & USB_EP_LIST_ADDRESS_MASK));
	size -= t;
	next += t;

	tmp_reg = fsl_readl(&dr_regs->portsc1);
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	tmp_reg2 = tmp_reg;
#else
	/* In Tegra3 the Phy Type Select(PTS) and Port Speed fields are specified in
	 * hostpc1devlc register instead of portsc1 register. */
	tmp_reg2 = fsl_readl(&dr_regs->hostpc1devlc);
#endif
	t = scnprintf(next, size,
		"USB Port Status&Control Reg:\n"
		"Port Transceiver Type : %s Port Speed: %s\n"
		"PHY Low Power Suspend: %s Port Reset: %s "
		"Port Suspend Mode: %s\n"
		"Over-current Change: %s "
		"Port Enable/Disable Change: %s\n"
		"Port Enabled/Disabled: %s\n"
		"Current Connect Status: %s\n\n"
		"Chareger type: %s\n"
		"OTG state: %d (3:B_PERIPHERAL 9:A_HOST 10:A_SUSPEND)\n"
		, ( {
			char *s;
			switch (tmp_reg2 & PORTSCX_PTS_FSLS) {
			case PORTSCX_PTS_UTMI:
				s = "UTMI"; break;
			case PORTSCX_PTS_ULPI:
				s = "ULPI "; break;
			case PORTSCX_PTS_FSLS:
				s = "FS/LS Serial"; break;
			default:
				s = "None"; break;
			}
			s;} ), ( {
			char *s;
			switch (tmp_reg2 & PORTSCX_PORT_SPEED_UNDEF) {
			case PORTSCX_PORT_SPEED_FULL:
				s = "Full Speed"; break;
			case PORTSCX_PORT_SPEED_LOW:
				s = "Low Speed"; break;
			case PORTSCX_PORT_SPEED_HIGH:
				s = "High Speed"; break;
			default:
				s = "Undefined"; break;
			}
			s;
		} ),
		(tmp_reg & PORTSCX_PHY_LOW_POWER_SPD) ?
		"Normal PHY mode" : "Low power mode",
		(tmp_reg & PORTSCX_PORT_RESET) ? "In Reset" :
		"Not in Reset",
		(tmp_reg & PORTSCX_PORT_SUSPEND) ? "In " : "Not in",
		(tmp_reg & PORTSCX_OVER_CURRENT_CHG) ? "Dected" :
		"No",
		(tmp_reg & PORTSCX_PORT_EN_DIS_CHANGE) ? "Disable" :
		"Not change",
		(tmp_reg & PORTSCX_PORT_ENABLE) ? "Enable" :
		"Not correct",
		(tmp_reg & PORTSCX_CURRENT_CONNECT_STATUS) ?
		"Attached" : "Not-Att",
		((tmp_reg & PORTSCX_LINE_STATUS_BITS) == PORTSCX_LINE_STATUS_BITS) ?
		"AC" : "USB"
		,udc->transceiver ? udc->transceiver->state:-2
		);
	size -= t;
	next += t;

	tmp_reg = fsl_readl(&dr_regs->usbmode);
	t = scnprintf(next, size,
			"USB Mode Reg: Controller Mode is: %s\n\n", ( {
				char *s;
				switch (tmp_reg & USB_MODE_CTRL_MODE_HOST) {
				case USB_MODE_CTRL_MODE_IDLE:
					s = "Idle"; break;
				case USB_MODE_CTRL_MODE_DEVICE:
					s = "Device Controller"; break;
				case USB_MODE_CTRL_MODE_HOST:
					s = "Host Controller"; break;
				default:
					s = "None"; break;
				}
				s;
			} ));
	size -= t;
	next += t;

	tmp_reg = fsl_readl(&dr_regs->endptsetupstat);
	t = scnprintf(next, size,
			"Endpoint Setup Status Reg: SETUP on ep 0x%x\n\n",
			(tmp_reg & EP_SETUP_STATUS_MASK));
	size -= t;
	next += t;

	for (i = 0; i < udc->max_ep / 2; i++) {
		tmp_reg = fsl_readl(&dr_regs->endptctrl[i]);
		t = scnprintf(next, size, "EP Ctrl Reg [0x%x]: = [0x%x]\n",
				i, tmp_reg);
		size -= t;
		next += t;
	}
	tmp_reg = fsl_readl(&dr_regs->endpointprime);
	t = scnprintf(next, size, "EP Prime Reg = [0x%x]\n\n", tmp_reg);
	size -= t;
	next += t;

#if !defined(CONFIG_ARCH_MXC) && !defined(CONFIG_ARCH_TEGRA)
	tmp_reg = usb_sys_regs->snoop1;
	t = scnprintf(next, size, "Snoop1 Reg : = [0x%x]\n\n", tmp_reg);
	size -= t;
	next += t;

	tmp_reg = usb_sys_regs->control;
	t = scnprintf(next, size, "General Control Reg : = [0x%x]\n\n",
			tmp_reg);
	size -= t;
	next += t;
#endif

	/* ------fsl_udc, fsl_ep, fsl_request structure information ----- */
	ep = &udc->eps[0];
	t = scnprintf(next, size, "For %s Maxpkt is 0x%x index is 0x%x\n",
			ep->ep.name, ep_maxpacket(ep), ep_index(ep));
	size -= t;
	next += t;

	if (list_empty(&ep->queue)) {
		t = scnprintf(next, size, "its req queue is empty\n\n");
		size -= t;
		next += t;
	} else {
		list_for_each_entry(req, &ep->queue, queue) {
			t = scnprintf(next, size,
				"req %p actual 0x%x length 0x%x buf %p\n",
				&req->req, req->req.actual,
				req->req.length, req->req.buf);
			size -= t;
			next += t;
		}
	}
	/* other gadget->eplist ep */
	list_for_each_entry(ep, &udc->gadget.ep_list, ep.ep_list) {
		if (ep->desc) {
			t = scnprintf(next, size,
					"\nFor %s Maxpkt is 0x%x "
					"index is 0x%x\n",
					ep->ep.name, ep_maxpacket(ep),
					ep_index(ep));
			size -= t;
			next += t;

			if (list_empty(&ep->queue)) {
				t = scnprintf(next, size,
						"its req queue is empty\n\n");
				size -= t;
				next += t;
			} else {
				list_for_each_entry(req, &ep->queue, queue) {
					t = scnprintf(next, size,
						"req %p actual 0x%x length "
						"0x%x  buf %p\n",
						&req->req, req->req.actual,
						req->req.length, req->req.buf);
					size -= t;
					next += t;
					}	/* end for each_entry of ep req */
				}	/* end for else */
			}	/* end for if(ep->queue) */
		}		/* end (ep->desc) */

	spin_unlock_irqrestore(&udc->lock, flags);

	*eof = 1;
	return count - size;
}

#define create_proc_file()	create_proc_read_entry(proc_filename, \
				0, NULL, fsl_proc_read, NULL)

#define remove_proc_file()	remove_proc_entry(proc_filename, NULL)

#else				/* !CONFIG_USB_GADGET_DEBUG_FILES */

#define create_proc_file()	do {} while (0)
#define remove_proc_file()	do {} while (0)

#endif				/* CONFIG_USB_GADGET_DEBUG_FILES */

/*-------------------------------------------------------------------------*/

/* Release udc structures */
static void fsl_udc_release(struct device *dev)
{
	complete(udc_controller->done);
#ifndef CONFIG_ARCH_TEGRA
	dma_free_coherent(dev->parent, udc_controller->ep_qh_size,
			udc_controller->ep_qh, udc_controller->ep_qh_dma);
#endif
	kfree(udc_controller);
}

/******************************************************************
	Internal structure setup functions
*******************************************************************/
/*------------------------------------------------------------------
 * init resource for globle controller
 * Return the udc handle on success or NULL on failure
 ------------------------------------------------------------------*/
static int __init struct_udc_setup(struct fsl_udc *udc,
		struct platform_device *pdev)
{
	struct fsl_usb2_platform_data *pdata;
	size_t size;

	pdata = pdev->dev.platform_data;
	udc->phy_mode = pdata->phy_mode;
//TODO:	udc->offmode_charge = pdata->offmode_charge;

	udc->eps = kzalloc(sizeof(struct fsl_ep) * udc->max_ep, GFP_KERNEL);
	if (!udc->eps) {
		ERR("malloc fsl_ep failed\n");
		return -1;
	}

#ifdef CONFIG_ARCH_TEGRA
	/* Tegra uses hardware queue heads */
	size = udc->max_ep * sizeof(struct ep_queue_head);
	udc->ep_qh = (struct ep_queue_head *)((u8 *)dr_regs + QH_OFFSET);
	udc->ep_qh_dma = platform_get_resource(pdev, IORESOURCE_MEM, 0)->start +
		QH_OFFSET;
#else
	/* initialized QHs, take care of alignment */
	size = udc->max_ep * sizeof(struct ep_queue_head);
	if (size < QH_ALIGNMENT)
		size = QH_ALIGNMENT;
	else if ((size % QH_ALIGNMENT) != 0) {
		size += QH_ALIGNMENT + 1;
		size &= ~(QH_ALIGNMENT - 1);
	}
	udc->ep_qh = dma_alloc_coherent(&pdev->dev, size,
					&udc->ep_qh_dma, GFP_KERNEL);
	if (!udc->ep_qh) {
		ERR("malloc QHs for udc failed\n");
		kfree(udc->eps);
		return -1;
	}
#endif

	udc->ep_qh_size = size;

	/* Initialize ep0 status request structure */
	/* FIXME: fsl_alloc_request() ignores ep argument */
	udc->status_req = container_of(fsl_alloc_request(NULL, GFP_KERNEL),
			struct fsl_req, req);
	/* allocate a small amount of memory to get valid address */
	udc->status_req->req.buf = dma_alloc_coherent(&pdev->dev,
				STATUS_BUFFER_SIZE, &udc->status_req->req.dma,
				GFP_KERNEL);
	if (!udc->status_req->req.buf) {
		ERR("alloc status_req buffer failed\n");
#ifndef CONFIG_ARCH_TEGRA
		dma_free_coherent(&pdev->dev, size, udc->ep_qh, udc->ep_qh_dma);
#endif
		kfree(udc->eps);
		return -ENOMEM;
	}

	udc->resume_state = USB_STATE_NOTATTACHED;
	udc->usb_state = USB_STATE_POWERED;
	udc->ep0_dir = 0;
	udc->remote_wakeup = 0;	/* default to 0 on reset */

	return 0;
}

/*----------------------------------------------------------------
 * Setup the fsl_ep struct for eps
 * Link fsl_ep->ep to gadget->ep_list
 * ep0out is not used so do nothing here
 * ep0in should be taken care
 *--------------------------------------------------------------*/
static int __init struct_ep_setup(struct fsl_udc *udc, unsigned char index,
		char *name, int link)
{
	struct fsl_ep *ep = &udc->eps[index];

	ep->udc = udc;
	strcpy(ep->name, name);
	ep->ep.name = ep->name;

	ep->ep.ops = &fsl_ep_ops;
	ep->stopped = 0;

	/* for ep0: maxP defined in desc
	 * for other eps, maxP is set by epautoconfig() called by gadget layer
	 */
	ep->ep.maxpacket = (unsigned short) ~0;

	/* the queue lists any req for this ep */
	INIT_LIST_HEAD(&ep->queue);

	/* gagdet.ep_list used for ep_autoconfig so no ep0 */
	if (link)
		list_add_tail(&ep->ep.ep_list, &udc->gadget.ep_list);
	ep->gadget = &udc->gadget;
	ep->qh = &udc->ep_qh[index];

	return 0;
}

/* Driver probe function
 * all intialization operations implemented here except enabling usb_intr reg
 * board setup should have been done in the platform code
 */
static int __init fsl_udc_probe(struct platform_device *pdev)
{
	struct resource *res;
	int ret = -ENODEV;
	unsigned int i;
	u32 dccparams;
#if defined(CONFIG_ARCH_TEGRA)
	struct resource *res_sys = NULL;
	struct fsl_usb2_platform_data *pdata = pdev->dev.platform_data;
#endif

	if (strcmp(pdev->name, driver_name)) {
		VDBG("Wrong device");
		return -ENODEV;
	}

	udc_controller = kzalloc(sizeof(struct fsl_udc), GFP_KERNEL);
	if (udc_controller == NULL) {
		ERR("malloc udc failed\n");
		return -ENOMEM;
	}

	spin_lock_init(&udc_controller->lock);
	udc_controller->stopped = 1;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		ret = -ENXIO;
		goto err_kfree;
	}

	if (!request_mem_region(res->start, res->end - res->start + 1,
				driver_name)) {
		ERR("request mem region for %s failed\n", pdev->name);
		ret = -EBUSY;
		goto err_kfree;
	}

	dr_regs = ioremap(res->start, resource_size(res));
	if (!dr_regs) {
		ret = -ENOMEM;
		goto err_release_mem_region;
	}

#if defined(CONFIG_ARCH_TEGRA)
	/* If the PHY registers are NOT provided as a seperate aperture, then
	 * we should be using the registers inside the controller aperture. */
	res_sys = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (res_sys)  {
		usb_sys_regs = ioremap(res_sys->start, resource_size(res_sys));
		if (!usb_sys_regs)
			goto err_release_mem_region;
	} else {
		usb_sys_regs = (struct usb_sys_interface *)
			((u32)dr_regs + USB_DR_SYS_OFFSET);
	}
#endif

#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	control_reg = &dr_regs->portsc1;
#else
	control_reg = &dr_regs->hostpc1devlc;
#endif
#if !defined(CONFIG_ARCH_MXC) && !defined(CONFIG_ARCH_TEGRA)
	usb_sys_regs = (struct usb_sys_interface *)
			((u32)dr_regs + USB_DR_SYS_OFFSET);
#endif

	/* Initialize USB clocks */
	ret = fsl_udc_clk_init(pdev);
	if (ret < 0)
		goto err_iounmap_noclk;

	/* Read Device Controller Capability Parameters register */
	dccparams = fsl_readl(&dr_regs->dccparams);
	if (!(dccparams & DCCPARAMS_DC)) {
		ERR("This SOC doesn't support device role\n");
		ret = -ENODEV;
		goto err_iounmap;
	}
	/* Get max device endpoints */
	/* DEN is bidirectional ep number, max_ep doubles the number */
	udc_controller->max_ep = (dccparams & DCCPARAMS_DEN_MASK) * 2;
	udc_controller->irq = platform_get_irq(pdev, 0);
	if (!udc_controller->irq) {
		ret = -ENODEV;
		goto err_iounmap;
	}

	ret = request_irq(udc_controller->irq, fsl_udc_irq, IRQF_SHARED,
			driver_name, udc_controller);
	if (ret != 0) {
		ERR("cannot request irq %d err %d\n",
				udc_controller->irq, ret);
		goto err_iounmap;
	}
	enable_irq_wake(udc_controller->irq);
	ret = tegra_usb_set_vbus_wakeup(INT_USB);

	/* Initialize the udc structure including QH member and other member */
	if (struct_udc_setup(udc_controller, pdev)) {
		ERR("Can't initialize udc data structure\n");
		ret = -ENOMEM;
		goto err_free_irq;
	}

	/* initialize usb hw reg except for regs for EP,
	 * leave usbintr reg untouched */
	dr_controller_setup(udc_controller);

	fsl_udc_clk_finalize(pdev);

	/* Setup gadget structure */
	udc_controller->gadget.ops = &fsl_gadget_ops;
	udc_controller->gadget.is_dualspeed = 1;
	udc_controller->gadget.ep0 = &udc_controller->eps[0].ep;
	INIT_LIST_HEAD(&udc_controller->gadget.ep_list);
	udc_controller->gadget.speed = USB_SPEED_UNKNOWN;
	udc_controller->gadget.name = driver_name;

	/* Setup gadget.dev and register with kernel */
	dev_set_name(&udc_controller->gadget.dev, "gadget");
	udc_controller->gadget.dev.release = fsl_udc_release;
	udc_controller->gadget.dev.parent = &pdev->dev;
	ret = device_register(&udc_controller->gadget.dev);
	if (ret < 0)
		goto err_free_irq;

	/* setup QH and epctrl for ep0 */
	ep0_setup(udc_controller);

	/* setup udc->eps[] for ep0 */
	struct_ep_setup(udc_controller, 0, "ep0", 0);
	/* for ep0: the desc defined here;
	 * for other eps, gadget layer called ep_enable with defined desc
	 */
	udc_controller->eps[0].desc = &fsl_ep0_desc;
	udc_controller->eps[0].ep.maxpacket = USB_MAX_CTRL_PAYLOAD;

	/* setup the udc->eps[] for non-control endpoints and link
	 * to gadget.ep_list */
	for (i = 1; i < (int)(udc_controller->max_ep / 2); i++) {
		char name[30];

		sprintf(name, "ep%dout", i);
		struct_ep_setup(udc_controller, i * 2, name, 1);
		sprintf(name, "ep%din", i);
		struct_ep_setup(udc_controller, i * 2 + 1, name, 1);
	}

	/* use dma_pool for TD management */
	udc_controller->td_pool = dma_pool_create("udc_td", &pdev->dev,
			sizeof(struct ep_td_struct),
			DTD_ALIGNMENT, UDC_DMA_BOUNDARY);
	if (udc_controller->td_pool == NULL) {
		ret = -ENOMEM;
		goto err_unregister;
	}
	usb_prepare(udc_controller);	//porting
	create_proc_file();
	ret = device_create_file(&pdev->dev,
		&dev_attr_check_count);
	if (ret != 0)
		USB_ERR("dev_attr_check_count failed\n");
	ret = device_create_file(&pdev->dev,
		&dev_attr_tps_vbus);
	if (ret != 0)
		USB_ERR("dev_attr_tps_vbus failed\n");

	ret = device_create_file(&pdev->dev,
		&dev_attr_turn_on_off_charger);
	if (ret != 0)
		USB_ERR("dev_attr_check_count failed\n");

	/* create a delayed work for detecting the USB charger */
	INIT_DELAYED_WORK(&udc_controller->work, fsl_udc_charger_detect_work);
	INIT_WORK(&udc_controller->charger_work, fsl_udc_set_current_limit_work);

	/* Get the regulator for drawing the vbus current in udc driver */
	/* htc don't need this regulator
	udc_controller->vbus_regulator = regulator_get(NULL, "usb_bat_chg");
	if (IS_ERR(udc_controller->vbus_regulator)) {
		dev_err(&pdev->dev,
			"can't get charge regulator,err:%ld\n",
			PTR_ERR(udc_controller->vbus_regulator));
		udc_controller->vbus_regulator = NULL;
	}*/
	udc_controller->vbus_regulator = NULL;

#ifdef CONFIG_USB_OTG_UTILS
	udc_controller->transceiver = otg_get_transceiver();
	if (udc_controller->transceiver) {
		dr_controller_stop(udc_controller);
		dr_controller_reset(udc_controller);
		fsl_udc_clk_suspend(false);
		udc_controller->vbus_active = 0;
		udc_controller->usb_state = USB_STATE_DEFAULT;
		otg_set_peripheral(udc_controller->transceiver, &udc_controller->gadget);
	}
#else
#ifdef CONFIG_ARCH_TEGRA
	/* Power down the phy if cable is not connected */
	if(!vbus_enabled())
		fsl_udc_clk_suspend(false);
#endif
#endif
	// ++ htc ++
	udc_controller->ac_detect_count = 0;
	udc_controller->ac_detect_timer.data = (unsigned long) udc_controller;
	udc_controller->ac_detect_timer.function = ac_detect_expired;
	init_timer(&udc_controller->ac_detect_timer);
	first_online = 0;
	usb_check_count = 0;
        udc_controller->myflags = 0;
	// -- htc --
	return 0;

err_unregister:
	device_unregister(&udc_controller->gadget.dev);
err_free_irq:
	free_irq(udc_controller->irq, udc_controller);
err_iounmap:
	fsl_udc_clk_release();
err_iounmap_noclk:
	iounmap(dr_regs);
err_release_mem_region:
	release_mem_region(res->start, res->end - res->start + 1);
err_kfree:
	kfree(udc_controller);
	udc_controller = NULL;
	return ret;
}

/* Driver removal function
 * Free resources and finish pending transactions
 */
static int __exit fsl_udc_remove(struct platform_device *pdev)
{
	struct resource *res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	DECLARE_COMPLETION(done);

	if (!udc_controller)
		return -ENODEV;
	udc_controller->done = &done;

	cancel_delayed_work(&udc_controller->work);
	if (udc_controller->vbus_regulator)
		regulator_put(udc_controller->vbus_regulator);

	if (udc_controller->transceiver)
		otg_set_peripheral(udc_controller->transceiver, NULL);

	fsl_udc_clk_release();

	/* DR has been stopped in usb_gadget_unregister_driver() */
	remove_proc_file();

	/* Free allocated memory */
	dma_free_coherent(&pdev->dev, STATUS_BUFFER_SIZE,
				udc_controller->status_req->req.buf,
				udc_controller->status_req->req.dma);
	kfree(udc_controller->status_req);
	kfree(udc_controller->eps);

	dma_pool_destroy(udc_controller->td_pool);
	free_irq(udc_controller->irq, udc_controller);
	iounmap(dr_regs);
	release_mem_region(res->start, res->end - res->start + 1);

	device_unregister(&udc_controller->gadget.dev);
	/* free udc --wait for the release() finished */
	wait_for_completion(&done);

	return 0;
}

/*-----------------------------------------------------------------
 * Modify Power management attributes
 * Used by OTG statemachine to disable gadget temporarily
 -----------------------------------------------------------------*/
static int fsl_udc_suspend(struct platform_device *pdev, pm_message_t state)
{
#if 0
    if (udc_controller->transceiver) {
        if (udc_controller->transceiver->state != OTG_STATE_B_PERIPHERAL) {
            /* we are not in device mode, return */
			//enable_irq_wake(udc_controller->irq);
			USB_INFO("fsl_udc_suspend note: otg-state:%d",udc_controller->transceiver->state);
            return 0;
        }
    }
    if (udc_controller->vbus_active) {
        spin_lock(&udc_controller->lock);
        /* Reset all internal Queues and inform client driver */
        reset_queues(udc_controller);
        udc_controller->vbus_active = 0;
        udc_controller->usb_state = USB_STATE_DEFAULT;
        spin_unlock(&udc_controller->lock);
    }
    /* stop the controller and turn off the clocks */
    dr_controller_stop(udc_controller);
    if (udc_controller->transceiver) {
        udc_controller->transceiver->state = OTG_STATE_UNDEFINED;
    }
    fsl_udc_clk_suspend(true);
	//enable_irq_wake(udc_controller->irq);
#endif
    USB_INFO("fsl_udc_suspend ");
    return 0;
}

/*-----------------------------------------------------------------
 * Invoked on USB resume. May be called in_interrupt.
 * Here we start the DR controller and enable the irq
 *-----------------------------------------------------------------*/
static int fsl_udc_resume(struct platform_device *pdev)
{
	unsigned long val;
	USB_INFO("fsl_udc_resume #0");
	irq_udc_debug =1 ;
	irq_otg_debug =1 ;
	if(global_wakeup_state == VBUS_WAKEUP_ENR)
		wake_lock_timeout(&udc_resume_wake_lock, 8*HZ);

#if 0
	val =fsl_readl(&usb_sys_regs->vbus_wakeup);
	USB_INFO("fsl_udc_resume#1 reg:%lx",val);
	if (udc_controller->transceiver) {
		fsl_udc_clk_enable();
#if 0
		if (!(fsl_readl(&usb_sys_regs->vbus_wakeup) & USB_SYS_ID_PIN_STATUS)) {
			/* If ID status is low means host is connected, return */
			USB_INFO("fsl_udc_resume #1");
			fsl_udc_clk_disable();
			//disable_irq_wake(udc_controller->irq);
			return 0;
		}
#endif
		/* check for VBUS */
		if (!(fsl_readl(&usb_sys_regs->vbus_wakeup) & USB_SYS_VBUS_STATUS)) {
			/* if there is no VBUS then power down the clocks and return */
			USB_INFO("fsl_udc_resume #2 no vbus");
			fsl_udc_clk_disable();
			//disable_irq_wake(udc_controller->irq);
			return 0;
		} else {

			fsl_udc_clk_disable();
			if (udc_controller->transceiver->state == OTG_STATE_A_HOST) {
			    //disable_irq_wake(udc_controller->irq);
			    return 0;
			}
			/* Detected VBUS set the transceiver state to device mode */
			udc_controller->transceiver->state = OTG_STATE_B_PERIPHERAL;
		}
	}

	fsl_udc_clk_resume(true);
#if defined(CONFIG_ARCH_TEGRA)
	fsl_udc_restart(udc_controller);
#else
	/* Enable DR irq reg and set controller Run */
	if (udc_controller->stopped) {
		dr_controller_setup(udc_controller);
		dr_controller_run(udc_controller);
	}
	udc_controller->usb_state = USB_STATE_ATTACHED;
	udc_controller->ep0_state = WAIT_FOR_SETUP;
	udc_controller->ep0_dir = 0;
#endif
	/* Power down the phy if cable is not connected */
	if (!(fsl_readl(&usb_sys_regs->vbus_wakeup) & USB_SYS_VBUS_STATUS))
		fsl_udc_clk_suspend(false);
	//disable_irq_wake(udc_controller->irq);
#endif
	return 0;
}

/*-------------------------------------------------------------------------
	Register entry point for the peripheral controller driver
--------------------------------------------------------------------------*/

static struct platform_driver udc_driver = {
	.remove  = __exit_p(fsl_udc_remove),
	/* these suspend and resume are not usb suspend and resume */
	.suspend = fsl_udc_suspend,
	.resume  = fsl_udc_resume,
	.driver  = {
		.name = (char *)driver_name,
		.owner = THIS_MODULE,
	},
};

static int __init udc_init(void)
{
	USB_INFO("%s (%s)\n", driver_desc, DRIVER_VERSION);
	wake_lock_init(&udc_wake_lock, WAKE_LOCK_SUSPEND, "usb_udc_lock");
	wake_lock_init(&udc_wake_lock2, WAKE_LOCK_SUSPEND, "usb_udc_lock2");
	wake_lock_init(&udc_resume_wake_lock, WAKE_LOCK_SUSPEND, "usb_udc_resume_lock");
	return platform_driver_probe(&udc_driver, fsl_udc_probe);
}

module_init(udc_init);

static void __exit udc_exit(void)
{
	platform_driver_unregister(&udc_driver);
	USB_WARNING("%s unregistered\n", driver_desc);
}

module_exit(udc_exit);

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:fsl-usb2-udc");
