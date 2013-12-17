/*
 * Copyright (c) 2012, NVIDIA CORPORATION.  All rights reserved.
 *
 * Description:
 * High-speed USB device controller driver.
 * The driver is based on Freescale driver code from Li Yang and Jiang Bo.
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
#include <linux/dmapool.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <linux/workqueue.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/pm_qos_params.h>

#include <asm/byteorder.h>
#include <asm/io.h>
#include <asm/system.h>
#include <asm/unaligned.h>
#include <asm/dma.h>

#include <mach/usb_phy.h>
#include <mach/iomap.h>
#include <linux/usb/htc_info.h>
#include "tegra_udc.h"

#include <mach/htc_battery_common.h>

#define VBUS_WAKEUP_ENR 19
extern int global_wakeup_state;

#define	DRIVER_DESC	"Nvidia Tegra High-Speed USB SOC \
					Device Controller driver"

#define	DRIVER_AUTHOR	"Venkat Moganty/Rakesh Bodla"
#define	DRIVER_VERSION	"Apr 30, 2012"
#define USB1_PREFETCH_ID	6

#define get_ep_by_pipe(udc, pipe)	((pipe == 1) ? &udc->eps[0] : \
						&udc->eps[pipe])
#define get_pipe_by_windex(windex)	((windex & USB_ENDPOINT_NUMBER_MASK) \
					* 2 + ((windex & USB_DIR_IN) ? 1 : 0))

static const char driver_name[] = "tegra-udc";
static const char driver_desc[] = DRIVER_DESC;

static void tegra_ep_fifo_flush(struct usb_ep *_ep);
static int reset_queues(struct tegra_udc *udc);

/*
 * High speed test mode packet(53 bytes).
 * See USB 2.0 spec, section 7.1.20.
 */
static const u8 tegra_udc_test_packet[53] = {
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

/* htc:remove static for cable_detect driver use */
struct tegra_udc *the_udc;
extern int USB_disabled;
static int vbus_active_USB_disabled = 0;

/* ++ htc ++ */
static int reset_queues_mute(struct tegra_udc *udc);
#define IO_USB_BASE   0x7D000000
#define UTMIP_HSRX_CFG1		0x814
#define UTMIP_HS_SYNC_START_DLY(x)	(((x) & 0x1f) << 1)
/* -- htc -- */

#ifdef CONFIG_TEGRA_GADGET_BOOST_CPU_FREQ
	static struct pm_qos_request_list boost_cpu_freq_req;
	static u32 ep_queue_request_count;
	static u8 boost_cpufreq_work_flag;
#endif

static inline void udc_writel(struct tegra_udc *udc, u32 val, u32 offset)
{
	writel(val, udc->regs + offset);
}

static inline unsigned int udc_readl(struct tegra_udc *udc, u32 offset)
{
	return readl(udc->regs + offset);
}

static int ep_index(struct tegra_ep *ep)
{
	if (ep->desc)
		return ep->desc->bEndpointAddress&0xF;
	else {
		pr_err("[USB] ep->desc is null\n");
		return -1;
	}
}

static int ep_is_in(struct tegra_ep *ep)
{
	if (ep->desc)
		return (ep_index(ep) == 0) ? (ep->udc->ep0_dir == USB_DIR_IN)
			 :((ep->desc->bEndpointAddress & USB_DIR_IN) == USB_DIR_IN);
	else {
		pr_err("[USB] ep->desc is null\n");
		return 0;
	}
}

/* checks vbus status */
static inline bool vbus_enabled(struct tegra_udc *udc)
{
	bool status = false;
	status = (udc_readl(udc, VBUS_WAKEUP_REG_OFFSET) & USB_SYS_VBUS_STATUS);
	return status;
}

#include "htc_setting.c"

/**
 * done() - retire a request; caller blocked irqs
 * @status : request status to be set, only works when
 * request is still in progress.
 */
static void done(struct tegra_ep *ep, struct tegra_req *req, int status)
{
	struct tegra_udc *udc = NULL;
	unsigned char stopped = ep->stopped;
	struct ep_td_struct *curr_td, *next_td = 0;
	int j;
	int count;
	BUG_ON(!(in_irq() || irqs_disabled()));
	udc = (struct tegra_udc *)ep->udc;
	/* Removed the req from tegra_ep->queue */
	list_del_init(&req->queue);

	/* req.status should be set as -EINPROGRESS in ep_queue() */
	if (req->req.status == -EINPROGRESS)
		req->req.status = status;
	else
		status = req->req.status;

	/* Free dtd for the request */
	count = 0;
	if (ep->last_td) {
		next_td = ep->last_td;
		count = ep->last_dtd_count;
	}
	ep->last_td = req->head;
	ep->last_dtd_count = req->dtd_count;

	for (j = 0; j < count; j++) {
		curr_td = next_td;
		if (j != count - 1) {
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
#ifdef CONFIG_TEGRA_GADGET_BOOST_CPU_FREQ
	if (req->req.complete && req->req.length >= BOOST_TRIGGER_SIZE) {
		ep_queue_request_count--;
		if (!ep_queue_request_count)
			schedule_work(&udc->boost_cpufreq_work);
	}
#endif

	/* complete() is from gadget layer,
	 * eg fsg->bulk_in_complete() */
	if (req->req.complete) {
		spin_unlock(&ep->udc->lock);
		req->req.complete(&ep->ep, &req->req);
		spin_lock(&ep->udc->lock);
	}

	ep->stopped = stopped;
}

/*
 * nuke(): delete all requests related to this ep
 * Must be called with spinlock held and interrupt disabled
 */
static void nuke(struct tegra_ep *ep, int status)
{
	ep->stopped = 1;

	/* Flush fifo */
	tegra_ep_fifo_flush(&ep->ep);

	/* Whether this eq has request linked */
	while (!list_empty(&ep->queue)) {
		struct tegra_req *req = NULL;

		req = list_entry(ep->queue.next, struct tegra_req, queue);
		done(ep, req, status);
	}
}

static int can_pullup(struct tegra_udc *udc)
{
	DBG("%s(%d) udc->softconnect = %d udc->vbus_active = %d\n",
		__func__, __LINE__, udc->softconnect, udc->vbus_active);
	return udc->driver && udc->softconnect && udc->vbus_active;
}

static int dr_controller_reset(struct tegra_udc *udc)
{
	unsigned int tmp;
	unsigned long timeout;
	DBG("%s(%d) BEGIN\n", __func__, __LINE__);

	/* Stop and reset the usb controller */
	tmp = udc_readl(udc, USB_CMD_REG_OFFSET);
	tmp &= ~USB_CMD_RUN_STOP;
	udc_writel(udc, tmp, USB_CMD_REG_OFFSET);

	tmp = udc_readl(udc, USB_CMD_REG_OFFSET);
	tmp |= USB_CMD_CTRL_RESET;
	udc_writel(udc, tmp, USB_CMD_REG_OFFSET);

	/* Wait for reset to complete */
	timeout = jiffies + UDC_RESET_TIMEOUT_MS;
	while (udc_readl(udc, USB_CMD_REG_OFFSET) & USB_CMD_CTRL_RESET) {
		if (time_after(jiffies, timeout)) {
			ERR("udc reset timeout!\n");
			return -ETIMEDOUT;
		}
		cpu_relax();
	}

	DBG("%s(%d) END\n", __func__, __LINE__);
	return 0;
}

static int dr_controller_setup(struct tegra_udc *udc)
{
	unsigned int tmp, portctrl;
	unsigned long timeout;
	int status;
	unsigned int port_control_reg_offset;
	DBG("%s(%d) BEGIN\n", __func__, __LINE__);

	if (udc->has_hostpc)
		port_control_reg_offset = USB_HOSTPCX_DEVLC_REG_OFFSET;
	else
		port_control_reg_offset = PORTSCX_REG_OFFSET;

	/* Config PHY interface */
	portctrl = udc_readl(udc, port_control_reg_offset);
	portctrl &= ~(PORTSCX_PHY_TYPE_SEL | PORTSCX_PORT_WIDTH);
	portctrl |= PORTSCX_PTS_UTMI;
	udc_writel(udc, portctrl, port_control_reg_offset);

	status = dr_controller_reset(udc);
	if (status)
		return status;

	/* Set the controller as device mode */
	tmp = udc_readl(udc, USB_MODE_REG_OFFSET);
	tmp |= USB_MODE_CTRL_MODE_DEVICE;
	/* Disable Setup Lockout */
	tmp |= USB_MODE_SETUP_LOCK_OFF;
	udc_writel(udc, tmp, USB_MODE_REG_OFFSET);

	/* Wait for controller to switch to device mode */
	timeout = jiffies + UDC_RESET_TIMEOUT_MS;
	while ((udc_readl(udc, USB_MODE_REG_OFFSET) &
		USB_MODE_CTRL_MODE_DEVICE) != USB_MODE_CTRL_MODE_DEVICE) {
		if (time_after(jiffies, timeout)) {
			ERR("udc device mode setup timeout!\n");
			return -ETIMEDOUT;
		}
		cpu_relax();
	}

	/* Clear the setup status */
	udc_writel(udc, 0, USB_STS_REG_OFFSET);

	tmp = udc->ep_qh_dma;
	tmp &= USB_EP_LIST_ADDRESS_MASK;
	udc_writel(udc, tmp, USB_EP_LIST_ADDRESS_REG_OFFSET);

	VDBG("vir[qh_base] is %p phy[qh_base] is 0x%8x reg is 0x%8x",
		udc->ep_qh, (int)tmp,
		udc_readl(udc, USB_EP_LIST_ADDRESS_REG_OFFSET));

	DBG("%s(%d) END\n", __func__, __LINE__);
	return 0;
}

/* Enable DR irq and set controller to run state */
static void dr_controller_run(struct tegra_udc *udc)
{
	u32 temp;
	unsigned long timeout;
	DBG("%s(%d) BEGIN\n", __func__, __LINE__);

	/* Clear stopped bit */
	udc->stopped = 0;

	/* If OTG transceiver is available, then it handles the VBUS detection*/
	if (!udc->transceiver) {
		/* Enable cable detection interrupt, without setting the
		 * USB_SYS_VBUS_WAKEUP_INT bit. USB_SYS_VBUS_WAKEUP_INT is
		 * clear on write */
		temp = udc_readl(udc, VBUS_WAKEUP_REG_OFFSET);
		temp |= (USB_SYS_VBUS_WAKEUP_INT_ENABLE
				| USB_SYS_VBUS_WAKEUP_ENABLE);
		temp &= ~USB_SYS_VBUS_WAKEUP_INT_STATUS;
		udc_writel(udc, temp, VBUS_WAKEUP_REG_OFFSET);
	}
	/* Enable DR irq reg */
	temp = USB_INTR_INT_EN | USB_INTR_ERR_INT_EN
		| USB_INTR_PTC_DETECT_EN | USB_INTR_RESET_EN
		| USB_INTR_DEVICE_SUSPEND | USB_INTR_SYS_ERR_EN;

	udc_writel(udc, temp, USB_INTR_REG_OFFSET);

	/* Set the controller as device mode */
	temp = udc_readl(udc, USB_MODE_REG_OFFSET);
	temp |= USB_MODE_CTRL_MODE_DEVICE;
	udc_writel(udc, temp, USB_MODE_REG_OFFSET);

	/* Set controller to Run */
	temp = udc_readl(udc, USB_CMD_REG_OFFSET);
	if (can_pullup(udc))
		temp |= USB_CMD_RUN_STOP;
	else
		temp &= ~USB_CMD_RUN_STOP;
	udc_writel(udc, temp, USB_CMD_REG_OFFSET);

	if (can_pullup(udc)) {
		/* Wait for controller to start */
		timeout = jiffies + UDC_RUN_TIMEOUT_MS;
		while ((udc_readl(udc, USB_CMD_REG_OFFSET) & USB_CMD_RUN_STOP)
			!= USB_CMD_RUN_STOP) {
			if (time_after(jiffies, timeout)) {
				ERR("udc start timeout!\n");
				return;
			}
			cpu_relax();
		}
	}

	DBG("%s(%d) END\n", __func__, __LINE__);
	return;
}

static void dr_controller_stop(struct tegra_udc *udc)
{
	unsigned int tmp;
	DBG("%s(%d) BEGIN\n", __func__, __LINE__);

	/* Clear pending interrupt status bits */
	tmp = udc_readl(udc, USB_STS_REG_OFFSET);
	udc_writel(udc, tmp, USB_STS_REG_OFFSET);

	/* disable all INTR */
	udc_writel(udc, 0, USB_INTR_REG_OFFSET);

	/* Set stopped bit for isr */
	udc->stopped = 1;

	/* set controller to Stop */
	tmp = udc_readl(udc, USB_CMD_REG_OFFSET);
	tmp &= ~USB_CMD_RUN_STOP;
	udc_writel(udc, tmp, USB_CMD_REG_OFFSET);

	DBG("%s(%d) END\n", __func__, __LINE__);
	return;
}

static void dr_ep_setup(struct tegra_udc *udc, unsigned char ep_num,
			unsigned char dir, unsigned char ep_type)
{
	unsigned int tmp_epctrl = 0;

	tmp_epctrl = udc_readl(udc, EP_CONTROL_REG_OFFSET + (ep_num * 4));
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

	udc_writel(udc, tmp_epctrl, EP_CONTROL_REG_OFFSET + (ep_num * 4));
}

static void dr_ep_change_stall(struct tegra_udc *udc, unsigned char ep_num,
			unsigned char dir, int value)
{
	u32 tmp_epctrl = 0;

	tmp_epctrl = udc_readl(udc, EP_CONTROL_REG_OFFSET + (ep_num * 4));
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
	udc_writel(udc, tmp_epctrl, EP_CONTROL_REG_OFFSET + (ep_num * 4));
}

/* Get stall status of a specific ep
   Return: 0: not stalled; 1:stalled */
static int dr_ep_get_stall(struct tegra_udc *udc, unsigned char ep_num,
		unsigned char dir)
{
	u32 epctrl;

	epctrl = udc_readl(udc, EP_CONTROL_REG_OFFSET + (ep_num * 4));
	if (dir)
		return (epctrl & EPCTRL_TX_EP_STALL) ? 1 : 0;
	else
		return (epctrl & EPCTRL_RX_EP_STALL) ? 1 : 0;
}

/**
 * struct_ep_qh_setup(): set the Endpoint Capabilites field of QH
 * @zlt: Zero Length Termination Select (1: disable; 0: enable)
 * @mult: Mult field
 */
static void struct_ep_qh_setup(struct tegra_udc *udc, unsigned char ep_num,
		unsigned char dir, unsigned char ep_type,
		unsigned int max_pkt_len, unsigned int zlt, unsigned char mult)
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

	return;
}

/* Setup qh structure and ep register for ep0. */
static void ep0_setup(struct tegra_udc *udc)
{
	/* the intialization of an ep includes: fields in QH, Regs,
	 * tegra_ep struct */
	struct_ep_qh_setup(udc, 0, USB_RECV, USB_ENDPOINT_XFER_CONTROL,
			USB_MAX_CTRL_PAYLOAD, 1, 0);
	struct_ep_qh_setup(udc, 0, USB_SEND, USB_ENDPOINT_XFER_CONTROL,
			USB_MAX_CTRL_PAYLOAD, 1, 0);
	dr_ep_setup(udc, 0, USB_RECV, USB_ENDPOINT_XFER_CONTROL);
	dr_ep_setup(udc, 0, USB_SEND, USB_ENDPOINT_XFER_CONTROL);

	return;

}

/**
 * when configurations are set, or when interface settings change
 * for example the do_set_interface() in gadget layer,
 * the driver will enable or disable the relevant endpoints
 * ep0 doesn't use this routine. It is always enabled.
 */
static int tegra_ep_enable(struct usb_ep *_ep,
		const struct usb_endpoint_descriptor *desc)
{
	struct tegra_udc *udc = NULL;
	struct tegra_ep *ep = NULL;
	unsigned short max = 0;
	unsigned char mult = 0, zlt;
	int retval = -EINVAL;
	unsigned long flags = 0;

	ep = container_of(_ep, struct tegra_ep, ep);

	/* catch various bogus parameters */
	if (!_ep || !desc)
		return -EINVAL;

	udc = ep->udc;

	if (!udc->driver || (udc->gadget.speed == USB_SPEED_UNKNOWN))
		return -ESHUTDOWN;

	max = le16_to_cpu(desc->wMaxPacketSize);

	/* Disable automatic zlp generation.  Driver is responsible to indicate
	 * explicitly through req->req.zero.  This is needed to enable multi-td
	 * request.
	 */
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
	ep->last_td = 0;
	ep->last_dtd_count = 0;

	/* Controller related setup
	 * Init EPx Queue Head (Ep Capabilites field in QH
	 * according to max, zlt, mult)
	 */
	struct_ep_qh_setup(udc, (unsigned char) ep_index(ep),
			(unsigned char) ((desc->bEndpointAddress & USB_DIR_IN)
					?  USB_SEND : USB_RECV),
			(unsigned char) (desc->bmAttributes
					& USB_ENDPOINT_XFERTYPE_MASK),
			max, zlt, mult);

	/* Init endpoint ctrl register */
	dr_ep_setup(udc, (unsigned char) ep_index(ep),
			(unsigned char) ((desc->bEndpointAddress & USB_DIR_IN)
					? USB_SEND : USB_RECV),
			(unsigned char) (desc->bmAttributes
					& USB_ENDPOINT_XFERTYPE_MASK));

	spin_unlock_irqrestore(&udc->lock, flags);
	retval = 0;

	VDBG("enabled %s (ep%d%s) maxpacket %d", ep->ep.name,
			ep->desc->bEndpointAddress & 0x0f,
			(desc->bEndpointAddress & USB_DIR_IN)
				? "in" : "out", max);
en_done:
	return retval;
}

/**
 * @ep : the ep being unconfigured. May not be ep0
 * Any pending and uncomplete req will complete with status (-ESHUTDOWN)
 */
static int tegra_ep_disable(struct usb_ep *_ep)
{
	struct tegra_udc *udc = NULL;
	struct tegra_ep *ep = NULL;

	unsigned long flags = 0;
	u32 epctrl;
	int ep_num;
	struct ep_td_struct *curr_td, *next_td;
	int j;

	ep = container_of(_ep, struct tegra_ep, ep);
	if (!_ep || !ep->desc) {
		VDBG("%s not enabled", _ep ? ep->ep.name : NULL);
		return -EINVAL;
	}
	udc = (struct tegra_udc *)ep->udc;

	/* disable ep on controller */
	ep_num = ep_index(ep);

	/* Touch the registers if cable is connected and phy is on */
	if (vbus_enabled(udc)) {
		epctrl = udc_readl(udc, EP_CONTROL_REG_OFFSET + (ep_num * 4));
		if (ep_is_in(ep))
			epctrl &= ~EPCTRL_TX_ENABLE;
		else
			epctrl &= ~EPCTRL_RX_ENABLE;
		udc_writel(udc, epctrl, EP_CONTROL_REG_OFFSET + (ep_num * 4));
	}

	spin_lock_irqsave(&udc->lock, flags);

	/* nuke all pending requests (does flush) */
	nuke(ep, -ESHUTDOWN);

	ep->desc = NULL;
	ep->stopped = 1;
	if (ep->last_td) {
		next_td = ep->last_td;
		for (j = 0; j < ep->last_dtd_count; j++) {
			curr_td = next_td;
			if (j != ep->last_dtd_count) {
				next_td = curr_td->next_td_virt;
			}
			dma_pool_free(udc->td_pool, curr_td, curr_td->td_dma);
		}
	}
	ep->last_td = 0;
	ep->last_dtd_count = 0;
	spin_unlock_irqrestore(&udc->lock, flags);

	VDBG("disabled %s OK", _ep->name);
	return 0;
}

/**
 * Allocate a request object used by this endpoint
 * the main operation is to insert the req->queue to the eq->queue
 * Returns the request, or null if one could not be allocated
 */
static struct usb_request *
tegra_alloc_request(struct usb_ep *_ep, gfp_t gfp_flags)
{
	struct tegra_req *req = NULL;

	req = kzalloc(sizeof *req, gfp_flags);
	if (!req)
		return NULL;

	req->req.dma = DMA_ADDR_INVALID;
	INIT_LIST_HEAD(&req->queue);

	return &req->req;
}

static void tegra_free_request(struct usb_ep *_ep, struct usb_request *_req)
{
	struct tegra_req *req = NULL;

	req = container_of(_req, struct tegra_req, req);

	if (_req)
		kfree(req);
}

static void tegra_queue_td(struct tegra_ep *ep, struct tegra_req *req)
{
	int i = ep_index(ep) * 2 + ep_is_in(ep);
	u32 temp, bitmask, tmp_stat;
	struct ep_queue_head *dQH = &ep->udc->ep_qh[i];
	struct tegra_udc *udc = ep->udc;

	bitmask = ep_is_in(ep)
		? (1 << (ep_index(ep) + 16))
		: (1 << (ep_index(ep)));

	/* Flush all the dTD structs out to memory */
	wmb();

	/* check if the pipe is empty */
	if (!(list_empty(&ep->queue))) {
		/* Add td to the end */
		struct tegra_req *lastreq;
		lastreq = list_entry(ep->queue.prev, struct tegra_req, queue);
		lastreq->tail->next_td_ptr =
			cpu_to_le32(req->head->td_dma & DTD_ADDR_MASK);
		wmb();
		/* Read prime bit, if 1 goto done */
		if (udc_readl(udc, EP_PRIME_REG_OFFSET) & bitmask)
			goto out;

		do {
			/* Set ATDTW bit in USBCMD */
			temp = udc_readl(udc, USB_CMD_REG_OFFSET);
			temp |= USB_CMD_ATDTW;
			udc_writel(udc, temp, USB_CMD_REG_OFFSET);

			/* Read correct status bit */
			tmp_stat = udc_readl(udc, EP_STATUS_REG_OFFSET)
						& bitmask;

		} while (!(udc_readl(udc, USB_CMD_REG_OFFSET) & USB_CMD_ATDTW));

		/* Write ATDTW bit to 0 */
		temp = udc_readl(udc, USB_CMD_REG_OFFSET);
		udc_writel(udc, temp & ~USB_CMD_ATDTW, USB_CMD_REG_OFFSET);

		if (tmp_stat)
			goto out;
	}

	/* Write dQH next pointer and terminate bit to 0 */
	temp = req->head->td_dma & EP_QUEUE_HEAD_NEXT_POINTER_MASK;
	dQH->next_dtd_ptr = cpu_to_le32(temp);

	/* Clear active and halt bit */
	temp = cpu_to_le32(~(EP_QUEUE_HEAD_STATUS_ACTIVE
			| EP_QUEUE_HEAD_STATUS_HALT));
	dQH->size_ioc_int_sts &= temp;

	tegra_usb_phy_memory_prefetch_on(udc->phy);

	/* Ensure that updates to the QH will occur before priming. */
	wmb();

	/* Prime endpoint by writing 1 to ENDPTPRIME */
	temp = ep_is_in(ep)
		? (1 << (ep_index(ep) + 16))
		: (1 << (ep_index(ep)));
	udc_writel(udc, temp, EP_PRIME_REG_OFFSET);
out:
	return;
}

/**
 * Fill in the dTD structure
 * @req     : request that the transfer belongs to
 * @length  : return actually data length of the dTD
 * @dma     : return dma address of the dTD
 * @is_last : return flag if it is the last dTD of the request
 * return   : pointer to the built dTD
 */
static struct ep_td_struct *tegra_build_dtd(struct tegra_req *req,
	unsigned *length, dma_addr_t *dma, int *is_last, gfp_t gfp_flags)
{
	u32 swap_temp;
	struct ep_td_struct *dtd;

	/* how big will this transfer be? */
	*length = min(req->req.length - req->req.actual,
			(unsigned)EP_MAX_LENGTH_TRANSFER);

	dtd = dma_pool_alloc(the_udc->td_pool, gfp_flags, dma);
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
static int tegra_req_to_dtd(struct tegra_req *req, gfp_t gfp_flags)
{
	unsigned	count;
	int		is_last;
	int		is_first = 1;
	struct ep_td_struct	*last_dtd = NULL, *dtd;
	dma_addr_t dma;

	tegra_usb_phy_memory_prefetch_off(the_udc->phy);

	do {
		dtd = tegra_build_dtd(req, &count, &dma, &is_last, gfp_flags);
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
tegra_ep_queue(struct usb_ep *_ep, struct usb_request *_req, gfp_t gfp_flags)
{
	struct tegra_ep *ep = container_of(_ep, struct tegra_ep, ep);
	struct tegra_req *req = container_of(_req, struct tegra_req, req);
	struct tegra_udc *udc = ep->udc;
	unsigned long flags;
	enum dma_data_direction dir;
	int status;

	if (udc->usb_state == USB_STATE_SUSPENDED)
		return -ESHUTDOWN;

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
	}

#ifdef CONFIG_TEGRA_GADGET_BOOST_CPU_FREQ
	if (req->req.length >= BOOST_TRIGGER_SIZE) {
		ep_queue_request_count++;
		if (ep_queue_request_count && boost_cpufreq_work_flag)
			schedule_work(&udc->boost_cpufreq_work);
	}
#endif

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
	status = tegra_req_to_dtd(req, gfp_flags);
	if (status)
		goto err_unmap;

	spin_lock_irqsave(&udc->lock, flags);

	/* re-check if the ep has not been disabled */
	if (unlikely(!ep->desc)) {
		spin_unlock_irqrestore(&udc->lock, flags);
		status = -EINVAL;
		goto err_unmap;
	}

	tegra_queue_td(ep, req);

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
static int tegra_ep_dequeue(struct usb_ep *_ep, struct usb_request *_req)
{
	struct tegra_ep *ep = container_of(_ep, struct tegra_ep, ep);
	struct tegra_req *req = container_of(_req, struct tegra_req, req);
	struct tegra_udc *udc = ep->udc;
	unsigned long flags;
	int ep_num, ep_in, stopped, ret = 0;
	u32 epctrl;

	if (_ep == NULL || _req == NULL || ep->desc == NULL
			|| list_empty(&req->queue) || !udc) {
		return -EINVAL;
	}
	spin_lock_irqsave(&ep->udc->lock, flags);
	stopped = ep->stopped;

	/* Stop the ep before we deal with the queue */
	ep->stopped = 1;
	ep_num = ep_index(ep);
	ep_in = ep_is_in(ep);

	/* Touch the registers if cable is connected and phy is on */
	if (vbus_enabled(udc)) {
		epctrl = udc_readl(udc, EP_CONTROL_REG_OFFSET + (ep_num * 4));
		if (ep_in)
			epctrl &= ~EPCTRL_TX_ENABLE;
		else
			epctrl &= ~EPCTRL_RX_ENABLE;
		udc_writel(udc, epctrl, EP_CONTROL_REG_OFFSET + (ep_num * 4));
	}

	/* After compared with other driver coding,
	 * cannot see any benefit to check the request
	 */
#if 0
	/* make sure it's actually queued on this endpoint */
	list_for_each_entry(req, &ep->queue, queue) {
		if (&req->req == _req)
			break;
	}
	if (&req->req != _req) {
		ret = -EINVAL;
		goto out;
	}
#endif

	/* The request is in progress, or completed but not dequeued */
	if (ep->queue.next == &req->queue) {
		_req->status = -ECONNRESET;
		tegra_ep_fifo_flush(_ep);	/* flush current transfer */

		/* The request isn't the last request in this ep queue */
		if (req->queue.next != &ep->queue) {
			struct ep_queue_head *qh;
			struct tegra_req *next_req;

			qh = ep->qh;
			next_req = list_entry(req->queue.next, struct tegra_req,
					queue);

			/* Point the QH to the first TD of next request */
			writel((u32) next_req->head, &qh->curr_dtd_ptr);
		}

		/* The request hasn't been processed, patch up the TD chain */
	} else {
		struct tegra_req *prev_req;

		prev_req = list_entry(req->queue.prev, struct tegra_req, queue);
		writel(readl(&req->tail->next_td_ptr),
				&prev_req->tail->next_td_ptr);
	}

	done(ep, req, -ECONNRESET);

	/* Enable EP */
	/* Touch the registers if cable is connected and phy is on */
	if (vbus_enabled(udc)) {
		epctrl = udc_readl(udc, EP_CONTROL_REG_OFFSET + (ep_num * 4));
		if (ep_in)
			epctrl |= EPCTRL_TX_ENABLE;
		else
			epctrl |= EPCTRL_RX_ENABLE;
		udc_writel(udc, epctrl, EP_CONTROL_REG_OFFSET + (ep_num * 4));
	}
	ep->stopped = stopped;
	spin_unlock_irqrestore(&ep->udc->lock, flags);
	return ret;
}

/**
 * modify the endpoint halt feature
 * @ep: the non-isochronous endpoint being stalled
 * @value: 1--set halt  0--clear halt
 * Returns zero, or a negative error code.
 */
static int tegra_ep_set_halt(struct usb_ep *_ep, int value)
{
	struct tegra_ep *ep = NULL;
	unsigned long flags = 0;
	int status = -EOPNOTSUPP;	/* operation not supported */
	unsigned char ep_dir = 0, ep_num = 0;
	struct tegra_udc *udc = NULL;

	ep = container_of(_ep, struct tegra_ep, ep);
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
	dr_ep_change_stall(udc, ep_num, ep_dir, value);
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

static int tegra_ep_fifo_status(struct usb_ep *_ep)
{
	struct tegra_ep *ep;
	struct tegra_udc *udc;
	int size = 0;
	u32 bitmask;
	struct ep_queue_head *d_qh;

	ep = container_of(_ep, struct tegra_ep, ep);
	if (!_ep || (!ep->desc && ep_index(ep) != 0))
		return -ENODEV;

	udc = (struct tegra_udc *)ep->udc;

	if (!udc->driver || udc->gadget.speed == USB_SPEED_UNKNOWN)
		return -ESHUTDOWN;

	d_qh = &ep->udc->ep_qh[ep_index(ep) * 2 + ep_is_in(ep)];

	bitmask = (ep_is_in(ep)) ? (1 << (ep_index(ep) + 16)) :
	    (1 << (ep_index(ep)));

	if (udc_readl(udc, EP_STATUS_REG_OFFSET) & bitmask)
		size = (d_qh->size_ioc_int_sts & DTD_PACKET_SIZE)
		    >> DTD_LENGTH_BIT_POS;

	pr_debug("%s %u\n", __func__, size);
	return size;
}

static void tegra_ep_fifo_flush(struct usb_ep *_ep)
{
	struct tegra_ep *ep;
	struct tegra_udc *udc;
	int ep_num, ep_dir;
	u32 bits;
	unsigned long timeout;

	if (!_ep) {
		return;
	} else {
		ep = container_of(_ep, struct tegra_ep, ep);
		if (!ep->desc)
			return;
	}
	ep_num = ep_index(ep);
	ep_dir = ep_is_in(ep) ? USB_SEND : USB_RECV;
	udc = ep->udc;

	if (ep_num == 0)
		bits = (1 << 16) | 1;
	else if (ep_dir == USB_SEND)
		bits = 1 << (16 + ep_num);
	else
		bits = 1 << ep_num;

	/* Touch the registers if cable is connected and phy is on */
	if (!vbus_enabled(udc))
		return;

	timeout = jiffies + UDC_FLUSH_TIMEOUT_MS;
	do {
		udc_writel(udc, bits, EPFLUSH_REG_OFFSET);

		/* Wait until flush complete */
		while (udc_readl(udc, EPFLUSH_REG_OFFSET)) {
			if (time_after(jiffies, timeout)) {
				ERR("ep flush timeout\n");
				return;
			}
			cpu_relax();
		}
		/* See if we need to flush again */
	} while (udc_readl(udc, EP_STATUS_REG_OFFSET) & bits);
}

static struct usb_ep_ops tegra_ep_ops = {
	.enable = tegra_ep_enable,
	.disable = tegra_ep_disable,

	.alloc_request = tegra_alloc_request,
	.free_request = tegra_free_request,

	.queue = tegra_ep_queue,
	.dequeue = tegra_ep_dequeue,

	.set_halt = tegra_ep_set_halt,
	.fifo_status = tegra_ep_fifo_status,
	.fifo_flush = tegra_ep_fifo_flush,	/* flush fifo */
};

/* Get the current frame number (from DR frame_index Reg ) */
static int tegra_get_frame(struct usb_gadget *gadget)
{
	struct tegra_udc *udc = container_of(gadget, struct tegra_udc, gadget);
	return (int)(udc_readl(udc, USB_FRINDEX_REG_OFFSET)
						& USB_FRINDEX_MASKS);
}

#ifndef CONFIG_USB_ANDROID
/* Tries to wake up the host connected to this gadget */
static int tegra_wakeup(struct usb_gadget *gadget)
{
	struct tegra_udc *udc = container_of(gadget, struct tegra_udc, gadget);
	u32 portsc;

	/* Remote wakeup feature not enabled by host */
	if (!udc->remote_wakeup)
		return -ENOTSUPP;

	portsc = udc_readl(udc, PORTSCX_REG_OFFSET);
	/* not suspended? */
	if (!(portsc & PORTSCX_PORT_SUSPEND))
		return 0;

	/* trigger force resume */
	portsc |= PORTSCX_PORT_FORCE_RESUME;
	udc_writel(udc, portsc, PORTSCX_REG_OFFSET);
	return 0;
}
#endif

static int tegra_set_selfpowered(struct usb_gadget *gadget, int is_on)
{
	struct tegra_udc *udc;
	udc = container_of(gadget, struct tegra_udc, gadget);
	udc->selfpowered = (is_on != 0);
	return 0;
}

/**
 * Notify controller that VBUS is powered, called by whatever
 * detects VBUS sessions
 */
static int tegra_vbus_session(struct usb_gadget *gadget, int is_active)
{
	struct tegra_udc *udc = container_of(gadget, struct tegra_udc, gadget);
	unsigned long flags;
	USB_INFO("%s(%d) turn VBUS state from %s to %s", __func__, __LINE__,
		udc->vbus_active ? "on" : "off", is_active ? "on" : "off");
	wake_lock_timeout(&udc_wake_lock2, 1*HZ);

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
		spin_unlock_irqrestore(&udc->lock,flags);
		tegra_usb_phy_power_off(udc->phy);
		if (udc->vbus_reg) {
			/* set the current limit to 0mA */
			regulator_set_current_limit(
				udc->vbus_reg, 0, 0);
		}
		/* ++ htc ++*/
		udc->connect_type = CONNECT_TYPE_NONE;
		cancel_delayed_work_sync(&udc->ac_detect_work);
		queue_work(udc->usb_wq, &udc->notifier_work);
		vbus_active_USB_disabled = 0;
		/* -- htc --*/
	} else if ((!udc->vbus_active && is_active) ||
			(udc->vbus_active && is_active && vbus_active_USB_disabled)) {
		if (!USB_disabled) {
			tegra_usb_phy_power_on(udc->phy);
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
			vbus_active_USB_disabled = 0;
		} else {
			USB_INFO("%s (%d) USB_disable", __func__, is_active);
			vbus_active_USB_disabled = 1;
			tegra_usb_phy_power_on(udc->phy);
			udc->vbus_active = 1;
			udc->usb_state = USB_STATE_DEFAULT;
		}
		if (udc->vbus_reg) {
			/* set the current limit to 100mA */
			regulator_set_current_limit(
				udc->vbus_reg, 0, 100);
		}
		udc->ac_detect_count = 0;
		/* Schedule work to wait for 1000 msec and check for
		 * charger if setup packet is not received */
		schedule_delayed_work(&udc->work,
			USB_CHARGER_DETECTION_WAIT_TIME_MS);
		charger_detect(udc);/* htc */
	}
	return 0;
}

/**
 * Constrain controller's VBUS power usage.
 * This call is used by gadget drivers during SET_CONFIGURATION calls,
 * reporting how much power the device may consume. For example, this
 * could affect how quickly batteries are recharged.
 *
 * Returns zero on success, else negative errno.
 */
static int tegra_vbus_draw(struct usb_gadget *gadget, unsigned mA)
{
	struct tegra_udc *udc;

	udc = container_of(gadget, struct tegra_udc, gadget);
	/* check udc regulator is available for drawing the vbus current */
	if (udc->vbus_reg) {
		udc->current_limit = mA;
		schedule_work(&udc->charger_work);
	}

	if (udc->transceiver)
		return otg_set_power(udc->transceiver, mA);
	return -ENOTSUPP;
}

/**
 * Change Data+ pullup status
 * this func is used by usb_gadget_connect/disconnect
 */
static int tegra_pullup(struct usb_gadget *gadget, int is_on)
{
	struct tegra_udc *udc;
	u32 tmp;

	udc = container_of(gadget, struct tegra_udc, gadget);
	udc->softconnect = (is_on != 0);
	if (udc->transceiver && udc->transceiver->state !=
			OTG_STATE_B_PERIPHERAL)
			return 0;

	if (can_pullup(udc)) {
		/* Enable DR irq reg */
		tmp = USB_INTR_INT_EN | USB_INTR_ERR_INT_EN
			| USB_INTR_PTC_DETECT_EN | USB_INTR_RESET_EN
			| USB_INTR_DEVICE_SUSPEND | USB_INTR_SYS_ERR_EN;
		udc_writel(udc, tmp, USB_INTR_REG_OFFSET);

		/* set controller to Run  */
		tmp = udc_readl(udc, USB_CMD_REG_OFFSET);
		udc_writel(udc, tmp | USB_CMD_RUN_STOP,
				USB_CMD_REG_OFFSET);
	} else {
		/* set controller to Stop */
		tmp = udc_readl(udc, USB_CMD_REG_OFFSET);
		udc_writel(udc, (tmp & ~USB_CMD_RUN_STOP),
				USB_CMD_REG_OFFSET);

		/* disable all INTR  */
		udc_writel(udc, 0, USB_INTR_REG_OFFSET);
	}

	return 0;
}

/* Release udc structures */
static void tegra_udc_release(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct tegra_udc *udc = platform_get_drvdata(pdev);

	complete(udc->done);
	tegra_usb_phy_close(udc->phy);
	kfree(udc);
}

static int tegra_udc_start(struct usb_gadget_driver *driver,
		int (*bind)(struct usb_gadget *));
static int tegra_udc_stop(struct usb_gadget_driver *driver);
/* defined in gadget.h */
static struct usb_gadget_ops tegra_gadget_ops = {
	.get_frame = tegra_get_frame,
#ifndef CONFIG_USB_ANDROID
	.wakeup = tegra_wakeup,
#endif
	.set_selfpowered = tegra_set_selfpowered,
	.vbus_session = tegra_vbus_session,
	.vbus_draw = tegra_vbus_draw,
	.pullup = tegra_pullup,
	.start = tegra_udc_start,
	.stop = tegra_udc_stop,
};

static int tegra_udc_setup_gadget_dev(struct tegra_udc *udc)
{
	/* Setup gadget structure */
	udc->gadget.ops = &tegra_gadget_ops;
	udc->gadget.is_dualspeed = 1;
	udc->gadget.ep0 = &udc->eps[0].ep;
	INIT_LIST_HEAD(&udc->gadget.ep_list);
	udc->gadget.speed = USB_SPEED_UNKNOWN;
	udc->gadget.name = driver_name;

	/* Setup gadget.dev and register with kernel */
	dev_set_name(&udc->gadget.dev, "gadget");
	udc->gadget.dev.release = tegra_udc_release;
	udc->gadget.dev.parent = &udc->pdev->dev;

	return device_register(&udc->gadget.dev);
}


/**
 * Set protocol stall on ep0, protocol stall will automatically be cleared
 * on new transaction.
 */
static void ep0stall(struct tegra_udc *udc)
{
	u32 tmp;

	/* must set tx and rx to stall at the same time */
	tmp = udc_readl(udc, EP_CONTROL_REG_OFFSET);
	tmp |= EPCTRL_TX_EP_STALL | EPCTRL_RX_EP_STALL;
	udc_writel(udc, tmp, EP_CONTROL_REG_OFFSET);
	udc->ep0_state = WAIT_FOR_SETUP;
	udc->ep0_dir = 0;
}

/* Prime a status phase for ep0 */
static int ep0_prime_status(struct tegra_udc *udc, int direction)
{
	struct tegra_req *req = udc->status_req;
	struct tegra_ep *ep;

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

	if (tegra_req_to_dtd(req, GFP_ATOMIC) == 0)
		tegra_queue_td(ep, req);
	else
		return -ENOMEM;

	list_add_tail(&req->queue, &ep->queue);

	return 0;
}

static void udc_reset_ep_queue(struct tegra_udc *udc, u8 pipe)
{
	struct tegra_ep *ep = get_ep_by_pipe(udc, pipe);

	if (ep->name)
		nuke(ep, -ESHUTDOWN);
}

/* ch9 Set address */
static void ch9setaddress(struct tegra_udc *udc, u16 value, u16 index,
		u16 length)
{
	/* Save the new address to device struct */
	udc->device_address = (u8) value;
	/* Update usb state */
	udc->usb_state = USB_STATE_ADDRESS;
	/* Status phase */
	if (ep0_prime_status(udc, EP_DIR_IN))
		ep0stall(udc);
}

/* ch9 Get status */
static void ch9getstatus(struct tegra_udc *udc, u8 request_type, u16 value,
		u16 index, u16 length)
{
	u16 tmp = 0;		/* Status, cpu endian */
	struct tegra_req *req;
	struct tegra_ep *ep;

	ep = &udc->eps[0];

	if ((request_type & USB_RECIP_MASK) == USB_RECIP_DEVICE) {
		/* Get device status */
		if (udc->selfpowered)
			tmp = 1 << USB_DEVICE_SELF_POWERED;
		tmp |= udc->remote_wakeup << USB_DEVICE_REMOTE_WAKEUP;
	} else if ((request_type & USB_RECIP_MASK) == USB_RECIP_INTERFACE) {
		/* Get interface status
		 * We don't have interface information in udc driver */
		tmp = 0;
	} else if ((request_type & USB_RECIP_MASK) == USB_RECIP_ENDPOINT) {
		/* Get endpoint status */
		struct tegra_ep *target_ep;

		target_ep = get_ep_by_pipe(udc, get_pipe_by_windex(index));

		/* stall if endpoint doesn't exist */
		if (!target_ep->desc)
			goto stall;
		tmp = dr_ep_get_stall(udc, ep_index(target_ep),
				ep_is_in(target_ep)) << USB_ENDPOINT_HALT;
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
	if ((tegra_req_to_dtd(req, GFP_ATOMIC) == 0))
		tegra_queue_td(ep, req);
	else			/* no mem */
		goto stall;

	list_add_tail(&req->queue, &ep->queue);
	udc->ep0_state = DATA_STATE_XMIT;
	return;
stall:
	ep0stall(udc);
}
struct delayed_work disable_charger_hizmode_work;
static void disable_charger_hizmode(struct work_struct *data)
{
	USB_INFO(" %s\n", __func__);
	htc_battery_charger_hiz();
}
static void udc_test_mode(struct tegra_udc *udc, u32 test_mode)
{
	struct tegra_req *req;
	struct tegra_ep *ep;
	u32 portsc, bitmask;
	unsigned long timeout;
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
	while (!(udc_readl(udc, EP_COMPLETE_REG_OFFSET) & bitmask)) {
		if (time_after(jiffies, timeout)) {
			pr_err("Timeout for Ep0 IN Ack\n");
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
		val = udc_readl(udc, UTMIP_HSRX_CFG1);
		val &= ~UTMIP_HS_SYNC_START_DLY(~0);
		val |= UTMIP_HS_SYNC_START_DLY(0x2);
		udc_writel(udc, val, UTMIP_HSRX_CFG1);
		VDBG("TEST_SE0_NAK\n");
		break;
	case PORTSCX_PTC_PACKET:
		VDBG("TEST_PACKET\n");

		/* get the ep and configure for IN direction */
		ep = &udc->eps[0];
		udc->ep0_dir = USB_DIR_IN;

		/* Initialize ep0 status request structure */
		req = container_of(tegra_alloc_request(NULL, GFP_ATOMIC),
				struct tegra_req, req);
		/* allocate a small amount of memory to get valid address */
		req->req.buf = kmalloc(sizeof(tegra_udc_test_packet),
					GFP_ATOMIC);
		if (req->req.buf == NULL) {
			ERR("malloc req->req.buf failed\n");
			goto stall;
		}
		req->req.dma = virt_to_phys(req->req.buf);

		/* Fill in the reqest structure */
		memcpy(req->req.buf, tegra_udc_test_packet,
					sizeof(tegra_udc_test_packet));
		req->ep = ep;
		req->req.length = sizeof(tegra_udc_test_packet);
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
		if ((tegra_req_to_dtd(req, GFP_ATOMIC) == 0))
			tegra_queue_td(ep, req);
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
	portsc = udc_readl(udc, PORTSCX_REG_OFFSET);
	/* set the test mode selector */
	portsc |= test_mode << PORTSCX_PTC_BIT_POS;
	udc_writel(udc, portsc, PORTSCX_REG_OFFSET);

	/*
	 * The device must have its power cycled to exit test mode.
	 * See USB 2.0 spec, section 9.4.9 for test modes operation
	 * in "Set Feature".
	 * See USB 2.0 spec, section 7.1.20 for test modes.
	 */
	USB_INFO("udc entering the test mode, power cycle to exit test mode\n");
	return;
stall:
	ep0stall(udc);
}

static void setup_received_irq(struct tegra_udc *udc,
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
		/* This delay is necessary for some windows drivers to
		 * properly recognize the device */
		mdelay(1);
		ch9setaddress(udc, wValue, wIndex, wLength);
		return;

	case USB_REQ_CLEAR_FEATURE:
	case USB_REQ_SET_FEATURE:
		/* Status phase from udc */
	{
		int rc = -EOPNOTSUPP;

		if (setup->bRequestType == USB_RECIP_DEVICE &&
				 wValue == USB_DEVICE_TEST_MODE) {

			/* disable battery charger in test mode */
			schedule_delayed_work(&disable_charger_hizmode_work, 0);

			/*
			 * If the feature selector is TEST_MODE, then the most
			 * significant byte of wIndex is used to specify the
			 * specific test mode and the lower byte of wIndex must
			 * be zero.
			 */
			udc_test_mode(udc, wIndex >> 8);
			return;

		} else if ((setup->bRequestType &
				(USB_RECIP_MASK | USB_TYPE_MASK)) ==
				(USB_RECIP_ENDPOINT | USB_TYPE_STANDARD)) {
			int pipe = get_pipe_by_windex(wIndex);
			struct tegra_ep *ep;

			if (wValue != 0 || wLength != 0 || pipe > udc->max_ep)
				break;
			ep = get_ep_by_pipe(udc, pipe);

			spin_unlock(&udc->lock);
			rc = tegra_ep_set_halt(&ep->ep,
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
static void ep0_req_complete(struct tegra_udc *udc, struct tegra_ep *ep0,
		struct tegra_req *req)
{
	if (udc->usb_state == USB_STATE_ADDRESS) {
		/* Set the new address */
		u32 new_address = (u32) udc->device_address;
		udc_writel(udc, new_address << USB_DEVICE_ADDRESS_BIT_POS,
				USB_DEVICE_ADDR_REG_OFFSET);
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
static void tripwire_handler(struct tegra_udc *udc, u8 ep_num, u8 *buffer_ptr)
{
	u32 temp;
	struct ep_queue_head *qh;

	qh = &udc->ep_qh[ep_num * 2 + EP_DIR_OUT];

	/* Clear bit in ENDPTSETUPSTAT */
	temp = udc_readl(udc, EP_SETUP_STATUS_REG_OFFSET);
	udc_writel(udc, temp | (1 << ep_num), EP_SETUP_STATUS_REG_OFFSET);

	/* while a hazard exists when setup package arrives */
	do {
		/* Set Setup Tripwire */
		temp = udc_readl(udc, USB_CMD_REG_OFFSET);
		udc_writel(udc, temp | USB_CMD_SUTW, USB_CMD_REG_OFFSET);

		/* Copy the setup packet to local buffer */
		memcpy(buffer_ptr, (u8 *) qh->setup_buffer, 8);
	} while (!(udc_readl(udc, USB_CMD_REG_OFFSET) & USB_CMD_SUTW));

	/* Clear Setup Tripwire */
	temp = udc_readl(udc, USB_CMD_REG_OFFSET);
	udc_writel(udc, temp & ~USB_CMD_SUTW, USB_CMD_REG_OFFSET);
}

/* process-ep_req(): free the completed Tds for this req */
static int process_ep_req(struct tegra_udc *udc, int pipe,
		struct tegra_req *curr_req)
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
		/* Fence read for coherency of AHB master intiated writes */
		readb(IO_ADDRESS(IO_PPCS_PHYS + USB1_PREFETCH_ID));

		dma_sync_single_for_cpu(udc->gadget.dev.parent, curr_td->td_dma,
				sizeof(struct ep_td_struct), DMA_FROM_DEVICE);

		remaining_length = (le32_to_cpu(curr_td->size_ioc_sts)
					& DTD_PACKET_SIZE)
				>> DTD_LENGTH_BIT_POS;
		actual -= remaining_length;
		errors = le32_to_cpu(curr_td->size_ioc_sts);
		if (errors & DTD_ERROR_MASK) {
			if (errors & DTD_STATUS_HALTED) {
				/* ERR("dTD error %08x QH=%d\n", errors, pipe); */
				USB_INFO("dTD error %08x QH=%d\n", errors, pipe);
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
				ERR("Unknown error has occurred (0x%x)!\n",
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
static void dtd_complete_irq(struct tegra_udc *udc)
{
	u32 bit_pos;
	int i, ep_num, direction, bit_mask, status;
	struct tegra_ep *curr_ep;
	struct tegra_req *curr_req, *temp_req;

	/* Clear the bits in the register */
	bit_pos = udc_readl(udc, EP_COMPLETE_REG_OFFSET);
	udc_writel(udc, bit_pos, EP_COMPLETE_REG_OFFSET);

	if (!bit_pos)
		return;

	for (i = 0; i < udc->max_ep; i++) {
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
static void port_change_irq(struct tegra_udc *udc)
{
	u32 speed;
	unsigned int port_control_reg_offset;

	if (udc->has_hostpc)
		port_control_reg_offset = USB_HOSTPCX_DEVLC_REG_OFFSET;
	else
		port_control_reg_offset = PORTSCX_REG_OFFSET;

	/* Bus resetting is finished */
	if (!(udc_readl(udc, port_control_reg_offset) & PORTSCX_PORT_RESET)) {
		/* Get the speed */
		speed = (udc_readl(udc, port_control_reg_offset)
				& PORTSCX_PORT_SPEED_MASK);
		if (speed == PORTSCX_PORT_SPEED_HIGH)
			udc->gadget.speed = USB_SPEED_HIGH;
		else if (speed == PORTSCX_PORT_SPEED_FULL)
			udc->gadget.speed = USB_SPEED_FULL;
		else if (speed == PORTSCX_PORT_SPEED_LOW)
			udc->gadget.speed = USB_SPEED_LOW;
		else
			udc->gadget.speed = USB_SPEED_UNKNOWN;
	}

	/* Update USB state */
	if (!udc->resume_state)
		udc->usb_state = USB_STATE_DEFAULT;
}

/* Process suspend interrupt */
static void suspend_irq(struct tegra_udc *udc)
{
	udc->resume_state = udc->usb_state;
	udc->usb_state = USB_STATE_SUSPENDED;

	/* report suspend to the driver, serial.c does not support this */
	if (udc->driver && udc->driver->suspend)
		udc->driver->suspend(&udc->gadget);
}

static void bus_resume(struct tegra_udc *udc)
{
	udc->usb_state = udc->resume_state;
	udc->resume_state = 0;

	/* report resume to the driver, serial.c does not support this */
	if (udc->driver && udc->driver->resume)
		udc->driver->resume(&udc->gadget);
}

/* Clear up all ep queues */
static int reset_queues(struct tegra_udc *udc)
{
	u8 pipe;

	for (pipe = 0; pipe < udc->max_pipes; pipe++)
		udc_reset_ep_queue(udc, pipe);

	/* report disconnect; the driver is already quiesced */
	spin_unlock(&udc->lock);
	if (udc->driver && udc->driver->disconnect)
		udc->driver->disconnect(&udc->gadget);
	spin_lock(&udc->lock);

	return 0;
}

/* ++ htc ++ */
/* Clear up all ep queues */
static int reset_queues_mute(struct tegra_udc *udc)
{
	u8 pipe;
	for (pipe = 0; pipe < udc->max_pipes; pipe++)
		udc_reset_ep_queue(udc, pipe);

	/* report disconnect; the driver is already quiesced */
	spin_unlock(&udc->lock);
	if (udc->driver && udc->driver->mute_disconnect)
		udc->driver->mute_disconnect(&udc->gadget);
	spin_lock(&udc->lock);

	return 0;
}
/* -- htc -- */

/* Process reset interrupt */
static void reset_irq(struct tegra_udc *udc)
{
	u32 temp;
	unsigned long timeout;

	/* Clear the device address */
	temp = udc_readl(udc, USB_DEVICE_ADDR_REG_OFFSET);
	udc_writel(udc, temp & ~USB_DEVICE_ADDRESS_MASK,
				USB_DEVICE_ADDR_REG_OFFSET);

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
	temp = udc_readl(udc, EP_SETUP_STATUS_REG_OFFSET);
	udc_writel(udc, temp, EP_SETUP_STATUS_REG_OFFSET);

	/* Clear all the endpoint complete status bits */
	temp = udc_readl(udc, EP_COMPLETE_REG_OFFSET);
	udc_writel(udc, temp, EP_COMPLETE_REG_OFFSET);

	timeout = jiffies + 100;
	while (udc_readl(udc, EP_PRIME_REG_OFFSET)) {
		/* Wait until all endptprime bits cleared */
		if (time_after(jiffies, timeout)) {
			ERR("Timeout for reset\n");
			break;
		}
		cpu_relax();
	}

	/* Write 1s to the flush register */
	udc_writel(udc, 0xffffffff, EPFLUSH_REG_OFFSET);

	/* When the bus reset is seen on Tegra, the PORTSCX_PORT_RESET bit
	 * is not set. Reset all the queues, include XD, dTD, EP queue
	 * head and TR Queue */
	VDBG("Bus reset");
	/* reset_queues(udc); */ /* htc */
	reset_queues_mute(udc); /* htc */
	udc->usb_state = USB_STATE_DEFAULT;
}

static void tegra_udc_set_current_limit_work(struct work_struct *work)
{
	struct tegra_udc *udc = container_of(work, struct tegra_udc,
						charger_work);
	/* check udc regulator is available for drawing vbus current*/
	if (udc->vbus_reg) {
		/* set the current limit in uA */
		regulator_set_current_limit(
			udc->vbus_reg, 0,
			udc->current_limit * 1000);
	}
}

#ifdef CONFIG_TEGRA_GADGET_BOOST_CPU_FREQ
static void tegra_udc_boost_cpu_frequency_work(struct work_struct *work)
{
	if (ep_queue_request_count && boost_cpufreq_work_flag) {
		pm_qos_update_request(&boost_cpu_freq_req,
			(s32)CONFIG_TEGRA_GADGET_BOOST_CPU_FREQ * 1000);
		boost_cpufreq_work_flag = 0;
	} else if (!ep_queue_request_count && !boost_cpufreq_work_flag) {
		pm_qos_update_request(&boost_cpu_freq_req,
			PM_QOS_DEFAULT_VALUE);
		boost_cpufreq_work_flag = 1;
	}
}
#endif

static void tegra_udc_irq_work(struct work_struct *irq_work)
{
	struct tegra_udc *udc = container_of(irq_work, struct tegra_udc,
						 irq_work);
	DBG("%s(%d) BEGIN\n", __func__, __LINE__);

	/* Check whether cable is connected*/
	if (vbus_enabled(udc))
		tegra_vbus_session(&udc->gadget, 1);
	else
		tegra_vbus_session(&udc->gadget, 0);

	DBG("%s(%d) END\n", __func__, __LINE__);
}

/**
 * If VBUS is detected and setup packet is not received in 100ms then
 * work thread starts and checks for the USB charger detection.
 */
static void tegra_udc_charger_detect_work(struct work_struct *work)
{
	struct tegra_udc *udc = container_of(work, struct tegra_udc, work.work);
	DBG("%s(%d) BEGIN\n", __func__, __LINE__);

	/* check for the platform charger detection */
	if (tegra_usb_phy_charger_detected(udc->phy)) {
		USB_INFO("USB compliant charger detected\n");
		/* check udc regulator is available for drawing vbus current*/
		if (udc->vbus_reg) {
			/* set the current limit in uA */
			regulator_set_current_limit(
				udc->vbus_reg, 0,
				USB_CHARGING_CURRENT_LIMIT_MA*1000);
		}
	}

	DBG("%s(%d) END\n", __func__, __LINE__);
}

/* Restart device controller in the OTG mode on VBUS detection */
static void tegra_udc_restart(struct tegra_udc *udc)
{
	DBG("%s(%d) BEGIN\n", __func__, __LINE__);

	/* setup the controller in the device mode */
	dr_controller_setup(udc);
	/* setup EP0 for setup packet */
	ep0_setup(udc);
	udc->vbus_active = 1;
	/* start the controller */
	dr_controller_run(udc);
	/* initialize the USB and EP states */
	udc->usb_state = USB_STATE_ATTACHED;
	udc->ep0_state = WAIT_FOR_SETUP;
	udc->ep0_dir = 0;

	DBG("%s(%d) END\n", __func__, __LINE__);
}

/* USB device controller interrupt handler */
static irqreturn_t tegra_udc_irq(int irq, void *_udc)
{
	struct tegra_udc *udc = _udc;
	u32 irq_src, temp;
	irqreturn_t status = IRQ_NONE;
	unsigned long flags;

	/* somehow we got an IRQ while in the reset sequence: ignore it */
	if (!udc->softconnect) {
		USB_INFO("Warning: USB controller issue interrupt during disconnect\n");
		irq_src = udc_readl(udc, USB_STS_REG_OFFSET) & udc_readl(udc, USB_INTR_REG_OFFSET);
		/* Clear notification bits */
		udc_writel(udc, irq_src, USB_STS_REG_OFFSET);
		return IRQ_HANDLED;
	}

	spin_lock_irqsave(&udc->lock, flags);

	if (!udc->transceiver) {
		temp = udc_readl(udc, VBUS_WAKEUP_REG_OFFSET);
		/* write back the register to clear the interrupt */
		udc_writel(udc, temp, VBUS_WAKEUP_REG_OFFSET);
		if (temp & USB_SYS_VBUS_WAKEUP_INT_STATUS)
			schedule_work(&udc->irq_work);
		status = IRQ_HANDLED;
	}

	/* Disable ISR for OTG host mode */
	if (udc->stopped) {
		spin_unlock_irqrestore(&udc->lock, flags);
		return status;
	}

	/* Fence read for coherency of AHB master intiated writes */
	readb(IO_ADDRESS(IO_PPCS_PHYS + USB1_PREFETCH_ID));

	irq_src = udc_readl(udc, USB_STS_REG_OFFSET) &
				udc_readl(udc, USB_INTR_REG_OFFSET);

	/* Clear notification bits */
	udc_writel(udc, irq_src, USB_STS_REG_OFFSET);

	/* Need to resume? */
	if (udc->usb_state == USB_STATE_SUSPENDED)
		if (!(udc_readl(udc, PORTSCX_REG_OFFSET)
				& PORTSCX_PORT_SUSPEND))
			bus_resume(udc);

	/* USB Interrupt */
	if (irq_src & USB_STS_INT) {
		VDBG("Packet int");
		/* Setup package, we only support ep0 as control ep */
		if (udc_readl(udc, EP_SETUP_STATUS_REG_OFFSET) &
				EP_SETUP_STATUS_EP0) {
			/* Setup packet received, we are connected to host
			 * and not to charger. Cancel any delayed work */
			__cancel_delayed_work(&udc->work);
			tripwire_handler(udc, 0,
					(u8 *) (&udc->local_setup_buff));
			setup_received_irq(udc, &udc->local_setup_buff);
			status = IRQ_HANDLED;
		}

		/* completion of dtd */
		if (udc_readl(udc, EP_COMPLETE_REG_OFFSET)) {
			dtd_complete_irq(udc);
			status = IRQ_HANDLED;
		}
	}

	/* SOF (for ISO transfer) */
	if (irq_src & USB_STS_SOF)
		status = IRQ_HANDLED;

	/* Port Change */
	if (irq_src & USB_STS_PORT_CHANGE) {
		port_change_irq(udc);
		status = IRQ_HANDLED;
	}

	/* Reset Received */
	if (irq_src & USB_STS_RESET) {

		/* ++ htc ++ */
		USB_INFO("reset\n"); /* PC confirmed */
		__cancel_delayed_work(&udc->ac_detect_work);
		udc->ac_detect_count = 0;
		/* ++ htc ++ */

		reset_irq(udc);
		status = IRQ_HANDLED;
	}

	/* Sleep Enable (Suspend) */
	if (irq_src & USB_STS_SUSPEND) {
		USB_INFO("suspend\n");
		suspend_irq(udc);
		status = IRQ_HANDLED;
	}

	if (irq_src & (USB_STS_ERR | USB_STS_SYS_ERR))
		VDBG("Error IRQ %x", irq_src);

	spin_unlock_irqrestore(&udc->lock, flags);
	return status;
}

/**
 * Hook to gadget drivers
 * Called by initialization code of gadget drivers
 */
static int tegra_udc_start(struct usb_gadget_driver *driver,
		int (*bind)(struct usb_gadget *))
{
	struct tegra_udc *udc = the_udc;
	int retval = -ENODEV;
	unsigned long flags = 0;
	DBG("%s(%d) BEGIN\n", __func__, __LINE__);

	if (!udc)
		return -ENODEV;

	if (!driver || (driver->speed != USB_SPEED_FULL
				&& driver->speed != USB_SPEED_HIGH)
			|| !bind || !driver->disconnect
			|| !driver->setup)
		return -EINVAL;

	if (udc->driver)
		return -EBUSY;

	/* lock is needed but whether should use this lock or another */
	spin_lock_irqsave(&udc->lock, flags);

	driver->driver.bus = NULL;
	/* hook up the driver */
	udc->driver = driver;
	udc->gadget.dev.driver = &driver->driver;
	spin_unlock_irqrestore(&udc->lock, flags);

	/* bind udc driver to gadget driver */
	retval = bind(&udc->gadget);
	if (retval) {
		VDBG("bind to %s --> %d", driver->driver.name, retval);
		udc->gadget.dev.driver = NULL;
		udc->driver = NULL;
		goto out;
	}


	/* Enable DR IRQ reg and Set usbcmd reg  Run bit */
	if (!udc->transceiver) {
		dr_controller_run(udc);
		udc->usb_state = USB_STATE_ATTACHED;
		udc->ep0_state = WAIT_FOR_SETUP;
		udc->ep0_dir = 0;
		udc->vbus_active = vbus_enabled(udc);
	}

	USB_INFO("%s: bind to driver %s\n",
			udc->gadget.name, driver->driver.name);

out:
	if (retval)
		USB_WARNING("gadget driver register failed %d\n",
								retval);

	DBG("%s(%d) END\n", __func__, __LINE__);
	return retval;
}

/* Disconnect from gadget driver */
static int tegra_udc_stop(struct usb_gadget_driver *driver)
{
	struct tegra_udc *udc = the_udc;
	struct tegra_ep *loop_ep;
	unsigned long flags;

	DBG("%s(%d) BEGIN\n", __func__, __LINE__);
	if (!udc)
		return -ENODEV;

	if (!driver || driver != udc->driver || !driver->unbind)
		return -EINVAL;

	/* stop DR, disable intr */
	dr_controller_stop(udc);

	/* in fact, no needed */
	udc->usb_state = USB_STATE_ATTACHED;
	udc->ep0_state = WAIT_FOR_SETUP;
	udc->ep0_dir = 0;

	/* stand operation */
	spin_lock_irqsave(&udc->lock, flags);
	udc->gadget.speed = USB_SPEED_UNKNOWN;
	nuke(&udc->eps[0], -ESHUTDOWN);
	list_for_each_entry(loop_ep, &udc->gadget.ep_list,
			ep.ep_list)
		nuke(loop_ep, -ESHUTDOWN);
	spin_unlock_irqrestore(&udc->lock, flags);

	/* report disconnect; the controller is already quiesced */
	driver->disconnect(&udc->gadget);

	/* unbind gadget and unhook driver. */
	driver->unbind(&udc->gadget);
	udc->gadget.dev.driver = NULL;
	udc->driver = NULL;

	USB_WARNING("unregistered gadget driver '%s'\n",
	       driver->driver.name);

	DBG("%s(%d) END\n", __func__, __LINE__);

	return 0;
}


/* Internal structure setup functions */
static int tegra_udc_setup_qh(struct tegra_udc *udc)
{
	u32 dccparams;
	size_t size;
	struct resource *res;

	/* Read Device Controller Capability Parameters register */
	dccparams = udc_readl(udc, DCCPARAMS_REG_OFFSET);
	if (!(dccparams & DCCPARAMS_DC)) {
		ERR("This SOC doesn't support device role\n");
		return -ENODEV;
	}

	/* Get max device endpoints */
	/* DEN is bidirectional ep number, max_ep doubles the number */
	udc->max_ep = (dccparams & DCCPARAMS_DEN_MASK) * 2;

	udc->eps = kzalloc(sizeof(struct tegra_ep) * udc->max_ep, GFP_KERNEL);
	if (!udc->eps) {
		ERR("malloc tegra_ep failed\n");
		return -1;
	}

	/* Setup hardware queue heads */
	size = udc->max_ep * sizeof(struct ep_queue_head);
	udc->ep_qh = (struct ep_queue_head *)((u8 *)(udc->regs) + QH_OFFSET);
	res = platform_get_resource(udc->pdev, IORESOURCE_MEM, 0);
	if (!res) {
		ERR("failed to get udc platform resource mem\n");
		return -ENXIO;
	}
	udc->ep_qh_dma = res->start + QH_OFFSET;
	udc->ep_qh_size = size;

	/* Initialize ep0 status request structure */
	/* FIXME: tegra_alloc_request() ignores ep argument */
	udc->status_req = container_of(tegra_alloc_request(NULL, GFP_KERNEL),
			struct tegra_req, req);
	if (udc->status_req == NULL) {
		ERR("udc->status_req is null\n");
		return -1;
	}
	/* Allocate a small amount of memory to get valid address */
	udc->status_req->req.buf = dma_alloc_coherent(&udc->pdev->dev,
				STATUS_BUFFER_SIZE, &udc->status_req->req.dma,
				GFP_KERNEL);
	if (!udc->status_req->req.buf) {
		ERR("alloc status_req buffer failed\n");
		kfree(udc->eps);
		return -ENOMEM;
	}

	udc->resume_state = USB_STATE_NOTATTACHED;
	udc->usb_state = USB_STATE_POWERED;
	udc->ep0_dir = 0;
	udc->remote_wakeup = 0;	/* default to 0 on reset */

	return 0;
}

/**
 * Setup the tegra_ep struct for eps
 * Link tegra_ep->ep to gadget->ep_list
 * ep0out is not used so do nothing here
 * ep0in should be taken care
 */
static int __init struct_ep_setup(struct tegra_udc *udc, unsigned char index,
		char *name, int link)
{
	struct tegra_ep *ep = &udc->eps[index];

	ep->udc = udc;
	strcpy(ep->name, name);
	ep->ep.name = ep->name;

	ep->ep.ops = &tegra_ep_ops;
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

static int tegra_udc_ep_setup(struct tegra_udc *udc)
{
	/* initialize EP0 descriptor */
	static const struct usb_endpoint_descriptor tegra_ep0_desc = {
		.bLength =		USB_DT_ENDPOINT_SIZE,
		.bDescriptorType =	USB_DT_ENDPOINT,
		.bEndpointAddress = 0,
		.bmAttributes = USB_ENDPOINT_XFER_CONTROL,
		.wMaxPacketSize =	USB_MAX_CTRL_PAYLOAD,
	};
	int i;

	/* setup QH and epctrl for ep0 */
	ep0_setup(udc);

	/* setup udc->eps[] for ep0 */
	struct_ep_setup(udc, 0, "ep0", 0);
	/* for ep0: the desc defined here;
	 * for other eps, gadget layer called ep_enable with defined desc
	 */
	udc->eps[0].desc = &tegra_ep0_desc;
	udc->eps[0].ep.maxpacket = USB_MAX_CTRL_PAYLOAD;

	/* setup the udc->eps[] for non-control endpoints and link
	 * to gadget.ep_list */
	for (i = 1; i < (int)(udc->max_ep / 2); i++) {
		char name[14];

		sprintf(name, "ep%dout", i);
		struct_ep_setup(udc, i * 2, name, 1);
		sprintf(name, "ep%din", i);
		struct_ep_setup(udc, i * 2 + 1, name, 1);
	}

	return 0;
}


/* Driver probe function
 * all intialization operations implemented here except enabling usb_intr reg
 * board setup should have been done in the platform code
 */
static int __init tegra_udc_probe(struct platform_device *pdev)
{
	struct tegra_udc *udc;
	struct resource *res;
	int err = -ENODEV;
	DBG("%s(%d) BEGIN\n", __func__, __LINE__);

	if (strcmp(pdev->name, driver_name)) {
		VDBG("Wrong device");
		return -ENODEV;
	}

	the_udc = udc = kzalloc(sizeof(struct tegra_udc), GFP_KERNEL);
	if (udc == NULL) {
		ERR("malloc udc failed\n");
		return -ENOMEM;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		err = -ENXIO;
		ERR("failed to get platform resources\n");
		goto err_kfree;
	}

	if (!request_mem_region(res->start, res->end - res->start + 1,
				driver_name)) {
		ERR("request mem region failed\n");
		err = -EBUSY;
		goto err_kfree;
	}

	udc->regs = ioremap(res->start, resource_size(res));
	if (!udc->regs) {
		err = -ENOMEM;
		ERR("failed to map mem region\n");
		goto err_rel_mem_region;
	}

	udc->irq = platform_get_irq(pdev, 0);
	if (!udc->irq) {
		err = -ENODEV;
		ERR("failed to get platform irq resources\n");
		goto err_iounmap;
	}

	err = request_irq(udc->irq, tegra_udc_irq,
				IRQF_SHARED | IRQF_TRIGGER_HIGH,
				driver_name, udc);
	if (err) {
		ERR("cannot request irq %d err %d\n", udc->irq, err);
		goto err_iounmap;
	}

	err = enable_irq_wake(udc->irq);
	if (err < 0) {
		dev_warn(&pdev->dev,
			"Couldn't enable USB udc mode wakeup, irq=%d, error=%d\n",
			udc->irq, err);
		err = 0;
	}

	udc->phy = tegra_usb_phy_open(pdev);
	if (IS_ERR(udc->phy)) {
		dev_err(&pdev->dev, "failed to open USB phy\n");
		err = -ENXIO;
		goto err_irq;
	}

	err = tegra_usb_phy_power_on(udc->phy);
	if (err) {
		dev_err(&pdev->dev, "failed to power on the phy\n");
		goto err_phy;
	}

	err = tegra_usb_phy_init(udc->phy);
	if (err) {
		dev_err(&pdev->dev, "failed to init the phy\n");
		goto err_phy;
	}
	spin_lock_init(&udc->lock);
	udc->stopped = 1;
	udc->pdev = pdev;
	udc->has_hostpc = tegra_usb_phy_has_hostpc(udc->phy) ? 1 : 0;
	platform_set_drvdata(pdev, udc);

	/* Initialize the udc structure including QH members */
	err = tegra_udc_setup_qh(udc);
	if (err) {
		dev_err(&pdev->dev, "failed to setup udc QH\n");
		goto err_phy;
	}

	/* Initialize usb hw reg except for regs for EP,
	 * leave usbintr reg untouched */
	err = dr_controller_setup(udc);
	if (err) {
		dev_err(&pdev->dev, "failed to setup udc controller\n");
		goto err_phy;
	}

	err = tegra_udc_setup_gadget_dev(udc);
	if (err) {
		dev_err(&pdev->dev, "failed to setup udc gadget device\n");
		goto err_phy;
	}

	err = tegra_udc_ep_setup(udc);
	if (err) {
		dev_err(&pdev->dev, "failed to setup end points\n");
		goto err_unregister;
	}

	/* Use dma_pool for TD management */
	udc->td_pool = dma_pool_create("udc_td", &pdev->dev,
			sizeof(struct ep_td_struct),
			DTD_ALIGNMENT, UDC_DMA_BOUNDARY);
	if (!udc->td_pool) {
		err = -ENOMEM;
		goto err_unregister;
	}

	err = usb_add_gadget_udc(&pdev->dev, &udc->gadget);
	if (err)
		goto err_del_udc;
#ifdef CONFIG_TEGRA_GADGET_BOOST_CPU_FREQ
	boost_cpufreq_work_flag = 1;
	ep_queue_request_count = 0;
	INIT_WORK(&udc->boost_cpufreq_work,
					tegra_udc_boost_cpu_frequency_work);
	pm_qos_add_request(&boost_cpu_freq_req, PM_QOS_CPU_FREQ_MIN,
					PM_QOS_DEFAULT_VALUE);
#endif

	usb_prepare(udc);	/* htc */

	/* Create work for controlling clocks to the phy if otg is disabled */
	INIT_WORK(&udc->irq_work, tegra_udc_irq_work);
	/* Create a delayed work for detecting the USB charger */
	INIT_DELAYED_WORK(&udc->work, tegra_udc_charger_detect_work);
	INIT_WORK(&udc->charger_work, tegra_udc_set_current_limit_work);

	/* Get the regulator for drawing the vbus current in udc driver */
	udc->vbus_reg = regulator_get(NULL, "usb_bat_chg");
	if (IS_ERR(udc->vbus_reg)) {
		dev_info(&pdev->dev,
			"usb_bat_chg regulator not registered:"
				" USB charging will not be enabled\n");
		udc->vbus_reg = NULL;
	}

#ifdef CONFIG_USB_OTG_UTILS
	if (tegra_usb_phy_otg_supported(udc->phy))
		udc->transceiver = otg_get_transceiver();

	if (udc->transceiver) {
		dr_controller_stop(udc);
		dr_controller_reset(udc);
		tegra_usb_phy_power_off(udc->phy);
		udc->vbus_active = 0;
		udc->usb_state = USB_STATE_DEFAULT;
		otg_set_peripheral(udc->transceiver, &udc->gadget);
	}
#else
	/* Power down the phy if cable is not connected */
	if (!vbus_enabled(udc))
		tegra_usb_phy_power_off(udc->phy);
#endif

	INIT_DELAYED_WORK(&disable_charger_hizmode_work, disable_charger_hizmode);

	DBG("%s(%d) END\n", __func__, __LINE__);
	return 0;

err_del_udc:
	dma_pool_destroy(udc->td_pool);

err_unregister:
	device_unregister(&udc->gadget.dev);

err_phy:
	tegra_usb_phy_close(udc->phy);

err_irq:
	free_irq(udc->irq, udc);

err_iounmap:
	iounmap(udc->regs);

err_rel_mem_region:
	release_mem_region(res->start, res->end - res->start + 1);

err_kfree:
	kfree(udc);

	return err;
}

/* Driver removal function
 * Free resources and finish pending transactions
 */
static int __exit tegra_udc_remove(struct platform_device *pdev)
{
	struct tegra_udc *udc = platform_get_drvdata(pdev);
	struct resource *res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	DECLARE_COMPLETION(done);

	if (!udc)
		return -ENODEV;

	usb_del_gadget_udc(&udc->gadget);
	udc->done = &done;

	cancel_delayed_work(&udc->work);
#ifdef CONFIG_TEGRA_GADGET_BOOST_CPU_FREQ
	cancel_work_sync(&udc->boost_cpufreq_work);
#endif

	if (udc->vbus_reg)
		regulator_put(udc->vbus_reg);

	if (udc->transceiver)
		otg_set_peripheral(udc->transceiver, NULL);


	/* Free allocated memory */
	dma_free_coherent(&pdev->dev, STATUS_BUFFER_SIZE,
				udc->status_req->req.buf,
				udc->status_req->req.dma);
	kfree(udc->status_req);
	kfree(udc->eps);

	dma_pool_destroy(udc->td_pool);
	free_irq(udc->irq, udc);
	iounmap(udc->regs);
	release_mem_region(res->start, res->end - res->start + 1);

	device_unregister(&udc->gadget.dev);
	/* Free udc -- wait for the release() finished */
	wait_for_completion(&done);

	return 0;
}

static int tegra_udc_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct tegra_udc *udc = platform_get_drvdata(pdev);
	unsigned long flags;
	DBG("%s(%d) BEGIN\n", __func__, __LINE__);

	/* If the controller is in otg mode, return */
	if (udc->transceiver)
			return 0;

	if (udc->vbus_active) {
		spin_lock_irqsave(&udc->lock, flags);
		/* Reset all internal Queues and inform client driver */
		reset_queues(udc);
		udc->vbus_active = 0;
		udc->usb_state = USB_STATE_DEFAULT;
		spin_unlock_irqrestore(&udc->lock, flags);
	}
	/* Stop the controller and turn off the clocks */
	dr_controller_stop(udc);
	if (udc->transceiver)
		udc->transceiver->state = OTG_STATE_UNDEFINED;

	tegra_usb_phy_power_off(udc->phy);

	DBG("%s(%d) END\n", __func__, __LINE__);
	return 0;
}

static int tegra_udc_resume(struct platform_device *pdev)
{
	struct tegra_udc *udc = platform_get_drvdata(pdev);
	DBG("%s(%d) BEGIN\n", __func__, __LINE__);

	if (udc->transceiver)
		return 0;

	tegra_usb_phy_power_on(udc->phy);
	tegra_udc_restart(udc);

	/* Power down the phy if cable is not connected */
	if (!vbus_enabled(udc)) {
		udc->vbus_active = 0;
		tegra_usb_phy_power_off(udc->phy);
	}

	DBG("%s(%d) END\n", __func__, __LINE__);
	return 0;
}

static void tegra_udc_shutdown(struct platform_device *pdev)
{
	struct tegra_udc *udc = platform_get_drvdata(pdev);
	u32 tmp;
	if (udc == NULL || udc->transceiver == NULL) {
		return;
	}
	dev_info(&pdev->dev, "%s\n", __func__);

	if (can_pullup(udc)) {
		dev_info(&pdev->dev, "%s: pull down D+\n", __func__);
		tmp = udc_readl(udc, USB_CMD_REG_OFFSET);
		udc_writel(udc, (tmp & ~USB_CMD_RUN_STOP),
				USB_CMD_REG_OFFSET);
	}
}

void tegra_udc_set_phy_clk(bool pull_up)
{
	struct tegra_udc *udc = the_udc;
	tegra_usb_set_usb_clk(udc->phy, pull_up);
}


static struct platform_driver tegra_udc_driver = {
	.remove  = __exit_p(tegra_udc_remove),
	.suspend = tegra_udc_suspend,
	.resume  = tegra_udc_resume,
	.shutdown = tegra_udc_shutdown,
	.driver  = {
		.name = (char *)driver_name,
		.owner = THIS_MODULE,
	},
};

static int __init udc_init(void)
{
	USB_INFO("%s (%s)\n", driver_desc, DRIVER_VERSION);
	return platform_driver_probe(&tegra_udc_driver, tegra_udc_probe);
}
module_init(udc_init);
static void __exit udc_exit(void)
{
	platform_driver_unregister(&tegra_udc_driver);
	USB_WARNING("%s unregistered\n", driver_desc);
}
module_exit(udc_exit);

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:tegra-udc");
