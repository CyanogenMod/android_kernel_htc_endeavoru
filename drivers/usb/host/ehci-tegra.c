/*
 * EHCI-compliant USB host controller driver for NVIDIA Tegra SoCs
 *
 * Copyright (c) 2010 Google, Inc.
 * Copyright (c) 2009-2012 NVIDIA CORPORATION. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */

#include <linux/platform_device.h>
#include <linux/platform_data/tegra_usb.h>
#include <linux/irq.h>
#include <linux/usb/otg.h>
#include <mach/usb_phy.h>
#include <mach/iomap.h>

#include <mach/clk.h>
#include <linux/clk.h>

//+HTC:0621:For AP37+QCT
#include <linux/wakelock.h>
#include <asm/mach-types.h>

#if 1
#define EHCI_DBG(stuff...)	pr_info("ehci-tegra: " stuff)
#else
#define EHCI_DBG(stuff...)	do {} while (0)
#endif

#define MODEM_EHCI_ID 1
static const char driver_name[] = "tegra-ehci";

extern unsigned int ehci_remove;
extern int uhsic_phy_remove(struct tegra_usb_phy *phy);

#define TEGRA_USB_DMA_ALIGN 32

struct tegra_ehci_hcd {
	struct ehci_hcd *ehci;
	struct tegra_usb_phy *phy;
         struct clk *clk; //+Sophia
#ifdef CONFIG_USB_OTG_UTILS
	struct otg_transceiver *transceiver;
#endif
	struct mutex sync_lock;
	bool port_resuming;
	unsigned int irq;
	bool bus_suspended_fail;
};

struct dma_align_buffer {
	void *kmalloc_ptr;
	void *old_xfer_buffer;
	u8 data[0];
};

/* SSD_RIL */
int debug_gpio_dump();
int trigger_radio_fatal_get_coredump(char *reason);
int Modem_is_IMC();

//htc++
#ifdef CONFIG_QCT_9K_MODEM
extern bool Modem_is_QCT_MDM9K(void);
struct usb_hcd *mdm_hsic_usb_hcd = NULL;
struct ehci_hcd *mdm_hsic_ehci_hcd = NULL;
struct tegra_usb_phy *mdm_hsic_phy = NULL;
#endif //CONFIG_QCT_9K_MODEM
//htc--

//++SSD_RIL
bool device_ehci_shutdown;
//--SSD_RIL

static void free_align_buffer(struct urb *urb)
{
	struct dma_align_buffer *temp = container_of(urb->transfer_buffer,
						struct dma_align_buffer, data);

	if (!(urb->transfer_flags & URB_ALIGNED_TEMP_BUFFER))
		return;

	/* In transaction, DMA from Device */
	if (usb_urb_dir_in(urb))
		memcpy(temp->old_xfer_buffer, temp->data,
					urb->transfer_buffer_length);

	urb->transfer_buffer = temp->old_xfer_buffer;
	urb->transfer_flags &= ~URB_ALIGNED_TEMP_BUFFER;
	kfree(temp->kmalloc_ptr);
}

static int alloc_align_buffer(struct urb *urb, gfp_t mem_flags)
{
	struct dma_align_buffer *temp, *kmalloc_ptr;
	size_t kmalloc_size;

	if (urb->num_sgs || urb->sg ||
		urb->transfer_buffer_length == 0 ||
		!((uintptr_t)urb->transfer_buffer & (TEGRA_USB_DMA_ALIGN - 1)))
		return 0;

	/* Allocate a buffer with enough padding for alignment */
	kmalloc_size = urb->transfer_buffer_length +
		sizeof(struct dma_align_buffer) + TEGRA_USB_DMA_ALIGN - 1;
	kmalloc_ptr = kmalloc(kmalloc_size, mem_flags);

	if (!kmalloc_ptr)
		return -ENOMEM;

	/* Position our struct dma_align_buffer such that data is aligned */
	temp = PTR_ALIGN(kmalloc_ptr + 1, TEGRA_USB_DMA_ALIGN) - 1;
	temp->kmalloc_ptr = kmalloc_ptr;
	temp->old_xfer_buffer = urb->transfer_buffer;
	/* OUT transaction, DMA to Device */
	if (!usb_urb_dir_in(urb))
		memcpy(temp->data, urb->transfer_buffer,
				urb->transfer_buffer_length);

	urb->transfer_buffer = temp->data;
	urb->transfer_flags |= URB_ALIGNED_TEMP_BUFFER;

	return 0;
}

static int tegra_ehci_map_urb_for_dma(struct usb_hcd *hcd,
	struct urb *urb, gfp_t mem_flags)
{
	int ret;

	ret = alloc_align_buffer(urb, mem_flags);
	if (ret)
		return ret;

	ret = usb_hcd_map_urb_for_dma(hcd, urb, mem_flags);

	/* Control packets over dma */
	if (urb->setup_dma)
		dma_sync_single_for_device(hcd->self.controller,
			urb->setup_dma, sizeof(struct usb_ctrlrequest),
			DMA_TO_DEVICE);

	/* urb buffers over dma */
	if (urb->transfer_dma) {
		enum dma_data_direction dir;
		dir = usb_urb_dir_in(urb) ? DMA_FROM_DEVICE : DMA_TO_DEVICE;

		dma_sync_single_for_device(hcd->self.controller,
			urb->transfer_dma, urb->transfer_buffer_length, dir);
	}

	if (ret)
		free_align_buffer(urb);

	return ret;
}

static void tegra_ehci_unmap_urb_for_dma(struct usb_hcd *hcd,
	struct urb *urb)
{
	usb_hcd_unmap_urb_for_dma(hcd, urb);
	free_align_buffer(urb);

	if (urb->transfer_dma) {
		enum dma_data_direction dir;
		dir = usb_urb_dir_in(urb) ? DMA_FROM_DEVICE : DMA_TO_DEVICE;
		if (dir == DMA_FROM_DEVICE)
			dma_sync_single_for_cpu(hcd->self.controller,
				urb->transfer_dma, urb->transfer_buffer_length,
									   DMA_FROM_DEVICE);
	}
}

static irqreturn_t tegra_ehci_irq(struct usb_hcd *hcd)
{
	struct tegra_ehci_hcd *tegra = dev_get_drvdata(hcd->self.controller);
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);
	irqreturn_t irq_status;
	bool pmc_remote_wakeup = false;
	struct platform_device *pdev = container_of(hcd->self.controller, struct platform_device, dev);
	int ehci_id = pdev->id;

	spin_lock(&ehci->lock);
	irq_status = tegra_usb_phy_irq(tegra->phy);
	if (irq_status == IRQ_NONE) {
		spin_unlock(&ehci->lock);
		return irq_status;
	}
	if (tegra_usb_phy_remote_wakeup(tegra->phy)) {
		ehci_info(ehci, "remote wakeup detected on controller.%d\n", ehci_id);

		/* +SSD_RIL: workaround for remote wake AT timeout. */
		if (Modem_is_IMC() && ehci_id == MODEM_EHCI_ID)
		{
			debug_gpio_dump();
			//trigger_radio_fatal_get_coredump("remote wakeup in PMC");
		}
		else
		{
			pmc_remote_wakeup = true;
			usb_hcd_resume_root_hub(hcd);
		}
		/* -SSD_RIL: workaround for remote wake AT timeout. */

		spin_unlock(&ehci->lock);
		return irq_status;
	}
	spin_unlock(&ehci->lock);

//	EHCI_DBG("%s() cmd = 0x%x, int_sts = 0x%x, portsc = 0x%x\n", __func__,
//		ehci_readl(ehci, &ehci->regs->command),
//		ehci_readl(ehci, &ehci->regs->status),
//		ehci_readl(ehci, &ehci->regs->port_status[0]));

	irq_status = ehci_irq(hcd);

	if (pmc_remote_wakeup) {
		ehci->controller_remote_wakeup = false;
	}

	if (ehci->controller_remote_wakeup) {
		ehci->controller_remote_wakeup = false;

		/* +SSD_RIL: workaround for remote wake hang. */
		if (Modem_is_IMC() && ehci_id == MODEM_EHCI_ID)
		{
			ehci_info(ehci, "controller.%d remote wakeup detected", ehci_id);
			debug_gpio_dump();
			//trigger_radio_fatal_get_coredump("remote wakeup in controller");
		}
		else
		{
			tegra_usb_phy_pre_resume(tegra->phy, true);
			tegra->port_resuming = 1;
		}
		/* -SSD_RIL: workaround for remote wake hang. */
	}
	return irq_status;
}

static int tegra_ehci_hub_control(
	struct usb_hcd	*hcd,
	u16	typeReq,
	u16	wValue,
	u16	wIndex,
	char	*buf,
	u16	wLength
)
{
	struct tegra_ehci_hcd *tegra = dev_get_drvdata(hcd->self.controller);
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);
	int	retval = 0;
	u32 __iomem	*status_reg;
	void  __iomem *apb_misc = IO_ADDRESS(TEGRA_APB_MISC_BASE);
	struct platform_device *pdev = container_of(hcd->self.controller, struct platform_device, dev);
	int ehci_id = pdev->id;

	if (!tegra_usb_phy_hw_accessible(tegra->phy)) {
		if (buf)
			memset(buf, 0, wLength);
		return retval;
	}

	if(ehci_id == MODEM_EHCI_ID){
	printk(KERN_INFO"%s: req:0x%x wV:0x%x wI:0x%x MODE:0x%x CMD:0x%x PORTSC:0x%x STS:0x%x HOSTPC1:0x%x\n",
        __func__, typeReq, wValue, wIndex,
		readl(hcd->regs + 0x1f8),
		readl(&ehci->regs->command),
		readl(&ehci->regs->port_status[0]),
		readl(&ehci->regs->status),
		readl(hcd->regs + 0x1b4));

	printk(KERN_INFO"%s: INTR:0x%x STAT_CFG0:0x%x ASDBGREG:0x%x OBSCTRL:0x%x OBSDATA:0x%x port_resuming:%d\n",
		__func__,
		readl(hcd->regs + 0x138),
		readl(hcd->regs + 0xc28),
		readl(apb_misc + 0x810),
		readl(apb_misc + 0x818),
		readl(apb_misc + 0x81c),
		tegra->port_resuming);
	}

	/* Do tegra phy specific actions based on the type request */
	switch (typeReq) {
	case GetPortStatus:
		if (tegra->port_resuming) {
			u32 cmd;
			int delay = ehci->reset_done[wIndex-1] - jiffies;
			/* Sometimes it seems we get called too soon... In that case, wait.*/
			if (delay > 0) {
				printk(KERN_INFO"GetPortStatus called too soon, waiting %dms...\n",jiffies_to_msecs(delay));
				ehci_dbg(ehci, "GetPortStatus called too soon, waiting %dms...\n", jiffies_to_msecs(delay));
				mdelay(jiffies_to_msecs(delay));
			}
			status_reg = &ehci->regs->port_status[(wIndex & 0xff) - 1];
			tegra_usb_phy_post_resume(tegra->phy);
			tegra->port_resuming = 0;
			/* If run bit is not set by now enable it */
			cmd = ehci_readl(ehci, &ehci->regs->command);
			if (!(cmd & CMD_RUN)) {
				cmd |= CMD_RUN;
				ehci->command |= CMD_RUN;
				ehci_writel(ehci, cmd, &ehci->regs->command);
			}
			/* Now we can safely re-enable irqs */
			ehci_writel(ehci, INTR_MASK, &ehci->regs->intr_enable);
		}
		break;
	case ClearPortFeature:
		if (wValue == USB_PORT_FEAT_SUSPEND) {
			tegra_usb_phy_pre_resume(tegra->phy, false);
			tegra->port_resuming = 1;
		} else if (wValue == USB_PORT_FEAT_ENABLE) {
			u32 temp;
			temp = ehci_readl(ehci, &ehci->regs->port_status[0]) & ~PORT_RWC_BITS;
			ehci_writel(ehci, temp & ~PORT_PE, &ehci->regs->port_status[0]);
			return retval;
		}
		break;
	}

	/* handle ehci hub control request */
	retval = ehci_hub_control(hcd, typeReq, wValue, wIndex, buf, wLength);

	/* do tegra phy specific actions based on the type request */
	if (!retval) {
		switch (typeReq) {
		case SetPortFeature:
			if (wValue == USB_PORT_FEAT_SUSPEND) {
				/* Need a 4ms delay for controller to suspend */
				mdelay(4);
				tegra_usb_phy_post_suspend(tegra->phy);
			} else if (wValue == USB_PORT_FEAT_RESET) {
				if (ehci->reset_done[0] && wIndex == 1)
					tegra_usb_phy_bus_reset(tegra->phy);
			} else if (wValue == USB_PORT_FEAT_POWER) {
				if (wIndex == 1)
					tegra_usb_phy_port_power(tegra->phy);
			}
			break;
		case ClearPortFeature:
			if (wValue == USB_PORT_FEAT_SUSPEND) {
				/* tegra USB controller needs 25 ms to resume the port */
				ehci->reset_done[wIndex-1] = jiffies + msecs_to_jiffies(25);
			}
			break;
		}
	}

	return retval;
}

static void tegra_ehci_shutdown(struct usb_hcd *hcd)
{
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);
	struct tegra_ehci_hcd *tegra = dev_get_drvdata(hcd->self.controller);
	mutex_lock(&tegra->sync_lock);
	del_timer_sync(&ehci->watchdog);
	del_timer_sync(&ehci->iaa_watchdog);
	if (tegra_usb_phy_hw_accessible(tegra->phy)) {
		spin_lock_irq(&ehci->lock);
		ehci_silence_controller(ehci);
		spin_unlock_irq(&ehci->lock);
	}
	mutex_unlock(&tegra->sync_lock);
}

static int tegra_ehci_setup(struct usb_hcd *hcd)
{
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);
	struct tegra_ehci_hcd *tegra = dev_get_drvdata(hcd->self.controller);
	int retval;

	/* EHCI registers start at offset 0x100 */
	ehci->caps = hcd->regs + 0x100;
	ehci->regs = hcd->regs + 0x100 +
				HC_LENGTH(ehci, readl(&ehci->caps->hc_capbase));

	dbg_hcs_params(ehci, "reset");
	dbg_hcc_params(ehci, "reset");

	/* cache this readonly data; minimize chip reads */
	ehci->hcs_params = readl(&ehci->caps->hcs_params);
	ehci->has_hostpc = tegra_usb_phy_has_hostpc(tegra->phy) ? 1 : 0;
	ehci->broken_hostpc_phcd = true;

	hcd->has_tt = 1;

	retval = ehci_halt(ehci);
	if (retval)
		return retval;

	/* data structure init */
	retval = ehci_init(hcd);
	if (retval)
		return retval;

	ehci->sbrn = 0x20;
	ehci->controller_remote_wakeup = false;
	ehci_reset(ehci);
	tegra_usb_phy_reset(tegra->phy);

	ehci_port_power(ehci, 1);
	return retval;
}


#ifdef CONFIG_PM
static int tegra_ehci_bus_suspend(struct usb_hcd *hcd)
{
	struct tegra_ehci_hcd *tegra = dev_get_drvdata(hcd->self.controller);
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);
	int err = 0;

	EHCI_DBG("%s() BEGIN\n", __func__);
	pr_info("%s::resetdone = %ld:tegra->port_resuming =%d\n", __func__, ehci->reset_done[0], tegra->port_resuming);
	mutex_lock(&tegra->sync_lock);
	tegra->bus_suspended_fail = false;
	err = ehci_bus_suspend(hcd);
	if (err) {
		if(tegra->port_resuming)
			pr_info("%s: bus_suspended aborted because of resume. err=(%d):resetdone = %ld:tegra->port_resuming =%d\n", __func__, err, ehci->reset_done[0], tegra->port_resuming);
		else
			pr_info("%s: bus_suspended_fail err=(%d):resetdone = %ld\n", __func__, err, ehci->reset_done[0]);
		tegra->bus_suspended_fail = true;
	}
	else
		tegra_usb_phy_suspend(tegra->phy);
	mutex_unlock(&tegra->sync_lock);
	EHCI_DBG("%s() END\n", __func__);
	return err;
}

static int tegra_ehci_bus_resume(struct usb_hcd *hcd)
{
	struct tegra_ehci_hcd *tegra = dev_get_drvdata(hcd->self.controller);
	int err = 0;

	EHCI_DBG("%s() BEGIN\n", __func__);
	mutex_lock(&tegra->sync_lock);
	tegra_usb_phy_resume(tegra->phy);
	err = ehci_bus_resume(hcd);
	mutex_unlock(&tegra->sync_lock);
	EHCI_DBG("%s() END\n", __func__);

	return err;
}
#endif

static const struct hc_driver tegra_ehci_hc_driver = {
	.description		= hcd_name,
	.product_desc		= "Tegra EHCI Host Controller",
	.hcd_priv_size		= sizeof(struct ehci_hcd),
	.flags			= HCD_USB2 | HCD_MEMORY,

	/* standard ehci functions */
	.start			= ehci_run,
	.stop			= ehci_stop,
	.urb_enqueue		= ehci_urb_enqueue,
	.urb_dequeue		= ehci_urb_dequeue,
	.endpoint_disable	= ehci_endpoint_disable,
	.endpoint_reset	= ehci_endpoint_reset,
	.get_frame_number	= ehci_get_frame,
	.hub_status_data	= ehci_hub_status_data,
	.clear_tt_buffer_complete = ehci_clear_tt_buffer_complete,
	.relinquish_port	= ehci_relinquish_port,
	.port_handed_over	= ehci_port_handed_over,

	/* modified ehci functions for tegra */
	.reset			= tegra_ehci_setup,
	.irq			= tegra_ehci_irq,
	.shutdown		= tegra_ehci_shutdown,
	.map_urb_for_dma	= tegra_ehci_map_urb_for_dma,
	.unmap_urb_for_dma	= tegra_ehci_unmap_urb_for_dma,
	.hub_control		= tegra_ehci_hub_control,
#ifdef CONFIG_PM
	.bus_suspend	= tegra_ehci_bus_suspend,
	.bus_resume	= tegra_ehci_bus_resume,
#endif
};

static int tegra_ehci_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct usb_hcd *hcd;
	struct tegra_ehci_hcd *tegra;
	int err = 0;
	int irq;

	pr_info("%s: ehci.id = %d\n", __func__, pdev->id);

	device_ehci_shutdown = false;

	tegra = devm_kzalloc(&pdev->dev, sizeof(struct tegra_ehci_hcd),
		GFP_KERNEL);
	if (!tegra) {
		dev_err(&pdev->dev, "memory alloc failed\n");
		return -ENOMEM;
	}

	mutex_init(&tegra->sync_lock);

	hcd = usb_create_hcd(&tegra_ehci_hc_driver, &pdev->dev,
					dev_name(&pdev->dev));
	if (!hcd) {
		dev_err(&pdev->dev, "unable to create HCD\n");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, tegra);

        //+Sophia:0608
         tegra->clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(tegra->clk)) {
		dev_err(&pdev->dev, "Can't get ehci clock\n");
		err = PTR_ERR(tegra->clk);
		goto fail_io;
	}

	err = clk_enable(tegra->clk);
	if (err)
		goto fail_clken;


	tegra_periph_reset_assert(tegra->clk);
	udelay(2);
	tegra_periph_reset_deassert(tegra->clk);
	udelay(2);
        //+Sophia:0608
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "failed to get I/O memory\n");
		err = -ENXIO;
		goto fail_io;
	}
	hcd->rsrc_start = res->start;
	hcd->rsrc_len = resource_size(res);
	hcd->regs = ioremap(res->start, resource_size(res));
	if (!hcd->regs) {
		dev_err(&pdev->dev, "failed to remap I/O memory\n");
		err = -ENOMEM;
		goto fail_io;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "failed to get IRQ\n");
		err = -ENODEV;
		goto fail_irq;
	}
	set_irq_flags(irq, IRQF_VALID);
	tegra->irq = irq;

	tegra->phy = tegra_usb_phy_open(pdev);
	if (IS_ERR(tegra->phy)) {
		dev_err(&pdev->dev, "failed to open USB phy\n");
		err = -ENXIO;
		goto fail_irq;
	}

	err = tegra_usb_phy_power_on(tegra->phy);
	if (err) {
		dev_err(&pdev->dev, "failed to power on the phy\n");
		goto fail_phy;
	}

	err = tegra_usb_phy_init(tegra->phy);
	if (err) {
		dev_err(&pdev->dev, "failed to init the phy\n");
		goto fail_phy;
	}

	err = usb_add_hcd(hcd, irq, IRQF_SHARED | IRQF_TRIGGER_HIGH);
	if (err) {
		dev_err(&pdev->dev, "Failed to add USB HCD, error=%d\n", err);
		goto fail_phy;
	}

	err = enable_irq_wake(tegra->irq);
	if (err < 0) {
		dev_warn(&pdev->dev,
				"Couldn't enable USB host mode wakeup, irq=%d, "
				"error=%d\n", irq, err);
		err = 0;
		tegra->irq = 0;
	}

	tegra->ehci = hcd_to_ehci(hcd);

	//htc++
	#ifdef CONFIG_QCT_9K_MODEM
	if (Modem_is_QCT_MDM9K())
	{
		extern struct platform_device tegra_ehci2_device;

		if (&tegra_ehci2_device == pdev)
		{
			mdm_hsic_ehci_hcd = tegra->ehci;
			mdm_hsic_usb_hcd = hcd;
			mdm_hsic_phy = tegra->phy;
			pr_info("%s:: mdm_hsic_ehci_hcd = %x, mdm_hsic_usb_hcd = %x, mdm_hsic_phy = %x\n",
				__func__, (unsigned int)mdm_hsic_ehci_hcd, (unsigned int)mdm_hsic_usb_hcd, (unsigned int)mdm_hsic_phy);
		}
	}
	#endif //CONFIG_QCT_9K_MODEM
	//htc--

#ifdef CONFIG_USB_OTG_UTILS
	if (tegra_usb_phy_otg_supported(tegra->phy)) {
		tegra->transceiver = otg_get_transceiver();
		if (tegra->transceiver)
			otg_set_host(tegra->transceiver, &hcd->self);
	}
#endif
	return err;

fail_phy:
	tegra_usb_phy_close(tegra->phy);
fail_irq:
	iounmap(hcd->regs);
fail_clken:
	clk_put(tegra->clk);
fail_io:
	usb_put_hcd(hcd);

	return err;
}


#ifdef CONFIG_PM
static int tegra_ehci_resume(struct platform_device *pdev)
{
	struct tegra_ehci_hcd *tegra = platform_get_drvdata(pdev);

	dev_info(&pdev->dev, "%s\n", __func__);			//htc_dbg

	return tegra_usb_phy_power_on(tegra->phy);
}

static int tegra_ehci_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct tegra_ehci_hcd *tegra = platform_get_drvdata(pdev);

	dev_info(&pdev->dev, "%s\n", __func__);		//htc_dbg

	pr_info("%s:: bus_suspended_fail = %d\n", __func__, tegra->bus_suspended_fail);
	pr_info("%s::ehci_remove = %d\n", __func__, ehci_remove);

	/* bus suspend could have failed because of remote wakeup resume */
	if (tegra->bus_suspended_fail)
		return -EBUSY;
	else
		return tegra_usb_phy_power_off(tegra->phy);
}
#endif

static int tegra_ehci_remove(struct platform_device *pdev)
{
	struct tegra_ehci_hcd *tegra = platform_get_drvdata(pdev);
	struct usb_hcd *hcd = ehci_to_hcd(tegra->ehci);

	dev_info(&pdev->dev, "%s+\n", __func__);		//htc_dbg

	if (tegra == NULL || hcd == NULL)
		return -EINVAL;

#ifdef CONFIG_USB_OTG_UTILS
	if (tegra->transceiver) {
		otg_set_host(tegra->transceiver, NULL);
		otg_put_transceiver(tegra->transceiver);
	}
#endif

	if (tegra->irq)
		disable_irq_wake(tegra->irq);

	/* Make sure phy is powered ON to access USB register */
	if(!tegra_usb_phy_hw_accessible(tegra->phy))
		tegra_usb_phy_power_on(tegra->phy);

	pr_info("+%s:usb_remove_hcd\n", __func__);
	uhsic_phy_remove(tegra->phy);
	usb_remove_hcd(hcd);
	usb_put_hcd(hcd);
	ehci_remove = 1;
	tegra_usb_phy_power_off(tegra->phy);
	tegra_usb_phy_close(tegra->phy);
	iounmap(hcd->regs);
	platform_set_drvdata(pdev, NULL);

	//htc++
	#ifdef CONFIG_QCT_9K_MODEM
	if (Modem_is_QCT_MDM9K())
	{
		extern struct platform_device tegra_ehci2_device;

		if (&tegra_ehci2_device == pdev)
		{
			mdm_hsic_ehci_hcd = NULL;
			mdm_hsic_usb_hcd = NULL;
			mdm_hsic_phy = NULL;
			pr_info("%s:: mdm_hsic_ehci_hcd = %x, mdm_hsic_usb_hcd = %x, mdm_hsic_phy = %x\n",
				__func__, (unsigned int)mdm_hsic_ehci_hcd, (unsigned int)mdm_hsic_usb_hcd, (unsigned int)mdm_hsic_phy);
		}
	}
	#endif	//CONFIG_QCT_9K_MODEM
	//htc--

	dev_info(&pdev->dev, "%s-\n", __func__);		//htc_dbg

	return 0;
}

static int _try_lock(struct device *dev)
{
	int i = 0;

	while (!device_trylock(dev) && i++ < 100)
		msleep(10);

	return i < 100;
}

static void tegra_ehci_hcd_shutdown(struct platform_device *pdev)
{

	struct tegra_ehci_hcd *tegra = NULL;
	struct usb_hcd *hcd = NULL;

	dev_info(&pdev->dev, "%s+\n", __func__);

	device_ehci_shutdown = true;

	if (!_try_lock(&pdev->dev)) {
		pr_info("%s failed to lock device\n", __func__);
		goto exit;
	}

	tegra = platform_get_drvdata(pdev);

	if (!tegra) {
		pr_info("%s device has no driver data\n", __func__);
		goto unlock_device;
	}

	hcd = ehci_to_hcd(tegra->ehci);

	if ( !hcd || !hcd->driver ) {
		pr_info("%s: hcd <%p> or driver is NULL\n", __func__, hcd);
		WARN_ON(1);
		goto unlock_device;
	}

	if (hcd->driver->shutdown)
		hcd->driver->shutdown(hcd);

	dev_info(&pdev->dev, "%s-\n", __func__);		//htc_dbg

unlock_device:
	device_unlock(&pdev->dev);
exit:
	return;
}

static struct platform_driver tegra_ehci_driver = {
	.probe		= tegra_ehci_probe,
	.remove	= tegra_ehci_remove,
	.shutdown	= tegra_ehci_hcd_shutdown,
#ifdef CONFIG_PM
	.suspend = tegra_ehci_suspend,
	.resume  = tegra_ehci_resume,
#endif
	.driver	= {
		.name	= driver_name,
	}
};


EXPORT_SYMBOL_GPL(device_ehci_shutdown);