/*
 * drivers/usb/otg/tegra-otg.c
 *
 * OTG transceiver driver for Tegra UTMI phy
 *
 * Copyright (C) 2010-2012 NVIDIA CORPORATION. All rights reserved.
 * Copyright (C) 2010 Google, Inc.
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
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/usb.h>
#include <linux/usb/otg.h>
#include <linux/usb/gadget.h>
#include <linux/platform_device.h>
#include <linux/platform_data/tegra_usb.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/fsl_devices.h>
#include <linux/switch.h>
#include <mach/board_htc.h>
#include <linux/regulator/consumer.h>
#include <linux/wakelock.h>
#include <linux/usb/htc_info.h>
#include <mach/board_htc.h>

#define USB_PHY_WAKEUP		0x408
#define  USB_ID_INT_EN		(1 << 0)
#define  USB_ID_INT_STATUS	(1 << 1)
#define  USB_ID_STATUS		(1 << 2)
#define  USB_ID_PIN_WAKEUP_EN	(1 << 6)
#define  USB_VBUS_WAKEUP_EN	(1 << 30)
#define  USB_VBUS_INT_EN	(1 << 8)
#define  USB_VBUS_INT_STATUS	(1 << 9)
#define  USB_VBUS_STATUS	(1 << 10)
#define  USB_INT_EN		(USB_VBUS_INT_EN | USB_ID_INT_EN | \
						USB_VBUS_WAKEUP_EN | USB_ID_PIN_WAKEUP_EN)

#ifdef DEBUG
#define DBG(stuff...)	pr_info("tegra-otg: " stuff)
#else
#define DBG(stuff...)	do {} while (0)
#endif

struct tegra_otg_data {
	struct otg_transceiver otg;
	unsigned long int_status;
	spinlock_t lock;
	void __iomem *regs;
	struct clk *clk;
	int irq;
	struct platform_device *pdev;
	struct work_struct work;
	unsigned int intr_reg_data;
	bool clk_enabled;
	bool interrupt_mode;
	bool builtin_host;
	bool suspended
};

static struct tegra_otg_data *tegra_clone;
static void tegra_change_otg_state(struct tegra_otg_data *tegra, enum usb_otg_state to);
static unsigned long enable_interrupt(struct tegra_otg_data *tegra, bool en);

int USB_disabled;
EXPORT_SYMBOL_GPL(USB_disabled);

enum {
    NOT_ON_AUTOBOT,
    DOCK_ON_AUTOBOT,
    HTC_MODE_RUNNING
};

#if defined(CONFIG_CABLE_DETECT_ACCESSORY)
extern void cable_detection_queue_recovery_host_work(time);
#endif

#if (defined(CONFIG_USB_OTG) && defined(CONFIG_USB_OTG_HOST))
void usb_host_status_notifier_func(int isEnable)
{
	unsigned long flags, val;
	USBH_INFO("%s %d", __func__, isEnable);
	if (isEnable) {
		enable_interrupt(tegra_clone, false);
		tegra_change_otg_state(tegra_clone, OTG_STATE_A_SUSPEND);
		tegra_change_otg_state(tegra_clone, OTG_STATE_A_HOST);
		tegra_clone->interrupt_mode = false;
	} else {
		tegra_clone->interrupt_mode = true;
		tegra_change_otg_state(tegra_clone, OTG_STATE_A_SUSPEND);
		val = enable_interrupt(tegra_clone, true);
		if ((val & USB_ID_STATUS) && (val & USB_VBUS_STATUS))
			val |= USB_VBUS_INT_STATUS;
		else if (!(val & USB_ID_STATUS)) {
			if (!tegra_clone->builtin_host)
				val &= ~USB_ID_INT_STATUS;
			else
				val |= USB_ID_INT_STATUS;
		} else
			val &= ~(USB_ID_INT_STATUS | USB_VBUS_INT_STATUS);

		if ((val & USB_ID_INT_STATUS) || (val & USB_VBUS_INT_STATUS)) {
			tegra_clone->int_status = val;
			USBH_INFO("re-detect");
			schedule_work(&tegra_clone->work);
		}
	}
}

static struct t_usb_host_status_notifier usb_host_status_notifier = {
	.name = "usb_host_enable",
	.func = usb_host_status_notifier_func,
};
#endif


static inline unsigned long otg_readl(struct tegra_otg_data *tegra,
				      unsigned int offset)
{
	return readl(tegra->regs + offset);
}

static inline void otg_writel(struct tegra_otg_data *tegra, unsigned long val,
			      unsigned int offset)
{
	writel(val, tegra->regs + offset);
}

static const char *tegra_state_name(enum usb_otg_state state)
{
	switch (state) {
		case OTG_STATE_A_HOST:
			return "HOST";
		case OTG_STATE_B_PERIPHERAL:
			return "PERIPHERAL";
		case OTG_STATE_A_SUSPEND:
			return "SUSPEND";
		case OTG_STATE_UNDEFINED:
			return "UNDEFINED";
		default:
			return "INVALID";
	}
}

static unsigned long enable_interrupt(struct tegra_otg_data *tegra, bool en)
{
	unsigned long val;

	clk_enable(tegra->clk);
	val = otg_readl(tegra, USB_PHY_WAKEUP);
	if (en) {
		if (tegra->builtin_host)
			val |= USB_INT_EN;
		else
			val |= USB_VBUS_INT_EN | USB_VBUS_WAKEUP_EN | USB_ID_PIN_WAKEUP_EN;
	}
	else
		val &= ~USB_INT_EN;
	otg_writel(tegra, val, USB_PHY_WAKEUP);
	/* Add delay to make sure register is updated */
	udelay(1);
	clk_disable(tegra->clk);

	return val;
}

static void tegra_start_host(struct tegra_otg_data *tegra)
{
	struct tegra_usb_otg_data *pdata = tegra->otg.dev->platform_data;
	struct platform_device *pdev, *ehci_device = pdata->ehci_device;
	void *platform_data;
	int val;
	DBG("%s(%d) Begin\n", __func__, __LINE__);

	if (tegra->pdev)
		return ;

	/* prepare device structure for registering host*/
	pdev = platform_device_alloc(ehci_device->name, ehci_device->id);
	if (!pdev)
		return ;

	val = platform_device_add_resources(pdev, ehci_device->resource,
					    ehci_device->num_resources);
	if (val)
		goto error;

	pdev->dev.dma_mask = ehci_device->dev.dma_mask;
	pdev->dev.coherent_dma_mask = ehci_device->dev.coherent_dma_mask;

	platform_data = kmalloc(sizeof(struct tegra_usb_platform_data), GFP_KERNEL);
	if (!platform_data)
		goto error;

	memcpy(platform_data, pdata->ehci_pdata,
					sizeof(struct tegra_usb_platform_data));
	pdev->dev.platform_data = platform_data;
	val = platform_device_add(pdev);
	if (val)
		goto error_add;

	tegra->pdev = pdev;
	DBG("%s(%d) End\n", __func__, __LINE__);
	return ;

error_add:
	kfree(platform_data);
error:
	pr_err("%s: failed to add the host controller device\n", __func__);
	platform_device_put(pdev);
	tegra->pdev = NULL;
}

static void tegra_stop_host(struct tegra_otg_data *tegra)
{
	struct platform_device *pdev = tegra->pdev;

	DBG("%s(%d) Begin\n", __func__, __LINE__);

	if (pdev) {
		/* unregister host from otg */
		kfree(pdev->dev.platform_data);
		pdev->dev.platform_data = NULL;
		platform_device_unregister(pdev);
		tegra->pdev = NULL;
	}

	DBG("%s(%d) End\n", __func__, __LINE__);
}

static void dump_otg_state(void)
{
	unsigned long flags;
	unsigned long status;
	unsigned long val;
	if (!tegra_clone)
		return;
	status = tegra_clone->int_status;

	/* Debug prints */
	USBH_INFO("%s(%d) status = 0x%x\n", __func__, __LINE__, status);
	if ((status & USB_ID_INT_STATUS) &&
			(status & USB_VBUS_INT_STATUS))
		USBH_INFO("%s(%d) got vbus & id interrupt\n", __func__, __LINE__);
	else {
		if (status & USB_ID_INT_STATUS)
			USBH_INFO("%s(%d) got id interrupt\n", __func__, __LINE__);
		if (status & USB_VBUS_INT_STATUS)
			USBH_INFO("%s(%d) got vbus interrupt\n", __func__, __LINE__);
	}

	spin_lock_irqsave(&tegra_clone->lock, flags);
	val = otg_readl(tegra_clone, USB_PHY_WAKEUP);
	spin_unlock_irqrestore(&tegra_clone->lock, flags);
	USBH_INFO("%s(%d) USB_PHY_WAKEUP val = 0x%x\n", __func__, __LINE__, val);
	if (val & (USB_VBUS_INT_EN | USB_ID_INT_EN)) {
		USBH_INFO("%s(%d) PHY_WAKEUP = 0x%x\n", __func__, __LINE__, val);
	}


}
static void tegra_change_otg_state(struct tegra_otg_data *tegra,
				enum usb_otg_state to)
{
	struct otg_transceiver *otg = &tegra->otg;
	enum usb_otg_state from = otg->state;

	if(!tegra->interrupt_mode){
		DBG("OTG: Vbus detection is disabled");
		return;
	}

	DBG("%s(%d) requested otg state %s-->%s\n", __func__,
		__LINE__, tegra_state_name(from), tegra_state_name(to));

	if (to != OTG_STATE_UNDEFINED && from != to) {
		otg->state = to;
		/*dev_info(tegra->otg.dev, "%s --> %s\n", tegra_state_name(from),
					      tegra_state_name(to));*/
		USBH_INFO("%s --> %s\n", tegra_state_name(from), tegra_state_name(to));

		if (from == OTG_STATE_A_SUSPEND) {
			if (to == OTG_STATE_B_PERIPHERAL && otg->gadget) {
				usb_gadget_vbus_connect(otg->gadget);
#ifdef CONFIG_USB_ANDROID_PROJECTOR
				if (check_htc_mode_status() != NOT_ON_AUTOBOT) {
					htc_mode_enable(0);
					android_switch_default();
				}
#endif
			} else if (to == OTG_STATE_A_HOST) {
				tegra_start_host(tegra);
				dump_otg_state();
			}
		} else if (from == OTG_STATE_A_HOST) {
			if (to == OTG_STATE_A_SUSPEND)
				tegra_stop_host(tegra);
		} else if (from == OTG_STATE_B_PERIPHERAL && otg->gadget) {
			if (to == OTG_STATE_A_SUSPEND)
				usb_gadget_vbus_disconnect(otg->gadget);
		}
	} else if (to != OTG_STATE_UNDEFINED && from == to) {
		USBH_INFO("%s --> %s (%d)\n", tegra_state_name(from), tegra_state_name(to), USB_disabled);
		if (USB_disabled) {
			usb_gadget_disconnect(otg->gadget);
		} else if (to == OTG_STATE_B_PERIPHERAL) {
			usb_gadget_vbus_connect(otg->gadget);
			usb_gadget_connect(otg->gadget);
		} else {
			usb_gadget_connect(otg->gadget);
		}
	}
}

static void irq_work(struct work_struct *work)
{
	struct tegra_otg_data *tegra =
		container_of(work, struct tegra_otg_data, work);
	struct otg_transceiver *otg = &tegra->otg;
	enum usb_otg_state from = otg->state;
	enum usb_otg_state to = OTG_STATE_UNDEFINED;
	unsigned long flags;
	unsigned long status;

	spin_lock_irqsave(&tegra->lock, flags);
	status = tegra->int_status;

	/* Debug prints */
	DBG("%s(%d) status = 0x%x\n", __func__, __LINE__, status);
	if ((status & USB_ID_INT_STATUS) &&
			(status & USB_VBUS_INT_STATUS))
		DBG("%s(%d) got vbus & id interrupt\n", __func__, __LINE__);
	else {
		if (status & USB_ID_INT_STATUS)
			DBG("%s(%d) got id interrupt\n", __func__, __LINE__);
		if (status & USB_VBUS_INT_STATUS)
			DBG("%s(%d) got vbus interrupt\n", __func__, __LINE__);
	}

	if (!(status & USB_ID_STATUS) && (status & USB_ID_INT_EN))
		to = OTG_STATE_A_HOST;
	else if (status & USB_VBUS_STATUS && from != OTG_STATE_A_HOST)
		to = OTG_STATE_B_PERIPHERAL;
	else {
		if (from != OTG_STATE_A_HOST)
			to = OTG_STATE_A_SUSPEND;
	}

	spin_unlock_irqrestore(&tegra->lock, flags);
	tegra_change_otg_state(tegra, to);
	DBG("%s(%d) leave\n", __func__, __LINE__);
}

static irqreturn_t tegra_otg_irq(int irq, void *data)
{
	struct tegra_otg_data *tegra = data;
	unsigned long flags;
	unsigned long val;

	spin_lock_irqsave(&tegra->lock, flags);
	val = otg_readl(tegra, USB_PHY_WAKEUP);
	DBG("%s(%d) interrupt val = 0x%x\n", __func__, __LINE__, val);

	if (val & (USB_VBUS_INT_EN | USB_ID_INT_EN)) {
		DBG("%s(%d) PHY_WAKEUP = 0x%x\n", __func__, __LINE__, val);
		otg_writel(tegra, val, USB_PHY_WAKEUP);
		if ((val & USB_ID_INT_STATUS) || (val & USB_VBUS_INT_STATUS)) {
			tegra->int_status = val;
			schedule_work(&tegra->work);
		}
	}
	spin_unlock_irqrestore(&tegra->lock, flags);

	return IRQ_HANDLED;
}


static int tegra_otg_set_peripheral(struct otg_transceiver *otg,
				struct usb_gadget *gadget)
{
	struct tegra_otg_data *tegra;
	unsigned long val;
	DBG("%s(%d) BEGIN\n", __func__, __LINE__);

	tegra = container_of(otg, struct tegra_otg_data, otg);
	otg->gadget = gadget;

	val = enable_interrupt(tegra, true);

	if ((val & USB_ID_STATUS) && (val & USB_VBUS_STATUS))
		val |= USB_VBUS_INT_STATUS;
	else if (!(val & USB_ID_STATUS)) {
		if(!tegra->builtin_host)
			val &= ~USB_ID_INT_STATUS;
		else
			val |= USB_ID_INT_STATUS;
	}
	else
		val &= ~(USB_ID_INT_STATUS | USB_VBUS_INT_STATUS);

	if ((val & USB_ID_INT_STATUS) || (val & USB_VBUS_INT_STATUS)) {
		tegra->int_status = val;
		schedule_work(&tegra->work);
	}

	DBG("%s(%d) END\n", __func__, __LINE__);
	return 0;
}

static int tegra_otg_set_host(struct otg_transceiver *otg,
				struct usb_bus *host)
{
	struct tegra_otg_data *tegra;
	unsigned long val;
	DBG("%s(%d) BEGIN\n", __func__, __LINE__);

	tegra = container_of(otg, struct tegra_otg_data, otg);
	otg->host = host;

	clk_enable(tegra->clk);
	val = otg_readl(tegra, USB_PHY_WAKEUP);
	val &= ~(USB_VBUS_INT_STATUS | USB_ID_INT_STATUS);
	val |= (USB_ID_INT_EN | USB_ID_PIN_WAKEUP_EN);
	otg_writel(tegra, val, USB_PHY_WAKEUP);
	clk_disable(tegra->clk);

	DBG("%s(%d) END\n", __func__, __LINE__);
	return 0;
}

static int tegra_otg_set_power(struct otg_transceiver *otg, unsigned mA)
{
	return 0;
}

static int tegra_otg_set_suspend(struct otg_transceiver *otg, int suspend)
{
	return 0;
}

static ssize_t show_host_en(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct tegra_otg_data *tegra = platform_get_drvdata(pdev);

	*buf = tegra->interrupt_mode ? '0': '1';
	strcat(buf, "\n");
	return strlen(buf);
}

static ssize_t store_host_en(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct tegra_otg_data *tegra = platform_get_drvdata(pdev);
	unsigned long host;

	if (sscanf(buf, "%d", &host) != 1 || host < 0 || host > 1)
		return -EINVAL;

	USBH_INFO("%s %d ", __func__, host);
	if (host) {
		enable_interrupt(tegra, false);
		tegra_change_otg_state(tegra, OTG_STATE_A_SUSPEND);
		tegra_change_otg_state(tegra, OTG_STATE_A_HOST);
		tegra->interrupt_mode = false;
	} else {
		tegra->interrupt_mode = true;
		tegra_change_otg_state(tegra, OTG_STATE_A_SUSPEND);
		enable_interrupt(tegra, true);
	}

	return count;
}

static DEVICE_ATTR(enable_host, 0644, show_host_en, store_host_en);

static int tegra_otg_probe(struct platform_device *pdev)
{
	struct tegra_otg_data *tegra;
	struct resource *res;
	struct tegra_usb_otg_data *pdata = dev_get_platdata(&pdev->dev);
	int err;

	tegra = kzalloc(sizeof(struct tegra_otg_data), GFP_KERNEL);
	if (!tegra)
		return -ENOMEM;

	tegra->otg.dev = &pdev->dev;
	tegra->otg.label = "tegra-otg";
	tegra->otg.state = OTG_STATE_UNDEFINED;
	tegra->otg.set_host = tegra_otg_set_host;
	tegra->otg.set_peripheral = tegra_otg_set_peripheral;
	tegra->otg.set_suspend = tegra_otg_set_suspend;
	tegra->otg.set_power = tegra_otg_set_power;
	spin_lock_init(&tegra->lock);

	if (pdata) {
		tegra->builtin_host = !pdata->ehci_pdata->builtin_host_disabled;
	}

	platform_set_drvdata(pdev, tegra);
	tegra_clone = tegra;
	tegra->interrupt_mode = true;
	tegra->suspended = false;

	tegra->clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(tegra->clk)) {
		dev_err(&pdev->dev, "Can't get otg clock\n");
		err = PTR_ERR(tegra->clk);
		goto err_clk;
	}

	err = clk_enable(tegra->clk);
	if (err)
		goto err_clken;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "Failed to get I/O memory\n");
		err = -ENXIO;
		goto err_io;
	}
	tegra->regs = ioremap(res->start, resource_size(res));
	if (!tegra->regs) {
		err = -ENOMEM;
		goto err_io;
	}

	tegra->otg.state = OTG_STATE_A_SUSPEND;

	err = otg_set_transceiver(&tegra->otg);
	if (err) {
		dev_err(&pdev->dev, "can't register transceiver (%d)\n", err);
		goto err_otg;
	}

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		dev_err(&pdev->dev, "Failed to get IRQ\n");
		err = -ENXIO;
		goto err_irq;
	}

	tegra->irq = res->start;
	err = request_threaded_irq(tegra->irq, tegra_otg_irq,
				   NULL,
				   IRQF_SHARED | IRQF_TRIGGER_HIGH,
				   "tegra-otg", tegra);
	if (err) {
		dev_err(&pdev->dev, "Failed to register IRQ\n");
		goto err_irq;
	}

	err = enable_irq_wake(tegra->irq);
	if (err < 0) {
		dev_warn(&pdev->dev,
			"Couldn't enable USB otg mode wakeup, irq=%d, error=%d\n",
			tegra->irq, err);
		err = 0;
	}

	INIT_WORK(&tegra->work, irq_work);

	dev_info(&pdev->dev, "otg transceiver registered\n");

	err = device_create_file(&pdev->dev, &dev_attr_enable_host);
	if (err) {
		dev_warn(&pdev->dev, "Can't register sysfs attribute\n");
		goto err_irq;
	}

	clk_disable(tegra->clk);
#if (defined(CONFIG_USB_OTG) && defined(CONFIG_USB_OTG_HOST))
	usb_host_detect_register_notifier(&usb_host_status_notifier);
#endif

	USBH_INFO("%s done!\n", __func__);
	return 0;

err_irq:
	otg_set_transceiver(NULL);
err_otg:
	iounmap(tegra->regs);
err_io:
	clk_disable(tegra->clk);
err_clken:
	clk_put(tegra->clk);
err_clk:
	platform_set_drvdata(pdev, NULL);
	kfree(tegra);
	return err;
}

static int __exit tegra_otg_remove(struct platform_device *pdev)
{
	struct tegra_otg_data *tegra = platform_get_drvdata(pdev);

	free_irq(tegra->irq, tegra);
	otg_set_transceiver(NULL);
	iounmap(tegra->regs);
	clk_disable(tegra->clk);
	clk_put(tegra->clk);
	platform_set_drvdata(pdev, NULL);
	kfree(tegra);

	return 0;
}

#ifdef CONFIG_PM
static int tegra_otg_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct tegra_otg_data *tegra = platform_get_drvdata(pdev);
	struct otg_transceiver *otg = &tegra->otg;
	int val;
	DBG("%s(%d) BEGIN state : %s\n", __func__, __LINE__,
					tegra_state_name(otg->state));

	clk_enable(tegra->clk);
	val = otg_readl(tegra, USB_PHY_WAKEUP);
	val &= ~(USB_ID_INT_EN | USB_VBUS_INT_EN);
	otg_writel(tegra, val, USB_PHY_WAKEUP);
	clk_disable(tegra->clk);

	/* Suspend peripheral mode, host mode is taken care by host driver */
	/* htc host mode change otg state in other function */
	/*if (otg->state == OTG_STATE_B_PERIPHERAL)
		tegra_change_otg_state(tegra, OTG_STATE_A_SUSPEND);*/

	tegra->suspended = true;

	DBG("%s(%d) END\n", __func__, __LINE__);
	return 0;
}

static void tegra_otg_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct tegra_otg_data *tegra = platform_get_drvdata(pdev);
	struct otg_transceiver *otg = &tegra->otg;
	int val;
	unsigned long flags;
	DBG("%s(%d) BEGIN\n", __func__, __LINE__);

	if (!tegra->suspended)
		return;

	/* Clear pending interrupts */
	clk_enable(tegra->clk);
	val = otg_readl(tegra, USB_PHY_WAKEUP);
	otg_writel(tegra, val, USB_PHY_WAKEUP);
	DBG("%s(%d) PHY WAKEUP register : 0x%x\n", __func__, __LINE__, val);
	clk_disable(tegra->clk);

	/* Handle if host cable is replaced with device during suspend state */
	if (otg->state == OTG_STATE_A_HOST && (val & USB_ID_STATUS))
		tegra_change_otg_state(tegra, OTG_STATE_A_SUSPEND);

	/* Enable interrupt and call work to set to appropriate state */
	spin_lock_irqsave(&tegra->lock, flags);
	if (tegra->builtin_host)
		tegra->int_status = val | USB_INT_EN;
	else
		tegra->int_status = val | USB_VBUS_INT_EN | USB_VBUS_WAKEUP_EN |
			USB_ID_PIN_WAKEUP_EN;

	spin_unlock_irqrestore(&tegra->lock, flags);
	irq_work(&tegra->work);

	enable_interrupt(tegra, true);

	tegra->suspended = false;

	DBG("%s(%d) END\n", __func__, __LINE__);
}

static void tegra_otg_shutdown(struct device *dev)
{
	DBG("%s(%d) BEGIN\n", __func__, __LINE__);
	struct otg_transceiver *otg = &tegra_clone->otg;
	usb_gadget_disconnect(otg->gadget);
}

static const struct dev_pm_ops tegra_otg_pm_ops = {
	.complete = tegra_otg_resume,
	.suspend = tegra_otg_suspend,
};
#endif

static struct platform_driver tegra_otg_driver = {
	.driver = {
		.name  = "tegra-otg",
#ifdef CONFIG_PM
		.pm    = &tegra_otg_pm_ops,
#endif
	},
	.remove  = __exit_p(tegra_otg_remove),
	.shutdown = tegra_otg_shutdown,
	.probe   = tegra_otg_probe,
};

static int __init tegra_otg_init(void)
{
	return platform_driver_register(&tegra_otg_driver);
}
subsys_initcall(tegra_otg_init);

static void __exit tegra_otg_exit(void)
{
	platform_driver_unregister(&tegra_otg_driver);
}

/* htc */
void tegra_otg_set_disable_usb(int disable_usb)
{
	USB_disabled = disable_usb;
	schedule_work(&tegra_clone->work);
}

module_exit(tegra_otg_exit);
