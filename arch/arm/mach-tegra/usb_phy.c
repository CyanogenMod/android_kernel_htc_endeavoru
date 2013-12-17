/*
 * arch/arm/mach-tegra/usb_phy.c
 *
 * Copyright (C) 2010 Google, Inc.
 * Copyright (c) 2010-2012, NVIDIA CORPORATION.  All rights reserved.
 *
 * Author:
 *	Erik Gilling <konkers@google.com>
 *	Benoit Goby <benoit@android.com>
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
#include <linux/resource.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/regulator/consumer.h>
#include <linux/platform_data/tegra_usb.h>
#include "tegra_usb_phy.h"
#include <mach/iomap.h>
#include "fuse.h"

#include "pm-irq.h"

#include <mach/board_htc.h>	//HTC
#include "clock.h"

#define MODULE_NAME "[USBPHY] "

#define ERR(stuff...)		pr_err("usb_phy: " stuff)
#define WARNING(stuff...)	pr_warning("usb_phy: " stuff)
#define INFO(stuff...)		pr_info("usb_phy: " stuff)

#define AHB_MEM_PREFETCH_CFG3		0xe0
#define AHB_MEM_PREFETCH_CFG4		0xe4
#define AHB_MEM_PREFETCH_CFG1		0xec
#define AHB_MEM_PREFETCH_CFG2		0xf0
#define PREFETCH_ENB			(1 << 31)

#define DEBUG
#ifdef DEBUG
#define DBG(stuff...)		if(get_radio_flag() & 0x0008){pr_info("usb_phy: " stuff);}
#else
#define DBG(stuff...)		do {} while (0)
#endif

#ifdef CONFIG_CABLE_DETECT_ACCESSORY
extern void cable_detection_queue_vbus_work(int);
#define DOCK_VBUS_DEBOUNCE 0.5*HZ
#endif

//htc ++
#ifdef HTC_DEBUG_USB_PHY_POWER_ON_STUCK
#define USB_PHY_ON_OFF_TIMEOUT	1000
extern struct tegra_usb_phy *mdm_hsic_phy;
unsigned int HTC_HSIC_PHY_FOOTPRINT = 0;
struct task_struct *hsic_phy_tsk = NULL;
struct clk *hsic_par_emc_clk = NULL;
int htc_hsic_phy_power_debug_flag = 0;

static void usb_phy_power_debug_switch(int on)
{
	#ifdef CONFIG_QCT_9K_MODEM
	extern bool Modem_is_QCT_MDM9K(void);
	extern bool mdm_in_fatal_handler;
	extern bool is_in_subsystem_restart;
	#endif	//CONFIG_QCT_9K_MODEM

	if (htc_hsic_phy_power_debug_flag == on)
		return;

	#ifdef CONFIG_QCT_9K_MODEM
	//don't change ftrace setting during modem restart
	if (mdm_in_fatal_handler || is_in_subsystem_restart)
		return;

	if (Modem_is_QCT_MDM9K()) {
		extern void ftrace_enable_sched_event(int on);
		ftrace_enable_sched_event(on);
	}
	#endif	//CONFIG_QCT_9K_MODEM

	pr_info("%s:(%d)\n", __func__, on);
	htc_hsic_phy_power_debug_flag = on;
}

static void usb_phy_power_on_off_timer_expire_fn(unsigned long unused)
{
	pr_info("%s:HTC_HSIC_PHY_FOOTPRINT(%d)\n", __func__, HTC_HSIC_PHY_FOOTPRINT);
	usb_phy_power_debug_switch(1);
	trace_printk("[%s]\n", __func__);
}
static DEFINE_TIMER(usb_phy_power_on_off_timer, usb_phy_power_on_off_timer_expire_fn, 0, 0);
#endif	//HTC_DEBUG_USB_PHY_POWER_ON_STUCK
//htc --

static void print_usb_plat_data_info(struct tegra_usb_phy *phy)
{
	struct tegra_usb_platform_data *pdata = phy->pdata;
	char op_mode[][50] = {
		"TEGRA_USB_OPMODE_DEVICE",
		"TEGRA_USB_OPMODE_HOST"
	};
	char phy_intf[][50] = {
		"USB_PHY_INTF_UTMI",
		"USB_PHY_INTF_ULPI_LINK",
		"USB_PHY_INTF_ULPI_NULL",
		"USB_PHY_INTF_HSIC",
		"USB_PHY_INTF_ICUSB"
	};

	pr_info("tegra USB phy - inst[%d] platform info:\n", phy->inst);
	pr_info("port_otg: %s\n", pdata->port_otg ? "yes" : "no");
	pr_info("has_hostpc: %s\n", pdata->has_hostpc ? "yes" : "no");
	pr_info("phy_interface: %s\n", phy_intf[pdata->phy_intf]);
	pr_info("op_mode: %s\n", op_mode[pdata->op_mode]);
	if (pdata->op_mode == TEGRA_USB_OPMODE_DEVICE) {
		pr_info("vbus_pmu_irq: %d\n", pdata->u_data.dev.vbus_pmu_irq);
		pr_info("vbus_gpio: %d\n", pdata->u_data.dev.vbus_gpio);
		pr_info("charging: %s\n", pdata->u_data.dev.charging_supported ?
				"enabled" : "disabled");
		pr_info("remote_wakeup: %s\n", pdata->u_data.dev.remote_wakeup_supported
				? "enabled" : "disabled");
	} else {
		pr_info("vbus_gpio: %d\n", pdata->u_data.host.vbus_gpio);
		pr_info("vbus_reg: %s\n", pdata->u_data.host.vbus_reg ?
				pdata->u_data.host.vbus_reg : "NULL");
		pr_info("hot_plug: %s\n", pdata->u_data.host.hot_plug ?
				"enabled" : "disabled");
		pr_info("remote_wakeup: %s\n", pdata->u_data.host.remote_wakeup_supported
				? "enabled" : "disabled");
	}
}

static void usb_host_vbus_enable(struct tegra_usb_phy *phy, bool enable)
{
	if (phy->vbus_reg) {
		if (enable)
			regulator_enable(phy->vbus_reg);
		else
			regulator_disable(phy->vbus_reg);
	} else {
		int gpio = phy->pdata->u_data.host.vbus_gpio;
		if (gpio == -1)
			return;
		gpio_set_value_cansleep(gpio, enable ? 1 : 0);
	}
}

int usb_phy_reg_status_wait(void __iomem *reg, u32 mask,
					u32 result, u32 timeout)
{
	do {
		if ((readl(reg) & mask) == result)
			return 0;
		udelay(1);
		timeout--;
	} while (timeout);

	return -1;
}

int tegra_usb_phy_init_ops(struct tegra_usb_phy *phy)
{
	int err = 0;

	DBG("%s(%d) inst:[%d]\n", __func__, __LINE__, phy->inst);

	if (phy->pdata->has_hostpc)
		err = tegra3_usb_phy_init_ops(phy);
	else
		err = tegra2_usb_phy_init_ops(phy);

	return err;
}


static irqreturn_t usb_phy_dev_vbus_pmu_irq_thr(int irq, void *pdata)
{
	struct tegra_usb_phy *phy = pdata;

	if (phy->vdd_reg && !phy->vdd_reg_on) {
		regulator_enable(phy->vdd_reg);
		phy->vdd_reg_on = true;
		/*
		 * Optimal time to get the regulator turned on
		 * before detecting vbus interrupt.
		 */
		mdelay(15);
	}

	/* clk is disabled during phy power off and not here*/
	if (!phy->ctrl_clk_on) {
		clk_enable(phy->ctrlr_clk);
		phy->ctrl_clk_on = true;
	}
#ifdef CONFIG_CABLE_DETECT_ACCESSORY
	cable_detection_queue_vbus_work(DOCK_VBUS_DEBOUNCE);
#endif
	return IRQ_HANDLED;
}

static void tegra_usb_phy_release_clocks(struct tegra_usb_phy *phy)
{
	clk_put(phy->emc_clk);
	clk_put(phy->sys_clk);
	if (phy->pdata->op_mode == TEGRA_USB_OPMODE_HOST)
		if (phy->pdata->u_data.host.hot_plug ||
			phy->pdata->u_data.host.remote_wakeup_supported)
			clk_disable(phy->ctrlr_clk);
	clk_put(phy->ctrlr_clk);
	clk_disable(phy->pllu_clk);
	clk_put(phy->pllu_clk);
}

static int tegra_usb_phy_get_clocks(struct tegra_usb_phy *phy)
{
	int err = 0;

	phy->pllu_clk = clk_get_sys(NULL, "pll_u");
	if (IS_ERR(phy->pllu_clk)) {
		ERR("inst:[%d] Can't get pllu_clk clock\n", phy->inst);
		err = PTR_ERR(phy->pllu_clk);
		goto fail_pll;
	}
	clk_enable(phy->pllu_clk);

	phy->ctrlr_clk = clk_get(&phy->pdev->dev, NULL);
	if (IS_ERR(phy->ctrlr_clk)) {
		dev_err(&phy->pdev->dev, "Can't get controller clock\n");
		err = PTR_ERR(phy->ctrlr_clk);
		goto fail_ctrlr_clk;
	}

	if (phy->pdata->op_mode == TEGRA_USB_OPMODE_HOST)
		if (phy->pdata->u_data.host.hot_plug ||
			phy->pdata->u_data.host.remote_wakeup_supported)
			clk_enable(phy->ctrlr_clk);

	phy->sys_clk = clk_get(&phy->pdev->dev, "sclk");
	if (IS_ERR(phy->sys_clk)) {
		dev_err(&phy->pdev->dev, "Can't get sclk clock\n");
		err = PTR_ERR(phy->sys_clk);
		goto fail_sclk;
	}
	if (phy->pdata->phy_intf == TEGRA_USB_PHY_INTF_UTMI)
		clk_set_rate(phy->sys_clk, 81600000);
	else
		clk_set_rate(phy->sys_clk, 80000000);

	phy->emc_clk = clk_get(&phy->pdev->dev, "emc");
	if (IS_ERR(phy->emc_clk)) {
		dev_err(&phy->pdev->dev, "Can't get emc clock\n");
		err = PTR_ERR(phy->emc_clk);
		goto fail_emc;
	}

	if(phy->pdata->has_hostpc)
		clk_set_rate(phy->emc_clk, 100000000);
	else
		clk_set_rate(phy->emc_clk, 300000000);

	if (phy->pdata->phy_intf == TEGRA_USB_PHY_INTF_UTMI)
		clk_set_rate(phy->emc_clk, 266500000);

	return err;

fail_emc:
	clk_put(phy->sys_clk);

fail_sclk:
	clk_put(phy->ctrlr_clk);

fail_ctrlr_clk:
	clk_disable(phy->pllu_clk);
	clk_put(phy->pllu_clk);

fail_pll:
	return err;
}

struct tegra_usb_phy *tegra_usb_phy_open(struct platform_device *pdev)
{
	struct tegra_usb_phy *phy;
	struct tegra_usb_platform_data *pdata;
	struct resource *res;
	int err;
	int plat_data_size = sizeof(struct tegra_usb_platform_data);

	DBG("%s(%d) inst:[%d]\n", __func__, __LINE__, pdev->id);
	pdata = dev_get_platdata(&pdev->dev);
	if (!pdata) {
		dev_err(&pdev->dev, "inst:[%d] Platform data missing\n",
								pdev->id);
		err = -EINVAL;
		goto fail_inval;
	}

	phy = devm_kzalloc(&pdev->dev, sizeof(struct tegra_usb_phy), GFP_KERNEL);
	if (!phy) {
		ERR("inst:[%d] malloc usb phy failed\n", pdev->id);
		err = -ENOMEM;
		goto fail_nomem;
	}

	phy->pdata = devm_kzalloc(&pdev->dev, plat_data_size, GFP_KERNEL);
	if (!phy->pdata) {
		ERR("inst:[%d] malloc usb phy pdata failed\n", pdev->id);
		devm_kfree(&pdev->dev, phy);
		err = -ENOMEM;
		goto fail_nomem;
	}

	memcpy(phy->pdata, pdata, plat_data_size);

	phy->pdev = pdev;
	phy->inst = pdev->id;

	print_usb_plat_data_info(phy);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		ERR("inst:[%d] failed to get I/O memory\n", phy->inst);
		err = -ENXIO;
		goto fail_io;
	}

	phy->regs = ioremap(res->start, resource_size(res));
	if (!phy->regs) {
		ERR("inst:[%d] Failed to remap I/O memory\n", phy->inst);
		err = -ENOMEM;
		goto fail_io;
	}

	phy->vdd_reg = regulator_get(&pdev->dev, "avdd_usb");
	if (IS_ERR_OR_NULL(phy->vdd_reg)) {
		ERR("inst:[%d] couldn't get regulator avdd_usb: %ld\n",
			phy->inst, PTR_ERR(phy->vdd_reg));
		phy->vdd_reg = NULL;
		err = PTR_ERR(phy->vdd_reg);
		goto fail_io;
	}

	err = tegra_usb_phy_get_clocks(phy);
	if (err) {
		ERR("inst:[%d] Failed to init clocks\n", phy->inst);
		goto fail_clk;
	}

	if (phy->pdata->op_mode == TEGRA_USB_OPMODE_DEVICE) {
		if (phy->pdata->u_data.dev.vbus_pmu_irq) {
			err = request_threaded_irq(
					phy->pdata->u_data.dev.vbus_pmu_irq,
					NULL, usb_phy_dev_vbus_pmu_irq_thr,
					IRQF_SHARED, "usb_pmu_vbus_irq", phy);
			if (err) {
				ERR("inst:[%d] Failed to register IRQ\n",
								phy->inst);
				goto fail_init;
			}
		} else {
			clk_enable(phy->ctrlr_clk);
		}
	} else {
		if (phy->pdata->u_data.host.vbus_reg) {
			phy->vbus_reg = regulator_get(NULL,
					phy->pdata->u_data.host.vbus_reg);
			if (WARN_ON(IS_ERR_OR_NULL(phy->vbus_reg))) {
				ERR("failed to get regulator vdd_vbus_usb: %ld,\
				 instance : %d\n", PTR_ERR(phy->vbus_reg),
								phy->inst);
				err = PTR_ERR(phy->vbus_reg);
				goto fail_init;
			}
		} else {
			int gpio = phy->pdata->u_data.host.vbus_gpio;
			if (gpio != -1) {
				if (gpio_request(gpio, "usb_host_vbus") < 0) {
					ERR("inst:[%d] host vbus gpio \
						 req failed\n", phy->inst);
					goto fail_init;
				}
				if (gpio < TEGRA_NR_GPIOS)
					tegra_gpio_enable(gpio);
				if (gpio_direction_output(gpio, 1) < 0) {
					ERR("inst:[%d] host vbus gpio \
						 dir failed\n", phy->inst);
					goto fail_init;
				}
			}
		}
		usb_host_vbus_enable(phy, true);
	}

	err = tegra_usb_phy_init_ops(phy);
	if (err) {
		ERR("inst:[%d] Failed to init ops\n", phy->inst);
		goto fail_init;
	}

	if (phy->pdata->ops && phy->pdata->ops->open)
		phy->pdata->ops->open();

	if (phy->ops && phy->ops->open) {
		err = phy->ops->open(phy);
		if (err) {
			ERR("inst:[%d] Failed to open hw ops\n", phy->inst);
			goto fail_init;
		}
	}

	return phy;

fail_init:
	tegra_usb_phy_release_clocks(phy);

	if (phy->pdata->op_mode == TEGRA_USB_OPMODE_DEVICE) {
		if (phy->pdata->u_data.dev.vbus_pmu_irq)
			free_irq(phy->pdata->u_data.dev.vbus_pmu_irq, phy);
	} else {
		usb_host_vbus_enable(phy, false);

		if (phy->vbus_reg)
			regulator_put(phy->vbus_reg);
		else {
			int gpio = phy->pdata->u_data.host.vbus_gpio;
			if (gpio != -1) {
				gpio_set_value_cansleep(gpio, 0);
				gpio_free(gpio);
			}
		}
	}

fail_clk:
	regulator_put(phy->vdd_reg);
	iounmap(phy->regs);
fail_io:
	devm_kfree(&pdev->dev, phy->pdata);
	devm_kfree(&pdev->dev, phy);

fail_nomem:
fail_inval:
	return ERR_PTR(err);
}



void tegra_usb_phy_close(struct tegra_usb_phy *phy)
{
	DBG("%s(%d) inst:[%d]\n", __func__, __LINE__, phy->inst);

	/* XXX
	 * not sure why auto init ops here, workaround for inst0 first
	 */
	if (phy->inst != 0) {
		if (phy->ops && phy->ops->close)
		{
			phy->ops->close(phy);
		}
	}

	if (phy->pdata->ops && phy->pdata->ops->close)
		phy->pdata->ops->close();

	if (phy->pdata->op_mode == TEGRA_USB_OPMODE_DEVICE) {
		if (phy->pdata->u_data.dev.vbus_pmu_irq)
			free_irq(phy->pdata->u_data.dev.vbus_pmu_irq, phy);
		else
			clk_disable(phy->ctrlr_clk);
	} else {
		usb_host_vbus_enable(phy, false);

		if (phy->vbus_reg)
			regulator_put(phy->vbus_reg);
		else {
			int gpio = phy->pdata->u_data.host.vbus_gpio;
			if (gpio != -1) {
				gpio_set_value_cansleep(gpio, 0);
				gpio_free(gpio);
			}
		}
	}

	if (phy->vdd_reg) {
		regulator_put(phy->vdd_reg);
	}

	tegra_usb_phy_release_clocks(phy);

	devm_kfree(&phy->pdev->dev, phy->pdata);
	devm_kfree(&phy->pdev->dev, phy);
}

irqreturn_t tegra_usb_phy_irq(struct tegra_usb_phy *phy)
{
	irqreturn_t status = IRQ_HANDLED;

	if (phy->ops && phy->ops->irq)
		status = phy->ops->irq(phy);

	return status;
}
int tegra_usb_phy_init(struct tegra_usb_phy *phy)
{
	int status = 0;

	DBG("%s(%d) inst:[%d]\n", __func__, __LINE__, phy->inst);

	if (phy->pdata->ops && phy->pdata->ops->init)
		phy->pdata->ops->init();

	if (phy->ops && phy->ops->init)
		status = phy->ops->init(phy);

	return status;
}

int tegra_usb_phy_power_off(struct tegra_usb_phy *phy)
{
	int err = 0;

	DBG("%s(%d) inst:[%d]\n", __func__, __LINE__, phy->inst);

	if (!phy->phy_power_on)
		return err;

	#ifdef HTC_DEBUG_USB_PHY_POWER_ON_STUCK
	if ((mdm_hsic_phy == phy) && (get_radio_flag() & 0x0008)) {
		hsic_phy_tsk = current;
		hsic_par_emc_clk = phy->emc_clk->parent;
		mod_timer(&usb_phy_power_on_off_timer, jiffies + msecs_to_jiffies(USB_PHY_ON_OFF_TIMEOUT));
	}
	#endif //HTC_DEBUG_USB_PHY_POWER_ON_STUCK

	if (phy->ops && phy->ops->power_off) {

		#ifdef HTC_DEBUG_USB_PHY_POWER_ON_STUCK
		if (mdm_hsic_phy == phy) HTC_HSIC_PHY_FOOTPRINT = __LINE__;
		#endif //HTC_DEBUG_USB_PHY_POWER_ON_STUCK

		if (phy->pdata->ops && phy->pdata->ops->pre_phy_off)
			phy->pdata->ops->pre_phy_off();

		#ifdef HTC_DEBUG_USB_PHY_POWER_ON_STUCK
		if (mdm_hsic_phy == phy) HTC_HSIC_PHY_FOOTPRINT = __LINE__;
		#endif //HTC_DEBUG_USB_PHY_POWER_ON_STUCK

		err = phy->ops->power_off(phy);

		#ifdef HTC_DEBUG_USB_PHY_POWER_ON_STUCK
		if (mdm_hsic_phy == phy) HTC_HSIC_PHY_FOOTPRINT = __LINE__;
		#endif //HTC_DEBUG_USB_PHY_POWER_ON_STUCK

		if (phy->pdata->ops && phy->pdata->ops->post_phy_off)
			phy->pdata->ops->post_phy_off();

		#ifdef HTC_DEBUG_USB_PHY_POWER_ON_STUCK
		if (mdm_hsic_phy == phy) HTC_HSIC_PHY_FOOTPRINT = __LINE__;
		#endif //HTC_DEBUG_USB_PHY_POWER_ON_STUCK
	}

	#ifdef HTC_DEBUG_USB_PHY_POWER_ON_STUCK
	if (mdm_hsic_phy == phy) HTC_HSIC_PHY_FOOTPRINT = __LINE__;
	#endif //HTC_DEBUG_USB_PHY_POWER_ON_STUCK

	clk_disable(phy->emc_clk);

	#ifdef HTC_DEBUG_USB_PHY_POWER_ON_STUCK
	if (mdm_hsic_phy == phy) HTC_HSIC_PHY_FOOTPRINT = __LINE__;
	#endif //HTC_DEBUG_USB_PHY_POWER_ON_STUCK

	clk_disable(phy->sys_clk);

	#ifdef HTC_DEBUG_USB_PHY_POWER_ON_STUCK
	if (mdm_hsic_phy == phy) HTC_HSIC_PHY_FOOTPRINT = __LINE__;
	#endif //HTC_DEBUG_USB_PHY_POWER_ON_STUCK

	if (phy->pdata->op_mode == TEGRA_USB_OPMODE_HOST) {
		if (!phy->pdata->u_data.host.hot_plug &&
			!phy->pdata->u_data.host.remote_wakeup_supported) {
			#ifdef HTC_DEBUG_USB_PHY_POWER_ON_STUCK
			if (mdm_hsic_phy == phy) HTC_HSIC_PHY_FOOTPRINT = __LINE__;
			#endif //HTC_DEBUG_USB_PHY_POWER_ON_STUCK

			clk_disable(phy->ctrlr_clk);

			#ifdef HTC_DEBUG_USB_PHY_POWER_ON_STUCK
			if (mdm_hsic_phy == phy) HTC_HSIC_PHY_FOOTPRINT = __LINE__;
			#endif //HTC_DEBUG_USB_PHY_POWER_ON_STUCK

			phy->ctrl_clk_on = false;
			if (phy->vdd_reg && phy->vdd_reg_on) {

				#ifdef HTC_DEBUG_USB_PHY_POWER_ON_STUCK
				if (mdm_hsic_phy == phy) HTC_HSIC_PHY_FOOTPRINT = __LINE__;
				#endif //HTC_DEBUG_USB_PHY_POWER_ON_STUCK

				regulator_disable(phy->vdd_reg);

				#ifdef HTC_DEBUG_USB_PHY_POWER_ON_STUCK
				if (mdm_hsic_phy == phy) HTC_HSIC_PHY_FOOTPRINT = __LINE__;
				#endif //HTC_DEBUG_USB_PHY_POWER_ON_STUCK

				phy->vdd_reg_on = false;
			}
		}
	} else {
		/* In device mode clock regulator/clocks will be turned off
		 * only if pmu interrupt is present on the board and host mode
		 * support through OTG is supported on the board.
		 */
		if (phy->pdata->u_data.dev.vbus_pmu_irq &&
			phy->pdata->builtin_host_disabled) {

			#ifdef HTC_DEBUG_USB_PHY_POWER_ON_STUCK
			if (mdm_hsic_phy == phy) HTC_HSIC_PHY_FOOTPRINT = __LINE__;
			#endif //HTC_DEBUG_USB_PHY_POWER_ON_STUCK

			clk_disable(phy->ctrlr_clk);

			#ifdef HTC_DEBUG_USB_PHY_POWER_ON_STUCK
			if (mdm_hsic_phy == phy) HTC_HSIC_PHY_FOOTPRINT = __LINE__;
			#endif //HTC_DEBUG_USB_PHY_POWER_ON_STUCK

			phy->ctrl_clk_on = false;
			if (phy->vdd_reg && phy->vdd_reg_on) {
				#ifdef HTC_DEBUG_USB_PHY_POWER_ON_STUCK
				if (mdm_hsic_phy == phy) HTC_HSIC_PHY_FOOTPRINT = __LINE__;
				#endif //HTC_DEBUG_USB_PHY_POWER_ON_STUCK

				regulator_disable(phy->vdd_reg);

				#ifdef HTC_DEBUG_USB_PHY_POWER_ON_STUCK
				if (mdm_hsic_phy == phy) HTC_HSIC_PHY_FOOTPRINT = __LINE__;
				#endif //HTC_DEBUG_USB_PHY_POWER_ON_STUCK

				phy->vdd_reg_on = false;
			}
		}
	}

	#ifdef HTC_DEBUG_USB_PHY_POWER_ON_STUCK
	if (mdm_hsic_phy == phy) HTC_HSIC_PHY_FOOTPRINT = __LINE__;
	#endif //HTC_DEBUG_USB_PHY_POWER_ON_STUCK

	#ifdef HTC_DEBUG_USB_PHY_POWER_ON_STUCK
	if ((mdm_hsic_phy == phy) && (get_radio_flag() & 0x0008)) {
		hsic_phy_tsk = NULL;
		hsic_par_emc_clk = NULL;
		usb_phy_power_debug_switch(0);
		del_timer(&usb_phy_power_on_off_timer);
	}
	#endif //HTC_DEBUG_USB_PHY_POWER_ON_STUCK

	phy->phy_power_on = false;

	return err;
}

int tegra_usb_phy_power_on(struct tegra_usb_phy *phy)
{
	int status = 0;

	DBG("%s(%d) inst:[%d]\n", __func__, __LINE__, phy->inst);

	if (phy->phy_power_on)
		return status;

	#ifdef HTC_DEBUG_USB_PHY_POWER_ON_STUCK
	if ((mdm_hsic_phy == phy) && (get_radio_flag() & 0x0008)) {
		hsic_phy_tsk = current;
		hsic_par_emc_clk = phy->emc_clk->parent;
		mod_timer(&usb_phy_power_on_off_timer, jiffies + msecs_to_jiffies(USB_PHY_ON_OFF_TIMEOUT));
	}
	#endif //HTC_DEBUG_USB_PHY_POWER_ON_STUCK

	if (phy->vdd_reg && !phy->vdd_reg_on) {
		#ifdef HTC_DEBUG_USB_PHY_POWER_ON_STUCK
		if (mdm_hsic_phy == phy) HTC_HSIC_PHY_FOOTPRINT = __LINE__;
		#endif //HTC_DEBUG_USB_PHY_POWER_ON_STUCK

		regulator_enable(phy->vdd_reg);

		#ifdef HTC_DEBUG_USB_PHY_POWER_ON_STUCK
		if (mdm_hsic_phy == phy) HTC_HSIC_PHY_FOOTPRINT = __LINE__;
		#endif //HTC_DEBUG_USB_PHY_POWER_ON_STUCK

		phy->vdd_reg_on = true;
	}

	/* In device mode clock is turned on by pmu irq handler
	 * if pmu irq is not available clocks will not be turned off/on
	 */
	#ifdef HTC_DEBUG_USB_PHY_POWER_ON_STUCK
	if (mdm_hsic_phy == phy) HTC_HSIC_PHY_FOOTPRINT = __LINE__;
	#endif //HTC_DEBUG_USB_PHY_POWER_ON_STUCK

	if (phy->pdata->op_mode == TEGRA_USB_OPMODE_HOST) {
		if (!phy->pdata->u_data.host.hot_plug &&
			!phy->pdata->u_data.host.remote_wakeup_supported)
			clk_enable(phy->ctrlr_clk);
	} else {
		if (phy->pdata->u_data.dev.vbus_pmu_irq &&
			!phy->ctrl_clk_on) {
			clk_enable(phy->ctrlr_clk);
			phy->ctrl_clk_on = true;
		}
	}

	#ifdef HTC_DEBUG_USB_PHY_POWER_ON_STUCK
	if (mdm_hsic_phy == phy) HTC_HSIC_PHY_FOOTPRINT = __LINE__;
	#endif //HTC_DEBUG_USB_PHY_POWER_ON_STUCK

	clk_enable(phy->sys_clk);

	#ifdef HTC_DEBUG_USB_PHY_POWER_ON_STUCK
	if (mdm_hsic_phy == phy) HTC_HSIC_PHY_FOOTPRINT = __LINE__;
	#endif //HTC_DEBUG_USB_PHY_POWER_ON_STUCK

	clk_enable(phy->emc_clk);

	#ifdef HTC_DEBUG_USB_PHY_POWER_ON_STUCK
	if (mdm_hsic_phy == phy) HTC_HSIC_PHY_FOOTPRINT = __LINE__;
	#endif //HTC_DEBUG_USB_PHY_POWER_ON_STUCK

	if (phy->ops && phy->ops->power_on) {
		#ifdef HTC_DEBUG_USB_PHY_POWER_ON_STUCK
		if (mdm_hsic_phy == phy) HTC_HSIC_PHY_FOOTPRINT = __LINE__;
		#endif //HTC_DEBUG_USB_PHY_POWER_ON_STUCK

		if (phy->pdata->ops && phy->pdata->ops->pre_phy_on)
			phy->pdata->ops->pre_phy_on();

		#ifdef HTC_DEBUG_USB_PHY_POWER_ON_STUCK
		if (mdm_hsic_phy == phy) HTC_HSIC_PHY_FOOTPRINT = __LINE__;
		#endif //HTC_DEBUG_USB_PHY_POWER_ON_STUCK

		status = phy->ops->power_on(phy);

		#ifdef HTC_DEBUG_USB_PHY_POWER_ON_STUCK
		if (mdm_hsic_phy == phy) HTC_HSIC_PHY_FOOTPRINT = __LINE__;
		#endif //HTC_DEBUG_USB_PHY_POWER_ON_STUCK

		if (phy->pdata->ops && phy->pdata->ops->post_phy_on)
			phy->pdata->ops->post_phy_on();

		#ifdef HTC_DEBUG_USB_PHY_POWER_ON_STUCK
		if (mdm_hsic_phy == phy) HTC_HSIC_PHY_FOOTPRINT = __LINE__;
		#endif //HTC_DEBUG_USB_PHY_POWER_ON_STUCK
	}

	#ifdef HTC_DEBUG_USB_PHY_POWER_ON_STUCK
	if (mdm_hsic_phy == phy) HTC_HSIC_PHY_FOOTPRINT = __LINE__;
	#endif //HTC_DEBUG_USB_PHY_POWER_ON_STUCK

	phy->phy_power_on = true;

	#ifdef HTC_DEBUG_USB_PHY_POWER_ON_STUCK
	if ((mdm_hsic_phy == phy) && (get_radio_flag() & 0x0008)) {
		hsic_phy_tsk = NULL;
		hsic_par_emc_clk = NULL;
		usb_phy_power_debug_switch(0);
		del_timer(&usb_phy_power_on_off_timer);
	}
	#endif //HTC_DEBUG_USB_PHY_POWER_ON_STUCK

	return status;
}

int tegra_usb_phy_reset(struct tegra_usb_phy *phy)
{
	int status = 0;

	DBG("%s(%d) inst:[%d]\n", __func__, __LINE__, phy->inst);
	if (phy->ops && phy->ops->reset)
		status = phy->ops->reset(phy);

	return status;
}
int tegra_usb_phy_pre_suspend(struct tegra_usb_phy *phy)
{
	int status = 0;

	DBG("%s(%d) inst:[%d]\n", __func__, __LINE__, phy->inst);

	if (phy->pdata->ops && phy->pdata->ops->pre_suspend)
		phy->pdata->ops->pre_suspend();

	if (phy->ops && phy->ops->pre_suspend)
		status = phy->ops->pre_suspend(phy);

	return status;
}
int tegra_usb_phy_suspend(struct tegra_usb_phy *phy)
{
	int err = 0;

	DBG("%s(%d) inst:[%d]\n", __func__, __LINE__, phy->inst);

	if (phy->pdata->ops && phy->pdata->ops->suspend)
		phy->pdata->ops->suspend();

	if (phy->ops && phy->ops->suspend)
		err = phy->ops->suspend(phy);

	if (!err && phy->pdata->u_data.host.power_off_on_suspend) {
		tegra_usb_phy_power_off(phy);
	}

	return err;
}
int tegra_usb_phy_post_suspend(struct tegra_usb_phy *phy)
{
	int status = 0;

	DBG("%s(%d) inst:[%d]\n", __func__, __LINE__, phy->inst);

	if (phy->ops && phy->ops->post_suspend)
		status = phy->ops->post_suspend(phy);

	if (phy->pdata->ops && phy->pdata->ops->post_suspend)
		phy->pdata->ops->post_suspend();

	return status;
}
int tegra_usb_phy_pre_resume(struct tegra_usb_phy *phy, bool remote_wakeup)
{
	int status = 0;

	DBG("%s(%d) inst:[%d]\n", __func__, __LINE__, phy->inst);

	if (phy->pdata->ops && phy->pdata->ops->pre_resume)
		phy->pdata->ops->pre_resume();

	if (phy->ops && phy->ops->pre_resume)
		status = phy->ops->pre_resume(phy, remote_wakeup);

	return status;
}
int tegra_usb_phy_resume(struct tegra_usb_phy *phy)
{
	int err = 0;

	DBG("%s(%d) inst:[%d]\n", __func__, __LINE__, phy->inst);

	if (phy->pdata->u_data.host.power_off_on_suspend) {
		tegra_usb_phy_power_on(phy);
	}

	if (!err && phy->ops && phy->ops->resume)
		err = phy->ops->resume(phy);

	if (phy->pdata->ops && phy->pdata->ops->resume)
		phy->pdata->ops->resume();

	return err;

}
int tegra_usb_phy_post_resume(struct tegra_usb_phy *phy)
{
	int status = 0;

	DBG("%s(%d) inst:[%d]\n", __func__, __LINE__, phy->inst);

	if (phy->ops && phy->ops->post_resume)
		status = phy->ops->post_resume(phy);

	if (phy->pdata->ops && phy->pdata->ops->post_resume)
		phy->pdata->ops->post_resume();

	return status;
}
int tegra_usb_phy_port_power(struct tegra_usb_phy *phy)
{
	int status = 0;

	DBG("%s(%d) inst:[%d]\n", __func__, __LINE__, phy->inst);

	if (phy->ops && phy->ops->port_power)
		status = phy->ops->port_power(phy);

	return status;
}
int tegra_usb_phy_bus_reset(struct tegra_usb_phy *phy)
{
	int status = 0;

	DBG("%s(%d) inst:[%d]\n", __func__, __LINE__, phy->inst);

	if (phy->ops && phy->ops->bus_reset)
		status = phy->ops->bus_reset(phy);

	return status;
}

bool tegra_usb_phy_charger_detected(struct tegra_usb_phy *phy)
{
	bool status = 0;

	DBG("%s(%d) inst:[%d]\n", __func__, __LINE__, phy->inst);
	if (phy->ops && phy->ops->charger_detect)
		status = phy->ops->charger_detect(phy);

	return status;
}

bool tegra_usb_phy_hw_accessible(struct tegra_usb_phy *phy)
{
	if (!phy->hw_accessible)
		DBG("%s(%d) inst:[%d] Not Accessible\n", __func__,
						__LINE__, phy->inst);

	return phy->hw_accessible;
}

bool tegra_usb_phy_remote_wakeup(struct tegra_usb_phy *phy)
{
	return phy->remote_wakeup;
}

bool tegra_usb_phy_has_hostpc(struct tegra_usb_phy *phy)
{
	return phy->pdata->has_hostpc;
}

bool tegra_usb_phy_otg_supported(struct tegra_usb_phy *phy)
{
	return phy->pdata->port_otg;
}

void tegra_usb_phy_memory_prefetch_on(struct tegra_usb_phy *phy)
{
	void __iomem *ahb_gizmo = IO_ADDRESS(TEGRA_AHB_GIZMO_BASE);
	unsigned long val;

	if (phy->inst == 0 && phy->pdata->op_mode == TEGRA_USB_OPMODE_DEVICE) {
		val = readl(ahb_gizmo + AHB_MEM_PREFETCH_CFG1);
		val |= PREFETCH_ENB;
		writel(val, ahb_gizmo + AHB_MEM_PREFETCH_CFG1);
		val = readl(ahb_gizmo + AHB_MEM_PREFETCH_CFG2);
		val |= PREFETCH_ENB;
		writel(val, ahb_gizmo + AHB_MEM_PREFETCH_CFG2);
	}
}

void tegra_usb_phy_memory_prefetch_off(struct tegra_usb_phy *phy)
{
	void __iomem *ahb_gizmo = IO_ADDRESS(TEGRA_AHB_GIZMO_BASE);
	unsigned long val;

	if (phy->inst == 0 && phy->pdata->op_mode == TEGRA_USB_OPMODE_DEVICE) {
		val = readl(ahb_gizmo + AHB_MEM_PREFETCH_CFG1);
		val &= ~(PREFETCH_ENB);
		writel(val, ahb_gizmo + AHB_MEM_PREFETCH_CFG1);
		val = readl(ahb_gizmo + AHB_MEM_PREFETCH_CFG2);
		val &= ~(PREFETCH_ENB);
		writel(val, ahb_gizmo + AHB_MEM_PREFETCH_CFG2);
	}
}

//++ htc ++
int tegra_usb_set_vbus_wakeup(int irq)
{
	int err = 0;
	err = tegra_pm_irq_set_wake(irq, true);
	if(err!=0){
		pr_err(MODULE_NAME "%s set wake error:%d\n", __func__,err);
		return err;
	}
	err = tegra_pm_irq_set_wake_type(irq, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING);
	if(err!=0){
		pr_err(MODULE_NAME "%s set wake_type error:%d\n", __func__,err);
	}
	return err;

}
void tegra_usb_set_usb_clk(struct tegra_usb_phy *phy, bool pull_up)
{
	if (!phy)
		return;
	DBG("%s pull_up:%d\n", __func__, pull_up);
	if (pull_up) {
		clk_set_rate(phy->sys_clk, 266000000);
		clk_set_rate(phy->emc_clk, 533000000);
	} else {
		clk_set_rate(phy->sys_clk, 81600000);
		clk_set_rate(phy->emc_clk, 266500000);
	}
}
//-- htc --
