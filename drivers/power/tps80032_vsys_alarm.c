/*
 * Copyright (C) 2012 HTC Incorporated
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

/* for pr_xxx log */
#define PR_TAG "[PMU_VSYS]"
#define pr_fmt(fmt) PR_TAG fmt

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/notifier.h>
#include <linux/suspend.h>
#include <linux/mfd/tps80031.h>
#include <linux/tps80032_vsys_alarm.h>

#define FUNC_CALL_CHECK 0

#if FUNC_CALL_CHECK
#define CHECK_LOG() pr_info("%s\n", __func__)
#else
#define CHECK_LOG() (void)0
#endif

#define TPS80032_VSYSMIN_HI_THRESHOLD	0x24
#define TPS80032_VSYSMIN_HI_CFG_TRANS	0xC9
#define TPS80032_VSYSMIN_HI_CFG_STATE	0xCA

struct tps80032_vsys_alarm_data {
	struct device 	*parent_dev;
	struct atomic_notifier_head irq_notifier_list;
	struct mutex vsys_alarm_mutex;
	unsigned int notifier_count;
	unsigned int reg_state;
	unsigned int reg_threshold;
	int vsys_low_irq;
};
struct tps80032_vsys_alarm_data *vsys_alarm_data;

static irqreturn_t tps80032_vsys_alarm_handler(int irq, void *data)
{
	CHECK_LOG();

	if (!vsys_alarm_data->reg_state)
		pr_err("%s:irq is not enabled by register\n", __func__);
	else
		atomic_notifier_call_chain(&vsys_alarm_data->irq_notifier_list,
							vsys_alarm_data->reg_state, NULL);

	return IRQ_HANDLED;
}

int tps80032_vsys_alarm_state_set(unsigned int state)
{
	CHECK_LOG();

	if (!vsys_alarm_data) {
		pr_err("%s:no tps80032 vsys alarm driver\n", __func__);
		return -ENXIO;
	}

	state = state ? 0x1 : 0x0;

	mutex_lock(&vsys_alarm_data->vsys_alarm_mutex);
	if (vsys_alarm_data->reg_state == state) {
		pr_info("%s: the same state set for vsys low alarm\n",
			__func__);
		goto done;
	}

	if (vsys_alarm_data->notifier_count) {
		if (state)
			enable_irq(vsys_alarm_data->vsys_low_irq);
		else
			disable_irq(vsys_alarm_data->vsys_low_irq);

		vsys_alarm_data->reg_state = state;
	}

done:
	mutex_unlock(&vsys_alarm_data->vsys_alarm_mutex);
	return 0;
}
EXPORT_SYMBOL_GPL(tps80032_vsys_alarm_state_set);

int tps80032_vsys_alarm_threshold_set(unsigned int threshold)
{
	int ret = 0;
	u8 reg;

	CHECK_LOG();

	if (!vsys_alarm_data) {
		pr_err("%s:no tps80032 vsys alarm driver\n", __func__);
		return -ENXIO;
	}

	if (threshold <= 2050)
		reg = 0x1;
	else if (threshold >= 4600)
		reg = 0x34;
	else
		reg = (threshold - 2000) / 50;

	mutex_lock(&vsys_alarm_data->vsys_alarm_mutex);
	if (vsys_alarm_data->reg_threshold == reg) {
		pr_info("%s: the same threshold set for vsys low alarm\n",
			__func__);
		goto fail;
	}

	ret = tps80031_write(vsys_alarm_data->parent_dev, SLAVE_ID1
				, TPS80032_VSYSMIN_HI_THRESHOLD
				, reg);
	if (ret < 0)
		pr_err("%s:write register VSYSMIN_HI_THRESHOLD fail\n"
				, __func__);
fail:
	mutex_unlock(&vsys_alarm_data->vsys_alarm_mutex);
	return ret;
}
EXPORT_SYMBOL_GPL(tps80032_vsys_alarm_threshold_set);

int tps80032_vsys_alarm_register_notifier(struct notifier_block *nb)
{
	int ret = 0;

	CHECK_LOG();

	if (!vsys_alarm_data) {
		pr_err("%s:no tps80032 vsys alarm driver\n", __func__);
		return -ENXIO;
	}

	ret = atomic_notifier_chain_register(
			&vsys_alarm_data->irq_notifier_list, nb);

	if (ret == 0) {
		mutex_lock(&vsys_alarm_data->vsys_alarm_mutex);
		if (vsys_alarm_data->notifier_count == 0) {
			ret = request_threaded_irq(vsys_alarm_data->vsys_low_irq,
					NULL, tps80032_vsys_alarm_handler, IRQF_ONESHOT,
					"tps80032_vsys_alarm", vsys_alarm_data);
			if (ret < 0) {
				pr_err("%s:request IRQ %d fail\n", __func__
							, vsys_alarm_data->vsys_low_irq);
				goto fail;
			}

			if (!vsys_alarm_data->reg_state)
				disable_irq(vsys_alarm_data->vsys_low_irq);
			else
				enable_irq(vsys_alarm_data->vsys_low_irq);

			ret = irq_set_irq_wake(vsys_alarm_data->vsys_low_irq, 1);
			if (ret < 0) {
				pr_err("%s:set_irq_wake failed irq %d\n",
					__func__, vsys_alarm_data->vsys_low_irq);
			}
		}

		vsys_alarm_data->notifier_count++;
fail:
		mutex_unlock(&vsys_alarm_data->vsys_alarm_mutex);
	}
	return ret;
}
EXPORT_SYMBOL_GPL(tps80032_vsys_alarm_register_notifier);

static int __devinit tps80032_vsys_alarm_probe(struct platform_device *pdev)
{
	struct tps80032_vsys_alarm_platform_data *pdata =
						pdev->dev.platform_data;
	struct tps80032_vsys_alarm_data *vsys_data;
	int ret;

	CHECK_LOG();

	vsys_data = kzalloc(sizeof(struct tps80032_vsys_alarm_data)
				, GFP_KERNEL);
	if (!vsys_data) {
		pr_err("%s:Unable to allocat memory!!\n", __func__);
		return -ENOMEM;
	}

	vsys_data->parent_dev = pdev->dev.parent;
	vsys_data->vsys_low_irq = pdata->vsys_low_irq;
	vsys_data->reg_state = 0;

	mutex_init(&vsys_data->vsys_alarm_mutex);

	ret = tps80031_write(vsys_data->parent_dev, SLAVE_ID1
				, TPS80032_VSYSMIN_HI_CFG_STATE
				, 0x1);
	if (ret < 0) {
		pr_err("%s:write register VSYSMIN_HI_CFG_STATE fail\n"
				, __func__);
		goto fail;
	}

#if 0	/* fixme: if we need set a default threshold */
	ret = tps80031_write(vsys_alarm_data->parent_dev, SLAVE_ID1
				, TPS80032_VSYSMIN_HI_THRESHOLD
				, 0x34);
	if (ret < 0) {
		pr_err("%s:write register VSYSMIN_HI_THRESHOLD fail\n"
				, __func__);
		goto fail;
	}
	vsys_data->reg_threshold = 0x00;
#endif

	ATOMIC_INIT_NOTIFIER_HEAD(&vsys_data->irq_notifier_list);

	vsys_alarm_data = vsys_data;
	return 0;
fail:
	kfree(vsys_data);
	return ret;
}

static int __devexit tps80032_vsys_alarm_teardown(struct platform_device *pdev)
{
	int ret;

	CHECK_LOG();

	if (vsys_alarm_data->notifier_count != 0)
		free_irq(vsys_alarm_data->vsys_low_irq, vsys_alarm_data);
	mutex_destroy(&vsys_alarm_data->vsys_alarm_mutex);
	kfree(vsys_alarm_data);
	return 0;
}

#ifdef CONFIG_PM
static int tps80032_vsys_alarm_suspend(struct platform_device *pdev,
						pm_message_t state)
{
	CHECK_LOG();

	return 0;
}

static int tps80032_vsys_alarm_resume(struct platform_device *pdev)
{
	CHECK_LOG();

	return 0;
}
#endif

static struct platform_driver tps80032_vsys_alarm_driver = {
	.probe = tps80032_vsys_alarm_probe,
	.remove = __devexit_p(tps80032_vsys_alarm_teardown),
#ifdef CONFIG_PM
	.suspend = tps80032_vsys_alarm_suspend,
	.resume = tps80032_vsys_alarm_resume,
#endif
	.driver = {
		.name = "tps80032-vsys_alarm",
		.owner = THIS_MODULE,
	},
};

static int __init tps80032_vsys_alarm_init(void)
{
	CHECK_LOG();

	return platform_driver_register(&tps80032_vsys_alarm_driver);
}
fs_initcall(tps80032_vsys_alarm_init);

static void __exit tps80032_vsys_alarm_exit(void)
{
	platform_driver_unregister(&tps80032_vsys_alarm_driver);
}
module_exit(tps80032_vsys_alarm_exit);

MODULE_DESCRIPTION("TPS80032 vsys alarm driver");
MODULE_LICENSE("GPL");
