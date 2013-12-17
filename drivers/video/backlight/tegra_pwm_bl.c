/*
 * linux/drivers/video/backlight/tegra_pwm_bl.c
 *
 * Tegra pwm backlight driver
 *
 * Copyright (C) 2011 NVIDIA Corporation
 * Author: Renuka Apte <rapte@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/fb.h>
#include <linux/backlight.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/tegra_pwm_bl.h>
#include <linux/delay.h>
#include <mach/dc.h>
#include <mach/panel_id.h>
#include <mach/board_htc.h>
extern int tegra_dsi_send_panel_short_cmd(struct tegra_dc *dc, u8 *pdata, u8 data_len);

extern struct tegra_dc *tegra_dc_get_dc(unsigned idx);
extern unsigned long g_panel_id;

static u8 enhance_on[]={0x55,0x83};
static u8 enhance_off[]={0x55,0x03};

static ssize_t store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct tegra_dc *dc;
	int value = 1;

	sscanf(buf,"%d\n",&value);

	dc = tegra_dc_get_dc(0);

	switch (PANEL_MASK(g_panel_id)) {
		case PANEL_ID_SHARP_HX_C3:
		case PANEL_ID_SHARP_HX_C4:
		case PANEL_ID_SHARP_HX_C5:
		case PANEL_ID_SHARP_HX:
		case PANEL_ID_SHARP_HX_43:
			enhance_on[0] = 0xE3;
			enhance_on[1] = 0x01;
			enhance_off[0] = 0xE3;
			enhance_off[1] = 0x00;
			break;
		default:
			break;
	}

	if ( value == 1)
	{
		tegra_dsi_send_panel_short_cmd(dc, enhance_on, ARRAY_SIZE(enhance_on));
		printk("[DISP]enable color enhancement %d\n", value);
	} else
	{
		tegra_dsi_send_panel_short_cmd(dc, enhance_off, ARRAY_SIZE(enhance_off));
		printk("[DISP]disable color enhancement %d\n", value);
	}
	return count;
}
static ssize_t show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return ;
}

static DEVICE_ATTR(enhance, 0664, show, store);

struct tegra_pwm_bl_data {
	struct device *dev;
	int which_dc;
	int (*notify)(struct device *, int brightness);
	struct tegra_dc_pwm_params params;
	int (*check_fb)(struct device *dev, struct fb_info *info);
};

static int tegra_pwm_backlight_update_status(struct backlight_device *bl)
{
	struct tegra_pwm_bl_data *tbl = dev_get_drvdata(&bl->dev);
	int brightness = bl->props.brightness;
	int max = bl->props.max_brightness;
	struct tegra_dc *dc;
	u8 *cur;
	int n_cur;

	u8 pdata[]={0x51,0xFF};
	if (!bl->props.bkl_on) {
		/*printk(KERN_DEBUG "[DISP] %s skip brightness=%d ,duty_cycle=%d\n",__FUNCTION__,brightness,tbl->params.duty_cycle);*/
		return 0;
	}

	if (bl->props.power != FB_BLANK_UNBLANK)
		brightness = 0;

	if (bl->props.fb_blank != FB_BLANK_UNBLANK)
		brightness = 0;

	if (tbl->notify)
		brightness = tbl->notify(tbl->dev, brightness);

	if (brightness > max)
		dev_err(&bl->dev, "Invalid brightness value: %d max: %d\n",
		brightness, max);

#if defined(CONFIG_ARCH_TEGRA_2x_SOC)
	/* map API brightness range from (0~255) to hw range (0~128) */
	tbl->params.duty_cycle = (brightness * 128) / 255;
#else
	tbl->params.duty_cycle = brightness & 0xFF;
#endif

	printk(KERN_INFO "[DISP] %s brightness=%d ,duty_cycle=%d\n",__FUNCTION__,brightness,tbl->params.duty_cycle);

	pdata[1]=tbl->params.duty_cycle;
	/* Call tegra display controller function to update backlight */
	dc = tegra_dc_get_dc(tbl->which_dc);
	if (tbl->params.backlight_mode==MIPI_BACKLIGHT){
		if ((brightness == 0) && tbl->params.dimming_off_cmd) {
			cur = tbl->params.dimming_off_cmd;
			n_cur = tbl->params.n_dimming_off_cmd;
		} else {
			cur = pdata;
			n_cur = ARRAY_SIZE(pdata);
		}
		if (dc)
			tegra_dsi_send_panel_short_cmd(dc, cur, n_cur);
	}
	else if (dc)
		tegra_dc_config_pwm(dc, &tbl->params);
	else
		dev_err(&bl->dev, "tegra display controller not available\n");

	return 0;
}

static int tegra_pwm_backlight_get_brightness(struct backlight_device *bl)
{
	return bl->props.brightness;
}

static int tegra_pwm_backlight_check_fb(struct backlight_device *bl,
					struct fb_info *info)
{
	struct tegra_pwm_bl_data *tbl = dev_get_drvdata(&bl->dev);
	return !tbl->check_fb || tbl->check_fb(tbl->dev, info);
}

static const struct backlight_ops tegra_pwm_backlight_ops = {
	.update_status	= tegra_pwm_backlight_update_status,
	.get_brightness	= tegra_pwm_backlight_get_brightness,
	.check_fb	= tegra_pwm_backlight_check_fb,
};

static int tegra_pwm_backlight_probe(struct platform_device *pdev)
{
	struct backlight_properties props;
	struct platform_tegra_pwm_backlight_data *data;
	struct backlight_device *bl;
	struct tegra_pwm_bl_data *tbl;
	int ret;

	data = pdev->dev.platform_data;
	if (!data) {
		dev_err(&pdev->dev, "failed to find platform data\n");
		return -EINVAL;
	}

	tbl = kzalloc(sizeof(*tbl), GFP_KERNEL);
	if (!tbl) {
		dev_err(&pdev->dev, "no memory for state\n");
		ret = -ENOMEM;
		goto err_alloc;
	}

	tbl->dev = &pdev->dev;
	tbl->which_dc = data->which_dc;
	tbl->notify = data->notify;
	tbl->check_fb = data->check_fb;
	tbl->params.which_pwm = data->which_pwm;
	tbl->params.gpio_conf_to_sfio = data->gpio_conf_to_sfio;
	tbl->params.switch_to_sfio = data->switch_to_sfio;
	tbl->params.period = data->period;
	tbl->params.clk_div = data->clk_div;
	tbl->params.clk_select = data->clk_select;
	tbl->params.backlight_mode = data->backlight_mode;
	tbl->params.dimming_off_cmd = data->dimming_off_cmd;
	tbl->params.n_dimming_off_cmd = data->n_dimming_off_cmd;

	memset(&props, 0, sizeof(struct backlight_properties));
	props.cam_launch_bkl_value = data->cam_launch_bkl_value;
	props.type = BACKLIGHT_RAW;
	props.max_brightness = data->max_brightness;
	bl = backlight_device_register(dev_name(&pdev->dev), &pdev->dev, tbl,
			&tegra_pwm_backlight_ops, &props);
	if (IS_ERR(bl)) {
		dev_err(&pdev->dev, "failed to register backlight\n");
		ret = PTR_ERR(bl);
		goto err_bl;
	}

	if (board_mfg_mode() == BOARD_MFG_MODE_FACTORY2)
		ret = device_create_file(&pdev->dev, &dev_attr_enhance);

	bl->props.bkl_on = 1;
	bl->props.brightness = data->dft_brightness;

	switch(data->backlight_status) {
		case BACKLIGHT_SKIP_WHEN_PROBE:
			break;
		case BACKLIGHT_DISABLE:
			bl->props.bkl_on = 0;
			break;
		case BACKLIGHT_ENABLE:
		default:
			backlight_update_status(bl);
	}

	platform_set_drvdata(pdev, bl);
	return 0;

err_bl:
	kfree(tbl);
err_alloc:
	return ret;
}

static int tegra_pwm_backlight_remove(struct platform_device *pdev)
{
	struct backlight_device *bl = platform_get_drvdata(pdev);
	struct tegra_pwm_bl_data *tbl = dev_get_drvdata(&bl->dev);

	backlight_device_unregister(bl);
	kfree(tbl);
	return 0;
}

static struct platform_driver tegra_pwm_backlight_driver = {
	.driver		= {
		.name	= "tegra-pwm-bl",
		.owner	= THIS_MODULE,
	},
	.probe		= tegra_pwm_backlight_probe,
	.remove		= tegra_pwm_backlight_remove,
};

static int __init tegra_pwm_backlight_init(void)
{
	return platform_driver_register(&tegra_pwm_backlight_driver);
}
late_initcall(tegra_pwm_backlight_init);

static void __exit tegra_pwm_backlight_exit(void)
{
	platform_driver_unregister(&tegra_pwm_backlight_driver);
}
module_exit(tegra_pwm_backlight_exit);

MODULE_DESCRIPTION("Tegra PWM Backlight Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:tegra-pwm-backlight");

