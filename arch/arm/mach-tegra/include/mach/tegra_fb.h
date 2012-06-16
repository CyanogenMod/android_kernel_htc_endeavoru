/*
 * arch/arm/mach-tegra/include/mach/tegra_fb.h
 *
 * Copyright (C) 2010 Google, Inc.
 * Author: Colin Cross <ccross@android.com>
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

/* Platform data structure to be passed to the driver */
struct tegra_fb_lcd_data {
	int	fb_xres;
	int	fb_yres;
	/* Resolution of the output to the LCD.  If different from the
	   framebuffer resolution, the Tegra display block will scale it */
	int	lcd_xres;
	int	lcd_yres;
	int	bits_per_pixel;
};
#if (defined(CONFIG_USB_FUNCTION_PROJECTOR) || defined(CONFIG_USB_ANDROID_PROJECTOR))
/* For USB Projector to quick access the frame buffer info */
struct tegra_fb_info {
	unsigned char *fb_addr;
	int msmfb_area;
	int xres;
	int yres;
};
extern int tegrafb_get_var(struct tegra_fb_info *tmp);
extern int tegrafb_get_fb_area(void);
#endif

