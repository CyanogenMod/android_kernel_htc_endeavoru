/*
 * include/video/tegrafb.h
 *
 * Copyright (C) 2010 Google, Inc.
 * Author: Erik Gilling <konkers@android.com>
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

#ifndef _LINUX_TEGRAFB_H_
#define _LINUX_TEGRAFB_H_

#include <linux/fb.h>
#include <linux/types.h>
#include <linux/ioctl.h>

struct tegra_fb_modedb {
	struct fb_var_screeninfo *modedb;
	__u32 modedb_len;
};

#define FBIO_TEGRA_GET_MODEDB	_IOWR('F', 0x42, struct tegra_fb_modedb)
#define FBIO_TEGRA_GET_USB_PROJECTOR_INFO _IOR('F', 301, struct tegra_usb_projector_info)
#define FBIO_TEGRA_SET_USB_PROJECTOR_INFO _IOW('F', 302, struct tegra_usb_projector_info)
struct  tegra_usb_projector_info {
       int usb_offset;
       int latest_offset;
};

#endif
