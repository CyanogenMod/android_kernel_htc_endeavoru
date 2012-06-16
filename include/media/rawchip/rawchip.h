/* Copyright (c) 2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef TEGRA_RAWCHIP_H
#define TEGRA_RAWCHIP_H

#include <linux/delay.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/earlysuspend.h>
#include <linux/wakelock.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/list.h>
#include <linux/cdev.h>
#include <linux/poll.h>
#include <linux/interrupt.h>

#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/device.h>

//#include <mach/camera.h>


#include <mach/gpio.h>
//#include <mach/vreg.h>
#include <asm/mach-types.h>
//#include <mach/board.h>

#include <linux/kernel.h>
#include <linux/string.h>


#include <media/rawchip/linux_rawchip.h>
#include <media/rawchip/Yushan_API.h>

struct rawchip_ctrl {
	struct tegra_camera_rawchip_info *pdata;
};

struct rawchip_sensor_data {
	uint8_t datatype;
	uint8_t lane_cnt;
	uint32_t pixel_clk;
	uint16_t width;
	uint16_t height;
	uint16_t line_length_pclk;
	uint16_t frame_length_lines;
	uint16_t fullsize_width;
	uint16_t fullsize_height;
	uint16_t fullsize_line_length_pclk;
	uint16_t fullsize_frame_length_lines;
	int mirror_flip;
};

int rawchip_vreg_enable(void);
int Yushan_common_init(void);
int rawchip_vreg_disable(void);
void rawchip_release(void);
int rawchip_open_init(void);
int rawchip_set_size(struct rawchip_sensor_data data);
int Yushan_common_deinit(void);
void tegra_rawchip_block_iotcl(bool_t);

//XXX static inline void rawchip_dump_register(void) { Yushan_dump_register(); }

#endif
