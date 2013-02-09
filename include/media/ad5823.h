/*
 * Copyright (C) 2010-2011 NVIDIA Corporation.
 *
 * Contributors:
 *      Sachin Nikam <snikam@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */

#ifndef __AD5823_H__
#define __AD5823_H__

#include <linux/ioctl.h>  /* For IOCTL macros */

#define AD5823_IOCTL_GET_CONFIG   _IOR('o', 1, struct ad5823_config)
#define AD5823_IOCTL_SET_POSITION _IOW('o', 2, u32)
//HTC_start: for g-sensor
#define AD5823_IOCTL_SET_GSENSOR_MODE _IOW('o', 3, u8)
#define AD5823_IOCTL_GET_GSENSOR_DATA _IOW('o', 4, struct g_sensor_info)
#define AD5823_IOCTL_GET_POWER_STATE _IOR('o', 5, u32)
#define AD5823_IOCTL_SET_FLASHLIGHT _IOR('o', 6, u32)
//HTC_end

//HTC_start: for g-sensor
struct g_sensor_info {
    __s16 x;
    __s16 y;
    __s16 z;
};
//HTC_end

struct ad5823_config {
	__u32 settle_time;
	__u32 actuator_range;
	__u32 pos_low;
	__u32 pos_high;
	float focal_length;
	float fnumber;
	float max_aperture;
};

#ifdef __KERNEL__
struct ad5823_platform_data {
	float focal_length;
	float fnumber;
	__u32 settle_time;
	__u32 pos_low;
	__u32 pos_high;
	int (*get_power_state)(void);
	int (*set_gsensor_mode)(unsigned char);
	int (*get_gsensor_data)(short *);
	int (*set_flashlight)(int);
};
#endif /* __KERNEL__ */

#endif  /* __AD5823_H__ */
