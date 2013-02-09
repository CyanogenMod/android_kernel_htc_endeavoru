/*
 * Copyright (C) 2011 HTC Incorporated
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
#ifndef _TPS80032_GAUGE_H_
#define _TPS80032_GAUGE_H_

/* ioctl define */
#define TPS80032_GAUGE_IOCTL_MAGIC	0xbf

#define TPS80032_GAUGE_IOCTL_RESET \
	_IOW(TPS80032_GAUGE_IOCTL_MAGIC, 0, enum update_rate)

#define TPS80032_GAUGE_IOCTL_SET_CLB \
	_IOW(TPS80032_GAUGE_IOCTL_MAGIC, 1, unsigned int)

#define TPS80032_GAUGE_IOCTL_GET_ALL \
	_IOR(TPS80032_GAUGE_IOCTL_MAGIC, 2, struct tps80032_gauge_reply)

#define TPS80032_GAUGE_IOCTL_SET_COUNTER \
	_IOW(TPS80032_GAUGE_IOCTL_MAGIC, 4, struct tps80032_gauge_counter)

#define TPS80032_GAUGE_IOCTL_GET_COUNTER \
	_IOR(TPS80032_GAUGE_IOCTL_MAGIC, 5, struct tps80032_gauge_counter)

#define TPS80032_GAUGE_IOCTL_SET_CAPACITY \
	_IOW(TPS80032_GAUGE_IOCTL_MAGIC, 6, unsigned int)

#define TPS80032_GAUGE_IOCTL_GET_DEBUG \
	_IOR(TPS80032_GAUGE_IOCTL_MAGIC, 7, struct tps80032_gauge_debug)

#define TPS80032_GAUGE_IOCTL_SET_OFFSET_SHIFT \
	_IOW(TPS80032_GAUGE_IOCTL_MAGIC, 8, int)

#define TPS80032_GAUGE_DEV "/dev/tps80032_gauge"

enum update_rate {
	UR250	= 0x0,
	UR62P5	= 0x1,
	UR15P6	= 0x2,
	UR3P9	= 0x3,
	URSTOP,
};

struct tps80032_gauge_reply {
	u32 pass_time;
	s32 accum_current;
	u32 batt_volt;
};

struct tps80032_gauge_platform_data {
	int irq;
	int adc_volt_channel;
	int adc_temp_channel;
};

struct tps80032_gauge_counter {
	s32 pass_time;
	s32 accum_current;
};

struct tps80032_gauge_debug {
	s32 offset;
	s32 offset_shift;
	s32 v_fs;
	u32 reserve[2];
};

#endif
