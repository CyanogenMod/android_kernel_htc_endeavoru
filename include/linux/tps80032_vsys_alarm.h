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
#ifndef _TPS80032_VSYS_ALARM_H_
#define _TPS80032_VSYS_ALARM_H_

struct tps80032_vsys_alarm_platform_data {
	int vsys_low_irq;
};

int tps80032_vsys_alarm_state_set(unsigned int state);
int tps80032_vsys_alarm_threshold_set(unsigned int threshold);
int tps80032_vsys_alarm_register_notifier(struct notifier_block *nb);
#endif
