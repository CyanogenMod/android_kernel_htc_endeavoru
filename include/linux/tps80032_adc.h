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
#ifndef _TPS80032_ADC_H_
#define _TPS80032_ADC_H_

/* ioctl define */
#define TPS80032_ADC_IOCTL_MAGIC	0xbc

#define TPS80032_ADC_IOCTL_GET_ADC_ALL \
	_IOR(TPS80032_ADC_IOCTL_MAGIC, 0, struct tps80032_adc_reply)


struct tps80032_adc_reply {
	u32 volt;
	s32 curr;
	s32 power;
};

#define TPS80032_GPADC_FAIL_VALUE	0xFFFF
#define TPS80032_GPADC_FAKE_VALUE	0xFFFFF

struct tps80032_adc_platform_data {
	uint32_t adc_vref;	/* reference voltage */
	int gpadc_rt_irq;
	int gpadc_sw_irq;
	unsigned int calib_bit_map;
};

int32_t tps80032_adc_select_and_read(int32_t *result, u8 channel);
int tps80032_read_vbus_detection(void);
int tps80032_read_opa_mode(void);
int tps80032_read_boots_hw_pwr(void);
#endif
