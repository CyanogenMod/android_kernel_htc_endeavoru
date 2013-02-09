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

/* for pr_xxx log */
#define PR_TAG "[PMU_ADC]"
#define pr_fmt(fmt) PR_TAG fmt

#include <linux/module.h>
#include <linux/init.h>
#include <linux/mutex.h>
#include <linux/kernel.h>
#include <linux/wakelock.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/mfd/tps80031.h>
#include <linux/tps80032_adc.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/completion.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>


#define FUNC_CALL_CHECK 0

#if FUNC_CALL_CHECK
#define CHECK_LOG() pr_info("%s\n", __func__)
#else
#define CHECK_LOG() (void)0
#endif

#define CHECK_ADC_ENABLED 1
#define ENABLE_CALIB 1

/* GPADC related control register */
#define TPS80032_GPADC_CTRL	0x2E
#define TPS80032_GPADC_CTRL2	0x2F
#define TPS80032_CTRL_P1	0x36

/* GPADC related select register */
#define TPS80032_RTSELECT_LSB	0x32
#define TPS80032_RTSELECT_ISB	0x33
#define TPS80032_RTSELECT_MSB	0x34
#define TPS80032_GPSELECT_ISB	0x35

/* GPADC related result register */
#define TPS80032_RTCH0_LSB	0x37
#define TPS80032_RTCH0_MSB	0x38
#define TPS80032_RTCH1_LSB	0x39
#define TPS80032_RTCH1_MSB	0x3A
#define TPS80032_GPCH0_LSB	0x3B
#define TPS80032_GPCH0_MSB	0x3C

/* GPADC related toggle register */
#define TPS80032_TOGGLE1	0x90

/* GPADC related status register */
#define TPS80032_PWDNSTATUS1	0x93

/* VBUS measurement control register */
#define TPS80032_USB_VBUS_CTRL_SET	0x04
#define TPS80032_USB_VBUS_CTRL_CLR	0x05

/* GPADC TRIMMING related register*/
#define CHANNEL_NUM 19
#define GPADC_TRIM_NUM 19

#define TPS80032_GPADC_TRIM0	0xCC
#define TPS80032_GPADC_TRIM1	0xCD
#define TPS80032_GPADC_TRIM2	0xCE
#define TPS80032_GPADC_TRIM3	0xCF
#define TPS80032_GPADC_TRIM4	0xD0
#define TPS80032_GPADC_TRIM5	0xD1
#define TPS80032_GPADC_TRIM6	0xD2
#define TPS80032_GPADC_TRIM7	0xD3
#define TPS80032_GPADC_TRIM8	0xD4
#define TPS80032_GPADC_TRIM9	0xD5
#define TPS80032_GPADC_TRIM10	0xD6
#define TPS80032_GPADC_TRIM11	0xD7
#define TPS80032_GPADC_TRIM12	0xD8
#define TPS80032_GPADC_TRIM13	0xD9
#define TPS80032_GPADC_TRIM14	0xDA
#define TPS80032_GPADC_TRIM15	0xDB
#define TPS80032_GPADC_TRIM16	0xDC
#define TPS80032_GPADC_TRIM17	0xDD
#define TPS80032_GPADC_TRIM18	0xDE

#define GPADC_TRIM1	1
#define GPADC_TRIM2	2
#define GPADC_TRIM3	3
#define GPADC_TRIM4	4
#define GPADC_TRIM5	5
#define GPADC_TRIM6	6
#define GPADC_TRIM7	7
#define GPADC_TRIM8	8
#define GPADC_TRIM9	9
#define GPADC_TRIM12	12
#define GPADC_TRIM13	13
#define GPADC_TRIM14	14
#define GPADC_NONE	0xFF

#define CH3_TRIM	3
#define CH4_TRIM	4
#define CH5_TRIM	5
#define CH6_TRIM	6
#define CH8_TRIM	8
#define CH10_TRIM	10
#define CH11_TRIM	11
#define CH14_TRIM	14
#define CH15_TRIM	15
#define CH16_TRIM	16
/* tricky: do not use 0 if Channel 0 trimming is not 0 */
#define CH_NONE		0

/* register CTRL_P1 GPADC related bit */
#define CP_SP1_BIT	(1U << 3)
#define CP_ECOCRT_BIT	(1U << 2)
#define CP_ECOCP1_BIT	(1U << 1)
#define CP_BUSY_BIT	(1U << 0)

/* register xxCHx_MSB GPADC related collision bit */
#define COLLISION_RT_BIT	(1U << 4)
#define COLLISION_GP_BIT	(1U << 4)

/* register TOGGLE1 GPADC related bit */
#define GPADC_START_POLARITY_BIT	(1U << 3)
#define GPADC_SAMP_WINDOW_BIT		(1U << 2)
#define GPADCS_BIT			(1U << 1)	/* enable bit */
#define GPADCR_BIT			(1U << 0)	/* reset bit */

/* register PWDNSTATUS1 GPADC related bit */
#define GPADC_EN_BIT	(1U << 0)

/* USB_VBUS_CTRL_SET realted bit */
#define VBUS_MEAS	(1U << 0)

#define CONTROLLER_STAT1	0xE3
#define VBUS_DET	(1U << 2)

#define CHARGERUSB_CTRL1_ADD	0xE8
#define OPA_MODE_EN		(1U << 6)

#define CHARGERUSB_CTRL3_ADD	0xEA
#define BOOST_HW_PWR_EN		(1U << 5)

#define CHANNEL_VAL_SHIFT	10000

static DEFINE_MUTEX(adc_conversion_lock);
struct wake_lock adc_wake_lock;

struct tps80032_adc_data {
	struct device 	*parent_dev;
	uint32_t 	adc_vref;	/* reference voltage */
	int gpadc_rt_irq;
	int gpadc_sw_irq;
	unsigned int calib_bit_map;
};
static struct tps80032_adc_data *adc_data;

static struct calib_eq_params {
	int gain;
	int offset;
} calib_params[CHANNEL_NUM];

static ssize_t tps80032_gpadc_show_attributes(struct device *dev,
					struct device_attribute *attr,
					char *buf);
static bool bAdcReady = 0;

static int tps80032_adc_wait_conversion_ready_locked(
				unsigned int timeout_ms)
{
	unsigned long timeout;
	int ret;
	u8 reg_value[3];

	CHECK_LOG();

	timeout = jiffies + msecs_to_jiffies(timeout_ms);
	do {
		ret = tps80031_read(adc_data->parent_dev, SLAVE_ID2,
					TPS80032_CTRL_P1, &reg_value[0]);
		if (ret < 0) {
			pr_err("%s:read CTRL_P1 fail\n", __func__);
			return ret;
		}

		if (!(reg_value[0] & CP_BUSY_BIT)
			&& (reg_value[0] & CP_ECOCP1_BIT))
			return 0;
	} while(!time_after(jiffies, timeout));

	ret = tps80031_read(adc_data->parent_dev, SLAVE_ID2,
			TPS80032_GPSELECT_ISB, &reg_value[1]);
	if (ret < 0) {
		pr_err("%s:read TPS80032_GPSELECT_ISB fail\n", __func__);
		return ret;
	}

	ret = tps80031_read(adc_data->parent_dev, SLAVE_ID2,
			TPS80032_GPCH0_MSB, &reg_value[2]);
	if (ret < 0) {
		pr_err("%s:read register GPCH0_MSB fail\n", __func__);
		return ret;
	}

	pr_info("adc conversion timeout: CTRL_P1:0x%X, GPSELECT:0x%X GPCH0_MSB:0x%X\n"
			, reg_value[0], reg_value[1], reg_value[2]);

	return -EAGAIN;
}

int32_t tps80032_adc_select_and_read(int32_t *result, u8 channel)
{
	u8 read_reg_value;
	int temp;
	int ret, i;

	CHECK_LOG();

	if (!bAdcReady) {
		*result = TPS80032_GPADC_FAIL_VALUE;
		return -ENODEV;
	}
	if (channel >= CHANNEL_NUM) {
		pr_err("%s:no such channel %u for gpadc\n", __func__, channel);
		ret = -ENODEV;
		goto no_channel_adc_fail;
	}

	mutex_lock(&adc_conversion_lock);

		if (channel == 10) {
			ret = tps80031_read(adc_data->parent_dev, SLAVE_ID2,
					TPS80032_USB_VBUS_CTRL_SET, &read_reg_value);
			if (ret < 0) {
				pr_warn("%s:read register USB_VBUS_CTRL_SET fail\n"
						, __func__);
				goto vbus_mea_enable_fail;
			}

			read_reg_value |= VBUS_MEAS;

			ret = tps80031_write(adc_data->parent_dev, SLAVE_ID2,
					TPS80032_USB_VBUS_CTRL_SET, read_reg_value);
			if (ret < 0) {
				pr_warn("%s:write register USB_VBUS_CTRL_SET fail\n"
						, __func__);
				goto vbus_mea_enable_fail;
			}
		}

		ret = tps80031_write(adc_data->parent_dev, SLAVE_ID2,
				TPS80032_GPSELECT_ISB, channel);
		if (ret < 0) {
			pr_err("%s:write register GPSELECT_ISB fail\n"
					, __func__);
			goto write_i2c_adc_fail;
		}

		ret = tps80031_write(adc_data->parent_dev, SLAVE_ID2,
				TPS80032_CTRL_P1, CP_SP1_BIT);
		if (ret < 0) {
			pr_err("%s:write register CTRL_P1 fail\n"
					, __func__);
			goto write_i2c_adc_fail;
		}

		i = 0;
		do {
			ret = tps80032_adc_wait_conversion_ready_locked(5);
		} while (ret != 0 && ++i < 5);

		if (ret < 0) {
			pr_err("%s:conversion fail or timeout\n"
					, __func__);
			goto conversion_fail;
		}

		ret = tps80031_read(adc_data->parent_dev, SLAVE_ID2,
				TPS80032_GPCH0_MSB, &read_reg_value);
		if (ret < 0) {
			pr_err("%s:read register GPCH0_MSB fail\n"
					, __func__);
			goto read_adc_fail;
		}

		if (read_reg_value & COLLISION_GP_BIT) {
			pr_err("%s:read register GPCH0_MSB encounter"
					" collision!!\n", __func__);
			ret = -EBUSY;
			goto read_adc_fail;
		}

		temp = (read_reg_value & 0x0F) << 8;
		ret = tps80031_read(adc_data->parent_dev, SLAVE_ID2,
				TPS80032_GPCH0_LSB, &read_reg_value);
		if (ret < 0) {
			pr_err("%s:read register GPCH0_LSB fail\n"
						, __func__);
			goto read_adc_fail;
		}
		temp += read_reg_value;

#if ENABLE_CALIB
		if (!!(adc_data->calib_bit_map & (0x1 << channel)))
			temp = (temp * CHANNEL_VAL_SHIFT - calib_params[channel].offset)
					/ calib_params[channel].gain;
#endif

		if (channel == 10) {
			ret = tps80031_read(adc_data->parent_dev, SLAVE_ID2,
					TPS80032_USB_VBUS_CTRL_CLR, &read_reg_value);
			if (ret < 0) {
				pr_warn("%s:read register USB_VBUS_CTRL_SET fail\n"
						, __func__);
				goto vbus_mea_disable_fail;
			}

			read_reg_value |= VBUS_MEAS;

			ret = tps80031_write(adc_data->parent_dev, SLAVE_ID2,
					TPS80032_USB_VBUS_CTRL_CLR, read_reg_value);
			if (ret < 0) {
				pr_warn("%s:write register USB_VBUS_CTRL_SET fail\n"
						, __func__);
			}
		}

vbus_mea_disable_fail:
	mutex_unlock(&adc_conversion_lock);

	if (temp > 4095)
		temp = 4095;
	else if (temp < 0)
		temp = 0;

	*result = temp;

	return ret;
no_channel_adc_fail:
	*result = TPS80032_GPADC_FAIL_VALUE;
	return ret;
conversion_fail:
read_adc_fail:
write_i2c_adc_fail:
	*result = TPS80032_GPADC_FAIL_VALUE;
	mutex_unlock(&adc_conversion_lock);
	pr_err("%s:adc write or read fail at channel %u\n"
					, __func__, channel);
	return ret;
vbus_mea_enable_fail:
	*result = TPS80032_GPADC_FAIL_VALUE;
	mutex_unlock(&adc_conversion_lock);
	pr_warn("%s:adc write or read fail at channel 10\n"
					, __func__);
	return ret;
}
EXPORT_SYMBOL_GPL(tps80032_adc_select_and_read);

int tps80032_read_vbus_detection(void)
{
	uint8_t ctrl0;
	int ret;

	ret = tps80031_read(adc_data->parent_dev, SLAVE_ID2,
			CONTROLLER_STAT1, &ctrl0);
	if (ret < 0) {
		pr_err("%s:read register CONTROLLER_STAT1 fail\n",__func__);
		return ret;
	}

	if ((ctrl0 & VBUS_DET))
		return 1;
	return 0;
}
EXPORT_SYMBOL_GPL(tps80032_read_vbus_detection);

int tps80032_read_opa_mode(void)
{
	uint8_t ctrl0;
	int ret;

	ret = tps80031_read(adc_data->parent_dev, SLAVE_ID2,
			CHARGERUSB_CTRL1_ADD, &ctrl0);
	if (ret < 0) {
		pr_err("%s:read register CHARGERUSB_CTRL1_ADD fail\n",__func__);
		return ret;
	}
	if (ctrl0 & OPA_MODE_EN)
		return 1;
	return 0;
}
EXPORT_SYMBOL_GPL(tps80032_read_opa_mode);

int tps80032_read_boots_hw_pwr(void)
{
	uint8_t ctrl0;
	int ret;

	ret = tps80031_read(adc_data->parent_dev, SLAVE_ID2,
			CHARGERUSB_CTRL3_ADD, &ctrl0);
	if (ret < 0) {
		pr_err("%s:read register CHARGERUSB_CTRL1_ADD fail\n",__func__);
		return ret;
	}
	if (ctrl0 & BOOST_HW_PWR_EN)
		return 1;
	return 0;
}
EXPORT_SYMBOL_GPL(tps80032_read_boots_hw_pwr);

static struct device_attribute tps80032_gpadc_attrs[] = {
	__ATTR(gpadc_0, S_IRUGO, tps80032_gpadc_show_attributes, NULL),
	__ATTR(gpadc_1, S_IRUGO, tps80032_gpadc_show_attributes, NULL),
	__ATTR(gpadc_2, S_IRUGO, tps80032_gpadc_show_attributes, NULL),
	__ATTR(gpadc_3, S_IRUGO, tps80032_gpadc_show_attributes, NULL),
	__ATTR(gpadc_4, S_IRUGO, tps80032_gpadc_show_attributes, NULL),
	__ATTR(gpadc_5, S_IRUGO, tps80032_gpadc_show_attributes, NULL),
	__ATTR(gpadc_6, S_IRUGO, tps80032_gpadc_show_attributes, NULL),
	__ATTR(gpadc_7, S_IRUGO, tps80032_gpadc_show_attributes, NULL),
	__ATTR(gpadc_8, S_IRUGO, tps80032_gpadc_show_attributes, NULL),
	__ATTR(gpadc_9, S_IRUGO, tps80032_gpadc_show_attributes, NULL),
	__ATTR(gpadc_10, S_IRUGO, tps80032_gpadc_show_attributes, NULL),
	__ATTR(gpadc_11, S_IRUGO, tps80032_gpadc_show_attributes, NULL),
	__ATTR(gpadc_12, S_IRUGO, tps80032_gpadc_show_attributes, NULL),
	__ATTR(gpadc_13, S_IRUGO, tps80032_gpadc_show_attributes, NULL),
	__ATTR(gpadc_14, S_IRUGO, tps80032_gpadc_show_attributes, NULL),
	__ATTR(gpadc_15, S_IRUGO, tps80032_gpadc_show_attributes, NULL),
	__ATTR(gpadc_16, S_IRUGO, tps80032_gpadc_show_attributes, NULL),
	__ATTR(gpadc_17, S_IRUGO, tps80032_gpadc_show_attributes, NULL),
	__ATTR(gpadc_18, S_IRUGO, tps80032_gpadc_show_attributes, NULL),
};

static ssize_t tps80032_gpadc_show_attributes(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	int i = 0;
	int32_t result;
	const ptrdiff_t off = attr - tps80032_gpadc_attrs;

	if (off < sizeof(tps80032_gpadc_attrs)
			/ sizeof(struct device_attribute)) {
		tps80032_adc_select_and_read(&result, off);
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
				result);
	} else
		i = -EINVAL;

	if (i < 0)
		pr_err("%s: attribute is not supported: %d\n",
			__func__, off);

	return i;
}

static int tps80032_gpadc_create_attrs(struct device *dev)
{
	int i = 0, rc = 0;

	for (i = 0; i < ARRAY_SIZE(tps80032_gpadc_attrs); i++) {
		rc = device_create_file(dev, &tps80032_gpadc_attrs[i]);
		if (rc)
			goto gpadc_attrs_failed;
	}

	goto succeed;

gpadc_attrs_failed:
	while (i--)
		device_remove_file(dev, &tps80032_gpadc_attrs[i]);
succeed:
	return rc;
}

#if ENABLE_CALIB

static int calculate_calib_trim(unsigned int trim1, unsigned int mask1, unsigned int shift1, unsigned int trim2, unsigned int mask2)
{
	unsigned int result = ((trim1 & mask1) << shift1);
	if (trim2 != GPADC_NONE) {
		result += ((trim2 & mask2) >> 1);
		result *= CHANNEL_VAL_SHIFT;
		if (!!(trim2 & 0x01))
			return 0 - ((int) result);
	} else {
		result >>= 1;
		result *= CHANNEL_VAL_SHIFT;
		if (!!(trim1 & 0x01))
			return 0 - ((int) result);
	}
	return ((int) result);
}

static int calculate_rest_trim(int ch_trim3, int ch_trim4)
{
	int i;
	int ret;
	int temp;
	int d1, d2;
	int ch_trim[CHANNEL_NUM];
	u8 gpadc_trim[GPADC_TRIM_NUM];

	const int trim_params[CHANNEL_NUM][4] = {
		/* calib trim 0 */
		{    0, 0,   GPADC_NONE,    0},
		{    0, 0,   GPADC_NONE,    0},
		{    0, 0,   GPADC_NONE,    0},
		{ 0x1F, 2,  GPADC_TRIM1, 0x06},
		{ 0x3F, 2,  GPADC_TRIM2, 0x06},
		/* calib trim 5 */
		{ 0x7E, 0,   GPADC_NONE,    0},
		{ 0xFE, 0,   GPADC_NONE,    0},
		{    0, 0,   GPADC_NONE,    0},
		{ 0x18, 1,  GPADC_TRIM7, 0x1E},
		{    0, 0,   GPADC_NONE,    0},
		/* calib trim 10 */
		{ 0x1F, 2,  GPADC_TRIM8, 0x06},
		{ 0x0F, 3,  GPADC_TRIM9, 0x0E},
		{    0, 0,   GPADC_NONE,    0},
		{    0, 0,   GPADC_NONE,    0},
		{ 0x18, 1, GPADC_TRIM12, 0x1E},
		/* calib trim 15 */
		{ 0x0F, 3, GPADC_TRIM13, 0x0E},
		{ 0x1F, 2, GPADC_TRIM14, 0x06},
		{    0, 0,   GPADC_NONE,    0},
		{    0, 0,   GPADC_NONE,    0},
	};

	/* paramerter of calibration gain and offset */
	const int ch_params[CHANNEL_NUM][6] = {
		/* x1    x2     D1_1st     D1_2nd     D2_1st     D2_2nd */
		/* channel 0*/
		{1441, 3276,  CH3_TRIM,   CH_NONE,  CH4_TRIM,   CH_NONE},
		{1441, 3276,  CH3_TRIM,   CH_NONE,  CH4_TRIM,   CH_NONE},
		{1441, 3276,  CH3_TRIM,   CH_NONE,  CH4_TRIM,   CH_NONE},
		{1441, 3276,  CH3_TRIM,   CH_NONE,  CH4_TRIM,   CH_NONE},
		{1441, 3276,  CH3_TRIM,   CH_NONE,  CH4_TRIM,   CH_NONE},
		/* channel 5 */
		{1441, 3276,  CH3_TRIM,   CH_NONE,  CH4_TRIM,   CH_NONE},
		{1441, 3276,  CH3_TRIM,   CH_NONE,  CH4_TRIM,   CH_NONE},
		{1441, 3276,  CH3_TRIM,  CH5_TRIM,  CH4_TRIM,  CH6_TRIM},
		{1441, 3276,  CH3_TRIM,  CH8_TRIM,  CH4_TRIM, CH10_TRIM},
		{1441, 3276,  CH3_TRIM, CH14_TRIM,  CH4_TRIM, CH16_TRIM},
		/* channel 10*/
		{ 149,  745, CH11_TRIM,   CH_NONE, CH15_TRIM,   CH_NONE},
		{1441, 3276,  CH3_TRIM,   CH_NONE,  CH4_TRIM,   CH_NONE},
		{1441, 3276,  CH3_TRIM,   CH_NONE,  CH4_TRIM,   CH_NONE},
		{1441, 3276,  CH3_TRIM,   CH_NONE,  CH4_TRIM,   CH_NONE},
		{1441, 3276,  CH3_TRIM,   CH_NONE,  CH4_TRIM,   CH_NONE},
		/* channel 15*/
		{   0,    0,   CH_NONE,   CH_NONE,   CH_NONE,   CH_NONE},
		{   0,    0,   CH_NONE,   CH_NONE,   CH_NONE,   CH_NONE},
		{   0,    0,   CH_NONE,   CH_NONE,   CH_NONE,   CH_NONE},
		{1441, 3276,  CH3_TRIM,  CH5_TRIM,  CH4_TRIM,  CH6_TRIM},
	};

	memset(ch_trim, 0, sizeof(ch_trim));

	ch_trim[3] = ch_trim3;
	ch_trim[4] = ch_trim4;

	for (i = 5; i < GPADC_TRIM_NUM; i++) {
		ret = tps80031_read(adc_data->parent_dev, SLAVE_ID3,
				TPS80032_GPADC_TRIM0 + i, &gpadc_trim[i]);
		if (ret < 0) {
			pr_err("%s:Fail to read register "
				"TPS80032_GPADC_TRIM%d\n", __func__, i);
			adc_data->calib_bit_map = 0;
			return ret;
		}

		if (trim_params[i][0] != 0) {
			temp = (trim_params[i][2] != GPADC_NONE ?
				gpadc_trim[trim_params[i][2]] : GPADC_NONE);
			ch_trim[i] = calculate_calib_trim(gpadc_trim[i]
							, trim_params[i][0]
							, trim_params[i][1]
							, temp
							, trim_params[i][3]);
		}
	}

	for (i = 7; i < CHANNEL_NUM; i++) {
		if (~adc_data->calib_bit_map & (0x1 << i))
			continue;

		d1 = ch_trim[ch_params[i][2]] + ch_trim[ch_params[i][3]];
		d2 = ch_trim[ch_params[i][4]] + ch_trim[ch_params[i][5]];

		temp = ((d2 - d1) / (ch_params[i][1] - ch_params[i][0]));
		calib_params[i].gain = 1 * CHANNEL_VAL_SHIFT + temp;
		calib_params[i].offset = d1 - (temp * ch_params[i][0]);
	}

	return 0;
}
#endif

static int __devinit tps80032_adc_probe(struct platform_device *pdev)
{
	struct tps80032_adc_platform_data *pdata = pdev->dev.platform_data;
	int ret;
#if CHECK_ADC_ENABLED
	u8 read_reg_value;
#endif
#if ENABLE_CALIB
	int i, temp;
	struct calib_eq_params temp_params;
	u8 gpadc_trim[4];
	int ch_trim3, ch_trim4;
#endif

	CHECK_LOG();

	adc_data = kzalloc(sizeof(struct tps80032_adc_data), GFP_KERNEL);
	if (!adc_data) {
		pr_err("%s:Unable to allocat memory!!\n", __func__);
		return -ENOMEM;
	}
	
	adc_data->parent_dev = pdev->dev.parent;	
	adc_data->adc_vref = pdata->adc_vref;	
	adc_data->gpadc_rt_irq = pdata->gpadc_rt_irq;
	adc_data->gpadc_sw_irq = pdata->gpadc_sw_irq;
#if ENABLE_CALIB
	adc_data->calib_bit_map = pdata->calib_bit_map;
#endif

	/* disable current source to GPADC_IN0 and 
	 * enable 1.875/1.25 V divider on GPADC channel 2
	 */
	ret = tps80031_write(adc_data->parent_dev, SLAVE_ID2,
			TPS80032_GPADC_CTRL, 0x04);
	if (ret < 0) {
		pr_err("%s:Fail to write register GPADC_CTRL\n", __func__);
		goto adc_rw_fail;
	}

	/* select 0 uA current source for GPADC_IN3 */
	ret = tps80031_write(adc_data->parent_dev, SLAVE_ID2,
			TPS80032_GPADC_CTRL2, 0x00);
	if (ret < 0) {
		pr_err("%s:Fail to write register GPADC_CTRL2\n"
				, __func__);
	}
	
	ret = tps80031_write(adc_data->parent_dev, SLAVE_ID2,
			TPS80032_TOGGLE1, GPADCS_BIT);
	if (ret < 0) {
		pr_err("%s:Fail to write register TOGGLE1"
			" to enable GPADC\n", __func__);
		goto adc_rw_fail;
	}

#if CHECK_ADC_ENABLED
	ret = tps80031_read(adc_data->parent_dev, SLAVE_ID2,
			TPS80032_PWDNSTATUS1, &read_reg_value);
	if (ret < 0) {
		pr_err("%s:read register RTCH0_LSB fail\n", __func__);
		goto adc_rw_fail;
	}

#if ENABLE_CALIB
	if (read_reg_value & GPADC_EN_BIT)
		pr_info("%s:GPADC eanbled, then check calibration with map 0x%x"
				, __func__, adc_data->calib_bit_map);
#else
	if (read_reg_value & GPADC_EN_BIT)
		pr_info("%s:GPADC eanbled\n", __func__);
#endif
	else {
		pr_err("%s:GPADC eanble fail!! value:%x\n"
				, __func__, read_reg_value);
		return -ENODEV;
	}
#endif	/* if CHECK_ADC_ENABLED */

#if ENABLE_CALIB
	/*
	 * fixme: here we hard code for the first 7 channel for performance
	 *        modify it if parameter differs.
	 *        here uses the channel trim 1, 2, 3 and 4.
	 */
	for (i = 0; i < 4; i++) {
		ret = tps80031_read(adc_data->parent_dev, SLAVE_ID3,
				TPS80032_GPADC_TRIM1 + i, &gpadc_trim[i]);
		if (ret < 0) {
			pr_err("%s:Fail to read register "
				"TPS80032_GPADC_TRIM%d\n", __func__, i);
			adc_data->calib_bit_map = 0;
			goto trim_read_fail;
		}
	}

	ch_trim3 = ((gpadc_trim[2] & 0x1F) << 2) + ((gpadc_trim[0] & 0x6) >> 1);
	ch_trim3 *= CHANNEL_VAL_SHIFT;
	if (!!(gpadc_trim[0] & 0x1))
		ch_trim3 = 0 - ch_trim3;

	ch_trim4 = ((gpadc_trim[3] & 0x3F) << 2) + ((gpadc_trim[1] & 0x6) >> 1);
	ch_trim4 *= CHANNEL_VAL_SHIFT;
	if (!!(gpadc_trim[1] & 0x1))
		ch_trim4 = 0 - ch_trim4;

	temp = (ch_trim4 - ch_trim3) / (3276 - 1441);
	temp_params.gain = 1 * CHANNEL_VAL_SHIFT + temp;
	temp_params.offset = ch_trim3 - (temp * 1441);

	for (i = 0; i < 7; i++) {
		if (adc_data->calib_bit_map & (1U << i)) {
			calib_params[i].gain = temp_params.gain;
			calib_params[i].offset = temp_params.offset;
		}
	}

	/* calculate the rest channels if in bit map */
	if (adc_data->calib_bit_map & 0x7FF80)
		ret = calculate_rest_trim(ch_trim3, ch_trim4);

trim_read_fail:
#endif
	tps80032_gpadc_create_attrs(&pdev->dev);
	bAdcReady = 1;
	return 0;
adc_rw_fail:
	return ret;
}

static int __devexit tps80032_adc_teardown(struct platform_device *pdev)
{
	CHECK_LOG();

	kfree(adc_data);
	return 0;
}

#ifdef CONFIG_PM
static int tps80032_adc_suspend(struct platform_device *pdev,
				pm_message_t state)
{
	CHECK_LOG();

	return 0;
}

static int tps80032_adc_resume(struct platform_device *pdev)
{
	CHECK_LOG();

	return 0;
}
#endif

static struct platform_driver tps80032_adc_driver = {
	.probe = tps80032_adc_probe,
	.remove = __devexit_p(tps80032_adc_teardown),
#ifdef CONFIG_PM
	.suspend = tps80032_adc_suspend,
	.resume = tps80032_adc_resume,
#endif
	.driver = {
		.name = "tps80032-adc",
		.owner = THIS_MODULE,
	},
};

static int __init tps80032_adc_init(void)
{
	CHECK_LOG();

	kfree(adc_data);
	wake_lock_init(&adc_wake_lock, WAKE_LOCK_SUSPEND,
			"tps80032_adc_wake");
	
	return platform_driver_register(&tps80032_adc_driver);
}
fs_initcall(tps80032_adc_init);

static void __exit tps80032_adc_exit(void)
{
	platform_driver_unregister(&tps80032_adc_driver);
}
module_exit(tps80032_adc_exit);

MODULE_DESCRIPTION("TPS80032 adc driver");
MODULE_LICENSE("GPL");
