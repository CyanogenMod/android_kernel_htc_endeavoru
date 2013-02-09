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
#define PR_TAG "[PMU_GAU]"
#define pr_fmt(fmt) PR_TAG fmt

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/wakelock.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/mfd/tps80031.h>
#include <linux/tps80032_gauge.h>
#include <linux/tps80032_adc.h>
#include <linux/workqueue.h>
#include <asm/div64.h>

#define FUNC_CALL_CHECK 0
#define GAUGE_DEBUG 0

#if FUNC_CALL_CHECK
#define CHECK_LOG() pr_info("%s\n", __func__)
#else
#define CHECK_LOG() (void)0
#endif

/* gauge related toggle register */
#define TPS80032_TOGGLE1	0x90
#define TPS80032_FG_REG_00	0xC0
#define TPS80032_FG_REG_01	0xC1
#define TPS80032_FG_REG_02	0xC2
#define TPS80032_FG_REG_03	0xC3
#define TPS80032_FG_REG_04	0xC4
#define TPS80032_FG_REG_05	0xC5
#define TPS80032_FG_REG_06	0xC6
#define TPS80032_FG_REG_07	0xC7
#define TPS80032_FG_REG_08	0xC8
#define TPS80032_FG_REG_09	0xC9
#define TPS80032_FG_REG_10	0xCA
#define TPS80032_FG_REG_11	0xCB
#define TPS80032_FG_REG_12	0xCC
#define TPS80032_FG_REG_13	0xCD

#define FGDITHS	BIT(7)
#define FGDITHR	BIT(6)
#define FGS	BIT(5)
#define FGR	BIT(4)

#define CC_ACTIVE_MODE_1	BIT(7)
#define CC_ACTIVE_MODE_0	BIT(6)
#define CC_AUTOCLEAR		BIT(2)
#define CC_CAL_EN		BIT(1)
#define CC_PAUSE		BIT(0)

#define V_FS	(60)
#define R_S	(10)
#define F_S_BIT	(15)
#define F_S	(1 << F_S_BIT)
#define AMPLIFIER	(100)


struct tps80032_gauge_data {
	struct device 	*parent_dev;
	int irq;
	int adc_volt_channel;
	int adc_temp_channel;

	spinlock_t gauge_lock;
	int is_open;

	struct mutex data_lock;
	enum update_rate rate_select;

	unsigned int clb_time;

	int cc_offset;
	unsigned int cc_sample_cntr[2];
	int cc_accum[2];
	int cc_active_mode;

	int offset_shift;
};
static struct tps80032_gauge_data *gauge_data;

static struct wake_lock gauge_clb_lock;

static void clb_work_func(struct work_struct *work);
static DECLARE_DELAYED_WORK(clb_work, clb_work_func);
static struct workqueue_struct *clb_wq;

static void cc_autocal_done_func(struct work_struct *work);
static DECLARE_WORK(cc_autocal_done_work
		    , cc_autocal_done_func);

static int do_gauge_calibration(void)
{
	int ret = 0;
	u8 value;

	CHECK_LOG();

	mutex_lock(&gauge_data->data_lock);
	ret = tps80031_read(gauge_data->parent_dev, SLAVE_ID2,
			TPS80032_FG_REG_00, &value);
	if (ret < 0) {
		pr_err("%s:read register TPS80032_FG_REG_00 fail\n"
			, __func__);
		goto gauge_fail;
	}

	ret = tps80031_write(gauge_data->parent_dev, SLAVE_ID2,
			TPS80032_FG_REG_00, value | CC_CAL_EN);
	if (ret < 0) {
		pr_err("%s:write register TPS80032_FG_REG_00"
				" fail value %lu\n"
				, __func__, value | CC_CAL_EN);
		goto gauge_fail;
	}

gauge_fail:
	mutex_unlock(&gauge_data->data_lock);
	return ret;
}

static void cc_autocal_done_func(struct work_struct *work)
{
	int ret;
	u8 value[2];

	CHECK_LOG();

	mutex_lock(&gauge_data->data_lock);
	ret = tps80031_read(gauge_data->parent_dev, SLAVE_ID2,
					TPS80032_FG_REG_08, value);
	if (ret < 0) {
		pr_err("%s:read register TPS80032_FG_REG_08 fail\n"
			, __func__);
		goto read_gauge_fail;
	}

	ret = tps80031_read(gauge_data->parent_dev, SLAVE_ID2,
					TPS80032_FG_REG_09, value + 1);
	if (ret < 0) {
		pr_err("%s:read register TPS80032_FG_REG_09 fail\n"
			, __func__);
		goto read_gauge_fail;
	}
	gauge_data->cc_offset =
		((value[1] & 0x03) << 8) + (value[0] & 0xFF);

	if ((gauge_data->cc_offset & 0x200) !=0)
		gauge_data->cc_offset = gauge_data->cc_offset | (~0x3FF);

	mutex_unlock(&gauge_data->data_lock);

	return;
read_gauge_fail:
	gauge_data->cc_offset = 0;
	mutex_unlock(&gauge_data->data_lock);
}

static irqreturn_t cc_autocal_handler(int irq, void *data)
{
	CHECK_LOG();

	queue_work(clb_wq, &cc_autocal_done_work);

	return IRQ_HANDLED;
}

static void clb_work_func(struct work_struct *work)
{
	CHECK_LOG();

	if (gauge_data->rate_select == URSTOP)
		return;

	do_gauge_calibration();
	queue_delayed_work(clb_wq, &clb_work
			, msecs_to_jiffies(gauge_data->clb_time));
}

static int reset_gauge(enum update_rate ur_select)
{
	int ret = 0;
	u8 value;

	CHECK_LOG();

	if (ur_select > URSTOP) {
		pr_err("%s: No such gauge update selections"
				" value=%d\n"
				, __func__, ur_select);
		ret = -EINVAL;
		goto gauge_fail;
	}

	gauge_data->rate_select = ur_select;
	gauge_data->cc_offset = 0;
	gauge_data->cc_sample_cntr[0] = 0;
	gauge_data->cc_accum[0] = 0;

	if (ur_select != URSTOP) {
		value = FGS | FGDITHS;
		ret = tps80031_write(gauge_data->parent_dev, SLAVE_ID2,
				TPS80032_TOGGLE1, value);
		if (ret < 0) {
			pr_err("%s:write register TPS80032_TOGGLE1"
					" fail value %d\n"
					, __func__, value);
			goto gauge_fail;
		}
	} else
		ur_select = UR250;

	value = (((unsigned int) ur_select) << 6)
		| CC_PAUSE;
	ret = tps80031_write(gauge_data->parent_dev, SLAVE_ID2,
			TPS80032_FG_REG_00, value);
	if (ret < 0) {
		pr_err("%s:write register TPS80032_FG_REG_00"
			" fail value %u\n"
			, __func__, value);
		goto gauge_fail;
	}

	value = (((unsigned int) ur_select) << 6)
		| CC_AUTOCLEAR;
	ret = tps80031_write(gauge_data->parent_dev, SLAVE_ID2,
			TPS80032_FG_REG_00, value);
	if (ret < 0) {
		pr_err("%s:write register TPS80032_FG_REG_00"
			" fail value %u\n"
			, __func__, value);
		goto gauge_fail;
	}

gauge_fail:
	return ret;
}

static void set_calibration_timer(unsigned int clb_time_ms)
{
	CHECK_LOG();

	cancel_delayed_work_sync(&clb_work);

	mutex_lock(&gauge_data->data_lock);
	if (gauge_data->rate_select == URSTOP)
		clb_time_ms = 0;
	else if (clb_time_ms > 1 && clb_time_ms < 3000)
		clb_time_ms = 3000;
	gauge_data->clb_time = clb_time_ms;
	mutex_unlock(&gauge_data->data_lock);

	if (clb_time_ms == 0)
		return;

	do_gauge_calibration();
	if (clb_time_ms != 1) {
		queue_delayed_work(clb_wq, &clb_work
				, msecs_to_jiffies(clb_time_ms));
	}
}

static int set_gauge_counter(struct tps80032_gauge_counter *gauge_counter)
{
	int ret = 0, i;
	u8 value, value2;
	unsigned long long temp;
	unsigned int cc_sample_cntr = 0;
	int cc_accum;

	CHECK_LOG();

	if (gauge_counter->pass_time > 0 && !!(gauge_counter->pass_time && 0x7F000000)) {
		pr_err("%s: SET_COUNTER paramter pass_timer might cause overflow reject\n", __func__);
		ret = -EFAULT;
		goto value_overflow;
	}

	temp = gauge_counter->accum_current < 0 ?
			-1 * gauge_counter->accum_current : gauge_counter->accum_current;

#if GAUGE_DEBUG
	pr_info("accum_current = %d => temp(unsigned) = %lld\n", gauge_counter->accum_current, temp);
#endif

	if (gauge_data->rate_select == URSTOP)
		goto gauge_fail;

	if (gauge_counter->pass_time >= 0)
		cc_sample_cntr = gauge_counter->pass_time
				* ( 1 << (2 * (gauge_data->rate_select + 1))) / 1000;

	temp = (temp << F_S_BIT);
	do_div(temp, ((V_FS * 1000) * AMPLIFIER / R_S));

	cc_accum = gauge_counter->accum_current < 0 ? -1 * ((int) temp) : (int) temp;
#if GAUGE_DEBUG
	pr_info("temp(unsigned) = %lld => cc_accum=%d\n", temp, cc_accum);
#endif

	ret = tps80031_read(gauge_data->parent_dev, SLAVE_ID2,
			TPS80032_FG_REG_00, &value);
	if (ret < 0) {
		pr_err("%s:read register TPS80032_FG_REG_00 fail\n"
			, __func__);
		goto i2c_fail;
	}

	ret = tps80031_write(gauge_data->parent_dev, SLAVE_ID2,
			TPS80032_FG_REG_00, value | CC_PAUSE);
	if (ret < 0) {
		pr_err("%s:write register TPSS80032_FG_REG_00"
			" faild value %lu\n"
			, __func__, value | CC_PAUSE);
		goto i2c_fail;
	}

	if (gauge_counter->pass_time >= 0) {
		gauge_data->cc_sample_cntr[0] = cc_sample_cntr;
		for (i = 0; i < 3; i++) {
			ret = tps80031_write(gauge_data->parent_dev, SLAVE_ID2,
					TPS80032_FG_REG_01 + i, cc_sample_cntr & 0xFF);
			if (ret < 0) {
				pr_err("%s:write register TPS80032_FG_REG_0%d"
						" fail\n"
						, __func__, i + 1);
				goto i2c_fail;
			}
			cc_sample_cntr >>= 8;
		}
	}

	gauge_data->cc_accum[0] = cc_accum;
	for (i = 0; i < 4; i++) {
		ret = tps80031_write(gauge_data->parent_dev, SLAVE_ID2,
				TPS80032_FG_REG_04 + i, cc_accum & 0xFF);
		if (ret < 0) {
			pr_err("%s:read register TPS80032_FG_REG_0%d"
				" fail\n"
				, __func__, i + 4);
			goto i2c_fail;
		}
		cc_accum >>= 8;
	}

#if GAUGE_DEBUG
	pr_info("cc_sample_cntr=0x%x, cc_accum=0x%x\n",gauge_data->cc_sample_cntr[0], gauge_data->cc_accum[0]);
#endif
	for (i = 0; i < 3; i++) {
		ret = tps80031_read(gauge_data->parent_dev, SLAVE_ID2,
				TPS80032_FG_REG_01 + i, &value2);
		if (ret < 0) {
			pr_err("%s:read register TPS80032_FG_REG_0%d"
				" fail\n"
				, __func__, i + 1);
			goto i2c_fail;
		}
#if GAUGE_DEBUG
		pr_info("RE_0%d=0x%x\n", i + 1, value2);
#endif
	}

	for (i = 0; i < 4; i++) {
		ret = tps80031_read(gauge_data->parent_dev, SLAVE_ID2,
				TPS80032_FG_REG_04 + i, &value2);
		if (ret < 0) {
			pr_err("%s:read register TPS80032_FG_REG_0%d"
				" fail\n"
				, __func__, i + 4);
			goto i2c_fail;
		}
#if GAUGE_DEBUG
		pr_info("RE_0%d=0x%x\n", i + 4, value2);
#endif
	}

	ret = tps80031_write(gauge_data->parent_dev, SLAVE_ID2,
			TPS80032_FG_REG_00, value);
	if (ret < 0) {
		pr_err("%s:write register TPSS80032_FG_REG_00"
			" fail value %d\n"
			, __func__, value);
	}

gauge_fail:
i2c_fail:
value_overflow:
	return ret;
}

static int read_gauge_all(int which)
{
	int ret = 0;
	int i;
	u8 value1, value2;

	CHECK_LOG();

	if (which !=0)
		which = 1;

	if (gauge_data->rate_select == URSTOP)
		goto gauge_fail;

	ret = tps80031_read(gauge_data->parent_dev, SLAVE_ID2,
			TPS80032_FG_REG_00, &value1);
	if (ret < 0) {
		pr_err("%s:read register TPS80032_FG_REG_00"
				" fail\n"
				, __func__);
		goto i2c_fail;
	}

	gauge_data->cc_active_mode = (value1 >> 6);

	value2 = value1 | CC_PAUSE;
	ret = tps80031_write(gauge_data->parent_dev, SLAVE_ID2,
			TPS80032_FG_REG_00, value2);
	if (ret < 0) {
		pr_err("%s:write register TPSS80032_FG_REG_00"
			" faild value %u\n"
			, __func__, value2);
		goto i2c_fail;
	}

	gauge_data->cc_sample_cntr[which] = 0;
	for (i = 0; i < 3; i++) {
		ret = tps80031_read(gauge_data->parent_dev, SLAVE_ID2,
				TPS80032_FG_REG_03 - i, &value2);
		if (ret < 0) {
			pr_err("%s:read register TPS80032_FG_REG_0%d"
				" fail\n"
				, __func__, 3 - i);
			goto i2c_fail;
		}
		gauge_data->cc_sample_cntr[which] =
			(gauge_data->cc_sample_cntr[which] << 8) + value2;
	}

	gauge_data->cc_accum[which] = 0;
	for (i = 0; i < 4; i++) {
		ret = tps80031_read(gauge_data->parent_dev, SLAVE_ID2,
				TPS80032_FG_REG_07 - i, &value2);
		if (ret < 0) {
			pr_err("%s:read register TPS80032_FG_REG_0%d"
				" fail\n"
				, __func__, 7 - i);
			goto i2c_fail;
		}
		gauge_data->cc_accum[which] =
			(gauge_data->cc_accum[which] << 8) + value2;
	}

	ret = tps80031_write(gauge_data->parent_dev, SLAVE_ID2,
			TPS80032_FG_REG_00, value1);
	if (ret < 0) {
		pr_err("%s:write register TPSS80032_FG_REG_00"
			" fail value %d\n"
			, __func__, value1);
		goto i2c_fail;
	}

	return 0;
gauge_fail:
i2c_fail:
	gauge_data->cc_sample_cntr[which] = 0;
	gauge_data->cc_accum[which] = 0;
	return ret;
}

static void calculate_time_and_iacc(
		struct tps80032_gauge_reply *gauge_reply, int from_zero)
{
	long cntr_diff = 0;
	unsigned int cc_sample_cntr_0 = 0;
	int cc_accum_0 = 0;

	if (gauge_data->rate_select == URSTOP) {
		gauge_reply->pass_time = 0;
		gauge_reply->accum_current = 0;
		gauge_data->cc_sample_cntr[1] = 0;
		gauge_data->cc_accum[1] = 0;
	} else {
		if (!from_zero) {
			cc_sample_cntr_0 = gauge_data->cc_sample_cntr[0];
			cc_accum_0 = gauge_data->cc_accum[0];
		}

		cntr_diff = gauge_data->cc_sample_cntr[1]
			- cc_sample_cntr_0;
		gauge_reply->pass_time = cntr_diff * 1000
			/ ( 1 << (2 * (gauge_data->cc_active_mode + 1)));
		gauge_reply->accum_current =
			((((long long) gauge_data->cc_accum[1] - cc_accum_0
			- (gauge_data->cc_offset + gauge_data->offset_shift) * cntr_diff))
			* (V_FS * 1000 * AMPLIFIER / R_S))
			>> F_S_BIT;
	}
#if GAUGE_DEBUG
	pr_info("active_mode=%d, rate_select=%d\n",gauge_data->cc_active_mode, gauge_data->rate_select);
	pr_info("offset=%d shift=%d\n",gauge_data->cc_offset, gauge_data->offset_shift);
	pr_info("cntr=0:%x 1:%x\n"
			, gauge_data->cc_sample_cntr[0]
			, gauge_data->cc_sample_cntr[1]);
	pr_info("accum=0:%x 1:%x\n"
			, gauge_data->cc_accum[0]
			, gauge_data->cc_accum[1]);
	pr_info("pass_time = %u, accum_current = %d\n"
			, gauge_reply->pass_time
			, gauge_reply->accum_current);
#endif
	gauge_data->cc_sample_cntr[0] = gauge_data->cc_sample_cntr[1];
	gauge_data->cc_accum[0] = gauge_data->cc_accum[1];
}

static int get_adc_volt(struct tps80032_gauge_reply *gauge_reply)
{
	int32_t adc_value;
	int ret = 0;

	mutex_lock(&gauge_data->data_lock);
	if (gauge_data->rate_select == URSTOP) {
		gauge_reply->batt_volt = 0;
		mutex_unlock(&gauge_data->data_lock);
		return ret;
	}
	mutex_unlock(&gauge_data->data_lock);

	if (gauge_data->adc_volt_channel < 0) {
		gauge_reply->batt_volt = 3800;
		return ret;
	}
	ret = tps80032_adc_select_and_read(
			&adc_value,
			gauge_data->adc_volt_channel);
	if (ret) {
		pr_err("%s:adc volt value get fail!!\n", __func__);
		gauge_reply->batt_volt = 0;
		return ret;
	}

	gauge_reply->batt_volt = 1250 * adc_value * 4 / 4095;
	return ret;
}

static int get_adc_temp(unsigned int *adc_temp)
{
	int32_t adc_value;
	int ret = 0;

	if (!adc_temp) {
		pr_err("%s: parameter adc_temp is NULL!!\n", __func__);
		return -EFAULT;
	}

	mutex_lock(&gauge_data->data_lock);
	if (gauge_data->rate_select == URSTOP) {
		*adc_temp = 0;
		mutex_unlock(&gauge_data->data_lock);
		return ret;
	}
	mutex_unlock(&gauge_data->data_lock);

	if (gauge_data->adc_temp_channel < 0) {
		*adc_temp = 416;
		return ret;
	}

	ret = tps80032_adc_select_and_read(
			&adc_value,
			gauge_data->adc_temp_channel);
	if (ret) {
		pr_err("%s:adc temp value get fail!!\n", __func__);
		*adc_temp = 0;
		return ret;
	}

	*adc_temp = 1250 * adc_value / 4095 ;
	return ret;
}

static ssize_t store_reset(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	unsigned long ur_select = 0;
	int opened = 0;
	int ret;

	CHECK_LOG();

	spin_lock(&gauge_data->gauge_lock);
	if (gauge_data->is_open)
		opened = 1;
	spin_unlock(&gauge_data->gauge_lock);

	if (opened)
		return 0;

	ur_select = simple_strtoul(buf, NULL, 10);

	mutex_lock(&gauge_data->data_lock);
	ret = reset_gauge(ur_select);
	if (ret < 0)
		goto use_gauge_fail;

	ret = read_gauge_all(0);
	if (ret < 0)
		goto use_gauge_fail;

	mutex_unlock(&gauge_data->data_lock);
	return count;
use_gauge_fail:
	mutex_unlock(&gauge_data->data_lock);
	return ret;
}

static ssize_t store_set_clb(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	unsigned int clb_time_ms = 0;
	int opened = 0;

	CHECK_LOG();

	spin_lock(&gauge_data->gauge_lock);
	if (gauge_data->is_open)
		opened = 1;
	spin_unlock(&gauge_data->gauge_lock);

	if (opened)
		return 0;

	clb_time_ms = simple_strtoul(buf, NULL, 10);

	set_calibration_timer(clb_time_ms);

	return count;
}

static ssize_t show_get_all(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	int len = 0, ret = 0;
	unsigned int adc_temp;
	int opened = 0;
	struct tps80032_gauge_reply gauge_reply;

	CHECK_LOG();

	spin_lock(&gauge_data->gauge_lock);
	if (gauge_data->is_open)
		opened = 1;
	spin_unlock(&gauge_data->gauge_lock);

	if (opened) {
		len += scnprintf(buf + len, PAGE_SIZE - len,
			"0\n0\n0\n0");
		return len;
	}

	mutex_lock(&gauge_data->data_lock);
	ret = read_gauge_all(1);
	if (ret < 0) {
		len += scnprintf(buf + len, PAGE_SIZE - len,
			"0\n0\n0\n0");
		mutex_unlock(&gauge_data->data_lock);
		return len;
	}

	calculate_time_and_iacc(&gauge_reply, 0);
	mutex_unlock(&gauge_data->data_lock);
	get_adc_volt(&gauge_reply);
	get_adc_temp(&adc_temp);
	len += scnprintf(buf + len, PAGE_SIZE - len,
			"%d\n", gauge_reply.accum_current);
	len += scnprintf(buf + len, PAGE_SIZE - len,
			"%u\n", gauge_reply.pass_time);
	len += scnprintf(buf + len, PAGE_SIZE - len,
			"%u\n", gauge_reply.batt_volt);
	len += scnprintf(buf + len, PAGE_SIZE - len,
			"%u", adc_temp);
	return len;
}

static struct device_attribute gauge_attrs[] = {
	__ATTR(reset, S_IWUSR | S_IWGRP, NULL, store_reset),
	__ATTR(set_clb, S_IWUSR | S_IWGRP, NULL, store_set_clb),
	__ATTR(get_all, S_IRUGO, show_get_all, NULL),
};

static int gauge_create_attrs(struct device *dev)
{
	int i = 0, rc = 0;

	for (i = 0; i < ARRAY_SIZE(gauge_attrs); i++) {
		rc = device_create_file(dev, &gauge_attrs[i]);
		if (rc)
			goto gauge_attrs_failed;
	}

	goto succeed;

gauge_attrs_failed:
	while (i--)
		device_remove_file(dev, &gauge_attrs[i]);
succeed:
	return rc;
}

static long tps80032_gauge_ioctl(struct file *filp,
			unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	enum update_rate ur_select;
	unsigned int clb_time_ms, capacity;
	int offset_shift;
	struct tps80032_gauge_reply gauge_reply;
	struct tps80032_gauge_counter gauge_counter;
	struct tps80032_gauge_debug gauge_debug;

	CHECK_LOG();

	switch (cmd) {
	case TPS80032_GAUGE_IOCTL_RESET: {

		if (copy_from_user(&ur_select, (void *)arg
				   , sizeof(enum update_rate))) {
			ret = -EFAULT;
			pr_err("%s: RESET copy_from_user failed!\n"
				, __func__);
			break;
		}

		mutex_lock(&gauge_data->data_lock);
		ret = reset_gauge(ur_select);
		if (ret < 0) {
			mutex_unlock(&gauge_data->data_lock);
			break;
		}

		ret = read_gauge_all(0);
		mutex_unlock(&gauge_data->data_lock);

		break;
	}
	case TPS80032_GAUGE_IOCTL_SET_CLB: {

		if (copy_from_user(&clb_time_ms, (void *)arg
				   , sizeof(unsigned int))) {
			ret = -EFAULT;
			pr_err("%s: SET_CLB copy_from_user failed!\n"
				, __func__);
			break;
		}

		set_calibration_timer(clb_time_ms);

		break;
	}
	case TPS80032_GAUGE_IOCTL_GET_ALL: {

		mutex_lock(&gauge_data->data_lock);
		ret = read_gauge_all(1);
		if (ret < 0) {
			mutex_unlock(&gauge_data->data_lock);
			break;
		}

		calculate_time_and_iacc(&gauge_reply, 0);
		mutex_unlock(&gauge_data->data_lock);
		get_adc_volt(&gauge_reply);

		if (copy_to_user((void __user *)arg
				, &gauge_reply
				, sizeof(struct tps80032_gauge_reply))) {
			pr_err("%s: GET_ALL copy_to_user failed!\n", __func__);
			ret = -EFAULT;
		}

		break;
	}
	case TPS80032_GAUGE_IOCTL_SET_COUNTER: {

		if (copy_from_user(&gauge_counter, (void *)arg
				   , sizeof(struct tps80032_gauge_counter))) {
			ret = -EFAULT;
			pr_err("%s: SET_COUNTER copy_from_user failed!\n"
				, __func__);
			break;
		}

		pr_info("Set tps80032 gauge updated capacity = %d0uAs total time = %dms\n", gauge_counter.accum_current, gauge_counter.pass_time);
		mutex_lock(&gauge_data->data_lock);
		ret = set_gauge_counter(&gauge_counter);
		mutex_unlock(&gauge_data->data_lock);

		break;
	}
	case TPS80032_GAUGE_IOCTL_GET_COUNTER: {

		mutex_lock(&gauge_data->data_lock);
		ret = read_gauge_all(1);
		if (ret < 0) {
			mutex_unlock(&gauge_data->data_lock);
			break;
		}

		calculate_time_and_iacc(&gauge_reply, 1);

		gauge_counter.accum_current = gauge_reply.accum_current;
		gauge_counter.pass_time = gauge_reply.pass_time;
		mutex_unlock(&gauge_data->data_lock);

		pr_info("Get tps80032 gauge updated capacity = %d0uAs total time = %dms\n", gauge_counter.accum_current, gauge_counter.pass_time);

		if (copy_to_user((void __user *)arg, &gauge_counter, sizeof(struct tps80032_gauge_counter))) {
			pr_err("%s: GET_COUNTER copy_to_user failed!\n", __func__);
			ret = -EFAULT;
		}

		break;
	}
	case TPS80032_GAUGE_IOCTL_SET_CAPACITY: {

		if (copy_from_user(&capacity, (void *)arg, sizeof(unsigned int))) {
			ret = -EFAULT;
			pr_err("%s: SET_CAPACITY copy_from_user failed!\n"
				, __func__);
			break;
		}

		pr_info("Set tps80032 gauge capacity = %d0uAh\n", capacity);
		gauge_counter.accum_current = capacity * 3600;
		gauge_counter.pass_time = 0;

		mutex_lock(&gauge_data->data_lock);
		ret = set_gauge_counter(&gauge_counter);
		mutex_unlock(&gauge_data->data_lock);

		break;
	}
	case TPS80032_GAUGE_IOCTL_GET_DEBUG: {
		memset(&gauge_debug, 0, sizeof(struct tps80032_gauge_debug));
		gauge_debug.offset = gauge_data->cc_offset;
		gauge_debug.offset_shift = gauge_data->offset_shift;
		gauge_debug.v_fs = V_FS;

		if (copy_to_user((void __user *)arg, &gauge_debug, sizeof(struct tps80032_gauge_debug))) {
			pr_err("%s: GET_DEBUG copy_to_user failed!\n", __func__);
			ret = -EFAULT;
		}
		break;
	}
	case TPS80032_GAUGE_IOCTL_SET_OFFSET_SHIFT: {

		if (copy_from_user(&offset_shift, (void *)arg, sizeof(int))) {
			ret = -EFAULT;
			pr_err("%s: SET_OFFSET_SHIFT copy_from_user failed!\n"
				, __func__);
			break;
		}

		pr_info("Set tps80032 gauge offset shift = %d\n", offset_shift);

		mutex_lock(&gauge_data->data_lock);
		gauge_data->offset_shift = offset_shift;
		mutex_unlock(&gauge_data->data_lock);

		break;
	}
	default:
		pr_err("%s: no matched ioctl cmd\n", __func__);
		break;
	}

	return ret;
}

static int tps80032_gauge_open(struct inode *inode, struct file *filp)
{
	int ret = 0;

	CHECK_LOG();

	spin_lock(&gauge_data->gauge_lock);
	if (gauge_data->is_open)
		ret = -EBUSY;
	else
		gauge_data->is_open = 1;
	spin_unlock(&gauge_data->gauge_lock);
	return ret;
}

static int tps80032_gauge_release(struct inode *inode, struct file *filp)
{
	CHECK_LOG();

	spin_lock(&gauge_data->gauge_lock);
	gauge_data->is_open = 0;
	spin_unlock(&gauge_data->gauge_lock);

	mutex_lock(&gauge_data->data_lock);
	gauge_data->rate_select = URSTOP;
	mutex_unlock(&gauge_data->data_lock);
	return 0;
}

const struct file_operations tps80032_gauge_fops = {
	.owner = THIS_MODULE,
	.open = tps80032_gauge_open,
	.release = tps80032_gauge_release,
	.unlocked_ioctl = tps80032_gauge_ioctl,
};

static struct miscdevice tps80032_gauge_device_node = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "tps80032_gauge",
	.fops = &tps80032_gauge_fops,
};

static int __devinit tps80032_gauge_probe(struct platform_device *pdev)
{
	struct tps80032_gauge_platform_data *pdata =
						pdev->dev.platform_data;
	int ret = 0;

	CHECK_LOG();

	wake_lock_init(&gauge_clb_lock, WAKE_LOCK_SUSPEND,
			"gauge_clb_lock");

	gauge_data = kzalloc(sizeof(struct tps80032_gauge_data)
			, GFP_KERNEL);
	if (!gauge_data) {
		pr_err("%s:Unable to allocat memory!!\n", __func__);
		return -ENOMEM;
	}

	mutex_init(&gauge_data->data_lock);
	gauge_data->parent_dev = pdev->dev.parent;
	gauge_data->irq = pdata->irq;
	gauge_data->adc_volt_channel = pdata->adc_volt_channel;
	gauge_data->adc_temp_channel = pdata->adc_temp_channel;
	gauge_data->is_open = 0;

	clb_wq = create_singlethread_workqueue("tps80032_gauge_wq");
	ret = request_threaded_irq(gauge_data->irq,
				NULL, cc_autocal_handler,
				IRQF_ONESHOT, "tps80031_cc_autocal"
				, gauge_data);
	if (ret < 0) {
		pr_err("%s:request gauge IRQ %d fail\n"
			, __func__, gauge_data->irq);
		goto request_irq_fail;
	}

	ret = misc_register(&tps80032_gauge_device_node);
	if (ret) {
		pr_err("%s:Unable to register misc device %d\n"
			, __func__, MISC_DYNAMIC_MINOR);
		goto misc_register_fail;
	}

	gauge_create_attrs(&pdev->dev);

	return 0;
misc_register_fail:
	free_irq(gauge_data->irq, gauge_data);
request_irq_fail:
	kfree(gauge_data);
	return ret;
}

static int __devexit tps80032_gauge_teardown(struct platform_device *pdev)
{
	CHECK_LOG();

	misc_deregister(&tps80032_gauge_device_node);
	free_irq(gauge_data->irq, gauge_data);
	kfree(gauge_data);

	return 0;
}

#ifdef CONFIG_PM
static int tps80032_gauge_suspend(struct platform_device *pdev,
				pm_message_t state)
{
	CHECK_LOG();

	return 0;
}

static int tps80032_gauge_resume(struct platform_device *pdev)
{
	CHECK_LOG();

	return 0;
}
#endif

static struct platform_driver tps80032_gauge_driver = {
	.probe = tps80032_gauge_probe,
	.remove = __devexit_p(tps80032_gauge_teardown),
#ifdef CONFIG_PM
	.suspend = tps80032_gauge_suspend,
	.resume = tps80032_gauge_resume,
#endif
	.driver = {
		.name = "tps80032-gauge",
		.owner = THIS_MODULE,
	},
};

static int __init tps80032_gauge_init(void)
{
	CHECK_LOG();

	return platform_driver_register(&tps80032_gauge_driver);
}
fs_initcall(tps80032_gauge_init);

static void __exit tps80032_gauge_exit(void)
{
	platform_driver_unregister(&tps80032_gauge_driver);
}
module_exit(tps80032_gauge_exit);

MODULE_DESCRIPTION("TPS80032 gauge driver");
MODULE_LICENSE("GPL");
