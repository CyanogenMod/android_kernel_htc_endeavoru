/* drivers/power/tps80032_charger.c
 *
 * Copyright (C) 2012 HTC Corporation
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

#define PR_TAG "[BATT][chg]"
#define pr_fmt(fmt) PR_TAG fmt

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/time.h>
#include <linux/slab.h>
#include <mach/board_htc.h>
#include <linux/power_supply.h>
#include <linux/mfd/tps80031.h>
#include <linux/tps80032_charger.h>
#include <linux/delay.h>

/* used for debug if function called */
#define FUNC_CALL_CHECK 0

#if FUNC_CALL_CHECK
#define CHECK_LOG() pr_info("%s\n", __func__)
#else
#define CHECK_LOG() (void)0
#endif

#define CHARGERUSB_VSYSREG	0xDC
#define CONTROLLER_CTRL1	0xE1
#define CONTROLLER_STAT1	0xE3
#define CHARGERUSB_INT_STATUS	0xE4
#define CHARGERUSB_INT_MASK	0xE5
#define CHARGERUSB_STATUS_INT1	0xE6
#define CHARGERUSB_STATUS_INT2	0xE7
#define CHARGERUSB_CTRL1	0xE8
#define CHARGERUSB_CTRL2	0xE9
#define CHARGERUSB_CTRL3	0xEA
#define CHARGERUSB_VOREG	0xEC
#define CHARGERUSB_VICHRG	0xED
#define CHARGERUSB_CINLIMIT	0xEE
#define CHARGERUSB_CTRLLIMIT2	0xF0
#define CHARGERUSB_CTRLLIMIT1	0xEF
#define CHARGERUSB_VICHRG_PC	0xDD
#define CONTROLLER_WDG		0xE2
#define LINEAR_CHRG_STS		0xDE
#define ANTICOLLAPSE_CTRL1	0xF1

/* LINEAR_CHRG_STS */
#define END_OF_CHARGE		(0x1U << 5)

/* CHARGERUSB_INT_STATUS */
#define STS_CURRENT_TERM	(0x1U << 3)
#define STS_CHARGETUSB_STAT	(0x1U << 2)
#define STS_CHARGERUSB_THMREG	(0x1U << 1)
#define STS_CHARGERUSB_FAULT	(0x1U << 0)

/* CHARGERUSB_STATUS_INT1 */
#define VBUS_OVP	(0x1U << 0)

/* CONTROLLER_STAT1 */
#define LINCH_GATED	(0x1U << 6)

/* CHARGERUSB_STATUS_INT2 */
#define CURRENT_TERM	(0x1U << 2)
#define CHARGE_DONE	(0x1U << 1)

#define TPS80032_CHARGER_CHECK_INTERVAL (5 * 1000)

struct tps80032_charger_data {
	struct device 	*parent_dev;
	struct mutex data_lock;
	int linch_gated_irq;
	int fault_wdg_irq;
	int int_chg_irq;
	int use_hiz_disable;
	u8 reg_volt_regh;
};

static struct tps80032_charger_data *charger_data;

static int tps80032_charger_initial = -1;
static unsigned int tps80032_low_chg;
static unsigned int tps80032_over_vchg;
static unsigned int tps80032_icl_750;

static int tps80032_charger_state = -1;

static struct workqueue_struct *tps80032_wq;
static struct work_struct chg_linch_work;
static struct work_struct wdg_fault_work;

static LIST_HEAD(tps80032_charger_int_list);
static DEFINE_MUTEX(notify_lock);

static int tps80032_charger_regulation_voltage_set(int regh)
{
	int ret = 0;

	mutex_lock(&charger_data->data_lock);
	if (regh >= 0)
		charger_data->reg_volt_regh = regh & 0xFF;
	else
		regh = charger_data->reg_volt_regh;

	ret = tps80031_update(charger_data->parent_dev, SLAVE_ID2,
			CHARGERUSB_VOREG, regh, 0x3F);
	if (ret < 0) {
		pr_err("%s(): Failed in updating register 0x%02x\n",
				__func__, regh);
	}
	mutex_unlock(&charger_data->data_lock);

	return ret;
}

void tps80032_charger_dump_status(int cond)
{
	u8 regh[16];

	CHECK_LOG();

	tps80031_read(charger_data->parent_dev, SLAVE_ID2,
			LINEAR_CHRG_STS, &regh[0]);
	tps80031_read(charger_data->parent_dev, SLAVE_ID2,
			CONTROLLER_STAT1, &regh[1]);
	tps80031_read(charger_data->parent_dev, SLAVE_ID2,
			CHARGERUSB_STATUS_INT1, &regh[2]);
	tps80031_read(charger_data->parent_dev, SLAVE_ID2,
			CHARGERUSB_STATUS_INT2, &regh[3]);
	tps80031_read(charger_data->parent_dev, SLAVE_ID2,
			CHARGERUSB_CINLIMIT, &regh[4]);
	tps80031_read(charger_data->parent_dev, SLAVE_ID2,
			CHARGERUSB_VOREG, &regh[5]);
	tps80031_read(charger_data->parent_dev, SLAVE_ID2,
			CHARGERUSB_VICHRG, &regh[6]);
	tps80031_read(charger_data->parent_dev, SLAVE_ID2,
			CHARGERUSB_CTRLLIMIT1, &regh[7]);
	tps80031_read(charger_data->parent_dev, SLAVE_ID2,
			CHARGERUSB_CTRLLIMIT2, &regh[8]);
	tps80031_read(charger_data->parent_dev, SLAVE_ID2,
			ANTICOLLAPSE_CTRL1, &regh[9]);
	tps80031_read(charger_data->parent_dev, SLAVE_ID2,
			CHARGERUSB_CTRL1, &regh[10]);
	pr_info("Check %d: "
			"STS=0x%X, STAT1=0x%X, INT1=0x%X, "
			"INT2=0x%X, CINL=0x%X, VO=0x%X, "
			"VI=0x%X, VOL=0x%X, VIL=0x%X, "
			"DPPM=0x%X, CC1=0x%X\n",
			cond,
			regh[0], regh[1], regh[2],
			regh[3], regh[4], regh[5],
			regh[6], regh[7], regh[8],
			regh[9], regh[10]);
}
EXPORT_SYMBOL_GPL(tps80032_charger_dump_status);

#define TPS80032_ID2_I2C_WRITE(offset, value) do {				\
		ret = tps80031_write(charger_data->parent_dev, SLAVE_ID2,	\
				(offset), (value));				\
		if (ret < 0) {							\
			pr_err("%s(): Failed in writing register 0x%02x\n",	\
					__func__, offset);			\
			goto tps80032_i2c_err;					\
		}								\
	} while (0)

#define TPS80032_ID2_I2C_UPDATE(offset, value, mask) do {			\
		ret = tps80031_update(charger_data->parent_dev, SLAVE_ID2,	\
				(offset), (value), (mask));			\
		if (ret < 0) {							\
			pr_err("%s(): Failed in updating register 0x%02x\n",	\
					__func__, (offset));			\
			goto tps80032_i2c_err;					\
		}								\
	} while (0)

#define TPS80032_ID2_I2C_READ(offset, value) do {				\
		ret = tps80031_read(charger_data->parent_dev, SLAVE_ID2,	\
				(offset), (value));				\
		if (ret < 0) {							\
			pr_err("%s(): Failed in readinging register 0x%02x\n",	\
					__func__, (offset));			\
			goto tps80032_i2c_err;					\
		}								\
	} while (0)

int tps80032_charger_set_ctrl(u32 ctl)
{
	int ret = 0;
	u8 regh;

	CHECK_LOG();

	if (tps80032_charger_initial < 0) {
		pr_info("tps80032 charger not ready, state = %d!!\n", tps80032_charger_initial);
		return 0;
	}

	switch (ctl) {
	case POWER_SUPPLY_DISABLE_CHARGE:
		pr_info("Switch charger OFF\n");

		tps80032_charger_dump_status(0);

		regh = 0x80;
		TPS80032_ID2_I2C_WRITE(CHARGERUSB_CTRL1, regh);

		tps80032_charger_state = 0;
		tps80032_charger_dump_status(1);

		break;
	case POWER_SUPPLY_ENABLE_SLOW_CHARGE:
		pr_info("Switch charger ON: SLOW\n");

		tps80032_charger_dump_status(0);

		TPS80032_ID2_I2C_WRITE(ANTICOLLAPSE_CTRL1, 0x64);
		if (!!tps80032_low_chg)
			TPS80032_ID2_I2C_WRITE(CHARGERUSB_VICHRG, 0x00);
		else
			TPS80032_ID2_I2C_WRITE(CHARGERUSB_VICHRG, 0x0A);

		TPS80032_ID2_I2C_UPDATE(CHARGERUSB_CINLIMIT, 0x09, 0x3F);
		tps80032_charger_regulation_voltage_set(-1);
		TPS80032_ID2_I2C_READ(CONTROLLER_WDG, &regh);
		regh |= 0x80;
		TPS80032_ID2_I2C_WRITE(CONTROLLER_WDG, regh);
		TPS80032_ID2_I2C_WRITE(CHARGERUSB_CTRL1, 0x10);

		tps80032_charger_state = 1;
		tps80032_charger_dump_status(1);

		break;
	case POWER_SUPPLY_ENABLE_FAST_CHARGE:
		pr_info("Switch charger ON: FAST\n");

		tps80032_charger_dump_status(0);

		TPS80032_ID2_I2C_WRITE(ANTICOLLAPSE_CTRL1, 0x64);
		if (!!tps80032_low_chg)
			TPS80032_ID2_I2C_WRITE(CHARGERUSB_VICHRG, 0x00);
		else
			TPS80032_ID2_I2C_WRITE(CHARGERUSB_VICHRG, 0x0A);

		if (!!tps80032_icl_750)
			TPS80032_ID2_I2C_UPDATE(CHARGERUSB_CINLIMIT, 0x0E, 0x3F);
		else
			TPS80032_ID2_I2C_UPDATE(CHARGERUSB_CINLIMIT, 0x2A, 0x3F);
		tps80032_charger_regulation_voltage_set(-1);
		TPS80032_ID2_I2C_READ(CONTROLLER_WDG, &regh);
		regh |= 0x80;
		TPS80032_ID2_I2C_WRITE(CONTROLLER_WDG, regh);
		TPS80032_ID2_I2C_WRITE(CHARGERUSB_CTRL1, 0x10);

		tps80032_charger_state = 2;
		tps80032_charger_dump_status(1);

		break;
	case POWER_SUPPLY_ENABLE_INTERNAL:
		pr_info("Switch charger BOOST\n");

		tps80032_charger_dump_status(0);
		TPS80032_ID2_I2C_WRITE(CHARGERUSB_CTRL1, 0x40);
		tps80032_charger_dump_status(1);

		break;
	case POWER_SUPPLY_ENABLE_9VAC_CHARGE:
	case POWER_SUPPLY_ENABLE_WIRELESS_CHARGE:
	case POWER_SUPPLY_ENABLE_SLOW_HV_CHARGE:
	case POWER_SUPPLY_ENABLE_FAST_HV_CHARGE:
		pr_err("charger control %d is not supportted now!!\n", ctl);
		ret = -EINVAL;
		break;
	case OVERTEMP_VREG:
		regh = 0x19;
		tps80032_charger_regulation_voltage_set(regh);
		TPS80032_ID2_I2C_READ(CHARGERUSB_VOREG, &regh);
		pr_info("Switch charger OVERTEMP_VREG: regh 0x%X=%x\n", CHARGERUSB_VOREG, regh);

		break;
	case UNDERTEMP_VREG:
		regh = 0x1E;
		tps80032_charger_regulation_voltage_set(regh);
		TPS80032_ID2_I2C_READ(CHARGERUSB_VOREG, &regh);
		pr_info("Switch charger UNDERTEMP_VREG: regh 0x%X=%x\n", CHARGERUSB_VOREG, regh);

		break;
	case NORMALTEMP_VREG:
		regh = 0x23;
		tps80032_charger_regulation_voltage_set(regh);
		TPS80032_ID2_I2C_READ(CHARGERUSB_VOREG, &regh);
		pr_info("Switch charger NORMALTEMP_VREG: regh 0x%X=%x\n", CHARGERUSB_VOREG, regh);

		break;
	case NORMALTEMP_VREG_HV:
		regh = 0x28;
		tps80032_charger_regulation_voltage_set(regh);
		TPS80032_ID2_I2C_READ(CHARGERUSB_VOREG, &regh);
		pr_info("Switch charger NORMALTEMP_VREG_HV: regh 0x%X=%x\n", CHARGERUSB_VOREG, regh);

		break;
	case NORMALTEMP_VREG_HV4340:
		regh = 0x2A;
		tps80032_charger_regulation_voltage_set(regh);
		TPS80032_ID2_I2C_READ(CHARGERUSB_VOREG, &regh);
		pr_info("Switch charger NORMALTEMP_VREG_HV4340: regh 0x%X=%x\n", CHARGERUSB_VOREG, regh);

		break;
	case ENABLE_HIZ_CHG:
		pr_info("Switch charger HIZ\n");

		tps80032_charger_dump_status(0);

		regh = 0x20;
		TPS80032_ID2_I2C_WRITE(CHARGERUSB_CTRL1, regh);

		tps80032_charger_state = 0;
		tps80032_charger_dump_status(1);
	case NORMALTEMP_VREG_HV4320:
		regh = 0x29;
		tps80032_charger_regulation_voltage_set(regh);
		TPS80032_ID2_I2C_READ(CHARGERUSB_VOREG, &regh);
		pr_info("Switch charger NORMALTEMP_VREG_HV4320: regh 0x%X=%x\n", CHARGERUSB_VOREG, regh);

		break;
	case ALLTEMP_VSYS_DISABLE:
		regh = 0x02;
		TPS80032_ID2_I2C_WRITE(CHARGERUSB_VSYSREG, regh);
		TPS80032_ID2_I2C_READ(CHARGERUSB_VSYSREG, &regh);
		pr_info("Switch charger ALLTEMP_VSYS_DISABLE: regh 0x%X=%x\n", CHARGERUSB_VSYSREG, regh);

		break;
	case NORMALTEMP_VSYS_4400:
		regh = 0x2D | 0x80;
		TPS80032_ID2_I2C_WRITE(CHARGERUSB_VSYSREG, regh);
		TPS80032_ID2_I2C_READ(CHARGERUSB_VSYSREG, &regh);
		pr_info("Switch charger NORMALTEMP_VSYS_4400: regh 0x%X=%x\n", CHARGERUSB_VSYSREG, regh);

		break;
	case NORMALTEMP_VSYS_4440:
		regh = 0x2F | 0x80;
		TPS80032_ID2_I2C_WRITE(CHARGERUSB_VSYSREG, regh);
		TPS80032_ID2_I2C_READ(CHARGERUSB_VSYSREG, &regh);
		pr_info("Switch charger NORMALTEMP_VSYS_4440: regh 0x%X=%x\n", CHARGERUSB_VSYSREG, regh);

		break;
	case OVERTEMP_VSYS_4300:
		regh = 0x28 | 0x80;
		TPS80032_ID2_I2C_WRITE(CHARGERUSB_VSYSREG, regh);
		TPS80032_ID2_I2C_READ(CHARGERUSB_VSYSREG, &regh);
		pr_info("Switch charger OVERTEMP_VSYS_4300: regh 0x%X=%x\n", CHARGERUSB_VSYSREG, regh);

		break;
	case OVERTEMP_VSYS_4340:
		regh = 0x2A | 0x80;
		TPS80032_ID2_I2C_WRITE(CHARGERUSB_VSYSREG, regh);
		TPS80032_ID2_I2C_READ(CHARGERUSB_VSYSREG, &regh);
		pr_info("Switch charger OVERTEMP_VSYS_4340: regh 0x%X=%x\n", CHARGERUSB_VSYSREG, regh);

		break;
	case OVERTEMP_VSYS_4140:
		regh = 0x20 | 0x80;
		TPS80032_ID2_I2C_WRITE(CHARGERUSB_VSYSREG, regh);
		TPS80032_ID2_I2C_READ(CHARGERUSB_VSYSREG, &regh);
		pr_info("Switch charger OVERTEMP_VSYS_4140: regh 0x%X=%x\n", CHARGERUSB_VSYSREG, regh);

		break;
	case UNDERTEMP_VSYS_4200:
		regh = 0x23 | 0x80;
		TPS80032_ID2_I2C_WRITE(CHARGERUSB_VSYSREG, regh);
		TPS80032_ID2_I2C_READ(CHARGERUSB_VSYSREG, &regh);
		pr_info("Switch charger UNDERTEMP_VSYS_4200: regh 0x%X=%x\n", CHARGERUSB_VSYSREG, regh);

		break;
	case CHECK_INT1:
		TPS80032_ID2_I2C_READ(CHARGERUSB_STATUS_INT1, &regh);
		pr_info("Switch charger CHECK_INT1: regh 0x%xh=%x\n", CHARGERUSB_STATUS_INT1, regh);
		ret = (int)regh;
		break;
	case CHECK_INT2:
		TPS80032_ID2_I2C_READ(CHARGERUSB_STATUS_INT2, &regh);
		pr_info("Switch charger CHECK_INT2: regh 0x%xh=%x\n", CHARGERUSB_STATUS_INT2, regh);
		ret = (int)regh;
		break;
	case ENABLE_LIMITED_CHG:
		tps80032_low_chg = 1;
		TPS80032_ID2_I2C_WRITE(CHARGERUSB_VICHRG, 0x00);
		TPS80032_ID2_I2C_READ(CHARGERUSB_VICHRG, &regh);
		pr_info("Switch charger ON (LIMITED): regh 0x%x=%x low_chg=%u\n", CHARGERUSB_VICHRG, regh, tps80032_low_chg);
		break;
	case CLEAR_LIMITED_CHG:
		tps80032_low_chg = 0;
		TPS80032_ID2_I2C_WRITE(CHARGERUSB_VICHRG, 0x0A);
		TPS80032_ID2_I2C_READ(CHARGERUSB_VICHRG, &regh);
		pr_info("Switch charger OFF (LIMITED): regh 0x%x=%x low_chg=%u\n", CHARGERUSB_VICHRG, regh, tps80032_low_chg);
		break;
	case SET_ICL_NORMAL:
		tps80032_icl_750 = 0;
		if (tps80032_charger_state == 2) {
			TPS80032_ID2_I2C_UPDATE(CHARGERUSB_CINLIMIT, 0x2A, 0x3F);
			TPS80032_ID2_I2C_READ(CHARGERUSB_CINLIMIT, &regh);
			pr_info("Switch charger SET_ICL_NORMAL: regh 0x%x=0x%x\n", CHARGERUSB_CINLIMIT, regh);
		} else
			pr_info("Switch charger SET_ICL_NORMAL \n");
		break;
	case SET_ICL750:
		tps80032_icl_750 = 1;
		if (tps80032_charger_state == 2) {
			TPS80032_ID2_I2C_UPDATE(CHARGERUSB_CINLIMIT, 0x0E, 0x3F);
			TPS80032_ID2_I2C_READ(CHARGERUSB_CINLIMIT, &regh);
			pr_info("Switch charger SET_ICL750: regh 0x%x=0x%x\n", CHARGERUSB_CINLIMIT, regh);
		} else
			pr_info("Switch charger SET_ICL750 \n");
		break;
	case CHECK_CHG:
	case CHECK_CONTROL:
	default:
		pr_err("charger control %d is not supportted now!!\n", ctl);
		ret = -EINVAL;
	}

tps80032_i2c_err:
	return ret;
}
EXPORT_SYMBOL_GPL(tps80032_charger_set_ctrl);

int tps80032_charger_register_notifier(struct tps80032_charger_int_notifier *notifier)
{

	if (!notifier || !notifier->name || !notifier->func)
		return -EINVAL;

	mutex_lock(&notify_lock);
	list_add(&notifier->notifier_link,
		&tps80032_charger_int_list);
	mutex_unlock(&notify_lock);
	return 0;
}
EXPORT_SYMBOL_GPL(tps80032_charger_register_notifier);

static void send_tps_chg_int_notify(int int_reg, int value)
{
	static struct tps80032_charger_int_notifier *notifier;

	CHECK_LOG();

	mutex_lock(&notify_lock);
	list_for_each_entry(notifier,
		&tps80032_charger_int_list,
		notifier_link) {
		if (notifier->func != NULL)
			notifier->func(int_reg, value);
	}
	mutex_unlock(&notify_lock);
}

static void chg_int_chg_work_func(struct work_struct *work)
{
	uint8_t chg_status[2];
	unsigned int count = 0;

	CHECK_LOG();

	chg_status[0] = 0xFF & tps80032_charger_set_ctrl(CHECK_INT1);
	chg_status[1] = 0xFF & tps80032_charger_set_ctrl(CHECK_INT2);

	if (tps80032_over_vchg == 0) {
		while (!!(chg_status[0] & VBUS_OVP)) {
			count++;
			if (count >= 3) {
				send_tps_chg_int_notify(CHECK_INT1, 1);
				tps80032_over_vchg = 1;
				break;
			}
			msleep(1000);
			chg_status[0] = 0xFF & tps80032_charger_set_ctrl(CHECK_INT1);
		}
	} else {
		send_tps_chg_int_notify(CHECK_INT1, 0);
		tps80032_over_vchg = 0;
	}
}

static irqreturn_t int_chg_isr(int irq, void *dev_id)
{
	uint8_t chg_status;
	int ret;

	CHECK_LOG();

	ret = tps80031_read(charger_data->parent_dev, SLAVE_ID2,
			CHARGERUSB_INT_STATUS, &chg_status);
	if (ret < 0) {
		pr_err("%s(): Failed in reading register 0x%02x\n",
				__func__, CHARGERUSB_INT_STATUS);
	} else {
		pr_info("%s():The status of CHARGERUSB_INT_STATUS is 0x%02x\n",
				__func__, chg_status);
		chg_int_chg_work_func(NULL);
	}

	return IRQ_HANDLED;
}

static void chg_linch_work_func(struct work_struct *work)
{
	uint8_t linch_status;
	int ret;

	CHECK_LOG();

	ret = tps80031_read(charger_data->parent_dev, SLAVE_ID2,
			LINEAR_CHRG_STS, &linch_status);
	if (ret < 0) {
		pr_err("%s(): Failed in reading register 0x%02x\n",
				__func__, LINEAR_CHRG_STS);
		return;
	}

	pr_info("%s():The status of LINEAR_CHRG_STS is 0x%02x\n",
			__func__, linch_status);
	if (!!(linch_status && END_OF_CHARGE)) {
		send_tps_chg_int_notify(CHG_CHARGE_DONE, 1);
	}
}

static irqreturn_t linch_status_isr(int irq, void *dev_id)
{
	CHECK_LOG();

	queue_work(tps80032_wq, &chg_linch_work);

	return IRQ_HANDLED;
}

static void wdg_fault_work_func(struct work_struct *work)
{
	CHECK_LOG();

	send_tps_chg_int_notify(CHECK_WATCHDOG, 1);
}

static irqreturn_t watchdog_expire_isr(int irq, void *dev_id)
{
	CHECK_LOG();

	queue_work(tps80032_wq, &wdg_fault_work);

	return IRQ_HANDLED;
}

static ssize_t tps80032_charger_charge_done_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	int ret = 0;
	uint8_t linch_status;

	ret = tps80031_read(charger_data->parent_dev, SLAVE_ID2,
			LINEAR_CHRG_STS, &linch_status);
	if (ret < 0) {
		pr_err("%s(): Failed in reading register 0x%02x\n",
				__func__, LINEAR_CHRG_STS);
		return scnprintf(buf, PAGE_SIZE, "0\n");
	}

	return scnprintf(buf, PAGE_SIZE, "%d\n", !!(linch_status & END_OF_CHARGE));
}

static ssize_t tps80032_charger_charge_enabled_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	int ret = 0;
	uint8_t status;

	ret = tps80031_read(charger_data->parent_dev, SLAVE_ID2,
			CONTROLLER_STAT1, &status);
	if (ret < 0) {
		pr_err("%s(): Failed in reading register 0x%02x\n",
				__func__, CONTROLLER_STAT1);
		return scnprintf(buf, PAGE_SIZE, "0\n");
	}

	return scnprintf(buf, PAGE_SIZE, "%d\n", !!(status & LINCH_GATED));
}

static ssize_t tps80032_charger_vsys_reg_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	int ret = 0;
	uint8_t status;

	ret = tps80031_read(charger_data->parent_dev, SLAVE_ID2,
			CHARGERUSB_VSYSREG, &status);
	if (ret < 0) {
		pr_err("%s(): Failed in reading register 0x%02x\n",
				__func__, CHARGERUSB_VSYSREG);
		return scnprintf(buf, PAGE_SIZE, "0\n");
	}

	return scnprintf(buf, PAGE_SIZE, "%d\n", status);
}

static struct device_attribute tps80032_charger_attrs[] = {
	__ATTR(charge_done, S_IRUGO, tps80032_charger_charge_done_show, NULL),
	__ATTR(charge_enabled, S_IRUGO, tps80032_charger_charge_enabled_show, NULL),
	__ATTR(vsys_reg, S_IRUGO, tps80032_charger_vsys_reg_show, NULL),
	};

int tps80032_charger_append_attr(struct device *dev)
{
	int i = 0, rc = 0;

	for (i = 0; i < ARRAY_SIZE(tps80032_charger_attrs); i++) {
		rc = device_create_file(dev, &tps80032_charger_attrs[i]);
		if (rc)
			goto charger_attrs_failed;
	}

	goto succeed;

charger_attrs_failed:
	while (i--)
		device_remove_file(dev, &tps80032_charger_attrs[i]);
succeed:
	return rc;
}
EXPORT_SYMBOL_GPL(tps80032_charger_append_attr);

static int __devinit tps80032_charger_probe(struct platform_device *pdev)
{
	struct tps80032_charger_platform_data *pdata = pdev->dev.platform_data;
	int ret = 0;
	u8 regh;

	CHECK_LOG();

	charger_data = kzalloc(sizeof(struct tps80032_charger_data), GFP_KERNEL);
	if (!charger_data) {
		pr_err("%s:Unable to allocat memory!!\n", __func__);
		return -ENOMEM;
	}

	mutex_init(&charger_data->data_lock);

	charger_data->parent_dev = pdev->dev.parent;
	charger_data->reg_volt_regh = 0x23;

	tps80032_wq = create_singlethread_workqueue("tps80032");
	if (!tps80032_wq) {
		pr_err("Failed to create tps80032 workqueue.\n");
		kfree(charger_data);
		ret = -ENOMEM;
		goto wq_create_fail;
	}

	ret = request_threaded_irq(pdata->int_chg_irq, NULL,
			int_chg_isr, 0, "tps80032-chg", charger_data);
	if (ret) {
		pr_err("Unable to register irq %d; error %d\n",
			pdata->int_chg_irq, ret);
		goto irq_chg_fail;
	}

	charger_data->int_chg_irq = pdata->int_chg_irq;

	ret = request_threaded_irq(pdata->linch_gated_irq, NULL,
			linch_status_isr, 0, "tps80032-linch", charger_data);
	if (ret) {
		pr_err("Unable to register irq %d; error %d\n",
			pdata->linch_gated_irq, ret);
		goto irq_linch_fail;
	} else {
		INIT_WORK(&chg_linch_work, chg_linch_work_func);
	}

	charger_data->linch_gated_irq = pdata->linch_gated_irq;

	ret = request_threaded_irq(pdata->fault_wdg_irq, NULL,
			watchdog_expire_isr, 0, "tps80032-wdg", charger_data);
	if (ret) {
		pr_err("Unable to register irq %d; error %d\n",
			pdata->fault_wdg_irq, ret);
		goto irq_wdg_fail;
	} else {
		INIT_WORK(&wdg_fault_work, wdg_fault_work_func);
	}

	charger_data->fault_wdg_irq = pdata->fault_wdg_irq;
	charger_data->use_hiz_disable = pdata->use_hiz_disable;

	TPS80032_ID2_I2C_READ(CHARGERUSB_CTRL3, &regh);
	regh |= 0x20;
	TPS80032_ID2_I2C_WRITE(CHARGERUSB_CTRL3, regh);

	tps80032_charger_initial = 1;
	pr_info("Driver registration done\n");

	return 0;
tps80032_i2c_err:
	free_irq(charger_data->fault_wdg_irq, charger_data);
	charger_data->fault_wdg_irq = 0;
irq_wdg_fail:
	free_irq(charger_data->linch_gated_irq, charger_data);
	charger_data->linch_gated_irq = 0;
irq_linch_fail:
	free_irq(charger_data->int_chg_irq, charger_data);
	charger_data->int_chg_irq = 0;
irq_chg_fail:
	destroy_workqueue(tps80032_wq);
wq_create_fail:
	kfree(charger_data);
	return ret;
}

static int __devexit tps80032_charger_teardown(struct platform_device *pdev)
{
	CHECK_LOG();

	if (!charger_data->fault_wdg_irq)
		free_irq(charger_data->fault_wdg_irq, charger_data);

	if (!charger_data->linch_gated_irq)
		free_irq(charger_data->linch_gated_irq, charger_data);

	if (!charger_data->int_chg_irq)
		free_irq(charger_data->int_chg_irq, charger_data);

	destroy_workqueue(tps80032_wq);
	kfree(charger_data);
	tps80032_charger_initial = -2;
	return 0;
}

static struct platform_driver tps80032_charger_driver = {
	.probe = tps80032_charger_probe,
	.remove = __devexit_p(tps80032_charger_teardown),
	.driver = {
		.name = "tps80032-charger",
		.owner = THIS_MODULE,
	},
};

static int __init tps80032_charger_init(void)
{
	CHECK_LOG();

	tps80032_low_chg = 0;

	return platform_driver_register(&tps80032_charger_driver);
}
fs_initcall(tps80032_charger_init);

static void __exit tps80032_charger_exit(void)
{
	CHECK_LOG();

	platform_driver_unregister(&tps80032_charger_driver);
}
module_exit(tps80032_charger_exit);

MODULE_DESCRIPTION("TPS80032 charger driver");
MODULE_LICENSE("GPL");
