/* drivers/misc/cable_detect.c - cable detect driver
 *
 * Copyright (C) 2009 HTC Corporation.
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

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/switch.h>
#include <linux/wakelock.h>
#include <mach/board_htc.h>
#include <mach/cable_detect.h>
#if defined(CONFIG_TEGRA_HDMI_MHL)
#include "../video/tegra/sii9234/TPI.h"
#endif

/*#define MHL_INTERNAL_POWER  1*/
static bool g_vbus = 0;
static bool g_desk_no_power = 0;

static struct switch_dev dock_switch = {
	.name = "dock",
};

struct cable_detect_info {
	spinlock_t lock;

	int vbus_mpp_gpio;
	int vbus_mpp_irq;
	enum usb_connect_type connect_type;
	/*for accessory*/
	int usb_id_pin_gpio;
	__u8 detect_type;
	__u8 accessory_type;
	int idpin_irq;
	u8 mfg_usb_carkit_enable;
	u8 mhl_reset_gpio;
	bool mhl_version_ctrl_flag;
	struct workqueue_struct *cable_detect_wq;
	struct delayed_work cable_detect_work;
	struct delayed_work vbus_detect_work;
	struct delayed_work recovery_host_work;
	struct wake_lock vbus_wlock;
	struct wake_lock cable_detect_wlock;
	void (*usb_uart_switch)(int);
	void (*usb_dpdn_switch)(int);
	struct usb_id_mpp_config_data *mpp_data;
	void (*config_usb_id_gpios)(bool enable);
	void (*config_desk_aud_gpios)(bool output, bool out_val);
	void (*mhl_1v2_power)(bool enable);
	int (*is_wireless_charger)(void);
	u8 cable_redetect;
	bool cable_detecting;
	int32_t (*get_adc_cb)(void);
	void (*cable_gpio_init)(void);

	u8 mhl_internal_3v3;

#ifdef CONFIG_CABLE_DETECT_GPIO_DOCK
	bool dock_detect;
	int dockpin_irq;
	int dock_pin_gpio;
	uint8_t dock_pin_state;
	struct delayed_work dock_work_isr;
	struct delayed_work dock_work;
#endif
} the_cable_info;

/* ---------------------------------------------------------------------------
			Routine prototype
-----------------------------------------------------------------------------*/
/*static irqreturn_t vbus_irq_handler(int irq, void *dev_id);*/ /* VBUS IRQ */
#ifdef CONFIG_CABLE_DETECT_ACCESSORY
static int cable_detect_get_adc(void);
static int sec_detect(struct cable_detect_info *pInfo);
static void usb_id_detect_init(struct cable_detect_info *info);
#endif
#ifdef CONFIG_CABLE_DETECT_GPIO_DOCK
static void dock_isr_work(struct work_struct *w);
static void dock_detect_work(struct work_struct *w);
static void dock_detect_init(struct cable_detect_info *pInfo);
#endif

int usb_get_vbus_value(void)
{
	extern int tps80031_vbus_on;
	return tps80031_vbus_on;
}

static DEFINE_MUTEX(cable_notify_sem);
static void send_cable_connect_notify(int cable_type)
{
	static struct t_cable_status_notifier *notifier;

	CABLE_DEBUG("%s: cable_type = %d\n", __func__, cable_type);
	mutex_lock(&cable_notify_sem);

	if (cable_type == CONNECT_TYPE_UNKNOWN)
		cable_type = CONNECT_TYPE_USB;
#if 0 /* DMB */
	if (cable_type > 0 && pInfo->accessory_type == DOCK_STATE_DMB) {
		CABLE_INFO("%s: DMB presents. Disabling charge.\n", __func__);
		cable_type = CONNECT_TYPE_CLEAR;
	}
#endif
	list_for_each_entry(notifier, &g_lh_cable_detect_notifier_list, cable_notifier_link) {
		if (notifier->func != NULL) {
			CABLE_INFO("Send to: %s, type %d\n", notifier->name, cable_type);
			/* Notify other drivers about connect type. */
			/* use slow charging for unknown type*/
			notifier->func(cable_type);
		}
	}
	mutex_unlock(&cable_notify_sem);
}

int cable_detect_register_notifier(struct t_cable_status_notifier *notifier)
{
	if (!notifier || !notifier->name || !notifier->func)
		return -EINVAL;

	mutex_lock(&cable_notify_sem);
	list_add(&notifier->cable_notifier_link, &g_lh_cable_detect_notifier_list);
	notifier->func(cable_get_connect_type());
	mutex_unlock(&cable_notify_sem);
	return 0;
}

#if (defined(CONFIG_USB_OTG) && defined(CONFIG_USB_OTG_HOST))
static DEFINE_MUTEX(usb_host_notify_sem);
static void send_usb_host_connect_notify(int cable_in)
{
	struct t_usb_host_status_notifier *notifier;

	mutex_lock(&usb_host_notify_sem);
	list_for_each_entry(notifier, &g_lh_usb_host_detect_notifier_list, usb_host_notifier_link) {
		if (notifier->func != NULL) {
			CABLE_INFO("[HostNotify] Send to: %s: %d\n", notifier->name, cable_in);
			/* Notify other drivers about connect type. */
			/* use slow charging for unknown type*/
			notifier->func(cable_in);
		}
	}
	mutex_unlock(&usb_host_notify_sem);
}

int usb_host_detect_register_notifier(struct t_usb_host_status_notifier *notifier)
{
	if (!notifier || !notifier->name || !notifier->func)
		return -EINVAL;

	mutex_lock(&usb_host_notify_sem);
	list_add(&notifier->usb_host_notifier_link, &g_lh_usb_host_detect_notifier_list);
	mutex_unlock(&usb_host_notify_sem);
	return 0;
}
#endif  // CONFIG_USB_OTG && CONFIG_USB_OTG_HOST

static void check_vbus_in(struct work_struct *w)
{
#if 1
	struct cable_detect_info *pInfo = container_of(w, struct cable_detect_info, vbus_detect_work.work);
	int g_vbus = usb_get_vbus_value();

	CABLE_INFO("%s: %d\n", __func__, g_vbus);
	if (g_vbus) {
		switch (pInfo->accessory_type) {
			case DOCK_STATE_DESK:
				if (g_desk_no_power) {
					if (gpio_get_value(pInfo->usb_id_pin_gpio)) {
						CABLE_INFO("%s: Cradle removed [ID]\n", __func__);
						g_desk_no_power = 0;
						if (pInfo->config_desk_aud_gpios)
							pInfo->config_desk_aud_gpios(0, 0);
						switch_set_state(&dock_switch, DOCK_STATE_UNDOCKED);
						pInfo->accessory_type = DOCK_STATE_UNDOCKED;
						irq_set_irq_type(pInfo->idpin_irq, IRQF_TRIGGER_LOW);
						enable_irq(pInfo->idpin_irq);
					}
					else {
						CABLE_INFO("%s: Cradle inserted\n", __func__);
						if (pInfo->config_desk_aud_gpios)
							pInfo->config_desk_aud_gpios(1, 1);
						switch_set_state(&dock_switch, DOCK_STATE_DESK);
						g_desk_no_power = 0;
					}
				}
				break;
			default:
				break;
		}
	}
	else {
		switch (pInfo->accessory_type) {
			case DOCK_STATE_DESK:
				CABLE_INFO("%s: Cradle removed\n", __func__);
				if (pInfo->config_desk_aud_gpios)
					pInfo->config_desk_aud_gpios(0, 0);
				switch_set_state(&dock_switch, DOCK_STATE_UNDOCKED);
				pInfo->accessory_type = DOCK_STATE_UNDOCKED;
				irq_set_irq_type(pInfo->idpin_irq, IRQF_TRIGGER_LOW);
				enable_irq(pInfo->idpin_irq);
				break;
			default:
				break;
		}
	}

#else /* Reference */
	int vbus_in;
	int level;
	struct cable_detect_info *pInfo = container_of(
			w, struct cable_detect_info, vbus_detect_work.work);

	level = pm8921_is_usb_chg_plugged_in();
	vbus_in = level;
	CABLE_INFO("%s: vbus = %d, vbus_in = %d\n", __func__, vbus, vbus_in);

/*        FIXME*/
/*         temperary update vbus event to battery driver*/
	send_cable_connect_notify(vbus_in);

#ifdef CONFIG_TEGRA_HDMI_MHL
	if (pInfo->cable_redetect) {
		CABLE_INFO("mhl re-detect\n");
		disable_irq_nosync(pInfo->idpin_irq);
		queue_delayed_work(pInfo->cable_detect_wq,
			&pInfo->cable_detect_work, ADC_DELAY);
	}
#endif

	if (vbus != vbus_in) {
		vbus = vbus_in;

		if (pInfo->usb_uart_switch)
			pInfo->usb_uart_switch(!vbus);
		msm_otg_set_vbus_state(vbus_in);

#ifdef CONFIG_CABLE_DETECT_GPIO_DOCK
		if (pInfo->dock_detect) {
			if (vbus) {
				if (pInfo->dockpin_irq)
					enable_irq(pInfo->dockpin_irq);
			} else {
				disable_irq_nosync(pInfo->dockpin_irq);
				if (cancel_delayed_work_sync(&pInfo->dock_work_isr))
					enable_irq(pInfo->dockpin_irq);

				if (cancel_delayed_work_sync(&pInfo->dock_work)) {
					if (pInfo->dock_pin_state == 0)
						set_irq_type(pInfo->dockpin_irq,
							IRQF_TRIGGER_LOW);
				}
				if (pInfo->accessory_type == DOCK_STATE_DESK) {
					pInfo->dock_pin_state |= 0x80;
					queue_delayed_work(pInfo->cable_detect_wq,
							&pInfo->dock_work, 0);
				}
			}
		}
#endif
	}
	pm8921_chg_enable_usbin_valid_irq();
	CABLE_INFO("%s: Enable usbin_valid_irq ++\n", __func__);
	wake_unlock(&pInfo->vbus_wlock);
#endif
}

static void recovery_host_mode(struct work_struct *w) /* Recovery Mode Only */
{
	switch_set_state(&dock_switch, DOCK_STATE_USB_HOST);
}

#ifdef CONFIG_CABLE_DETECT_ACCESSORY
static int cable_detect_get_type(struct cable_detect_info *pInfo)
{
	int id_pin, adc, type;
	static int prev_type, stable_count;

	if (stable_count >= ADC_RETRY)
		stable_count = 0;

	id_pin = gpio_get_value(pInfo->usb_id_pin_gpio);
	if (id_pin == 0 || pInfo->cable_redetect) {
		CABLE_INFO("%s: id pin low\n", __func__);

		adc = cable_detect_get_adc();
		CABLE_INFO("[1] accessory adc = %d\n", adc);

		if (adc >= 0 && adc < 50)
			type = sec_detect(pInfo);
		else {
			if (adc >= 150 && adc < 170)
				type = DOCK_STATE_CAR;
			else
				type = DOCK_STATE_UNDEFINED;
		}
	}
	else {
		CABLE_INFO("%s: id pin high\n", __func__);
		type = DOCK_STATE_UNDOCKED;
	}

	if (prev_type == type)
		stable_count++;
	else
		stable_count = 0;

	CABLE_INFO("%s prev_type %d, type %d, stable_count %d\n", __func__, prev_type, type, stable_count);

	prev_type = type;
	return (stable_count >= ADC_RETRY) ? type : -2;
}

static void cable_detect_handler(struct work_struct *w)
{
	struct cable_detect_info *pInfo = container_of(
			w, struct cable_detect_info, cable_detect_work.work);
	int value;
	int accessory_type;

	if (pInfo == NULL)
		return;
#ifdef CONFIG_TEGRA_HDMI_MHL
	if (pInfo->mhl_reset_gpio != 0)
		gpio_set_value(pInfo->mhl_reset_gpio, 0); /* Reset Low */
#endif
	if (pInfo->detect_type == CABLE_TYPE_PMIC_ADC) {
		accessory_type = cable_detect_get_type(pInfo);
		if (accessory_type == -2) {
			queue_delayed_work(pInfo->cable_detect_wq, &pInfo->cable_detect_work, ADC_DELAY);
			return;
		}
	} else
		accessory_type = DOCK_STATE_UNDOCKED;

#ifdef CONFIG_TEGRA_HDMI_MHL
	if (pInfo->mhl_reset_gpio != 0)
		gpio_set_value(pInfo->mhl_reset_gpio, 1); /* Reset High */

	if (accessory_type != DOCK_STATE_MHL)
		D2ToD3();
#endif

	switch (accessory_type) {
	case DOCK_STATE_DESK:
		CABLE_INFO("Cradle inserted\n");
		if (usb_get_vbus_value()) {
			if (pInfo->config_desk_aud_gpios)
				pInfo->config_desk_aud_gpios(1, 1);
			switch_set_state(&dock_switch, DOCK_STATE_DESK);
			pInfo->accessory_type = DOCK_STATE_DESK;
		}
		else {
			CABLE_INFO("Cradle no power\n");
			pInfo->accessory_type = DOCK_STATE_DESK;
			g_desk_no_power = 1;
		}
		break;
	case DOCK_STATE_CAR:
		CABLE_INFO("CarKit inserted\n");
		switch_set_state(&dock_switch, DOCK_STATE_CAR);
		pInfo->accessory_type = DOCK_STATE_CAR;
		break;
#if 0  /* USB Headset */
	case DOCK_STATE_USB_HEADSET:
		CABLE_INFO("USB headset inserted\n");
		pInfo->accessory_type = DOCK_STATE_USB_HEADSET;
		if (pInfo->usb_dpdn_switch)
			pInfo->usb_dpdn_switch(PATH_USB_AUD);
#ifdef CONFIG_HTC_HEADSET_MGR
		headset_ext_detect(USB_AUDIO_OUT);
#endif
		break;
#endif
#ifdef CONFIG_TEGRA_HDMI_MHL
	case DOCK_STATE_MHL:
		CABLE_INFO("MHL inserted\n");
		switch_set_state(&dock_switch, DOCK_STATE_MHL);
		pInfo->accessory_type = DOCK_STATE_MHL;
		pInfo->usb_dpdn_switch(PATH_MHL);
#ifdef MHL_INTERNAL_POWER
		if (!pInfo->mhl_internal_3v3 && !g_vbus)
			send_cable_connect_notify(CONNECT_TYPE_INTERNAL);
#endif
		sii9234_mhl_device_wakeup();
		break;
#endif
#if (defined(CONFIG_USB_OTG) && defined(CONFIG_USB_OTG_HOST))
	case DOCK_STATE_USB_HOST:
		CABLE_INFO("USB Host inserted\n");
		send_usb_host_connect_notify(1);
		pInfo->accessory_type = DOCK_STATE_USB_HOST;
		break;
#endif
#if 0  /* DMB */
	case DOCK_STATE_DMB:
		CABLE_INFO("DMB inserted\n");
		send_cable_connect_notify(CONNECT_TYPE_CLEAR);
		switch_set_state(&dock_switch, DOCK_STATE_DMB);
		pInfo->accessory_type = DOCK_STATE_DMB;
		break;
#endif
	case DOCK_STATE_UNDEFINED:
	case DOCK_STATE_UNDOCKED:
		switch (pInfo->accessory_type) {
#if 0  /* In check_vbus_in() */
		case DOCK_STATE_DESK:
			CABLE_INFO("Cradle removed\n");
			if (pInfo->config_desk_aud_gpios)
				pInfo->config_desk_aud_gpios(0, 0);
			switch_set_state(&dock_switch, DOCK_STATE_UNDOCKED);
			g_desk_no_power = 0;
			pInfo->accessory_type = DOCK_STATE_UNDOCKED;
			break;
#endif
		case DOCK_STATE_CAR:
			CABLE_INFO("CarKit removed\n");
			switch_set_state(&dock_switch, DOCK_STATE_UNDOCKED);
			pInfo->accessory_type = DOCK_STATE_UNDOCKED;
			break;
#if 0  /* USB Headset */
		case DOCK_STATE_USB_HEADSET:
			CABLE_INFO("USB Headset removed\n");
#ifdef CONFIG_HTC_HEADSET_MGR
			headset_ext_detect(USB_NO_HEADSET);
#endif
			if (pInfo->usb_dpdn_switch)
				pInfo->usb_dpdn_switch(PATH_USB);
			pInfo->accessory_type = DOCK_STATE_UNDOCKED;
			break;
#endif
#ifdef CONFIG_TEGRA_HDMI_MHL
		case DOCK_STATE_MHL:
			CABLE_INFO("MHL removed\n");
			switch_set_state(&dock_switch, DOCK_STATE_UNDOCKED);
			sii9234_disableIRQ();
			break;
#endif
#if (defined(CONFIG_USB_OTG) && defined(CONFIG_USB_OTG_HOST))
		case DOCK_STATE_USB_HOST:
			CABLE_INFO("USB Host removed\n");
			pInfo->accessory_type = DOCK_STATE_UNDOCKED;
			send_usb_host_connect_notify(0);
			break;
#endif
#if 0  /* DMB */
		case DOCK_STATE_DMB:
			CABLE_INFO("DMB removed\n");
			switch_set_state(&dock_switch, DOCK_STATE_UNDOCKED);
			pInfo->accessory_type = DOCK_STATE_UNDOCKED;
			break;
#endif
		}
	default :
		break;
	}

	pInfo->cable_detecting = 0;
	value = gpio_get_value(pInfo->usb_id_pin_gpio);
	CABLE_INFO("%s ID pin %d, type %d\n", __func__, value, pInfo->accessory_type);

	if (usb_get_vbus_value() && pInfo->accessory_type != DOCK_STATE_UNDOCKED) {
		if (cable_detection_ac_only())
			send_cable_connect_notify(CONNECT_TYPE_AC);
		else
			send_cable_connect_notify(CONNECT_TYPE_UNKNOWN);
	}

	if (pInfo->accessory_type == DOCK_STATE_DESK)
		return;
#ifdef CONFIG_TEGRA_HDMI_MHL
	if (pInfo->accessory_type == DOCK_STATE_MHL)
		return;
#endif
	if (pInfo->accessory_type == DOCK_STATE_UNDOCKED)
		irq_set_irq_type(pInfo->idpin_irq, value ? IRQF_TRIGGER_LOW : IRQF_TRIGGER_HIGH);
	else
		irq_set_irq_type(pInfo->idpin_irq, IRQF_TRIGGER_HIGH);

	enable_irq(pInfo->idpin_irq);
}

void set_mfg_usb_carkit_enable(int enable)
{
	the_cable_info.mfg_usb_carkit_enable = enable;
}

int cable_get_accessory_type(void)
{
	return the_cable_info.accessory_type;
}

static int32_t cable_detect_get_adc(void)
{
	struct cable_detect_info *pInfo = &the_cable_info;
	return pInfo->get_adc_cb();
}

int cable_get_usb_id_level(void)
{
	struct cable_detect_info *pInfo = &the_cable_info;

	if (pInfo->usb_id_pin_gpio)
		return gpio_get_value(pInfo->usb_id_pin_gpio);
	else {
		printk(KERN_INFO "usb id is not defined\n");
		return 1;
	}
}

static int sec_detect(struct cable_detect_info *pInfo)
{
	uint32_t adc_value = ~0;
	int type;

	if (pInfo->config_usb_id_gpios)
		pInfo->config_usb_id_gpios(1);

	adc_value = cable_detect_get_adc();
	CABLE_INFO("[2] accessory adc = %d\n", adc_value);

	if (adc_value >= 1000 && adc_value < 1200)
		type = DOCK_STATE_DESK;
	else if ((adc_value >= 800 && adc_value < 1000) || pInfo->mhl_version_ctrl_flag)
#ifdef CONFIG_TEGRA_HDMI_MHL
		type = DOCK_STATE_MHL;
#else
		type = DOCK_STATE_UNDEFINED;
#endif
	else
#if (defined(CONFIG_USB_OTG) && defined(CONFIG_USB_OTG_HOST))
		type = DOCK_STATE_USB_HOST;
#else
		type = DOCK_STATE_UNDEFINED;
#endif

	if (pInfo->config_usb_id_gpios)
		pInfo->config_usb_id_gpios(0);

	return type;
}

static int get_usb_id_adc(char *buffer, struct kernel_param *kp)
{
	unsigned length = 0;
	int adc;

	adc = cable_detect_get_adc();
	length += sprintf(buffer, "%d\n", adc);

	return length;
}
module_param_call(usb_id_adc, NULL, get_usb_id_adc, NULL, 0664);

static ssize_t dock_status_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct cable_detect_info *pInfo = &the_cable_info;

	if (pInfo->accessory_type == 1)
		return sprintf(buf, "online\n");
	else if (pInfo->accessory_type == 3) /*desk dock*/
		return sprintf(buf, "online\n");
	else
		return sprintf(buf, "offline\n");
}
static DEVICE_ATTR(status, S_IRUGO | S_IWUSR, dock_status_show, NULL);

static irqreturn_t usbid_interrupt(int irq, void *data)
{
	struct cable_detect_info *pInfo = (struct cable_detect_info *)data;

	disable_irq_nosync(pInfo->idpin_irq);
	pInfo->cable_detecting = 1;
	pInfo->cable_redetect = 0;
	CABLE_INFO("usb: id interrupt\n");
	queue_delayed_work(pInfo->cable_detect_wq, &pInfo->cable_detect_work, ADC_DELAY);
	return IRQ_HANDLED;
}

static void usb_id_detect_init(struct cable_detect_info *pInfo)
{
	int ret;

	if (pInfo->usb_id_pin_gpio == 0)
		return;

	pInfo->idpin_irq = gpio_to_irq(pInfo->usb_id_pin_gpio);
	if (pInfo->idpin_irq < 0) {
		CABLE_ERR("%s: gpio_to_irq failed", __func__);
		return;
	}

	pInfo->idpin_irq = (unsigned int)pInfo->idpin_irq;
	ret = request_irq(pInfo->idpin_irq, usbid_interrupt, IRQF_TRIGGER_LOW, "idpin_irq", pInfo);
	if (ret < 0) {
		CABLE_ERR("%s: request_irq failed\n", __func__);
		return;
	}
#if 0 /* IRQ Wake */
	ret = enable_irq_wake(pInfo->idpin_irq);
	if (ret < 0) {
		CABLE_ERR("%s: set_irq_wake failed\n", __func__);
		goto err;
	}
#endif
	CABLE_INFO("%s: ID=%d, IRQ=0x%x", __func__, pInfo->usb_id_pin_gpio, pInfo->idpin_irq);
	return;
#if 0 /* IRQ Wake */
err:
	free_irq(pInfo->idpin_irq, 0);
#endif
}

#ifdef CONFIG_TEGRA_HDMI_MHL
static void mhl_status_notifier_func(bool isMHL, int charging_type)
{
	struct cable_detect_info *pInfo = &the_cable_info;
	int id_pin = gpio_get_value(pInfo->usb_id_pin_gpio);
	static uint8_t mhl_connected;

	CABLE_INFO("%s: isMHL %d, charging type %d, id_pin %d\n", __func__, isMHL, charging_type, id_pin);
	if (pInfo->accessory_type != DOCK_STATE_MHL) {
		CABLE_INFO("%s: accessory is not MHL, type %d\n", __func__, pInfo->accessory_type);
		return;
	}
#if 0  /* Headset */
#ifdef CONFIG_HTC_HEADSET_MISC
	headset_mhl_audio_jack_enable(isMHL);
#endif
#endif
	if (!isMHL) {
		CABLE_INFO("MHL removed\n");

		if (pInfo->usb_dpdn_switch)
			pInfo->usb_dpdn_switch(PATH_USB);

		if (pInfo->mhl_1v2_power)
			pInfo->mhl_1v2_power(0);
#ifdef MHL_INTERNAL_POWER
		send_cable_connect_notify(CONNECT_TYPE_CLEAR);
#endif
#ifdef MHL_REDETECT
		if (mhl_connected == 0) {
			CABLE_INFO("MHL re-detect\n");
			irq_set_irq_type(pInfo->idpin_irq, id_pin ? IRQF_TRIGGER_LOW : IRQF_TRIGGER_HIGH);
			pInfo->cable_redetect = 1;
		}
#endif
		mhl_connected = 0;

		pInfo->accessory_type = DOCK_STATE_UNDOCKED;
		sii9234_disableIRQ();
		enable_irq(pInfo->idpin_irq);
		return;
	}
	else {
		mhl_connected = 1;
		irq_set_irq_type(pInfo->idpin_irq, id_pin ? IRQF_TRIGGER_LOW : IRQF_TRIGGER_HIGH);
#if 0
		if (g_vbus && (charging_type > CONNECT_TYPE_NONE))
			send_cable_connect_notify(charging_type);
#endif
#if 0
#ifdef MHL_INTERNAL_POWER
		else if (g_vbus)
			send_cable_connect_notify(CONNECT_TYPE_USB);
		if (pInfo->config_desk_aud_gpios)
			pInfo->config_desk_aud_gpios(1, 1);
#endif
#endif
	}
}

static struct t_mhl_status_notifier mhl_status_notifier = {
	.name = "mhl_detect",
	.func = mhl_status_notifier_func,
};
#endif /*CONFIG_TEGRA_HDMI_MHL*/
#endif /*CONFIG_CABLE_DETECT_ACCESSORY*/

#ifdef CONFIG_CABLE_DETECT_GPIO_DOCK
static irqreturn_t dock_interrupt(int irq, void *data)
{
	struct cable_detect_info *pInfo = data;
	disable_irq_nosync(pInfo->dockpin_irq);
	cancel_delayed_work(&pInfo->dock_work);
	queue_delayed_work(pInfo->cable_detect_wq, &pInfo->dock_work_isr, DOCK_DET_DELAY);
	return IRQ_HANDLED;
}
static void dock_isr_work(struct work_struct *w)
{
	struct cable_detect_info *pInfo = container_of(w, struct cable_detect_info, dock_work_isr.work);
	pInfo->dock_pin_state = gpio_get_value(pInfo->dock_pin_gpio);

	if (pInfo->dock_pin_state == 1)
		set_irq_type(pInfo->dockpin_irq, IRQF_TRIGGER_LOW);
	else
		set_irq_type(pInfo->dockpin_irq, IRQF_TRIGGER_HIGH);

	queue_delayed_work(pInfo->cable_detect_wq, &pInfo->dock_work, DOCK_DET_DELAY);
	enable_irq(pInfo->dockpin_irq);
}
static void dock_detect_work(struct work_struct *w)
{
	struct cable_detect_info *pInfo = container_of(
			w, struct cable_detect_info, dock_work.work);
	int value;

	value = gpio_get_value(pInfo->dock_pin_gpio);
	CABLE_INFO("%s: dock_pin = %s\n", __func__, value ? "high" : "low");
	if (pInfo->dock_pin_state != value && (pInfo->dock_pin_state & 0x80) == 0) {
		CABLE_ERR("%s: dock_pin_state changed\n", __func__);
		return;
	}

	if (value == 0 && g_vbus) {
		if (pInfo->accessory_type == DOCK_STATE_DESK)
			return;
		set_irq_type(pInfo->dockpin_irq, IRQF_TRIGGER_HIGH);
		switch_set_state(&dock_switch, DOCK_STATE_DESK);
		pInfo->accessory_type = DOCK_STATE_DESK;
		CABLE_INFO("dock: set state %d\n", DOCK_STATE_DESK);
	}
	else {
		if (pInfo->accessory_type == DOCK_STATE_UNDOCKED)
			return;
		set_irq_type(pInfo->dockpin_irq, IRQF_TRIGGER_LOW);
		switch_set_state(&dock_switch, DOCK_STATE_UNDOCKED);
		pInfo->accessory_type = DOCK_STATE_UNDOCKED;
		CABLE_INFO("dock: set state %d\n", DOCK_STATE_UNDOCKED);
	}
}
static void dock_detect_init(struct cable_detect_info *pInfo)
{
	int ret;

	CABLE_INFO("%s in, gpio %d, irq %d\n",
			__func__, pInfo->dock_pin_gpio, pInfo->dockpin_irq);
	if (pInfo->dock_pin_gpio == 0)
		return;
	if (pInfo->dockpin_irq == 0)
		pInfo->dockpin_irq = gpio_to_irq(pInfo->dock_pin_gpio);

	set_irq_flags(pInfo->dockpin_irq, IRQF_VALID | IRQF_NOAUTOEN);
	ret = request_any_context_irq(pInfo->dockpin_irq, dock_interrupt, IRQF_TRIGGER_LOW, "dock_irq", pInfo);
	if (ret < 0) {
		CABLE_ERR("[GPIO DOCK] %s: request_irq failed\n", __func__);
		return;
	}
	CABLE_INFO("%s: dock irq %d\n", __func__, pInfo->dockpin_irq);

	if (g_vbus)
		enable_irq(pInfo->dockpin_irq);
}
#endif // CONFIG_CABLE_DETECT_GPIO_DOCK

static ssize_t vbus_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int vbus_in = 0;

	vbus_in = usb_get_vbus_value();
	CABLE_INFO("%s: vbus state = %d\n", __func__, vbus_in);
	return sprintf(buf, "%d\n", vbus_in);
}
static DEVICE_ATTR(vbus, S_IRUGO | S_IWUSR, vbus_status_show, NULL);

#ifdef CONFIG_CABLE_DETECT_ACCESSORY
static ssize_t adc_status_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int adc;

	adc = cable_detect_get_adc();
	CABLE_INFO("%s: ADC = %d\n", __func__, adc);
	return sprintf(buf, "%d\n", adc);
}
static DEVICE_ATTR(adc, S_IRUGO | S_IWUSR, adc_status_show, NULL);

#if 0 /* DMB */
static ssize_t dmb_wakeup_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct cable_detect_info *pInfo = &the_cable_info;
	uint32_t wakeup;

	if (pInfo->accessory_type != DOCK_STATE_DMB) {
		CABLE_INFO("%s: DMB not exist. Do nothing.\n", __func__);
		return count;
	}

	sscanf(buf, "%d", &wakeup);
	CABLE_DEBUG("%s: wakeup = %d\n", __func__, wakeup);
	if (!!wakeup) {
		disable_irq_nosync(pInfo->idpin_irq);

		gpio_direction_output(pInfo->usb_id_pin_gpio, 0);
		msleep(1);
		gpio_direction_output(pInfo->usb_id_pin_gpio, 1);
		msleep(10);
		gpio_direction_output(pInfo->usb_id_pin_gpio, 0);
		msleep(1);

		gpio_direction_input(pInfo->usb_id_pin_gpio);
		enable_irq(pInfo->idpin_irq);
	}
	CABLE_INFO("%s(parent:%s): request DMB wakeup done.\n", current->comm, current->parent->comm);

	return count;
}
static DEVICE_ATTR(dmb_wakeup, S_IRUGO | S_IWUSR, NULL, dmb_wakeup_store);
#endif
#endif  // CONFIG_CABLE_DETECT_ACCESSORY

int cable_get_connect_type(void)
{
	struct cable_detect_info *pInfo = &the_cable_info;

	return pInfo->connect_type;
}

static int cable_detect_probe(struct platform_device *pdev)
{
	int ret;
	struct cable_detect_platform_data *pdata = pdev->dev.platform_data;
	struct cable_detect_info *pInfo = &the_cable_info;

	spin_lock_init(&the_cable_info.lock);

	if (pdata) {
		/*pInfo->vbus_mpp_gpio = pdata->vbus_mpp_gpio;*/
		/*pInfo->vbus_mpp_irq = pdata->vbus_mpp_irq;*/
		/*pInfo->usb_uart_switch = pdata->usb_uart_switch;*/
		pInfo->usb_dpdn_switch = pdata->usb_dpdn_switch;
		if (pInfo->usb_dpdn_switch)
			pInfo->usb_dpdn_switch(PATH_USB);
		pInfo->mhl_internal_3v3 = pdata->mhl_internal_3v3;
		pInfo->cable_detecting = 0;

#ifdef CONFIG_CABLE_DETECT_ACCESSORY
		pInfo->detect_type = pdata->detect_type;
		pInfo->usb_id_pin_gpio = pdata->usb_id_pin_gpio;
		pInfo->mhl_reset_gpio = pdata->mhl_reset_gpio;
		pInfo->mpp_data = &pdata->mpp_data;
		pInfo->config_usb_id_gpios = pdata->config_usb_id_gpios;
		pInfo->config_desk_aud_gpios = pdata->config_desk_aud_gpios;
		pInfo->mhl_version_ctrl_flag = pdata->mhl_version_ctrl_flag;
		pInfo->mhl_1v2_power = pdata->mhl_1v2_power;
		pInfo->get_adc_cb = pdata->get_adc_cb;
		pInfo->cable_gpio_init = pdata->cable_gpio_init;

		if (pInfo->cable_gpio_init)
			pInfo->cable_gpio_init();

		if (pInfo->config_usb_id_gpios)
			pInfo->config_usb_id_gpios(0);
#endif
#ifdef CONFIG_CABLE_DETECT_GPIO_DOCK
		pInfo->dock_detect = pdata->dock_detect;
		pInfo->dock_pin_gpio = pdata->dock_pin_gpio;
#endif
#if 0 /* Wireless Charger */
		if (pdata->is_wireless_charger)
			pInfo->is_wireless_charger = pdata->is_wireless_charger;
#endif
#ifdef CONFIG_CABLE_DETECT_ACCESSORY
		INIT_DELAYED_WORK(&pInfo->cable_detect_work, cable_detect_handler);
#endif
		INIT_DELAYED_WORK(&pInfo->vbus_detect_work, check_vbus_in);
		INIT_DELAYED_WORK(&pInfo->recovery_host_work, recovery_host_mode);

		pInfo->cable_detect_wq = create_workqueue("cable_detect");
		if (pInfo->cable_detect_wq == 0) {
			CABLE_ERR("usb: fail to create workqueue\n");
			return -ENOMEM;
		}
#if 0 /* VBUS */
		if (pdata->vbus_mpp_config)
			pdata->vbus_mpp_config();

		wake_lock_init(&pInfo->vbus_wlock,
			WAKE_LOCK_SUSPEND, "vbus_lock");
#endif
#ifdef CONFIG_CABLE_DETECT_ACCESSORY
		wake_lock_init(&pInfo->cable_detect_wlock, WAKE_LOCK_SUSPEND, "cable_detect_lock");
#endif
#if 0 /* VBUS MAPP GPIO */
		if (pdata->vbus_mpp_gpio) {
			gpio_request(pdata->vbus_mpp_gpio, "vbus_cable_detect");
			CABLE_INFO("vbus_mpp_gpio: %d\n", pdata->vbus_mpp_gpio);

			set_irq_flags(pdata->vbus_mpp_irq, IRQF_VALID | IRQF_NOAUTOEN);
			ret = request_any_context_irq(
					pdata->vbus_mpp_irq, vbus_irq_handler,
					IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING,
					"vbus_irq", pInfo);
			set_irq_wake(pdata->vbus_mpp_irq, 1);
		}
#endif
	}
	if (switch_dev_register(&dock_switch) < 0) {
		CABLE_ERR("fail to register dock switch!\n");
		return 0;
	}

	ret = device_create_file(dock_switch.dev, &dev_attr_vbus);
	if (ret != 0)
		CABLE_ERR("dev_attr_vbus failed\n");

#ifdef CONFIG_CABLE_DETECT_ACCESSORY
	ret = device_create_file(dock_switch.dev, &dev_attr_status);
	if (ret != 0)
		CABLE_ERR("dev_attr_status failed\n");

	ret = device_create_file(dock_switch.dev, &dev_attr_adc);
	if (ret != 0)
		CABLE_ERR("dev_attr_adc failed\n");
#if 0 /* DMB */
	ret = device_create_file(dock_switch.dev, &dev_attr_dmb_wakeup);
	if (ret != 0)
		CABLE_ERR("dev_attr_dmb_wakeup failed\n");
#endif
	usb_id_detect_init(pInfo);
#endif  // CONFIG_CABLE_DETECT_ACCESSORY

#ifdef CONFIG_CABLE_DETECT_GPIO_DOCK
	if (pInfo->dock_detect) {
		INIT_DELAYED_WORK(&pInfo->dock_work_isr, dock_isr_work);
		INIT_DELAYED_WORK(&pInfo->dock_work, dock_detect_work);
		dock_detect_init(pInfo);
	}
#endif
	CABLE_INFO("%s: Done\n", __func__);
	return 0;
}

#if 0 /* VBUS IRQ */
static irqreturn_t vbus_irq_handler(int irq, void *dev_id)
{
	unsigned long flags;
	struct cable_detect_info *pInfo = (struct cable_detect_info *)dev_id;

	disable_irq_nosync(irq);
	CABLE_INFO("%s: Disable vbus irq --\n", __func__);
	spin_lock_irqsave(&pInfo->lock, flags);
	queue_delayed_work(pInfo->cable_detect_wq, &pInfo->vbus_detect_work, HZ/10);
	spin_unlock_irqrestore(&pInfo->lock, flags);
	/*wake_lock_timeout(&pInfo->vbus_wlock, HZ*2);*/

	return IRQ_HANDLED;
}

irqreturn_t cable_detection_vbus_irq_handler(void)
{
	unsigned long flags;
	struct cable_detect_info *pInfo = &the_cable_info;

	pm8921_chg_disable_usbin_valid_irq();
	CABLE_INFO("%s: Disable usbin_valid_irq --\n", __func__);
	spin_lock_irqsave(&pInfo->lock, flags);
	queue_delayed_work(pInfo->cable_detect_wq, &pInfo->vbus_detect_work, HZ/10);
	spin_unlock_irqrestore(&pInfo->lock, flags);
	wake_lock_timeout(&pInfo->vbus_wlock, HZ*2);

	return IRQ_HANDLED;
}
EXPORT_SYMBOL(cable_detection_vbus_irq_handler);
#endif /* VBUS IRQ */

bool cable_detection_ac_only(void)
{
	bool ret = 0;
	struct cable_detect_info *pInfo = &the_cable_info;

	switch (pInfo->accessory_type) {
		case DOCK_STATE_CAR:
		case DOCK_STATE_DESK:
		case DOCK_STATE_MHL:
			ret = 1;
			break;
		default:
			break;
	}
	return ret;
}
EXPORT_SYMBOL(cable_detection_ac_only);

bool cable_detection_det(void)
{
	struct cable_detect_info *pInfo = &the_cable_info;
	return pInfo->cable_detecting;
}
EXPORT_SYMBOL(cable_detection_det);

void cable_detection_queue_recovery_host_work(int time)
{
	struct cable_detect_info *pInfo = &the_cable_info;

	if (pInfo->cable_detect_wq) {
		CABLE_INFO("%s: time=%d\n", __func__, time);
		queue_delayed_work(pInfo->cable_detect_wq, &pInfo->recovery_host_work, time);
	}
}
EXPORT_SYMBOL(cable_detection_queue_recovery_host_work);

void cable_detection_queue_vbus_work(int time)
{
	struct cable_detect_info *pInfo = &the_cable_info;

	if (pInfo->cable_detect_wq)
		queue_delayed_work(pInfo->cable_detect_wq, &pInfo->vbus_detect_work, time);
}
EXPORT_SYMBOL(cable_detection_queue_vbus_work);

struct platform_driver cable_detect_driver = {
	.probe = cable_detect_probe,
	/*.remove = __devexit_p(vbus_cable_detect_remove),*/
	.driver = {
		.name = "cable_detect",
		.owner = THIS_MODULE,
	},
};

static void usb_status_notifier_func(int cable_type)
{
	struct cable_detect_info*pInfo = &the_cable_info;

	CABLE_INFO("%s: cable_type = %d\n", __func__, cable_type);
#if 0  /* Wireless Charger */
	if (cable_type > CONNECT_TYPE_NONE) {
		if (pInfo->is_wireless_charger) {
			if (pInfo->is_wireless_charger())
				cable_type = CONNECT_TYPE_WIRELESS;
		}
	}
#endif
#ifdef CONFIG_TEGRA_HDMI_MHL
#ifdef MHL_INTERNAL_POWER
	if (!pInfo->mhl_internal_3v3 && pInfo->accessory_type == DOCK_STATE_MHL) {
		CABLE_INFO("%s: MHL detected. Do nothing\n", __func__);
		return;
	}
#endif
#endif
	pInfo->connect_type = cable_type;
	send_cable_connect_notify(cable_type);
}

static struct t_usb_status_notifier usb_status_notifier = {
	.name = "cable_detect",
	.func = usb_status_notifier_func,
};

static int __init cable_detect_init(void)
{
	the_cable_info.connect_type = CONNECT_TYPE_NONE;
	/*usb_register_notifier(&usb_status_notifier);*/
#if (defined(CONFIG_CABLE_DETECT_ACCESSORY) && defined(CONFIG_TEGRA_HDMI_MHL))
	mhl_detect_register_notifier(&mhl_status_notifier);
#endif
	return platform_driver_register(&cable_detect_driver);
}

static void __exit cable_detect_exit(void)
{
	platform_driver_unregister(&cable_detect_driver);
}

MODULE_DESCRIPTION("CABLE_DETECT");
MODULE_LICENSE("GPL");

module_init(cable_detect_init);
module_exit(cable_detect_exit);
