#include <mach/cable_detect.h>
#include <linux/wakelock.h>
#include <linux/usb/gadget.h>
#include <mach/board_htc.h>
#include <mach/usb_phy.h>

static int first_online;

static struct wake_lock udc_wake_lock;
static struct wake_lock udc_wake_lock2;
struct wake_lock udc_resume_wake_lock;
static DEFINE_MUTEX(notify_sem);

static void update_wake_lock(int status);
static void ac_detect_expired(unsigned long _data);
static void usb_prepare(struct tegra_udc *udc);
static void charger_detect(struct tegra_udc *udc);

static void send_usb_connect_notify(struct work_struct *w)
{
	static struct t_usb_status_notifier *notifier;
	struct tegra_udc *udc = container_of(w, struct tegra_udc, notifier_work);
	if (!udc)
		return;

	USB_INFO("send connect type %d\n", udc->connect_type);
	update_wake_lock(udc->connect_type);
	mutex_lock(&notify_sem);
	list_for_each_entry(notifier,
		&g_lh_usb_notifier_list,
		notifier_link) {
			if (notifier->func != NULL) {
				/* Notify other drivers about connect type. */
				/* use slow charging for unknown type*/
				if (udc->connect_type == CONNECT_TYPE_UNKNOWN)
					notifier->func(CONNECT_TYPE_USB);
				else
					notifier->func(udc->connect_type);
			}
		}
	mutex_unlock(&notify_sem);

}
#ifdef CONFIG_HTC_USB_NOTIFIER
int usb_register_notifier(struct t_usb_status_notifier *notifier)
{
	if (!notifier || !notifier->name || !notifier->func)
		return -EINVAL;

	mutex_lock(&notify_sem);
	list_add(&notifier->notifier_link, &g_lh_usb_notifier_list);
#if 1
	if (the_udc) {
		if (vbus_enabled(the_udc))
			charger_detect(the_udc);
		else
			notifier->func(CONNECT_TYPE_NONE);
	}
#endif
	mutex_unlock(&notify_sem);
	return 0;
}
#endif


static void update_wake_lock(int status)
{
	if ((status == CONNECT_TYPE_USB || status == CONNECT_TYPE_UNKNOWN) && board_mfg_mode() != BOARD_MFG_MODE_OFFMODE_CHARGING) {
		wake_lock(&udc_wake_lock);
		USB_INFO("wake_lock");
	} else {
		wake_lock_timeout(&udc_wake_lock, 5*HZ);
		USB_INFO("wake_lock_timeout 5*HZ");
	}
}
#define DELAY_FOR_CHECK_CHG msecs_to_jiffies(300)
static void charger_detect(struct tegra_udc *udc)
{
	u32 portsc;
	u32 ret;
	unsigned long flags;

	msleep(10);
	/* detect shorted D+/D-, indicating AC power */
	spin_lock_irqsave(&udc->lock, flags);
	portsc = udc_readl(udc, PORTSCX_REG_OFFSET);
	ret = (portsc & PORTSCX_LINE_STATUS_BITS);
	spin_unlock_irqrestore(&udc->lock, flags);

#if defined(CONFIG_CABLE_DETECT_ACCESSORY)
	if (udc->connect_type == CONNECT_TYPE_INTERNAL) {
		USB_INFO("%s: internrl, return\n", __func__);
		return;
	}

	if (ret != PORTSCX_LINE_STATUS_BITS && !cable_detection_ac_only())
#else
	if (ret != PORTSCX_LINE_STATUS_BITS)
#endif
	{
		USB_INFO("Charger :USB [portsc:%x]\n", portsc);
		udc->connect_type = CONNECT_TYPE_UNKNOWN;
		queue_delayed_work(udc->usb_wq, &udc->chg_work,
			DELAY_FOR_CHECK_CHG);
		queue_delayed_work(system_nrt_wq, &udc->ac_detect_work, 3 * HZ);
	} else {
		USB_INFO("Charger :AC [portsc:%x]\n", portsc);
		udc->connect_type = CONNECT_TYPE_AC;
		queue_work(udc->usb_wq, &udc->notifier_work);
	}

}

static void check_charger(struct work_struct *w)
{
	struct tegra_udc *udc = container_of(w, struct tegra_udc, chg_work.work);
	/* unknown charger */
	if (vbus_enabled(udc) && udc->connect_type == CONNECT_TYPE_UNKNOWN)
		queue_work(udc->usb_wq, &udc->notifier_work);
}

int usb_get_connect_type(void)
{
	if (!the_udc)
		return 0;
	return the_udc->connect_type;
}
EXPORT_SYMBOL(usb_get_connect_type);


static void ac_detect_expired_work(struct work_struct *w)
{
	struct tegra_udc *udc = the_udc;
	u32 delay = 0;
	u32 portsc;
	u32 ret;
	USB_INFO("%s: count = %d, connect_type = 0x%04x\n", __func__,
			udc->ac_detect_count, udc->connect_type);

	if (udc->connect_type == CONNECT_TYPE_USB || udc->ac_detect_count >= 3)
		return;
#if defined(CONFIG_CABLE_DETECT_ACCESSORY)
	if (cable_detection_det() || cable_detection_ac_only())
		return;
#endif
	/* detect shorted D+/D-, indicating AC power */
	portsc = udc_readl(udc, PORTSCX_REG_OFFSET);
	ret = (portsc & PORTSCX_LINE_STATUS_BITS);
	if (ret != PORTSCX_LINE_STATUS_BITS) {
		/* Some carkit can't be recognized as AC mode.
		 * Add SW solution here to notify battery driver should
		 * work as AC charger when car mode activated.
		 */
		udc->ac_detect_count++;
		/* detect delay: 3 sec, 5 sec, 10 sec */
		if (udc->ac_detect_count == 1)
			delay = 5 * HZ;
		else if (udc->ac_detect_count == 2)
			delay = 10 * HZ;

		queue_delayed_work(system_nrt_wq, &udc->ac_detect_work, delay);
	} else {
		USB_INFO("USB -> AC charger ret(%d)\n", ret == PORTSCX_LINE_STATUS_BITS);
		udc->connect_type = CONNECT_TYPE_AC;
		queue_work(udc->usb_wq, &udc->notifier_work);
		udc->ac_detect_count = 0;
	}
}

#if defined(CONFIG_CABLE_DETECT_ACCESSORY)
void cable_status_notifier_func(int cable_type)
{
	if (cable_type != CONNECT_TYPE_NONE) {
		USB_INFO("%s: cable=%d\n", __func__, cable_type);
		the_udc->connect_type = cable_type;
		queue_work(the_udc->usb_wq, &the_udc->notifier_work);
	}
}

static struct t_cable_status_notifier cable_status_notifier = {
	.name = "cable_charge",
	.func = cable_status_notifier_func,
};
#endif

static void usb_prepare(struct tegra_udc *udc)
{
	udc->usb_wq = create_singlethread_workqueue("tegra_udc");
	if (udc->usb_wq == 0) {
		USB_ERR("usb_prepare fail to create workqueue\n");
		return;
	}
	INIT_WORK(&udc->notifier_work, send_usb_connect_notify);
	INIT_DELAYED_WORK(&udc->chg_work, check_charger);
	INIT_DELAYED_WORK(&udc->ac_detect_work, ac_detect_expired_work);
	wake_lock_init(&udc_wake_lock, WAKE_LOCK_SUSPEND, "usb_udc_lock");
	wake_lock_init(&udc_wake_lock2, WAKE_LOCK_SUSPEND, "usb_udc_lock2");
	wake_lock_init(&udc_resume_wake_lock, WAKE_LOCK_SUSPEND, "usb_udc_resume_lock");

	udc->ac_detect_count = 0;

#if defined(CONFIG_CABLE_DETECT_ACCESSORY)
	cable_detect_register_notifier(&cable_status_notifier);
#endif
}
