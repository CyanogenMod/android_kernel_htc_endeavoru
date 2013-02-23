/*
 * Driver for keys on GPIO lines capable of generating interrupts.
 *
 * Copyright 2005 Phil Blundell
 *
 * Copyright 2010-2011 NVIDIA Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>

#include <linux/init.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/gpio_keys.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/wakelock.h>

#include <asm/gpio.h>
#include <linux/cm3629.h>

#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_SWEEP2WAKE
#include <linux/synaptics_i2c_rmi.h>
#endif

struct gpio_button_data {
	struct gpio_keys_button *button;
	struct input_dev *input;
	struct timer_list timer;
	struct work_struct work;
	int timer_debounce;	/* in msecs */
	bool disabled;
};

struct gpio_keys_drvdata {
	struct input_dev *input;
	struct mutex disable_lock;
	unsigned int n_buttons;
	int (*enable)(struct device *dev);
	void (*disable)(struct device *dev);
	struct gpio_button_data data[0];
};

struct wake_lock power_key_wake_lock;
/*extern int resume_from_deep_suspend;*/
extern bool is_resume_from_deep_suspend(void);
static volatile bool doCheck = true;

#define PWRKEYCHKRST_DELAY (3*HZ + HZ/2)
#define PWRKEYCLRCHK_DELAY 0
#define PWRKEYCHKRST_WAKELOCK_TIMEOUT (PWRKEYCHKRST_DELAY + 1 * HZ)
struct wake_lock key_reset_clr_wake_lock;

static int PWR_MISTOUCH_gpio = 0;
static volatile int Mistouch_powerkey_requested = 0;
static volatile int Powerkey_pressed = 0;
static struct workqueue_struct *delay_wq;
static struct delayed_work      delay_work;

static int (*mistouch_gpio_normal)(void); /* change the gpio to nonactive status */
static int (*mistouch_gpio_active)(void); /* change the gpio to active status */

static void Mistouch_powerkey_func(struct work_struct *work)
{
	Mistouch_powerkey_requested = power_key_check_in_pocket();
	pr_info("[KEY] %s in (%x)\n", __func__, Mistouch_powerkey_requested);
	if ( Mistouch_powerkey_requested ) {
		if ( mistouch_gpio_active != NULL )
			mistouch_gpio_active();
		else
			pr_err("[KEY] mistouch_gpio_active() is NULL\n");
	}
	else {
		if ( mistouch_gpio_normal != NULL )
			mistouch_gpio_normal();
		else
			pr_err("[KEY] mistouch_gpio_normal() is NULL\n");
	}
	if ( Powerkey_pressed ) {
		wake_lock_timeout(&key_reset_clr_wake_lock, PWRKEYCHKRST_WAKELOCK_TIMEOUT);
		queue_delayed_work(delay_wq, &delay_work, msecs_to_jiffies(3500));
	}
}

static DEFINE_MUTEX(wakeup_mutex);
static unsigned char wakeup_bitmask;
static unsigned char set_wakeup;
static unsigned int vol_up_irq;
static unsigned int vol_down_irq;
/* for change the volup/voldown wakeup status */
static ssize_t vol_wakeup_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	unsigned char bitmask = 0;
	bitmask = simple_strtoull(buf, NULL, 10);
	mutex_lock(&wakeup_mutex);
	if (bitmask) {
		if (bitmask == 127)
			wakeup_bitmask &= bitmask;
		else if (bitmask > 128)
			wakeup_bitmask &= bitmask;
		else
			wakeup_bitmask |= bitmask;
	}

	if (wakeup_bitmask && (!set_wakeup)) {
		enable_irq_wake(vol_up_irq);
		enable_irq_wake(vol_down_irq);
		set_wakeup = 1;
		printk(KERN_INFO "%s:change to wake up function(%d, %d)\n", __func__, vol_up_irq, vol_down_irq);
	} else if ((!wakeup_bitmask) && set_wakeup){
		disable_irq_wake(vol_up_irq);
		disable_irq_wake(vol_down_irq);
		set_wakeup = 0;
		printk(KERN_INFO "%s:change to non-wake up function(%d, %d)\n", __func__, vol_up_irq, vol_down_irq);
	}
	mutex_unlock(&wakeup_mutex);
	return count;
}
static ssize_t vol_wakeup_show(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%x\n", wakeup_bitmask);
}

static DEVICE_ATTR(vol_wakeup, 0664, vol_wakeup_show, vol_wakeup_store);
/*
 * SYSFS interface for enabling/disabling keys and switches:
 *
 * There are 4 attributes under /sys/devices/platform/gpio-keys/
 *	keys [ro]              - bitmap of keys (EV_KEY) which can be
 *	                         disabled
 *	switches [ro]          - bitmap of switches (EV_SW) which can be
 *	                         disabled
 *	disabled_keys [rw]     - bitmap of keys currently disabled
 *	disabled_switches [rw] - bitmap of switches currently disabled
 *
 * Userland can change these values and hence disable event generation
 * for each key (or switch). Disabling a key means its interrupt line
 * is disabled.
 *
 * For example, if we have following switches set up as gpio-keys:
 *	SW_DOCK = 5
 *	SW_CAMERA_LENS_COVER = 9
 *	SW_KEYPAD_SLIDE = 10
 *	SW_FRONT_PROXIMITY = 11
 * This is read from switches:
 *	11-9,5
 * Next we want to disable proximity (11) and dock (5), we write:
 *	11,5
 * to file disabled_switches. Now proximity and dock IRQs are disabled.
 * This can be verified by reading the file disabled_switches:
 *	11,5
 * If we now want to enable proximity (11) switch we write:
 *	5
 * to disabled_switches.
 *
 * We can disable only those keys which don't allow sharing the irq.
 */

/**
 * get_n_events_by_type() - returns maximum number of events per @type
 * @type: type of button (%EV_KEY, %EV_SW)
 *
 * Return value of this function can be used to allocate bitmap
 * large enough to hold all bits for given type.
 */
static inline int get_n_events_by_type(int type)
{
	BUG_ON(type != EV_SW && type != EV_KEY);

	return (type == EV_KEY) ? KEY_CNT : SW_CNT;
}

/**
 * gpio_keys_disable_button() - disables given GPIO button
 * @bdata: button data for button to be disabled
 *
 * Disables button pointed by @bdata. This is done by masking
 * IRQ line. After this function is called, button won't generate
 * input events anymore. Note that one can only disable buttons
 * that don't share IRQs.
 *
 * Make sure that @bdata->disable_lock is locked when entering
 * this function to avoid races when concurrent threads are
 * disabling buttons at the same time.
 */
static void gpio_keys_disable_button(struct gpio_button_data *bdata)
{
	if (!bdata->disabled) {
		/*
		 * Disable IRQ and possible debouncing timer.
		 */
		disable_irq(gpio_to_irq(bdata->button->gpio));
		if (bdata->timer_debounce)
			del_timer_sync(&bdata->timer);

		bdata->disabled = true;
	}
}

/**
 * gpio_keys_enable_button() - enables given GPIO button
 * @bdata: button data for button to be disabled
 *
 * Enables given button pointed by @bdata.
 *
 * Make sure that @bdata->disable_lock is locked when entering
 * this function to avoid races with concurrent threads trying
 * to enable the same button at the same time.
 */
static void gpio_keys_enable_button(struct gpio_button_data *bdata)
{
	if (bdata->disabled) {
		enable_irq(gpio_to_irq(bdata->button->gpio));
		bdata->disabled = false;
	}
}

/**
 * gpio_keys_attr_show_helper() - fill in stringified bitmap of buttons
 * @ddata: pointer to drvdata
 * @buf: buffer where stringified bitmap is written
 * @type: button type (%EV_KEY, %EV_SW)
 * @only_disabled: does caller want only those buttons that are
 *                 currently disabled or all buttons that can be
 *                 disabled
 *
 * This function writes buttons that can be disabled to @buf. If
 * @only_disabled is true, then @buf contains only those buttons
 * that are currently disabled. Returns 0 on success or negative
 * errno on failure.
 */
static ssize_t gpio_keys_attr_show_helper(struct gpio_keys_drvdata *ddata,
					  char *buf, unsigned int type,
					  bool only_disabled)
{
	int n_events = get_n_events_by_type(type);
	unsigned long *bits;
	ssize_t ret;
	int i;

	bits = kcalloc(BITS_TO_LONGS(n_events), sizeof(*bits), GFP_KERNEL);
	if (!bits)
		return -ENOMEM;

	for (i = 0; i < ddata->n_buttons; i++) {
		struct gpio_button_data *bdata = &ddata->data[i];

		if (bdata->button->type != type)
			continue;

		if (only_disabled && !bdata->disabled)
			continue;

		__set_bit(bdata->button->code, bits);
	}

	ret = bitmap_scnlistprintf(buf, PAGE_SIZE - 2, bits, n_events);
	buf[ret++] = '\n';
	buf[ret] = '\0';

	kfree(bits);

	return ret;
}

/**
 * gpio_keys_attr_store_helper() - enable/disable buttons based on given bitmap
 * @ddata: pointer to drvdata
 * @buf: buffer from userspace that contains stringified bitmap
 * @type: button type (%EV_KEY, %EV_SW)
 *
 * This function parses stringified bitmap from @buf and disables/enables
 * GPIO buttons accordinly. Returns 0 on success and negative error
 * on failure.
 */
static ssize_t gpio_keys_attr_store_helper(struct gpio_keys_drvdata *ddata,
					   const char *buf, unsigned int type)
{
	int n_events = get_n_events_by_type(type);
	unsigned long *bits;
	ssize_t error;
	int i;

	bits = kcalloc(BITS_TO_LONGS(n_events), sizeof(*bits), GFP_KERNEL);
	if (!bits)
		return -ENOMEM;

	error = bitmap_parselist(buf, bits, n_events);
	if (error)
		goto out;

	/* First validate */
	for (i = 0; i < ddata->n_buttons; i++) {
		struct gpio_button_data *bdata = &ddata->data[i];

		if (bdata->button->type != type)
			continue;

		if (test_bit(bdata->button->code, bits) &&
		    !bdata->button->can_disable) {
			error = -EINVAL;
			goto out;
		}
	}

	mutex_lock(&ddata->disable_lock);

	for (i = 0; i < ddata->n_buttons; i++) {
		struct gpio_button_data *bdata = &ddata->data[i];

		if (bdata->button->type != type)
			continue;

		if (test_bit(bdata->button->code, bits))
			gpio_keys_disable_button(bdata);
		else
			gpio_keys_enable_button(bdata);
	}

	mutex_unlock(&ddata->disable_lock);

out:
	kfree(bits);
	return error;
}

#define ATTR_SHOW_FN(name, type, only_disabled)				\
static ssize_t gpio_keys_show_##name(struct device *dev,		\
				     struct device_attribute *attr,	\
				     char *buf)				\
{									\
	struct platform_device *pdev = to_platform_device(dev);		\
	struct gpio_keys_drvdata *ddata = platform_get_drvdata(pdev);	\
									\
	return gpio_keys_attr_show_helper(ddata, buf,			\
					  type, only_disabled);		\
}

ATTR_SHOW_FN(keys, EV_KEY, false);
ATTR_SHOW_FN(switches, EV_SW, false);
ATTR_SHOW_FN(disabled_keys, EV_KEY, true);
ATTR_SHOW_FN(disabled_switches, EV_SW, true);

/*
 * ATTRIBUTES:
 *
 * /sys/devices/platform/gpio-keys/keys [ro]
 * /sys/devices/platform/gpio-keys/switches [ro]
 */
static DEVICE_ATTR(keys, S_IRUGO, gpio_keys_show_keys, NULL);
static DEVICE_ATTR(switches, S_IRUGO, gpio_keys_show_switches, NULL);

#define ATTR_STORE_FN(name, type)					\
static ssize_t gpio_keys_store_##name(struct device *dev,		\
				      struct device_attribute *attr,	\
				      const char *buf,			\
				      size_t count)			\
{									\
	struct platform_device *pdev = to_platform_device(dev);		\
	struct gpio_keys_drvdata *ddata = platform_get_drvdata(pdev);	\
	ssize_t error;							\
									\
	error = gpio_keys_attr_store_helper(ddata, buf, type);		\
	if (error)							\
		return error;						\
									\
	return count;							\
}

ATTR_STORE_FN(disabled_keys, EV_KEY);
ATTR_STORE_FN(disabled_switches, EV_SW);

/*
 * ATTRIBUTES:
 *
 * /sys/devices/platform/gpio-keys/disabled_keys [rw]
 * /sys/devices/platform/gpio-keys/disables_switches [rw]
 */
static DEVICE_ATTR(disabled_keys, S_IWUSR | S_IRUGO,
		   gpio_keys_show_disabled_keys,
		   gpio_keys_store_disabled_keys);
static DEVICE_ATTR(disabled_switches, S_IWUSR | S_IRUGO,
		   gpio_keys_show_disabled_switches,
		   gpio_keys_store_disabled_switches);

static struct attribute *gpio_keys_attrs[] = {
	&dev_attr_keys.attr,
	&dev_attr_switches.attr,
	&dev_attr_disabled_keys.attr,
	&dev_attr_disabled_switches.attr,
	NULL,
};

static struct attribute_group gpio_keys_attr_group = {
	.attrs = gpio_keys_attrs,
};

static void gpio_keys_report_event(struct gpio_button_data *bdata)
{
	struct gpio_keys_button *button = bdata->button;
	struct input_dev *input = bdata->input;
	unsigned int type = button->type ?: EV_KEY;
	int state = (gpio_get_value_cansleep(button->gpio) ? 1 : 0) ^ button->active_low;

	if(!doCheck && (KEY_POWER == button->code)) {
		if (is_resume_from_deep_suspend() && (KEY_POWER == button->code) && state == 0) {
			input_event(input, type, button->code, 1);
			pr_info("[KEY] send power key code 1.\n");
			/*workaround for isr lost. Send down key code to input subsystem.*/
		}
		doCheck = true;
	}

	if ((KEY_POWER == button->code) && (0 == state)) {
		if ( PWR_MISTOUCH_gpio ) {
			Mistouch_powerkey_requested = 0;
			cancel_delayed_work_sync(&delay_work);
			if ( mistouch_gpio_normal != NULL )
				mistouch_gpio_normal();
			else
				pr_err("[KEY] mistouch_gpio_normal() is NULL\n");
			wake_unlock(&key_reset_clr_wake_lock);
		}
		Powerkey_pressed = 0;
		printk(KERN_INFO "[KEY] Power key released\n");
	} else if ((KEY_POWER == button->code) && (1 == state)) {
		if ( PWR_MISTOUCH_gpio ) {
			wake_lock_timeout(&key_reset_clr_wake_lock, PWRKEYCHKRST_WAKELOCK_TIMEOUT);
			queue_delayed_work(delay_wq, &delay_work, msecs_to_jiffies(3500));
			printk(KERN_DEBUG "[KEY] Enable power key Mistouch function\n");
		}
		Powerkey_pressed = 1;
		printk(KERN_INFO "[KEY] Power key press\n");
	}

	input_event(input, type, button->code, !!state);
	input_sync(input);
	printk(KERN_INFO "[KEY] GPIO key status, key code = %d, state = %d\n", button->code, state);
}

static void gpio_keys_work_func(struct work_struct *work)
{
	struct gpio_button_data *bdata =
		container_of(work, struct gpio_button_data, work);

	gpio_keys_report_event(bdata);
}

static void gpio_keys_timer(unsigned long _data)
{
	struct gpio_button_data *data = (struct gpio_button_data *)_data;

	schedule_work(&data->work);
}

static irqreturn_t gpio_keys_isr(int irq, void *dev_id)
{
	struct gpio_button_data *bdata = dev_id;
	struct gpio_keys_button *button = bdata->button;

	BUG_ON(irq != gpio_to_irq(button->gpio));

	if (bdata->timer_debounce)
		mod_timer(&bdata->timer,
			jiffies + msecs_to_jiffies(bdata->timer_debounce));
	else
		schedule_work(&bdata->work);

	return IRQ_HANDLED;
}

static int __devinit gpio_keys_setup_key(struct platform_device *pdev,
					 struct gpio_button_data *bdata,
					 struct gpio_keys_button *button)
{
	char *desc = button->desc ? button->desc : "gpio_keys";
	struct device *dev = &pdev->dev;
	unsigned long irqflags;
	int irq, error;

	setup_timer(&bdata->timer, gpio_keys_timer, (unsigned long)bdata);
	INIT_WORK(&bdata->work, gpio_keys_work_func);

	error = gpio_request(button->gpio, desc);
	if (error < 0) {
		dev_err(dev, "failed to request GPIO %d, error %d\n",
			button->gpio, error);
		goto fail2;
	}

	error = gpio_direction_input(button->gpio);
	if (error < 0) {
		dev_err(dev, "failed to configure"
			" direction for GPIO %d, error %d\n",
			button->gpio, error);
		goto fail3;
	}

	if (button->debounce_interval)
			bdata->timer_debounce = button->debounce_interval;

	irq = gpio_to_irq(button->gpio);
	if (irq < 0) {
		error = irq;
		dev_err(dev, "Unable to get irq number for GPIO %d, error %d\n",
			button->gpio, error);
		goto fail3;
	}

	if (button->code == KEY_VOLUMEUP ||
            button->code == KEY_VOLUMEDOWN ) {
		if (button->code == KEY_VOLUMEUP)
			vol_up_irq = irq;
		else
			vol_down_irq = irq;
	}
	irqflags = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING;
	/*
	 * If platform has specified that the button can be disabled,
	 * we don't want it to share the interrupt line.
	 */
	if (!button->can_disable)
		irqflags |= IRQF_SHARED;

	error = request_any_context_irq(irq, gpio_keys_isr, irqflags, desc, bdata);
	if (error < 0) {
		dev_err(dev, "Unable to claim irq %d; error %d\n",
			irq, error);
		goto fail3;
	}

	return 0;

fail3:
	gpio_free(button->gpio);
fail2:
	return error;
}

static int gpio_keys_open(struct input_dev *input)
{
	struct gpio_keys_drvdata *ddata = input_get_drvdata(input);

	return ddata->enable ? ddata->enable(input->dev.parent) : 0;
}

static void gpio_keys_close(struct input_dev *input)
{
	struct gpio_keys_drvdata *ddata = input_get_drvdata(input);

	if (ddata->disable)
		ddata->disable(input->dev.parent);
}

static int __devinit gpio_keys_probe(struct platform_device *pdev)
{
	struct gpio_keys_platform_data *pdata = pdev->dev.platform_data;
	struct gpio_keys_drvdata *ddata;
	struct device *dev = &pdev->dev;
	struct input_dev *input;
	int i, error, ret;
	int wakeup = 0;
	struct kobject *keyboard_kobj;
	doCheck = true;

        printk("[KEY] gpio_keys_probe\n");
	ddata = kzalloc(sizeof(struct gpio_keys_drvdata) +
			pdata->nbuttons * sizeof(struct gpio_button_data),
			GFP_KERNEL);
	input = input_allocate_device();
	if (!ddata || !input) {
		dev_err(dev, "failed to allocate state\n");
		error = -ENOMEM;
		goto fail1;
	}

	ddata->input = input;
	ddata->n_buttons = pdata->nbuttons;
	ddata->enable = pdata->enable;
	ddata->disable = pdata->disable;
	mutex_init(&ddata->disable_lock);

	platform_set_drvdata(pdev, ddata);
	input_set_drvdata(input, ddata);

	input->name = pdev->name;
	input->phys = "gpio-keys/input0";
	input->dev.parent = &pdev->dev;
	input->open = gpio_keys_open;
	input->close = gpio_keys_close;

	input->id.bustype = BUS_HOST;
	input->id.vendor = 0x0001;
	input->id.product = 0x0001;
	input->id.version = 0x0100;

#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_SWEEP2WAKE
	if (!strcmp(input->name, "gpio-keys")) {
	sweep2wake_setdev(input);
	printk(KERN_INFO "[sweep2wake]: set device %s\n", input->name);
	}
#endif


	PWR_MISTOUCH_gpio = pdata->PWR_MISTOUCH_gpio;
	mistouch_gpio_normal = pdata->mistouch_gpio_normal;
	mistouch_gpio_active = pdata->mistouch_gpio_active;
	if ( PWR_MISTOUCH_gpio ) {
		gpio_free(PWR_MISTOUCH_gpio);
		ret = gpio_request(PWR_MISTOUCH_gpio, "PWR_MISTOUCH");
		if (ret < 0) {
			pr_err("[KEY]Requesting GPIO %d failes\n", PWR_MISTOUCH_gpio);
		}
		if ( mistouch_gpio_normal != NULL )
			mistouch_gpio_normal();
		else
			pr_err("[KEY] mistouch_gpio_normal() is NULL\n");
        }
	delay_wq = create_singlethread_workqueue("gpio_key_work");
	INIT_DELAYED_WORK(&delay_work, Mistouch_powerkey_func);
	wake_lock_init(&key_reset_clr_wake_lock, WAKE_LOCK_SUSPEND, "gpio_input_pwr_clear");

	/* Enable auto repeat feature of Linux input subsystem */
	if (pdata->rep)
		__set_bit(EV_REP, input->evbit);

	for (i = 0; i < pdata->nbuttons; i++) {
		struct gpio_keys_button *button = &pdata->buttons[i];
		struct gpio_button_data *bdata = &ddata->data[i];
		unsigned int type = button->type ?: EV_KEY;

		bdata->input = input;
		bdata->button = button;

		error = gpio_keys_setup_key(pdev, bdata, button);
		if (error)
			goto fail2;

		if (button->wakeup)
			wakeup = 1;

		input_set_capability(input, type, button->code);
	}

	error = sysfs_create_group(&pdev->dev.kobj, &gpio_keys_attr_group);
	if (error) {
		dev_err(dev, "Unable to export keys/switches, error: %d\n",
			error);
		goto fail2;
	}

	keyboard_kobj = kobject_create_and_add("keyboard", NULL);
	if (keyboard_kobj == NULL) {
		printk(KERN_ERR "[KEY] KEY_ERR: %s: subsystem_register failed\n", __func__);
		return -ENOMEM;
	}
	if (sysfs_create_file(keyboard_kobj, &dev_attr_vol_wakeup.attr))
		pr_err("%s: sysfs_create_file error", __func__);
	wakeup_bitmask = 0;
	set_wakeup = 0;

	error = input_register_device(input);
	if (error) {
		dev_err(dev, "Unable to register input device, error: %d\n",
			error);
		goto fail3;
	}

	/* get current state of buttons */
	for (i = 0; i < pdata->nbuttons; i++) {
		if (KEY_POWER != pdata->buttons[i].code)
			gpio_keys_report_event(&ddata->data[i]);
	}
	input_sync(input);

	device_init_wakeup(&pdev->dev, wakeup);
	wake_lock_init(&power_key_wake_lock, WAKE_LOCK_SUSPEND, "power_key_wake_lock");
	pr_info("[KEY] gpio_keys_probe end.\n");

	return 0;

 fail3:
	sysfs_remove_group(&pdev->dev.kobj, &gpio_keys_attr_group);
 fail2:
	while (--i >= 0) {
		free_irq(gpio_to_irq(pdata->buttons[i].gpio), &ddata->data[i]);
		if (ddata->data[i].timer_debounce)
			del_timer_sync(&ddata->data[i].timer);
		cancel_work_sync(&ddata->data[i].work);
		gpio_free(pdata->buttons[i].gpio);
	}

	platform_set_drvdata(pdev, NULL);
 fail1:
	input_free_device(input);
	kfree(ddata);

	return error;
}

static int __devexit gpio_keys_remove(struct platform_device *pdev)
{
	struct gpio_keys_platform_data *pdata = pdev->dev.platform_data;
	struct gpio_keys_drvdata *ddata = platform_get_drvdata(pdev);
	struct input_dev *input = ddata->input;
	int i;

	sysfs_remove_group(&pdev->dev.kobj, &gpio_keys_attr_group);

	device_init_wakeup(&pdev->dev, 0);
	wake_lock_destroy(&key_reset_clr_wake_lock);
	wake_lock_destroy(&power_key_wake_lock);

	for (i = 0; i < pdata->nbuttons; i++) {
		int irq = gpio_to_irq(pdata->buttons[i].gpio);
		free_irq(irq, &ddata->data[i]);
		if (ddata->data[i].timer_debounce)
			del_timer_sync(&ddata->data[i].timer);
		cancel_work_sync(&ddata->data[i].work);
		gpio_free(pdata->buttons[i].gpio);
	}

	input_unregister_device(input);

	return 0;
}


#ifdef CONFIG_PM
static int gpio_keys_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct gpio_keys_platform_data *pdata = pdev->dev.platform_data;
	int i;

	if (device_may_wakeup(&pdev->dev)) {
		for (i = 0; i < pdata->nbuttons; i++) {
			struct gpio_keys_button *button = &pdata->buttons[i];
			if (button->wakeup) {
				int irq = gpio_to_irq(button->gpio);
				enable_irq_wake(irq);
			}
		}
	}
	doCheck = false;
	pr_info("[KEY] doCheck = false\n");
	return 0;
}

static int gpio_keys_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct gpio_keys_drvdata *ddata = platform_get_drvdata(pdev);
	struct gpio_keys_platform_data *pdata = pdev->dev.platform_data;
	int wakeup_key = KEY_RESERVED;
	int i;

	if (pdata->wakeup_key)
		wakeup_key = pdata->wakeup_key();

	for (i = 0; i < pdata->nbuttons; i++) {

		struct gpio_keys_button *button = &pdata->buttons[i];
		if (button->wakeup && device_may_wakeup(&pdev->dev)) {
			int irq = gpio_to_irq(button->gpio);
			disable_irq_wake(irq);

			if (wakeup_key == button->code) {
				wake_lock_timeout(&power_key_wake_lock, 5 * HZ);
			}
		}
		if (KEY_POWER != button->code)
			gpio_keys_report_event(&ddata->data[i]);
	}
	input_sync(ddata->input);

	return 0;
}

static const struct dev_pm_ops gpio_keys_pm_ops = {
	.suspend	= gpio_keys_suspend,
	.resume		= gpio_keys_resume,
};
#endif

static struct platform_driver gpio_keys_device_driver = {
	.probe		= gpio_keys_probe,
	.remove		= __devexit_p(gpio_keys_remove),
	.driver		= {
		.name	= "gpio-keys",
		.owner	= THIS_MODULE,
#ifdef CONFIG_PM
		.pm	= &gpio_keys_pm_ops,
#endif
	}
};

static int __init gpio_keys_init(void)
{
	printk("[KEY] gpio_keys_init\n");
	return platform_driver_register(&gpio_keys_device_driver);
}

static void __exit gpio_keys_exit(void)
{
	printk("[KEY] gpio_keys_exit\n");
	platform_driver_unregister(&gpio_keys_device_driver);
}

module_init(gpio_keys_init);
module_exit(gpio_keys_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Phil Blundell <pb@handhelds.org>");
MODULE_DESCRIPTION("Keyboard driver for CPU GPIOs");
MODULE_ALIAS("platform:gpio-keys");
