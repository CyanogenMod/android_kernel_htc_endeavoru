/* drivers/input/keyreset.c
 *
 * Copyright (C) 2008 Google, Inc.
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

#include <linux/reboot.h>
#include <linux/input.h>
#include <linux/keyreset.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/syscalls.h>
#include <linux/hrtimer.h>
#include <linux/nct1008.h>
#if defined(CONFIG_MACH_ENDEAVORU) || defined(CONFIG_MACH_ENDEAVORTD)
	#include <linux/leds-lp5521_htc.h>
#endif
#include <mach/board_htc.h>
#include <mach/restart.h>
#include <htc/log.h>

extern unsigned engineer_id;
extern struct htc_reboot_params *reboot_params;
void set_dirty_state(int dirty);

#define KEYRESET_DELAY 3*HZ
#define PWROFFLED_DELAY 3*HZ

struct keyreset_state {
	struct input_handler input_handler;
	unsigned long keybit[BITS_TO_LONGS(KEY_CNT)];
	unsigned long upbit[BITS_TO_LONGS(KEY_CNT)];
	unsigned long key[BITS_TO_LONGS(KEY_CNT)];
	spinlock_t lock;
	int key_down_target;
	int key_down;
	int key_up;
	int restart_disabled;
	int (*reset_fn)(void);
};

static int restart_requested;
static unsigned long restart_timeout;

static volatile int power_off_led_requested = 0;
static struct hrtimer led_timer;

static volatile int power_reset_requested = 0;
static struct hrtimer reset_timer;

static int read_temp_requested = 0;
static struct hrtimer temp_timer;

static void deferred_restart(struct work_struct *dummy)
{
	pr_info("keyreset::%s in\n", __func__);
	restart_requested = 2;
	sys_sync();
	restart_requested = 3;
	kernel_restart(NULL);
}
static DECLARE_WORK(restart_work, deferred_restart);

static enum hrtimer_restart led_timer_func(struct hrtimer *timer)
{
	pr_info("[PWR] %s in (%x)\n", __func__, power_off_led_requested);
	if (power_off_led_requested == 1) {
		power_off_led_requested = 2;
		pr_info("[PWR] power off led turn on\n");
#if defined(CONFIG_MACH_ENDEAVORU) || defined(CONFIG_MACH_ENDEAVORTD)
		lp5521_led_current_set_for_key(1);
#endif
	}
	return HRTIMER_NORESTART;
}

static enum hrtimer_restart power_reset_func(struct hrtimer *timer)
{
	pr_info("[PWR] %s in (%x)\n", __func__, power_reset_requested);
	if (power_reset_requested == 1) {
		pr_info("[PWR] power reset flag turn on\n");
		power_reset_requested = 2;
		pr_info("[PWR] clear reboot reason to RESTART_REASON_REBOOT\n");
		reboot_params->reboot_reason = RESTART_REASON_REBOOT;
		pr_info("[PWR] clear dirty\n");
		set_dirty_state(0);
	}
	return HRTIMER_NORESTART;
}

static enum hrtimer_restart temp_timer_func(struct hrtimer *timer)
{
	pr_info("[PWR] %s in (%x)\n", __func__, read_temp_requested);
	if (read_temp_requested == 1) {
		read_temp_requested = 2;
		pr_info("[PWR] read temp turn on\n");
		nct1008_read_temp_for_key(1);
	}
	return HRTIMER_NORESTART;
}

static void keyreset_event(struct input_handle *handle, unsigned int type,
			   unsigned int code, int value)
{
	unsigned long flags;
	struct keyreset_state *state = handle->private;
	int pcbid = htc_get_pcbid_info();

	if (type != EV_KEY)
		return;

	if (code >= KEY_MAX)
		return;

	if (!test_bit(code, state->keybit))
		return;

	spin_lock_irqsave(&state->lock, flags);
	if (!test_bit(code, state->key) == !value)
		goto done;
	__change_bit(code, state->key);

	if (code == KEY_POWER) {
		if (value) {
			sp_pr_info("[PWR] start count for power off led\n");
			hrtimer_start(&led_timer, ktime_set(5,0), HRTIMER_MODE_REL);
			power_off_led_requested = 1;
#if defined(CONFIG_MACH_ENDEAVORU) || defined(CONFIG_MACH_ENDEAVORTD)
			if ( ( pcbid < PROJECT_PHASE_A ) || ( (pcbid == PROJECT_PHASE_A) && (engineer_id == 0x01 ))) {
				sp_pr_info("[PWR] start count for power reset\n");
				hrtimer_start(&reset_timer, ktime_set(5,500000000), HRTIMER_MODE_REL);
				power_reset_requested = 1;
			}
#endif
			sp_pr_info("[PWR] start count for read temp\n");
			hrtimer_start(&temp_timer, ktime_set(5,0), HRTIMER_MODE_REL);
			read_temp_requested = 1;
		}
		else {
			if (power_off_led_requested) {
				if (hrtimer_cancel(&led_timer)) {
					sp_pr_info("[PWR] cancel power off led timer (%x)\n", power_off_led_requested);
				} else
					sp_pr_info("[PWR] cancel power off led timer, timer has done (%x)\n", power_off_led_requested);

				if (power_off_led_requested == 2) {
					sp_pr_info("[PWR] power off led turn off\n");
#if defined(CONFIG_MACH_ENDEAVORU) || defined(CONFIG_MACH_ENDEAVORTD)
					lp5521_led_current_set_for_key(0);
#endif
				}
				power_off_led_requested = 0;
			}
#if defined(CONFIG_MACH_ENDEAVORU) || defined(CONFIG_MACH_ENDEAVORTD)
			if ( ( pcbid < PROJECT_PHASE_A ) || ( (pcbid == PROJECT_PHASE_A) && (engineer_id == 0x01 ))) {
				if ( power_reset_requested ) {
					if (hrtimer_cancel(&reset_timer)) {
						sp_pr_info("[PWR] cancel power reset timer (%x).\n", power_reset_requested);
					} else
						sp_pr_info("[PWR] cancel power reset timer, timer has done (%x).\n", power_reset_requested);
					if ( power_reset_requested == 2 ) {
						sp_pr_info("[PWR] reset device\n");
						schedule_work(&restart_work);
					}
					power_reset_requested = 0;
				}
			}
#endif
			if (read_temp_requested) {
				if (hrtimer_cancel(&temp_timer)) {
					sp_pr_info("[PWR] cancel read temp timer (%x)\n", read_temp_requested);
				} else
					sp_pr_info("[PWR] cancel read temp timer, timer has done (%x)\n", read_temp_requested);

				if (read_temp_requested == 2) {
					sp_pr_info("[PWR] read temp turn off\n");
					nct1008_read_temp_for_key(0);
				}
				read_temp_requested = 0;
			}
		}
	}
done:
	spin_unlock_irqrestore(&state->lock, flags);
}

static int keyreset_connect(struct input_handler *handler,
					  struct input_dev *dev,
					  const struct input_device_id *id)
{
	int i;
	int ret;
	struct input_handle *handle;
	struct keyreset_state *state =
		container_of(handler, struct keyreset_state, input_handler);

	for (i = 0; i < KEY_MAX; i++) {
		if (test_bit(i, state->keybit) && test_bit(i, dev->keybit))
			break;
	}
	if (i == KEY_MAX)
		return -ENODEV;

	hrtimer_init(&led_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	led_timer.function = led_timer_func;
#if defined(CONFIG_MACH_ENDEAVORU) || defined(CONFIG_MACH_ENDEAVORTD)
	hrtimer_init(&reset_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	reset_timer.function = power_reset_func;
#endif
	hrtimer_init(&temp_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	temp_timer.function = temp_timer_func;

	handle = kzalloc(sizeof(*handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = "keyreset";
	handle->private = state;

	ret = input_register_handle(handle);
	if (ret)
		goto err_input_register_handle;

	ret = input_open_device(handle);
	if (ret)
		goto err_input_open_device;

	pr_info("using input dev %s for key reset\n", dev->name);

	return 0;

err_input_open_device:
	input_unregister_handle(handle);
err_input_register_handle:
	kfree(handle);
	return ret;
}

static void keyreset_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static const struct input_device_id keyreset_ids[] = {
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT,
		.evbit = { BIT_MASK(EV_KEY) },
	},
	{ },
};
MODULE_DEVICE_TABLE(input, keyreset_ids);

static int keyreset_probe(struct platform_device *pdev)
{
	int ret;
	int key, *keyp;
	struct keyreset_state *state;
	struct keyreset_platform_data *pdata = pdev->dev.platform_data;

	if (!pdata)
		return -EINVAL;

	state = kzalloc(sizeof(*state), GFP_KERNEL);
	if (!state)
		return -ENOMEM;

	spin_lock_init(&state->lock);
	keyp = pdata->keys_down;
	while ((key = *keyp++)) {
		if (key >= KEY_MAX)
			continue;
		state->key_down_target++;
		__set_bit(key, state->keybit);
	}
	if (pdata->keys_up) {
		keyp = pdata->keys_up;
		while ((key = *keyp++)) {
			if (key >= KEY_MAX)
				continue;
			__set_bit(key, state->keybit);
			__set_bit(key, state->upbit);
		}
	}

	if (pdata->reset_fn)
		state->reset_fn = pdata->reset_fn;

	state->input_handler.event = keyreset_event;
	state->input_handler.connect = keyreset_connect;
	state->input_handler.disconnect = keyreset_disconnect;
	state->input_handler.name = KEYRESET_NAME;
	state->input_handler.id_table = keyreset_ids;
	ret = input_register_handler(&state->input_handler);
	if (ret) {
		kfree(state);
		return ret;
	}
	platform_set_drvdata(pdev, state);
	return 0;
}

int keyreset_remove(struct platform_device *pdev)
{
	struct keyreset_state *state = platform_get_drvdata(pdev);
	input_unregister_handler(&state->input_handler);
	kfree(state);
	return 0;
}


struct platform_driver keyreset_driver = {
	.driver.name = KEYRESET_NAME,
	.probe = keyreset_probe,
	.remove = keyreset_remove,
};

static int __init keyreset_init(void)
{
	return platform_driver_register(&keyreset_driver);
}

static void __exit keyreset_exit(void)
{
	return platform_driver_unregister(&keyreset_driver);
}

module_init(keyreset_init);
module_exit(keyreset_exit);
