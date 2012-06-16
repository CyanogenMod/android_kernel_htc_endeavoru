/*
 *
 * Copyright (C) 2011 HTC Corporation.
 * Author: Quaker NTj <Quaker_Chung@htc.com>
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

#include <linux/workqueue.h>
#include <linux/wakelock.h>
#include <linux/module.h>
#include <htc/usb_notifier.h>
#include <linux/slab.h>
#define TAG "[USBN]"
#define DEBUG 1
#define DRIVER_VERSION			"0.3.0"

static LIST_HEAD(callback_list);

// prevent add / remove at calling
static DEFINE_MUTEX(op_mutex);
static enum usb_status gstatus;
struct wake_lock work_wake_lock;

static void usb_notifier_call_work(struct work_struct *work) {
	int succ, count, ret;

	struct list_head *pos, *head;
	struct usb_callback_list *node;
	enum usb_status status;
	
#if DEBUG > 0
	printk(KERN_INFO TAG "%s:\n", __func__);
#endif
	head = &callback_list;
 
	mutex_lock(&op_mutex);
	status = gstatus;

	succ = count = 0;
	if(list_empty(head)) {
		goto unlock;
	}

	list_for_each(pos, head) {
		node = list_entry(pos,struct usb_callback_list, list);

		if(node->callback) {
			ret = node->callback(status, node->data);
			if( ret == 0 )
				succ++;
		}
		count++;
	}

#if DEBUG > 0
	printk(KERN_INFO TAG "%s: success %d, fail %d\n", __func__, succ, count-succ);
#endif

unlock:
	wake_unlock(&work_wake_lock);
	mutex_unlock(&op_mutex);
	return;
}
DECLARE_WORK(usb_notifier_work, usb_notifier_call_work);

int usb_notifier_call(enum usb_status status) {
	//No need to lock.  We will reschedule work again.
	gstatus = status;

	//Wait work callback complete before reschedule it.
	flush_work(&usb_notifier_work); //may sleep

	wake_lock(&work_wake_lock);
	schedule_work(&usb_notifier_work);

#if DEBUG > 0
	printk(KERN_INFO TAG "%s: notifify work are scheduled success\n", __func__);
#endif
	return 0;
}

struct usb_callback_list* usb_notifier_register(
			int (*callback)(enum usb_status status, void * data), void * data) {
	struct usb_callback_list *node;

#if DEBUG > 0
	printk(KERN_INFO TAG "%s:\n", __func__);
#endif

	node = kzalloc(sizeof(*node), GFP_KERNEL);
	if (!node) {
		printk(KERN_INFO TAG "failed to allocate new usb_notifier data\n");
		goto exit;
	}
 
	node->callback = callback;
	node->data = data;

	mutex_lock(&op_mutex);
	list_add(&node->list, &callback_list);
	mutex_unlock(&op_mutex);

exit:
	return node;
}

void usb_notifier_unregister(struct usb_callback_list* node) {
			struct list_head *head, *pos;

#if DEBUG > 0
	printk(KERN_INFO TAG "%s:\n", __func__);
#endif

	head = &callback_list;
	mutex_lock(&op_mutex);

	if(list_empty(head)) {
		printk(KERN_INFO TAG "list is null\n");
		goto unlock;
	}
	list_for_each(pos, head) {
		if( pos == &node->list ) {
			list_del(&node->list);
			kfree(node);
			break;
		}
	}

unlock:
	mutex_unlock(&op_mutex);
	return;
}

static int __init usb_notifier_init(void)
{
	wake_lock_init(&work_wake_lock, WAKE_LOCK_SUSPEND, "usbnotifier");
	return 0;
}
module_init(usb_notifier_init);

static void __exit usb_notifier_exit(void)
{
	wake_lock_destroy(&work_wake_lock);
}
module_exit(usb_notifier_exit);

MODULE_AUTHOR("Quaker NTj <Quaker_Chung@htc.com>");
MODULE_DESCRIPTION("Let USB isr can call battery to change charger gpio.");
MODULE_LICENSE("GPL");

