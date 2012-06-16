/*
 *
 * Copyright (C) 2011 HTC Corporation.
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

#ifndef HTC_USB_NOTIFIER_H
#define HTC_USB_NOTIFIER_H
#include <linux/list.h>

enum usb_status {
	USBST_UNPLUG,
	USBST_NO_ID,
	USBST_ID,
	USBST_5V_0A2,
	USBST_5V_0A25,
	USBST_5V_0A3,
	USBST_5V_0A35,
	USBST_5V_0A4,
	USBST_5V_0A45,
	USBST_5V_0A5,
	USBST_5V_0A6,
	USBST_5V_0A7,
	USBST_5V_0A8,
	USBST_5V_0A9,
	USBST_5V_1A,
	USBST_5V_1A1,
	USBST_5V_1A2,
	USBST_5V_1A3,
	USBST_5V_1A4,
	USBST_5V_1A5,
	USBST_5V_1A6,
	USBST_5V_1A7,
	USBST_5V_1A8,
	USBST_5V_1A9,
	USBST_5V_2A,
	USBST_5V_3A,
	USBST_5V_4A,
	USBST_9V_1A6,
	USBST_9V,
	USBST_LOW,
	USBST_DISABLE,
	USBST_DONT_CARE,
	USBST_TOGGLE,
	USBST_END,
};

struct usb_callback_list {
	int (*callback)(enum usb_status status, void *data);
	void *data;
    struct list_head list;
};

struct usb_callback_list* usb_notifier_register(
			int (*callback)(enum usb_status status, void * data), 
			void * data);

void usb_notifier_unregister(struct usb_callback_list* callback);

int usb_notifier_call(enum usb_status status);

#endif // HTC_USB_NOTIFIER_H
