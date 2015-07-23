/*
 *  HID driver for OUYA Game Controller(s)
 *
 *  Copyright (c) 2013 OUYA
 */

#include <linux/device.h>
#include <linux/input.h>
#include <linux/hid.h>
#include <linux/module.h>

#include "hid-ids.h"

#define OUYA_TOUCHPAD_FIXUP     (1 << 0)

struct ouya_sc {
	unsigned long quirks;
};

/* Fixed report descriptor */
static __u8 ouya_rdesc_fixed[] = {

	0x05, 0x01,         /*  Usage Page (Desktop),           */
	0x09, 0x05,         /*  Usage (Game Pad),               */

	0xA1, 0x01,         /*  Collection (Application),       */
	0x85, 0x07,         /*      Report ID (7),              */

	0xA1, 0x00,         /*      Collection (Physical),      */
	0x09, 0x30,         /*          Usage (X),              */
	0x09, 0x31,         /*          Usage (Y),              */
	0x15, 0x00,         /*          Logical Minimum (0),    */
	0x26, 0xFF, 0x00,   /*          Logical Maximum (255),  */
	0x35, 0x00,         /*          Physical Minimum (0),   */
	0x46, 0xFF, 0x00,   /*          Physical Maximum (255), */
	0x95, 0x02,         /*          Report Count (2),       */
	0x75, 0x08,         /*          Report Size (8),        */
	0x81, 0x02,         /*          Input (Variable),       */
	0xC0,               /*      End Collection,             */

	0xA1, 0x00,         /*      Collection (Physical),      */
	0x09, 0x33,         /*          Usage (Rx),             */
	0x09, 0x34,         /*          Usage (Ry),             */
	0x15, 0x00,         /*          Logical Minimum (0),    */
	0x26, 0xFF, 0x00,   /*          Logical Maximum (255),  */
	0x35, 0x00,         /*          Physical Minimum (0),   */
	0x46, 0xFF, 0x00,   /*          Physical Maximum (255), */
	0x95, 0x02,         /*          Report Count (2),       */
	0x75, 0x08,         /*          Report Size (8),        */
	0x81, 0x02,         /*          Input (Variable),       */
	0xC0,               /*      End Collection,             */

	0xA1, 0x00,         /*      Collection (Physical),      */
	0x09, 0x32,         /*          Usage (Z),              */
	0x15, 0x00,         /*          Logical Minimum (0),    */
	0x26, 0xFF, 0x00,   /*          Logical Maximum (255),  */
	0x35, 0x00,         /*          Physical Minimum (0),   */
	0x46, 0xFF, 0x00,   /*          Physical Maximum (255), */
	0x95, 0x01,         /*          Report Count (1),       */
	0x75, 0x08,         /*          Report Size (8),        */
	0x81, 0x02,         /*          Input (Variable),       */
	0xC0,               /*      End Collection,             */

	0xA1, 0x00,         /*      Collection (Physical),      */
	0x09, 0x35,         /*          Usage (Rz),             */
	0x15, 0x00,         /*          Logical Minimum (0),    */
	0x26, 0xFF, 0x00,   /*          Logical Maximum (255),  */
	0x35, 0x00,         /*          Physical Minimum (0),   */
	0x46, 0xFF, 0x00,   /*          Physical Maximum (255), */
	0x95, 0x01,         /*          Report Count (1),       */
	0x75, 0x08,         /*          Report Size (8),        */
	0x81, 0x02,         /*          Input (Variable),       */
	0xC0,               /*      End Collection,             */

	0x05, 0x09,         /*      Usage Page (Button),        */
	0x19, 0x01,         /*      Usage Minimum (01h),        */
	0x29, 0x10,         /*      Usage Maximum (10h),        */
	0x95, 0x10,         /*      Report Count (16),          */
	0x75, 0x01,         /*      Report Size (1),            */
	0x81, 0x02,         /*      Input (Variable),           */

	/*   ORIGINAL REPORT DESCRIPTOR FOR TOUCHPAD INPUT  */
    /* 06 00 ff a1 02 09 02 15 00 26 ff 00 35 00 46 ff 00 95 03 75 08 81 02 c0 */

	0x06, 0x00, 0xFF,   /*      Usage Page (Custom),        */
	0x09, 0x02,         /*      Usage (Mouse),              */
	0x09, 0x01,         /*      Usage (Pointer),            */
	0xA1, 0x00,         /*      Collection (Physical),      */
	0x05, 0x09,         /*          Usage Page (Button),    */
	0x19, 0x01,         /*          Usage Minimum (01h),    */
	0x29, 0x03,         /*          Usage Maximum (03h),    */
	0x15, 0x00,         /*          Logical Minimum (0),    */
	0x25, 0x01,         /*          Logical Maximum (1),    */
	0x95, 0x03,         /*          Report Count (3),       */
	0x75, 0x01,         /*          Report Size (1),        */
	0x81, 0x02,         /*          Input (Variable),       */
	0x95, 0x01,         /*          Report Count (1),       */
	0x75, 0x05,         /*          Report Size (5),        */
	0x81, 0x01,         /*          Input (Constant),       */
	0x05, 0x01,         /*          Usage Page (Desktop),   */
	0x09, 0x30,         /*          Usage (X),              */
	0x09, 0x31,         /*          Usage (Y),              */
	0x15, 0x81,         /*          Logical Minimum (-127), */
	0x25, 0x7f,         /*          Logical Maximum (127),  */
	0x95, 0x02,         /*          Report Count (2),       */
	0x75, 0x08,         /*          Report Size (8),        */
	0x81, 0x06,         /*          Input (Relative),       */
	0xC0,               /*      End Collection,             */

	0x06, 0x00, 0xFF,   /*      Usage Page (Custom),        */
	0xA1, 0x02,         /*      Collection (Logical),       */
	0x75, 0x08,         /*          Report Size (8),        */
	0x95, 0x07,         /*          Report Count (7),       */
	0x46, 0xFF, 0x00,   /*          Physical Maximum (255), */
	0x26, 0xFF, 0x00,   /*          Logical Maximum (255),  */
	0x09, 0x01,         /*          Usage (Pointer),        */
	0x91, 0x02,         /*          Output (Variable),      */
	0xC0,               /*      End Collection,             */

	0xC0,               /*  End Collection                  */


	0x06, 0x00, 0xFF,   /*  Usage Page (Custom),            */
	0x05, 0x0C,         /*  Usage Page (Consumer),          */
	0x09, 0x01,         /*  Usage (Consumer Control),       */

	0xA1, 0x01,         /*  Collection (Application),       */
	0x85, 0x03,         /*      Report ID (3),              */
	0x05, 0x01,         /*      Usage Page (Desktop),       */
	0x09, 0x06,         /*      Usage (Keyboard),           */
	0xA1, 0x02,         /*      Collection (Logical),       */
	0x05, 0x06,         /*          Usage Page (Generic),   */
	0x09, 0x20,         /*          Usage (Battery Strgth), */
	0x15, 0x00,         /*          Logical Minimum (0),    */
	0x26, 0xFF, 0x00,   /*          Logical Maximum (255),  */
	0x75, 0x08,         /*          Report Size (8),        */
	0x95, 0x01,         /*          Report Count (1),       */
	0x81, 0x02,         /*          Input (Variable),       */
	0x06, 0xBC, 0xFF,   /*          Usage Page (Custom),    */

	0x0A, 0xAD, 0xBD,   /*  UNKNOWN */

	0x75, 0x08,         /*          Report Size (8),        */
	0x95, 0x06,         /*          Report Count (6),       */
	0x81, 0x02,         /*          Input (Variable),       */
	0xC0,               /*      End Collection,             */

	0xC0,               /*  End Collection                  */
	
	0x00 
};

static __u8 *ouya_report_fixup(struct hid_device *hdev, __u8 *rdesc,
		unsigned int *rsize)
{
	struct ouya_sc *sc = hid_get_drvdata(hdev);

	if (sc->quirks & OUYA_TOUCHPAD_FIXUP) {
		rdesc = ouya_rdesc_fixed;
		*rsize = sizeof(ouya_rdesc_fixed);
	}
    return rdesc;
}

static int ouya_input_mapping(struct hid_device *hdev, struct hid_input *hi,
			       struct hid_field *field, struct hid_usage *usage,
			       unsigned long **bit, int *max)
{
	struct ouya_sc *sc = hid_get_drvdata(hdev);

	if (!(sc->quirks & OUYA_TOUCHPAD_FIXUP)) {
		return 0;
	}

	if ((usage->hid & 0x90000) == 0x90000 &&
		(field->physical & 0xff000000) == 0xff000000 &&
		usage->collection_index == 5 &&
		field->report_count == 3) {

		hid_map_usage(hi, usage, bit, max, EV_KEY, BTN_MOUSE + (usage->hid - 0x90001));

		return 1;
	}

	return 0;
}

static int ouya_probe(struct hid_device *hdev, const struct hid_device_id *id)
{
	int ret;
	struct ouya_sc *sc;

	sc = kzalloc(sizeof(*sc), GFP_KERNEL);
	if (sc == NULL) {
		hid_err(hdev, "can't alloc ouya descriptor\n");
		return -ENOMEM;
	}

	if(((hdev->version & 0xff00) == 0x0100 && (hdev->version & 0xff) >= 0x04) ||
		((hdev->version & 0xff00) == 0xe100 && (hdev->version & 0xff) >= 0x3a)) {
		hid_info(hdev, "ouya controller - new version\n");
		sc->quirks = OUYA_TOUCHPAD_FIXUP;	
	} else {
		sc->quirks = 0;	
	}
	hid_set_drvdata(hdev, sc);

	ret = hid_parse(hdev);
	if (ret) {
		hid_err(hdev, "parse failed\n");
		goto err_free;
	}

	ret = hid_hw_start(hdev, HID_CONNECT_DEFAULT |
			HID_CONNECT_HIDDEV_FORCE);
	if (ret) {
		hid_err(hdev, "hw start failed\n");
		goto err_free;
	}

	return 0;

err_free:
	kfree(sc);
	return ret;
}

static void ouya_remove(struct hid_device *hdev)
{
	hid_hw_stop(hdev);
	kfree(hid_get_drvdata(hdev));
}

static const struct hid_device_id ouya_devices[] = {
	{ HID_BLUETOOTH_DEVICE(USB_VENDOR_ID_OUYA, USB_DEVICE_ID_OUYA_CONTROLLER) },
	{ }
};
MODULE_DEVICE_TABLE(hid, ouya_devices);

static struct hid_driver ouya_driver = {
	.name = "ouya",
	.id_table = ouya_devices,
	.probe = ouya_probe,
	.remove = ouya_remove,
	.input_mapping = ouya_input_mapping,	
	.report_fixup = ouya_report_fixup
};

static int __init ouya_init(void)
{
	return hid_register_driver(&ouya_driver);
}

static void __exit ouya_exit(void)
{
	hid_unregister_driver(&ouya_driver);
}

module_init(ouya_init);
module_exit(ouya_exit);
