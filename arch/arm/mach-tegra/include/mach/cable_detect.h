#ifndef _CABLE_DETECT_H_
#define _CABLE_DETECT_H_

#include <linux/list.h>

#define DOCK_STATE_UNDEFINED		-1
#define DOCK_STATE_UNDOCKED		0
#define DOCK_STATE_DESK			(1 << 0)
#define DOCK_STATE_CAR			(1 << 1)
#define DOCK_STATE_USB_HEADSET		(1 << 2)
#define DOCK_STATE_MHL			(1 << 3)
#define DOCK_STATE_USB_HOST		(1 << 4)
#define DOCK_STATE_DMB			(1 << 5)

#define DOCK_DET_DELAY		HZ/4

#define ADC_RETRY 3
#define ADC_DELAY HZ/8

#define TPS80032ADC_12BIT(x) ((x * 1250) >> 12) /* vref=1.2v, 12-bits resolution */

#define CABLE_ERR(fmt, args...) \
	printk(KERN_ERR "[CABLE:ERR] " fmt, ## args)
#define CABLE_WARNING(fmt, args...) \
	printk(KERN_WARNING "[CABLE] " fmt, ## args)
#define CABLE_INFO(fmt, args...) \
	printk(KERN_INFO "[CABLE] " fmt, ## args)
#define CABLE_DEBUG(fmt, args...) \
	printk(KERN_DEBUG "[CABLE] " fmt, ## args)

enum accessory_type {
	CABLE_TYPE_UNKOWN = 0,
	CABLE_TYPE_ID_PIN,
	CABLE_TYPE_PMIC_ADC,
};

enum dpdn_path_type {
	PATH_USB = 0,
	PATH_MHL,
	PATH_USB_AUD,
	PATH_UART,
};

#if 0
static struct switch_dev dock_switch = {
	.name = "dock",
};
#endif

struct usb_id_mpp_config_data {
	u32 usbid_mpp;
	u32 usbid_amux;
};

struct cable_detect_platform_data {
	int vbus_mpp_gpio;
	int vbus_mpp_irq;
	void (*vbus_mpp_config)(void);
	void (*usb_uart_switch)(int);
	void (*usb_dpdn_switch)(int);

	/* for accessory detection */
	u8 accessory_type;
	u8 mfg_usb_carkit_enable;
	int usb_id_pin_gpio;
	__u8 detect_type;
	u8 mhl_reset_gpio;
	bool mhl_version_ctrl_flag;
	struct usb_id_mpp_config_data mpp_data;
	void (*config_usb_id_gpios)(bool enable);
	void (*config_desk_aud_gpios)(bool output, bool out_val);
	void (*mhl_1v2_power)(bool enable);
	int (*is_wireless_charger)(void);
	int32_t (*get_adc_cb)(void);
	void (*cable_gpio_init)(void);

	int ac_9v_gpio;
	void (*configure_ac_9v_gpio) (int);
	u8 mhl_internal_3v3;

#ifdef CONFIG_CABLE_DETECT_GPIO_DOCK
	bool dock_detect;
	int dock_pin_gpio;
#endif
};

/* START: add USB connected notify function */
void tegra_usb_set_vbus_state(int online);
enum usb_connect_type {
	CONNECT_TYPE_NONE,
	CONNECT_TYPE_UNKNOWN,
	CONNECT_TYPE_AUDIO,
	CONNECT_TYPE_CARKIT,
	CONNECT_TYPE_MHL,
	CONNECT_TYPE_UNDEFINED,
	CONNECT_TYPE_USB, /* 0A5 */
	CONNECT_TYPE_0A9_AC,
	CONNECT_TYPE_AC, /* 1A */
	CONNECT_TYPE_1A1_AC,
	CONNECT_TYPE_1A2_AC,
	CONNECT_TYPE_1A3_AC,
	CONNECT_TYPE_1A4_AC,
	CONNECT_TYPE_1A5_AC,
	CONNECT_TYPE_1A6_AC,
	CONNECT_TYPE_1A7_AC,
	CONNECT_TYPE_1A8_AC,
	CONNECT_TYPE_1A9_AC,
	CONNECT_TYPE_2A_AC,
};

struct t_usb_status_notifier {
	struct list_head notifier_link;
	const char *name;
	void (*func)(int cable_type);
};
static LIST_HEAD(g_lh_usb_notifier_list);

/***********************************
Direction: cable detect drvier -> battery driver or other
***********************************/
struct t_cable_status_notifier {
	struct list_head cable_notifier_link;
	const char *name;
	void (*func)(int cable_type);
};
int cable_detect_register_notifier(struct t_cable_status_notifier *);
static LIST_HEAD(g_lh_cable_detect_notifier_list);
/* END: add USB connected notify function */ 

#ifdef CONFIG_HTC_USB_NOTIFIER
int usb_register_notifier(struct t_usb_status_notifier *);
#else
#define usb_register_notifier(z) 0
#endif

/* -----------------------------------------------------------------------------
»       »       »       External routine declaration
-----------------------------------------------------------------------------*/
#ifdef CONFIG_TEGRA_HDMI_MHL
struct t_mhl_status_notifier {
	struct list_head mhl_notifier_link;
	const char *name;
	void (*func)(bool isMHL, int charging_type);
};
int mhl_detect_register_notifier(struct t_mhl_status_notifier *);
static LIST_HEAD(g_lh_mhl_detect_notifier_list);
extern void sii9234_mhl_device_wakeup(void);
#endif
extern int cable_get_connect_type(void);
extern void set_mfg_usb_carkit_enable(int enable);
extern int cable_get_accessory_type(void);
extern int cable_get_usb_id_level(void);
extern void cable_set_uart_switch(int);
/*extern irqreturn_t cable_detection_vbus_irq_handler(void);*/ /* VBUS IRQ */
extern bool cable_detection_ac_only(void);
extern bool cable_detection_det(void);
extern void cable_detection_queue_vbus_work(int time);
#endif /* _CABLE_DETECTION_H */
