#ifndef __HTC_GPIO_H
#define __HTC_GPIO_H

#define GPIO_SETUP_INPUT           (0x4)
#define GPIO_SETUP_INPUT_PULL_UP   (GPIO_SETUP_INPUT  | 0x1)
#define GPIO_SETUP_INPUT_PULL_DOWN (GPIO_SETUP_INPUT  | 0x2)
#define GPIO_SETUP_INPUT_NO_PULL   (GPIO_SETUP_INPUT  | 0x3)
#define GPIO_SETUP_OUTPUT          (0x8)
#define GPIO_SETUP_OUTPUT_LOW      (GPIO_SETUP_OUTPUT | 0x0)
#define GPIO_SETUP_OUTPUT_HIGH     (GPIO_SETUP_OUTPUT | 0x1)

#define INVALID_BALL (-1)

void quo_xa_unused_gpio_init(void);
void quo_xb_no_owner_gpio_init(void);
void quo_xc_no_owner_gpio_init(void);
void quo_xd_no_owner_gpio_init(void);

void ble_xa_unused_gpio_init(void);
void ble_xb_unused_gpio_init(void);
void ble_xb_no_owner_gpio_init(void);
void enr_td_xc_no_owner_gpio_init(void);
void enr_u_xc_no_owner_gpio_init(void);
void enr_u_xd_no_owner_gpio_init(void);
void enr_u_xe_no_owner_gpio_init(void);
#ifdef CONFIG_PM
void enr_xc_no_owner_gpio_suspend(void);
void enr_xc_no_owner_gpio_resume(void);
void ble_xa_unused_gpio_suspend(void);
void quo_xa_unused_gpio_suspend(void);
void quo_xb_no_owner_gpio_suspend(void);
void quo_xc_no_owner_gpio_suspend(void);
void quo_xd_no_owner_gpio_suspend(void);

void quo_local_test_gpio_suspend(void);
void local_gpio_config_test(char** gpioTable);
#endif

struct gpio_callbacks {
	void (*init) (void);
	void (*suspend) (void);
	void (*during_suspend) (void);
	void (*resume) (void);
};

void register_gpio_basic_callbacks(struct gpio_callbacks* callbacks);
void gpio_basic_during_suspend(void);
#endif
