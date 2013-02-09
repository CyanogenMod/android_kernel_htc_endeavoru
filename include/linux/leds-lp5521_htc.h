#ifndef _LINUX_LP5521_HTC_H
#define _LINUX_LP5521_HTC_H

#define LED_I2C_NAME "LP5521-LED"

#define LED_RESET		0
#define GREEN_ON		1
#define GREEN_BLINK		2
#define AMBER_ON		3
#define AMBER_BLINK		4
#define AMBER_LOW_BLINK		5
#define DUAL_COLOR_BLINK	6
#define BACKLIGHT_ON		7
#define	BACKLIGHT_BLINK		8

#define I2C_WRITE_RETRY_TIMES		1
#define LED_I2C_WRITE_BLOCK_SIZE	80

struct led_i2c_config {
	const char *name;
	int led_cur;
	int led_lux;
};

struct led_i2c_platform_data {
	struct led_i2c_config *led_config;
	int num_leds;
	int ena_gpio;
};

void led_behavior(struct i2c_client *client, int val);
void lp5521_led_current_set_for_key(int brightness_key);

#endif /*_LINUXLP5521-LED_H*/
