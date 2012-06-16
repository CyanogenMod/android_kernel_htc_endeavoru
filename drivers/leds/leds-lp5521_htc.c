/* driver/leds/leds-lp5521_htc.c
 *
 * Copyright (C) 2010 HTC Corporation.
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

#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/hrtimer.h>
#include <linux/interrupt.h>
#include <linux/android_alarm.h>
#include <linux/earlysuspend.h>
#include <linux/leds.h>
#include <linux/leds-lp5521_htc.h>
#include "../../arch/arm/mach-tegra/gpio-names.h"
#include <linux/regulator/consumer.h>

#define LP5521_MAX_LEDS			3	/* Maximum number of LEDs */
#define LED_DEBUG				1
#if LED_DEBUG
	#define D(x...) printk(KERN_DEBUG "[LED]" x)
	#define I(x...) printk(KERN_INFO "[LED]" x)		
#else
	#define D(x...)
	#define I(x...)
#endif

static int led_rw_delay;
static int current_state, current_blink, current_time;
static int current_mode, backlight_mode, suspend_mode, offtimer_mode;
static int amber_mode;
static struct regulator *regulator;
static struct i2c_client *private_lp5521_client;
static struct mutex	led_mutex;
static struct workqueue_struct *g_led_work_queue;
static struct work_struct led_powerkey_work;
static struct workqueue_struct *led_powerkey_work_queue;


struct lp5521_led {
	int			id;
	u8			chan_nr;
	u8			led_current;
	u8			max_current;
	struct led_classdev	cdev;
	struct work_struct	brightness_work;
	struct mutex led_data_mutex;
	u8			brightness;
	uint8_t 	blink;
	struct alarm led_alarm;
	struct work_struct led_work;
};

struct lp5521_chip {
	struct led_i2c_platform_data *pdata;
	struct mutex		led_i2c_rw_mutex; /* Serialize control */
	struct i2c_client	*client;
	struct lp5521_led	leds[LP5521_MAX_LEDS];
	struct early_suspend early_suspend_led;
};

static long unsigned int lp5521_led_tag_status = 0;
static int __init lp5521_led_tag(char *tag)
{
	if (strlen(tag))
		strict_strtoul(tag, 16, &lp5521_led_tag_status);
	/* mapping */
	if (lp5521_led_tag_status == 2)
		lp5521_led_tag_status = DUAL_COLOR_BLINK;
	else if(lp5521_led_tag_status == 3)
		lp5521_led_tag_status = GREEN_ON;
	else if(lp5521_led_tag_status == 4)
		lp5521_led_tag_status = AMBER_ON;
	else if(lp5521_led_tag_status == 5)
		lp5521_led_tag_status = AMBER_BLINK;
	else if(lp5521_led_tag_status == 6)
		lp5521_led_tag_status = AMBER_LOW_BLINK;
	else
		lp5521_led_tag_status = 0;

	return 1;
}
 __setup("led=", lp5521_led_tag);

static char *hex2string(uint8_t *data, int len)
{
	static char buf[LED_I2C_WRITE_BLOCK_SIZE*4];
	int i;

	i = (sizeof(buf) - 1) / 4;
	if (len > i)
		len = i;

	for (i = 0; i < len; i++)
		sprintf(buf + i * 4, "[%02X]", data[i]);

	return buf;
}

static int i2c_write_block(struct i2c_client *client, uint8_t addr,
	uint8_t *data, int length)
{
	int retry;
	uint8_t buf[LED_I2C_WRITE_BLOCK_SIZE];
	int i;
	struct lp5521_chip *cdata;
	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = length + 1,
			.buf = buf,
		}
	};

	dev_dbg(&client->dev, "W [%02X] = %s\n",
			addr, hex2string(data, length));

	cdata = i2c_get_clientdata(client);
	if (length + 1 > LED_I2C_WRITE_BLOCK_SIZE) {
		dev_err(&client->dev, "[LED] i2c_write_block length too long\n");
		return -E2BIG;
	}

	buf[0] = addr;
	for (i = 0; i < length; i++)
		buf[i+1] = data[i];

	mutex_lock(&cdata->led_i2c_rw_mutex);
	hr_msleep(1);
	for (retry = 0; retry < I2C_WRITE_RETRY_TIMES; retry++) {
		if (i2c_transfer(client->adapter, msg, 1) == 1)
			break;
		msleep(led_rw_delay);
	}
	if (retry >= I2C_WRITE_RETRY_TIMES) {
		dev_err(&client->dev, "[LED] i2c_write_block retry over %d times\n",
			I2C_WRITE_RETRY_TIMES);
		mutex_unlock(&cdata->led_i2c_rw_mutex);
		return -EIO;
	}
	mutex_unlock(&cdata->led_i2c_rw_mutex);
	
	return 0;
}

static void lp5521_led_enable(struct i2c_client *client)
{
	int ret = 0;
	uint8_t data;
	struct led_i2c_platform_data *pdata;

	pdata = client->dev.platform_data;
	// === led pin enable ===
	ret = gpio_direction_output(pdata->ena_gpio, 1);
	if (ret < 0) {
		pr_err("[LED] %s: gpio_direction_output high failed %d\n", __func__, ret);
		gpio_free(pdata->ena_gpio);
	}
	mutex_lock(&led_mutex);
	// === enable CHIP_EN ===
	data = 0x40;
	ret = i2c_write_block(client, 0x00, &data, 1);
	udelay(550);
	// === configuration control in power save mode===
	data = 0x29;
	ret = i2c_write_block(client, 0x08, &data, 1);
	// === set red current to 9.5mA ===
	data = (u8)95;
	ret = i2c_write_block(client, 0x05, &data, 1);
	// === set green current to 9.5mA ===
	data = (u8)95;
	ret = i2c_write_block(client, 0x06, &data, 1);
	// === set blue current to 0.2mA ===
	data = (u8)2;
	ret = i2c_write_block(client, 0x07, &data, 1);
	// === set blue channel to direct PWM control mode ===
	data = 0x03;
	ret = i2c_write_block(client, 0x01, &data, 1);
	mutex_unlock(&led_mutex);
}

static void lp5521_green_on(struct i2c_client *client)
{
	uint8_t data = 0x00;
	int ret;

	I(" %s +++\n" , __func__);
	if( current_mode == 0 && backlight_mode == 0 )
		lp5521_led_enable(client);
	current_mode = 1;
	mutex_lock(&led_mutex);
	// === set green pwm to 255 ===
	data = 0xff;
	ret = i2c_write_block(client, 0x03, &data, 1);

	// === run program with green direct control and blue direct program ===
	if ( backlight_mode == 2 )
		data = 0x0e;
	else
		data = 0x0f;
	ret = i2c_write_block(client, 0x01, &data, 1);
	udelay(200);
	if ( backlight_mode == 2 )
		data = 0x42;
	else
		data = 0x40;
	ret = i2c_write_block(client, 0x00, &data, 1);
	udelay(500);
	mutex_unlock(&led_mutex);
	I(" %s ---\n" , __func__);
}

static void lp5521_green_blink(struct i2c_client *client)
{
	uint8_t data = 0x00;
	int ret;

	I(" %s +++\n" , __func__);
	if( current_mode == 0 && backlight_mode == 0 )
		lp5521_led_enable(client);
	current_mode = 2;
	mutex_lock(&led_mutex);
	// === load program with green load program and blue direct program ===
	if ( backlight_mode == 2 )
		data = 0x06;
	else
		data = 0x07;
	ret = i2c_write_block(client, 0x01, &data, 1);
	udelay(200);
	
	// === function blink ===
	// === set pwm to 255 ===
	data = 0x40;
	ret = i2c_write_block(client, 0x30, &data, 1);
	data = 0xff;
	ret = i2c_write_block(client, 0x31, &data, 1);
	// === wait 0.064s ===
	data = 0x44;
	ret = i2c_write_block(client, 0x32, &data, 1);
	data = 0x00;
	ret = i2c_write_block(client, 0x33, &data, 1);
	// === set pwm to 0 ===
	data = 0x40;
	ret = i2c_write_block(client, 0x34, &data, 1);
	data = 0x00;
	ret = i2c_write_block(client, 0x35, &data, 1);
	// === wait 1.936s ===	
	data = 0x7c;
	ret = i2c_write_block(client, 0x36, &data, 1);
	data = 0x00;
	ret = i2c_write_block(client, 0x37, &data, 1);
	data = 0x7f;
	ret = i2c_write_block(client, 0x38, &data, 1);
	data = 0x00;
	ret = i2c_write_block(client, 0x39, &data, 1);
	// === clear register ===
	data = 0x00;
	ret = i2c_write_block(client, 0x3a, &data, 1);
	data = 0x00;
	ret = i2c_write_block(client, 0x3b, &data, 1);
	data = 0x00;
	ret = i2c_write_block(client, 0x3c, &data, 1);
	data = 0x00;
	ret = i2c_write_block(client, 0x3d, &data, 1);
	
	// === run program ===
	if ( backlight_mode == 2 )
		data = 0x0a;
	else
		data = 0x0b;
	ret = i2c_write_block(client, 0x01, &data, 1);
	udelay(200);
	if ( backlight_mode == 2 )
		data = 0x4a;
	else
		data = 0x48;
	ret = i2c_write_block(client, 0x00, &data, 1);
	udelay(500);
	mutex_unlock(&led_mutex);
	I(" %s ---\n" , __func__);
} 


static void lp5521_amber_on(struct i2c_client *client)
{
	uint8_t data = 0x00;
	int ret;

	I(" %s +++\n" , __func__);
	if( current_mode == 0 && backlight_mode == 0 )
		lp5521_led_enable(client);
	current_mode = 3;
	mutex_lock(&led_mutex);
	// === set amber pwm to 255 ===
	data = 0xff;
	ret = i2c_write_block(client, 0x02, &data, 1);

	// === run program with amber direct control and blue direct program ===
	if ( backlight_mode == 2 )
		data = 0x32;
	else
		data = 0x33;
	ret = i2c_write_block(client, 0x01, &data, 1);
	udelay(200);
	if ( backlight_mode == 2 )
		data = 0x42;
	else
		data = 0x40;
	ret = i2c_write_block(client, 0x00, &data, 1);
	udelay(500);
	mutex_unlock(&led_mutex);
	I(" %s ---\n" , __func__);
}

static void lp5521_amber_blink(struct i2c_client *client)
{
	uint8_t data = 0x00;
	int ret;

	I(" %s +++\n" , __func__);
	if( current_mode == 0 && backlight_mode == 0 )
		lp5521_led_enable(client);
	current_mode = 4;
	amber_mode = 2;
	mutex_lock(&led_mutex);
	// === load program with amber load program and blue direct program ===
	if ( backlight_mode == 2 )
		data = 0x12;
	else
		data = 0x13;
	ret = i2c_write_block(client, 0x01, &data, 1);
	udelay(200);

	// === function blink ===
	// === wait 0.999s ===
	data = 0x7f;
	ret = i2c_write_block(client, 0x10, &data, 1);
	data = 0x00;
	ret = i2c_write_block(client, 0x11, &data, 1);
	// === set pwm to 255 ===
	data = 0x40;
	ret = i2c_write_block(client, 0x12, &data, 1);
	data = 0xff;
	ret = i2c_write_block(client, 0x13, &data, 1);
	// === wait 0.064s ===
	data = 0x44;
	ret = i2c_write_block(client, 0x14, &data, 1);
	data = 0x00;
	ret = i2c_write_block(client, 0x15, &data, 1);
	// === set pwm to 0 ===
	data = 0x40;
	ret = i2c_write_block(client, 0x16, &data, 1);
	data = 0x00;
	ret = i2c_write_block(client, 0x17, &data, 1);
	// === wait 0.935s ===	
	data = 0x7c;
	ret = i2c_write_block(client, 0x18, &data, 1);
	data = 0x00;
	ret = i2c_write_block(client, 0x19, &data, 1);
	
	// === run program ===
	if ( backlight_mode == 2 )
		data = 0x22;
	else
		data = 0x23;
	ret = i2c_write_block(client, 0x01, &data, 1);
	udelay(200);
	if ( backlight_mode == 2 )
		data = 0x62;
	else
		data = 0x60;
	ret = i2c_write_block(client, 0x00, &data, 1);
	udelay(550);
	mutex_unlock(&led_mutex);
	I(" %s ---\n" , __func__);
}

static void lp5521_amber_low_blink(struct i2c_client *client)
{
	uint8_t data = 0x00;
	int ret;

	I(" %s +++\n" , __func__);
	if( current_mode == 0 && backlight_mode == 0 )
		lp5521_led_enable(client);
	current_mode = 5;
	mutex_lock(&led_mutex);
	// === load program with amber load program and blue direct program ===
	if ( backlight_mode == 2 )
		data = 0x12;
	else
		data = 0x13;
	ret = i2c_write_block(client, 0x01, &data, 1);
	udelay(200);
		
	// === function low blink ===
	// === set pwm to 255 ===
	data = 0x40;
	ret = i2c_write_block(client, 0x10, &data, 1);
	data = 0xff;
	ret = i2c_write_block(client, 0x11, &data, 1);
	// === wait 0.999s ===
	data = 0x7f;
	ret = i2c_write_block(client, 0x12, &data, 1);
	data = 0x00;
	ret = i2c_write_block(client, 0x13, &data, 1);
	// === set pwm to 0 ===
	data = 0x40;
	ret = i2c_write_block(client, 0x14, &data, 1);
	data = 0x00;
	ret = i2c_write_block(client, 0x15, &data, 1);
	// === wait 0.999s ===	
	data = 0x7f;
	ret = i2c_write_block(client, 0x16, &data, 1);
	// === clear register ===	
	data = 0x00;
	ret = i2c_write_block(client, 0x17, &data, 1);
	data = 0x00;
	ret = i2c_write_block(client, 0x18, &data, 1);
	data = 0x00;
	ret = i2c_write_block(client, 0x19, &data, 1);
	
	// === run program ===
	if ( backlight_mode == 2 )
		data = 0x22;
	else
		data = 0x23;
	ret = i2c_write_block(client, 0x01, &data, 1);
	udelay(200);
	if ( backlight_mode == 2 )
		data = 0x62;
	else
		data = 0x60;
	ret = i2c_write_block(client, 0x00, &data, 1);
	udelay(550);
	mutex_unlock(&led_mutex);
	I(" %s ---\n" , __func__);
}

static void lp5521_dual_color_blink(struct i2c_client *client)
{
	uint8_t data = 0x00;
	int ret;

	I(" %s +++\n" , __func__);
	if( current_mode == 0 && backlight_mode == 0 )
		lp5521_led_enable(client);
	current_mode = 6;
	mutex_lock(&led_mutex);
	// === load program to with amber/green load program and blue direct program ===
	if ( backlight_mode == 2 )
		data = 0x16;
	else
		data = 0x17;
	ret = i2c_write_block(client, 0x01, &data, 1);
	udelay(200);


	// === function fast blink ===
	// === set pwm to 255 ===
	data = 0x40;
	ret = i2c_write_block(client, 0x10, &data, 1);
	data = 0xff;
	ret = i2c_write_block(client, 0x11, &data, 1);
	// === wait 0.064s ===
	data = 0x44;
	ret = i2c_write_block(client, 0x12, &data, 1);
	data = 0x00;
	ret = i2c_write_block(client, 0x13, &data, 1);
	// === set pwm to 0 ===
	data = 0x40;
	ret = i2c_write_block(client, 0x14, &data, 1);
	data = 0x00;
	ret = i2c_write_block(client, 0x15, &data, 1);
	// === wait 0.25s ===	
	data = 0x50;
	ret = i2c_write_block(client, 0x16, &data, 1);
	data = 0x00;
	ret = i2c_write_block(client, 0x17, &data, 1);
	// === trigger sg, wg ===
	data = 0xe1;
	ret = i2c_write_block(client, 0x18, &data, 1);
	data = 0x04;
	ret = i2c_write_block(client, 0x19, &data, 1);
	udelay(550);

	// === trigger wr ===
	data = 0xe0;
	ret = i2c_write_block(client, 0x30, &data, 1);
	data = 0x80;
	ret = i2c_write_block(client, 0x31, &data, 1);
	udelay(550);
	// set pwm to 255
	data = 0x40;
	ret = i2c_write_block(client, 0x32, &data, 1);
	data = 0xff;
	ret = i2c_write_block(client, 0x33, &data, 1);
	// === wait 0.064s ===
	data = 0x44;
	ret = i2c_write_block(client, 0x34, &data, 1);
	data = 0x00;
	ret = i2c_write_block(client, 0x35, &data, 1);
	// === set pwm to 0 ===
	data = 0x40;
	ret = i2c_write_block(client, 0x36, &data, 1);
	data = 0x00;
	ret = i2c_write_block(client, 0x37, &data, 1);
	// === wait 0.999s ===	
	data = 0x7f;
	ret = i2c_write_block(client, 0x38, &data, 1);
	data = 0x00;
	ret = i2c_write_block(client, 0x39, &data, 1);
	// === wait 0.622s ===
	data = 0x68;
	ret = i2c_write_block(client, 0x3a, &data, 1);
	data = 0x00;
	ret = i2c_write_block(client, 0x3b, &data, 1);	
	// === trigger sr ===
	data = 0xe0;
	ret = i2c_write_block(client, 0x3c, &data, 1);
	data = 0x02;
	ret = i2c_write_block(client, 0x3d, &data, 1);
	udelay(550);
	
	// === run program ===
	if ( backlight_mode == 2 )
		data = 0x2a;
	else
		data = 0x2b;
	ret = i2c_write_block(client, 0x01, &data, 1);
	udelay(200);
	if ( backlight_mode == 2 )
		data = 0x6a;
	else
		data = 0x68;
	ret = i2c_write_block(client, 0x00, &data, 1);
	udelay(550);
	mutex_unlock(&led_mutex);
	I(" %s ---\n" , __func__);
}

static void lp5521_backlight_on(struct i2c_client *client)
{
	uint8_t data = 0x00;
	int ret, brightness; 

	I(" %s +++\n" , __func__);
	if( current_mode == 0 && backlight_mode == 0 )
		lp5521_led_enable(client);
	backlight_mode = 1;
	mutex_lock(&led_mutex);
	// === set blue pwm to 60 ===
	for( brightness = 1;brightness <= 6;brightness++) {
		data = (u8)brightness*10;
		ret = i2c_write_block(client, 0x04, &data, 1);
		if( ret < 0 )
			break;
		msleep(20);
	}
	mutex_unlock(&led_mutex);
	I(" %s ---\n" , __func__);
}

static void lp5521_backlight_off(struct i2c_client *client)
{
	uint8_t data = 0x00;
	int ret, brightness; 
	struct led_i2c_platform_data *pdata;

	I(" %s +++\n" , __func__);
	pdata = client->dev.platform_data;
	backlight_mode = 0;
	mutex_lock(&led_mutex);
	// === set blue pwm to 0 ===
	for( brightness = 6;brightness >= 0;brightness--) {
		data = (u8)brightness*10;
		ret = i2c_write_block(client, 0x04, &data, 1);
		if( ret < 0 )
			break;
		msleep(20);
	}
	if( current_mode == 0 ) {
		if( suspend_mode == 1 ) {
			// === reset register ===
			data = 0xff;
			ret = i2c_write_block(client, 0x0d, &data, 1);
			udelay(550);
			gpio_direction_output(pdata->ena_gpio, 0);
			I(" no LED command now in suspend, reset chip & gpio, no ack in i2c 0x32 is correct in LED chip lp5521.\n");
		} else {
			// === disable CHIP_EN ===
			data = 0x00;
			ret = i2c_write_block(client, 0x00, &data, 1);
			udelay(550);
			gpio_direction_output(pdata->ena_gpio, 0);
			I(" no LED command now, disable chip & gpio.\n");
		}
	}
	mutex_unlock(&led_mutex);
	I(" %s ---\n" , __func__);

}

static void lp5521_dual_off(struct i2c_client *client)
{
	uint8_t data = 0x00;
	int ret;
	struct led_i2c_platform_data *pdata;

	I(" %s +++\n" , __func__);
	pdata = client->dev.platform_data;
	mutex_lock(&led_mutex);
	// === set green pwm to 0 ===
	data = 0x00;
	if( current_mode == 1 )
		ret = i2c_write_block(client, 0x03, &data, 1);
	else if( current_mode == 3 )
		ret = i2c_write_block(client, 0x02, &data, 1);
	current_mode = 0;
	if( backlight_mode == 1 ) {
		data = 0x03;
		ret = i2c_write_block(client, 0x01, &data, 1);
		udelay(200);
	} else if ( backlight_mode == 2 )  {
		data = 0x02;
		ret = i2c_write_block(client, 0x01, &data, 1);
		udelay(200);
	} else {
		data = 0x00;
		ret = i2c_write_block(client, 0x01, &data, 1);
		udelay(200);
		if( suspend_mode == 1 ) {
			// === reset register ===
			data = 0xff;
			ret = i2c_write_block(client, 0x0d, &data, 1);
			udelay(550);
			gpio_direction_output(pdata->ena_gpio, 0);
			I(" no LED command now in suspend, reset chip & gpio, no ack in i2c 0x32 is correct in LED chip lp5521.\n");
		} else {
			// === disable CHIP_EN ===
			data = 0x00;
			ret = i2c_write_block(client, 0x00, &data, 1);
			udelay(550);
			gpio_direction_output(pdata->ena_gpio, 0);
			I(" no LED command now in idle, disable chip & gpio.\n");
		}
	}
	mutex_unlock(&led_mutex);
	I(" %s ---\n" , __func__);
}

void lp5521_led_current_set_for_key(int brightness_key)
{
	I(" %s +++\n" , __func__);
	if (brightness_key)
		backlight_mode = 2;
	else 
		backlight_mode = 0;
	queue_work(led_powerkey_work_queue, &led_powerkey_work);
	I(" %s ---\n" , __func__);
}


void led_behavior(struct i2c_client *client, int val)
{
	switch(val)
	{	
		case LED_RESET:
			lp5521_led_enable(client);
			break;	
		case GREEN_ON:
			lp5521_green_on(client);
			break;	
		case GREEN_BLINK:
			lp5521_green_blink(client);
			break;	
		case AMBER_ON:
			lp5521_amber_on(client);
			break;	
		case AMBER_BLINK:
			lp5521_amber_blink(client);
			break;	
		case AMBER_LOW_BLINK:
			lp5521_amber_low_blink(client);
			break;	
		case DUAL_COLOR_BLINK:
			lp5521_dual_color_blink(client);
			break;	
		case BACKLIGHT_ON:
			lp5521_backlight_on(client);
			break;
		case BACKLIGHT_BLINK:
			lp5521_led_current_set_for_key(1);
			break;
		default:
			break;
	}
}
EXPORT_SYMBOL(led_behavior);

static ssize_t led_behavior_set(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct i2c_client *client;
	int val;

	val = -1;
	sscanf(buf, "%d", &val);
	if (val < 0 && val > 8)
		return -EINVAL;
	
	client = to_i2c_client(dev);
	led_behavior(client, val);
	current_state = val;
	
	return count;
}

static ssize_t led_behavior_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", current_state);
}

static DEVICE_ATTR(behavior, 0644, led_behavior_show, led_behavior_set);

static void lp5521_led_birghtness_set(struct led_classdev *led_cdev,
			       enum led_brightness brightness)
{
	struct i2c_client *client = private_lp5521_client;
	struct lp5521_led *ldata;

	if (brightness < 0)
		brightness = 0;
	else if (brightness > 255)
		brightness = 255;
	ldata = container_of(led_cdev, struct lp5521_led, cdev);
	if(brightness) {
		if(!strcmp(ldata->cdev.name, "green"))	 {
			lp5521_green_on(client);
		} else if (!strcmp(ldata->cdev.name, "amber")) {
			lp5521_amber_on(client);
		} else if (!strcmp(ldata->cdev.name, "button-backlight")) {
			if ( backlight_mode != 2 )
				lp5521_backlight_on(client);
		}
	} else {
		if (!strcmp(ldata->cdev.name, "button-backlight")) {
			if( backlight_mode == 1 ) 
				lp5521_backlight_off(client);
		}else if(!strcmp(ldata->cdev.name, "amber"))	 {
			if( current_mode == 3 )
				lp5521_dual_off(client);
		}else if(!strcmp(ldata->cdev.name, "green"))	 {
			if( current_mode == 1)
				lp5521_dual_off(client);
		}
	}
}

static void led_powerkey_work_func(struct work_struct *work)
{
	struct i2c_client *client = private_lp5521_client;
	struct led_i2c_platform_data *pdata;
	uint8_t data;
	int ret;
	
	I(" %s +++\n" , __func__);
	pdata = client->dev.platform_data;
	if( current_mode == 0  )
		lp5521_led_enable(client);
	mutex_lock(&led_mutex);
	if (backlight_mode == 2) {
		if(current_mode == 1) {
			// === load program with green direct and blue load program ===
			data = 0x0d;
			ret = i2c_write_block(client, 0x01, &data, 1);
			udelay(200);
		}else if( current_mode == 3 ) {
			// === load program with red direct and blue load program ===
			data = 0x31;
			ret = i2c_write_block(client, 0x01, &data, 1);
			udelay(200);
		}else if( current_mode == 2 ) {
			// === load program with green run and blue load program ===
			data = 0x09;
			ret = i2c_write_block(client, 0x01, &data, 1);
			udelay(200);
		}else if( current_mode == 4 || current_mode == 5 ) {
			// === load program with red run and blue load program ===
			data = 0x21;
			ret = i2c_write_block(client, 0x01, &data, 1);
			udelay(200);
		}else if( current_mode == 6 ) {
			// === load program with red and green run and blue load program ===
			data = 0x29;
			ret = i2c_write_block(client, 0x01, &data, 1);
			udelay(200);
		}else {
			// === load program with blue load program ===
			data = 0x01;
			ret = i2c_write_block(client, 0x01, &data, 1);
			udelay(200);
		}
		// === function virtual key blink ===
		// === set pwm to 60 ===
		data = 0x40;
		ret = i2c_write_block(client, 0x50, &data, 1);
		data = (u8)60;
		ret = i2c_write_block(client, 0x51, &data, 1);
		// === wait 0.2s ===
		data = 0x4d;
		ret = i2c_write_block(client, 0x52, &data, 1);
		data = 0x00;
		ret = i2c_write_block(client, 0x53, &data, 1);
		// === set pwm to 0 ===
		data = 0x40;
		ret = i2c_write_block(client, 0x54, &data, 1);
		data = 0x00;
		ret = i2c_write_block(client, 0x55, &data, 1);
		// === wait 0.2s ===
		data = 0x4d;
		ret = i2c_write_block(client, 0x56, &data, 1);
		data = 0x00;
		ret = i2c_write_block(client, 0x57, &data, 1);
		if(current_mode == 1) {
			// === run program with green direct and blue run program ===
			data = 0x0e;
			ret = i2c_write_block(client, 0x01, &data, 1);
			udelay(200);
			data = 0x42;
			ret = i2c_write_block(client, 0x00, &data, 1);
			udelay(550);
		}else if( current_mode == 3 ) {
			// === run program with red direct and blue run program ===
			data = 0x32;
			ret = i2c_write_block(client, 0x01, &data, 1);
			udelay(200);
			data = 0x42;
			ret = i2c_write_block(client, 0x00, &data, 1);
			udelay(550);
		}else if( current_mode == 2 ) {
			// === run program with green run and blue run program ===
			data = 0x0a;
			ret = i2c_write_block(client, 0x01, &data, 1);
			udelay(200);
			data = 0x4a;
			ret = i2c_write_block(client, 0x00, &data, 1);
			udelay(550);
		}else if( current_mode == 4 || current_mode == 5 ) {
			// === run program with red run and blue run program ===
			data = 0x22;
			ret = i2c_write_block(client, 0x01, &data, 1);
			udelay(200);
			data = 0x62;
			ret = i2c_write_block(client, 0x00, &data, 1);
			udelay(550);
		}else if( current_mode == 6 ) {
			// === run program with red and green run and blue run program ===
			data = 0x2a;
			ret = i2c_write_block(client, 0x01, &data, 1);
			udelay(200);
			data = 0x6a;
			ret = i2c_write_block(client, 0x00, &data, 1);
			udelay(550);
		}else {
			// === run program with blue load program ===
			data = 0x02;
			ret = i2c_write_block(client, 0x01, &data, 1);
			udelay(200);
			data = 0x42;
			ret = i2c_write_block(client, 0x00, &data, 1);
			udelay(550);
		}
	}else if (backlight_mode == 0) {
		// === set blue channel to direct PWM control mode ===
		if(current_mode == 1) {
			// === run program with green direct and blue direct program ===
			data = 0x0f;
			ret = i2c_write_block(client, 0x01, &data, 1);
			udelay(200);
			data = 0x40;
			ret = i2c_write_block(client, 0x00, &data, 1);
			udelay(500);
			data = 0x00;
			ret = i2c_write_block(client, 0x04, &data, 1);
		}else if( current_mode == 3 ) {
			// === run program with red direct and blue direct program ===
			data = 0x33;
			ret = i2c_write_block(client, 0x01, &data, 1);
			udelay(200);
			data = 0x40;
			ret = i2c_write_block(client, 0x00, &data, 1);
			udelay(500);
			data = 0x00;
			ret = i2c_write_block(client, 0x04, &data, 1);
		}else if( current_mode == 2 ) {
			// === run program with green run and blue direct program ===
			data = 0x0b;
			ret = i2c_write_block(client, 0x02, &data, 1);
			udelay(200);
			data = 0x48;
			ret = i2c_write_block(client, 0x00, &data, 1);
			udelay(500);
			data = 0x00;
			ret = i2c_write_block(client, 0x04, &data, 1);
		}else if( current_mode == 4 || current_mode == 5 ) {
			// === run program with red run and blue direct program ===
			data = 0x23;
			ret = i2c_write_block(client, 0x02, &data, 1);
			udelay(200);
			data = 0x60;
			ret = i2c_write_block(client, 0x00, &data, 1);
			udelay(500);
		}else if( current_mode == 6 ) {
			// === run program with red and green run and blue direct program ===
			data = 0x2b;
			ret = i2c_write_block(client, 0x02, &data, 1);
			udelay(200);
			data = 0x68;
			ret = i2c_write_block(client, 0x00, &data, 1);
			udelay(500);
			data = 0x00;
			ret = i2c_write_block(client, 0x04, &data, 1);
		}else {
			// === run program with blue direct program ===
			data = 0x03;
			ret = i2c_write_block(client, 0x02, &data, 1);
			udelay(200);
			data = 0x40;
			ret = i2c_write_block(client, 0x00, &data, 1);
			udelay(500);
			data = 0x00;
			ret = i2c_write_block(client, 0x04, &data, 1);
			// === disable CHIP_EN ===
			data = 0x00;
			ret = i2c_write_block(client, 0x00, &data, 1);
			udelay(550);
			gpio_direction_output(pdata->ena_gpio, 0);
			I(" no LED command now, disable chip & gpio.\n");
		}
	}
	mutex_unlock(&led_mutex);
	I(" %s ---\n" , __func__);

}

static void led_work_func(struct work_struct *work)
{
	struct i2c_client *client = private_lp5521_client;
	struct lp5521_led *ldata;
	
	I(" %s +++\n" , __func__);
	ldata = container_of(work, struct lp5521_led, led_work);
	if ( offtimer_mode == current_mode )
		lp5521_dual_off(client);
	offtimer_mode = 0;
	I(" %s ---\n" , __func__);
}

static void led_alarm_handler(struct alarm *alarm)
{
	struct lp5521_led *ldata;
	
	I(" %s +++\n" , __func__);
	ldata = container_of(alarm, struct lp5521_led, led_alarm);
	queue_work(g_led_work_queue, &ldata->led_work);
	I(" %s ---\n" , __func__);
}
static ssize_t lp5521_led_off_timer_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", current_time);;
}

static ssize_t lp5521_led_off_timer_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct led_classdev *led_cdev;
	struct lp5521_led *ldata;
	int min, sec;
	uint16_t off_timer;
	ktime_t interval;
	ktime_t next_alarm;

	min = -1;
	sec = -1;
	sscanf(buf, "%d %d", &min, &sec);
	I(" %s +++, min = %d, sec = %d\n" , __func__, min, sec);
	if (min < 0 || min > 255)
		return -EINVAL;
	if (sec < 0 || sec > 255)
		return -EINVAL;

	led_cdev = (struct led_classdev *)dev_get_drvdata(dev);
	ldata = container_of(led_cdev, struct lp5521_led, cdev);

	off_timer = min * 60 + sec;

	if(off_timer) {
		if(!strcmp(ldata->cdev.name, "green"))	 {
			if( current_mode != 1 &&  current_mode != 2 && current_mode != 6 )
				return count;
		} else if (!strcmp(ldata->cdev.name, "amber")) {
			if( current_mode != 3 &&  current_mode != 4 && current_mode != 5 && current_mode != 6)
				return count;
		} else if (!strcmp(ldata->cdev.name, "button-backlight")) {
			if( backlight_mode != 1 &&  backlight_mode != 2 )
				return count;
		} 
	}
	current_time = off_timer;
	alarm_cancel(&ldata->led_alarm);
	cancel_work_sync(&ldata->led_work);
	offtimer_mode = current_mode;
	if (off_timer) {
		interval = ktime_set(off_timer, 0);
		next_alarm = ktime_add(alarm_get_elapsed_realtime(), interval);
		alarm_start_range(&ldata->led_alarm, next_alarm, next_alarm);
	}

	return count;
}

static DEVICE_ATTR(off_timer, 0644, lp5521_led_off_timer_show,
					lp5521_led_off_timer_store);

static ssize_t lp5521_led_blink_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", current_blink);
}

static ssize_t lp5521_led_blink_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct i2c_client *client = private_lp5521_client;
	struct led_classdev *led_cdev;
	struct lp5521_led *ldata;
	int val;

	val = -1;
	sscanf(buf, "%d", &val);
	I(" %s +++, val = %d\n" , __func__, val);
	if (val < 0 )
		val = 0;
	else if (val > 255)
		val = 255;

	led_cdev = (struct led_classdev *)dev_get_drvdata(dev);
	ldata = container_of(led_cdev, struct lp5521_led, cdev);

	if(val) {
		if(!strcmp(ldata->cdev.name, "green"))	 {
			if( (current_mode == 4 || amber_mode == 2) && val == 3 )
				lp5521_dual_color_blink(client);
			else
				lp5521_green_blink(client);
		} else if (!strcmp(ldata->cdev.name, "amber")) {
			if( val == 4 )
				lp5521_amber_low_blink(client);
			else
				lp5521_amber_blink(client);
		} else if (!strcmp(ldata->cdev.name, "button-backlight")) {
			if ( backlight_mode != 2 )
				lp5521_led_current_set_for_key(1);
		} 
	} else {
		if(!strcmp(ldata->cdev.name, "amber"))	 {
			if( amber_mode == 2 )
				amber_mode = 0;
			if( current_mode == 4 || current_mode == 5 || current_mode == 6 )
				lp5521_dual_off(client);
		}else if(!strcmp(ldata->cdev.name, "green")) {
			if( current_mode == 2 || current_mode == 6 )
				lp5521_dual_off(client);
		}else if (!strcmp(ldata->cdev.name, "button-backlight")) {
			if ( backlight_mode == 2 )
				lp5521_led_current_set_for_key(0);
		} 
	}
	current_blink= val;
	
	return count;
}

static DEVICE_ATTR(blink, 0644, lp5521_led_blink_show,
					lp5521_led_blink_store);

static void lp5521_led_early_suspend(struct early_suspend *handler)
{
	struct i2c_client *client = private_lp5521_client;
	
	printk("[LED][SUSPEND] lp5521_led_early_suspend +++\n");
	suspend_mode = 1;
	if( backlight_mode == 1 )
		lp5521_backlight_off(client);
	else if ( backlight_mode == 2 )
		lp5521_led_current_set_for_key(0);
	printk("[LED][SUSPEND] lp5521_led_early_suspend ---\n");
}

static void lp5521_led_late_resume(struct early_suspend *handler)
{
	printk("[LED][RESUME] lp5521_led_late_resume +++\n");
	suspend_mode = 0;
	printk("[LED][RESUME] lp5521_led_late_resume ---\n");
}

static int lp5521_led_probe(struct i2c_client *client
	, const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct lp5521_chip		*cdata;
	struct led_i2c_platform_data *pdata;
	int ret, i;
	uint8_t data;

	printk("[LED][PROBE] led driver probe +++\n");
	
	// === init platform and client data ===
	cdata = kzalloc(sizeof(struct lp5521_chip), GFP_KERNEL);
	if (!cdata) {
		ret = -ENOMEM;
		dev_err(&client->dev, "[LED][PROBE_ERR] failed on allocat cdata\n");
		goto err_cdata;
	}
	i2c_set_clientdata(client, cdata);
	cdata->client = client;

	pdata = client->dev.platform_data;
	if (!pdata) {
		ret = -EBUSY;
		dev_err(&client->dev, "[LED][PROBE_ERR] failed on get pdata\n");
		goto err_exit;
	}
	led_rw_delay = 5;

	// === led enable pin ===
	ret = gpio_request(pdata->ena_gpio, "led_enable");
	if (ret < 0) {
		pr_err("[LED] %s: gpio_request failed %d\n", __func__, ret);
		return ret;
	}
	ret = gpio_direction_output(pdata->ena_gpio, lp5521_led_tag_status ? 1 : 0);
	if (ret < 0) {
		pr_err("[LED] %s: gpio_direction_output failed %d\n", __func__, ret);
		gpio_free(pdata->ena_gpio);
		return ret;
	}   
   	tegra_gpio_enable(pdata->ena_gpio);

	// === led trigger signal pin ===
	ret = gpio_request(pdata->tri_gpio, "led_trigger");
	if (ret < 0) {
		pr_err("[LED] %s: gpio_request failed %d\n", __func__, ret);
		return ret;
	}
	ret = gpio_direction_output(pdata->tri_gpio, 0); 
	if (ret < 0) {
		pr_err("[LED] %s: gpio_direction_output failed %d\n", __func__, ret);
		gpio_free(pdata->tri_gpio);
		return ret;
	}
	tegra_gpio_enable(pdata->tri_gpio);

	private_lp5521_client = client;
	g_led_work_queue = create_workqueue("led");
	if (!g_led_work_queue)
		goto err_create_work_queue;
	led_powerkey_work_queue = create_workqueue("led_powerkey");
	if (!led_powerkey_work_queue)
		goto err_create_work_queue;
	//intail LED config
	for (i = 0; i < pdata->num_leds; i++) {
		cdata->leds[i].cdev.name = pdata->led_config[i].name;
		cdata->leds[i].cdev.brightness_set = lp5521_led_birghtness_set;
		ret = led_classdev_register(dev, &cdata->leds[i].cdev);
		if (ret < 0) {
			dev_err(dev, "couldn't register led[%d]\n", i);
			return ret;
		}
		ret = device_create_file(cdata->leds[i].cdev.dev, &dev_attr_blink);
		if (ret < 0) {
			pr_err("%s: failed on create attr blink [%d]\n", __func__, i);
			goto err_register_attr_blink;
		}
		ret = device_create_file(cdata->leds[i].cdev.dev, &dev_attr_off_timer);
		if (ret < 0) {
			pr_err("%s: failed on create attr blink [%d]\n", __func__, i);
			goto err_register_attr_off_timer;
		}
		INIT_WORK(&cdata->leds[i].led_work, led_work_func);
		alarm_init(&cdata->leds[i].led_alarm,
				   ANDROID_ALARM_ELAPSED_REALTIME_WAKEUP,
				   led_alarm_handler);
	}
	INIT_WORK(&led_powerkey_work, led_powerkey_work_func);

	// === create device node ===
	ret = device_create_file(&client->dev, &dev_attr_behavior);
	if (ret) {
		dev_err(&client->dev, "device_create_file failed\n");
		goto err_fun_init;
	}
	
	regulator = regulator_get(NULL, "v_led_3v3");
	if( (regulator==NULL) | (IS_ERR(regulator)))  
		pr_err("Fail to get regulator: v_led_3v3");
	regulator_enable(regulator);
	
	mutex_init(&cdata->led_i2c_rw_mutex);
	mutex_init(&led_mutex);
	current_mode = backlight_mode = suspend_mode = offtimer_mode = amber_mode = 0;
	
#ifdef CONFIG_HAS_EARLYSUSPEND
	cdata->early_suspend_led.suspend = lp5521_led_early_suspend;
	cdata->early_suspend_led.resume = lp5521_led_late_resume;
	register_early_suspend(&cdata->early_suspend_led);
#endif
	if (lp5521_led_tag_status) {
		led_behavior(client, lp5521_led_tag_status);
	} else {
		// === disable CHIP_EN ===
		data = 0x00;
		ret = i2c_write_block(client, 0x00, &data, 1);
		udelay(550);
		gpio_direction_output(pdata->ena_gpio, 0);
	}

	printk("[LED][PROBE] led driver probe ---\n");
	return 0;

err_fun_init:
	device_remove_file(&client->dev, &dev_attr_behavior);
	kfree(cdata);
err_register_attr_blink:
	for (i = 0; i < pdata->num_leds; i++) {
		device_remove_file(cdata->leds[i].cdev.dev,&dev_attr_blink);
	}
err_register_attr_off_timer:
	for (i = 0; i < pdata->num_leds; i++) {
		device_remove_file(cdata->leds[i].cdev.dev,&dev_attr_off_timer);
	}
err_create_work_queue:
	kfree(pdata);
err_exit:
	kfree(cdata);
err_cdata:
	return ret;
}

static int __devexit lp5521_led_remove(struct i2c_client *client)
{
	struct led_i2c_platform_data *pdata;
	struct lp5521_chip *cdata;
	int i;

	cdata = i2c_get_clientdata(client);
	cdata = kzalloc(sizeof(struct lp5521_chip), GFP_KERNEL);
	i2c_set_clientdata(client, cdata);
	cdata->client = client;
	pdata = client->dev.platform_data;
	gpio_direction_output(pdata->ena_gpio, 0);
	if ( (regulator_is_enabled(regulator)) > 0)
		regulator_disable(regulator);
	regulator_put(regulator);
	device_remove_file(&client->dev, &dev_attr_behavior);
	unregister_early_suspend(&cdata->early_suspend_led);
	for (i = 0; i < pdata->num_leds; i++) {
		device_remove_file(cdata->leds[i].cdev.dev,&dev_attr_blink);
		device_remove_file(cdata->leds[i].cdev.dev,&dev_attr_off_timer);
		led_classdev_unregister(&cdata->leds[i].cdev);
	}
	destroy_workqueue(g_led_work_queue);
	destroy_workqueue(led_powerkey_work_queue);
	kfree(cdata);

	return 0;
}

// === LED driver info ===
static const struct i2c_device_id led_i2c_id[] = {
	{ LED_I2C_NAME, 0 },
	{}
};


static struct i2c_driver led_i2c_driver = {
	.driver = {
		   .name = LED_I2C_NAME,
		   },
	.id_table = led_i2c_id,
	.probe = lp5521_led_probe,
	.remove = __devexit_p(lp5521_led_remove),
};

static int __init lp5521_led_init(void)
{
	int ret;
	
	ret = i2c_add_driver(&led_i2c_driver);
	if (ret)
		return ret;
	return 0;
}

static void __exit lp5521_led_exit(void)
{
	i2c_del_driver(&led_i2c_driver);
}

module_init(lp5521_led_init);
module_exit(lp5521_led_exit);

MODULE_AUTHOR("<ShihHao_Shiung@htc.com>, <Dirk_Chang@htc.com>");
MODULE_DESCRIPTION("LP5521 LED driver");

