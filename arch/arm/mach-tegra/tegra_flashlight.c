/*
 * arch/arm/mach-msm/msm_flashlight.c - The flashlight driver
 * Copyright (C) 2009  HTC Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */


#include <asm/gpio.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <mach/tegra_flashlight.h>
//#include <mach/msm_iomap.h>
#include <asm/io.h>
#include <linux/leds.h>
#include <linux/hrtimer.h>
#include <linux/wakelock.h>
#include <linux/slab.h>
#include <media/aat1271.h>

#define FLT_DEBUG				1

#if FLT_DEBUG
#define FLT_DBG_LOG(fmt, ...) \
		printk(KERN_DEBUG "[FLT]" fmt, ##__VA_ARGS__)
#define FLT_INFO_LOG(fmt, ...) \
		printk(KERN_INFO "[FLT]" fmt, ##__VA_ARGS__)
#define FLT_ERR_LOG(fmt, ...) \
		printk(KERN_ERR "[FLT][ERR]" fmt, ##__VA_ARGS__)
#else
#define FLT_DBG_LOG(fmt, ...)
#define FLT_INFO_LOG(fmt, ...)
#define FLT_ERR_LOG(fmt, ...)
#endif


struct flashlight_struct {
	struct led_classdev fl_lcdev;
	struct early_suspend early_suspend_flashlight;
	struct hrtimer timer;
	struct wake_lock wake_lock;
	spinlock_t spin_lock;
	uint32_t gpio_torch;
	uint32_t gpio_flash;
	uint32_t torch_set1;
	uint32_t torch_set2;
	uint32_t gpio_flash_adj;
	uint32_t flash_sw_timeout_ms;
	enum flashlight_mode_flags mode_status;
	unsigned long spinlock_flags;
	unsigned flash_adj_gpio_status;
	/* inactive: 0x0
	 * active: 0x1
	 * force disable flashlight function: 0x2 */
	uint8_t flash_adj_value;
	uint8_t led_count;
	uint32_t chip_model;
};

/* disable it, we didn't need to adjust GPIO */
/* #define FLASHLIGHT_ADJ_FUNC */

static struct flashlight_struct *this_fl_str;

static void flashlight_aat3177_hw_command(uint8_t data)
{
	uint8_t loop_j;
	for (loop_j = 0; loop_j < data; loop_j++) {
		gpio_direction_output(this_fl_str->gpio_flash, 0);
		ndelay(50);
		gpio_direction_output(this_fl_str->gpio_flash, 1);
		ndelay(50);
	}
	udelay(500);
}

static void flashlight_hw_command(uint8_t addr, uint8_t data)
{
	uint8_t loop_i, loop_j;
	const uint8_t fl_addr_to_rising_count[4] = { 17, 18, 19, 20 };
	uint8_t loop_tmp;
	if (!this_fl_str->gpio_torch && !this_fl_str->gpio_flash) {
		FLT_ERR_LOG("%s: not setup GPIO??? torch: %d, flash: %d\n",
					__func__, this_fl_str->gpio_torch,
						this_fl_str->gpio_flash);
		return;
	}
	for (loop_j = 0; loop_j < 2; loop_j++) {
		if (!loop_j)
			loop_tmp = fl_addr_to_rising_count[addr];
		else
			loop_tmp = data;
		for (loop_i = 0; loop_i < loop_tmp; loop_i++) {
			gpio_direction_output(this_fl_str->gpio_torch, 0);
			udelay(2);
			gpio_direction_output(this_fl_str->gpio_torch, 1);
			udelay(2);
		}
		udelay(500);
	}
}

static void flashlight_turn_off(void)
{
	if (this_fl_str->mode_status == FL_MODE_OFF)
		return;

	gpio_direction_output(this_fl_str->gpio_flash, 0);
	if (this_fl_str->chip_model == AAT1271 || this_fl_str->chip_model == AAT1277)
		gpio_direction_output(this_fl_str->gpio_torch, 0);
	this_fl_str->mode_status = FL_MODE_OFF;
	this_fl_str->fl_lcdev.brightness = LED_OFF;
}

static enum hrtimer_restart flashlight_hrtimer_func(struct hrtimer *timer)
{
	struct flashlight_struct *fl_str = container_of(timer,
			struct flashlight_struct, timer);
	spin_lock_irqsave(&fl_str->spin_lock, fl_str->spinlock_flags);
	flashlight_turn_off();
	spin_unlock_irqrestore(&fl_str->spin_lock, fl_str->spinlock_flags);
	FLT_INFO_LOG("%s: turn off flash mode\n", __func__);
	return HRTIMER_NORESTART;
}

int aat1277_flashlight_control(int mode)
{
	int ret = 0;
	uint32_t flash_ns = ktime_to_ns(ktime_get());

#if 0 /* disable flash_adj_value check now */
	if (this_fl_str->flash_adj_value == 2) {
		printk(KERN_WARNING "%s: force disable function!\n", __func__);
		return -EIO;
	}
#endif
	if (this_fl_str->mode_status == mode) {
		FLT_INFO_LOG("%s: mode is same: %d\n",
							FLASHLIGHT_NAME, mode);

		if (!hrtimer_active(&this_fl_str->timer) &&
			this_fl_str->mode_status == FL_MODE_OFF) {
			FLT_INFO_LOG("flashlight hasn't been enable or" \
				" has already reset to 0 due to timeout\n");
			return ret;
		} else
			return -EINVAL;
	}

	spin_lock_irqsave(&this_fl_str->spin_lock,
						this_fl_str->spinlock_flags);
	if (this_fl_str->mode_status == FL_MODE_FLASH) {
		hrtimer_cancel(&this_fl_str->timer);
		flashlight_turn_off();
	}

	switch (mode) {
	case FL_MODE_OFF:
		flashlight_turn_off();

	break;
	case FL_MODE_TORCH:
		gpio_direction_output(this_fl_str->gpio_torch, 0);
		gpio_set_value(this_fl_str->torch_set1, 1);
		gpio_set_value(this_fl_str->torch_set2, 1);
		gpio_direction_output(this_fl_str->gpio_torch, 1);
		this_fl_str->mode_status = FL_MODE_TORCH;
		this_fl_str->fl_lcdev.brightness = LED_HALF;
	break;

	case FL_MODE_FLASH:
		gpio_direction_output(this_fl_str->gpio_flash, 1);
		this_fl_str->mode_status = FL_MODE_FLASH;
		this_fl_str->fl_lcdev.brightness = LED_FULL;

		hrtimer_start(&this_fl_str->timer,
			ktime_set(this_fl_str->flash_sw_timeout_ms / 1000,
				(this_fl_str->flash_sw_timeout_ms % 1000) *
					NSEC_PER_MSEC), HRTIMER_MODE_REL);
	break;
	case FL_MODE_PRE_FLASH:
		gpio_direction_output(this_fl_str->gpio_torch, 0);
		gpio_set_value(this_fl_str->torch_set1, 1);
		gpio_set_value(this_fl_str->torch_set2, 1);
		gpio_direction_output(this_fl_str->gpio_torch, 1);
		this_fl_str->mode_status = FL_MODE_PRE_FLASH;
		this_fl_str->fl_lcdev.brightness = LED_HALF + 1;
	break;
	case FL_MODE_TORCH_LEVEL_1:
		gpio_direction_output(this_fl_str->gpio_torch, 0);
		gpio_set_value(this_fl_str->torch_set1, 0);
		gpio_set_value(this_fl_str->torch_set2, 0);
		gpio_direction_output(this_fl_str->gpio_torch, 1);
		this_fl_str->mode_status = FL_MODE_TORCH_LEVEL_1;
		this_fl_str->fl_lcdev.brightness = LED_HALF - 2;
	break;
	case FL_MODE_TORCH_LEVEL_2:
		gpio_direction_output(this_fl_str->gpio_torch, 0);
		gpio_set_value(this_fl_str->torch_set1, 0);
		gpio_set_value(this_fl_str->torch_set2, 1);
		gpio_direction_output(this_fl_str->gpio_torch, 1);
		this_fl_str->mode_status = FL_MODE_TORCH_LEVEL_2;
		this_fl_str->fl_lcdev.brightness = LED_HALF - 1;
	break;

	default:
		FLT_ERR_LOG("%s: unknown flash_light flags: %d\n",
							__func__, mode);
		ret = -EINVAL;
	break;
	}

	FLT_INFO_LOG("%s: mode: %d, %u\n", FLASHLIGHT_NAME, mode,
		flash_ns/(1000*1000));

	spin_unlock_irqrestore(&this_fl_str->spin_lock,
						this_fl_str->spinlock_flags);
	return ret;
}

int aat3177_flashlight_control(int mode)
{
	int ret = 0;
	uint32_t flash_ns = ktime_to_ns(ktime_get());

#if 0 /* disable flash_adj_value check now */
	if (this_fl_str->flash_adj_value == 2) {
		printk(KERN_WARNING "%s: force disable function!\n", __func__);
		return -EIO;
	}
#endif
	if (this_fl_str->mode_status == mode) {
		FLT_INFO_LOG("%s: mode is same: %d\n",
							FLASHLIGHT_NAME, mode);

		if (!hrtimer_active(&this_fl_str->timer) &&
			this_fl_str->mode_status == FL_MODE_OFF) {
			FLT_INFO_LOG("flashlight hasn't been enable or" \
				" has already reset to 0 due to timeout\n");
			return ret;
		} else
			return -EINVAL;
	}

	spin_lock_irqsave(&this_fl_str->spin_lock,
						this_fl_str->spinlock_flags);
	if (this_fl_str->mode_status == FL_MODE_FLASH) {
		hrtimer_cancel(&this_fl_str->timer);
		flashlight_turn_off();
	}

	switch (mode) {
	case FL_MODE_OFF:
		flashlight_turn_off();
	break;
	case FL_MODE_TORCH:
		flashlight_aat3177_hw_command(11);
		this_fl_str->mode_status = FL_MODE_TORCH;
		this_fl_str->fl_lcdev.brightness = LED_HALF;
	break;
	case FL_MODE_TORCH_LEVEL_1:
		flashlight_aat3177_hw_command(15);
		this_fl_str->mode_status = FL_MODE_TORCH_LEVEL_1;
		this_fl_str->fl_lcdev.brightness = LED_HALF - 2;
	break;
	case FL_MODE_TORCH_LEVEL_2:
		flashlight_aat3177_hw_command(13);
		this_fl_str->mode_status = FL_MODE_TORCH_LEVEL_2;
		this_fl_str->fl_lcdev.brightness = LED_HALF - 1;
	break;
	case FL_MODE_PRE_FLASH:
		flashlight_aat3177_hw_command(15);
		this_fl_str->mode_status = FL_MODE_PRE_FLASH;
		this_fl_str->fl_lcdev.brightness = LED_HALF + 1;
	break;
	case FL_MODE_FLASH:
		flashlight_aat3177_hw_command(1);
		this_fl_str->mode_status = FL_MODE_FLASH;
		this_fl_str->fl_lcdev.brightness = LED_FULL;
		hrtimer_start(&this_fl_str->timer,
			ktime_set(this_fl_str->flash_sw_timeout_ms / 1000,
				(this_fl_str->flash_sw_timeout_ms % 1000) *
					NSEC_PER_MSEC), HRTIMER_MODE_REL);
	break;
	default:
		FLT_ERR_LOG("%s: unknown flash_light flags: %d\n",
							__func__, mode);
		ret = -EINVAL;
	break;
	}

	FLT_INFO_LOG("%s: mode: %d, %u\n", FLASHLIGHT_NAME, mode,
		flash_ns/(1000*1000));

	spin_unlock_irqrestore(&this_fl_str->spin_lock,
						this_fl_str->spinlock_flags);
	return ret;
}

int aat1271_flashlight_control(int mode)
{
	int ret = 0;
	uint32_t flash_ns = ktime_to_ns(ktime_get());

#if 0 /* disable flash_adj_value check now */
	if (this_fl_str->flash_adj_value == 2) {
		printk(KERN_WARNING "%s: force disable function!\n", __func__);
		return -EIO;
	}
#endif
	if (this_fl_str->mode_status == mode) {
		FLT_INFO_LOG("%s: mode is same: %d\n",
							FLASHLIGHT_NAME, mode);

		if (!hrtimer_active(&this_fl_str->timer) &&
			this_fl_str->mode_status == FL_MODE_OFF) {
			FLT_INFO_LOG("flashlight hasn't been enable or" \
				" has already reset to 0 due to timeout\n");
			return ret;
		}
		else
			return -EINVAL;
	}

	spin_lock_irqsave(&this_fl_str->spin_lock,
						this_fl_str->spinlock_flags);
	if (this_fl_str->mode_status == FL_MODE_FLASH) {
		hrtimer_cancel(&this_fl_str->timer);
		flashlight_turn_off();
	}
	switch (mode) {
	case FL_MODE_OFF:
		flashlight_turn_off();
	break;
	case FL_MODE_TORCH:
		if (this_fl_str->led_count)
			flashlight_hw_command(3, 4);
		else
			flashlight_hw_command(3, 3);
		flashlight_hw_command(0, 6);
		flashlight_hw_command(2, 4);
		this_fl_str->mode_status = FL_MODE_TORCH;
		this_fl_str->fl_lcdev.brightness = LED_HALF;
	break;
	case FL_MODE_TORCH_LED_A:
		flashlight_hw_command(3, 1);
		flashlight_hw_command(0, 15);
		flashlight_hw_command(2, 3);
		this_fl_str->mode_status = FL_MODE_TORCH_LED_A;
		this_fl_str->fl_lcdev.brightness = 1;
	break;
	case FL_MODE_TORCH_LED_B:
		flashlight_hw_command(3, 1);
		flashlight_hw_command(0, 15);
		flashlight_hw_command(2, 2);
		this_fl_str->mode_status = FL_MODE_TORCH_LED_B;
		this_fl_str->fl_lcdev.brightness = 2;
	break;
	case FL_MODE_FLASH: /*750mA*/
		//flashlight_hw_command(2, 4);
		gpio_direction_output(this_fl_str->gpio_flash, 1);
		this_fl_str->mode_status = FL_MODE_FLASH;
		this_fl_str->fl_lcdev.brightness = LED_FULL;
		hrtimer_start(&this_fl_str->timer,
			ktime_set(this_fl_str->flash_sw_timeout_ms / 1000,
				(this_fl_str->flash_sw_timeout_ms % 1000) *
					NSEC_PER_MSEC), HRTIMER_MODE_REL);
	break;
	case FL_MODE_PRE_FLASH:
		flashlight_hw_command(3, 3);
		flashlight_hw_command(0, 6);
		flashlight_hw_command(2, 4);
		this_fl_str->mode_status = FL_MODE_PRE_FLASH;
		this_fl_str->fl_lcdev.brightness = LED_HALF + 1;
	break;
	case FL_MODE_TORCH_LEVEL_1:
		if (this_fl_str->led_count)
			flashlight_hw_command(3, 4);
		else
			flashlight_hw_command(3, 3);
		flashlight_hw_command(0, 15);
		flashlight_hw_command(2, 4);
		this_fl_str->mode_status = FL_MODE_TORCH_LEVEL_1;
		this_fl_str->fl_lcdev.brightness = LED_HALF - 2;
	break;
	case FL_MODE_TORCH_LEVEL_2:
		if (this_fl_str->led_count)
			flashlight_hw_command(3, 4);
		else
			flashlight_hw_command(3, 3);
		flashlight_hw_command(0, 10);
		flashlight_hw_command(2, 4);
		this_fl_str->mode_status = FL_MODE_TORCH_LEVEL_2;
		this_fl_str->fl_lcdev.brightness = LED_HALF - 1;
	break;
	case FL_MODE_FLASH_LEVEL_1: /*200mA*/
		gpio_direction_output(this_fl_str->gpio_flash, 1);
		//flashlight_hw_command(3, 1);
		flashlight_hw_command(0, 12);
		flashlight_hw_command(2, 4);
		this_fl_str->mode_status = FL_MODE_FLASH_LEVEL_1;
		this_fl_str->fl_lcdev.brightness = 3;
		hrtimer_start(&this_fl_str->timer,
			ktime_set(this_fl_str->flash_sw_timeout_ms / 1000,
				(this_fl_str->flash_sw_timeout_ms % 1000) *
					NSEC_PER_MSEC), HRTIMER_MODE_REL);
	break;
	case FL_MODE_FLASH_LEVEL_2: /*300mA*/
		gpio_direction_output(this_fl_str->gpio_flash, 1);
		//flashlight_hw_command(3, 1);
		flashlight_hw_command(0, 9);
		flashlight_hw_command(2, 4);
		this_fl_str->mode_status = FL_MODE_FLASH_LEVEL_2;
		this_fl_str->fl_lcdev.brightness = 4;
		hrtimer_start(&this_fl_str->timer,
			ktime_set(this_fl_str->flash_sw_timeout_ms / 1000,
				(this_fl_str->flash_sw_timeout_ms % 1000) *
					NSEC_PER_MSEC), HRTIMER_MODE_REL);
	break;
	case FL_MODE_FLASH_LEVEL_3: /*400mA*/
		gpio_direction_output(this_fl_str->gpio_flash, 1);
		//flashlight_hw_command(3, 1);
		flashlight_hw_command(0, 6);
		flashlight_hw_command(2, 4);
		this_fl_str->mode_status = FL_MODE_FLASH_LEVEL_3;
		this_fl_str->fl_lcdev.brightness = 5;
		hrtimer_start(&this_fl_str->timer,
			ktime_set(this_fl_str->flash_sw_timeout_ms / 1000,
				(this_fl_str->flash_sw_timeout_ms % 1000) *
					NSEC_PER_MSEC), HRTIMER_MODE_REL);
	break;
	case FL_MODE_FLASH_LEVEL_4: /*600mA*/
		gpio_direction_output(this_fl_str->gpio_flash, 1);
		//flashlight_hw_command(3, 1);
		flashlight_hw_command(0, 3);
		flashlight_hw_command(2, 4);
		this_fl_str->mode_status = FL_MODE_FLASH_LEVEL_4;
		this_fl_str->fl_lcdev.brightness = 6;
		hrtimer_start(&this_fl_str->timer,
			ktime_set(this_fl_str->flash_sw_timeout_ms / 1000,
				(this_fl_str->flash_sw_timeout_ms % 1000) *
					NSEC_PER_MSEC), HRTIMER_MODE_REL);
	break;

	default:
		FLT_ERR_LOG("%s: unknown flash_light flags: %d\n",
							__func__, mode);
		ret = -EINVAL;
	break;
	}

	FLT_INFO_LOG("%s: mode: %d, %u\n", FLASHLIGHT_NAME, mode,
		flash_ns/(1000*1000));

	spin_unlock_irqrestore(&this_fl_str->spin_lock,
						this_fl_str->spinlock_flags);
	return ret;
}

static void fl_lcdev_brightness_set(struct led_classdev *led_cdev,
						enum led_brightness brightness)
{
	struct flashlight_struct *fl_str;
	enum flashlight_mode_flags mode;

	fl_str = container_of(led_cdev, struct flashlight_struct, fl_lcdev);
	if (brightness > 0 && brightness <= LED_HALF) {
		/* Torch mode */
		if (brightness == (LED_HALF - 2))
			mode = FL_MODE_TORCH_LEVEL_1;
		else if (brightness == (LED_HALF - 1))
			mode = FL_MODE_TORCH_LEVEL_2;
		else if (brightness == 1 && fl_str->led_count)
			mode = FL_MODE_TORCH_LED_A;
		else if (brightness == 2 && fl_str->led_count)
			mode = FL_MODE_TORCH_LED_B;
		else if (brightness == 3)
			mode = FL_MODE_FLASH_LEVEL_1;
		else if (brightness == 4)
			mode = FL_MODE_FLASH_LEVEL_2;
		else if (brightness == 5)
			mode = FL_MODE_FLASH_LEVEL_3;
		else if (brightness == 6)
			mode = FL_MODE_FLASH_LEVEL_4;
		else
			mode = FL_MODE_TORCH;
	} else if (brightness > LED_HALF && brightness <= LED_FULL) {
		/* Flashlight mode */
		if (brightness == (LED_HALF + 1))
			mode = FL_MODE_PRE_FLASH; /* pre-flash mode */
		else
			mode = FL_MODE_FLASH;
	} else
		/* off and else */
		mode = FL_MODE_OFF;

	if (fl_str->chip_model == AAT3177)
		aat3177_flashlight_control(mode);
	else if (fl_str->chip_model == AAT1277)
		aat1277_flashlight_control(mode);
	else
		aat1271_flashlight_control(mode);

	return;
}

static void flashlight_early_suspend(struct early_suspend *handler)
{
	struct flashlight_struct *fl_str = container_of(handler,
			struct flashlight_struct, early_suspend_flashlight);

	printk("[FLT] flashlight_early_suspend start\n");
	if (fl_str != NULL && fl_str->mode_status) {
		if (fl_str->mode_status == FL_MODE_FLASH)
			hrtimer_cancel(&fl_str->timer);
		spin_lock_irqsave(&fl_str->spin_lock, fl_str->spinlock_flags);
		flashlight_turn_off();
		spin_unlock_irqrestore(&fl_str->spin_lock,
						fl_str->spinlock_flags);
	}
	printk("[FLT] flashlight_early_suspend end\n");
}

static void flashlight_late_resume(struct early_suspend *handler)
{
	printk("[FLT] flashlight_late_resume start\n");
	/*
	struct flashlight_struct *fl_str = container_of(handler,
			struct flashlight_struct, early_suspend_flashlight);
	*/
	printk("[FLT] flashlight_late_resume end\n");
}

static int flashlight_setup_gpio(struct flashlight_platform_data *flashlight,
					struct flashlight_struct *fl_str)
{
	int ret = 0;
	if (flashlight->gpio_init)
		flashlight->gpio_init();
	// have been config gpio in board-enterprise.c
	if (flashlight->torch) {
		/*
		ret = gpio_request(flashlight->torch, "fl_torch");
		if (ret < 0) {
			FLT_ERR_LOG("%s: gpio_request(torch) failed\n", __func__);
			return ret;
		}
		*/
		fl_str->gpio_torch = flashlight->torch;
	}

	if (flashlight->flash) {
		/*
		ret = gpio_request(flashlight->flash, "fl_flash");
		if (ret < 0) {
			FLT_ERR_LOG("%s: gpio_request(flash) failed\n", __func__);
			return ret;
		}
		*/
		fl_str->gpio_flash = flashlight->flash;
	}
	if (flashlight->torch_set1) {
		ret = gpio_request(flashlight->torch_set1, "fl_torch_set1");
		if (ret < 0) {
			FLT_ERR_LOG("%s: gpio_request(fl_torch_set1) failed\n", __func__);
			return ret;
		}
		fl_str->torch_set1 = flashlight->torch_set1;
	}

	if (flashlight->torch_set2) {
		ret = gpio_request(flashlight->torch_set2, "fl_torch_set2");
		if (ret < 0) {
			FLT_ERR_LOG("%s: gpio_request(fl_torch_set2) failed\n", __func__);
			return ret;
		}
		fl_str->torch_set2 = flashlight->torch_set2;
	}

	if (flashlight->flash_adj) {
		ret = gpio_request(flashlight->flash_adj, "fl_flash_adj");
		if (ret < 0) {
			FLT_ERR_LOG("%s: gpio_request(flash_adj) failed\n", __func__);
			return ret;
		}
		fl_str->gpio_flash_adj = flashlight->flash_adj;
		gpio_set_value(fl_str->gpio_flash_adj, 0);
		fl_str->flash_adj_gpio_status = 0;
		FLT_INFO_LOG("%s: enable flash_adj function\n", FLASHLIGHT_NAME);
	}
	if (flashlight->flash_duration_ms)
		fl_str->flash_sw_timeout_ms = flashlight->flash_duration_ms;
	else /* load default value */
		fl_str->flash_sw_timeout_ms = 600;
	return ret;
}

static int flashlight_free_gpio(struct flashlight_platform_data *flashlight,
					struct flashlight_struct *fl_str)
{
	int ret = 0;
	if (fl_str->gpio_torch) {
		gpio_free(flashlight->torch);
		fl_str->gpio_torch = 0;
	}

	if (fl_str->gpio_flash) {
		gpio_free(flashlight->flash);
		fl_str->gpio_flash = 0;
	}

	if (fl_str->gpio_flash_adj) {
		gpio_free(flashlight->flash_adj);
		fl_str->gpio_flash_adj = 0;
	}

	return ret;
}

#ifdef FLASHLIGHT_ADJ_FUNC
static ssize_t show_flash_adj(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t length;
	length = sprintf(buf, "%d\n", this_fl_str->flash_adj_value);
	return length;
}

static ssize_t store_flash_adj(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	static int tmp, adj_tmp;
	if ((buf[0] == '0' || buf[0] == '1' || buf[0] == '2')
							&& buf[1] == '\n') {
		spin_lock_irqsave(&this_fl_str->spin_lock,
						this_fl_str->spinlock_flags);
		tmp = buf[0] - 0x30;
		if (tmp == this_fl_str->flash_adj_value) {
			spin_unlock_irqrestore(&this_fl_str->spin_lock,
						this_fl_str->spinlock_flags);
			FLT_INFO_LOG("%s: status is same(%d)\n",
				__func__, this_fl_str->flash_adj_value);
			return count;
		}
		adj_tmp = this_fl_str->gpio_flash_adj;
		switch (tmp) {
		case 2:
			flashlight_turn_off();
		break;
		case 1:
			/*
			if (this_fl_str->flash_adj_gpio_status) {
				gpio_set_value(adj_tmp, 0);
				this_fl_str->flash_adj_gpio_status = 0;
			}
			*/
		break;
		case 0:
			/*
			if (!this_fl_str->flash_adj_gpio_status) {
				gpio_set_value(adj_tmp, 1);
				this_fl_str->flash_adj_gpio_status = 1;
			}
			*/
		break;
		}
		this_fl_str->flash_adj_value = tmp;
		spin_unlock_irqrestore(&this_fl_str->spin_lock,
						this_fl_str->spinlock_flags);
	}
	return count;
}

static DEVICE_ATTR(flash_adj, 0666, show_flash_adj, store_flash_adj);
#endif

//A Project flashlight aat1271
static int aat1271_param_wr(int param, uint32_t val)
{
	pr_info("[CAM] flashlight flash/torch param = %d, val = %d\n", param, val);

	switch(param) {
		case nvodmimagerparameter_flashlevel:
		{
			switch (val) {
				case 0:
					return aat1271_flashlight_control(FL_MODE_OFF);
				case 1:
					return aat1271_flashlight_control(FL_MODE_FLASH_LEVEL_1); /*200mA*/
				case 2:
					return aat1271_flashlight_control(FL_MODE_FLASH_LEVEL_2); /*300mA*/
				case 3:
					return aat1271_flashlight_control(FL_MODE_FLASH_LEVEL_3); /*400mA*/
				case 4:
					return aat1271_flashlight_control(FL_MODE_FLASH_LEVEL_4); /*600mA*/
				case 7:
					return aat1271_flashlight_control(FL_MODE_FLASH); /*750mA -default*/
				default:
					return aat1271_flashlight_control(FL_MODE_FLASH);
			}
		}
		case nvodmimagerparameter_torchlevel:
		{
			switch (val) {
				case 0:
					return aat1271_flashlight_control(FL_MODE_OFF);
				default:
					return aat1271_flashlight_control(FL_MODE_TORCH);
			}
		}
		default:
		{
			pr_err("%s: unsupported flash/\n", __func__);
			return -1;
		}
	}

	return 0;
}

static long aat1271_ioctl(struct file *file,
             unsigned int cmd, unsigned long arg)
{
	pr_info("[CAM] flashlight ioctl cmd = %d\n", cmd);

	switch(cmd) {
		case AAT1271_IOCTL_CAP:
		{
			return 0;
		}
		case AAT1271_IOCTL_PWR:
		{
			return 0;
		}
		case AAT1271_IOCTL_PARAM_RD:
		{
			return 0;
		}
		case AAT1271_IOCTL_PARAM_WR:
		{
            struct nvodmimager_param param_data;
			if (copy_from_user(&param_data,
						(const void __user *)arg,
						sizeof(struct nvodmimager_param))) {
				return -EFAULT;
			}

			return aat1271_param_wr(param_data.param, *(uint32_t *)param_data.p_value);
		}
		default:
		{
			pr_err("%s: unsupported ioctl\n", __func__);
			return -1;
		}
	}
	
	return 0;
}

static int aat1271_open(struct inode *inode, struct file *file)
{
	pr_info("[CAM] open flashlight\n");
	return 0;
}

int aat1271_release(struct inode *inode, struct file *file)
{
    pr_info("[CAM] release flashlight\n");
    /* make sure that flashlight is off */
    aat1271_param_wr(nvodmimagerparameter_torchlevel, 0);
    return 0;
}

static const struct file_operations aat1271_fileops = {
    .owner = THIS_MODULE,
    .open = aat1271_open,
    .unlocked_ioctl = aat1271_ioctl,
    .release = aat1271_release,
};

static struct miscdevice aat1271_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "aat1271",
    .fops = &aat1271_fileops,
};

static int flashlight_probe(struct platform_device *pdev)
{
	struct flashlight_platform_data *flashlight = pdev->dev.platform_data;
	struct flashlight_struct *fl_str;
	int err = 0;
	int aat1271_err =0;

	printk("[FLT][PROBE] flashlight driver probe +++\n");
	aat1271_err = misc_register(&aat1271_device);
	if (aat1271_err < 0) {
		FLT_ERR_LOG("%s: failed on misc_register aat1271\n", __func__);
		return aat1271_err;
	}


	fl_str = kzalloc(sizeof(struct flashlight_struct), GFP_KERNEL);
	if (!fl_str) {
		FLT_ERR_LOG("%s: kzalloc fail !!!\n", __func__);
		return -ENOMEM;
	}

	err = flashlight_setup_gpio(flashlight, fl_str);
	if (err < 0) {
		FLT_ERR_LOG("%s: setup GPIO fail !!!\n", __func__);
		goto fail_free_mem;
	}
	spin_lock_init(&fl_str->spin_lock);
	fl_str->chip_model = flashlight->chip_model;
	fl_str->fl_lcdev.name = pdev->name;
	fl_str->fl_lcdev.brightness_set = fl_lcdev_brightness_set;
	fl_str->fl_lcdev.brightness = 0;
	err = led_classdev_register(&pdev->dev, &fl_str->fl_lcdev);
	if (err < 0) {
		FLT_ERR_LOG("%s: failed on led_classdev_register\n", __func__);
		goto fail_free_gpio;
	}
#ifdef FLASHLIGHT_ADJ_FUNC
	if (fl_str->gpio_flash_adj) {
		FLT_INFO_LOG("%s: flash_adj exist, create attr file\n", __func__);
		err = device_create_file(fl_str->fl_lcdev.dev,
							&dev_attr_flash_adj);
		if (err != 0)
			FLT_ERR_LOG("%s: dev_attr_flash_adj failed\n", __func__);
	}
#endif
#ifdef CONFIG_HAS_EARLYSUSPEND
	fl_str->early_suspend_flashlight.suspend = flashlight_early_suspend;
	fl_str->early_suspend_flashlight.resume = flashlight_late_resume;
	register_early_suspend(&fl_str->early_suspend_flashlight);
#endif
	hrtimer_init(&fl_str->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	fl_str->timer.function = flashlight_hrtimer_func;
	fl_str->led_count = flashlight->led_count;
	FLT_INFO_LOG("%s: led_count = %d\n", __func__, fl_str->led_count);

	this_fl_str = fl_str;
	//FLT_INFO_LOG("%s: The Flashlight Driver is ready\n", __func__);
	
	printk("[FLT][PROBE] flashlight driver probe ---, led_count = %d\n", fl_str->led_count+1);
	return 0;

fail_free_gpio:
	flashlight_free_gpio(flashlight, fl_str);
fail_free_mem:
	kfree(fl_str);
	FLT_ERR_LOG("make %s: The Flashlight driver is Failure\n", __func__);
	return err;
}

static int flashlight_remove(struct platform_device *pdev)
{
	struct flashlight_platform_data *flashlight = pdev->dev.platform_data;

	misc_deregister(&aat1271_device);

	flashlight_turn_off();
	hrtimer_cancel(&this_fl_str->timer);
	unregister_early_suspend(&this_fl_str->early_suspend_flashlight);
#ifdef FLASHLIGHT_ADJ_FUNC
	if (this_fl_str->gpio_flash_adj) {
		device_remove_file(this_fl_str->fl_lcdev.dev,
							&dev_attr_flash_adj);
	}
#endif
	led_classdev_unregister(&this_fl_str->fl_lcdev);
	flashlight_free_gpio(flashlight, this_fl_str);

	kfree(this_fl_str);
	return 0;
}

static struct platform_driver flashlight_driver = {
	.probe		= flashlight_probe,
	.remove		= flashlight_remove,
	.driver		= {
		.name		= FLASHLIGHT_NAME,
		.owner		= THIS_MODULE,
	},
};

static int __init flashlight_init(void)
{
	return platform_driver_register(&flashlight_driver);
}

static void __exit flashlight_exit(void)
{
	platform_driver_unregister(&flashlight_driver);
}

module_init(flashlight_init);
module_exit(flashlight_exit);

MODULE_DESCRIPTION("flash light driver");
MODULE_LICENSE("GPL");
