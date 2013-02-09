/* driver/i2c/chip/tpa6185.c
 *
 * tpa6185 Headset Amp
 *
 * Copyright (C) 2012 HTC Corporation
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

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/freezer.h>
#include <linux/i2c/tpa6185.h>
#include <linux/mutex.h>
#include <linux/debugfs.h>
#include <linux/gpio.h>

//htc audio ++
#undef LOG_TAG
#define LOG_TAG "AUD"

#undef AUDIO_DEBUG
#define AUDIO_DEBUG 0

#define AUD_ERR(fmt, ...) pr_tag_err(LOG_TAG, fmt, ##__VA_ARGS__)
#define AUD_INFO(fmt, ...) pr_tag_info(LOG_TAG, fmt, ##__VA_ARGS__)

#if AUDIO_DEBUG
#define AUD_DBG(fmt, ...) pr_tag_info(LOG_TAG, fmt, ##__VA_ARGS__)
#else
#define AUD_DBG(fmt, ...) do { } while (0)
#endif
//htc audio --

#define TPA6185_IOCTL_MAGIC 'b'
#define TPA6185_WRITE_CONFIG	_IOW(TPA6185_IOCTL_MAGIC, 0x01, unsigned int)
#define TPA6185_READ_CONFIG		_IOW(TPA6185_IOCTL_MAGIC, 0x02, unsigned int)


static struct i2c_client *this_client;
static struct tpa6185_platform_data *pdata;
struct mutex headset_amp_lock;
static int tpa6185_opened; 
static int last_headsetamp_state;

static int tpa6185_i2c_write(char *txData, int length);
static int tpa6185_i2c_read(char *rxData, int length);

static unsigned char cf_dsp_bypass[2][2] = {
	{0x01, 0xC0},
	{0x02, 0x37},
};

static unsigned char amp_off[2][2] = {
	{0x01, 0xC0},
	{0x02, 0x00},
};

static int tpa6185_i2c_write(char *txData, int length)
{
	if (this_client == NULL) {
		AUD_ERR("tpa6185_i2c_write client is NULL\n");
		return -EFAULT;
	}

	int rc;
	struct i2c_msg msgs[] = {
		{
			.addr = this_client->addr,
			.flags = 0,
			.len = length,
			.buf = txData,
		},
	};

	rc = i2c_transfer(this_client->adapter, msgs, 1);
	if (rc < 0) {
		AUD_ERR("%s: transfer error %d\n", __func__, rc);
		return rc;
	}

#if AUDIO_DEBUG
	{
		int i = 0;
		for (i = 0; i < length; i++)
			AUD_INFO("%s: tx[%d] = %2x\n", \
				__func__, i, txData[i]);
	}
#endif

	return 0;
}

static int tpa6185_i2c_read(char *rxData, int length)
{
	if (this_client == NULL) {
		AUD_ERR("tpa6185_i2c_read client is NULL\n");
		return -EFAULT;
	}

	int rc;
	struct i2c_msg msgs[] = {
		{
		 .addr = this_client->addr,
		 .flags = I2C_M_RD,
		 .len = length,
		 .buf = rxData,
		},
	};

	rc = i2c_transfer(this_client->adapter, msgs, 1);
	if (rc < 0) {
		AUD_ERR("%s: transfer error %d\n", __func__, rc);
		return rc;
	}

#if AUDIO_DEBUG
	{
		int i = 0;
		for (i = 0; i < length; i++)
			AUD_INFO("i2c_read %s: rx[%d] = %2x\n", __func__, i, \
				rxData[i]);
	}
#endif

	return 0;
}

static int tpa6185_open(struct inode *inode, struct file *file)
{
	int rc = 0;

	mutex_lock(&headset_amp_lock);

	if (tpa6185_opened) {
		AUD_ERR("%s: busy\n", __func__);
		rc = -EBUSY;
		goto done;
	}
	tpa6185_opened = 1;
done:
	mutex_unlock(&headset_amp_lock);
	return rc;
}

static int tpa6185_release(struct inode *inode, struct file *file)
{
	mutex_lock(&headset_amp_lock);
	tpa6185_opened = 0;
	mutex_unlock(&headset_amp_lock);

	return 0;
}

void set_tpa6185_headsetamp(int en, int dsp_mode)
{
	if (this_client == NULL) {
		AUD_ERR("set_tpa6185_headsetamp client is NULL\n");
		return;
	}

	int i = 0;

	AUD_INFO("%s: en = %d dsp_mode = %d\n", __func__, en, dsp_mode);
	mutex_lock(&headset_amp_lock);
	if (en && !last_headsetamp_state) {
		last_headsetamp_state = 1;
		/* DSP Bypass mode */
		if (dsp_mode == 0) {
			for (i = 0; i < 2; i++)
				tpa6185_i2c_write(cf_dsp_bypass[i], 2);
		}
	} else if (!en && last_headsetamp_state) {
		last_headsetamp_state = 0;
		for (i = 0; i < 2; i++)
			tpa6185_i2c_write(amp_off[i], 2);
	}
	mutex_unlock(&headset_amp_lock);
}

static long tpa6185_ioctl(struct file *file, unsigned int cmd,
	   unsigned long arg)
{
	int rc = 0;
	unsigned int reg_value[2];
	unsigned int len = 0;
	char *addr;
	void __user *argp = (void __user *)arg;

	switch (cmd) {
	case TPA6185_WRITE_CONFIG:
		AUD_INFO("%s: TPA6185_WRITE_CONFIG\n", __func__);
		rc = copy_from_user(reg_value, argp, sizeof(reg_value));
		if (rc < 0) {
			AUD_ERR("%s: copy from user failed.\n", __func__);
			goto err;
		}

		len = reg_value[0];
		addr = (char *)reg_value[1];
		tpa6185_i2c_write(addr+1, len -1);
		break;
	case TPA6185_READ_CONFIG:
		AUD_INFO("%s: TPA6185_READ_CONFIG\n", __func__);
		rc = copy_from_user(reg_value, argp, sizeof(reg_value));;
		if (rc < 0) {
			AUD_ERR("%s: copy from user failed.\n", __func__);
			goto err;
		}

		len = reg_value[0];
		addr = (char *)reg_value[1];
		tpa6185_i2c_read(addr, len);

		rc = copy_to_user(argp, reg_value, sizeof(reg_value));
		if (rc < 0) {
			AUD_ERR("%s: copy to user failed.\n", __func__);
			goto err;
		}
		break;
	}
err:
	return rc;
}

static struct file_operations tpa6185_fops = {
	.owner = THIS_MODULE,
	.open = tpa6185_open,
	.release = tpa6185_release,
	.unlocked_ioctl = tpa6185_ioctl,
};

static struct miscdevice tpa6185_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "tpa6185",
	.fops = &tpa6185_fops,
};

int tpa6185_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;

	pdata = client->dev.platform_data;

	if (pdata == NULL) {
		pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
		if (pdata == NULL) {
			ret = -ENOMEM;
			AUD_ERR("%s: platform data is NULL\n", __func__);
			goto err_alloc_data_failed;
		}
	}

	AUD_INFO("tpa6185_probe client %p\n", client);
	this_client = client;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		AUD_ERR("%s: i2c check functionality error\n", __func__);
		ret = -ENODEV;
		goto err_free_gpio_all;
	}

	ret = misc_register(&tpa6185_device);
	if (ret) {
		AUD_ERR("%s: tpa6185_device register failed\n", __func__);
		goto err_free_gpio_all;
	}

	return 0;

err_free_gpio_all:
err_alloc_data_failed:
	return ret;
}

static int tpa6185_remove(struct i2c_client *client)
{
	struct tpa6185_platform_data *p6185data = i2c_get_clientdata(client);
	kfree(p6185data);

	return 0;
}

static int tpa6185_suspend(struct i2c_client *client, pm_message_t mesg)
{
	return 0;
}

static int tpa6185_resume(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id tpa6185_id[] = {
	{ TPA6185_I2C_NAME, 0 },
	{ }
};

static struct i2c_driver tpa6185_driver = {
	.probe = tpa6185_probe,
	.remove = tpa6185_remove,
	.suspend = tpa6185_suspend,
	.resume = tpa6185_resume,
	.id_table = tpa6185_id,
	.driver = {
		.name = TPA6185_I2C_NAME,
	},
};

static int __init tpa6185_init(void)
{
	mutex_init(&headset_amp_lock);
	return i2c_add_driver(&tpa6185_driver);
}

static void __exit tpa6185_exit(void)
{
	i2c_del_driver(&tpa6185_driver);
}

module_init(tpa6185_init);
module_exit(tpa6185_exit);

MODULE_DESCRIPTION("tpa6185 Headset Amp driver");
MODULE_LICENSE("GPL");
