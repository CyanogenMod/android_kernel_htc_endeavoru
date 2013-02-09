/*
 * AD5823 focuser driver.
 *
 * Copyright (C) 2010-2011 NVIDIA Corporation.
 *
 * Contributors:
 *      Sachin Nikam <snikam@nvidia.com>
 *
 * Based on ov5650.c.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <media/ad5823.h>

//#define FPOS_COUNT 1024

#define AD5823_MAX_RETRIES (3)

#define VCM_MODE			0x02
#define VCM_MOVE_TIME		0x03
#define VCM_CODE_MSB		0x04
#define VCM_CODE_LSB		0x05
#define VCM_THRESHOLD_MSB	0x06
#define VCM_THRESHOLD_LSB	0x07

#define RING_CTRL			0x1  //enable ringing
//#define RING_MODE			0x00 //ARC RES1
#define RING_MODE			0x15 //ARC MODE = 0x15 STEP_RATIO = '101', ARC_RES05.
#define VCM_FREQ			0xA4 //67Hz Resonant freq.
#define VCM_CDOE_MValue		0x04 //ARC enable
#define VCM_CDOE_LValue		0x00 //ARC enable
#define VCM_TRSH_MValue		0x00 //Threshold MSB
#define VCM_TRSH_LValue		0xC8 //Threshold LSB

static bool first = 0; // set registers when first write position

struct ad5823_info {
	struct i2c_client *i2c_client;
	struct ad5823_config config;
	struct ad5823_platform_data *pdata;
};

static int ad5823_write(struct i2c_client *client, u8 addr, u32 value)
{
	int count;
	struct i2c_msg msg[1];
	unsigned char data[3];
	int retry = 0;

	if (!client->adapter)
		return -ENODEV;

	data[0] = addr;
	data[1] = (u8)(((value >> 8) & 0x3) | RING_CTRL << 2);
	if (addr == VCM_CODE_MSB){
		data[2] =(u8)(value & 0xFF);
	}

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = addr == VCM_CODE_MSB ? 3 : 2;
	msg[0].buf = data;

	do {
		count = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
		if (count == ARRAY_SIZE(msg))
			return 0;
		retry++;
		pr_err("[CAM] ad5823: i2c transfer failed, retrying addr: %x, value: %x\n",
		       addr, value);
		msleep(3);
	} while (retry <= AD5823_MAX_RETRIES);
	return -EIO;
}

static int ad5823_set_position(struct ad5823_info *info, u32 position)
{
//	u8 msb = 0;

	if (position < info->config.pos_low ||
		position > info->config.pos_high)
		return -EINVAL;

	if(first)
	{
		//owen@Optical open Arc function
		pr_info("[CAM] ad5823 writes initial values\n");
		first = 0;
		ad5823_write(info->i2c_client, VCM_MODE, RING_MODE);
		ad5823_write(info->i2c_client, VCM_MOVE_TIME, VCM_FREQ);
		//ad5823_write(info->i2c_client, VCM_THRESHOLD_MSB,  VCM_TRSH_MValue);
		ad5823_write(info->i2c_client, VCM_THRESHOLD_LSB,  VCM_TRSH_LValue);
	}

	//pr_info("[CAM] ad5823 position: %x\n", position);
	return ad5823_write(info->i2c_client, VCM_CODE_MSB, position);

//	msb = (u8)(((position >> 8) & 0x3) | RING_CTRL << 2);
//	if (ad5823_write(info->i2c_client, VCM_CODE_MSB, msb) == 0){
//  	return ad5823_write(info->i2c_client, VCM_CODE_LSB, (u8)(position & 0xFF));
//	}

//	return -EIO;
}

static long ad5823_ioctl(struct file *file,
			unsigned int cmd, unsigned long arg)
{
	struct ad5823_info *info = file->private_data;
	struct g_sensor_info data;

	switch (cmd) {
	case AD5823_IOCTL_GET_CONFIG:
	{
		if (copy_to_user((void __user *) arg,
				 &info->config,
				 sizeof(info->config))) {
			pr_err("[CAM] AD5823: %s: 0x%x\n", __func__, __LINE__);
			return -EFAULT;
		}

		break;
	}
	case AD5823_IOCTL_SET_POSITION:
		if (info->pdata &&
			info->pdata->get_power_state &&
			(!info->pdata->get_power_state()))
			return -EFAULT;

		return ad5823_set_position(info, (u32) arg);

	case AD5823_IOCTL_GET_POWER_STATE:
	{
		// always return power_on if no such power detect method
		int power_status = 1;
		if (info->pdata && info->pdata->get_power_state)
			power_status = info->pdata->get_power_state();
		if (copy_to_user((void __user *) arg,
				 &power_status,
				 sizeof(power_status))) {
			pr_err("[CAM] AD5823: %s: 0x%x\n", __func__, __LINE__);
			return -EFAULT;
		}
		break;
	}
    //HTC_start: add g-sensor info
    case AD5823_IOCTL_SET_GSENSOR_MODE:
        if (info->pdata && info->pdata->set_gsensor_mode)
	{
		if(info->pdata->set_gsensor_mode((u8) arg) < 0)
		{
			pr_err("[CAM] AD5823: %s: 0x%x\n", __func__, __LINE__);
			return -EFAULT;
		}
	}
        break;
    case AD5823_IOCTL_GET_GSENSOR_DATA:
    {
	if (info->pdata && info->pdata->get_gsensor_data)
	{
		short gsensor_info[3];
		info->pdata->get_gsensor_data(gsensor_info);

		data.x = gsensor_info[0];
		data.y = gsensor_info[1];
		data.z = gsensor_info[2];

		if (copy_to_user((void __user *) arg,
			 &data,
			sizeof(struct g_sensor_info))) {
			pr_err("[CAM] AD5823: %s: 0x%x\n", __func__, __LINE__);
			return -EFAULT;
		}
	}
        break;
    }
	/* set flash light */
	case AD5823_IOCTL_SET_FLASHLIGHT:
	if (info->pdata && info->pdata->set_flashlight)
	{
		if(info->pdata->set_flashlight((int) arg) < 0)
		{
			pr_err("[CAM] AD5823: %s: 0x%x, brightness = %lu\n", __func__, __LINE__, arg);
			return -EFAULT;
		}
	}
	break;
    //HTC_end

	default:
		return -EINVAL;
	}

	return 0;
}

struct ad5823_info *info;

static int ad5823_open(struct inode *inode, struct file *file)
{
	file->private_data = info;
    pr_info("[CAM] ad5823_open::%s\n", __func__);
//	ad5823_write(info->i2c_client, VCM_MODE, RING_MODE); //other bit default are 0
	first = 1;
	return 0;
}

int ad5823_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}


static const struct file_operations ad5823_fileops = {
	.owner = THIS_MODULE,
	.open = ad5823_open,
	.unlocked_ioctl = ad5823_ioctl,
	.release = ad5823_release,
};

static struct miscdevice ad5823_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ad5823",
	.fops = &ad5823_fileops,
};

static int ad5823_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int err;

	pr_info("[CAM] ad5823: probing sensor.\n");

	info = kzalloc(sizeof(struct ad5823_info), GFP_KERNEL);
	if (!info) {
		pr_err("[CAM] ad5823: Unable to allocate memory!\n");
		return -ENOMEM;
	}

	err = misc_register(&ad5823_device);
	if (err) {
		pr_err("[CAM] ad5823: Unable to register misc device!\n");
		kfree(info);
		return err;
	}

	info->pdata = client->dev.platform_data;
	info->i2c_client = client;
	info->config.settle_time = info->pdata->settle_time;
	info->config.focal_length = info->pdata->focal_length;
	info->config.fnumber = info->pdata->fnumber;
	info->config.pos_low = info->pdata->pos_low;
	info->config.pos_high = info->pdata->pos_high;
	i2c_set_clientdata(client, info);
	return 0;
}

static int ad5823_remove(struct i2c_client *client)
{
	struct ad5823_info *info;
	info = i2c_get_clientdata(client);
	misc_deregister(&ad5823_device);
	kfree(info);
	return 0;
}

static const struct i2c_device_id ad5823_id[] = {
	{ "ad5823", 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, ad5823_id);

static struct i2c_driver ad5823_i2c_driver = {
	.driver = {
		.name = "ad5823",
		.owner = THIS_MODULE,
	},
	.probe = ad5823_probe,
	.remove = ad5823_remove,
	.id_table = ad5823_id,
};

static int __init ad5823_init(void)
{
	pr_info("[CAM] ad5823 sensor driver loading\n");
	return i2c_add_driver(&ad5823_i2c_driver);
}

static void __exit ad5823_exit(void)
{
	i2c_del_driver(&ad5823_i2c_driver);
}

module_init(ad5823_init);
module_exit(ad5823_exit);
