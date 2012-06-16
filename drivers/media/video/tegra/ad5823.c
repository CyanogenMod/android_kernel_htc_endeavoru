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

#include <linux/bma250.h> //HTC_add: get g-sensor info

//HTC_tune
#define POS_LOW (96)
#define POS_HIGH (496)
#define SETTLETIME_MS 55

#if defined(CONFIG_MACH_ENDEAVORU) || defined(CONFIG_MACH_ENDEAVORTD) || defined(CONFIG_MACH_BLUE) || defined(CONFIG_MACH_TEGRA_ENTERPRISE)
#define FOCAL_LENGTH (3.03f)
#define FNUMBER (2.0f)
#endif

#if defined(CONFIG_MACH_VERTEXF) || defined(CONFIG_MACH_QUATTRO_U)
#define FOCAL_LENGTH (2.95f)
#define FNUMBER (2.4f)
#endif

//#define FPOS_COUNT 1024

#define AD5823_MAX_RETRIES (3)

#define VCM_MODE     0x02
#define VCM_CODE_MSB 0x04
#define VCM_CODE_LSB 0x05
#define RING_CTRL    0x1  //enable ringing
#define RING_MODE    0x00 //ARC RES1

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
	u8 msb = 0;

	if (position < info->config.pos_low ||
	    position > info->config.pos_high)
		return -EINVAL;

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
        #if defined(CONFIG_MACH_ENDEAVORU) || defined(CONFIG_MACH_ENDEAVORTD)
        GSensor_set_mode((u8) arg);
        #else
			//pr_err("[CAM] %s: AD5823_IOCTL_SET_GSENSOR_MODE is not supported.", __func__);
        #endif
        break;
    case AD5823_IOCTL_GET_GSENSOR_DATA:
    {
        #if defined(CONFIG_MACH_ENDEAVORU) || defined(CONFIG_MACH_ENDEAVORTD)
        short gsensor_info[3];
        GSensorReadData(gsensor_info);

        struct g_sensor_info data = {
            .x = gsensor_info[0],
            .y = gsensor_info[1],
            .z = gsensor_info[2],
        };
        if (copy_to_user((void __user *) arg,
                 &data,
                 sizeof(struct g_sensor_info))) {
            pr_err("[CAM] AD5823: %s: 0x%x\n", __func__, __LINE__);
            return -EFAULT;
        }
        #else
			//pr_err("[CAM] %s: AD5823_IOCTL_GET_GSENSOR_DATA is not supported.", __func__);
        #endif
        break;
    }
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
	info->config.settle_time = SETTLETIME_MS;
	info->config.focal_length = FOCAL_LENGTH;
	info->config.fnumber = FNUMBER;
	info->config.pos_low = POS_LOW;
	info->config.pos_high = POS_HIGH;
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

