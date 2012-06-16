/*
 * s5k6a1gx03.c - s5k6a1gx03xa3 sensor driver
 *
 * Copyright (C) 2011 Google Inc.
 *
 * Contributors:
 *      Rebecca Schultz Zavin <rebecca@android.com>
 *
 * Leverage ov5650.c
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <media/s5k6a1gx03.h>
#if 0
#include <mach/atmega_microp.h>
#endif
enum StereoCameraMode{
       /* Sets the default camera to Main */
       Main,
       /* Sets the stereo camera to stereo mode. */
       Stereo,
       /* Only the sensor on the left is on. */
       LeftOnly,
       /* Only the sensor on the right is on. */
       RightOnly,
       /* Ignore -- Forces compilers to make 32-bit enums. */
       StereoCameraMode_Force32 = 0x7FFFFFFF
};


struct s5k6a1gx03_reg {
	u16 addr;
	u8 val;
};

struct s5k6a1gx03_info {
	int mode;
	enum StereoCameraMode camera_mode;
	struct i2c_client *i2c_client;
	struct s5k6a1gx03_platform_data *pdata;
};

static struct s5k6a1gx03_info *info;
struct i2c_client *s5k6a1_i2c_client;
#define s5k6a1gx03_REG_MODEL_ID 0x0000
#define s5k6a1gx03_MODEL_ID 0x6A10

/* HTC_START */
static int sensor_probe_node = 0;
/* HTC_END */

#define s5k6a1gx03_TABLE_WAIT_MS 0
#define s5k6a1gx03_TABLE_END 1
#define s5k6a1gx03_MAX_RETRIES 3

#if 0//For isp, we don't use it here. It will be ready in nvidia cpu.
static struct s5k6a1gx03_reg tp_none_seq[] = {
	{0x5046, 0x00}, /* isp_off */
	{s5k6a1gx03_TABLE_END, 0x0000}
};

static struct s5k6a1gx03_reg tp_cbars_seq[] = {
	{0x503D, 0xC0},
	{0x503E, 0x00},
	{0x5046, 0x01}, /* isp_on */
	{s5k6a1gx03_TABLE_END, 0x0000}
};

static struct s5k6a1gx03_reg tp_checker_seq[] = {
	{0x503D, 0xC0},
	{0x503E, 0x0A},
	{0x5046, 0x01}, /* isp_on */
	{s5k6a1gx03_TABLE_END, 0x0000}
};

static struct s5k6a1gx03_reg *test_pattern_modes[] = {
	tp_none_seq,
	tp_cbars_seq,
	tp_checker_seq,
};
#endif

static struct s5k6a1gx03_reg reset_seq[] = {
	{0x0100, 0x00},// streaming off	
	{s5k6a1gx03_TABLE_WAIT_MS, 5},
	{0x0103, 0x01},// sw reset	
	{s5k6a1gx03_TABLE_WAIT_MS, 5},// delay 1ms
	{s5k6a1gx03_TABLE_END, 0x00},
};

static struct s5k6a1gx03_reg mode_1296x1040[] = {
	//{0x0344, 0x00},//x_addr_start
	//{0x0345, 0x00},
	//{0x0346, 0x00},//y_addr_start
	//{0x0347, 0x00},	
	//{0x0348, 0x05},//x_addr_end
	//{0x0349, 0x0F},
	//{0x034A, 0x04},//y_addr_end
	//{0x034B, 0x0F},
#if 0
	{0x034C,0x05 },//x_output_size: 0x0500(1280)
	{0x034D,0x00 },
	{0x034E,0x04 },//y_output_size: 0x0400(1024)
	{0x034F,0x00 },
#endif
/*
	{0x0100, 0x00},// streaming off
	{s5k6a1gx03_TABLE_WAIT_MS, 5},
	{0x0103, 0x01},// sw reset
	{s5k6a1gx03_TABLE_WAIT_MS, 5},
	{0x0101, 0x03}, //Vfilp + H mirror
*/
	/* Add i2c register writes to make sure the key registers are set correctly */
	{0x0204, 0x00}, //Analog Gain code
	{0x0205, 0x80}, //Analog Gain code
	{0x0340, 0x04},
	{0x0341, 0x22},
	{0x0342, 0x05},
	{0x0343, 0xCE},

	{0x301C, 0x35}, //APS
	{0x3016, 0x05}, //Analog
	{0x3034, 0x73}, //Analog
	{0x3037, 0x01}, //Analog
	{0x3035, 0x05}, //Analog
	{0x301E, 0x09}, //Analog
	{0x301B, 0xC0}, //Analog
	{0x3013, 0x28}, //Analog
	{0x3042, 0x01}, //Analog
	{0x303C, 0x01}, //Analog

	/* Mclk=24Mhz */
	{0x30BC, 0x38}, /* outif_mld_ulpm_rxinit_limit[15:8] */
	{0x30BD, 0x40}, /* outif_mld_ulpm_rxinit_limit[7:0] */
	{0x3110, 0x70}, /* outif_enable_time[15:8] */
	{0x3111, 0x80}, /* outif_enable_time[7:0] */
	{0x3112, 0x7B}, /* streaming_enable_time[15:8] */
	{0x3113, 0xC0}, /* streaming_enable_time[7:0] */
	{0x30C7, 0x1A}, /* [5:4]esc_ref_div, [3] dphy_ulps_auto, [1]dphy_enable */

	/* PLL */ 		/* For MIPI = 480MHz */
	{0x0305, 0x04}, 	/* pll_p */
	{0x0306, 0x00}, 	/* pll_m */
	{0x0307, 0xA0},
	{0x308D, 0x01}, 	/* pll_s */
	{0x0301, 0x0A}, 	/* vt_pix_clk_div */
	{0x0303, 0x01}, 	/* vt_sys_clk_div */

	{s5k6a1gx03_TABLE_WAIT_MS, 5},
	{0x0100, 0x01}, //stream ON

	{s5k6a1gx03_TABLE_END, 0x00},
};

static struct s5k6a1gx03_reg mode_640x480[] = {
	//{0x0344, 0x00},//x_addr_start
	//{0x0345, 0x00},
	//{0x0346, 0x00},//y_addr_start
	//{0x0347, 0x00},	
	//{0x0348, 0x05},//x_addr_end
	//{0x0349, 0x0F},
	//{0x034A, 0x04},//y_addr_end
	//{0x034B, 0x0F},
	
	{0x3084, 0x00},//1/2_down_scaling_mode	
	{0x034C,0x02 },//x_output_size: 0x0280(640)
	{0x034D,0x80 },
	{0x034E,0x01 },//y_output_size: 0x01E0(480)
	{0x034F,0xE0 },
	
	{0x0100, 0x00},// streaming off
	{0x0103, 0x01},// sw reset
	#if 1
	{0x0101, 0x03}, //Vfilp + H mirror
	#else
	{0x0101, 0x01}, //H mirror
	#endif
	{0x301C, 0x35}, //APS
	{0x3016, 0x05}, //Analog
	{0x3034, 0x73}, //Analog
	{0x3037, 0x01}, //Analog
	{0x3035, 0x05}, //Analog
	{0x301E, 0x09}, //Analog
	{0x301B, 0xC0}, //Analog
	{0x3013, 0x28}, //Analog
	{0x0305, 0x04},	//PLL_P		//04
	{0x0306, 0x00},	//PLL_M MSB	//01
	{0x0307, 0xA0}, //PLL_M LSB	//40

	{s5k6a1gx03_TABLE_WAIT_MS, 5},
	{0x0100, 0x01}, //stream ON

	{s5k6a1gx03_TABLE_END, 0x00},
};

enum {
	s5k6a1gx03_MODE_1296x1040,
	s5k6a1gx03_MODE_640x480,
};

static struct s5k6a1gx03_reg *mode_table[] = {
	[s5k6a1gx03_MODE_1296x1040] = mode_1296x1040,
//	[s5k6a1gx03_MODE_640x480] = mode_640x480,
};

/* 2 regs to program frame length */
static inline void s5k6a1gx03_get_frame_length_regs(struct s5k6a1gx03_reg *regs,
						u32 frame_length)
{
	regs->addr = 0x340;
	regs->val = (frame_length >> 8) & 0xff;
	(regs + 1)->addr = 0x341;
	(regs + 1)->val = (frame_length) & 0xff;
}

/* 3 regs to program coarse time */
static inline void s5k6a1gx03_get_coarse_time_regs(struct s5k6a1gx03_reg *regs,
                                               u32 coarse_time)
{
	regs->addr = 0x202;
	regs->val = (coarse_time >> 8) & 0xff;
	(regs + 1)->addr = 0x203;
	(regs + 1)->val = (coarse_time) & 0xff;
}

/* 1 reg to program gain */
static inline void s5k6a1gx03_get_gain_reg(struct s5k6a1gx03_reg *regs, u16 gain)
{
	regs->addr = 0x204;
	regs->val = (gain >> 8) & 0xff;
	(regs + 1)->addr = 0x205;
	(regs + 1)->val = (gain) & 0xff;
}

static int s5k6a1gx03_write_reg(struct i2c_client *client, u16 addr, u8 val)
{
#if 0
pr_info("s5k6a1gx03_write_reg: addr(0x%4x), data(0x%4x)\n", addr, val);
#endif
	int err;
	struct i2c_msg msg;
	unsigned char data[3];
	int retry = 0;

	if (!client->adapter)
		return -ENODEV;

	data[0] = (u8) (addr >> 8);;
	data[1] = (u8) (addr & 0xff);
	data[2] = (u8) (val & 0xff);

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = 3;
	msg.buf = data;

	do {
		err = i2c_transfer(client->adapter, &msg, 1);
		if (err == 1)
			return 0;
		retry++;
		pr_err("[CAM] s5k6a1gx03: i2c transfer failed, retrying %x %x\n",
		       addr, val);
		msleep(3);
	} while (retry <= s5k6a1gx03_MAX_RETRIES);

	return err;
}

static int s5k6a1gx03_i2c_rxdata(unsigned short saddr,
	unsigned char *rxdata, int length)
{
	struct i2c_msg msgs[] = {
	{
		.addr   = saddr,
		.flags = 0,
		.len   = 2,
		.buf   = rxdata,
	},
	{
		.addr  = saddr,
		.flags = I2C_M_RD,
		.len   = length,
		.buf   = rxdata,
	},
	};

	if (i2c_transfer(s5k6a1_i2c_client->adapter, msgs, 2) < 0) {
		pr_err("[CAM]s5k6a1gx03_i2c_rxdata failed!\n");
		return -EIO;
	}

	return 0;
}

static int32_t s5k6a1gx03_i2c_read(unsigned short raddr,
				unsigned short *rdata, int rlen)
{
	int32_t rc = 0;
	unsigned char buf[2];
	int count = 0;
	if (!rdata)
		return -EIO;

	memset(buf, 0, sizeof(buf));

	buf[0] = (raddr & 0xFF00) >> 8;
	buf[1] = (raddr & 0x00FF);
retry:
	rc = s5k6a1gx03_i2c_rxdata(s5k6a1_i2c_client->addr, buf, rlen);

	if (rc < 0) {
		pr_err("[CAM]s5k6a1gx03_i2c_read 0x%x failed!\n", raddr);
		printk(KERN_ERR "starting read retry policy count:%d\n", count);
		udelay(10);
		count++;
		if (count < 20) {
			if (count > 10)
				udelay(100);
		} else
			return rc;
		goto retry;
	}

	*rdata = (rlen == 2 ? buf[0] << 8 | buf[1] : buf[0]);
	return rc;
}

static int s5k6a1gx03_check_sensorid()
{
	uint16_t chipid = 0;
	int32_t rc = 0;

	/* Read sensor Model ID: */
	rc = s5k6a1gx03_i2c_read(s5k6a1gx03_REG_MODEL_ID, &chipid, 2);
	if (rc < 0) {
		pr_err("[CAM]read sensor id fail\n");
	      rc = EIO;
	}

	/* Compare sensor ID to s5k6a1gx03 ID: */
	pr_info("[CAM]%s, Expected id=0x%x\n", __func__, s5k6a1gx03_MODEL_ID);
	pr_info("[CAM]%s, Read id=0x%x\n", __func__, chipid);

	if (chipid != s5k6a1gx03_MODEL_ID) {
		pr_err("[CAM]sensor model id is incorrect\n");
		rc = -ENODEV;
	}
	return rc;
}
static int s5k6a1gx03_write_reg_helper(struct s5k6a1gx03_info *info,
					u16 addr, u8 val)
{
	int ret;

	ret = s5k6a1gx03_write_reg(info->i2c_client, addr, val);

	if (ret){
		pr_err("[CAM] s5k6a1gx03: Unable to s5k6a1gx03_write_reg => 0x%x!\n", ret);
	}

	return ret;
}

static int s5k6a1gx03_write_table(struct s5k6a1gx03_info *info,
				const struct s5k6a1gx03_reg table[],
				const struct s5k6a1gx03_reg override_list[],
				int num_override_regs)
{
	int err;
	const struct s5k6a1gx03_reg *next;
	int i;
	u16 val;

	for (next = table; next->addr != s5k6a1gx03_TABLE_END; next++) {
		if (next->addr == s5k6a1gx03_TABLE_WAIT_MS) {
			msleep(next->val);
			continue;
		}

		val = next->val;

		/* When an override list is passed in, replace the reg */
		/* value to write if the reg is in the list            */
		if (override_list) {
			for (i = 0; i < num_override_regs; i++) {
				if (next->addr == override_list[i].addr) {
					val = override_list[i].val;
					break;
				}
			}
		}
		err = s5k6a1gx03_write_reg_helper(info, next->addr, val);
		if (err){
			pr_err("[CAM] s5k6a1gx03: Unable to write_reg_helper => 0x%x!\n", err);
			return err;
		}
	}
	return 0;
}
#if 0
void greenLED_on_off(bool on)
{
		int ret;
		uint8_t data = 0x80;
		if (on) {
			ret = microp_i2c_write(MICROP_I2C_WCMD_GPO_LED_STATUS_EN, &data, 1);
			pr_info("[CAM] writing microp add(0x%x), value(0x%x)\n", MICROP_I2C_WCMD_GPO_LED_STATUS_EN, data);
			if (ret != 0)
				pr_err("[CAM] write microp i2c fail(GREEN LED ON/OFF)!add(0x%x), value(0x%x)\n", MICROP_I2C_WCMD_GPO_LED_STATUS_EN, data);
		} else {
			ret = microp_i2c_write(MICROP_I2C_WCMD_GPO_LED_STATUS_DIS, &data, 1);
			pr_info("[CAM] writing microp add(0x%x), value(0x%x)\n", MICROP_I2C_WCMD_GPO_LED_STATUS_DIS, data);
			if (ret != 0)
				pr_err("[CAM] write microp i2c fail(GREEN LED ON/OFF)!%d,%d,%d\n", MICROP_I2C_WCMD_GPO_LED_STATUS_DIS, data);
		}
}
#endif
static int s5k6a1gx03_set_mode(struct s5k6a1gx03_info *info, struct s5k6a1gx03_mode *mode)
{
	int sensor_mode;
	int err;
	struct s5k6a1gx03_reg reg_list[6];

/* HTC_START */
/* get sensor id and check*/
	if(s5k6a1gx03_check_sensorid()!=0)
		return err;
	mdelay(5);
/* HTC_END */

	pr_info("[CAM] %s: xres %u yres %u framelength %u coarsetime %u gain %u\n",
		__func__, mode->xres, mode->yres, mode->frame_length,
		mode->coarse_time, mode->gain);
	if (mode->xres == 1280 && mode->yres == 1024)
		sensor_mode = s5k6a1gx03_MODE_1296x1040;
//	else if (mode->xres == 640 && mode->yres == 480)
//		sensor_mode = s5k6a1gx03_MODE_640x480;
	else {
		pr_err("[CAM] %s: invalid resolution supplied to set mode %d %d\n",
		       __func__, mode->xres, mode->yres);
		return -EINVAL;
	}

	/* get a list of override regs for the asking frame length, */
	/* coarse integration time, and gain.                       */
	s5k6a1gx03_get_frame_length_regs(reg_list, mode->frame_length);
	s5k6a1gx03_get_coarse_time_regs(reg_list + 2, mode->coarse_time);
	s5k6a1gx03_get_gain_reg(reg_list + 4, mode->gain);

	err = s5k6a1gx03_write_table(info, reset_seq, NULL, 0);
	if (err)
		return err;

	err = s5k6a1gx03_write_table(info, mode_table[sensor_mode], reg_list, 6);
	if (err){		
		pr_err("[CAM] s5k6a1gx03: Unable to write_table!\n");
		return err;
	}

	pr_info("[CAM] s5k6a1gx03 stream on!\n");
#if 0
	greenLED_on_off(true);
#endif
	info->mode = sensor_mode;
	return 0;
}
static int s5k6a1gx03_set_frame_length(struct s5k6a1gx03_info *info, u32 frame_length)
{
	struct s5k6a1gx03_reg reg_list[2];
	int i = 0;
	int ret;

	s5k6a1gx03_get_frame_length_regs(reg_list, frame_length);

	for (i = 0; i < 2; i++)	{
		ret = s5k6a1gx03_write_reg(info->i2c_client, reg_list[i].addr,
			reg_list[i].val);
		if (ret)
			return ret;
	}

	return 0;
}

static int s5k6a1gx03_set_coarse_time(struct s5k6a1gx03_info *info, u32 coarse_time)
{
	int ret;

	struct s5k6a1gx03_reg reg_list[2];
	int i = 0;

	s5k6a1gx03_get_coarse_time_regs(reg_list, coarse_time);

	for (i = 0; i < 2; i++)	{
		ret = s5k6a1gx03_write_reg(info->i2c_client, reg_list[i].addr,
			reg_list[i].val);
		if (ret)
			return ret;
	}

	return 0;
}

static int s5k6a1gx03_set_gain(struct s5k6a1gx03_info *info, u16 gain)
{
	int ret;
	struct s5k6a1gx03_reg reg_list[2];
	int i = 0;
    
	s5k6a1gx03_get_gain_reg(reg_list, gain);

	for (i = 0; i < 2; i++)	{
		ret = s5k6a1gx03_write_reg(info->i2c_client, reg_list[i].addr,
			reg_list[i].val);
		if (ret)
			return ret;
	}
	
		return ret;

	return ret;
}

static int s5k6a1gx03_set_group_hold(struct s5k6a1gx03_info *info,
	struct s5k6a1gx03_ae *ae)
{
	int ret;
	int count = 0;
	bool groupHoldEnabled = false;

	if (ae->gain_enable)
		count++;
	if (ae->coarse_time_enable)
		count++;
	if (ae->frame_length_enable)
		count++;
	if ((count >= 2) || ae->coarse_time_enable || ae->frame_length_enable)
		groupHoldEnabled = true;

	if (groupHoldEnabled) {
		ret = s5k6a1gx03_write_reg(info->i2c_client, 0x0104, 0x01);
		if (ret)
			return ret;
	}

	if (ae->gain_enable)
		s5k6a1gx03_set_gain(info, ae->gain);
	if (ae->coarse_time_enable)
		s5k6a1gx03_set_coarse_time(info, ae->coarse_time);
	if (ae->frame_length_enable)
		s5k6a1gx03_set_frame_length(info, ae->frame_length);

	if (groupHoldEnabled) {
		ret = s5k6a1gx03_write_reg(info->i2c_client, 0x0104, 0x0);
		if (ret)
			return ret;
	}

	return 0;
}

#if 0
static int s5k6a1gx03_test_pattern(struct s5k6a1gx03_info *info,
			       enum s5k6a1gx03_test_pattern pattern)
{
	if (pattern >= ARRAY_SIZE(test_pattern_modes))
		return -EINVAL;

	return s5k6a1gx03_write_table(info,
				  test_pattern_modes[pattern],
				  NULL, 0);
}
#endif

static int s5k6a1gx03_set_power(int powerLevel)
{
	pr_info("[CAM] %s: powerLevel=%d \n", __func__, powerLevel);

	if (info->pdata) {
		if (powerLevel && info->pdata->power_on)
			info->pdata->power_on();
		else if (info->pdata->power_off)
			info->pdata->power_off();
	}
	else
		return -1;
	return 0;
}

static long s5k6a1gx03_ioctl(struct file *file,
			 unsigned int cmd, unsigned long arg)
{
	int err;
	struct s5k6a1gx03_info *info = file->private_data;


	switch (cmd) {
	case S5K6A1G_IOCTL_SET_CAMERA_MODE:
	{
		pr_info("[CAM] %s::S5K6A1G_IOCTL_SET_CAMERA_MODE",__FUNCTION__);
		if (info->camera_mode != arg) {
			err = s5k6a1gx03_set_power(0);
			if (err) {
				pr_info("[CAM] %s %d\n", __func__, __LINE__);
				return err;
			}
			info->camera_mode = arg;
			err = s5k6a1gx03_set_power(1);
			if (err)
				return err;
		}
		return 0;
	}
	case S5K6A1G_IOCTL_SYNC_SENSORS:
		pr_info("[CAM] %s::S5K6A1G_IOCTL_SYNC_SENSORS",__FUNCTION__);
		if (info->pdata->synchronize_sensors)
			info->pdata->synchronize_sensors();
		return 0;
	case S5K6A1G_IOCTL_SET_MODE:
	{
		pr_info("[CAM] %s::S5K6A1G_IOCTL_SET_MODE",__FUNCTION__);
		struct s5k6a1gx03_mode mode;
		if (copy_from_user(&mode,
				   (const void __user *)arg,
				   sizeof(struct s5k6a1gx03_mode))) {
			pr_info("[CAM] %s %d\n", __func__, __LINE__);
			return -EFAULT;
		}

		return s5k6a1gx03_set_mode(info, &mode);
	}
	case S5K6A1G_IOCTL_SET_FRAME_LENGTH:
		//pr_info("s5k6a1gx03::%s::S5K6A1G_IOCTL_SET_FRAME_LENGTH",__FUNCTION__);
		return s5k6a1gx03_set_frame_length(info, (u32)arg);
	case S5K6A1G_IOCTL_SET_COARSE_TIME:
		//pr_info("s5k6a1gx03::%s::S5K6A1G_IOCTL_SET_COARSE_TIME",__FUNCTION__);
		return s5k6a1gx03_set_coarse_time(info, (u32)arg);
	case S5K6A1G_IOCTL_SET_GAIN:
		//pr_info("s5k6a1gx03::%s::S5K6A1G_IOCTL_SET_GAIN",__FUNCTION__);
		return s5k6a1gx03_set_gain(info, (u16)arg);
	case S5K6A1G_IOCTL_GET_STATUS:
	{
		//pr_info("s5k6a1gx03::%s::S5K6A1G_IOCTL_GET_STATUS",__FUNCTION__);
		u16 status = 0;
		if (copy_to_user((void __user *)arg, &status, 2)) {
			pr_info("[CAM] %s %d\n", __func__, __LINE__);
			return -EFAULT;
		}
		return 0;
	}
	case S5K6A1G_IOCTL_SET_GROUP_HOLD:
	{
		struct s5k6a1gx03_ae ae;
		if (copy_from_user(&ae,
				(const void __user *)arg,
				sizeof(struct s5k6a1gx03_ae))) {
			pr_info("%s %d\n", __func__, __LINE__);
			return -EFAULT;
		}
		return s5k6a1gx03_set_group_hold(info, &ae);
	}
#if 0    
	case S5K6A1G_IOCTL_TEST_PATTERN:
	{
		err = s5k6a1gx03_test_pattern(info, (enum s5k6a1gx03_test_pattern) arg);
		if (err)
			pr_err("[CAM] %s %d %d\n", __func__, __LINE__, err);
		return err;
	}
#endif    
	default:
		pr_info("s5k6a1gx03::%s::default",__FUNCTION__);
		return -EINVAL;
	}
	return 0;
}

static int s5k6a1gx03_open(struct inode *inode, struct file *file)
{
	pr_info("[CAM] %s\n", __func__);
	file->private_data = info;
	s5k6a1gx03_set_power(1);
	return 0;
}

int s5k6a1gx03_release(struct inode *inode, struct file *file)
{
	pr_info("[CAM] %s\n", __func__);
	s5k6a1gx03_set_power(0);
#if 0
	greenLED_on_off(false);
#endif
	file->private_data = NULL;
	return 0;
}


static const struct file_operations s5k6a1gx03_fileops = {
	.owner = THIS_MODULE,
	.open = s5k6a1gx03_open,
	.unlocked_ioctl = s5k6a1gx03_ioctl,
	.release = s5k6a1gx03_release,
};

static struct miscdevice s5k6a1gx03_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "s5k6a1gx03",
	.fops = &s5k6a1gx03_fileops,
};

/* HTC START */
static const char *S5K6A1GVendor = "samsung";
static const char *S5K6A1GNAME = "s5k6a1g";
static const char *S5K6A1GSize = "1M";
static uint32_t htcwc_value;

static ssize_t sensor_vendor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;

	sprintf(buf, "%s %s %s\n", S5K6A1GVendor, S5K6A1GNAME, S5K6A1GSize);
	ret = strlen(buf) + 1;

	return ret;
}

static ssize_t htcwc_get(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t length;
	length = sprintf(buf, "%d\n", htcwc_value);
	return length;
}

static ssize_t htcwc_set(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	uint32_t tmp = 0;

	tmp = buf[0] - 0x30; /* only get the first char */

#if 0
	if (strcmp(current->comm,"com.android.camera")!=0){
		pr_info("[CAM]No permission : not camera ap\n");
		return -EINVAL;
	}
#endif

	htcwc_value = tmp;
	//pr_info("[CAM]current_comm = %s\n", current->comm);
	pr_info("[CAM]htcwc_value = %d\n", htcwc_value);
	return count;
}

static ssize_t sensor_read_node(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t length;
	length = sprintf(buf, "%d\n", sensor_probe_node);
	return length;
}

static DEVICE_ATTR(sensor, 0444, sensor_vendor_show, NULL);
static DEVICE_ATTR(htcwc, 0664, htcwc_get, htcwc_set);
static DEVICE_ATTR(node, 0444, sensor_read_node, NULL);

static struct kobject *android_s5k6a1g;

static int s5k6a1g_sysfs_init(void)
{
	int ret ;
	pr_info("[CAM]s5k6a1g:kobject creat and add\n");
	android_s5k6a1g = kobject_create_and_add("android_camera2", NULL);
	if (android_s5k6a1g == NULL) {
		pr_info("[CAM]s5k6a1g_sysfs_init: subsystem_register " \
		"failed\n");
		ret = -ENOMEM;
		return ret ;
	}
	pr_info("[CAM]s5k6a1g:sysfs_create_file\n");
	ret = sysfs_create_file(android_s5k6a1g, &dev_attr_sensor.attr);
	if (ret) {
		pr_info("[CAM]s5k6a1g_sysfs_init: sysfs_create_file " \
		"failed\n");
		kobject_del(android_s5k6a1g);
	}

	ret = sysfs_create_file(android_s5k6a1g, &dev_attr_htcwc.attr);
	if (ret) {
		pr_info("[CAM]s5k6a1g_sysfs_init: sysfs_create_file htcwc failed\n");
		kobject_del(android_s5k6a1g);
	}

       ret = sysfs_create_file(android_s5k6a1g, &dev_attr_node.attr);
	if (ret) {
		pr_info("[CAM]s5k6a1g_sysfs_init: dev_attr_node failed\n");
		kobject_del(android_s5k6a1g);
	}

	return 0 ;
}
/* HTC END */
static int s5k6a1gx03_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int err;
	pr_info("[CAM] %s: probing sensor.\n", __func__);

	if (!info) {
		info = kzalloc(sizeof(struct s5k6a1gx03_info), GFP_KERNEL);
		if (!info) {
			pr_err("[CAM] s5k6a1gx03: Unable to allocate memory!\n");
			return -ENOMEM;
		}
	}

	err = misc_register(&s5k6a1gx03_device);
	if (err) {
		pr_err("[CAM] s5k6a1gx03: Unable to register misc device!\n");
		kfree(info);
		return err;
	}

	info->pdata = client->dev.platform_data;
	info->i2c_client = client;

	/* HTC START */
	s5k6a1g_sysfs_init();
	/* HTC END */
	s5k6a1_i2c_client = client;
	return 0;
}

static int s5k6a1gx03_remove(struct i2c_client *client)
{
	pr_info("[CAM] %s:\n",__func__);
	misc_deregister(&s5k6a1gx03_device);
	kfree(info);
	return 0;
}

static const struct i2c_device_id s5k6a1gx03_id[] = {
	{ "s5k6a1gx03", 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, s5k6a1gx03_id);

static struct i2c_driver s5k6a1gx03_i2c_driver = {
	.driver = {
		.name = "s5k6a1gx03",
		.owner = THIS_MODULE,
	},
	.probe = s5k6a1gx03_probe,
	.remove = s5k6a1gx03_remove,
	.id_table = s5k6a1gx03_id,
};

static int __init s5k6a1gx03_init(void)
{
	int ret;
	pr_info("[CAM] s5k6a1gx03 sensor driver loading. %s\n", __func__);
	ret = i2c_add_driver(&s5k6a1gx03_i2c_driver);
	if (ret)
    	{
        pr_info("[CAM] fc99: i2c_add_driver returned %d\n", ret);
		return ret;
    	}
	return ret;
}

static void __exit s5k6a1gx03_exit(void)
{
	i2c_del_driver(&s5k6a1gx03_i2c_driver);
}

module_init(s5k6a1gx03_init);
module_exit(s5k6a1gx03_exit);




