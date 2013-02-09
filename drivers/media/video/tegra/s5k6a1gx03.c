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
#include <linux/clk.h>
#include <media/s5k6a1gx03.h>
#include <media/rawchip/Yushan_HTC_Functions.h>
#include <mach/board_htc.h>

#include <linux/clk.h>
#include <../arch/arm/mach-tegra/clock.h>

//define use 1296x1040
#define A2_USE_1296_1040 0

#define SIZEOF_I2C_TRANSBUF 32


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

enum {
	s5k6a1gx03_MODE_1296x1040,
	s5k6a2_MODE_1472x1104,
};

struct s5k6a1gx03_reg {
	u16 addr;
	u8 val;
};

struct s5k6a1gx03_info {
	int mode;
	enum StereoCameraMode camera_mode;
	struct work_struct set_mode_work;
	struct mutex camera_lock;
	struct i2c_client *i2c_client;
	struct s5k6a1gx03_platform_data *pdata;
	u8 i2c_trans_buf[SIZEOF_I2C_TRANSBUF];
	bool power_state;
};

static struct s5k6a1gx03_info *info;
struct i2c_client *s5k6a1_i2c_client;
#define s5k6a1gx03_REG_MODEL_ID 0x0000
#define s5k6a1gx03_MODEL_ID 0x6A10
#define s5k6a2_MODEL_ID     0x6A20

static u16 coarse_time_reg;
static u16 group_hold_reg;
#define s5k6a1gx03_REG_COARSE_TIME 0x202
#define s5k6a2_REG_COARSE_TIME 0x222
#define s5k6a1gx03_REG_GROUP_HOLD 0x0104
#define s5k6a2_REG_GROUP_HOLD 0x3E90

/* HTC_START */
static bool_t sensor_source = SENSOR_S5K6A2; /* set s5k6a2 as default */
static int sensor_probe_node = 0;
static bool clock_init_done = 0;
/* HTC_END */

#define s5k6a1gx03_TABLE_WAIT_MS 0
#define s5k6a1gx03_TABLE_END 1
#define s5k6a1gx03_MAX_RETRIES 3

static struct rawchip_sensor_data rawchip_mode_table[] =
{
	[s5k6a1gx03_MODE_1296x1040] =
	{	/* 1.3M 1296x1040 */
		.datatype = 0x2B,
		.pixel_clk = 48000000,//.bitrate = 480,
		.width = 1296,
		.height = 1040,
		.line_length_pclk = 1486,
		.frame_length_lines = 1076,
		.x_addr_start = 0,
		.y_addr_start = 0,
		.x_addr_end = 1296 -1,
		.y_addr_end = 1040 -1,
		.x_even_inc = 1,
		.x_odd_inc = 1,
		.y_even_inc = 1,
		.y_odd_inc = 1,
		.binning_rawchip = 0x11,
	},
	[s5k6a2_MODE_1472x1104] =
	{	/* 1.6M 1472x1104 */
		.datatype = 0x2B,
		.pixel_clk = 62000000,//.bitrate = 620,
#if A2_USE_1296_1040
		.width = 1296,
		.height = 1040,
		.line_length_pclk = 1662,
		.frame_length_lines = 1238,
#else
		.width = 1472,
		.height = 1104,
		.line_length_pclk = 1662,
		.frame_length_lines = 1238,
#endif
#if A2_USE_1296_1040
		.x_addr_start = 0,
		.y_addr_start = 0,
		.x_addr_end = 1296 -1,
		.y_addr_end = 1040 -1,
		.x_even_inc = 1,
		.x_odd_inc = 1,
		.y_even_inc = 1,
		.y_odd_inc = 1,
		.binning_rawchip = 0x11,
#else
		.x_addr_start = 0,
		.y_addr_start = 0,
		.x_addr_end = 1472 -1,
		.y_addr_end = 1104 -1,
		.x_even_inc = 1,
		.x_odd_inc = 1,
		.y_even_inc = 1,
		.y_odd_inc = 1,
		.binning_rawchip = 0x11,
#endif
	},
};

static struct s5k6a1gx03_reg reset_seq[] = {
	{0x0100, 0x00},// streaming off
	{s5k6a1gx03_TABLE_WAIT_MS, 5},
	{0x0103, 0x01},// sw reset
	{s5k6a1gx03_TABLE_WAIT_MS, 5},// delay 1ms
	{s5k6a1gx03_TABLE_END, 0x00},
};

static struct s5k6a1gx03_reg mode_1296x1040[] = {
#if defined(CONFIG_MACH_OPERAUL)
	{0x0101, 0x03}, /* V filp + H mirror */
#endif
	/* Add i2c register writes to make sure the key registers are set correctly */
	{0x0204, 0x00}, //Analog Gain code
	{0x0205, 0x20}, //Analog Gain code
	{0x0340, 0x04}, /* Frame_length_lines 0x0434 = 1076 */
	{0x0341, 0x34}, /* 02120607, steven modified: 0x22 -> 0x34 */
	{0x0342, 0x05}, /* Line_length_pck = 0x05CE = 1486 */
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

static struct s5k6a1gx03_reg mode_1472x1104[] = {
#if defined(CONFIG_MACH_OPERAUL)
	{0x0101, 0x03}, /* V filp + H mirror */
#endif
	/* analog setting */
	/* These registers are for Factory use only. */
	/* You don¡¯t have to change these registers arbitrarily */
	{ 0x303E, 0x20 }, // VPIX
	{ 0x303F, 0x10 }, // VTG
	{ 0x3040, 0x40 }, // NTG
	{ 0x3041, 0x10 }, // VRG
	{ 0x3310, 0x0C }, // [3:2] nop_flob : must be written before streaming on
	{ 0x3074, 0x0E }, // [3] f_lob_read_opt : must be written before streaming on
	{ 0x3017, 0x01 }, // cds_option for CFPN
	{ 0x3E33, 0x3C }, // ts_x
	{ 0x3029, 0x0E }, // tmc_gain
	{ 0x300D, 0x14 }, // ct_rmp_rst_start
	{ 0x300E, 0x8E }, // ct_rmp_sig_start
	{ 0x301B, 0x08 }, // off_rst
	{ 0x305B, 0x9C }, // adc_clp
	{ 0x3315, 0x5B }, // adlc
	{ 0x3148, 0x00 }, // misc.
	{ 0x3149, 0x00 }, // misc.

	/* External clock target frequency 	{ 0xetti, 0xng },*/
	{ 0x3E84, 0x18 }, // External clock frequency : 24MHz
	{ 0x3E85, 0x00 },

	/* PLL 	{ 0xetti, 0xng },, Raw10, Frame rate 30.138fps */
	/* MPLL : 620MHz */
	{ 0x0820, 0x06 }, // M_PLL_P	24MHz / 6 = 4MHz
	{ 0x0821, 0x00 }, // M_PLL_M MSB
	{ 0x0822, 0x9B }, // M_PLL_M LSB	4MHz x 155 = 620MHz
	{ 0x0823, 0x00 }, // M_PLL_S	732MHz / 2^0 = 620MHz

	/* Sy	{ 0xtem , 0xcl },ock divider setting */
	{ 0x082A, 0x0A }, // DIV_SYS
	{ 0x082B, 0x06 }, // DIV_DBR

	/* MIPI target frequency 	{ 0xetti, 0xng }, : 620.00MHz */
	{ 0x0858, 0x02 }, // serial_clock_rate_mbps[31:24]
	{ 0x0859, 0x6C }, // serial_clock_rate_mbps[23:16]
	{ 0x085A, 0x00 }, // serial_clock_rate_mbps[15: 8]
	{ 0x085B, 0x00 }, // serial_clock_rate_mbps[ 7: 0]

	/* Frame rate 	{ 0xetti, 0xng }, */
	{ 0x0340, 0x04 }, // frame_length_lines_A MSB
	{ 0x0341, 0xD6 }, // frame_length_lines_A LSB : 1238d
	{ 0x0342, 0x06 }, // line_length_pck_A MSB
	{ 0x0343, 0x7E }, // line_length_pck_A LSB : 1662d

	/* Image 	{ 0xize , 0xse },tting : Full (1472x1104) */
#if A2_USE_1296_1040
	{ 0x0344, 0x00 }, // x_addr_start_A MSB
	{ 0x0345, 0x58 }, // x_addr_start_A LSB : 0d
	{ 0x0346, 0x00 }, // y_addr_start_A MSB
	{ 0x0347, 0x20 }, // y_addr_start_A LSB : 0d
	{ 0x0348, 0x05 }, // x_addr_end_A MSB
	{ 0x0349, 0x67 }, // x_addr_end_A LSB : 1471d
	{ 0x034A, 0x04 }, // y_addr_end_A MSB
	{ 0x034B, 0x2F }, // y_addr_end_A LSB : 1103d
#else
	{ 0x0344, 0x00 }, // x_addr_start_A MSB
	{ 0x0345, 0x00 }, // x_addr_start_A LSB : 0d
	{ 0x0346, 0x00 }, // y_addr_start_A MSB
	{ 0x0347, 0x00 }, // y_addr_start_A LSB : 0d
	{ 0x0348, 0x05 }, // x_addr_end_A MSB
	{ 0x0349, 0xBF }, // x_addr_end_A LSB : 1471d
	{ 0x034A, 0x04 }, // y_addr_end_A MSB
	{ 0x034B, 0x4F }, // y_addr_end_A LSB : 1103d
#endif

	/* Image output 	{ 0xize , 0xse },tting */
#if A2_USE_1296_1040
	{ 0x034C, 0x05 }, // x_output_size_A MSB
	{ 0x034D, 0x10 }, // x_output_size_A LSB : 1472d
	{ 0x034E, 0x04 }, // y_output_size_A MSB
	{ 0x034F, 0x10 }, // y_output_size_A LSB : 1104d
#else
	{ 0x034C, 0x05 }, // x_output_size_A MSB
	{ 0x034D, 0xC0 }, // x_output_size_A LSB : 1472d
	{ 0x034E, 0x04 }, // y_output_size_A MSB
	{ 0x034F, 0x50 }, // y_output_size_A LSB : 1104d
#endif

	/* Sub-	{ 0xampl, 0xin },g setting */
	{ 0x0381, 0x01 }, // x_even_inc_A
	{ 0x0383, 0x01 }, // x_odd_inc_A
	{ 0x0385, 0x01 }, // y_even_inc_A
	{ 0x0387, 0x01 }, // y_odd_inc_A

	/* Analog gain 	{ 0xetti, 0xng }, */
	{ 0x0204, 0x00 }, // analogue_gain_code_global_A MSB
	{ 0x0205, 0x20 }, // analogue_gain_code_global_A LSB : 32d

	/* Shutter 	{ 0xetti, 0xng }, */
	{ 0x0220, 0x01 }, //cintc
	{ 0x0221, 0xF4 },
	{ 0x0222, 0x02 }, //cintr
	{ 0x0223, 0x6F },

	/* Mirror & flip 	{ 0xetti, 0xng }, */
	{ 0x0216, 0x00 }, // image_orientation_A

	/* Analog binning 	{ 0xetti, 0xng }, */
	{ 0x0800, 0x00 }, // binning mode_A

	{s5k6a1gx03_TABLE_WAIT_MS, 5},
	/* Streaming 	{ 0xetti, 0xng }, */
	{ 0x0100, 0x01 }, // streaming on

	{ s5k6a1gx03_TABLE_END, 0x00 },
};

static struct s5k6a1gx03_reg *mode_table[] = {
	[s5k6a1gx03_MODE_1296x1040] = mode_1296x1040,
	[s5k6a2_MODE_1472x1104] = mode_1472x1104,
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
	regs->addr = coarse_time_reg;
	regs->val = (coarse_time >> 8) & 0xff;
	(regs + 1)->addr = coarse_time_reg + 1;
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

static int32_t s5k6a1gx03_i2c_read_simple(unsigned short raddr,
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
		pr_err("[CAM] %s 0x%x failed!\n", __func__, raddr);
		printk(KERN_ERR "starting read retry policy count:%d\n", count);
		if (count++ < 3) {
			udelay(10);
		} else
			return rc;
		goto retry;
	}

	*rdata = (rlen == 2 ? buf[0] << 8 | buf[1] : buf[0]);
	return rc;
}

static int s5k6a1gx03_check_sensorid(void)
{
	uint16_t chipid = 0;
	int32_t except_id = 0;
	int32_t rc = 0;

	/* Read sensor Model ID: */
	rc = s5k6a1gx03_i2c_read(s5k6a1gx03_REG_MODEL_ID, &chipid, 2);
	if (rc < 0) {
		pr_err("[CAM]read sensor id fail\n");
	      rc = EIO;
	}

	/* Compare sensor ID to s5k6a1gx03 ID: */
	except_id = (sensor_source == SENSOR_S5K6A1 ? s5k6a1gx03_MODEL_ID : s5k6a2_MODEL_ID);
	pr_debug("[CAM]%s, Expected id=0x%x\n", __func__, except_id);
	pr_debug("[CAM]%s, Read id=0x%x\n", __func__, chipid);

	if (chipid != except_id) {
		pr_err("[CAM]sensor model id is incorrect\n");
		rc = -ENODEV;
	}
	return rc;
}

static int s5k6a1gx03_write_bulk_reg(struct i2c_client *client, u8 *data, int len)
{
	int err;
	struct i2c_msg msg;

	if (!client->adapter)
		return -ENODEV;

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = len;
	msg.buf = data;

	err = i2c_transfer(client->adapter, &msg, 1);
	if (err == 1)
		return 0;

	pr_err("[CAM] s5k6a1gx03: i2c transfer failed, retrying %x\n",
		(int)data[0] << 8 | data[1]);

	return err;
}

static int s5k6a1gx03_write_table(struct s5k6a1gx03_info *info,
				const struct s5k6a1gx03_reg table[],
				const struct s5k6a1gx03_reg override_list[],
				int num_override_regs)
{
	int err;
	const struct s5k6a1gx03_reg *next, *n_next;
	u8 *b_ptr = info->i2c_trans_buf;
	unsigned int buf_filled = 0;
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

		if (!buf_filled) {
			b_ptr = info->i2c_trans_buf;
			*b_ptr++ = next->addr >> 8;
			*b_ptr++ = next->addr & 0xff;
			buf_filled = 2;
		}
		*b_ptr++ = val;
		buf_filled++;

		n_next = next + 1;
		if (n_next->addr != s5k6a1gx03_TABLE_END &&
			n_next->addr != s5k6a1gx03_TABLE_WAIT_MS &&
			buf_filled < SIZEOF_I2C_TRANSBUF &&
			n_next->addr == next->addr + 1) {
			continue;
		}

		err =  s5k6a1gx03_write_bulk_reg(info->i2c_client, info->i2c_trans_buf, buf_filled);
		if (err){
			pr_err("[CAM] s5k6a1gx03: Unable to write_reg_helper => 0x%x!\n", err);
			return err;
		}

		buf_filled = 0;
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
	int err = -1;
	struct s5k6a1gx03_reg reg_list[6];
	struct rawchip_sensor_data rawchip_data;
	int retry = 0;
        struct clk *emc_clk = NULL;

	if (mode) {
		pr_info("[CAM] %s: xres %u yres %u framelength %u coarsetime %u gain %u\n",
			__func__, mode->xres, mode->yres, mode->frame_length,
			mode->coarse_time, mode->gain);
#if A2_USE_1296_1040
		if (mode->xres == 1280 && mode->yres == 1024 && sensor_source == SENSOR_S5K6A2)
#else
		if (mode->xres == 1456 && mode->yres == 1088)
#endif
		{
			pr_info("[CAM] set mode to -> s5k6a2_MODE_1472x1104\n");
			sensor_mode = s5k6a2_MODE_1472x1104;
		}
		else if (mode->xres == 1280 && mode->yres == 1024)
		{
			pr_info("[CAM] set mode to -> s5k6a1gx03_MODE_1296x1040\n");
			sensor_mode = s5k6a1gx03_MODE_1296x1040;
		}
		else {
			pr_err("[CAM] %s: invalid resolution supplied to set mode %d %d\n",
			       __func__, mode->xres, mode->yres);
			return -EINVAL;
		}

		if(sensor_mode == info->mode)
		{
			pr_info("[CAM] skip set_mode\n");
			return 0;
		}
	}else {
		if(sensor_source == SENSOR_S5K6A2)
			sensor_mode = s5k6a2_MODE_1472x1104;
		else
			sensor_mode = s5k6a1gx03_MODE_1296x1040;
	}

retry:
	/* get a list of override regs for the asking frame length, */
	/* coarse integration time, and gain.                       */
	if (mode) {
		s5k6a1gx03_get_frame_length_regs(reg_list, mode->frame_length);
		s5k6a1gx03_get_coarse_time_regs(reg_list + 2, mode->coarse_time);
		s5k6a1gx03_get_gain_reg(reg_list + 4, mode->gain);
	}

	err = s5k6a1gx03_write_table(info, reset_seq, NULL, 0);
	if (err)
		return err;

	if ( info->pdata->use_rawchip ) {
		/* do rawchip setting before sensor setting */
		pr_info("[CAM] call rawchip_set_size(%d) +++", sensor_mode);

		emc_clk = tegra_get_clock_by_name("camera.emc");
		if (IS_ERR_OR_NULL(emc_clk)) {
			pr_err("[CAM] %s: couldn't get emc clock\n", __func__);
			emc_clk = NULL;
		}
		else
		{
			clk_set_rate(emc_clk, 230000000);
			pr_info("[CAM] set emc clock back to 533MHz");
		}
		rawchip_data.sensor_name = info->pdata->sensor_name;
		rawchip_data.datatype = rawchip_mode_table[sensor_mode].datatype;
		rawchip_data.lane_cnt = info->pdata->data_lane;
		rawchip_data.pixel_clk = rawchip_mode_table[sensor_mode].pixel_clk;
		rawchip_data.mirror_flip = info->pdata->mirror_flip;
		rawchip_data.width = rawchip_mode_table[sensor_mode].width;
		rawchip_data.height = rawchip_mode_table[sensor_mode].height;
		rawchip_data.line_length_pclk = rawchip_mode_table[sensor_mode].line_length_pclk;
		rawchip_data.frame_length_lines = rawchip_mode_table[sensor_mode].frame_length_lines;
		rawchip_data.x_addr_start = rawchip_mode_table[sensor_mode].x_addr_start;
		rawchip_data.y_addr_start = rawchip_mode_table[sensor_mode].y_addr_start;
		rawchip_data.x_addr_end = rawchip_mode_table[sensor_mode].x_addr_end;
		rawchip_data.y_addr_end = rawchip_mode_table[sensor_mode].y_addr_end;
		rawchip_data.x_even_inc = rawchip_mode_table[sensor_mode].x_even_inc;
		rawchip_data.x_odd_inc = rawchip_mode_table[sensor_mode].x_odd_inc;
		rawchip_data.y_even_inc = rawchip_mode_table[sensor_mode].y_even_inc;
		rawchip_data.y_odd_inc = rawchip_mode_table[sensor_mode].y_odd_inc;
		rawchip_data.binning_rawchip = rawchip_mode_table[sensor_mode].binning_rawchip;
		/* override for 6A1 for backword compat of old devices, 6A1 is not using correct calibration data, thus bypass */
		rawchip_data.use_rawchip = (sensor_source == SENSOR_S5K6A1 ? RAWCHIP_MIPI_BYPASS : info->pdata->use_rawchip);
		err = rawchip_set_size(rawchip_data, &clock_init_done);
		if (err < 0 && retry++ < 5) {
			pr_err("[CAM] rawchip_set_size FAIL!!!");
			//s5k6a1gx03_reset(info); TODO
			goto retry;
		}
	}

	if (mode)
		err = s5k6a1gx03_write_table(info, mode_table[sensor_mode], reg_list, 6);
	else
		err = s5k6a1gx03_write_table(info, mode_table[sensor_mode], NULL, 0);

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

	/* Maverick: to prevent analog gain less than 1x, 20121105 */
	if ( gain < 32 )
		gain = 32;

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

	bool groupHoldEnabled = ae->gain_enable | ae->coarse_time_enable | ae->frame_length_enable;

	if (groupHoldEnabled) {
		ret = s5k6a1gx03_write_reg(info->i2c_client, group_hold_reg, 0x01);
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
		ret = s5k6a1gx03_write_reg(info->i2c_client, group_hold_reg, 0x0);
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
		if (powerLevel && info->pdata->power_on) {
			info->pdata->power_on();
			if ( info->pdata->use_rawchip )
				rawchip_open_init();
		}
		else if (info->pdata->power_off) {
			if ( info->pdata->use_rawchip )
				rawchip_release();
			info->pdata->power_off();
		}
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
	struct s5k6a1gx03_mode mode;
	struct s5k6a1gx03_ae ae;

	while (!info->power_state){
		pr_debug("[CAM] %s power is not on, let's wait\n", __func__);
		msleep(5);
	};

	switch (cmd) {
	case S5K6A1G_IOCTL_SET_MODE:
	{
		pr_info("[CAM] %s::S5K6A1G_IOCTL_SET_MODE",__FUNCTION__);
		if (copy_from_user(&mode,
				   (const void __user *)arg,
				   sizeof(struct s5k6a1gx03_mode))) {
			pr_info("[CAM] %s %d\n", __func__, __LINE__);
			return -EFAULT;
		}

		mutex_lock(&info->camera_lock);
		err = s5k6a1gx03_set_mode(info, &mode);
		mutex_unlock(&info->camera_lock);
		return err;
	}
	case S5K6A1G_IOCTL_SET_FRAME_LENGTH:
		//pr_info("s5k6a1gx03::%s::S5K6A1G_IOCTL_SET_FRAME_LENGTH",__FUNCTION__);
		mutex_lock(&info->camera_lock);
		err = s5k6a1gx03_set_frame_length(info, (u32)arg);
		mutex_unlock(&info->camera_lock);
		return err;
	case S5K6A1G_IOCTL_SET_COARSE_TIME:
		//pr_info("s5k6a1gx03::%s::S5K6A1G_IOCTL_SET_COARSE_TIME",__FUNCTION__);
		mutex_lock(&info->camera_lock);
		err = s5k6a1gx03_set_coarse_time(info, (u32)arg);
		mutex_unlock(&info->camera_lock);
		return err;
	case S5K6A1G_IOCTL_SET_GAIN:
		//pr_info("s5k6a1gx03::%s::S5K6A1G_IOCTL_SET_GAIN",__FUNCTION__);
		mutex_lock(&info->camera_lock);
		err = s5k6a1gx03_set_gain(info, (u16)arg);
		mutex_unlock(&info->camera_lock);
		return err;
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
	case S5K6A1G_IOCTL_GET_INIT_STATE:
	{
		struct s5k6a1gx03_init_state init_state;
		init_state.use_rawchip = info->pdata->use_rawchip;
		if (copy_to_user((void __user *)arg, &init_state,
					sizeof(struct s5k6a1gx03_init_state))) {
			pr_info("[CAM] %s %d\n", __func__, __LINE__);
			return -EFAULT;
		}
		return 0;
	}
	case S5K6A1G_IOCTL_SET_GROUP_HOLD:
	{
		if (copy_from_user(&ae,
				(const void __user *)arg,
				sizeof(struct s5k6a1gx03_ae))) {
			pr_info("[CAM] %s %d\n", __func__, __LINE__);
			return -EFAULT;
		}
		mutex_lock(&info->camera_lock);
		err = s5k6a1gx03_set_group_hold(info, &ae);
		mutex_unlock(&info->camera_lock);
		return err;
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
		pr_info("[CAM] s5k6a1gx03::%s::default",__FUNCTION__);
		return -EINVAL;
	}
	return 0;
}

static int s5k6a1gx03_open(struct inode *inode, struct file *file)
{
	pr_info("[CAM] %s ++\n", __func__);

	mutex_lock(&info->camera_lock);

	file->private_data = info;

	info->power_state = false;
	info->mode = -1;

	pr_info("[CAM] %s schedule_work(set_mode_work)\n", __func__);
	schedule_work(&info->set_mode_work);

	mutex_unlock(&info->camera_lock);
	pr_info("[CAM] %s --\n", __func__);

	return 0;
}

int s5k6a1gx03_release(struct inode *inode, struct file *file)
{
	pr_info("[CAM] %s ++\n", __func__);

	mutex_lock(&info->camera_lock); /* use camera_lock to prevent  releasing camera when set_mode */

	s5k6a1gx03_set_power(0);
#if 0
	greenLED_on_off(false);
#endif
	file->private_data = NULL;

	info->power_state = false;

	pr_info("[CAM] %s cancel_work_sync(set_mode_work)\n", __func__);
	cancel_work_sync(&info->set_mode_work);

	mutex_unlock(&info->camera_lock);

	pr_info("[CAM] %s --\n", __func__);
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
static const char *S5K6A1GSize = "1.3M";
static const char *S5K6A2Vendor = "samsung";
static const char *S5K6A2NAME = "s5k6a2";
static const char *S5K6A2Size = "1.6M";
static uint32_t htcwc_value;

static ssize_t sensor_vendor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;

	if (sensor_source == SENSOR_S5K6A1) {
		sprintf(buf, "%s %s %s\n", S5K6A1GVendor, S5K6A1GNAME, S5K6A1GSize);
	} else {
		sprintf(buf, "%s %s %s\n", S5K6A2Vendor, S5K6A2NAME, S5K6A2Size);
	}
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

	if (!strncmp(buf, "dxo on", 6)) {
		info->pdata->use_rawchip = RAWCHIP_ENABLE;
	        pr_info("[CAM] rawchip DxO turned ON\n");
		return count;
	} else if (!strncmp(buf, "dxo off", 7)) {
		info->pdata->use_rawchip = RAWCHIP_MIPI_BYPASS;
		pr_info("[CAM] rawchip DxO turned OFF\n");
		return count;
	}

	tmp = buf[0] - 0x30; /* only get the first char */

#if 0
	if (strcmp(current->comm,"com.android.camera")!=0){
		pr_info("[CAM] No permission : not camera ap\n");
		return -EINVAL;
	}
#endif

	htcwc_value = tmp;
	//pr_info("[CAM]current_comm = %s\n", current->comm);
	pr_info("[CAM] htcwc_value = %d\n", htcwc_value);
	return count;
}

static ssize_t sensor_read_node(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t length;
	length = sprintf(buf, "%d\n", sensor_probe_node);
	return length;
}

static ssize_t sensor_read_data_port(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t length;
	length = sprintf(buf, "%d\n", info->pdata->csi_if);
	return length;
}

static ssize_t sensor_read_source(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t length;
	length = sprintf(buf, "%d\n", sensor_source);
	return length;
}

static ssize_t pcid_detect_frontcam_rawchip(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t bFrontcamRawchip;

	if (info->pdata->use_rawchip && sensor_source != SENSOR_S5K6A1)
		bFrontcamRawchip = sprintf(buf, "1\n");
    else
		bFrontcamRawchip = sprintf(buf, "0\n");
	return bFrontcamRawchip;
}
static DEVICE_ATTR(sensor, 0444, sensor_vendor_show, NULL);
static DEVICE_ATTR(htcwc, 0664, htcwc_get, htcwc_set);
static DEVICE_ATTR(node, 0444, sensor_read_node, NULL);
static DEVICE_ATTR(data_port, 0444, sensor_read_data_port, NULL);
static DEVICE_ATTR(source, 0444, sensor_read_source, NULL);
static DEVICE_ATTR(frontcam_rawchip, 0444, pcid_detect_frontcam_rawchip, NULL);

static struct kobject *android_s5k6a1g;

static int s5k6a1g_sysfs_init(void)
{
	int ret ;
	pr_info("[CAM] s5k6a1g:kobject creat and add\n");
	android_s5k6a1g = kobject_create_and_add("android_camera2", NULL);
	if (android_s5k6a1g == NULL) {
		pr_info("[CAM]s5k6a1g_sysfs_init: subsystem_register " \
		"failed\n");
		ret = -ENOMEM;
		return ret ;
	}
	pr_info("[CAM] s5k6a1g:sysfs_create_file\n");
	ret = sysfs_create_file(android_s5k6a1g, &dev_attr_sensor.attr);
	if (ret) {
		pr_info("[CAM]s5k6a1g_sysfs_init: sysfs_create_file " \
		"failed\n");
		kobject_del(android_s5k6a1g);
	}

	ret = sysfs_create_file(android_s5k6a1g, &dev_attr_htcwc.attr);
	if (ret) {
		pr_info("[CAM] s5k6a1g_sysfs_init: sysfs_create_file htcwc failed\n");
		kobject_del(android_s5k6a1g);
	}

	ret = sysfs_create_file(android_s5k6a1g, &dev_attr_node.attr);
	if (ret) {
		pr_info("[CAM] s5k6a1g_sysfs_init: dev_attr_node failed\n");
		kobject_del(android_s5k6a1g);
	}

	ret = sysfs_create_file(android_s5k6a1g, &dev_attr_data_port.attr);
	if (ret) {
		pr_info("[CAM] %s: dev_attr_data_port failed\n", __func__);
		kobject_del(android_s5k6a1g);
	}

	ret = sysfs_create_file(android_s5k6a1g, &dev_attr_source.attr);
	if (ret) {
		pr_info("[CAM] %s: dev_attr_source failed\n", __func__);
		kobject_del(android_s5k6a1g);
	}

	ret = sysfs_create_file(android_s5k6a1g, &dev_attr_frontcam_rawchip.attr);
	if (ret) {
		pr_info("[CAM] %s: dev_attr_frontcam_rawchip failed\n", __func__);
		kobject_del(android_s5k6a1g);
	}

	return 0 ;
}

static int check_sensor_module(void)
{
	uint16_t chipid = 0;
	int32_t rc = 0;
	struct clk *csus_clk = NULL;
	struct clk *sensor_clk = NULL;

	/* enable main clock */
	csus_clk = clk_get(NULL, "csus");
	if (IS_ERR_OR_NULL(csus_clk)) {
		pr_err("[CAM] %s: couldn't get csus clock\n", __func__);
		goto fail_3;
	}
	sensor_clk = clk_get(NULL, "vi_sensor");
	if (IS_ERR_OR_NULL(sensor_clk)) {
		pr_err("[CAM] %s: couldn't get sensor clock\n", __func__);
		goto fail_2;
	}
	clk_enable(csus_clk);
	clk_enable(sensor_clk);
	clk_set_rate(sensor_clk, 24000000);  /* 24MHz */

	if (info->pdata->power_on == NULL) {
		pr_err("[CAM] %s: power_on is NULL\n", __func__);
		goto fail_1;
	}
	info->pdata->power_on();

	/* Read sensor Model ID: */
	rc = s5k6a1gx03_i2c_read_simple(s5k6a1gx03_REG_MODEL_ID, &chipid, 2);
	if (rc < 0) {
		pr_err("[CAM] %s: read sensor id fail\n", __func__);
		goto fail_1;
	}

	if (chipid == s5k6a1gx03_MODEL_ID) {
		sensor_source = SENSOR_S5K6A1;
		coarse_time_reg = s5k6a1gx03_REG_COARSE_TIME;
		group_hold_reg = s5k6a1gx03_REG_GROUP_HOLD;
		pr_info("[CAM] %s: front camera uses s5k6a1\n", __func__);
	} else if (chipid == s5k6a2_MODEL_ID) {
		coarse_time_reg = s5k6a2_REG_COARSE_TIME;
		group_hold_reg = s5k6a2_REG_GROUP_HOLD;
		sensor_source = SENSOR_S5K6A2;
		pr_info("[CAM] %s: front camera uses s5k6a2\n", __func__);
	} else {
		pr_err("[CAM] sensor id is wrong (%x)\n", chipid);
		goto fail_1;
	}

	if (info->pdata->power_off == NULL) {
		pr_err("[CAM] %s: power_off is NULL\n", __func__);
		goto fail_1;
	}
	info->pdata->power_off();

	/* disable main clock */
	clk_disable(csus_clk);
	clk_disable(sensor_clk);
	clk_put(csus_clk);
	clk_put(sensor_clk);

	return 0;

fail_1:
	if (info->pdata->power_off) {
		info->pdata->power_off();
	}
	clk_put(sensor_clk);
fail_2:
	clk_put(csus_clk);
fail_3:
	return -rc;
}
/* HTC END */

static void set_mode_handler(struct work_struct *work)
{
	struct s5k6a1gx03_info *info;
	int err;

	pr_info("[CAM] %s ++\n", __func__);
	info = container_of(work, struct s5k6a1gx03_info,
			set_mode_work);
	mutex_lock(&info->camera_lock);

	s5k6a1gx03_set_power(1);

	if((err = s5k6a1gx03_check_sensorid())!=0)
	{
		pr_err("[CAM] %s reading sensor id failed\n", __func__);
		goto s5k6a1gx03_set_mode_handler_error;
	}

	err = s5k6a1gx03_set_mode(info, NULL);
	if (err)
		pr_err("[CAM] set sensor mode failed!!!\n");

s5k6a1gx03_set_mode_handler_error:

	info->power_state = true;
	mutex_unlock(&info->camera_lock);
	pr_info("[CAM] %s --\n", __func__);
}

static int s5k6a1gx03_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int err, retry;
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
	s5k6a1_i2c_client = client;

	INIT_WORK(&info->set_mode_work, set_mode_handler);
	mutex_init(&info->camera_lock);

	/* HTC START */
	retry = 0;
	while (check_sensor_module() && retry++ < s5k6a1gx03_MAX_RETRIES) {
		msleep(30);
	}
	s5k6a1g_sysfs_init();
	/* HTC END */

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
