/*
 * ov8838.c - ov8838 sensor driver
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
#include <media/ov8838.h>
#include <media/rawchip/Yushan_HTC_Functions.h>
#include <media/rawchip/rawchip.h>
#include <mach/board_htc.h>


#include <../arch/arm/mach-tegra/include/mach/tegra_flashlight.h>
#include <linux/clk.h>
#include <../arch/arm/mach-tegra/clock.h>

struct i2c_client *ov8838_i2c_client;
#define OV8838_REG_MODEL_ID 0x300A
#define OV8838_MODEL_ID 0x8830

#define SIZEOF_I2C_TRANSBUF 32

typedef enum ov8838_mode_e{
	OV8838_MODE_3280x2464_DEFAULT,
	OV8838_MODE_3280x2464,
	OV8838_MODE_3280x1856,
	OV8838_MODE_3280x1856_EVITA,
	OV8838_MODE_3088x1736,
	OV8838_MODE_1920x1080,
	OV8838_MODE_1640x510,
} ov8838_mode;

struct ov8838_reg {
	u16 addr;
	u16 val;
};

struct ov8838_info {
	ov8838_mode mode;
	struct work_struct set_mode_work;
	struct work_struct rawchip_work;
	struct mutex camera_lock;
	struct mutex rawchip_lock;
	struct i2c_client *i2c_client;
	struct ov8838_platform_data *pdata;
	u8 i2c_trans_buf[SIZEOF_I2C_TRANSBUF];
	bool power_state;
};

static struct ov8838_info *info;

static int init_sensor_mode = OV8838_MODE_3280x1856;
static int sensor_probe_node = 0;
static bool FirstTimeSetMode = 0;
static bool clock_init_done = 0;

#define OV8838_TABLE_WAIT_MS 0
#define OV8838_TABLE_END 1
#define OV8838_MAX_RETRIES 3

/*
 * -------------------
 * | rawchip setting |
 * -------------------
 */


struct rawchip_sensor_data rawchip_mode_table_1[] =
{
	[OV8838_MODE_3280x2464_DEFAULT] =
	{	/* 8M 3280x2464 */
		.datatype = 0x2B,
		.pixel_clk = 259200000,
		.width = 3280,
		.height = 2464,
		.line_length_pclk = 3624,
		.frame_length_lines = 2530,
		.x_addr_start = 0,
		.y_addr_start = 0,
		.x_addr_end = 3280 -1,
		.y_addr_end = 2464 -1,
		.x_even_inc = 1,
		.x_odd_inc = 1,
		.y_even_inc = 1,
		.y_odd_inc = 1,
		.binning_rawchip = 0x11,
	},
	[OV8838_MODE_3280x2464] =
	{	/* 8M 3280x2464 */
		.datatype = 0x2B,
		.pixel_clk = 259200000,
		.width = 3280,
		.height = 2464,
		.line_length_pclk = 3624,
		.frame_length_lines = 2530,
		.x_addr_start = 0,
		.y_addr_start = 0,
		.x_addr_end = 3280 -1,
		.y_addr_end = 2464 -1,
		.x_even_inc = 1,
		.x_odd_inc = 1,
		.y_even_inc = 1,
		.y_odd_inc = 1,
		.binning_rawchip = 0x11,
	},
	[OV8838_MODE_3280x1856] =
	{	/* 6M 3280x1856 */
		.datatype = 0x2B,
		.pixel_clk = 195200000,
		.width = 3280,
		.height = 1856,
		.line_length_pclk = 3648,
		.frame_length_lines = 1922,
		.x_addr_start = 0,
		.y_addr_start = 0x134 + 4,
		.x_addr_end = 3280 - 1,
		.y_addr_end = 0x87b - 4,
		.x_even_inc = 1,
		.x_odd_inc = 1,
		.y_even_inc = 1,
		.y_odd_inc = 1,
		.binning_rawchip = 0x11,
	},
	[OV8838_MODE_3280x1856_EVITA] =
	{	/* 6M 3280x1856 */
		.datatype = 0x2B,
		.pixel_clk = 199200000,
		.width = 3280,
		.height = 1856,
		.line_length_pclk = 3648,
		.frame_length_lines = 1922,
		.x_addr_start = 0,
		.y_addr_start = 0x134 + 4,
		.x_addr_end = 3280 - 1,
		.y_addr_end = 0x87b - 4,
		.x_even_inc = 1,
		.x_odd_inc = 1,
		.y_even_inc = 1,
		.y_odd_inc = 1,
		.binning_rawchip = 0x11,
	},
	[OV8838_MODE_1920x1080] =
	{	/* 1080p 1920x1080 */
		.datatype = 0x2B,
		.pixel_clk = 259200000,
		.width = 1920,
		.height = 1080,
		.line_length_pclk = 3624,
		.frame_length_lines = 1250,
		.x_addr_start = 0x2ac + 4,
		.y_addr_start = 0x2b8 + 4,
		.x_addr_end = 0xa33 - 4,
		.y_addr_end = 0x6f7 - 4,
		.x_even_inc = 1,
		.x_odd_inc = 1,
		.y_even_inc = 1,
		.y_odd_inc = 1,
		.binning_rawchip = 0x11,
	},
	[OV8838_MODE_1640x510]
	{	/* slow motion mode 1640x510 */	
		.datatype = 0x2B,
		.pixel_clk = 259200000,
		.width = 1640,
		.height = 510,
		.line_length_pclk = 3624,
		.frame_length_lines = 610,
		.x_addr_start = 0x338 + 4,
		.y_addr_start = 0x3d4 + 4,
		.x_addr_end = 0x9a7 - 4,
		.y_addr_end = 0x5d9 - 4,
		.x_even_inc = 1,
		.x_odd_inc = 1,
		.y_even_inc = 1,
		.y_odd_inc = 1,
		.binning_rawchip = 0x11,
	},
};

struct rawchip_sensor_data rawchip_mode_table_2[] =
{
	[OV8838_MODE_3280x2464] =
	{	/* 8M 3280x2464 */
		.datatype = 0x2B,
		.pixel_clk = 182400000,
		.width = 3280,
		.height = 2464,
		.line_length_pclk = 3624,
		.frame_length_lines = 2524,
		.x_addr_start = 0,
		.y_addr_start = 0,
		.x_addr_end = 3280 -1,
		.y_addr_end = 2464 -1,
		.x_even_inc = 1,
		.x_odd_inc = 1,
		.y_even_inc = 1,
		.y_odd_inc = 1,
		.binning_rawchip = 0x11,
	},
	[OV8838_MODE_3088x1736] =
	{	/* 5M 3088x1736 */
		.datatype = 0x2B,
		.pixel_clk = 182400000,
		.width = 3088,
		.height = 1736,
		.line_length_pclk = 3624,
		.frame_length_lines = 1772,
		.x_addr_start = 0x64 + 4,
		.y_addr_start = 0x170 + 4,
		.x_addr_end = 0xc7b - 4,
		.y_addr_end = 0x83f - 4,
		.x_even_inc = 1,
		.x_odd_inc = 1,
		.y_even_inc = 1,
		.y_odd_inc = 1,
		.binning_rawchip = 0x11,
	},
	[OV8838_MODE_1920x1080] =
	{	/* 1080p 1920x1080 */
		.datatype = 0x2B,
		.pixel_clk = 180000000,
		.width = 1920,
		.height = 1080,
		.line_length_pclk = 3624,
		.frame_length_lines = 1152,
		.x_addr_start = 0x2ac + 4,
		.y_addr_start = 0x2b8 + 4,
		.x_addr_end = 0xa33 - 4,
		.y_addr_end = 0x6f7 - 4,
		.x_even_inc = 1,
		.x_odd_inc = 1,
		.y_even_inc = 1,
		.y_odd_inc = 1,
		.binning_rawchip = 0x11,
	},
	[OV8838_MODE_1640x510]
	{	/* slow motion mode 1640x510 */	
		.datatype = 0x2B,
		.pixel_clk = 180000000,
		.width = 1640,
		.height = 510,
		.line_length_pclk = 3624,
		.frame_length_lines = 552,
		.x_addr_start = 0x338 + 4,
		.y_addr_start = 0x3d4 + 4,
		.x_addr_end = 0x9a7 - 4,
		.y_addr_end = 0x5d9 - 4,
		.x_even_inc = 1,
		.x_odd_inc = 1,
		.y_even_inc = 1,
		.y_odd_inc = 1,
		.binning_rawchip = 0x11,
	},
};

/*
 * --------------------
 * | register setting |
 * --------------------
 */

static struct ov8838_reg stream_off_mipi_data[] = {
	{ 0x4202, 0x0f },
	{ OV8838_TABLE_WAIT_MS, 33 },
	{ OV8838_TABLE_END, 0x0000 },
};

static struct ov8838_reg stream_on_mipi_data[] = {
	{ 0x4202, 0x00 },
	{ OV8838_TABLE_WAIT_MS, 5 },
	{ OV8838_TABLE_END, 0x0000 },
};

static struct ov8838_reg reset_seq[] = {
	{ 0x0100, 0x00 },	// stream off
	{ OV8838_TABLE_WAIT_MS, 5 },
	{ OV8838_TABLE_END, 0x0000 },
};

/*
 * Mipi 4 lane, 648Mbps initial setting
 */
static struct ov8838_reg mode_start_1[] = {
	/* Sensor initial setting */
	{ 0x0100, 0x00 },
	{ 0x0103, 0x01 },
	{ OV8838_TABLE_WAIT_MS, 10 },

	{ 0x0102, 0x01 },
	{ 0x3001, 0x2a },
	{ 0x3002, 0x88 },
	{ 0x3005, 0x00 },
	{ 0x3011, 0x41 },
	{ 0x3015, 0x08 },
	{ 0x301b, 0xb4 },
	{ 0x301d, 0x02 },
	{ 0x3021, 0x00 },
	{ 0x3022, 0x00 },
	{ 0x3081, 0x02 },
	{ 0x3083, 0x01 },

	/* System clock PLL Start */
	{ 0x3090, 0x03 },
	{ 0x3091, 0x22 },
	{ 0x3092, 0x00 },
	{ 0x3093, 0x00 },
	{ 0x3094, 0x00 },
	/* System clock PLL End */

	/* Reference Clock PLL Start */
	{ 0x3098, 0x04 },
	{ 0x3099, 0x10 },
	{ 0x309a, 0x00 },
	{ 0x309b, 0x00 },
	/* Reference Clock PLL End */

	{ 0x30a2, 0x01 }, /* for ov internal */

	/* Mipi clock PLL Start */
	{ 0x30b0, 0x05 },
	{ 0x30b2, 0x00 },
	{ 0x30b3, 0x51 },
	{ 0x30b4, 0x03 },
	{ 0x30b5, 0x04 },
	/* Mipi clock PLL End */

	{ 0x30b6, 0x01 },
//	{ 0x3104, 0xa1 }, //SCLK_SW2PLL2 Default 0xA1
	{ 0x3106, 0x01 },
//	{ 0x3400, 0x04 }, //MWB Gain00 Default 0x04
//	{ 0x3401, 0x00 }, //MWB Gain01 Default 0x00
//	{ 0x3402, 0x04 }, //MWB Gain02 Default 0x04
//	{ 0x3403, 0x00 }, //MWB Gain03 Default 0x00
//	{ 0x3404, 0x04 }, //MWB Gain04 Default 0x04
//	{ 0x3405, 0x00 }, //MWB Gain05 Default 0x00
	{ 0x3406, 0x01 }, //MWB CTRL Default 0x00
	{ 0x3500, 0x00 }, //AEC LONG EXPO
	{ 0x3501, 0x20 }, //AEC LONG EXPO
	{ 0x3502, 0xc0 }, //AEC LONG EXPO
	{ 0x3503, 0x07 }, //AEC Manual
	{ 0x3504, 0x00 },
	{ 0x3505, 0x30 },
	{ 0x3506, 0x00 }, //AEC short EXPO
	{ 0x3507, 0x10 }, //AEC short EXPO
	{ 0x3508, 0x80 }, //AEC short EXPO
//	{ 0x3509, 0x10 }, //AEC Gain convert Default 0x10
	{ 0x350a, 0x00 },
	{ 0x350b, 0x38 }, //AEC AGC ADJ Gain=0x350B/16

	/* for ov internal */
	{ 0x3600, 0xb8 },
	{ 0x3601, 0x0a },
	{ 0x3602, 0x9c },
	{ 0x3604, 0x38 },
	{ 0x3612, 0x80 },
	{ 0x3620, 0x41 },
	{ 0x3621, 0xa4 },
	{ 0x3622, 0x0b },
	{ 0x3625, 0x44 },
	{ 0x3630, 0x55 },
	{ 0x3631, 0xd2 },
	{ 0x3632, 0x00 },
	{ 0x3633, 0x34 },
	{ 0x3634, 0x03 },
	{ 0x364d, 0x0d },
	{ 0x364f, 0x60 },
	{ 0x3660, 0x80 },
	{ 0x3662, 0x10 },
	{ 0x3665, 0x00 },
	{ 0x3666, 0x00 },
	{ 0x3667, 0x00 },
	{ 0x366a, 0x80 },
	{ 0x366c, 0x00 },
	{ 0x366d, 0x00 },
	{ 0x366e, 0x00 },
	{ 0x366f, 0x20 },
	{ 0x3680, 0xb5 },
	{ 0x3681, 0x00 },
	{ 0x3701, 0x14 },
	{ 0x3702, 0x50 },
	{ 0x3703, 0x8c },
	{ 0x3704, 0x68 },
	{ 0x3705, 0x02 },
	{ 0x3708, 0xe3 },
	{ 0x3709, 0x43 },
	{ 0x370a, 0x00 },
	{ 0x370b, 0x20 },
	{ 0x370c, 0x0c },
	{ 0x370d, 0x11 },
	{ 0x370e, 0x00 },
	{ 0x370f, 0x00 },
	{ 0x3710, 0x00 },
	{ 0x371c, 0x01 },
	{ 0x371f, 0x0c },
	{ 0x3721, 0x00 },
	{ 0x3724, 0x10 },
	{ 0x3726, 0x00 },
	{ 0x372a, 0x01 },
	{ 0x3730, 0x18 },
	{ 0x3738, 0x22 },
	{ 0x3739, 0xd0 },
	{ 0x373a, 0x50 },
	{ 0x373b, 0x02 },
	{ 0x373c, 0x20 },
	{ 0x373f, 0x02 },
	{ 0x3740, 0x42 },
	{ 0x3741, 0x02 },
	{ 0x3742, 0x18 },
	{ 0x3743, 0x01 },
	{ 0x3744, 0x02 },
	{ 0x3747, 0x10 },
	{ 0x374c, 0x04 },
	{ 0x3751, 0xf0 },
	{ 0x3752, 0x00 },
	{ 0x3753, 0x00 },
	{ 0x3754, 0xc0 },
	{ 0x3755, 0x00 },
	{ 0x3756, 0x1a },
	{ 0x3758, 0x00 },
	{ 0x3759, 0x0f },
	{ 0x375c, 0x04 },
	{ 0x3767, 0x01 },
	{ 0x376b, 0x44 },
	{ 0x3774, 0x10 },
	{ 0x3776, 0x00 },
	{ 0x377f, 0x08 },
	{ 0x3780, 0x22 },
	{ 0x3781, 0xcc },
	{ 0x3784, 0x2c },
	{ 0x3785, 0x08 },
	{ 0x3786, 0x16 },
	{ 0x378f, 0xf5 },
	{ 0x3791, 0xb0 },
	{ 0x3795, 0x00 },
	{ 0x3796, 0x94 },
	{ 0x3797, 0x11 },
	{ 0x3798, 0x30 },
	{ 0x3799, 0x41 },
	{ 0x379a, 0x07 },
	{ 0x379b, 0xb0 },
	{ 0x379c, 0x0c },
	{ 0x37c5, 0x00 },
	{ 0x37c6, 0xa0 },
	{ 0x37c7, 0x00 },
	{ 0x37c9, 0x00 },
	{ 0x37ca, 0x00 },
	{ 0x37cb, 0x00 },
	{ 0x37cc, 0x00 },
	{ 0x37cd, 0x00 },
	{ 0x37ce, 0x01 },
	{ 0x37cf, 0x00 },
	{ 0x37d1, 0x01 },
	{ 0x37de, 0x00 },
	{ 0x37df, 0x00 },

	/* resolution setting for default */
//	{ 0x3800, 0x00 }, /* TIMING_X_ADDR_START */
//	{ 0x3801, 0x04 }, /* TIMING_X_ADDR_START */ //X_ADDR_START = 4
//	{ 0x3802, 0x00 }, /* TIMING_Y_ADDR_START */
//	{ 0x3803, 0x04 }, /* TIMING_Y_ADDR_START */ //Y_ADDR_START = 4
//	{ 0x3804, 0x0c }, /* TIMING_X_ADDR_END */
//	{ 0x3805, 0xdb }, /* TIMING_X_ADDR_END */   //X_ADDR_END = 3291
//	{ 0x3806, 0x09 }, /* TIMING_Y_ADDR_END */
//	{ 0x3807, 0xAB }, /* TIMING_Y_ADDR_END */   //Y_ADDR_END = 2475
//	{ 0x3808, 0x0c }, /* TIMING_X_OUTPUT_SIZE */
//	{ 0x3809, 0xd0 }, /* TIMING_X_OUTPUT_SIZE *///X_OUTPUT_SIZE = 3280
//	{ 0x380a, 0x09 }, /* TIMING_Y_OUTPUT_SIZE */
//	{ 0x380b, 0xA0 }, /* TIMING_Y_OUTPUT_SIZE *///Y_OUTPUT_SIZE = 2464
//	{ 0x380c, 0x0e }, /* TIMING_HTS */
//	{ 0x380d, 0x28 }, /* TIMING_HTS */          //HTS = 3624
//	{ 0x380e, 0x09 }, /* TIMING_VTS */
//	{ 0x380f, 0xCE }, /* TIMING_VTS */          //VTS = 2510
//	{ 0x3810, 0x00 }, /* TIMING_ISP_X_WIN */
//	{ 0x3811, 0x04 }, /* TIMING_ISP_X_WIN */    //X_WIN = 4
//	{ 0x3812, 0x00 }, /* TIMING_ISP_Y_WIN */
//	{ 0x3813, 0x04 }, /* TIMING_ISP_Y_WIN */    //Y_WIN = 4
//	{ 0x3814, 0x11 }, /* TIMING_X_INC */
//	{ 0x3815, 0x11 }, /* TIMING_Y_INC */

#if defined(CONFIG_MACH_OPERAUL)
	{ 0x3820, 0x10 },
	{ 0x3821, 0x0E },
#else
	{ 0x3820, 0x52 },
	{ 0x3821, 0x08 },
#endif
	{ 0x3823, 0x00 },
	{ 0x3824, 0x00 },
	{ 0x3825, 0x00 },
	{ 0x3826, 0x00 },
	{ 0x3827, 0x00 },
	{ 0x382a, 0x04 },
	{ 0x3a04, 0x09 },
	{ 0x3a05, 0xa9 },
	{ 0x3a06, 0x00 },
	{ 0x3a07, 0xf8 },
//	{ 0x3b00, 0x00 }, //Strobe control related Default 0x00
//	{ 0x3b02, 0x00 }, //Strobe control related Default 0x00
//	{ 0x3b03, 0x00 }, //Strobe control related Default 0x00
//	{ 0x3b04, 0x00 }, //Strobe control related Default 0x00
//	{ 0x3b05, 0x00 }, //Strobe control related Default 0x00
//	{ 0x3d00, 0x00 }, //OTP setting
//	{ 0x3d01, 0x00 }, //OTP setting
//	{ 0x3d02, 0x00 }, //OTP setting
//	{ 0x3d03, 0x00 }, //OTP setting
//	{ 0x3d04, 0x00 }, //OTP setting
//	{ 0x3d05, 0x00 }, //OTP setting
//	{ 0x3d06, 0x00 }, //OTP setting
//	{ 0x3d07, 0x00 }, //OTP setting
//	{ 0x3d08, 0x00 }, //OTP setting
//	{ 0x3d09, 0x00 }, //OTP setting
//	{ 0x3d0a, 0x00 }, //OTP setting
//	{ 0x3d0b, 0x00 }, //OTP setting
//	{ 0x3d0c, 0x00 }, //OTP setting
//	{ 0x3d0d, 0x00 }, //OTP setting
//	{ 0x3d0e, 0x00 }, //OTP setting
//	{ 0x3d0f, 0x00 }, //OTP setting
//	{ 0x3d80, 0x00 }, //OTP setting 0x01 for program OTP
//	{ 0x3d81, 0x00 }, //OTP setting 0x01 for Read from OTP
//	{ 0x3d84, 0x00 }, //OTP setting 0xC5 for Disable program, select bank 5
	{ 0x4000, 0x18 }, //BLC Bypass Default 0x10
	{ 0x4001, 0x04 }, //BLC start LN Default 0x06
//	{ 0x4002, 0x45 }, //BLC Auto Default 0x45
	{ 0x4004, 0x08 },
	{ 0x4005, 0x18 },
	{ 0x4006, 0x20 },
	{ 0x4008, 0x24 },
	{ 0x4009, 0x13 }, /* David@Optical, 10 */
	{ 0x404f, 0xA0 }, //change from 0x90 to 0xA0 by OV.
	{ 0x4100, 0x20 },
	{ 0x4101, 0x03 },
	{ 0x4102, 0x04 },
	{ 0x4103, 0x03 },
	{ 0x4104, 0x5a },
	{ 0x4307, 0x30 },
	{ 0x4315, 0x00 },
	{ 0x4511, 0x05 },
	{ 0x4512, 0x01 },
	{ 0x4805, 0x21 },
	{ 0x4806, 0x00 },
	{ 0x481f, 0x36 },
	{ 0x4831, 0x6c },
	{ 0x4837, 0x0c },
	{ 0x4a00, 0xaa },
	{ 0x4a03, 0x01 },
	{ 0x4a05, 0x08 },
	{ 0x4a0a, 0x88 },
	{ 0x4d03, 0xbb },
//	{ 0x5000, 0x06 }, //ISP CTRL0 Default 0x06
//	{ 0x5001, 0x01 }, //ISP CTRL1 Default 0x01
	{ 0x5002, 0x80 },
	{ 0x5003, 0x20 },
	{ 0x5013, 0x00 },
	{ 0x5046, 0x4a },
	{ 0x5780, 0x1c },
	{ 0x5786, 0x20 },
	{ 0x5787, 0x10 },
	{ 0x5788, 0x18 },
	{ 0x578a, 0x04 },
	{ 0x578b, 0x02 },
	{ 0x578c, 0x02 },
	{ 0x578e, 0x06 },
	{ 0x578f, 0x02 },
	{ 0x5790, 0x02 },
	{ 0x5791, 0xff },
	{ 0x5a08, 0x02 },
	{ 0x5e00, 0x00 },
	{ 0x5e10, 0x0c },

	{ OV8838_TABLE_END, 0x0000 }
};

/*
 * @@ 3280x2464_30fps_4lane setting
 * ----------------------------------------------
 *  3280x2464_30fps_4lane setting
 *  SystemCLK     :272Mhz
 *  FPS	        :30
 *  HTS		:3624(R380c:R380d)
 *  VTS		:2510(R380e:R380f)
 *  Tline 	:13.32us
 *  Max exp line	:2490 ( VTS-12)
 * ---------------------------------------------
 */
static struct ov8838_reg mode_3280x2464[] = {
	/* resolution setting for default */
//	{ 0x3208, 0x00 }, /* Group 0 hold start/Group Bank 0 */

	/* PLL setting */
	{ 0x3090, 0x03 },
	{ 0x3091, 0x22 }, /* PLL_PLL11 */
	{ 0x30b3, 0x51 }, /* PLL_MULTIPLIER */
	{ 0x30b4, 0x03 },
	{ 0x30b6, 0x01 },
	{ 0x4837, 0x0c },

	{ 0x3500, 0x00 },
	{ 0x3501, 0x20 },
	{ 0x3502, 0xc0 },
	{ 0x350b, 0x38 },

	{ 0x3800, 0x00 }, /* TIMING_X_ADDR_START */
	{ 0x3801, 0x04 }, /* TIMING_X_ADDR_START */ //X_ADDR_START = 4
	{ 0x3802, 0x00 }, /* TIMING_Y_ADDR_START */
	{ 0x3803, 0x04 }, /* TIMING_Y_ADDR_START */ //Y_ADDR_START = 4
	{ 0x3804, 0x0c }, /* TIMING_X_ADDR_END */
	{ 0x3805, 0xdb }, /* TIMING_X_ADDR_END */   //X_ADDR_END = 3291
	{ 0x3806, 0x09 }, /* TIMING_Y_ADDR_END */
	{ 0x3807, 0xab }, /* TIMING_Y_ADDR_END */   //Y_ADDR_END = 2475
	{ 0x3808, 0x0c }, /* TIMING_X_OUTPUT_SIZE */
	{ 0x3809, 0xd0 }, /* TIMING_X_OUTPUT_SIZE *///X_OUTPUT_SIZE = 3280
	{ 0x380a, 0x09 }, /* TIMING_Y_OUTPUT_SIZE */
	{ 0x380b, 0xa0 }, /* TIMING_Y_OUTPUT_SIZE *///Y_OUTPUT_SIZE = 2464
	{ 0x380c, 0x0e }, /* TIMING_HTS */
	{ 0x380d, 0x28 }, /* TIMING_HTS */          //HTS = 3624
	{ 0x380e, 0x09 }, /* TIMING_VTS */
	{ 0x380f, 0xe2 }, /* TIMING_VTS */          //VTS = 2530
	{ 0x3810, 0x00 }, /* TIMING_ISP_X_WIN */
	{ 0x3811, 0x04 }, /* TIMING_ISP_X_WIN */    //X_WIN = 4
	{ 0x3812, 0x00 }, /* TIMING_ISP_Y_WIN */
	{ 0x3813, 0x04 }, /* TIMING_ISP_Y_WIN */    //Y_WIN = 4

	{ 0x3814, 0x11 }, /* TIMING_X_INC */
	{ 0x3815, 0x11 }, /* TIMING_Y_INC */

#if defined(CONFIG_MACH_OPERAUL)
	{ 0x3820, 0x10 },
	{ 0x3821, 0x0E },
#else
	{ 0x3820, 0x52 },
	{ 0x3821, 0x08 },
#endif

	{ 0x3708, 0xe3 },
	{ 0x3709, 0x43 },
	{ 0x4512, 0x01 },

	{ 0x3a04, 0x09 },
	{ 0x3a05, 0xa9 },

//	{ 0x3208, 0x10 }, /* Group 0 hold end/Group Bank 0 */
//	{ 0x3208, 0xa0 },

	{ OV8838_TABLE_END, 0x0000 }
};

/*
 *@@ 3280x1856_30fps_4lane setting
 *----------------------------------------------
 * MIPI 4 Lane
 * Mipi data rate: 488Mbps
 * SystemCLK     :208Mhz
 * FPS	        :30
 * HTS		:3648 (R380c:R380d)
 * VTS		:1922 (R380e:R380f)
 * Tline 	:17.54us
 * Max exp line	:1908 ( VTS-14)
 *----------------------------------------------
 */
static struct ov8838_reg mode_3280x1856[] = {
	/* resolution setting for default */
//	{ 0x3208, 0x02 }, /* Group 2 hold start/Group Bank 2 */

	/* PLL setting */
	{ 0x3090, 0x03 },
	{ 0x3091, 0x1A }, /* PLL_PLL11 */
	{ 0x30b3, 0x7A }, /* PLL_MULTIPLIER */
	{ 0x30b4, 0x03 },
	{ 0x30b6, 0x02 },
	{ 0x4837, 0x10 },

	{ 0x3500, 0x00 },
	{ 0x3501, 0x20 },
	{ 0x3502, 0xc0 },
	{ 0x350b, 0x38 },

	{ 0x3800, 0x00 }, /* TIMING_X_ADDR_START */
	{ 0x3801, 0x04 }, /* TIMING_X_ADDR_START */
	{ 0x3802, 0x01 }, /* TIMING_Y_ADDR_START */
	{ 0x3803, 0x34 }, /* TIMING_Y_ADDR_START */
	{ 0x3804, 0x0c }, /* TIMING_X_ADDR_END */
	{ 0x3805, 0xdb }, /* TIMING_X_ADDR_END */
	{ 0x3806, 0x08 }, /* TIMING_Y_ADDR_END */
	{ 0x3807, 0x7b }, /* TIMING_Y_ADDR_END */
	{ 0x3808, 0x0c }, /* TIMING_X_OUTPUT_SIZE */
	{ 0x3809, 0xd0 }, /* TIMING_X_OUTPUT_SIZE */
	{ 0x380a, 0x07 }, /* TIMING_Y_OUTPUT_SIZE */
	{ 0x380b, 0x40 }, /* TIMING_Y_OUTPUT_SIZE */
	{ 0x380c, 0x0e }, /* TIMING_HTS */
	{ 0x380d, 0x40 }, /* TIMING_HTS */
	{ 0x380e, 0x07 }, /* TIMING_VTS */
	{ 0x380f, 0x82 }, /* TIMING_VTS */
	{ 0x3810, 0x00 }, /* TIMING_ISP_X_WIN */
	{ 0x3811, 0x04 }, /* TIMING_ISP_X_WIN */
	{ 0x3812, 0x00 }, /* TIMING_ISP_Y_WIN */
	{ 0x3813, 0x04 }, /* TIMING_ISP_Y_WIN */

	{ 0x3814, 0x11 }, /* TIMING_X_INC */
	{ 0x3815, 0x11 }, /* TIMING_Y_INC */

#if defined(CONFIG_MACH_OPERAUL)
	{ 0x3820, 0x10 },
	{ 0x3821, 0x0E },
#else
	{ 0x3820, 0x52 },
	{ 0x3821, 0x08 },
#endif

	{ 0x3708, 0xe3 },
	{ 0x3709, 0x43 },
	{ 0x4512, 0x01 },

	{ 0x3a04, 0x07 },
	{ 0x3a05, 0x49 },

//	{ 0x3208, 0x12 }, /* Group 2 hold end/Group Bank 2 */
//	{ 0x3208, 0xA2 },

	{ OV8838_TABLE_END, 0x0000 }
};

/*
 *@@ 3280x1856_30fps_4lane setting
 *----------------------------------------------
 * MIPI 4 Lane
 * Mipi data rate: 498Mbps
 * SystemCLK     :210Mhz
 * FPS	        :30
 * HTS		:3648 (R380c:R380d)
 * VTS		:1922 (R380e:R380f)
 * Tline 	:17.54us
 * Max exp line	:1908 ( VTS-14)
 *----------------------------------------------
 */
static struct ov8838_reg mode_3280x1856_evita[] = {
	/* resolution setting for default */
//	{ 0x3208, 0x02 }, /* Group 2 hold start/Group Bank 2 */

	/* PLL setting */
	{ 0x3090, 0x04 },
	{ 0x3091, 0x23 }, /* PLL_PLL11 */
	{ 0x30b3, 0xA6 }, /* PLL_MULTIPLIER */
	{ 0x30b4, 0x04 },
	{ 0x30b6, 0x02 },
	{ 0x4837, 0x10 },

	{ 0x3500, 0x00 },
	{ 0x3501, 0x20 },
	{ 0x3502, 0xc0 },
	{ 0x350b, 0x38 },

	{ 0x3800, 0x00 }, /* TIMING_X_ADDR_START */
	{ 0x3801, 0x04 }, /* TIMING_X_ADDR_START */
	{ 0x3802, 0x01 }, /* TIMING_Y_ADDR_START */
	{ 0x3803, 0x34 }, /* TIMING_Y_ADDR_START */
	{ 0x3804, 0x0c }, /* TIMING_X_ADDR_END */
	{ 0x3805, 0xdb }, /* TIMING_X_ADDR_END */
	{ 0x3806, 0x08 }, /* TIMING_Y_ADDR_END */
	{ 0x3807, 0x7b }, /* TIMING_Y_ADDR_END */
	{ 0x3808, 0x0c }, /* TIMING_X_OUTPUT_SIZE */
	{ 0x3809, 0xd0 }, /* TIMING_X_OUTPUT_SIZE */
	{ 0x380a, 0x07 }, /* TIMING_Y_OUTPUT_SIZE */
	{ 0x380b, 0x40 }, /* TIMING_Y_OUTPUT_SIZE */
	{ 0x380c, 0x0e }, /* TIMING_HTS */
	{ 0x380d, 0x40 }, /* TIMING_HTS */
	{ 0x380e, 0x07 }, /* TIMING_VTS */
	{ 0x380f, 0x82 }, /* TIMING_VTS */
	{ 0x3810, 0x00 }, /* TIMING_ISP_X_WIN */
	{ 0x3811, 0x04 }, /* TIMING_ISP_X_WIN */
	{ 0x3812, 0x00 }, /* TIMING_ISP_Y_WIN */
	{ 0x3813, 0x04 }, /* TIMING_ISP_Y_WIN */

	{ 0x3814, 0x11 }, /* TIMING_X_INC */
	{ 0x3815, 0x11 }, /* TIMING_Y_INC */

#if defined(CONFIG_MACH_OPERAUL)
	{ 0x3820, 0x10 },
	{ 0x3821, 0x0E },
#else
	{ 0x3820, 0x52 },
	{ 0x3821, 0x08 },
#endif

	{ 0x3708, 0xe3 },
	{ 0x3709, 0x43 },
	{ 0x4512, 0x01 },

	{ 0x3a04, 0x07 },
	{ 0x3a05, 0x49 },

//	{ 0x3208, 0x12 }, /* Group 2 hold end/Group Bank 2 */
//	{ 0x3208, 0xA2 },

	{ OV8838_TABLE_END, 0x0000 }
};


/*
 * @@ 1920x1080_60fps_4lane setting
 * ----------------------------------------------
 *  (1920x1080_60fps_4lane setting
 *  SystemCLK     :272Mhz
 *  FPS	        :60
 *  HTS		:3624(R380c:R380d)
 *  VTS		:1250(R380e:R380f)
 *  Tline 	:13.32us
 *  Max exp line	:1238 ( VTS-12)
 * ---------------------------------------------
 */
static struct ov8838_reg mode_1920x1080[] = {
//	{ 0x3208, 0x02 }, /* Group 2 hold start/Group Bank 2 */

	{ 0x3500, 0x00 },
	{ 0x3501, 0x20 },
	{ 0x3502, 0xc0 },
	{ 0x350b, 0x38 },

	{ 0x3800, 0x02 },
	{ 0x3801, 0xac },
	{ 0x3802, 0x02 },
	{ 0x3803, 0xb8 },
	{ 0x3804, 0x0a },
	{ 0x3805, 0x33 },
	{ 0x3806, 0x06 },
	{ 0x3807, 0xf7 },
	{ 0x3808, 0x07 },
	{ 0x3809, 0x80 },
	{ 0x380a, 0x04 },
	{ 0x380b, 0x38 },
	{ 0x380c, 0x0e },
	{ 0x380d, 0x28 },
	{ 0x380e, 0x04 },
	{ 0x380f, 0xe2 },
	{ 0x3810, 0x00 },
	{ 0x3811, 0x04 },
	{ 0x3812, 0x00 },
	{ 0x3813, 0x04 },
	{ 0x3814, 0x11 },
	{ 0x3815, 0x11 },

#if defined(CONFIG_MACH_OPERAUL)
	{ 0x3820, 0x10 },
	{ 0x3821, 0x0E },
#else
	{ 0x3820, 0x52 },
	{ 0x3821, 0x08 },
#endif

	{ 0x3708, 0xe3},
	{ 0x3709, 0x43},
	{ 0x4512, 0x01},

	{ 0x3a04, 0x07 },
	{ 0x3a05, 0x49 },

//	{ 0x3208, 0x12 }, /* Group 2 hold end/Group Bank 2 */
//	{ 0x3208, 0xa2 },

	{ OV8838_TABLE_END, 0x0000 }
};

/*
 * @@ 1640x510_120fps_4lane setting
 * ----------------------------------------------
 *  1640x510_120fps_4lane setting
 *  SystemCLK     :272Mhz
 *  FPS	  :120
 *  HTS		:3624(R380c:R380d)
 *  VTS		:626(R380e:R380f)
 *  Tline 	:13.32us
 *  Max exp line	:614 ( VTS-12)
 * ---------------------------------------------
 */
static struct ov8838_reg mode_1640x510[] = {
	/* resolution setting */
//	{ 0x3208, 0x03 }, /* Group 3 hold start/Group Bank 3 */

	{ 0x3500, 0x00 },
	{ 0x3501, 0x20 },
	{ 0x3502, 0xc0 },
	{ 0x350b, 0x38 },

	/* PLL setting */
	{ 0x3090, 0x03 },
	{ 0x3091, 0x22 }, /* PLL_PLL11 */
	{ 0x30b3, 0x51 }, /* PLL_MULTIPLIER */
	{ 0x30b4, 0x03 },
	{ 0x30b6, 0x01 },
	{ 0x4837, 0x0c },

	{ 0x3800, 0x00 }, /* TIMING_X_ADDR_START */
	{ 0x3801, 0x00 }, /* TIMING_X_ADDR_START */ //X_ADDR_START = 0
	{ 0x3802, 0x00 }, /* TIMING_Y_ADDR_START */
	{ 0x3803, 0xD6 }, /* TIMING_Y_ADDR_START */ //Y_ADDR_START = 214
	{ 0x3804, 0x0c }, /* TIMING_X_ADDR_END */
	{ 0x3805, 0xdf }, /* TIMING_X_ADDR_END */   //X_ADDR_END = 3295
	{ 0x3806, 0x08 }, /* TIMING_Y_ADDR_END */
	{ 0x3807, 0xed }, /* TIMING_Y_ADDR_END */   //Y_ADDR_END = 2285
	{ 0x3808, 0x06 }, /* TIMING_X_OUTPUT_SIZE */
	{ 0x3809, 0x68 }, /* TIMING_X_OUTPUT_SIZE *///X_OUTPUT_SIZE = 1640
	{ 0x380a, 0x01 }, /* TIMING_Y_OUTPUT_SIZE */
	{ 0x380b, 0xfe }, /* TIMING_Y_OUTPUT_SIZE *///Y_OUTPUT_SIZE = 510
	{ 0x380c, 0x0e }, /* TIMING_HTS */
	{ 0x380d, 0x28 }, /* TIMING_HTS */          //HTS = 3624
	{ 0x380e, 0x02 }, /* TIMING_VTS */
	{ 0x380f, 0x62 }, /* TIMING_VTS */          //VTS = 610
	{ 0x3810, 0x00 }, /* TIMING_ISP_X_WIN */
	{ 0x3811, 0x04 }, /* TIMING_ISP_X_WIN */    //X_WIN = 4
	{ 0x3812, 0x00 }, /* TIMING_ISP_Y_WIN */
	{ 0x3813, 0x04 }, /* TIMING_ISP_Y_WIN */    //Y_WIN = 4

	{ 0x3814, 0x31 }, /* TIMING_X_INC */
	{ 0x3815, 0x35 }, /* TIMING_Y_INC */

#if defined(CONFIG_MACH_OPERAUL)
	{ 0x3820, 0x15 },
	{ 0x3821, 0x0F },
#else
	{ 0x3820, 0x57 }, //Bit[0]: Vertical binning
	{ 0x3821, 0x09 }, //Bit[0]: Horizontal binning
#endif

	{ 0x3708, 0xe6 },
	{ 0x3709, 0xcb },
	{ 0x4512, 0x00 }, //Bit[0]: 0 sum, 1 Average

	{ 0x3a04, 0x04 },
	{ 0x3a05, 0xc9 },

//	{ 0x3208, 0x13 }, /* Group 3 hold end/Group Bank 3 */
//	{ 0x3208, 0xa3 },

	{ OV8838_TABLE_END, 0x0000 }
};

/*
 * Mipi 2 lane, 900Mbps initial setting
 */
static struct ov8838_reg mode_start_2[] = {
	{ 0x0100, 0x00 },
	{ 0x0103, 0x01 },
	{ 0x0100, 0x00 },
	{ 0x0100, 0x00 },
	{ 0x0100, 0x00 },
	{ 0x0100, 0x00 },
	{ 0x0102, 0x01 },
	{ 0x3001, 0x2a },
	{ 0x3002, 0x88 },
	{ 0x3005, 0x00 },
	{ 0x3011, 0x21 },
	{ 0x3013, 0x90 }, /* Adjust Common mode voltage and voltage swing: R3013 0x10 =>0x90, ovt@20120521 */
	{ 0x3014, 0xff }, /* MIPI data to clock skew: R3014 0x00=> 0xff, ovt@20120521 */
	{ 0x3015, 0xc8 },
	{ 0x301b, 0xb4 },
	{ 0x301d, 0x02 },
	{ 0x3021, 0x00 },
	{ 0x3022, 0x00 },
	{ 0x3081, 0x02 },
	{ 0x3083, 0x01 },
	{ 0x3090, 0x01 },
	{ 0x3091, 0x10 },
	{ 0x3092, 0x01 },
	{ 0x3093, 0x00 },
	{ 0x3094, 0x00 },
	{ 0x3098, 0x04 },
	{ 0x3099, 0x10 },
	{ 0x309a, 0x00 },
	{ 0x309b, 0x00 },
	{ 0x30a2, 0x01 },
	{ 0x30b0, 0x05 },
	{ 0x30b2, 0x00 },
	{ 0x30b3, 0x4b }, /* MIPI data rate = 900Mbps/ Lane */
	{ 0x30b4, 0x02 },
	{ 0x30b5, 0x04 },
	{ 0x30b6, 0x01 },
	{ 0x3104, 0xa1 },
	{ 0x3106, 0x01 },
	{ 0x3400, 0x04 },
	{ 0x3401, 0x00 },
	{ 0x3402, 0x04 },
	{ 0x3403, 0x00 },
	{ 0x3404, 0x04 },
	{ 0x3405, 0x00 },
	{ 0x3406, 0x01 },
	{ 0x3500, 0x00 },
	{ 0x3501, 0x20 },
	{ 0x3502, 0x80 },
	{ 0x3503, 0x07 },
	{ 0x3504, 0x00 },
	{ 0x3505, 0x30 },
	{ 0x3506, 0x00 },
	{ 0x3507, 0x10 },
	{ 0x3508, 0x80 },
	{ 0x3509, 0x10 },
	{ 0x350a, 0x00 },
	{ 0x350b, 0x38 },
	{ 0x3600, 0xb8 },
	{ 0x3601, 0x0a },
	{ 0x3602, 0x9c },
	{ 0x3604, 0x38 },
	{ 0x3612, 0x80 },
	{ 0x3620, 0x41 },
	{ 0x3621, 0xa4 },
	{ 0x3622, 0x0b },
	{ 0x3625, 0x44 },
	{ 0x3630, 0x55 },
	{ 0x3631, 0xd2 },
	{ 0x3632, 0x00 },
	{ 0x3633, 0x34 },
	{ 0x3634, 0x03 },
	{ 0x364d, 0x0d },
	{ 0x364f, 0x60 },
	{ 0x3660, 0x80 },
	{ 0x3662, 0x10 },
	{ 0x3665, 0x00 },
	{ 0x3666, 0x00 },
	{ 0x3667, 0x00 },
	{ 0x366a, 0x80 },
	{ 0x366c, 0x00 },
	{ 0x366d, 0x00 },
	{ 0x366e, 0x00 },
	{ 0x366f, 0x20 },
	{ 0x3680, 0xb5 },
	{ 0x3681, 0x00 },
	{ 0x3701, 0x14 },
	{ 0x3702, 0x50 },
	{ 0x3703, 0x8c },
	{ 0x3704, 0x68 },
	{ 0x3705, 0x02 },
	{ 0x3708, 0xe3 },
	{ 0x3709, 0x43 },
	{ 0x370a, 0x00 },
	{ 0x370b, 0x20 },
	{ 0x370c, 0x0c },
	{ 0x370d, 0x11 },
	{ 0x370e, 0x00 },
	{ 0x370f, 0x00 },
	{ 0x3710, 0x00 },
	{ 0x371c, 0x01 },
	{ 0x371f, 0x0c },
	{ 0x3721, 0x00 },
	{ 0x3724, 0x10 },
	{ 0x3726, 0x00 },
	{ 0x372a, 0x01 },
	{ 0x3730, 0x18 },
	{ 0x3738, 0x22 },
	{ 0x3739, 0xd0 },
	{ 0x373a, 0x50 },
	{ 0x373b, 0x02 },
	{ 0x373c, 0x20 },
	{ 0x373f, 0x02 },
	{ 0x3740, 0x42 },
	{ 0x3741, 0x02 },
	{ 0x3742, 0x18 },
	{ 0x3743, 0x01 },
	{ 0x3744, 0x02 },
	{ 0x3747, 0x10 },
	{ 0x374c, 0x04 },
	{ 0x3751, 0xf0 },
	{ 0x3752, 0x00 },
	{ 0x3753, 0x00 },
	{ 0x3754, 0xc0 },
	{ 0x3755, 0x00 },
	{ 0x3756, 0x1a },
	{ 0x3758, 0x00 },
	{ 0x3759, 0x0f },
	{ 0x375c, 0x04 },
	{ 0x3767, 0x01 },
	{ 0x376b, 0x44 },
	{ 0x3774, 0x10 },
	{ 0x3776, 0x00 },
	{ 0x377f, 0x08 },
	{ 0x3780, 0x22 },
	{ 0x3781, 0xcc },
	{ 0x3784, 0x2c },
	{ 0x3785, 0x08 },
	{ 0x3786, 0x16 },
	{ 0x378f, 0xf5 },
	{ 0x3791, 0xb0 },
	{ 0x3795, 0x00 },
	{ 0x3796, 0x94 },
	{ 0x3797, 0x11 },
	{ 0x3798, 0x30 },
	{ 0x3799, 0x41 },
	{ 0x379a, 0x07 },
	{ 0x379b, 0xb0 },
	{ 0x379c, 0x0c },
	{ 0x37c5, 0x00 },
	{ 0x37c6, 0xa0 },
	{ 0x37c7, 0x00 },
	{ 0x37c9, 0x00 },
	{ 0x37ca, 0x00 },
	{ 0x37cb, 0x00 },
	{ 0x37cc, 0x00 },
	{ 0x37cd, 0x00 },
	{ 0x37ce, 0x01 },
	{ 0x37cf, 0x00 },
	{ 0x37d1, 0x01 },
	{ 0x37de, 0x00 },
	{ 0x37df, 0x00 },
	{ 0x3800, 0x00 },
	{ 0x3801, 0x04 },
	{ 0x3802, 0x00 },
	{ 0x3803, 0x04 },
	{ 0x3804, 0x0c },
	{ 0x3805, 0xdb },
	{ 0x3806, 0x09 },
	{ 0x3807, 0xab },
	{ 0x3808, 0x0c },
	{ 0x3809, 0xd0 },
	{ 0x380a, 0x09 },
	{ 0x380b, 0xa0 },
	{ 0x380c, 0x0e },
	{ 0x380d, 0x28 },
	{ 0x380e, 0x09 },
	{ 0x380f, 0xe6 },
	{ 0x3810, 0x00 },
	{ 0x3811, 0x04 },
	{ 0x3812, 0x00 },
	{ 0x3813, 0x04 },
	{ 0x3814, 0x11 },
	{ 0x3815, 0x11 },
	{ 0x3820, 0x52 },
	{ 0x3821, 0x08 },
	{ 0x3823, 0x00 },
	{ 0x3824, 0x00 },
	{ 0x3825, 0x00 },
	{ 0x3826, 0x00 },
	{ 0x3827, 0x00 },
	{ 0x382a, 0x04 },
	{ 0x3a04, 0x09 },
	{ 0x3a05, 0xa9 },
	{ 0x3a06, 0x00 },
	{ 0x3a07, 0xf8 },
	{ 0x3b00, 0x00 },
	{ 0x3b02, 0x00 },
	{ 0x3b03, 0x00 },
	{ 0x3b04, 0x00 },
	{ 0x3b05, 0x00 },
	{ 0x3d00, 0x00 },
	{ 0x3d01, 0x00 },
	{ 0x3d02, 0x00 },
	{ 0x3d03, 0x00 },
	{ 0x3d04, 0x00 },
	{ 0x3d05, 0x00 },
	{ 0x3d06, 0x00 },
	{ 0x3d07, 0x00 },
	{ 0x3d08, 0x00 },
	{ 0x3d09, 0x00 },
	{ 0x3d0a, 0x00 },
	{ 0x3d0b, 0x00 },
	{ 0x3d0c, 0x00 },
	{ 0x3d0d, 0x00 },
	{ 0x3d0e, 0x00 },
	{ 0x3d0f, 0x00 },
	{ 0x3d80, 0x00 },
	{ 0x3d81, 0x00 },
	{ 0x3d84, 0x00 },
	{ 0x4000, 0x18 },
	{ 0x4001, 0x04 },
	{ 0x4002, 0x45 },
	{ 0x4004, 0x08 },
	{ 0x4005, 0x18 },
	{ 0x4006, 0x20 },
	{ 0x4008, 0x24 },
	{ 0x4009, 0x13 }, /* David@Optical, 10 */
	{ 0x404f, 0xA0 }, //change from 0x90 to 0xA0 by OV.
	{ 0x4100, 0x20 },
	{ 0x4101, 0x03 },
	{ 0x4102, 0x04 },
	{ 0x4103, 0x03 },
	{ 0x4104, 0x5a },
	{ 0x4307, 0x30 },
	{ 0x4315, 0x00 },
	{ 0x4511, 0x05 },
	{ 0x4512, 0x01 },
	{ 0x4805, 0x21 },
	{ 0x4806, 0x00 },
	{ 0x481f, 0x36 },
	{ 0x4826, 0x28 },
	{ 0x4831, 0x6c },
	{ 0x4837, 0x09 },
	{ 0x4a00, 0xaa },
	{ 0x4a03, 0x01 },
	{ 0x4a05, 0x08 },
	{ 0x4a0a, 0x88 },
	{ 0x4d03, 0xbb },
	{ 0x5000, 0x06 },
	{ 0x5001, 0x01 },
	{ 0x5002, 0x80 },
	{ 0x5003, 0x20 },
	{ 0x5013, 0x00 },
	{ 0x5046, 0x4a },
	{ 0x5780, 0x1c },
	{ 0x5786, 0x20 },
	{ 0x5787, 0x10 },
	{ 0x5788, 0x18 },
	{ 0x578a, 0x04 },
	{ 0x578b, 0x02 },
	{ 0x578c, 0x02 },
	{ 0x578e, 0x06 },
	{ 0x578f, 0x02 },
	{ 0x5790, 0x02 },
	{ 0x5791, 0xff },
	{ 0x5a08, 0x02 },
	{ 0x5e00, 0x00 },
	{ 0x5e10, 0x0c },

	{ OV8838_TABLE_END, 0x0000 }
};

/*
 * @@ 3280x2464 21fps Group 0 config and delay trigger
 * 3280x2464
 * ----------------------------------------------
 *  3280x2464_21fps_2lane key setting key setting
 *  System Clock 192MHz
 *  MIPI Clock 900Mhz
 *  FPS		:21  ( = System_Clock / (HTS*VTS) )
 *  HTS		:3624
 *  VTS		:2524
 *  Tline 	:18.875us  ( = HTS/System_Clock)
 *  Max exp line	:2512 (VTS-12)
 * ---------------------------------------------
 */
static struct ov8838_reg mode_3280x2464_2[] = {
//	{ 0x3208, 0x00 }, /* Group 0 hold start/Group Bank 0 */

#if 0
	{ 0x3011, 0x21 }, /* SC_MIPI_SC_CTRL0 */
	{ 0x3015, 0xc8 }, /* SC_MIPI_SC_CTRL2 */
	{ 0x3090, 0x01 }, /* PLL_PLL10 */
	{ 0x3091, 0x10 }, /* PLL_PLL11 */
	{ 0x3092, 0x01 }, /* PLL_PLL12 */
	{ 0x30b3, 0x4b }, /* PLL_MULTIPLIER */
	{ 0x30b4, 0x02 }, /* PLL_PL1_PRE_PLL_DIV */
	{ 0x3660, 0x80 }, /* one ANALOG CTRL */
	{ 0x3708, 0xe4 }, /* one SENSOR TIMING CTRL */
#endif

	{ 0x3500, 0x00 },
	{ 0x3501, 0x20 },
	{ 0x3502, 0xc0 },
	{ 0x350b, 0x38 },

	{ 0x3800, 0x00 }, /* TIMING_X_ADDR_START */
	{ 0x3801, 0x04 }, /* TIMING_X_ADDR_START */
	{ 0x3802, 0x00 }, /* TIMING_Y_ADDR_START */
	{ 0x3803, 0x04 }, /* TIMING_Y_ADDR_START */
	{ 0x3804, 0x0c }, /* TIMING_X_ADDR_END */
	{ 0x3805, 0xdb }, /* TIMING_X_ADDR_END */
	{ 0x3806, 0x09 }, /* TIMING_Y_ADDR_END */
	{ 0x3807, 0xab }, /* TIMING_Y_ADDR_END */
	{ 0x3808, 0x0c }, /* TIMING_X_OUTPUT_SIZE */
	{ 0x3809, 0xd0 }, /* TIMING_X_OUTPUT_SIZE */
	{ 0x380a, 0x09 }, /* TIMING_Y_OUTPUT_SIZE */
	{ 0x380b, 0xa0 }, /* TIMING_Y_OUTPUT_SIZE */
	{ 0x380c, 0x0e }, /* TIMING_HTS */
	{ 0x380d, 0x28 }, /* TIMING_HTS */
	{ 0x380e, 0x09 }, /* TIMING_VTS */
	{ 0x380f, 0xdc }, /* TIMING_VTS */
	{ 0x3810, 0x00 }, /* TIMING_ISP_X_WIN */
	{ 0x3811, 0x04 }, /* TIMING_ISP_X_WIN */
	{ 0x3812, 0x00 }, /* TIMING_ISP_Y_WIN */
	{ 0x3813, 0x04 }, /* TIMING_ISP_Y_WIN */

#if 0
	{ 0x3814, 0x11 }, /* TIMING_X_INC */
	{ 0x3815, 0x11 }, /* TIMING_Y_INC */
	{ 0x3820, 0x52 }, /* TIMING FORMAT CTRL */
	{ 0x3821, 0x08 }, /* TIMING FORMAT CTRL */
#endif
	{ 0x3a04, 0x09 },
	{ 0x3a05, 0xa9 },
#if 0
	{ 0x4058, 0x00 },
	{ 0x4512, 0x01 }, /* INPUT_SWAP_MAN_EN */
	{ 0x4837, 0x09 }, /* PCLK_PERIOD */
	{ 0x350b, 0x35 }, /* AEC AGC ADJ */
#endif

	{ 0x350b, 0x35 }, /* AEC AGC ADJ */

	{ 0x3016, 0xf9 }, /* SC_CLKRST0 */
	{ 0x3016, 0xf0 },
//	{ 0x3208, 0x10 }, /* Group 0 hold end/Group Bank 0 */
//	{ 0x3208, 0xa0 }, /* Group0 Trigger */

	{ OV8838_TABLE_END, 0x0000 }
};

/*
 * @@ 3088x1736 29.9fps Group 1 config and delay trigger
 * ----------------------------------------------
 *  3088x1736_29.9fps_2lane setting
 *  SystemCLK     :192Mhz
 *  MIPI Clock 900Mhz
 *  FPS		:29.9  ( = System_Clock / (HTS*VTS) )
 *  HTS		:3624
 *  VTS		:1772
 *  Tline 	:18.875us  ( = HTS/System_Clock)
 *  Max exp line	:1760 (VTS-12)
 * ---------------------------------------------
 */
static struct ov8838_reg mode_3088x1736_2[] = {
	/* resolution setting for default */
//	{ 0x3208, 0x01 }, /* Group 1 hold start/Group Bank 1 */

	{ 0x3500, 0x00 },
	{ 0x3501, 0x20 },
	{ 0x3502, 0xc0 },
	{ 0x350b, 0x38 },

	{ 0x3800, 0x00 },
	{ 0x3801, 0x64 },
	{ 0x3802, 0x01 },
	{ 0x3803, 0x70 },
	{ 0x3804, 0x0c },
	{ 0x3805, 0x7b },
	{ 0x3806, 0x08 },
	{ 0x3807, 0x3f },
	{ 0x3808, 0x0c },
	{ 0x3809, 0x10 },
	{ 0x380a, 0x06 },
	{ 0x380b, 0xc8 },
	{ 0x380c, 0x0e },
	{ 0x380d, 0x28 },
	{ 0x380e, 0x06 },
	{ 0x380f, 0xec },
	{ 0x3810, 0x00 },
	{ 0x3811, 0x04 },
	{ 0x3812, 0x00 },
	{ 0x3813, 0x04 },

	{ 0x3a04, 0x07 },
	{ 0x3a05, 0x49 },

	{ 0x3016, 0xf9 },
	{ 0x3016, 0xf0 },
//	{ 0x3208, 0x11 }, /* Group 1 hold end/Group Bank 1 */
//	{ 0x3208, 0xa1 },

	{ OV8838_TABLE_END, 0x0000 }
};

/*
 * @@ 1920x1080 46fps Group 2 config and delay trigger
 * ----------------------------------------------
 *  1920x1080_46fps_2lane setting
 *  SystemCLK     :192Mhz
 *  FPS	        :46
 *  HTS		:3624(R380c:R380d)
 *  VTS		:1152(R380e:R380f)
 *  Tline 	:13.64us
 *  Max exp line	:1140 ( VTS-12)
 * ---------------------------------------------
 */
static struct ov8838_reg mode_1920x1080_2[] = {
//	{ 0x3208, 0x02 } , /* Group 2 hold start/Group Bank 2 */

	{ 0x3500, 0x00 },
	{ 0x3501, 0x20 },
	{ 0x3502, 0xc0 },
	{ 0x350b, 0x38 },

	{ 0x3800, 0x02 },
	{ 0x3801, 0xac },
	{ 0x3802, 0x02 },
	{ 0x3803, 0xb8 },
	{ 0x3804, 0x0a },
	{ 0x3805, 0x33 },
	{ 0x3806, 0x06 },
	{ 0x3807, 0xf7 },
	{ 0x3808, 0x07 },
	{ 0x3809, 0x80 },
	{ 0x380a, 0x04 },
	{ 0x380b, 0x38 },
	{ 0x380c, 0x0e },
	{ 0x380d, 0x28 },
	{ 0x380e, 0x04 },
	{ 0x380f, 0x80 },
	{ 0x3810, 0x00 },
	{ 0x3811, 0x04 },
	{ 0x3812, 0x00 },
	{ 0x3813, 0x04 },
	{ 0x3a04, 0x07 },
	{ 0x3a05, 0x49 },
	{ 0x350b, 0x35 },

	{ 0x3016, 0xf9 },
	{ 0x3016, 0xf0 },
//	{ 0x3208, 0x12 }, /* Group 2 hold end/Group Bank 2 */
//	{ 0x3208, 0xa2 },

	{ OV8838_TABLE_END, 0x0000 }
};

/*
 * 1640x510
 * ---------------------------------------------
 *  1640x510_96fps_2lane scale key setting
 *  System Clock 192MHz
 *  MIPI Clock 900Mhz
 *  FPS		:96 ( = System_Clock / (HTS*VTS) )
 *  HTS		:3624
 *  VTS		:552
 *  Tline 	:13.64us ( = HTS/System_Clock)
 *  Max exp line	:540 (VTS-12)
 * ---------------------------------------------
 */
static struct ov8838_reg mode_1640x510_2[] = {
//	{ 0x3208, 0x03 }, /* Group 2 hold start/Group Bank 3 */

	{ 0x3500, 0x00 },
	{ 0x3501, 0x20 },
	{ 0x3502, 0xc0 },
	{ 0x350b, 0x38 },

	{ 0x3800, 0x03 },
	{ 0x3801, 0x38 },
	{ 0x3802, 0x03 },
	{ 0x3803, 0xd4 },
	{ 0x3804, 0x09 },
	{ 0x3805, 0xa7 },
	{ 0x3806, 0x05 },
	{ 0x3807, 0xd9 },
	{ 0x3808, 0x06 },
	{ 0x3809, 0x68 },
	{ 0x380a, 0x01 },
	{ 0x380b, 0xfe },
	{ 0x380c, 0x0e },
	{ 0x380d, 0x28 },
	{ 0x380e, 0x02 },
	{ 0x380f, 0x28 },
	{ 0x3810, 0x00 },
	{ 0x3811, 0x04 },
	{ 0x3812, 0x00 },
	{ 0x3813, 0x04 },

	{ 0x3a04, 0x04 },
	{ 0x3a05, 0xc9 },
	{ 0x350b, 0x33 },

	{ 0x3016, 0xf9 },
	{ 0x3016, 0xf0 },
//	{ 0x3208, 0x13 }, /* Group 3 hold end/Group Bank 3 */
//	{ 0x3208, 0xa3 },

	{ OV8838_TABLE_END, 0x0000 }
};

static struct ov8838_reg mode_end[] = {
	{ OV8838_TABLE_WAIT_MS, 1 },
	{ 0x0100, 0x01 },

	{ OV8838_TABLE_END, 0x0000 }
};


/* all mode settings */
static struct rawchip_sensor_data *rawchip_mode_table = rawchip_mode_table_1;
static struct ov8838_reg *mode_start = mode_start_1;
static struct ov8838_reg *mode_table[] = {
	[OV8838_MODE_3280x2464] = mode_3280x2464,
	[OV8838_MODE_3280x1856] = mode_3280x1856,
	[OV8838_MODE_3280x1856_EVITA] = mode_3280x1856_evita,
	[OV8838_MODE_1920x1080] = mode_1920x1080,
	[OV8838_MODE_1640x510] = mode_1640x510,
};


/*
 * -------------
 * | functions |
 * -------------
 */

/* 2 regs to program frame length */
static inline void ov8838_get_frame_length_regs(struct ov8838_reg *regs,
						u32 frame_length)
{
	regs->addr = 0x380e;
	regs->val = (frame_length >> 8) & 0xff;
	(regs + 1)->addr = 0x380f;
	(regs + 1)->val = (frame_length) & 0xff;
}

/* 2 regs to program coarse time */
static inline void ov8838_get_coarse_time_regs(struct ov8838_reg *regs,
						u32 coarse_time)
{
	regs->addr = 0x3500;
	regs->val = (coarse_time >> 12) & 0xf;
	(regs + 1)->addr = 0x3501;
	(regs + 1)->val = (coarse_time >> 4) & 0xff;
	(regs + 2)->addr = 0x3502;
	(regs + 2)->val = (coarse_time & 0xf) << 4;
}

/* 2 reg to program gain */
static inline void ov8838_get_gain_reg(struct ov8838_reg *regs, u16 gain)
{
	regs->addr = 0x350b;
	regs->val = gain;
}

static int ov8838_i2c_rxdata(unsigned short saddr,
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

	if (i2c_transfer(ov8838_i2c_client->adapter, msgs, 2) < 0) {
		pr_err("[CAM] ov8838_i2c_rxdata failed!\n");
		return -EIO;
	}

	return 0;
}

static int32_t ov8838_i2c_read(unsigned short raddr,
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
	rc = ov8838_i2c_rxdata(ov8838_i2c_client->addr, buf, rlen);

	if (rc < 0) {
		pr_err("[CAM] ov8838_i2c_read 0x%x failed!\n", raddr);
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

#ifdef KERNEL_SENSOR_DEBUG
static int ov8838_read_reg(struct i2c_client *client, u16 addr, u8 *val)
{
	int err;
	struct i2c_msg msg[2];
	unsigned char data[4] = {0};

	if (!client->adapter)
		return -ENODEV;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = data;

	/* high byte goes out first */
	data[0] = (u8) (addr >> 8);;
	data[1] = (u8) (addr & 0xff);

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 2;
	msg[1].buf = data + 2;

	err = i2c_transfer(client->adapter, msg, 2);

	if (err != 2)
		return -EINVAL;

	*val = data[2];
	pr_info("[CAM] ov8838 read_reg 0x%02x%02x 0x%2x\n", data[0], data[1], data[2]);

	return 0;
}
#endif

static int ov8838_check_sensorid(void)
{
	uint16_t chipid = 0;
	int32_t rc = 0;

	/* Read sensor Model ID: */
	rc = ov8838_i2c_read(OV8838_REG_MODEL_ID, &chipid, 2);
	if (rc < 0) {
		pr_err("[CAM] read sensor id fail\n");
		rc = EIO;
	}

	/* Compare sensor ID to OV8838 ID: */
/*	pr_info("[CAM]%s, Expected id=0x%x\n", __func__, OV8838_MODEL_ID);
	pr_info("[CAM]%s, Read id=0x%x\n", __func__, chipid); */

	if (chipid != OV8838_MODEL_ID) {
		pr_err("[CAM] sensor model id is incorrect\n");
		rc = -ENODEV;
	}
	return rc;
}

static int ov8838_write_reg(struct i2c_client *client, u16 addr, u8 val)
{
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
		pr_err("[CAM] ov8838: i2c transfer failed, retrying %x %x\n",
			addr, val);
		msleep(3);
	} while (retry <= OV8838_MAX_RETRIES);

	return err;
}

static int ov8838_write_bulk_reg(struct i2c_client *client, u8 *data, int len)
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

	pr_err("[CAM] ov8838: i2c bulk transfer failed at %x\n",
		(int)data[0] << 8 | data[1]);

	return err;
}

static int ov8838_write_table(struct i2c_client *client,
				const struct ov8838_reg table[],
				const struct ov8838_reg override_list[],
				int num_override_regs)
{
	int err;
	const struct ov8838_reg *next, *n_next;
	u8 *b_ptr = info->i2c_trans_buf;
	unsigned int buf_filled = 0;
	int i;
	u16 val;

	for (next = table; next->addr != OV8838_TABLE_END; next++) {
		if (next->addr == OV8838_TABLE_WAIT_MS) {
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
		if (n_next->addr != OV8838_TABLE_END &&
			n_next->addr != OV8838_TABLE_WAIT_MS &&
			buf_filled < SIZEOF_I2C_TRANSBUF &&
			n_next->addr == next->addr + 1) {
			continue;
		}

		/*wait for rawchip clock is done*/
		while (!clock_init_done) {
			msleep(5);
		}

		err = ov8838_write_bulk_reg(info->i2c_client, info->i2c_trans_buf, buf_filled);
		if (err)
			return err;

		buf_filled = 0;
	}
	return 0;
}

static int ov8838_reset(struct ov8838_info *info)
{
	int err = 0;
	pr_info("[CAM] ov8838_reset +++\n");
	if ( info->pdata->use_rawchip )
		rawchip_release();
	info->pdata->power_off();
	msleep(50);
	info->pdata->power_on();
	if ( info->pdata->use_rawchip )
		rawchip_open_init();
	/*sensor stream off*/
	pr_info("[CAM] %s ov8838_write_table reset_seq\n", __func__);
	err = ov8838_write_table(info->i2c_client, reset_seq, NULL, 0);
	if (err)
		pr_err("[CAM] %s set sensor stream off failed!!!\n", __func__);

	/*set sensor init*/
	pr_info("[CAM] %s ov8838_write_table mode_start\n", __func__);
	err = ov8838_write_table(info->i2c_client, mode_start, NULL, 0);
	if (err)
		pr_err("[CAM] %s set sensor init failed!!\n", __func__);

	pr_info("[CAM] ov8838_reset ---\n");
	return err;
}

static int ov8838_get_mode(struct ov8838_mode * mode)
{
	pr_info("[CAM] %s: xres %u yres %u framelength %u coarsetime %u gain %u\n",
		__func__, mode->xres, mode->yres, mode->frame_length,
		mode->coarse_time, mode->gain);
	if (mode->xres == 3280 && mode->yres == 2464)
	{
		pr_info("[CAM] set mode to (%d) -> OV8838_MODE_3280x2464\n", OV8838_MODE_3280x2464);
		return OV8838_MODE_3280x2464;
	}
	else if (mode->xres == 3280 && mode->yres == 1856)
	{
		pr_info("[CAM] set mode to (%d) -> OV8838_MODE_3280x1856\n", OV8838_MODE_3280x1856);
		if (machine_is_evitareul())
			return OV8838_MODE_3280x1856_EVITA;
		else
			return OV8838_MODE_3280x1856;
	}
	else if (mode->xres == 1640 && mode->yres == 510)
	{
		pr_info("[CAM] set mode to (%d) -> OV8838_MODE_1640x510\n", OV8838_MODE_1640x510);
		return OV8838_MODE_1640x510;
	}
	else
	{
		pr_err("[CAM] %s: invalid resolution supplied to set mode %d %d\n",__func__, mode->xres, mode->yres);
		return -1;
	}

}

static int ov8838_set_mode(struct ov8838_info *info, struct ov8838_mode *mode)
{
	int sensor_mode = -1;
	int err = -1;
	struct ov8838_reg reg_list[6];
	struct rawchip_sensor_data rawchip_data;
	int retry = 0;
	struct clk *spi_clk = NULL;
	struct clk *emc_clk = NULL;

	pr_info("[CAM] %s++\n", __func__);

	if (mode)
	{
		sensor_mode = ov8838_get_mode(mode);
		if( sensor_mode != -1 )
			init_sensor_mode = sensor_mode;
	} else {

		if ( init_sensor_mode == OV8838_MODE_3280x1856 && machine_is_evitareul() )
			init_sensor_mode = OV8838_MODE_3280x1856_EVITA;

		sensor_mode = init_sensor_mode;
		pr_info("[CAM] %s: select sensor mode with init_sensor_mode(%d)\n", __func__,init_sensor_mode);
	}

	if (sensor_mode == info->mode)
		goto ov8838_set_mode_done;

	if ( info->pdata->use_rawchip && !FirstTimeSetMode ) {

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

		spi_clk = tegra_get_clock_by_name("sbc4");
		if (IS_ERR_OR_NULL(spi_clk)) {
			pr_err("[CAM] %s: couldn't get spi clock\n", __func__);
			spi_clk = NULL;
		}

		if (spi_clk) {
			pr_info("[CAM] set spi4 clock to 24MHz\n");
			clk_set_rate(spi_clk, 96000000);
		}

		if ( info->pdata->rawchip_need_powercycle && rawchip_mode_table[info->mode].pixel_clk != rawchip_mode_table[sensor_mode].pixel_clk ){
			ov8838_reset(info);
		}
retry:
		/* do rawchip setting before sensor setting */
		pr_info("[CAM] call rawchip_set_size(%d) +++\n", sensor_mode);
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
		rawchip_data.use_rawchip = info->pdata->use_rawchip;
		err = rawchip_set_size(rawchip_data, &clock_init_done);
		if (err < 0 && retry++ < 5) {
			pr_err("[CAM] rawchip_set_size FAIL!!!\n");
			ov8838_reset(info);
			goto retry;
		}

    	if(spi_clk)
		{
			clk_set_rate(spi_clk, 60000000);  /* 15MHz */
			pr_info("[CAM] set spi4 clock to 15MHz\n");
		}

		if(emc_clk)
		{
			clk_set_rate(emc_clk, 65000000);  /* 15MHz */
			pr_info("[CAM] set emc clock to 437MHz\n");
		}
	}

	/* get a list of override regs for the asking frame length, */
	/* coarse integration time, and gain.                       */
	if(mode)
	{
		ov8838_get_frame_length_regs(reg_list, mode->frame_length);
		ov8838_get_coarse_time_regs(reg_list + 2, mode->coarse_time);
		ov8838_get_gain_reg(reg_list + 5, mode->gain);
	} else {
		// AE breakdown 875*45 == 1240*32 sensor default
		ov8838_get_coarse_time_regs(reg_list, 0x4E2); //HTC_Optical: modify for initial AE
		ov8838_get_gain_reg(reg_list + 3, 0x20); //HTC_Optical: modify for initial AE
	}

	/*sensor stream off*/
	/*err = ov8838_write_table(info->i2c_client, reset_seq, NULL, 0);
	if (err)
		goto error;*/

	/*set sensor config*/
	if(mode)
		err = ov8838_write_table(info->i2c_client, mode_table[sensor_mode],reg_list, 6);
	else
		err = ov8838_write_table(info->i2c_client, mode_table[sensor_mode],reg_list, 4);

	if (err)
		goto ov8838_set_mode_error;

	/*make sure that the rawchip is initialized before stream on*/
	cancel_work_sync(&info->rawchip_work);

	/*stream on*/
	err = ov8838_write_table(info->i2c_client, mode_end, NULL, 0);
	if (err)
		goto ov8838_set_mode_error;

ov8838_set_mode_done:
	info->mode = sensor_mode;
	FirstTimeSetMode = 0;
	pr_info("[CAM] ov8838 stream on!\n");

	pr_info("[CAM] %s --\n", __func__);

	return 0;

ov8838_set_mode_error:
	FirstTimeSetMode = 0;
	pr_err("[CAM] %s failed!!!\n", __func__);

	return err;
}

static void rawchip_handler(struct work_struct *work)
{
	int sensor_mode;
	int err = -1;
	int retry = 0;
	struct clk *spi_clk = NULL;
	struct rawchip_sensor_data rawchip_data;
	struct ov8838_info *info;

	pr_info("[CAM] %s++\n", __func__);

	info = container_of(work, struct ov8838_info,
			rawchip_work);

	mutex_lock(&info->rawchip_lock);

	if ( init_sensor_mode == OV8838_MODE_3280x1856 && machine_is_evitareul() )
		init_sensor_mode = OV8838_MODE_3280x1856_EVITA;

	sensor_mode = init_sensor_mode;

	pr_info("[CAM] %s: select sensor mode(%d)\n", __func__,sensor_mode);
	spi_clk = tegra_get_clock_by_name("sbc4");
	if (IS_ERR_OR_NULL(spi_clk)) {
		pr_err("[CAM] %s: couldn't get spi clock\n", __func__);
		spi_clk = NULL;
	}

	if (spi_clk) {
		pr_debug("[CAM] set spi4 clock to 24MHz\n");
		clk_set_rate(spi_clk, 96000000);
	}

retry:
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
	rawchip_data.use_rawchip = info->pdata->use_rawchip;
	err = rawchip_set_size(rawchip_data, &clock_init_done);
	if (err < 0 && retry++ < 5) {
		pr_err("[CAM] rawchip_set_size FAIL!!!\n");
		ov8838_reset(info);
		goto retry;
	}

	if(spi_clk)
	{
		clk_set_rate(spi_clk, 60000000);  /* 15MHz */
		pr_info("[CAM] set spi4 clock to 15MHz\n");
	}

	mutex_unlock(&info->rawchip_lock);
	pr_info("[CAM] %s--\n", __func__);
}

static void set_mode_handler(struct work_struct *work)
{
	struct ov8838_info *info;
	int err;

	pr_debug("[CAM] %s ++\n", __func__);
	info = container_of(work, struct ov8838_info,
			set_mode_work);
	mutex_lock(&info->camera_lock);

	if (info->pdata && info->pdata->power_on) {
		info->pdata->power_on();
	}
	if ( info->pdata->use_rawchip )
		rawchip_open_init();

	if((err = ov8838_check_sensorid())!=0)
	{
		FirstTimeSetMode = 0;
		pr_err("[CAM] %s reading sensor id failed\n", __func__);
		goto ov8838_set_mode_handler_error;
	}

	/*schedule rawchip work thread*/
	clock_init_done = 0;
	FirstTimeSetMode = 1;
	schedule_work(&info->rawchip_work);

	/*sensor stream off*/
	pr_debug("[CAM] %s sensor stream off, ov8838_write_table reset_seq",__func__);
	err = ov8838_write_table(info->i2c_client, reset_seq, NULL, 0);
	if (err)
		pr_err("[CAM] set sensor stream off failed!!!\n");

	/*set sensor init*/
	pr_debug("[CAM] %s set sensor init, ov8838_write_table mode_start",__func__);
	err = ov8838_write_table(info->i2c_client, mode_start, NULL, 0);
	if (err)
		pr_err("[CAM] set sensor init failed!!!\n");

	err = ov8838_set_mode(info, NULL);
	if (err)
		pr_err("[CAM] set sensor mode failed!!!\n");

ov8838_set_mode_handler_error:

	info->power_state = true;
	mutex_unlock(&info->camera_lock);
	pr_debug("[CAM] %s --\n", __func__);
}


static int ov8838_set_frame_length(struct ov8838_info *info, u32 frame_length)
{
	int ret;
	struct ov8838_reg reg_list[2];
	u8 *b_ptr = info->i2c_trans_buf;

	ov8838_get_frame_length_regs(reg_list, frame_length);

	*b_ptr++ = reg_list[0].addr >> 8;
	*b_ptr++ = reg_list[0].addr & 0xff;
	*b_ptr++ = reg_list[0].val & 0xff;
	*b_ptr++ = reg_list[1].val & 0xff;
	ret = ov8838_write_bulk_reg(info->i2c_client, info->i2c_trans_buf, 4);

	return 0;
}

static int ov8838_set_coarse_time(struct ov8838_info *info, u32 coarse_time)
{
	int ret;
	struct ov8838_reg reg_list[3];
	u8 *b_ptr = info->i2c_trans_buf;

	ov8838_get_coarse_time_regs(reg_list, coarse_time);

	*b_ptr++ = reg_list[0].addr >> 8;
	*b_ptr++ = reg_list[0].addr & 0xff;
	*b_ptr++ = reg_list[0].val & 0xff;
	*b_ptr++ = reg_list[1].val & 0xff;
	*b_ptr++ = reg_list[2].val & 0xff;
	ret = ov8838_write_bulk_reg(info->i2c_client, info->i2c_trans_buf, 5);

	return 0;
}

static int ov8838_set_gain(struct ov8838_info *info, u16 gain)
{
	int ret;
	struct ov8838_reg reg_list[1];
	u8 *b_ptr = info->i2c_trans_buf;

	ov8838_get_gain_reg(reg_list, gain);

	*b_ptr++ = reg_list[0].addr >> 8;
	*b_ptr++ = reg_list[0].addr & 0xff;
	*b_ptr++ = reg_list[0].val & 0xff;
	ret = ov8838_write_bulk_reg(info->i2c_client, info->i2c_trans_buf, 3);

	return ret;
}

static int ov8838_set_group_hold(struct ov8838_info *info,
	struct ov8838_ae *ae)
{
	int ret = 0;

	bool groupHoldEnabled = ae->gain_enable | ae->coarse_time_enable | ae->frame_length_enable;

	if (groupHoldEnabled) {
		ret = ov8838_write_reg(info->i2c_client, 0x3208, 0x0);
		if (ret)
			return ret;
	}

	if (ae->gain_enable)
		ret = ov8838_set_gain(info, ae->gain);
	if (ae->coarse_time_enable)
		ret = ov8838_set_coarse_time(info, ae->coarse_time);
	if (ae->frame_length_enable)
		ret = ov8838_set_frame_length(info, ae->frame_length);

	if (groupHoldEnabled) {
		ret = ov8838_write_reg(info->i2c_client, 0x3208, 0x10);
		if (ret)
			return ret;
		ret = ov8838_write_reg(info->i2c_client, 0x3208, 0xa0);
		if (ret)
			return ret;
	}

	return ret;
}

#if 0
static int ov8838_test_pattern(struct ov8838_info *info,
			       enum ov8838_test_pattern pattern)
{
	if (pattern >= ARRAY_SIZE(test_pattern_modes))
		return -EINVAL;

	return ov8838_write_table(info,
				  test_pattern_modes[pattern],
				  NULL, 0);
}
#endif

static long ov8838_ioctl(struct file *file,
			 unsigned int cmd, unsigned long arg)
{
	struct ov8838_info *info = file->private_data;
	struct ov8838_mode mode;

	int ret;

	pr_debug("[CAM] %s ++%x\n", __func__, cmd);

	while (!info->power_state){
		pr_debug("[CAM] %s power is not on, let's wait\n", __func__);
		msleep(5);
	};

	switch (cmd) {
	case OV8838_IOCTL_SET_MODE:
	{
		if (copy_from_user(&mode,
				   (const void __user *)arg,
				   sizeof(struct ov8838_mode))) {
			pr_err("[CAM] %s %d SET_MODE return -EFAULT\n", __func__, __LINE__);
			return -EFAULT;
		}
		pr_debug("[CAM] %s: OV8838_IOCTL_SET_MODE\n", __func__);
		mutex_lock(&info->camera_lock);
		ret = ov8838_set_mode(info, &mode);
		mutex_unlock(&info->camera_lock);
		return ret;
	}
	case OV8838_IOCTL_SET_FRAME_LENGTH:
		pr_debug("[CAM] %s: OV8838_IOCTL_SET_FRAME_LENGTH\n", __func__);
		mutex_lock(&info->camera_lock);
		ret = ov8838_set_frame_length(info, (u32)arg);
		mutex_unlock(&info->camera_lock);
		return ret;
	case OV8838_IOCTL_SET_COARSE_TIME:
		pr_debug("[CAM] %s: OV8838_IOCTL_SET_COARSE_TIME\n", __func__);
		mutex_lock(&info->camera_lock);
		ret = ov8838_set_coarse_time(info, (u32)arg);
		mutex_unlock(&info->camera_lock);
		return ret;
	case OV8838_IOCTL_SET_GAIN:
		pr_debug("[CAM] %s: OV8838_IOCTL_SET_GAIN\n", __func__);
		mutex_lock(&info->camera_lock);
		ret = ov8838_set_gain(info, (u16)arg);
		mutex_unlock(&info->camera_lock);
		return ret;
	case OV8838_IOCTL_GET_STATUS:
	{
		u16 status = 0;
		pr_debug("[CAM] %s: OV8838_IOCTL_GET_STATUS\n", __func__);
		if (copy_to_user((void __user *)arg, &status,
				 2)) {
			pr_err("[CAM] %s %d GET_STATUS return -EFAULT\n", __func__, __LINE__);
			return -EFAULT;
		}
		return 0;
	}
	case OV8838_IOCTL_SET_GROUP_HOLD:
	{
		struct ov8838_ae ae;
		if (copy_from_user(&ae,
				(const void __user *)arg,
				sizeof(struct ov8838_ae))) {
			pr_err("[CAM] %s %d SET_GROUP_HOLD return -EFAULT\n", __func__, __LINE__);
			return -EFAULT;
		}
		pr_debug("[CAM] %s: OV8838_IOCTL_SET_GROUP_HOLD\n", __func__);
		mutex_lock(&info->camera_lock);
		ret = ov8838_set_group_hold(info, &ae);
		mutex_unlock(&info->camera_lock);
		return ret;
	}
#if 0
	case OV8838_IOCTL_TEST_PATTERN:
	{
		err = ov8838_test_pattern(info, (enum ov8838_test_pattern) arg);
		if (err)
			pr_err("%s %d %d\n", __func__, __LINE__, err);
		return err;
	}
#endif
	case OV8838_IOCTL_GET_INIT_STATE:
	{
		struct ov8838_init_state init_state;
		init_state.use_rawchip = info->pdata->use_rawchip;
		if (copy_to_user((void __user *)arg, &init_state,
					sizeof(struct ov8838_init_state))) {
			pr_debug("[CAM] %s %d\n", __func__, __LINE__);
			return -EFAULT;
		}
		return 0;
	}
	case OV8838_IOCTL_RESET_RAWCHIP:
	{
		pr_info("[CAM] %s: OV8838_IOCTL_RESET_RAWCHIP\n", __func__);

		if (copy_from_user(&mode,
				   (const void __user *)arg,
				   sizeof(struct ov8838_mode))) {
			pr_err("[CAM] %s %d RESET_RAWCHIP return -EFAULT\n", __func__, __LINE__);
			return -EFAULT;
		}

		mutex_lock(&info->camera_lock);
		tegra_rawchip_block_iotcl(TRUE);

		/*stream off*/
		ret = ov8838_write_table(info->i2c_client, stream_off_mipi_data, NULL, 0);
		if (ret)
			pr_err("[CAM] set sensor stream off mipi data failed!!!\n");

		ret = ov8838_write_table(info->i2c_client, reset_seq, NULL, 0);
		if (ret)
			pr_err("[CAM] set sensor stream off mipi data failed!!!\n");

		Reset_Yushan();
		mdelay(10);

		/*stream on*/
		ret = ov8838_write_table(info->i2c_client, mode_end, NULL, 0);
		if (ret)
			pr_err("[CAM] set sensor stream on failed!!!\n");

		ret = ov8838_write_table(info->i2c_client, stream_on_mipi_data, NULL, 0);
		if (ret)
			pr_err("[CAM] %s set sensor stream on mipi data failed!!!", __func__);

		tegra_rawchip_block_iotcl(FALSE);
		mutex_unlock(&info->camera_lock);
		return ret;
	}
	default:
		return -EINVAL;
	}
	return 0;
}

static int ov8838_open(struct inode *inode, struct file *file)
{
	pr_debug("[CAM] %s ++\n", __func__);

	mutex_lock(&info->camera_lock);
	/* pr_info(KERN_INFO "START_TIME: Entering %s", __func__); */
	file->private_data = info;

	info->power_state = false;
	// If mode is NULL, set_mode will be executed.
	info->mode = 0;
	pr_debug("[CAM] %s schedule_work(set_mode_work)\n", __func__);
	schedule_work(&info->set_mode_work);
	//ov8838_get_status(info, &status);
	/* pr_info(KERN_INFO "START_TIME: Leaving %s", __func__); */

	mutex_unlock(&info->camera_lock);
	pr_debug("[CAM] %s --\n", __func__);
	return 0;
}

int ov8838_release(struct inode *inode, struct file *file)
{
	struct clk *spi_clk = NULL;

	pr_info("[CAM] %s ++\n", __func__);

	mutex_lock(&info->camera_lock); /* use camera_lock to prevent  releasing camera when set_mode */
	if ( info->pdata->use_rawchip )
		rawchip_release();
	if (info->pdata && info->pdata->power_off)
		info->pdata->power_off();
	file->private_data = NULL;
	info->power_state = false;

	pr_info("[CAM] %s cancel_work_sync(set_mode_work)\n", __func__);
	cancel_work_sync(&info->set_mode_work);

	cancel_work_sync(&info->rawchip_work);

	spi_clk = tegra_get_clock_by_name("sbc4");
	if (IS_ERR_OR_NULL(spi_clk)) {
		pr_err("[CAM] %s: couldn't get spi clock\n", __func__);
		spi_clk = NULL;
	}
	else
	{
		clk_set_rate(spi_clk, 96000000);
		pr_info("[CAM] set spi4 clock back to 24MHz");
	}

	mutex_unlock(&info->camera_lock);

	pr_info("[CAM] %s --\n", __func__);
	return 0;
}


static const struct file_operations ov8838_fileops = {
	.owner = THIS_MODULE,
	.open = ov8838_open,
	.unlocked_ioctl = ov8838_ioctl,
	.release = ov8838_release,
};

static struct miscdevice ov8838_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ov8838",
	.fops = &ov8838_fileops,
};

/* HTC START */
static const char *OV8838Vendor = "omnivision";
static const char *OV8838NAME = "ov8838";
static const char *OV8838Size = "8M";
static uint32_t htcwc_value;

static ssize_t sensor_vendor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;

	sprintf(buf, "%s %s %s\n", OV8838Vendor, OV8838NAME, OV8838Size);
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

static ssize_t sensor_read_data_lane(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t length;
	length = sprintf(buf, "%d\n", info->pdata->data_lane);
	return length;
}

static DEVICE_ATTR(sensor, 0444, sensor_vendor_show, NULL);
static DEVICE_ATTR(htcwc, 0664, htcwc_get, htcwc_set);
static DEVICE_ATTR(node, 0444, sensor_read_node, NULL);
static DEVICE_ATTR(data_lane, 0444, sensor_read_data_lane, NULL);

static struct kobject *android_ov8838;

static int ov8838_sysfs_init(void)
{
	int ret ;
	pr_info("[CAM] %s: kobject creat and add\n", __func__);
	android_ov8838 = kobject_create_and_add("android_camera", NULL);
	if (android_ov8838 == NULL) {
		pr_info("[CAM] %s: subsystem_register " \
		"failed\n", __func__);
		ret = -ENOMEM;
		return ret ;
	}
	pr_info("[CAM] %s:sysfs_create_file\n", __func__);
	ret = sysfs_create_file(android_ov8838, &dev_attr_sensor.attr);
	if (ret) {
		pr_info("[CAM] %s: sysfs_create_file " \
		"failed\n", __func__);
		kobject_del(android_ov8838);
	}

	ret = sysfs_create_file(android_ov8838, &dev_attr_htcwc.attr);
	if (ret) {
		pr_info("[CAM] %s: sysfs_create_file htcwc failed\n", __func__);
		kobject_del(android_ov8838);
	}

	ret = sysfs_create_file(android_ov8838, &dev_attr_node.attr);
	if (ret) {
		pr_info("[CAM] %s: dev_attr_node failed\n", __func__);
		kobject_del(android_ov8838);
	}

	ret = sysfs_create_file(android_ov8838, &dev_attr_data_lane.attr);
	if (ret) {
		pr_info("[CAM] %s: dev_attr_data_lane failed\n", __func__);
		kobject_del(android_ov8838);
	}

	return 0 ;
}

/**
 *  LED status
 */

/* Andrew_Cheng linear led */
struct camera_led_info {
	uint16_t enable;
	uint16_t low_limit_led_state;
	uint16_t max_led_current_ma;
	uint16_t num_led_est_table;
};

struct camera_led_est {
	uint16_t enable;
	uint16_t led_state;
	uint16_t current_ma;
	uint16_t lumen_value;
	uint16_t min_step;
	uint16_t max_step;
};

struct camera_flash_info {
	struct camera_led_info *led_info;
	struct camera_led_est *led_est_table;
};

struct camera_flash_cfg {
	int num_flash_levels;
	int (*camera_flash)(int level);
	uint16_t low_temp_limit;
	uint16_t low_cap_limit;
	uint8_t postpone_led_mode;
	struct camera_flash_info *flash_info;
};

static struct camera_flash_info *p_flash_led_info;
/* Andrew_Cheng linear led
  * 200 mA FL_MODE_FLASH_LEVEL1
  * 200 mA FL_MODE_FLASH_LEVEL2
  * 300 mA FL_MODE_FLASH_LEVEL3
  * 400 mA FL_MODE_FLASH_LEVEL4
  * 750 mA FL_MODE_FLASH
*/
/* for ENR, step range: 0 ~ 464 */
static struct camera_led_est camera_sensor_ov8838_led_table[] = {
	{
		.enable = 1,
		.led_state = FL_MODE_FLASH_LEVEL_1,
		.current_ma = 200,
		.lumen_value = 200,
		.min_step = 274,
		.max_step = 464
	},
	{
		.enable = 1,
		.led_state = FL_MODE_FLASH_LEVEL_2,
		.current_ma = 300,
		.lumen_value = 300,
		.min_step = 257,
		.max_step = 273
	},
	{
		.enable = 1,
		.led_state = FL_MODE_FLASH_LEVEL_3,
		.current_ma = 400,
		.lumen_value = 400,
		.min_step = 248,
		.max_step = 256
	},
	{
		.enable = 1,
		.led_state = FL_MODE_FLASH_LEVEL_4,
		.current_ma = 600,
		.lumen_value = 600,
		.min_step = 231,
		.max_step = 247
	},
	{
		.enable = 1,
		.led_state = FL_MODE_FLASH,
		.current_ma = 750,
		.lumen_value = 750,
		.min_step = 0,
		.max_step = 230
	},
	{
		.enable = 0,
		.led_state = FL_MODE_OFF,
		.current_ma = 0,
		.lumen_value = 0,
		.min_step = 0,
		.max_step = 0
	},
	{
		.enable = 0,
		.led_state = FL_MODE_TORCH,
		.current_ma = 150,
		.lumen_value = 150,
		.min_step = 0,
		.max_step = 0
	},
	{
		.enable = 2,
		.led_state = FL_MODE_FLASH,
		.current_ma = 750,
		.lumen_value = 750,
		.min_step = 271,
		.max_step = 317
	},
	{
		.enable = 0,
		.led_state = FL_MODE_FLASH,
		.current_ma = 750,
		.lumen_value = 750,
		.min_step = 271,
		.max_step = 325
	},
	{
		.enable = 0,
		.led_state = FL_MODE_TORCH_LEVEL_2,
		.current_ma = 200,
		.lumen_value = 75,
		.min_step = 0,
		.max_step = 40
	},
};

static struct camera_led_info camera_sensor_ov8838_led_info = {
	.enable = 1,
	.low_limit_led_state = FL_MODE_TORCH,
	.max_led_current_ma = 750,
	.num_led_est_table = ARRAY_SIZE(camera_sensor_ov8838_led_table),
};

static struct camera_flash_info camera_sensor_ov8838_flash_info = {
	.led_info = &camera_sensor_ov8838_led_info,
	.led_est_table = camera_sensor_ov8838_led_table,
};

static ssize_t flash_led_info_get(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t length = 0;

	if (p_flash_led_info != NULL)
		length = sprintf(buf, "%d %d %d %d\n",
			p_flash_led_info->led_info->enable,
			p_flash_led_info->led_info->low_limit_led_state,
			p_flash_led_info->led_info->max_led_current_ma,
			p_flash_led_info->led_info->num_led_est_table);
	else
		length = sprintf(buf, "%d\n", 0);
	pr_info("[CAM] %s: length(%d)\n", __func__, length);
	return length;
}

static ssize_t flash_led_tbl_get(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t length = 0;
	uint16_t i = 0;
	char sub[64] = {0};
	struct camera_led_est *sub_tbl = NULL;

	if (p_flash_led_info != NULL)
		for (i = 0; i < p_flash_led_info->led_info->num_led_est_table; i++) {
			sub_tbl = (struct camera_led_est *)
				(((char *)p_flash_led_info->led_est_table) +
				(i * sizeof(struct camera_led_est)));
			if (sub_tbl != NULL) {
			length += sprintf(sub, "%d %d %d %d %d %d ",
				sub_tbl->enable,
				sub_tbl->led_state,
				sub_tbl->current_ma,
				sub_tbl->lumen_value,
				sub_tbl->min_step,
				sub_tbl->max_step);
			strcat(buf, sub);
			}
		}
	else
		length = sprintf(buf, "%d\n", 0);
	return length;
}

static DEVICE_ATTR(flash_led_info, 0444,
	flash_led_info_get,
	NULL);

static DEVICE_ATTR(flash_led_tbl, 0444,
	flash_led_tbl_get,
	NULL);
/* Andrew_Cheng linear led */

static uint32_t led_ril_status_value;
static uint32_t led_wimax_status_value;
static uint32_t led_hotspot_status_value;
static uint16_t led_low_temp_limit;
static uint16_t led_low_cap_limit;
static struct kobject *led_status_obj;

static ssize_t led_ril_status_get(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t length;
	length = sprintf(buf, "%d\n", led_ril_status_value);
	return length;
}

static ssize_t led_ril_status_set(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	uint32_t tmp = 0;

	tmp = buf[0] - 0x30; /* only get the first char */

	led_ril_status_value = tmp;
	pr_info("[CAM] %s: led_ril_status_value = %d\n", __func__, led_ril_status_value);
	return count;
}

static ssize_t led_wimax_status_get(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t length;
	length = sprintf(buf, "%d\n", led_wimax_status_value);
	return length;
}

static ssize_t led_wimax_status_set(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	uint32_t tmp = 0;

	tmp = buf[0] - 0x30; /* only get the first char */

	led_wimax_status_value = tmp;
	pr_info("[CAM] %s: led_wimax_status_value = %d\n", __func__, led_wimax_status_value);
	return count;
}

static ssize_t led_hotspot_status_get(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t length;
	length = sprintf(buf, "%d\n", led_hotspot_status_value);
	return length;
}

static ssize_t led_hotspot_status_set(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	uint32_t tmp = 0;

	tmp = buf[0] - 0x30; /* only get the first char */

	led_hotspot_status_value = tmp;
	pr_info("[CAM] %s: led_hotspot_status_value = %d\n", __func__, led_hotspot_status_value);
	return count;
}

static ssize_t low_temp_limit_get(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t length;
	length = sprintf(buf, "%d\n", led_low_temp_limit);
	return length;
}

static ssize_t low_cap_limit_get(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t length;
	length = sprintf(buf, "%d\n", led_low_cap_limit);
	return length;
}

static DEVICE_ATTR(led_ril_status, 0644,
	led_ril_status_get,
	led_ril_status_set);

static DEVICE_ATTR(led_wimax_status, 0644,
	led_wimax_status_get,
	led_wimax_status_set);

static DEVICE_ATTR(led_hotspot_status, 0644,
	led_hotspot_status_get,
	led_hotspot_status_set);

static DEVICE_ATTR(low_temp_limit, 0444,
	low_temp_limit_get,
	NULL);

static DEVICE_ATTR(low_cap_limit, 0444,
	low_cap_limit_get,
	NULL);
static int camera_led_sysfs_init(void)
{
	int ret = 0;

	pr_info("[CAM] camera_led: kobject creat and add\n");
	led_status_obj = kobject_create_and_add("camera_led_status", NULL);
	if (led_status_obj == NULL) {
		pr_info("[CAM] %s: camera_led: subsystem_register failed\n", __func__);
		ret = -ENOMEM;
		goto error;
	}

	ret = sysfs_create_file(led_status_obj,
		&dev_attr_led_ril_status.attr);
	if (ret) {
		pr_info("[CAM] %s: camera_led: sysfs_create_file ril failed\n", __func__);
		ret = -EFAULT;
		goto error;
	}

	ret = sysfs_create_file(led_status_obj,
		&dev_attr_led_wimax_status.attr);
	if (ret) {
		pr_info("[CAM] %s: camera_led: sysfs_create_file wimax failed\n", __func__);
		ret = -EFAULT;
		goto error;
	}

	ret = sysfs_create_file(led_status_obj,
		&dev_attr_led_hotspot_status.attr);
	if (ret) {
		pr_info("[CAM] %s: camera_led: sysfs_create_file hotspot failed\n", __func__);
		ret = -EFAULT;
		goto error;
	}

	ret = sysfs_create_file(led_status_obj,
		&dev_attr_low_temp_limit.attr);
	if (ret) {
		pr_info("[CAM] %s: camera_led: sysfs_create_file low_temp_limit failed\n", __func__);
		ret = -EFAULT;
		goto error;
	}

	ret = sysfs_create_file(led_status_obj,
		&dev_attr_low_cap_limit.attr);
	if (ret) {
		pr_info("[CAM] %s: camera_led: sysfs_create_file low_cap_limit failed\n", __func__);
		ret = -EFAULT;
		goto error;
	}

	/* Linear LED */
	ret = sysfs_create_file(led_status_obj,
		&dev_attr_flash_led_info.attr);
	if (ret) {
		pr_info("[CAM] %s: sysfs_create_file flash_led_info failed\n", __func__);
		ret = -EFAULT;
		goto error;
	}

	ret = sysfs_create_file(led_status_obj,
		&dev_attr_flash_led_tbl.attr);
	if (ret) {
		pr_info("[CAM] %s: sysfs_create_file flash_led_tbl failed\n", __func__);
		ret = -EFAULT;
		goto error;
	}

	p_flash_led_info = &camera_sensor_ov8838_flash_info;
	/* Linear LED */

	led_low_temp_limit = 5;
	led_low_cap_limit = 15;

	return ret;
error:
	kobject_del(led_status_obj);
	return ret;
}

static int ov8838_select_mode(enum ov8838_data_lane num)
{
	if (num == DATA_LANE_4) {
		/* default setting is 4 lane setting */
		pr_info("[CAM] ov8838 uses mipi 4 lane\n");
	} else if (num == DATA_LANE_2) {
		pr_info("[CAM] ov8838 uses mipi 2 lane\n");
		rawchip_mode_table = rawchip_mode_table_2;
		mode_start = mode_start_2;
		mode_table[OV8838_MODE_3280x2464] = mode_3280x2464_2;
		mode_table[OV8838_MODE_3088x1736] = mode_3088x1736_2;
		mode_table[OV8838_MODE_1920x1080] = mode_1920x1080_2;
		mode_table[OV8838_MODE_1640x510]  = mode_1640x510_2;
	} else {
		pr_err("[CAM] ov8838 mipi lane number (%d) is wrong\n", num);
		return -1;
	}
	return 0;
}
/* HTC END */

static int ov8838_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int err;
	pr_info("[CAM] %s: probing sensor.\n", __func__);

	info = kzalloc(sizeof(struct ov8838_info), GFP_KERNEL);
	if (!info) {
		pr_err("[CAM] ov8838: Unable to allocate memory!\n");
		return -ENOMEM;
	}

	err = misc_register(&ov8838_device);
	if (err) {
		pr_err("[CAM] ov8838: Unable to register misc device!\n");
		kfree(info);
		return err;
	}

	info->pdata = client->dev.platform_data;
	info->i2c_client = client;
	info->mode = 0;
	i2c_set_clientdata(client, info);

	INIT_WORK(&info->set_mode_work, set_mode_handler);
	mutex_init(&info->camera_lock);

	INIT_WORK(&info->rawchip_work, rawchip_handler);
	mutex_init(&info->rawchip_lock);

	/* HTC START */
	ov8838_select_mode(info->pdata->data_lane);
	ov8838_sysfs_init();
	camera_led_sysfs_init();
	/* HTC END */

	ov8838_i2c_client = client;
	return 0;
}

static int ov8838_remove(struct i2c_client *client)
{
	struct ov8838_info *info;
	info = i2c_get_clientdata(client);
	misc_deregister(&ov8838_device);
	kfree(info);
	return 0;
}

static const struct i2c_device_id ov8838_id[] = {
	{ "ov8838", 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, ov8838_id);

static struct i2c_driver ov8838_i2c_driver = {
	.driver = {
		.name = "ov8838",
		.owner = THIS_MODULE,
	},
	.probe = ov8838_probe,
	.remove = ov8838_remove,
	.id_table = ov8838_id,
};

static int __init ov8838_init(void)
{
	pr_info("[CAM] ov8838 sensor driver loading\n");
	return i2c_add_driver(&ov8838_i2c_driver);
}

static void __exit ov8838_exit(void)
{
	i2c_del_driver(&ov8838_i2c_driver);
}

module_init(ov8838_init);
module_exit(ov8838_exit);
