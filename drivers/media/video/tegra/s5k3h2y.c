/*
 * s5k3h2y.c - s5k3h2y sensor driver
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
#include <media/s5k3h2y.h>
#include <media/rawchip/Yushan_API.h>
#include <../arch/arm/mach-tegra/include/mach/tegra_flashlight.h>

struct i2c_client *s5k3h2_i2c_client;
#define S5K3H2Y_REG_MODEL_ID 0x0000
#define S5K3H2Y_MODEL_ID 0x382b

#define SIZEOF_I2C_TRANSBUF 32

typedef enum s5k3h2y_mode_e{
	S5K3H2Y_MODE_3280x2464_912MIPI =1,
	S5K3H2Y_MODE_3084x1736,
	S5K3H2Y_MODE_768x432,
	S5K3H2Y_MODE_1640x510,
} s5k3h2y_mode;

struct s5k3h2y_reg {
	u16 addr;
	u16 val;
};

struct s5k3h2y_info {
	s5k3h2y_mode mode;
	struct work_struct set_mode_work;
	struct mutex camera_lock;
	struct i2c_client *i2c_client;
	struct s5k3h2y_platform_data *pdata;
	u8 i2c_trans_buf[SIZEOF_I2C_TRANSBUF];
	bool power_state;
};
unsigned long exposure_coarse_time; //HTC_Steven
unsigned long exposure_gain; //HTC_Steven
static struct s5k3h2y_info *info;
/* HTC_START */
static int sensor_probe_node = 0;
static int is_rawchip_init = 0;

/* HTC_END */
#define S5K3H2Y_TABLE_WAIT_MS 0
#define S5K3H2Y_TABLE_END 1
#define S5K3H2Y_MAX_RETRIES 3

static struct rawchip_sensor_init_data rawchip_mode_table[] =
{
	[S5K3H2Y_MODE_3280x2464_912MIPI] =
	{	/*  8M 3280x2464 */
		.spi_clk = 25,
		.ext_clk = 24,
		.lane_cnt = 2,
		.orientation = 0,
		.use_ext_1v2 = 1,
		.bitrate = 912,
		.width = 3280,
		.height = 2464,
		.blk_pixels = 190,
		.blk_lines = 36,
	},
	[S5K3H2Y_MODE_3084x1736] =
	{	/* 5M  3084x1736 */
		.spi_clk = 25,
		.ext_clk = 24,
		.lane_cnt = 2,
		.orientation = 0,
		.use_ext_1v2 = 1,
		.bitrate = 912,
		.width = 3084,
		.height = 1736,
		.blk_pixels = 386,
		.blk_lines = 36,
	},
	[S5K3H2Y_MODE_768x432]
	{	/* slow motion mode 768x432 */
		.spi_clk = 25,
		.ext_clk = 24,
		.lane_cnt = 2,
		.orientation = 0,
		.use_ext_1v2 = 1,
		.bitrate = 912,
		.width = 768,
		.height = 432,
		.blk_pixels = 2702,
		.blk_lines = 36,
	},
	[S5K3H2Y_MODE_1640x510]
	{	/* slow motion mode 1640x510 */
		.spi_clk = 25,
		.ext_clk = 24,
		.lane_cnt = 2,
		.orientation = 0,
		.use_ext_1v2 = 1,
		.bitrate = 912,
		.width = 1640,
		.height = 510,
		.blk_pixels = 1830,
		.blk_lines = 36,
	}
};

static Yushan_New_Context_Config_t mode_rawchip[] = {
	{   //full size
		// Active pixels in a line( Give the worst case here for stills).
		.uwActivePixels = 3280, 		// HSize
		// Line blanking
		.uwLineBlank = 190,
		// Active frame length (VSize): For DXO Image Char
		.uwActiveFrameLength = 2464,	// VSize;
		// STILL/VF or NORMAL
		.bSelectStillVfMode = 2,
		// Similar as the programming in Yushan_Init_Struct_t
		.uwPixelFormat = 0x0A0A,
		.orientation = 0,
	},
	{	//quarter size
		// Active pixels in a line( Give the worst case here for stills).
		.uwActivePixels = 1640, 		// HSize
		// Line blanking
		.uwLineBlank = 1830,
		// Active frame length (VSize): For DXO Image Char
		.uwActiveFrameLength = 1232,	// VSize;
		// STILL/VF or NORMAL
		.bSelectStillVfMode = 1,
		// Similar as the programming in Yushan_Init_Struct_t
		.uwPixelFormat = 0x0A0A,
		.orientation = 0,
	},
	{	//slow motion
		// Active pixels in a line( Give the worst case here for stills).
		.uwActivePixels = 768, 		// HSize
		// Line blanking
		.uwLineBlank = 2702,
		// Active frame length (VSize): For DXO Image Char
		.uwActiveFrameLength = 432,	// VSize;
		// STILL/VF or NORMAL
		.bSelectStillVfMode = 1,
		// Similar as the programming in Yushan_Init_Struct_t
		.uwPixelFormat = 0x0A0A,
		.orientation = 0,
	},
	{	// 5mp mode
		// Active pixels in a line( Give the worst case here for stills).
		.uwActivePixels = 3084, 		// HSize
		// Line blanking
		.uwLineBlank = 386, // 3470-3084
		// Active frame length (VSize): For DXO Image Char
		.uwActiveFrameLength = 1736,	// VSize;
		// STILL/VF or NORMAL
		.bSelectStillVfMode = 1,
		// Similar as the programming in Yushan_Init_Struct_t
		.uwPixelFormat = 0x0A0A,
		.orientation = 0,
	},
	{	// 1640x510 mode
		// Active pixels in a line( Give the worst case here for stills).
		.uwActivePixels = 1640, 		// HSize
		// Line blanking
		.uwLineBlank = 1830, // 3470-1640
		// Active frame length (VSize): For DXO Image Char
		.uwActiveFrameLength = 510,	// VSize;
		// STILL/VF or NORMAL
		.bSelectStillVfMode = 1,
		// Similar as the programming in Yushan_Init_Struct_t
		.uwPixelFormat = 0x0A0A,
		.orientation = 0,
	},
};

static struct s5k3h2y_reg reset_seq[] = {
	{ 0x0100, 0x00 },	// stream off
	{S5K3H2Y_TABLE_WAIT_MS, 5},
#if 0
    { 0x0101, 0x00 },   // Mirror off
    {S5K3H2Y_TABLE_WAIT_MS, 1},
	{ 0x0103, 0x01 },	// software reset
	{S5K3H2Y_TABLE_WAIT_MS, 1},
#endif
	{S5K3H2Y_TABLE_END, 0x0000},
};

static struct s5k3h2y_reg mode_start[] = {
    //MIPI Setting
    //Address   Data
    { 0x3065, 0x35 },
    { 0x310E, 0x00 },
    { 0x3098, 0xAB },
    { 0x30C7, 0x0A },
    { 0x309A, 0x01 },
    { 0x310D, 0xC6 },
    { 0x30c3, 0x40 },
    { 0x30BB, 0x02 },
    { 0x30BC, 0x38 }, //According to MCLK, these registers should be changed
    { 0x30BD, 0x40 }, //
    { 0x3110, 0x70 }, //
    { 0x3111, 0x80 }, //
    { 0x3112, 0x7B }, //
    { 0x3113, 0xC0 }, //
    { 0x30C7, 0x1A }, //

    //Manufacture Setting
    //         Address   Data
    { 0x3000, 0x08 },
    { 0x3001, 0x05 },
    { 0x3002, 0x0D },
    { 0x3003, 0x21 },
    { 0x3004, 0x62 },
    { 0x3005, 0x0B },
    { 0x3006, 0x6D },
    { 0x3007, 0x02 },
    { 0x3008, 0x62 },
    { 0x3009, 0x62 },
    { 0x300A, 0x41 },
    { 0x300B, 0x10 },
    { 0x300C, 0x21 },
    { 0x300D, 0x04 },
    { 0x307E, 0x03 },
    { 0x307F, 0xA5 },
    { 0x3080, 0x04 },
    { 0x3081, 0x29 },
    { 0x3082, 0x03 },
    { 0x3083, 0x21 },
    { 0x3011, 0x5F },
    { 0x3156, 0xE2 },
    { 0x3027, 0xBE },
    { 0x300f, 0x02 },
    { 0x3010, 0x10 },
    { 0x3017, 0x74 },
    { 0x3018, 0x00 },
    { 0x3020, 0x02 },
    { 0x3021, 0x24 },		// { 0x3021, 0x00 },
    { 0x3023, 0x80 },
    { 0x3024, 0x04 },		// { 0x3024, 0x08 },
    { 0x3025, 0x08 },
    { 0x301C, 0xD4 },
    { 0x315D, 0x00 },
    { 0x3054, 0x00 },
    { 0x3055, 0x35 },
    { 0x3062, 0x04 },
    { 0x3063, 0x38 },
    { 0x31A4, 0x04 },
    { 0x3016, 0x54 },
    { 0x3157, 0x02 },
    { 0x3158, 0x00 },
    { 0x315B, 0x02 },
    { 0x315C, 0x00 },
    { 0x301B, 0x05 },
    { 0x3028, 0x41 },
    { 0x302A, 0x00 },		// { 0x302A, 0x10 },
    { 0x3060, 0x00 },
    { 0x302D, 0x19 },
    { 0x302B, 0x04 },		// { 0x302B, 0x05 },
    { 0x3072, 0x13 },
    { 0x3073, 0x21 },
    { 0x3074, 0x82 },
    { 0x3075, 0x20 },
    { 0x3076, 0xA2 },
    { 0x3077, 0x02 },
    { 0x3078, 0x91 },
    { 0x3079, 0x91 },
    { 0x307A, 0x61 },
    { 0x307B, 0x28 },
    { 0x307C, 0x31 },
#if 0 /* Maverick modify (Black level)*/

    { 0x304E, 0x40 },
    { 0x304F, 0x01 },
    { 0x3050, 0x00 },
    { 0x3088, 0x01 },
    { 0x3089, 0x00 },
    { 0x3210, 0x01 },
    { 0x3211, 0x00 },
    { 0x308E, 0x01 },
    { 0x308F, 0x8F },
    { 0x3064, 0x03 },
    { 0x31A7, 0x0F },
#endif
    //MIPI
    {0x0305, 0x04},/*pre_pll_clk_div = 4*/
    {0x0306, 0x00},/*pll_multiplier*/
    {0x0307, 0x98},/*pll_multiplier  = 152*/
    {0x0303, 0x01},/*vt_sys_clk_div = 1*/
    {0x0301, 0x05},/*vt_pix_clk_div = 5*/
    {0x030B, 0x01},/*op_sys_clk_div = 1*/
    {0x0309, 0x05},/*op_pix_clk_div = 5*/
    {0x30CC, 0xE0},/*DPHY_band_ctrl 870 MHz ~ 950 MHz*/
    {0x31A1, 0x5A},

    { S5K3H2Y_TABLE_END, 0x0000 }
};

static struct s5k3h2y_reg mode_3280x2464_912MIPI[] = {
    //Mode Setting PLL EXCLK 24Mhz, vt_pix_clk_freq_mhz=182.4Mhz,op_sys_clk_freq_mhz=912Mhz
    { 0x0305, 0x04 }, //pre_pll_clk_div = 4
    { 0x0306, 0x00 }, //pll_multiplier
    { 0x0307, 0x98 }, //pll_multiplier
    { 0x0303, 0x01 }, //vt_sys_clk_div = 1
    { 0x0301, 0x05 }, //vt_pix_clk_div = 5
    { 0x030B, 0x01 }, //op_sys_clk_div = 1
    { 0x0309, 0x05 }, //op_pix_clk_div = 5
    { 0x30CC, 0xE0 }, //DPHY_band_ctrl
    { 0x31A1, 0x5A },

    //Readout Full
    //Address   Data
    { 0x0344, 0x00 }, //X addr start 0d
    { 0x0345, 0x00 },
    { 0x0346, 0x00 }, //Y addr start 0d
    { 0x0347, 0x00 },
    { 0x0348, 0x0C }, //X addr end 3279d
    { 0x0349, 0xCF },
    { 0x034A, 0x09 }, //Y addr end 2463d
    { 0x034B, 0x9F },

    { 0x0381, 0x01 }, //x_even_inc = 1
    { 0x0383, 0x01 }, //x_odd_inc = 1
    { 0x0385, 0x01 }, //y_even_inc = 1
    { 0x0387, 0x01 }, //y_odd_inc = 1

    { 0x0401, 0x00 }, //Derating_en  = 0 (disable)
    { 0x0405, 0x10 },
    { 0x0700, 0x05 }, //fifo_water_mark_pixels = 1328
    { 0x0701, 0x30 },

    { 0x034C, 0x0C }, //x_output_size = 3280 = 0x0cD0
    { 0x034D, 0xD0 }, //
    { 0x034E, 0x09 }, //y_output_size = 2464 = 0x09A0
    { 0x034F, 0xA0 },

    { 0x0200, 0x02 }, //fine integration time
    { 0x0201, 0x50 },
    { 0x0202, 0x04 }, //Coarse integration time
    { 0x0203, 0xE7 },
    { 0x0204, 0x00 }, //Analog gain
    { 0x0205, 0x20 },
    { 0x0342, 0x0D }, //Line_length_pck 3470d
    { 0x0343, 0x8E },
    { 0x0340, 0x09 }, /*FRAME_LENGTH_LINES*/
#if 1// CONFIG_RAWCHIP
    { 0x0341, 0xC4 },
#else
    { 0x0341, 0xC0 },
#endif

    //Manufacture Setting
    //Address   Data
    { 0x300E, 0x29 }, //Hbinnning[2] : 1b enale / 0b disable
    { 0x31A3, 0x00 }, //Vbinning enable[6] : 1b enale / 0b disable
    { 0x301A, 0xA7 }, //"In case of using the Vt_Pix_Clk more than 137Mhz, 0xA7h should be adopted! "
    { 0x3053, 0xCF },
    {S5K3H2Y_TABLE_END, 0x0000}
};

static struct s5k3h2y_reg mode_3084x1736[] = {
    {0x0305, 0x04 },	//pre_pll_clk_div = 4
    {0x0306, 0x00 },	//pll_multiplier
    {0x0307, 0x98 },	//pll_multiplier  = 152
    {0x0303, 0x01 },	//vt_sys_clk_div = 1
    {0x0301, 0x05 },	//vt_pix_clk_div = 5
    {0x030B, 0x01 },	//op_sys_clk_div = 1
    {0x0309, 0x05 },	//op_pix_clk_div = 5
    {0x30CC, 0xE0 },	//DPHY_band_ctrl 870 MHz ~ 950 MHz
    {0x31A1, 0x5A },	//"DBR_CLK = PLL_CLK / DIV_DBR(0x31A1[3:0])

    //Readout
    //Address	Data	Comment
    {0x0344, 0x00 },	//X addr start 96d
    {0x0345, 0x60 },
    {0x0346, 0x01 },	//Y addr start 364d
    {0x0347, 0x6C },
    {0x0348, 0x0C },	//X addr end 3179d
    {0x0349, 0x6B },
    {0x034A, 0x08 },	//Y addr end 2099d
    {0x034B, 0x33 },

    {0x0381, 0x01 },	//x_even_inc = 1
    {0x0383, 0x01 },	//x_odd_inc = 1
    {0x0385, 0x01 },	//y_even_inc = 1
    {0x0387, 0x01 },	//y_odd_inc = 1

    {0x0401, 0x00 },	//Derating_en  = 0 (disable)
    {0x0405, 0x10 },
    {0x0700, 0x05 },	//fifo_water_mark_pixels = 1328
    {0x0701, 0x30 },

    {0x034C, 0x0C },	//x_output_size = 3084
    {0x034D, 0x0C },
    {0x034E, 0x06 },	//y_output_size = 1736
    {0x034F, 0xC8 },

    {0x0200, 0x02 },	//fine integration time
    {0x0201, 0x50 },
    {0x0202, 0x04 },	//Coarse integration time
    {0x0203, 0xDB },
    {0x0204, 0x00 },	//Analog gain
    {0x0205, 0x20 },
    {0x0342, 0x0D },	//Line_length_pck 3470d
    {0x0343, 0x8E },
    {0x0340, 0x06 },	//Frame_length_lines 1772d
    //{0x0341, 0xD8 },

    // below is newly added to follow similar way as full mode. To test
    // raw chip requires 4 more lines in frame length
    // totally 20 more lines comparing to initial setting from vendor
    // 0x6DB => 0x6EC
#if 1// CONFIG_RAWCHIP
    { 0x0341, 0xEC },
#else
    {0x0341, 0xE8},
#endif


    //Manufacture Setting
    //Address	Data	Comment
    {0x300E, 0x29 },
    {0x31A3, 0x00 },
    {0x301A, 0xA7 },
    {0x3053, 0xCB },	//CF for full/preview/ ,CB for HD/FHD/QVGA120fps

    {S5K3H2Y_TABLE_END, 0x0000}
};

static struct s5k3h2y_reg mode_768x432[] = {
	//Address	Data	Comment
    { 0x0305, 0x04 },	//pre_pll_clk_div = 4
    { 0x0306, 0x00 },	//pll_multiplier
    { 0x0307, 0x98 },	//pll_multiplier  = 152
    { 0x0303, 0x01 },	//vt_sys_clk_div = 1
    { 0x0301, 0x05 },	//vt_pix_clk_div = 5
    { 0x030B, 0x01 },	//op_sys_clk_div = 1
    { 0x0309, 0x05 },	//op_pix_clk_div = 5
    { 0x30CC, 0xE0 },	//DPHY_band_ctrl 870 MHz ~ 950 MHz
    { 0x31A1, 0x5A },	//"DBR_CLK = PLL_CLK / DIV_DBR(0x31A1[3:0]= 912Mhz / 10 = 91.2Mhz[7:4]
	//Readout
	//Address	Data	Comment
    { 0x0344, 0x00 },	//X addr start 104d
    { 0x0345, 0x68 },
    { 0x0346, 0x01 },	//Y addr start 456d
    { 0x0347, 0xC8 },
    { 0x0348, 0x0C },	//X addr end 3175d
    { 0x0349, 0x67 },
    { 0x034A, 0x08 },	//Y addr end 2183d
    { 0x034B, 0x87 },

    { 0x0381, 0x01 },	//x_even_inc = 1
    { 0x0383, 0x07 },	//x_odd_inc = 7
    { 0x0385, 0x01 },	//y_even_inc = 1
    { 0x0387, 0x07 },	//y_odd_inc = 7

    { 0x0401, 0x00 },	//Derating_en  = 0 (disable)
    { 0x0405, 0x10 },
    { 0x0700, 0x05 },	//fifo_water_mark_pixels = 1328
    { 0x0701, 0x30 },

    { 0x034C, 0x03 },	//x_output_size = 768
    { 0x034D, 0x00 },
    { 0x034E, 0x01 },	//y_output_size = 432
    { 0x034F, 0xB0 },

    { 0x0200, 0x02 },	//fine integration time
    { 0x0201, 0x50 },
    { 0x0202, 0x01 },	//Coarse integration time
    { 0x0203, 0xB0 },
    { 0x0204, 0x00 },	//Analog gain
    { 0x0205, 0x20 },
    { 0x0342, 0x0D },	//Line_length_pck 3470d
    { 0x0343, 0x8E },
    { 0x0340, 0x01 },	//Frame_length_lines 468d
    { 0x0341, 0xD4 },

	//Manufacture Setting
	//Address	Data	Comment
    { 0x300E, 0x2D },	//
    { 0x31A3, 0x40 },	//
    { 0x301A, 0xA7 },	//
    { 0x3053, 0xCB },//CF for full ,CB for preview/HD/FHD/QVGA120fps

    { S5K3H2Y_TABLE_END, 0x0000 }
};

static struct s5k3h2y_reg mode_1640x1232[] = {
    //Readout	H:1/2 SubSampling binning, V:1/2 SubSampling binning
    //EXCLK 24Mhz, vt_pix_clk_freq_mhz=182.4Mhz,op_sys_clk_freq_mhz=912Mhz
    //Address	Data	Comment
    { 0x0305, 0x04 },	//pre_pll_clk_div = 4
    { 0x0306, 0x00 },	//pll_multiplier
    { 0x0307, 0x98 },//pll_multiplier  = 152
    { 0x0303, 0x01 },	//vt_sys_clk_div = 1
    { 0x0301, 0x05 },	//vt_pix_clk_div = 5
    { 0x030B, 0x01 },//op_sys_clk_div = 1
    { 0x0309, 0x05 },	//op_pix_clk_div = 5
    { 0x30CC, 0xE0 },	//DPHY_band_ctrl 870 MHz ~ 950 MHz
    { 0x31A1, 0x5A },	//"DBR_CLK = PLL_CLK / DIV_DBR(s31A1[3:0]) = 912Mhz / 10 = 91.2Mhz[7:4] must be same as vt_pix_clk_div (s0301)"

	//Readout	//H:1/2 SubSampling binning, V:1/2 SubSampling binning
	//Address	//Data	Comment
    { 0x0344, 0x00 },	//X addr start 0d
    { 0x0345, 0x00 },	//
    { 0x0346, 0x00 },	//Y addr start 0d
    { 0x0347, 0x00 },	//
    { 0x0348, 0x0C },	//X addr end 3277d
    { 0x0349, 0xCD },	//
    { 0x034A, 0x09 },	//Y addr end 2463d
    { 0x034B, 0x9F },	//
		//
    { 0x0381, 0x01 },	//x_even_inc = 1
    { 0x0383, 0x03 },	//x_odd_inc = 3
    { 0x0385, 0x01 },	//y_even_inc = 1
    { 0x0387, 0x03 },	//y_odd_inc = 3
	//
    { 0x0401, 0x00 },	//Derating_en  = 0 (disable)
    { 0x0405, 0x10 },	//
    { 0x0700, 0x05 },	//fifo_water_mark_pixels = 1328
    { 0x0701, 0x30 },	//
	//
    { 0x034C, 0x06 },	//x_output_size = 1640
    { 0x034D, 0x68 },	//
    { 0x034E, 0x04 },	//y_output_size = 1232
    { 0x034F, 0xD0 },	//
	//
    { 0x0200, 0x02 },	//fine integration time
    { 0x0201, 0x50 },	//
    { 0x0202, 0x04 },	//Coarse integration time
    { 0x0203, 0xDB },	//
    { 0x0204, 0x00 },	//Analog gain
    { 0x0205, 0x20 },	//
    { 0x0342, 0x0D },	//Line_length_pck 3470d
    { 0x0343, 0x8E },	//
    { 0x0340, 0x04 },	//Frame_length_lines 1268d
    { 0x0341, 0xF4 },

    //Manufacture Setting
    //Address	Data	Comment
    { 0x300E, 0x2D },	//Hbinnning[2] : 1b enale / 0b disable
    { 0x31A3, 0x40 },	//Vbinning enable[6] : 1b enale / 0b disable
    { 0x301A, 0xA7 },	//"In case of using the Vt_Pix_Clk more than 137Mhz, sA7h should be adopted! "
    { 0x3053, 0xCF },

	{ S5K3H2Y_TABLE_END, 0x0000 }
};

static struct s5k3h2y_reg mode_1640x510[] = {
    { 0x0305, 0x04 },	//pre_pll_clk_div = 4
    { 0x0306, 0x00 },	//pll_multiplier
    { 0x0307, 0x98 },	//pll_multiplier  = 152
    { 0x0303, 0x01 },	//vt_sys_clk_div = 1
    { 0x0301, 0x05 },	//vt_pix_clk_div = 5
    { 0x030B, 0x01 },	//op_sys_clk_div = 1
    { 0x0309, 0x05 },	//op_pix_clk_div = 5
    { 0x30CC, 0xE0 },	//DPHY_band_ctrl 870 MHz ~ 950 MHz
    { 0x31A1, 0x5A },	//"DBR_CLK = PLL_CLK / DIV_DBR(0x31A1[3:0]= 912Mhz / 10 = 91.2Mhz[7:4]

    { 0x0344, 0x00 },	//X addr start 0d
    { 0x0345, 0x00 },
    { 0x0346, 0x00 },	//Y addr start 212d
    { 0x0347, 0xD4 },
    { 0x0348, 0x0C },	//X addr end 3279d
    { 0x0349, 0xCF },
    { 0x034A, 0x08 },	//Y addr end 2251d
    { 0x034B, 0xCB },

    { 0x0381, 0x01 },	//x_even_inc = 1
    { 0x0383, 0x03 },	//x_odd_inc = 3
    { 0x0385, 0x01 },	//y_even_inc = 1
    { 0x0387, 0x07 },	//y_odd_inc = 7

    { 0x0401, 0x00 },	//Derating_en  = 0 (disable)
    { 0x0405, 0x10 },
    { 0x0700, 0x05 },	//fifo_water_mark_pixels = 1328
    { 0x0701, 0x30 },

    { 0x034C, 0x06 },	//x_output_size = 1640
    { 0x034D, 0x68 },
    { 0x034E, 0x01 },	//y_output_size = 510
    { 0x034F, 0xFE },

    { 0x0200, 0x02 },	//fine integration time
    { 0x0201, 0x50 },
    { 0x0202, 0x01 },	//Coarse integration time
    { 0x0203, 0x39 },
    { 0x0204, 0x00 },	//Analog gain
    { 0x0205, 0x20 },
    { 0x0342, 0x0D },	//Line_length_pck 3470d
    { 0x0343, 0x8E },
#if 1// CONFIG_RAWCHIP
    { 0x0340, 0x02 },	//Frame_length_lines 546d
    { 0x0341, 0x22 },
#else
    { 0x0340, 0x02 },	//Frame_length_lines 526d
    { 0x0341, 0x0E },
#endif
    //Manufacture Setting
    { 0x300E, 0x2D },
    { 0x31A3, 0x40 },
    { 0x301A, 0xA7 },
    { 0x3053, 0xCB },//CF for full ,CB for preview/HD/FHD/QVGA120fps
};

static struct s5k3h2y_reg mode_end[] = {
	{ S5K3H2Y_TABLE_WAIT_MS, 1 },
	{ 0x0100, 0x01 },	// stream on
    { S5K3H2Y_TABLE_END, 0x0000 }
};

static struct s5k3h2y_reg *mode_table[] = {
	[S5K3H2Y_MODE_3280x2464_912MIPI] = mode_3280x2464_912MIPI,
    [S5K3H2Y_MODE_3084x1736] = mode_3084x1736,
	[S5K3H2Y_MODE_768x432] = mode_768x432,
	[S5K3H2Y_MODE_1640x510] = mode_1640x510,
};

/* 2 regs to program frame length */
static inline void s5k3h2y_get_frame_length_regs(struct s5k3h2y_reg *regs,
						u32 frame_length)
{
	regs->addr = 0x340;
	regs->val = (frame_length >> 8) & 0xff;
	(regs + 1)->addr = 0x341;
	(regs + 1)->val = (frame_length) & 0xff;
}

/* 2 regs to program coarse time */
static inline void s5k3h2y_get_coarse_time_regs(struct s5k3h2y_reg *regs,
                                               u32 coarse_time)
{
	regs->addr = 0x202;
	regs->val = (coarse_time >> 8) & 0xff;
	(regs + 1)->addr = 0x203;
	(regs + 1)->val = (coarse_time) & 0xff;
}

/* 2 reg to program gain */
static inline void s5k3h2y_get_gain_reg(struct s5k3h2y_reg *regs, u16 gain)
{
	regs->addr = 0x204;
	regs->val = (gain >> 8) & 0xff;
	(regs + 1)->addr = 0x205;
	(regs + 1)->val = (gain) & 0xff;
}

static int s5k3h2y_i2c_rxdata(unsigned short saddr,
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

	if (i2c_transfer(s5k3h2_i2c_client->adapter, msgs, 2) < 0) {
		pr_err("[CAM]s5k3h2y_i2c_rxdata failed!\n");
		return -EIO;
	}

	return 0;
}

static int32_t s5k3h2y_i2c_read(unsigned short raddr,
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
	rc = s5k3h2y_i2c_rxdata(s5k3h2_i2c_client->addr, buf, rlen);

	if (rc < 0) {
		pr_err("[CAM]s5k3h2y_i2c_read 0x%x failed!\n", raddr);
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
static int s5k3h2y_read_reg(struct i2c_client *client, u16 addr, u8 *val)
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
	pr_info("s5k3h2y read_reg 0x%02X%02X %d\n", data[0], data[1], (data[2]<<8) + data[3]);

	return 0;
}
#endif

static int s5k3h2y_check_sensorid()
{
	uint16_t chipid = 0;
	int32_t rc = 0;

	/* Read sensor Model ID: */
	rc = s5k3h2y_i2c_read(S5K3H2Y_REG_MODEL_ID, &chipid, 2);
	if (rc < 0) {
		pr_err("[CAM]read sensor id fail\n");
	      rc = EIO;
	}

	/* Compare sensor ID to S5K3H2Y ID: */
/*	pr_info("[CAM]%s, Expected id=0x%x\n", __func__, S5K3H2Y_MODEL_ID);
	pr_info("[CAM]%s, Read id=0x%x\n", __func__, chipid); */

	if (chipid != S5K3H2Y_MODEL_ID) {
		pr_err("[CAM]sensor model id is incorrect\n");
		rc = -ENODEV;
	}
	return rc;
}

static int s5k3h2y_write_reg(struct i2c_client *client, u16 addr, u8 val)
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
		pr_err("s5k3h2y: i2c transfer failed, retrying %x %x\n",
			addr, val);
		msleep(3);
	} while (retry <= S5K3H2Y_MAX_RETRIES);

	return err;
}

static int s5k3h2y_write_bulk_reg(struct i2c_client *client, u8 *data, int len)
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

	pr_err("s5k3h2y: i2c bulk transfer failed at %x\n",
		(int)data[0] << 8 | data[1]);

	return err;
}

static int s5k3h2y_write_table(struct i2c_client *client,
				const struct s5k3h2y_reg table[],
				const struct s5k3h2y_reg override_list[],
				int num_override_regs)
{
	int err;
	const struct s5k3h2y_reg *next;
	int i;
	u16 val;

	for (next = table; next->addr != S5K3H2Y_TABLE_END; next++) {
		if (next->addr == S5K3H2Y_TABLE_WAIT_MS) {
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

		err = s5k3h2y_write_reg(client, next->addr, val);;
		if (err)
			return err;
	}
	return 0;
}

static int s5k3h2y_reset(struct s5k3h2y_info *info)
{
	int err = 0;
	pr_info("[CAM]35k3h2y_reset +++");

	info->pdata->power_off();
	msleep(50);
	info->pdata->power_on();

	/*sensor stream off*/
	pr_info("[CAM]s5k3h2y_write_table reset_seq");
	err = s5k3h2y_write_table(info->i2c_client, reset_seq, NULL, 0);
	if (err)
		pr_err("[CAM]set sensor stream off failed!!!");

	/*set sensor init*/
	pr_info("[CAM]s5k3h2y_write_table mode_start");
	err = s5k3h2y_write_table(info->i2c_client, mode_start, NULL, 0);
	if (err)
		pr_err("[CAM]set sensor init failed!!");

	pr_info("[CAM]35k3h2y_reset ---");
	return err;
}


static int s5k3h2y_set_mode(struct s5k3h2y_info *info, struct s5k3h2y_mode *mode)
{
	int sensor_mode;
	int err = -1;
	bool_t	bStatus = SUCCESS;
	struct s5k3h2y_reg reg_list[6];
	static Yushan_New_Context_Config_t *newContextConfig;
	int retry = 0;
	pr_debug("%s++\n", __func__);

	if((err = s5k3h2y_check_sensorid())!=0)
	{
		info->mode = 0;
		pr_debug("%s reading sensor id failed\n", __func__);
		return err;
	}

	if (mode) {
		pr_info("%s: xres %u yres %u framelength %u coarsetime %u gain %u\n",
			__func__, mode->xres, mode->yres, mode->frame_length,
			mode->coarse_time, mode->gain);
		if (mode->xres == 3280 && mode->yres == 2464)
		{
			pr_info("[CAM] set mode to -> S5K3H2Y_MODE_3280x2464_912MIPI\n");
			sensor_mode = S5K3H2Y_MODE_3280x2464_912MIPI;
			newContextConfig=&(mode_rawchip[0]);
		}
		else if (mode->xres == 3084 && mode->yres == 1736)
		{
			sensor_mode = S5K3H2Y_MODE_3084x1736;
			newContextConfig=&(mode_rawchip[3]);
			pr_info("[CAM] set mode to -> S5K3H2Y_MODE_3084x1736\n");
		}
		else if (mode->xres == 768 && mode->yres == 432) {
			sensor_mode = S5K3H2Y_MODE_768x432;
			pr_info("[CAM] set mode to -> S5K3H2Y_MODE_768x432\n");
			newContextConfig=&(mode_rawchip[2]);
		}
		else if (mode->xres == 1640 && mode->yres == 510) {
			sensor_mode = S5K3H2Y_MODE_1640x510;
			pr_info("[CAM] set mode to -> S5K3H2Y_MODE_1640x510\n");
			newContextConfig=&(mode_rawchip[4]);
		}
		else {
			pr_err("%s: invalid resolution supplied to set mode %d %d\n",__func__, mode->xres, mode->yres);
			return -EINVAL;
		}
		if (sensor_mode == info->mode) {
			pr_info("skip set_mode\n");
			return 0;
		}

		info->mode = sensor_mode;
		pr_info("[CAM] set_mode sensor_mode %d ", sensor_mode);
	} else {
		pr_debug("%s: setting mode with 8M\n", __func__);
		/* If mode == NULL, then it's called in open. */
		info->mode = S5K3H2Y_MODE_3280x2464_912MIPI;
		sensor_mode = info->mode;
	}

retry:
	/* get a list of override regs for the asking frame length, */
	/* coarse integration time, and gain.                       */
	if(mode)
	{
		s5k3h2y_get_frame_length_regs(reg_list, mode->frame_length);
		s5k3h2y_get_coarse_time_regs(reg_list + 2, mode->coarse_time);
		s5k3h2y_get_gain_reg(reg_list + 4, mode->gain);
	} else {
		// AE breakdown 875*45 == 1240*32 sensor default
		s5k3h2y_get_coarse_time_regs(reg_list, 2190); //HTC_Optical: modify for initial AE
		s5k3h2y_get_gain_reg(reg_list + 2, 38); //HTC_Optical: modify for initial AE
	}

	/*sensor stream off*/
	err = s5k3h2y_write_table(info->i2c_client, reset_seq, NULL, 0);
	if (err)
		goto error;

	/*set sensor config*/
	if(mode)
		err = s5k3h2y_write_table(info->i2c_client, mode_table[sensor_mode],reg_list, 6);
	else
		err = s5k3h2y_write_table(info->i2c_client, mode_table[sensor_mode],reg_list, 4);
	
	if (err)
		goto error;

	if(!is_rawchip_init) {
		pr_info("[CAM] call Yushan_sensor_open_init(%d) +++", sensor_mode);
		err = Yushan_sensor_open_init(rawchip_mode_table[sensor_mode]);
		if (err && retry++ < 5) {
			pr_err("[CAM]Yushan_sensor_open_init FAIL!!!");
			s5k3h2y_reset(info);
			goto retry;
		}
		msleep(1);
		pr_info("[CAM] call Yushan_sensor_open_init(%d) ---", sensor_mode);
		is_rawchip_init = 1;
	}
	else {
		pr_info("[CAM] call Yushan_ContextUpdate_Wrapper+++");
		bStatus = Yushan_ContextUpdate_Wrapper(newContextConfig);
		if ((!bStatus) && retry++ < 5) {
			pr_err("[CAM] Yushan_ContextUpdate_Wrapper FAIL!!!");
			is_rawchip_init = 0;
			s5k3h2y_reset(info);
			goto retry;
		}
		pr_info("[CAM] call Yushan_ContextUpdate_Wrapper---");
	}

	/*stream on*/
	err = s5k3h2y_write_table(info->i2c_client, mode_end, NULL, 0);
	if (err)
		goto error;
	pr_info("[CAM] s5k3h2y stream on!");
	pr_debug("%s --\n", __func__);
	return 0;
error:
	pr_debug("%s failed\n", __func__);
	return err;
}

static void set_mode_handler(struct work_struct *work)
{
	struct s5k3h2y_info *info;
	int err;

	pr_debug("%s ++\n", __func__);
	info = container_of(work, struct s5k3h2y_info,
			set_mode_work);
	mutex_lock(&info->camera_lock);

	if (info->pdata && info->pdata->power_on) {
		info->pdata->power_on();
		is_rawchip_init = 0;
	}

	/*sensor stream off*/
	pr_info("[CAM]s5k3h2y_write_table reset_seq");
	err = s5k3h2y_write_table(info->i2c_client, reset_seq, NULL, 0);
	if (err) 
		pr_err("[CAM]set sensor stream off failed!!!");

	/*set sensor init*/
	pr_info("[CAM]s5k3h2y_write_table mode_start");
	err = s5k3h2y_write_table(info->i2c_client, mode_start, NULL, 0);
	if (err) 
		pr_err("[CAM]set sensor init failed!!!");

	s5k3h2y_set_mode(info, NULL);
	info->power_state = true;
	mutex_unlock(&info->camera_lock);
	pr_debug("%s --\n", __func__);
}


static int s5k3h2y_set_frame_length(struct s5k3h2y_info *info, u32 frame_length)
{
	int ret;
	struct s5k3h2y_reg reg_list[2];
	u8 *b_ptr = info->i2c_trans_buf;

	s5k3h2y_get_frame_length_regs(reg_list, frame_length);

	*b_ptr++ = reg_list[0].addr >> 8;
	*b_ptr++ = reg_list[0].addr & 0xff;
	*b_ptr++ = reg_list[0].val & 0xff;
	*b_ptr++ = reg_list[1].val & 0xff;
	ret = s5k3h2y_write_bulk_reg(info->i2c_client, info->i2c_trans_buf, 4);

	return 0;
}

static int s5k3h2y_set_coarse_time(struct s5k3h2y_info *info, u32 coarse_time)
{
	int ret;
	struct s5k3h2y_reg reg_list[2];
	u8 *b_ptr = info->i2c_trans_buf;

	s5k3h2y_get_coarse_time_regs(reg_list, coarse_time);

	*b_ptr++ = reg_list[0].addr >> 8;
	*b_ptr++ = reg_list[0].addr & 0xff;
	*b_ptr++ = reg_list[0].val & 0xff;
	*b_ptr++ = reg_list[1].val & 0xff;
	ret = s5k3h2y_write_bulk_reg(info->i2c_client, info->i2c_trans_buf, 4);

	return 0;
}

static int s5k3h2y_set_gain(struct s5k3h2y_info *info, u16 gain)
{
	int ret;
	struct s5k3h2y_reg reg_list[2];
	u8 *b_ptr = info->i2c_trans_buf;

	s5k3h2y_get_gain_reg(reg_list, gain);

	*b_ptr++ = reg_list[0].addr >> 8;
	*b_ptr++ = reg_list[0].addr & 0xff;
	*b_ptr++ = reg_list[0].val & 0xff;
	*b_ptr++ = reg_list[1].val & 0xff;
	ret = s5k3h2y_write_bulk_reg(info->i2c_client, info->i2c_trans_buf, 4);

	return ret;
}

static int s5k3h2y_set_group_hold(struct s5k3h2y_info *info,
	struct s5k3h2y_ae *ae)
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
	if ((count >= 2) || (ae->coarse_time_enable) || (ae->frame_length_enable))
		groupHoldEnabled = true;

	if (groupHoldEnabled) {
		ret = s5k3h2y_write_reg(info->i2c_client, 0x0104, 0x01);
		if (ret)
			return ret;
	}

	if (ae->gain_enable)
		s5k3h2y_set_gain(info, ae->gain);
	if (ae->coarse_time_enable)
		s5k3h2y_set_coarse_time(info, ae->coarse_time);
	if (ae->frame_length_enable)
		s5k3h2y_set_frame_length(info, ae->frame_length);

	if (groupHoldEnabled) {
		ret = s5k3h2y_write_reg(info->i2c_client, 0x0104, 0x0);
		if (ret)
			return ret;
	}

	return 0;
}

#if 0
static int s5k3h2y_test_pattern(struct s5k3h2y_info *info,
			       enum s5k3h2y_test_pattern pattern)
{
	if (pattern >= ARRAY_SIZE(test_pattern_modes))
		return -EINVAL;

	return s5k3h2y_write_table(info,
				  test_pattern_modes[pattern],
				  NULL, 0);
}
#endif

static long s5k3h2y_ioctl(struct file *file,
			 unsigned int cmd, unsigned long arg)
{
	struct s5k3h2y_info *info = file->private_data;
	int ret;
	//Yushan_Write_Exp_Time_Gain(exposure_coarse_time,exposure_gain); //HTC_Steven
	pr_debug("%s ++%x\n", __func__, cmd);

	while (!info->power_state){
		pr_debug("%s power is not on, let's wait\n", __func__);
		msleep(5);
	};

	switch (cmd) {
	case S5K3H2Y_IOCTL_SET_MODE:
	{
		struct s5k3h2y_mode mode;
		if (copy_from_user(&mode,
				   (const void __user *)arg,
				   sizeof(struct s5k3h2y_mode))) {
			pr_info("%s %d\n", __func__, __LINE__);
			return -EFAULT;
		}
		pr_debug("%s: S5K3H2Y_IOCTL_SET_MODE\n", __func__);
		mutex_lock(&info->camera_lock);
		ret = s5k3h2y_set_mode(info, &mode);
		mutex_unlock(&info->camera_lock);
		return ret;
	}
	case S5K3H2Y_IOCTL_SET_FRAME_LENGTH:
		pr_debug("%s: S5K3H2Y_IOCTL_SET_FRAME_LENGTH\n", __func__);
		mutex_lock(&info->camera_lock);
		ret = s5k3h2y_set_frame_length(info, (u32)arg);
		mutex_unlock(&info->camera_lock);
		return ret;
	case S5K3H2Y_IOCTL_SET_COARSE_TIME:
		pr_debug("%s: S5K3H2Y_IOCTL_SET_COARSE_TIME\n", __func__);
		exposure_coarse_time = arg; //HTC_Steven
		mutex_lock(&info->camera_lock);
		ret = s5k3h2y_set_coarse_time(info, (u32)arg);
		mutex_unlock(&info->camera_lock);
		return ret;
	case S5K3H2Y_IOCTL_SET_GAIN:
		pr_debug("%s: S5K3H2Y_IOCTL_SET_GAIN\n", __func__);
		exposure_gain = arg; //HTC_Steven
		mutex_lock(&info->camera_lock);
		ret = s5k3h2y_set_gain(info, (u16)arg);
		mutex_unlock(&info->camera_lock);
		return ret;
	case S5K3H2Y_IOCTL_GET_STATUS:
	{
		u16 status = 0;
		pr_debug("%s: S5K3H2Y_IOCTL_GET_STATUS\n", __func__);
		if (copy_to_user((void __user *)arg, &status,
				 2)) {
			pr_info("%s %d\n", __func__, __LINE__);
			return -EFAULT;
		}
		return 0;
	}
	case S5K3H2Y_IOCTL_SET_GROUP_HOLD:
	{
		struct s5k3h2y_ae ae;
		if (copy_from_user(&ae,
				(const void __user *)arg,
				sizeof(struct s5k3h2y_ae))) {
			pr_info("%s %d\n", __func__, __LINE__);
			return -EFAULT;
		}
		pr_debug("%s: S5K3H2Y_IOCTL_SET_GROUP_HOLD\n", __func__);
		mutex_lock(&info->camera_lock);
		ret = s5k3h2y_set_group_hold(info, &ae);
		mutex_unlock(&info->camera_lock);
		return ret;
	}
#if 0
	case S5K3H2Y_IOCTL_TEST_PATTERN:
	{
		err = s5k3h2y_test_pattern(info, (enum s5k3h2y_test_pattern) arg);
		if (err)
			pr_err("%s %d %d\n", __func__, __LINE__, err);
		return err;
	}
#endif
	default:
		return -EINVAL;
	}
	return 0;
}

static int s5k3h2y_open(struct inode *inode, struct file *file)
{
	pr_info("%s ++\n", __func__);
	/* pr_info(KERN_INFO "START_TIME: Entering %s", __func__); */
	file->private_data = info;

	info->power_state = false;
	// If mode is NULL, set_mode will be executed.
	schedule_work(&info->set_mode_work);
	//s5k3h2y_get_status(info, &status);
	/* pr_info(KERN_INFO "START_TIME: Leaving %s", __func__); */
	pr_info("%s --\n", __func__);
	return 0;
}

int s5k3h2y_release(struct inode *inode, struct file *file)
{
	pr_info("%s ++\n", __func__);
	if (info->pdata && info->pdata->power_off)
		info->pdata->power_off();
	file->private_data = NULL;
	info->power_state = false;
	cancel_work_sync(&info->set_mode_work);

	pr_info("%s --\n", __func__);
	return 0;
}


static const struct file_operations s5k3h2y_fileops = {
	.owner = THIS_MODULE,
	.open = s5k3h2y_open,
	.unlocked_ioctl = s5k3h2y_ioctl,
	.release = s5k3h2y_release,
};

static struct miscdevice s5k3h2y_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "s5k3h2y",
	.fops = &s5k3h2y_fileops,
};

/* HTC START */
static const char *S5K3H2YVendor = "samsung";
static const char *S5K3H2YNAME = "s5k3h2y";
static const char *S5K3H2YSize = "8M";
static uint32_t htcwc_value;

static ssize_t sensor_vendor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;

	sprintf(buf, "%s %s %s\n", S5K3H2YVendor, S5K3H2YNAME, S5K3H2YSize);
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

static struct kobject *android_s5k3h2y;

static int s5k3h2y_sysfs_init(void)
{
	int ret ;
	pr_info("[CAM] %s: kobject creat and add\n", __func__);
	android_s5k3h2y = kobject_create_and_add("android_camera", NULL);
	if (android_s5k3h2y == NULL) {
		pr_info("[CAM] %s: subsystem_register " \
		"failed\n", __func__);
		ret = -ENOMEM;
		return ret ;
	}
	pr_info("[CAM] %s:sysfs_create_file\n", __func__);
	ret = sysfs_create_file(android_s5k3h2y, &dev_attr_sensor.attr);
	if (ret) {
		pr_info("[CAM] %s: sysfs_create_file " \
		"failed\n", __func__);
		kobject_del(android_s5k3h2y);
	}

	ret = sysfs_create_file(android_s5k3h2y, &dev_attr_htcwc.attr);
	if (ret) {
		pr_info("[CAM] %s: sysfs_create_file htcwc failed\n", __func__);
		kobject_del(android_s5k3h2y);
	}

       ret = sysfs_create_file(android_s5k3h2y, &dev_attr_node.attr);
	if (ret) {
		pr_info("[CAM] %s: dev_attr_node failed\n", __func__);
		kobject_del(android_s5k3h2y);
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
static struct camera_led_est camera_sensor_s5k3h2yx_led_table[] = {
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

static struct camera_led_info camera_sensor_s5k3h2yx_led_info = {
	.enable = 1,
	.low_limit_led_state = FL_MODE_TORCH,
	.max_led_current_ma = 750,
	.num_led_est_table = ARRAY_SIZE(camera_sensor_s5k3h2yx_led_table),
};

static struct camera_flash_info camera_sensor_s5k3h2yx_flash_info = {
	.led_info = &camera_sensor_s5k3h2yx_led_info,
	.led_est_table = camera_sensor_s5k3h2yx_led_table,
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
		pr_info("[CAM]msm_camera: sysfs_create_file flash_led_info failed\n");
		ret = -EFAULT;
		goto error;
	}

	ret = sysfs_create_file(led_status_obj,
		&dev_attr_flash_led_tbl.attr);
	if (ret) {
		pr_info("[CAM]msm_camera: sysfs_create_file flash_led_tbl failed\n");
		ret = -EFAULT;
		goto error;
	}

	p_flash_led_info = &camera_sensor_s5k3h2yx_flash_info;
	/* Linear LED */

	led_low_temp_limit = 10;
	led_low_cap_limit = 15;

	return ret;
error:
	kobject_del(led_status_obj);
	return ret;
}
/* HTC END */
static int s5k3h2y_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int err;
	pr_info("%s: probing sensor.\n", __func__);

	info = kzalloc(sizeof(struct s5k3h2y_info), GFP_KERNEL);
	if (!info) {
		pr_err("s5k3h2y: Unable to allocate memory!\n");
		return -ENOMEM;
	}

	err = misc_register(&s5k3h2y_device);
	if (err) {
		pr_err("s5k3h2y: Unable to register misc device!\n");
		kfree(info);
		return err;
	}

	info->pdata = client->dev.platform_data;
	info->i2c_client = client;

	i2c_set_clientdata(client, info);

	INIT_WORK(&info->set_mode_work, set_mode_handler);
	mutex_init(&info->camera_lock);

	/* HTC START */
	s5k3h2y_sysfs_init();
	camera_led_sysfs_init();
	/* HTC END */

	s5k3h2_i2c_client = client;
	return 0;
}

static int s5k3h2y_remove(struct i2c_client *client)
{
	struct s5k3h2y_info *info;
	info = i2c_get_clientdata(client);
	misc_deregister(&s5k3h2y_device);
	kfree(info);
	return 0;
}

static const struct i2c_device_id s5k3h2y_id[] = {
	{ "s5k3h2y", 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, s5k3h2y_id);

static struct i2c_driver s5k3h2y_i2c_driver = {
	.driver = {
		.name = "s5k3h2y",
		.owner = THIS_MODULE,
	},
	.probe = s5k3h2y_probe,
	.remove = s5k3h2y_remove,
	.id_table = s5k3h2y_id,
};

static int __init s5k3h2y_init(void)
{
	pr_info("s5k3h2y sensor driver loading\n");
	return i2c_add_driver(&s5k3h2y_i2c_driver);
}

static void __exit s5k3h2y_exit(void)
{
	i2c_del_driver(&s5k3h2y_i2c_driver);
}

module_init(s5k3h2y_init);
module_exit(s5k3h2y_exit);

	
