#ifndef YUSHAN_HTC_FUNCTIONS_H
#define YUSHAN_HTC_FUNCTIONS_H

#ifdef __cplusplus
extern "C"{
#endif   /*__cplusplus*/

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/delay.h>

enum yushan_sensor_id {
	YUSHAN_S5K3H2YX = 0,
	YUSHAN_OV8838,
	YUSHAN_S5K6A1GX03,
	YUSHAN_S5K6A2,
};
#include <media/rawchip/rawchip.h>
#include <media/rawchip/yushan_registermap.h>
#include <media/rawchip/rawchip_spi.h>

#include "Yushan_API.h"
#include "Yushan_Platform_Specific.h"


#define SENSOR_FULL_SIZE_WIDTH 			3280
#define SENSOR_FULL_SIZE_HEIGHT 		2464
#define SENSOR_VIDEO_SIZE_WIDTH 		3084
#define SENSOR_VIDEO_SIZE_HEIGHT 		1736
#define SENSOR_VIDEO_SIZE_WIDTH_FAST 	 768
#define SENSOR_VIDEO_SIZE_HEIGHT_FAST 	 432
#define SENSOR_QTR_SIZE_WIDTH 			1640
#define SENSOR_QTR_SIZE_HEIGHT 			1232

#define SENSOR_VER_FULL_BLK_LINES 		 190
#define SENSOR_VER_VIDEO_BLK_LINES		 386
#define SENSOR_VER_VIDEO_BLK_LINES_FAST 2708
#define SENSOR_VER_QTR_BLK_LINES 		1830
#define SENSOR_ORIENTATION				   0


enum yushan_orientation_type {
	YUSHAN_ORIENTATION_NONE,
	YUSHAN_ORIENTATION_MIRROR,
	YUSHAN_ORIENTATION_FLIP,
	YUSHAN_ORIENTATION_MIRROR_FLIP,
};

struct yushan_reg_conf {
	uint16_t addr;
	uint8_t  data;
};

struct yushan_reg_t {
	uint16_t pdpcode_first_addr;
	uint8_t  *pdpcode;
	uint16_t pdpcode_size;

	uint16_t pdpclib_first_addr;
	uint8_t *pdpclib;
	uint16_t pdpclib_size;

	uint16_t pdpBootAddr;
	uint16_t pdpStartAddr;

	uint16_t dppcode_first_addr;
	uint8_t *dppcode;
	uint16_t dppcode_size;

	uint16_t dppclib_first_addr;
	uint8_t *dppclib;
	uint16_t dppclib_size;

	uint16_t dppBootAddr;
	uint16_t dppStartAddr;

	uint16_t dopcode_first_addr;
	uint8_t *dopcode;
	uint16_t dopcode_size;

	uint16_t dopclib_first_addr;
	uint8_t *dopclib;
	uint16_t dopclib_size;

	uint16_t dopBootAddr;
	uint16_t dopStartAddr;
};

extern struct yushan_reg_t yushan_regs_s5k3h2yx;
extern struct yushan_reg_t yushan_regs_ov8838;
extern struct yushan_reg_t yushan_regs_s5k6a2gx;




struct rawchip_sensor_init_data {
	const char *sensor_name;
	uint8_t spi_clk;
	uint8_t ext_clk;
	uint8_t lane_cnt;
	uint8_t orientation;
	uint8_t use_ext_1v2;
	uint16_t bitrate;
	uint16_t width;
	uint16_t height;
	uint16_t blk_pixels;
	uint16_t blk_lines;
	uint16_t x_addr_start;
	uint16_t y_addr_start;
	uint16_t x_addr_end;
	uint16_t y_addr_end;
	uint16_t x_even_inc;
	uint16_t x_odd_inc;
	uint16_t y_even_inc;
	uint16_t y_odd_inc;
	uint8_t binning_rawchip;
	uint8_t use_rawchip;/* HTC_START_Simon.Ti_Liu_20120702_Enhance_bypass */
};

typedef struct {
  uint16_t gain;
  uint16_t line;
} rawchip_aec_params_t;

typedef struct {
  uint8_t rg_ratio; /* Q6 format */
  uint8_t bg_ratio; /* Q6 format */
} rawchip_awb_params_t;

typedef struct {
  int update;
  rawchip_aec_params_t aec_params;
  rawchip_awb_params_t awb_params;
} rawchip_update_aec_awb_params_t;

typedef struct {
  uint8_t active_number;
  Yushan_AF_ROI_t sYushanAfRoi[5];
} rawchip_af_params_t;

typedef struct {
  int update;
  rawchip_af_params_t af_params;
} rawchip_update_af_params_t;


typedef struct
{
  Yushan_AF_Stats_t udwAfStats[5];
  uint16_t frameIdx;
} rawchip_af_stats;

#ifdef YUSHAN_API_DEBUG
#define CDBG(fmt, args...) pr_info(fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#endif

void Reset_Yushan(void);


int Yushan_sensor_open_init(struct rawchip_sensor_init_data data, bool *clock_init_done);
void Yushan_dump_register(void);
void Yushan_dump_all_register(void);

/* Status Functions */
bool_t	Yushan_ContextUpdate_Wrapper(Yushan_New_Context_Config_t	sYushanNewContextConfig, Yushan_ImageChar_t	sImageNewChar_context);


bool_t Yushan_Dxo_Dop_Af_Run(Yushan_AF_ROI_t	*sYushanAfRoi, uint32_t *pAfStatsGreen, uint8_t	bRoiActiveNumber);
int Yushan_get_AFSU(rawchip_af_stats* af_stats);



void select_mode(uint8_t mode);



int Yushan_Update_AEC_AWB_Params(rawchip_update_aec_awb_params_t *update_aec_awb_params);
int Yushan_Update_AF_Params(rawchip_update_af_params_t *update_af_params);
int Yushan_Update_3A_Params(uint8_t enable_newframe_ack);


#ifdef __cplusplus
}
#endif   /*__cplusplus*/

#endif
