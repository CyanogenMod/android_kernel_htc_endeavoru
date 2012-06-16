
#ifndef __EXTERNAL_COMMON_H__
#define __EXTERNAL_COMMON_H__

#ifdef CONFIG_TEGRA_HDMI_MHL_SUPERDEMO
typedef unsigned char uint8;
typedef int uint16;
typedef unsigned int uint32;

extern bool g_bDemoTvFound;

/*
  We used the block#0 offset 0x6C Monitor Descriptor be the fixed HTC privite descriptor
  --Descriptor data layout--
  Offset 0x00-0x01   (2 Bytes)   Flag = 0x0000 to indicate to use as descriptor.
  Offset 0x02-0x02   (1 Bytes)   Reserved = 0x00 to indicate to use as descriptor.
  Offset 0x03-0x03   (1 Bytes)   Data Type Tag =  0x0F means "Descriptor defined by manufactureer
  Offset 0x04-0x04   (1 Bytes)   Flag = 0x00 to indicate to use as descriptor.
  Offset 0x05-0x06   (2 Bytes)   Magic string with version check, offset 0x00 means version, 0x01 was inverse vaule of offset 0
  Offset 0x07-0x08   (2 Bytes)   SUPERDEMO_TV_ID
  Offset 0x09-0x0A   (2 Bytes)   1st prefer timing description (BIT17-BIT14: Block#, BIT13-BIT0: Block offset)
  Offset 0x0B-0x0C   (2 Bytes)   2nd prefer timing description (BIT17-BIT14: Block#, BIT13-BIT0: Block offset)
  Offset 0x0D-0x0E   (2 Bytes)   Special feature bit identify
    BIT0: Optical position sensor
    BIT1: RCP support
    BIT2: Proximity sensor support
    BIT3: Sonar sensor support
    BIT4-BIT23: Reserved for feature
  Offset 0x0F-0x10   (2 Bytes)   Reserved for feature
  Offset 0x11-0x11   (1 Bytes)   Checksum
*/

/* 1. Remember to use little-endian convention.
   2. keep reserved bits be zeros for compatibility */

#define DEMOTV_DESC_FLAG0			0
#define DEMOTV_DESC_FLAG1			2
#define DEMOTV_DESC_CSTM			3
#define DEMOTV_DESC_FLAG2			4
#define DEMOTV_DESC_DATA			5
#define DEMOTV_DESC_VERSION			5 /* all versions have the same VERSION offset */
#define DEMOTV_DESC_MAGIC			6 /* all versions have the same MAGIC offset */
#define DEMOTV_DESC_TV_ID			7 /* definitions below are just for reference of version 1 */
#define DEMOTV_DESC_1ST_TIMING		9
#define DEMOTV_DESC_2ND_TIMING		11
#define DEMOTV_DESC_FEATURES		13
	#define DEMOTV_BIT_OPTICAL			0x0001
	#define DEMOTV_BIT_RCP				0x0002
	#define DEMOTV_BIT_PROXIMITY		0x0004
	#define DEMOTV_BIT_SONAR			0x0008
#define DEMOTV_DESC_RESERVED		15
#define DEMOTV_DESC_CHECKSUM		17

/* dummy */
struct st_demotv_data_v0 {
	uint8 version;
	uint8 magic;
	uint8 data[11];
};

struct st_demotv_data_v1 {
	uint8 version;
	uint8 magic;
	uint16 tv_id;
	uint16 timing_1st;
	uint16 timing_2nd;
	uint16 features;
	uint16 reserved;
	uint8 checksum;
};

struct st_demotv_patterns {
	const char vendor_id[4];
};

#endif /* CONFIG_FB_MSM_HDMI_MHL_SUPERDEMO */

#endif
