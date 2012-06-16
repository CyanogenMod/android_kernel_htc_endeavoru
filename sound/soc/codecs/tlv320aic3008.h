/*
 * ALSA SoC TLV320AIC3008 codec driver
 *
 * Copyright (C) 2010-2011 HTC Corporation.
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

#ifndef _AIC3008_H
#define _AIC3008_H

#include <sound/soc.h>
#include <linux/regulator/consumer.h>
#include <linux/ioctl.h> /* for defining the IO controls */
#include <../../../arch/arm/mach-tegra/gpio-names.h>

/*****************************************************************************/
/* IO CONTROL definition for AIC3008                                         */
/*****************************************************************************/
#define AIC3008_IO_IOCTL_MAGIC     's'
#define AIC3008_IO_SET_TX_PARAM     _IOW(AIC3008_IO_IOCTL_MAGIC, 0x10, unsigned)
#define AIC3008_IO_SET_RX_PARAM     _IOW(AIC3008_IO_IOCTL_MAGIC, 0x11, unsigned)
#define AIC3008_IO_CONFIG_TX        _IOW(AIC3008_IO_IOCTL_MAGIC, 0x12, unsigned int)
#define AIC3008_IO_CONFIG_RX        _IOW(AIC3008_IO_IOCTL_MAGIC, 0x13, unsigned int)
#define AIC3008_IO_SET_DSP_PARAM    _IOW(AIC3008_IO_IOCTL_MAGIC, 0x20, unsigned)
#define AIC3008_IO_CONFIG_MEDIA     _IOW(AIC3008_IO_IOCTL_MAGIC, 0x21, unsigned int)
#define AIC3008_IO_CONFIG_VOICE     _IOW(AIC3008_IO_IOCTL_MAGIC, 0x22, unsigned int)
#define AIC3008_IO_CONFIG_VOLUME_L  _IOW(AIC3008_IO_IOCTL_MAGIC, 0x23, unsigned int)
#define AIC3008_IO_CONFIG_VOLUME_R  _IOW(AIC3008_IO_IOCTL_MAGIC, 0x24, unsigned int)
#define AIC3008_IO_POWERDOWN        _IOW(AIC3008_IO_IOCTL_MAGIC, 0x25, unsigned int)
#define AIC3008_IO_LOOPBACK         _IOW(AIC3008_IO_IOCTL_MAGIC, 0x26, unsigned int)
#define AIC3008_IO_DUMP_PAGES       _IOW(AIC3008_IO_IOCTL_MAGIC, 0x30, unsigned int)
#define AIC3008_IO_READ_REG         _IOWR(AIC3008_IO_IOCTL_MAGIC, 0x31, unsigned)
#define AIC3008_IO_WRITE_REG        _IOW(AIC3008_IO_IOCTL_MAGIC, 0x32, unsigned)
#define AIC3008_IO_RESET            _IOW(AIC3008_IO_IOCTL_MAGIC, 0x33, unsigned int)
#define AIC3008_IO_DUMP_DSP         _IOW(AIC3008_IO_IOCTL_MAGIC, 0x34, unsigned int)
#define AIC3008_IO_GET_PCBID        _IOR(AIC3008_IO_IOCTL_MAGIC, 0x35, unsigned int)

#define AIC3008_MAX_PAGES        255
#define AIC3008_MAX_REGS         128
#define AIC3008_MAX_RETRY        10

#define IO_CTL_ROW_MAX      64
#define IO_CTL_COL_MAX      1024
#define MINIDSP_ROW_MAX     32
#define MINIDSP_COL_MAX     20000

/* structures for SPI commands */
typedef struct _CODEC_SPI_CMD {
	unsigned char act;
	unsigned char reg;
	unsigned char data;
} CODEC_SPI_CMD;

typedef struct _CODEC_SPI_CMD_PARAM {
	CODEC_SPI_CMD *data;
	unsigned int len;
} CODEC_SPI_CMD_PARAM;

struct AIC3008_PARAM {
	unsigned int row_num;
	unsigned int col_num;
	void *cmd_data;
};

struct CODEC_CFG {
	unsigned char tb_idx;
	unsigned char index;
};


/* enumerated modes */
enum aic3008_uplink_mode {
	INITIAL = 0,
	CALL_UPLINK_IMIC_RECEIVER = 1,
	CALL_UPLINK_EMIC_HEADPHONE,
	CALL_UPLINK_IMIC_HEADPHONE,
	CALL_UPLINK_IMIC_SPEAKER,
	CALL_UPLINK_IMIC_RECEIVER_DUALMIC,
	CALL_UPLINK_EMIC_HEADPHONE_DUALMIC,
	CALL_UPLINK_IMIC_SPEAKER_DUALMIC,
	VOICERECORD_IMIC = 8,
	VOICERECORD_EMIC,
	VIDEORECORD_IMIC,
	VIDEORECORD_EMIC,
	VOICERECOGNITION_IMIC = 12,
	VOICERECOGNITION_EMIC,
	UPLINK_BT_AP = 14,
	UPLINK_BT_BB,
	FM_IN_SPEAKER = 16,
	FM_IN_HEADPHONE,
	TTY_IN_HCO = 18,
	TTY_IN_VCO,
	TTY_IN_FULL,
	UPLINK_MUSE = 21,
	UPLINK_HAC,
	UPLINK_PATH_OFF = 23,
	UPLINK_WAKEUP,
	POWER_OFF = 25,
	STEREO_VIDEO_RECORDING_IMIC_PORTRAIT = 26,
	STEREO_VIDEO_RECORDING_IMIC_LANDSCAPE,
	VOIP_UPLINK_IMIC_RECEIVER = 28,
	VOIP_UPLINK_EMIC_HEADPHONE,
	VOIP_UPLINK_IMIC_HEADPHONE,
	VOIP_UPLINK_IMIC_SPEAKER,
	VOIP_UPLINK_BT,
	CALL_UPLINK_IMIC_RECEIVER_DUALMIC_WB = 33,
	CALL_UPLINK_EMIC_HEADPHONE_DUALMIC_WB,
	CALL_UPLINK_IMIC_SPEAKER_DUALMIC_WB,
	CALL_UPLINK_IMIC_DOCK,
	MFG_VIDEORECORD_2ndMIC = 37,
	CALL_UPLINK_EMIC_HEADPHONE_BEATS = 38,
	CALL_UPLINK_EMIC_HEADPHONE_BEATS_WB,
	VOIP_UPLINK_EMIC_HEADPHONE_BEATS,
	UPLINK_MODE_END,
};

enum aic3008_downlink_mode {
	CALL_DOWNLINK_IMIC_RECEIVER = 1,
	CALL_DOWNLINK_EMIC_HEADPHONE,
	CALL_DOWNLINK_IMIC_HEADPHONE,
	CALL_DOWNLINK_IMIC_SPEAKER,
	CALL_DOWNLINK_IMIC_RECEIVER_DUALMIC,
	CALL_DOWNLINK_EMIC_HEADPHONE_DUALMIC,
	CALL_DOWNLINK_IMIC_SPEAKER_DUALMIC,
	PLAYBACK_RECEIVER = 8,
	PLAYBACK_HEADPHONE,
	PLAYBACK_SPEAKER,
	RING_HEADPHONE_SPEAKER = 11,
	PLAYBACK_SPEAKER_ALT = 12,
	USB_AUDIO = 13,
	DOWNLINK_BT_AP = 14,
	DOWNLINK_BT_BB,
	FM_OUT_SPEAKER = 16,
	FM_OUT_HEADPHONE,
	TTY_OUT_HCO = 18,
	TTY_OUT_VCO,
	TTY_OUT_FULL,
	DOWNLINK_MUSE,
	DOWNLINK_HAC,
	DOWNLINK_PATH_OFF = 23,
	DOWNLINK_WAKEUP,
	PLAYBACK_HEADPHONE_URBEATS = 25,
	PLAYBACK_HEADPHONE_SOLO,
	PLAYBACK_SPEAKER_BEATS,
	VOIP_DOWNLINK_IMIC_RECEIVER = 28,
	VOIP_DOWNLINK_EMIC_HEADPHONE,
	VOIP_DOWNLINK_IMIC_HEADPHONE,
	VOIP_DOWNLINK_IMIC_SPEAKER,
	VOIP_DOWNLINK_BT,
	CALL_DOWNLINK_IMIC_RECEIVER_DUALMIC_WB,
	CALL_DOWNLINK_EMIC_HEADPHONE_DUALMIC_WB,
	CALL_DOWNLINK_IMIC_SPEAKER_DUALMIC_WB,
	CALL_DOWNLINK_IMIC_DOCK,
	PLAYBACK_DOCK,
	CALL_DOWNLINK_EMIC_HEADPHONE_BEATS = 38,
	CALL_DOWNLINK_EMIC_HEADPHONE_BEATS_WB,
	VOIP_DOWNLINK_EMIC_HEADPHONE_BEATS,
	CALL_DOWNLINK_IMIC_HEADPHONE_WB,
	MFG_PLAYBACK_L_SPEAKER = 42,
	MFG_PLAYBACK_R_SPEAKER,
	PLAYBACK_HEADPHONE_FULLDELPX,
	PLAYBACK_SPK_FULLDELPX,
	DOWNLINK_MODE_END,
};

enum htc_audio_sound_effect {
    MFG = 0,
	Phone_Default = 1,
	Phone_Receiver_Dualmic,
	Phone_Speaker_Dualmic,
	Phone_Headset,
	Phone_HAC,
	Phone_TTY,
	Phone_BT,
	Phone_Handset_Dualmic_WB,
	Phone_Speaker_Dualmic_WB,
	Phone_Headset_WB,
	Phone_Dock = 11,
	Playback_Default = 12,
	Playback_Speaker_Default,
	Playback_Headset_Generic,
	Playback_Headset_urBeats,
	Playback_Headset_Solo,
	Playback_Headset_Pro,
	Playback_Headset_Studio,
	Playback_SPK_Beats,
	Playback_Dock,
	Playback_SPK_Ring = 21,
	Record_Default = 22,
	Record_V_Mono,
	Record_V_IMIC_Landscape_Stereo,
	Record_V_IMIC_Portrait_Stereo,
	Record_A_VoiceRecoder_AMR,
	Record_A_VoiceRecoder_AAC,
	Record_A_Note_Recording,
	Record_A_Speaker_VR = 29,
	FM_Headset = 30,
	FM_Speaker = 31,
	VOIP_Receiver = 32,
	VOIP_Speaker,
	VOIP_Headset,
	VOIP_BT = 35,
	SKYPE_Receiver = 36,
	SKYPE_Speaker,
	SPYPE_Headset = 38,
    End_Audio_Effect,
};


/* control operations structure */
struct aic3008_ctl_ops {
	void (*tx_amp_enable)(int en);
	void (*rx_amp_enable)(int en);
	int (*panel_sleep_in)(void);
	void (*reset_3008)(void);
	void (*spibus_enable)(int en);
	CODEC_SPI_CMD_PARAM *downlink_off;
	CODEC_SPI_CMD_PARAM *uplink_off;
	CODEC_SPI_CMD_PARAM *downlink_on;
	CODEC_SPI_CMD_PARAM *uplink_on;
	CODEC_SPI_CMD_PARAM *lb_dsp_init;
	CODEC_SPI_CMD_PARAM *lb_downlink_receiver;
	CODEC_SPI_CMD_PARAM *lb_downlink_speaker;
	CODEC_SPI_CMD_PARAM *lb_downlink_headset;
	CODEC_SPI_CMD_PARAM *lb_uplink_imic;
	CODEC_SPI_CMD_PARAM *lb_uplink_emic;
	CODEC_SPI_CMD_PARAM *lb_receiver_imic;
	CODEC_SPI_CMD_PARAM *lb_speaker_imic;
	CODEC_SPI_CMD_PARAM *lb_headset_emic;
	CODEC_SPI_CMD_PARAM *lb_receiver_bmic;
	CODEC_SPI_CMD_PARAM *lb_speaker_bmic;
	CODEC_SPI_CMD_PARAM *lb_headset_bmic;
};

extern void aic3008_register_ctl_ops(struct aic3008_ctl_ops *ops);
void aic3008_CodecInit(void);
int aic3008_setMode(int cmd, int idx, int is_call_mode);
void aic3008_set_mic_bias(int en);




/*****************************************************************************/
/* AIC3008 register addresses                                                */
/*****************************************************************************/
#define PAGE0							0
#define PAGE1							1
#define PAGE3							3
#define PAGE4							4

#define AIC3008_PAGE_SELECT				0	/* Page select register */
#define AIC3008_SW_RESET				1	/* Software reset register */

/**********/
/* Page 0 */
/**********/
/* CLOCKS */
#define AIC3008_CLK_DACADC_SEL			4	/* DAC/ADC selection register */
#define AIC3008_CLK_RANGE_SEL			5	/* PPL clock range selection control */
#define AIC3008_CLK_PPL_PR_FACTOR		6	/* PPL Power(Divider) and R(Multiplier) */
#define AIC3008_CLK_PPL_J_FACTOR		7	/* PPL J Multiplier*/
#define AIC3008_CLK_PPL_D_FACTOR_MSB	8	/* PPL D MSB 6 bits of 14-bits fraction */
#define AIC3008_CLK_PPL_D_FACTOR_LSB	9	/* PPL D LSB 8 bits of 14-bits fraction */
#define AIC3008_CLK_PPL_CLKIN			10	/* PPL PLL CLKIN Divider */
#define AIC3008_CLK_NDAC				11	/* NDAC divider power and value */
#define AIC3008_CLK_MDAC				12	/* MDAC divider power and value */
#define AIC3008_CLK_DAC_OSR_MSB			13	/* DAC OSR MSB */
#define AIC3008_CLK_DAC_OSR_LSB			14	/* DAC OSR LSB */

#define AIC3008_CLK_NADC				18	/* NADC divider power and value */
#define AIC3008_CLK_MADC				19	/* MADC divider power and value */
#define AIC3008_CLK_AOSR				20	/* ADC Over Sampling Rate (AOSR) */
#define AIC3008_CLK_OUT_MUX				21	/* CLKOUT MUX */
#define AIC3008_CLK_OUT_MDIV			22	/* CLKOUT M divider pwr and value */

/* Timer/Frequency/Clock generations */

/* Flags */
#define AIC3008_ADC_FLAG				36	/* ADC Flag Register */
#define AIC3008_DAC_FLAG				37	/* DAC Flag Register */
#define AIC3008_DAC_PGA_FLAG			38	/* DAC PGA Flag Register */
#define AIC3008_STICKY1_FLAG			42	/* Sticky Flag 1 Register */
#define AIC3008_INT1_FLAG				43	/* Interrupt Flag 1 Register */
#define AIC3008_STICKY2_FLAG			44	/* Sticky Flag 2 Register */
#define AIC3008_STICKY3_FLAG			45	/* Sticky Flag 3 Register */
#define AIC3008_INT2_FLAG				46	/* Interrupt Flag 2 Register */
#define AIC3008_INT3_FLAG				47	/* Interrupt Flag 3 Register */
#define AIC3008_INT1_CTL				48	/* Interrupt 1 control register */
#define AIC3008_INT2_CTL				49	/* Interrupt 2 control register */
#define AIC3008_SAR_CTL					50	/* SAR control 1 */
#define AIC3008_INT_FMT_CTL				51	/* Interrupt Format Control Register */

/* DAC */
#define AIC3008_DAC_PBLOCK_CTL			60	/* DAC Processing Block and MiniDSP Power Control */
#define AIC3008_ADC_PBLOCK_CTL			61	/* ADC Processing Block and MiniDSP Power Control */
#define AIC3008_DAC_PATH_SETUP			63	/* PRI ASI DAC Datapath Setup */
#define AIC3008_DAC_MASTER_VOL			64	/* Primary DAC Master Volume Config */
#define AIC3008_DAC_LEFT_VOL			65	/* Primary DAC Left Vol Control Setting */
#define AIC3008_DAC_RIGHT_VOL			66	/* Primary DAC Right Vol Control Setting */
#define AIC3008_HEADSET_DET				67	/* Headset Detection */
#define AIC3008_DRC_CTL1				68	/* DRC control register 1 */
#define AIC3008_DRC_CTL2				69	/* DRC control register 2 */
#define AIC3008_DRC_CTL3				70	/* DRC control register 3 */

/* BEEP */

/* ADC */
#define AIC3008_ADC_CH_POWER			81	/* ADC channel power control */
#define AIC3008_ADC_FINE_GAIN_VOL		82	/* ADC fine gain volume control */
#define AIC3008_ADC_LEFT_VOL			83	/* Left ADC Vol */
#define AIC3008_ADC_RIGHT_VOL			84	/* Right ADC Vol */
#define AIC3008_ADC_PHASE				85	/* ADC phase control */

/* Left AGC */
#define AIC3008_LAGC_CTL1				86	/* Left AGC control 1 */
#define AIC3008_LAGC_CTL2				87	/* Left AGC control 2 */
#define AIC3008_LAGC_CTL3				88	/* Left AGC control 3 */
#define AIC3008_LAGC_ATT_TIME			89	/* Left AGC attach time setting */
#define AIC3008_LAGC_DCAY_TIME			90	/* Left AGC decay time setting */
#define AIC3008_LAGC_NOISE_DBOUNCE		91	/* Left AGC noise debounce */
#define AIC3008_LAGC_SIG_DBOUNCE		92	/* Left AGC signal debounce */
#define AIC3008_LAGC_GAIN				93	/* Left AGC gain */

/* Right AGC */
#define AIC3008_RAGC_CTL1				94	/* Right AGC control 1 */
#define AIC3008_RAGC_CTL2				95	/* Right AGC control 2 */
#define AIC3008_RAGC_CTL3				96	/* Right AGC control 3 */
#define AIC3008_RAGC_ATT_TIME			97	/* Right AGC attach time setting */
#define AIC3008_RAGC_DCAY_TIME			98	/* Right AGC decay time setting */
#define AIC3008_RAGC_NOISE_DBOUNCE		99	/* Right AGC noise debounce */
#define AIC3008_RAGC_SIG_DBOUNCE		100	/* Right AGC signal debounce */
#define AIC3008_RAGC_GAIN				101	/* Right AGC gain */

/* DC measurement */
#define AIC3008_ADC_DC_MEASURE1			102	/* ADC DC measurement 1 */
#define AIC3008_ADC_DC_MEASURE2			103	/* ADC DC measurement 2 */
#define AIC3008_ADC_DC_LCH_MEASURE1		104	/* Left Ch DC measurement out 1 */
#define AIC3008_ADC_DC_LCH_MEASURE2		105	/* Left Ch DC measurement out 2 */
#define AIC3008_ADC_DC_LCH_MEASURE3		106	/* Left Ch DC measurement out 3 */
#define AIC3008_ADC_DC_RCH_MEASURE1		107	/* Right Ch DC measurement out 1 */
#define AIC3008_ADC_DC_RCH_MEASURE2		108	/* Right Ch DC measurement out 2 */
#define AIC3008_ADC_DC_RCH_MEASURE3		109	/* Right Ch DC measurement out 3 */

/* misc */
#define AIC3008_I2C_MISC_CTL			115	/* I2C I/F misc control */
#define AIC3008_DSP_CTL					121	/* miniDSP control reg, register access ctrl */

/**********/
/* Page 1 */
/**********/
#define AIC3008_PWR_CFG					1	/* Power Config Reg */
#define AIC3008_PLAYBACK_LDAC_PTM		3	/* Playback Config Reg LDAC PTM */
#define AIC3008_PLAYBACK_RDAC_PTM		4	/* Playback Config Reg RDAC PTM */

#define AIC3008_COMMON_MODE				8	/* Common Mode for input and output */
#define AIC3008_HP_OUT_DRV				9	/* Head phone output driver control */
#define AIC3008_RCVR_OUT_DRV			10	/* Receiver Output Driver Control */
#define AIC3008_HP_OUT_DRV_DEPOP		11	/* Head phone output driver depop control */
#define AIC3008_RCV_OUT_DRV_DEPOP		12	/* Receiver output driver depop control */
#define AIC3008_MIXER_AMP				17	/* Mixer AMP control */

#define AIC3008_LPGA_MAL_VOL			18	/* Left ADC PGA to Left Mixer AMP (MAL) Vol */
#define AIC3008_RPGA_MAR_VOL			19	/* Right ADC PGA to Right Mixer AMP (MAR) Vol */

#define AIC3008_LOUT_AMP_CTL1			22	/* Route L/R DAC to Lineout Driver and Power */
#define AIC3008_LOUT_AMP_CTL2			23	/* Route Mixer/IN1L to Lineout Driver and Power */

#define AIC3008_HP_AMP_CTL1				27	/* MIXER/DAP to HP driver route and power */
#define AIC3008_HP_AMP_CTL2				28	/* HP AMP CTL Vol in dB */
#define AIC3008_HP_AMP_CTL3				29	/* LO output to HP Driver Master Vol */
#define AIC3008_HPL_VOL					31	/* HPL driver vol -6dB to 14 dB */
#define AIC3008_HPR_VOL					32	/* HPR driver vol -6dB to 14 dB */

#define AIC3008_CHG_PUMP_CLK_DIV		33	/* Charge Pump Clock Div */
#define AIC3008_CHG_PUMP_PWR_CFG		34	/* Charge Pump Power Config */
#define AIC3008_CHG_PUMP_PWR			35	/* Charge Pump Power Control */

#define AIC3008_RCVL_LOL_LVOL			36	/* LOL output to RCVL driver Vol 0~-72.3 dB */
#define AIC3008_RCVR_LOR_RVOL			37	/* LOR output to RCVR driver Vol 0~-72.3 dB + ch lock */
#define AIC3008_RCVL_IN1L_LVOL			38	/* IN1L output to RCVL drivere Vol 0~-72.3 dB */
#define AIC3008_RCVR_IN1R_RVOL			39	/* IN1R output to RCVR drivere Vol 0~-72.3 dB + ch lock */
#define AIC3008_RCV_PWR_RCVL_LVOL		40	/* RCV power + RCVL driver vol -6 ~ 29dB */
#define AIC3008_RCV_LOCK_RCVR_RVOL		41	/* RCV ch lock + RCVR driver vol -6 ~ 29dB */
#define AIC3008_RCV_CALIBRATE			42	/* RCV amp calibrate */

#define AIC3008_SPK_PWR					45	/* L/R SPK pwr + Mixer Route */
#define AIC3008_SPKL_LOL_LVOL			46	/* LOL output to SPKL driver Vol 0~-72.3 dB */
#define AIC3008_SPKR_LOR_RVOL			46	/* LOR output to SPKR driver Vol 0~-72.3 dB + ch lock */
#define AIC3008_SPK_AMP_VOL				47	/* SPK L/R AMP vol 6 ~ 30 dB */

#define AIC3008_MIC_BIAS				51	/* Mic Bias Control */

#define AIC3008_IN_SEL_LMICPGA_P		52	/* Input select 1~3 for Left Mic PGA P-term */
#define AIC3008_IN4_SEL_LMICPGA_P		53	/* Input select 4 for Left Mic PGA P-term */
#define AIC3008_IN_SEL_LMICPGA_M		54	/* Input select for Left Mic PGA M-term */
#define AIC3008_IN_SEL_RMICPGA_P		55	/* Input select 1~3 for Right Mic PGA P-term */
#define AIC3008_IN4_SEL_RMICPGA_P		56	/* Input select 4 for Right Mic PGA P-term */
#define AIC3008_IN_SEL_RMICPGA_M		57	/* Input select for Right Mic PGA M-term */
#define AIC3008_IN_COMMON_MODE			58	/* Input Common Mode control */
#define AIC3008_LMICPGA_LVOL			59	/* Left Mic PGA enable + vol 0~47.5 dB */
#define AIC3008_RMICPGA_RVOL			60	/* Right Mic PGA enable + vol 0~47.5 dB */

#define AIC3008_ADC_PWR_TUNE_CFG		61	/* PTM_R4 ~ PTM_R1 */

/* Flags */
#define AIC3008_ADC_PGA_FLAG			62	/* L/R ch volume == or != */
#define AIC3008_DAC_GAIN_FLAG			63	/* HP + RCV gain == or != */
#define AIC3008_IN_MIXER_GAIN_FLAG		64	/* LO to HP/RCV/SPK gain == or != , CP on/off */
#define AIC3008_IN_RCV_GAIN_FLAG		65	/* IN1 to RCV == or !=, ADC PGA to MIXER AMP == or != */
#define AIC3008_DRV_PWR_FLAG			66	/* LO/HP/RCV/SPK power up/down */

#define AIC3008_MICPGA_PWR_DELAY		121	/* L/R Mic Quick Charge Duration Control */
#define AIC3008_REF_PWR_DELAY			122	/* Vref power up delay */

/**********/
/* Page 4 */
/**********/
#define AIC3008_ASI1_BASE				0
#define AIC3008_ASI2_BASE				16
#define AIC3008_ASI3_BASE				32

#define AIC3008_ASI_BUS_FMT				1	/* ASI bus format */
#define AIC3008_ASI_LCH_OFFSET			2	/* ASI L ch offset */
#define AIC3008_ASI_RCH_OFFSET			3	/* ASI L ch offset */
#define AIC3008_ASI_CH_SETUP			4	/* ASI ch setup */
#define AIC3008_ASI_MULTI_CH_IN_CFG		5	/* ASI multi ch config: def all ch on DIN1 */
#define AIC3008_ASI_MULTI_CH_OUT_CFG	6	/* ASI multi ch config: def all ch on DOUT1 */
#define AIC3008_ASI_ADC_IN_CTL			7	/* ASI ADC input ctl, eg in from DSP_A, ASI2, ASI3 (loopback) */
#define AIC3008_ASI_DAC_OUT_CTL			8	/* ASI DAC datapath, eg left/right data */
#define AIC3008_ASI_TRI					9	/* ASI set ch 1 ~ 4 tri */
#define AIC3008_ASI_WCLK_BCLK			10	/* ASI	WCLK dir, BCLK dir, Polarity, */
#define AIC3008_ASI_BCLK_IN				11	/* ASI BCLK N divider input control */
#define AIC3008_ASI_BCLK_NDIV			12	/* ASI BCLK N divider pwr + divide by 1 ~ 128 */
#define AIC3008_ASI_WCLK_NDIV			13	/* ASI WCLK N divider pwr + divide by 32 ~ 128 */
#define AIC3008_ASI_BCLK_WCLK_OUT		14	/* ASI WCLK/BLCK output mux. */
#define AIC3008_ASI_DATA_OUT			15	/* ASI Data Out from eg DOUT1 from ADI1 out, AD1/2/3 in, pin-pin loop back */

#define AIC3008_WCLK1_GPIO				65	/* pin control for WCLK1 */
#define AIC3008_DOUT1_GPIO				67	/* pin control for DOUT1 */
#define AIC3008_DIN1_GPIO				68	/* pin control for DIN1 */
#define AIC3008_WCLK2_GPIO				69	/* pin control for WCLK2 */
#define AIC3008_BCLK2_GPIO				70	/* pin control for BCLK2 */
#define AIC3008_DOUT2_GPIO				71	/* pin control for DOUT2 */
#define AIC3008_DIN2_GPIO				72	/* pin control for DIN2 */
#define AIC3008_WCLK3_GPIO				73	/* pin control for BCLK3 */
#define AIC3008_BCLK3_GPIO				74	/* pin control for WCLK3 */
#define AIC3008_DOUT3_GPIO				75	/* pin control for DOUT3 */
#define AIC3008_DIN3_GPIO				76	/* pin control for DIN3 */
#define AIC3008_MCLK2					82	/* pin control for MCLK2 */
#define AIC3008_GPIO1					86	/* pin control for GPIO1 */
#define AIC3008_GPIO2					87	/* pin control for GPIO2 */
#define AIC3008_GPI1					91	/* pin control for GPI1 */
#define AIC3008_GPI2					92	/* pin control for GPI2 */
#define AIC3008_GPO1					96	/* pin control for GPO1 */
#define AIC3008_DIG_MIC					101	/* Digital Microphone Input Pin Control */
#define AIC3008_DSP_DATA_PORT			118	/* miniDSP data port control */
/*****************************************************************************/

/* Audio serial data interface control register A bits */
#define BIT_CLK_MASTER          0x80
#define WORD_CLK_MASTER         0x40

/* Codec Datapath setup register 7 */
#define FSREF_44100		(1 << 7)
#define FSREF_48000		(0 << 7)
#define DUAL_RATE_MODE		((1 << 5) | (1 << 6))
#define LDAC2LCH		(0x1 << 3)
#define RDAC2RCH		(0x1 << 1)

/* PLL registers bitfields */
#define PLLP_SHIFT		0
#define PLLQ_SHIFT		3
#define PLLR_SHIFT		0
#define PLLJ_SHIFT		2
#define PLLD_MSB_SHIFT		0
#define PLLD_LSB_SHIFT		2

/* Clock generation register bits */
#define CODEC_CLKIN_PLLDIV	0
#define CODEC_CLKIN_CLKDIV	1
#define PLL_CLKIN_SHIFT		4
#define MCLK_SOURCE		0x0
#define PLL_CLKDIV_SHIFT	0

/* Route bits */
#define ROUTE_ON		0x80

/* Mute bits */
#define UNMUTE			0x08
#define MUTE_ON			0x80

/* Power bits */
#define LADC_PWR_ON		0x04
#define RADC_PWR_ON		0x04
#define LDAC_PWR_ON		0x80
#define RDAC_PWR_ON		0x40
#define HPLOUT_PWR_ON		0x01
#define HPROUT_PWR_ON		0x01
#define HPLCOM_PWR_ON		0x01
#define HPRCOM_PWR_ON		0x01
#define MONOLOPM_PWR_ON		0x01
#define LLOPM_PWR_ON		0x01
#define RLOPM_PWR_ON	0x01

#define INVERT_VOL(val)   (0x7f - val)

/* Default output volume (inverted) */
#define DEFAULT_VOL     INVERT_VOL(0x50)
/* Default input volume */
#define DEFAULT_GAIN    0x20

/* GPIO API */
enum {
	AIC3008_GPIO1_FUNC_DISABLED		= 0,
	AIC3008_GPIO1_FUNC_AUDIO_WORDCLK_ADC	= 1,
	AIC3008_GPIO1_FUNC_CLOCK_MUX		= 2,
	AIC3008_GPIO1_FUNC_CLOCK_MUX_DIV2		= 3,
	AIC3008_GPIO1_FUNC_CLOCK_MUX_DIV4		= 4,
	AIC3008_GPIO1_FUNC_CLOCK_MUX_DIV8		= 5,
	AIC3008_GPIO1_FUNC_SHORT_CIRCUIT_IRQ	= 6,
	AIC3008_GPIO1_FUNC_AGC_NOISE_IRQ		= 7,
	AIC3008_GPIO1_FUNC_INPUT			= 8,
	AIC3008_GPIO1_FUNC_OUTPUT			= 9,
	AIC3008_GPIO1_FUNC_DIGITAL_MIC_MODCLK	= 10,
	AIC3008_GPIO1_FUNC_AUDIO_WORDCLK		= 11,
	AIC3008_GPIO1_FUNC_BUTTON_IRQ		= 12,
	AIC3008_GPIO1_FUNC_HEADSET_DETECT_IRQ	= 13,
	AIC3008_GPIO1_FUNC_HEADSET_DETECT_OR_BUTTON_IRQ	= 14,
	AIC3008_GPIO1_FUNC_ALL_IRQ		= 16
};

enum {
	AIC3008_GPIO2_FUNC_DISABLED		= 0,
	AIC3008_GPIO2_FUNC_HEADSET_DETECT_IRQ	= 2,
	AIC3008_GPIO2_FUNC_INPUT			= 3,
	AIC3008_GPIO2_FUNC_OUTPUT			= 4,
	AIC3008_GPIO2_FUNC_DIGITAL_MIC_INPUT	= 5,
	AIC3008_GPIO2_FUNC_AUDIO_BITCLK		= 8,
	AIC3008_GPIO2_FUNC_HEADSET_DETECT_OR_BUTTON_IRQ = 9,
	AIC3008_GPIO2_FUNC_ALL_IRQ		= 10,
	AIC3008_GPIO2_FUNC_SHORT_CIRCUIT_OR_AGC_IRQ = 11,
	AIC3008_GPIO2_FUNC_HEADSET_OR_BUTTON_PRESS_OR_SHORT_CIRCUIT_IRQ = 12,
	AIC3008_GPIO2_FUNC_SHORT_CIRCUIT_IRQ	= 13,
	AIC3008_GPIO2_FUNC_AGC_NOISE_IRQ		= 14,
	AIC3008_GPIO2_FUNC_BUTTON_PRESS_IRQ	= 15
};

struct aic3008_setup_data {
	unsigned int gpio_func[2];
};

/* Number of supplies voltages */
#define AIC3008_NUM_SUPPLIES    4
#define AIC3008_CACHEREGNUM     100

struct aic3008_platform_data {
	bool irq_active_low;   /* Set if IRQ active low, default high */

    /* Default register value for R6 (Mic bias), used to configure
	 * microphone detection.  In conjunction with gpio_cfg this
	 * can be used to route the microphone status signals out onto
	 * the GPIOs for use with snd_soc_jack_add_gpios().
	 */
	u16 micdet_cfg;

	int micdet_delay;      /* Delay after microphone detection (ms) */

	int gpio_base;

	u32 gpio_cfg[AIC3008_CACHEREGNUM]; /* Default register values for GPIO pin mux */
};

/* codec private data */
struct aic3008_priv {
	struct snd_soc_codec *codec;
	struct regulator_bulk_data supplies[AIC3008_NUM_SUPPLIES];
	unsigned int sysclk;
	int master;
	int gpio_reset;

	/* Reference counts */
	int class_w_users;
	int playback_active;
	int capture_active;

	struct completion wseq; /* race condition */

	struct snd_soc_jack *mic_jack;
	int mic_det;
	int mic_short;
	int mic_last_report;
	int mic_delay;

	struct snd_pcm_substream *master_substream;
	struct snd_pcm_substream *slave_substream;
};

static int aic3008_config(CODEC_SPI_CMD *cmds, int size);

#endif /* _AIC3008_H */
