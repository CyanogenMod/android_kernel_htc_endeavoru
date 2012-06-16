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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/initval.h>
#include <sound/tlv.h>

#include <linux/spi/spi.h>
#include <linux/wakelock.h>

#include "tlv320aic3008.h"
#include "tlv320aic3008-reg.h"
#include "tlv320aic3008-tonegen.h"
#include <linux/spi-tegra.h>
#include <../arch/arm/mach-tegra/htc_audio_power.h>
#include <mach/board_htc.h>
#include <linux/pm_qos_params.h>

#undef LOG_TAG
#define LOG_TAG "AUD"

#undef PWR_DEVICE_TAG
#define PWR_DEVICE_TAG LOG_TAG

#undef AUDIO_DEBUG_BEEP
#undef AUDIO_DEBUG
#define AUDIO_DEBUG 0

#define AUD_ERR(fmt, ...) pr_tag_err(LOG_TAG, fmt, ##__VA_ARGS__)
#define AUD_INFO(fmt, ...) pr_tag_info(LOG_TAG, fmt, ##__VA_ARGS__)

#if AUDIO_DEBUG
#define AUDIO_DEBUG_BEEP 1
#define AUD_DBG(fmt, ...) pr_tag_info(LOG_TAG, fmt, ##__VA_ARGS__)
#else
#define AUDIO_DEBUG_BEEP 0
#define AUD_DBG(fmt, ...) do { } while (0)
#endif

#define AUD_CPU_FREQ_MIN 102000

/* for quattro --- */
int64_t pwr_up_time;
int64_t drv_up_time;

static struct mutex lock;

static struct spi_device *codec_spi_dev;
static uint8_t *bulk_tx;
static int aic3008_opened;
static CODEC_SPI_CMD **aic3008_uplink;
static CODEC_SPI_CMD **aic3008_downlink;
static CODEC_SPI_CMD **aic3008_minidsp;
static int aic3008_rx_mode;
static int aic3008_tx_mode;
static int aic3008_dsp_mode;
static bool first_boot_path = false;
static struct pm_qos_request_list aud_cpu_minfreq_req;

extern struct aic3008_power *aic3008_power_ctl;

static struct aic3008_ctl_ops default_ctl_ops;
static struct aic3008_ctl_ops *ctl_ops = &default_ctl_ops;

struct aic3008_clk_state {
	int enabled;
	struct clk *rx_mclk;
	struct clk *rx_sclk;
	struct wake_lock idlelock;
	struct wake_lock wakelock;
};

static struct aic3008_clk_state codec_clk;

static const struct snd_kcontrol_new aic3008_snd_controls[] = { };
static const struct snd_soc_dapm_widget aic3008_dapm_widgets[] = { };
static const struct snd_soc_dapm_route intercon[] = { };

static int aic3008_set_config(int config_tbl, int idx, int en);
static void aic3008_powerdown(void);
static void aic3008_powerup(void);
void aic3008_votecpuminfreq(bool bflag);

/*****************************************************************************/
/* Specific SPI read/write command for AIC3008                               */
/*****************************************************************************/
static int codec_spi_write(unsigned char addr, unsigned char data, bool flag)
{
	unsigned char buffer[2];
	int rc;

	if (!codec_spi_dev)
		return 0;

	if(addr == 0x00 && flag)
	{
		AUD_DBG("------ write page: 0x%02X ------\n", data);
	}
	else if(addr == 0x7F && flag)
	{
		AUD_DBG("------ write book: 0x%02X ------\n", data);
	}
	else if(flag)
	{
		AUD_DBG("write : reg: 0x%02X data: 0x%02X\n", addr, data);
	}

	codec_spi_dev->bits_per_word = 16;
	/* addr and data swapped around because 16bit word */
	buffer[1] = addr << 1;
	buffer[0] = data;
	rc = spi_write(codec_spi_dev, buffer, 2);

	return rc;
}

static int codec_spi_read(unsigned char addr, unsigned char *data, bool flag)
{
	int rc;
	u8 buffer[2] = { 0, 0 };
	u8 result[2] = { 0, 0 };

	codec_spi_dev->bits_per_word = 16;
	buffer[1] = addr << 1 | 1; /* high byte because 16bit word */

	/*AUD_DBG("before read: buf[1]:0x%02X buf[0]:0x%02X res[1]:0x%02X res[0]:0x%02X \n",
			buffer[1], buffer[0], result[1], result[0]);*/

	/* because aic3008 does symmetric SPI write and read */
	rc = spi_write_then_read(codec_spi_dev, buffer, 2, result, 2);
	if (rc < 0)
		return rc;

	if(flag)
	{
		AUD_DBG("read: reg: 0x%02X , data: 0x%02X \n", addr, result[0]);
	}

	*data = result[0]; /* seems address on high byte, data on low byte */
	return 0;
}

static int32_t spi_write_table_parsepage(CODEC_SPI_CMD *cmds, int num)
{
	int i;
	int bulk_counter;
	int status = 0;
	struct spi_message	m;
	struct spi_transfer	tx_addr;
	bool is_page_zero = false;
	unsigned char page_select = 0x00;
	unsigned char book_select = 0x7F;
	unsigned int reg_long1, reg_long2;

	if (codec_spi_dev == NULL) {
		status = -ESHUTDOWN;
		return status;
	}

	i = 0;

	while (i < num - 1) {
		if (cmds[i].reg == book_select && is_page_zero) {
			/* select book */
			codec_spi_write(cmds[i].reg, cmds[i].data, false);
			i++;
		} else if (cmds[i].reg == page_select) {
			/* select page */
			if (cmds[i].data == 0x00) is_page_zero = true;
			else is_page_zero = false;
			codec_spi_write(cmds[i].reg, cmds[i].data, false);
			i++;
		} else {
			spi_message_init(&m);
			memset(bulk_tx, 0, MINIDSP_COL_MAX * 2 * \
				sizeof(uint8_t));
			memset(&tx_addr, 0, sizeof(struct spi_transfer));

			bulk_counter = 0;
			bulk_tx[bulk_counter] = cmds[i].reg << 1;
			bulk_tx[bulk_counter + 1] = cmds[i].data;
			bulk_counter += 2;

			do {
				reg_long1 = (unsigned int)cmds[i].reg;
				reg_long2 = (unsigned int)cmds[i+1].reg;
				if (reg_long2 == (reg_long1+1)) {
					bulk_tx[bulk_counter] = cmds[i+1].data;
					bulk_counter++;
				}
				i++;
			} while (reg_long2 == (reg_long1+1) && i < num-1);

			/*int j = 0;
			AUD_DBG("bulk_write : start reg: 0x%02X\n", bulk_tx[j] >> 1);
			for (j = 1; j < bulk_counter; j++)
				AUD_DBG("bulk_write : data: 0x%02X\n", bulk_tx[j]);
			AUD_DBG("bulk_counter = %d, i = %d\n", bulk_counter, i);*/

			tx_addr.tx_buf = bulk_tx;
			tx_addr.len = (bulk_counter);
			tx_addr.cs_change = 1;
			tx_addr.bits_per_word = 8;
			spi_message_add_tail(&tx_addr, &m);
			status = spi_sync(codec_spi_dev, &m);
		}
	}

	return status;
}

/* write a register then read a register, compare them ! */
static int32_t spi_write_read_list(CODEC_SPI_CMD *cmds, int num)
{
	int i;
	int rc;
	unsigned char write_buffer[2];
	unsigned char read_result[2] = { 0, 0 };

	if (!codec_spi_dev)
		return 0;

	codec_spi_dev->bits_per_word = 16;
	for (i = 0; i < num; i++) {
		/*if writing page, then don't read its value */
		switch (cmds[i].act) {
		case 'w':
			write_buffer[1] = cmds[i].reg << 1;
			write_buffer[0] = cmds[i].data;
			if (cmds[i].reg == 0x00 || cmds[i].reg == 0x7F) {
				rc = spi_write(codec_spi_dev, write_buffer, sizeof(write_buffer));
				if (rc < 0)
					return rc;
				if(cmds[i].reg == 0x00)
				{
					AUD_DBG("------ write page: 0x%02X ------\n", cmds[i].data);
				}
				else if(cmds[i].reg == 0x7F)
				{
					AUD_DBG("------ write book: 0x%02X ------\n", cmds[i].data);
				}
			} else {
				rc = spi_write_then_read(codec_spi_dev, write_buffer, 2, read_result, 2);
				if (rc < 0)
					return rc;

				if (read_result[0] != cmds[i].data)
					AUD_INFO("incorrect value,reg 0x%02x, write 0x%02x, read 0x%02x",
						cmds[i].reg, cmds[i].data, read_result[0]);
			}
			break;
		case 'd':
			msleep(cmds[i].data);
			break;
		default:
			break;
		}
	}
	return 0;
}

/*
 * This function arranges the address and data bytes in a large command list
 * and use kernel SPI API,
 * i.e. use single spi_write() to write spi commands to the audio codec.
 */
static int32_t spi_write_list(CODEC_SPI_CMD *cmds, int num)
{
	int i;
	int rc;
	unsigned char buffer[2];

	if (!codec_spi_dev)
		return 0;

	codec_spi_dev->bits_per_word = 16;
	for (i = 0; i < num; i++) {
		buffer[1] = cmds[i].reg << 1;
		buffer[0] = cmds[i].data;
		rc = spi_write(codec_spi_dev, buffer, sizeof(buffer));
		if (rc < 0)
			return rc;
	}
	return 0;
}

/*
 * This function arranges the address and data bytes in a large command list
 * and use kernel SPI API,
 * i.e. use single spi_write() to write spi commands to the audio codec.
 */
static int32_t spi_read_list(CODEC_SPI_CMD *cmds, int num)
{
	int i;
	int rc;
	unsigned char page = 0;
	unsigned char buffer[2] = { 0, 0 };
	unsigned char result[2] = { 0, 0 };

	if (!codec_spi_dev)
		return 0;

	codec_spi_dev->bits_per_word = 16;
	for (i = 0; i < num; i++) {
		udelay(20);
		/* if writing page, then don't read its value */
		if(i==2)
		{
			AUD_INFO("Skip software reset cmd when dump spi! %02X\n", cmds[i].reg);
			continue;
		}
		if (cmds[i].reg == 0x00) {
			buffer[1] = cmds[i].reg << 1;
			buffer[0] = cmds[i].data;
			rc = spi_write(codec_spi_dev, buffer, sizeof(buffer));
			if (rc < 0)
			{
				return rc;
			}
			AUD_INFO("====== write page %02X ======", buffer[0]);
			page = buffer[0];
		} else if (cmds[i].reg == 0x7F && page == 0x00) {
			buffer[1] = cmds[i].reg << 1;
			buffer[0] = cmds[i].data;
			rc = spi_write(codec_spi_dev, buffer, sizeof(buffer));
			if (rc < 0)
			{
				return rc;
			}
			AUD_INFO("====== write book %02X ======", buffer[0]);
		} else {
			buffer[1] = cmds[i].reg << 1 | 1;
			rc = spi_write_then_read(codec_spi_dev, buffer, 2, result, 2);
			if (rc < 0)
				return rc;
			AUD_INFO("read: reg: 0x%02X , data: 0x%02X\n", cmds[i].reg, result[0]);
		}
	}
	return 0;
}

/*****************************************************************************/
/* Codec Initialisation Sequence                                             */
/*****************************************************************************/
void aic3008_MicSwitch(int on)
{
	/*
	 * NOTE: on AIC3008, the various inputs are as follows
	 * 			IN1LR = 2nd Int MIC
	 * 			IN2LR = 1st Int MIC
	 * 			IN3LR = HP MIC
	 * 			IN4LR = FM LR
	 */
	if (aic3008_power_ctl->mic_switch)
	{
		if (on) aic3008_power_ctl->mic_powerup();
		else aic3008_power_ctl->mic_powerdown();
		mdelay(1);
	}
}

void aic3008_AmpSwitch(int idx, int on)
{
	// headset amplifier
	if (// RX
		idx == CALL_DOWNLINK_EMIC_HEADPHONE ||
		idx == CALL_DOWNLINK_IMIC_HEADPHONE ||
		idx == CALL_DOWNLINK_EMIC_HEADPHONE_DUALMIC ||
		idx == PLAYBACK_HEADPHONE ||
		idx == RING_HEADPHONE_SPEAKER ||
		idx == FM_OUT_SPEAKER ||
		idx == FM_OUT_HEADPHONE ||
		idx == PLAYBACK_HEADPHONE_URBEATS ||
		idx == PLAYBACK_HEADPHONE_SOLO ||
		idx == VOIP_DOWNLINK_EMIC_HEADPHONE ||
		idx == VOIP_DOWNLINK_IMIC_HEADPHONE ||
		idx == CALL_DOWNLINK_EMIC_HEADPHONE_DUALMIC_WB ||
		idx == CALL_DOWNLINK_EMIC_HEADPHONE_BEATS ||
		idx == CALL_DOWNLINK_EMIC_HEADPHONE_BEATS_WB ||
		idx == VOIP_DOWNLINK_EMIC_HEADPHONE_BEATS ||
		idx == PLAYBACK_HEADPHONE_FULLDELPX)
	{
		if (on)
		{
			aic3008_power_ctl->amp_powerup(HEADSET_AMP);
			AUD_DBG("[PWR] headset amplifier power up\n");
		}
		else
		{
			aic3008_power_ctl->amp_powerdown(HEADSET_AMP);
			AUD_DBG("[PWR] headset amplifier power down\n");
		}
	}
	// speaker amplifier
	if (// RX
		idx == CALL_DOWNLINK_IMIC_SPEAKER ||
		idx == CALL_DOWNLINK_IMIC_SPEAKER_DUALMIC ||
		idx == PLAYBACK_SPEAKER ||
		idx == RING_HEADPHONE_SPEAKER ||
		idx == PLAYBACK_SPEAKER_ALT ||
		idx == FM_OUT_SPEAKER ||
		idx == PLAYBACK_SPEAKER_BEATS ||
		idx == VOIP_DOWNLINK_IMIC_SPEAKER ||
		idx == CALL_DOWNLINK_IMIC_SPEAKER_DUALMIC_WB ||
		idx == MFG_PLAYBACK_L_SPEAKER ||
		idx == MFG_PLAYBACK_R_SPEAKER ||
		idx == PLAYBACK_SPK_FULLDELPX)
	{
		if (on)
		{
			aic3008_power_ctl->amp_powerup(SPEAKER_AMP);
			AUD_DBG("[PWR] speaker amplifier power up\n");
		}
		else
		{
			aic3008_power_ctl->amp_powerdown(SPEAKER_AMP);
			AUD_DBG("[PWR] speaker amplifier power down\n");
		}
	}
	// dock amplifier
	if (// RX
		idx == CALL_DOWNLINK_IMIC_DOCK ||
		idx == PLAYBACK_DOCK)
	{
		if (on)
		{
			aic3008_power_ctl->amp_powerup(DOCK_AMP);
			AUD_DBG("[PWR] dock amplifier power up\n");
		}
		else
		{
			aic3008_power_ctl->amp_powerdown(DOCK_AMP);
			AUD_DBG("[PWR] dock amplifier power down\n");
		}
	}
}
bool aic3008_IsSoundPlayBack(int idx)
{
	if (
		idx == PLAYBACK_HEADPHONE ||
		idx == PLAYBACK_HEADPHONE_URBEATS ||
		idx == PLAYBACK_HEADPHONE_SOLO ||
		idx == PLAYBACK_SPEAKER ||
		idx == PLAYBACK_SPEAKER_ALT ||
		idx == PLAYBACK_SPEAKER_BEATS ||
		idx == PLAYBACK_HEADPHONE_FULLDELPX ||
		idx == PLAYBACK_SPK_FULLDELPX ||
		idx == PLAYBACK_DOCK ||
		idx == PLAYBACK_RECEIVER ||
		idx == MFG_PLAYBACK_L_SPEAKER ||
		idx == MFG_PLAYBACK_R_SPEAKER
        )
        return true;
    else
        return false;
}
void aic3008_votecpuminfreq(bool bflag)
{
    static bool boldCPUMinReq = false;
    if (bflag == boldCPUMinReq)
    {
        return;
    }
    boldCPUMinReq = bflag;
    if (bflag)
    {
        pm_qos_update_request(&aud_cpu_minfreq_req, (s32)AUD_CPU_FREQ_MIN);
        AUD_INFO("VoteMinFreqS:%d", AUD_CPU_FREQ_MIN);
    }
    else
    {
        pm_qos_update_request(&aud_cpu_minfreq_req, (s32)PM_QOS_CPU_FREQ_MIN_DEFAULT_VALUE);
        AUD_INFO("VoteMinFreqE:%d", PM_QOS_CPU_FREQ_MIN_DEFAULT_VALUE);
    }

    return;
}

void aic3008_CodecInit()
{
	AUD_DBG("aic3008_CodecInit: start\n");
	pwr_up_time = ktime_to_ms(ktime_get());

	pr_device_power_on();

	// aic3008 power up
	aic3008_power_ctl->powerinit();

	// close mic
	aic3008_MicSwitch(0);

	spi_write_table_parsepage(POWER_UP_SEQ, ARRAY_SIZE(POWER_UP_SEQ));
	AUD_DBG("***** SPI CMD: Power Up Seq *****\n");

	AUD_DBG("Audio Codec Power Up takes %lld ms\n",
			ktime_to_ms(ktime_get()) - pwr_up_time);
#if AUDIO_DEBUG_BEEP
	mdelay(100);
	spi_write_table_parsepage(GENERATE_BEEP_LEFT_SPK, ARRAY_SIZE(GENERATE_BEEP_LEFT_SPK));

	AUD_DBG("***** SPI CMD: BEEP *****\n");
	mdelay(300);
	spi_write_table_parsepage(CODEC_SW_RESET, ARRAY_SIZE(CODEC_SW_RESET));
#endif
	pm_qos_add_request(&aud_cpu_minfreq_req, PM_QOS_CPU_FREQ_MIN, (s32)PM_QOS_CPU_FREQ_MIN_DEFAULT_VALUE);
	aic3008_rx_mode = DOWNLINK_PATH_OFF;
	aic3008_tx_mode = UPLINK_PATH_OFF;
}
EXPORT_SYMBOL_GPL(aic3008_CodecInit);


int aic3008_setMode(int cmd, int idx, int is_call_mode)
{
	int ret;
	AUD_DBG("setMode: cmd:%d, idx:%d, call_mode:%d\n", cmd, idx, is_call_mode);
	if (aic3008_power_ctl->mic_switch)
	{
		// set int/ext mic power off
		aic3008_MicSwitch(0);
		if (cmd == AIC3008_IO_CONFIG_TX && (idx == VOICERECORD_IMIC || idx == VOIP_DOWNLINK_IMIC_SPEAKER)) {
			AUD_DBG("[PWR] aic3008_setMode aic3008_MicSwitch");
			aic3008_MicSwitch(1);
		}
	}

	ret = aic3008_set_config(cmd, idx, 1);
	return ret;
}
EXPORT_SYMBOL_GPL(aic3008_setMode);

void aic3008_set_mic_bias(int on)
{
	if (on) {
		AUD_INFO("[PWR] enalbe_AUD_HPMIC_BIAS\n");
		if (!aic3008_power_ctl->isPowerOn) {
			AUD_INFO("[PWR] codec was power off wakeup first\n");
			aic3008_powerup();
			aic3008_config(ENABLE_AUD_HPMIC_EXT, ARRAY_SIZE(ENABLE_AUD_HPMIC_EXT));
			aic3008_powerdown();
		} else aic3008_config(ENABLE_AUD_HPMIC_EXT, ARRAY_SIZE(ENABLE_AUD_HPMIC_EXT));
	} else {
		AUD_INFO("[PWR] disalbe_AUD_HPMIC_BIAS\n");
		if (aic3008_tx_mode == CALL_UPLINK_IMIC_SPEAKER ||
				aic3008_tx_mode == CALL_UPLINK_IMIC_SPEAKER_DUALMIC ||
				aic3008_tx_mode == VOIP_UPLINK_IMIC_SPEAKER ||
				aic3008_tx_mode == CALL_UPLINK_IMIC_SPEAKER_DUALMIC_WB
				)
		{
			if (!aic3008_power_ctl->isPowerOn) {
				AUD_INFO("[PWR] codec was power off wakeup first\n");
				aic3008_powerup();
				aic3008_config(DISABLE_AUD_HPMIC_EXT_ONLY, ARRAY_SIZE(DISABLE_AUD_HPMIC_EXT_ONLY));
				aic3008_powerdown();
			} else aic3008_config(DISABLE_AUD_HPMIC_EXT_ONLY, ARRAY_SIZE(DISABLE_AUD_HPMIC_EXT_ONLY));
		}
		else
		{
			if (!aic3008_power_ctl->isPowerOn) {
				AUD_INFO("[PWR] codec was power off wakeup first\n");
				aic3008_powerup();
				aic3008_config(DISABLE_AUD_HPMIC_EXT, ARRAY_SIZE(DISABLE_AUD_HPMIC_EXT));
				aic3008_powerdown();
			} else aic3008_config(DISABLE_AUD_HPMIC_EXT, ARRAY_SIZE(DISABLE_AUD_HPMIC_EXT));
		}
	}
}
EXPORT_SYMBOL_GPL(aic3008_set_mic_bias);

/*****************************************************************************/
/* other Audio Codec controls                                                */
/*****************************************************************************/
static void aic3008_sw_reset(struct snd_soc_codec *codec)
{
	AUD_DBG("aic3008 soft reset\n");
	/*  SW RESET on AIC3008. */
	aic3008_config(CODEC_SW_RESET, ARRAY_SIZE(CODEC_SW_RESET));
}

static int aic3008_volatile_register(unsigned int reg)
{
	/* check which registers are volatile on the T30S side */
	return 0;
}

void aic3008_register_ctl_ops(struct aic3008_ctl_ops *ops)
{
	ctl_ops = ops;
}

/* call by SPI probe to create space for the SPI commands */
static CODEC_SPI_CMD **init_2d_array(int row_sz, int col_sz)
{
	CODEC_SPI_CMD *table = NULL;
	CODEC_SPI_CMD **table_ptr = NULL;
	int i = 0;

	table_ptr = kzalloc(row_sz * sizeof(CODEC_SPI_CMD *), GFP_KERNEL);
	table = kzalloc(row_sz * col_sz * sizeof(CODEC_SPI_CMD), GFP_KERNEL);
	if (table_ptr == NULL || table == NULL) {
		AUD_ERR("%s: out of memory\n", __func__);
		kfree(table);
		kfree(table_ptr);
	} else
		for (i = 0; i < row_sz; i++)
			table_ptr[i] = (CODEC_SPI_CMD *) table + i * col_sz;

	return table_ptr;
}

static void spi_aic3008_prevent_sleep(void)
{
	wake_lock(&codec_clk.wakelock);
	wake_lock(&codec_clk.idlelock);
}

static void spi_aic3008_allow_sleep(void)
{
	wake_unlock(&codec_clk.idlelock);
	wake_unlock(&codec_clk.wakelock);
}

/* Access function pointed by ctl_ops to call control operations */
static int aic3008_config(CODEC_SPI_CMD *cmds, int size)
{
	int i, retry, ret;
	unsigned char data;
	if(!aic3008_power_ctl->isPowerOn)
	{
		AUD_INFO("aic3008_config: AIC3008 is power off now");
		return -EINVAL;
	}

	if (!codec_spi_dev) {
		AUD_ERR("no spi device\n");
		return -EFAULT;
	}

	if (cmds == NULL) {
		AUD_ERR("invalid spi parameters\n");
		return -EINVAL;
	}

	/* large dsp image use bulk mode to transfer */
	if (size < 1000) {
		for (i = 0; i < size; i++) {
			switch (cmds[i].act) {
			case 'w':
				codec_spi_write(cmds[i].reg, cmds[i].data, true);
				break;
			case 'r':
				for (retry = AIC3008_MAX_RETRY; retry > 0; retry--) {
					ret = codec_spi_read(cmds[i].reg, &data, true);
					if (ret < 0) {
						AUD_ERR("read fail %d, retry\n", ret);
						hr_msleep(1);
					} else if (data == cmds[i].data) {
						AUD_DBG("data == cmds\n");
						break;
					}
				}
				if (retry <= 0)
					AUD_DBG("3008 power down procedure,"
							" flag 0x%02X=0x%02X(0x%02X)\n",
							cmds[i].reg, ret, cmds[i].data);
				break;
			case 'd':
				msleep(cmds[i].data);
				break;
			default:
				break;
			}
		}
	} else {
		/* use bulk to transfer large data */
		spi_write_table_parsepage(cmds, size);
		AUD_DBG("Here is bulk mode\n");
	}
	return 0;
}

static int aic3008_config_ex(CODEC_SPI_CMD *cmds, int size)
{
	int i = 0;
	int ret = -EINVAL;
	struct spi_transfer *spi_t_cmds = NULL;
	struct spi_message m;
	unsigned char *buffer = NULL;
	unsigned char *ptr = NULL;

	if (!codec_spi_dev) {
		AUD_ERR("no spi device\n");
		return -EFAULT;
	}

	if (cmds == NULL || size == 0) {
		AUD_ERR("invalid spi parameters\n");
		return -EINVAL;
	}

	spi_t_cmds = kmalloc(size * sizeof(struct spi_transfer), GFP_KERNEL);
	if (spi_t_cmds == NULL) {
		AUD_ERR("kmalloc spi transfer struct fail\n");
		goto error;
	} else
		memset(spi_t_cmds, 0, size * sizeof(struct spi_transfer));

	buffer = kmalloc(size * 2 * sizeof(unsigned char),
			GFP_KERNEL);
	if (buffer == NULL) {
		AUD_ERR("kmalloc buffer fail\n");
		goto error;
	} else
		memset(buffer, 0, size * sizeof(CODEC_SPI_CMD) * sizeof(unsigned char));

	spi_message_init(&m);
	for (i = 0, ptr = buffer; i < size; i++, ptr += 2) {
		ptr[0] = cmds[i].reg << 1;
		ptr[1] = cmds[i].data;

		spi_t_cmds[i].tx_buf = ptr;
		spi_t_cmds[i].len = 2;
		spi_message_add_tail(&spi_t_cmds[i], &m);
	}
	codec_spi_dev->bits_per_word = 16;
	ret = spi_sync(codec_spi_dev, &m);

error:
	if (buffer != NULL)
		kfree(buffer);

	if (spi_t_cmds != NULL)
		kfree(spi_t_cmds);
	return ret;
}

int route_rx_enable(int path, int en)
{
	AUD_DBG("[RX] (%d, %d) uses AIC3008 default RX setting...\n", path, en);
	if (en) {
		/* Downlink_Wakeup */
		AUD_INFO("[RX] route_rx_enable call Downlink_Wakeup");
		aic3008_config(CODEC_DOWNLINK_ON, ARRAY_SIZE(CODEC_DOWNLINK_ON));
		/* Path switching */
		switch (path) {
		default:
			/* By pass */
			AUD_INFO("[RX] route_rx_enable call DOWNLINK_IMIC_RECEIVER");
			aic3008_config(DOWNLINK_IMIC_RECEIVER,
					ARRAY_SIZE(DOWNLINK_IMIC_RECEIVER));
			break;
		}
	} else {
		/* Downlink_Off */
		AUD_INFO("[RX] route_rx_enable call CODEC_DOWNLINK_OFF");
		aic3008_config(CODEC_DOWNLINK_OFF, ARRAY_SIZE(CODEC_DOWNLINK_OFF));
	}

	return 0;
}

int route_tx_enable(int path, int en)
{
	AUD_DBG("[TX] (%d, %d) uses aic3008 default TX setting\n", path, en);
	if (en) {
		/* Uplink_Wakeup */
		AUD_INFO("[TX] route_tx_enable call Uplink_Wakeup");
		aic3008_config(CODEC_UPLINK_ON, ARRAY_SIZE(CODEC_UPLINK_ON));
		/* Path switching */
		switch (path) {
		case CALL_UPLINK_IMIC_RECEIVER:
		case CALL_UPLINK_IMIC_HEADPHONE:
		case CALL_UPLINK_IMIC_SPEAKER:
		case VOICERECORD_IMIC:
			/* By pass */
			AUD_INFO("[TX] route_tx_enable call MECHA_UPLINK_IMIC");
			aic3008_config(MECHA_UPLINK_IMIC, ARRAY_SIZE(MECHA_UPLINK_IMIC));
			break;
		case CALL_UPLINK_EMIC_HEADPHONE:
		case VOICERECORD_EMIC:
			AUD_INFO("[TX] route_tx_enable call UPLINK_EMIC,");
			aic3008_config(UPLINK_EMIC, ARRAY_SIZE(UPLINK_EMIC));
			break;
		}
	} else {
		/* Uplink_Off */
		AUD_INFO("[TX] route_tx_enable call CODEC_UPLINK_OFF");
		aic3008_config(CODEC_UPLINK_OFF, ARRAY_SIZE(CODEC_UPLINK_OFF));
	}
	return 0;
}

static void aic3008_tx_config(int mode)
{
	/* mode = 0 for initialisation */

	/* use default setting when tx table doesn't be updated*/
	if (aic3008_uplink == NULL) {
		AUD_DBG("[TX] use default setting since tx table doesn't be updated");
		if (mode == UPLINK_PATH_OFF)
			route_tx_enable(mode, 0); /* uploink off */
		else
			route_tx_enable(mode, 1); /* if no mem for aic3008_uplink + on */
		return;
	}

	/* if not uplink off or power off */
	if (mode != UPLINK_PATH_OFF && mode != POWER_OFF) {
		/* uplink_Wakeup */
		AUD_DBG("[TX] ----- uplink wakeup len(%d) -----\n",
				(aic3008_uplink[UPLINK_WAKEUP][0].data-1));

		aic3008_config(&aic3008_uplink[UPLINK_WAKEUP][1],
				aic3008_uplink[UPLINK_WAKEUP][0].data);
	}

	/* route tx device */
	AUD_INFO("[TX] ----- change i/o uplink TX %d len(%d) -----\n", mode,
			(aic3008_uplink[mode][0].data-1));

	aic3008_config(&aic3008_uplink[mode][1], aic3008_uplink[mode][0].data);
}

static void aic3008_rx_config(int mode)
{
	/* use default setting when rx table doesn't be updated*/
	if (aic3008_downlink == NULL) {
		AUD_DBG("[RX] use default setting since rx table doesn't be updated");
		if (mode == DOWNLINK_PATH_OFF)
			route_rx_enable(mode, 0);
		else
			route_rx_enable(mode, 1);
		return;
	}

	if (mode != DOWNLINK_PATH_OFF) {
		/* Downlink Wakeup */
		AUD_DBG("[RX] ----- downlink wakeup len(%d) -----\n",
				(aic3008_downlink[DOWNLINK_WAKEUP][0].data-1));
		aic3008_config(&aic3008_downlink[DOWNLINK_WAKEUP][1],
				aic3008_downlink[DOWNLINK_WAKEUP][0].data);
	}

	/* route rx device */
	AUD_INFO("[RX] ----- change i/o downlink RX %d len(%d) -----\n", mode,
			(aic3008_downlink[mode][0].data-1));

	aic3008_config(&aic3008_downlink[mode][1], aic3008_downlink[mode][0].data);

    if (aic3008_IsSoundPlayBack(mode))
        aic3008_votecpuminfreq(true);
    else
        aic3008_votecpuminfreq(false);
}

static void aic3008_powerdown(void)
{
	int64_t t1, t2;

	t1 = ktime_to_ms(ktime_get());

	if (aic3008_uplink != NULL) {
		AUD_DBG("[PWR] power off AIC3008 by table len(%d)\n",
				(aic3008_uplink[POWER_OFF][0].data-1));
		aic3008_config(&aic3008_uplink[POWER_OFF][1],
				aic3008_uplink[POWER_OFF][0].data);
	} else {
		AUD_DBG("[PWR] power off AIC3008 by default len(%d)\n",
				(ARRAY_SIZE(CODEC_POWER_OFF)));
		aic3008_config(CODEC_POWER_OFF, ARRAY_SIZE(CODEC_POWER_OFF));
	}

	aic3008_power_ctl->suspend();

	t2 = ktime_to_ms(ktime_get()) - t1;
	AUD_INFO("[PWR] power down AIC3008 %lldms\n", t2);

	return;
}

static void aic3008_powerup(void)
{
	int64_t t1, t2;

	t1 = ktime_to_ms(ktime_get());

	aic3008_power_ctl->resume();

	t2 = ktime_to_ms(ktime_get()) - t1;
	AUD_INFO("[PWR] power on AIC3008 %lldms\n", t2);

	return;
}

static void aic3008_set_loopback(int mode)
{
	aic3008_MicSwitch(0);

	if (!(ctl_ops->lb_dsp_init && ctl_ops->lb_receiver_imic
			&& ctl_ops->lb_speaker_imic && ctl_ops->lb_headset_emic)) {
		AUD_DBG("AIC3008 LOOPBACK not supported\n");
		return;
	}

	/* Init AIC3008 A00 */
	aic3008_config(ctl_ops->lb_dsp_init->data, ctl_ops->lb_dsp_init->len);

	AUD_DBG("set AIC3008 in LOOPBACK mode\n");
	switch (mode) {
	case 0:
		/* receiver v.s. imic */
		aic3008_MicSwitch(1);
		aic3008_config(ctl_ops->lb_receiver_imic->data,
				ctl_ops->lb_receiver_imic->len);
		break;
	case 1:
		/* speaker v.s. imic */
		aic3008_MicSwitch(1);
		aic3008_config(ctl_ops->lb_speaker_imic->data,
				ctl_ops->lb_speaker_imic->len);
		break;
	case 2:
		/* headphone v.s emic */
		aic3008_config(ctl_ops->lb_headset_emic->data,
				ctl_ops->lb_headset_emic->len);
		break;
	case 13:
		/* receiver v.s 2nd mic */
		aic3008_MicSwitch(1);
		if (ctl_ops->lb_receiver_bmic)
			aic3008_config(ctl_ops->lb_receiver_bmic->data,
					ctl_ops->lb_receiver_bmic->len);
		else
			AUD_DBG("receiver v.s. 2nd mic loopback not supported\n");
		break;

	case 14:
		/* speaker v.s 2nd mic */
		aic3008_MicSwitch(1);
		if (ctl_ops->lb_speaker_bmic)
			aic3008_config(ctl_ops->lb_speaker_bmic->data,
					ctl_ops->lb_speaker_bmic->len);
		else
			AUD_DBG("speaker v.s. 2nd mic loopback not supported\n");
		break;

	case 15:
		/* headphone v.s 2nd mic */
		aic3008_MicSwitch(1);
		if (ctl_ops->lb_headset_bmic)
			aic3008_config(ctl_ops->lb_headset_bmic->data,
					ctl_ops->lb_headset_bmic->len);
		else
			AUD_DBG("headset v.s. 2nd mic loopback not supported\n");
		break;
	default:
		break;
	}
}

static int aic3008_set_config(int config_tbl, int idx, int en)
{
	int rc = 0, len = 0;
	int64_t t1, t2;

	mutex_lock(&lock);
/*	spi_aic3008_prevent_sleep(); */

	switch (config_tbl) {
	case AIC3008_IO_CONFIG_TX:
		/* TX */
		if(!aic3008_power_ctl->isPowerOn)
		{
			AUD_ERR("[TX] AIC3008 is power off now, can't do IO CONFIG TX = %d, please check this condition!!", idx);
			AUD_ERR("[TX] Since IO CONFIG TX = %d can't be done, it maybe no sound on device");
			break;
		}
		if (en) {
			AUD_INFO("[TX] AIC3008_IO_CONFIG_TX: UPLINK idx = %d",idx);
			aic3008_tx_config(idx);
			aic3008_tx_mode = idx;
		} else {
			AUD_INFO("[TX] AIC3008_IO_CONFIG_TX: UPLINK_PATH_OFF");
			aic3008_tx_config(UPLINK_PATH_OFF);
			aic3008_tx_mode = UPLINK_PATH_OFF;
		}

		if ((aic3008_tx_mode == UPLINK_PATH_OFF) && (aic3008_rx_mode == DOWNLINK_PATH_OFF))
		{
			AUD_INFO("[TX] AIC3008_IO_CONFIG_TX: PATH OFF Call aic3008_powerdown()");
			aic3008_powerdown();
		}
		break;
	case AIC3008_IO_CONFIG_RX:
		/* RX */
		aic3008_AmpSwitch(aic3008_rx_mode, 0);
		if(!aic3008_power_ctl->isPowerOn)
		{
			AUD_ERR("[RX] AIC3008 is power off now, can't do IO CONFIG RX = %d, please check this condition!!", idx);
			AUD_ERR("[RX] Since IO CONFIG RX = %d can't be done, it maybe no sound on device");
			break;
		}
		if(!first_boot_path && idx == 10)
		{
			AUD_INFO("[RX] AIC3008_IO_CONFIG_RX: first_boot_path = 10\n");
			idx = DOWNLINK_PATH_OFF;
			aic3008_rx_mode = DOWNLINK_PATH_OFF;
			first_boot_path = true;
		}
		if (en) {
			AUD_INFO("[RX] AIC3008_IO_CONFIG_RX: DOWNLINK idx = %d",idx);
			aic3008_rx_config(idx);
			aic3008_rx_mode = idx;
			aic3008_AmpSwitch(idx, 1);
		} else {
			AUD_INFO("[RX] AIC3008_IO_CONFIG_RX: DOWNLINK_PATH_OFF");
			aic3008_rx_config(DOWNLINK_PATH_OFF);
			aic3008_rx_mode = DOWNLINK_PATH_OFF;
		}
		if ((aic3008_tx_mode == UPLINK_PATH_OFF) && (aic3008_rx_mode == DOWNLINK_PATH_OFF))
		{
			AUD_INFO("[RX] AIC3008_IO_CONFIG_RX: PATH OFF Call aic3008_powerdown()");
			aic3008_powerdown();
		}
		break;
	case AIC3008_IO_CONFIG_MEDIA:
		if(idx == 20 && !aic3008_power_ctl->isPowerOn)
		{
			aic3008_powerup();
			break;
		}
		else if(idx == 49)
		{
			AUD_DBG("[DSP] idx = %d, Mic Mute!!", idx);
			if (aic3008_tx_mode == VOIP_UPLINK_BT ||
					aic3008_tx_mode == UPLINK_BT_AP ||
					aic3008_tx_mode == UPLINK_BT_BB ){
				aic3008_config(BT_MIC_MUTE, ARRAY_SIZE(BT_MIC_MUTE));		// mute mic
			}
			else{
				aic3008_config(ADC_MUTE, ARRAY_SIZE(ADC_MUTE));		// mute mic
			}
			break;
		}
		else if(idx == 50)
		{
			AUD_DBG("[DSP] idx = %d, Mic unMute!!", idx);
			if (aic3008_tx_mode == VOIP_UPLINK_BT ||
					aic3008_tx_mode == UPLINK_BT_AP ||
					aic3008_tx_mode == UPLINK_BT_BB ){
				aic3008_config(BT_MIC_UNMUTE, ARRAY_SIZE(BT_MIC_UNMUTE));		// mute mic
			}
			else{
				aic3008_config(ADC_UNMUTE, ARRAY_SIZE(ADC_UNMUTE));		// mute mic
			}
			break;
		}
		else if(idx == 51)
		{
			AUD_DBG("[DSP] idx = %d, Output Mute!!", idx);
			aic3008_config(DAC_MUTE, ARRAY_SIZE(DAC_MUTE));		// mute output
			break;
		}
		else if(idx == 52)
		{
			AUD_DBG("[DSP] idx = %d, Output unMute!!", idx);
			aic3008_config(DAC_UNMUTE, ARRAY_SIZE(DAC_UNMUTE));	// unmute output
			break;
		}
		else if(idx == 53)
		{
			AUD_INFO("[DSP] idx = %d, BEATS_ON!!", idx);
			aic3008_config(BEATS_ON, ARRAY_SIZE(BEATS_ON));	// Increase the gain for BEATS_EFFECT_ON
			break;
		}
		else if(idx == 54)
		{
			AUD_INFO("[DSP] idx = %d, BEATS_OFF!!", idx);
			aic3008_config(BEATS_OFF, ARRAY_SIZE(BEATS_OFF)); // Decrease the gain for BEATS_EFFECT_OFF
			break;
		}
		else if(idx == 55)
		{
			AUD_INFO("[DSP] idx = %d, disable SPK_AMP!!", idx);
			aic3008_AmpSwitch(PLAYBACK_SPEAKER, 0);
			break;
		}
		else if(idx == 56)
		{
			AUD_INFO("[DSP] idx = %d, enable SPK_AMP!!", idx);
			aic3008_AmpSwitch(PLAYBACK_SPEAKER, 1);
			break;
		}
		else if(idx == 57)
		{
			AUD_INFO("[DSP] idx = %d, disable HS_Output!!", idx);
			aic3008_config(HS_MUTE, ARRAY_SIZE(HS_MUTE));
			break;
		}
		else if(idx == 58)
		{//Might have noise when unmute, use this carefully.
			AUD_INFO("[DSP] idx = %d, enable HS_Output!!", idx);
			aic3008_config(HS_UNMUTE, ARRAY_SIZE(HS_UNMUTE));
			break;
		}
		if(!aic3008_power_ctl->isPowerOn)
		{
//			AUD_ERR("[DSP] AIC3008 is power off now, do you want change DSP = %d??", idx);
//			AUD_ERR("[DSP] If DSP %d must to be done, please make sure I/O config won't be forgot!!", idx);
			aic3008_powerup();
			AUD_INFO("[DSP] Recovery this condition, AIC3008 is power up now and DSP %d will be config", idx);
		}
		if (aic3008_minidsp == NULL) {
			AUD_INFO("[DSP] AIC3008_IO_CONFIG_MEDIA: aic3008_minidsp == NULL");
			rc = -EFAULT;
			break;
		}
		/* we use this value to dump dsp. */
		aic3008_dsp_mode = idx;

		len = (aic3008_minidsp[idx][0].reg << 8) | aic3008_minidsp[idx][0].data;

		AUD_INFO("[DSP] AIC3008_IO_CONFIG_MEDIA: Original RX %d, TX %d. start DSP = %d, len = %d ++. ",
				aic3008_rx_mode, aic3008_tx_mode, idx, len);

		t1 = ktime_to_ms(ktime_get());
		/* step 1: path off first */
		if (aic3008_rx_mode != DOWNLINK_PATH_OFF)
			aic3008_rx_config(DOWNLINK_PATH_OFF);

		/* step 2: config DSP */
		aic3008_config(&aic3008_minidsp[idx][1], len);

		t2 = ktime_to_ms(ktime_get()) - t1;

		AUD_INFO("[DSP] AIC3008_IO_CONFIG_MEDIA: configure miniDSP index(%d) time: %lldms --\n", idx, (t2));
		break;
	}

/*	spi_aic3008_allow_sleep(); */
	mutex_unlock(&lock);
	return rc;
}

/* sets the codec driver's set_bias_level call back fn here */
/* This fn is very important!!!. We need to control the audio codec
   according to the different bias levels. i.e. to turn on/off essential
   audio components for different bias level */
static int aic3008_set_bias_level(struct snd_soc_codec *codec,
		enum snd_soc_bias_level level)
{
	/* struct i2c_client *i2c = codec->control_data; */
	/* u16 reg, reg2; */

	switch (level) {
	case SND_SOC_BIAS_ON:
	case SND_SOC_BIAS_PREPARE: /* Preparing to playback */

		/* FIXME: more actions here before playback or recording */
		break;

	case SND_SOC_BIAS_STANDBY: /* when done playback / recording */
		/* bias_level is set to SND_SOC_BIAS_OFF when powering down */
		/* So will run this if-block only once when powering up */
		if (codec->dapm.bias_level == SND_SOC_BIAS_OFF) {
			/* first time powering up the audio codec */
			/*AUD_DBG("First time STANDBY block\n");*/
			/*aic3008_CodecInit();*/
		}
		/* FIXME: should return to HEADPHONE if Headphone is plugged */
		AUD_DBG("common STANDBY block\n");
		break;

	case SND_SOC_BIAS_OFF: /* power down the audio codec */
		/*AUD_DBG("SND_SOC_BIAS_OFF\n");*/
		/* power down the audio codec */
		/*aic3008_powerdown();*/
		break;
	}

	codec->dapm.bias_level = level;
	return 0;
}

/*****************************************************************************/
/* Audio Codec IOCTL callback functions                                      */
/*****************************************************************************/
static int aic3008_open(struct inode *inode, struct file *pfile)
{
	int ret = 0;
	AUD_DBG("IOCTL open\n");
	mutex_lock(&lock);

	if (aic3008_opened) {
		AUD_ERR("busy\n");
		ret = -EBUSY;
	} else
		aic3008_opened = 1;

	mutex_unlock(&lock);
	return ret;
}

static int aic3008_release(struct inode *inode, struct file *pfile)
{
	mutex_lock(&lock);

	aic3008_opened = 0;

	mutex_unlock(&lock);
	AUD_DBG("IOCTL release\n");
	return 0;
}

static long aic3008_ioctl(struct file *file, unsigned int cmd,
		unsigned long argc)
{
	struct AIC3008_PARAM para;
	void *table;
	int ret = 0, i = 0, mem_size, volume = 0;
	CODEC_SPI_CMD reg[4];
	unsigned char data;
	int len= 0; /* for dump dsp length. */
	int pcbid = 0;

	AUD_DBG("IOCTL command:0x%02X, argc:%ld\n", cmd, argc);

	if (aic3008_uplink == NULL || aic3008_downlink == NULL || aic3008_minidsp
			== NULL) {
		AUD_ERR("cmd 0x%x, invalid pointers\n", cmd);
		return -EFAULT;
	}

	switch (cmd) {
	/* first IO command from HAL */
	case AIC3008_IO_SET_TX_PARAM:
	/* second IO command from HAL */
	case AIC3008_IO_SET_RX_PARAM:
		if (copy_from_user(&para, (void *) argc, sizeof(para))) {
			AUD_ERR("failed on copy_from_user\n");
			return -EFAULT;
		}

		AUD_DBG("parameters(%d, %d, %p)\n",
				para.row_num, para.col_num, para.cmd_data);

		if (cmd == AIC3008_IO_SET_TX_PARAM)
			table = aic3008_uplink[0];
		else
			table = aic3008_downlink[0];

		/* confirm indicated size doesn't exceed the allocated one */
		if (para.row_num > IO_CTL_ROW_MAX || para.col_num != IO_CTL_COL_MAX) {
			AUD_ERR("data size mismatch with allocated memory (%d,%d)\n",
					IO_CTL_ROW_MAX, IO_CTL_COL_MAX);
			return -EFAULT;
		}

		mem_size = para.row_num * para.col_num * sizeof(CODEC_SPI_CMD);
		if (copy_from_user(table, para.cmd_data, mem_size)) {
			AUD_ERR("failed on copy_from_user\n");
			return -EFAULT;
		}

		/* invoking initialization procedure of AIC3008 */
		if (cmd == AIC3008_IO_SET_TX_PARAM)
			aic3008_tx_config(INITIAL);

		AUD_INFO("update RX/TX tables(%d, %d) successfully\n",
				para.row_num, para.col_num);
		break;

		/* third io command from HAL */
	case AIC3008_IO_SET_DSP_PARAM:
		if (copy_from_user(&para, (void *) argc, sizeof(para))) {
			AUD_ERR("failed on copy_from_user\n");
			return -EFAULT;
		}

		AUD_DBG("parameters(%d, %d, %p)\n",
				para.row_num, para.col_num, para.cmd_data);

		table = aic3008_minidsp[0];

		/* confirm indicated size doesn't exceed the allocated one */
		if (para.row_num > MINIDSP_ROW_MAX ||
				para.col_num != MINIDSP_COL_MAX) {
			AUD_ERR("data size mismatch with allocated memory (%d, %d)\n",
					MINIDSP_ROW_MAX, MINIDSP_COL_MAX);
			return -EFAULT;
		}

		mem_size = para.row_num * para.col_num * sizeof(CODEC_SPI_CMD);
		if (copy_from_user(table, para.cmd_data, mem_size)) {
			AUD_ERR("failed on copy_from_user\n");
			return -EFAULT;
		}

		AUD_INFO("update dsp table(%d, %d) successfully\n",
				para.row_num, para.col_num);
		break;

		/* these IO commands are called to set path */
	case AIC3008_IO_CONFIG_TX:
	case AIC3008_IO_CONFIG_RX:
	case AIC3008_IO_CONFIG_MEDIA:
		if (copy_from_user(&i, (void *) argc, sizeof(int))) {
			AUD_ERR("failed on copy_from_user\n");
			return -EFAULT;
		}
		/* call aic3008_set_config() to issue SPI commands */
		ret = aic3008_set_config(cmd, i, 1);
		if (ret < 0)
			AUD_ERR("configure(%d) error %d\n", i, ret);
		break;

	case AIC3008_IO_CONFIG_VOLUME_L:
		if (copy_from_user(&volume, (void *) argc, sizeof(int))) {
			AUD_ERR("failed on copy_from_user\n");
			return -EFAULT;
		}

		if (volume < -127 || volume > 48) {
			AUD_ERR("volume out of range\n");
			return -EFAULT;
		}

		AUD_DBG("AIC3008 config left volume %d\n", volume);

		CODEC_SET_VOLUME_L[1].data = volume;

		/* call extended aic3008_config_ex() to set up volume */
		aic3008_config_ex(CODEC_SET_VOLUME_L, ARRAY_SIZE(CODEC_SET_VOLUME_L));
		break;

	case AIC3008_IO_CONFIG_VOLUME_R:
		if (copy_from_user(&volume, (void *) argc, sizeof(int))) {
			AUD_ERR("failed on copy_from_user\n");
			return -EFAULT;
		}

		if (volume < -127 || volume > 48) {
			AUD_ERR("volume out of range\n");
			return -EFAULT;
		}

		AUD_DBG("AIC3008 config right volume %d\n", volume);

		CODEC_SET_VOLUME_R[1].data = volume;

		/* call extended aic3008_config_ex() to set up volume */
		aic3008_config_ex(CODEC_SET_VOLUME_R, ARRAY_SIZE(CODEC_SET_VOLUME_R));
		break;

		/* dump specific audio codec page */
	case AIC3008_IO_DUMP_PAGES:
		if (copy_from_user(&i, (void *) argc, sizeof(int))) {
			AUD_ERR("failed on copy_from_user\n");
			return -EFAULT;
		}
		if (i > AIC3008_MAX_PAGES) {
			AUD_ERR("invalid page number %d\n", i);
			return -EINVAL;
		}

		AUD_DBG("========== dump page %d ==========\n", i);
		/* indicated page number of AIC3008 */
		codec_spi_write(0x00, i, true);
		for (i = 0; i < AIC3008_MAX_REGS; i++) {
			ret = codec_spi_read(i, &data, true);
			if (ret < 0) {
				AUD_ERR("read fail on register 0x%X\n", i);
				0;
			} else {
				AUD_DBG("(addr:0x%02X, data:0x%02X)\n", i, data);
				0;
			}
		}
		AUD_DBG("=============================================\n");
		break;
	case AIC3008_IO_DUMP_DSP:
		AUD_DBG("========== dump dsp %d ==========\n", aic3008_dsp_mode);

		/* indicated dsp number of AIC3008 */
		len = (aic3008_minidsp[aic3008_dsp_mode][0].reg << 8) | aic3008_minidsp[aic3008_dsp_mode][0].data;
		AUD_DBG("len = %d", len);
		spi_read_list(&aic3008_minidsp[aic3008_dsp_mode][1], len);
		AUD_DBG("=============================================\n");
		break;

		/* write specific audio codec register */
	case AIC3008_IO_WRITE_REG:
		AUD_INFO("========== WRITE_REG ==========\n");
		if (copy_from_user(&reg, (void *) argc, sizeof(CODEC_SPI_CMD) * 4)) {
			AUD_ERR("failed on copy_from_user\n");
			return -EFAULT;
		}
		AUD_DBG("command list (%c,%02X,%02X) (%c,%02X,%02X) (%c,%02X,%02X) (%c,%02X,%02X)\n",
				reg[0].act, reg[0].reg, reg[0].data,
				reg[1].act, reg[1].reg, reg[1].data,
				reg[2].act, reg[2].reg, reg[2].data,
				reg[3].act, reg[3].reg, reg[3].data);
//		aic3008_config_ex(reg, 4);
		for (i = 0; i < 4; i++) {
			if (reg[i].act == 'r' || reg[i].act == 'R')
				codec_spi_read(reg[i].reg, &reg[i].data, true);
			else if (reg[i].act == 'w' || reg[i].act == 'W')
				codec_spi_write(reg[i].reg, reg[i].data, true);
			else
				return -EINVAL;
		}
		AUD_INFO("========== WRITE_REG end ==========\n");
		break;

		/* read specific audio codec register */
	case AIC3008_IO_READ_REG:
		AUD_INFO("========== READ_REG ==========\n");
		if (copy_from_user(&reg, (void *) argc, sizeof(CODEC_SPI_CMD) * 4)) {
			AUD_ERR("failed on copy_from_user\n");
			return -EFAULT;
		}
		for (i = 0; i < 4; i++) {
			if (reg[i].act == 'r' || reg[i].act == 'R')
				codec_spi_read(reg[i].reg, &reg[i].data, true);
			else if (reg[i].act == 'w' || reg[i].act == 'W')
				codec_spi_write(reg[i].reg, reg[i].data, true);
			else
				return -EINVAL;
		}
		if (copy_to_user((void *) argc, &reg, sizeof(CODEC_SPI_CMD) * 2)) {
			AUD_ERR("failed on copy_to_user\n");
			return -EFAULT;
		}
		AUD_INFO("========== READ_REG end==========\n");
		break;

	case AIC3008_IO_POWERDOWN: /* power down IO command */
		mutex_lock(&lock);
		aic3008_powerdown();
		mutex_unlock(&lock);
		break;

	case AIC3008_IO_LOOPBACK: /* loopback IO command */
		if (copy_from_user(&i, (void *) argc, sizeof(int))) {
			AUD_ERR("failed on copy_from_user\n");
			return -EFAULT;
		}
		AUD_DBG("index %d for LOOPBACK\n", i);

		/* set up the loopback with specific id */
		aic3008_set_loopback(i);
		break;

	case AIC3008_IO_GET_PCBID: /* get pcbid */
		pcbid = htc_get_pcbid_info();
		if (copy_to_user((void *) argc, &pcbid, sizeof(int))) {
			AUD_ERR("failed on copy_to_user\n");
			return -EFAULT;
		}
		break;

	default:
		AUD_ERR("invalid command %d\n", _IOC_NR(cmd));
		ret = -EINVAL;
	}

	return ret;
}

static const struct file_operations aic3008_fops = {
		.owner = THIS_MODULE,
		.open = aic3008_open,
		.release = aic3008_release,
		.unlocked_ioctl = aic3008_ioctl,
};

static struct miscdevice aic3008_misc = {
		.minor = MISC_DYNAMIC_MINOR,
		.name = "codec_aic3008",
		.fops = &aic3008_fops,
};

/*****************************************************************************/
/* Audio Codec specific DAI callback functions                               */
/*****************************************************************************/
static int aic3008_set_dai_sysclk(struct snd_soc_dai *codec_dai, int clk_id,
		unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct aic3008_priv *aic3008 = snd_soc_codec_get_drvdata(codec);

	aic3008->sysclk = freq;
	/* TODO: should allow setting of MCLK here. */

	return 0;
}

static int aic3008_set_dai_fmt(struct snd_soc_dai *codec_dai, unsigned int fmt)
{
	/* 	struct snd_soc_codec *codec = codec_dai->codec; */
	/* 	u16 aif1; */

	/*  TODO: add DAI format control here */

	return 0;
}

static int aic3008_dai_digital_mute(struct snd_soc_dai *codec_dai, int mute)
{
	/* struct snd_soc_codec *codec = codec_dai->codec; */

	/*  TODO: add dai digital_mute controls here */

	return 0;
}

static int aic3008_dai_startup(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
	struct aic3008_priv *aic3008 = snd_soc_codec_get_drvdata(codec);
	struct spi_device *spi = codec->control_data;
	struct snd_pcm_runtime *master_runtime;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		aic3008->playback_active++;
	else
		aic3008->capture_active++;

	/* The DAI has shared clocks so if we already have a playback or
	 * capture going then constrain this substream to match it.
	 */
	if (aic3008->master_substream) {
		master_runtime = aic3008->master_substream->runtime;

		/* dev_dbg(&spi->dev, "Constraining to %d bits\n",
		 master_runtime->sample_bits); */
		AUD_DBG("%p Constraining to %d bits\n", &spi->dev,
				master_runtime->sample_bits);

		snd_pcm_hw_constraint_minmax(substream->runtime,
				SNDRV_PCM_HW_PARAM_SAMPLE_BITS, master_runtime->sample_bits,
				master_runtime->sample_bits);

		aic3008->slave_substream = substream;
	} else
		aic3008->master_substream = substream;

	return 0;
}

static void aic3008_dai_shutdown(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
	struct aic3008_priv *aic3008 = snd_soc_codec_get_drvdata(codec);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		aic3008->playback_active--;
	else
		aic3008->capture_active--;

	if (aic3008->master_substream == substream)
		aic3008->master_substream = aic3008->slave_substream;

	aic3008->slave_substream = NULL;
}

static int aic3008_dai_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	/*
	 struct snd_soc_pcm_runtime *rtd = substream->private_data;
	 struct snd_soc_device *socdev = rtd->socdev;
	 struct snd_soc_codec *codec = socdev->card->codec;
	 struct aic3008_priv *aic3008 = snd_soc_codec_get_drvdata(codec);
	 struct spi_device *spi = codec->control_data;
	 int fs = params_rate(params);
	 int bclk;
	 int bclk_div;
	 int i;
	 int dsp_config;
	 int clk_config;
	 int best_val;
	 int cur_val;
	 int clk_sys;
	 */

	/* TODO: */
	/* set codec hw_params through SPI commands. e.g. sampling and bclk */
	return 0;
}

#define AIC3008_PLAYBACK_RATES (SNDRV_PCM_RATE_8000 |\
			       SNDRV_PCM_RATE_11025 |	\
			       SNDRV_PCM_RATE_16000 |	\
			       SNDRV_PCM_RATE_22050 |	\
			       SNDRV_PCM_RATE_32000 |	\
			       SNDRV_PCM_RATE_44100 |	\
			       SNDRV_PCM_RATE_48000 |	\
			       SNDRV_PCM_RATE_88200 |	\
			       SNDRV_PCM_RATE_96000)

#define AIC3008_CAPTURE_RATES (SNDRV_PCM_RATE_8000 |\
			      SNDRV_PCM_RATE_11025 |	\
			      SNDRV_PCM_RATE_16000 |	\
			      SNDRV_PCM_RATE_22050 |	\
			      SNDRV_PCM_RATE_32000 |	\
			      SNDRV_PCM_RATE_44100 |	\
			      SNDRV_PCM_RATE_48000)

#define AIC3008_FORMATS (SNDRV_PCM_FMTBIT_S16_LE |\
			SNDRV_PCM_FMTBIT_S20_3LE |\
			SNDRV_PCM_FMTBIT_S24_LE)

static struct snd_soc_dai_ops aic3008_dai_ops = {
		.startup = aic3008_dai_startup,
		.shutdown = aic3008_dai_shutdown,
		.hw_params = aic3008_dai_hw_params,
		.digital_mute = aic3008_dai_digital_mute,
		.set_fmt = aic3008_set_dai_fmt,
		.set_sysclk = aic3008_set_dai_sysclk,
};

struct snd_soc_dai_driver aic3008_dai = {
		.name = "aic3008-hifi",
		.playback = {
				.stream_name = "Playback",
				.channels_min = 2,
				.channels_max = 2,
				.rates = AIC3008_PLAYBACK_RATES,
				.formats = AIC3008_FORMATS,
		},
		.capture = {
				.stream_name = "Capture",
				.channels_min = 2,
				.channels_max = 2,
				.rates = AIC3008_CAPTURE_RATES,
				.formats = AIC3008_FORMATS,
		},
		.ops = &aic3008_dai_ops,
		.symmetric_rates = 1,
};

/*****************************************************************************/
/* start of audio codec specific functions.                                  */
/*****************************************************************************/
static int __devinit aic3008_probe(struct snd_soc_codec *codec)
{
	AUD_INFO("aic3008_probe() start... aic3008_codec:%p", codec);
	int ret = 0;

	struct aic3008_priv *aic3008 = snd_soc_codec_get_drvdata(codec);
	aic3008->codec = codec;
	aic3008->codec->control_data = (void *)codec_spi_dev;

	if (!aic3008) {
		AUD_ERR("%s: Codec not registered, SPI device not yet probed\n",
				&aic3008->codec->name);
		return -ENODEV;
	}
	aic3008_sw_reset(codec); // local call to reset codec
	aic3008_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

	// request space for SPI commands data of AIC3008
	aic3008_uplink = init_2d_array(IO_CTL_ROW_MAX, IO_CTL_COL_MAX);
	aic3008_downlink = init_2d_array(IO_CTL_ROW_MAX, IO_CTL_COL_MAX);
	aic3008_minidsp = init_2d_array(MINIDSP_ROW_MAX, MINIDSP_COL_MAX);
	bulk_tx = kcalloc(MINIDSP_COL_MAX * 2 , sizeof(uint8_t), GFP_KERNEL);

	AUD_INFO("Audio Codec Driver init complete in %lld ms\n",
			 ktime_to_ms(ktime_get()) - drv_up_time);
	return ret;
}

static int __devexit aic3008_remove(struct snd_soc_codec *codec)
{
	aic3008_set_bias_level(codec, SND_SOC_BIAS_OFF);
	return 0;
}

static int aic3008_suspend(struct snd_soc_codec *codec, pm_message_t state)
{
	//AUD_DBG("call set_bias_level(SND_SOC_BIAS_OFF)\n");
	//aic3008_set_bias_level(codec, SND_SOC_BIAS_OFF);
	AUD_DBG("aic3008_suspend\n");
	return 0;
}

static int aic3008_resume(struct snd_soc_codec *codec)
{
	//AUD_DBG("call set_bias_level(SND_SOC_BIAS_STANDBY)\n");
	//aic3008_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
	AUD_DBG("aic3008_resume\n");
	return 0;
}

static struct snd_soc_codec_driver soc_codec_dev_aic3008 = {
	.probe =	aic3008_probe,
	.remove =	aic3008_remove,
	.suspend =	aic3008_suspend,
	.resume =	aic3008_resume,
	.set_bias_level = aic3008_set_bias_level,
	.volatile_register = aic3008_volatile_register,
};

/*****************************************************************************/
/* start of audio codec SPI specific functions.                              */
/*****************************************************************************/
static int spi_aic3008_probe(struct spi_device *spi_aic3008)
{
	AUD_DBG("spi device: %s, addr = 0x%p. YAY! ***** Start to Test *****\n",
		spi_aic3008->modalias, spi_aic3008);
	int ret = 0;
	codec_spi_dev = spi_aic3008; /* assign global pointer to SPI device. */

	struct aic3008_priv *aic3008 = kzalloc(sizeof(struct aic3008_priv), GFP_KERNEL);;
	if (aic3008 == NULL)
		return -ENOMEM;
	
	spi_set_drvdata(spi_aic3008, aic3008);

	ret = snd_soc_register_codec(&spi_aic3008->dev,
			&soc_codec_dev_aic3008, &aic3008_dai, 1);
	if (ret < 0)
	{
		AUD_ERR("snd_soc_register_codec() Failed\n");
		kfree(aic3008);
	}
	AUD_INFO("SPI control for AIC3008 started successfully...\n");

	return ret;
}

static int spi_aic3008_suspend(struct spi_device *spi_aic3008,
		pm_message_t pmsg)
{
	AUD_DBG("spi aic3008 suspend (msg:%d)\n", pmsg.event);
	return 0;
}

static int spi_aic3008_resume(struct spi_device *spi_aic3008)
{
	AUD_DBG("spi aic3008 resume");
	return 0;
}

static int spi_aic3008_remove(struct spi_device *spi_aic3008)
{
	AUD_DBG("Remove the SPI driver for aic3008.\n");
	snd_soc_unregister_codec(&spi_aic3008->dev);
	kfree(spi_get_drvdata(spi_aic3008));
	return 0;
}

static struct spi_driver aic3008_spi_driver = {
	.driver = {
		.name = "aic3008",
		.owner = THIS_MODULE,
	},
	.probe = spi_aic3008_probe,
	.suspend = spi_aic3008_suspend,
	.resume = spi_aic3008_resume,
  	.remove = spi_aic3008_remove,
};

static inline int spi_aic3008_init(void)
{
	int ret = 0;

	AUD_DBG("starting aic3008 spi driver init...\n");
	mutex_init(&lock);

	ret = spi_register_driver(&aic3008_spi_driver);
	AUD_DBG("DONE spi_register_driver(&aic3008_spi_driver).\n");
	if (ret < 0) {
		AUD_ERR("failed to register spi driver(%d)\n", ret);
		return ret;
	}
	ret = misc_register(&aic3008_misc);
	AUD_DBG("DONE misc_register(&aic3008_misc).\n");
	if (ret < 0) {
		AUD_ERR("failed to register misc device(%d)\n", ret);
		spi_unregister_driver(&aic3008_spi_driver);
		return ret;
	}

	return 0;
}

static inline void spi_aic3008_exit(void)
{
	AUD_DBG("exiting aic3008 spi driver...\n");

	spi_unregister_driver(&aic3008_spi_driver);
	misc_deregister(&aic3008_misc);
}

static int __init aic3008_modinit(void)
{
	AUD_DBG("Start aic3008_modinit\n");
	drv_up_time = ktime_to_ms(ktime_get());
	spi_aic3008_init();
	return 0;
}
module_init(aic3008_modinit);

static void __exit aic3008_exit(void)
{
	AUD_DBG("Exit aic3008...\n");
	spi_aic3008_exit();
	return;
}
module_exit(aic3008_exit);

MODULE_DESCRIPTION("ASoC TLV320AIC3008 codec driver");
MODULE_AUTHOR("HTC Coporation.");
MODULE_LICENSE("GPL");
