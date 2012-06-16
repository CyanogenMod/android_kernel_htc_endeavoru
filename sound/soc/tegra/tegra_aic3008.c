/*
 * tegra_wm8903.c - Tegra machine ASoC driver for boards using WM8903 codec.
 *
 * Author: Stephen Warren <swarren@nvidia.com>
 * Copyright (C) 2010-2011 - NVIDIA, Inc.
 *
 * Based on code copyright/by:
 *
 * (c) 2009, 2010 Nvidia Graphics Pvt. Ltd.
 *
 * Copyright 2007 Wolfson Microelectronics PLC.
 * Author: Graeme Gregory
 *         graeme.gregory@wolfsonmicro.com or linux@wolfsonmicro.com
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#include <asm/mach-types.h>

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <sound/core.h>
#include <sound/jack.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include "tegra_pcm.h"
#include "tegra_asoc_utils.h"
#include "../codecs/tlv320aic3008.h"
#if (defined(CONFIG_TEGRA_ES305))
#include "../../../include/sound/es305.h"
#endif

#undef LOG_TAG
#define LOG_TAG "AUD"
#undef PWR_DEVICE_TAG
#define PWR_DEVICE_TAG LOG_TAG

#undef AUDIO_DEBUG
#define AUDIO_DEBUG 0

#define AUD_ERR(fmt, ...) pr_tag_err(LOG_TAG, fmt, ##__VA_ARGS__);
#define AUD_INFO(fmt, ...) pr_tag_info(LOG_TAG, fmt, ##__VA_ARGS__);

#if AUDIO_DEBUG
#define AUD_DBG(fmt, ...) pr_tag_info(LOG_TAG, fmt, ##__VA_ARGS__);
#else
#define AUD_DBG(fmt, ...) do { } while (0)
#endif


#define DRV_NAME "tegra-snd-aic3008"
#define TEGRA_AUDIO_DEVICE_NONE	0x00000000
#define TEGRA_AUDIO_DEVICE_MAX	0x7FFFFFFF

struct tegra_asoc_utils_data  *util_data;

struct tegra_aic3008 {
	struct tegra_asoc_utils_data util_data;
};

/*******************************************************************/
/* DAI options                                                     */
/*******************************************************************/
static int tegra_hifi_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params)
{
	AUD_DBG("Start tegra_hifi_hw_params()\n");
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_card *card = codec->card;
	struct tegra_aic3008 *machine = snd_soc_card_get_drvdata(card);
	int dai_flag = 0, mclk, srate;
	int err;

	AUD_DBG("set I2S Master\n");

	dai_flag |= SND_SOC_DAIFMT_I2S; 	// i2s mode
	dai_flag |= SND_SOC_DAIFMT_CBM_CFM; // bclk and frame master

	srate = params_rate(params);
	switch (srate) {
	case 64000:
	case 88200:
	case 96000:
		mclk = 128 * srate;
		break;
	default:
		mclk = 256 * srate;
		break;
	}
	/* FIXME: Codec only requires >= 3MHz if OSR==0 */
	while (mclk < 6000000)
		mclk *= 2;

	mclk = 12288000;
	err = tegra_asoc_utils_set_rate(&machine->util_data, srate, mclk);
	if (err < 0) {
		dev_err(card->dev, "Can't configure clocks\n");
		return err;
	}
	
	// eventually calls audio codec to set dai format, which sets slave
	err = snd_soc_dai_set_fmt(codec_dai, dai_flag);
	if (err < 0) {
		AUD_ERR("codec_dai fmt not set \n");
		return err;
	}
	AUD_DBG("*** snd_soc_dai_set_fmt(codec_dai, dai_flag) ok ***\n");

	// eventually calls t2s driver to set dai format, which sets master
	err = snd_soc_dai_set_fmt(cpu_dai, dai_flag);
	if (err < 0) {
		AUD_ERR("cpu_dai fmt not set \n");
		return err;
	}
	AUD_DBG("*** snd_soc_dai_set_fmt(cpu_dai, dai_flag) ok ***\n");

	// FIXME: not sure this is the right way.
	// Sets the audio codec clks.
	// This should be sample rate times 256 or 128 based on codec need
	err = snd_soc_dai_set_sysclk(codec_dai, 0, mclk, SND_SOC_CLOCK_IN);
	if (err < 0) {
		AUD_ERR("codec_dai clock not set\n");
		return err;
	}
	AUD_DBG("*** snd_soc_dai_set_sysclk(codec_dai, 0, mclk, SND_SOC_CLOCK_IN) ok ***\n");

	return 0;
}

static int tegra_spdif_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params)
{
	AUD_DBG("SPDIF not implemented.\n");
	return 0;
}

int tegra_codec_startup(struct snd_pcm_substream *substream)
{
	AUD_DBG("audio codec startup.\n");
	return 0;
}

void tegra_codec_shutdown(struct snd_pcm_substream *substream)
{
	AUD_DBG("audio codec shutdown\n");
}

static struct snd_soc_ops tegra_hifi_ops = { .hw_params = tegra_hifi_hw_params,
		.startup = tegra_codec_startup, .shutdown = tegra_codec_shutdown, };

static struct snd_soc_ops tegra_spdif_ops = { .hw_params =
		tegra_spdif_hw_params, };

/*******************************************************************/
/* ALSA controls                                                   */
/*******************************************************************/
int tegra_i2sloopback_func = 0;	//OFF

static void tegra_audio_route(struct snd_soc_codec *audio_data,
		int cmd, int idx)
{
	switch (cmd) {
	case AIC3008_IO_CONFIG_RX:
		AUD_DBG("tegra_audio_route, AIC3008_IO_CONFIG_RX, idx = %d, call mode %d \n",
				idx, audio_data->is_call_mode);
		aic3008_setMode(cmd, idx, audio_data->is_call_mode);
		break;

	case AIC3008_IO_CONFIG_TX:
		AUD_DBG("tegra_audio_route, AIC3008_IO_CONFIG_TX, idx = %d, call mode %d \n",
				idx, audio_data->is_call_mode);
		aic3008_setMode(cmd, idx, audio_data->is_call_mode);
		break;

	case AIC3008_IO_CONFIG_MEDIA:
		AUD_DBG("tegra_audio_route, AIC3008_IO_CONFIG_MEDIA, idx = %d, call mode %d \n",
				idx, audio_data->is_call_mode);
		aic3008_setMode(cmd, idx, audio_data->is_call_mode);
		break;
#if (defined(CONFIG_TEGRA_ES305))
	case ES305_SET_CONFIG:
		AUD_DBG("tegra_audio_route, ES305_SET_CONFIG, idx = %d, call mode %d \n",
				idx, audio_data->is_call_mode);
		es305_set_config(idx, ES305_CONFIG_FULL);
		break;
#endif
	}

}

// play reoute control
static int tegra_play_route_info(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = TEGRA_AUDIO_DEVICE_NONE;
	uinfo->value.integer.max = TEGRA_AUDIO_DEVICE_MAX;
	return 0;
}

static int tegra_play_route_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *audio_data = snd_kcontrol_chip(kcontrol);

	ucontrol->value.integer.value[0] = DOWNLINK_PATH_OFF;
	if (audio_data) {
		ucontrol->value.integer.value[0] = audio_data->downlink_id;
		return 0;
	}
	return -EINVAL;
}

static int tegra_play_route_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *audio_data = snd_kcontrol_chip(kcontrol);
	int downlink;

	AUD_DBG("tegra_play_route_put\n");

	if (audio_data) {
		downlink = ucontrol->value.integer.value[0];

		if (audio_data->downlink_id != downlink)
			audio_data->downlink_id = downlink;

		tegra_audio_route(audio_data, AIC3008_IO_CONFIG_RX, downlink);
		return 0;
	}
	return -EINVAL;
}

// capture route control
static int tegra_capture_route_info(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = TEGRA_AUDIO_DEVICE_NONE;
	uinfo->value.integer.max = TEGRA_AUDIO_DEVICE_MAX;
	return 0;
}

static int tegra_capture_route_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *audio_data = snd_kcontrol_chip(kcontrol);

	ucontrol->value.integer.value[0] = UPLINK_PATH_OFF;
	if (audio_data) {
		ucontrol->value.integer.value[0] = audio_data->uplink_id;
		return 0;
	}
	return -EINVAL;
}

static int tegra_capture_route_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *audio_data = snd_kcontrol_chip(kcontrol);
	int uplink;

	AUD_DBG("tegra_capture_route_put\n");

	if (audio_data) {
		uplink = ucontrol->value.integer.value[0];

		if (audio_data->uplink_id != uplink)
			audio_data->uplink_id = uplink;

		tegra_audio_route(audio_data, AIC3008_IO_CONFIG_TX, uplink);
		return 0;
	}
	return -EINVAL;
}

// call mode control
static int tegra_call_mode_info(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 1;
	return 0;
}

static int tegra_call_mode_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *audio_data = snd_kcontrol_chip(kcontrol);

	ucontrol->value.integer.value[0] = false;
	if (audio_data) {
		ucontrol->value.integer.value[0] = audio_data->is_call_mode;
		return 0;
	}
	return -EINVAL;
}

static int tegra_call_mode_put(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *audio_data = snd_kcontrol_chip(kcontrol);
	int is_call_mode_new;

	if (audio_data) {
		is_call_mode_new = ucontrol->value.integer.value[0];

		if (is_call_mode_new == true)
		{
			AUD_DBG("in_call_mode.\n");
		}
		else
		{
#if (defined(CONFIG_TEGRA_ES305))
			if (audio_data->is_call_mode != is_call_mode_new) {
				AUD_DBG("not_in_call_mode. put es305 into sleep mode\n");
				es305_sleep();
				audio_data->es305_cfg_id = -1;
			}
#endif
		}

		audio_data->is_call_mode = is_call_mode_new;
		return 0;
	}
	return -EINVAL;
}

// dsp control
static int tegra_config_aic3008_dsp_info(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 52;
	return 0;
}

static int tegra_config_aic3008_dsp_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *audio_data = snd_kcontrol_chip(kcontrol);

	ucontrol->value.integer.value[0] = MFG;
	if (audio_data) {
		ucontrol->value.integer.value[0] = audio_data->aic3008_dsp_id;
		return 0;
	}
	return -EINVAL;
}

static int tegra_config_aic3008_dsp_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{

	struct snd_soc_codec *audio_data = snd_kcontrol_chip(kcontrol);
	int aic3008_dsp_id;

	if (audio_data) {
		aic3008_dsp_id = ucontrol->value.integer.value[0];

		if (audio_data->aic3008_dsp_id != aic3008_dsp_id)
			audio_data->aic3008_dsp_id = aic3008_dsp_id;

		tegra_audio_route(audio_data, AIC3008_IO_CONFIG_MEDIA,
				aic3008_dsp_id);
		return 0;
	}
	return -EINVAL;

}

// es305 control
#if (defined(CONFIG_TEGRA_ES305))
static int tegra_config_es305_info(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 200;
	return 0;
}

static int tegra_config_es305_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *audio_data = snd_kcontrol_chip(kcontrol);

	ucontrol->value.integer.value[0] = ES305_PATH_SUSPEND; /* -1 */
	if (audio_data) {
		ucontrol->value.integer.value[0] = audio_data->es305_cfg_id;
		return 0;
	}
	return -EINVAL;
}

static int tegra_config_es305_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *audio_data = snd_kcontrol_chip(kcontrol);
	int new_es305_cfg_id;
	if (audio_data) {
		new_es305_cfg_id = ucontrol->value.integer.value[0];
		if (audio_data->es305_cfg_id != new_es305_cfg_id) {
			AUD_DBG("tegra_config_es305_put, %d", new_es305_cfg_id);
			tegra_audio_route(audio_data, ES305_SET_CONFIG, new_es305_cfg_id);
			audio_data->es305_cfg_id = new_es305_cfg_id;
		}
		return 0;
	}
	return -EINVAL;
}
#endif

// loopback control
static int tegra_i2s_loopback_info(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 1;
	return 0;
}

static int tegra_i2s_loopback_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = tegra_i2sloopback_func;
	return 0;
}

static int tegra_i2s_loopback_put(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	if (tegra_i2sloopback_func == ucontrol->value.integer.value[0])
	return 0;

	tegra_i2sloopback_func = ucontrol->value.integer.value[0];
	return 1;
}

static const struct snd_kcontrol_new tegra_controls[] = {
	{
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "Pcm Playback Route",
		.private_value = 0xffff,
		.info = tegra_play_route_info,
		.get = tegra_play_route_get,
		.put = tegra_play_route_put
	},
	{
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "Pcm Capture Route",
		.private_value = 0xffff,
		.info = tegra_capture_route_info,
		.get = tegra_capture_route_get,
		.put = tegra_capture_route_put
	},
	{
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "Call Mode Switch",
		.private_value = 0xffff,
		.info = tegra_call_mode_info,
		.get = tegra_call_mode_get,
		.put = tegra_call_mode_put
	},
	{
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "Config AIC3008 DSP",
		.private_value = 0xffff,
		.info = tegra_config_aic3008_dsp_info,
		.get = tegra_config_aic3008_dsp_get,
		.put = tegra_config_aic3008_dsp_put
	},
	{
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "I2S loopback",
		.private_value = 0xffff,
		.info = tegra_i2s_loopback_info,
		.get = tegra_i2s_loopback_get,
		.put = tegra_i2s_loopback_put,
	},
#if (defined(CONFIG_TEGRA_ES305))
	{
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "Config ES305",
		.private_value = 0xffff,
		.info = tegra_config_es305_info,
		.get = tegra_config_es305_get,
		.put = tegra_config_es305_put
	},
#endif
};

static int tegra_aic3008_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_card *card = codec->card;

	int err = 0;
	AUD_DBG("tegra_codec_init().\n");

	err = tegra_asoc_utils_clk_enable(util_data);
	if (err) {
		AUD_ERR("Failed to enable dap mclk \n");
		err = -ENODEV;
		goto aic3008_init_fail;
	}
	AUD_DBG("Get tegra mclk handle util_data %p.\n", util_data);
	
	/* calls tegra_controls_init() in tegra_soc_controls.c
	* to set up playback, capture, mode, i2s loop back
	* routes controls. tegra_soc_controls handles the ALSA
	* IOCTL calls issued from ALSA HAL
	*/
	// Add controls
	err = snd_soc_add_controls(codec, tegra_controls,
			ARRAY_SIZE(tegra_controls));
	if (err < 0)
		return err;

	aic3008_CodecInit();
	AUD_DBG("DONE aic3008_CodecInit().\n");

	return err;
	
aic3008_init_fail:
	
	tegra_asoc_utils_clk_disable(util_data);
	return err;
}

/*******************************************************************/
/* Codec suspend/resume controls                                   */
/*******************************************************************/
int tegra_suspend_pre(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

int tegra_suspend_post(struct platform_device *pdev, pm_message_t state)
{
	AUD_DBG("tegra_soc_suspend_post - disable mclk through DAS\n");
	tegra_asoc_utils_clk_disable(util_data);
	pr_device_clk_off();
	return 0;
}

int tegra_resume_pre(struct platform_device *pdev)
{
	AUD_DBG("tagra_soc_resume_pre - enable mclk through DAS\n");
	tegra_asoc_utils_clk_enable(util_data);
	pr_device_clk_on();
	return 0;
}

int tegra_resume_post(struct platform_device *pdev)
{
	return 0;
}

#define TEGRA_CREATE_SOC_DAI_LINK(xname, xstreamname, xcodecname, xplatformname, \
								 xcpudainame, xcodecdainame, xinit, xops) \
{ \
		.name = xname, \
		.stream_name = xstreamname, \
		.codec_name = xcodecname, \
		.platform_name = xplatformname, \
		.cpu_dai_name = xcpudainame, \
		.codec_dai_name = xcodecdainame, \
		.init = xinit, \
		.ops = xops, \
}

static struct snd_soc_dai_link tegra_soc_dai[] = {
	TEGRA_CREATE_SOC_DAI_LINK("AIC3008", "AIC3008 HiFi", "spi1.0", "tegra-pcm-audio",
				"tegra30-i2s.1", "aic3008-hifi", tegra_aic3008_init, &tegra_hifi_ops),
	TEGRA_CREATE_SOC_DAI_LINK("SPDIF", "SPDIF PCM", "spdif-dit.0", "tegra-pcm-audio",
				"tegra30-spdif", "dit-hifi", NULL, &tegra_spdif_ops),
};
		
static struct snd_soc_card snd_soc_tegra_aic3008 = {
	.name = "tegra-aic3008",
	.dai_link = tegra_soc_dai,
	.num_links = ARRAY_SIZE(tegra_soc_dai),
	.suspend_pre = tegra_suspend_pre,
	.suspend_post = tegra_suspend_post,
	.resume_pre = tegra_resume_pre,
	.resume_post = tegra_resume_post,
};

static __devinit int tegra_aic3008_driver_probe(struct platform_device *pdev)
{
	AUD_INFO("starting tegra_aic3008_driver_probe...\n");
	struct snd_soc_card *card = &snd_soc_tegra_aic3008;
	struct tegra_aic3008 *machine;
	int ret;

	machine = kzalloc(sizeof(struct tegra_aic3008), GFP_KERNEL);
	if (!machine) {
		AUD_ERR("Can't allocate tegra_aic3008 struct\n");
		return -ENOMEM;
	}

	ret = tegra_asoc_utils_init(&machine->util_data, &pdev->dev);
	util_data = &machine->util_data;
	if (ret)
		goto err_free_machine;
	AUD_DBG("DONE tegra_asoc_utils_init()\n");

	card->dev = &pdev->dev;
	platform_set_drvdata(pdev, card);
	snd_soc_card_set_drvdata(card, machine);

	ret = snd_soc_register_card(card);
	if (ret) {
		AUD_ERR("snd_soc_register_card failed %d\n",ret);
		goto err_unregister_switch;
	}
	AUD_DBG("DONE snd_soc_register_card()\n");

	return 0;

err_unregister_switch:

err_fini_utils:
	tegra_asoc_utils_fini(&machine->util_data);
err_free_machine:
	kfree(machine);
	return ret;
}

static int __devexit tegra_aic3008_driver_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	struct tegra_aic3008 *machine = snd_soc_card_get_drvdata(card);

	snd_soc_unregister_card(card);
	kfree(machine);

	return 0;
}

static struct platform_driver tegra_aic3008_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.pm = &snd_soc_pm_ops,
	},
	.probe = tegra_aic3008_driver_probe,
	.remove = __devexit_p(tegra_aic3008_driver_remove),
};

static int __init tegra_aic3008_modinit(void)
{
	AUD_DBG("Start tegra_aic3008_modinit\n");
	return platform_driver_register(&tegra_aic3008_driver);
}
module_init(tegra_aic3008_modinit);

static void __exit tegra_aic3008_modexit(void)
{
	AUD_DBG("Exit tegra_aic3008...\n");
	platform_driver_unregister(&tegra_aic3008_driver);
}
module_exit(tegra_aic3008_modexit);

MODULE_AUTHOR("Kuowei Li <kuowei_li@htc.com>");
MODULE_DESCRIPTION("Tegra+AIC3008 machine ASoC driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);
