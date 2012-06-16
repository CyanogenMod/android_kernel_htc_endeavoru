#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include "gpio-names.h"
#include <linux/regulator/consumer.h>
#include "htc_audio_power.h"
#include <mach/board_htc.h>

#undef LOG_TAG
#define LOG_TAG "AUD"

#undef PWR_DEVICE_TAG
#define PWR_DEVICE_TAG LOG_TAG

#undef AUDIO_DEBUG
#define AUDIO_DEBUG 0

#define AUD_ERR(fmt, ...) pr_tag_err(LOG_TAG, fmt, ##__VA_ARGS__)
#define AUD_INFO(fmt, ...) pr_tag_info(LOG_TAG, fmt, ##__VA_ARGS__)

#if AUDIO_DEBUG
#define AUD_DBG(fmt, ...) pr_tag_info(LOG_TAG, fmt, ##__VA_ARGS__)
#else
#define AUD_DBG(fmt, ...) pr_tag_dbg(LOG_TAG, fmt, ##__VA_ARGS__)
#endif

struct aic3008_power aic3008_power;
extern void config_tegra_desk_aud_gpios(bool output, bool out_val);

static void aic3008_powerinit(void)
{
	int value = htc_get_pcbid_info();

	if (value >= PROJECT_PHASE_XA) {
		power_config("AUD_MCLK_EN", TEGRA_GPIO_PX7, INIT_OUTPUT_HIGH);
		power_config("AUD_MCLK_EN", TEGRA_GPIO_PX7, GPIO_OUTPUT);
		power_config("AUD_AIC3008_RST#", TEGRA_GPIO_PW5, INIT_OUTPUT_HIGH);
		power_config("AUD_AIC3008_RST#", TEGRA_GPIO_PW5, GPIO_OUTPUT);
		power_config("v_aud_a1v8", TEGRA_GPIO_PD2, REGULATOR_METHOD);
		power_config("v_aud_3v3", TEGRA_GPIO_PB2, REGULATOR_METHOD);
	} else AUD_ERR("%s: no pcbid satisfy.", __func__);

	power_config("AUD_MCLK", TEGRA_GPIO_PW4, INIT_OUTPUT_LOW);
	sfio_deconfig("AUD_MCLK", TEGRA_GPIO_PW4);
	power_config("AUD_SPK_EN", TEGRA_GPIO_PP6, INIT_OUTPUT_LOW);
	sfio_deconfig("AUD_SPK_EN", TEGRA_GPIO_PP6);
	power_config("AUD_LINEOUT_EN", TEGRA_GPIO_PP7, INIT_OUTPUT_LOW);
	sfio_deconfig("AUD_LINEOUT_EN", TEGRA_GPIO_PP7);
	common_init();

	spin_lock_init(&aic3008_power.spin_lock);
	aic3008_power.isPowerOn = true;

	return;
}

static void aic3008_resume(void)
{
	spin_lock(&aic3008_power.spin_lock);
	power_config("AUD_MCLK_EN", TEGRA_GPIO_PX7, GPIO_OUTPUT);
	common_config();
	aic3008_power.isPowerOn = true;
	spin_unlock(&aic3008_power.spin_lock);
	return;
}

static void aic3008_suspend(void)
{
	spin_lock(&aic3008_power.spin_lock);
	power_deconfig("AUD_MCLK_EN", TEGRA_GPIO_PX7, GPIO_OUTPUT);
	common_deconfig();
	aic3008_power.isPowerOn = false;
	spin_unlock(&aic3008_power.spin_lock);
	return;
}

static void aic3008_mic_powerup(void)
{
	/* No Need to Mic PowerUp */
	return;
}

static void aic3008_mic_powerdown(void)
{
	/* No Need to Mic PowerDown */
	return;
}

static void aic3008_amp_powerup(int type)
{
	switch (type) {
	case HEADSET_AMP:
		break;
	case SPEAKER_AMP:
		mdelay(50);
		power_config("AUD_SPK_EN", TEGRA_GPIO_PP6, GPIO_OUTPUT);
		break;
	case DOCK_AMP:
		mdelay(50);
		power_config("AUD_LINEOUT_EN", TEGRA_GPIO_PP7, GPIO_OUTPUT);
		config_tegra_desk_aud_gpios(true, true);
		break;
	}
	return;
}

static void aic3008_amp_powerdown(int type)
{
	switch (type) {
	case HEADSET_AMP:
		break;
	case SPEAKER_AMP:
		power_deconfig("AUD_SPK_EN", TEGRA_GPIO_PP6, GPIO_OUTPUT);
		break;
	case DOCK_AMP:
		power_deconfig("AUD_LINEOUT_EN", TEGRA_GPIO_PP7, GPIO_OUTPUT);
		config_tegra_desk_aud_gpios(false, true);
		break;
	}
	return;
}

struct aic3008_power aic3008_power = {
	.mic_switch = false,
	.amp_switch = true,
	.powerinit = aic3008_powerinit,
	.resume = aic3008_resume,
	.suspend = aic3008_suspend,
	.mic_powerup = aic3008_mic_powerup,
	.mic_powerdown = aic3008_mic_powerdown,
	.amp_powerup = aic3008_amp_powerup,
	.amp_powerdown = aic3008_amp_powerdown,
};
struct aic3008_power *aic3008_power_ctl = &aic3008_power;
EXPORT_SYMBOL_GPL(aic3008_power_ctl);
