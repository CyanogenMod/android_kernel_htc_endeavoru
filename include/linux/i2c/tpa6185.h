/*
 * Definitions for tpa6185 speaker amp chip.
 */
#ifndef TPA6185_H
#define TPA6185_H

#include <linux/ioctl.h>
#include <sound/audio_debug.h>

#define TPA6185_I2C_NAME "tpa6185"
struct tpa6185_platform_data {
	uint32_t gpio_tpa6185_headset_en;

};

void set_tpa6185_headsetamp(int en, int dsp_mode);
#endif
