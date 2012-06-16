#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include "gpio-names.h"
#include <linux/regulator/consumer.h>
#include "htc_audio_power.h"

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
#define AUD_DBG(fmt, ...) do { } while (0)
#endif

static struct regulator *audio_regulator = NULL;

void power_config(const char *name, int pin, int method)
{
	int ret = 0;

	switch (method) {
	case REGULATOR_METHOD:
		audio_regulator = regulator_get(NULL, name);
		if (IS_ERR_OR_NULL(audio_regulator)) {
			AUD_ERR("[PWR] couldn't get regulator %s, pin = %d, addr = %p.\n", name, pin, &audio_regulator);
			return;
		}

		ret = regulator_is_enabled(audio_regulator);
		if (ret > 0) {
			AUD_DBG("[PWR] regulator %s was enabled, pin = %d.\n", name, pin);
			return;
		} else if (ret < 0) {
			AUD_ERR("[PWR] regulator_is_enable error.\n");
			return;
		}

		ret = regulator_enable(audio_regulator);
		if (ret < 0) {
			AUD_ERR("[PWR] couldn't enable regulator %s, pin = %d, ret = %d.\n", name, pin, ret);
		}

		AUD_INFO("[PWR] ***** regulator %s %d enable *****\n", name, pin);
		break;
	case GPIO_OUTPUT:
		gpio_direction_output(pin, 1);
		tegra_gpio_enable(pin);
		gpio_set_value(pin, 1);
		AUD_INFO("[PWR] ***** gpio %s %d output enable *****\n", name, pin);
		break;
	case GPIO_INPUT:
		gpio_direction_input(pin);
		tegra_gpio_enable(pin);
		AUD_INFO("[PWR] ***** gpio %s %d input enable *****\n", name, pin);
		break;
	case INIT_OUTPUT_LOW:
		gpio_request(pin, name);
		gpio_direction_output(pin, 0);
		break;
	case INIT_OUTPUT_HIGH:
		gpio_request(pin, name);
		gpio_direction_output(pin, 1);
		break;
	case INIT_INPUT:
		gpio_request(pin, name);
		gpio_direction_input(pin);
		break;
	default:
		AUD_ERR("[PWR] ***** power_configure nothing *****\n", name, pin);
	}

	return;
}

void power_deconfig(const char *name, int pin, int method)
{
	int ret = 0;

	switch (method) {
	case REGULATOR_METHOD:
		audio_regulator = regulator_get(NULL, name);
		if (IS_ERR_OR_NULL(audio_regulator)) {
			AUD_ERR("[PWR] couldn't get regulator %s %d, addr =  %p.\n", name, pin, &audio_regulator);
			return;
		}

		ret = regulator_is_enabled(audio_regulator);
		if (ret == 0)	{
			AUD_DBG("[PWR] regulator %s was disabled, pin = %d.\n", name, pin);
			return;
		} else if (ret < 0) {
			AUD_ERR("[PWR] regulator_is_enable error.\n");
			return;
		}

		ret = regulator_disable(audio_regulator);
		if (ret < 0) {
			AUD_ERR("[PWR] couldn't enable regulator %s %d, ret = %d.\n", name, pin, ret);
		}

		AUD_INFO("[PWR] ***** regulator %s %d disable *****\n", name, pin);
		break;
	case GPIO_OUTPUT:
		gpio_set_value(pin, 0);
		AUD_INFO("[PWR] ***** gpio %s %d disable *****\n", name, pin);
		break;
	default:
		AUD_ERR("[PWR] ***** power_deconfig nothing *****\n", name, pin);
	}

	return;
}

void sfio_config(const char *name, int pin)
{
	tegra_gpio_disable(pin);
	AUD_DBG("[PWR] ***** SFIO %s %d enable *****\n", name, pin);
	return;
}

void sfio_deconfig(const char *name, int pin)
{
	tegra_gpio_enable(pin);
	AUD_DBG("[PWR] ***** SFIO %s %d disable *****\n", name, pin);
	return;
}

void common_init(void)
{
	power_config("AUD_I2S_WS", TEGRA_GPIO_PA2, INIT_INPUT);
	power_config("AUD_I2S_SCK", TEGRA_GPIO_PA3, INIT_INPUT);
	power_config("AUD_I2S_DOUT", TEGRA_GPIO_PA4, INIT_INPUT);
	power_config("AUD_I2S_DIN", TEGRA_GPIO_PA5, INIT_INPUT);
	power_config("AUD_SPI_MOSI", TEGRA_GPIO_PX0, INIT_OUTPUT_LOW);
	power_config("AUD_SPI_MISO", TEGRA_GPIO_PX1, INIT_INPUT);
	power_config("AUD_SPI_SCK", TEGRA_GPIO_PX2, INIT_OUTPUT_LOW);
	power_config("AUD_SPI_CS#", TEGRA_GPIO_PX3, INIT_OUTPUT_HIGH);
}

void common_config(void)
{
	sfio_config("AUD_I2S_WS", TEGRA_GPIO_PA2);
	sfio_config("AUD_I2S_SCK", TEGRA_GPIO_PA3);
	sfio_config("AUD_I2S_DOUT", TEGRA_GPIO_PA4);
	sfio_config("AUD_I2S_DIN", TEGRA_GPIO_PA5);
	sfio_config("AUD_SPI_MOSI", TEGRA_GPIO_PX0);
	sfio_config("AUD_SPI_MISO", TEGRA_GPIO_PX1);
	sfio_config("AUD_SPI_SCK", TEGRA_GPIO_PX2);
	sfio_config("AUD_SPI_CS#", TEGRA_GPIO_PX3);
}

void common_deconfig(void)
{
	sfio_deconfig("AUD_I2S_WS", TEGRA_GPIO_PA2);
	sfio_deconfig("AUD_I2S_SCK", TEGRA_GPIO_PA3);
	sfio_deconfig("AUD_I2S_DOUT", TEGRA_GPIO_PA4);
	sfio_deconfig("AUD_I2S_DIN", TEGRA_GPIO_PA5);
	sfio_deconfig("AUD_SPI_MOSI", TEGRA_GPIO_PX0);
	sfio_deconfig("AUD_SPI_MISO", TEGRA_GPIO_PX1);
	sfio_deconfig("AUD_SPI_SCK", TEGRA_GPIO_PX2);
	sfio_deconfig("AUD_SPI_CS#", TEGRA_GPIO_PX3);
}
