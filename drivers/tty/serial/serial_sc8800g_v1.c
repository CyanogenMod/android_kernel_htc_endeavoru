/*
 * drivers/serial/serial_sc8800g.c
 *
 * Serial driver over SPI for Spreadtrum SC8800G modem
 *
 * Copyright (C) 2011 NVIDIA Corporation
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

//#define ENABLE_SC8800_AP_RDY_DELAY_V1
#define ENABLE_SC8800_DATA_CHECKSUM_V1

/*For TX data stall debug*/
//#define ENABLE_SC8800_962049_DEBUG

/* enable/disable SC8800 log */
//#define SC8800_ENABLE_LOG_V1

#ifdef SC8800_ENABLE_LOG_V1
#define DEBUG           1
#define VERBOSE_DEBUG   1
#endif//SC8800_ENABLE_LOG
#define SC8800G_POWER_ONOFF 1

#include <linux/module.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/termios.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/string.h>
#include <linux/gpio.h>
#include <linux/spi/spi.h>
//#include <linux/debugfs.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/serial_sc8800g.h>
#include <linux/kthread.h>
#include <linux/ctype.h>
#include <linux/wakelock.h>

#include <mach/dma.h>
#include <mach/clk.h>
#include <mach/cable_detect.h>
/* debugging stuffs */
#define DRIVER_NAME "sc8800g_v1: "
#define LEVEL_VERBOSE	5
#define LEVEL_DEBUG		4
#define LEVEL_INFO		3
#define LEVEL_WARNING	2
#define LEVEL_ERROR		1

static int sc8800g_debug_v1 = LEVEL_VERBOSE;

#ifdef VERBOSE_DEBUG
#define VDBG(fmt, args...) if (sc8800g_debug_v1 >= LEVEL_VERBOSE) \
        pr_debug(DRIVER_NAME fmt, ## args);
#else
#define VDBG(fmt, args...)
#endif

#ifdef DEBUG
#define DBG(fmt, args...) if (sc8800g_debug_v1 >= LEVEL_DEBUG) \
        pr_debug(DRIVER_NAME fmt, ## args);
#else
#define DBG(fmt, args...)
#endif

#define INFO(fmt, args...) if (sc8800g_debug_v1 >= LEVEL_INFO) \
        pr_info(DRIVER_NAME fmt, ## args);

#define WARNING(fmt, args...) if (sc8800g_debug_v1 >= LEVEL_WARNING) \
        pr_warn(DRIVER_NAME fmt, ## args);

#define ERROR(fmt, args...) if (sc8800g_debug_v1 >= LEVEL_ERROR) \
        pr_err(DRIVER_NAME fmt, ## args);

static int dumpsize_v1 = 16 * 8; /*TODO: sysfs*/
#ifdef VERBOSE_DEBUG
static void hexdump_v1(const unsigned char *data, int len)
{
#define DATA_PER_LINE   16
#define CHAR_PER_DATA   3 /* in "%02X " format */
        char linebuf[DATA_PER_LINE*CHAR_PER_DATA + 1];
        int i;
        char *ptr = &linebuf[0];

        if (len > dumpsize_v1)
                len = dumpsize_v1;

        for (i = 0; i < len; i++) {
                sprintf(ptr, " %02x ", data[i]);
                ptr += CHAR_PER_DATA;
                if (((i +1) % DATA_PER_LINE) == 0) {
                        *ptr = '\0';
                        VDBG("%s\n", linebuf);
                        ptr = &linebuf[0];
                }
        }

        /* print the last line */
        if (ptr != &linebuf[0]) {
                *ptr = '\0';
                VDBG("%s\n", linebuf);
        }

}
#else
#define hexdump_v1(args...)
#endif

static void error_hexdump_v1(const unsigned char *data, int len)
{
#define DATA_PER_LINE   16
#define CHAR_PER_DATA   3
        char linebuf[DATA_PER_LINE*CHAR_PER_DATA + 1];
        int i;
        char *ptr = &linebuf[0];
        if (len > dumpsize_v1)
                len = dumpsize_v1;

        for (i = 0; i < len; i++) {
		sprintf(ptr, " %02x", data[i]);
		ptr += CHAR_PER_DATA;
		if (((i + 1) % DATA_PER_LINE) == 0) {
			*ptr = '\0';
			ERROR("%s\n", linebuf);
			ptr = &linebuf[0];
                }
        }

        /* print the last line */
        if (ptr != &linebuf[0]) {
                *ptr = '\0';
                ERROR("%s\n", linebuf);
        }

}

#ifdef ENABLE_SC8800_962049_DEBUG
static void error_hexdump2_v1(const unsigned char *data, int len)
{
#define DATA_PER_LINE   16
#define CHAR_PER_DATA   3
        char linebuf[DATA_PER_LINE*CHAR_PER_DATA + 1];
        int i;
        char *ptr = &linebuf[0];

        for (i = 0; i < len; i++) {
		sprintf(ptr, " %02x", data[i]);
		ptr += CHAR_PER_DATA;
		if (((i + 1) % DATA_PER_LINE) == 0) {
			*ptr = '\0';
			ERROR("%s\n", linebuf);
			ptr = &linebuf[0];
                }
        }

        /* print the last line */
        if (ptr != &linebuf[0]) {
                *ptr = '\0';
                ERROR("%s\n", linebuf);
        }

}
#endif

#define sc8800g_align_v1(len, align)		(((len) + (align) - 1)/(align)) * (align)

static DEFINE_MUTEX(dev_mutex);
static struct wake_lock g_sc8800g_wakelock_tx_v1 = {NULL};
static struct wake_lock g_sc8800g_wakelock_rx_v1 = {NULL};
struct sc8800g_mdm_dev *sc8800g_mdv_v1 = NULL;
static int g_enable_spi_byte_counter = 0;
static int g_enable_tty_write_log = 0;
static int g_sc8800g_spi_sync_tx_state = -1;
static int g_sc8800g_spi_sync_rx_state = -1;
static int norxheader = 0;

#ifdef ENABLE_SC8800_962049_DEBUG
extern void spi_tegra_962049_debug_set(int enable);
extern void spi_tegra_962049_debug_get_info();
unsigned char g_mdm_temp_data_buf[SEND_BUFFER_SIZE] = {NULL};
int g_mdm_temp_data_buf_size = 0;
int g_tx_pending_count = 0;
#endif

static int sc8800g_irq_onoff_v1(struct serial_sc8800g_platform_data *pdata, MODEM_IRQ_STATE onoff, struct sc8800g_mdm_dev *mdv);
static int sc8800g_gpio_onoff_v1(struct serial_sc8800g_platform_data *pdata, MODEM_GPIO_STATE onoff, bool include_power);
static int sc8800g_gpio_onoff_ext_v1(struct serial_sc8800g_platform_data *pdata, int n_gpio, MODEM_GPIO_STATE onoff);
static int sc8800g_gpio_print_v1(struct serial_sc8800g_platform_data *pdata);

#ifdef ENABLE_SC8800_DATA_CHECKSUM_V1
static inline unsigned int sc8800g_checksum_v1( const unsigned char * data_buf, uint buf_size )
{
	int i = 0;
	unsigned int sum =0;

	if ( data_buf == NULL || buf_size == 0 ) {
		ERROR("[%s] data_buf=[0x%p], buf_size=[%d]\n", __func__, data_buf, buf_size);
		return 0;
	}

	for ( i = 0; i < buf_size; i++ ) {
	        sum += data_buf[i];
	}

       return sum;
}
#endif

static void sc8800g_print_info(struct sc8800g_mdm_dev *mdv)
{
	struct spi_device *spi = NULL;
	struct serial_sc8800g_platform_data *pdata = NULL;
	if ( mdv == NULL ) {
		ERROR("[%s] mdv=NULL", __func__);
		return;
	}

	if ( mdv->spi == NULL ) {
		ERROR("[%s] mdv->spi=NULL", __func__);
		return;
	}
	spi = mdv->spi;
	pdata = spi->dev.platform_data;

	sc8800g_gpio_print_v1(pdata);
	INFO("[%s]atomic: modem_live=[%d], tx_done=[%d], tx_pending=[%d], rx_pending=[%d]", __func__,
		atomic_read(&mdv->modem_live),
		atomic_read(&mdv->tx_done),
		atomic_read(&mdv->tx_pending),
		atomic_read(&mdv->rx_pending));
}

static void sc8800g_notify_kobject_release_v1(struct kobject *kobj)
{
	INFO("sc8800g_notify_kobject_release_v1\n");
	return;
}

static struct kobj_type sc8800g_notify_ktype_v1 = {
	.release = sc8800g_notify_kobject_release_v1,
};

static void sc8800g_sim_door_notify_work_func_v1(struct work_struct *work)
{
	struct sc8800g_mdm_dev *mdv = NULL;
	struct spi_device *spi = NULL;
	struct serial_sc8800g_platform_data *pdata = NULL;
	struct sc8800g_gpio *gpio = NULL;
	int val_sim_door = 0;
	char message[20] = "GPIO_SIM_DOOR:%d";
	char *envp[] = { message, NULL };

	mdv = container_of(work, struct sc8800g_mdm_dev, sim_door_notify_work);

	if ( mdv == NULL ) {
		ERROR("[%s] mdv=NULL", __func__);
		goto out;
	}

	if ( mdv->spi == NULL ) {
		ERROR("[%s] mdv->spi=NULL", __func__);
		goto out;
	}
	spi = mdv->spi;
	pdata = spi->dev.platform_data;
	gpio = &pdata->gpio[SIM_DOOR];
	mdelay(100);

	val_sim_door = gpio_get_value(gpio->pinnum);

	INFO("[%s] new sim_door=[%d], old sim_door=[%d]\n", __func__, val_sim_door, mdv->sim_door_state);
	if(mdv->sim_door_state != val_sim_door) {
		mdv->sim_door_state = val_sim_door;
		sprintf( envp[0], "GPIO_SIM_DOOR:%d", val_sim_door);
		INFO("[%s] kobject_uevent_env+, send=[%s]\n", __func__, envp[0]);
		kobject_uevent_env(&mdv->sim_door_kobj, KOBJ_CHANGE, envp);
		INFO("[%s] kobject_uevent_env-\n", __func__);
	}
out:
	return;
}

static void sc8800g_crash_notify_work_func_v1(struct work_struct *work)
{
	struct sc8800g_mdm_dev *mdv = NULL;
	struct spi_device *spi = NULL;
	struct serial_sc8800g_platform_data *pdata = NULL;
	struct sc8800g_gpio *gpio_alive = NULL, *gpio_power = NULL, *gpio_mdm2ap1 = NULL;
	int val_alive = 0, val_power = 0, val_mdm2ap1 = 0;
	char message[20] = "GPIO_MDM_CRASH";
	char *envp[] = { message, NULL };

	mdv = container_of(work, struct sc8800g_mdm_dev, crash_notify_work);

	if ( mdv == NULL ) {
		ERROR("[%s] mdv=NULL", __func__);
		goto out;
	}

	if ( mdv->spi == NULL ) {
		ERROR("[%s] mdv->spi=NULL", __func__);
		goto out;
	}

	spi = mdv->spi;
	pdata = spi->dev.platform_data;
	gpio_alive = &pdata->gpio[MDM_ALIVE];
	gpio_power = &pdata->gpio[MDM_POWER];
	gpio_mdm2ap1 = &pdata->gpio[MDM_TO_AP1];

	val_alive = gpio_get_value(gpio_alive->pinnum);
	val_power = gpio_get_value(gpio_power->pinnum);
	val_mdm2ap1 = gpio_get_value(gpio_mdm2ap1->pinnum);

	INFO("[%s] val_alive=[%d], val_power=[%d], val_mdm2ap1=[%d]\n", __func__, val_alive, val_power, val_mdm2ap1);

	// Only notify modem crash when MDM_ALIVE is low && MDM_POWER is hi
	if( val_alive == 0 && val_power == 1 ) {
		atomic_set(&sc8800g_mdv_v1->crash_state, 1);
		INFO("[%s] kobject_uevent_env+, send=[%s]\n", __func__, envp[0]);
		kobject_uevent_env(&mdv->crash_kobj, KOBJ_CHANGE, envp);
		INFO("[%s] kobject_uevent_env-\n", __func__);
	}
out:
	return;
}

static void sc8800g_usb_plugged_func_v1(int cable_type)
{
	struct sc8800g_mdm_dev *mdv = sc8800g_mdv_v1;
	char message[20] = "USB_PLUG";
	char *envp[] = { message, NULL };

	INFO("[%s] USB plug status=[%d]\n", __func__, cable_type);
	sprintf( envp[0], "USB_PLUG:%d", cable_type);

	INFO("[%s] kobject_uevent_env+, send=[%s]\n", __func__, envp[0]);
	kobject_uevent_env(&mdv->usb_plugged_kobj, KOBJ_CHANGE, envp);
	INFO("[%s] kobject_uevent_env-\n", __func__);
out:
	return;
}

static void sc8800g_mdm_to_ap1_work_func_v1(struct work_struct *work)
{
	struct sc8800g_mdm_dev *mdv = NULL;
	struct spi_device *spi = NULL;
	struct serial_sc8800g_platform_data *pdata = NULL;
	struct sc8800g_gpio
		*gpio_mdm_rts = NULL,
		*gpio_mdm_rdy = NULL,
		*gpio_mdm_alive = NULL,
		*gpio_mdm_to_ap1 = NULL;
	//unsigned long flags = 0;

	mdv = container_of(work, struct sc8800g_mdm_dev, mdm_to_ap1_work);

	if ( mdv == NULL ) {
		ERROR("[%s] mdv=NULL", __func__);
		goto out;
	}

	if ( mdv->spi == NULL ) {
		ERROR("[%s] mdv->spi=NULL", __func__);
		goto out;
	}
	spi = mdv->spi;
	pdata = spi->dev.platform_data;

	if ( pdata == NULL ) {
		ERROR("[%s] pdata=NULL", __func__);
		goto out;
	}

	//spin_lock_irqsave(&mdv->power_lock, flags);
	mutex_lock(&mdv->power_mutex);

	gpio_mdm_rts = &pdata->gpio[MDM_RTS];
	gpio_mdm_rdy = &pdata->gpio[MDM_RDY];
	gpio_mdm_alive = &pdata->gpio[MDM_ALIVE];
	gpio_mdm_to_ap1 = &pdata->gpio[MDM_TO_AP1];
#if 0
	if (gpio_mdm_rts->irq_enabled == true) {
		//INFO("[%s] disable_irq mdm_rts_gpio[%d].\n",  __func__, gpio_to_irq(pdata->mdm_rts_gpio));
		disable_irq(gpio_to_irq(pdata->mdm_rts_gpio));
		gpio_mdm_rts->irq_enabled = false;
	}
	if (gpio_mdm_rdy->irq_enabled == true) {
		//INFO("[%s] disable_irq mdm_rdy_gpio[%d].\n",  __func__, gpio_to_irq(pdata->mdm_rdy_gpio));
		disable_irq(gpio_to_irq(pdata->mdm_rdy_gpio));
		gpio_mdm_rdy->irq_enabled = false;
	}
	if (gpio_mdm_alive->irq_enabled == true) {
		//INFO("[%s] disable_irq mdm_alive_gpio[%d].\n",  __func__, gpio_to_irq(pdata->mdm_alive_gpio));
		disable_irq(gpio_to_irq(pdata->mdm_alive_gpio));
		gpio_mdm_alive->irq_enabled = false;
	}
	if (gpio_mdm_to_ap1->irq_enabled == true) {
		//INFO("[%s] disable_irq mdm_to_ap1_gpio[%d].\n",  __func__, gpio_to_irq(pdata->mdm_to_ap1_gpio));
		disable_irq(gpio_to_irq(pdata->mdm_to_ap1_gpio));
		gpio_mdm_to_ap1->irq_enabled = false;
	}
#else
	sc8800g_irq_onoff_v1( pdata, MDM_IRQ_DISABLE, mdv );
#endif
	sc8800g_gpio_onoff_ext_v1( pdata, MDM_POWER, MDM_GPIO_OFF );
	sc8800g_gpio_onoff_v1( pdata, MDM_GPIO_OFF, true );
	//spin_unlock_irqrestore(&mdv->power_lock, flags);
	atomic_set(&mdv->modem_live, 0);
	mutex_unlock(&mdv->power_mutex);

	INFO("[%s] modem power off process done\n", __func__);
out:
	return;
}

static int sc8800g_gpio_print_v1(struct serial_sc8800g_platform_data *pdata)
{
	struct sc8800g_gpio *gpio = NULL;
	int i = 0;
	int val_gpio = 0;
	/*initial gpio setting*/
	for (i = 0; i < SC8800G_GPIO_MAX; i++) {
		gpio = &pdata->gpio[i];

		if ( (!gpio->name) || (!strlen(gpio->name))) {
			ERROR("[%s] invalid name for gpio %d\n", __func__, i);
			return -1;
		}

		val_gpio = gpio_get_value( gpio->pinnum);

		VDBG("[%s]: i=%d, name=%s, num=%d, type=%d, val=%d\n",
			__func__, i, gpio->name, gpio->pinnum, gpio->type, val_gpio );
	}
	return 0;
}

#ifdef ENABLE_SC8800_962049_DEBUG
static int sc8800g_gpio_print2_v1(struct serial_sc8800g_platform_data *pdata)
{
	struct sc8800g_gpio *gpio = NULL;
	int i = 0;
	int val_gpio = 0;
	/*initial gpio setting*/
	for (i = 0; i < SC8800G_GPIO_MAX; i++) {
		gpio = &pdata->gpio[i];

		if ( (!gpio->name) || (!strlen(gpio->name))) {
			ERROR("[%s] invalid name for gpio %d\n", __func__, i);
			return -1;
		}

		val_gpio = gpio_get_value( gpio->pinnum);

		INFO("name=%s, num=%d, val=%d", gpio->name, gpio->pinnum, val_gpio );
	}
	return 0;
}
#endif

static int sc8800g_gpio_print_ext_v1(struct serial_sc8800g_platform_data *pdata, int n_gpio)
{
	struct sc8800g_gpio *gpio = NULL;
	int val_gpio = 0;

	if ( n_gpio < 0 && n_gpio > SC8800G_GPIO_MAX ) {
		VDBG("[%s]: n_gpio=[%d]\n", __func__, n_gpio);
		return 0;
	}
	gpio = &pdata->gpio[n_gpio];

	if ( gpio == NULL ) {
		VDBG("[%s]: gpio(%d) == NULL\n", __func__, n_gpio);
		return 0;
	}

	/*initial gpio setting*/

	if ( (!gpio->name) || (!strlen(gpio->name))) {
		ERROR("[%s] invalid name \n", __func__);
		return -1;
	}

	val_gpio = gpio_get_value( gpio->pinnum);

	INFO("name=%s, num=%d, type=%d, val=%d\n",
		gpio->name, gpio->pinnum, gpio->type, val_gpio );
	return 0;
}

static int sc8800g_gpio_onoff_ext_v1(struct serial_sc8800g_platform_data *pdata, int n_gpio, MODEM_GPIO_STATE onoff)
{
	struct sc8800g_gpio *gpio = NULL;
	int i = 0;
	bool True = (1 == 1);

	VDBG("[%s]: onoff=[%d]\n", __func__, onoff);
	if ( n_gpio < 0 && n_gpio > SC8800G_GPIO_MAX ) {
		VDBG("[%s]: n_gpio=[%d]\n", __func__, n_gpio);
		return 0;
	}
	gpio = &pdata->gpio[n_gpio];

	if ( gpio == NULL ) {
		VDBG("[%s]: gpio(%d) == NULL\n", __func__, n_gpio);
		return 0;
	}

	if ( onoff == MDM_GPIO_ON
		|| onoff == MDM_GPIO_INPUT
		|| onoff == MDM_GPIO_OUTPUT_LOW
		|| onoff == MDM_GPIO_OUTPUT_HIGH ) {
		/*initial gpio setting*/
		VDBG("[%s]: enable GPIO: i=%d, name=%s, num=%d, type=%d, disabled=%d\n",
			__func__, i, gpio->name, gpio->pinnum, gpio->type, gpio->disabled);

		if (gpio->is_spi == 1) {
			tegra_gpio_disable(gpio->pinnum);
			return 0;
		}

		if ( (!gpio->name) || (!strlen(gpio->name))) {
			ERROR("[%s] invalid name for gpio %d\n", __func__, i);
			return -1;
		}

		if ((gpio->type != INPUT_PIN) && (gpio->type != OUTPUT_PIN)) {
			ERROR("[%s] invalid gpio pin type for %s\n", __func__, gpio->name);
			return -1;
		}

		if ( gpio->requested == 0 ) {
			gpio_request(gpio->pinnum, gpio->name);
			gpio->requested = 1;
		}

		if (onoff == MDM_GPIO_ON) {
			if (gpio->type == INPUT_PIN) {
				if (gpio_direction_input(gpio->pinnum) < 0) {
					ERROR("[%s] gpio_direction_input failed for %s\n", __func__, gpio->name);
					return -1;
				}
			}

			if (gpio->type == OUTPUT_PIN) {
				if (gpio_direction_output(gpio->pinnum,
					gpio->init_output) < 0) {
					ERROR("[%s] gpio_direction_output failed for %s\n", __func__, gpio->name);
					return -1;
				}
			}
		} else if ( onoff == MDM_GPIO_INPUT) {
			INFO("[%s] set GPIO to input\n", __func__);
			if (gpio_direction_input(gpio->pinnum) < 0) {
				ERROR("[%s] gpio_direction_input failed for %s\n", __func__, gpio->name);
				return -1;
			}
		} else if ( onoff == MDM_GPIO_OUTPUT_LOW || onoff == MDM_GPIO_OUTPUT_HIGH ) {
			int val_def = ( (onoff == MDM_GPIO_OUTPUT_LOW) ? 0 : 1);

			if ( onoff == MDM_GPIO_OUTPUT_LOW ) {
				INFO("[%s] set GPIO to output low\n", __func__);
			} else if ( onoff == MDM_GPIO_OUTPUT_HIGH ) {
				INFO("[%s] set GPIO to output high\n", __func__);
			}

			INFO("[%s] val_def=[%d]\n", __func__, val_def);
			if (gpio_direction_output(gpio->pinnum,
				val_def) < 0) {
				ERROR("[%s] gpio_direction_output failed for %s\n", __func__, gpio->name);
				return -1;
			}
		}
		if ( gpio->exported == 0 ) {
			gpio_export(gpio->pinnum, True);
			gpio->exported = 1;
		}
		tegra_gpio_enable(gpio->pinnum);
	} else if ( onoff == MDM_GPIO_OFF ) {
		VDBG("[%s]: disable GPIO: i=%d, name=%s, num=%d, type=%d, disabled=%d\n",
			__func__, i, gpio->name, gpio->pinnum, gpio->type, gpio->disabled);

		if ( (!gpio->name) || (!strlen(gpio->name))) {
			ERROR("[%s] invalid name for gpio %d\n", __func__, i);
			return -1;
		}

		if ( gpio->requested == 0 ) {
			gpio_request(gpio->pinnum, gpio->name);
			gpio->requested = 1;
		}

		if ( gpio->offtype == INPUT_PIN ) {
			if (gpio_direction_input(gpio->pinnum) < 0) {
				ERROR("[%s] gpio_direction_input failed for %s\n", __func__, gpio->name);
				return -1;
			}
		} else if ( gpio->offtype == OUTPUT_PIN ) {
			if (gpio_direction_output(gpio->pinnum,
				gpio->init_output) < 0) {
				ERROR("[%s] gpio_direction_output failed for %s\n", __func__, gpio->name);
				return -1;
			}
		}
		if ( gpio->exported == 0 ) {
			gpio_export(gpio->pinnum, True);
			gpio->exported = 1;
		}
		tegra_gpio_enable(gpio->pinnum);
	}

	return 0;
}

static int sc8800g_gpio_onoff_v1(struct serial_sc8800g_platform_data *pdata, MODEM_GPIO_STATE onoff, bool include_power)
{
	struct sc8800g_gpio *gpio = NULL;
	int i = 0;
	bool True = (1 == 1);

	VDBG("[%s]: onoff=[%d]\n", __func__, onoff);
	if ( onoff == MDM_GPIO_ON ) {
		for (i = 0; i < SC8800G_GPIO_MAX; i++) {
			gpio = &pdata->gpio[i];

			if ( gpio->disabled == 1 ) {
				continue;
			}

			if (i == MDM_POWER && !include_power) {
				continue;
			}

			if (gpio->is_spi == 1) {
				tegra_gpio_disable(gpio->pinnum);
				continue;
			}

			if ( (!gpio->name) || (!strlen(gpio->name))) {
				ERROR("[%s] invalid name for gpio %d\n", __func__, i);
				return -1;
			}

			if ((gpio->type != INPUT_PIN) && (gpio->type != OUTPUT_PIN)) {
				ERROR("[%s] invalid gpio pin type for %s\n", __func__, gpio->name);
				return -1;
			}

			if ( gpio->requested == 0 ) {
				gpio_request(gpio->pinnum, gpio->name);
				gpio->requested = 1;
			}

			if (gpio->type == INPUT_PIN) {
				if (gpio_direction_input(gpio->pinnum) < 0) {
					ERROR("[%s] gpio_direction_input failed for %s\n", __func__, gpio->name);
					return -1;
				}
			}

			if (gpio->type == OUTPUT_PIN) {
				if (gpio_direction_output(gpio->pinnum,
					gpio->init_output) < 0) {
					ERROR("[%s] gpio_direction_output failed for %s\n", __func__, gpio->name);
					return -1;
				}
			}
			if ( gpio->exported == 0 ) {
				gpio_export(gpio->pinnum, True);
				gpio->exported = 1;
			}

			tegra_gpio_enable(gpio->pinnum);
		}
	} else if ( onoff == MDM_GPIO_OFF ) {
		for (i = 0; i < SC8800G_GPIO_MAX; i++) {
			gpio = &pdata->gpio[i];

			if (i == MDM_POWER && !include_power) {
				continue;
			}

			if ( (!gpio->name) || (!strlen(gpio->name))) {
				ERROR("[%s] invalid name for gpio %d\n", __func__, i);
				return -1;
			}

			if ( gpio->requested == 0 ) {
				gpio_request(gpio->pinnum, gpio->name);
				gpio->requested = 1;
			}

			if ( gpio->offtype == INPUT_PIN ) {
				if (gpio_direction_input(gpio->pinnum) < 0) {
					ERROR("[%s] gpio_direction_input failed for %s\n", __func__, gpio->name);
					return -1;
				}
			} else if ( gpio->offtype == OUTPUT_PIN ) {
				if (gpio_direction_output(gpio->pinnum, 0) < 0) {
					ERROR("[%s] gpio_direction_output failed for %s\n", __func__, gpio->name);
					return -1;
				}
			}
			if ( gpio->exported == 0 ) {
				gpio_export(gpio->pinnum, True);
				gpio->exported = 1;
			}
			tegra_gpio_enable(gpio->pinnum);
		}

	} else if ( onoff == MDM_GPIO_INPUT) {
		for (i = 0; i < SC8800G_GPIO_MAX; i++) {
			gpio = &pdata->gpio[i];

			if (i == MDM_POWER && !include_power) {
				continue;
			}

			if ( (!gpio->name) || (!strlen(gpio->name))) {
				ERROR("[%s] invalid name for gpio %d\n", __func__, i);
				return -1;
			}

			if ( gpio->requested == 0 ) {
				gpio_request(gpio->pinnum, gpio->name);
				gpio->requested = 1;
			}
			if (gpio_direction_input(gpio->pinnum) < 0) {
				ERROR("[%s] gpio_direction_input failed for %s\n", __func__, gpio->name);
				return -1;
			}
			if ( gpio->exported == 0 ) {
				gpio_export(gpio->pinnum, True);
				gpio->exported = 1;
			}
			tegra_gpio_enable(gpio->pinnum);
		}

	}
	return 0;
}

static inline int sc8800g_atomic_read_timeout_v1(atomic_t *ato, int loop)
{
	if(loop <= 0)
		return 0;
	while(1 == atomic_read(ato) && loop--) {
		udelay(10);
//		usleep(1);
	}
	if(loop) {
		return 0;
	} else {
		return -1;
	}
 }

static inline void sc8800g_tx_data_set_v1(struct sc8800g_mdm_dev *mdv, const unsigned char *buf, ssize_t len)
{
	unsigned long flags;

	spin_lock_irqsave(&mdv->tx_lock, flags);
	mdv->tx.addr = (void *)buf;
	mdv->tx.len = len;
	spin_unlock_irqrestore(&mdv->tx_lock, flags);
}

static inline void sc8800g_tx_data_clear_v1(struct sc8800g_mdm_dev *mdv)
{
	unsigned long flags;

	spin_lock_irqsave(&mdv->tx_lock, flags);
	mdv->tx.addr = NULL;
	mdv->tx.len = 0;
	spin_unlock_irqrestore(&mdv->tx_lock, flags);
}

static struct sc8800g_mdm_packet *sc8800g_tx_pack_data_v1(struct sc8800g_mdm_dev *mdv, int *len)
{
	struct sc8800g_mdm_packet *pkt;

	pkt = (struct sc8800g_mdm_packet *)mdv->send.buf;

	pkt->head.tag = HEADER_TAG;
	pkt->head.type = HEADER_TYPE;
	pkt->head.length = mdv->tx.len;
	pkt->head.index = 0;//no sense
#ifdef ENABLE_SC8800_DATA_CHECKSUM_V1
	pkt->head.header_valid = sc8800g_checksum_v1((const unsigned char *)mdv->tx.addr, mdv->tx.len);//no sense
#else
	pkt->head.header_valid = HEADER_INVALID;//no sense
#endif

	/* fix memory corruption issue, it will leakage and set  more 16 byte to 0x00 */
	//memset((void *)pkt->data, 0x00, SEND_BUFFER_SIZE);
	memset((void *)pkt->data, 0x00, SC8800G_TX_BUFSIZE);

	memcpy((void *)pkt->data, mdv->tx.addr, mdv->tx.len);

	*len = pkt->head.length + PACKET_HEADER_SIZE;

#ifdef ENABLE_SC8800_962049_DEBUG
	memset((void *)&g_mdm_temp_data_buf, 0x00, SEND_BUFFER_SIZE);

	memcpy((void *)&g_mdm_temp_data_buf, mdv->tx.addr, mdv->tx.len);

	g_mdm_temp_data_buf_size = *len;
#endif

	return pkt;

}

static int sc8800g_write_v1(struct sc8800g_mdm_dev *mdv, const u8 * buf,
			  int count)
{
	struct spi_transfer	t;
	struct spi_message m;
	struct spi_device *spi = mdv->spi;

	int ret = 0;
	DBG("[%s] %d bytes\n", __func__, count);
//#ifdef SC8800_DATA_DEBUG
	error_hexdump_v1(buf, count>10?10:count);
//#endif//SC8800_DATA_DEBUG
#if 1
	memset(&t, 0, sizeof(struct spi_transfer));
	t.tx_buf = buf;
	t.len = count;
	t.bits_per_word = SC8800G_TRANSFER_BITS_PER_WORD;
	t.speed_hz = spi->max_speed_hz;

	mutex_lock(&mdv->data_mutex);
	g_sc8800g_spi_sync_tx_state = 1;
	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	ret = spi_sync(spi, &m);
	g_sc8800g_spi_sync_tx_state = 0;
	mutex_unlock(&mdv->data_mutex);
#else
	ret = spi_write(spi, buf, count);
#endif
	if (ret) {
		ERROR("[%s] spi_sync failed, ret=%d\n",	__func__, ret);
	} else {
		if ( g_enable_spi_byte_counter ) {
			mdv->total_tx_bytes += count;
			INFO("write %d Bytes\n", mdv->total_tx_bytes);
		}
	}
	return count;
}

static void sc8800g_tx_work_v1(struct work_struct * work)
{
	struct sc8800g_mdm_dev *mdv = container_of(work, struct sc8800g_mdm_dev, tx_work);
	struct spi_device *spi = mdv->spi;
	struct serial_sc8800g_platform_data *pdata = spi->dev.platform_data;
	struct sc8800g_mdm_packet *pkt = NULL;
	int pkt_len = 0;
	int ret = -1;
	unsigned long flags;
	/*only for debug tx timeout, should remove after find root cause*/
	if ( g_enable_tty_write_log ) {
		INFO("[%s]~\n", __func__);
	} else {
		DBG("[%s]~\n", __func__);
	}

	if(mdv->suspended) {
		goto tx_work_end;
	}

	if(!atomic_read(&mdv->modem_live))
		goto tx_work_end;

	spin_lock_irqsave(&mdv->tx_lock, flags);
	if(!mdv->tx.addr || (0 == mdv->tx.len)) {
		 spin_unlock_irqrestore(&mdv->tx_lock, flags);
		goto tx_work_end;
	}
	spin_unlock_irqrestore(&mdv->tx_lock, flags);

	if(test_and_clear_bit(FLAG_BP_RDY, &mdv->flags)) {
		ret = sc8800g_atomic_read_timeout_v1(&mdv->rx_pending, 2000);
		if(ret < 0) {
			ERROR("[%s] rx still pending, 20ms timeout.\n", __func__);
			goto tx_work_end;
		}

		atomic_set(&mdv->tx_pending, 1);
		mdv->transfer_pending = 1;

		pkt = sc8800g_tx_pack_data_v1(mdv, &pkt_len);

		pkt_len = sc8800g_align_v1(pkt_len, PACKET_TX_ALIGNMENT);

		ret = sc8800g_write_v1(mdv, (const u8 *)pkt, pkt_len);
		if(ret < 0) {
			ERROR("[%s] sc8800g tx error, ret=%d\n", __func__, ret);
		}

		atomic_set(&mdv->tx_pending, 0);
		mdv->transfer_pending = 0;
	}
tx_work_end:
	/*deassert ap_rts*/
	/*only for debug tx timeout, should remove after find root cause*/
	if ( g_enable_tty_write_log ) {
		INFO("[%s]set AP_RTS to Low\n", __func__);
	} else {
		DBG("[%s]set AP_RTS to Low\n", __func__);
	}
	gpio_set_value(pdata->gpio[AP_RTS].pinnum, 0);
	sc8800g_tx_data_clear_v1(mdv);

	if (wake_lock_active(&g_sc8800g_wakelock_tx_v1)) {
		wake_unlock(&g_sc8800g_wakelock_tx_v1);
		if ( g_enable_tty_write_log ) {
			INFO("[%s] wake_unlock [%s]\n",
				__func__, g_sc8800g_wakelock_tx_v1.name);
		}
	} else {
		ERROR("[%s] wake_unlock [%s] already unlock\n",
			__func__, g_sc8800g_wakelock_tx_v1.name);
	}

	atomic_set(&mdv->tx_done, (ret < 0)? 0:1);
	wake_up_interruptible(&mdv->tx_end_wait);
	return;
}

/*AP--BP handshaking*/
static int sc8800g_ap_rts_request_v1(struct serial_sc8800g_platform_data *pdata)
{
	int i;
	int mdm_rts;
	int mdm_rdy;

	/*mdm_rts should be high before asserting ap_rts*/
	for (i=0; i < 10000; i++) {
		mdm_rts = gpio_get_value(pdata->gpio[MDM_RTS].pinnum);
		mdm_rdy = gpio_get_value(pdata->gpio[MDM_RDY].pinnum);
		if (0 == mdm_rts || 0 == mdm_rdy) {
			udelay (1);
			/*only for debug tx timeout, should remove after find root cause*/
			if ( g_enable_tty_write_log  && (i % 1000) == 0) {
				int ap_rts = gpio_get_value(pdata->gpio[AP_RTS].pinnum);
				int ap_rdy = gpio_get_value(pdata->gpio[AP_RDY].pinnum);
				INFO("[%s] mdm_rts=[%d], mdm_rdy=[%d], ap_rts=[%d], ap_rdy=[%d], r=[%d]\n", __func__, mdm_rts, mdm_rdy, ap_rts, ap_rdy, i);
			}
		} else  {
			break;
		}
	}

	if ( i == 10000 && mdm_rts == 0) {
		/* wait mdm_rts fali  */
		int ap_rts = gpio_get_value(pdata->gpio[AP_RTS].pinnum);
		int ap_rdy = gpio_get_value(pdata->gpio[AP_RDY].pinnum);
		INFO("[%s] mdm_rts=[%d], mdm_rdy=[%d], ap_rts=[%d], ap_rdy=[%d], r=[%d]\n", __func__, mdm_rts, mdm_rdy, ap_rts, ap_rdy, i);

		//* We meet Rx header not found issue, modem is lost our AP_RDY signal. Raise again to avoid modem timeout.
		if(norxheader) {
			i = 0;
			norxheader = 0; 
			while(ap_rdy == 1 && mdm_rts==0 && i++<5) {
				gpio_set_value(pdata->gpio[AP_RDY].pinnum, 0);
				udelay(50);
				gpio_set_value(pdata->gpio[AP_RDY].pinnum, 1);
				udelay(100);
				mdm_rts = gpio_get_value(pdata->gpio[MDM_RDY].pinnum);
				mdm_rdy = gpio_get_value(pdata->gpio[MDM_RDY].pinnum);
				ap_rts = gpio_get_value(pdata->gpio[AP_RTS].pinnum);
				ap_rdy = gpio_get_value(pdata->gpio[AP_RDY].pinnum);
				ERROR("Rx Header not found retry:mdm_rts=[%d], mdm_rdy=[%d], ap_rts=[%d], ap_rdy=[%d]\n", mdm_rts, mdm_rdy, ap_rts, ap_rdy);
			}
		};

		/* try again, retry 3 secs*/
		for (i=0; i < 1000 * 1000 * 3; i++) {
			mdm_rts = gpio_get_value(pdata->gpio[MDM_RTS].pinnum);
			mdm_rdy = gpio_get_value(pdata->gpio[MDM_RDY].pinnum);
			if (0 == mdm_rts || 0 == mdm_rdy) {
				udelay (1);
			} else  {
				break;
			}
		}
		if ( mdm_rts == 0 ) {
			ap_rts = gpio_get_value(pdata->gpio[AP_RTS].pinnum);
			ap_rdy = gpio_get_value(pdata->gpio[AP_RDY].pinnum);
			INFO("[%s] mdm_rts=[%d], mdm_rdy=[%d], ap_rts=[%d], ap_rdy=[%d], r=[%d]\n", __func__, mdm_rts, mdm_rdy, ap_rts, ap_rdy, i);
			if ( ap_rdy == 0 ) {
				/* try set ap_rdy to high */
				gpio_set_value(pdata->gpio[AP_RDY].pinnum, 1);
				udelay(10);
			}
			mdm_rts = gpio_get_value(pdata->gpio[MDM_RTS].pinnum);
			mdm_rdy = gpio_get_value(pdata->gpio[MDM_RDY].pinnum);
			ap_rts = gpio_get_value(pdata->gpio[AP_RTS].pinnum);
			ap_rdy = gpio_get_value(pdata->gpio[AP_RDY].pinnum);
			INFO("[%s] mdm_rts=[%d], mdm_rdy=[%d], ap_rts=[%d], ap_rdy=[%d], r=[%d]\n", __func__, mdm_rts, mdm_rdy, ap_rts, ap_rdy, i);
			if ( mdm_rts == 0 ) {
				return -1;
			}
		}
	}

	/*Assert ap_rts*/
	DBG("[%s]set AP_RTS to Low\n", __func__);
	gpio_set_value(pdata->gpio[AP_RTS].pinnum, 0);
	udelay (10);
	/*only for debug tx timeout, should remove after find root cause*/
	DBG("[%s]set AP_RTS to High\n", __func__);
	gpio_set_value(pdata->gpio[AP_RTS].pinnum, 1);

	/*TODO: CACY:Need to check whether BP_RDY is issued?*/
	return 0;
/*
err_ret:
	DBG("[%s] MDM_RDY=%d, MDM_RTS=%d,skip ap_rts assert. ret=[-1]\n",
			__func__, (gpio_get_value(pdata->gpio[MDM_RDY].pinnum)),
			(gpio_get_value(pdata->gpio[MDM_RTS].pinnum)));
	return -1;
err_ret_2:
	DBG("[%s] MDM_RDY=%d, MDM_RTS=%d,skip ap_rts assert. ret=[-2]\n",
			__func__, (gpio_get_value(pdata->gpio[MDM_RDY].pinnum)),
			(gpio_get_value(pdata->gpio[MDM_RTS].pinnum)));
	return -2;
*/
}

static int sc8800g_tty_rx_v1(struct sc8800g_mdm_dev *mdv, unsigned char *buf, ssize_t len)
{
	struct tty_struct *tty = mdv->tty;

	DBG("[%s]: len=%d\n", __func__, len);
#ifdef SC8800_DATA_DEBUG
	hexdump_v1(buf, len);
#endif//SC8800_DATA_DEBUG
	if (!tty) {
		ERROR("[%s]++: tty is NULL.\n", __func__);
		return -1;
	}

	if(tty_buffer_request_room(tty, len) < len) {
		ERROR("[%s]++: tty_buffer_request_room less than needed.\n", __func__);
		return -1;
	} else {
		tty_insert_flip_string(tty, buf, len);
		tty_flip_buffer_push(tty);
	}

	return 0;
}

static packet_header *sc8800g_rx_find_header_v1(void *buf, int len, int *offset)
{
	u8 *phead = (u8 *)buf;
	int find = 0;
	int i = len;
#if 0
	while(i > 0) {
		if((((packet_header *)phead)->tag == HEADER_TAG) &&
			(((packet_header *)phead)->type == HEADER_TYPE)) {
			find = 1;
			*offset = len-i;
			break;
		}
		phead++;
		i--;
	}
#else
	while(i > 0) {
		if( phead[0] == 0x7f && phead[1] == 0x7e && phead[2] == 0x55 ) {
			if ( phead[3] != 0xaa ) {
				ERROR("[%s]:RX Header: [0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x] \n", __func__,
					phead[0], phead[1], phead[2], phead[3],
					phead[4], phead[5], phead[6], phead[7],
					phead[8], phead[9], phead[10], phead[11],
					phead[12], phead[13], phead[14], phead[15]);
			}

			find = 1;
			*offset = len-i;
			break;
		}
		phead++;
		i--;
	}
#endif
	if(find)
		return (packet_header *)phead;
	else
		return NULL;
}

#ifdef ENABLE_SC8800_DATA_CHECKSUM_V1
static inline int sc8800g_rx_checksum_valid_v1(packet_header *phead)
{
	static int error_times = 0;
	uint val_checksum = 0;
	int ret = 0;

	if ( phead->length > ( RECV_BUFFER_SIZE - PACKET_HEADER_SIZE ) ) {
		ERROR("[%s] phead->length(%d) > ( RECV_BUFFER_SIZE - PACKET_HEADER_SIZE )(%d)", __func__, phead->length, ( RECV_BUFFER_SIZE - PACKET_HEADER_SIZE ) );
		return -1;
	}

	val_checksum = sc8800g_checksum_v1( (unsigned char *)phead + PACKET_HEADER_SIZE, phead->length );
	if ( phead->header_valid != val_checksum) {
		ret = -1;
		if ( error_times <= CHECKSUM_ERROR_PRINT_TIMES )
			error_times ++;
		if ( error_times < CHECKSUM_ERROR_PRINT_TIMES ) {
			ERROR("[%s] phead->header_valid=[%d], val_checksum=[%d]", __func__, phead->header_valid, val_checksum);
			error_hexdump_v1((void*)phead, phead->length + PACKET_HEADER_SIZE );
		} else if ( error_times == CHECKSUM_ERROR_PRINT_TIMES ) {
			ERROR("[%s] checksum invalid over 100 times, stop dump data", __func__);
		}
	} else {
		if ( error_times > 0 )
			error_times --;
	}
	return ret;
}
#endif
/* check packet header. */
static inline int sc8800g_rx_header_valid_v1(packet_header *phead)
{
	if ((phead->length > MAX_RX_PACKET_LENGTH)) {
		return -1;
	} else {		/* ok */
		return 0;
	}

}

static int sc8800g_read_v1(struct sc8800g_mdm_dev *mdv, void *buffer, int length)
{
	struct spi_transfer	t;
	struct spi_message m;
	struct spi_device *spi = NULL;

	int offset = 0;
	int temp_size = 0;
	//int len = length;
	int ret = 0;
#ifdef CONFIG_MODEM_PERF_PROFILING
	ktime_t start, diff;
#endif
	DBG("[%s]\n", __func__);

	if ( mdv == NULL ) {
		ERROR("[%s] mdv = NULL\n", __func__);
		return -1;
	}

	spi = mdv->spi;
	if ( spi == NULL ) {
		ERROR("[%s] spi = NULL\n", __func__);
		return -1;
	}

	memset(&t, 0, sizeof(struct spi_transfer));

	while (length) {
	#ifdef CONFIG_MODEM_PERF_PROFILING
		start = ktime_get();
	#endif
		temp_size = (length >= RECV_SIZE_ONE_LOOP) ? RECV_SIZE_ONE_LOOP : length;

		t.rx_buf = buffer + offset;
		t.len = (size_t) temp_size;
		t.bits_per_word = SC8800G_TRANSFER_BITS_PER_WORD;
		t.speed_hz = spi->max_speed_hz;

		mutex_lock(&mdv->data_mutex);
		g_sc8800g_spi_sync_rx_state = 1;
		spi_message_init(&m);
		spi_message_add_tail(&t, &m);
		ret = spi_sync(spi, &m);
		g_sc8800g_spi_sync_rx_state = 0;
		mutex_unlock(&mdv->data_mutex);

		if (ret) {
			ERROR("%s: read from modem device error!(%d)",
				 __func__, ret);
			break;
		}

		offset += temp_size;
		length -= temp_size;

	#ifdef CONFIG_MODEM_PERF_PROFILING
		diff = ktime_sub(ktime_get(), start);
		mdv->perf.rbytes_drv += temp_size;
		mdv->perf.rtime_spi =	ktime_add(mdv->perf.rtime_spi, diff);
	#endif

	}
	//if(len >= 48) {
		//printk("============read data convert========");
		//sc8800g_convert_data((u8 *)buffer, len);
	//}

	if ( g_enable_spi_byte_counter ) {
		mdv->total_rx_bytes += offset;
		INFO("read %d Bytes\n", mdv->total_rx_bytes);
	}

	DBG("[%s] read %d bytes: \n", __func__, offset);
#ifdef SC8800_DATA_DEBUG
	hexdump_v1(buffer, offset);
#endif//SC8800_DATA_DEBUG
	return ret;
}

static int sc8800g_rx_irq_process_v1(struct sc8800g_mdm_dev *mdv)
{
	struct spi_device *spi = mdv->spi;
	struct serial_sc8800g_platform_data *pdata = spi->dev.platform_data;
	packet_header *phead = NULL;
	int head_offset = 0;
	int bytes_read;
	int bytes_to_read, bytes_to_read_align;
	int ret = -1;
	int wait_count = 0;
#ifdef ENABLE_SC8800_AP_RDY_DELAY_V1
	u64 nsec_diff = 0, nsec_start = 0, nsec_end = 0;
#endif

	DBG("[%s]++\n", __func__);
	if(test_and_clear_bit(FLAG_BP_RTS, &mdv->flags)) {
		ret = sc8800g_atomic_read_timeout_v1(&mdv->tx_pending, 1000);
		if(ret < 0) {
			ERROR("[%s]:tx still pending, 10ms timeout\n", __func__);
			return ret;
		}

		if (!wake_lock_active(&g_sc8800g_wakelock_rx_v1)) {
			wake_lock(&g_sc8800g_wakelock_rx_v1);
			if ( g_enable_tty_write_log ) {
				INFO("%s: wake_lock [%s] in L0\n",
					__func__, g_sc8800g_wakelock_rx_v1.name);
			}
		} else {
			ERROR("%s: wake_lock [%s] already lock\n",
				__func__, g_sc8800g_wakelock_rx_v1.name);
		}

		//wait resume complete
		if ( mdv->suspended != 0 ) {
			INFO("%s: mdv->suspended\n", __func__);
			for ( wait_count = 0; wait_count < 2 * 1000 * 100; wait_count++ ) {
				if ( mdv->suspended == 0 ) {
					break;
				}
				udelay(10);
			}
			INFO("%s: mdv->suspended=[%d]  break\n", __func__, mdv->suspended);
		}

		atomic_set(&mdv->rx_pending, 1);
		mdv->transfer_pending = 1;

#ifdef ENABLE_SC8800_AP_RDY_DELAY_V1
		nsec_start = cpu_clock(UINT_MAX);
#endif
		/*de-assert ap_rdy*/
		DBG("[%s]set AP_RDY to Low\n", __func__);
		udelay (100);
		gpio_set_value(pdata->gpio[AP_RDY].pinnum, 0);
		udelay (100);

		ret = sc8800g_read_v1(mdv, mdv->recv.buf, RX_FIRST_PACKET_LENGTH);
		if(ret < 0) {
			ERROR("[%s]:sc8800g read error!\n", __func__);
			goto rx_end;
		}

		phead = sc8800g_rx_find_header_v1(mdv->recv.buf, RX_FIRST_PACKET_LENGTH, &head_offset);
		if(!phead){
			ERROR("[%s]:RX Header NOT found!\n", __func__);
			ERROR("[%s]:RX Header: [0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x] \n", __func__,
				mdv->recv.buf[0], mdv->recv.buf[1], mdv->recv.buf[2], mdv->recv.buf[3],
				mdv->recv.buf[4], mdv->recv.buf[5], mdv->recv.buf[6], mdv->recv.buf[7],
				mdv->recv.buf[8], mdv->recv.buf[9], mdv->recv.buf[10], mdv->recv.buf[11],
				mdv->recv.buf[12], mdv->recv.buf[13], mdv->recv.buf[14], mdv->recv.buf[15]);
			ret = -1;
			norxheader = 1;
			goto rx_end;
		}

		//if(head_offset > 0)
		//	MODEM_INFO("RX: header corrected\n");
		ret = sc8800g_rx_header_valid_v1(phead);
		if(ret < 0) {
			ERROR("[%s]:RX: Header invalid!\n", __func__);
			goto rx_end;
		}

		//MODEM_INFO("packet info: tag = %04x, type = %04x, len = %04x, index = %d\n",
		//	packet->tag, packet->type, packet->length, packet->index);
		bytes_read = RX_FIRST_PACKET_LENGTH - PACKET_HEADER_SIZE - head_offset;
		norxheader = 0;

		if(phead->length > bytes_read) {
			bytes_to_read = phead->length - bytes_read;
			bytes_to_read_align = sc8800g_align_v1(bytes_to_read, PACKET_RX_ALIGNMENT);
			ret = sc8800g_read_v1(mdv, mdv->recv.buf + RX_FIRST_PACKET_LENGTH, bytes_to_read_align);
			if(ret < 0) {
				ERROR("[%s]:Read left data error!\n", __func__);
				goto rx_end;
			}
			DBG("[%s]:RX: left len: %d, align to %d!\n", __func__, bytes_to_read, bytes_to_read_align);
		}
rx_end:

#ifdef ENABLE_SC8800_AP_RDY_DELAY_V1
		nsec_end = cpu_clock(UINT_MAX);
		nsec_diff = nsec_end - nsec_start;

		if ( (nsec_diff / 1000 ) < 1010 && nsec_diff > 0 ) {
			udelay((1010 - ((int)nsec_diff / 1000)));
		}
#endif
		/*assert ap_rdy*/
		DBG("[%s]set AP_RDY to High, MDM_RTS=%d\n", __func__, gpio_get_value(pdata->gpio[MDM_RTS].pinnum));
		gpio_set_value(pdata->gpio[AP_RDY].pinnum, 1);

		atomic_set(&mdv->rx_pending, 0);
		mdv->transfer_pending = 0;

#ifdef ENABLE_SC8800_DATA_CHECKSUM_V1
		if (ret == 0) {
			ret = sc8800g_rx_checksum_valid_v1(phead);
		}
#endif

		if (ret == 0)
			sc8800g_tty_rx_v1 (mdv,(unsigned char *)phead + PACKET_HEADER_SIZE, phead->length);

		if (mdv->suspended)
			wake_up_interruptible(&mdv->continue_suspend);

		#ifdef CONFIG_MODEM_PERF_THREAD
		if(1 == atomic_read(&mdv->wait_rx)) {
			atomic_set(&mdv->wait_rx, 0);
		}
		#endif

		if (wake_lock_active(&g_sc8800g_wakelock_rx_v1)) {
			wake_unlock(&g_sc8800g_wakelock_rx_v1);
			if ( g_enable_tty_write_log ) {
				INFO("%s: wake_unlock [%s]\n",
					__func__, g_sc8800g_wakelock_rx_v1.name);
			}
		} else {
			ERROR("%s: wake_unlock [%s] already unlock\n",
				__func__, g_sc8800g_wakelock_rx_v1.name);
		}

	}
	DBG("[%s]--\n", __func__);
	return ret;
}

static int sc8800g_rx_thread_v1(void *data)
{
	struct sc8800g_mdm_dev *mdv = data;
#ifdef CONFIG_MODEM_PERF_PROFILING
	ktime_t start, diff;
#endif

	do {
		set_current_state(TASK_INTERRUPTIBLE);
		if (!test_bit(FLAG_BP_RTS, &mdv->flags))
			schedule();
		set_current_state(TASK_RUNNING);
	#ifdef CONFIG_MODEM_PERF_PROFILING
		start = ktime_get();
	#endif
		mutex_lock(&mdv->close_mutex);
		if (mdv->mdm_closed != 1) {
			sc8800g_rx_irq_process_v1(mdv);
		} else {
			/*modem has been closed, skip to handle rx process*/
			clear_bit(FLAG_BP_RTS, &mdv->flags);
		}
		mutex_unlock(&mdv->close_mutex);

	#ifdef CONFIG_MODEM_PERF_PROFILING
		diff = ktime_sub(ktime_get(), start);
		mdv->perf.rtime_drv =	ktime_add(mdv->perf.rtime_drv, diff);
	#endif

	} while (!kthread_should_stop());

	return 0;
}

static int sc8800g_wait_for_send_v1(struct sc8800g_mdm_dev *mdv)
{
	long timeout = TX_END_WAIT_TIMEOUT;
	atomic_set(&mdv->tx_done, 0);

	timeout = wait_event_interruptible_timeout(mdv->tx_end_wait,
			(atomic_read(&mdv->tx_done)!=0),TX_END_WAIT_TIMEOUT);
//	timeout = interruptible_sleep_on_timeout(&mdv->tx_end_wait,TX_END_WAIT_TIMEOUT);
	if(!timeout && !atomic_read(&mdv->tx_done))
		return -ETIME;
	else
		return 0;

}
/* local functions */
static int sc8800g_tty_open_v1(struct tty_struct *tty, struct file *filp)
{
	struct sc8800g_mdm_dev *mdv = dev_get_drvdata(tty->dev);
	mutex_lock(&dev_mutex);
	DBG("[%s]\n", __func__);
	mdv->tty = tty;

	mutex_lock(&mdv->close_mutex);
	mdv->mdm_closed = 0;
	mutex_unlock(&mdv->close_mutex);
	mutex_unlock(&dev_mutex);
	return 0;
}

static void sc8800g_tty_close_v1(struct tty_struct *tty, struct file *filp)
{
	struct sc8800g_mdm_dev *mdv = dev_get_drvdata(tty->dev);

	mutex_lock(&dev_mutex);
	DBG("[%s]\n", __func__);

	//cancel_work_sync(&mdv->tx_work);
	cancel_delayed_work_sync(&mdv->tx_work);

	mutex_lock(&mdv->close_mutex);
	mdv->mdm_closed = 1;
	mutex_unlock(&mdv->close_mutex);
	mutex_unlock(&dev_mutex);
}

static int sc8800g_tty_write_v1(struct tty_struct *tty,
			const unsigned char *buf, int count)
{
	int ret = 0;
	int ap_rts_rty_count = 0;

	struct sc8800g_mdm_dev *mdv = NULL;;
	struct spi_device *spi = NULL;
	struct serial_sc8800g_platform_data *pdata = NULL;

	mutex_lock(&dev_mutex);
	/*only for debug tx timeout, should remove after find root cause*/
	if ( g_enable_tty_write_log ) {
		INFO("[%s] mutex_lock~\n", __func__);
	}

	if ( tty == NULL ) {
		ERROR("[%s] tty is NULL!\n", __func__);
		mutex_unlock(&dev_mutex);
		return -1;
	}

	if ( buf == NULL ) {
		ERROR("[%s] buf is NULL!\n", __func__);
		mutex_unlock(&dev_mutex);
		return -1;
	}

	mdv = dev_get_drvdata(tty->dev);

	if ( mdv == NULL ) {
		ERROR("[%s] mdv is NULL!\n", __func__);
		mutex_unlock(&dev_mutex);
		return -1;
	}

	spi = mdv->spi;

	if ( spi == NULL ) {
		ERROR("[%s] spi is NULL!\n", __func__);
		mutex_unlock(&dev_mutex);
		return -1;
	}

	pdata = spi->dev.platform_data;

	if ( pdata == NULL ) {
		ERROR("[%s] pdata is NULL!\n", __func__);
		mutex_unlock(&dev_mutex);
		return -1;
	}

	if ( count > SC8800G_TX_BUFSIZE ) {
		ERROR("[%s] count(%d) > SC8800G_TX_BUFSIZE(%d)\n",	__func__, count, SC8800G_TX_BUFSIZE);
		mutex_unlock(&dev_mutex);
		return -1;
	}

#ifdef SC8800_DATA_DEBUG
	hexdump_v1(buf, count);
#endif//SC8800_DATA_DEBUG

	if(mdv->suspended) {
		ERROR("[%s] sc8800g is suspending!\n", __func__);
		mutex_unlock(&dev_mutex);
		return -1;
	}

	if(!atomic_read(&mdv->modem_live)) {
		ERROR("[%s] sc8800g is not alive!\n", __func__);
		mutex_unlock(&dev_mutex);
		return -1;
	}

	if ( 1 == atomic_read(&mdv->crash_state) ) {
		INFO("[%s] mdm has crashed, [%d][%d][%d][%d][%d][%d]\n", __func__,
				gpio_get_value( pdata->gpio[MDM_RTS].pinnum ),
				gpio_get_value( pdata->gpio[MDM_RDY].pinnum ),
				gpio_get_value( pdata->gpio[AP_RTS].pinnum ),
				gpio_get_value( pdata->gpio[AP_RTS].pinnum ),
				gpio_get_value( pdata->gpio[MDM_TO_AP1].pinnum ),
				gpio_get_value( pdata->gpio[MDM_ALIVE].pinnum )
			);
		INFO("[%s]atomic: modem_live=[%d], tx_done=[%d], tx_pending=[%d], rx_pending=[%d]", __func__,
			atomic_read(&mdv->modem_live),
			atomic_read(&mdv->tx_done),
			atomic_read(&mdv->tx_pending),
			atomic_read(&mdv->rx_pending));
		mutex_unlock(&dev_mutex);
		return -1;
	}

	if(1 == atomic_read(&mdv->tx_pending)) {
		ERROR("[%s] sc8800g tx buffer pending!,[%d,%d]\n", __func__, g_sc8800g_spi_sync_tx_state, g_sc8800g_spi_sync_rx_state);
		mutex_unlock(&dev_mutex);
#ifdef ENABLE_SC8800_962049_DEBUG
		g_tx_pending_count++;
		if ( ( g_tx_pending_count % 5 ) == 0) {
			spi_tegra_962049_debug_get_info();
		}
		if (g_tx_pending_count > 50 ) {
			BUG();
		}
#endif
		return -1;
	}
#ifdef ENABLE_SC8800_962049_DEBUG
	else {
		if ( g_tx_pending_count > 0 ) {
			g_tx_pending_count--;
		}
	}
#endif

	if (!wake_lock_active(&g_sc8800g_wakelock_tx_v1)) {
		wake_lock(&g_sc8800g_wakelock_tx_v1);
		if ( g_enable_tty_write_log ) {
			INFO("%s: wake_lock [%s] in L0\n",
				__func__, g_sc8800g_wakelock_tx_v1.name);
		}
	} else {
		ERROR("%s: wake_lock [%s] already lock\n",
			__func__, g_sc8800g_wakelock_tx_v1.name);
	}

	sc8800g_tx_data_set_v1(mdv, buf, count);

	ap_rts_rty_count = 0;

	/*Assert ap_rts*/
	ret = sc8800g_ap_rts_request_v1(pdata);
	if(ret < 0){
		ERROR("[%s] sc8800g_ap_rts_request_v1 failed, ret=%d\n",	__func__, ret);
		goto out;
	}
	ret = sc8800g_wait_for_send_v1(mdv);
	if(ret < 0) {
		ERROR("[%s] sc8800g tx wait finish timeout\n", __func__);
#ifdef ENABLE_SC8800_962049_DEBUG
		spi_tegra_962049_debug_set(1);
		spi_tegra_962049_debug_get_info();
		sc8800g_print_info(mdv);
		sc8800g_gpio_print2_v1(pdata);
		INFO("[%s] last data size [%d]\n", __func__, g_mdm_temp_data_buf_size);
		error_hexdump2_v1((void*)&g_mdm_temp_data_buf, g_mdm_temp_data_buf_size);
#endif
		/*print log 5 times*/
		if ( g_enable_tty_write_log <= 100 )
			g_enable_tty_write_log += 5;
		INFO("[%s] enable tx log for [%d] times\n", __func__, g_enable_tty_write_log);
	} else {
		if ( g_enable_tty_write_log > 0 ) {
			g_enable_tty_write_log--;
			if( g_enable_tty_write_log == 0 ) {
#ifdef ENABLE_SC8800_962049_DEBUG
				spi_tegra_962049_debug_set(0);
				spi_tegra_962049_debug_get_info();
#endif
				INFO("[%s] disable tx log\n", __func__);
			}
		}
	}
out:
	if(ret < 0) {
		/*only for debug tx timeout, should remove after find root cause*/
		if ( g_enable_tty_write_log ) {
			INFO("[%s]set AP_RTS to Low\n", __func__);
		} else {
			DBG("[%s]set AP_RTS to Low\n", __func__);
		}
		gpio_set_value(pdata->gpio[AP_RTS].pinnum, 0);
		sc8800g_tx_data_clear_v1(mdv);

		if (wake_lock_active(&g_sc8800g_wakelock_tx_v1)) {
			wake_unlock(&g_sc8800g_wakelock_tx_v1);
			if ( g_enable_tty_write_log ) {
				INFO("%s: wake_unlock [%s]\n",
					__func__, g_sc8800g_wakelock_tx_v1.name);
			}
		} else {
			ERROR("%s: wake_unlock [%s] already unlock\n",
				__func__, g_sc8800g_wakelock_tx_v1.name);
		}

	}
	//sc8800g_tx_data_clear_v1(mdv);

	if (mdv->suspended)
			wake_up_interruptible(&mdv->continue_suspend);

	mutex_unlock(&dev_mutex);

	/*only for debug tx timeout, should remove after find root cause*/
	if ( g_enable_tty_write_log ) {
		INFO("[%s] mutex_unlock~\n", __func__);
	}
	return count;
}

static int sc8800g_tty_write_room_v1(struct tty_struct *tty)
{
	struct sc8800g_mdm_dev *mdv = dev_get_drvdata(tty->dev);

	int room = -ENODEV;

	if (!mdv)
		goto exit;

	if(!(mdv->send.alloc)) {
		goto exit;
		}

	room = SEND_BUFFER_SIZE;
exit:
	DBG("[%s]: write_room =%d\n", __func__, room);
	return room;
}

static void sc8800g_tty_flush_buffer_v1(struct tty_struct *tty)
{
	DBG("[%s]\n", __func__);
}

static int sc8800g_tty_chars_in_buffer_v1(struct tty_struct *tty)
{
	return 0;
}

static void sc8800g_tty_throttle_v1(struct tty_struct *tty)
{
	DBG("[%s]\n", __func__);
}

static void sc8800g_tty_unthrottle_v1(struct tty_struct *tty)
{
	DBG("[%s]\n", __func__);
}

static int sc8800g_tty_ioctl_v1(struct tty_struct *tty, unsigned int cmd, unsigned long arg)
{
	DBG("[%s] cmd=0x%X\n", __func__, cmd);
	return -ENOIOCTLCMD;
}

const struct tty_operations sc8800g_tty_ops_v1 = {
	.open = sc8800g_tty_open_v1,
	.close = sc8800g_tty_close_v1,
	.write = sc8800g_tty_write_v1,
	.write_room = sc8800g_tty_write_room_v1,
	.flush_buffer = sc8800g_tty_flush_buffer_v1,
	.chars_in_buffer = sc8800g_tty_chars_in_buffer_v1,
	.throttle = sc8800g_tty_throttle_v1,
	.unthrottle = sc8800g_tty_unthrottle_v1,
	.ioctl = sc8800g_tty_ioctl_v1,
};

static irqreturn_t sc8800g_irq_sim_door_v1(int irq, void *data)
{
	struct sc8800g_mdm_dev *mdv = NULL;
	VDBG("[%s]\n", __func__);

	if ( data == NULL ) {
		ERROR("[%s] data=NULL", __func__);
		goto out;
	}
	mdv = data;
	schedule_work(&mdv->sim_door_notify_work);
out:
	return IRQ_HANDLED;
}

static irqreturn_t sc8800g_irq_crash_v1(int irq, void *data)
{
	struct sc8800g_mdm_dev *mdv = NULL;
	INFO("[%s]\n", __func__);

	if ( data == NULL ) {
		ERROR("[%s] data=NULL", __func__);
		goto out;
	}
	mdv = data;
	schedule_work(&mdv->crash_notify_work);
out:
	return IRQ_HANDLED;
}

static irqreturn_t sc8800g_irq_rdy_v1(int irq, void *data)
{
	struct sc8800g_mdm_dev *mdv = data;
	unsigned long flags;

	/*only for debug tx timeout, should remove after find root cause*/
	if ( g_enable_tty_write_log ) {
		INFO("[%s]\n", __func__);
	} else {
		VDBG("[%s]\n", __func__);
	}

	if(mdv->suspended) {
		INFO("[%s] sc8800g is suspending!\n", __func__);
		goto out;
	}

	if(!atomic_read(&mdv->modem_live)) {
		INFO("[%s] sc8800g is not alive!\n", __func__);
		goto out;
	}

	spin_lock_irqsave(&mdv->tx_lock, flags);
	if(!mdv->tx.addr || (0 == mdv->tx.len)) {
		spin_unlock_irqrestore(&mdv->tx_lock, flags);
		VDBG("[%s] sc8800g tx buffer is not ready.\n", __func__);
		goto out;
	}
	spin_unlock_irqrestore(&mdv->tx_lock, flags);

	set_bit(FLAG_BP_RDY, &mdv->flags);
	if ( g_enable_tty_write_log ) {
		INFO("[%s] queue_work\n", __func__);
	}
	//queue_work(mdv->tx_workqueue, &mdv->tx_work);
	queue_delayed_work(mdv->tx_workqueue, &mdv->tx_work, usecs_to_jiffies(0));

out:
	return IRQ_HANDLED;
}

static irqreturn_t sc8800g_irq_mdm_to_ap1_v1(int irq, void *data)
{
	struct sc8800g_mdm_dev *mdv = NULL;
	struct serial_sc8800g_platform_data *pdata = NULL;
	int val_mdm_to_ap1 = 0;

	if ( data == NULL ) {
		ERROR("[%s] data=NULL", __func__);
		goto out;
	}

	mdv = data;
	pdata = mdv->pdata;
	if ( pdata == NULL ) {
		ERROR("[%s] pdata=NULL", __func__);
		goto out;
	}
	if ( gpio_get_value(pdata->gpio[MDM_TO_AP1].pinnum) == 1 ) {
		pdata->mdm_to_ap1_gpio_value = 1;
		INFO("[%s] MDM_TO_AP1=[1]", __func__);
	} else {
		pdata->mdm_to_ap1_gpio_value = 0;
		schedule_work(&mdv->mdm_to_ap1_work);
	}
out:
	return IRQ_HANDLED;
}

static irqreturn_t sc8800g_irq_rts_v1(int irq, void *data)
{
	struct sc8800g_mdm_dev *mdv = data;
	struct spi_device *spi = mdv->spi;
	struct serial_sc8800g_platform_data *pdata = spi->dev.platform_data;

	VDBG("[%s]\n", __func__);

	if(!atomic_read(&mdv->modem_live)) {
		INFO("[%s] sc8800g is not alive!\n", __func__);
		goto out;
	}

	if(1 == gpio_get_value(pdata->gpio[MDM_RTS].pinnum)) {
		ERROR("[%s] mdm_rts is high\n", __func__);
		goto out;
	}

	if(!test_bit(FLAG_BP_RTS, &mdv->flags)) {
		set_bit(FLAG_BP_RTS, &mdv->flags);
	}
	wake_up_process(mdv->rx_thread);

out:
	return IRQ_HANDLED;
}

static int sc8800g_irq_onoff_v1(struct serial_sc8800g_platform_data *pdata, MODEM_IRQ_STATE onoff, struct sc8800g_mdm_dev *mdv) {
	int ret = 0;
	struct sc8800g_gpio
		*gpio_mdm_rts = NULL,
		*gpio_mdm_rdy = NULL,
		*gpio_mdm_alive = NULL,
		*gpio_mdm_to_ap1 = NULL,
		*gpio_sim_door = NULL;

	if ( pdata == NULL ) {
		ERROR("[%s] pdata = NULL\n",  __func__);
		ret = -1;
		goto end;
	}
	if ( mdv == NULL ) {
		ERROR("[%s] mdv = NULL\n",  __func__);
		ret = -1;
		goto end;
	}
	gpio_mdm_rts = &pdata->gpio[MDM_RTS];
	gpio_mdm_rdy = &pdata->gpio[MDM_RDY];
	gpio_mdm_alive = &pdata->gpio[MDM_ALIVE];
	gpio_mdm_to_ap1 = &pdata->gpio[MDM_TO_AP1];
	gpio_sim_door = &pdata->gpio[SIM_DOOR];

	if ( onoff == MDM_IRQ_REQ) {

		mdv->rts_irq = gpio_to_irq(gpio_mdm_rts->pinnum);
		INFO("[%s] mdv->rts_irq=[%d].\n",  __func__, mdv->rts_irq);
		mdv->rdy_irq = gpio_to_irq(gpio_mdm_rdy->pinnum);
		INFO("[%s] mdv->rdy_irq=[%d].\n",  __func__, mdv->rdy_irq);
		mdv->crash_irq = gpio_to_irq(gpio_mdm_alive->pinnum);
		INFO("[%s] mdv->crash_irq=[%d].\n",  __func__, mdv->crash_irq);
		mdv->mdm_to_ap1_irq = gpio_to_irq(gpio_mdm_to_ap1->pinnum);
		INFO("[%s] mdv->mdm_to_ap1_irq=[%d].\n",  __func__, mdv->mdm_to_ap1_irq);
		mdv->sim_door_irq = gpio_to_irq(gpio_sim_door->pinnum);
		INFO("[%s] mdv->sim_door_irq=[%d].\n",  __func__, mdv->sim_door_irq);

		ret = request_irq(mdv->rts_irq, sc8800g_irq_rts_v1, IRQF_TRIGGER_FALLING, "MODEM RTS V1", mdv);
		if (ret) {
			ERROR("[%s] MODEM request rts irq failed.\n",  __func__);
		}
		gpio_mdm_rts->irq_enabled = true;

		ret = request_irq(mdv->rdy_irq, sc8800g_irq_rdy_v1, IRQF_TRIGGER_FALLING, "MODEM RDY V1", mdv);
		if (ret) {
			ERROR("[%s] MODEM request rdy irq failed.\n",  __func__);
		}
		gpio_mdm_rdy->irq_enabled = true;

		ret = request_irq(mdv->crash_irq, sc8800g_irq_crash_v1, IRQF_TRIGGER_FALLING, "MDM CRASH V1", mdv);
		if (ret) {
			ERROR("[%s] MODEM request mdm crash irq failed.\n",  __func__);
		}
		gpio_mdm_alive->irq_enabled = true;

		ret = request_irq(mdv->mdm_to_ap1_irq, sc8800g_irq_mdm_to_ap1_v1, IRQF_TRIGGER_FALLING, "MDM TO AP 1 V1", mdv);
		if (ret) {
			ERROR("[%s] MODEM request mdm to ap 1 irq failed.\n",  __func__);
		}
		gpio_mdm_to_ap1->irq_enabled = true;

		ret = request_irq(mdv->sim_door_irq, sc8800g_irq_sim_door_v1, IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING, "SIM DOOR V1", mdv);
		if (ret) {
			ERROR("[%s] MODEM request sim_door irq failed.\n",  __func__);
		}
		gpio_sim_door->irq_enabled = true;
	} else if ( onoff == MDM_IRQ_ENABLE) {
		if (gpio_mdm_rts->irq_enabled == false) {
			INFO("[%s] enable_irq mdm_rts_gpio[%d].\n",  __func__, gpio_to_irq(pdata->mdm_rts_gpio));
			enable_irq(gpio_to_irq(pdata->mdm_rts_gpio));
			gpio_mdm_rts->irq_enabled = true;
		}
		if (gpio_mdm_rdy->irq_enabled == false) {
			INFO("[%s] enable_irq mdm_rdy_gpio[%d].\n",  __func__, gpio_to_irq(pdata->mdm_rdy_gpio));
			enable_irq(gpio_to_irq(pdata->mdm_rdy_gpio));
			gpio_mdm_rdy->irq_enabled = true;
		}
		if (gpio_mdm_alive->irq_enabled == false) {
			INFO("[%s] enable_irq mdm_alive_gpio[%d].\n",  __func__, gpio_to_irq(pdata->mdm_alive_gpio));
			enable_irq(gpio_to_irq(pdata->mdm_alive_gpio));
			gpio_mdm_alive->irq_enabled = true;
		}
		if (gpio_mdm_to_ap1->irq_enabled == false) {
			INFO("[%s] enable_irq mdm_to_ap1_gpio[%d].\n",  __func__, gpio_to_irq(pdata->mdm_to_ap1_gpio));
			enable_irq(gpio_to_irq(pdata->mdm_to_ap1_gpio));
			gpio_mdm_to_ap1->irq_enabled = true;
		}
	} else if ( onoff == MDM_IRQ_DISABLE) {
		if (gpio_mdm_rts->irq_enabled == true) {
			INFO("[%s] disable_irq mdm_rts_gpio[%d].\n",  __func__, gpio_to_irq(pdata->mdm_rts_gpio));
			disable_irq(gpio_to_irq(pdata->mdm_rts_gpio));
			gpio_mdm_rts->irq_enabled = false;
		}
		if (gpio_mdm_rdy->irq_enabled == true) {
			INFO("[%s] disable_irq mdm_rdy_gpio[%d].\n",  __func__, gpio_to_irq(pdata->mdm_rdy_gpio));
			disable_irq(gpio_to_irq(pdata->mdm_rdy_gpio));
			gpio_mdm_rdy->irq_enabled = false;
		}
		if (gpio_mdm_alive->irq_enabled == true) {
			INFO("[%s] disable_irq mdm_alive_gpio[%d].\n",  __func__, gpio_to_irq(pdata->mdm_alive_gpio));
			disable_irq(gpio_to_irq(pdata->mdm_alive_gpio));
			gpio_mdm_alive->irq_enabled = false;
		}
		if (gpio_mdm_to_ap1->irq_enabled == true) {
			INFO("[%s] disable_irq mdm_to_ap1_gpio[%d].\n",  __func__, gpio_to_irq(pdata->mdm_to_ap1_gpio));
			disable_irq(gpio_to_irq(pdata->mdm_to_ap1_gpio));
			gpio_mdm_to_ap1->irq_enabled = false;
		}
	} else if ( onoff == MDM_IRQ_WAKE_ENABLE) {
		if ( gpio_get_value(mdv->pdata->mdm_alive_gpio) == 1 ) {
			INFO("[%s]: enable wake-up.\n", __func__);
			if (gpio_mdm_rts->irq_enabled == true) {
				enable_irq_wake(mdv->rts_irq);
			}
			if (gpio_mdm_rdy->irq_enabled == true) {
				enable_irq_wake(mdv->rdy_irq);
			}
			if (gpio_mdm_alive->irq_enabled == true) {
				enable_irq_wake(mdv->crash_irq);
			}
			/* remove MDM_TO_AP1 irq wakw setting because it is not a wakeup pin */
#if 0
			if (gpio_mdm_to_ap1->irq_enabled == true) {
				INFO("[%s]: enable mdm_to_ap1_irq[%d] wake-up.\n", __func__, mdv->mdm_to_ap1_irq);
				enable_irq_wake(mdv->mdm_to_ap1_irq);
			}
#endif
			if (gpio_mdm_to_ap1->irq_enabled == true) {
				enable_irq_wake(mdv->sim_door_irq);
			}
		}

	} else if ( onoff == MDM_IRQ_WAKE_DISABLE) {
		if ( gpio_get_value(mdv->pdata->mdm_alive_gpio) == 1 ) {
			INFO("[%s]: disable wake-up.\n", __func__);
			if (gpio_mdm_rts->irq_enabled == true) {
				disable_irq_wake(mdv->rts_irq);
			}
			if (gpio_mdm_rdy->irq_enabled == true) {
				disable_irq_wake(mdv->rdy_irq);
			}
			if (gpio_mdm_alive->irq_enabled == true) {
				disable_irq_wake(mdv->crash_irq);
			}
			/* remove MDM_TO_AP1 irq wakw setting because it is not a wakeup pin */
#if 0
			if (gpio_mdm_to_ap1->irq_enabled == true) {
				INFO("[%s]: disable mdm_to_ap1_irq[%d] wake-up.\n", __func__, mdv->mdm_to_ap1_irq);
				disable_irq_wake(mdv->mdm_to_ap1_irq);
			}
#endif
			if (gpio_mdm_to_ap1->irq_enabled == true) {
				disable_irq_wake(mdv->sim_door_irq);
			}
		}
	}
end:
	return ret;
}

static int sc8800g_mdm_dev_init_v1(struct spi_device *spi, struct sc8800g_mdm_dev *mdv)
{
	struct serial_sc8800g_platform_data *pdata;
	int ret;

	pdata = (struct serial_sc8800g_platform_data *)spi->dev.platform_data;

	/*
	* Start putting together one of our big modem device structures.
	*/
	mdv->spi = spi;
	mdv->pdata = pdata;

	mdv->send.buf= kzalloc(SEND_BUFFER_SIZE, GFP_KERNEL);
	if(!mdv->send.buf){
		ERROR("[%s] Modem send buffer alloc fail!\n", __func__);
		goto out_free;
	}
	mdv->send.alloc = 1;

	mdv->recv.buf = kzalloc(RECV_BUFFER_SIZE, GFP_KERNEL);
	if(!mdv->recv.buf){
		ERROR("[%s] Modem recv buffer alloc fail!\n", __func__);
		goto out_free_buf;
	}
	mdv->recv.alloc = 1;


	mdv->flags = 0;
	mdv->pwr_sts = PWR_RESETING;

	atomic_set(&mdv->modem_live, 0);
	atomic_set(&mdv->tx_done, 0);
	atomic_set(&mdv->tx_pending, 0);
	atomic_set(&mdv->rx_pending, 0);
	mdv->transfer_pending = 0;
	mdv->suspended = 0;

	spin_lock_init(&mdv->tx_lock);
	spin_lock_init(&mdv->usr_lock);
//	spin_lock_init(&mdv->pm_lock);

	mutex_init(&mdv->power_mutex);

	mutex_init(&mdv->close_mutex);

	mutex_init(&mdv->data_mutex);

//	INIT_LIST_HEAD(&mdv->dev_list); /*TODO: CACY: check whether this could be removed later*/

	init_waitqueue_head(&mdv->tx_end_wait);
	init_waitqueue_head(&mdv->continue_suspend);
	//wake_lock_init(&sc8800g_wakeup_wake_lock, WAKE_LOCK_SUSPEND, "sc8800g_wake_lock");

    /** Create workqueue */
	mdv->tx_workqueue = create_freezable_workqueue("kmodemtxd_v1");
	if (!mdv->tx_workqueue)
		goto out_free_buf;

//	INIT_WORK(&mdv->tx_work, sc8800g_tx_work_v1);
	INIT_DELAYED_WORK(&mdv->tx_work, sc8800g_tx_work_v1);

	wake_lock_init(&g_sc8800g_wakelock_tx_v1, WAKE_LOCK_SUSPEND, "sc8800g_tx");
	wake_lock_init(&g_sc8800g_wakelock_rx_v1, WAKE_LOCK_SUSPEND, "sc8800g_rx");

	mdv->rx_thread = kthread_run(sc8800g_rx_thread_v1, mdv, "kmodemrxd_v1");
	if (IS_ERR(mdv->rx_thread)) {
		ret = PTR_ERR(mdv->rx_thread);
		goto out_free_buf;
	}
//	dev_set_drvdata(&spi->dev, mdv);

//	atomic_set(&mdv->modem_live, 1);
	mdv->pwr_sts= PWR_ON;

	return 0;
out_free_buf:
	if(mdv->recv.alloc) {
		kzfree(mdv->recv.buf);
		mdv->recv.alloc = 0;
	}
	if(mdv->send.alloc) {
		kzfree(mdv->send.buf);
		mdv->send.alloc = 0;
	}
out_free:
	kzfree(mdv);
	return -1;
}

#ifdef SC8800G_POWER_ONOFF
static int sc8800g_power_onoff_v1(struct serial_sc8800g_platform_data *pdata, MODEM_PWR_STATE onoff)
{
	struct sc8800g_gpio* g_mdm_pwr = NULL;
	struct sc8800g_gpio* g_mdm_state = NULL;
	struct sc8800g_gpio* g_mdm_to_ap1 = NULL;
	struct timeval tv = {NULL}, tv_start = {NULL};

	int ret = 0;
	int val_alive = 0;
	int val_power = 0;
	int val_mdm2ap1 = 0;
	int i = 0;
	if ( pdata == NULL ) {
		VDBG("[%s] pdata == NULL\n", __func__);
		ret = -1;
		goto end;
	}

	/* check modem power state */
	g_mdm_pwr = &pdata->gpio[MDM_POWER];
	g_mdm_state = &pdata->gpio[MDM_ALIVE];
	g_mdm_to_ap1 = &pdata->gpio[MDM_TO_AP1];

	/* get power status value*/
	val_power = gpio_get_value(g_mdm_pwr->pinnum);
	val_alive = gpio_get_value(g_mdm_state->pinnum);

	sc8800g_gpio_print_ext_v1(pdata, MDM_POWER);
	sc8800g_gpio_print_ext_v1(pdata, MDM_TO_AP1);
	sc8800g_gpio_print_ext_v1(pdata, MDM_ALIVE);

	if ( onoff == PWR_ON ) {
		/* power on modem */
		INFO("[%s] power on modem+\n", __func__);

		if (val_power == 0) {
			int alive_hi_count = 0;
			int val_alive_new = 0;
			int val_mdm_to_ap1_new = 0;
			bool is_turn_on_gpio = false;
			mutex_lock(&sc8800g_mdv_v1->power_mutex);
			VDBG("[%s] sc8800g_gpio_onoff_v1+\n", __func__);
			ret = sc8800g_gpio_onoff_v1 (pdata, MDM_GPIO_OFF, true);
			VDBG("[%s] sc8800g_gpio_onoff_v1-, r=[%d]\n", __func__, ret);

			val_alive = gpio_get_value(g_mdm_state->pinnum);
			if (val_alive == 1) {
				ERROR("[%s] val_alive should lo\n", __func__);
				val_alive = 0;
			}
			sc8800g_gpio_print_ext_v1(pdata, MDM_ALIVE);

			sc8800g_gpio_onoff_ext_v1(pdata, AP_RDY, MDM_GPIO_ON);
			/* for offline log, need enable AP_TO_MDM2 before power on modem */
			sc8800g_gpio_onoff_ext_v1(pdata, AP_TO_MDM2, MDM_GPIO_ON);
			/* pull nboot to high */
			sc8800g_gpio_onoff_ext_v1(pdata, MDM_NBOOT, MDM_GPIO_ON);

			sc8800g_gpio_onoff_ext_v1(pdata, MDM_ALIVE, MDM_GPIO_ON);

			sc8800g_gpio_onoff_ext_v1(pdata, MDM_TO_AP1, MDM_GPIO_ON);

			/* set AP side gpio output pins to default setting*/
			//gpio_set_value(pdata->gpio[AP_RDY].pinnum, 1);
			gpio_set_value(pdata->gpio[AP_RTS].pinnum, 0);
			gpio_set_value(pdata->gpio[AP_RESEND].pinnum, 0);

			/* keep the time for MDM_POWER_ON */
			do_gettimeofday(&tv_start);

			/* set MDM_POWER to high */
			gpio_set_value(g_mdm_pwr->pinnum, 1);
			mdelay(50);
			sc8800g_gpio_print_ext_v1(pdata, MDM_POWER);
			sc8800g_gpio_print_ext_v1(pdata, MDM_ALIVE);
			sc8800g_gpio_print_ext_v1(pdata, MDM_TO_AP1);

			/* Wait MDM_ALIVE high for 10 seconds */
			for ( i = 0; i < 100; i++ ) {
				/* print message every 500 ms*/
				if ( i % 10 == 0 ) {
					sc8800g_gpio_print_ext_v1(pdata, MDM_POWER);
					sc8800g_gpio_print_ext_v1(pdata, MDM_ALIVE);
					sc8800g_gpio_print_ext_v1(pdata, MDM_TO_AP1);
				}
				/* get MDM_ALIVE value*/
				val_alive_new = gpio_get_value(g_mdm_state->pinnum);
				val_mdm2ap1 = gpio_get_value(g_mdm_to_ap1->pinnum);
				VDBG("[%s] PWR_STATE: [%s]=[%d]\n", __func__, g_mdm_state->name, val_alive_new);
				if (val_alive_new != val_alive) {
					INFO("[%s] val_alive_new != val_alive: val_alive_new=[%d], val_mdm2ap1=[%d]\n", __func__, val_alive_new, val_mdm2ap1);
					if (val_alive_new == 1 && val_mdm2ap1 == 0) {
						// Check mdm_to_ap1 to make sure modem is turn on.
						INFO("[%s] val_alive_new=[%d], val_mdm2ap1=[%d]\n", __func__, val_alive_new, val_mdm2ap1);
						val_alive_new = 0;
					}
					if (val_alive_new == 1) {
						alive_hi_count++;
						if ( alive_hi_count == 1 ) {
							INFO("[%s] sc8800g_gpio_onoff_v1+\n", __func__);
							ret = sc8800g_gpio_onoff_v1 (pdata, MDM_GPIO_ON, false);
							INFO("[%s] sc8800g_gpio_onoff_v1-, r=[%d]\n", __func__, ret);
							is_turn_on_gpio = true;
						}
						/* don't break and we wll check ap_alive after mdm_power_on 5 secs*/
						/*
						if (alive_hi_count >= 2 ) {
							break;
						}
						*/
					}
					val_alive = val_alive_new;
				}

				/* check ap_alive after power_on 5 secs */
				do_gettimeofday(&tv);
				if ( (tv.tv_sec - tv_start.tv_sec) >= 5 ) {
					if ( val_alive_new == 1 && alive_hi_count > 0) {
						INFO("[%s] get ap_alive 1, alive_hi_count=[%d]\n", __func__, alive_hi_count);
						break;
					}
				}
				msleep_interruptible(100);
			}

			if ( is_turn_on_gpio == false ) {
				/* if we can't get MDM_ALIVE hi, we still need to enable all GPIO at final*/
				INFO("[%s] sc8800g_gpio_onoff_v1+\n", __func__);
				ret = sc8800g_gpio_onoff_v1 (pdata, MDM_GPIO_ON, false);
				INFO("[%s] sc8800g_gpio_onoff_v1-, r=[%d]\n", __func__, ret);
				is_turn_on_gpio = true;
			}

			atomic_set(&sc8800g_mdv_v1->crash_state, 0);

			sc8800g_irq_onoff_v1( pdata, MDM_IRQ_ENABLE,  sc8800g_mdv_v1);
			atomic_set(&sc8800g_mdv_v1->modem_live, 1);
			pdata->mdm_to_ap1_gpio_value = gpio_get_value(g_mdm_to_ap1->pinnum);
			mutex_unlock(&sc8800g_mdv_v1->power_mutex);
		} else {
			INFO("[%s] MDM_POWER already high\n", __func__);
		}
		sc8800g_gpio_print_ext_v1(pdata, MDM_POWER);
		sc8800g_gpio_print_ext_v1(pdata, MDM_TO_AP1);
		sc8800g_gpio_print_ext_v1(pdata, MDM_ALIVE);
		INFO("[%s] power on modem-\n", __func__);
	} else if ( onoff == PWR_OFF ){
		/* power off modem */
		INFO("[%s] power off modem+\n", __func__);

		if (val_power == 1) {
			mutex_lock(&sc8800g_mdv_v1->power_mutex);
			INFO("[%s] set GPIOs\n", __func__);

			/* set ap side gpio output pins to low*/
			gpio_set_value(pdata->gpio[AP_RDY].pinnum, 0);
			gpio_set_value(pdata->gpio[AP_RTS].pinnum, 0);
			gpio_set_value(pdata->gpio[AP_RESEND].pinnum, 0);
			sc8800g_irq_onoff_v1( pdata, MDM_IRQ_DISABLE,  sc8800g_mdv_v1);

			/*set GPIO to input*/
			INFO("[%s] set GPIO to input\n", __func__);
			sc8800g_gpio_onoff_v1(pdata, MDM_GPIO_INPUT, false);

			INFO("[%s] PWR_STATE: [%s]=[%d]\n", __func__, g_mdm_to_ap1->name, val_alive);

			//disable all gpio and set to output low
			VDBG("[%s] sc8800g_gpio_onoff_v1+\n", __func__);
			ret = sc8800g_gpio_onoff_v1 (pdata, MDM_GPIO_OFF, false);
			VDBG("[%s] sc8800g_gpio_onoff_v1-, r=[%d]\n", __func__, ret);
			gpio_set_value(pdata->gpio[MDM_POWER].pinnum, 0);
			atomic_set(&sc8800g_mdv_v1->modem_live, 0);
			mutex_unlock(&sc8800g_mdv_v1->power_mutex);
		} else {
			VDBG("[%s] MDM_POWER already low\n", __func__);
		}
		sc8800g_gpio_print_ext_v1(pdata, MDM_POWER);
		sc8800g_gpio_print_ext_v1(pdata, MDM_TO_AP1);
		sc8800g_gpio_print_ext_v1(pdata, MDM_ALIVE);
		INFO("[%s] power off modem-\n", __func__);
	} else {
		VDBG("[%s] unsupported state=[%d]\n", __func__, onoff);
	}
end:
	return ret;
}

static ssize_t sc8800g_onoff_show_v1(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct serial_sc8800g_platform_data *pdata = dev->platform_data;
	int power_onoff = 0;

	VDBG("[%s]\n", __func__);
	/* check for platform data */
	if (!pdata)
		return -EINVAL;

	power_onoff = pdata->mdm_to_ap1_gpio_value;
	VDBG("[%s] mdm_to_ap1=[%d]\n", __func__, power_onoff);

	return sprintf(buf, "%d\n", power_onoff);
}

static ssize_t sc8800g_onoff_store_v1(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t count)
{
	struct serial_sc8800g_platform_data *pdata = dev->platform_data;
	int power_onoff;

	VDBG("[%s]\n", __func__);

	if (!pdata)
		return -EINVAL;

	if (sscanf(buf, "%d", &power_onoff) != 1)
		return -EINVAL;

	if (power_onoff == 0)
		sc8800g_power_onoff_v1(pdata, PWR_OFF);
	else if (power_onoff == 1)
		sc8800g_power_onoff_v1(pdata, PWR_ON);
	else if ( power_onoff == 2 ) {
		sc8800g_gpio_onoff_v1(pdata, MDM_GPIO_OFF, 1);
	} else if ( power_onoff == 3 ) {
		sc8800g_gpio_onoff_v1(pdata, MDM_GPIO_ON, 1);
	} else
		return -EINVAL;

	return count;

}
static DEVICE_ATTR(onoff, S_IRUSR | S_IWUSR | S_IRGRP,
	sc8800g_onoff_show_v1, sc8800g_onoff_store_v1);

static int sc8800g_create_sysfs_attrs_v1(struct device *dev)
{
	dev_vdbg(dev, "[%s]", __func__);

	return device_create_file(dev, &dev_attr_onoff);
}

static int sc8800g_remove_sysfs_attrs_v1(struct device *dev)
{
	dev_vdbg(dev, "[%s]", __func__);

	device_remove_file(dev, &dev_attr_onoff);
	return 0;
}

static ssize_t sc8800g_mdm_alive_show_v1(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct serial_sc8800g_platform_data *pdata = dev->platform_data;
	int mdm_alive = 0;

	INFO("[%s]\n", __func__);
	/* check for platform data */
	if (!pdata)
		return -EINVAL;

	mdm_alive = gpio_get_value(pdata->gpio[MDM_ALIVE].pinnum);
	INFO("[%s] mdm_alive=[%d]\n", __func__, mdm_alive);

	return sprintf(buf, "%d\n", mdm_alive);
}

static ssize_t sc8800g_mdm_alive_store_v1(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	return -EINVAL;
}

static DEVICE_ATTR(mdm_alive, S_IRUSR | S_IWUSR | S_IRGRP,
	sc8800g_mdm_alive_show_v1, sc8800g_mdm_alive_store_v1);

static int sc8800g_create_sysfs_mdm_alive_attrs_v1(struct device *dev)
{
	dev_vdbg(dev, "[%s]", __func__);

	return device_create_file(dev, &dev_attr_mdm_alive);
}

static int sc8800g_remove_sysfs_mdm_alive_attrs_v1(struct device *dev)
{
	dev_vdbg(dev, "[%s]", __func__);

	device_remove_file(dev, &dev_attr_mdm_alive);
	return 0;
}

#endif

static struct tty_driver *sc8800g_tty_driver_v1;
static int tty_registered_v1 = 0;;
static int __devinit sc8800g_probe_v1(struct spi_device *spi)
{
	struct serial_sc8800g_platform_data *pdata;
	struct device *tty_dev;
	struct sc8800g_mdm_dev *mdv;
	int ret;
	struct sc8800g_gpio *sim_door_gpio = NULL;

	if (tty_registered_v1) {
		ERROR("[%s] tty_registered_v1, support only one modem\n", __func__);
		return -EBUSY;
	}

	pdata = (struct serial_sc8800g_platform_data *)spi->dev.platform_data;

	if (!pdata) {
		ERROR("[%s] missing platform data\n", __func__);
		return -ENODEV;
	}

	sc8800g_gpio_print_v1(pdata);

	spi->bits_per_word = SC8800G_TRANSFER_BITS_PER_WORD;

	ret = spi_setup(spi);
	if (ret < 0) {
		ERROR("[%s] spi_setup failed\n", __func__);
		goto error_out;
	}
	VDBG("[%s] spi_setup done\n", __func__);

	mdv = kzalloc(sizeof(struct sc8800g_mdm_dev), GFP_KERNEL);
	if (mdv == NULL)
		goto error_out;
	sc8800g_mdv_v1 = mdv;
	ret = sc8800g_mdm_dev_init_v1 (spi, mdv);
	if (ret)
		goto error_out;

	dev_set_drvdata(&spi->dev, mdv);

	/* support only one modem */
	sc8800g_tty_driver_v1 = alloc_tty_driver(1);
	if (!sc8800g_tty_driver_v1) {
		ERROR("[%s] alloc_tty_driver failed\n", __func__);
		return -ENOMEM;
	}
	VDBG("[%s] alloc_tty_driver done\n", __func__);

	sc8800g_tty_driver_v1->owner = THIS_MODULE;
	sc8800g_tty_driver_v1->driver_name = "sc8800g_tty_driver_v1";
	sc8800g_tty_driver_v1->name = "ttySC";
	sc8800g_tty_driver_v1->major = 0;
	sc8800g_tty_driver_v1->minor_start = 0;
	sc8800g_tty_driver_v1->type = TTY_DRIVER_TYPE_SERIAL;
	sc8800g_tty_driver_v1->subtype = SERIAL_TYPE_NORMAL;
	sc8800g_tty_driver_v1->init_termios = tty_std_termios;
	sc8800g_tty_driver_v1->init_termios.c_iflag = 0;
	sc8800g_tty_driver_v1->init_termios.c_oflag = 0;
	sc8800g_tty_driver_v1->init_termios.c_lflag = 0;
	sc8800g_tty_driver_v1->flags = TTY_DRIVER_RESET_TERMIOS |
		TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV;

	tty_set_operations(sc8800g_tty_driver_v1, &sc8800g_tty_ops_v1);
	VDBG("[%s] tty_set_operations done\n", __func__);

	if ((ret = tty_register_driver(sc8800g_tty_driver_v1)) != 0) {
		ERROR("[%s] tty_register_driver() failed\n", __func__);
		put_tty_driver(sc8800g_tty_driver_v1);
		return ret;
	}
	VDBG("[%s] tty_register_driver done\n", __func__);

	tty_dev = tty_register_device(sc8800g_tty_driver_v1, 0, NULL);
	mdv->tty_dev = tty_dev;
	dev_set_drvdata(tty_dev, mdv);

	/* TODO: error check */
	tty_registered_v1 = 1;
	VDBG("[%s] tty_registered_v1\n", __func__);

#ifdef SC8800G_POWER_ONOFF
	sc8800g_create_sysfs_attrs_v1(&spi->dev);
	sc8800g_create_sysfs_mdm_alive_attrs_v1(&spi->dev);
#endif
	VDBG("[%s] sc8800g_gpio_onoff_v1+\n", __func__);
	ret = sc8800g_gpio_onoff_v1 (pdata, MDM_GPIO_OFF, true);
	VDBG("[%s] sc8800g_gpio_onoff_v1-\n", __func__);
	if (ret)
		goto error_out;
	sc8800g_gpio_print_v1(pdata);
	sim_door_gpio = &pdata->gpio[SIM_DOOR];
	mdv->sim_door_state = gpio_get_value(sim_door_gpio->pinnum);

	atomic_set(&mdv->crash_state, 0);

	//Add notify function
	INIT_WORK(&mdv->sim_door_notify_work, sc8800g_sim_door_notify_work_func_v1);
//	mdv->sim_door_notify_wq = create_singlethread_workqueue("sc8800g_sim_door_notify");

	INIT_WORK(&mdv->crash_notify_work, sc8800g_crash_notify_work_func_v1);
//	mdv->crash_notify_wq = create_singlethread_workqueue("sc8800g_crash_notify");

	mdv->notify_kset = kset_create_and_add("event", NULL, kobject_get(&spi->dev.kobj));

	mdv->sim_door_kobj.kset = mdv->notify_kset;

	ret = kobject_init_and_add(&mdv->sim_door_kobj,
			&sc8800g_notify_ktype_v1, NULL, "sim_door_notify");
	if (ret) {
		kobject_put(&mdv->sim_door_kobj);
	}

	mdv->crash_kobj.kset = mdv->notify_kset;

	ret = kobject_init_and_add(&mdv->crash_kobj,
			&sc8800g_notify_ktype_v1, NULL, "crash_notify");
	if (ret) {
		kobject_put(&mdv->crash_kobj);
	}
	if ( pdata->enable_usb_status_to_mdm == 1 ) {
		static struct t_usb_status_notifier usb_status_notifier = {
			.name = "sc8800g",
			.func = sc8800g_usb_plugged_func_v1,
		};
		usb_register_notifier(&usb_status_notifier);
		mdv->usb_plugged_kobj.kset = mdv->notify_kset;

		ret = kobject_init_and_add(&mdv->usb_plugged_kobj,
				&sc8800g_notify_ktype_v1, NULL, "usb_plugged_notify");
		if (ret) {
			kobject_put(&mdv->usb_plugged_kobj);
		}
	}

	if ( pdata->enable_spi_byte_counter == 1 ) {
		g_enable_spi_byte_counter = 1;
	}

	INIT_WORK(&mdv->mdm_to_ap1_work, sc8800g_mdm_to_ap1_work_func_v1);

	sc8800g_irq_onoff_v1(pdata, MDM_IRQ_REQ,  mdv);

	sc8800g_irq_onoff_v1(pdata, MDM_IRQ_DISABLE,  mdv);

	VDBG("[%s] sc8800g_power_onoff_v1+\n", __func__);
	sc8800g_power_onoff_v1(pdata, PWR_ON);
	VDBG("[%s] sc8800g_power_onoff_v1-\n", __func__);

	return 0;

error_out:
	return ret;
}

static int sc8800g_remove_v1(struct spi_device *spi)
{
	struct sc8800g_mdm_dev *mdv = dev_get_drvdata(&spi->dev);

	INFO("[%s]\n", __func__);

	//cancel_work_sync(&mdv->tx_work);
	cancel_delayed_work_sync(&mdv->tx_work);
	destroy_workqueue(mdv->tx_workqueue);
	wake_lock_destroy(&g_sc8800g_wakelock_tx_v1);
	wake_lock_destroy(&g_sc8800g_wakelock_rx_v1);
	kthread_stop(mdv->rx_thread);

	cancel_work_sync(&mdv->sim_door_notify_work);
	cancel_work_sync(&mdv->crash_notify_work);
	cancel_work_sync(&mdv->mdm_to_ap1_work);

	tty_unregister_device(sc8800g_tty_driver_v1, 0);
	tty_unregister_driver(sc8800g_tty_driver_v1);
	tty_registered_v1 = 0;

	if(mdv->recv.alloc) {
		kzfree(mdv->recv.buf);
		mdv->recv.alloc = 0;
	}
	if(mdv->send.alloc) {
		kzfree(mdv->send.buf);
		mdv->send.alloc = 0;
	}

	if (mdv) {
		kzfree(mdv);
	}
#ifdef SC8800G_POWER_ONOFF
	sc8800g_remove_sysfs_attrs_v1(&spi->dev);
	sc8800g_remove_sysfs_mdm_alive_attrs_v1(&spi->dev);
#endif
	return 0;
}

#ifdef CONFIG_PM
static int sc8800g_suspend_v1(struct spi_device *spi, pm_message_t state)
{
	struct sc8800g_mdm_dev *mdv = dev_get_drvdata(&spi->dev);
	struct timeval tv = {NULL};
	struct timeval tv_start = {NULL};
	int i = 0;
	do_gettimeofday(&tv_start);

	while(!mutex_trylock(&dev_mutex)){
		do_gettimeofday(&tv);
		if ( (tv.tv_sec - tv_start.tv_sec) >= 3 ) {
			if ( g_enable_tty_write_log <= 100 )
				g_enable_tty_write_log += 5;
			INFO("[%s] can't lock mutex in [%d] times, enable log [%d]\n", __func__, i, g_enable_tty_write_log);
			sc8800g_print_info(mdv);
			return -1;
		}
		if ( i % 100 * 1000 * 100  == 0 ) {
			INFO("[%s] mutex_trylock [%d] times...\n", __func__, i);
		}
		udelay(10);
		i++;
	}

	if ( g_enable_tty_write_log > 0 ) {
		g_enable_tty_write_log--;
		if( g_enable_tty_write_log == 0 )
			INFO("[%s] disable tx log\n", __func__);
	}

	INFO("[%s] state=[%d]\n", __func__, state.event);
	mdv->suspended = 1;
	wait_event_interruptible_timeout(mdv->continue_suspend, !mdv->transfer_pending, msecs_to_jiffies(1000));
	sc8800g_irq_onoff_v1(mdv->pdata, MDM_IRQ_WAKE_ENABLE, mdv);
	mutex_unlock(&dev_mutex);
	return 0;
}

static int sc8800g_resume_v1(struct spi_device *spi)
{
	struct sc8800g_mdm_dev *mdv = dev_get_drvdata(&spi->dev);
	//unsigned long flags = 0;
	mutex_lock(&dev_mutex);
	INFO("[%s]\n", __func__);
	sc8800g_irq_onoff_v1(mdv->pdata, MDM_IRQ_WAKE_DISABLE, mdv);

	mdv->suspended = 0;

	mutex_unlock(&dev_mutex);

	return 0;
}
#endif
static struct spi_driver sc8800g_driver_v1 = {
	.driver = {
		.name		= "sc8800gv1",
		.bus		= &spi_bus_type,
		.owner		= THIS_MODULE,
	},
	.probe		= sc8800g_probe_v1,
	.remove		= __devexit_p(sc8800g_remove_v1),
#ifdef CONFIG_PM
	.suspend	= sc8800g_suspend_v1,
	.resume		= sc8800g_resume_v1,
#endif
};

/*
 * Module initialization
 */

static int __init serial_sc8800g_init_v1(void)
{
	DBG("[%s]\n", __func__);
	return spi_register_driver(&sc8800g_driver_v1);
}

static void __exit serial_sc8800g_exit_v1(void)
{
	DBG("[%s]\n", __func__);
	spi_unregister_driver(&sc8800g_driver_v1);
}

module_init(serial_sc8800g_init_v1);
module_exit(serial_sc8800g_exit_v1);
MODULE_DESCRIPTION("Serial driver over SPI for Spreadtrum SC8800G modem version 1");
MODULE_LICENSE("GPL");
