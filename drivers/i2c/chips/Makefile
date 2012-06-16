#
# Makefile for miscellaneous I2C chip drivers.
#
# Do not add new drivers to this directory! It is DEPRECATED.
#
# Device drivers are better grouped according to the functionality they
# implement rather than to the bus they are connected to. In particular:
# * Hardware monitoring chip drivers go to drivers/hwmon
# * RTC chip drivers go to drivers/rtc
# * I/O expander drivers go to drivers/gpio
#

obj-$(CONFIG_SENSORS_NFC_PN544) += pn544.o
obj-$(CONFIG_SENSORS_BMA250)	+= bma250.o
obj-$(CONFIG_SENSORS_ONLY_BMA250)      += bma250_without_Ecompass.o
obj-$(CONFIG_SENSORS_CAPSENSE_SAR) += cap_sense_vertex.o
obj-$(CONFIG_SENSORS_CAPSENSE_HSNR) += cap_sense_a_project.o
obj-$(CONFIG_SENSORS_AKM8975_PANA_GYRO)   += akm8975_pana_gyro.o
obj-$(CONFIG_SENSORS_PANASONIC_GYRO)	+= ewtzmu2.o

obj-$(CONFIG_INPUT_ISL29028_CM3628)    += cm3628.o
obj-$(CONFIG_INPUT_CAPELLA_CM3629)     += cm3629.o

ifeq ($(CONFIG_I2C_DEBUG_CHIP),y)
EXTRA_CFLAGS += -DDEBUG
endif
EXTRA_CFLAGS += -DPRINT_SP

