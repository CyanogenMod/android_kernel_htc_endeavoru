#
# Makefile for USB Class drivers
# (one step up from the misc category)
#

ifeq (${CONFIG_MACH_ENDEAVORU},y)
obj-$(CONFIG_USB_ACM)		+= cdc-acm.o
endif
obj-$(CONFIG_USB_PRINTER)	+= usblp.o
obj-$(CONFIG_USB_WDM)		+= cdc-wdm.o
obj-$(CONFIG_USB_TMC)		+= usbtmc.o
