#
# Makefile for the ts27010mux driver over USB.
#
#
MODULE_NAME = ts27010mux_usb
MODULE_NAME_UT = ts27010mux_usb_ut

obj-y += $(MODULE_NAME).o
#obj-y += $(MODULE_NAME_UT).o

$(MODULE_NAME)-objs := ts27010_mux.o \
			ts27010_tty.o \
			ts27010_td.o \
			ts27010_misc.o \
			ts27010_logger.o \
			ts27010_ldisc.o \
			ts27010_net.o

$(MODULE_NAME_UT)-objs := ut_ts27010_usb.o
