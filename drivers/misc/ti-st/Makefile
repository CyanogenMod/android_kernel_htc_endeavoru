#
# Makefile for TI's shared transport line discipline
# and its protocol drivers (BT, FM, GPS)
#
obj-$(CONFIG_TI_ST) 		+= st_drv.o
st_drv-objs			:= st_core.o st_kim.o st_ll.o
obj-$(CONFIG_TI_ST)		+= ti_hci_drv.o
ti_hci_drv-objs			:= hci_if_drv.o
#obj-$(CONFIG_TI_ST_FM) += fm_drv.o
#fm_drv-objs     := fmdrv_core.o fmdrv_v4l2.o fmdrv_st.o fmdrv_mixer.o fmdrv_chr.o
obj-$(CONFIG_TI_ST)		+= gps_drv.o
