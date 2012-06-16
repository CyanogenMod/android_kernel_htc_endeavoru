GCOV_PROFILE := y
#
# Makefile for the video capture/playback device drivers.
#
obj-y				+= avp/
obj-$(CONFIG_TEGRA_MEDIASERVER)	+= mediaserver/
obj-$(CONFIG_TEGRA_NVAVP)	+= nvavp/
obj-$(CONFIG_VIDEO_RAWCHIP) += rawchip/
obj-$(CONFIG_TEGRA_DTV)		+= tegra_dtv.o
obj-$(CONFIG_TEGRA_CAMERA)	+= tegra_camera.o
obj-$(CONFIG_VIDEO_AR0832)	+= ar0832_main.o
obj-$(CONFIG_VIDEO_OV5650)	+= ov5650.o
obj-$(CONFIG_VIDEO_OV14810)	+= ov14810.o
obj-$(CONFIG_VIDEO_OV9726)	+= ov9726.o
obj-$(CONFIG_VIDEO_OV2710)	+= ov2710.o
obj-$(CONFIG_VIDEO_SOC380)	+= soc380.o
obj-$(CONFIG_TORCH_SSL3250A)	+= ssl3250a.o
obj-$(CONFIG_TORCH_TPS61050)	+= tps61050.o
obj-$(CONFIG_VIDEO_SH532U)	+= sh532u.o

# Camera sensro driver
# Vertex
obj-$(CONFIG_VIDEO_S5K5B3G)			+= s5k5b3g.o
obj-$(CONFIG_VIDEO_S5K4E1GA)		+= s5k4e1ga.o
obj-$(CONFIG_VIDEO_S5K6A1G)			+= s5k6a1g.o
# Blue
obj-$(CONFIG_VIDEO_S5K4E5YX)		+= s5k4e5yx.o
obj-$(CONFIG_VIDEO_S5K6AAFX)		+= s5k6aafx.o
# ENR
obj-$(CONFIG_VIDEO_S5K3H2Y)			+= s5k3h2y.o
obj-$(CONFIG_VIDEO_S5K6A1GX03)		+= s5k6a1gx03.o

# Camera focuser driver
obj-$(CONFIG_VIDEO_AD5820)		+= ad5820.o
obj-$(CONFIG_VIDEO_AD5823)		+= ad5823.o
