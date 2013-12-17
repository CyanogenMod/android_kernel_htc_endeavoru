GCOV_PROFILE := y

obj-y                                   += ahb.o
obj-y                                   += apbio.o
obj-y                                   += common.o
obj-y                                   += board-info.o
obj-$(CONFIG_ARCH_TEGRA_2x_SOC)         += common-t2.o
obj-$(CONFIG_ARCH_TEGRA_3x_SOC)         += common-t3.o
obj-y                                   += io.o
obj-y                                   += irq.o
obj-y                                   += clock.o
obj-y                                   += clock-common.o
obj-y                                   += timer.o
obj-y                                   += timerinfo.o
ifeq ($(CONFIG_ARCH_TEGRA_2x_SOC),y)
obj-y                                   += tegra2_clocks.o
obj-y                                   += timer-t2.o
else
obj-y                                   += tegra3_clocks.o
obj-y                                   += timer-t3.o
endif
obj-y                                   += devices.o
obj-y                                   += delay.o
obj-y                                   += powergate.o
obj-y                                   += pm.o
obj-$(CONFIG_TEGRA_WDT_RECOVERY)        += wdt-recovery.o
obj-$(CONFIG_PM_SLEEP)                  += pm-irq.o
obj-y                                   += gic.o
obj-y                                   += pinmux.o


obj-y                                   += sleep.o

plus_sec := $(call as-instr,.arch_extension sec,+sec)
AFLAGS_sleep.o :=-Wa,-march=armv7-a$(plus_sec)

obj-$(CONFIG_ARCH_TEGRA_2x_SOC)         += sleep-t2.o
obj-$(CONFIG_ARCH_TEGRA_3x_SOC)         += sleep-t3.o
obj-y                                   += fuse.o
obj-y                                   += kfuse.o
obj-y                                   += csi.o
obj-$(CONFIG_TEGRA_SILICON_PLATFORM)    += tegra_odm_fuses.o
obj-y                                   += i2c_error_recovery.o
obj-y                                   += mc.o
obj-$(CONFIG_TEGRA_STAT_MON)            += tegra2_statmon.o
obj-$(CONFIG_USB_SUPPORT)               += usb_phy.o
obj-$(CONFIG_USB_SUPPORT)               += tegra3_usb_phy.o
obj-$(CONFIG_USB_SUPPORT)               += tegra2_usb_phy.o
obj-$(CONFIG_FIQ)                       += fiq.o
obj-$(CONFIG_TEGRA_FIQ_DEBUGGER)        += tegra_fiq_debugger.o
obj-$(CONFIG_TEGRA_PWM)                 += pwm.o
obj-$(CONFIG_TEGRA_ARB_SEMAPHORE)       += arb_sema.o

ifeq ($(CONFIG_TEGRA_SILICON_PLATFORM),y)
obj-y                                   += dvfs.o
obj-$(CONFIG_ARCH_TEGRA_2x_SOC)         += tegra2_dvfs.o
obj-$(CONFIG_ARCH_TEGRA_3x_SOC)         += tegra3_dvfs.o
obj-$(CONFIG_ARCH_TEGRA_3x_SOC)         += latency_allowance.o
obj-$(CONFIG_TEGRA_EDP_LIMITS)          += edp.o
endif
ifeq ($(CONFIG_TEGRA_SILICON_PLATFORM),y)
obj-$(CONFIG_ARCH_TEGRA_2x_SOC)         += tegra2_speedo.o
obj-$(CONFIG_ARCH_TEGRA_3x_SOC)         += tegra3_speedo.o
obj-$(CONFIG_ARCH_TEGRA_3x_SOC)         += tegra3_actmon.o
endif
obj-$(CONFIG_ARCH_TEGRA_2x_SOC)         += tegra2_emc.o
obj-$(CONFIG_ARCH_TEGRA_3x_SOC)         += tegra3_emc.o
obj-$(CONFIG_ARCH_TEGRA_2x_SOC)         += wakeups-t2.o
obj-$(CONFIG_ARCH_TEGRA_3x_SOC)         += wakeups-t3.o
obj-y                                   += wakeups.o
obj-$(CONFIG_ARCH_TEGRA_2x_SOC)         += pm-t2.o
obj-$(CONFIG_ARCH_TEGRA_3x_SOC)         += pm-t3.o

obj-$(CONFIG_ARCH_TEGRA_2x_SOC)         += pinmux-t2-tables.o
obj-$(CONFIG_ARCH_TEGRA_3x_SOC)         += pinmux-t3-tables.o
obj-$(CONFIG_LOCAL_TIMERS)              += localtimer.o
obj-$(CONFIG_SMP)                       += platsmp.o
obj-$(CONFIG_HOTPLUG_CPU)               += hotplug.o
obj-y                                   += headsmp.o
obj-y                                   += reset.o
obj-$(CONFIG_TEGRA_SYSTEM_DMA)          += dma.o
obj-$(CONFIG_CPU_FREQ)                  += cpu-tegra.o
ifeq ($(CONFIG_TEGRA_AUTO_HOTPLUG),y)
ifeq ($(CONFIG_CPUQUIET_FRAMEWORK),y)
obj-$(CONFIG_ARCH_TEGRA_3x_SOC)         += cpuquiet.o
else
obj-$(CONFIG_ARCH_TEGRA_3x_SOC)         += cpu-tegra3.o
endif
endif
obj-$(CONFIG_TEGRA_PCI)                 += pcie.o
obj-$(CONFIG_USB_SUPPORT)               += usb_phy.o
ifeq ($(CONFIG_CPU_IDLE),y)
obj-y                                   += cpuidle.o
ifeq ($(CONFIG_PM_SLEEP),y)
obj-$(CONFIG_ARCH_TEGRA_2x_SOC)         += cpuidle-t2.o
obj-$(CONFIG_ARCH_TEGRA_3x_SOC)         += cpuidle-t3.o
endif
endif
ifeq ($(CONFIG_TEGRA_THERMAL_THROTTLE),y)
obj-$(CONFIG_ARCH_TEGRA_2x_SOC)         += tegra2_throttle.o
obj-$(CONFIG_ARCH_TEGRA_3x_SOC)         += tegra3_throttle.o
endif
obj-$(CONFIG_ARCH_TEGRA_3x_SOC)         += tegra3_thermal.o
obj-$(CONFIG_TEGRA_IOVMM)               += iovmm.o
obj-$(CONFIG_TEGRA_IOVMM_GART)          += iovmm-gart.o
obj-$(CONFIG_TEGRA_IOVMM_SMMU)          += iovmm-smmu.o
obj-$(CONFIG_DEBUG_ICEDCC)              += sysfs-dcc.o
obj-$(CONFIG_TEGRA_CLUSTER_CONTROL)     += sysfs-cluster.o
ifeq ($(CONFIG_TEGRA_MC_PROFILE),y)
obj-$(CONFIG_ARCH_TEGRA_2x_SOC)         += tegra2_mc.o
endif
obj-$(CONFIG_SENSORS_TEGRA_TSENSOR)     += tegra3_tsensor.o
obj-$(CONFIG_TEGRA_DYNAMIC_PWRDET)      += powerdetect.o
obj-$(CONFIG_TEGRA_USB_MODEM_POWER)     += tegra_usb_modem_power.o
obj-$(CONFIG_TEGRA_PCI)                 += pcie.o
obj-${CONFIG_MACH_HARMONY}              += board-harmony.o
obj-${CONFIG_MACH_HARMONY}              += board-harmony-kbc.o
obj-${CONFIG_MACH_HARMONY}              += board-harmony-panel.o
obj-${CONFIG_MACH_HARMONY}              += board-harmony-pinmux.o
obj-${CONFIG_MACH_HARMONY}              += board-harmony-pcie.o
obj-${CONFIG_MACH_HARMONY}              += board-harmony-power.o

obj-${CONFIG_MACH_PAZ00}                += board-paz00.o
obj-${CONFIG_MACH_PAZ00}                += board-paz00-pinmux.o

obj-${CONFIG_MACH_SEABOARD}             += board-seaboard.o
obj-${CONFIG_MACH_SEABOARD}             += board-seaboard-pinmux.o

obj-${CONFIG_MACH_TEGRA_DT}             += board-dt.o
obj-${CONFIG_MACH_TEGRA_DT}             += board-harmony-pinmux.o

obj-${CONFIG_MACH_TRIMSLICE}            += board-trimslice.o
obj-${CONFIG_MACH_TRIMSLICE}            += board-trimslice-pinmux.o

obj-${CONFIG_MACH_P852}                 += p852/

obj-${CONFIG_MACH_VENTANA}              += board-ventana.o
obj-${CONFIG_MACH_VENTANA}              += board-ventana-pinmux.o
obj-${CONFIG_MACH_VENTANA}              += board-ventana-sdhci.o
obj-${CONFIG_MACH_VENTANA}              += board-ventana-power.o
obj-${CONFIG_MACH_VENTANA}              += board-ventana-panel.o
obj-${CONFIG_MACH_VENTANA}              += board-ventana-sensors.o
obj-${CONFIG_MACH_VENTANA}              += board-ventana-memory.o

obj-${CONFIG_MACH_ARUBA}                += board-aruba.o
obj-${CONFIG_MACH_ARUBA}                += board-aruba-panel.o
obj-${CONFIG_MACH_ARUBA}                += board-aruba-pinmux.o
obj-${CONFIG_MACH_ARUBA}                += board-aruba-power.o
obj-${CONFIG_MACH_ARUBA}                += board-aruba-sdhci.o
obj-${CONFIG_MACH_ARUBA}                += board-aruba-sensors.o

obj-${CONFIG_MACH_WHISTLER}             += board-whistler.o
obj-${CONFIG_MACH_WHISTLER}             += board-whistler-pinmux.o
obj-${CONFIG_MACH_WHISTLER}             += board-whistler-sdhci.o
obj-${CONFIG_MACH_WHISTLER}             += board-whistler-power.o
obj-${CONFIG_MACH_WHISTLER}             += board-whistler-panel.o
obj-${CONFIG_MACH_WHISTLER}             += board-whistler-sensors.o
obj-${CONFIG_MACH_WHISTLER}             += board-whistler-kbc.o
obj-${CONFIG_MACH_WHISTLER}             += board-whistler-baseband.o
obj-${CONFIG_MACH_WHISTLER}             += board-whistler-memory.o

obj-${CONFIG_MACH_CARDHU}               += board-cardhu.o
obj-${CONFIG_MACH_CARDHU}               += board-cardhu-kbc.o
obj-${CONFIG_MACH_CARDHU}               += board-cardhu-panel.o
obj-${CONFIG_MACH_CARDHU}               += board-cardhu-pinmux.o
obj-${CONFIG_MACH_CARDHU}               += board-cardhu-power.o
obj-${CONFIG_MACH_CARDHU}               += board-cardhu-pm298-power-rails.o
obj-${CONFIG_MACH_CARDHU}               += board-cardhu-pm299-power-rails.o
obj-${CONFIG_MACH_CARDHU}               += board-cardhu-sdhci.o
obj-${CONFIG_MACH_CARDHU}               += board-cardhu-sensors.o
obj-${CONFIG_MACH_CARDHU}               += board-cardhu-memory.o
obj-${CONFIG_MACH_CARDHU}               += board-cardhu-powermon.o

obj-${CONFIG_MACH_KAI}                  += board-touch-kai-synaptics-spi.o
obj-y                                   += board-touch-raydium_spi.o

obj-${CONFIG_MACH_P1852}               += board-p1852.o
obj-${CONFIG_MACH_P1852}               += board-p1852-panel.o
obj-${CONFIG_MACH_P1852}               += board-p1852-pinmux.o
obj-${CONFIG_MACH_P1852}               += board-p1852-sdhci.o

obj-${CONFIG_MACH_TEGRA_ENTERPRISE}     += board-enterprise.o
obj-${CONFIG_MACH_TEGRA_ENTERPRISE}     += board-enterprise-panel.o
obj-${CONFIG_MACH_TEGRA_ENTERPRISE}     += board-enterprise-pinmux.o
obj-${CONFIG_MACH_TEGRA_ENTERPRISE}     += board-enterprise-sdhci.o
obj-${CONFIG_MACH_TEGRA_ENTERPRISE}     += board-enterprise-memory.o
obj-${CONFIG_MACH_TEGRA_ENTERPRISE}     += board-enterprise-power.o
obj-${CONFIG_MACH_TEGRA_ENTERPRISE}     += board-enterprise-baseband.o
obj-${CONFIG_MACH_TEGRA_ENTERPRISE}     += board-enterprise-kbc.o
obj-${CONFIG_MACH_TEGRA_ENTERPRISE}     += board-enterprise-sensors.o

obj-${CONFIG_MACH_KAI}                  += board-kai.o
obj-${CONFIG_MACH_KAI}                  += board-kai-kbc.o
obj-${CONFIG_MACH_KAI}                  += board-kai-memory.o
obj-${CONFIG_MACH_KAI}                  += board-kai-panel.o
obj-${CONFIG_MACH_KAI}                  += board-kai-pinmux.o
obj-${CONFIG_MACH_KAI}                  += board-kai-power.o
obj-${CONFIG_MACH_KAI}                  += board-kai-sdhci.o
obj-${CONFIG_MACH_KAI}                  += board-kai-sensors.o
obj-y                                   += eeprom-wifi-mac.o

obj-${CONFIG_MACH_ENRC2U}               += board-enrc2u.o
obj-${CONFIG_MACH_ENRC2U}               += board-enrc2u-panel.o
obj-${CONFIG_MACH_ENRC2U}               += board-enrc2u-pinmux.o
obj-${CONFIG_MACH_ENRC2U}               += board-enrc2u-sdhci.o
obj-${CONFIG_MACH_ENRC2U}               += board-enrc2u-memory.o
obj-${CONFIG_MACH_ENRC2U}               += board-enrc2u-power.o
obj-${CONFIG_MACH_ENRC2U}               += board-enrc2u-baseband.o
obj-${CONFIG_MACH_ENRC2U}               += board-enrc2u-kbc.o
obj-${CONFIG_MACH_ENRC2U}               += board-enrc2u-sensors.o
obj-${CONFIG_MACH_ENRC2U}               += board-enrc2u-audio.o
obj-${CONFIG_MACH_ENRC2U}               += board-enrc2u-rfkill.o
obj-${CONFIG_MACH_ENRC2U}               += htc_bdaddress.o
obj-${CONFIG_MACH_ENRC2U}               += board-enrc2u-camera.o

obj-${CONFIG_MACH_ENRC2B}               += board-enrc2b.o
obj-${CONFIG_MACH_ENRC2B}               += board-enrc2b-panel.o
obj-${CONFIG_MACH_ENRC2B}               += board-enrc2b-pinmux.o
obj-${CONFIG_MACH_ENRC2B}               += board-enrc2b-sdhci.o
obj-${CONFIG_MACH_ENRC2B}               += board-enrc2b-memory.o
obj-${CONFIG_MACH_ENRC2B}               += board-enrc2b-power.o
obj-${CONFIG_MACH_ENRC2B}               += board-enrc2b-baseband.o
obj-${CONFIG_MACH_ENRC2B}               += board-enrc2b-kbc.o
obj-${CONFIG_MACH_ENRC2B}               += board-enrc2b-sensors.o
obj-${CONFIG_MACH_ENRC2B}               += board-enrc2b-audio.o
obj-${CONFIG_MACH_ENRC2B}               += board-enrc2b-rfkill.o
obj-${CONFIG_MACH_ENRC2B}               += htc_bdaddress.o
obj-${CONFIG_MACH_ENRC2B}               += board-enrc2b-camera.o

obj-${CONFIG_MACH_ENDEAVORU}               += board-endeavoru.o
obj-${CONFIG_MACH_ENDEAVORU}               += board-endeavor-panel.o
obj-${CONFIG_MACH_ENDEAVORU}               += board-endeavoru-pinmux.o
obj-${CONFIG_MACH_ENDEAVORU}               += board-endeavoru-sdhci.o
obj-${CONFIG_MACH_ENDEAVORU}               += board-endeavoru-memory.o
obj-${CONFIG_MACH_ENDEAVORU}               += board-endeavoru-power.o
obj-${CONFIG_MACH_ENDEAVORU}               += board-endeavoru-baseband.o
obj-${CONFIG_MACH_ENDEAVORU}               += board-endeavoru-kbc.o
obj-${CONFIG_MACH_ENDEAVORU}               += board-endeavoru-sensors.o
obj-${CONFIG_MACH_ENDEAVORU}               += board-endeavoru-audio.o
obj-${CONFIG_MACH_ENDEAVORU}               += board-endeavoru-rfkill.o
obj-${CONFIG_MACH_ENDEAVORU}               += htc_bdaddress.o
obj-${CONFIG_MACH_ENDEAVORU}               += board-endeavoru-pinmux.o
obj-${CONFIG_MACH_ENDEAVORU}               += tegra_flashlight.o
obj-${CONFIG_MACH_ENDEAVORU}               += board-endeavoru-camera.o

obj-${CONFIG_MACH_ENDEAVORTD}               += board-endeavortd.o
obj-${CONFIG_MACH_ENDEAVORTD}               += board-endeavor-panel.o
obj-${CONFIG_MACH_ENDEAVORTD}               += board-endeavortd-pinmux.o
obj-${CONFIG_MACH_ENDEAVORTD}               += board-endeavortd-sdhci.o
obj-${CONFIG_MACH_ENDEAVORTD}               += board-endeavortd-memory.o
obj-${CONFIG_MACH_ENDEAVORTD}               += board-endeavortd-power.o
obj-${CONFIG_MACH_ENDEAVORTD}               += board-endeavortd-baseband.o
obj-${CONFIG_MACH_ENDEAVORTD}               += board-endeavortd-kbc.o
obj-${CONFIG_MACH_ENDEAVORTD}               += board-endeavortd-sensors.o
obj-${CONFIG_MACH_ENDEAVORTD}               += board-endeavortd-audio.o
obj-${CONFIG_MACH_ENDEAVORTD}               += board-endeavortd-rfkill.o
obj-${CONFIG_MACH_ENDEAVORTD}               += htc_bdaddress.o
obj-${CONFIG_MACH_ENDEAVORTD}               += board-endeavortd-pinmux.o
obj-${CONFIG_MACH_ENDEAVORTD}               += tegra_flashlight.o
obj-${CONFIG_MACH_ENDEAVORTD}               += board-endeavortd-camera.o

obj-${CONFIG_MACH_EVITAREUL}            += board-evitareul.o
obj-${CONFIG_MACH_EVITAREUL}            += board-evitareul-panel.o
obj-${CONFIG_MACH_EVITAREUL}            += board-evitareul-pinmux.o
obj-${CONFIG_MACH_EVITAREUL}            += board-evitareul-sdhci.o
obj-${CONFIG_MACH_EVITAREUL}            += board-evitareul-memory.o
obj-${CONFIG_MACH_EVITAREUL}            += board-evitareul-power.o
obj-${CONFIG_MACH_EVITAREUL}            += board-evitareul-baseband.o
obj-${CONFIG_MACH_EVITAREUL}            += board-evitareul-kbc.o
obj-${CONFIG_MACH_EVITAREUL}            += board-evitareul-sensors.o
obj-${CONFIG_MACH_EVITAREUL}            += board-evitareul-audio.o
obj-${CONFIG_MACH_EVITAREUL}            += board-evitareul-rfkill.o
obj-${CONFIG_MACH_EVITAREUL}            += htc_bdaddress.o
obj-${CONFIG_MACH_EVITAREUL}            += board-evitareul-camera.o

obj-${CONFIG_MACH_OPERAUL}            += board-operaul.o
obj-${CONFIG_MACH_OPERAUL}            += board-operaul-panel.o
obj-${CONFIG_MACH_OPERAUL}            += board-operaul-pinmux.o
obj-${CONFIG_MACH_OPERAUL}            += board-operaul-sdhci.o
obj-${CONFIG_MACH_OPERAUL}            += board-operaul-memory.o
obj-${CONFIG_MACH_OPERAUL}            += board-operaul-power.o
#obj-${CONFIG_MACH_OPERAUL}            += board-operaul-baseband.o
obj-${CONFIG_MACH_OPERAUL}            += board-operaul-kbc.o
obj-${CONFIG_MACH_OPERAUL}            += board-operaul-sensors.o
obj-${CONFIG_MACH_OPERAUL}            += board-operaul-audio.o
obj-${CONFIG_MACH_OPERAUL}            += board-operaul-rfkill.o
obj-${CONFIG_MACH_OPERAUL}            += htc_bdaddress.o
obj-${CONFIG_MACH_OPERAUL}            += board-operaul-camera.o

obj-$(CONFIG_HTC_HEADSET_MGR) += htc_headset_mgr.o
obj-$(CONFIG_HTC_HEADSET_H2W) += htc_headset_h2w.o
obj-$(CONFIG_HTC_HEADSET_GPIO) += htc_headset_gpio.o
obj-$(CONFIG_HTC_HEADSET_MICROP) += htc_headset_microp.o
obj-$(CONFIG_HTC_HEADSET_PMIC) += htc_headset_pmic.o
obj-$(CONFIG_HTC_HEADSET_MISC) += htc_headset_misc.o
obj-$(CONFIG_HTC_HEADSET_8X60) += htc_headset_8x60.o
obj-$(CONFIG_HTC_HEADSET_ONE_WIRE) += htc_headset_one_wire.o

obj-${CONFIG_TEGRA_BB_XMM_POWER}        += baseband-xmm-power.o
obj-${CONFIG_TEGRA_BB_XMM_POWER2}       += baseband-xmm-power2.o
obj-${CONFIG_HTC_SIMHOTSWAP}                 += htc_simhotswap.o

obj-${CONFIG_HTC_PROJECT}               += devices_htc.o
obj-${CONFIG_HTC_PROJECT}               += nand_partitions.o
obj-${CONFIG_HTC_PROJECT}               += htc_audio_power.o
obj-${CONFIG_HTC_PROJECT}               += htc_wifi_nvs.o
obj-${CONFIG_HTC_PROJECT}               += nvdumper.o
obj-${CONFIG_HTC_BATT_CORE}             += htc_battery_core.o
obj-$(CONFIG_BATTERY_TPS80032)          += htc_battery_tps80032.o
obj-y					+= htc_perf.o
obj-y                                   += cpuidle_notifier.o
obj-${CONFIG_MEMORY_FOOTPRINT_DEBUGGING}+= mfootprint.o
obj-${CONFIG_MEMORY_FOOTPRINT_DEBUGGING}+= mfootprint-entry.o

# sophia: add mdm driver
ifdef CONFIG_QCT_9K_MODEM
obj-y += mdm2.o mdm_common.o
obj-y += baseband-qct-mdm-hsic.o
obj-$(CONFIG_MSM_SYSMON_COMM) += sysmon.o
obj-$(CONFIG_MSM_HSIC_SYSMON) += hsic_sysmon.o
obj-$(CONFIG_MSM_HSIC_SYSMON_TEST) += hsic_sysmon_test.o
endif
# Mars_Lin@20120521: add subsystem restart function
ifdef CONFIG_MSM_SUBSYSTEM_RESTART
	obj-y += subsystem_notif.o
	obj-y += subsystem_restart.o
#	obj-y += ramdump.o
#	obj-$(CONFIG_ARCH_MSM8X60) += modem-8660.o lpass-8660.o
endif

# Kenson_Lee@20120615: add for packet filter
obj-y += htc_port_list_lte.o

obj-y 					+= htc_util.o

obj-$(CONFIG_HTC_MONITOR) += htc_monitor.o

obj-$(CONFIG_HTC_FIQ_DUMPER) += fiq_dumper.o
