#ifndef __CAMERA_MFG_H__
#define __CAMERA_MFG_H__

/* read rawchip ID and write into /sys/camera_rawchip_status/probed_rawchip_id */
#if defined(CONFIG_BOARD_MFG)
#define MFG_READ_RAWCHIP_ID() \
do { \
	if (rawchipCtrl->pdata->camera_rawchip_power_on != NULL) { \
		rawchipCtrl->pdata->camera_rawchip_power_on(); \
		rawchip_match_id(); \
		tegra_rawchip_attr_node(); \
		if (rawchipCtrl->pdata->camera_rawchip_power_off != NULL) { \
			rawchipCtrl->pdata->camera_rawchip_power_off(); \
		} \
	} \
} while (0)
#else
#define MFG_READ_RAWCHIP_ID() \
do { \
	(void) 0; \
} while (0)
#endif

#endif /* __CAMERA_MFG_H__ */
