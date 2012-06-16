/*
 * =====================================================================================
 *
 *       Filename:  debug.h
 *
 *    Description:  macro for mmc subsystem
 *
 *        Version:  1.0
 *        Created:  03/30/2011 09:08:58 AM
 *
 *        Author:  chuck_huang
 *        Company:  htc
 *
 * =====================================================================================
 */

#define CONFIG_SD_DEBUG

#define DRV_SD "[SD]"

#ifdef CONFIG_SD_DEBUG
#define LOGD(fmt...) printk(KERN_DEBUG DRV_SD fmt)
#else
#define LOGD(fmt...) do {} while (0)
#endif


#define LOGI(fmt...) printk(KERN_INFO DRV_SD fmt)
#define LOGW(fmt...) printk(KERN_WARNING DRV_SD " WARN " fmt)
#define LOGE(fmt...) printk(KERN_ERR DRV_SD " ERR " fmt)
