/***************************************************************************
**+----------------------------------------------------------------------+**
**|                                ****                                  |**
**|                                ****                                  |**
**|                                ******o***                            |**
**|                          ********_///_****                           |**
**|                           ***** /_//_/ ****                          |**
**|                            ** ** (__/ ****                           |**
**|                                *********                             |**
**|                                 ****                                 |**
**|                                  ***                                 |**
**|                                                                      |**
**|     Copyright (c) 1998 - 2008 Texas Instruments Incorporated         |**
**|                        ALL RIGHTS RESERVED                           |**
**|                                                                      |**
**| Permission is hereby granted to licensees of Texas Instruments       |**
**| Incorporated (TI) products to use this computer program for the sole |**
**| purpose of implementing a licensee product based on TI products.     |**
**| No other rights to reproduce, use, or disseminate this computer      |**
**| program, whether in part or in whole, are granted.                   |**
**|                                                                      |**
**| TI makes no representation or warranties with respect to the         |**
**| performance of this computer program, and specifically disclaims     |**
**| any responsibility for any damages, special or consequential,        |**
**| connected with the use of this program.                              |**
**|                                                                      |**
**+----------------------------------------------------------------------+**
***************************************************************************/

#ifndef MSM_SDIODRV_DEBUG_H
#define MSM_SDIODRV_DEBUG_H

#include <linux/kernel.h>

typedef enum{
SDIO_DEBUGLEVEL_EMERG = 1,
SDIO_DEBUGLEVEL_ALERT,
SDIO_DEBUGLEVEL_CRIT,
SDIO_DEBUGLEVEL_ERR = 4,
SDIO_DEBUGLEVEL_WARNING,
SDIO_DEBUGLEVEL_NOTICE,
SDIO_DEBUGLEVEL_INFO,
SDIO_DEBUGLEVEL_DEBUG = 8
} sdio_debuglevel;

extern int g_sdio_debug_level1;

#ifdef SDIO_DEBUG

#define PERR(format, args...) \
		{ if (g_sdio_debug_level1 >= SDIO_DEBUGLEVEL_ERR)\
		printk(format , ##args); }
#define PDEBUG(format, args...) \
		{ if (g_sdio_debug_level1 >= SDIO_DEBUGLEVEL_DEBUG)\
		printk(format , ##args); }
#define PINFO(format, ...) \
		{ if (g_sdio_debug_level1 >= SDIO_DEBUGLEVEL_INFO)\
		printk(format , ##__VA_ARGS__); }
#define PNOTICE(format, ...) \
		{ if (g_sdio_debug_level1 >= SDIO_DEBUGLEVEL_NOTICE)\
		printk(format , ##__VA_ARGS__); }
#define PWARNING(format, ...) \
		{ if (g_sdio_debug_level1 >= SDIO_DEBUGLEVEL_WARNING)\
		printk(format , ##__VA_ARGS__); }

#else

#define PERR(format, args...) \
		{ if (g_sdio_debug_level1 >= SDIO_DEBUGLEVEL_ERR)\
		printk(format , ##args); }
#define PDEBUG(format, args...)
#define PINFO(format, ...)
#define PNOTICE(format, ...)
#define PWARNING(format, ...)

#endif

/* we want errors reported anyway */

#define PERR1 PERR
#define PERR2 PERR
#define PERR3 PERR

#endif /* OMAP3430_SDIODRV_DEBUG_H */
