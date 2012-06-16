/*
 *  Abstract:
 *      Debug Flags for EHCI/CDC-ACM host
 *		driver debugging
 *
 *  Working environment:
 *      Android/NVidia AP30 projects
 *
 *  Referenced documents:
 *      N/A
 *
 *  Revision history:
 *      Trial26OCT2011 --Bert Lin--
 */
#ifndef __HOSTDBG_H__
#define __HOSTDBG_H__

#include <mach/bits.h>

#define DBG_EHCI_URB BIT12

#define DBG_USBCHR_L4 BIT11
#define DBG_USBCHR_L3 BIT10
#define DBG_USBCHR_L2 BIT9
#define DBG_USBCHR_L1 BIT8

#define DBG_RAWIP_L4 BIT7
#define DBG_RAWIP_L3 BIT6
#define DBG_RAWIP_L2 BIT5
#define DBG_RAWIP_L1 BIT4

#define DBG_ACM_REFCNT BIT3
#define DBG_ACM1_RW BIT1
#define DBG_ACM0_RW BIT0

#endif /* __HOSTDBG_H__ */

