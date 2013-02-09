#ifndef YUSHAN_PLATFORM_SPECIFIC_H
#define YUSHAN_PLATFORM_SPECIFIC_H

#ifdef __cplusplus
extern "C"{
#endif   /*__cplusplus*/

#include <media/rawchip/Yushan_API.h>
#include <media/rawchip/yushan_registermap.h>
#include <linux/sched.h>
#include <media/rawchip/rawchip.h>


#define RAWCHIP_INT_TYPE_ERROR (0x01<<0)
#define RAWCHIP_INT_TYPE_NEW_FRAME (0x01<<1)
#define RAWCHIP_INT_TYPE_PDP_EOF_EXECCMD (0x01<<2)
#define RAWCHIP_INT_TYPE_DPP_EOF_EXECCMD (0x01<<3)
#define RAWCHIP_INT_TYPE_DOP_EOF_EXECCMD (0x01<<4)


/* Interrupt functions */
bool_t Yushan_WaitForInterruptEvent (uint8_t bInterruptId ,uint32_t udwTimeOut);
bool_t Yushan_WaitForInterruptEvent2 (uint8_t bInterruptId ,uint32_t udwTimeOut);
uint8_t Yushan_parse_interrupt(int intr_pad, int error_times[TOTAL_INTERRUPT_COUNT]);
void	Yushan_Interrupt_Manager_Pad0(void);
void	Yushan_Interrupt_Manager_Pad1(void);



#ifdef __cplusplus
}
#endif   /*__cplusplus*/

#endif
