#include <linux/const.h>
#include <linux/init.h>
#include <linux/linkage.h>

#include <asm/assembler.h>
#include <asm/asm-offsets.h>
#include <asm/system.h>
#include <asm/cache.h>
#include <asm/mfootprint.h>

.macro get_cpu, tmp
	bic \tmp, sp, #THREAD_SIZE_MASK_TH
	bic \tmp, \tmp, #THREAD_SIZE_MASK_BH
	ldr \tmp, [\tmp, #OFFSET_TI_CPU]
.endm

.macro get_last_step, out, class, cpu
	ldr \out, =last_steps
	add \cpu, \class
	add \out, \out, \cpu, lsl #ORDER_LAST_STEPS
.endm

.macro mf_enter, class
	bx lr            /* disabled by default */
	get_cpu r3
	get_last_step r12, \class, r3
	ldm r12, {r2, r3}
	eor r1, r0, lr
	teq r2, r1
	mov r2, #MF_TYPE_ENTER
	streq r2, [r3, #OFFSET_MF_TYPE]
	bxeq lr

	/* stepping */
	add r3, r3, #MEMORY_FOOTPRINT_STEP_SZ
	bic r3, r3, #MEMORY_FOOTPRINT_SUBMASK

	stm r3,  {r0, r2, lr}
	stm r12, {r1, r3}
	bx lr
.endm

.macro mf_leave, class
	bx lr
	get_cpu r3
	get_last_step r1, \class, r3
	mov r3, #MF_TYPE_LEAVE
	ldr r2, [r1, #OFFSET_LS_STEP]
	str r3, [r2, #OFFSET_MF_TYPE]
	bx lr
.endm

.align L1_CACHE_SHIFT
ENTRY(__mf_irq_enter)
	mf_enter #MF_CLASS_IRQ_SHIFT
ENDPROC(__mf_irq_enter)

ENTRY(__mf_irq_leave)
	mf_leave #MF_CLASS_IRQ_SHIFT
ENDPROC(__mf_irq_leave)

.align L1_CACHE_SHIFT
ENTRY(__mf_int_enter)
	mf_enter #MF_CLASS_INT_SHIFT
ENDPROC(__mf_int_enter)

ENTRY(__mf_int_leave)
	mf_leave #MF_CLASS_INT_SHIFT
ENDPROC(__mf_int_leave)
