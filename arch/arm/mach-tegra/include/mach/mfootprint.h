#ifndef __MFOOTPRINT_H_
#define __MFOOTPRINT_H_

#if 0
#define HEADER_LENGTH	  14
#define LATEST_INDEX       5
#define FLAG_LENGTH	   8
#define FOOTPRINT_LENGTH 396
#define MF_DEBUG(debug_flag)	do { if (likely(mf)) {                         \
					memcpy(&mf->latest[LATEST_INDEX], &debug_flag, FLAG_LENGTH); \
					mf->latest[HEADER_LENGTH-1] = '\n'; \
					memcpy(&mf->flag[mf_index], &debug_flag, FLAG_LENGTH); \
					mf_index += 9;                         \
					if (mf_index >= FOOTPRINT_LENGTH) mf_index = 0; \
					}	                                       \
				} while(0)
#else
#define MF_DEBUG(debug_flag) do {} while (0)
#endif

#if defined(CONFIG_MEMORY_FOOTPRINT_DEBUGGING)
extern void __mf_int_enter(void*);
extern void __mf_int_leave(void*);
extern void __mf_irq_enter(void*);
extern void __mf_irq_leave(void*);

#define mfootprint_entry(type, dir) \
static inline void mf_ ##type## _ ##dir \
(void* callsite) \
{ \
	asm volatile( \
			"mov r0, %[callsite]\n" \
			"1:\n" \
			"bl __mf_" #type "_" #dir "\n" \
			".pushsection __mfoot_entries,\"a\"\n" \
			".long 1b\n" \
			".popsection\n" \
			:: [callsite] "r" (callsite) \
			:"r0", "r1", "r2", "r3", "r12", "lr", "cc", "memory"); \
}

mfootprint_entry(int, enter);
mfootprint_entry(int, leave);
mfootprint_entry(irq, enter);
mfootprint_entry(irq, leave);

#undef mfootprint_entry

#define notrace_spin_lock_irqsave(lock, flags)       \
do {                                                 \
	asm volatile(                                    \
		"	mrs	%0, cpsr	@ arch_local_irq_save\n" \
		"	cpsid	i"                               \
		: "=r" (flags) : : "memory", "cc");          \
	preempt_disable();                               \
	spin_acquire(&lock->dep_map, 0, 0, _RET_IP_);    \
	do_raw_spin_lock_flags(lock, &flags);            \
} while (0)

#define notrace_spin_unlock_irqrestore(lock, flags)  \
do {                                                 \
	spin_release(&lock->dep_map, 1, _RET_IP_);       \
	do_raw_spin_unlock(lock);                        \
	asm volatile(                                    \
		"	msr	cpsr_c, %0	@ local_irq_restore"     \
		:                                            \
		: "r" (flags)                                \
		: "memory", "cc");                           \
	preempt_enable();                                \
} while (0)

#else
#define mf_int_enter(x) do {} while (0)
#define mf_int_leave(x) do {} while (0)
#define mf_irq_enter(x) do {} while (0)
#define mf_irq_leave(x) do {} while (0)
#endif
#endif
