#ifndef __ASM_MFOOTPRINT_H_
#define __ASM_MFOOTPRINT_H_

#define THREAD_SIZE_MASK_TH (8128)
#define THREAD_SIZE_MASK_BH (63)
#define OFFSET_TI_CPU       (20)
#define ORDER_LAST_STEPS    (3)
#define OFFSET_MF_TYPE      (4)
#define OFFSET_LS_STEP      (4)
#define MF_CLASS_IRQ_SHIFT  (0)
#define MF_CLASS_INT_SHIFT  (4)
#define MF_TYPE_LEAVE       (0)
#define MF_TYPE_ENTER       (1)

#define MEMORY_FOOTPRINT_STEP_SZ (16)
#define MEMORY_FOOTPRINT_SUBMASK (128)

#endif /* __ASM_MFOOTPRINT_H_ */
