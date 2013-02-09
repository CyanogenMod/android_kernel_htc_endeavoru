#ifndef __MFOOTPRINT_H_
#define __MFOOTPRINT_H_

#include <linux/string.h>

#if defined(CONFIG_MEMORY_FOOTPRINT_DEBUGGING)
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
				} while(0)                                     \


typedef struct memory_footptint
{
	char latest[HEADER_LENGTH];
	char flag[FOOTPRINT_LENGTH];
} mfootprint;

#ifdef __MFOOTPRINT_C_
int mf_index = 0;
mfootprint *mf = NULL;
mfootprint *old_mf = NULL;

EXPORT_SYMBOL(mf_index);
EXPORT_SYMBOL(mf);
EXPORT_SYMBOL(old_mf);
#else
extern int mf_index;
extern mfootprint *mf;
extern mfootprint *old_mf;
#endif

#else
#define MF_DEBUG(debug_flag)
#endif

#endif
