/**
 * Copyright (c) 2011 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#ifndef __AAT1271_H__
#define __AAT1271_H__

#include <linux/ioctl.h> /* For IOCTL macros */

enum {
	AAT1271_PWR_ERR,
	AAT1271_PWR_OFF,
	AAT1271_PWR_STDBY,
	AAT1271_PWR_COMM,
	AAT1271_PWR_ON,
};

#define AAT1271_IOCTL_CAP		_IOWR('o', 1, struct nvodmimager_param)
#define AAT1271_IOCTL_PWR		_IOW('o', 2, __u8)
#define AAT1271_IOCTL_PARAM_RD		_IOWR('o', 3, struct nvodmimager_param)
#define AAT1271_IOCTL_PARAM_WR		_IOW('o', 4, struct nvodmimager_param)

#define nvodmimagerparameter_flashlevel		7
#define nvodmimagerparameter_flashpinstate	8
#define nvodmimagerparameter_torchlevel		10

struct nvodmimager_param {
	int param;
	__s32 sizeofvalue;
	void *p_value;
};

struct nvodmimagerflash_levelinfo {
	__u32 guidenum;
	__u32 sustaintime;
	__u32 rechargefactor;
};

struct nvodmimagerflash_capabilities {
	__u32 numberoflevels;
	struct nvodmimagerflash_levelinfo levels[9];
};

struct nvodmimagerflash_pinstate {
	__u16 mask;
	__u16 values;
};

struct nvodmimagertorch_capabilities {
	__u32 numberoflevels;
	__u32 guidenum[8];
};


#ifdef __KERNEL__
struct aat1271_platform_data {
	int api_pwr;
	int max_amp_torch;
	int max_amp_flash;
	void (*pinstate);
	int (*init)(void);
	void (*exit)(void);
	int (*pm)(int);
	int (*gpio_envm)(int);
	int (*gpio_sync)(int);
};
#endif /* __KERNEL__ */

#endif /* __AAT1271_H__ */

