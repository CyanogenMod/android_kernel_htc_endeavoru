/*
 * Copyright (C) 2013 HTC, Inc.
 */

#ifndef _LAST_BOOT_H_
#define _LAST_BOOT_H_

struct seq_file;

#if defined(CONFIG_LAST_BOOT_DEBUG)
int add_last_boot_info(const char*, void (*)(struct seq_file *));
int last_boot_show(struct seq_file *);

#else
static inline int add_last_boot_info(const char* c, void (*s)(struct seq_file *))
{
	return 0;
}
static inline int last_boot_show(struct seq_file *s)
{
	return 0;
}

#endif //CONFIG_LAST_BOOT_DEBUG

#endif //_LAST_BOOT_H_
