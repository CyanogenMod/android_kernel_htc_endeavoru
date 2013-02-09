/*
 * arch/arm/mach-tegra/nvdumper.c
 *
 * Copyright (C) 2011 NVIDIA Corporation
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/errno.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/syscore_ops.h>
#include <linux/memblock.h>
#include <linux/slab.h>

#include "board.h"

#define NVDUMPER_CLEAN 0xf000caf3U
#define NVDUMPER_DIRTY 0xdeadbeefU
#define NVDUMPER_RESERVED_LEN 4096

static uint32_t *nvdumper_ptr = NULL;
static unsigned long nvdumper_reserved = 0;
static void* reboot_params = NULL;
static void* last_reboot_params = NULL;

void* get_last_reboot_params()
{
	return last_reboot_params;
}

void* get_reboot_params()
{
	return reboot_params;
}

static int __init tegra_nvdumper_arg(char *options)
{
	char *p = options;

	nvdumper_reserved = memparse(p, &p);
	return 0;
}
early_param("nvdumper_reserved", tegra_nvdumper_arg);

/*
 * NOTE:
 *     nvdumper reserves one page at nvdumper_reserved, but htc_reboot_params
 *     uses previous one page from nvdumper_reserved.
 */

void nvdumper_reserve_init(void)
{
	unsigned long reserve_start, reserve_len;

	if (!nvdumper_reserved)
		return;

	reserve_start = nvdumper_reserved - NVDUMPER_RESERVED_LEN;
	reserve_len   = 2 * NVDUMPER_RESERVED_LEN;

	if (memblock_reserve(nvdumper_reserved, NVDUMPER_RESERVED_LEN)) {
		pr_err("Failed to reserve nvdumper page %08lx@%08lx\n",
			   reserve_start, reserve_len);
		nvdumper_reserved = 0;
		return;
	}

	pr_info("Nvdumper reserved: %08lx - %08lx\n",
		reserve_start, reserve_len);
}

static int get_dirty_state(void)
{
	uint32_t val;

	val = ioread32(nvdumper_ptr);
	if (val == NVDUMPER_DIRTY)
		return 1;
	else if (val == NVDUMPER_CLEAN)
		return 0;
	else
		return -1;
}

static void set_dirty_state(int dirty)
{
	if (dirty)
		iowrite32(NVDUMPER_DIRTY, nvdumper_ptr);
	else
		iowrite32(NVDUMPER_CLEAN, nvdumper_ptr);
}

static	void nvdumper_syscore_shutdown(void)
{
	printk(KERN_INFO "nvdumper: reboot / shutdown cleanly.\n");
	set_dirty_state(0);
}

static struct syscore_ops nvdumper_syscore_ops = {
	.shutdown = nvdumper_syscore_shutdown,
};

static int __init nvdumper_init(void)
{
	int dirty;

	printk(KERN_INFO "nvdumper: nvdumper_reserved: phys: 0x%p\n", (void*) nvdumper_reserved);
	if (!nvdumper_reserved) {
		printk(KERN_INFO "nvdumper: not configured\n");
		return -ENOTSUPP;
	}
	nvdumper_ptr = ioremap_nocache(nvdumper_reserved,
			NVDUMPER_RESERVED_LEN);
	printk(KERN_INFO "nvdumper: nvdumper_ptr: virt: 0x%p\n", (void*) nvdumper_ptr);
	if (!nvdumper_ptr) {
		printk(KERN_INFO "nvdumper: failed to ioremap memory "
			"at 0x%08lx\n", nvdumper_reserved);
		BUG();
		return -EIO;
	}
	reboot_params = ioremap_nocache(nvdumper_reserved - NVDUMPER_RESERVED_LEN,
			NVDUMPER_RESERVED_LEN);
	printk(KERN_INFO "nvdumper: reboot_params: virt: 0x%p\n", reboot_params);
	if (!reboot_params) {
		printk(KERN_INFO "nvdumper: failed to ioremap memory "
			"at 0x%08lx\n", nvdumper_reserved - NVDUMPER_RESERVED_LEN);
		BUG();
		return -EIO;
	}
	last_reboot_params = (void*) kzalloc(NVDUMPER_RESERVED_LEN, GFP_KERNEL);
	if (!last_reboot_params) {
		printk(KERN_INFO "nvdumper: failed to save last_reboot_params - out of memory\n");
		BUG();
		return -ENOMEM;
	}
	memcpy(last_reboot_params, reboot_params, NVDUMPER_RESERVED_LEN);
	memset(reboot_params, 0x0, NVDUMPER_RESERVED_LEN);
	register_syscore_ops(&nvdumper_syscore_ops);

	dirty = get_dirty_state();
	printk(KERN_INFO "nvdumper: dirty:%d\n", dirty);
	switch (dirty) {
	case 0:
		printk(KERN_INFO "nvdumper: last reboot was clean\n");
		break;
	case 1:
		printk(KERN_INFO "nvdumper: last reboot was dirty\n");
		break;
	default:
		printk(KERN_INFO "nvdumper: last reboot was unknown\n");
		break;
	}

	/*
	 * default dirty
	 */
	set_dirty_state(1);
	return 0;
}

static int __exit nvdumper_exit(void)
{
	unregister_syscore_ops(&nvdumper_syscore_ops);
	set_dirty_state(0);
	iounmap(nvdumper_ptr);
	return 0;
}

module_init(nvdumper_init);
module_exit(nvdumper_exit);

MODULE_LICENSE("GPL");
