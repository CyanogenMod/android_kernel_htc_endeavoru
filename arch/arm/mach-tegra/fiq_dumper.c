/*
 * arch/arm/mach-tegra/fiq_dumper.c
 *
 * Copyright (C) 2012 HTC Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/serial_reg.h>
#include <linux/slab.h>
#include <linux/stacktrace.h>
#include <linux/seq_file.h>
#include <asm/fiq_debugger.h>
#include <mach/system.h>
#include <mach/fiq.h>

#include <linux/uaccess.h>
#include <linux/last_boot.h>

#define DUMP_BUFFER_MAGIC (0x9922aaee)

static unsigned int* dump_buffer_head = NULL;
static char* dump_buffer_end = NULL;
static char* dump_buffer_pos = NULL;

static int debug_getc(struct platform_device *pdev)
{
	return FIQ_DEBUGGER_NO_CHAR;
}

static void _putc(char c)
{
	// write c to buffer in space remains
	if (dump_buffer_pos < dump_buffer_end)
		*dump_buffer_pos++ = c;
}

static void _puts(const char* s)
{
	int i;
	for (i = 0; i < strlen(s); i++)
		_putc(s[i]);
}

static void debug_putc(struct platform_device *pdev, unsigned int c)
{
	if (c != 0x0d) // skip ^M
		_putc(c);
}

static void debug_flush(struct platform_device *pdev)
{
	// nop
}

static void fiq_enable(struct platform_device *pdev, unsigned int irq, bool on)
{
	if (on)
		tegra_fiq_enable(irq);
	else
		tegra_fiq_disable(irq);
}

static void fiq_ack(struct platform_device *pdev, unsigned int fiq)
{
	char buffer[64];
	sprintf(buffer, "current cpu: %d (online: %d%d%d%d)",
		    smp_processor_id(),
		    cpu_online(0), cpu_online(1), cpu_online(2), cpu_online(3));
	_puts(buffer);
	*dump_buffer_head = DUMP_BUFFER_MAGIC;
	while (1); /* no return */
}

static struct fiq_debugger_pdata fiq_debugger_pdata = {
	.uart_getc = debug_getc,
	.uart_putc = debug_putc,
	.uart_flush = debug_flush,
	.fiq_ack = fiq_ack,
	.fiq_enable = fiq_enable,
};

static struct resource fiq_debugger_resource[] = {
	{
		.name  = "fiq",
		.flags = IORESOURCE_IRQ,
		.start = INT_WDT_CPU,
		.end   = INT_WDT_CPU,
	},
};

static struct platform_device fiq_debugger_device = {
	.name = "fiq_debugger",
	.id = -1,
	.resource = fiq_debugger_resource,
	.num_resources = ARRAY_SIZE(fiq_debugger_resource),
	.dev		= {
		.platform_data = &fiq_debugger_pdata,
	},
};

static char* saved_dump_buffer = NULL;
static void show_last_dump(struct seq_file *s)
{
	seq_printf(s, "%s\n", saved_dump_buffer);
}

static int fiq_dumper_driver_probe(struct platform_device *pdev)
{
	struct resource *res = pdev->resource;
	size_t buffer_size;
	int ret;

	if ((res == NULL) || (pdev->num_resources != 1) ||
		!(res->flags & IORESOURCE_MEM))
	{
		pr_err("fiq_dumper: invalid resource, %p, %d flags "
			"%lx\n", res, pdev->num_resources, res ? res->flags : 0);
		return -ENXIO;
	}

	buffer_size = res->end - res->start + 1;

	if (!request_mem_region(res->start, res->end - res->start + 1, pdev->name)) {
		pr_err("fiq_dumper: request mem region failed\n");
		return -ENOMEM;
	}

	dump_buffer_head = ioremap(res->start, buffer_size);
	if (!dump_buffer_head) {
		pr_err("fiq_dumper: failed to map memory\n");
		return -ENOMEM;
	}

	/* subtract size of magic word from buffer head */
	buffer_size -= sizeof(unsigned int);
	dump_buffer_pos = ((char*) dump_buffer_head) + sizeof(unsigned int);
	dump_buffer_end = ((char*) dump_buffer_pos) + buffer_size - 1;

	pr_info("fiq dumper: dump_buffer: virt: 0x%p\n", (void*) dump_buffer_pos);
	if (*dump_buffer_head == DUMP_BUFFER_MAGIC) {
		saved_dump_buffer = kmalloc(buffer_size, GFP_KERNEL);
		if (!saved_dump_buffer)
			pr_err("fiq_dumper: failed to alloc for saving old dump_buffer\n");
		else {
			memcpy(saved_dump_buffer, dump_buffer_pos, buffer_size);
			((char*) saved_dump_buffer)[buffer_size - 1] = 0;
			add_last_boot_info("FIQ dumper", show_last_dump);
		}
	}
	*dump_buffer_head = 0;
	memset(dump_buffer_pos, 0, buffer_size);

	ret = platform_device_register(&fiq_debugger_device);
	if (ret) {
		pr_err("fiq_dumper: failed to register device (error=%d)\n", ret);
		return ret;
	}

	return 0;
}

static struct platform_driver fiq_dumper_driver = {
	.probe = fiq_dumper_driver_probe,
	.driver = {
		.name = "fiq_dumper",
	},
};

static int __init fiq_dumper_init(void)
{
    return platform_driver_register(&fiq_dumper_driver);
}
postcore_initcall(fiq_dumper_init);
