#include <linux/io.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <linux/jiffies.h>
#include <linux/time.h>
#include <linux/mutex.h>
#include <linux/last_boot.h>

#include <mach/mfootprint.h>
#include <mach/board_htc.h>

#include <asm/mfootprint.h>

#define MAX_STEPS_ORDER (3)
#define MAX_STEPS       (1 << MAX_STEPS_ORDER)
#define FOOTPRINT_MAGIC (0xeeaacc22)
#define NOP             (0xe320f000)
#define BX_LR           (0xe12fff1e)

#define STEP_TIME 0

extern const char *__start__mfoot_entries[];
extern const char *__stop__mfoot_entries[];

#define MFOOT_ENTRIES_SZ \
	(((unsigned long) __stop__mfoot_entries) - ((unsigned long) __start__mfoot_entries))

#define for_each_mfoot_entry(_entry) \
		for (_entry = (unsigned long**) __start__mfoot_entries; \
				((unsigned long) _entry) < ((unsigned long) __stop__mfoot_entries); \
				_entry++)

static unsigned long* backup_mfoot_intr;

struct memory_footprint_step {
	void* callsite1;
	int type;
	void* callsite0;
	unsigned long time;
};

struct memory_footprint {
	struct memory_footprint_step steps[MAX_STEPS];
	struct memory_footprint_step unused[MAX_STEPS];
};

enum mf_class {
	MF_CLASS_IRQ = 0,
	MF_CLASS_INT,
	MF_CLASS_MAX,
};

#define FOOTS_ALIGN (sizeof(struct memory_footprint))
struct memory_footprint_block {
	unsigned int magic;
	struct memory_footprint foots[MF_CLASS_MAX][NR_CPUS]
		__attribute__ ((aligned(FOOTS_ALIGN)));
};

struct memory_footprint_last_step {
	void* callsite1;
	struct memory_footprint_step* step;
};

struct memory_footprint_last_step last_steps[MF_CLASS_MAX][NR_CPUS] __read_mostly;
static struct memory_footprint_block* last_mf = NULL;
static struct memory_footprint_block* mbuf = NULL;

#if STEP_TIME
static unsigned long _pl(unsigned long long clock)
{
	do_div(clock, 1000000000);
	return (unsigned long) clock;
}

static unsigned long _po(unsigned long long clock)
{
	unsigned long nanosec_rem;
	nanosec_rem = do_div(clock, 1000000000);
	return (unsigned long) nanosec_rem / 1000;
}

#define CLOCK_SPLT(c) _pl(c), _po(c)

static unsigned long long jiffies_to_sched_clock(unsigned long jif)
{
	return (unsigned long long)(jif - INITIAL_JIFFIES)
					* (NSEC_PER_SEC / HZ);
}
#endif

static void seq_printf_items(struct seq_file *s, struct memory_footprint* m)
{
	const size_t max_steps = ARRAY_SIZE(m->steps);
	int i;
	int last_step = 0;

	for (i = 0; i < max_steps; i++)
		if (m->steps[i].type == MF_TYPE_ENTER) {
			last_step = i;
			break;
		}

	for (i = 0; i < max_steps; i++) {
		struct memory_footprint_step* step =
			&m->steps[(last_step + 1 + i) % max_steps];
		if (step->callsite0 == NULL)
			continue;
#if STEP_TIME
		seq_printf(s, "    [%5lu.%06lu] %s %pS <- %pS\n",
			CLOCK_SPLT(jiffies_to_sched_clock(step->time)),
			step->type == MF_TYPE_ENTER ? "I/ " : "I/O",
			step->callsite0, step->callsite1);
#else
		seq_printf(s, "    %s %pS <- %pS\n",
			step->type == MF_TYPE_ENTER ? "I/ " : "I/O",
			step->callsite0, step->callsite1);
#endif
	}
}

static void show_last_mf(struct seq_file *s)
{
	/* show_last_mf is registered iff last_mf != NULL */
	int cpu;
	seq_printf(s, "* interrupt\n");
	for_each_possible_cpu(cpu) {
		seq_printf(s, "  CPU#%d\n", cpu);
		seq_printf_items(s, &last_mf->foots[MF_CLASS_INT][cpu]);
	}
	seq_printf(s, "* irq\n");
	for_each_possible_cpu(cpu) {
		seq_printf(s, "  CPU#%d\n", cpu);
		seq_printf_items(s, &last_mf->foots[MF_CLASS_IRQ][cpu]);
	}
}

static bool mfootprint_enable = true;
static DEFINE_MUTEX(entries_mutex);

static void set_mfootprint_enable_locked(bool enable)
{
	static bool mfootprint_current = true;
	unsigned long** entry;
	if (mfootprint_current == enable) return;
	if (enable) {
		int i = 0;
		for_each_mfoot_entry(entry)
			**entry = backup_mfoot_intr[i++];
		mbuf->magic = FOOTPRINT_MAGIC;
	} else {
		for_each_mfoot_entry(entry)
			**entry = NOP;
		mbuf->magic = (unsigned int) NULL;
	}
	mfootprint_current = enable;
}

static void set_mfootprint_enable(bool enable)
{
	if (!mbuf) return;
	mutex_lock(&entries_mutex);
	set_mfootprint_enable_locked(enable);
	mutex_unlock(&entries_mutex);
}

static int mfootprint_enable_set(const char *arg, const struct kernel_param *kp)
{
	int ret = param_set_bool(arg, kp);
	if (ret == 0)
		set_mfootprint_enable(mfootprint_enable);
	return ret;
}

static int mfootprint_enable_get(char *buffer, const struct kernel_param *kp)
{
	return param_get_bool(buffer, kp);
}

static struct kernel_param_ops mfootprint_enable_ops = {
	.set = mfootprint_enable_set,
	.get = mfootprint_enable_get,
};
module_param_cb(enable, &mfootprint_enable_ops, &mfootprint_enable, 0644);

static int memory_footprint_driver_probe(struct platform_device *pdev)
{
	struct resource *res = pdev->resource;
	size_t start = 0;
	size_t buffer_size = 0;

	pr_info("[MF] %s\n", __func__);

	if ((res == NULL) || (pdev->num_resources != 1) ||
		!(res->flags & IORESOURCE_MEM))
	{
		pr_err("[MF] invalid resource, %p, %d flags "
			"%lx\n", res, pdev->num_resources, res ? res->flags : 0);
		return -ENXIO;
	}

	buffer_size = res->end - res->start +1;
	start       = res->start;

	if (buffer_size < sizeof(struct memory_footprint_block)) {
		pr_err("[MF] struct memory_footprint_block "
			   "can't fit in reserve buffer (size=%zx)\n", buffer_size);
		return -ENOMEM;
	}
	pr_info("[MF] got buffer at 0x%zx, size %u\n",
			start, buffer_size);

	mbuf = ioremap(start, buffer_size);
	if (mbuf == NULL) {
		pr_err("[MF] failed to map memory\n");
		return -ENOMEM;
	} else {
		int class, cpu, i;
		unsigned long** entry;

		pr_info("[MF] mapping buffer to 0x%p\n", mbuf);

		if (mbuf->magic == FOOTPRINT_MAGIC) {
			last_mf = kmalloc(sizeof(*last_mf), GFP_KERNEL);

			if (!last_mf)
				pr_info("[MF] last footprint is found but fail to save\n");

			else {
				pr_debug("[MF] last footprint is found; save it to: %p\n", last_mf);
				memcpy(last_mf, mbuf, sizeof(*last_mf));
				add_last_boot_info("Memory footprint", show_last_mf);
			}
		} else
			pr_info("[MF] previous footprint is invalid\n");

		memset(mbuf, 0, buffer_size);
		mbuf->magic = FOOTPRINT_MAGIC;

		memset(last_steps, 0, sizeof(last_steps));
		for (class = 0; class < MF_CLASS_MAX; class++)
			for_each_possible_cpu(cpu)
				last_steps[class][cpu].step = &mbuf->foots[class][cpu].steps[0];

		i = 0;
		backup_mfoot_intr = kmalloc(MFOOT_ENTRIES_SZ, GFP_KERNEL);
		for_each_mfoot_entry(entry)
			backup_mfoot_intr[i++] = **entry;

		/* all prepared */
		dsb();
		* (unsigned long*) __mf_irq_enter = NOP;
		* (unsigned long*) __mf_irq_leave = NOP;
		* (unsigned long*) __mf_int_enter = NOP;
		* (unsigned long*) __mf_int_leave = NOP;
	}
	return 0;
}

EXPORT_SYMBOL(__mf_irq_enter);
EXPORT_SYMBOL(__mf_irq_leave);

static struct platform_driver memory_footprint_driver = {
	.probe = memory_footprint_driver_probe,
	.driver = {
		.name = "memory_footprint",
	},
};

static int __init memory_footprint_module_init(void)
{
	int err;

	/* assert the pre-defined asm const */
	BUILD_BUG_ON(FOOTS_ALIGN & (FOOTS_ALIGN - 1));
	BUILD_BUG_ON((THREAD_SIZE_MASK_TH | THREAD_SIZE_MASK_BH) != (THREAD_SIZE - 1));
	BUILD_BUG_ON(OFFSET_TI_CPU != (unsigned long) &((struct thread_info *)0)->cpu);
	BUILD_BUG_ON((1 << ORDER_LAST_STEPS) != sizeof(struct memory_footprint_last_step));
	BUILD_BUG_ON(OFFSET_LS_STEP != (unsigned long) &((struct memory_footprint_last_step *)0)->step);
	BUILD_BUG_ON(OFFSET_MF_TYPE != (unsigned long) &((struct memory_footprint_step *)0)->type);
	BUILD_BUG_ON(MEMORY_FOOTPRINT_STEP_SZ != sizeof(struct memory_footprint_step));
	BUILD_BUG_ON(MEMORY_FOOTPRINT_SUBMASK != (sizeof(struct memory_footprint) >> 1));
	BUILD_BUG_ON(MF_CLASS_IRQ_SHIFT != (MF_CLASS_IRQ << 2));
	BUILD_BUG_ON(MF_CLASS_INT_SHIFT != (MF_CLASS_INT << 2));

	pr_info("[MF] %s\n", __func__);

	err = platform_driver_register(&memory_footprint_driver);
	return err;
}
postcore_initcall(memory_footprint_module_init);
