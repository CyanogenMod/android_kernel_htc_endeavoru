/*
 * drivers/base/cpu.c - basic CPU class support
 */

#include <linux/sysdev.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/cpu.h>
#include <linux/topology.h>
#include <linux/device.h>
#include <linux/node.h>
#include <linux/gfp.h>
#include <linux/delay.h>

#include "base.h"

static struct workqueue_struct *cpuplug_wq;
static struct work_struct cpuplug_work;

static struct sysdev_class_attribute *cpu_sysdev_class_attrs[];

struct sysdev_class cpu_sysdev_class = {
	.name = "cpu",
	.attrs = cpu_sysdev_class_attrs,
};
EXPORT_SYMBOL(cpu_sysdev_class);

static DEFINE_PER_CPU(struct sys_device *, cpu_sys_devices);

#ifdef CONFIG_HOTPLUG_CPU

static int target_number_of_online_cpus = 0;
static int cpu_on_mdelay = 0;

static void tegra_cpuplug_work_func(struct work_struct *work)
{
	unsigned int cpu;
	int should_on_cpu;

	should_on_cpu = target_number_of_online_cpus - num_online_cpus();

	pr_info("[cpu] should on %d of cpus\n", should_on_cpu);

	if(should_on_cpu <= 0)
		return;

	for (cpu = 0; cpu < 4, should_on_cpu != 0; cpu++) {
		if (!cpu_online(cpu)) {
			cpu_up(cpu);
			pr_info("[cpu] cpu %d is on", cpu);
			should_on_cpu--;
			mdelay(cpu_on_mdelay);
		}
	}
}

static ssize_t show_cpu_on(struct sysdev_class *class,
			struct sysdev_class_attribute *attr, char *buf)
{
	unsigned int cpu = num_online_cpus();

	return sprintf(buf, "%u\n", cpu);
}

static ssize_t store_cpu_on(struct sysdev_class *class,
			 struct sysdev_class_attribute *attr,
			 const char *buf,
			 size_t count)
{
	strict_strtol(buf, 0, &target_number_of_online_cpus );

	if(target_number_of_online_cpus < 2)
		return;

	if(target_number_of_online_cpus > 4)
		target_number_of_online_cpus  = 4;

	pr_info("[cpu] number of online cpus is %d",
			target_number_of_online_cpus );

	return queue_work(cpuplug_wq, &cpuplug_work);
}


static ssize_t show_cpu_on_mdelay(struct sysdev_class *class,
			struct sysdev_class_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", cpu_on_mdelay);
}

static ssize_t store_cpu_on_mdelay(struct sysdev_class *class,
			 struct sysdev_class_attribute *attr,
			 const char *buf,
			 size_t count)
{
	strict_strtol(buf, 0, &cpu_on_mdelay);
}


static SYSDEV_CLASS_ATTR(cpu_on, 0644, show_cpu_on, store_cpu_on);
static SYSDEV_CLASS_ATTR(cpu_on_mdelay, 0644,
		show_cpu_on_mdelay, store_cpu_on_mdelay);

static ssize_t show_online(struct sys_device *dev, struct sysdev_attribute *attr,
			   char *buf)
{
	struct cpu *cpu = container_of(dev, struct cpu, sysdev);

	return sprintf(buf, "%u\n", !!cpu_online(cpu->sysdev.id));
}

static ssize_t __ref store_online(struct sys_device *dev, struct sysdev_attribute *attr,
				 const char *buf, size_t count)
{
	struct cpu *cpu = container_of(dev, struct cpu, sysdev);
	ssize_t ret;

	cpu_hotplug_driver_lock();
	switch (buf[0]) {
	case '0':
		ret = cpu_down(cpu->sysdev.id);
		if (!ret)
			kobject_uevent(&dev->kobj, KOBJ_OFFLINE);
		break;
	case '1':
		ret = cpu_up(cpu->sysdev.id);
		if (!ret)
			kobject_uevent(&dev->kobj, KOBJ_ONLINE);
		break;
	default:
		ret = -EINVAL;
	}
	cpu_hotplug_driver_unlock();

	if (ret >= 0)
		ret = count;
	return ret;
}
static SYSDEV_ATTR(online, 0644, show_online, store_online);

static void __cpuinit register_cpu_control(struct cpu *cpu)
{
	sysdev_create_file(&cpu->sysdev, &attr_online);
}
void unregister_cpu(struct cpu *cpu)
{
	int logical_cpu = cpu->sysdev.id;

	unregister_cpu_under_node(logical_cpu, cpu_to_node(logical_cpu));

	sysdev_remove_file(&cpu->sysdev, &attr_online);

	sysdev_unregister(&cpu->sysdev);
	per_cpu(cpu_sys_devices, logical_cpu) = NULL;
	return;
}

#ifdef CONFIG_ARCH_CPU_PROBE_RELEASE
static ssize_t cpu_probe_store(struct sysdev_class *class,
			       struct sysdev_class_attribute *attr,
			       const char *buf,
			       size_t count)
{
	return arch_cpu_probe(buf, count);
}

static ssize_t cpu_release_store(struct sysdev_class *class,
				 struct sysdev_class_attribute *attr,
				 const char *buf,
				 size_t count)
{
	return arch_cpu_release(buf, count);
}

static SYSDEV_CLASS_ATTR(probe, S_IWUSR, NULL, cpu_probe_store);
static SYSDEV_CLASS_ATTR(release, S_IWUSR, NULL, cpu_release_store);
#endif /* CONFIG_ARCH_CPU_PROBE_RELEASE */

#else /* ... !CONFIG_HOTPLUG_CPU */
static inline void register_cpu_control(struct cpu *cpu)
{
}
#endif /* CONFIG_HOTPLUG_CPU */

#ifdef CONFIG_KEXEC
#include <linux/kexec.h>

static ssize_t show_crash_notes(struct sys_device *dev, struct sysdev_attribute *attr,
				char *buf)
{
	struct cpu *cpu = container_of(dev, struct cpu, sysdev);
	ssize_t rc;
	unsigned long long addr;
	int cpunum;

	cpunum = cpu->sysdev.id;

	/*
	 * Might be reading other cpu's data based on which cpu read thread
	 * has been scheduled. But cpu data (memory) is allocated once during
	 * boot up and this data does not change there after. Hence this
	 * operation should be safe. No locking required.
	 */
	addr = per_cpu_ptr_to_phys(per_cpu_ptr(crash_notes, cpunum));
	rc = sprintf(buf, "%Lx\n", addr);
	return rc;
}
static SYSDEV_ATTR(crash_notes, 0400, show_crash_notes, NULL);
#endif

/*
 * Print cpu online, possible, present, and system maps
 */

struct cpu_attr {
	struct sysdev_class_attribute attr;
	const struct cpumask *const * const map;
};

static ssize_t show_cpus_attr(struct sysdev_class *class,
			      struct sysdev_class_attribute *attr,
			      char *buf)
{
	struct cpu_attr *ca = container_of(attr, struct cpu_attr, attr);
	int n = cpulist_scnprintf(buf, PAGE_SIZE-2, *(ca->map));

	buf[n++] = '\n';
	buf[n] = '\0';
	return n;
}

#define _CPU_ATTR(name, map)						\
	{ _SYSDEV_CLASS_ATTR(name, 0444, show_cpus_attr, NULL), map }

/* Keep in sync with cpu_sysdev_class_attrs */
static struct cpu_attr cpu_attrs[] = {
	_CPU_ATTR(online, &cpu_online_mask),
	_CPU_ATTR(possible, &cpu_possible_mask),
	_CPU_ATTR(present, &cpu_present_mask),
};

/*
 * Print values for NR_CPUS and offlined cpus
 */
static ssize_t print_cpus_kernel_max(struct sysdev_class *class,
				     struct sysdev_class_attribute *attr, char *buf)
{
	int n = snprintf(buf, PAGE_SIZE-2, "%d\n", NR_CPUS - 1);
	return n;
}
static SYSDEV_CLASS_ATTR(kernel_max, 0444, print_cpus_kernel_max, NULL);

/* arch-optional setting to enable display of offline cpus >= nr_cpu_ids */
unsigned int total_cpus;

static ssize_t print_cpus_offline(struct sysdev_class *class,
				  struct sysdev_class_attribute *attr, char *buf)
{
	int n = 0, len = PAGE_SIZE-2;
	cpumask_var_t offline;

	/* display offline cpus < nr_cpu_ids */
	if (!alloc_cpumask_var(&offline, GFP_KERNEL))
		return -ENOMEM;
	cpumask_andnot(offline, cpu_possible_mask, cpu_online_mask);
	n = cpulist_scnprintf(buf, len, offline);
	free_cpumask_var(offline);

	/* display offline cpus >= nr_cpu_ids */
	if (total_cpus && nr_cpu_ids < total_cpus) {
		if (n && n < len)
			buf[n++] = ',';

		if (nr_cpu_ids == total_cpus-1)
			n += snprintf(&buf[n], len - n, "%d", nr_cpu_ids);
		else
			n += snprintf(&buf[n], len - n, "%d-%d",
						      nr_cpu_ids, total_cpus-1);
	}

	n += snprintf(&buf[n], len - n, "\n");
	return n;
}
static SYSDEV_CLASS_ATTR(offline, 0444, print_cpus_offline, NULL);

/*
 * register_cpu - Setup a sysfs device for a CPU.
 * @cpu - cpu->hotpluggable field set to 1 will generate a control file in
 *	  sysfs for this CPU.
 * @num - CPU number to use when creating the device.
 *
 * Initialize and register the CPU device.
 */
int __cpuinit register_cpu(struct cpu *cpu, int num)
{
	int error;
	cpu->node_id = cpu_to_node(num);
	cpu->sysdev.id = num;
	cpu->sysdev.cls = &cpu_sysdev_class;

	error = sysdev_register(&cpu->sysdev);

	if (!error && cpu->hotpluggable)
		register_cpu_control(cpu);
	if (!error)
		per_cpu(cpu_sys_devices, num) = &cpu->sysdev;
	if (!error)
		register_cpu_under_node(num, cpu_to_node(num));

#ifdef CONFIG_KEXEC
	if (!error)
		error = sysdev_create_file(&cpu->sysdev, &attr_crash_notes);
#endif
	return error;
}

struct sys_device *get_cpu_sysdev(unsigned cpu)
{
	if (cpu < nr_cpu_ids && cpu_possible(cpu))
		return per_cpu(cpu_sys_devices, cpu);
	else
		return NULL;
}
EXPORT_SYMBOL_GPL(get_cpu_sysdev);

int __init cpu_dev_init(void)
{
	int err;

	err = sysdev_class_register(&cpu_sysdev_class);
#if defined(CONFIG_SCHED_MC) || defined(CONFIG_SCHED_SMT)
	if (!err)
		err = sched_create_sysfs_power_savings_entries(&cpu_sysdev_class);
#endif

	cpuplug_wq = alloc_workqueue(
                "cpu-plug", WQ_UNBOUND | WQ_RESCUER | WQ_FREEZABLE, 1);
        if (!cpuplug_wq)
                return -ENOMEM;
        INIT_WORK(&cpuplug_work, tegra_cpuplug_work_func);

	return err;
}

static struct sysdev_class_attribute *cpu_sysdev_class_attrs[] = {
#ifdef CONFIG_ARCH_CPU_PROBE_RELEASE
	&attr_probe,
	&attr_release,
#endif
	&cpu_attrs[0].attr,
	&cpu_attrs[1].attr,
	&cpu_attrs[2].attr,
	&attr_kernel_max,
	&attr_offline,
	&attr_cpu_on,
	&attr_cpu_on_mdelay,
	NULL
};
