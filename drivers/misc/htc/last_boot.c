#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

static struct list_head lboot_list = LIST_HEAD_INIT(lboot_list);
static DEFINE_SPINLOCK(lboot_lock); /* protect the lboot_list */

struct seq_file_item
{
	struct list_head list;
	char name[32];
	void (*show)(struct seq_file *);
};

int add_last_boot_info(const char* caption, void (*show)(struct seq_file *))
{
	unsigned long flags;
	struct seq_file_item *item;

	item = kmalloc(sizeof(*item), GFP_KERNEL);
	if (!item)
		return -ENOMEM;

	memset(item, 0, sizeof(*item));

	item->show = show;
	strncpy(item->name, caption, sizeof(item->name));

	spin_lock_irqsave(&lboot_lock, flags);
	list_add_tail(&item->list, &lboot_list);
	spin_unlock_irqrestore(&lboot_lock, flags);

	return 0;
}

static int lboot_show(struct seq_file *s, void *unused)
{
	unsigned long flags;
	struct seq_file_item *item;
	spin_lock_irqsave(&lboot_lock, flags);
	list_for_each_entry(item, &lboot_list, list) {
		seq_printf(s, "=== %s ===\n", item->name);
		item->show(s);
		seq_printf(s, "\n");
	}
	spin_unlock_irqrestore(&lboot_lock, flags);
	return 0;
}

static int lboot_open(struct inode *inode, struct file *file)
{
	return single_open(file, lboot_show, NULL);
}

static const struct file_operations lboot_file_ops = {
	.open		= lboot_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init lboot_late_init(void)
{
	struct proc_dir_entry *entry;
	entry = create_proc_entry("last_boot", S_IFREG | S_IRUGO, NULL);
	if (!entry) {
		printk(KERN_ERR "last_boot: failed to create proc entry\n");
		return 0;
	}

	entry->proc_fops = &lboot_file_ops;
	entry->size = 1; // TODO: make sure the usage of size here is correct
	return 0;
}
late_initcall(lboot_late_init);

int last_boot_show(struct seq_file *s)
{
	return lboot_show(s, NULL);
}
