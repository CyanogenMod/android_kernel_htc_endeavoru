#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
/* printk() */

MODULE_LICENSE("GPL");

/* parameter */
static unsigned int wakeup_after;


static int wakeup_delay_time = 0;




static int wakeup_delay_time_set(const char *arg, const struct kernel_param *kp)
{
	int ret = param_set_int(arg, kp);
	return ret;
}

static int wakeup_delay_time_get(char *buffer, const struct kernel_param *kp)
{
	int ret = param_get_int(buffer, kp);
	return param_get_int(buffer, kp);
}




static struct kernel_param_ops tegra_power_wakeup_after_ops = {
	.set = wakeup_delay_time_set,
	.get = wakeup_delay_time_get,
};
module_param_cb(wakeup_after, &tegra_power_wakeup_after_ops, &wakeup_delay_time, 0644);


static int __init power_init(void)
{
	return 0;
}




static void __exit power_exit(void)
{
}

module_init(power_init);
module_exit(power_exit);

