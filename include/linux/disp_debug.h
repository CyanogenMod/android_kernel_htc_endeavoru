#ifndef DISP_DEBUG_H
#define DISP_DEBUG_H


#define DISP_DEBUG_ENABLE 0

#define DISP_EMERG(fmt, args...) 		printk(KERN_EMERG "[DISP]"fmt, ##args);
#define DISP_ALERT(fmt, args...) 		printk(KERN_ALERT "[DISP]"fmt, ##args);
#define DISP_CRIT(fmt, args...)			printk(KERN_CRIT "[DISP]"fmt, ##args);
#define DISP_ERR(fmt, args...)			printk(KERN_ERR "[DISP]"fmt, ##args);
#define DISP_WARN(fmt, args...)			printk(KERN_WARNING "[DISP]"fmt, ##args);
#define DISP_NOTICE(fmt, args...)		printk(KERN_NOTICE "[DISP]"fmt, ##args);
#define DISP_INFO(fmt, args...)			printk(KERN_INFO "[DISP]"fmt, ##args)
#define DISP_CONT(fmt, args...)			printk(KERN_CONT "[DISP]"fmt, ##args);
#define DISP_DEBUG(fmt, args...) 		printk(KERN_DEBUG "[DISP]"fmt, ##args);

#define DISP_DEBUG_LN(fmt, args...) \
	printk(KERN_DEBUG "[DISP]%s(%d) "fmt, __FUNCTION__, __LINE__, ##args);
#define DISP_INFO_LN(fmt, args...) \
	printk(KERN_INFO "[DISP]%s(%d) "fmt, __FUNCTION__, __LINE__, ##args);
#define DISP_INFO_IN(fmt, args...) \
	printk(KERN_INFO "[DISP]%s(%d) "fmt" IN", __FUNCTION__, __LINE__, ##args);
#define DISP_INFO_OUT(fmt, args...) \
	printk(KERN_INFO "[DISP]%s(%d) "fmt" OUT", __FUNCTION__, __LINE__, ##args);

#define GPIO_GET(pin, label) \
		err = gpio_request(pin, label); \
		if (err < 0) { \
			printk(KERN_ERR "[DISP][ERR] %s(%d) %s Request Failed\r\n",__func__, __LINE__, label); \
			gpio_free(pin); \
		} \

#define GPIO_OUTPUT(pin, label, value) \
		GPIO_GET(pin, label) \
		err = gpio_direction_output(pin, value); \
		if (err < 0) { \
			printk(KERN_ERR "[DISP][ERR] %s(%d) %s Output Failed\r\n", __func__, __LINE__, label); \
			gpio_free(pin); \
		} else \
		tegra_gpio_enable(pin);

#define GPIO_INPUT(pin, label) \
		GPIO_GET(pin, label) \
		err = gpio_direction_input(pin); \
		if (err < 0) { \
			printk(KERN_ERR "[DISP][ERR] %s(%d) %s Input Failed\r\n", __func__, __LINE__, label); \
			gpio_free(pin); \
		} else \
		tegra_gpio_enable(pin);

#define GPIO_KEEP_LOW(pin, label) GPIO_OUTPUT(pin, label, 0)

#define REGULATOR_GET(reg, name) \
	if (reg == NULL) { \
		reg = regulator_get(NULL, name); \
		if (IS_ERR_OR_NULL(reg)) { \
			printk(KERN_ERR "[DISP][ERR] %s(%d) Could not get regulator %d\r\n", __func__, __LINE__, name); \
			reg = NULL; \
			goto failed; \
		} \
	}

#define disp_i2c_lock() \
	switch (pcbid) { \
    case PROJECT_PHASE_XB: \
    case PROJECT_PHASE_XA: \
		mutex_lock(&i2c_lock); \
		/*printk("[DISP] %s(%d) Lock %p +\n", __FUNCTION__, __LINE__, &i2c_lock);*/ \
		break; \
	default: \
		break; \
    }

#define disp_i2c_unlock() \
	switch (pcbid) { \
    case PROJECT_PHASE_XB: \
    case PROJECT_PHASE_XA: \
		mutex_unlock(&i2c_lock); \
		/*printk("[DISP] %s(%d) UnLock %p -\n", __FUNCTION__, __LINE__, &i2c_lock);*/ \
		break; \
	default: \
		break; \
    }

#endif
