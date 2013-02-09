#ifdef DEBUG
#define pr_tag_fmt(level, tag, fmt, ...) \
		printk (level "[" tag "] " KBUILD_MODNAME ":%s():%d: " fmt, \
				__func__, __LINE__, ##__VA_ARGS__);
#else
#define pr_tag_fmt(level, tag, fmt, ...) \
		printk (level "[" tag "] " fmt, ##__VA_ARGS__);
#endif

#define pr_tag_emerg(tag, fmt, ...) \
		pr_tag_fmt(KERN_EMERG, tag, fmt, ##__VA_ARGS__);
#define pr_tag_alert(tag, fmt, ...) \
		pr_tag_fmt(KERN_ALERT, tag, fmt, ##__VA_ARGS__);
#define pr_tag_crit(tag, fmt, ...) \
		pr_tag_fmt(KERN_CRIT, tag, fmt, ##__VA_ARGS__);
#define pr_tag_err(tag, fmt, ...) \
		pr_tag_fmt(KERN_ERR, tag, fmt, ##__VA_ARGS__);
#define pr_tag_warn(tag, fmt, ...) \
		pr_tag_fmt(KERN_WARNING, tag, fmt, ##__VA_ARGS__);
#define pr_tag_info(tag, fmt, ...) \
		pr_tag_fmt(KERN_INFO, tag, fmt, ##__VA_ARGS__);
#define pr_tag_cont(tag, fmt, ...) \
		pr_tag_fmt(KERN_CONT, tag, fmt, ##__VA_ARGS__);

#ifdef DEBUG
#define pr_tag_dbg(tag, fmt, ...) \
	pr_tag_fmt(KERN_DEBUG, tag, fmt, ##__VA_ARGS__);
#else
#define pr_tag_dbg(tag, fmt, ...) \
    ({ if (0) pr_tag_fmt(KERN_DEBUG, tag, fmt, ##__VA_ARGS__); 0; })
#endif

#define AUD_ERR(fmt, ...) pr_tag_err(LOG_TAG, fmt, ##__VA_ARGS__)
#define AUD_INFO(fmt, ...) pr_tag_info(LOG_TAG, fmt, ##__VA_ARGS__)
