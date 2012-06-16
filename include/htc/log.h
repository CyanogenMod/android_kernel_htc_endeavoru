#ifndef __HTC_LOG_H
#define __HTC_LOG_H

//#define CONFIG_SPIN_LOCK_PRINT
//#undef CONFIG_SPIN_LOCK_PRINT
#ifdef CONFIG_SPIN_LOCK_PRINT
#define PM_SP_FORMATION "[SP] "
#define sp_pr_emerg(fmt, ...)    printk( KERN_EMERG   pr_fmt( PM_SP_FORMATION fmt), ##__VA_ARGS__)
#define sp_pr_alert(fmt, ...)    printk( KERN_ALERT   pr_fmt( PM_SP_FORMATION fmt), ##__VA_ARGS__)
#define sp_pr_crit(fmt, ...)     printk( KERN_CRIT    pr_fmt( PM_SP_FORMATION fmt), ##__VA_ARGS__)
#define sp_pr_err(fmt, ...)      printk( KERN_ERR     pr_fmt( PM_SP_FORMATION fmt), ##__VA_ARGS__)
#define sp_pr_warning(fmt, ...)  printk( KERN_WARNING pr_fmt( PM_SP_FORMATION fmt), ##__VA_ARGS__)
#define sp_pr_warn               pm_pr_warning
#define sp_pr_notice(fmt, ...)   printk( KERN_NOTICE  pr_fmt( PM_SP_FORMATION fmt), ##__VA_ARGS__)
#define sp_pr_info(fmt, ...)     printk( KERN_INFO    pr_fmt( PM_SP_FORMATION fmt), ##__VA_ARGS__)
#define sp_pr_debug(fmt, ...)     printk( KERN_INFO    pr_fmt( PM_SP_FORMATION fmt), ##__VA_ARGS__)
#else
#define sp_pr_emerg(fmt, ...) do {;} while(0)
#define sp_pr_alert(fmt, ...) do {;} while(0)
#define sp_pr_crit(fmt, ...) do {;} while(0)
#define sp_pr_err(fmt, ...) do {;} while(0) 
#define sp_pr_warning(fmt, ...) do {;} while(0)
#define sp_pr_warn               pm_pr_warning
#define sp_pr_notice(fmt, ...) do {;} while(0)
#define sp_pr_info(fmt, ...) do {;} while(0)
#define sp_pr_debug(fmt, ...) do {;} while(0)
#endif


#ifdef CONFIG_PM_R_DEBUG
/**
 * Power Manager [R] log
 * use defined function to add [R] header.
 */

#define __PM_R_DEBUG_2_INFO
#define PM_R_FORMATION "[R] "

#define pm_pr_emerg(fmt, ...)    printk( KERN_EMERG   pr_fmt( PM_R_FORMATION fmt), ##__VA_ARGS__)
#define pm_pr_alert(fmt, ...)    printk( KERN_ALERT   pr_fmt( PM_R_FORMATION fmt), ##__VA_ARGS__)
#define pm_pr_crit(fmt, ...)     printk( KERN_CRIT    pr_fmt( PM_R_FORMATION fmt), ##__VA_ARGS__)
#define pm_pr_err(fmt, ...)      printk( KERN_ERR     pr_fmt( PM_R_FORMATION fmt), ##__VA_ARGS__)
#define pm_pr_warning(fmt, ...)  printk( KERN_WARNING pr_fmt( PM_R_FORMATION fmt), ##__VA_ARGS__)
#define pm_pr_warn               pm_pr_warning
#define pm_pr_notice(fmt, ...)   printk( KERN_NOTICE  pr_fmt( PM_R_FORMATION fmt), ##__VA_ARGS__)
#define pm_pr_info(fmt, ...)     printk( KERN_INFO    pr_fmt( PM_R_FORMATION fmt), ##__VA_ARGS__)
#ifdef __PM_R_DEBUG_2_INFO
#define pm_pr_debug(fmt, ...)    printk( KERN_INFO    pr_fmt( PM_R_FORMATION fmt), ##__VA_ARGS__)
#else 
#define pm_pr_debug(fmt, ...)    printk( KERN_CONT    pr_fmt( PM_R_FORMATION fmt), ##__VA_ARGS__)
#endif

// for PM
#define pmr_pr_info(fmt, ...)    printk(KERN_INFO     pr_fmt(                fmt), ##__VA_ARGS__)

#else // CONFIG_PM_R_DEBUG

#define pm_pr_emerg(fmt, ...) do {;} while(0)
#define pm_pr_alert(fmt, ...) do {;} while(0)
#define pm_pr_crit(fmt, ...) do {;} while(0)
#define pm_pr_err(fmt, ...) do {;} while(0) 
#define pm_pr_warning(fmt, ...) do {;} while(0)
#define pm_pr_warn               pm_pr_warning
#define pm_pr_notice(fmt, ...) do {;} while(0)
#define pm_pr_info(fmt, ...) do {;} while(0)
#define pm_pr_debug(fmt, ...) do {;} while(0)
#define pm_pr_debug(fmt, ...) do {;} while(0)
#define pmr_pr_info(fmt, ...) do {;} while(0)

#endif // CONFIG_PM_R_DEBUG
/**
 * 
 * 
 */

#endif // __HTC_LOG_H
