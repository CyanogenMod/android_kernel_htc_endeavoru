#ifndef __MAC80211_DEBUGFS_H
#define __MAC80211_DEBUGFS_H

#ifdef CONFIG_MAC80211_DEBUGFS
extern void debugfs_hw_add(struct ieee80211_local *local);
//HTC_WIFI_START
//HTC_WIFI_OFFLOAD+
extern void debugfs_hw_remove();
//HTC_WIFI_OFFLOAD-
//HTC_WIFI_END
extern int mac80211_open_file_generic(struct inode *inode, struct file *file);
extern int mac80211_format_buffer(char __user *userbuf, size_t count,
				  loff_t *ppos, char *fmt, ...);
#else
static inline void debugfs_hw_add(struct ieee80211_local *local)
{
}
#endif

#endif /* __MAC80211_DEBUGFS_H */
