
#ifndef __TS27010_NET_H__
#define __TS27010_NET_H__

#include <linux/netdevice.h>

struct ts27010_netconfig {
//	unsigned int adaption; /* Adaption to use in network mode */
//	unsigned short protocol;/* Protocol to use - only ETH_P_IP supported */
//	unsigned short unused2;
	char if_name[IFNAMSIZ];	/* interface name format string */
//	__u8 unused[28]; /* For future use */
};

#define TS27010IOC_ENABLE_NET _IOW('m', 4, struct ts27010_netconfig)
#define TS27010IOC_DISABLE_NET _IO('m', 5)

struct dlci_struct;
struct ts27010_netconfig;

void ts27010_mux_rx_netchar(struct dlci_struct *dlci,
	 unsigned char *in_buf, int size);

void ts27010_destroy_network(struct dlci_struct *dlci);

int ts27010_create_network(struct dlci_struct *dlci, struct ts27010_netconfig *nc);

#endif /* __TS27010_NET_H__ */
