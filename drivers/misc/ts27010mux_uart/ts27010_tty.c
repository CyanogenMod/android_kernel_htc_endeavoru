#include <linux/module.h>
#include <linux/types.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/slab.h>

#include "ts27010.h"
#include "ts27010_mux.h"
#include "ts27010_ringbuf.h"
#include <linux/gpio.h>
#include "../../../arch/arm/mach-tegra/gpio-names.h"
#define SPRD_SC800x_GPIO_MDM_TO_AP2 TEGRA_GPIO_PJ0

/* +++ Stanley_Chang@Raw-IP */
#ifndef TS27010_NET
#define TS27010_NET
#endif

#ifdef TS27010_NET
#include "ts27010_net.h"
#endif
/* --- Stanley_Chang@Raw-IP */

#ifndef UART_QUEUE_RECV
/* #define UART_QUEUE_RECV */
#endif

struct ts27010_tty_channel_data {
	atomic_t ref_count;
	struct tty_struct *tty;
#ifdef UART_QUEUE_RECV
	struct mutex lock; /* channel r/w lock */
#endif
#ifdef PROC_DEBUG_MUX_STAT
	int sent_size;
	int recv_size;
	int back_size;
	int sent;
	int recv;
#endif
};

struct ts27010_tty_data {
	struct ts27010_tty_channel_data		chan[TS0710_MAX_MUX];
};

#ifndef HAS_PS_IND
#define HAS_PS_IND
#endif

#ifdef HAS_PS_IND
/*
 * /dev/mux6 is used for PS indicator,
 * all data on this device from BP,
 * will be duplicated to /dev/mux13
 */
#define PS_ORG_CHN	6
struct tty_struct *ts27010_uart_tty_table[TS0710_MAX_MUX + 1];
static const int dual_tty[TS0710_MAX_MUX] = {
	-1, -1, -1, -1, -1, -1, TS0710_MAX_MUX, -1, -1, -1, -1, -1, -1,
};
static const int tty2dual[TS0710_MAX_MUX + 1] = {
	0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, PS_ORG_CHN,
};
#else /* HAS_PS_IND */
struct tty_struct *ts27010_uart_tty_table[TS0710_MAX_MUX];
static const int dual_tty[TS0710_MAX_MUX] = {
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
};
static const int tty2dual[TS0710_MAX_MUX] = {
	0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12,
};
#endif /* HAS_PS_IND */

static struct tty_driver *driver;

#define TS0710MUX_MAJOR 234
#define TS0710MUX_MINOR_START 0

#ifdef PROC_DEBUG_MUX_STAT
static int s_nWriteTotal;
static int s_nWriteProt;
static int s_nWriteCount;
static int s_nReadTotal;
static int s_nRecvCount;
static int s_nBackCount;

void ts27010_tty_uart_dump_io_clear(void)
{
	int i;
	struct ts27010_tty_data *td = driver->driver_state;
	for (i = 0; i < TS0710_MAX_MUX; i++) {
		td->chan[i].sent_size = 0;
		td->chan[i].recv_size = 0;
		td->chan[i].back_size = 0;
	}
	s_nWriteTotal = 0;
	s_nWriteProt = 0;
	s_nWriteCount = 0;
	s_nReadTotal = 0;
	s_nRecvCount = 0;
	s_nBackCount = 0;
}

void ts27010_tty_uart_dump_io(void)
{
	int i;
	struct ts27010_tty_data *td = driver->driver_state;
	for (i = 0; i < TS0710_MAX_MUX; i++) {
		mux_print(MSG_ERROR,
			"(%d) refcount: %d, sent: %d, recv: %d, back: %d\n",
			i, atomic_read(&td->chan[i].ref_count),
			td->chan[i].sent_size, td->chan[i].recv_size,
			td->chan[i].back_size);
	}
	mux_print(MSG_ERROR, "Total writen size: %d, prot: %d, "
		"total read size: %d\n",
		s_nWriteTotal, s_nWriteProt, s_nReadTotal);
	mux_print(MSG_ERROR, "Total writen count: %d, recved: %d, back: %d\n",
		s_nWriteCount, s_nRecvCount, s_nBackCount);
}
#endif /* PROC_DEBUG_MUX_STAT */

static int tty_push(struct tty_struct *tty, u8 *buf, int len)
{
	int sent = 0;
	FUNC_ENTER();
	while (sent < len) {
		int count = tty_insert_flip_string(
			tty, buf + sent, len - sent);
		if (count != (len - sent)) {
			mux_print(MSG_WARNING,
				"written size %d is not wanted %d\n",
				count, len - sent);
			/* should I call it? */
			/* tty_flip_buffer_push(tty); */
			msleep_interruptible(100);
			if (signal_pending(current)) {
				mux_print(MSG_ERROR, "but got signal, "
					"send failed\n");
				break;
			}
		}
		sent += count;
	}
	tty_flip_buffer_push(tty);
	FUNC_EXIT();
	return sent;
}

#ifdef UART_QUEUE_RECV
typedef void (*WORKER_T)(struct work_struct *work);

struct mux_data_list_t {
	struct list_head list;
	struct ts27010_tty_data *td;
	int line;
	int size;
	u8 *body;
};

struct mux_recver_t {
	spinlock_t m_locker; /* recver list r/w lock */
	struct list_head m_datalist;
	const char *m_name;

	struct delayed_work m_recv_work;
};
static struct mux_recver_t *s_mux_uart_recver;

static int mux_user_recv(struct mux_recver_t *recver)
{
	struct mux_data_list_t *entry;
	struct list_head *ptr;
	struct tty_struct *tty;
	unsigned long flags;

	FUNC_ENTER();

	spin_lock_irqsave(&recver->m_locker, flags);
	while (!(list_empty(&recver->m_datalist))) {
		ptr = recver->m_datalist.next;
		entry = list_entry(ptr, struct mux_data_list_t, list);
		spin_unlock_irqrestore(&recver->m_locker, flags);

		tty = entry->td->chan[entry->line].tty;
		WARN_ON(tty != ts27010_uart_tty_table[entry->line]);

		if (tty) {
			int sent = 0;
			mutex_lock(&entry->td->chan[entry->line].lock);
			while (sent < entry->size) {
				int count;
				mutex_lock(&tty->termios_mutex);
				if (test_bit(TTY_THROTTLED, &tty->flags)
					|| tty->receive_room < entry->size) {
					mutex_unlock(&tty->termios_mutex);
					break;
				}
				mutex_unlock(&tty->termios_mutex);
				count = tty_insert_flip_string(
					tty, entry->body + sent,
					entry->size - sent);
				if (count != (len - sent)) {
					mux_print(MSG_WARNING,
						"written size %d "
						"is not wanted %d\n",
						count, entry->size - sent);
				}
				sent += count;
				tty_flip_buffer_push(tty);
			}
			mutex_unlock(&entry->td->chan[entry->line].lock);
			if (sent < entry->size) {
				mux_print(MSG_INFO,
					"!!!/dev/mux%d yet no room recv, "
					"recv_room: %d, len: %d\n",
					entry->line,
					tty->receive_room, entry->size);
				queue_delayed_work(g_mux_uart_queue,
					&recver->m_recv_work,
					msecs_to_jiffies(20));
				return 0;
			}
			mux_print(MSG_DEBUG,
				"%d data sent to user space by %s\n",
				entry->size, recver->m_name);
		} else {
			mux_print(MSG_WARNING,
				"/dev/mux%d no tty waiting for data, "
				"discard data %d\n",
				entry->line, entry->size);
		}

		spin_lock_irqsave(&recver->m_locker, flags);
#ifdef PROC_DEBUG_MUX_STAT
		entry->td->chan[entry->line].back_size += entry->size;
		s_nBackCount++;
#endif
		list_del(ptr);
		kfree(entry->body);
		kfree(entry);
	}
	spin_unlock_irqrestore(&recver->m_locker, flags);

	FUNC_EXIT();

	return 0;
}

static unsigned long s_mux_queuerecv_flags;
#define QUEUERECV_RUNNING 0

static void mux_uart_recv_worker(struct work_struct *work)
{
	struct mux_recver_t *recver =
		container_of(work, struct mux_recver_t, m_recv_work.work);

	FUNC_ENTER();

	if (test_and_set_bit(QUEUERECV_RUNNING, &s_mux_queuerecv_flags)) {
		mux_print(MSG_WARNING, "a queuerecv_work is running\n");
		queue_delayed_work(g_mux_uart_queue, &recver->m_recv_work,
			msecs_to_jiffies(20));
		return;
	}
	mux_user_recv(recver);
	clear_bit(QUEUERECV_RUNNING, &s_mux_queuerecv_flags);
	FUNC_EXIT();
}

static struct mux_recver_t *mux_alloc_recver(
	void *mux_recv_func, const char *name)
{
	struct mux_recver_t *recver;
	FUNC_ENTER();

	recver = kmalloc(sizeof(*recver), GFP_KERNEL);
	if (recver == NULL) {
		mux_print(MSG_ERROR, "alloc mux_recver_t failed\n");
		goto out;
	}

	INIT_LIST_HEAD(&recver->m_datalist);
	recver->m_locker = __SPIN_LOCK_UNLOCKED(recver->m_locker);
	recver->m_name = name;

	INIT_DELAYED_WORK(&recver->m_recv_work, (WORKER_T)mux_recv_func);
	FUNC_EXIT();

	return recver;
out:
	kfree(recver);
	return NULL;
}

static void mux_recver_free(struct mux_recver_t *recver)
{
	struct mux_data_list_t *entry;
	struct list_head *ptr;
	unsigned long flags;

	FUNC_ENTER();

	if (!recver)
		return;
	spin_lock_irqsave(&recver->m_locker, flags);
	while (!(list_empty(&recver->m_datalist))) {
		ptr = recver->m_datalist.next;
		entry = list_entry(ptr, struct mux_data_list_t, list);
		list_del(ptr);
		kfree(entry->body);
		kfree(entry);
	}
	spin_unlock_irqrestore(&recver->m_locker, flags);

	kfree(recver);
	recver = NULL;
	FUNC_EXIT();
}

static int ts27010_uart_tty_queue_recv_data(struct mux_recver_t *recver,
	struct tty_struct *tty, int line,
	struct ts27010_ringbuf *rbuf, int data_idx, int len)
{
	int ret = -ENOMEM;
	int count;
	struct mux_data_list_t *mux_list;
	unsigned long flags;

	FUNC_ENTER();

	if (!recver) {
		mux_print(MSG_ERROR, "No mux_uart_queue_recver available\n");
		goto EXIT;
	}

	if (len == 0) {
		mux_print(MSG_WARNING, "Don't queue empty data\n");
		goto EXIT;
	}

	mux_list = kmalloc(sizeof(*mux_list), GFP_KERNEL);
	if (!mux_list) {
		mux_print(MSG_ERROR, "can not malloc (%d) memory!\n",
			sizeof(struct mux_data_list_t));
		goto EXIT;
	}

	mux_list->body = kmalloc(len, GFP_KERNEL);
	if (!(mux_list->body)) {
		kfree(mux_list);
		mux_print(MSG_ERROR, "can not malloc (%d) memory!\n", len);
		goto EXIT;
	}
	mux_list->size = len;
	mux_list->td = td;
	mux_list->line = line;
	count = ts27010_ringbuf_read(rbuf, data_idx, mux_list->body, len);
	if (count != len)
		mux_print(MSG_WARNING, "read size %d is not %d\n", count, len);
#ifdef DUMP_FRAME
	if (g_mux_uart_dump_user_data)
		mux_uart_hexdump(MSG_ERROR, "dump tty_read",
			__func__, __LINE__, mux_list->body, len);
	else
		mux_uart_hexdump(MSG_MSGDUMP, "dump tty_read",
			__func__, __LINE__, mux_list->body, len);
#endif

	spin_lock_irqsave(&recver->m_locker, flags);
	list_add_tail(&mux_list->list, &recver->m_datalist);
#ifdef PROC_DEBUG_MUX_STAT
	td->chan[line].recv_size += len;
	td->chan[line].recv += len;
	s_nRecvCount++;
	s_nReadTotal += len;
#endif
	spin_unlock_irqrestore(&recver->m_locker, flags);

	if (g_mux_uart_print_level != MSG_LIGHT && g_mux_uart_print_level != MSG_ERROR) {
		mux_print(MSG_DEBUG, "%d into queue buffer by %s\n",
			len, recver->m_name);
	} else {
		mux_print(MSG_LIGHT, "%d into queue buffer by %s\n",
			len, recver->m_name);
	}

	queue_delayed_work(g_mux_uart_queue, &recver->m_recv_work, 0);

	FUNC_EXIT();
	return len;

EXIT:
	return ret;
}
#endif /* UART_QUEUE_RECV */

static int tty_direct_push(struct tty_struct *tty, int line,
	struct ts27010_ringbuf *rbuf, int data_idx, int len)
{
	int ret = -ENOMEM;
	int count;
	u8 *buf;

	FUNC_ENTER();

	if (len == 0) {
		mux_print(MSG_WARNING, "Don't queue empty data\n");
		goto EXIT;
	}

	buf = kmalloc(len, GFP_KERNEL);
	if (!buf) {
		mux_print(MSG_ERROR, "can not malloc (%d) memory!\n", len);
		goto EXIT;
	}
	count = ts27010_ringbuf_read(rbuf, data_idx, buf, len);
	if (count != len)
		mux_print(MSG_WARNING, "read size %d is not %d\n", count, len);
#ifdef DUMP_FRAME
	if (g_mux_uart_dump_user_data)
		mux_uart_hexdump(MSG_ERROR, "dump tty_read",
			__func__, __LINE__, buf, len);
	else
		mux_uart_hexdump(MSG_MSGDUMP, "dump tty_read",
			__func__, __LINE__, buf, len);
#endif
	ret = tty_push(tty, buf, len);
	kfree(buf);

#ifdef DUMP_FRAME
	if (g_mux_uart_dump_seq)
		mux_print(MSG_INFO, "push %d into /dev/mux%d "
			"tty buffer\n", len, line);
	else {
		if ( g_mux_uart_print_level != MSG_LIGHT && g_mux_uart_print_level != MSG_ERROR ) {
			mux_print(MSG_DEBUG, "push %d into /dev/mux%d "
				"tty buffer\n", len, line);
		} else {
			mux_print(MSG_LIGHT, "push %d into /dev/mux%d "
				"tty buffer\n", len, line);
		}
	}
#endif

	FUNC_EXIT();

EXIT:
	return ret;
}

/*
int ts27010_tty_uart_send(int line, u8 *data, int len)
{
	struct ts27010_tty_data *td = driver->driver_state;
	struct tty_struct *tty;

	FUNC_ENTER();
	if ((line < 0) || (line >= TS0710_MAX_MUX)) {
		mux_print(MSG_ERROR, "tty index out of range: %d.\n", line);
		return -ENODEV;
	}

	tty = td->chan[line].tty;

	if (!tty) {
		mux_print(MSG_ERROR, "mux%d no open.  discarding %d bytes\n",
			line, len);
		return 0;
	}

	BUG_ON(tty_insert_flip_string(tty, data, len) != len);
	tty_flip_buffer_push(tty);

	FUNC_EXIT();
	return len;
}
*/

int ts27010_tty_uart_send_rbuf(int line, struct ts27010_ringbuf *rbuf,
			  int data_idx, int len)
{
	/* +++ Stanley_Chang@Raw-IP */
#ifdef TS27010_NET
	extern struct ts0710_con ts0710_connection;
	mux_print(MSG_MSGDUMP, "ts0710_connection Addrress %x \n", &ts0710_connection);
	u8 tty2dlci[TS0710_MAX_MUX] = {
		1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12 ,13};
	struct ts0710_con *ts0710 = &ts0710_connection;
	int dlci;
	dlci = tty2dlci[line];
#endif
	/* --- Stanley_Chang@Raw-IP */
	struct ts27010_tty_data *td = driver->driver_state;
	struct tty_struct *tty;
#ifdef HAS_PS_IND
	struct tty_struct *ps_ctrl_tty = NULL;
#endif

	FUNC_ENTER();
	if ((line < 0) || (line >= TS0710_MAX_MUX)) {
		mux_print(MSG_ERROR, "tty index out of range: %d.\n", line);
		return -ENODEV;
	}

	tty = td->chan[line].tty;
	WARN_ON(tty != ts27010_uart_tty_table[line]);
	if (!tty) {
		mux_print(MSG_ERROR,
			"mux%d not be opened, discarding %d bytes data\n",
			line, len);
		return 0;
	}
#ifdef UART_QUEUE_RECV
	ts27010_uart_tty_queue_recv_data(
		s_mux_uart_recver, tty, line, rbuf, data_idx, len);
#else /* UART_QUEUE_RECV */
#ifdef PROC_DEBUG_MUX_STAT
	td->chan[line].recv_size += len;
	td->chan[line].recv += len;
	s_nRecvCount++;
	s_nReadTotal += len;
#endif
		/* +++ Stanley_Chang@Raw-IP */
#ifdef TS27010_NET
	if (ts0710->dlci[dlci].net){
		u8 *buf;
		buf = kmalloc(len, GFP_KERNEL);
		if (buf) {
			int count = ts27010_ringbuf_read(rbuf, data_idx, buf, len);
			if (count != len)
				mux_print(MSG_WARNING,
					"read size %d is not %d\n", count, len);
#ifdef DUMP_FRAME
			if (g_mux_uart_dump_user_data)
				mux_uart_hexdump(MSG_DEBUG, "dump tty_read",
					__func__, __LINE__, buf, len);
			else
				mux_uart_hexdump(MSG_MSGDUMP, "dump tty_read",
					__func__, __LINE__, buf, len);
#endif
			mux_print(MSG_MSGDUMP, "buf len: %d\n", len);
			mux_print(MSG_MSGDUMP, "Start receive net data...\n");
			ts27010_mux_rx_netchar(&ts0710->dlci[dlci], buf, len);
			mux_print(MSG_MSGDUMP, "Stop receive net data...\n");
			kfree(buf);
			buf = NULL;
		}
	} else {
		mux_print(MSG_MSGDUMP, "Start tty_push...\n");
		tty_direct_push(tty, line, rbuf, data_idx, len);
		mux_print(MSG_MSGDUMP, "Stop tty_push\n");
	}
#else
	tty_direct_push(tty, line, rbuf, data_idx, len);
#endif
		/* --- Stanley_Chang@Raw-IP */
#ifdef PROC_DEBUG_MUX_STAT
	td->chan[line].back_size += len;
	s_nBackCount++;
#endif
#endif /* UART_QUEUE_RECV */

#ifdef HAS_PS_IND
	/* handle ps tty */
	if (dual_tty[line] >= 0) {
		ps_ctrl_tty = ts27010_uart_tty_table[dual_tty[line]];
		if (ps_ctrl_tty) {
			mux_print(MSG_INFO,
				"send data to PS indicator channel: %d\n",
				dual_tty[line]);
#ifdef UART_QUEUE_RECV
			ts27010_uart_tty_queue_recv_data(
				s_mux_uart_recver, ps_ctrl_tty,
				line, rbuf, data_idx, len);
#else
			tty_direct_push(
				ps_ctrl_tty, line, rbuf, data_idx, len);
#endif
		} else {
			mux_print(MSG_DEBUG,
				"PS indicator channel: %d not opened\n",
				dual_tty[line]);
		}
	}
#endif /* ifdef HAS_PS_IND */

	FUNC_EXIT();
	return len;
}

#ifdef HAS_PS_IND
static int ts27010_tty_open_ps_ind(int line, struct tty_struct *tty)
{
	if (!ts27010_uart_tty_table[line]) {
		ts27010_uart_tty_table[line] = tty;
		mux_print(MSG_INFO,
			"open softmodem tty: /dev/mux%d by %s:%d\n",
			line, current->comm, current->pid);
	} else {
		mux_print(MSG_WARNING,
			"tty device: /dev/mux%d opened "
			"more than 1 times by %s:%d\n",
			line, current->comm, current->pid);
	}
	return 0;
}
#endif /* HAS_PS_IND */

static int ts27010_tty_open(struct tty_struct *tty, struct file *filp)
{
	struct ts27010_tty_data *td = tty->driver->driver_state;
	int err;
	int line = tty->index;
	FUNC_ENTER();

	if (!ts27010_mux_uart_active()) {
		mux_print(MSG_ERROR,
			"tty open when line discipline not active.\n");
		return -ENODEV;
	}

	mux_print(MSG_INFO, "tty device /dev/mux%d will be opened by %s:%d\n",
		line, current->comm, current->pid);
#ifdef HAS_PS_IND
	if (line == TS0710_MAX_MUX)
		return ts27010_tty_open_ps_ind(line, tty);
#endif
	if ((line < 0) || (line >= TS0710_MAX_MUX)) {
		mux_print(MSG_ERROR, "tty index out of range: %d.\n", line);
		return -ENODEV;
	}

	err = ts27010_mux_uart_line_open(line);
	if (err < 0)
		return err;

	atomic_inc(&td->chan[line].ref_count);
	if (atomic_read(&td->chan[line].ref_count) > 1) {
		mux_print(MSG_WARNING,
			"tty device: /dev/mux%d opened more than 1 times\n",
			line);
		WARN_ON(tty != ts27010_uart_tty_table[line]);
	} else {
		td->chan[line].tty = tty;
#ifdef PROC_DEBUG_MUX_STAT
		td->chan[line].sent = 0;
		td->chan[line].recv = 0;
#endif
		ts27010_uart_tty_table[line] = tty;
		mux_print(MSG_INFO, "tty /dev/mux%d opened successfully\n",
			line);
		if (atomic_read(&td->chan[line].ref_count) < 1) {
			mux_print(MSG_WARNING,
				"tty device: /dev/mux%d wrong refcount: %d\n",
				line, atomic_read(&td->chan[line].ref_count));
			atomic_set(&td->chan[line].ref_count, 1);
		}
	}

	FUNC_EXIT();
	return 0;
}

void ts27010_tty_uart_reset_tty(int line)
{
	struct ts27010_tty_data *td = driver->driver_state;
	mux_print(MSG_INFO, "reset tty /dev/mux%d while td: %p\n",
		line, td);
	if (td == NULL)
		return;
	if (line < 0 || line >= TS0710_MAX_MUX)
		return;

	td->chan[line].tty = NULL;
	ts27010_uart_tty_table[line] = NULL;
	atomic_set(&td->chan[line].ref_count, 0);
}

static void ts27010_tty_close(struct tty_struct *tty, struct file *filp)
{
	struct ts27010_tty_data *td = tty->driver->driver_state;
	int line = tty->index;
	FUNC_ENTER();

	mux_print(MSG_INFO, "tty device /dev/mux%d will be closed by %s:%d\n",
		line, current->comm, current->pid);
#ifdef HAS_PS_IND
	if (line == TS0710_MAX_MUX) {
		ts27010_uart_tty_table[line] = NULL;
		mux_print(MSG_INFO,
			"close softmodem tty: /dev/mux%d by %s:%d\n",
			line, current->comm, current->pid);
		return;
	}
#endif
	if ((line < 0) || (line >= TS0710_MAX_MUX)) {
		mux_print(MSG_ERROR, "tty index out of range: %d.\n", line);
		return;
	}

	if (atomic_read(&td->chan[line].ref_count) == 0) {
		mux_print(MSG_ERROR,
			"close error: tty device /dev/mux%d wasn't opened.\n",
			line);
		return;
	} else if (atomic_read(&td->chan[line].ref_count) < 0) {
		mux_print(MSG_WARNING,
			"tty device: /dev/mux%d wrong refcount: %d\n",
			line, atomic_read(&td->chan[line].ref_count));
		atomic_set(&td->chan[line].ref_count, 0);
	} else if (atomic_read(&td->chan[line].ref_count) > 1) {
		ts27010_mux_uart_line_close(tty->index);
		mux_print(MSG_WARNING,
			"tty device: /dev/mux%d isn't closed "
			"because of non-zero refcount\n", line);
		atomic_dec(&td->chan[line].ref_count);
	} else {
		ts27010_mux_uart_line_close(tty->index);

#ifdef PROC_DEBUG_MUX_STAT
		mux_print(MSG_INFO, "tty /dev/mux%d sent: %d, recv: %d\n",
			line, td->chan[line].sent, td->chan[line].recv);
#endif
		td->chan[line].tty = NULL;
		ts27010_uart_tty_table[line] = NULL;
		atomic_set(&td->chan[line].ref_count, 0);

		/*
		 * the old code did:
		 *   wake_up_interruptible(&tty->read_wait);
		 *   wake_up_interruptible(&tty->write_wait);
		 *   tty->packet = 0;
		 *
		 * I belive this is unecessary
		 */
		mux_print(MSG_INFO, "tty /dev/mux%d closed successfully\n",
			line);
	}
	FUNC_EXIT();
}

static int ts27010_tty_write(struct tty_struct *tty,
			     const unsigned char *buf, int count)
{
	int ret;
	int line = tty->index;
	FUNC_ENTER();
#ifdef VT_WATERMARK
	if(line == 9) {
		int water_mark = gpio_get_value(SPRD_SC800x_GPIO_MDM_TO_AP2);
		if(water_mark ==1) {
			mux_print(MSG_ERROR, "VT water mark over, Stop the flow!!\n");
			return -1;
		}
	}
#endif
	if ((line < 0) || (line >= TS0710_MAX_MUX)) {
		mux_print(MSG_ERROR, "tty index out of range: %d.\n", line);
		return -ENODEV;
	}
#ifdef DUMP_FRAME
	if (g_mux_uart_dump_seq)
		mux_print(MSG_INFO, "write /dev/mux%d %d by %s:%d\n",
			line, count, current->comm, current->pid);
	else {
		if ( g_mux_uart_print_level != MSG_LIGHT && g_mux_uart_print_level != MSG_ERROR ) {
			mux_print(MSG_DEBUG, "write /dev/mux%d %d by %s:%d\n",
				line, count, current->comm, current->pid);
		} else {
			mux_print(MSG_LIGHT, "write /dev/mux%d %d by %s:%d\n",
				line, count, current->comm, current->pid);
		}
	}

	if (g_mux_uart_dump_user_data)
		mux_uart_hexdump(MSG_ERROR, "dump tty_write",
			__func__, __LINE__, buf, count);
	else {
		if ( g_mux_uart_print_level != MSG_LIGHT && g_mux_uart_print_level != MSG_ERROR ) {
			mux_uart_hexdump(MSG_MSGDUMP, "dump tty_write",
				__func__, __LINE__, buf, count);
		} else {
			mux_uart_hexdump(MSG_LIGHT, "dump tty_write",
				__func__, __LINE__, buf, count);
		}
	}
#endif

	ret = ts27010_mux_uart_line_write(tty->index, buf, count);
#ifdef PROC_DEBUG_MUX_STAT
	if (ret == count) {
		((struct ts27010_tty_data *)tty->driver->driver_state)
			->chan[tty->index].sent_size += count;
		((struct ts27010_tty_data *)tty->driver->driver_state)
			->chan[tty->index].sent += count;
		s_nWriteTotal += count;
		s_nWriteProt += count + (count < 128 ? 6 : 7);
		s_nWriteCount++;
	}
#endif
	FUNC_EXIT();
	return ret;
}


static int ts27010_tty_write_room(struct tty_struct *tty)
{
	int ret;
	int line = tty->index;
	FUNC_ENTER();
#ifdef HAS_PS_IND
	if ((line < 0) || (line > TS0710_MAX_MUX)) {
#else
	if ((line < 0) || (line >= TS0710_MAX_MUX)) {
#endif
		mux_print(MSG_ERROR, "tty index out of range: %d.\n", line);
		return -ENODEV;
	}
	line = tty2dual[line];
	ret = ts27010_mux_uart_line_write_room(tty->index);
	FUNC_EXIT();
	return ret;
}

static void ts27010_tty_flush_buffer(struct tty_struct *tty)
{
	int line = tty->index;
	FUNC_ENTER();
#ifdef HAS_PS_IND
	if ((line < 0) || (line > TS0710_MAX_MUX)) {
#else
	if ((line < 0) || (line >= TS0710_MAX_MUX)) {
#endif
		mux_print(MSG_ERROR, "tty index out of range: %d.\n", line);
		return ;
	}
	line = tty2dual[line];
	ts27010_mux_uart_line_flush_buffer(line);
	FUNC_EXIT();
}

static int ts27010_tty_chars_in_buffer(struct tty_struct *tty)
{
	int ret;
	int line = tty->index;
	FUNC_ENTER();
#ifdef HAS_PS_IND
	if ((line < 0) || (line > TS0710_MAX_MUX)) {
#else
	if ((line < 0) || (line >= TS0710_MAX_MUX)) {
#endif
		mux_print(MSG_ERROR, "tty index out of range: %d.\n", line);
		return -ENODEV;
	}
	line = tty2dual[line];
	ret = ts27010_mux_uart_line_chars_in_buffer(line);
	FUNC_EXIT();
	return ret;
}

static void ts27010_tty_throttle(struct tty_struct *tty)
{
	int line = tty->index;
	FUNC_ENTER();
#ifdef HAS_PS_IND
	if ((line < 0) || (line > TS0710_MAX_MUX)) {
#else
	if ((line < 0) || (line >= TS0710_MAX_MUX)) {
#endif
		mux_print(MSG_ERROR, "tty index out of range: %d.\n", line);
		return;
	}
	line = tty2dual[line];
	ts27010_mux_uart_line_throttle(line);
	FUNC_EXIT();
}

static void ts27010_tty_unthrottle(struct tty_struct *tty)
{
	int line = tty->index;
	FUNC_ENTER();
#ifdef HAS_PS_IND
	if ((line < 0) || (line > TS0710_MAX_MUX)) {
#else
	if ((line < 0) || (line >= TS0710_MAX_MUX)) {
#endif
		mux_print(MSG_ERROR, "tty index out of range: %d.\n", line);
		return;
	}
	line = tty2dual[line];
	ts27010_mux_uart_line_unthrottle(line);
	FUNC_EXIT();
}

static int ts27010_tty_ioctl(struct tty_struct *tty, unsigned int cmd, unsigned long arg)
{
	int ret;
	int line = tty->index;
	FUNC_ENTER();
#ifdef HAS_PS_IND
	if ((line < 0) || (line > TS0710_MAX_MUX)) {
#else
	if ((line < 0) || (line >= TS0710_MAX_MUX)) {
#endif
		mux_print(MSG_ERROR, "tty index out of range: %d.\n", line);
		return -ENODEV;
	}
	line = tty2dual[line];
	/* +++ Stanley_Chang@Raw-IP */
	mux_print(MSG_DEBUG, "ioctl cmd: %x \n", cmd);
	/* --- Stanley_Chang@Raw-IP */
	ret = ts27010_mux_uart_line_ioctl(cmd, arg, line);
	FUNC_EXIT();
	return ret;
}

int ts27010_tty_uart_open(void)
{
	FUNC_ENTER();
#ifdef UART_QUEUE_RECV
	s_mux_uart_recver = mux_alloc_recver(
		mux_uart_recv_worker, "mux_uart_queue_recv");
	if (!s_mux_uart_recver)
		return -ENOMEM;
#endif
	FUNC_EXIT();
	return 0;
}

void ts27010_tty_uart_close(void)
{
	FUNC_ENTER();

#ifdef UART_QUEUE_RECV
	if (s_mux_uart_recver) {
		mux_recver_free(s_mux_uart_recver);
		s_mux_uart_recver = NULL;
	}
#endif
	FUNC_EXIT();
}

static const struct tty_operations ts27010_tty_ops = {
	.open = ts27010_tty_open,
	.close = ts27010_tty_close,
	.write = ts27010_tty_write,
	.write_room = ts27010_tty_write_room,
	.flush_buffer = ts27010_tty_flush_buffer,
	.chars_in_buffer = ts27010_tty_chars_in_buffer,
	.throttle = ts27010_tty_throttle,
	.unthrottle = ts27010_tty_unthrottle,
	.ioctl = ts27010_tty_ioctl,
};

int ts27010_tty_uart_init(void)
{
	struct ts27010_tty_data *td;
	int err;
	int i;

	FUNC_ENTER();

#ifdef UART_QUEUE_RECV
	s_mux_uart_recver = NULL;
	s_mux_queuerecv_flags = 0;
#endif

#ifdef PROC_DEBUG_MUX_STAT
	s_nWriteTotal = 0;
	s_nWriteProt = 0;
	s_nWriteCount = 0;
	s_nReadTotal = 0;
	s_nRecvCount = 0;
	s_nBackCount = 0;
#endif

#ifdef HAS_PS_IND
	/* 1 for PS indicator */
	driver = alloc_tty_driver(TS0710_MAX_MUX + 1);
#else
	driver = alloc_tty_driver(TS0710_MAX_MUX);
#endif
	if (driver == NULL) {
		err = -ENOMEM;
		goto err0;
	}

	td = kmalloc(sizeof(*td), GFP_KERNEL);
	if (td == NULL) {
		mux_print(MSG_ERROR, "alloc ts27010_tty_data failed\n");
		err = -ENOMEM;
		goto err1;
	}

	for (i = 0; i < TS0710_MAX_MUX; i++) {
		atomic_set(&td->chan[i].ref_count, 0);
		td->chan[i].tty = NULL;
		ts27010_uart_tty_table[i] = NULL;
#ifdef UART_QUEUE_RECV
		mutex_init(&td->chan[i].lock);
#endif

#ifdef PROC_DEBUG_MUX_STAT
		td->chan[i].sent_size = 0;
		td->chan[i].recv_size = 0;
		td->chan[i].back_size = 0;
#endif
	}

#ifdef HAS_PS_IND
	/* PS indicator */
	ts27010_uart_tty_table[i] = NULL;
#endif

	driver->driver_state = td;

	driver->driver_name = "ts0710mux_uart";
	driver->name = "mux";
	driver->major = TS0710MUX_MAJOR;
	driver->minor_start = TS0710MUX_MINOR_START;
	driver->type = TTY_DRIVER_TYPE_SERIAL;
	driver->subtype = SERIAL_TYPE_NORMAL;
	driver->init_termios = tty_std_termios;
	driver->init_termios.c_iflag = 0;
	driver->init_termios.c_oflag = 0;
	driver->init_termios.c_cflag = B38400 | CS8 | CREAD;
	driver->init_termios.c_lflag = 0;
	driver->other = NULL;
	driver->owner = THIS_MODULE;

	driver->flags = TTY_DRIVER_RESET_TERMIOS | TTY_DRIVER_REAL_RAW
/*
		| TTY_DRIVER_DYNAMIC_DEV
*/
		;

	tty_set_operations(driver, &ts27010_tty_ops);
	err = tty_register_driver(driver);
	if (err) {
		mux_print(MSG_ERROR, "can't register tty driver: %d\n", err);
		err = -EINVAL;
		goto err2;
	}

	/*
	for (i = 0; i < TS0710_MAX_MUX; i++)
		tty_register_device(driver, i, NULL);
	*/

	FUNC_EXIT();
	return 0;

err2:
	kfree(td);
err1:
	put_tty_driver(driver);
err0:
	return err;
}

void ts27010_tty_uart_remove(void)
{
	struct ts27010_tty_data *td = driver->driver_state;
	FUNC_ENTER();

	tty_unregister_driver(driver);
	kfree(td);
	put_tty_driver(driver);
	FUNC_EXIT();
}

