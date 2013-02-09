/* include/linux/serial_sc8800g.h
 *
 * Copyright (C) 2011 NVIDIA Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef _SERIAL_SC8800G_H_
#define _SERIAL_SC8800G_H_

//#define SC8800G_V1
#define SC8800G_V2
#define ENRTD

/*
 * struct serial_sc8800g_platform_data - SC8800G SPI serial platform data
 *
 * You should use this structure in your machine description to specify
 * how the SC8800G is connected. Example:
 *
 * static struct serial_sc8800g_platform_data serial_sc8800g_pdata = {
 * };
 *
 * static struct spi_board_info sc8800g_spi_board_info[] = {
 * 	{
 *  	.modalias	= "sc8800g",
 *  	.platform_data	= &serial_sc8800g_pdata,
 *	.mode		= SPI_MODE_3,
 *	.bus_num	= 0,
 *	.chip_select	= 0,
 *	.max_speed_hz	= 9963243,
 * 	},
 * };
 *
 */
/* constants */
#define SC8800G_TRANSFER_BITS_PER_WORD	(32)
/*TODO: CACY Need to check these definition later*/
#define SC8800G_TX_BUFSIZE					PAGE_SIZE
#define SC8800G_TRANSFER_SIZE				(4096)

#ifdef SC8800G_V1
//SC8800G V1
#define SEND_BUFFER_SIZE					(SC8800G_TRANSFER_SIZE)
#elif defined(SC8800G_V2)
//SC8800G V2
#define SEND_BUFFER_SIZE					(SC8800G_TX_BUFSIZE + sizeof(packet_header))
#else
//SC8800G V1
#define SEND_BUFFER_SIZE					(SC8800G_TRANSFER_SIZE)
#endif

#define RECV_BUFFER_SIZE					(SC8800G_TRANSFER_SIZE)
#define SEND_SIZE_ONE_LOOP					(4096)
#define RECV_SIZE_ONE_LOOP					(4096)
#define PACKET_HEADER_SIZE					(16)
#define PACKET_TX_ALIGNMENT				(64)
#define PACKET_RX_ALIGNMENT				(64)
#define RX_FIRST_PACKET_LENGTH			(64)
#define MAX_RX_PACKET_LENGTH				(RECV_BUFFER_SIZE - PACKET_HEADER_SIZE)
#define BP_RDY_TIMEOUT		(msecs_to_jiffies(3000))

#define FLAG_BP_RDY							(0)
#define FLAG_BP_RTS							(1)
#define FLAG_BP_WAKEUP						(2)
#define FLAG_BP_LIFE						(3)
#define FLAG_TX_BUSY						(4)
#define FLAG_TX_TASKLET_BUSY				(4)//SC8800G V2
#define FLAG_RX_BUSY						(5)
#define FLAG_RX_TASKLET_BUSY				(5)//SC8800G V2
#define FLAG_TRANSFER_IN_PROGRESS		(6) /* TX or RX is in place */
#define FLAG_AP_RTS_REQUESTING			(7)

/* header magic */
#define HEADER_TAG							(0x7e7f)
#define HEADER_TYPE							(0xaa55)

#define HEADER_VALID						1
#define HEADER_INVALID						0

#define TX_END_WAIT_TIMEOUT				(4*HZ)

#define SC8800G_CIRCBUF_RESET(circ_buf) \
		((circ_buf)->head = (circ_buf)->tail = 0)

enum power_state {SC8800G_OFF = 0, SC8800G_ON, SC8800G_POWERING, SC8800G_CRASH};

#define POWER_MDM_TO_AP1_WAIT_TIMEOUT	(6)

#define CHECKSUM_ERROR_PRINT_TIMES 20

#if 1
enum {
	AP_RTS, AP_RDY,
	AP_TO_MDM1, AP_TO_MDM2, AP_SPI_RTY, AP_ALIVE, 
	AP_RESEND, MDM_RTS, MDM_RDY, MDM_TO_AP1,
	MDM_TO_AP2, MDM_RESET,
	MDM_RESEND,
	MDM_SPI_RTY,
	MDM_ALIVE,
	MDM_POWER,
	MDM_NBOOT,
	SPI_CLK, SPI_DO, SPI_CS, SPI_DI,
	UART4_RXD, UART4_CTS, UART4_TXD, UART4_RTS,
	SIM_DOOR,
	SC8800G_GPIO_MAX
	};
#else
#ifdef ENRTD
enum {MDM_RTS, MDM_RDY, AP_RTS, AP_RDY, AP_RESEND, MDM_POWER, MDM_ALIVE, SC8800G_GPIO_MAX};
#else /* Enterprise */
enum {MDM_RTS, MDM_RDY, AP_RTS, AP_RDY, AP_RESEND, SC8800G_GPIO_MAX};
#endif
#endif

enum sc8800g_gpio_type {INPUT_PIN = 1, OUTPUT_PIN};

typedef enum {
	PWR_OFF = 0,
	PWR_ON,
	PWR_RESETING,
	PWR_MAX_STATE,
} MODEM_PWR_STATE;

typedef enum {
	MDM_GPIO_OFF = 0,
	MDM_GPIO_ON,
	MDM_GPIO_INPUT,
	MDM_GPIO_OUTPUT_LOW,
	MDM_GPIO_OUTPUT_HIGH,
} MODEM_GPIO_STATE;

typedef enum {
	MDM_IRQ_REQ = 0,
	MDM_IRQ_ENABLE,
	MDM_IRQ_DISABLE,
	MDM_IRQ_WAKE_ENABLE,
	MDM_IRQ_WAKE_DISABLE,
	MDM_IRQ_FREE,
} MODEM_IRQ_STATE;

struct sc8800g_gpio {
	unsigned char *name;
	int pinnum;
	enum sc8800g_gpio_type type;
	int init_output;
	int disabled;
	int is_spi;
	enum sc8800g_gpio_type offtype;
	bool exported;
	bool requested;
	bool irq_enabled;
};

struct serial_sc8800g_platform_data {
	struct sc8800g_gpio gpio[SC8800G_GPIO_MAX];
	int mdm_rdy_gpio;
	int mdm_rts_gpio;
	int ap_rdy_gpio;
	int ap_rts_gpio;
	int mdm_to_ap1_gpio;
	int mdm_alive_gpio;
	int sim_door_gpio;
	int enable_spi_byte_counter;
	int mdm_to_ap1_gpio_value;
	int enable_usb_status_to_mdm;
};

typedef struct {
	unsigned char	*buf;
	bool	alloc;
} modem_buf;


/* packet header. */
typedef struct  {
	u16 tag;
	u16 type;
	u32 length;
	u32 index;
	/* local access only */
	u32 header_valid;
}packet_header;

typedef struct {
	void *addr;
	unsigned int len;
}transfer_data;

struct sc8800g_mdm_packet {
	packet_header head;
	u32 data[0] __attribute__ ((aligned(sizeof(unsigned long))));
};

struct sc8800g_mdm_dev {
	enum power_state power;
	spinlock_t lock;	/* main lock */
	struct serial_sc8800g_platform_data	*pdata;
	struct spi_device	*spi;
	struct tty_struct	*tty;

	unsigned long	flags;

#ifdef AP_RTS_REQUEST_IN_TASKLET
	struct tasklet_struct ap_rts_tasklet;
#else
	struct timer_list ap_rts_request_timer;
#endif
	struct timer_list bp_rdy_timer;//SC8800G V2

	struct timer_list mdm2ap1_timer;

	/* TTY Layer logic */
	struct device *tty_dev;


	modem_buf	send;
	modem_buf	recv;

	struct tasklet_struct rx_tasklet;//SC8800G V2
	struct spi_transfer rx_t;//SC8800G V2
	struct spi_message rx_m;//SC8800G V2
	int rx_look_for_header;//SC8800G V2
	int rx_bytes_left;//SC8800G V2
	int rx_packet_size;//SC8800G V2
	int rx_retry;//SC8800G V2
	unsigned char *recvbuf;

	struct tasklet_struct tx_tasklet;//SC8800G V2
	struct spi_transfer tx_t;//SC8800G V2
	struct spi_message tx_m;//SC8800G V2
	struct circ_buf tx_cirbuf;//SC8800G V2
	unsigned char *sendbuf;

	struct tasklet_struct ap_rts_tasklet;//SC8800G V2

	unsigned long last_ap_rts_jiffies;//SC8800G V2
	unsigned long tx_data_len;	/* only for debug purpose */
	MODEM_PWR_STATE	pwr_sts;

	atomic_t	modem_live;
	atomic_t 	tx_done;
	atomic_t 	tx_pending;
	atomic_t	rx_pending;
	int	transfer_pending:1;
	int suspending;
	int	suspended:1;
	int closed;
	int tty_closed;
	int tty_closing;

	int mdm_alive_cout;

	int ignore_mdm_alive;
	int ignore_mdm_rts;
	int ignore_mdm_rdy;
	int ignore_mdm2ap1;

	spinlock_t	tx_lock;
	spinlock_t	usr_lock;
	spinlock_t	pm_lock;

	struct mutex 	power_mutex;

	struct mutex rw_mutex;//SC8800G V2

	struct mutex close_mutex;

	struct mutex data_mutex;

	unsigned int mdm_closed;

	wait_queue_head_t	tx_end_wait;
	wait_queue_head_t continue_close;
	wait_queue_head_t 	continue_suspend;

	wait_queue_head_t power_on_done;

	transfer_data			tx;

	struct workqueue_struct *tx_workqueue;
//	struct work_struct	tx_work;
	struct delayed_work tx_work;
	struct task_struct 	*rx_thread;

	int rdy_irq;
	int rts_irq;
	int sim_door_irq;
	int crash_irq;
	int mdm_to_ap1_irq;
	int mdm2ap1_irq;
	int alive_irq;

	struct kset *notify_kset;

	struct kobject sim_door_kobj;
	struct work_struct sim_door_notify_work;
//	struct workqueue_struct *sim_door_notify_wq;

	struct kobject crash_kobj;
	struct work_struct crash_notify_work;
	atomic_t crash_state;
//	struct workqueue_struct *crash_notify_wq;

	struct kobject usb_plugged_kobj;

	struct work_struct mdm_to_ap1_work;

	int stop_data_trans;
	int sim_door_state;

	unsigned int total_tx_bytes;
	unsigned int total_rx_bytes;

	u64 nsec_rx_start;

	u32 rx_valid_val;
};
#endif /* _SERIAL_SC8800G_H_ */
