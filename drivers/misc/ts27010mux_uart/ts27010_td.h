/*
 * File: ts27010_td.h
 *
 * Portions derived from rfcomm.c, original header as follows:
 *
 * Copyright (C) 2000, 2001  Axis Communications AB
 * Copyright (C) 2002, 2004, 2009 Motorola
 *
 * Author: Mats Friden <mats.friden@axis.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 *
 * Exceptionally, Axis Communications AB grants discretionary and
 * conditional permissions for additional use of the text contained
 * in the company's release of the AXIS OpenBT Stack under the
 * provisions set forth hereunder.
 *
 * Provided that, if you use the AXIS OpenBT Stack with other files,
 * that do not implement functionality as specified in the Bluetooth
 * System specification, to produce an executable, this does not by
 * itself cause the resulting executable to be covered by the GNU
 * General Public License. Your use of that executable is in no way
 * restricted on account of using the AXIS OpenBT Stack code with it.
 *
 * This exception does not however invalidate any other reasons why
 * the executable file might be covered by the provisions of the GNU
 * General Public License.
 *
 */
#ifndef __TS27010_TD_H__
#define __TS27010_TD_H__

#ifdef TS27010_UART_RETRAN
#define ACK 0x4F
#define ACK_OK 0
#define ACK_ERROR 1
#define INDIFFERENT_SN 0x7F
#define SHORT_CRC_CHECK 4
#define LONG_CRC_CHECK 5
#endif

#ifndef UNUSED_PARAM
#define UNUSED_PARAM(v) (void)(v)
#endif

struct ts0710_con;
struct ts27010_ringbuf;

#ifdef TS27010_UART_RETRAN
int ts27010_check_sequence_number(struct ts0710_con *ts0710, u8 sn);
/*
typedef void (*handle_uih) (struct ts0710_con *ts0710, u8 control, int dlci,
				struct ts27010_ringbuf *rbuf,
				int data_idx, int len);

void ts27010_handle_retran_uih(
	struct ts0710_con *ts0710, u8 sn, u8 control, int dlci,
	struct ts27010_ringbuf *rbuf,
	int data_idx, int len, handle_uih handle);
*/
int ts27010_uart_process_ack(struct ts0710_con *ts0710, u8 sn);
#endif

int ts27010_uart_control_send(struct ts0710_con *ts0710, u8 *buf, u32 len);
int ts27010_uart_uih_send(struct ts0710_con *ts0710, u8 *data, u32 len);
int ts27010_uart_td_open(void);
void ts27010_uart_td_close(void);
int ts27010_uart_td_init(void);
void ts27010_uart_td_remove(void);

#endif /* __TS27010_TD_H__ */
