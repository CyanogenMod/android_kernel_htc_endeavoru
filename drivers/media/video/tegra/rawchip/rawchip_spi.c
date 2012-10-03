/* Copyright (c) 2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <media/rawchip/rawchip_spi.h>

#ifdef RAWCHIP_SPI_DEBUG
#define CDBG(fmt, args...) pr_debug(fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#endif

#define yushan_MAX_ALLOCATE 1000
static uint8_t *yushan_spi_write_addr;

static struct spi_device *rawchip_dev;

struct yushan_spi_ctrl_blk {
	struct spi_device *spi;
	spinlock_t		spinlock;
};
struct yushan_spi_ctrl_blk *yushan_spi_ctrl;

static int
yushan_spi_sync_write_then_read(uint8_t *txbuf, size_t n_tx,
	uint8_t *rxbuf, size_t n_rx)
{
#if 0
	struct spi_transfer	tx_t = {
			.tx_buf		= txbuf,
			.len		= n_tx,
		};
	struct spi_transfer	rx_t = {
			.rx_buf		= rxbuf,
			.len		= n_rx,
		};
	struct spi_message	m;

	spi_message_init(&m);
	spi_message_add_tail(&tx_t, &m);
	spi_message_add_tail(&rx_t, &m);
#else
	struct spi_transfer	rx_t = {
		.tx_buf = txbuf,
		.rx_buf		= rxbuf,
		.len		= n_rx,
		};
	struct spi_message	m;

	spi_message_init(&m);
	spi_message_add_tail(&rx_t, &m);
#endif

	return spi_sync(yushan_spi_ctrl->spi, &m); /* USE spi_sync function for now.*/
}
int yushan_spi_read(uint16_t reg, uint8_t *rval)
{
	int rc = 0;
	uint8_t /*tx[3],*/ rx[6];
/*
	tx[0] = 0x60;
	tx[1] = (reg & 0xff00) >> 8;
	tx[2] = reg & 0x00ff;

	rc = yushan_spi_single_write(&tx[0], 3);
*/
	rx[0] = 0x60;
	rx[1] = (reg & 0xff00) >> 8;
	rx[2] = reg & 0x00ff;
	rx[3] = 0x61;
	rx[4] = 0;
	/*rx[5] = 0;*/

	rc = yushan_spi_sync_write_then_read(&rx[0], 3,
		&rx[3], 2);
#if 0
	pr_info("%s: rc = %d\n", __func__, rc);
	pr_info("rx[3] = 0x%x, rx[4] = 0x%x, rx[5] = 0x%x\n",
		rx[3], rx[4], rx[5]);
#endif
	if (rc >= 0)
		/* *rval = rx[4] << 8 | rx[5];*/
		*rval = rx[4];
	else {
		pr_err("yushan_spi_sync_write_then_read failed\n");
		*rval = 0;
	}

	return rc;
}

int SPI_Read(uint16_t uwIndex , uint16_t uwCount , uint8_t *pData)
{
	uint16_t reg;
	uint8_t i, val, rc = 0;
	val = 0;
	for (i=0; i<uwCount; i++)
	{
		reg = uwIndex+i;
		/*rc = yushan_spi_read(reg,&val);*/
		rc = rawchip_spi_read_2B1B(reg, &val);
		if (rc==0)
		{
			*(pData+i) = val;
			CDBG("[CAM]%s 0x%x[%d]=0x%x SPI_Read OK", __func__, uwIndex, i, *(pData+i));
		}
		else
		{
			pr_err("[CAM]%s 0x%x[%d]=0x%x " \
				"SPI_Read Fail", __func__, uwIndex, i, *(pData+i));
			break;
		}
	}
	if (rc == 0)
		return SUCCESS;
	else
		return FAILURE;
}

#if 0
static int yushan_spi_sync_write_once(uint8_t *tbuf, uint8_t *wbuf)
{
	struct spi_message	m;

	struct spi_transfer tx_addr = {
		.tx_buf	= tbuf,
		.len = 3,
	};

	struct spi_transfer tx_buf = {
		.tx_buf = wbuf,
		.len = 3,
	};

	spi_message_init(&m);
	spi_message_add_tail(&tx_addr, &m);
	spi_message_add_tail(&tx_buf, &m);

	return yushan_spi_transaction(&m);
}
#if 1
static int yushan_spi_write(uint16_t reg, uint16_t val)
{
	uint8_t tx[3], wb[3];

	tx[0] = yushan_REGADDR_WR;
	tx[1] = (reg & 0xff00) >> 8;
	tx[2] = reg & 0x00ff;

	wb[0] = yushan_REGVAL_WR;
	wb[1] = (val & 0xff00) >> 8;
	wb[2] = val & 0x00ff;

    return yushan_spi_sync_write_once(&tx[0], &wb[0]);
}
#endif
#endif

static void yushan_spi_complete(void *arg)
{
	complete(arg);
}

static int yushan_spi_transaction(struct spi_message *msg)
{
	DECLARE_COMPLETION_ONSTACK(yushan_done);
	int status;

	msg->complete = yushan_spi_complete;
	msg->context = &yushan_done;

	CDBG("[CAMSPI] %s spin_lock_irq\n", __func__);
	spin_lock_irq(&yushan_spi_ctrl->spinlock);
	if (yushan_spi_ctrl->spi == NULL)
		status = -ESHUTDOWN;
	else
		{
		CDBG("[CAMSPI] %s spi_async\n", __func__);
		status = spi_async(yushan_spi_ctrl->spi, msg);
	}
	CDBG("[CAMSPI] %s spin_unlock_irq\n", __func__);
	spin_unlock_irq(&yushan_spi_ctrl->spinlock);

	if (status == 0) {
		CDBG("[CAMSPI] %s wait_for_completion\n", __func__);
		wait_for_completion(&yushan_done);
		CDBG("[CAMSPI] %s wait_for_completion DONE\n", __func__);
		status = msg->status;
#if 0//HTC FIX ME
		if (status == 0)
			status = msg->actual_length;
#endif
	}

	//HTC_START fix Klocwork
	msg->context = NULL;
	//HTC_END
	return status;
}

static int yushan_spi_sync_write_once(uint8_t *tbuf, uint8_t *wbuf)
{
	struct spi_message	m;

	struct spi_transfer tx_addr = {
		.tx_buf	= tbuf,
		.len = 4,
	};
/*
	struct spi_transfer tx_buf = {
		.tx_buf = wbuf,
		.len = 1,
	};
*/
	spi_message_init(&m);
	spi_message_add_tail(&tx_addr, &m);
/*
	spi_message_add_tail(&tx_buf, &m);
*/
	return yushan_spi_transaction(&m);
}

int yushan_spi_write(uint16_t reg, uint8_t/*uint16_t*/ val)
{
	uint8_t tx[4]/*, wb[1]*/;

	tx[0] = 0x60;
	tx[1] = (reg & 0xff00) >> 8;
	tx[2] = reg & 0x00ff;
	tx[3] = val;
/*
	wb[0] = yushan_REGVAL_WR;
	wb[1] = (val & 0xff00) >> 8;
	wb[2] = val & 0x00ff;
*/

    return yushan_spi_sync_write_once(&tx[0], NULL/*&wb[0]*/);
}

int SPI_Write(uint16_t uwIndex , uint16_t uwCount , uint8_t *pData)
{
	uint16_t reg, i;
	uint8_t val, rc = 0;
	val = 0;
	for (i = 0; i < uwCount; i++) {
		reg = uwIndex+i;
		rc = yushan_spi_write(reg,*(pData+i));
		if (rc == 0)
			CDBG("[DxO]%s 0x%x[%d]=0x%x SPI_Write OK",__func__, uwIndex, i, *(pData+i));
		else {
			pr_err("[CAM]%s 0x%x[%d]=0x%x " \
			"SPI_Write Fail",__func__, uwIndex, i, *(pData+i));
			break;
		}
	}
	if (rc == 0)
		return SUCCESS;
	else
		return FAILURE;
}

static int32_t Yushan_spi_write_table(
	uint16_t uwIndex , uint16_t uwCount , uint8_t *pData)
{
	int i, status;
	struct spi_message	m;
	struct spi_transfer	tx_addr;
	uint16_t transferedIndex = 0;

	if (!yushan_spi_write_addr) {
		pr_err("Error allocating memory retrying num:%d\n", uwCount);
		return FAILURE;
	}

	while (transferedIndex < uwCount) {
		spi_message_init(&m);
		memset(&tx_addr, 0, sizeof(struct spi_transfer));

		yushan_spi_write_addr[0] = 0x60;
		yushan_spi_write_addr[1] = ((uwIndex + transferedIndex) & 0xff00) >> 8;
		yushan_spi_write_addr[2] = (uwIndex + transferedIndex) & 0x00ff;

		for (i = 0; (i < yushan_MAX_ALLOCATE && transferedIndex < uwCount); i++, transferedIndex++)
			yushan_spi_write_addr[i+3] = *(pData+2+4*transferedIndex);

		tx_addr.tx_buf = yushan_spi_write_addr;
		tx_addr.len = i + 3;
		tx_addr.cs_change = 0;
		//FIX_ME!!!! EVA is using 32
		tx_addr.bits_per_word = 8;
		spi_message_add_tail(&tx_addr, &m);
		status = yushan_spi_transaction(&m);
		if (status != 0) {
			pr_err("[CAM]%s, spi write status::%d", __func__, status);
			return FAILURE;
		}
	}
	return SUCCESS;
}
int SPI_Write_4thByte(uint16_t uwIndex , uint16_t uwCount , uint8_t *pData)
{
#if 0
	uint16_t reg, i;
	uint8_t val, rc;
	val = 0;
	for (i = 0; i < uwCount; i++) {
		reg = uwIndex+i;
		rc = yushan_spi_write(reg, *(pData+2+4*i));
		if (rc == 0) {
			CDBG("[CAM]%s 0x%x=0x%x \
				SPI_Write OK (%d)", __func__, reg, *(pData+2+4*i), 2+4*i);
		} else {
			pr_err("[CAM]%s 0x%x=0x%x\
				SPI_Write Fail", __func__, reg, *(pData+2+4*i));
			break;
		}
	}
  if (rc == 0)
	return SUCCESS;
  else
	return FAILURE;
#else
	return Yushan_spi_write_table(uwIndex, uwCount, pData);
#endif
}


int rawchip_spi_write(unsigned char addr, unsigned char data)
{
	unsigned char buffer[2];
	int rc;
	CDBG("[CAM] rawchip_spi_write+++\n");
	if (!rawchip_dev)
		return 0;

	rawchip_dev->bits_per_word = 16;
	buffer[0] = addr;
	buffer[1] = data;
	rc = spi_write(rawchip_dev, buffer, 2);
	CDBG("[CAM] rawchip_spi_write---, rc=%d\n", rc);
	return 1;
}

int rawchip_spi_write_2B1B(uint16_t addr, unsigned char data)
{
	unsigned char buffer[4];
	int rc;
	/*uint8_t rb;*/

	if (!rawchip_dev)
		return 0;

	rawchip_dev->bits_per_word = 8;
	buffer[0] = 0x60;
	buffer[1] = (addr & 0xff00) >> 8;
	buffer[2] = addr & 0x00ff;
	buffer[3] = data;
	rc = spi_write(rawchip_dev, buffer, 4);
	CDBG("[CAM] rawchip_spi_write_2B1B--\
		, rc=%d (addr=0x%x, data=0x%x)\n", rc, addr, data);

/*read back check*/
#if 0
	msleep(10);
	rawchip_spi_read_2B1B(addr, &rb);
	if (data != rb)
		pr_info("[CAM]rawchip_spi_write_2B1B!!!!, %d %d\n", data, rb);
#endif

	return 1;
}


int rawchip_spi_read_2B1B(uint16_t addr, unsigned char *data)
{
	unsigned char buffer[4], tx_buf[2], rx_buf[2];
	int rc;
	if (!rawchip_dev)
		return 0;

	rawchip_dev->bits_per_word = 8;
	buffer[0] = 0x60;
	buffer[1] = (addr & 0xff00) >> 8;
	buffer[2] = addr & 0x00ff;

	rc = spi_write(rawchip_dev, buffer, 3);

	tx_buf[0] = 0x61;
	tx_buf[1] = 0x00;

	rx_buf[0] = 0x00;
	rx_buf[1] = 0x00;

	rc = spi_write_and_read(rawchip_dev, tx_buf, rx_buf, 2);
	*data = rx_buf[1];
	CDBG("[CAM]rawchip_spi_read_2B1B---, \
		rc=%d, (addr=0x%x, data=0x%x)\n", rc, addr, *data);

	return 0;
}

int rawchip_spi_read_2B2B(uint16_t addr, uint16_t *data)
{
	unsigned char buffer[4], tx_buf[4], rx_buf[4];
	int rc;
	if (!rawchip_dev)
		return 0;

	rawchip_dev->bits_per_word = 8;
	buffer[0] = 0x60;
	buffer[1] = (addr & 0xff00) >> 8;
	buffer[2] = addr & 0x00ff;

	rc = spi_write(rawchip_dev, buffer, 3);

	tx_buf[0] = 0x61;
	tx_buf[1] = 0x00;

	rx_buf[0] = 0x00;
	rx_buf[1] = 0x00;
	rx_buf[2] = 0x00;

	rc = spi_write_and_read(rawchip_dev, tx_buf, rx_buf, 3);
	*data = (rx_buf[2]<<8 | rx_buf[1]);
	if (rc != 0)
		pr_info("[CAM]rawchip_spi_read_2B2B---, rc=%d\n", rc);
	return 1;
}

int spi_rawchip_probe(struct spi_device *rawchip)
{
	pr_info("[CAM]:%s\n", __func__);

	rawchip_dev = rawchip;

	//from yushan
	yushan_spi_ctrl = kzalloc(sizeof(*yushan_spi_ctrl), GFP_KERNEL);
	if (!yushan_spi_ctrl)
		return -ENOMEM;

	yushan_spi_ctrl->spi = rawchip;
	spin_lock_init(&yushan_spi_ctrl->spinlock);

	spi_set_drvdata(rawchip, yushan_spi_ctrl);

	return 0;
}

static struct spi_driver spi_rawchip = {
	.driver = {
		.name = "spi_rawchip",
		.owner = THIS_MODULE,
	},
	.probe = spi_rawchip_probe,
};

int rawchip_spi_init(void)
{
	int rc = -1;

	pr_info("[CAM]%s \n", __func__);

	rc = spi_register_driver(&spi_rawchip);
	if (rc < 0) {
		pr_err("[CAM]%s:failed to register \
			spi driver(%d) for camera\n", __func__, rc);
		return -EINVAL;
	}

	if (!yushan_spi_ctrl) {
		pr_err("yushan_spi_ctrl is NULL!\n");
		return -EINVAL;
	}

	if(yushan_spi_write_addr == NULL)
		yushan_spi_write_addr =
			kcalloc(yushan_MAX_ALLOCATE+3, sizeof(uint8_t), GFP_KERNEL);

	return 0;
}

