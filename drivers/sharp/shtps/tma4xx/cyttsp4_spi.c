/*
 * Source for:
 * Cypress TrueTouch(TM) Standard Product (TTSP) SPI touchscreen driver.
 * For use with Cypress Gen4 and Solo parts.
 * Supported parts include:
 * CY8CTMA398
 * CY8CTMA884
 * CY8CTMA4XX
 *
 * Copyright (C) 2009-2011 Cypress Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2, and only version 2, as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * Contact Cypress Semiconductor at www.cypress.com <kev@cypress.com>
 *
 */

#include "cyttsp4_core.h"

#include <linux/spi/spi.h>
#include <linux/delay.h>

#define CY_SPI_WR_OP      0x00 /* r/~w */
#define CY_SPI_RD_OP      0x01
#ifdef CONFIG_SHTPS_TMA4XX_TMA443
#define CY_SPI_ADR_BYTES  1
#define CY_SPI_CMD_BYTES  2
#define CY_SPI_SYNC_BYTE  0
#else
#define CY_SPI_CMD_BYTES  4
#define CY_SPI_SYNC_BYTE  2
#endif	/* CONFIG_SHTPS_TMA4XX_TMA443 */
#define CY_SPI_SYNC_ACK1  0x62 /* from protocol v.2 */
#define CY_SPI_SYNC_ACK2  0x9D /* from protocol v.2 */
//#define CY_SPI_DATA_SIZE  128
#define CY_SPI_DATA_SIZE  256
#define CY_SPI_DATA_BUF_SIZE (CY_SPI_CMD_BYTES + CY_SPI_DATA_SIZE)
#define CY_SPI_BITS_PER_WORD 8

#ifdef CONFIG_TOUCHSCREEN_DEBUG
#define cyttsp4_spidbg(ts, l, f, a...) {\
	if (ts->bus_ops.tsdebug >= (l)) \
		pr_info(f, ## a);\
}
#else
#define cyttsp4_spidbg(ts, l, f, a...)
#endif

struct cyttsp4_spi {
	struct cyttsp4_bus_ops bus_ops;
	struct spi_device *spi_client;
	void *ttsp_client;
	u8 wr_buf[CY_SPI_DATA_BUF_SIZE];
	u8 rd_buf[CY_SPI_DATA_BUF_SIZE];
};

#ifdef CONFIG_SHTPS_TMA4XX_TMA443
static int cyttsp4_spi_xfer(u8 op, struct cyttsp4_spi *ts,
			u16 reg, u8 *buf, int length)
{
	struct spi_message msg;
	struct spi_transfer xfer;
	u8 *wr_buf = ts->wr_buf;
	u8 *rd_buf = ts->rd_buf;
	int retval;

	if (length > CY_SPI_DATA_SIZE) {
		cyttsp4_spidbg(ts, CY_DBG_LVL_3,
			"%s: length %d is too big.\n",
			__func__, length);
		return -EINVAL;
	}

	memset(wr_buf, 0, CY_SPI_DATA_BUF_SIZE);
	memset(rd_buf, 0, CY_SPI_DATA_BUF_SIZE);
	memset((void *)&xfer, 0, sizeof(xfer));

	spi_message_init(&msg);
	if (op == CY_SPI_WR_OP) {
		/* Data Write */
		wr_buf[0] = ((u8)(reg >> 7) & 0x02) | op;
		wr_buf[1] = (u8)reg;
		memcpy(wr_buf + CY_SPI_CMD_BYTES, buf, length);

		xfer.tx_buf = wr_buf;
		xfer.rx_buf = rd_buf;
		xfer.len = length + CY_SPI_CMD_BYTES;
		spi_message_add_tail(&xfer, &msg);
	} else if (op == CY_SPI_RD_OP) {
		/* Adress Set */
		retval = cyttsp4_spi_xfer(CY_SPI_WR_OP, ts, reg, buf, 0);
		if (retval != 0)
			return retval;
		udelay(1);

		/* Data Read */
		memset(wr_buf, 0, CY_SPI_DATA_BUF_SIZE);
		memset(rd_buf, 0, CY_SPI_DATA_BUF_SIZE);
		wr_buf[0] = op;

		xfer.tx_buf = wr_buf;
		xfer.rx_buf = rd_buf;
		xfer.len = length + (CY_SPI_CMD_BYTES - CY_SPI_ADR_BYTES);
		spi_message_add_tail(&xfer, &msg);
	}

	retval = spi_sync(ts->spi_client, &msg);
	if (retval < 0) {
		cyttsp4_spidbg(ts, CY_DBG_LVL_3,
			"%s: spi_sync() error %d, len=%d, op=%d\n",
			__func__, retval, xfer.len, op);

		/*
		 * do not return here since probably a bad ACK sequence
		 * let the following ACK check handle any errors and
		 * allow silent retries
		 */
	}

	if (op == CY_SPI_RD_OP)
		memcpy(buf, rd_buf + CY_SPI_ADR_BYTES, length);

	if ((rd_buf[CY_SPI_SYNC_BYTE] == CY_SPI_SYNC_ACK1))
		retval = 0;
	else {
		int i;
		for (i = 0; i < (CY_SPI_CMD_BYTES); i++)
			cyttsp4_spidbg(ts, CY_DBG_LVL_3,
				"%s: test rd_buf[%d]:0x%02x\n",
				__func__, i, rd_buf[i]);
		for (i = 0; i < (length); i++)
			cyttsp4_spidbg(ts, CY_DBG_LVL_3,
				"%s: test buf[%d]:0x%02x\n",
				__func__, i, buf[i]);

		/* signal ACK error so silent retry */
		retval = 1;
	}

	return retval;
}
#else
static int cyttsp4_spi_xfer(u8 op, struct cyttsp4_spi *ts,
	u8 reg, u8 *buf, int length)
{
	struct spi_message msg;
	struct spi_transfer xfer[2];
	u8 *wr_buf = ts->wr_buf;
	u8 *rd_buf = ts->rd_buf;
	int retval;

	if (length > CY_SPI_DATA_SIZE) {
		cyttsp4_spidbg(ts, CY_DBG_LVL_3,
			"%s: length %d is too big.\n",
			__func__, length);
		return -EINVAL;
	}

	memset(wr_buf, 0, CY_SPI_DATA_BUF_SIZE);
	memset(rd_buf, 0, CY_SPI_DATA_BUF_SIZE);

	wr_buf[0] = 0x00; /* header byte 0 */
	wr_buf[1] = 0xFF; /* header byte 1 */
	wr_buf[2] = reg;  /* reg index */
	wr_buf[3] = op;   /* r/~w */
	if (op == CY_SPI_WR_OP)
		memcpy(wr_buf + CY_SPI_CMD_BYTES, buf, length);

	memset((void *)xfer, 0, sizeof(xfer));
	spi_message_init(&msg);
	xfer[0].tx_buf = wr_buf;
	xfer[0].rx_buf = rd_buf;
	if (op == CY_SPI_WR_OP) {
		xfer[0].len = length + CY_SPI_CMD_BYTES;
		spi_message_add_tail(&xfer[0], &msg);
	} else if (op == CY_SPI_RD_OP) {
		xfer[0].len = CY_SPI_CMD_BYTES;
		spi_message_add_tail(&xfer[0], &msg);

		xfer[1].rx_buf = buf;
		xfer[1].len = length;
		spi_message_add_tail(&xfer[1], &msg);
	}

	retval = spi_sync(ts->spi_client, &msg);
	if (retval < 0) {
		cyttsp4_spidbg(ts, CY_DBG_LVL_3,
			"%s: spi_sync() error %d, len=%d, op=%d\n",
			__func__, retval, xfer[1].len, op);

		/*
		 * do not return here since probably a bad ACK sequence
		 * let the following ACK check handle any errors and
		 * allow silent retries
		 */
	}

	if ((rd_buf[CY_SPI_SYNC_BYTE] == CY_SPI_SYNC_ACK1) &&
		(rd_buf[CY_SPI_SYNC_BYTE+1] == CY_SPI_SYNC_ACK2))
		retval = 0;
	else {
		int i;
		for (i = 0; i < (CY_SPI_CMD_BYTES); i++)
			cyttsp4_spidbg(ts, CY_DBG_LVL_3,
				"%s: test rd_buf[%d]:0x%02x\n",
				__func__, i, rd_buf[i]);
		for (i = 0; i < (length); i++)
			cyttsp4_spidbg(ts, CY_DBG_LVL_3,
				"%s: test buf[%d]:0x%02x\n",
				__func__, i, buf[i]);

		/* signal ACK error so silent retry */
		retval = 1;
	}

	return retval;
}
#endif	/* CONFIG_SHTPS_TMA4XX_TMA443 */

static s32 cyttsp4_spi_read_block_data(void *handle, u16 addr,
	size_t length, void *data, int spi_addr, bool use_subaddr)
{
	struct cyttsp4_spi *ts =
		container_of(handle, struct cyttsp4_spi, bus_ops);
	int retval;

	retval = cyttsp4_spi_xfer(CY_SPI_RD_OP, ts, addr, data, length);
	if (retval < 0)
		pr_err("%s: cyttsp4_spi_read_block_data failed\n",
			__func__);

	/*
	 * Do not print the above error if the data sync bytes were not found.
	 * This is a normal condition for the bootloader loader startup and need
	 * to retry until data sync bytes are found.
	 */
	if (retval > 0)
		retval = -EIO;	/* now signal fail; so retry can be done */

	return retval;
}

static s32 cyttsp4_spi_write_block_data(void *handle, u16 addr,
	size_t length, const void *data, int spi_addr, bool use_subaddr)
{
	struct cyttsp4_spi *ts =
		container_of(handle, struct cyttsp4_spi, bus_ops);
	int retval;

	retval = cyttsp4_spi_xfer(CY_SPI_WR_OP, ts, addr, (void *)data, length);
	if (retval < 0)
		pr_err("%s: cyttsp4_spi_write_block_data failed\n",
			__func__);

	/*
	 * Do not print the above error if the data sync bytes were not found.
	 * This is a normal condition for the bootloader loader startup and need
	 * to retry until data sync bytes are found.
	 */
	if (retval > 0)
		retval = -EIO;	/* now signal fail; so retry can be done */

	return retval;
}

static int __devinit cyttsp4_spi_probe(struct spi_device *spi)
{
	struct cyttsp4_spi *ts;
	int retval = 0;

	/* Set up SPI*/
	spi->bits_per_word = CY_SPI_BITS_PER_WORD;
	spi->mode = SPI_MODE_0;
	retval = spi_setup(spi);
	if (retval < 0) {
		pr_err("%s: SPI setup error %d\n",
			__func__, retval);
		goto cyttsp4_spi_probe_exit;
	}

	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (ts == NULL) {
		pr_err("%s: Error, kzalloc.\n", __func__);
		retval = -ENOMEM;
		goto cyttsp4_spi_probe_exit;
	}

	ts->spi_client = spi;
	dev_set_drvdata(&spi->dev, ts);
	ts->bus_ops.write = cyttsp4_spi_write_block_data;
	ts->bus_ops.read = cyttsp4_spi_read_block_data;
	ts->bus_ops.dev = &spi->dev;

	ts->ttsp_client = cyttsp4_core_init(&ts->bus_ops, &spi->dev,
		spi->irq, spi->modalias);
	if (ts->ttsp_client == NULL) {
		kfree(ts);
		ts = NULL;
		retval = -ENODATA;
		pr_err("%s: Registration fail ret=%d\n", __func__, retval);
		goto cyttsp4_spi_probe_exit;
	}

	pr_info("%s: Registration complete\n", __func__);

cyttsp4_spi_probe_exit:
	return retval;
}

static int __devexit cyttsp4_spi_remove(struct spi_device *spi)
{
	struct cyttsp4_spi *ts = dev_get_drvdata(&spi->dev);

	cyttsp4_core_release(ts->ttsp_client);
	kfree(ts);
	return 0;
}

#if defined(CONFIG_PM) && !defined(CONFIG_HAS_EARLYSUSPEND)
static int cyttsp4_spi_suspend(struct spi_device *spi, pm_message_t message)
{
	return cyttsp4_suspend(dev_get_drvdata(&spi->dev));
}

static int cyttsp4_spi_resume(struct spi_device *spi)
{
	return cyttsp4_resume(dev_get_drvdata(&spi->dev));
}
#endif

static struct spi_driver cyttsp4_spi_driver = {
	.driver = {
		.name = CY_SPI_NAME,
		.bus = &spi_bus_type,
		.owner = THIS_MODULE,
	},
	.probe = cyttsp4_spi_probe,
	.remove = __devexit_p(cyttsp4_spi_remove),
#if defined(CONFIG_PM) && !defined(CONFIG_HAS_EARLYSUSPEND)
	.suspend = cyttsp4_spi_suspend,
	.resume = cyttsp4_spi_resume,
#endif
};

static int __init cyttsp4_spi_init(void)
{
	return spi_register_driver(&cyttsp4_spi_driver);
}
module_init(cyttsp4_spi_init);

static void __exit cyttsp4_spi_exit(void)
{
	spi_unregister_driver(&cyttsp4_spi_driver);
}
module_exit(cyttsp4_spi_exit);

MODULE_ALIAS("spi:cyttsp");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Cypress TrueTouch(R) Standard Product (TTSP) SPI driver");
MODULE_AUTHOR("Cypress");

