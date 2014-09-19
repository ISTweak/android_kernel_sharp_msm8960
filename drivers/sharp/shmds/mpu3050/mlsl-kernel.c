/*
	$License:
	Copyright (C) 2011 InvenSense Corporation, All Rights Reserved.

	This program is free software; you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation; either version 2 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
	$
 */

#include "mlsl.h"
#include <linux/i2c.h>
#include "mpu3050.h"
/* shmds add -> */
#include <linux/export.h>
/* shmds add <- */

static int inv_i2c_write(struct i2c_adapter *i2c_adap,
			    unsigned char address,
			    unsigned int len, unsigned char const *data)
{
	struct i2c_msg msgs[1];
	int res;

	if (!data || !i2c_adap)
		return -EINVAL;

	msgs[0].addr = address;
	msgs[0].flags = 0;	/* write */
	msgs[0].buf = (unsigned char *)data;
	msgs[0].len = len;

	res = i2c_transfer(i2c_adap, msgs, 1);
	if (res < 1) {
		if (res == 0)
			res = -EIO;
		return res;
	} else
		return 0;
}

/* shmds mod -> */
int inv_i2c_write_register(struct i2c_adapter *i2c_adap,
/* <- shmds mod */
				     unsigned char address,
				     unsigned char reg, unsigned char value)
{
	unsigned char data[2];

	data[0] = reg;
	data[1] = value;
	return inv_i2c_write(i2c_adap, address, 2, data);
}

/* shmds mod -> */
int inv_i2c_read(struct i2c_adapter *i2c_adap,
/* <- shmds mod */
			   unsigned char address, unsigned char reg,
			   unsigned int len, unsigned char *data)
{
	struct i2c_msg msgs[2];
	int res;

	if (!data || !i2c_adap)
		return -EINVAL;

	msgs[0].addr = address;
	msgs[0].flags = 0;	/* write */
	msgs[0].buf = &reg;
	msgs[0].len = 1;

	msgs[1].addr = address;
	msgs[1].flags = I2C_M_RD;
	msgs[1].buf = data;
	msgs[1].len = len;

	res = i2c_transfer(i2c_adap, msgs, 2);
	if (res < 2) {
		if (res >= 0)
			res = -EIO;
		return res;
	} else
		return 0;
}

static int mpu_memory_read(struct i2c_adapter *i2c_adap,
			   unsigned char mpu_addr,
			   unsigned short mem_addr,
			   unsigned int len, unsigned char *data)
{
	unsigned char bank[2];
	unsigned char addr[2];
	unsigned char buf;
/* shmds mod -> */
	struct i2c_msg msgs[2];
/* <- shmds mod */
	int res;

	if (!data || !i2c_adap)
		return -EINVAL;

	bank[0] = MPUREG_BANK_SEL;
	bank[1] = mem_addr >> 8;

	addr[0] = MPUREG_MEM_START_ADDR;
	addr[1] = mem_addr & 0xFF;

	buf = MPUREG_MEM_R_W;

	/* write message */
	msgs[0].addr = mpu_addr;
	msgs[0].flags = 0;
	msgs[0].buf = bank;
	msgs[0].len = sizeof(bank);
/* shmds mod -> */
	res = i2c_transfer(i2c_adap, msgs, 1);

	msgs[0].addr = mpu_addr;
	msgs[0].flags = 0;
	msgs[0].buf = addr;
	msgs[0].len = sizeof(addr);

	res = i2c_transfer(i2c_adap, msgs, 1);

	msgs[0].addr = mpu_addr;
	msgs[0].flags = 0;
	msgs[0].buf = &buf;
	msgs[0].len = 1;

	msgs[1].addr = mpu_addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].buf = data;
	msgs[1].len = len;

	res = i2c_transfer(i2c_adap, msgs, 2);
	if (res != 2) {
		if (res >= 0)
			res = -EIO;
		return res;
	} else
		return 0;
/* <- shmds mod */
}

static int mpu_memory_write(struct i2c_adapter *i2c_adap,
		     unsigned char mpu_addr,
		     unsigned short mem_addr,
		     unsigned int len, unsigned char const *data)
{
	unsigned char bank[2];
	unsigned char addr[2];
	unsigned char buf[513];
	
/* shmds mod -> */
	struct i2c_msg msgs[1];
/* <- shmds mod */

	int res;

	if (!data || !i2c_adap)
		return -EINVAL;
	if (len >= (sizeof(buf) - 1))
		return -ENOMEM;

	bank[0] = MPUREG_BANK_SEL;
	bank[1] = mem_addr >> 8;

	addr[0] = MPUREG_MEM_START_ADDR;
	addr[1] = mem_addr & 0xFF;

	buf[0] = MPUREG_MEM_R_W;
	memcpy(buf + 1, data, len);

	/* Write Message */
	msgs[0].addr = mpu_addr;
	msgs[0].flags = 0;
	msgs[0].buf = bank;
	msgs[0].len = sizeof(bank);
/*shmds mod->*/
	res = i2c_transfer(i2c_adap, msgs, 1);

	msgs[0].addr = mpu_addr;
	msgs[0].flags = 0;
	msgs[0].buf = addr;
	msgs[0].len = sizeof(addr);

	res = i2c_transfer(i2c_adap, msgs, 1);

	msgs[0].addr = mpu_addr;
	msgs[0].flags = 0;
	msgs[0].buf = (unsigned char *) buf;
	msgs[0].len = len + 1;

	res = i2c_transfer(i2c_adap, msgs, 1);

	if (res != 1)
		return res;
	else
		return 0;
/*<-shmds mod*/
}

int inv_serial_single_write(
	void *sl_handle,
	unsigned char slave_addr,
	unsigned char register_addr,
	unsigned char data)
{
	return inv_i2c_write_register((struct i2c_adapter *)sl_handle,
				      slave_addr, register_addr, data);
}
EXPORT_SYMBOL(inv_serial_single_write);

int inv_serial_write(
	void *sl_handle,
	unsigned char slave_addr,
	unsigned short length,
	unsigned char const *data)
{
	int result;
	const unsigned short data_length = length - 1;
	const unsigned char start_reg_addr = data[0];
	unsigned char i2c_write[SERIAL_MAX_TRANSFER_SIZE + 1];
	unsigned short bytes_written = 0;

	while (bytes_written < data_length) {
		unsigned short this_len = min(SERIAL_MAX_TRANSFER_SIZE,
					     data_length - bytes_written);
		if (bytes_written == 0) {
			result = inv_i2c_write((struct i2c_adapter *)
					       sl_handle, slave_addr,
					       1 + this_len, data);
		} else {
			/* manually increment register addr between chunks */
			i2c_write[0] = start_reg_addr + bytes_written;
			memcpy(&i2c_write[1], &data[1 + bytes_written],
				this_len);
			result = inv_i2c_write((struct i2c_adapter *)
					       sl_handle, slave_addr,
					       1 + this_len, i2c_write);
		}
		if (result)
			return result;
		bytes_written += this_len;
	}
	return 0;
}
EXPORT_SYMBOL(inv_serial_write);

int inv_serial_read(
	void *sl_handle,
	unsigned char slave_addr,
	unsigned char register_addr,
	unsigned short length,
	unsigned char *data)
{
	int result;
	unsigned short bytes_read = 0;

	if (((slave_addr & 0x7E) == DEFAULT_MPU_SLAVEADDR)
		&& (register_addr == MPUREG_FIFO_R_W ||
		    register_addr == MPUREG_MEM_R_W))
		return INV_ERROR_INVALID_PARAMETER;

	while (bytes_read < length) {
		unsigned short this_len =
		    min(SERIAL_MAX_TRANSFER_SIZE, length - bytes_read);
		result = inv_i2c_read((struct i2c_adapter *)sl_handle,
				      slave_addr, register_addr + bytes_read,
				      this_len, &data[bytes_read]);
		if (result)
			return result;
		bytes_read += this_len;
	}
	return 0;
}
EXPORT_SYMBOL(inv_serial_read);

int inv_serial_write_mem(
	void *sl_handle,
	unsigned char slave_addr,
	unsigned short mem_addr,
	unsigned short length,
	unsigned char const *data)
{
	int result;
	unsigned short bytes_written = 0;

	if ((mem_addr & 0xFF) + length > MPU_MEM_BANK_SIZE) {
		pr_err("memory read length (%d B) extends beyond its"
		       " limits (%d) if started at location %d\n", length,
		       MPU_MEM_BANK_SIZE, mem_addr & 0xFF);
		return INV_ERROR_INVALID_PARAMETER;
	}
	while (bytes_written < length) {
		unsigned short this_len =
		    min(SERIAL_MAX_TRANSFER_SIZE, length - bytes_written);
		result = mpu_memory_write((struct i2c_adapter *)sl_handle,
					  slave_addr, mem_addr + bytes_written,
					  this_len, &data[bytes_written]);
		if (result)
			return result;
		bytes_written += this_len;
	}
	return 0;
}
EXPORT_SYMBOL(inv_serial_write_mem);

int inv_serial_read_mem(
	void *sl_handle,
	unsigned char slave_addr,
	unsigned short mem_addr,
	unsigned short length,
	unsigned char *data)
{
	int result;
	unsigned short bytes_read = 0;

	if ((mem_addr & 0xFF) + length > MPU_MEM_BANK_SIZE) {
		printk
		    ("memory read length (%d B) extends beyond its limits (%d) "
		     "if started at location %d\n", length,
		     MPU_MEM_BANK_SIZE, mem_addr & 0xFF);
		return INV_ERROR_INVALID_PARAMETER;
	}
	while (bytes_read < length) {
		unsigned short this_len =
		    min(SERIAL_MAX_TRANSFER_SIZE, length - bytes_read);
		result =
		    mpu_memory_read((struct i2c_adapter *)sl_handle,
				    slave_addr, mem_addr + bytes_read,
				    this_len, &data[bytes_read]);
		if (result)
			return result;
		bytes_read += this_len;
	}
	return 0;
}
EXPORT_SYMBOL(inv_serial_read_mem);

int inv_serial_write_fifo(
	void *sl_handle,
	unsigned char slave_addr,
	unsigned short length,
	unsigned char const *data)
{
	int result;
	unsigned char i2c_write[SERIAL_MAX_TRANSFER_SIZE + 1];
	unsigned short bytes_written = 0;

	if (length > FIFO_HW_SIZE) {
		printk(KERN_ERR
		       "maximum fifo write length is %d\n", FIFO_HW_SIZE);
		return INV_ERROR_INVALID_PARAMETER;
	}
	while (bytes_written < length) {
		unsigned short this_len =
		    min(SERIAL_MAX_TRANSFER_SIZE, length - bytes_written);
		i2c_write[0] = MPUREG_FIFO_R_W;
		memcpy(&i2c_write[1], &data[bytes_written], this_len);
		result = inv_i2c_write((struct i2c_adapter *)sl_handle,
				       slave_addr, this_len + 1, i2c_write);
		if (result)
			return result;
		bytes_written += this_len;
	}
	return 0;
}
EXPORT_SYMBOL(inv_serial_write_fifo);

int inv_serial_read_fifo(
	void *sl_handle,
	unsigned char slave_addr,
	unsigned short length,
	unsigned char *data)
{
	int result;
	unsigned short bytes_read = 0;

	if (length > FIFO_HW_SIZE) {
		printk(KERN_ERR
		       "maximum fifo read length is %d\n", FIFO_HW_SIZE);
		return INV_ERROR_INVALID_PARAMETER;
	}
	while (bytes_read < length) {
		unsigned short this_len =
		    min(SERIAL_MAX_TRANSFER_SIZE, length - bytes_read);
		result = inv_i2c_read((struct i2c_adapter *)sl_handle,
				      slave_addr, MPUREG_FIFO_R_W, this_len,
				      &data[bytes_read]);
		if (result)
			return result;
		bytes_read += this_len;
	}
/*shmds mod->*/
#ifdef CONFIG_MPU_SENSORS_DEBUG
	printk("shmds debug inv_serial_read_fifo() bytesRead:%d length:%d\ndata: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X", 
			bytesRead, length, data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8], data[9]);
	printk(" %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X", 
			data[10], data[11], data[12], data[13], data[14], data[15], data[16], data[17], data[18], data[19]);
	printk(" %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X", 
			data[20], data[21], data[22], data[23], data[24], data[25], data[26], data[27], data[28], data[29]);
	printk(" %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X", 
			data[30], data[31], data[32], data[33], data[34], data[35], data[36], data[37], data[38], data[39]);
	printk(" %02X %02X\n", 
			data[40], data[41]);
#endif	/* CONFIG_MPU_SENSORS_DEBUG */
/*shmds mod->*/
	return 0;
}
EXPORT_SYMBOL(inv_serial_read_fifo);

/**
 *  @}
 */
