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

/**
 *  @addtogroup ACCELDL
 *  @brief      Provides the interface to setup and handle an accelerometer.
 *
 *  @{
 *      @file   lis331.c
 *      @brief  Accelerometer setup and handling methods for ST LIS331DLH.
 */

/* -------------------------------------------------------------------------- */

#undef MPL_LOG_NDEBUG
#define MPL_LOG_NDEBUG 1

#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include "mpu-dev.h"
/* shmds add -> */
#include <asm/atomic.h>
#include "slaveirq.h"
/* shmds add <- */
#include <log.h>
#include <sharp/mpu.h>
#include "mlsl.h"
#include "mldl_cfg.h"
#undef MPL_LOG_TAG
#define MPL_LOG_TAG "MPL-acc"
/* shmds add-> */
#include <linux/interrupt.h>
//#include <sharp/sh_boot_manager.h>
/*<-shmds add */


/* full scale setting - register & mask */
#define LIS331DLH_CTRL_REG1         (0x20)
#define LIS331DLH_CTRL_REG2         (0x21)
#define LIS331DLH_CTRL_REG3         (0x22)
#define LIS331DLH_CTRL_REG4         (0x23)
#define LIS331DLH_CTRL_REG5         (0x24)
#define LIS331DLH_HP_FILTER_RESET   (0x25)
#define LIS331DLH_REFERENCE         (0x26)
#define LIS331DLH_STATUS_REG        (0x27)
#define LIS331DLH_OUT_X_L           (0x28)
#define LIS331DLH_OUT_X_H           (0x29)
#define LIS331DLH_OUT_Y_L           (0x2a)
#define LIS331DLH_OUT_Y_H           (0x2b)
#define LIS331DLH_OUT_Z_L           (0x2b)
#define LIS331DLH_OUT_Z_H           (0x2d)

#define LIS331DLH_INT1_CFG          (0x30)
#define LIS331DLH_INT1_SRC          (0x31)
#define LIS331DLH_INT1_THS          (0x32)
#define LIS331DLH_INT1_DURATION     (0x33)

#define LIS331DLH_INT2_CFG          (0x34)
#define LIS331DLH_INT2_SRC          (0x35)
#define LIS331DLH_INT2_THS          (0x36)
#define LIS331DLH_INT2_DURATION     (0x37)

/* CTRL_REG1 */
#define LIS331DLH_CTRL_MASK         (0x30)
#define LIS331DLH_SLEEP_MASK        (0x20)
#define LIS331DLH_PWR_MODE_NORMAL   (0x20)

#define LIS331DLH_MAX_DUR           (0x7F)


/* -------------------------------------------------------------------------- */

/* shmds add -> */
#ifdef SHMDS_DETECT
int shmds_ths_value = 12;
int shmds_duration_value = 0x01;
static unsigned long shmds_count = 0;
static int shmds_buf_size = 10;
int shmds_buf_size_temp = 10;
int shmds_md_thresh = 200;
int shmds_md_thresh2 = 200;
static int shmds_detect_switch = DETECT_OFF;
static signed short *shmds_detect_buf[3];
static signed short shmds_economize_buf[3] = {0, 0, 0};
atomic_t shmds_detect_mode_flg = ATOMIC_INIT(SHMDS_DETECT_NORMAL);
#endif /* SHMDS_DETECT */
/* shmds add <- */

struct lis331dlh_config {
	unsigned int odr;
	unsigned int fsr;	/* full scale range mg */
	unsigned int ths;	/* Motion no-motion thseshold mg */
	unsigned int dur;	/* Motion no-motion duration ms */
	unsigned char reg_ths;
	unsigned char reg_dur;
	unsigned char ctrl_reg1;
	unsigned char irq_type;
	unsigned char mot_int1_cfg;
};

struct lis331dlh_private_data {
	struct lis331dlh_config suspend;
	struct lis331dlh_config resume;
};

/* -------------------------------------------------------------------------- */
static int lis331dlh_set_ths(void *mlsl_handle,
			     struct ext_slave_platform_data *pdata,
			     struct lis331dlh_config *config,
			     int apply, long ths)
{
	int result = INV_SUCCESS;
	if ((unsigned int)ths >= config->fsr)
		ths = (long)config->fsr - 1;

	if (ths < 0)
		ths = 0;

	config->ths = ths;
	config->reg_ths = (unsigned char)(long)((ths * 128L) / (config->fsr));
	MPL_LOGV("THS: %d, 0x%02x\n", config->ths, (int)config->reg_ths);
	if (apply)
		result = inv_serial_single_write(mlsl_handle, pdata->address,
						 LIS331DLH_INT1_THS,
						 config->reg_ths);
	return result;
}

static int lis331dlh_set_dur(void *mlsl_handle,
			     struct ext_slave_platform_data *pdata,
			     struct lis331dlh_config *config,
			     int apply, long dur)
{
	int result = INV_SUCCESS;
	long reg_dur = (dur * config->odr) / 1000000L;
	config->dur = dur;

	if (reg_dur > LIS331DLH_MAX_DUR)
		reg_dur = LIS331DLH_MAX_DUR;

	config->reg_dur = (unsigned char)reg_dur;
	MPL_LOGV("DUR: %d, 0x%02x\n", config->dur, (int)config->reg_dur);
	if (apply)
		result = inv_serial_single_write(mlsl_handle, pdata->address,
						 LIS331DLH_INT1_DURATION,
						 (unsigned char)reg_dur);
	return result;
}

/**
 * Sets the IRQ to fire when one of the IRQ events occur.  Threshold and
 * duration will not be used uless the type is MOT or NMOT.
 *
 * @param config configuration to apply to, suspend or resume
 * @param irq_type The type of IRQ.  Valid values are
 * - MPU_SLAVE_IRQ_TYPE_NONE
 * - MPU_SLAVE_IRQ_TYPE_MOTION
 * - MPU_SLAVE_IRQ_TYPE_DATA_READY
 */
static int lis331dlh_set_irq(void *mlsl_handle,
			     struct ext_slave_platform_data *pdata,
			     struct lis331dlh_config *config,
			     int apply, long irq_type)
{
	int result = INV_SUCCESS;
	unsigned char reg1;
	unsigned char reg2;
/* shmds add -> */
	static unsigned char irq_flag = 1;
/* shmds add <- */
	config->irq_type = (unsigned char)irq_type;
	if (irq_type == MPU_SLAVE_IRQ_TYPE_DATA_READY) {
/* shmds add -> */
		if (irq_flag == 0) {
			enable_irq(pdata->irq);
 			irq_flag = 1;
		}
/* shmds add <- */
		reg1 = 0x02;
		reg2 = 0x00;
	} else if (irq_type == MPU_SLAVE_IRQ_TYPE_MOTION) {
/* shmds add -> */
		if (irq_flag == 1) {
			disable_irq_nosync(pdata->irq);
			irq_flag = 0;
		}
/* shmds add <- */
		reg1 = 0x00;
		reg2 = config->mot_int1_cfg;
	} else {
/* shmds add -> */
		if (irq_flag == 0) {
			enable_irq(pdata->irq);
			irq_flag = 1;
		}
/* shmds add <- */
		reg1 = 0x00;
		reg2 = 0x00;
	}

	if (apply) {
		result = inv_serial_single_write(mlsl_handle, pdata->address,
						 LIS331DLH_CTRL_REG3, reg1);
		result = inv_serial_single_write(mlsl_handle, pdata->address,
						 LIS331DLH_INT1_CFG, reg2);
	}

	return result;
}

/**
 * Set the Output data rate for the particular configuration
 *
 * @param config Config to modify with new ODR
 * @param odr Output data rate in units of 1/1000Hz
 */
static int lis331dlh_set_odr(void *mlsl_handle,
			     struct ext_slave_platform_data *pdata,
			     struct lis331dlh_config *config,
			     int apply, long odr)
{
	unsigned char bits;
	int result = INV_SUCCESS;

	/* normal power modes */
	if (odr > 400000) {
		config->odr = 1000000;
		bits = LIS331DLH_PWR_MODE_NORMAL | 0x18;
	} else if (odr > 100000) {
		config->odr = 400000;
		bits = LIS331DLH_PWR_MODE_NORMAL | 0x10;
	} else if (odr > 50000) {
		config->odr = 100000;
		bits = LIS331DLH_PWR_MODE_NORMAL | 0x08;
	} else if (odr > 10000) {
		config->odr = 50000;
		bits = LIS331DLH_PWR_MODE_NORMAL | 0x00;
	/* low power modes */
	} else if (odr > 5000) {
		config->odr = 10000;
		bits = 0xC0;
	} else if (odr > 2000) {
		config->odr = 5000;
		bits = 0xA0;
	} else if (odr > 1000) {
		config->odr = 2000;
		bits = 0x80;
	} else if (odr > 500) {
		config->odr = 1000;
		bits = 0x60;
	} else if (odr > 0) {
		config->odr = 500;
		bits = 0x40;
	} else {
		config->odr = 0;
		bits = 0;
	}
/* shmds add -> */
#ifdef SHMDS_DETECT
	if (odr == 1000) {
		config->odr = 5000;
		bits = 0xA0;
		shmds_detect_switch = DETECT_ON;
	} else{
		shmds_detect_switch = DETECT_OFF;
	}
/* shmds add <- */
#endif /* SHMDS_DETECT */

	config->ctrl_reg1 = bits | (config->ctrl_reg1 & 0x7);
	lis331dlh_set_dur(mlsl_handle, pdata, config, apply, config->dur);
	MPL_LOGV("ODR: %d, 0x%02x\n", config->odr, (int)config->ctrl_reg1);
	if (apply)
		result = inv_serial_single_write(mlsl_handle, pdata->address,
						 LIS331DLH_CTRL_REG1,
						 config->ctrl_reg1);
	return result;
}

/**
 * Set the full scale range of the accels
 *
 * @param config pointer to configuration
 * @param fsr requested full scale range
 */
static int lis331dlh_set_fsr(void *mlsl_handle,
			     struct ext_slave_platform_data *pdata,
			     struct lis331dlh_config *config,
			     int apply, long fsr)
{
	unsigned char reg1 = 0x40;
	int result = INV_SUCCESS;

	if (fsr <= 2048) {
		config->fsr = 2048;
	} else if (fsr <= 4096) {
		reg1 |= 0x30;
		config->fsr = 4096;
	} else {
		reg1 |= 0x10;
		config->fsr = 8192;
	}

	lis331dlh_set_ths(mlsl_handle, pdata, config, apply, config->ths);
	MPL_LOGV("FSR: %d\n", config->fsr);
	if (apply)
		result = inv_serial_single_write(mlsl_handle, pdata->address,
						 LIS331DLH_CTRL_REG4, reg1);

	return result;
}

static int lis331dlh_suspend(void *mlsl_handle,
			     struct ext_slave_descr *slave,
			     struct ext_slave_platform_data *pdata)
{
	int result = INV_SUCCESS;
	unsigned char reg1;
	unsigned char reg2;
	struct lis331dlh_private_data *private_data =
		(struct lis331dlh_private_data *)(pdata->private_data);

/* shmds add -> */
#ifdef SHMDS_DETECT
	int i;
	if ((atomic_read(&shmds_detect_mode_flg) == SHMDS_DETECT_ECONOMIZE)){
		atomic_set(&shmds_detect_mode_flg, SHMDS_DETECT_NORMAL);
		shmds_count = 0;
		for (i = 0; i < 3; i++)
		{
			memset(shmds_detect_buf[i], 0, (shmds_buf_size * sizeof(signed short)));
		}
		memset(shmds_economize_buf, 0, sizeof(shmds_economize_buf));
	}
#endif /* SHMDS_DETECT */
/* shmds add <- */

	result = inv_serial_single_write(mlsl_handle, pdata->address,
					 LIS331DLH_CTRL_REG1,
					 private_data->suspend.ctrl_reg1);

	result = inv_serial_single_write(mlsl_handle, pdata->address,
					 LIS331DLH_CTRL_REG2, 0x0f);
	reg1 = 0x40;
	if (private_data->suspend.fsr == 8192)
		reg1 |= 0x30;
	else if (private_data->suspend.fsr == 4096)
		reg1 |= 0x10;
	/* else bits [4..5] are already zero */

	result = inv_serial_single_write(mlsl_handle, pdata->address,
					 LIS331DLH_CTRL_REG4, reg1);
	result = inv_serial_single_write(mlsl_handle, pdata->address,
					 LIS331DLH_INT1_THS,
					 private_data->suspend.reg_ths);
	result = inv_serial_single_write(mlsl_handle, pdata->address,
					 LIS331DLH_INT1_DURATION,
					 private_data->suspend.reg_dur);

	if (private_data->suspend.irq_type == MPU_SLAVE_IRQ_TYPE_DATA_READY) {
		reg1 = 0x02;
		reg2 = 0x00;
	} else if (private_data->suspend.irq_type ==
		   MPU_SLAVE_IRQ_TYPE_MOTION) {
		reg1 = 0x00;
		reg2 = private_data->suspend.mot_int1_cfg;
	} else {
		reg1 = 0x00;
		reg2 = 0x00;
	}
	result = inv_serial_single_write(mlsl_handle, pdata->address,
					 LIS331DLH_CTRL_REG3, reg1);
	result = inv_serial_single_write(mlsl_handle, pdata->address,
					 LIS331DLH_INT1_CFG, reg2);
	result = inv_serial_read(mlsl_handle, pdata->address,
				 LIS331DLH_HP_FILTER_RESET, 1, &reg1);
	return result;
}

static int lis331dlh_resume(void *mlsl_handle,
			    struct ext_slave_descr *slave,
			    struct ext_slave_platform_data *pdata)
{
	int result = INV_SUCCESS;
	unsigned char reg1;
	unsigned char reg2;
	struct lis331dlh_private_data *private_data =
		(struct lis331dlh_private_data *)(pdata->private_data);

/* shmds add -> */
#ifdef SHMDS_DETECT
	int i;
	if ((atomic_read(&shmds_detect_mode_flg) == SHMDS_DETECT_ECONOMIZE)){
		atomic_set(&shmds_detect_mode_flg,SHMDS_DETECT_NORMAL);
		shmds_count = 0;
		for (i = 0; i < 3; i++)
		{
			memset(shmds_detect_buf[i], 0, (shmds_buf_size * sizeof(signed short)));
		}
		memset(shmds_economize_buf, 0, sizeof(shmds_economize_buf));
	}
#endif /* SHMDS_DETECT */
/* shmds add <- */		

	result = inv_serial_single_write(mlsl_handle, pdata->address,
					 LIS331DLH_CTRL_REG1,
					 private_data->resume.ctrl_reg1);
	if (result) {
		LOG_RESULT_LOCATION(result);
		return result;
	}
#if 0 /* shmds mod -> */
		msleep(6);
#else
		msleep(20);
#endif /* <- shmds mod */

	/* Full Scale */
	reg1 = 0x40;
	if (private_data->resume.fsr == 8192)
		reg1 |= 0x30;
	else if (private_data->resume.fsr == 4096)
		reg1 |= 0x10;

	result = inv_serial_single_write(mlsl_handle, pdata->address,
					 LIS331DLH_CTRL_REG4, reg1);
	if (result) {
		LOG_RESULT_LOCATION(result);
		return result;
	}

	/* Configure high pass filter */
	result = inv_serial_single_write(mlsl_handle, pdata->address,
					 LIS331DLH_CTRL_REG2, 0x0F);
	if (result) {
		LOG_RESULT_LOCATION(result);
		return result;
	}

	if (private_data->resume.irq_type == MPU_SLAVE_IRQ_TYPE_DATA_READY) {
		reg1 = 0x02;
		reg2 = 0x00;
	} else if (private_data->resume.irq_type == MPU_SLAVE_IRQ_TYPE_MOTION) {
		reg1 = 0x00;
		reg2 = private_data->resume.mot_int1_cfg;
	} else {
		reg1 = 0x00;
		reg2 = 0x00;
	}
	result = inv_serial_single_write(mlsl_handle, pdata->address,
					 LIS331DLH_CTRL_REG3, reg1);
	if (result) {
		LOG_RESULT_LOCATION(result);
		return result;
	}
	result = inv_serial_single_write(mlsl_handle, pdata->address,
					 LIS331DLH_INT1_THS,
					 private_data->resume.reg_ths);
	if (result) {
		LOG_RESULT_LOCATION(result);
		return result;
	}
	result = inv_serial_single_write(mlsl_handle, pdata->address,
					 LIS331DLH_INT1_DURATION,
					 private_data->resume.reg_dur);
	if (result) {
		LOG_RESULT_LOCATION(result);
		return result;
	}
	result = inv_serial_single_write(mlsl_handle, pdata->address,
					 LIS331DLH_INT1_CFG, reg2);
	if (result) {
		LOG_RESULT_LOCATION(result);
		return result;
	}
	result = inv_serial_read(mlsl_handle, pdata->address,
				 LIS331DLH_HP_FILTER_RESET, 1, &reg1);
	if (result) {
		LOG_RESULT_LOCATION(result);
		return result;
	}

	msleep(20);	/* shmds add */

	return result;
}

static int lis331dlh_read(void *mlsl_handle,
			  struct ext_slave_descr *slave,
			  struct ext_slave_platform_data *pdata,
			  unsigned char *data)
{
/* shmds add -> */
#ifdef SHMDS_DETECT
	int i,j;
	unsigned char temp;
	signed short sub[3] = {0,0,0};
	signed short max, min, mes_data[6];
	signed short keepdata = 0;
#endif /* SHMDS_DETECT */
/* shmds add <- */

	int result = INV_SUCCESS;
	result = inv_serial_read(mlsl_handle, pdata->address,
				 LIS331DLH_STATUS_REG, 1, data);
/* shmds add -> */
	if (result) {
		LOG_RESULT_LOCATION(result);
		return result;
	}
/* shmds add <- */
	if (data[0] & 0x0F) {
		result = inv_serial_read(mlsl_handle, pdata->address,
					 slave->read_reg, slave->read_len,
					 data);
/* shmds add -> */
		if (result) {
			LOG_RESULT_LOCATION(result);
			return result;
		}
#ifdef SHMDS_DETECT
		if (shmds_buf_size != shmds_buf_size_temp){
			for (i = 0; i < 3; i++) {
				kfree(shmds_detect_buf[i]);
				shmds_detect_buf[i] = NULL;
			}
			
			for (i = 0; i < 3; i++) {
				shmds_detect_buf[i] = (signed short *)kmalloc(shmds_buf_size_temp * sizeof(signed short), GFP_KERNEL);
				if (NULL == shmds_detect_buf[i]) {
					for (j = 0; j < i; j ++) {
						kfree(shmds_detect_buf[j]);
					}
					return -1;
				}
				memset(shmds_detect_buf[i], 0, shmds_buf_size_temp * sizeof(signed short));
			}
			shmds_buf_size = shmds_buf_size_temp;
		}
		
		if ((atomic_read(&shmds_detect_mode_flg) == SHMDS_DETECT_NORMAL) && (shmds_detect_switch == DETECT_ON)){
			mes_data[0] = data[0];
			mes_data[1] = data[1];
			mes_data[2] = data[2];
			mes_data[3] = data[3];
			mes_data[4] = data[4];
			mes_data[5] = data[5];

			keepdata = (((mes_data[0] << 4) & 0x0FF0) | ((mes_data[1] >> 4) & 0x000F));
			if(((keepdata & 0x0FFF) >> 11) & 0x01){
				shmds_detect_buf[0][shmds_count] = keepdata | 0xF000;
			}else{
				shmds_detect_buf[0][shmds_count] = keepdata & 0x0FFF;
			}
			
			keepdata = (((mes_data[2] << 4) & 0x0FF0) | ((mes_data[3] >> 4) & 0x000F));
			if(((keepdata & 0x0FFF) >> 11) & 0x01){
				shmds_detect_buf[1][shmds_count] = keepdata | 0xF000;
			}else{
				shmds_detect_buf[1][shmds_count] = keepdata & 0x0FFF;
			}
			
			keepdata = (((mes_data[4] << 4) & 0x0FF0) | ((mes_data[5] >> 4) & 0x000F));
			if(((keepdata & 0x0FFF) >> 11) & 0x01){
				shmds_detect_buf[2][shmds_count] = keepdata | 0xF000;
			}else{
				shmds_detect_buf[2][shmds_count] = keepdata & 0x0FFF;
			}
		
			for (i = 0; i < 3; i++) {
				max = -2048;
				min = 2047;
					
				for (j = 0; j < shmds_buf_size; j++) {
					if (shmds_detect_buf[i][j] > max)
						max = shmds_detect_buf[i][j];
					if (shmds_detect_buf[i][j] < min)
						min = shmds_detect_buf[i][j];
				}
				sub[i] = max - min;
			}
			
				
			if ((sub[0] <= shmds_md_thresh) && (sub[1] <= shmds_md_thresh)) {
				result = inv_serial_single_write(mlsl_handle, pdata->address,
							LIS331DLH_INT1_THS, shmds_ths_value);
				if (result) {
					LOG_RESULT_LOCATION(result);
					return result;
				}
				result = inv_serial_single_write(mlsl_handle, pdata->address,
							LIS331DLH_INT1_CFG, 0x0A);
				if (result) {
					LOG_RESULT_LOCATION(result);
					return result;
				}
				result = inv_serial_single_write(mlsl_handle, pdata->address,
							LIS331DLH_CTRL_REG3, 0x00);
				if (result) {
					LOG_RESULT_LOCATION(result);
					return result;
				}
				result = inv_serial_single_write(mlsl_handle, pdata->address,
							LIS331DLH_INT1_DURATION, shmds_duration_value);
				if (result) {
					LOG_RESULT_LOCATION(result);
					return result;
				}
				result = inv_serial_read(mlsl_handle, pdata->address,
							LIS331DLH_HP_FILTER_RESET, 1, &temp);
				if (result) {
					LOG_RESULT_LOCATION(result);
					return result;
				}
				atomic_set(&shmds_detect_mode_flg, SHMDS_DETECT_ECONOMIZE);
				shmds_economize_buf[0] = shmds_detect_buf[0][shmds_count];
				shmds_economize_buf[1] = shmds_detect_buf[1][shmds_count];
				shmds_economize_buf[2] = shmds_detect_buf[2][shmds_count];
				shmds_count = 0;
				for (i = 0; i < 3; i++) {
					memset(shmds_detect_buf[i], 0, (shmds_buf_size * sizeof(signed short)));
				}
				return result;
			}

			shmds_count = (shmds_count+1) % shmds_buf_size;
		}
		else if (atomic_read(&shmds_detect_mode_flg) == SHMDS_DETECT_ECONOMIZE) {
			result = inv_serial_read(mlsl_handle, pdata->address,
						LIS331DLH_INT1_SRC, 1, &temp);
			if (((temp & 0x40)!=0) && (((temp & 0x08) != 0) || ((temp & 0x02) != 0))) {
				result = inv_serial_single_write(mlsl_handle, pdata->address,
							LIS331DLH_INT1_THS, shmds_ths_value);
				if (result) {
					LOG_RESULT_LOCATION(result);
					return result;
				}
				result = inv_serial_single_write(mlsl_handle, pdata->address,
							LIS331DLH_INT1_CFG, 0x00);
				if (result) {
					LOG_RESULT_LOCATION(result);
					return result;
				}
				result = inv_serial_single_write(mlsl_handle, pdata->address,
							LIS331DLH_CTRL_REG3, 0x02);
				if (result) {
					LOG_RESULT_LOCATION(result);
					return result;
				}
				result = inv_serial_single_write(mlsl_handle, pdata->address,
							LIS331DLH_INT1_DURATION, shmds_duration_value);
				if (result) {
					LOG_RESULT_LOCATION(result);
					return result;
				}

				atomic_set(&shmds_detect_mode_flg,SHMDS_DETECT_NORMAL);
				shmds_count = 0;
				for (i = 0; i < 3; i++) {
					memset(shmds_detect_buf[i], 0, (shmds_buf_size * sizeof(signed short)));
				}
			}
			else {
				mes_data[0] = data[0];
				mes_data[1] = data[1];
				mes_data[2] = data[2];
				mes_data[3] = data[3];
				mes_data[4] = data[4];
				mes_data[5] = data[5];

				keepdata = (((mes_data[0] << 4) & 0x0FF0) | ((mes_data[1] >> 4) & 0x000F));
				if(((keepdata & 0x0FFF) >> 11) & 0x01){
					sub[0] = keepdata | 0xF000;
				}else{
					sub[0] = keepdata & 0x0FFF;
				}
				
				keepdata = (((mes_data[2] << 4) & 0x0FF0) | ((mes_data[3] >> 4) & 0x000F));
				if(((keepdata & 0x0FFF) >> 11) & 0x01){
					sub[1] = keepdata | 0xF000;
				}else{
					sub[1] = keepdata & 0x0FFF;
				}
				
				keepdata = (((mes_data[4] << 4) & 0x0FF0) | ((mes_data[5] >> 4) & 0x000F));
				if(((keepdata & 0x0FFF) >> 11) & 0x01){
					sub[2] = keepdata | 0xF000;
				}else{
					sub[2] = keepdata & 0x0FFF;
				}
				
				for (i = 0; i < 3; i++) {
					sub[i] = abs(sub[i] - shmds_economize_buf[i]);
				}
				if ((sub[0] > shmds_md_thresh2) || (sub[1] > shmds_md_thresh2)) {
					result = inv_serial_single_write(mlsl_handle, pdata->address,
								LIS331DLH_INT1_THS, shmds_ths_value);
					if (result) {
						LOG_RESULT_LOCATION(result);
						return result;
					}
					result = inv_serial_single_write(mlsl_handle, pdata->address,
								LIS331DLH_INT1_CFG, 0x00);
					if (result) {
						LOG_RESULT_LOCATION(result);
						return result;
					}
					result = inv_serial_single_write(mlsl_handle, pdata->address,
								LIS331DLH_CTRL_REG3, 0x02);
					if (result) {
						LOG_RESULT_LOCATION(result);
						return result;
					}
					result = inv_serial_single_write(mlsl_handle, pdata->address,
								LIS331DLH_INT1_DURATION, shmds_duration_value);
					if (result) {
						LOG_RESULT_LOCATION(result);
						return result;
					}

					atomic_set(&shmds_detect_mode_flg,SHMDS_DETECT_NORMAL);
					shmds_count = 0;
					for (i = 0; i < 3; i++) {
						memset(shmds_detect_buf[i], 0, (shmds_buf_size * sizeof(signed short)));
					}
					memset(shmds_economize_buf, 0, sizeof(shmds_economize_buf));
				}
			}
		}
#endif /* SHMDS_DETECT */
/* shmds add <- */
		return result;
	} else
		return INV_ERROR_ACCEL_DATA_NOT_READY;
}

static int lis331dlh_init(void *mlsl_handle,
			  struct ext_slave_descr *slave,
			  struct ext_slave_platform_data *pdata)
{
	struct lis331dlh_private_data *private_data;
	long range;
#ifdef SHMDS_DETECT
	int i, j;
#endif /* SHMDS_DETECT */

	private_data = (struct lis331dlh_private_data *)
	    kzalloc(sizeof(struct lis331dlh_private_data), GFP_KERNEL);

/* shmds add */
#ifdef SHMDS_DETECT
	for (i = 0; i < 3; i++) {
		shmds_detect_buf[i] = (signed short *)kmalloc(shmds_buf_size * sizeof(signed short), GFP_KERNEL);
		if (NULL == shmds_detect_buf[i]) {
			for (j = 0; j < i; j ++) {
				kfree(shmds_detect_buf[j]);
			}
			return -1;
		}
		memset(shmds_detect_buf[i], 0, shmds_buf_size * sizeof(signed short));
	}
#endif /* SHMDS_DETECT */
/* shmds add */

	if (!private_data)
		return INV_ERROR_MEMORY_EXAUSTED;

	pdata->private_data = private_data;

	private_data->resume.ctrl_reg1 = 0x37;
	private_data->suspend.ctrl_reg1 = 0x47;
	private_data->resume.mot_int1_cfg = 0x95;
	private_data->suspend.mot_int1_cfg = 0x2a;

	lis331dlh_set_odr(mlsl_handle, pdata, &private_data->suspend, false, 0);
	lis331dlh_set_odr(mlsl_handle, pdata, &private_data->resume,
			  false, 200000);

	range = range_fixedpoint_to_long_mg(slave->range);
	lis331dlh_set_fsr(mlsl_handle, pdata, &private_data->suspend,
			false, range);
	lis331dlh_set_fsr(mlsl_handle, pdata, &private_data->resume,
			false, range);

	lis331dlh_set_ths(mlsl_handle, pdata, &private_data->suspend,
			  false, 80);
	lis331dlh_set_ths(mlsl_handle, pdata, &private_data->resume, false, 40);


	lis331dlh_set_dur(mlsl_handle, pdata, &private_data->suspend,
			  false, 1000);
	lis331dlh_set_dur(mlsl_handle, pdata, &private_data->resume,
			  false, 2540);

	lis331dlh_set_irq(mlsl_handle, pdata, &private_data->suspend,
			  false, MPU_SLAVE_IRQ_TYPE_NONE);
	lis331dlh_set_irq(mlsl_handle, pdata, &private_data->resume,
			  false, MPU_SLAVE_IRQ_TYPE_NONE);
	return INV_SUCCESS;
}

static int lis331dlh_exit(void *mlsl_handle,
			  struct ext_slave_descr *slave,
			  struct ext_slave_platform_data *pdata)
{
#ifdef SHMDS_DETECT
	int i;
	
	for (i = 0; i < 3; i++) {
		kfree(shmds_detect_buf[i]);
		shmds_detect_buf[i] = NULL;
	}
#endif /* SHMDS_DETECT */

	kfree(pdata->private_data);
	return INV_SUCCESS;
}

static int lis331dlh_config(void *mlsl_handle,
			    struct ext_slave_descr *slave,
			    struct ext_slave_platform_data *pdata,
			    struct ext_slave_config *data)
{
	struct lis331dlh_private_data *private_data = pdata->private_data;
	if (!data->data)
		return INV_ERROR_INVALID_PARAMETER;

	switch (data->key) {
	case MPU_SLAVE_CONFIG_ODR_SUSPEND:
		return lis331dlh_set_odr(mlsl_handle, pdata,
					 &private_data->suspend,
					 data->apply, *((long *)data->data));
	case MPU_SLAVE_CONFIG_ODR_RESUME:
		return lis331dlh_set_odr(mlsl_handle, pdata,
					 &private_data->resume,
					 data->apply, *((long *)data->data));
	case MPU_SLAVE_CONFIG_FSR_SUSPEND:
		return lis331dlh_set_fsr(mlsl_handle, pdata,
					 &private_data->suspend,
					 data->apply, *((long *)data->data));
	case MPU_SLAVE_CONFIG_FSR_RESUME:
		return lis331dlh_set_fsr(mlsl_handle, pdata,
					 &private_data->resume,
					 data->apply, *((long *)data->data));
	case MPU_SLAVE_CONFIG_MOT_THS:
		return lis331dlh_set_ths(mlsl_handle, pdata,
					 &private_data->suspend,
					 data->apply, *((long *)data->data));
	case MPU_SLAVE_CONFIG_NMOT_THS:
		return lis331dlh_set_ths(mlsl_handle, pdata,
					 &private_data->resume,
					 data->apply, *((long *)data->data));
	case MPU_SLAVE_CONFIG_MOT_DUR:
		return lis331dlh_set_dur(mlsl_handle, pdata,
					 &private_data->suspend,
					 data->apply, *((long *)data->data));
	case MPU_SLAVE_CONFIG_NMOT_DUR:
		return lis331dlh_set_dur(mlsl_handle, pdata,
					 &private_data->resume,
					 data->apply, *((long *)data->data));
	case MPU_SLAVE_CONFIG_IRQ_SUSPEND:
		return lis331dlh_set_irq(mlsl_handle, pdata,
					 &private_data->suspend,
					 data->apply, *((long *)data->data));
	case MPU_SLAVE_CONFIG_IRQ_RESUME:
		return lis331dlh_set_irq(mlsl_handle, pdata,
					 &private_data->resume,
					 data->apply, *((long *)data->data));
	default:
		LOG_RESULT_LOCATION(INV_ERROR_FEATURE_NOT_IMPLEMENTED);
		return INV_ERROR_FEATURE_NOT_IMPLEMENTED;
	};

	return INV_SUCCESS;
}

static int lis331dlh_get_config(void *mlsl_handle,
				struct ext_slave_descr *slave,
				struct ext_slave_platform_data *pdata,
				struct ext_slave_config *data)
{
	struct lis331dlh_private_data *private_data = pdata->private_data;
	if (!data->data)
		return INV_ERROR_INVALID_PARAMETER;

	switch (data->key) {
	case MPU_SLAVE_CONFIG_ODR_SUSPEND:
		(*(unsigned long *)data->data) =
		    (unsigned long)private_data->suspend.odr;
		break;
	case MPU_SLAVE_CONFIG_ODR_RESUME:
		(*(unsigned long *)data->data) =
		    (unsigned long)private_data->resume.odr;
		break;
	case MPU_SLAVE_CONFIG_FSR_SUSPEND:
		(*(unsigned long *)data->data) =
		    (unsigned long)private_data->suspend.fsr;
		break;
	case MPU_SLAVE_CONFIG_FSR_RESUME:
		(*(unsigned long *)data->data) =
		    (unsigned long)private_data->resume.fsr;
		break;
	case MPU_SLAVE_CONFIG_MOT_THS:
		(*(unsigned long *)data->data) =
		    (unsigned long)private_data->suspend.ths;
		break;
	case MPU_SLAVE_CONFIG_NMOT_THS:
		(*(unsigned long *)data->data) =
		    (unsigned long)private_data->resume.ths;
		break;
	case MPU_SLAVE_CONFIG_MOT_DUR:
		(*(unsigned long *)data->data) =
		    (unsigned long)private_data->suspend.dur;
		break;
	case MPU_SLAVE_CONFIG_NMOT_DUR:
		(*(unsigned long *)data->data) =
		    (unsigned long)private_data->resume.dur;
		break;
	case MPU_SLAVE_CONFIG_IRQ_SUSPEND:
		(*(unsigned long *)data->data) =
		    (unsigned long)private_data->suspend.irq_type;
		break;
	case MPU_SLAVE_CONFIG_IRQ_RESUME:
		(*(unsigned long *)data->data) =
		    (unsigned long)private_data->resume.irq_type;
		break;
	default:
		LOG_RESULT_LOCATION(INV_ERROR_FEATURE_NOT_IMPLEMENTED);
		return INV_ERROR_FEATURE_NOT_IMPLEMENTED;
	};

	return INV_SUCCESS;
}

static struct ext_slave_descr lis331dlh_descr = {
	.init             = lis331dlh_init,
	.exit             = lis331dlh_exit,
	.suspend          = lis331dlh_suspend,
	.resume           = lis331dlh_resume,
	.read             = lis331dlh_read,
	.config           = lis331dlh_config,
	.get_config       = lis331dlh_get_config,
	.name             = "lis331dlh",
	.type             = EXT_SLAVE_TYPE_ACCEL,
	.id               = ACCEL_ID_LIS331,
	.read_reg         = (0x28 | 0x80), /* 0x80 for burst reads */
	.read_len         = 6,
	.endian           = EXT_SLAVE_BIG_ENDIAN,
	.range            = {2, 480},
	.trigger          = NULL,
};

static
struct ext_slave_descr *lis331_get_slave_descr(void)
{
	return &lis331dlh_descr;
}

/* -------------------------------------------------------------------------- */
struct lis331_mod_private_data {
	struct i2c_client *client;
	struct ext_slave_platform_data *pdata;
};

static unsigned short normal_i2c[] = { I2C_CLIENT_END };

static int lis331_mod_probe(struct i2c_client *client,
			   const struct i2c_device_id *devid)
{
	struct ext_slave_platform_data *pdata;
	struct lis331_mod_private_data *private_data;
	int result = 0;
	
/* shmds add -> */
	int ii =0;
#if defined (CONFIG_SHMDS_POLARITY_1)
	signed char acc_orient[9] = { 1,  0,  0, 
								  0,  1,  0, 
								  0,  0,  1 };
#else
	signed char acc_orient[9] =	{ 1,  0,  0,
								  0,  1,  0,
								  0,  0,  1 };
#endif /* defined (CONFIG_SHMDS_POLARITY_1) */
/* <- shmds add */

	dev_info(&client->adapter->dev, "%s: %s\n", __func__, devid->name);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		result = -ENODEV;
		goto out_no_free;
	}

	pdata = client->dev.platform_data;
	if (!pdata) {
		dev_err(&client->adapter->dev,
			"Missing platform data for slave %s\n", devid->name);
		result = -EFAULT;
		goto out_no_free;
	}

/* shmds add -> */
	for (ii = 0; ii < ARRAY_SIZE(pdata->orientation); ii++){
		pdata->orientation[ii] = acc_orient[ii];
	}

#ifdef CONFIG_MPU_SENSORS_DEBUG
	printk("shmds debug pdata 0 : %8ld, %8ld, %8ld\n",
		 pdata->orientation[0], pdata->orientation[1],pdata->orientation[2]);
	printk("shmds debug pdata 1 : %8ld, %8ld, %8ld\n",
		 pdata->orientation[3], pdata->orientation[4],pdata->orientation[5]);
	printk("shmds debug pdata 2 : %8ld, %8ld, %8ld\n",
		 pdata->orientation[6], pdata->orientation[7],pdata->orientation[8]);
#endif	/* CONFIG_MPU_SENSORS_DEBUG */
/* <- shmds add */

	private_data = kzalloc(sizeof(*private_data), GFP_KERNEL);
	if (!private_data) {
		result = -ENOMEM;
		goto out_no_free;
	}

	i2c_set_clientdata(client, private_data);
	private_data->client = client;
	private_data->pdata = pdata;

	result = inv_mpu_register_slave(THIS_MODULE, client, pdata,
					lis331_get_slave_descr);
	if (result) {
		dev_err(&client->adapter->dev,
			"Slave registration failed: %s, %d\n",
			devid->name, result);
		goto out_free_memory;
	}

	return result;

out_free_memory:
	kfree(private_data);
out_no_free:
	dev_err(&client->adapter->dev, "%s failed %d\n", __func__, result);
	return result;

}

static int lis331_mod_remove(struct i2c_client *client)
{
	struct lis331_mod_private_data *private_data =
		i2c_get_clientdata(client);

	dev_dbg(&client->adapter->dev, "%s\n", __func__);

	inv_mpu_unregister_slave(client, private_data->pdata,
				lis331_get_slave_descr);

	kfree(private_data);
	return 0;
}

static const struct i2c_device_id lis331_mod_id[] = {
	{ "lis331", ACCEL_ID_LIS331 },
	{}
};

MODULE_DEVICE_TABLE(i2c, lis331_mod_id);

static struct i2c_driver lis331_mod_driver = {
	.class = I2C_CLASS_HWMON,
	.probe = lis331_mod_probe,
	.remove = lis331_mod_remove,
	.id_table = lis331_mod_id,
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "lis331_mod",
		   },
	.address_list = normal_i2c,
};

static int __init lis331_mod_init(void)
{
	int res = i2c_add_driver(&lis331_mod_driver);
	pr_info("%s: Probe name %s\n", __func__, "lis331_mod");
	if (res)
		pr_err("%s failed\n", __func__);
	return res;
}

static void __exit lis331_mod_exit(void)
{
	pr_info("%s\n", __func__);
	i2c_del_driver(&lis331_mod_driver);
}

module_init(lis331_mod_init);
module_exit(lis331_mod_exit);

MODULE_AUTHOR("Invensense Corporation");
MODULE_DESCRIPTION("Driver to integrate LIS331 sensor with the MPU");
MODULE_LICENSE("GPL");
MODULE_ALIAS("lis331_mod");

/**
 *  @}
 */
