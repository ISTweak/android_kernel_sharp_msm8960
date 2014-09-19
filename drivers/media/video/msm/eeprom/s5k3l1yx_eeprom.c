/* Copyright (c) 2012, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include "msm_camera_eeprom.h"
#include "msm_camera_i2c.h"

DEFINE_MUTEX(s5k3l1yx_eeprom_mutex);
static struct msm_eeprom_ctrl_t s5k3l1yx_eeprom_t;

static const struct i2c_device_id s5k3l1yx_eeprom_i2c_id[] = {
	{"s5k3l1yx_eeprom", (kernel_ulong_t)&s5k3l1yx_eeprom_t},
	{ }
};

static struct i2c_driver s5k3l1yx_eeprom_i2c_driver = {
	.id_table = s5k3l1yx_eeprom_i2c_id,
	.probe  = msm_eeprom_i2c_probe,
	.remove = __exit_p(s5k3l1yx_eeprom_i2c_remove),
	.driver = {
		.name = "s5k3l1yx_eeprom",
	},
};

static int __init s5k3l1yx_eeprom_i2c_add_driver(void)
{
	int rc = 0;
	rc = i2c_add_driver(s5k3l1yx_eeprom_t.i2c_driver);
	return rc;
}

static struct v4l2_subdev_core_ops s5k3l1yx_eeprom_subdev_core_ops = {
	.ioctl = msm_eeprom_subdev_ioctl,
};

static struct v4l2_subdev_ops s5k3l1yx_eeprom_subdev_ops = {
	.core = &s5k3l1yx_eeprom_subdev_core_ops,
};

uint8_t s5k3l1yx_sensvty_calib_data[2];
uint8_t s5k3l1yx_spec_sensvty_calib_data[12];
uint8_t s5k3l1yx_af_calib_data[6];
uint8_t s5k3l1yx_bl_calib_data[39];
uint8_t s5k3l1yx_dpc_calib_data[1152];
uint8_t s5k3l1yx_msc_calib_data[256];
struct msm_calib_af s5k3l1yx_af_data;
struct msm_calib_wb s5k3l1yx_wb_data;
struct msm_calib_dpc s5k3l1yx_dpc_data;


static struct msm_camera_eeprom_info_t s5k3l1yx_calib_supp_info = {
//	{TRUE, sizeof(struct msm_calib_af), 0, 1},
	{FALSE, sizeof(struct msm_calib_af), 0, 1},
/* SHLOCAL_CAMERA_IMAGE_QUALITY-> */
/* QUALCOMM NADESHIKO MERGE -> */
	{FALSE, sizeof(struct msm_calib_wb), 1, 1},
	{FALSE, 0, 0, 1},
	{TRUE, sizeof(struct msm_calib_dpc), 2, 1},
/* QUALCOMM NADESHIKO MERGE <- */
/* SHLOCAL_CAMERA_IMAGE_QUALITY<- */
};

static struct msm_camera_eeprom_read_t s5k3l1yx_eeprom_read_tbl[] = {
	{0x050A3E, &s5k3l1yx_sensvty_calib_data[0], 2, 0},
	{0x050A1E, &s5k3l1yx_spec_sensvty_calib_data[0], 12, 0},
	{0x050A14, &s5k3l1yx_af_calib_data[0], 6, 0},
	{0x060A06, &s5k3l1yx_bl_calib_data[0], 5, 0}, /* Gain 1X mode Still*/
	{0x060A0E, &s5k3l1yx_bl_calib_data[5], 5, 0}, /* Gain 16X mode Still*/
	{0x060A16, &s5k3l1yx_bl_calib_data[10], 5, 0}, /* Gain 1X mode B*/
	{0x060A1E, &s5k3l1yx_bl_calib_data[15], 5, 0}, /* Gain 1X mode B*/
	{0x060A36, &s5k3l1yx_bl_calib_data[20], 5, 0}, /* Gain 1X mode D*/
	{0x060A3E, &s5k3l1yx_bl_calib_data[25], 5, 0}, /* Gain 1X mode D*/
	{0x050A2E, &s5k3l1yx_bl_calib_data[30], 5, 0}, /* Temp Comp*/
	{0x050A36, &s5k3l1yx_bl_calib_data[35], 4, 0}, /* Temp Slope*/
	{0x100A04, &s5k3l1yx_dpc_calib_data[0], 64, 0},
	{0x110A04, &s5k3l1yx_dpc_calib_data[64], 64, 0},
	{0x120A04, &s5k3l1yx_dpc_calib_data[128], 64, 0},
	{0x130A04, &s5k3l1yx_dpc_calib_data[192], 64, 0},
	{0x140A04, &s5k3l1yx_dpc_calib_data[256], 64, 0},
	{0x150A04, &s5k3l1yx_dpc_calib_data[320], 64, 0},
	{0x160A04, &s5k3l1yx_dpc_calib_data[384], 64, 0},
	{0x170A04, &s5k3l1yx_dpc_calib_data[448], 64, 0},
	{0x180A04, &s5k3l1yx_dpc_calib_data[512], 64, 0},
	{0x190A04, &s5k3l1yx_dpc_calib_data[576], 64, 0},
	{0x1A0A04, &s5k3l1yx_dpc_calib_data[640], 64, 0},
	{0x1B0A04, &s5k3l1yx_dpc_calib_data[704], 64, 0},
	{0x1C0A04, &s5k3l1yx_dpc_calib_data[768], 64, 0},
	{0x1D0A04, &s5k3l1yx_dpc_calib_data[832], 64, 0},
	{0x0C0A04, &s5k3l1yx_dpc_calib_data[896], 64, 0},
	{0x0D0A04, &s5k3l1yx_dpc_calib_data[960], 64, 0},
	{0x0E0A04, &s5k3l1yx_dpc_calib_data[1024], 64, 0},
	{0x0F0A04, &s5k3l1yx_dpc_calib_data[1088], 64, 0},

	{0x080A04, &s5k3l1yx_msc_calib_data[0], 64, 0},
	{0x090A04, &s5k3l1yx_msc_calib_data[64], 64, 0},
	{0x0A0A04, &s5k3l1yx_msc_calib_data[128], 64, 0},
	{0x0B0A04, &s5k3l1yx_msc_calib_data[192], 64, 0},
};


static struct msm_camera_eeprom_data_t s5k3l1yx_eeprom_data_tbl[] = {
/* SHLOCAL_CAMERA_IMAGE_QUALITY-> */
/* QUALCOMM NADESHIKO MERGE -> */
	{&s5k3l1yx_af_data, sizeof(struct msm_calib_af)},
	{&s5k3l1yx_wb_data,sizeof(s5k3l1yx_wb_data)},
	{&s5k3l1yx_dpc_data, sizeof(struct msm_calib_dpc)},

	{&s5k3l1yx_spec_sensvty_calib_data[0],sizeof(s5k3l1yx_spec_sensvty_calib_data)},
	{&s5k3l1yx_sensvty_calib_data[0],sizeof(s5k3l1yx_sensvty_calib_data)},
	{&s5k3l1yx_bl_calib_data[0],sizeof(s5k3l1yx_bl_calib_data)},
	{&s5k3l1yx_dpc_calib_data[0],sizeof(s5k3l1yx_dpc_calib_data)},
	{&s5k3l1yx_msc_calib_data[0],sizeof(s5k3l1yx_msc_calib_data)},
/* QUALCOMM NADESHIKO MERGE <- */
/* SHLOCAL_CAMERA_IMAGE_QUALITY<- */
};

static void s5k3l1yx_format_afdata(void)
{
	uint16_t vcmcode_msb;
	int i;

	for(i=0; i < sizeof(s5k3l1yx_af_calib_data);i++){
		CDBG("%s s5k3l1yx_af_calib_data[%d]=0x%0x\n", __func__, i, s5k3l1yx_af_calib_data[i]);
	}

	if(s5k3l1yx_af_calib_data[0] == 0x55){
		vcmcode_msb = s5k3l1yx_af_calib_data[2];

		s5k3l1yx_af_data.macro_dac =
			(( vcmcode_msb & 0x0030) << 4) | s5k3l1yx_af_calib_data[3];
		s5k3l1yx_af_data.start_dac =
			(( vcmcode_msb & 0x0003) << 8)| s5k3l1yx_af_calib_data[5];
		s5k3l1yx_af_data.inf_dac =
			(( vcmcode_msb & 0x000C) << 6)| s5k3l1yx_af_calib_data[4];
	}
	CDBG("%s startDac =%d MacroDac =%d 50cm = %d\n", __func__,
		s5k3l1yx_af_data.start_dac, s5k3l1yx_af_data.macro_dac,
		s5k3l1yx_af_data.inf_dac);
}

#define DPC_SIZE 287
struct pixel_t dpc_cord[DPC_SIZE];
struct pixel_t snapshot_dpc_cord[DPC_SIZE];

static void s5k3l1yx_format_dpcdata(void)
{
	uint16_t dpc_count = 0;
	int i;
	uint16_t msb_xcord, lsb_xcord, msb_ycord, lsb_ycord;
	uint16_t defect1_count;
	uint16_t yaddr_start = 0x1AC;

	for (i=0; i<DPC_SIZE-63; i++) {
		msb_ycord = s5k3l1yx_dpc_calib_data[i*4];
		lsb_ycord = s5k3l1yx_dpc_calib_data[i*4+1];
		msb_xcord = s5k3l1yx_dpc_calib_data[i*4+2];
		lsb_xcord = s5k3l1yx_dpc_calib_data[i*4+3];
		dpc_cord[i].x = (msb_xcord << 8) | lsb_xcord;
		dpc_cord[i].y = (msb_ycord << 8) | lsb_ycord;
	}

	for (i=0; i<DPC_SIZE-63; i++)
		if (!((dpc_cord[i].x ==0) && (dpc_cord[i].y ==0)))
			snapshot_dpc_cord[dpc_count++] = dpc_cord[i];

	defect1_count = dpc_count;
	CDBG("%s dpc_count = %d\n", __func__, dpc_count);
	for (i=0; i<dpc_count; i++) 
		CDBG("snapshot_dpc_cord[%d] X= %d Y = %d \n",i, snapshot_dpc_cord[i].x, snapshot_dpc_cord[i].y);

//MODE B
	for (i=0; i<dpc_count; i++) {
		s5k3l1yx_dpc_data.preview_coord[i].x = ((snapshot_dpc_cord[i].x /4)*2) + (snapshot_dpc_cord[i].x % 2);
		s5k3l1yx_dpc_data.preview_coord[i].y = (((3015 -snapshot_dpc_cord[i].y) /4)*2) + (snapshot_dpc_cord[i].y % 2);
		s5k3l1yx_dpc_data.video_coord[i].x = ((snapshot_dpc_cord[i].x /4)*2) + (snapshot_dpc_cord[i].x % 2);
		s5k3l1yx_dpc_data.video_coord[i].y = (((3015 -snapshot_dpc_cord[i].y) /4)*2) + (snapshot_dpc_cord[i].y % 2);
	}

//MODE D
	for (i=0; i<dpc_count; i++) {
		s5k3l1yx_dpc_data.video60_coord[i].x = ((snapshot_dpc_cord[i].x /4)*2) + (snapshot_dpc_cord[i].x % 2);
		s5k3l1yx_dpc_data.video60_coord[i].y = ((((3015-snapshot_dpc_cord[i].y) - yaddr_start)/8)*2) + (((3015-snapshot_dpc_cord[i].y) - yaddr_start) % 2);
	}

//MODE STL
	for (i=0; i<dpc_count; i++) {
		s5k3l1yx_dpc_data.snapshot_coord[i].x = snapshot_dpc_cord[i].x;
		s5k3l1yx_dpc_data.snapshot_coord[i].y = 3015 - snapshot_dpc_cord[i].y;
	}

//DPC2
	CDBG("%s Black Defect Flag = 0x%0x \n", __func__, s5k3l1yx_dpc_calib_data[896]);
	if (s5k3l1yx_dpc_calib_data[896] == 0x55) {
		for (i=0; i<63; i++) {
			msb_xcord = s5k3l1yx_dpc_calib_data[896+4 + i*4];
			lsb_xcord = s5k3l1yx_dpc_calib_data[896+5 + i*4];
			lsb_ycord = s5k3l1yx_dpc_calib_data[896+6 + i*4];
			dpc_cord[DPC_SIZE-63 + i].x = ((msb_xcord&0xF0) << 4) | lsb_xcord;
			dpc_cord[DPC_SIZE-63 + i].y = ((msb_xcord&0x0F) << 8) | lsb_ycord;
		}
  
		for (i=DPC_SIZE-63; i<DPC_SIZE; i++)
			if (!((dpc_cord[i].x ==0) && (dpc_cord[i].y ==0)))
				snapshot_dpc_cord[dpc_count++] = dpc_cord[i];
	}

//MODE B
	for (i=defect1_count; i<dpc_count; i++) {
		s5k3l1yx_dpc_data.preview_coord[i].x = ((snapshot_dpc_cord[i].x /4)*2) + (snapshot_dpc_cord[i].x % 2);
		s5k3l1yx_dpc_data.preview_coord[i].y = ((snapshot_dpc_cord[i].y /4)*2) + (snapshot_dpc_cord[i].y % 2);
		s5k3l1yx_dpc_data.video_coord[i].x = ((snapshot_dpc_cord[i].x /4)*2) + (snapshot_dpc_cord[i].x % 2);
		s5k3l1yx_dpc_data.video_coord[i].y = ((snapshot_dpc_cord[i].y /4)*2) + (snapshot_dpc_cord[i].y % 2);
	}

//MODE D
	for (i=defect1_count; i<dpc_count; i++) {
		s5k3l1yx_dpc_data.video60_coord[i].x = ((snapshot_dpc_cord[i].x /4)*2) + (snapshot_dpc_cord[i].x % 2);
		s5k3l1yx_dpc_data.video60_coord[i].y = (((snapshot_dpc_cord[i].y - yaddr_start)/8)*2) + ((snapshot_dpc_cord[i].y - yaddr_start)% 2);
	}

	for (i=defect1_count; i<dpc_count; i++) {
		s5k3l1yx_dpc_data.snapshot_coord[i].x = snapshot_dpc_cord[i].x;
		s5k3l1yx_dpc_data.snapshot_coord[i].y = snapshot_dpc_cord[i].y;
	}

	CDBG("%s Accending order \n", __func__);
	for (i=0; i<dpc_count; i++)
		CDBG("snapshot_dpc_cord[%d] X= %d Y = %d \n", i, s5k3l1yx_dpc_data.snapshot_coord[i].x, s5k3l1yx_dpc_data.snapshot_coord[i].y);

	for (i=0; i<dpc_count; i++)
		CDBG("preview_dpc_cord[%d] X= %d Y = %d \n", i, s5k3l1yx_dpc_data.preview_coord[i].x, s5k3l1yx_dpc_data.preview_coord[i].y);

	for (i=0; i<dpc_count; i++)
		CDBG("video_dpc_cord[%d] X= %d Y = %d \n", i, s5k3l1yx_dpc_data.video_coord[i].x, s5k3l1yx_dpc_data.video_coord[i].y);

	for (i=0; i<dpc_count; i++)
		CDBG("video60_coord[%d] X= %d Y = %d \n", i, s5k3l1yx_dpc_data.video60_coord[i].x, s5k3l1yx_dpc_data.video60_coord[i].y);

	s5k3l1yx_dpc_data.validcount = dpc_count;
	CDBG("%s dpc_count =%d \n", __func__, dpc_count);
}

/* SHLOCAL_CAMERA_IMAGE_QUALITY-> */
/* QUALCOMM NADESHIKO MERGE -> */
static void s5k3l1yx_format_wbdata(void)
{
	uint16_t wb_msb;
	uint32_t r, gr, gb, b;
	uint32_t g_max = 0;

	wb_msb = s5k3l1yx_spec_sensvty_calib_data[0];
	r = ((wb_msb & 0x00C0) << 2) | s5k3l1yx_spec_sensvty_calib_data[1];
	gr = ((wb_msb & 0x0030) << 4) | s5k3l1yx_spec_sensvty_calib_data[2];
	b = ((wb_msb & 0x000C) << 6) | s5k3l1yx_spec_sensvty_calib_data[3];
	gb = ((wb_msb & 0x0003) << 8) | s5k3l1yx_spec_sensvty_calib_data[4];

	CDBG("%s s5k3l1yx_spec_sensvty_calib_data[0] =%d r =%d gr = %d gb =%d, b=%d\n",
	__func__, wb_msb, r, gr, gb, b);

	if(gr > gb) g_max = gr;
	else        g_max = gb;
	s5k3l1yx_wb_data.r_over_g = (r * 256) / g_max;
	s5k3l1yx_wb_data.b_over_g = (b * 256) / g_max;
	s5k3l1yx_wb_data.gr_over_gb = (gr * 256) / gb;
	CDBG("%s r_over_g =%d b_over_g =%d gr_over_gb = %d\n", __func__,
		s5k3l1yx_wb_data.r_over_g, s5k3l1yx_wb_data.b_over_g,
		s5k3l1yx_wb_data.gr_over_gb);
}
/* QUALCOMM NADESHIKO MERGE <- */
/* SHLOCAL_CAMERA_IMAGE_QUALITY<- */

void s5k3l1yx_format_calibrationdata(void)
{
	s5k3l1yx_format_afdata();
/* SHLOCAL_CAMERA_IMAGE_QUALITY-> */
/* QUALCOMM NADESHIKO MERGE -> */
	s5k3l1yx_format_wbdata();
/* QUALCOMM NADESHIKO MERGE <- */
/* SHLOCAL_CAMERA_IMAGE_QUALITY<- */
	s5k3l1yx_format_dpcdata();
}

void s5k3l1yx_set_dev_addr(struct msm_eeprom_ctrl_t *ectrl,
	uint32_t* reg_addr) {
	int32_t rc = 0;

	rc = msm_camera_i2c_write(&ectrl->i2c_client, 0x0A02,
		(*reg_addr) >> 16, MSM_CAMERA_I2C_BYTE_DATA);
	if (rc != 0) {
		CDBG("%s: Page write error\n", __func__);
		return;
	}
	(*reg_addr) = (*reg_addr) & 0xFFFF;

	rc = msm_camera_i2c_write(&ectrl->i2c_client, 0x0A00, 0x01,
		MSM_CAMERA_I2C_BYTE_DATA);
	if (rc != 0) {
		CDBG("%s: Ctl write error\n", __func__);
		return;
	}

	rc = msm_camera_i2c_poll(&ectrl->i2c_client, 0x0A01, 0x01,
		MSM_CAMERA_I2C_SET_BYTE_MASK);
	if (rc != 0) {
		CDBG("%s: Read Status2 error\n", __func__);
		return;
	}

}

int32_t s5k3l1yx_eeprom_init(struct msm_eeprom_ctrl_t *ectrl,
		struct i2c_adapter *adapter) {
	int32_t rc = 0;
	rc = msm_camera_i2c_write(&ectrl->i2c_client, 0x0A00, 0x04,
		MSM_CAMERA_I2C_BYTE_DATA);
	if (rc != 0) {
		CDBG("%s: Ctl write error\n", __func__);
		return 0;
	}

	rc = msm_camera_i2c_write(&ectrl->i2c_client, 0x3B1B, 0x00,
		MSM_CAMERA_I2C_BYTE_DATA);
	if (rc != 0) {
		CDBG("%s: Ctl write error\n", __func__);
		return 0;
	}

	return 0;
}

int32_t s5k3l1yx_eeprom_release(struct msm_eeprom_ctrl_t *ectrl)
{
	int32_t rc = 0;
	rc = msm_camera_i2c_write(&ectrl->i2c_client, 0x0A00, 0x04,
		MSM_CAMERA_I2C_BYTE_DATA);
	if (rc != 0) {
		CDBG("%s: Ctl write error\n", __func__);
		return 0;
	}
	return 0;
}

static struct msm_eeprom_ctrl_t s5k3l1yx_eeprom_t = {
	.i2c_driver = &s5k3l1yx_eeprom_i2c_driver,
	.i2c_addr = 0x20,
	.eeprom_v4l2_subdev_ops = &s5k3l1yx_eeprom_subdev_ops,

	.i2c_client = {
		.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
	},

	.eeprom_mutex = &s5k3l1yx_eeprom_mutex,

	.func_tbl = {
		.eeprom_init = s5k3l1yx_eeprom_init,
		.eeprom_release = s5k3l1yx_eeprom_release,
		.eeprom_get_info = msm_camera_eeprom_get_info,
		.eeprom_get_data = msm_camera_eeprom_get_data,
		.eeprom_set_dev_addr = s5k3l1yx_set_dev_addr,
		.eeprom_format_data = s5k3l1yx_format_calibrationdata,
	},
	.info = &s5k3l1yx_calib_supp_info,
	.info_size = sizeof(struct msm_camera_eeprom_info_t),
	.read_tbl = s5k3l1yx_eeprom_read_tbl,
	.read_tbl_size = ARRAY_SIZE(s5k3l1yx_eeprom_read_tbl),
	.data_tbl = s5k3l1yx_eeprom_data_tbl,
	.data_tbl_size = ARRAY_SIZE(s5k3l1yx_eeprom_data_tbl),
};

subsys_initcall(s5k3l1yx_eeprom_i2c_add_driver);
MODULE_DESCRIPTION("s5k3l1yx EEPROM");
MODULE_LICENSE("GPL v2");
