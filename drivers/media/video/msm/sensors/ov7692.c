/* Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
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

#include "msm_sensor.h"
#include "msm.h"
#include "msm_ispif.h"


DEFINE_MUTEX(ov7692_mut);
static struct msm_sensor_ctrl_t ov7692_s_ctrl;
#define SENSOR_NAME "ov7692"

/*=============================================================
	SENSOR REGISTER DEFINES
==============================================================*/
#define Q8    0x00000100

/* Omnivision8810 product ID register address */
#define REG_OV7692_MODEL_ID_MSB                       0x0A
#define REG_OV7692_MODEL_ID_LSB                       0x0B

#define OV7692_MODEL_ID                       0x7692
/* Omnivision8810 product ID */

/* Time in milisecs for waiting for the sensor to reset */
#define OV7692_RESET_DELAY_MSECS    66
#define OV7692_DEFAULT_CLOCK_RATE   24000000
/* Registers*/

/* Color bar pattern selection */
#define OV7692_COLOR_BAR_PATTERN_SEL_REG     0x82
/* Color bar enabling control */
#define OV7692_COLOR_BAR_ENABLE_REG           0x601
/* Time in milisecs for waiting for the sensor to reset*/
#define OV7692_RESET_DELAY_MSECS    66

#define OV7692_SENSOR_MCLK_27HZ     27000000

/*============================================================================
							DATA DECLARATIONS
============================================================================*/
/*  96MHz PCLK @ 24MHz MCLK */

struct msm_camera_i2c_reg_conf ov7692_init_module_settings_array[] = {
    {0x12, 0x80},
    {0x0e, 0x08},
};

struct msm_camera_i2c_reg_conf ov7692_init_settings_array[] = {
    {0x69,   0x52},
    {0x1e,   0xb3},
    {0x48,   0x42},
    {0xff,   0x01},
    {0xae,   0xa0},
    {0xa8,   0x25},
    {0xb4,   0xc0},
    {0xb5,   0x40},
    {0x87,   0x50},
    {0x86,   0x48},
    {0x80,   0x24},
    {0xff,   0x00},
#if defined(CONFIG_MACH_TOR) || defined(CONFIG_MACH_DECKARD_GP4) || defined(CONFIG_MACH_MNB) || defined(CONFIG_MACH_LYNX_DL12) || defined(CONFIG_MACH_LYNX_DL10) || defined(CONFIG_MACH_LYNX_DL15) || defined(CONFIG_MACH_BLT)
    {0x0c,   0xc0},
#else
    {0x0c,   0x00},
#endif /* defined(CONFIG_MACH_TOR) */
    {0x62,   0x10},
    {0x12,   0x00},
    {0x17,   0x69},
    {0x18,   0xa4},
    {0x19,   0x0c},
    {0x1a,   0xf6},
    {0x3e,   0x30},
    {0x64,   0x0a},
    {0xff,   0x01},
    {0xb4,   0xc0},
    {0xff,   0x00},
    {0x67,   0x20},
    {0x81,   0x3f},
    {0xcc,   0x02},
    {0xcd,   0x80},
    {0xce,   0x01},
    {0xcf,   0xe0},
    {0xc8,   0x02},
    {0xc9,   0x80},
    {0xca,   0x01},
    {0xcb,   0xe0},
    {0xd0,   0x48},
    {0x82,   0x03},
    {0x70,   0x00},
    {0x71,   0x34},
    {0x74,   0x28},
    {0x75,   0x98},
    {0x76,   0x00},
    {0x77,   0x64},
    {0x78,   0x01},
    {0x79,   0xc2},
    {0x7a,   0x4e},
    {0x7b,   0x1f},
    {0x7c,   0x00},
#if defined(CONFIG_MACH_LYNX_DL12) || defined(CONFIG_MACH_LYNX_DL10) || defined(CONFIG_MACH_LYNX_DL15)
    {0x11,   0x01},
#else
    {0x11,   0x00},
#endif /* CONFIG_MACH_LYNX_DL10 */
    {0x20,   0x00},
    {0x21,   0x23},
    {0x50,   0x9a},
    {0x51,   0x80},
    {0x4c,   0x7d},
    {0x80,   0x7f},
    {0x85,   0x90},
    {0x86,   0x00},
    {0x87,   0xa0},
    {0x88,   0x10},
    {0x89,   0x08},
    {0x8a,   0x00},
    {0x8b,   0x00},
    {0xbb,   0xc8},
    {0xbc,   0xa4},
    {0xbd,   0x1d},
    {0xbe,   0x44},
    {0xbf,   0x84},
    {0xc0,   0xc8},
    {0xc1,   0x1e},
    {0xb7,   0x01},
    {0xb8,   0x1e},
    {0xb9,   0xff},
    {0xba,   0x78},
    {0x5a,   0x14},
    {0x5b,   0xa2},
    {0x5c,   0x70},
    {0x5d,   0x20},
#if defined(CONFIG_MACH_MNB) || defined(CONFIG_MACH_LYNX_DL12) || defined(CONFIG_MACH_LYNX_DL10) || defined(CONFIG_MACH_LYNX_DL15)
    {0x24,   0x5b},
#else /* CONFIG_MACH_MNB */
    {0x24,   0x78},
#endif /* CONFIG_MACH_MNB */
    {0x25,   0x4f},
    {0x26,   0x72},
    {0xa3,   0x10},
    {0xa4,   0x12},
    {0xa5,   0x35},
    {0xa6,   0x5a},
    {0xa7,   0x69},
    {0xa8,   0x76},
    {0xa9,   0x80},
    {0xaa,   0x88},
    {0xab,   0x8f},
    {0xac,   0x96},
    {0xad,   0xa3},
    {0xae,   0xaf},
    {0xaf,   0xc4},
    {0xb0,   0xd7},
    {0xb1,   0xe8},
    {0xb2,   0x20},
    {0x8c,   0x5d},
    {0x8d,   0x11},
    {0x8e,   0x92},
    {0x8f,   0x11},
    {0x90,   0x50},
    {0x91,   0x22},
    {0x92,   0xd1},
    {0x93,   0xa7},
    {0x94,   0x23},
    {0x95,   0x3b},
    {0x96,   0xff},
    {0x97,   0x00},
    {0x98,   0x4a},
    {0x99,   0x46},
    {0x9a,   0x3d},
    {0x9b,   0x3a},
    {0x9c,   0xf0},
    {0x9d,   0xf0},
    {0x9e,   0xf0},
    {0x9f,   0xff},
    {0xa0,   0x56},
    {0xa1,   0x55},
    {0xa2,   0x13},
    {0xb4,   0x08},
    {0xb6,   0x04},
    {0x81,   0x02},
    {0xd2,   0x04},
    {0xd4,   0x2d},
    {0xd5,   0x20},
    {0xd3,   0x00},
    {0x81,   0x04},
    {0xdc,   0x00},
    {0xd2,   0x04},
    {0xd3,   0x00},
    {0x81,   0x02},
    {0xd8,   0x40},
    {0xd9,   0x40},
    {0xb5,   0x06},
    {0x14,   0x19},
    {0x81,   0x3f},
#if defined(CONFIG_MACH_TOR) || defined(CONFIG_MACH_MNB)
    {0x30,   0xc7},
    {0x31,   0x81},
    {0x2a,   0xd0},
    {0x2b,   0x3c},
#elif defined(CONFIG_MACH_LYNX_DL12) || defined(CONFIG_MACH_LYNX_DL10) || defined(CONFIG_MACH_LYNX_DL15)
    {0x30,   0xa6},
    {0x31,   0x02},
    {0x2a,   0xc0},
    {0x2b,   0xcc},
#else
    {0x30,   0x47},
    {0x31,   0x81},
    {0x2a,   0xe0},
    {0x2b,   0x4b},
#endif /* defined(CONFIG_MACH_TOR) */
    {0x15,   0x00},
    {0x68,   0xb2},
    {0x5d,   0x22},
    {0x81,   0x2d},
    {0x0e,   0x00},
};

struct msm_camera_i2c_reg_conf ov7692_init_param_settings_array[] = {
    {0xd3,   0x00},
    {0xd2,   0x04},
    {0xdc,   0x00},
};

static struct msm_camera_i2c_conf_array ov7692_init_conf[] = {
	{&ov7692_init_module_settings_array[0],
	ARRAY_SIZE(ov7692_init_module_settings_array), 0, MSM_CAMERA_I2C_BYTE_DATA}
};

static struct msm_camera_i2c_conf_array ov7692_prev_conf[] = {
	{&ov7692_init_settings_array[0],
	ARRAY_SIZE(ov7692_init_settings_array), 0, MSM_CAMERA_I2C_BYTE_DATA}
};

static struct msm_camera_i2c_conf_array ov7692_param_conf[] = {
	{&ov7692_init_param_settings_array[0],
	ARRAY_SIZE(ov7692_init_param_settings_array), 0, MSM_CAMERA_I2C_BYTE_DATA}
};

static struct msm_sensor_output_info_t ov7692_dimensions[] = {
	{
		.x_output = 0x280,
		.y_output = 0x1E0,
		.line_length_pclk = 0x290,
		.frame_length_lines = 0x1E0,
		.vt_pixel_clk = 200000000,
		.op_pixel_clk = 200000000,
		.binning_factor = 1,
	},
};

static struct msm_camera_csid_vc_cfg ov7692_cid_cfg[] = {
	{0, 0x1E, CSI_DECODE_8BIT},
	{1, CSI_EMBED_DATA, CSI_DECODE_8BIT},
};

static struct msm_camera_csi2_params ov7692_csi_params = {
	.csid_params = {
		.lane_assign = 0xe4,
		.lane_cnt = 1,
		.lut_params = {
			.num_cid = ARRAY_SIZE(ov7692_cid_cfg),
			.vc_cfg = ov7692_cid_cfg,
		},
	},
	.csiphy_params = {
		.lane_cnt = 1,
		.settle_cnt = 0x0a,
	},
};

static struct msm_camera_csi2_params *ov7692_csi_params_array[] = {
	&ov7692_csi_params,
};

static int ov7692_sensor_config(struct msm_sensor_ctrl_t *s_ctrl, void __user *argp)
{
	struct sensor_cfg_data cdata;
	long   rc = 0;
	if (copy_from_user(&cdata,
		(void *)argp,
		sizeof(struct sensor_cfg_data)))
		return -EFAULT;
	mutex_lock(&ov7692_mut);
	CDBG("ov7692_sensor_config: cfgtype = %d\n",
	cdata.cfgtype);
	switch (cdata.cfgtype) {
	case CFG_SHDIAG_GET_I2C_DATA:
		{
			void *data;
			data = kmalloc(cdata.cfg.i2c_info.length, GFP_KERNEL);
			if(data == NULL){
				return -EFAULT;
			}
			CDBG("%s:%d i2c_read addr=0x%0x\n",__func__,__LINE__,cdata.cfg.i2c_info.addr);
			rc = msm_camera_i2c_read_seq(s_ctrl->sensor_i2c_client, cdata.cfg.i2c_info.addr , data, cdata.cfg.i2c_info.length);
			CDBG("%s:%d i2c_read data=0x%0x\n",__func__,__LINE__,*(unsigned char *)data);
			if (copy_to_user((void *)cdata.cfg.i2c_info.data,
				data,
				cdata.cfg.i2c_info.length)){
				kfree(data);
				CDBG("%s copy_to_user error\n",__func__);
				return -EFAULT;
			}
			kfree(data);
			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
			return -EFAULT;
		}
			break;
	case CFG_SHDIAG_SET_I2C_DATA:
		{
			void *data;
			data = kmalloc(cdata.cfg.i2c_info.length, GFP_KERNEL);
			if(data == NULL){
				return -EFAULT;
			}
			if (copy_from_user(data,
				(void *)cdata.cfg.i2c_info.data,
				cdata.cfg.i2c_info.length)){
				kfree(data);
				CDBG("%s copy_to_user error\n",__func__);
				return -EFAULT;
			}
			rc = msm_camera_i2c_write_seq(s_ctrl->sensor_i2c_client, cdata.cfg.i2c_info.addr, data, cdata.cfg.i2c_info.length);
			kfree(data);
			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
			return -EFAULT;
		}
			break;
#if 1 /* custom */
	case CFG_SHDIAG_SET_I2C_TABLE_DATA:
		{
			int i=0;
			void *data;
			struct msm_camera_i2c_reg_conf *wk = NULL;
			struct msm_camera_i2c_conf_array conf_array[] = {
				{NULL, (cdata.cfg.i2c_info.length/2), 0, MSM_CAMERA_I2C_BYTE_DATA}
			};
			CDBG("%s:%d CFG_SHDIAG_SET_I2C_TABLE_DATA cdata.cfg.i2c_info.length = %d\n",__func__,__LINE__,cdata.cfg.i2c_info.length);
			data = kmalloc(cdata.cfg.i2c_info.length, GFP_KERNEL);
			if(data == NULL){
				return -EFAULT;
			}
			if (copy_from_user(data,
				(void *)cdata.cfg.i2c_info.data,
				cdata.cfg.i2c_info.length)){
				kfree(data);
				CDBG("%s copy_to_user error\n",__func__);
				return -EFAULT;
			}
			conf_array[0].conf = (struct msm_camera_i2c_reg_conf *)kmalloc(
				(sizeof(struct msm_camera_i2c_reg_conf)*(cdata.cfg.i2c_info.length/2)),
				GFP_KERNEL);
			if(conf_array[0].conf == NULL){
				kfree(data);
				return -EFAULT;
			}
			wk = conf_array[0].conf;
			for(i=0; i<cdata.cfg.i2c_info.length; i++) {
				wk->reg_addr = (uint16_t)((uint8_t *)data)[i++];
				wk->reg_data = (uint16_t)((uint8_t *)data)[i];
				wk->dt       = MSM_CAMERA_I2C_BYTE_DATA;
				wk->cmd_type = MSM_CAMERA_I2C_CMD_WRITE;
				wk++;
			}
			msm_sensor_write_conf_array(s_ctrl->sensor_i2c_client, &conf_array[0] ,0);
			kfree(conf_array[0].conf);
			kfree(data);
		}
			break;
#endif /* custom */
	default:
		mutex_unlock(&ov7692_mut);
		return msm_sensor_config(s_ctrl, argp);
		break;
	}
	
	mutex_unlock(&ov7692_mut);
	return rc;

}

static int32_t ov7692_sensor_setting(struct msm_sensor_ctrl_t *s_ctrl,
			int update_type, int res)
{
	int32_t rc = 0;

	s_ctrl->func_tbl->sensor_stop_stream(s_ctrl);
	msleep(30);
	if (update_type == MSM_SENSOR_REG_INIT) {
		s_ctrl->curr_csi_params = NULL;
		msm_sensor_enable_debugfs(s_ctrl);
		msm_sensor_write_conf_array(s_ctrl->sensor_i2c_client, &ov7692_param_conf[0] ,0);
	} else if (update_type == MSM_SENSOR_UPDATE_PERIODIC) {
		msm_sensor_write_conf_array(s_ctrl->sensor_i2c_client, &ov7692_prev_conf[0] ,0);
		if (s_ctrl->curr_csi_params != s_ctrl->csi_params[res]) {
			s_ctrl->curr_csi_params = s_ctrl->csi_params[res];
			s_ctrl->curr_csi_params->csid_params.lane_assign =
				s_ctrl->sensordata->sensor_platform_info->
				csi_lane_params->csi_lane_assign;
			s_ctrl->curr_csi_params->csiphy_params.lane_mask =
				s_ctrl->sensordata->sensor_platform_info->
				csi_lane_params->csi_lane_mask;
			v4l2_subdev_notify(&s_ctrl->sensor_v4l2_subdev,
				NOTIFY_CSID_CFG,
				&s_ctrl->curr_csi_params->csid_params);
			mb();
			v4l2_subdev_notify(&s_ctrl->sensor_v4l2_subdev,
				NOTIFY_CSIPHY_CFG,
				&s_ctrl->curr_csi_params->csiphy_params);
			mb();
			msleep(20);
		}

		v4l2_subdev_notify(&s_ctrl->sensor_v4l2_subdev,
			NOTIFY_PCLK_CHANGE, &s_ctrl->msm_sensor_reg->
			output_settings[res].op_pixel_clk);
		s_ctrl->func_tbl->sensor_start_stream(s_ctrl);
		msleep(30);
	}
	return rc;
}

static struct v4l2_subdev_info ov7692_subdev_info[] = {
	{
	.code   = V4L2_MBUS_FMT_YUYV8_2X8,
	.colorspace = V4L2_COLORSPACE_JPEG,
	.fmt    = 1,
	.order    = 0,
	},
	/* more can be supported, to be added later */
};

static struct msm_cam_clk_info cam_clk_info[] = {
	{"cam_clk", OV7692_SENSOR_MCLK_27HZ},
};

static int32_t ov7692_sensor_power_up(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	struct msm_camera_sensor_info *data = s_ctrl->sensordata;
	CDBG("%s: %d\n", __func__, __LINE__);
	s_ctrl->reg_ptr = kzalloc(sizeof(struct regulator *)
			* data->sensor_platform_info->num_vreg, GFP_KERNEL);
	if (!s_ctrl->reg_ptr) {
		pr_err("%s: could not allocate mem for regulators\n",
			__func__);
		return -ENOMEM;
	}

	rc = msm_camera_request_gpio_table(data, 1);
	if (rc < 0) {
		pr_err("%s: request gpio failed\n", __func__);
		goto request_gpio_failed;
	}

	rc = msm_camera_config_gpio_table(data, 1);
	if (rc < 0) {
		pr_err("%s: config gpio en failed\n", __func__);
		goto config_gpio_en_failed;
	}

	rc = msm_camera_config_vreg(&s_ctrl->sensor_i2c_client->client->dev,
			s_ctrl->sensordata->sensor_platform_info->cam_vreg,
			s_ctrl->sensordata->sensor_platform_info->num_vreg,
			s_ctrl->reg_ptr, 1);
	if (rc < 0) {
		pr_err("%s: regulator on failed\n", __func__);
		goto config_vreg_failed;
	}

	rc = msm_camera_enable_vreg(&s_ctrl->sensor_i2c_client->client->dev,
			s_ctrl->sensordata->sensor_platform_info->cam_vreg,
			s_ctrl->sensordata->sensor_platform_info->num_vreg,
			s_ctrl->reg_ptr, 1);
	if (rc < 0) {
		pr_err("%s: enable regulator failed\n", __func__);
		goto enable_vreg_failed;
	}

	usleep(1000);

	rc = msm_camera_config_gpio_table(data, 0);
	if (rc < 0) {
		pr_err("%s: config gpio failed\n", __func__);
		goto config_gpio_failed;
	}

	if (s_ctrl->clk_rate != 0)
		cam_clk_info->clk_rate = s_ctrl->clk_rate;

	rc = msm_cam_clk_enable(&s_ctrl->sensor_i2c_client->client->dev,
		cam_clk_info, &s_ctrl->cam_clk, ARRAY_SIZE(cam_clk_info), 1);
	if (rc < 0) {
		pr_err("%s: clk enable failed\n", __func__);
		goto enable_clk_failed;
	}

	msm_sensor_write_init_settings(s_ctrl);

	return rc;
enable_clk_failed:
		msm_camera_config_gpio_table(data, 0);
config_gpio_failed:
	msm_camera_enable_vreg(&s_ctrl->sensor_i2c_client->client->dev,
			s_ctrl->sensordata->sensor_platform_info->cam_vreg,
			s_ctrl->sensordata->sensor_platform_info->num_vreg,
			s_ctrl->reg_ptr, 0);

enable_vreg_failed:
	msm_camera_config_vreg(&s_ctrl->sensor_i2c_client->client->dev,
		s_ctrl->sensordata->sensor_platform_info->cam_vreg,
		s_ctrl->sensordata->sensor_platform_info->num_vreg,
		s_ctrl->reg_ptr, 0);
config_vreg_failed:
	msm_camera_config_gpio_table(data, 0);
config_gpio_en_failed:
	msm_camera_request_gpio_table(data, 0);
request_gpio_failed:
	kfree(s_ctrl->reg_ptr);
	return rc;
}

static int32_t ov7692_sensor_power_down(struct msm_sensor_ctrl_t *s_ctrl)
{
	struct msm_camera_sensor_info *data = s_ctrl->sensordata;

	CDBG("%s\n", __func__);

	msm_camera_config_gpio_table(data, 1);

	msm_cam_clk_enable(&s_ctrl->sensor_i2c_client->client->dev,
		cam_clk_info, &s_ctrl->cam_clk, ARRAY_SIZE(cam_clk_info), 0);

	msm_camera_config_gpio_table(data, 0);

	msm_camera_enable_vreg(&s_ctrl->sensor_i2c_client->client->dev,
		s_ctrl->sensordata->sensor_platform_info->cam_vreg,
		s_ctrl->sensordata->sensor_platform_info->num_vreg,
		s_ctrl->reg_ptr, 0);

	msm_camera_config_vreg(&s_ctrl->sensor_i2c_client->client->dev,
		s_ctrl->sensordata->sensor_platform_info->cam_vreg,
		s_ctrl->sensordata->sensor_platform_info->num_vreg,
		s_ctrl->reg_ptr, 0);

	usleep(1000);

	msm_camera_request_gpio_table(data, 0);

	kfree(s_ctrl->reg_ptr);

	return 0;
}

static struct msm_sensor_id_info_t ov7692_id_info = {
	.sensor_id_reg_addr = REG_OV7692_MODEL_ID_MSB,
	.sensor_id = OV7692_MODEL_ID,
};


static const struct i2c_device_id ov7692_i2c_id[] = {
	{SENSOR_NAME, (kernel_ulong_t)&ov7692_s_ctrl},
	{ }
};

static struct i2c_driver ov7692_i2c_driver = {
	.id_table = ov7692_i2c_id,
	.probe  = msm_sensor_i2c_probe,
	.driver = {
		.name = SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client ov7692_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
};

static int __init msm_sensor_init_module(void)
{
	return i2c_add_driver(&ov7692_i2c_driver);
}

static struct v4l2_subdev_core_ops ov7692_subdev_core_ops = {
	.ioctl = msm_sensor_subdev_ioctl,
	.s_power = msm_sensor_power,
};

static struct v4l2_subdev_video_ops ov7692_subdev_video_ops = {
	.enum_mbus_fmt = msm_sensor_v4l2_enum_fmt,
};

static struct v4l2_subdev_ops ov7692_subdev_ops = {
	.core = &ov7692_subdev_core_ops,
	.video  = &ov7692_subdev_video_ops,
};

static struct msm_sensor_fn_t ov7692_func_tbl = {
	.sensor_start_stream = msm_sensor_start_stream,
	.sensor_stop_stream = msm_sensor_stop_stream,
	.sensor_group_hold_on = NULL,
	.sensor_group_hold_off = NULL,
	.sensor_set_fps = NULL,
	.sensor_write_exp_gain = NULL,
	.sensor_write_snapshot_exp_gain = NULL,
	.sensor_setting = ov7692_sensor_setting,
	.sensor_set_sensor_mode = msm_sensor_set_sensor_mode,
	.sensor_mode_init = msm_sensor_mode_init,
	.sensor_get_output_info = msm_sensor_get_output_info,
	.sensor_config = ov7692_sensor_config,
	.sensor_power_up = ov7692_sensor_power_up,
	.sensor_power_down = ov7692_sensor_power_down,
	.sensor_get_csi_params = msm_sensor_get_csi_params,
};

static struct msm_sensor_reg_t ov7692_regs = {
	.default_data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.start_stream_conf = NULL,
	.start_stream_conf_size = 0,
	.stop_stream_conf = NULL,
	.stop_stream_conf_size = 0,
	.group_hold_on_conf = NULL,
	.group_hold_on_conf_size = 0,
	.group_hold_off_conf = NULL,
	.group_hold_off_conf_size = 0,
	.init_settings = &ov7692_init_conf[0],
	.init_size = ARRAY_SIZE(ov7692_init_conf),
	.mode_settings = NULL,
	.output_settings = &ov7692_dimensions[0],
	.num_conf = 1,
};

static struct msm_sensor_ctrl_t ov7692_s_ctrl = {
	.msm_sensor_reg = &ov7692_regs,
	.sensor_i2c_client = &ov7692_sensor_i2c_client,
	.sensor_i2c_addr = 0x78,
	.sensor_output_reg_addr = NULL,
	.sensor_id_info = &ov7692_id_info,
	.sensor_exp_gain_info = NULL,
	.cam_mode = MSM_SENSOR_MODE_INVALID,
	.csi_params = &ov7692_csi_params_array[0],
	.msm_sensor_mutex = &ov7692_mut,
	.sensor_i2c_driver = &ov7692_i2c_driver,
	.sensor_v4l2_subdev_info = ov7692_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(ov7692_subdev_info),
	.sensor_v4l2_subdev_ops = &ov7692_subdev_ops,
	.func_tbl = &ov7692_func_tbl,
	.clk_rate = OV7692_SENSOR_MCLK_27HZ,
};

module_init(msm_sensor_init_module);

MODULE_DESCRIPTION("OMNI VGA YUV sensor driver");
MODULE_LICENSE("GPL v2");
