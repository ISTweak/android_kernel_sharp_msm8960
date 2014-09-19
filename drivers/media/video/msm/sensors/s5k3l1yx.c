/* Copyright (c) 2012, The Linux Foundation. All rights reserved.
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
#include "linux/leds.h"

#include "msm_sensor.h"
#include "msm.h"
#include "msm_ispif.h"
#include <linux/gpio.h>
#include <mach/gpio.h>
#include <mach/board.h>
#include <mach/gpiomux.h>
#define SENSOR_NAME "s5k3l1yx"
#define PLATFORM_DRIVER_NAME "msm_camera_s5k3l1yx"
#define s5k3l1yx_obj s5k3l1yx_##obj

#define S5K3L1YX_SENSOR_MCLK_6P75HZ  6750000

DEFINE_MUTEX(s5k3l1yx_mut);
static struct msm_sensor_ctrl_t s5k3l1yx_s_ctrl;
static uint8_t s5k3l1yx_curr_mode = 10;

static struct msm_camera_i2c_reg_conf s5k3l1yx_start_settings[] = {
	{0x0100, 0x01},
};

static struct msm_camera_i2c_reg_conf s5k3l1yx_stop_settings[] = {
	{0x0100, 0x00},
};

static struct msm_camera_i2c_reg_conf s5k3l1yx_groupon_settings[] = {
	{0x104, 0x01},
};

static struct msm_camera_i2c_reg_conf s5k3l1yx_groupoff_settings[] = {
	{0x104, 0x00},
};

static struct msm_camera_i2c_reg_conf s5k3l1yx_prev_settings[] = {
	{0x0342, 0x17}, /* line_length_pck */
	{0x0343, 0xC4}, /* line_length_pck */
	{0x0344, 0x00}, /* x_addr_start */
	{0x0345, 0x00}, /* x_addr_start */
	{0x0346, 0x00}, /* y_addr_start */
	{0x0347, 0x00}, /* y_addr_start */
	{0x0348, 0x0F}, /* x_addr_end */
	{0x0349, 0xAF}, /* x_addr_end */
	{0x034A, 0x0B}, /* y_addr_end */
	{0x034B, 0xC7}, /* y_addr_end */
	{0x034C, 0x07}, /* x_output_size */
	{0x034D, 0xD8}, /* x_output_size */
	{0x034E, 0x05}, /* y_output_size */
	{0x034F, 0xE4}, /* y_output_size */
	{0x0380, 0x00}, /* x_even_inc */
	{0x0381, 0x01}, /* x_even_inc */
	{0x0382, 0x00}, /* x_odd_inc */
	{0x0383, 0x03}, /* x_odd_inc */
	{0x0384, 0x00}, /* y_even_inc */
	{0x0385, 0x01}, /* y_even_inc */
	{0x0386, 0x00}, /* y_odd_inc */
	{0x0387, 0x03}, /* y_odd_inc */
	{0x0900, 0x01}, /* binning_mode */
	{0x0901, 0x22}, /* binning_type */
	{0x0902, 0x01}, /* binning_weighting */
};

static struct msm_camera_i2c_reg_conf s5k3l1yx_snap_settings[] = {
	{0x0342, 0x17}, /* line_length_pck */
	{0x0343, 0xC4}, /* line_length_pck */
	{0x0344, 0x00}, /* x_addr_start */
	{0x0345, 0x00}, /* x_addr_start */
	{0x0346, 0x00}, /* y_addr_start */
	{0x0347, 0x00}, /* y_addr_start */
	{0x0348, 0x0F}, /* x_addr_end */
	{0x0349, 0xAF}, /* x_addr_end */
	{0x034A, 0x0B}, /* y_addr_end */
	{0x034B, 0xC7}, /* y_addr_end */
	{0x034C, 0x0F}, /* x_output_size */
	{0x034D, 0xB0}, /* x_output_size */
	{0x034E, 0x0B}, /* y_output_size */
	{0x034F, 0xC8}, /* y_output_size */
	{0x0380, 0x00}, /* x_even_inc */
	{0x0381, 0x01}, /* x_even_inc */
	{0x0382, 0x00}, /* x_odd_inc */
	{0x0383, 0x01}, /* x_odd_inc */
	{0x0384, 0x00}, /* y_even_inc */
	{0x0385, 0x01}, /* y_even_inc */
	{0x0386, 0x00}, /* y_odd_inc */
	{0x0387, 0x01}, /* y_odd_inc */
	{0x0900, 0x00}, /* binning_mode */
	{0x0901, 0x22}, /* binning_type */
	{0x0902, 0x01}, /* binning_weighting */
};

static struct msm_camera_i2c_reg_conf s5k3l1yx_video_60fps_settings[] = {
	{0x0342, 0x11}, /* line_length_pck */
	{0x0343, 0x8C}, /* line_length_pck */
	{0x0344, 0x00}, /* x_addr_start */
	{0x0345, 0x00}, /* x_addr_start */
	{0x0346, 0x01}, /* y_addr_start */
	{0x0347, 0xAC}, /* y_addr_start */
	{0x0348, 0x0F}, /* x_addr_end */
	{0x0349, 0xBF}, /* x_addr_end */
	{0x034A, 0x0A}, /* y_addr_end */
	{0x034B, 0x1B}, /* y_addr_end */
	{0x034C, 0x07}, /* x_output_size */
	{0x034D, 0xE0}, /* x_output_size */
	{0x034E, 0x02}, /* y_output_size */
	{0x034F, 0x1C}, /* y_output_size */
	{0x0380, 0x00}, /* x_even_inc */
	{0x0381, 0x01}, /* x_even_inc */
	{0x0382, 0x00}, /* x_odd_inc */
	{0x0383, 0x03}, /* x_odd_inc */
	{0x0384, 0x00}, /* y_even_inc */
	{0x0385, 0x01}, /* y_even_inc */
	{0x0386, 0x00}, /* y_odd_inc */
	{0x0387, 0x07}, /* y_odd_inc */
	{0x0900, 0x01}, /* binning_mode */
	{0x0901, 0x24}, /* binning_type */
	{0x0902, 0x01}, /* binning_weighting */
};

static struct msm_camera_i2c_reg_conf s5k3l1yx_video_90fps_settings[] = {
	{0x0342, 0x11}, /* line_length_pck */
	{0x0343, 0x8C}, /* line_length_pck */
	{0x0344, 0x00}, /* x_addr_start */
	{0x0345, 0x00}, /* x_addr_start */
	{0x0346, 0x01}, /* y_addr_start */
	{0x0347, 0xAC}, /* y_addr_start */
	{0x0348, 0x0F}, /* x_addr_end */
	{0x0349, 0xBF}, /* x_addr_end */
	{0x034A, 0x0A}, /* y_addr_end */
	{0x034B, 0x1B}, /* y_addr_end */
	{0x034C, 0x07}, /* x_output_size */
	{0x034D, 0xE0}, /* x_output_size */
	{0x034E, 0x02}, /* y_output_size */
	{0x034F, 0x1C}, /* y_output_size */
	{0x0380, 0x00}, /* x_even_inc */
	{0x0381, 0x01}, /* x_even_inc */
	{0x0382, 0x00}, /* x_odd_inc */
	{0x0383, 0x03}, /* x_odd_inc */
	{0x0384, 0x00}, /* y_even_inc */
	{0x0385, 0x01}, /* y_even_inc */
	{0x0386, 0x00}, /* y_odd_inc */
	{0x0387, 0x07}, /* y_odd_inc */
	{0x0900, 0x01}, /* binning_mode */
	{0x0901, 0x24}, /* binning_type */
	{0x0902, 0x01}, /* binning_weighting */
};

static struct msm_camera_i2c_reg_conf s5k3l1yx_video_120fps_settings[] = {
	{0x0342, 0x11}, /* line_length_pck */
	{0x0343, 0x8C}, /* line_length_pck */
	{0x0344, 0x00}, /* x_addr_start */
	{0x0345, 0x00}, /* x_addr_start */
	{0x0346, 0x01}, /* y_addr_start */
	{0x0347, 0xAC}, /* y_addr_start */
	{0x0348, 0x0F}, /* x_addr_end */
	{0x0349, 0xBF}, /* x_addr_end */
	{0x034A, 0x0A}, /* y_addr_end */
	{0x034B, 0x1B}, /* y_addr_end */
	{0x034C, 0x07}, /* x_output_size */
	{0x034D, 0xE0}, /* x_output_size */
	{0x034E, 0x02}, /* y_output_size */
	{0x034F, 0x1C}, /* y_output_size */
	{0x0380, 0x00}, /* x_even_inc */
	{0x0381, 0x01}, /* x_even_inc */
	{0x0382, 0x00}, /* x_odd_inc */
	{0x0383, 0x03}, /* x_odd_inc */
	{0x0384, 0x00}, /* y_even_inc */
	{0x0385, 0x01}, /* y_even_inc */
	{0x0386, 0x00}, /* y_odd_inc */
	{0x0387, 0x07}, /* y_odd_inc */
	{0x0900, 0x01}, /* binning_mode */
	{0x0901, 0x24}, /* binning_type */
	{0x0902, 0x01}, /* binning_weighting */
};

static struct msm_camera_i2c_reg_conf s5k3l1yx_recommend_settings[] = {
	{0x0103, 0x00}, /* software_reset */
	{0x0101, 0x00}, /* image_orientation, mirror & flip off*/
	{0x0104, 0x00}, /* grouped_parameter_hold */
	{0x0105, 0x00}, /* mask_corrupted_frames */
	{0x0114, 0x03}, /* CSI_lane_mode, 4 lane setting */
	{0x0120, 0x00}, /* gain_mode, global analogue gain*/
	{0x0121, 0x00}, /* exposure_mode, global exposure */
	{0x0136, 0x09}, /* Extclk_frequency_mhz */
	{0x0137, 0x00}, /* Extclk_frequency_mhz */
	{0x0200, 0x08}, /* fine_integration_time */
	{0x0201, 0x88}, /* fine_integration_time */
	{0x0204, 0x00}, /* analogue_gain_code_global */
	{0x0205, 0x20}, /* analogue_gain_code_global */
	{0x020E, 0x01}, /* digital_gain_greenR */
	{0x020F, 0x00}, /* digital_gain_greenR */
	{0x0210, 0x01}, /* digital_gain_red */
	{0x0211, 0x00}, /* digital_gain_red */
	{0x0212, 0x01}, /* digital_gain_blue */
	{0x0213, 0x00}, /* digital_gain_blue */
	{0x0214, 0x01}, /* digital_gain_greenB */
	{0x0215, 0x00}, /* digital_gain_greenB */
	{0x0300, 0x00}, /* vt_pix_clk_div */
	{0x0301, 0x02}, /* vt_pix_clk_div */
	{0x0302, 0x00}, /* vt_sys_clk_div */
	{0x0303, 0x01}, /* vt_sys_clk_div */
	{0x0304, 0x00},	/* pre_pll_clk_div */
	{0x0305, 0x02}, /* pre_pll_clk_div */
	{0x0306, 0x00}, /* pll_multiplier */
	{0x0307, 0xB8}, /* pll_multiplier */
	{0x0308, 0x00}, /* op_pix_clk_div */
	{0x0309, 0x01}, /* op_pix_clk_div */
	{0x030A, 0x00}, /* op_sys_clk_div */
	{0x030B, 0x01}, /* op_sys_clk_div */
	{0x0800, 0x00}, /* tclk_post for D-PHY control */
	{0x0801, 0x00}, /* ths_prepare for D-PHY control */
	{0x0802, 0x00}, /* ths_zero_min for D-PHY control */
	{0x0803, 0x00}, /* ths_trail for D-PHY control */
	{0x0804, 0x00}, /* tclk_trail_min for D-PHY control */
	{0x0805, 0x00}, /* tclk_prepare for D-PHY control */
	{0x0806, 0x00}, /* tclk_zero_zero for D-PHY control */
	{0x0807, 0x00}, /* tlpx for D-PHY control */
	{0x0820, 0x02}, /* requested_link_bit_rate_mbps */
	{0x0821, 0x6D}, /* requested_link_bit_rate_mbps */
	{0x0822, 0x00}, /* requested_link_bit_rate_mbps */
	{0x0823, 0x00}, /* requested_link_bit_rate_mbps */
	{0x3000, 0x0A},
	{0x3001, 0xF7},
	{0x3002, 0x0A},
	{0x3003, 0xF7},
	{0x3004, 0x08},
	{0x3005, 0xF8},
	{0x3006, 0x5B},
	{0x3007, 0x73},
	{0x3008, 0x49},
	{0x3009, 0x0C},
	{0x300A, 0xF8},
	{0x300B, 0x4E},
	{0x300C, 0x64},
	{0x300D, 0x5C},
	{0x300E, 0x71},
	{0x300F, 0x0C},
	{0x3010, 0x6A},
	{0x3011, 0x14},
	{0x3012, 0x14},
	{0x3013, 0x0C},
	{0x3014, 0x24},
	{0x3015, 0x4F},
	{0x3016, 0x86},
	{0x3017, 0x0E},
	{0x3018, 0x2C},
	{0x3019, 0x30},
	{0x301A, 0x31},
	{0x301B, 0x32},
	{0x301C, 0xFF},
	{0x301D, 0x33},
	{0x301E, 0x5C},
	{0x301F, 0xFA},
	{0x3020, 0x36},
	{0x3021, 0x47},
	{0x3022, 0x92},
	{0x3023, 0xF5},
	{0x3024, 0x6E},
	{0x3025, 0x19},
	{0x3026, 0x32},
	{0x3027, 0x4B},
	{0x3028, 0x04},
	{0x3029, 0x50},
	{0x302A, 0x0C},
	{0x302B, 0x04},
	{0x302C, 0xEF},
	{0x302D, 0xC1},
	{0x302E, 0x74},
	{0x302F, 0x40},
	{0x3030, 0x00},
	{0x3031, 0x00},
	{0x3032, 0x00},
	{0x3033, 0x00},
	{0x3034, 0x0F},
	{0x3035, 0x01},
	{0x3036, 0x00},
	{0x3037, 0x00},
	{0x3038, 0x88},
	{0x3039, 0x98},
	{0x303A, 0x1F},
	{0x303B, 0x01},
	{0x303C, 0x00},
	{0x303D, 0x03},
	{0x303E, 0x2F},
	{0x303F, 0x09},
	{0x3040, 0xFF},
	{0x3041, 0x22},
	{0x3042, 0x03},
	{0x3043, 0x03},
	{0x3044, 0x20},
	{0x3045, 0x10},
	{0x3046, 0x10},
	{0x3047, 0x08},
	{0x3048, 0x10},
	{0x3049, 0x01},
	{0x304A, 0x00},
	{0x304B, 0x80},
	{0x304C, 0x80},
	{0x304D, 0x00},
	{0x304E, 0x00},
	{0x304F, 0x00},
	{0x3050, 0x01},
	{0x3051, 0x09},
	{0x3052, 0xC4},
	{0x305A, 0xE0},
	{0x3202, 0x01},
	{0x3203, 0x01},
	{0x3204, 0x01},
	{0x3205, 0x01},
	{0x3206, 0x01},
	{0x3207, 0x01},
	{0x320A, 0x05},
	{0x320B, 0x20},
	{0x3235, 0xB7},
	{0x323D, 0x04},
	{0x323E, 0x38},
	{0x324A, 0x07},
	{0x324C, 0x04},
	{0x3300, 0x01},
	{0x3305, 0xDD},
	{0x3900, 0xFF},
	{0x3902, 0x01},
	{0x3914, 0x08},
	{0x3915, 0x2D},
	{0x3916, 0x0E},
	{0x3A00, 0x01},
	{0x3A05, 0x01},
	{0x3A06, 0x03},
	{0x3A07, 0x2B},
	{0x3A09, 0x01},
	{0x3B29, 0x01},
	{0x3C11, 0x08},
	{0x3C12, 0x31},
	{0x3C13, 0x8F},
	{0x3C14, 0x2D},
	{0x3C15, 0x0E},
	{0x3C20, 0x04},
	{0x3C23, 0x03},
	{0x3C24, 0x00},
	{0x3C50, 0x62},
	{0x3C51, 0x85},
	{0x3C53, 0x60},
	{0x3C55, 0xA0},
};

static struct v4l2_subdev_info s5k3l1yx_subdev_info[] = {
	{
	.code   = V4L2_MBUS_FMT_SBGGR10_1X10,
	.colorspace = V4L2_COLORSPACE_JPEG,
	.fmt    = 1,
	.order    = 0,
	},
	/* more can be supported, to be added later */
};

static struct msm_camera_i2c_conf_array s5k3l1yx_init_conf[] = {
	{&s5k3l1yx_recommend_settings[0],
	ARRAY_SIZE(s5k3l1yx_recommend_settings), 0, MSM_CAMERA_I2C_BYTE_DATA}
};

static struct msm_camera_i2c_conf_array s5k3l1yx_confs[] = {
	{&s5k3l1yx_snap_settings[0],
	ARRAY_SIZE(s5k3l1yx_snap_settings), 0, MSM_CAMERA_I2C_BYTE_DATA},
	{&s5k3l1yx_prev_settings[0],
	ARRAY_SIZE(s5k3l1yx_prev_settings), 0, MSM_CAMERA_I2C_BYTE_DATA},
	{&s5k3l1yx_video_60fps_settings[0],
	ARRAY_SIZE(s5k3l1yx_video_60fps_settings), 0, MSM_CAMERA_I2C_BYTE_DATA},
	{&s5k3l1yx_video_90fps_settings[0],
	ARRAY_SIZE(s5k3l1yx_video_90fps_settings), 0, MSM_CAMERA_I2C_BYTE_DATA},
	{&s5k3l1yx_video_120fps_settings[0],
	ARRAY_SIZE(s5k3l1yx_video_120fps_settings), 0,
			MSM_CAMERA_I2C_BYTE_DATA},
};

static struct msm_sensor_output_info_t s5k3l1yx_dimensions[] = {
	/* 15 fps snapshot */
	{
		.x_output = 0x0FB0, /* 4016 */
		.y_output = 0x0BC8, /* 3016 */
		.line_length_pclk = 0x17C4, /* 6084 */
		.frame_length_lines = 0x0D4A, /* 3402 */
		.vt_pixel_clk = 310500000,
		.op_pixel_clk = 264000000,
		.binning_factor = 1,
	},
	/* 30 fps preview */
	{
		.x_output = 0x07D8, /* 2008 */
		.y_output = 0x05E4, /* 1508 */
		.line_length_pclk = 0x17C4, /* 6084 */
		.frame_length_lines = 0x0689, /* 1700 */
		.vt_pixel_clk = 310500000,
		.op_pixel_clk = 264000000,
		.binning_factor = 1,
	},
	/* 60 fps video */
	{
		.x_output = 0x07E0, /* 2016 */
		.y_output = 0x021C, /* 540 */
		.line_length_pclk = 0x118C, /* 4492 */
		.frame_length_lines = 0x0480, /* 1152 */
		.vt_pixel_clk = 310500000,
		.op_pixel_clk = 264000000,
		.binning_factor = 1,
	},
	/* 90 fps video */
	{
		.x_output = 0x07E0, /* 2016 */
		.y_output = 0x021C, /* 540 */
		.line_length_pclk = 0x118C, /* 4492 */
		.frame_length_lines = 0x0300, /* 768 */
		.vt_pixel_clk = 310500000,
		.op_pixel_clk = 264000000,
		.binning_factor = 1,
	},
	/* 120 fps video */
	{
		.x_output = 0x07E0, /* 2016 */
		.y_output = 0x021C, /* 540 */
		.line_length_pclk = 0x118C, /* 4492 */
		.frame_length_lines = 0x0240, /* 576 */
		.vt_pixel_clk = 310500000,
		.op_pixel_clk = 264000000,
		.binning_factor = 1,
	},
};

static struct msm_camera_csid_vc_cfg s5k3l1yx_cid_cfg[] = {
	{0, CSI_RAW10, CSI_DECODE_10BIT},
	{1, CSI_EMBED_DATA, CSI_DECODE_8BIT},
};

static struct msm_camera_csi2_params s5k3l1yx_csi_params = {
	.csid_params = {
		.lane_assign = 0xe4,
		.lane_cnt = 4,
		.lut_params = {
			.num_cid = ARRAY_SIZE(s5k3l1yx_cid_cfg),
			.vc_cfg = s5k3l1yx_cid_cfg,
		},
	},
	.csiphy_params = {
		.lane_cnt = 4,
		.settle_cnt = 0x1B,
	},
};

static struct msm_camera_csi2_params *s5k3l1yx_csi_params_array[] = {
	&s5k3l1yx_csi_params,
	&s5k3l1yx_csi_params,
	&s5k3l1yx_csi_params,
	&s5k3l1yx_csi_params,
	&s5k3l1yx_csi_params,
};

static struct msm_sensor_output_reg_addr_t s5k3l1yx_reg_addr = {
	.x_output = 0x34C,
	.y_output = 0x34E,
	.line_length_pclk = 0x342,
	.frame_length_lines = 0x340,
};

static struct msm_sensor_id_info_t s5k3l1yx_id_info = {
	.sensor_id_reg_addr = 0x0,
	.sensor_id = 0x3121,
};

static struct msm_sensor_exp_gain_info_t s5k3l1yx_exp_gain_info = {
	.coarse_int_time_addr = 0x202,
	.global_gain_addr = 0x204,
	.vert_offset = 8,
};

typedef struct {
	uint32_t fl_lines;
	uint32_t line;
	uint16_t gain;
	uint16_t digital_gain;
} s5k3l1yx_exp_gain_t;

s5k3l1yx_exp_gain_t s5k3l1yx_exp_gain[MSM_SENSOR_INVALID_RES] ={
	{ 0x0D4A, 0x0040, 0x0020, 0x0100},
	{ 0x0689, 0x0040, 0x0020, 0x0100},
	{ 0x0480, 0x0250, 0x0020, 0x0100},
	{ 0x0300, 0x0250, 0x0020, 0x0100},
	{ 0x0240, 0x0250, 0x0020, 0x0100},
};

static int s5k3l1yx_sensor_config(struct msm_sensor_ctrl_t *s_ctrl, void __user *argp)
{
	struct sensor_cfg_data cdata;
	long   rc = 0;
	if (copy_from_user(&cdata,
		(void *)argp,
		sizeof(struct sensor_cfg_data)))
		return -EFAULT;
	mutex_lock(&s5k3l1yx_mut);
	CDBG("s5k3l1yx_sensor_config: cfgtype = %d\n",
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
				mutex_unlock(&s5k3l1yx_mut);
				return -EFAULT;
			}
			kfree(data);
			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data))){
				mutex_unlock(&s5k3l1yx_mut);
				return -EFAULT;
			}
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
				mutex_unlock(&s5k3l1yx_mut);
				return -EFAULT;
			}
			rc = msm_camera_i2c_write_seq(s_ctrl->sensor_i2c_client, cdata.cfg.i2c_info.addr, data, cdata.cfg.i2c_info.length);
			kfree(data);
			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data))){
				mutex_unlock(&s5k3l1yx_mut);
				return -EFAULT;
			}
		}
			break;
	case CFG_SH_SET_EXP_GAIN:
		{
			uint32_t fl_lines;
			uint8_t offset;
			long fps = 0;
			uint16_t ll_pclk;
			uint16_t cur_fl_lines;
			uint32_t delay = 0;

			fl_lines = s_ctrl->curr_frame_length_lines;
			fl_lines = (fl_lines * s_ctrl->fps_divider) / Q10;
			offset = s_ctrl->sensor_exp_gain_info->vert_offset;
			if (cdata.cfg.exp_gain.line > (fl_lines - offset))
				fl_lines = cdata.cfg.exp_gain.line + offset;

			if( s5k3l1yx_curr_mode == 0 ) {
				rc = msm_camera_i2c_read(
						s_ctrl->sensor_i2c_client,
						s_ctrl->sensor_output_reg_addr->line_length_pclk, &ll_pclk,
						MSM_CAMERA_I2C_WORD_DATA);
				rc = msm_camera_i2c_read(
						s_ctrl->sensor_i2c_client,
						s_ctrl->sensor_output_reg_addr->frame_length_lines, &cur_fl_lines,
						MSM_CAMERA_I2C_WORD_DATA);
				CDBG("%s ll_pclk = %d, frame fl_lines = %d\n", __func__, ll_pclk, cur_fl_lines);
				if((ll_pclk != 0) && (cur_fl_lines != 0) && (s_ctrl->curr_res != MSM_SENSOR_INVALID_RES)){
					fps = s_ctrl->msm_sensor_reg->
						output_settings[s_ctrl->curr_res].vt_pixel_clk /
						cur_fl_lines / ll_pclk;

					if(fps != 0)
						delay = 1000 / fps;
				}
				CDBG("%s fps = %ld, delay = %d\n", __func__, fps, delay);
			}

			s_ctrl->func_tbl->sensor_group_hold_on(s_ctrl);
			msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
				s_ctrl->sensor_output_reg_addr->frame_length_lines, fl_lines,
				MSM_CAMERA_I2C_WORD_DATA);
			msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
				s_ctrl->sensor_exp_gain_info->coarse_int_time_addr, cdata.cfg.exp_gain.line,
				MSM_CAMERA_I2C_WORD_DATA);
			msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
				s_ctrl->sensor_exp_gain_info->global_gain_addr, cdata.cfg.exp_gain.gain,
				MSM_CAMERA_I2C_WORD_DATA);
			msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
				0x020E, cdata.cfg.exp_gain.digital_gain,
				MSM_CAMERA_I2C_WORD_DATA);
			msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
				0x0210, cdata.cfg.exp_gain.digital_gain,
				MSM_CAMERA_I2C_WORD_DATA);
			msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
				0x0212, cdata.cfg.exp_gain.digital_gain,
				MSM_CAMERA_I2C_WORD_DATA);
			msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
				0x0214, cdata.cfg.exp_gain.digital_gain,
				MSM_CAMERA_I2C_WORD_DATA);
			s_ctrl->func_tbl->sensor_group_hold_off(s_ctrl);
			
			if(s_ctrl->curr_res != MSM_SENSOR_INVALID_RES){
				s5k3l1yx_exp_gain[s_ctrl->curr_res].fl_lines = fl_lines;
				s5k3l1yx_exp_gain[s_ctrl->curr_res].line = cdata.cfg.exp_gain.line;
				s5k3l1yx_exp_gain[s_ctrl->curr_res].gain = cdata.cfg.exp_gain.gain;
				s5k3l1yx_exp_gain[s_ctrl->curr_res].digital_gain = cdata.cfg.exp_gain.digital_gain;
			}

			CDBG("%s fl_lines=%d line=%d gain=%d digital_gain=%d", __func__, fl_lines, cdata.cfg.exp_gain.line, cdata.cfg.exp_gain.gain, cdata.cfg.exp_gain.digital_gain);
			if( s5k3l1yx_curr_mode == 0 ) {
				msleep(delay);
			}
		}
		break;
	default:
		mutex_unlock(&s5k3l1yx_mut);
		return msm_sensor_config(s_ctrl, argp);
		break;
	}
	
	mutex_unlock(&s5k3l1yx_mut);
	return rc;
}

int shcamled_pmic_set_torch_led_1_current(unsigned mA);

int s5k3l1yx_pmic_set_flash_led_current(enum pmic8058_leds id, unsigned mA)
{
	int ret;

	ret = shcamled_pmic_set_torch_led_1_current(mA);
	if (ret)
		CDBG("%s: Set Failed ret=%d mA=0x%x\n", __FUNCTION__, ret, mA);
	else
		CDBG("%s: Set Success ret=%d mA=0x%x\n", __FUNCTION__, ret, mA);

	return ret;
}

static const struct i2c_device_id s5k3l1yx_i2c_id[] = {
	{SENSOR_NAME, (kernel_ulong_t)&s5k3l1yx_s_ctrl},
	{ }
};

static struct i2c_driver s5k3l1yx_i2c_driver = {
	.id_table = s5k3l1yx_i2c_id,
	.probe  = msm_sensor_i2c_probe,
	.driver = {
		.name = SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client s5k3l1yx_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static int __init msm_sensor_init_module(void)
{
	return i2c_add_driver(&s5k3l1yx_i2c_driver);
}

static struct v4l2_subdev_core_ops s5k3l1yx_subdev_core_ops = {
	.ioctl = msm_sensor_subdev_ioctl,
	.s_power = msm_sensor_power,
};

static struct v4l2_subdev_video_ops s5k3l1yx_subdev_video_ops = {
	.enum_mbus_fmt = msm_sensor_v4l2_enum_fmt,
};

static struct v4l2_subdev_ops s5k3l1yx_subdev_ops = {
	.core = &s5k3l1yx_subdev_core_ops,
	.video  = &s5k3l1yx_subdev_video_ops,
};

EXPORT_SYMBOL(s5k3l1yx_pmic_set_flash_led_current);

static struct gpio bh6455gul_actuator_gpio[] = {
	{3, GPIOF_DIR_OUT, "VCM_PWD"},
};

static struct msm_gpio_set_tbl bh6455gul_actuator_gpio_set_tbl[] = {
	{3, GPIOF_OUT_INIT_LOW, 1000},
	{3, GPIOF_OUT_INIT_HIGH, 4000},
};

static struct gpio bh6455gul_actuator_common_cam_gpio[] = {
};

static struct msm_camera_gpio_conf bh6455gul_actuator_gpio_conf = {
	.cam_gpiomux_conf_tbl = NULL,
	.cam_gpiomux_conf_tbl_size = 0,
	.cam_gpio_common_tbl = bh6455gul_actuator_common_cam_gpio,
	.cam_gpio_common_tbl_size = ARRAY_SIZE(bh6455gul_actuator_common_cam_gpio),
	.cam_gpio_req_tbl = bh6455gul_actuator_gpio,
	.cam_gpio_req_tbl_size = ARRAY_SIZE(bh6455gul_actuator_gpio),
	.cam_gpio_set_tbl = bh6455gul_actuator_gpio_set_tbl,
	.cam_gpio_set_tbl_size = ARRAY_SIZE(bh6455gul_actuator_gpio_set_tbl),
};

static struct msm_camera_sensor_platform_info board_info_bh6455gul_act = {
	.gpio_conf = &bh6455gul_actuator_gpio_conf,
};

struct msm_camera_sensor_info bh6455gul_act_data = {
	.sensor_platform_info = &board_info_bh6455gul_act,
};

static struct msm_cam_clk_info cam_clk_info[] = {
	{"cam_clk", S5K3L1YX_SENSOR_MCLK_6P75HZ},
};

/* SHLOCAL_CAMERA_IMAGE_QUALITY-> */
/* QUALCOMM NADESHIKO MERGE -> */
int32_t s5k3l1yx_get_runtime_info(struct msm_sensor_ctrl_t *s_ctrl,
	struct runtime_info *data) {
	int32_t rc = 0;
	uint16_t info = 0, reg_data = 0;
	uint16_t *ptr = (uint16_t *)data->data_ptr;

	rc = msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x3258, 0x05,
		MSM_CAMERA_I2C_BYTE_DATA);
	if (rc != 0) {
		CDBG("%s: Ctl write error\n", __func__);
		return 0;
	}

	rc = msm_camera_i2c_read(s_ctrl->sensor_i2c_client, 0x3259, &info,
		MSM_CAMERA_I2C_BYTE_DATA);
	if (rc < 0) {
		CDBG("%s read fail\n", __func__);
		return rc;
	}
	info <<= 8;
	rc = msm_camera_i2c_read(s_ctrl->sensor_i2c_client, 0x325A, &reg_data,
		MSM_CAMERA_I2C_BYTE_DATA);
	if (rc < 0) {
		CDBG("%s read fail\n", __func__);
		return rc;
	}
	info |= reg_data;
	*ptr = info;
	return rc;
}
/* QUALCOMM NADESHIKO MERGE <- */
/* SHLOCAL_CAMERA_IMAGE_QUALITY<- */

int32_t s5k3l1yx_power_up(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	struct msm_camera_sensor_info *data = s_ctrl->sensordata;
	struct msm_camera_sensor_info *act_data = &bh6455gul_act_data;

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

	rc = msm_camera_request_gpio_table(act_data, 1);
	if (rc < 0) {
		pr_err("%s: request actuator gpio failed\n", __func__);
		goto request_act_gpio_failed;
	}

	rc = msm_camera_config_gpio_table(data, 0);
	if (rc < 0) {
		pr_err("%s: config gpio dis failed\n", __func__);
		goto config_gpio_dis_failed;
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

	if (s_ctrl->clk_rate != 0)
		cam_clk_info->clk_rate = s_ctrl->clk_rate;

	rc = msm_cam_clk_enable(&s_ctrl->sensor_i2c_client->client->dev,
		cam_clk_info, &s_ctrl->cam_clk, ARRAY_SIZE(cam_clk_info), 1);
	if (rc < 0) {
		pr_err("%s: clk enable failed\n", __func__);
		goto enable_clk_failed;
	}

	rc = msm_camera_config_gpio_table(data, 1);
	if (rc < 0) {
		pr_err("%s: config gpio failed\n", __func__);
		goto config_gpio_failed;
	}
	msleep(3);

	rc = msm_camera_config_gpio_table(act_data, 1);
	if (rc < 0) {
		pr_err("%s: config actuator gpio failed\n", __func__);
		goto config_act_gpio_failed;
	}
	msleep(5);

	s5k3l1yx_curr_mode = 0;

	return rc;

config_act_gpio_failed:
		msm_camera_config_gpio_table(data, 0);
config_gpio_failed:
		msm_cam_clk_enable(&s_ctrl->sensor_i2c_client->client->dev,
			cam_clk_info, &s_ctrl->cam_clk, ARRAY_SIZE(cam_clk_info), 0);
enable_clk_failed:
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
config_gpio_dis_failed:
	msm_camera_request_gpio_table(act_data, 0);
request_act_gpio_failed:
	msm_camera_request_gpio_table(data, 0);
request_gpio_failed:
	kfree(s_ctrl->reg_ptr);
	return rc;
}

int32_t s5k3l1yx_power_down(struct msm_sensor_ctrl_t *s_ctrl)
{
	struct msm_camera_sensor_info *data = s_ctrl->sensordata;
	struct msm_camera_sensor_info *act_data = &bh6455gul_act_data;
	CDBG("%s\n", __func__);

	s_ctrl->func_tbl->sensor_stop_stream(s_ctrl);

	msm_camera_config_gpio_table(act_data, 0);
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
	msm_camera_request_gpio_table(act_data, 0);
	msm_camera_request_gpio_table(data, 0);
	kfree(s_ctrl->reg_ptr);
	return 0;
}


int32_t s5k3l1yx_sensor_write_init_settings(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc;
	int32_t i;
	uint16_t size;
	struct msm_camera_i2c_reg_conf *reg_conf_tbl;
	uint16_t addr,addr_pos,burst_num;
	unsigned char buf[256];

	reg_conf_tbl = s_ctrl->msm_sensor_reg->init_settings[0].conf;
    size = s_ctrl->msm_sensor_reg->init_settings[0].size;
    addr_pos = reg_conf_tbl->reg_addr;
	addr = reg_conf_tbl->reg_addr;
	buf[0] = reg_conf_tbl->reg_data;
	reg_conf_tbl++;
	burst_num = 1;
	for (i = 1; i < size; i++) {
		if(reg_conf_tbl->reg_addr == (addr + 1)){
			buf[burst_num] = reg_conf_tbl->reg_data;
			burst_num++;
		} else {
			rc = msm_camera_i2c_write_seq(s_ctrl->sensor_i2c_client,
																addr_pos,
																buf,
																burst_num);
			addr_pos = reg_conf_tbl->reg_addr;
			buf[0] = reg_conf_tbl->reg_data;
			burst_num = 1;
		}
		addr = reg_conf_tbl->reg_addr;
		reg_conf_tbl++;
	}
	rc = msm_camera_i2c_write_seq(s_ctrl->sensor_i2c_client,
																addr_pos,
																buf,
																burst_num);
	usleep_range(0,0);
	return rc;
}

int32_t s5k3l1yx_sensor_write_res_settings(struct msm_sensor_ctrl_t *s_ctrl,
	uint16_t res)
{
	int32_t rc;
	int32_t i;
	uint16_t size;
	struct msm_camera_i2c_reg_conf *reg_conf_tbl;
	uint16_t addr,addr_pos,burst_num;
	unsigned char buf[256];

	reg_conf_tbl = s_ctrl->msm_sensor_reg->mode_settings[res].conf;
    size = s_ctrl->msm_sensor_reg->mode_settings[res].size;
    addr_pos = reg_conf_tbl->reg_addr;
	addr = reg_conf_tbl->reg_addr;
	buf[0] = reg_conf_tbl->reg_data;
	reg_conf_tbl++;
	burst_num = 1;
	for (i = 1; i < size; i++) {
		if(reg_conf_tbl->reg_addr == (addr + 1)){
			buf[burst_num] = reg_conf_tbl->reg_data;
			burst_num++;
		} else {
			rc = msm_camera_i2c_write_seq(s_ctrl->sensor_i2c_client,
																addr_pos,
																buf,
																burst_num);
			addr_pos = reg_conf_tbl->reg_addr;
			buf[0] = reg_conf_tbl->reg_data;
			burst_num = 1;
		}
		addr = reg_conf_tbl->reg_addr;
		reg_conf_tbl++;
	}
	rc = msm_camera_i2c_write_seq(s_ctrl->sensor_i2c_client,
																addr_pos,
																buf,
																burst_num);
	usleep_range(0,0);

	if (rc < 0)
		return rc;

	rc = msm_sensor_write_output_settings(s_ctrl, res);
	return rc;
}

int32_t s5k3l1yx_sensor_setting(struct msm_sensor_ctrl_t *s_ctrl,
			int update_type, int res)
{
	int32_t rc = 0;
	long fps = 0;
	uint16_t ll_pclk;
	uint16_t fl_lines;
	uint32_t delay = 0;

	if (update_type == MSM_SENSOR_REG_INIT) {
		s_ctrl->curr_csi_params = NULL;
		msm_sensor_enable_debugfs(s_ctrl);
		s5k3l1yx_sensor_write_init_settings(s_ctrl);
		
	} else if (update_type == MSM_SENSOR_UPDATE_PERIODIC) {
		if(s_ctrl->curr_res != MSM_SENSOR_INVALID_RES){
			s_ctrl->func_tbl->sensor_stop_stream(s_ctrl);
			s_ctrl->func_tbl->sensor_stop_stream(s_ctrl);
			rc = msm_camera_i2c_read(
					s_ctrl->sensor_i2c_client,
					s_ctrl->sensor_output_reg_addr->line_length_pclk, &ll_pclk,
					MSM_CAMERA_I2C_WORD_DATA);
			rc = msm_camera_i2c_read(
					s_ctrl->sensor_i2c_client,
					s_ctrl->sensor_output_reg_addr->frame_length_lines, &fl_lines,
					MSM_CAMERA_I2C_WORD_DATA);
			if((ll_pclk != 0) && (fl_lines != 0)){
				fps = s_ctrl->msm_sensor_reg->
					output_settings[s_ctrl->curr_res].vt_pixel_clk /
					fl_lines / ll_pclk;
				if(fps != 0)
					delay = 1000 / fps;
			}
			CDBG("%s fps = %ld, frame time = %d\n", __func__, fps, delay);
			delay += 10;
			CDBG("%s delay = %d\n", __func__, delay);
			msleep(delay);
		}

		s5k3l1yx_sensor_write_res_settings(s_ctrl, res);

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
		if(s_ctrl->curr_res != MSM_SENSOR_INVALID_RES){
			msleep(30);
		}
	}
	return rc;
}

int32_t s5k3l1yx_sensor_adjust_frame_lines(struct msm_sensor_ctrl_t *s_ctrl,
	uint16_t res)
{
//	if((res == MSM_SENSOR_RES_QTR) || (res == MSM_SENSOR_RES_5)){
		s_ctrl->func_tbl->sensor_group_hold_on(s_ctrl);
		msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
			s_ctrl->sensor_output_reg_addr->frame_length_lines, s5k3l1yx_exp_gain[res].fl_lines,
			MSM_CAMERA_I2C_WORD_DATA);
		msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
			s_ctrl->sensor_exp_gain_info->coarse_int_time_addr, s5k3l1yx_exp_gain[res].line,
			MSM_CAMERA_I2C_WORD_DATA);
		msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
			s_ctrl->sensor_exp_gain_info->global_gain_addr, s5k3l1yx_exp_gain[res].gain,
			MSM_CAMERA_I2C_WORD_DATA);
		msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
			0x020E, s5k3l1yx_exp_gain[res].digital_gain,
			MSM_CAMERA_I2C_WORD_DATA);
		msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
			0x0210, s5k3l1yx_exp_gain[res].digital_gain,
			MSM_CAMERA_I2C_WORD_DATA);
		msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
			0x0212, s5k3l1yx_exp_gain[res].digital_gain,
			MSM_CAMERA_I2C_WORD_DATA);
		msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
			0x0214, s5k3l1yx_exp_gain[res].digital_gain,
			MSM_CAMERA_I2C_WORD_DATA);
		s_ctrl->func_tbl->sensor_group_hold_off(s_ctrl);
		
		CDBG("%s fl_lines=%d line=%d gain=%d digital_gain=%d", __func__, s5k3l1yx_exp_gain[res].fl_lines, s5k3l1yx_exp_gain[res].line, s5k3l1yx_exp_gain[res].gain, s5k3l1yx_exp_gain[res].digital_gain);
//	}
	return 0;
}

static struct msm_sensor_fn_t s5k3l1yx_func_tbl = {
	.sensor_start_stream = msm_sensor_start_stream,
	.sensor_stop_stream = msm_sensor_stop_stream,
	.sensor_group_hold_on = msm_sensor_group_hold_on,
	.sensor_group_hold_off = msm_sensor_group_hold_off,
	.sensor_set_fps = msm_sensor_set_fps,
	.sensor_write_exp_gain = msm_sensor_write_exp_gain1,
	.sensor_write_snapshot_exp_gain = msm_sensor_write_exp_gain1,
	.sensor_setting = s5k3l1yx_sensor_setting,
	.sensor_set_sensor_mode = msm_sensor_set_sensor_mode,
	.sensor_mode_init = msm_sensor_mode_init,
	.sensor_get_output_info = msm_sensor_get_output_info,
	.sensor_config = s5k3l1yx_sensor_config,
	.sensor_power_up = s5k3l1yx_power_up,
	.sensor_power_down =  s5k3l1yx_power_down,
	.sensor_adjust_frame_lines = s5k3l1yx_sensor_adjust_frame_lines,
	.sensor_get_csi_params = msm_sensor_get_csi_params,
/* SHLOCAL_CAMERA_IMAGE_QUALITY-> */
/* QUALCOMM NADESHIKO MERGE -> */
	.sensor_get_runtime_info = s5k3l1yx_get_runtime_info,
/* QUALCOMM NADESHIKO MERGE <- */
/* SHLOCAL_CAMERA_IMAGE_QUALITY<- */
};

static struct msm_sensor_reg_t s5k3l1yx_regs = {
	.default_data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.start_stream_conf = s5k3l1yx_start_settings,
	.start_stream_conf_size = ARRAY_SIZE(s5k3l1yx_start_settings),
	.stop_stream_conf = s5k3l1yx_stop_settings,
	.stop_stream_conf_size = ARRAY_SIZE(s5k3l1yx_stop_settings),
	.group_hold_on_conf = s5k3l1yx_groupon_settings,
	.group_hold_on_conf_size = ARRAY_SIZE(s5k3l1yx_groupon_settings),
	.group_hold_off_conf = s5k3l1yx_groupoff_settings,
	.group_hold_off_conf_size =
		ARRAY_SIZE(s5k3l1yx_groupoff_settings),
	.init_settings = &s5k3l1yx_init_conf[0],
	.init_size = ARRAY_SIZE(s5k3l1yx_init_conf),
	.mode_settings = &s5k3l1yx_confs[0],
	.output_settings = &s5k3l1yx_dimensions[0],
	.num_conf = ARRAY_SIZE(s5k3l1yx_confs),
};

static struct msm_sensor_ctrl_t s5k3l1yx_s_ctrl = {
	.msm_sensor_reg = &s5k3l1yx_regs,
	.sensor_i2c_client = &s5k3l1yx_sensor_i2c_client,
	.sensor_i2c_addr = 0x20,
	.sensor_output_reg_addr = &s5k3l1yx_reg_addr,
	.sensor_id_info = &s5k3l1yx_id_info,
	.sensor_exp_gain_info = &s5k3l1yx_exp_gain_info,
	.cam_mode = MSM_SENSOR_MODE_INVALID,
	.csi_params = &s5k3l1yx_csi_params_array[0],
	.msm_sensor_mutex = &s5k3l1yx_mut,
	.sensor_i2c_driver = &s5k3l1yx_i2c_driver,
	.sensor_v4l2_subdev_info = s5k3l1yx_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(s5k3l1yx_subdev_info),
	.sensor_v4l2_subdev_ops = &s5k3l1yx_subdev_ops,
	.func_tbl = &s5k3l1yx_func_tbl,
	.clk_rate = S5K3L1YX_SENSOR_MCLK_6P75HZ,
};

module_init(msm_sensor_init_module);
MODULE_DESCRIPTION("Samsung 12MP Bayer sensor driver");
MODULE_LICENSE("GPL v2");
