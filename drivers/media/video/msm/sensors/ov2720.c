/* Copyright (c) 2011-2012, The Linux Foundation. All rights reserved.
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

#include "msm_sensor.h"
#include "ov2720.h"
#define SENSOR_NAME "ov2720"
#define PLATFORM_DRIVER_NAME "msm_camera_ov2720"
#define ov2720_obj ov2720_##obj

DEFINE_MUTEX(ov2720_mut);
static struct msm_sensor_ctrl_t ov2720_s_ctrl;

static struct msm_camera_i2c_reg_conf ov2720_start_settings[] = {
	{0x0100, 0x01},
};

static struct msm_camera_i2c_reg_conf ov2720_stop_settings[] = {
	{0x0100, 0x00},
};

static struct msm_camera_i2c_reg_conf ov2720_groupon_settings[] = {
	{0x3208, 0x00},
};

static struct msm_camera_i2c_reg_conf ov2720_groupoff_settings[] = {
	{0x3208, 0x10},
	{0x3208, 0xA0},
};

static struct msm_camera_i2c_reg_conf ov2720_prev_settings[] = {
	{0x3800, 0x00},
	{0x3801, 0x02},
	{0x3802, 0x00},
	{0x3803, 0x00},
	{0x3804, 0x07},
	{0x3805, 0xA1},
	{0x3806, 0x04},
	{0x3807, 0x47},
	{0x3810, 0x00},
	{0x3811, 0x09},
	{0x3812, 0x00},
	{0x3813, 0x02},
	{0x3820, 0x80},
	{0x3821, 0x06},
	{0x3814, 0x11},
	{0x3815, 0x11},
	{0x3612, 0x0b},
	{0x3618, 0x04},
	{0x3a08, 0x01},
	{0x3a09, 0x50},
	{0x3a0a, 0x01},
	{0x3a0b, 0x18},
	{0x3a0d, 0x03},
	{0x3a0e, 0x03},
	{0x4520, 0x00},
	{0x4837, 0x1b},
	{0x3000, 0xff},
	{0x3001, 0xff},
	{0x3002, 0xf0},
	{0x3600, 0x08},
	{0x3621, 0xc0},
	{0x3632, 0xd2},
	{0x3633, 0x23},
	{0x3634, 0x54},
	{0x3f01, 0x0c},
	{0x5001, 0xc1},
	{0x3614, 0xf0},
	{0x3630, 0x2d},
	{0x370b, 0x62},
	{0x3706, 0x61},
	{0x4000, 0x02},
	{0x4002, 0xc5},
	{0x4005, 0x08},
	{0x404f, 0x84},
	{0x4051, 0x00},
	{0x5000, 0xcf},
	{0x3a18, 0x00},
	{0x3a19, 0x80},
	{0x3503, 0x03},
	{0x4521, 0x00},
	{0x5183, 0xb0},
	{0x5184, 0xb0},
	{0x5185, 0xb0},
	{0x370c, 0x0c},
	{0x3035, 0x10},
	{0x3036, 0x1e},
	{0x3037, 0x21},
	{0x303e, 0x19},
	{0x3038, 0x06},
	{0x3018, 0x04},
	{0x3000, 0x00},
	{0x3001, 0x00},
	{0x3002, 0x00},
	{0x3a0f, 0x40},
	{0x3a10, 0x38},
	{0x3a1b, 0x48},
	{0x3a1e, 0x30},
	{0x3a11, 0x90},
	{0x3a1f, 0x10},
	{0x4800, 0x24},
};

static struct msm_camera_i2c_reg_conf ov2720_720_settings[] = {
	{0x3800, 0x01},
	{0x3801, 0x4a},
	{0x3802, 0x00},
	{0x3803, 0xba},
	{0x3804, 0x06},
	{0x3805, 0x51+32},
	{0x3806, 0x03},
	{0x3807, 0x8d+24},
	{0x3810, 0x00},
	{0x3811, 0x05},
	{0x3812, 0x00},
	{0x3813, 0x02},
	{0x3820, 0x80},
	{0x3821, 0x06},
	{0x3814, 0x11},
	{0x3815, 0x11},
	{0x3612, 0x0b},
	{0x3618, 0x04},
	{0x3a08, 0x01},
	{0x3a09, 0x50},
	{0x3a0a, 0x01},
	{0x3a0b, 0x18},
	{0x3a0d, 0x03},
	{0x3a0e, 0x03},
	{0x4520, 0x00},
	{0x4837, 0x1b},
	{0x3000, 0xff},
	{0x3001, 0xff},
	{0x3002, 0xf0},
	{0x3600, 0x08},
	{0x3621, 0xc0},
	{0x3632, 0xd2},
	{0x3633, 0x23},
	{0x3634, 0x54},
	{0x3f01, 0x0c},
	{0x5001, 0xc1},
	{0x3614, 0xf0},
	{0x3630, 0x2d},
	{0x370b, 0x62},
	{0x3706, 0x61},
	{0x4000, 0x02},
	{0x4002, 0xc5},
	{0x4005, 0x08},
	{0x404f, 0x84},
	{0x4051, 0x00},
	{0x5000, 0xff},
	{0x3a18, 0x00},
	{0x3a19, 0x80},
	{0x3503, 0x13},
	{0x4521, 0x00},
	{0x5183, 0xb0},
	{0x5184, 0xb0},
	{0x5185, 0xb0},
	{0x370c, 0x0c},
	{0x3035, 0x10},
	{0x3036, 0x04},
	{0x3037, 0x61},
	{0x303e, 0x19},
	{0x3038, 0x06},
	{0x3018, 0x04},
	{0x3000, 0x00},
	{0x3001, 0x00},
	{0x3002, 0x00},
	{0x3a0f, 0x40},
	{0x3a10, 0x38},
	{0x3a1b, 0x48},
	{0x3a1e, 0x30},
	{0x3a11, 0x90},
	{0x3a1f, 0x10},
	{0x4800, 0x24},
};

static struct msm_camera_i2c_reg_conf ov2720_vga_settings[] = {
	{0x3800, 0x00},
	{0x3801, 0x0c},
	{0x3802, 0x00},
	{0x3803, 0x02},
	{0x3804, 0x07},
	{0x3805, 0x97+32},
	{0x3806, 0x04},
	{0x3807, 0x45+24},
	{0x3810, 0x00},
	{0x3811, 0x03},
	{0x3812, 0x00},
	{0x3813, 0x03},
	{0x3820, 0x80},
	{0x3821, 0x06},
	{0x3814, 0x11},
	{0x3815, 0x11},
	{0x3612, 0x0b},
	{0x3618, 0x04},
	{0x3a08, 0x01},
	{0x3a09, 0x50},
	{0x3a0a, 0x01},
	{0x3a0b, 0x18},
	{0x3a0d, 0x03},
	{0x3a0e, 0x03},
	{0x4520, 0x00},
	{0x4837, 0x1b},
	{0x3000, 0xff},
	{0x3001, 0xff},
	{0x3002, 0xf0},
	{0x3600, 0x08},
	{0x3621, 0xc0},
	{0x3632, 0xd2},
	{0x3633, 0x23},
	{0x3634, 0x54},
	{0x3f01, 0x0c},
	{0x5001, 0xc1},
	{0x3614, 0xf0},
	{0x3630, 0x2d},
	{0x370b, 0x62},
	{0x3706, 0x61},
	{0x4000, 0x02},
	{0x4002, 0xc5},
	{0x4005, 0x08},
	{0x404f, 0x84},
	{0x4051, 0x00},
	{0x5000, 0xff},
	{0x3a18, 0x00},
	{0x3a19, 0x80},
	{0x3503, 0x13},
	{0x4521, 0x00},
	{0x5183, 0xb0},
	{0x5184, 0xb0},
	{0x5185, 0xb0},
	{0x370c, 0x0c},
	{0x3035, 0x10},
	{0x3036, 0x04},
	{0x3037, 0x61},
	{0x303e, 0x19},
	{0x3038, 0x06},
	{0x3018, 0x04},
	{0x3000, 0x00},
	{0x3001, 0x00},
	{0x3002, 0x00},
	{0x3a0f, 0x40},
	{0x3a10, 0x38},
	{0x3a1b, 0x48},
	{0x3a1e, 0x30},
	{0x3a11, 0x90},
	{0x3a1f, 0x10},
	{0x4800, 0x24},
	{0x3500, 0x00},
	{0x3501, 0x17},
	{0x3502, 0xf0},
	{0x3508, 0x00},
	{0x3509, 0x20},
};

static struct msm_camera_i2c_reg_conf ov2720_60fps_settings[] = {
	{0x3718, 0x10},
	{0x3702, 0x18},
	{0x373a, 0x3c},
	{0x3715, 0x01},
	{0x3703, 0x1d},
	{0x3705, 0x0b},
	{0x3730, 0x1f},
	{0x3704, 0x3f},
	{0x3f06, 0x1d},
	{0x371c, 0x00},
	{0x371d, 0x83},
	{0x371e, 0x00},
	{0x371f, 0xb6},
	{0x3708, 0x63},
	{0x3709, 0x52},
	{0x3800, 0x01},
	{0x3801, 0x42},
	{0x3802, 0x00},
	{0x3803, 0x40},
	{0x3804, 0x06},
	{0x3805, 0x61},
	{0x3806, 0x04},
	{0x3807, 0x08},
	{0x3808, 0x02},
	{0x3809, 0x80},
	{0x380a, 0x01},
	{0x380b, 0xe0},
	{0x380c, 0x03},
	{0x380d, 0x0c},
	{0x380e, 0x02},
	{0x380f, 0x00},
	{0x3810, 0x00},
	{0x3811, 0x0f},
	{0x3812, 0x00},
	{0x3813, 0x02},
	{0x3820, 0x80},
	{0x3821, 0x06},
	{0x3814, 0x31},
	{0x3815, 0x31},
	{0x3612, 0x0b},
	{0x3618, 0x04},
	{0x3a08, 0x02},
	{0x3a09, 0x67},
	{0x3a0a, 0x02},
	{0x3a0b, 0x00},
	{0x3a0d, 0x00},
	{0x3a0e, 0x00},
	{0x4520, 0x0a},
	{0x4837, 0x29},
	{0x3000, 0xff},
	{0x3001, 0xff},
	{0x3002, 0xf0},
	{0x3600, 0x08},
	{0x3621, 0xc0},
	{0x3632, 0xd2},
	{0x3633, 0x23},
	{0x3634, 0x54},
	{0x3f01, 0x0c},
	{0x5001, 0xc1},
	{0x3614, 0xf0},
	{0x3630, 0x2d},
	{0x370b, 0x62},
	{0x3706, 0x61},
	{0x4000, 0x02},
	{0x4002, 0xc5},
	{0x4005, 0x08},
	{0x404f, 0x84},
	{0x4051, 0x00},
	{0x5000, 0xcf},
	{0x3a18, 0x00},
	{0x3a19, 0x80},
	{0x3503, 0x07},
	{0x4521, 0x00},
	{0x5183, 0xb0},
	{0x5184, 0xb0},
	{0x5185, 0xb0},
	{0x370c, 0x0c},
	{0x3035, 0x30},
	{0x3036, 0x14},
	{0x3037, 0x21},
	{0x303e, 0x19},
	{0x3038, 0x06},
	{0x3018, 0x04},
	{0x3000, 0x00},
	{0x3001, 0x00},
	{0x3002, 0x00},
	{0x3a0f, 0x40},
	{0x3a10, 0x38},
	{0x3a1b, 0x48},
	{0x3a1e, 0x30},
	{0x3a11, 0x90},
	{0x3a1f, 0x10},
	{0x3011, 0x22},
	{0x3a00, 0x58},
};

static struct msm_camera_i2c_reg_conf ov2720_90fps_settings[] = {
	{0x3718, 0x10},
	{0x3702, 0x18},
	{0x373a, 0x3c},
	{0x3715, 0x01},
	{0x3703, 0x1d},
	{0x3705, 0x0b},
	{0x3730, 0x1f},
	{0x3704, 0x3f},
	{0x3f06, 0x1d},
	{0x371c, 0x00},
	{0x371d, 0x83},
	{0x371e, 0x00},
	{0x371f, 0xb6},
	{0x3708, 0x63},
	{0x3709, 0x52},
	{0x3800, 0x01},
	{0x3801, 0x42},
	{0x3802, 0x00},
	{0x3803, 0x40},
	{0x3804, 0x06},
	{0x3805, 0x61},
	{0x3806, 0x04},
	{0x3807, 0x08},
	{0x3808, 0x02},
	{0x3809, 0x80},
	{0x380a, 0x01},
	{0x380b, 0xe0},
	{0x380c, 0x03},
	{0x380d, 0x0c},
	{0x380e, 0x02},
	{0x380f, 0x00},
	{0x3810, 0x00},
	{0x3811, 0x0f},
	{0x3812, 0x00},
	{0x3813, 0x02},
	{0x3820, 0x80},
	{0x3821, 0x06},
	{0x3814, 0x31},
	{0x3815, 0x31},
	{0x3612, 0x0b},
	{0x3618, 0x04},
	{0x3a08, 0x02},
	{0x3a09, 0x67},
	{0x3a0a, 0x02},
	{0x3a0b, 0x00},
	{0x3a0d, 0x00},
	{0x3a0e, 0x00},
	{0x4520, 0x0a},
	{0x4837, 0x29},
	{0x3000, 0xff},
	{0x3001, 0xff},
	{0x3002, 0xf0},
	{0x3600, 0x08},
	{0x3621, 0xc0},
	{0x3632, 0xd2},
	{0x3633, 0x23},
	{0x3634, 0x54},
	{0x3f01, 0x0c},
	{0x5001, 0xc1},
	{0x3614, 0xf0},
	{0x3630, 0x2d},
	{0x370b, 0x62},
	{0x3706, 0x61},
	{0x4000, 0x02},
	{0x4002, 0xc5},
	{0x4005, 0x08},
	{0x404f, 0x84},
	{0x4051, 0x00},
	{0x5000, 0xcf},
	{0x3a18, 0x00},
	{0x3a19, 0x80},
	{0x3503, 0x07},
	{0x4521, 0x00},
	{0x5183, 0xb0},
	{0x5184, 0xb0},
	{0x5185, 0xb0},
	{0x370c, 0x0c},
	{0x3035, 0x30},
	{0x3036, 0x1e},
	{0x3037, 0x21},
	{0x303e, 0x19},
	{0x3038, 0x06},
	{0x3018, 0x04},
	{0x3000, 0x00},
	{0x3001, 0x00},
	{0x3002, 0x00},
	{0x3a0f, 0x40},
	{0x3a10, 0x38},
	{0x3a1b, 0x48},
	{0x3a1e, 0x30},
	{0x3a11, 0x90},
	{0x3a1f, 0x10},
	{0x3011, 0x22},
	{0x3a00, 0x58},
};

static struct msm_camera_i2c_reg_conf ov2720_120fps_settings[] = {
	{0x3718, 0x10},
	{0x3702, 0x18},
	{0x373a, 0x3c},
	{0x3715, 0x01},
	{0x3703, 0x1d},
	{0x3705, 0x0b},
	{0x3730, 0x1f},
	{0x3704, 0x3f},
	{0x3f06, 0x1d},
	{0x371c, 0x00},
	{0x371d, 0x83},
	{0x371e, 0x00},
	{0x371f, 0xb6},
	{0x3708, 0x63},
	{0x3709, 0x52},
	{0x3800, 0x01},
	{0x3801, 0x42},
	{0x3802, 0x00},
	{0x3803, 0x40},
	{0x3804, 0x06},
	{0x3805, 0x61},
	{0x3806, 0x04},
	{0x3807, 0x08},
	{0x3808, 0x02},
	{0x3809, 0x80},
	{0x380a, 0x01},
	{0x380b, 0xe0},
	{0x380c, 0x03},
	{0x380d, 0x0c},
	{0x380e, 0x02},
	{0x380f, 0x00},
	{0x3810, 0x00},
	{0x3811, 0x0f},
	{0x3812, 0x00},
	{0x3813, 0x02},
	{0x3820, 0x80},
	{0x3821, 0x06},
	{0x3814, 0x31},
	{0x3815, 0x31},
	{0x3612, 0x0b},
	{0x3618, 0x04},
	{0x3a08, 0x02},
	{0x3a09, 0x67},
	{0x3a0a, 0x02},
	{0x3a0b, 0x00},
	{0x3a0d, 0x00},
	{0x3a0e, 0x00},
	{0x4520, 0x0a},
	{0x4837, 0x29},
	{0x3000, 0xff},
	{0x3001, 0xff},
	{0x3002, 0xf0},
	{0x3600, 0x08},
	{0x3621, 0xc0},
	{0x3632, 0xd2},
	{0x3633, 0x23},
	{0x3634, 0x54},
	{0x3f01, 0x0c},
	{0x5001, 0xc1},
	{0x3614, 0xf0},
	{0x3630, 0x2d},
	{0x370b, 0x62},
	{0x3706, 0x61},
	{0x4000, 0x02},
	{0x4002, 0xc5},
	{0x4005, 0x08},
	{0x404f, 0x84},
	{0x4051, 0x00},
	{0x5000, 0xcf},
	{0x3a18, 0x00},
	{0x3a19, 0x80},
	{0x3503, 0x07},
	{0x4521, 0x00},
	{0x5183, 0xb0},
	{0x5184, 0xb0},
	{0x5185, 0xb0},
	{0x370c, 0x0c},
	{0x3035, 0x10},
	{0x3036, 0x14},
	{0x3037, 0x21},
	{0x303e, 0x19},
	{0x3038, 0x06},
	{0x3018, 0x04},
	{0x3000, 0x00},
	{0x3001, 0x00},
	{0x3002, 0x00},
	{0x3a0f, 0x40},
	{0x3a10, 0x38},
	{0x3a1b, 0x48},
	{0x3a1e, 0x30},
	{0x3a11, 0x90},
	{0x3a1f, 0x10},
	{0x3011, 0x22},
	{0x3a00, 0x58},
};

static struct msm_camera_i2c_reg_conf ov2720_recommend_settings[] = {
	{0x0103, 0x01},
	{0x3718, 0x10},
	{0x3702, 0x24},
	{0x373a, 0x60},
	{0x3715, 0x01},
	{0x3703, 0x2e},
	{0x3705, 0x10},
	{0x3730, 0x30},
	{0x3704, 0x62},
	{0x3f06, 0x3a},
	{0x371c, 0x00},
	{0x371d, 0xc4},
	{0x371e, 0x01},
	{0x371f, 0x0d},
	{0x3708, 0x61},
	{0x3709, 0x12},
};

static struct v4l2_subdev_info ov2720_subdev_info[] = {
	{
	.code   = V4L2_MBUS_FMT_SBGGR10_1X10,
	.colorspace = V4L2_COLORSPACE_JPEG,
	.fmt    = 1,
	.order    = 0,
	},
	/* more can be supported, to be added later */
};

static struct msm_camera_i2c_conf_array ov2720_init_conf[] = {
	{&ov2720_recommend_settings[0],
	ARRAY_SIZE(ov2720_recommend_settings), 0, MSM_CAMERA_I2C_BYTE_DATA}
};

static struct msm_camera_i2c_conf_array ov2720_confs[] = {
	{&ov2720_prev_settings[0],
	ARRAY_SIZE(ov2720_prev_settings), 0, MSM_CAMERA_I2C_BYTE_DATA},
	{&ov2720_vga_settings[0],
	ARRAY_SIZE(ov2720_vga_settings), 0, MSM_CAMERA_I2C_BYTE_DATA},
	{&ov2720_720_settings[0],
	ARRAY_SIZE(ov2720_720_settings), 0, MSM_CAMERA_I2C_BYTE_DATA},
	{&ov2720_60fps_settings[0],
	ARRAY_SIZE(ov2720_60fps_settings), 0, MSM_CAMERA_I2C_BYTE_DATA},
	{&ov2720_90fps_settings[0],
	ARRAY_SIZE(ov2720_90fps_settings), 0, MSM_CAMERA_I2C_BYTE_DATA},
	{&ov2720_120fps_settings[0],
	ARRAY_SIZE(ov2720_120fps_settings), 0, MSM_CAMERA_I2C_BYTE_DATA},
};

static struct msm_sensor_output_info_t ov2720_dimensions[] = {
	{
		.x_output = 0x78C,
		.y_output = 0x444,
		.line_length_pclk = 0x85c,
		.frame_length_lines = 0x460,
		.vt_pixel_clk = 72000000,
		.op_pixel_clk = 72000000,
		.binning_factor = 1,
	},
	{
		.x_output = 0x510,
		.y_output = 0x278,
		.line_length_pclk = 0x85c,
		.frame_length_lines = 0x460,
		.vt_pixel_clk = 72000000,
		.op_pixel_clk = 72000000,
		.binning_factor = 1,
	},
	{
		.x_output = 0x298,
		.y_output = 0x1F2,
		.line_length_pclk = 0x85c,
		.frame_length_lines = 0x460,
		.vt_pixel_clk = 72000000,
		.op_pixel_clk = 72000000,
		.binning_factor = 1,
	},
	{
		.x_output = 0x280, /* 640 */
		.y_output = 0x1E0, /* 480 */
		.line_length_pclk = 0x30C, /* 780 */
		.frame_length_lines = 0x200, /* 512 */
		.vt_pixel_clk = 24000000,
		.op_pixel_clk = 24000000,
		.binning_factor = 1,
	},
	{
		.x_output = 0x280, /* 640 */
		.y_output = 0x1E0, /* 480 */
		.line_length_pclk = 0x30C, /* 780 */
		.frame_length_lines = 0x200, /* 512 */
		.vt_pixel_clk = 36000000,
		.op_pixel_clk = 36000000,
		.binning_factor = 1,
	},
	{
		.x_output = 0x280, /* 640 */
		.y_output = 0x1E0, /* 480 */
		.line_length_pclk = 0x30C, /* 780 */
		.frame_length_lines = 0x200, /* 512 */
		.vt_pixel_clk = 48000000,
		.op_pixel_clk = 48000000,
		.binning_factor = 1,
	},
};

static struct msm_camera_csid_vc_cfg ov2720_cid_cfg[] = {
	{0, CSI_RAW10, CSI_DECODE_10BIT},
	{1, CSI_EMBED_DATA, CSI_DECODE_8BIT},
};

static struct msm_camera_csi2_params ov2720_csi_params = {
	.csid_params = {
		.lane_cnt = 2,
		.lut_params = {
			.num_cid = 2,
			.vc_cfg = ov2720_cid_cfg,
		},
	},
	.csiphy_params = {
		.lane_cnt = 2,
		.settle_cnt = 0x1B,
	},
};

static struct msm_camera_csi2_params *ov2720_csi_params_array[] = {
	&ov2720_csi_params,
	&ov2720_csi_params,
	&ov2720_csi_params,
	&ov2720_csi_params,
	&ov2720_csi_params,
	&ov2720_csi_params,
};

static struct msm_sensor_output_reg_addr_t ov2720_reg_addr = {
	.x_output = 0x3808,
	.y_output = 0x380a,
	.line_length_pclk = 0x380c,
	.frame_length_lines = 0x380e,
};

static struct msm_sensor_id_info_t ov2720_id_info = {
	.sensor_id_reg_addr = 0x300A,
	.sensor_id = 0x2720,
};

static struct msm_sensor_exp_gain_info_t ov2720_exp_gain_info = {
	.coarse_int_time_addr = 0x3501,
	.global_gain_addr = 0x3508,
	.vert_offset = 6,
};

static int32_t ov2720_write_exp_gain(struct msm_sensor_ctrl_t *s_ctrl,
		uint16_t gain, uint32_t line)
{
	uint32_t fl_lines, offset;
	uint8_t int_time[3];
	fl_lines =
		(s_ctrl->curr_frame_length_lines * s_ctrl->fps_divider) / Q10;
	offset = s_ctrl->sensor_exp_gain_info->vert_offset;
	if (line > (fl_lines - offset))
		fl_lines = line + offset;

	s_ctrl->func_tbl->sensor_group_hold_on(s_ctrl);
	msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
		s_ctrl->sensor_output_reg_addr->frame_length_lines, fl_lines,
		MSM_CAMERA_I2C_WORD_DATA);
	int_time[0] = line >> 12;
	int_time[1] = line >> 4;
	int_time[2] = line << 4;
	msm_camera_i2c_write_seq(s_ctrl->sensor_i2c_client,
		s_ctrl->sensor_exp_gain_info->coarse_int_time_addr-1,
		&int_time[0], 3);
	msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
		s_ctrl->sensor_exp_gain_info->global_gain_addr, gain,
		MSM_CAMERA_I2C_WORD_DATA);
	s_ctrl->func_tbl->sensor_group_hold_off(s_ctrl);
	return 0;
}

static const struct i2c_device_id ov2720_i2c_id[] = {
	{SENSOR_NAME, (kernel_ulong_t)&ov2720_s_ctrl},
	{ }
};

static struct i2c_driver ov2720_i2c_driver = {
	.id_table = ov2720_i2c_id,
	.probe  = msm_sensor_i2c_probe,
	.driver = {
		.name = SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client ov2720_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static int __init msm_sensor_init_module(void)
{
	return i2c_add_driver(&ov2720_i2c_driver);
}

static struct v4l2_subdev_core_ops ov2720_subdev_core_ops = {
	.ioctl = msm_sensor_subdev_ioctl,
	.s_power = msm_sensor_power,
};

static struct v4l2_subdev_video_ops ov2720_subdev_video_ops = {
	.enum_mbus_fmt = msm_sensor_v4l2_enum_fmt,
};

static struct v4l2_subdev_ops ov2720_subdev_ops = {
	.core = &ov2720_subdev_core_ops,
	.video  = &ov2720_subdev_video_ops,
};

static struct msm_sensor_fn_t ov2720_func_tbl = {
	.sensor_start_stream = msm_sensor_start_stream,
	.sensor_stop_stream = msm_sensor_stop_stream,
	.sensor_group_hold_on = msm_sensor_group_hold_on,
	.sensor_group_hold_off = msm_sensor_group_hold_off,
	.sensor_set_fps = msm_sensor_set_fps,
	.sensor_write_exp_gain = ov2720_write_exp_gain,
	.sensor_write_snapshot_exp_gain = ov2720_write_exp_gain,
	.sensor_setting = msm_sensor_setting,
	.sensor_set_sensor_mode = msm_sensor_set_sensor_mode,
	.sensor_mode_init = msm_sensor_mode_init,
	.sensor_get_output_info = msm_sensor_get_output_info,
	.sensor_config = msm_sensor_config,
	.sensor_power_up = msm_sensor_power_up,
	.sensor_power_down = msm_sensor_power_down,
	.sensor_adjust_frame_lines = msm_sensor_adjust_frame_lines,
	.sensor_get_csi_params = msm_sensor_get_csi_params,
};

static struct msm_sensor_reg_t ov2720_regs = {
	.default_data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.start_stream_conf = ov2720_start_settings,
	.start_stream_conf_size = ARRAY_SIZE(ov2720_start_settings),
	.stop_stream_conf = ov2720_stop_settings,
	.stop_stream_conf_size = ARRAY_SIZE(ov2720_stop_settings),
	.group_hold_on_conf = ov2720_groupon_settings,
	.group_hold_on_conf_size = ARRAY_SIZE(ov2720_groupon_settings),
	.group_hold_off_conf = ov2720_groupoff_settings,
	.group_hold_off_conf_size =
		ARRAY_SIZE(ov2720_groupoff_settings),
	.init_settings = &ov2720_init_conf[0],
	.init_size = ARRAY_SIZE(ov2720_init_conf),
	.mode_settings = &ov2720_confs[0],
	.output_settings = &ov2720_dimensions[0],
	.num_conf = ARRAY_SIZE(ov2720_confs),
};

static struct msm_sensor_ctrl_t ov2720_s_ctrl = {
	.msm_sensor_reg = &ov2720_regs,
	.sensor_i2c_client = &ov2720_sensor_i2c_client,
	.sensor_i2c_addr = 0x6C,
	.sensor_output_reg_addr = &ov2720_reg_addr,
	.sensor_id_info = &ov2720_id_info,
	.sensor_exp_gain_info = &ov2720_exp_gain_info,
	.cam_mode = MSM_SENSOR_MODE_INVALID,
	.csi_params = &ov2720_csi_params_array[0],
	.msm_sensor_mutex = &ov2720_mut,
	.sensor_i2c_driver = &ov2720_i2c_driver,
	.sensor_v4l2_subdev_info = ov2720_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(ov2720_subdev_info),
	.sensor_v4l2_subdev_ops = &ov2720_subdev_ops,
	.func_tbl = &ov2720_func_tbl,
	.clk_rate = MSM_SENSOR_MCLK_24HZ,
};

module_init(msm_sensor_init_module);
MODULE_DESCRIPTION("Omnivision 2MP Bayer sensor driver");
MODULE_LICENSE("GPL v2");


