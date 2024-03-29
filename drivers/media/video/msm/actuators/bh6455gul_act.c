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
 *
 */

#include <linux/module.h>
#include "msm_actuator.h"
#include <sharp/sh_smem.h>

#define BH6455GUL_TOTAL_STEPS_NEAR_TO_FAR_MAX		41

/*Direct Mode DAC code setting*/
#define AF_DIRECT_MODE_ADDR (0xC0)
/*Step Mode DAC code setting*/
#define AF_STEP_MODE_ADDR (0xC8)
/*Step Time Setting and Resolution Setting in Step Mode*/
#define AF_STS_RS_VALUE_ADDR (0xCC)
/*Stand by mode place in high impedance state*/
#define AF_DISABLE (0x00)


DEFINE_MUTEX(bh6455gul_act_mutex);
static struct msm_actuator_ctrl_t bh6455gul_actuator_t;

int32_t bh6455gul_actuator_init_step_table(struct msm_actuator_ctrl_t *a_ctrl,
	struct msm_actuator_set_info_t *set_info);

int32_t bh6455gul_actuator_i2c_write(struct msm_actuator_ctrl_t *a_ctrl,
	int16_t next_lens_position, uint32_t hw_params);

static struct msm_actuator bh6455gul_vcm_actuator_table = {
	.act_type = ACTUATOR_VCM,
	.func_tbl = {
//		.actuator_init_step_table = msm_actuator_init_step_table,
		.actuator_init_step_table  = bh6455gul_actuator_init_step_table,
		.actuator_move_focus = msm_actuator_move_focus,
		.actuator_write_focus = msm_actuator_write_focus,
		.actuator_set_default_focus = msm_actuator_set_default_focus,
		.actuator_init_focus = msm_actuator_init_focus,
		.actuator_i2c_write = bh6455gul_actuator_i2c_write,
	},
};

static struct msm_actuator *actuators[] = {
	&bh6455gul_vcm_actuator_table,
};

int32_t bh6455gul_actuator_i2c_write(struct msm_actuator_ctrl_t *a_ctrl,
	int16_t next_lens_position, uint32_t hw_params)
{
	uint8_t codeval_msb, codeval_lsb;
	uint16_t bh6455gul_vcm_step_time;
	int32_t rc = 0;

	bh6455gul_vcm_step_time = hw_params;

	CDBG("%s bh6455gul_vcm_step_time:0x%0x next_lens_position:%d\n",
		__func__, bh6455gul_vcm_step_time, next_lens_position);

	rc = msm_camera_i2c_write(&a_ctrl->i2c_client,
			AF_STS_RS_VALUE_ADDR,
			bh6455gul_vcm_step_time,
			MSM_CAMERA_I2C_WORD_DATA);
	if (rc < 0) {
		pr_err("%s: i2c write error:%d\n",
			__func__, rc);
		return rc;
	}

	codeval_msb = (AF_STEP_MODE_ADDR |
		((next_lens_position & 0x0300)>>8));
	codeval_lsb = next_lens_position & 0x00FF;

	rc = msm_camera_i2c_write(&a_ctrl->i2c_client,
			codeval_msb,
			codeval_lsb,
			MSM_CAMERA_I2C_BYTE_DATA);
	return rc;
}

int32_t bh6455gul_actuator_power_down(struct msm_actuator_ctrl_t *a_ctrl)
{
	int32_t rc = 0;
	if (a_ctrl->vcm_enable) {
		rc = gpio_direction_output(a_ctrl->vcm_pwd, 0);
		if (!rc)
			gpio_free(a_ctrl->vcm_pwd);
	}

	kfree(a_ctrl->step_position_table);
	a_ctrl->step_position_table = NULL;
	return rc;
}

int32_t bh6455gul_actuator_init(struct msm_actuator_ctrl_t *a_ctrl,
	struct msm_actuator_set_info_t *set_info) {
	struct reg_settings_t *init_settings = NULL;
	int32_t rc = -EFAULT;
	uint16_t i = 0;
	CDBG("%s: IN\n", __func__);

	for (i = 0; i < ARRAY_SIZE(actuators); i++) {
		if (set_info->actuator_params.act_type ==
			actuators[i]->act_type) {
			a_ctrl->func_tbl = &actuators[i]->func_tbl;
			rc = 0;
		}
	}

	if (rc < 0) {
		pr_err("%s: Actuator function table not found\n", __func__);
		return rc;
	}

	a_ctrl->region_size = set_info->af_tuning_params.region_size;
	if (a_ctrl->region_size > MAX_ACTUATOR_REGION) {
		pr_err("%s: MAX_ACTUATOR_REGION is exceeded.\n", __func__);
		return -EFAULT;
	}
	a_ctrl->total_steps = set_info->af_tuning_params.total_steps;
	a_ctrl->pwd_step = set_info->af_tuning_params.pwd_step;
	a_ctrl->total_steps = set_info->af_tuning_params.total_steps;

	if (copy_from_user(&a_ctrl->region_params,
		(void *)set_info->af_tuning_params.region_params,
		a_ctrl->region_size * sizeof(struct region_params_t)))
		return -EFAULT;

	a_ctrl->i2c_data_type = set_info->actuator_params.i2c_data_type;
	a_ctrl->i2c_client.client->addr = set_info->actuator_params.i2c_addr;
	a_ctrl->i2c_client.addr_type = set_info->actuator_params.i2c_addr_type;
	a_ctrl->reg_tbl_size = set_info->actuator_params.reg_tbl_size;
	if (a_ctrl->reg_tbl_size > MAX_ACTUATOR_REG_TBL_SIZE) {
		pr_err("%s: MAX_ACTUATOR_REG_TBL_SIZE is exceeded.\n",
			__func__);
		return -EFAULT;
	}
	if (copy_from_user(&a_ctrl->reg_tbl,
		(void *)set_info->actuator_params.reg_tbl_params,
		a_ctrl->reg_tbl_size *
		sizeof(struct msm_actuator_reg_params_t)))
		return -EFAULT;

	if (set_info->actuator_params.init_setting_size) {
		if (a_ctrl->func_tbl->actuator_init_focus) {
			init_settings = kmalloc(sizeof(struct reg_settings_t) *
				(set_info->actuator_params.init_setting_size),
				GFP_KERNEL);
			if (init_settings == NULL) {
				pr_err("%s Error allocating memory for init_settings\n",
					__func__);
				return -EFAULT;
			}
			if (copy_from_user(init_settings,
				(void *)set_info->actuator_params.init_settings,
				set_info->actuator_params.init_setting_size *
				sizeof(struct reg_settings_t))) {
				kfree(init_settings);
				pr_err("%s Error copying init_settings\n",
					__func__);
				return -EFAULT;
			}
			rc = a_ctrl->func_tbl->actuator_init_focus(a_ctrl,
				set_info->actuator_params.init_setting_size,
				a_ctrl->i2c_data_type,
				init_settings);
			kfree(init_settings);
			if (rc < 0) {
				pr_err("%s Error actuator_init_focus\n",
					__func__);
				return -EFAULT;
			}
		}
	}

#if 0
	a_ctrl->initial_code = set_info->af_tuning_params.initial_code;
#else
	a_ctrl->initial_code = set_info->af_tuning_params.initial_code;
	a_ctrl->terminal_code =  set_info->af_tuning_params.terminal_code;
	CDBG("%s %d : a_ctrl->initial_code = %d, a_ctrl->terminal_code=%d\n",__func__ ,__LINE__ ,a_ctrl->initial_code, a_ctrl->terminal_code);
#endif
	if (a_ctrl->func_tbl->actuator_init_step_table)
		rc = a_ctrl->func_tbl->
			actuator_init_step_table(a_ctrl, set_info);

	a_ctrl->curr_step_pos = 0;
	a_ctrl->curr_region_index = 0;

	return rc;
}

int32_t bh6455gul_actuator_init_step_table(struct msm_actuator_ctrl_t *a_ctrl,
	struct msm_actuator_set_info_t *set_info)
{
	int32_t rc = 0;
	int16_t cur_code = 0;
	int16_t end_code = 0;
	int16_t step_index = 0, region_index = 0;
    int16_t code_per_step = 0;
	uint16_t step_boundary = 0;
	uint32_t max_code_size = 1;
	uint32_t total_steps = 0;
	uint16_t data_size = set_info->actuator_params.data_size;
	CDBG("%s called\n", __func__);

	for (; data_size > 0; data_size--)
		max_code_size *= 2;

	if(a_ctrl->step_position_table)
		kfree(a_ctrl->step_position_table);

	a_ctrl->step_position_table = NULL;

	/* Fill step position table */
	a_ctrl->step_position_table =
		kmalloc(sizeof(uint16_t) *
		(set_info->af_tuning_params.total_steps + 1), GFP_KERNEL);

	if (a_ctrl->step_position_table == NULL)
		return -EFAULT;

	if(bh6455gul_actuator_t.af_adjustflg == 0x55){
		
		
		cur_code = a_ctrl->initial_code;
		end_code = a_ctrl->terminal_code;
		total_steps = a_ctrl->total_steps;

		CDBG("cur_code= %d, end_code =%d, total_steps=%d\n",cur_code, end_code, total_steps);

		for ( step_index = 0; step_index <= total_steps; step_index++)
		{
			a_ctrl->step_position_table[step_index] = cur_code + step_index * ( end_code - cur_code ) / (total_steps);
			CDBG("step_position_table[%d]= %d\n",step_index,a_ctrl->step_position_table[step_index]);
		}
	}else
	{
		cur_code = 0x0130;
		a_ctrl->step_position_table[step_index++] = cur_code;
		for (region_index = 0;
			region_index < a_ctrl->region_size;
			region_index++) {
			code_per_step =
				a_ctrl->region_params[region_index].code_per_step;
			step_boundary =
				a_ctrl->region_params[region_index].
				step_bound[MOVE_NEAR];
			for (; step_index <= step_boundary;
				step_index++) {
				cur_code += code_per_step;
				if (cur_code < max_code_size)
					a_ctrl->step_position_table[step_index] =
						cur_code;
				else {
					for (; step_index <
						set_info->af_tuning_params.total_steps;
						step_index++)
						a_ctrl->
							step_position_table[
							step_index] =
							max_code_size;

				return rc;
				}
			}
		}
		
	}
	return rc;
}


static int bh6455gul_actuator_config(struct msm_actuator_ctrl_t *a_ctrl,
							void __user *argp)
{
	struct msm_actuator_cfg_data cdata;
	int32_t rc = 0;
	if (copy_from_user(&cdata,
		(void *)argp,
		sizeof(struct msm_actuator_cfg_data)))
		return -EFAULT;
	mutex_lock(a_ctrl->actuator_mutex);
	CDBG("%s called, type %d\n", __func__, cdata.cfgtype);
	switch (cdata.cfgtype) {
	case CFG_GET_ACTUATOR_INFO:
		cdata.cfg.get_info.step_shido = bh6455gul_actuator_t.step_shido;
		cdata.cfg.get_info.step_100_up = bh6455gul_actuator_t.step_100_up;
		cdata.cfg.get_info.step_500_up = bh6455gul_actuator_t.step_500_up;
		cdata.cfg.get_info.step_shido_up = bh6455gul_actuator_t.step_shido_up;
		if (copy_to_user((void *)argp,
				 &cdata,
				 sizeof(struct msm_actuator_cfg_data)))
			rc = -EFAULT;
		break;
	case CFG_SET_ACTUATOR_INFO:
		rc = bh6455gul_actuator_init(a_ctrl, &cdata.cfg.set_info);
		if (rc < 0)
			pr_err("%s init table failed %d\n", __func__, rc);
		break;

	case CFG_SET_DEFAULT_FOCUS:
		rc = a_ctrl->func_tbl->actuator_set_default_focus(a_ctrl,
			&cdata.cfg.move);
		if (rc < 0)
			pr_err("%s move focus failed %d\n", __func__, rc);
		break;

	case CFG_MOVE_FOCUS:
		rc = a_ctrl->func_tbl->actuator_move_focus(a_ctrl,
			&cdata.cfg.move);
		if (rc < 0)
			pr_err("%s move focus failed %d\n", __func__, rc);
		break;

	case CFG_SHDIAG_GET_I2C_DATA:
		{
			void *data;
			uint16_t saddr = a_ctrl->i2c_client.client->addr >> 1;
			struct i2c_msg msgs[] = {
				{
					.flags = I2C_M_RD,
					.len   = 3,
				},
			};

			data = kmalloc(cdata.cfg.i2c_info.length, GFP_KERNEL);
			if(data == NULL){
				return -EFAULT;
			}
			CDBG("%s:%d saddr=0x%0x\n",__func__,__LINE__, saddr);
			
			msgs[0].addr=saddr;
			msgs[0].buf=data;

			rc = i2c_transfer(a_ctrl->i2c_client.client->adapter, msgs, 1);
			if (rc < 0){
				LINFO("msm_camera_i2c_rxdata failed 0x%x\n", saddr);
				kfree(data);
				return -EFAULT;
			}

			CDBG("%s:%d i2c_read data=0x%02x%02x\n",__func__,__LINE__,*(unsigned char *)data,*((unsigned char *)data + 1));
			if (copy_to_user((void *)cdata.cfg.i2c_info.data,
				data,
				cdata.cfg.i2c_info.length)){
				kfree(data);
				LINFO("%s copy_to_user error\n",__func__);
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
			uint16_t saddr = a_ctrl->i2c_client.client->addr >> 1;
			struct i2c_msg msg[] = {
				{
					.flags = 0,
					.len = 3,
				 },
			};
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
			CDBG("%s:%d i2c_write data=0x%02x%02x%02x\n",__func__,__LINE__,*(unsigned char *)data,*((unsigned char *)data + 1),*((unsigned char *)data +2));
			
			msg[0].addr=saddr;
			msg[0].buf=data;

			rc = i2c_transfer(a_ctrl->i2c_client.client->adapter, msg, 1);
			if (rc < 0){
				CDBG("i2c_transfer faild 0x%x\n", saddr);
				kfree(data);
				return -EFAULT;
			}
			
			kfree(data);
			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
			return -EFAULT;
		}
		break;

	case CFG_SHDIAG_ACTUATOR_SET_TABLE:
		{
			uint16_t lens_start, lens_end, search_point;
			int16_t step_index = 0;
			
			lens_start = cdata.cfg.af_info.lens_start;
			lens_end = cdata.cfg.af_info.lens_end;
			search_point = cdata.cfg.af_info.search_point - 1;
			CDBG("%s lens_start = %d lens_end = %d search_point = %d\n", __func__, lens_start, lens_end, search_point );
			
			if(a_ctrl->step_position_table != NULL){
				CDBG("%s free default step_position_table\n", __func__ );
				kfree(a_ctrl->step_position_table);
			}
			if(lens_start < 0){
				return -EFAULT;
			}
			if(lens_end > 0x3ff){
				return -EFAULT;
			}
			if(search_point > ( lens_end - lens_start )){
				return -EFAULT;
			}
			a_ctrl->step_position_table = kmalloc(sizeof(uint16_t) * (search_point + 1) , GFP_KERNEL);
			if(a_ctrl->step_position_table == NULL){
				return -EFAULT;
			}
			
			a_ctrl->step_position_table[step_index++] = lens_start;
			
			for (; step_index <= search_point; step_index++) {
				a_ctrl->step_position_table[step_index] = lens_start + ( step_index * ( lens_end - lens_start ) + search_point / 2) / search_point;
			}
			
			for (step_index = 0; step_index <= search_point; step_index++) {
				CDBG("step_position_table[%d]= 0x%0x\n", step_index, a_ctrl->step_position_table[step_index]);
			}
			a_ctrl->curr_step_pos = 0;
			a_ctrl->curr_region_index = 0;
			a_ctrl->total_steps = search_point + 1;
			a_ctrl->region_params[0].step_bound[0] = search_point + 1;

			if (copy_to_user((void *)cdata.cfg.af_info.p_step_position_table,
				a_ctrl->step_position_table,
				sizeof(uint16_t) * (search_point + 1)))
			return -EFAULT;

		}
		break;
		case CFG_SH_ACTUATOR_INIT_LENSPOS:
		{
			int16_t lens_pos;
			int16_t *lens;
			void *data;
			lens = &lens_pos;
			
			data = kmalloc(cdata.cfg.i2c_info.length, GFP_KERNEL);
			if(data == NULL){
				return -EFAULT;
			}
			if(copy_from_user(data,
			   (void *)cdata.cfg.i2c_info.data,
			   cdata.cfg.i2c_info.length)){
			   LINFO("%s copy_to_user error\n",__func__);
			   return -EFAULT;
			
			}
			
			memcpy(lens, data, cdata.cfg.i2c_info.length);
			LINFO("%s lens_pos : %d\n", __func__ , lens_pos);

			if(lens_pos >= bh6455gul_actuator_t.initial_code && 
			   lens_pos <= bh6455gul_actuator_t.terminal_code )
			{
				uint8_t codeval_msb, codeval_lsb;
				
				codeval_msb = (AF_DIRECT_MODE_ADDR |
					((lens_pos & 0x0300)>>8));
				codeval_lsb = lens_pos & 0x00FF;

				msm_camera_i2c_write(&bh6455gul_actuator_t.i2c_client,
					codeval_msb,
					codeval_lsb,
					MSM_CAMERA_I2C_BYTE_DATA);

			    if(lens_pos - bh6455gul_actuator_t.step_position_table[0] <
			       bh6455gul_actuator_t.step_position_table[BH6455GUL_TOTAL_STEPS_NEAR_TO_FAR_MAX-1] - lens_pos){
					bh6455gul_actuator_t.curr_step_pos = 0;
				}
				else{
					bh6455gul_actuator_t.curr_step_pos = BH6455GUL_TOTAL_STEPS_NEAR_TO_FAR_MAX-1;
				}

			}
			
			kfree(data);
		}
		break;
	case CFG_SHDIAG_SET_SMEM:
		{
			void *data;
			sharp_smem_common_type *p_sh_smem_common_type = NULL;
			data = kmalloc(cdata.cfg.i2c_info.length, GFP_KERNEL);
			if(data == NULL){
				rc = -EFAULT;
				break;
			}
			if (copy_from_user(data,
				(void *)cdata.cfg.i2c_info.data,
				cdata.cfg.i2c_info.length)){
				kfree(data);
				CDBG("%s copy_to_user error\n",__func__);
				rc = -EFAULT;
				break;
			}
			p_sh_smem_common_type = sh_smem_get_common_address();
		    p_sh_smem_common_type->sh_camOtpData[0] = *(unsigned char *)data;
		    p_sh_smem_common_type->sh_camOtpData[1] = *((unsigned char *)data + 1);
		    p_sh_smem_common_type->sh_camOtpData[2] = *((unsigned char *)data +2);
		    
			LINFO("%s sh_camOtpData[0]=0x%02x\n", __func__, p_sh_smem_common_type->sh_camOtpData[0]);
			LINFO("%s sh_camOtpData[1]=0x%02x\n", __func__, p_sh_smem_common_type->sh_camOtpData[1]);
			LINFO("%s sh_camOtpData[2]=0x%02x\n", __func__, p_sh_smem_common_type->sh_camOtpData[2]);
			LINFO("%s sh_camOtpData[3]=0x%02x\n", __func__, p_sh_smem_common_type->sh_camOtpData[3]);
			if(p_sh_smem_common_type->sh_camOtpData[0] == 0x55){
				bh6455gul_actuator_t.step_shido = (p_sh_smem_common_type->sh_camOtpData[1]<<8) + p_sh_smem_common_type->sh_camOtpData[2];
			}

		    kfree(data);
			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data))) {
				rc = -EFAULT;
				break;
			}
		}
		break;

	default:
		break;
	}
	mutex_unlock(a_ctrl->actuator_mutex);
	return rc;
}

static int32_t bh6455gul_actuator_i2c_probe(
	struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int rc = 0;
	struct msm_actuator_ctrl_t *act_ctrl_t = NULL;
	sharp_smem_common_type *p_sh_smem_common_type = NULL;
	CDBG("%s called\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("i2c_check_functionality failed\n");
		goto probe_failure;
	}

	act_ctrl_t = (struct msm_actuator_ctrl_t *)(id->driver_data);
	CDBG("%s client = %x\n",
		__func__, (unsigned int) client);
	act_ctrl_t->i2c_client.client = client;

	/* Assign name for sub device */
	snprintf(act_ctrl_t->sdev.name, sizeof(act_ctrl_t->sdev.name),
			 "%s", act_ctrl_t->i2c_driver->driver.name);

	/* Initialize sub device */
	v4l2_i2c_subdev_init(&act_ctrl_t->sdev,
		act_ctrl_t->i2c_client.client,
		act_ctrl_t->act_v4l2_subdev_ops);

	p_sh_smem_common_type = sh_smem_get_common_address();

	if( p_sh_smem_common_type != NULL){
		CDBG("%s sh_camOtpData[0]=0x%02x\n", __func__, p_sh_smem_common_type->sh_camOtpData[0]);
		CDBG("%s sh_camOtpData[0]=0x%02x\n", __func__, p_sh_smem_common_type->sh_camOtpData[1]);
		CDBG("%s sh_camOtpData[0]=0x%02x\n", __func__, p_sh_smem_common_type->sh_camOtpData[2]);
		CDBG("%s sh_camOtpData[0]=0x%02x\n", __func__, p_sh_smem_common_type->sh_camOtpData[3]);
		if(p_sh_smem_common_type->sh_camOtpData[0] == 0x55){
			bh6455gul_actuator_t.af_adjustflg = p_sh_smem_common_type->sh_camOtpData[0];
		
			bh6455gul_actuator_t.step_shido = (p_sh_smem_common_type->sh_camOtpData[1]<<8) + p_sh_smem_common_type->sh_camOtpData[2];
		}
		CDBG("%s step_shido=0x%08x\n", __func__, (unsigned int)bh6455gul_actuator_t.step_shido);
	}

	CDBG("%s succeeded\n", __func__);
	return rc;

probe_failure:
	pr_err("%s failed! rc = %d\n", __func__, rc);
	return rc;
}

int32_t bh6455gul_actuator_power_up(struct msm_actuator_ctrl_t *a_ctrl)
{
	int rc = 0;
	CDBG("%s called\n", __func__);

	CDBG("vcm info: %x %x\n", a_ctrl->vcm_pwd,
		a_ctrl->vcm_enable);
	if (a_ctrl->vcm_enable) {
		rc = gpio_request(a_ctrl->vcm_pwd, "msm_actuator");
		if (!rc) {
			CDBG("Enable VCM PWD\n");
			gpio_direction_output(a_ctrl->vcm_pwd, 1);
		}
	}
	return rc;
}

DEFINE_MUTEX(bh6455gul_actuator_mutex);

static const struct i2c_device_id bh6455gul_actuator_i2c_id[] = {
	{"bh6455gul_act", (kernel_ulong_t)&bh6455gul_actuator_t},
	{ }
};

static struct i2c_driver bh6455gul_actuator_i2c_driver = {
	.id_table = bh6455gul_actuator_i2c_id,
	.probe  = bh6455gul_actuator_i2c_probe,
	.remove = __exit_p(msm_actuator_i2c_remove),
	.driver = {
		.name = "bh6455gul_act",
	},
};

static int __init bh6455gul_actuator_i2c_add_driver(
	void)
{
	CDBG("%s called\n", __func__);
	return i2c_add_driver(bh6455gul_actuator_t.i2c_driver);
}

long bh6455gul_actuator_subdev_ioctl(struct v4l2_subdev *sd,
			unsigned int cmd, void *arg)
{
	struct msm_actuator_ctrl_t *a_ctrl = get_actrl(sd);
	void __user *argp = (void __user *)arg;
	switch (cmd) {
	case VIDIOC_MSM_ACTUATOR_CFG:
		return bh6455gul_actuator_config(a_ctrl, argp);
	default:
		return -ENOIOCTLCMD;
	}
}

int32_t bh6455gul_actuator_power(struct v4l2_subdev *sd, int on)
{
	int rc = 0;
	struct msm_actuator_ctrl_t *a_ctrl = get_actrl(sd);
	mutex_lock(a_ctrl->actuator_mutex);
	if (on)
		rc = bh6455gul_actuator_power_up(a_ctrl);
	else
		rc = bh6455gul_actuator_power_down(a_ctrl);
	mutex_unlock(a_ctrl->actuator_mutex);
	return rc;
}

static struct v4l2_subdev_core_ops bh6455gul_act_subdev_core_ops = {
	.ioctl = bh6455gul_actuator_subdev_ioctl,
	.s_power = bh6455gul_actuator_power,
};

static struct v4l2_subdev_ops bh6455gul_act_subdev_ops = {
	.core = &bh6455gul_act_subdev_core_ops,
};

static struct msm_actuator_ctrl_t bh6455gul_actuator_t = {
	.i2c_driver = &bh6455gul_actuator_i2c_driver,
	.act_v4l2_subdev_ops = &bh6455gul_act_subdev_ops,

	.total_steps = BH6455GUL_TOTAL_STEPS_NEAR_TO_FAR_MAX,
	.step_shido    = 0,
	.step_100_up   = 0,
	.step_500_up   = 0,
	.step_shido_up = 0,
	.curr_step_pos = 0,
	.curr_region_index = 0,
	.initial_code = 0,
	.terminal_code = 0,
	.actuator_mutex = &bh6455gul_actuator_mutex,

};

subsys_initcall(bh6455gul_actuator_i2c_add_driver);
MODULE_DESCRIPTION("BH6455GUL actuator");
MODULE_LICENSE("GPL v2");
