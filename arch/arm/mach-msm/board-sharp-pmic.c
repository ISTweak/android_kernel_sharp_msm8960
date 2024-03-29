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

#include <linux/interrupt.h>
#include <linux/mfd/pm8xxx/pm8921.h>
#include <linux/mfd/pm8xxx/pm8xxx-adc.h>
#include <linux/leds.h>
#include <linux/leds-pm8xxx.h>
#include <linux/msm_ssbi.h>
#ifdef CONFIG_KEYBOARD_SCKEY
#include <linux/input/sh_pmic8xxx-keypad.h>
#endif
#include <asm/mach-types.h>
#include <mach/msm_bus_board.h>
#include <mach/restart.h>
#ifdef CONFIG_BATTERY_SH
#include <mach/gpio.h>
#ifdef CONFIG_PM_MPP_11_PU_30KOHM
#include <sharp/sh_smem.h>
#endif
#endif /* CONFIG_BATTERY_SH */
#include "devices.h"
#include "board-8960.h"

struct pm8xxx_gpio_init {
	unsigned			gpio;
	struct pm_gpio			config;
};

struct pm8xxx_mpp_init {
	unsigned			mpp;
	struct pm8xxx_mpp_config_data	config;
};

#define PM8XXX_GPIO_INIT(_gpio, _dir, _buf, _val, _pull, _vin, _out_strength, \
			_func, _inv, _disable) \
{ \
	.gpio	= PM8921_GPIO_PM_TO_SYS(_gpio), \
	.config	= { \
		.direction	= _dir, \
		.output_buffer	= _buf, \
		.output_value	= _val, \
		.pull		= _pull, \
		.vin_sel	= _vin, \
		.out_strength	= _out_strength, \
		.function	= _func, \
		.inv_int_pol	= _inv, \
		.disable_pin	= _disable, \
	} \
}

#define PM8XXX_MPP_INIT(_mpp, _type, _level, _control) \
{ \
	.mpp	= PM8921_MPP_PM_TO_SYS(_mpp), \
	.config	= { \
		.type		= PM8XXX_MPP_TYPE_##_type, \
		.level		= _level, \
		.control	= PM8XXX_MPP_##_control, \
	} \
}

#define PM8XXX_GPIO_DISABLE(_gpio) \
	PM8XXX_GPIO_INIT(_gpio, PM_GPIO_DIR_IN, 0, 0, 0, PM_GPIO_VIN_S4, \
			 0, 0, 0, 1)

#define PM8XXX_GPIO_OUTPUT(_gpio, _val) \
	PM8XXX_GPIO_INIT(_gpio, PM_GPIO_DIR_OUT, PM_GPIO_OUT_BUF_CMOS, _val, \
			PM_GPIO_PULL_NO, PM_GPIO_VIN_S4, \
			PM_GPIO_STRENGTH_HIGH, \
			PM_GPIO_FUNC_NORMAL, 0, 0)

#define PM8XXX_GPIO_INPUT(_gpio, _pull) \
	PM8XXX_GPIO_INIT(_gpio, PM_GPIO_DIR_IN, PM_GPIO_OUT_BUF_CMOS, 0, \
			_pull, PM_GPIO_VIN_S4, \
			PM_GPIO_STRENGTH_NO, \
			PM_GPIO_FUNC_NORMAL, 0, 0)

#define PM8XXX_GPIO_OUTPUT_FUNC(_gpio, _val, _func) \
	PM8XXX_GPIO_INIT(_gpio, PM_GPIO_DIR_OUT, PM_GPIO_OUT_BUF_CMOS, _val, \
			PM_GPIO_PULL_NO, PM_GPIO_VIN_S4, \
			PM_GPIO_STRENGTH_HIGH, \
			_func, 0, 0)

#define PM8XXX_GPIO_OUTPUT_VIN(_gpio, _val, _vin) \
	PM8XXX_GPIO_INIT(_gpio, PM_GPIO_DIR_OUT, PM_GPIO_OUT_BUF_CMOS, _val, \
			PM_GPIO_PULL_NO, _vin, \
			PM_GPIO_STRENGTH_HIGH, \
			PM_GPIO_FUNC_NORMAL, 0, 0)

#define PM8XXX_GPIO_OUTPUT_STRENGTH(_gpio, _val, _out_strength) \
	PM8XXX_GPIO_INIT(_gpio, PM_GPIO_DIR_OUT, PM_GPIO_OUT_BUF_CMOS, _val, \
			PM_GPIO_PULL_NO, PM_GPIO_VIN_S4, \
			_out_strength, \
			PM_GPIO_FUNC_NORMAL, 0, 0)

#ifdef CONFIG_BATTERY_SH
#define PM8XXX_GPIO_INPUT_DISABLE(_gpio) \
	PM8XXX_GPIO_INIT(_gpio, PM_GPIO_DIR_IN, 0, 0, PM_GPIO_PULL_DN, PM_GPIO_VIN_S4, \
			 0, 0, 0, 0)

#define PM8XXX_GPIO_INPUT_FUNC(_gpio, _pull, _func) \
	PM8XXX_GPIO_INIT(_gpio, PM_GPIO_DIR_IN, PM_GPIO_OUT_BUF_CMOS, 0, \
			_pull, PM_GPIO_VIN_S4, \
			PM_GPIO_STRENGTH_NO, \
			_func, 0, 0)

#define PM8XXX_GPIO_OUTPUT_FUNC_VIN(_gpio, _val, _vin, _func) \
	PM8XXX_GPIO_INIT(_gpio, PM_GPIO_DIR_OUT, PM_GPIO_OUT_BUF_CMOS, _val, \
			PM_GPIO_PULL_NO, _vin, \
			PM_GPIO_STRENGTH_HIGH, \
			_func, 0, 0)
#endif /* CONFIG_BATTERY_SH */

/* Initial PM8921 GPIO configurations */
static struct pm8xxx_gpio_init pm8921_gpios[] __initdata = {
#ifndef CONFIG_BATTERY_SH
	PM8XXX_GPIO_OUTPUT_VIN(6, 1, PM_GPIO_VIN_VPH),	 /* MHL power EN_N */
	PM8XXX_GPIO_DISABLE(7),				 /* Disable NFC */
	PM8XXX_GPIO_INPUT(16,	    PM_GPIO_PULL_UP_30), /* SD_CARD_WP */
    /* External regulator shared by display and touchscreen on LiQUID */
	PM8XXX_GPIO_OUTPUT(17,	    0),			 /* DISP 3.3 V Boost */
	PM8XXX_GPIO_OUTPUT(18,	0),	/* TABLA SPKR_LEFT_EN=off */
	PM8XXX_GPIO_OUTPUT(19,	0),	/* TABLA SPKR_RIGHT_EN=off */
	PM8XXX_GPIO_DISABLE(22),			 /* Disable NFC */
	PM8XXX_GPIO_OUTPUT_FUNC(25, 0, PM_GPIO_FUNC_2),	 /* TN_CLK */
	PM8XXX_GPIO_INPUT(26,	    PM_GPIO_PULL_UP_30), /* SD_CARD_DET_N */
	PM8XXX_GPIO_OUTPUT(43, 1),                       /* DISP_RESET_N */
	PM8XXX_GPIO_OUTPUT(42, 0),                      /* USB 5V reg enable */
	/* TABLA CODEC RESET */
	PM8XXX_GPIO_OUTPUT_STRENGTH(34, 1, PM_GPIO_STRENGTH_MED)
#else  /* CONFIG_BATTERY_SH */
#if defined(CONFIG_PM_GPIO_KEY)
	PM8XXX_GPIO_INPUT_FUNC(1, PM_GPIO_PULL_UP_1P5_30, PM_GPIO_FUNC_1),	/* KYPD_SNS1 1,2,3,NA */
	PM8XXX_GPIO_INPUT_FUNC(2, PM_GPIO_PULL_UP_1P5_30, PM_GPIO_FUNC_1),	/* KYPD_SNS2 4,5,6,back */
	PM8XXX_GPIO_INPUT_FUNC(3, PM_GPIO_PULL_UP_1P5_30, PM_GPIO_FUNC_1),	/* KYPD_SNS3 7,7,8, */
	PM8XXX_GPIO_INPUT_FUNC(4, PM_GPIO_PULL_UP_1P5_30, PM_GPIO_FUNC_1),	/* KYPD_SNS4 *,0,#,AF */
#else
	PM8XXX_GPIO_INPUT_DISABLE(1),				/* Disable unused */
	PM8XXX_GPIO_INPUT_DISABLE(2),				/* Disable unused */
	PM8XXX_GPIO_INPUT_DISABLE(3),				/* Disable unused */
	PM8XXX_GPIO_INPUT_DISABLE(4),				/* Disable unused */
#endif
	PM8XXX_GPIO_INPUT_FUNC(5, PM_GPIO_PULL_UP_1P5_30, PM_GPIO_FUNC_1),	/* KYPD_SNS5 UP */
	PM8XXX_GPIO_INPUT_FUNC(6, PM_GPIO_PULL_UP_1P5_30, PM_GPIO_FUNC_1),	/* KYPD_SNS6 DOWN */
#if defined(CONFIG_PM_GPIO_KEY)
	PM8XXX_GPIO_INPUT_FUNC(7, PM_GPIO_PULL_UP_1P5_30, PM_GPIO_FUNC_1),	/* KYPD_SNS7 left,right */
#else
	PM8XXX_GPIO_INPUT_DISABLE(7),				/* Disable unused */
#endif
	PM8XXX_GPIO_INPUT_DISABLE(8),				/* Disable unused */
#if defined(CONFIG_PM_GPIO_KEY)
	PM8XXX_GPIO_OUTPUT_FUNC(9, 0, PM_GPIO_FUNC_1),	/* KEYPD_DRV1 1,4,7,*,menu */
	PM8XXX_GPIO_OUTPUT_FUNC(10, 0, PM_GPIO_FUNC_1),	/* KEYPD_DRV2 2,5,8,0,mail */
	PM8XXX_GPIO_OUTPUT_FUNC(11, 0, PM_GPIO_FUNC_1),	/* KEYPD_DRV3 3,6,9,#,web */
	PM8XXX_GPIO_OUTPUT_FUNC(12, 0, PM_GPIO_FUNC_1),	/* KEYPD_DRV4 back,up,left */
#else
	PM8XXX_GPIO_INPUT_DISABLE(9),				/* Disable unused */
	PM8XXX_GPIO_INPUT_DISABLE(10),				/* Disable unused */
	PM8XXX_GPIO_INPUT_DISABLE(11),				/* Disable unused */
	PM8XXX_GPIO_INPUT_DISABLE(12),				/* Disable unused */
#endif
#if defined(CONFIG_PM_GPIO_KEY_DISABLE)
	PM8XXX_GPIO_INPUT_DISABLE(13),				/* Disable unused */
	PM8XXX_GPIO_INPUT_DISABLE(14),				/* Disable unused */
#elif defined(CONFIG_PM_GPIO_KEY)
	PM8XXX_GPIO_OUTPUT_FUNC(13, 0, PM_GPIO_FUNC_1),	/* KEYPD_DRV5 vol-,vol- */
	PM8XXX_GPIO_OUTPUT_FUNC(14, 0, PM_GPIO_FUNC_1),	/* KEYPD_DRV6 0,AF,CENTER,DOWN,RIGHT */
#else
	PM8XXX_GPIO_INPUT(13, PM_GPIO_PULL_NO),		/* INTU */
	PM8XXX_GPIO_OUTPUT(14, 0),					/* HSEL */
#endif
#if defined(CONFIG_PM_GPIO_MM_ANT)
	PM8XXX_GPIO_INPUT(15, PM_GPIO_PULL_DN),		/* [ANT_SW] */
	PM8XXX_GPIO_INPUT(16, PM_GPIO_PULL_DN),		/* [MANT_SEL] */
	PM8XXX_GPIO_INPUT(17, PM_GPIO_PULL_DN),		/* [UANT_SEL] */
	PM8XXX_GPIO_INPUT(18, PM_GPIO_PULL_DN),		/* [CRANT_SEL] */
#else
	PM8XXX_GPIO_INPUT_DISABLE(15),				/* Disable unused */
	PM8XXX_GPIO_INPUT_DISABLE(16),				/* Disable unused */
	PM8XXX_GPIO_INPUT_DISABLE(17),				/* Disable unused */
	PM8XXX_GPIO_INPUT_DISABLE(18),				/* Disable unused */
#endif
#if defined(CONFIG_PM_GPIO_FLIP_DET)
	PM8XXX_GPIO_INPUT(19, PM_GPIO_PULL_NO),		/* FLIP_DET_N XChoM */
#else
	PM8XXX_GPIO_INPUT_DISABLE(19),				/* Disable unused */
#endif
	PM8XXX_GPIO_INPUT_DISABLE(20),				/* Disable unused */
	PM8XXX_GPIO_INPUT_DISABLE(21),				/* Disable unused */
#if defined(CONFIG_PM_GPIO_JACKDET)
	PM8XXX_GPIO_INPUT(22, PM_GPIO_PULL_UP_30),	/* JACKDET_N */
#else
	PM8XXX_GPIO_INPUT_DISABLE(22),				/* Disable unused */
#endif
#if defined(CONFIG_PM_GPIO_MHL_RST_OUT_DISABLE)
	PM8XXX_GPIO_INPUT_DISABLE(23),				/* Disable unused */
#elif defined(CONFIG_PM_GPIO_MHL_RST_IN_DISABLE)
	PM8XXX_GPIO_INPUT_DISABLE(23),				/* Disable unused */
#else
	PM8XXX_GPIO_OUTPUT(23, 0),					/* MHL_RST_N */
#endif
	PM8XXX_GPIO_OUTPUT_FUNC(24, 0, PM_GPIO_FUNC_2),	/* PWM_OUT */
#if defined(CONFIG_PM_GPIO_MHL_RST_OUT_DISABLE)
	PM8XXX_GPIO_INPUT_DISABLE(25),				/* Disable unused */
#elif defined(CONFIG_PM_GPIO_MHL_RST_IN_DISABLE)
	PM8XXX_GPIO_INPUT_DISABLE(25),				/* Disable unused */
#else
	PM8XXX_GPIO_OUTPUT(25, 0),					/* MHL_REG_12_EN */
#endif
	PM8XXX_GPIO_INPUT(26, PM_GPIO_PULL_NO),		/* SDDET_N */
	PM8XXX_GPIO_OUTPUT_FUNC_VIN(27, 0, PM_GPIO_VIN_L15, PM_GPIO_FUNC_1),/* UIM_RST_CONN */
	PM8XXX_GPIO_INPUT_DISABLE(28),				/* Disable unused */
	PM8XXX_GPIO_INPUT_FUNC(29, PM_GPIO_PULL_NO, PM_GPIO_FUNC_1),		/* UIM_CLK_MSM */
	PM8XXX_GPIO_OUTPUT_FUNC_VIN(30, 0, PM_GPIO_VIN_L15, PM_GPIO_FUNC_1),/* UIM_CLK_CONN */
	PM8XXX_GPIO_INPUT_DISABLE(31),				/* Disable unused */
	PM8XXX_GPIO_INPUT_DISABLE(32),				/* Disable unused */
#if defined(CONFIG_PM_GPIO_TP_PWR_EN)
	PM8XXX_GPIO_OUTPUT(33, 0),					/* TP_PWR_EN */
#else
	PM8XXX_GPIO_INPUT_DISABLE(33),				/* Disable unused */
#endif
	PM8XXX_GPIO_OUTPUT(34, 0),					/* WCD_RST_N */
	PM8XXX_GPIO_OUTPUT(35, 0),					/* SPPD_N */
	PM8XXX_GPIO_OUTPUT(36, 0),					/* DTV_RST_N/[ISDB_RST_N] */
	PM8XXX_GPIO_OUTPUT(37, 0),					/* MCAM_RST_N */
	PM8XXX_GPIO_OUTPUT(38, 0),					/* SCAM_RST_N */
	PM8XXX_GPIO_INPUT_FUNC(39, PM_GPIO_PULL_NO, PM_GPIO_FUNC_1),		/* SSBI_PMIC_FLCK */
	PM8XXX_GPIO_INPUT_DISABLE(40),				/* Disable unused */
	PM8XXX_GPIO_INPUT_DISABLE(41),				/* Disable unused */
#if defined(CONFIG_PM_GPIO_OTG_OVP_CTRL_DISABLE)
	PM8XXX_GPIO_INPUT_DISABLE(42),				/* Disable unused */
#else
	PM8XXX_GPIO_OUTPUT(42, 0),					/* OTG_OVP_CTRL */
#endif
	PM8XXX_GPIO_INPUT_DISABLE(43),				/* Disable unused */
	PM8XXX_GPIO_INPUT_DISABLE(44),				/* Disable unused */
#endif /* CONFIG_BATTERY_SH */
};

/* Initial PM8921 MPP configurations */
static struct pm8xxx_mpp_init pm8921_mpps[] __initdata = {
#ifndef CONFIG_BATTERY_SH
	/* External 5V regulator enable; shared by HDMI and USB_OTG switches. */
	PM8XXX_MPP_INIT(7, D_INPUT, PM8921_MPP_DIG_LEVEL_VPH, DIN_TO_INT),
	PM8XXX_MPP_INIT(PM8XXX_AMUX_MPP_8, A_INPUT, PM8XXX_MPP_AIN_AMUX_CH8,
								DOUT_CTRL_LOW),
#else  /* CONFIG_BATTERY_SH */
	PM8XXX_MPP_INIT(1, D_BI_DIR, PM8921_MPP_DIG_LEVEL_S4,   BI_PULLUP_1KOHM),	/* UIM_DATA_MSM */
	PM8XXX_MPP_INIT(2, D_BI_DIR, PM8921_MPP_DIG_LEVEL_L15,  BI_PULLUP_1KOHM),	/* UIM_DATA_CONN */
	PM8XXX_MPP_INIT(3, A_OUTPUT, PM8XXX_MPP_AOUT_LVL_1V25,  AOUT_CTRL_DISABLE),	/* CAM_TEMP */
	PM8XXX_MPP_INIT(4, A_OUTPUT, PM8XXX_MPP_AOUT_LVL_1V25,  AOUT_CTRL_DISABLE),	/* Unused */
	PM8XXX_MPP_INIT(5, A_OUTPUT, PM8XXX_MPP_AOUT_LVL_1V25,  AOUT_CTRL_ENABLE),	/* VREF_PADS */
	PM8XXX_MPP_INIT(6, A_OUTPUT, PM8XXX_MPP_AOUT_LVL_0V625, AOUT_CTRL_ENABLE),	/* VREF_DAC */
	PM8XXX_MPP_INIT(7, D_OUTPUT, PM8921_MPP_DIG_LEVEL_S4,   DOUT_CTRL_LOW),		/* 5V_BOOST_EN */
	PM8XXX_MPP_INIT(8, A_OUTPUT, PM8XXX_MPP_AOUT_LVL_1V25,  AOUT_CTRL_DISABLE),	/* Unused */
	PM8XXX_MPP_INIT(9, A_OUTPUT, PM8XXX_MPP_AOUT_LVL_1V25,  AOUT_CTRL_DISABLE),	/* Unused */
	PM8XXX_MPP_INIT(10,A_OUTPUT, PM8XXX_MPP_AOUT_LVL_1V25,  AOUT_CTRL_DISABLE),	/* ICHG_EX */
	PM8XXX_MPP_INIT(11,A_OUTPUT, PM8XXX_MPP_AOUT_LVL_1V25,  AOUT_CTRL_DISABLE),	/* USB_ID */
	PM8XXX_MPP_INIT(12,A_OUTPUT, PM8XXX_MPP_AOUT_LVL_1V25,  AOUT_CTRL_DISABLE),	/* Unused */
#endif /* CONFIG_BATTERY_SH */
};

void __init msm8960_pm8921_gpio_mpp_init(void)
{
	int i, rc;
#ifdef CONFIG_BATTERY_SH
#ifdef CONFIG_PM_MPP_11_PU_30KOHM
	sharp_smem_common_type *sharp_smem;
	unsigned long rev;
#endif
#endif /* CONFIG_BATTERY_SH */

	for (i = 0; i < ARRAY_SIZE(pm8921_gpios); i++) {
#ifndef CONFIG_BATTERY_SH
		rc = pm8xxx_gpio_config(pm8921_gpios[i].gpio,
					&pm8921_gpios[i].config);
		if (rc) {
			pr_err("%s: pm8xxx_gpio_config: rc=%d\n", __func__, rc);
			break;
		}
#else  /* CONFIG_BATTERY_SH */
		rc = gpio_request(pm8921_gpios[i].gpio, NULL);
		if (rc)
		{
			pr_err("%s: gpio_request gpio[%d]: rc=%d\n", __func__, pm8921_gpios[i].gpio, rc);
			break;
		}

		rc = pm8xxx_gpio_config(pm8921_gpios[i].gpio, &pm8921_gpios[i].config);
		if (rc) {
			pr_err("%s: pm8xxx_gpio_config gpio[%d]: rc=%d\n", __func__, pm8921_gpios[i].gpio, rc);
			gpio_free(pm8921_gpios[i].gpio);
			break;
		}

		if ((pm8921_gpios[i].config.function == PM_GPIO_FUNC_NORMAL) && (pm8921_gpios[i].config.direction == PM_GPIO_DIR_OUT))
		{
			rc = gpio_direction_output(pm8921_gpios[i].gpio, -1);		// value(-1) = invalid value
			if (rc)
			{
				pr_err("%s: gpio_direction_output gpio[%d]: rc=%d\n", __func__, pm8921_gpios[i].gpio, rc);
				gpio_free(pm8921_gpios[i].gpio);
				break;
			}
		}

		gpio_free(pm8921_gpios[i].gpio);
#endif /* CONFIG_BATTERY_SH */
	}

	for (i = 0; i < ARRAY_SIZE(pm8921_mpps); i++) {
#ifndef CONFIG_BATTERY_SH
		rc = pm8xxx_mpp_config(pm8921_mpps[i].mpp,
					&pm8921_mpps[i].config);
		if (rc) {
			pr_err("%s: pm8xxx_mpp_config: rc=%d\n", __func__, rc);
			break;
		}
#else  /* CONFIG_BATTERY_SH */
		rc = gpio_request(pm8921_mpps[i].mpp, NULL);
		if (rc)
		{
			pr_err("%s: gpio_request mpp[%d]: rc=%d\n", __func__, pm8921_mpps[i].mpp, rc);
			break;
		}

#ifdef CONFIG_PM_MPP_11_PU_30KOHM
		if (pm8921_mpps[i].mpp == PM8921_MPP_PM_TO_SYS(11))
		{
			sharp_smem = sh_smem_get_common_address();
			rev = sharp_smem->sh_hw_revision;

			if (rev == 0x00000005/*SH_HW_REV_ES1*/)
			{
				pm8921_mpps[i].config.type		= PM8XXX_MPP_TYPE_D_BI_DIR;
				pm8921_mpps[i].config.level		= PM8921_MPP_DIG_LEVEL_S4;
				pm8921_mpps[i].config.control	= PM8XXX_MPP_BI_PULLUP_30KOHM;
			}
		}
#endif

		rc = pm8xxx_mpp_config(pm8921_mpps[i].mpp, &pm8921_mpps[i].config);
		if (rc)
		{
			pr_err("%s: pm8xxx_mpp_config mpp[%d]: rc=%d\n", __func__, pm8921_mpps[i].mpp, rc);
			gpio_free(pm8921_mpps[i].mpp);
			break;
		}

		if (pm8921_mpps[i].config.type == PM8XXX_MPP_TYPE_D_OUTPUT)
		{
			rc = gpio_direction_output(pm8921_mpps[i].mpp, -1);		// value(-1) = invalid value
			if (rc)
			{
				pr_err("%s: gpio_direction_output mpp[%d]: rc=%d\n", __func__, pm8921_mpps[i].mpp, rc);
				gpio_free(pm8921_mpps[i].mpp);
				break;
			}
		}

		gpio_free(pm8921_mpps[i].mpp);
#endif /* CONFIG_BATTERY_SH */
	}
}

static struct pm8xxx_adc_amux pm8xxx_adc_channels_data[] = {
	{"vcoin", CHANNEL_VCOIN, CHAN_PATH_SCALING2, AMUX_RSV1,
		ADC_DECIMATION_TYPE2, ADC_SCALE_DEFAULT},
#ifdef CONFIG_BATTERY_SH
	{"vbat", CHANNEL_VBAT, CHAN_PATH_SCALING2, AMUX_RSV1,
		ADC_DECIMATION_TYPE2, ADC_SCALE_VBATT},
#else  /* CONFIG_BATTERY_SH */
	{"vbat", CHANNEL_VBAT, CHAN_PATH_SCALING2, AMUX_RSV1,
		ADC_DECIMATION_TYPE2, ADC_SCALE_DEFAULT},
#endif /* CONFIG_BATTERY_SH */
	{"dcin", CHANNEL_DCIN, CHAN_PATH_SCALING4, AMUX_RSV1,
		ADC_DECIMATION_TYPE2, ADC_SCALE_DEFAULT},
#ifdef CONFIG_BATTERY_SH
	{"ichg", /*CHANNEL_ICHG*/ADC_MPP_1_CHG_AMUX6, CHAN_PATH_SCALING1, AMUX_RSV1,
		ADC_DECIMATION_TYPE2, ADC_SCALE_DEFAULT},
#else  /* CONFIG_BATTERY_SH */
	{"ichg", CHANNEL_ICHG, CHAN_PATH_SCALING1, AMUX_RSV1,
		ADC_DECIMATION_TYPE2, ADC_SCALE_DEFAULT},
#endif /* CONFIG_BATTERY_SH */
	{"vph_pwr", CHANNEL_VPH_PWR, CHAN_PATH_SCALING2, AMUX_RSV1,
		ADC_DECIMATION_TYPE2, ADC_SCALE_DEFAULT},
	{"ibat", CHANNEL_IBAT, CHAN_PATH_SCALING1, AMUX_RSV1,
		ADC_DECIMATION_TYPE2, ADC_SCALE_DEFAULT},
#ifdef CONFIG_BATTERY_SH
	{"cam_temp", /*CHANNEL_CAM_TEMP*/ADC_MPP_1_AMUX6, CHAN_PATH_SCALING1, AMUX_RSV1,
		ADC_DECIMATION_TYPE2, ADC_SCALE_CAM_TEMP},
#ifdef CONFIG_PM_MPP_12_USE_BACKLIGHT_THERM
	{"backlight_temp", /*CHANNEL_BACKLIGHT_TEMP*/ADC_MPP_1_BACKLIGHT_AMUX6, CHAN_PATH_SCALING1, AMUX_RSV1,
		ADC_DECIMATION_TYPE2, ADC_SCALE_BACKLIGHT_TEMP},
#endif /* CONFIG_PM_MPP_12_USE_BACKLIGHT_THERM */
#endif /* CONFIG_BATTERY_SH */
	{"batt_therm", CHANNEL_BATT_THERM, CHAN_PATH_SCALING1, AMUX_RSV2,
		ADC_DECIMATION_TYPE2, ADC_SCALE_BATT_THERM},
	{"batt_id", CHANNEL_BATT_ID, CHAN_PATH_SCALING1, AMUX_RSV1,
		ADC_DECIMATION_TYPE2, ADC_SCALE_DEFAULT},
	{"usbin", CHANNEL_USBIN, CHAN_PATH_SCALING3, AMUX_RSV1,
		ADC_DECIMATION_TYPE2, ADC_SCALE_DEFAULT},
	{"pmic_therm", CHANNEL_DIE_TEMP, CHAN_PATH_SCALING1, AMUX_RSV1,
		ADC_DECIMATION_TYPE2, ADC_SCALE_PMIC_THERM},
	{"625mv", CHANNEL_625MV, CHAN_PATH_SCALING1, AMUX_RSV1,
		ADC_DECIMATION_TYPE2, ADC_SCALE_DEFAULT},
	{"125v", CHANNEL_125V, CHAN_PATH_SCALING1, AMUX_RSV1,
		ADC_DECIMATION_TYPE2, ADC_SCALE_DEFAULT},
	{"chg_temp", CHANNEL_CHG_TEMP, CHAN_PATH_SCALING1, AMUX_RSV1,
		ADC_DECIMATION_TYPE2, ADC_SCALE_DEFAULT},
	{"pa_therm1", ADC_MPP_1_AMUX8, CHAN_PATH_SCALING1, AMUX_RSV1,
		ADC_DECIMATION_TYPE2, ADC_SCALE_PA_THERM},
	{"xo_therm", CHANNEL_MUXOFF, CHAN_PATH_SCALING1, AMUX_RSV0,
		ADC_DECIMATION_TYPE2, ADC_SCALE_XOTHERM},
	{"pa_therm0", ADC_MPP_1_AMUX3, CHAN_PATH_SCALING1, AMUX_RSV1,
		ADC_DECIMATION_TYPE2, ADC_SCALE_PA_THERM},
#ifdef CONFIG_BATTERY_SH
	{"usb_id", ADC_MPP_1_USB_AMUX6, CHAN_PATH_SCALING1, AMUX_RSV1,
		ADC_DECIMATION_TYPE2, ADC_SCALE_DEFAULT},
#endif /* CONFIG_BATTERY_SH */
};

static struct pm8xxx_adc_properties pm8xxx_adc_data = {
	.adc_vdd_reference	= 1800, /* milli-voltage for this adc */
	.bitresolution		= 15,
	.bipolar                = 0,
};

static struct pm8xxx_adc_platform_data pm8xxx_adc_pdata = {
	.adc_channel            = pm8xxx_adc_channels_data,
	.adc_num_board_channel  = ARRAY_SIZE(pm8xxx_adc_channels_data),
	.adc_prop               = &pm8xxx_adc_data,
	.adc_mpp_base		= PM8921_MPP_PM_TO_SYS(1),
};

static struct pm8xxx_irq_platform_data pm8xxx_irq_pdata __devinitdata = {
	.irq_base		= PM8921_IRQ_BASE,
	.devirq			= MSM_GPIO_TO_INT(104),
	.irq_trigger_flag	= IRQF_TRIGGER_LOW,
};

static struct pm8xxx_gpio_platform_data pm8xxx_gpio_pdata __devinitdata = {
	.gpio_base	= PM8921_GPIO_PM_TO_SYS(1),
};

static struct pm8xxx_mpp_platform_data pm8xxx_mpp_pdata __devinitdata = {
	.mpp_base	= PM8921_MPP_PM_TO_SYS(1),
};

static struct pm8xxx_rtc_platform_data pm8xxx_rtc_pdata __devinitdata = {
	.rtc_write_enable       = false,
	.rtc_alarm_powerup	= false,
};

static struct pm8xxx_pwrkey_platform_data pm8xxx_pwrkey_pdata = {
	.pull_up		= 1,
	.kpd_trigger_delay_us	= 15625,
	.wakeup			= 1,
};

/* Rotate lock key is not available so use F1 */
#define KEY_ROTATE_LOCK KEY_F1

static const unsigned int keymap_liquid[] = {
	KEY(0, 0, KEY_VOLUMEUP),
	KEY(0, 1, KEY_VOLUMEDOWN),
	KEY(1, 3, KEY_ROTATE_LOCK),
	KEY(1, 4, KEY_HOME),
};

static struct matrix_keymap_data keymap_data_liquid = {
	.keymap_size    = ARRAY_SIZE(keymap_liquid),
	.keymap         = keymap_liquid,
};

static struct pm8xxx_keypad_platform_data keypad_data_liquid = {
	.input_name             = "keypad_8960_liquid",
	.input_phys_device      = "keypad_8960/input0",
	.num_rows               = 2,
	.num_cols               = 5,
	.rows_gpio_start	= PM8921_GPIO_PM_TO_SYS(9),
	.cols_gpio_start	= PM8921_GPIO_PM_TO_SYS(1),
	.debounce_ms            = 15,
	.scan_delay_ms          = 32,
	.row_hold_ns            = 91500,
	.wakeup                 = 1,
	.keymap_data            = &keymap_data_liquid,
};

/* KEY Drv key map change start */
#ifdef CONFIG_KEYBOARD_SCKEY

static const unsigned int keymap[] = {
	KEY(0, 0, KEY_ROW1_COL1),
	KEY(0, 1, KEY_ROW1_COL2),
	KEY(0, 2, KEY_ROW1_COL3),
	KEY(0, 3, KEY_ROW1_COL4),
	KEY(0, 4, KEY_ROW1_COL5),
	KEY(0, 5, KEY_ROW1_COL6),
	KEY(0, 6, KEY_ROW1_COL7),
	KEY(0, 7, KEY_ROW1_COL8),

	KEY(1, 0, KEY_ROW2_COL1),
	KEY(1, 1, KEY_ROW2_COL2),
	KEY(1, 2, KEY_ROW2_COL3),
	KEY(1, 3, KEY_ROW2_COL4),
	KEY(1, 4, KEY_ROW2_COL5),
	KEY(1, 5, KEY_ROW2_COL6),
	KEY(1, 6, KEY_ROW2_COL7),
	KEY(1, 7, KEY_ROW2_COL8),

	KEY(2, 0, KEY_ROW3_COL1),
	KEY(2, 1, KEY_ROW3_COL2),
	KEY(2, 2, KEY_ROW3_COL3),
	KEY(2, 3, KEY_ROW3_COL4),
	KEY(2, 4, KEY_ROW3_COL5),
	KEY(2, 5, KEY_ROW3_COL6),
	KEY(2, 6, KEY_ROW3_COL7),
	KEY(2, 7, KEY_ROW3_COL8),

	KEY(3, 0, KEY_ROW4_COL1),
	KEY(3, 1, KEY_ROW4_COL2),
	KEY(3, 2, KEY_ROW4_COL3),
	KEY(3, 3, KEY_ROW4_COL4),
	KEY(3, 4, KEY_ROW4_COL5),
	KEY(3, 5, KEY_ROW4_COL6),
	KEY(3, 6, KEY_ROW4_COL7),
	KEY(3, 7, KEY_ROW4_COL8),

	KEY(4, 0, KEY_ROW5_COL1),
	KEY(4, 1, KEY_ROW5_COL2),
	KEY(4, 2, KEY_ROW5_COL3),
	KEY(4, 3, KEY_ROW5_COL4),
	KEY(4, 4, KEY_ROW5_COL5),
	KEY(4, 5, KEY_ROW5_COL6),
	KEY(4, 6, KEY_ROW5_COL7),
	KEY(4, 7, KEY_ROW5_COL8),

	KEY(5, 0, KEY_ROW6_COL1),
	KEY(5, 1, KEY_ROW6_COL2),
	KEY(5, 2, KEY_ROW6_COL3),
	KEY(5, 3, KEY_ROW6_COL4),
	KEY(5, 4, KEY_ROW6_COL5),
	KEY(5, 5, KEY_ROW6_COL6),
	KEY(5, 6, KEY_ROW6_COL7),
	KEY(5, 7, KEY_ROW6_COL8),

	KEY(6, 0, KEY_ROW7_COL1),
	KEY(6, 1, KEY_ROW7_COL2),
	KEY(6, 2, KEY_ROW7_COL3),
	KEY(6, 3, KEY_ROW7_COL4),
	KEY(6, 4, KEY_ROW7_COL5),
	KEY(6, 5, KEY_ROW7_COL6),
	KEY(6, 6, KEY_ROW7_COL7),
	KEY(6, 7, KEY_ROW7_COL8),

	KEY(7, 0, KEY_ROW8_COL1),
	KEY(7, 1, KEY_ROW8_COL2),
	KEY(7, 2, KEY_ROW8_COL3),
	KEY(7, 3, KEY_ROW8_COL4),
	KEY(7, 4, KEY_ROW8_COL5),
	KEY(7, 5, KEY_ROW8_COL6),
	KEY(7, 6, KEY_ROW8_COL7),
	KEY(7, 7, KEY_ROW8_COL8),

	KEY(8, 0, KEY_ROW9_COL1),
	KEY(8, 1, KEY_ROW9_COL2),
	KEY(8, 2, KEY_ROW9_COL3),
	KEY(8, 3, KEY_ROW9_COL4),
	KEY(8, 4, KEY_ROW9_COL5),
	KEY(8, 5, KEY_ROW9_COL6),
	KEY(8, 6, KEY_ROW9_COL7),
	KEY(8, 7, KEY_ROW9_COL8),

	KEY(9, 0, KEY_ROW10_COL1),
	KEY(9, 1, KEY_ROW10_COL2),
	KEY(9, 2, KEY_ROW10_COL3),
	KEY(9, 3, KEY_ROW10_COL4),
	KEY(9, 4, KEY_ROW10_COL5),
	KEY(9, 5, KEY_ROW10_COL6),
	KEY(9, 6, KEY_ROW10_COL7),
	KEY(9, 7, KEY_ROW10_COL8),

	KEY(10, 0, KEY_ROW11_COL1),
	KEY(10, 1, KEY_ROW11_COL2),
	KEY(10, 2, KEY_ROW11_COL3),
	KEY(10, 3, KEY_ROW11_COL4),
	KEY(10, 4, KEY_ROW11_COL5),
	KEY(10, 5, KEY_ROW11_COL6),
	KEY(10, 6, KEY_ROW11_COL7),
	KEY(10, 7, KEY_ROW11_COL8),

	KEY(11, 0, KEY_ROW12_COL1),
	KEY(11, 1, KEY_ROW12_COL2),
	KEY(11, 2, KEY_ROW12_COL3),
	KEY(11, 3, KEY_ROW12_COL4),
	KEY(11, 4, KEY_ROW12_COL5),
	KEY(11, 5, KEY_ROW12_COL6),
	KEY(11, 6, KEY_ROW12_COL7),
	KEY(11, 7, KEY_ROW12_COL8),
};

#else	/* #ifdef CONFIG_KEYBOARD_SCKEY */

static const unsigned int keymap[] = {
	KEY(0, 0, KEY_VOLUMEUP),
	KEY(0, 1, KEY_VOLUMEDOWN),
	KEY(0, 2, KEY_CAMERA_SNAPSHOT),
	KEY(0, 3, KEY_CAMERA_FOCUS),
};

#endif	/* CONFIG_KEYBOARD_SCKEY */
/* KEY Drv key map change end */

#ifdef CONFIG_KEYBOARD_SCKEY
static struct matrix_keymap_data keymap_data = {
	.keymap_size    = ARRAY_SIZE(keymap),
	.keymap         = keymap,
};

static struct pm8xxx_keypad_platform_data keypad_data = {
	.input_name             = SH_INPUT_NAME,
	.input_phys_device      = SH_INPUT_PHYS_DEVICE,
	.num_rows               = SH_NUM_ROWS,
	.num_cols               = SH_NUM_COLS,
	.rows_gpio_start		= SH_ROWS_GPIO_START,
	.cols_gpio_start		= SH_COLS_GPIO_START,
	.init_num_rows			= SH_INIT_NUM_ROWS,
	.init_num_cols			= SH_INIT_NUM_COLS,
	.debounce_ms            = SH_DEBOUNCE_MS,
	.scan_delay_ms          = SH_SCAN_DELAY_MS,
	.row_hold_ns            = SH_HOLD_NS,
	.wakeup                 = 0,
	.keymap_data            = &keymap_data,
};

#else	/* CONFIG_KEYBOARD_SCKEY */

static struct matrix_keymap_data keymap_data = {
	.keymap_size    = ARRAY_SIZE(keymap),
	.keymap         = keymap,
};

static struct pm8xxx_keypad_platform_data keypad_data = {
	.input_name             = "keypad_8960",
	.input_phys_device      = "keypad_8960/input0",
	.num_rows               = 1,
	.num_cols               = 5,
	.rows_gpio_start	= PM8921_GPIO_PM_TO_SYS(9),
	.cols_gpio_start	= PM8921_GPIO_PM_TO_SYS(1),
	.debounce_ms            = 15,
	.scan_delay_ms          = 32,
	.row_hold_ns            = 91500,
	.wakeup                 = 1,
	.keymap_data            = &keymap_data,
};

#endif	/* CONFIG_KEYBOARD_SCKEY */

static const unsigned int keymap_sim[] = {
	KEY(0, 0, KEY_7),
	KEY(0, 1, KEY_DOWN),
	KEY(0, 2, KEY_UP),
	KEY(0, 3, KEY_RIGHT),
	KEY(0, 4, KEY_ENTER),
	KEY(0, 5, KEY_L),
	KEY(0, 6, KEY_BACK),
	KEY(0, 7, KEY_M),

	KEY(1, 0, KEY_LEFT),
	KEY(1, 1, KEY_SEND),
	KEY(1, 2, KEY_1),
	KEY(1, 3, KEY_4),
	KEY(1, 4, KEY_CLEAR),
	KEY(1, 5, KEY_MSDOS),
	KEY(1, 6, KEY_SPACE),
	KEY(1, 7, KEY_COMMA),

	KEY(2, 0, KEY_6),
	KEY(2, 1, KEY_5),
	KEY(2, 2, KEY_8),
	KEY(2, 3, KEY_3),
	KEY(2, 4, KEY_NUMERIC_STAR),
	KEY(2, 5, KEY_UP),
	KEY(2, 6, KEY_DOWN),
	KEY(2, 7, KEY_LEFTSHIFT),

	KEY(3, 0, KEY_9),
	KEY(3, 1, KEY_NUMERIC_POUND),
	KEY(3, 2, KEY_0),
	KEY(3, 3, KEY_2),
	KEY(3, 4, KEY_SLEEP),
	KEY(3, 5, KEY_F1),
	KEY(3, 6, KEY_F2),
	KEY(3, 7, KEY_F3),

	KEY(4, 0, KEY_BACK),
	KEY(4, 1, KEY_HOME),
	KEY(4, 2, KEY_MENU),
	KEY(4, 3, KEY_VOLUMEUP),
	KEY(4, 4, KEY_VOLUMEDOWN),
	KEY(4, 5, KEY_F4),
	KEY(4, 6, KEY_F5),
	KEY(4, 7, KEY_F6),

	KEY(5, 0, KEY_R),
	KEY(5, 1, KEY_T),
	KEY(5, 2, KEY_Y),
	KEY(5, 3, KEY_LEFTALT),
	KEY(5, 4, KEY_KPENTER),
	KEY(5, 5, KEY_Q),
	KEY(5, 6, KEY_W),
	KEY(5, 7, KEY_E),

	KEY(6, 0, KEY_F),
	KEY(6, 1, KEY_G),
	KEY(6, 2, KEY_H),
	KEY(6, 3, KEY_CAPSLOCK),
	KEY(6, 4, KEY_PAGEUP),
	KEY(6, 5, KEY_A),
	KEY(6, 6, KEY_S),
	KEY(6, 7, KEY_D),

	KEY(7, 0, KEY_V),
	KEY(7, 1, KEY_B),
	KEY(7, 2, KEY_N),
	KEY(7, 3, KEY_MENU),
	KEY(7, 4, KEY_PAGEDOWN),
	KEY(7, 5, KEY_Z),
	KEY(7, 6, KEY_X),
	KEY(7, 7, KEY_C),

	KEY(8, 0, KEY_P),
	KEY(8, 1, KEY_J),
	KEY(8, 2, KEY_K),
	KEY(8, 3, KEY_INSERT),
	KEY(8, 4, KEY_LINEFEED),
	KEY(8, 5, KEY_U),
	KEY(8, 6, KEY_I),
	KEY(8, 7, KEY_O),

	KEY(9, 0, KEY_4),
	KEY(9, 1, KEY_5),
	KEY(9, 2, KEY_6),
	KEY(9, 3, KEY_7),
	KEY(9, 4, KEY_8),
	KEY(9, 5, KEY_1),
	KEY(9, 6, KEY_2),
	KEY(9, 7, KEY_3),

	KEY(10, 0, KEY_F7),
	KEY(10, 1, KEY_F8),
	KEY(10, 2, KEY_F9),
	KEY(10, 3, KEY_F10),
	KEY(10, 4, KEY_FN),
	KEY(10, 5, KEY_9),
	KEY(10, 6, KEY_0),
	KEY(10, 7, KEY_DOT),

	KEY(11, 0, KEY_LEFTCTRL),
	KEY(11, 1, KEY_F11),
	KEY(11, 2, KEY_ENTER),
	KEY(11, 3, KEY_SEARCH),
	KEY(11, 4, KEY_DELETE),
	KEY(11, 5, KEY_RIGHT),
	KEY(11, 6, KEY_LEFT),
	KEY(11, 7, KEY_RIGHTSHIFT),
	KEY(0, 0, KEY_VOLUMEUP),
	KEY(0, 1, KEY_VOLUMEDOWN),
	KEY(0, 2, KEY_CAMERA_SNAPSHOT),
	KEY(0, 3, KEY_CAMERA_FOCUS),
};

static struct matrix_keymap_data keymap_data_sim = {
	.keymap_size    = ARRAY_SIZE(keymap_sim),
	.keymap         = keymap_sim,
};

static struct pm8xxx_keypad_platform_data keypad_data_sim = {
	.input_name             = "keypad_8960",
	.input_phys_device      = "keypad_8960/input0",
	.num_rows               = 12,
	.num_cols               = 8,
	.rows_gpio_start	= PM8921_GPIO_PM_TO_SYS(9),
	.cols_gpio_start	= PM8921_GPIO_PM_TO_SYS(1),
	.debounce_ms            = 15,
	.scan_delay_ms          = 32,
	.row_hold_ns            = 91500,
	.wakeup                 = 1,
	.keymap_data            = &keymap_data_sim,
};

static int pm8921_therm_mitigation[] = {
	1100,
	700,
	600,
	325,
};

#ifdef CONFIG_BATTERY_SH
#if defined(CONFIG_PM_HIGH_CAPA_BATTERY)
	#define MAX_VOLTAGE_MV		4320
#else
	#define MAX_VOLTAGE_MV		4200
#endif
#else  /* CONFIG_BATTERY_SH */
#define MAX_VOLTAGE_MV		4200
#endif /* CONFIG_BATTERY_SH */
#define CHG_TERM_MA		100
static struct pm8921_charger_platform_data pm8921_chg_pdata __devinitdata = {
	.safety_time		= 180,
	.update_time		= 60000,
	.max_voltage		= MAX_VOLTAGE_MV,
	.min_voltage		= 3200,
	.uvd_thresh_voltage	= 4050,
	.resume_voltage_delta	= 60,
	.resume_charge_percent	= 99,
	.term_current		= CHG_TERM_MA,
	.cool_temp		= 10,
	.warm_temp		= 40,
	.temp_check_period	= 1,
	.max_bat_chg_current	= 1100,
	.cool_bat_chg_current	= 350,
	.warm_bat_chg_current	= 350,
	.cool_bat_voltage	= 4100,
	.warm_bat_voltage	= 4100,
	.thermal_mitigation	= pm8921_therm_mitigation,
	.thermal_levels		= ARRAY_SIZE(pm8921_therm_mitigation),
	.rconn_mohm		= 18,
};

static struct pm8xxx_misc_platform_data pm8xxx_misc_pdata = {
	.priority		= 0,
};

static struct pm8921_bms_platform_data pm8921_bms_pdata __devinitdata = {
	.battery_type			= BATT_UNKNOWN,
	.r_sense			= 10,
	.v_cutoff			= 3400,
	.max_voltage_uv			= MAX_VOLTAGE_MV * 1000,
	.rconn_mohm			= 18,
	.shutdown_soc_valid_limit	= 20,
	.adjust_soc_low_threshold	= 25,
	.chg_term_ua			= CHG_TERM_MA * 1000,
};

#define	PM8921_LC_LED_MAX_CURRENT	4	/* I = 4mA */
#define	PM8921_LC_LED_LOW_CURRENT	1	/* I = 1mA */
#define PM8XXX_LED_PWM_PERIOD		1000
#define PM8XXX_LED_PWM_DUTY_MS		20
/**
 * PM8XXX_PWM_CHANNEL_NONE shall be used when LED shall not be
 * driven using PWM feature.
 */
#define PM8XXX_PWM_CHANNEL_NONE		-1

static struct led_info pm8921_led_info_liquid[] = {
	{
		.name		= "led:red",
		.flags		= PM8XXX_ID_LED_0,
		.default_trigger	= "battery-charging",
	},
	{
		.name		= "led:green",
		.flags		= PM8XXX_ID_LED_0,
		.default_trigger	= "battery-full",
	},
	{
		.name		= "led:blue",
		.flags		= PM8XXX_ID_LED_2,
		.default_trigger	= "notification",
	},
};

static struct pm8xxx_led_config pm8921_led_configs_liquid[] = {
	[0] = {
		.id = PM8XXX_ID_LED_0,
		.mode = PM8XXX_LED_MODE_MANUAL,
		.max_current = PM8921_LC_LED_MAX_CURRENT,
	},
	[1] = {
		.id = PM8XXX_ID_LED_1,
		.mode = PM8XXX_LED_MODE_MANUAL,
		.max_current = PM8921_LC_LED_LOW_CURRENT,
	},
	[2] = {
		.id = PM8XXX_ID_LED_2,
		.mode = PM8XXX_LED_MODE_MANUAL,
		.max_current = PM8921_LC_LED_MAX_CURRENT,
	},
};

static struct led_platform_data pm8xxx_leds_core_liquid = {
	.num_leds = ARRAY_SIZE(pm8921_led_info_liquid),
	.leds = pm8921_led_info_liquid,
};

static struct pm8xxx_led_platform_data pm8xxx_leds_pdata_liquid = {
	.led_core = &pm8xxx_leds_core_liquid,
	.configs = pm8921_led_configs_liquid,
	.num_configs = ARRAY_SIZE(pm8921_led_configs_liquid),
};

static struct led_info pm8921_led_info[] = {
#ifndef CONFIG_BATTERY_SH
	[0] = {
		.name			= "led:battery_charging",
		.default_trigger	= "battery-charging",
	},
	[1] = {
		.name			= "led:battery_full",
		.default_trigger	= "battery-full",
	},
#else  /* CONFIG_BATTERY_SH */
	[0] = {
		.name			= "led:drv0",
		.default_trigger	= "cam-red-led",
	},
	[1] = {
		.name			= "led:drv1",
		.default_trigger	= "cam-led1",
	},
	[2] = {
		.name			= "led:drv2",
		.default_trigger	= "cam-led2",
	},
#endif /* CONFIG_BATTERY_SH */
};

static struct led_platform_data pm8921_led_core_pdata = {
	.num_leds = ARRAY_SIZE(pm8921_led_info),
	.leds = pm8921_led_info,
};

#ifdef CONFIG_SH_AUDIO_DRIVER /*02-093*/
/* vibrator parameter initialize     */
#ifdef CONFIG_PMIC8XXX_VIBRATOR
static struct pm8xxx_vibrator_platform_data pm8xxx_vibrator_pdata = {
	.initial_vibrate_ms = 0,
	.max_timeout_ms = 15000,
#if (CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER == 303) /*->*//*02-152*/
	.level_mV = 3100,
#elif (CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER == 202 || CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER == 203)
	.level_mV = 3100,
#else
	.level_mV = 3000,
#endif /*<-*//*02-152*/
};
#endif /* CONFIG_PMIC8XXX_VIBRATOR */
#endif /* CONFIG_SH_AUDIO_DRIVER *//*02-093*/

static int pm8921_led0_pwm_duty_pcts[56] = {
		1, 4, 8, 12, 16, 20, 24, 28, 32, 36,
		40, 44, 46, 52, 56, 60, 64, 68, 72, 76,
		80, 84, 88, 92, 96, 100, 100, 100, 98, 95,
		92, 88, 84, 82, 78, 74, 70, 66, 62, 58,
		58, 54, 50, 48, 42, 38, 34, 30, 26, 22,
		14, 10, 6, 4, 1
};

/*
 * Note: There is a bug in LPG module that results in incorrect
 * behavior of pattern when LUT index 0 is used. So effectively
 * there are 63 usable LUT entries.
 */
static struct pm8xxx_pwm_duty_cycles pm8921_led0_pwm_duty_cycles = {
	.duty_pcts = (int *)&pm8921_led0_pwm_duty_pcts,
	.num_duty_pcts = ARRAY_SIZE(pm8921_led0_pwm_duty_pcts),
	.duty_ms = PM8XXX_LED_PWM_DUTY_MS,
	.start_idx = 1,
};

static struct pm8xxx_led_config pm8921_led_configs[] = {
#ifndef CONFIG_BATTERY_SH
	[0] = {
		.id = PM8XXX_ID_LED_0,
		.mode = PM8XXX_LED_MODE_PWM2,
		.max_current = PM8921_LC_LED_MAX_CURRENT,
		.pwm_channel = 5,
		.pwm_period_us = PM8XXX_LED_PWM_PERIOD,
		.pwm_duty_cycles = &pm8921_led0_pwm_duty_cycles,
	},
	[1] = {
		.id = PM8XXX_ID_LED_1,
		.mode = PM8XXX_LED_MODE_PWM1,
		.max_current = PM8921_LC_LED_MAX_CURRENT,
		.pwm_channel = 4,
		.pwm_period_us = PM8XXX_LED_PWM_PERIOD,
	},
#else  /* CONFIG_BATTERY_SH */
	[0] = {
		.id = PM8XXX_ID_LED_KB_LIGHT,
		.mode = PM8XXX_LED_MODE_MANUAL,
		.max_current = 20,
		.pwm_channel = 5,
		.pwm_period_us = PM8XXX_LED_PWM_PERIOD,
		.pwm_duty_cycles = &pm8921_led0_pwm_duty_cycles,
	},
	[1] = {
#if defined(CONFIG_PM_LED_MAX_CURRENT_CHANGE)
		.id = PM8XXX_ID_LED_1,
		.mode = PM8XXX_LED_MODE_MANUAL,
		.max_current = 40,
		.pwm_channel = 4,
		.pwm_period_us = PM8XXX_LED_PWM_PERIOD,
#else
		.id = PM8XXX_ID_LED_1,
		.mode = PM8XXX_LED_MODE_MANUAL,
		.max_current = 20,
		.pwm_channel = 4,
		.pwm_period_us = PM8XXX_LED_PWM_PERIOD,
#endif
	},
	[2] = {
		.id = PM8XXX_ID_LED_2,
		.mode = PM8XXX_LED_MODE_MANUAL,
		.max_current = 20,
		.pwm_channel = 4,
		.pwm_period_us = PM8XXX_LED_PWM_PERIOD,
	},
#endif /* CONFIG_BATTERY_SH */
};

static struct pm8xxx_led_platform_data pm8xxx_leds_pdata = {
		.led_core = &pm8921_led_core_pdata,
		.configs = pm8921_led_configs,
		.num_configs = ARRAY_SIZE(pm8921_led_configs),
};

static struct pm8xxx_ccadc_platform_data pm8xxx_ccadc_pdata = {
	.r_sense		= 10,
	.calib_delay_ms		= 600000,
};

/**
 * PM8XXX_PWM_DTEST_CHANNEL_NONE shall be used when no LPG
 * channel should be in DTEST mode.
 */

#define PM8XXX_PWM_DTEST_CHANNEL_NONE   (-1)

static struct pm8xxx_pwm_platform_data pm8xxx_pwm_pdata = {
	.dtest_channel	= PM8XXX_PWM_DTEST_CHANNEL_NONE,
};

static struct pm8921_platform_data pm8921_platform_data __devinitdata = {
	.irq_pdata		= &pm8xxx_irq_pdata,
	.gpio_pdata		= &pm8xxx_gpio_pdata,
	.mpp_pdata		= &pm8xxx_mpp_pdata,
	.rtc_pdata              = &pm8xxx_rtc_pdata,
	.pwrkey_pdata		= &pm8xxx_pwrkey_pdata,
	.keypad_pdata		= &keypad_data,
	.misc_pdata		= &pm8xxx_misc_pdata,
	.regulator_pdatas	= msm_pm8921_regulator_pdata,
	.charger_pdata		= &pm8921_chg_pdata,
	.bms_pdata		= &pm8921_bms_pdata,
	.adc_pdata		= &pm8xxx_adc_pdata,
	.leds_pdata		= &pm8xxx_leds_pdata,
	.ccadc_pdata		= &pm8xxx_ccadc_pdata,
#ifdef CONFIG_SH_AUDIO_DRIVER /*02-093*/
/* vibrator parameter initialize     */
#ifdef CONFIG_PMIC8XXX_VIBRATOR
	.vibrator_pdata	= &pm8xxx_vibrator_pdata,
#endif /* CONFIG_PMIC8XXX_VIBRATOR */
#endif /* CONFIG_SH_AUDIO_DRIVER *//*02-093*/
	.pwm_pdata		= &pm8xxx_pwm_pdata,
};

static struct msm_ssbi_platform_data msm8960_ssbi_pm8921_pdata __devinitdata = {
	.controller_type = MSM_SBI_CTRL_PMIC_ARBITER,
	.slave	= {
		.name			= "pm8921-core",
		.platform_data		= &pm8921_platform_data,
	},
};

void __init msm8960_init_pmic(void)
{
	pmic_reset_irq = PM8921_IRQ_BASE + PM8921_RESOUT_IRQ;
	msm8960_device_ssbi_pmic.dev.platform_data =
				&msm8960_ssbi_pm8921_pdata;
	pm8921_platform_data.num_regulators = msm_pm8921_regulator_pdata_len;

	/* Simulator supports a QWERTY keypad */
	if (machine_is_msm8960_sim())
		pm8921_platform_data.keypad_pdata = &keypad_data_sim;

	if (machine_is_msm8960_liquid()) {
		pm8921_platform_data.keypad_pdata = &keypad_data_liquid;
		pm8921_platform_data.leds_pdata = &pm8xxx_leds_pdata_liquid;
		pm8921_platform_data.bms_pdata->battery_type = BATT_DESAY;
	} else if (machine_is_msm8960_mtp()) {
		pm8921_platform_data.bms_pdata->battery_type = BATT_PALLADIUM;
	} else if (machine_is_msm8960_cdp()) {
		pm8921_chg_pdata.has_dc_supply = true;
	}

	if (machine_is_msm8960_fluid())
		pm8921_bms_pdata.rconn_mohm = 20;
}
