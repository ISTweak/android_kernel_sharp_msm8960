/* drivers/sharp/shdisp/data/shdisp_bl69y6_data_dl15.h  (Display Driver)
 *
 * Copyright (C) 2011-2012 SHARP CORPORATION
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/* ------------------------------------------------------------------------- */
/* SHARP DISPLAY DRIVER FOR KERNEL MODE                                      */
/* ------------------------------------------------------------------------- */

#ifndef SHDISP_BL69Y6_DATA_DL15_H
#define SHDISP_BL69Y6_DATA_DL15_H

/* ------------------------------------------------------------------------- */
/* INCLUDE FILES                                                             */
/* ------------------------------------------------------------------------- */

#include "../shdisp_bl69y6.h"

/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */

#define SHDISP_BKL_FIX_TBL_NUM          23
#define SHDISP_BKL_AUTO_TBL_NUM         16
#define SHDISP_TRI_LED_COLOR_TBL_NUM    8
#define NUM_SHDISP_BKL_TBL_MODE         (SHDISP_BKL_TBL_MODE_CHARGE + 1)

#define SHDISP_INT_ENABLE_GFAC          0x002C0308
#define SHDISP_LUX_CHANGE_LEVEL1        0x0C
#define SHDISP_LUX_CHANGE_LEVEL2        0x01

#define CABC_LUX_LEVEL_LUT0_1           0x01
#define CABC_LUX_LEVEL_LUT1_2           0x02
#define CABC_LUX_LEVEL_LUT2_3           0x03
#define CABC_LUX_LEVEL_LUT3_4           0x04
#define CABC_LUX_LEVEL_LUT4_5           0x05

/* ------------------------------------------------------------------------- */
/* MACROS(Register Value)                                                    */
/* ------------------------------------------------------------------------- */

#define BDIC_REG_SYSTEM4_VAL            0x28
#define BDIC_REG_SLOPE_VAL              0xCB
#define BDIC_REG_DCDC1_VLIM_VAL         0xF0
#define BDIC_REG_DCDC_SYS_VAL           0x00
#define BDIC_REG_DCDC2_VO_VAL           0xE8
#define BDIC_REG_SYSTEM2_BKL            0x01
#define BDIC_REG_ALS_ADJ0_L_DEFAULT     0x5C
#define BDIC_REG_ALS_ADJ0_H_DEFAULT     0x3F
#define BDIC_REG_ALS_ADJ1_L_DEFAULT     0xAA
#define BDIC_REG_ALS_ADJ1_H_DEFAULT     0x4C
#define BDIC_REG_ALS_SHIFT_DEFAULT      0x03
#define BDIC_REG_CLEAR_OFFSET_DEFAULT   0x00
#define BDIC_REG_IR_OFFSET_DEFAULT      0x00

/* ------------------------------------------------------------------------- */
/* VARIABLES                                                                 */
/* ------------------------------------------------------------------------- */

static const unsigned char shdisp_main_bkl_tbl[SHDISP_BKL_FIX_TBL_NUM][NUM_SHDISP_BKL_TBL_MODE] = {
    { 0x00,   0x00,   0x00,   0x00 },
    { 0x04,   0x04,   0x04,   0x04 },
    { 0x0B,   0x0A,   0x0A,   0x0B },
    { 0x0C,   0x0B,   0x0B,   0x0C },
    { 0x0D,   0x0C,   0x0C,   0x0D },
    { 0x0F,   0x0D,   0x0D,   0x0F },
    { 0x11,   0x0E,   0x0E,   0x11 },
    { 0x13,   0x0F,   0x0F,   0x13 },
    { 0x16,   0x0F,   0x0F,   0x16 },
    { 0x19,   0x10,   0x10,   0x19 },
    { 0x1C,   0x11,   0x11,   0x1C },
    { 0x1F,   0x13,   0x13,   0x1F },
    { 0x24,   0x15,   0x15,   0x24 },
    { 0x29,   0x17,   0x15,   0x29 },
    { 0x2F,   0x1A,   0x15,   0x2F },
    { 0x36,   0x1E,   0x15,   0x36 },
    { 0x3D,   0x22,   0x15,   0x3D },
    { 0x46,   0x27,   0x15,   0x46 },
    { 0x51,   0x2D,   0x15,   0x51 },
    { 0x5C,   0x33,   0x15,   0x5C },
    { 0x69,   0x3B,   0x15,   0x69 },
    { 0x78,   0x44,   0x15,   0x78 },
    { 0x89,   0x4F,   0x15,   0x89 }
};

static const unsigned char shdisp_main_dtv_bkl_tbl[SHDISP_BKL_FIX_TBL_NUM][NUM_SHDISP_BKL_TBL_MODE] = {
    { 0x00,   0x00,   0x00,   0x00 },
    { 0x04,   0x04,   0x04,   0x04 },
    { 0x0B,   0x0A,   0x0A,   0x0B },
    { 0x0C,   0x0B,   0x0B,   0x0C },
    { 0x0D,   0x0C,   0x0C,   0x0D },
    { 0x0F,   0x0D,   0x0D,   0x0F },
    { 0x11,   0x0E,   0x0E,   0x11 },
    { 0x13,   0x0F,   0x0F,   0x13 },
    { 0x16,   0x0F,   0x0F,   0x16 },
    { 0x19,   0x10,   0x10,   0x19 },
    { 0x1C,   0x11,   0x11,   0x1C },
    { 0x1F,   0x13,   0x13,   0x1F },
    { 0x24,   0x15,   0x15,   0x24 },
    { 0x29,   0x17,   0x15,   0x29 },
    { 0x2F,   0x1A,   0x15,   0x2F },
    { 0x36,   0x1E,   0x15,   0x36 },
    { 0x3D,   0x22,   0x15,   0x3D },
    { 0x46,   0x27,   0x15,   0x46 },
    { 0x51,   0x2D,   0x15,   0x51 },
    { 0x5C,   0x33,   0x15,   0x5C },
    { 0x69,   0x3B,   0x15,   0x69 },
    { 0x78,   0x44,   0x15,   0x78 },
    { 0x89,   0x4F,   0x15,   0x89 }
};



static const unsigned char shdisp_main_bkl_opt_low_tbl[SHDISP_BKL_AUTO_TBL_NUM][1+NUM_SHDISP_BKL_TBL_MODE] = {
    { BDIC_REG_OPT0,    0x0A,   0x05,   0x05,   0x0A },
    { BDIC_REG_OPT1,    0x0A,   0x05,   0x05,   0x0A },
    { BDIC_REG_OPT2,    0x0A,   0x05,   0x05,   0x0A },
    { BDIC_REG_OPT3,    0x13,   0x0B,   0x0B,   0x13 },
    { BDIC_REG_OPT4,    0x18,   0x0E,   0x0E,   0x1A },
    { BDIC_REG_OPT5,    0x1E,   0x12,   0x12,   0x22 },
    { BDIC_REG_OPT6,    0x25,   0x15,   0x15,   0x2B },
    { BDIC_REG_OPT7,    0x2B,   0x19,   0x19,   0x35 },
    { BDIC_REG_OPT8,    0x32,   0x1D,   0x1D,   0x3F },
    { BDIC_REG_OPT9,    0x3B,   0x22,   0x1D,   0x4C },
    { BDIC_REG_OPT10,   0x45,   0x28,   0x1D,   0x5D },
    { BDIC_REG_OPT11,   0x4F,   0x2E,   0x1D,   0x70 },
    { BDIC_REG_OPT12,   0x59,   0x34,   0x1D,   0x7B },
    { BDIC_REG_OPT13,   0x62,   0x39,   0x1D,   0x80 },
    { BDIC_REG_OPT14,   0x6A,   0x3D,   0x1D,   0x84 },
    { BDIC_REG_OPT15,   0x89,   0x4B,   0x1D,   0x89 }
};

static const unsigned char shdisp_main_dtv_bkl_opt_low_tbl[SHDISP_BKL_AUTO_TBL_NUM][1+NUM_SHDISP_BKL_TBL_MODE] = {
    { BDIC_REG_OPT0,    0x0A,   0x05,   0x05,   0x0A },
    { BDIC_REG_OPT1,    0x0A,   0x05,   0x05,   0x0A },
    { BDIC_REG_OPT2,    0x0A,   0x05,   0x05,   0x0A },
    { BDIC_REG_OPT3,    0x13,   0x0B,   0x0B,   0x13 },
    { BDIC_REG_OPT4,    0x18,   0x0E,   0x0E,   0x1A },
    { BDIC_REG_OPT5,    0x1E,   0x12,   0x12,   0x22 },
    { BDIC_REG_OPT6,    0x25,   0x15,   0x15,   0x2B },
    { BDIC_REG_OPT7,    0x2B,   0x19,   0x19,   0x35 },
    { BDIC_REG_OPT8,    0x32,   0x1D,   0x1D,   0x3F },
    { BDIC_REG_OPT9,    0x3B,   0x22,   0x1D,   0x4C },
    { BDIC_REG_OPT10,   0x45,   0x28,   0x1D,   0x5D },
    { BDIC_REG_OPT11,   0x4F,   0x2E,   0x1D,   0x70 },
    { BDIC_REG_OPT12,   0x59,   0x34,   0x1D,   0x7B },
    { BDIC_REG_OPT13,   0x62,   0x39,   0x1D,   0x80 },
    { BDIC_REG_OPT14,   0x6A,   0x3D,   0x1D,   0x84 },
    { BDIC_REG_OPT15,   0x89,   0x4B,   0x1D,   0x89 }
};

static const unsigned char shdisp_main_bkl_opt_high_tbl[SHDISP_BKL_AUTO_TBL_NUM][1+NUM_SHDISP_BKL_TBL_MODE] = {
    { BDIC_REG_OPT0,    0x55,   0x32,   0x1D,   0x78 },
    { BDIC_REG_OPT1,    0x5C,   0x36,   0x1D,   0x7D },
    { BDIC_REG_OPT2,    0x62,   0x39,   0x1D,   0x80 },
    { BDIC_REG_OPT3,    0x67,   0x3C,   0x1D,   0x83 },
    { BDIC_REG_OPT4,    0x6D,   0x3F,   0x1D,   0x85 },
    { BDIC_REG_OPT5,    0x72,   0x42,   0x1D,   0x87 },
    { BDIC_REG_OPT6,    0x78,   0x45,   0x1D,   0x88 },
    { BDIC_REG_OPT7,    0x89,   0x4B,   0x1D,   0x89 },
    { BDIC_REG_OPT8,    0x89,   0x4B,   0x1D,   0x89 },
    { BDIC_REG_OPT9,    0x89,   0x4B,   0x1D,   0x89 },
    { BDIC_REG_OPT10,   0x89,   0x4B,   0x1D,   0x89 },
    { BDIC_REG_OPT11,   0x89,   0x4B,   0x1D,   0x89 },
    { BDIC_REG_OPT12,   0x89,   0x4B,   0x1D,   0x89 },
    { BDIC_REG_OPT13,   0x89,   0x4B,   0x1D,   0x89 },
    { BDIC_REG_OPT14,   0x89,   0x4B,   0x1D,   0x89 },
    { BDIC_REG_OPT15,   0x89,   0x4B,   0x1D,   0x89 }
};

static const unsigned char shdisp_main_dtv_bkl_opt_high_tbl[SHDISP_BKL_AUTO_TBL_NUM][1+NUM_SHDISP_BKL_TBL_MODE] = {
    { BDIC_REG_OPT0,    0x55,   0x32,   0x1D,   0x78 },
    { BDIC_REG_OPT1,    0x5C,   0x36,   0x1D,   0x7D },
    { BDIC_REG_OPT2,    0x62,   0x39,   0x1D,   0x80 },
    { BDIC_REG_OPT3,    0x67,   0x3C,   0x1D,   0x83 },
    { BDIC_REG_OPT4,    0x6D,   0x3F,   0x1D,   0x85 },
    { BDIC_REG_OPT5,    0x72,   0x42,   0x1D,   0x87 },
    { BDIC_REG_OPT6,    0x78,   0x45,   0x1D,   0x88 },
    { BDIC_REG_OPT7,    0x89,   0x4B,   0x1D,   0x89 },
    { BDIC_REG_OPT8,    0x89,   0x4B,   0x1D,   0x89 },
    { BDIC_REG_OPT9,    0x89,   0x4B,   0x1D,   0x89 },
    { BDIC_REG_OPT10,   0x89,   0x4B,   0x1D,   0x89 },
    { BDIC_REG_OPT11,   0x89,   0x4B,   0x1D,   0x89 },
    { BDIC_REG_OPT12,   0x89,   0x4B,   0x1D,   0x89 },
    { BDIC_REG_OPT13,   0x89,   0x4B,   0x1D,   0x89 },
    { BDIC_REG_OPT14,   0x89,   0x4B,   0x1D,   0x89 },
    { BDIC_REG_OPT15,   0x89,   0x4B,   0x1D,   0x89 }
};

static const unsigned char shdisp_main_bkl_adj_tbl[NUM_SHDISP_MAIN_BKL_ADJ] = {
    0x00,
    0x00,
    0x00,
    0x00
};

static const unsigned char shdisp_triple_led_tbl[SHDISP_TRI_LED_COLOR_TBL_NUM][3] = {
    { 0x00, 0x00, 0x00 },
    { 0x0F, 0x00, 0x00 },
    { 0x00, 0x1E, 0x00 },
    { 0x0F, 0x1E, 0x00 },
    { 0x00, 0x00, 0x1E },
    { 0x0F, 0x00, 0x1E },
    { 0x00, 0x1E, 0x1E },
    { 0x0F, 0x1E, 0x1E }
};

static const unsigned char shdisp_triple_led_anime_tbl[2][SHDISP_TRI_LED_COLOR_TBL_NUM][3] = {
    {
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 }
    },
    {
        { 0x00, 0x00, 0x00 },
        { 0x0F, 0x00, 0x00 },
        { 0x00, 0x1E, 0x00 },
        { 0x0F, 0x1E, 0x00 },
        { 0x00, 0x00, 0x1E },
        { 0x0F, 0x00, 0x1E },
        { 0x00, 0x1E, 0x1E },
        { 0x0F, 0x1E, 0x1E }
    }
};

static const unsigned char shdisp_main_bkl_chg_high_tbl[14][2] = {
    { 0x00, 0xA9 },
    { 0x00, 0x9B },
    { 0x00, 0xB5 },
    { 0x00, 0xAF },
    { 0x00, 0xC1 },
    { 0x00, 0xBD },
    { 0x00, 0xC9 },
    { 0x00, 0xC6 },
    { 0x00, 0xD4 },
    { 0x00, 0xD0 },
    { 0x00, 0xDE },
    { 0x00, 0xDB },
    { 0x00, 0xEA },
    { 0x00, 0xE7 }
};

static const struct shdisp_bdic_bkl_lux_str shdisp_bdic_bkl_lux_tbl[2][16] = {
    {
        { 0, 0x000F,      4 },
        { 0, 0x001F,      7 },
        { 0, 0x002F,     13 },
        { 0, 0x003F,     25 },
        { 0, 0x004F,     45 },
        { 0, 0x005F,     84 },
        { 0, 0x006F,    155 },
        { 0, 0x007F,    290 },
        { 0, 0x008F,    530 },
        { 0, 0x009F,   1000 },
        { 0, 0x00AF,   2200 },
        { 0, 0x00BF,   4700 },
        { 0, 0x00CF,   8800 },
        { 0, 0x00DF,  16000 },
        { 1, 0x00EF,  36000 },
        { 1, 0x00FF,  78000 }
    },
    {
        { 0, 0x00A2,   4800 },
        { 1, 0x00B2,   7000 },
        { 1, 0x00BF,  11500 },
        { 1, 0x00C7,  16500 },
        { 1, 0x00D2,  24000 },
        { 1, 0x00DD,  34000 },
        { 1, 0x00E9,  51500 },
        { 1, 0x00FF,  65000 },
        { 1, 0x00FF,  65000 },
        { 1, 0x00FF,  65000 },
        { 1, 0x00FF,  65000 },
        { 1, 0x00FF,  65000 },
        { 1, 0x00FF,  65000 },
        { 1, 0x00FF,  65000 },
        { 1, 0x00FF,  65000 },
        { 1, 0x00FF,  65000 }
    }
};

/* ------------------------------------------------------------------------- */
/* TYPES                                                                     */
/* ------------------------------------------------------------------------- */



/* ------------------------------------------------------------------------- */
/* PROTOTYPES                                                                */
/* ------------------------------------------------------------------------- */



#endif /* SHDISP_BL69Y6_DATA_DL15_H */

/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
