/*
 * Core Source for:
 * Cypress TrueTouch(TM) Standard Product (TTSP) touchscreen drivers.
 * For use with Cypress Gen4 and Solo parts.
 * Supported parts include:
 * CY8CTMA398
 * CY8CTMA884
 * CY8CTMA4XX
 *
 * Copyright (C) 2009-2011 Cypress Semiconductor, Inc.
 * Copyright (C) 2011 Motorola Mobility, Inc.
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

#include <linux/delay.h>
#include <linux/input.h>
#ifdef MULTI_TOUCH_PROTOCOL_B
#include <linux/input/mt.h>
#endif	/* MULTI_TOUCH_PROTOCOL_B */
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <sharp/touch_platform.h>
#ifdef SH_TPSIF_COMMAND
#include <sharp/shtps_dev.h>
#endif	/* SH_TPSIF_COMMAND */
#include <linux/firmware.h>	/* This enables firmware class loader code */

/* platform address lookup offsets */
#define CY_TCH_ADDR_OFS		0
#define CY_LDR_ADDR_OFS		1

/* helpers */
#define GET_NUM_TOUCHES(x)          ((x) & 0x1F)
#define IS_LARGE_AREA(x)            ((x) & 0x20)
#define IS_BAD_PKT(x)               ((x) & 0x20)
#define IS_VALID_APP(x)             ((x) & 0x01)
#define IS_OPERATIONAL_ERR(x)       ((x) & 0x3F)
#define GET_HSTMODE(reg)            ((reg & 0x70) >> 4)
#define GET_BOOTLOADERMODE(reg)     ((reg & 0x10) >> 4)
#ifdef SH_TPSIF_COMMAND
#define	MINMAX(min, max, val)	((min)>(val) ? (min) : ((max)<(val) ? (max) : (val)))
#define	SET_POINT(val, x1, y1)      val.x = (x1); val.y = (y1)
#define	SET_AREA(val, x1, y1, x2, y2, x3, y3, x4, y4)	\
	val.p.x=x1;val.p.y=y1;val.q.x=x2;val.q.y=y2;	\
	val.r.x=x3;val.r.y=y3;val.s.x=x4;val.s.y=y4;
#endif	/* SH_TPSIF_COMMAND */

/* maximum number of concurrent tracks */
#define CY_NUM_TCH_ID               10
/* maximum number of track IDs */
#define CY_NUM_TRK_ID               16
/* maximum number of command data bytes */
#define CY_NUM_DAT                  6
/* maximum number of config block read data */
#define CY_NUM_CONFIG_BYTES        128

#define CY_REG_BASE                 0x00
#define CY_DELAY_DFLT               20 /* ms */
#define CY_DELAY_MAX                (500/CY_DELAY_DFLT) /* half second */
#define CY_HNDSHK_BIT               0x80
/* power mode select bits */
#define CY_SOFT_RESET_MODE          0x01
#define CY_DEEP_SLEEP_MODE          0x02
#define CY_LOW_POWER_MODE           0x04
/* device mode bits */
#define CY_MODE_CHANGE              0x08 /* rd/wr hst_mode */
#define CY_OPERATE_MODE             0x00 /* rd/wr hst_mode */
#define CY_SYSINFO_MODE             0x10 /* rd/wr hst_mode */
#define CY_CONFIG_MODE              0x20 /* rd/wr hst_mode */
#define CY_BL_MODE                  0x01 /*
					  * wr hst mode == soft reset
					  * was 0X10 to rep_stat for LTS
					  */
#define CY_IGNORE_VALUE             0xFFFF
#define CY_CMD_RDY_BIT              0x40

#define CY_REG_OP_START             0
#define CY_REG_SI_START             0
#define CY_REG_OP_END               0x20
#define CY_REG_SI_END               0x20

/* register field lengths */
#define CY_NUM_REVCTRL              8
#define CY_NUM_MFGID                8
#define CY_NUM_TCHREC               10
#define CY_NUM_DDATA                32
#define CY_NUM_MDATA                64
#define CY_TMA884_MAX_BYTES         255 /*
					  * max reg access for TMA884
					  * in config mode
					  */
#define CY_TMA400_MAX_BYTES         512 /*
					  * max reg access for TMA400
					  * in config mode
					  */

/* touch event id codes */
#define CY_GET_EVENTID(reg)         ((reg & 0x60) >> 5)
#define CY_GET_TRACKID(reg)         (reg & 0x1F)
#define CY_NOMOVE                   0
#define CY_TOUCHDOWN                1
#define CY_MOVE                     2
#define CY_LIFTOFF                  3

#define CY_CFG_BLK_SIZE             126

#define CY_BL_VERS_SIZE             12
#define CY_MAX_PRBUF_SIZE           PIPE_BUF
#define CY_NUM_TMA400_TT_CFG_BLK    51 /* Rev84 mapping */

#ifdef CONFIG_TOUCHSCREEN_DEBUG
#define CY_BL_TXT_FW_IMG_SIZE       128261
#define CY_BL_BIN_FW_IMG_SIZE       128261
#define CY_BL_FW_NAME_SIZE          NAME_MAX
#define CY_RW_REGID_MAX             0x1F
#define CY_RW_REG_DATA_MAX          0xFF
#define CY_NUM_PKG_PKT              4
#define CY_NUM_PKT_DATA             32
#define CY_MAX_PKG_DATA             (CY_NUM_PKG_PKT * CY_NUM_PKT_DATA)
#define CY_MAX_IC_BUF               256
#endif

/* abs settings */
#define CY_NUM_ABS_SET  5 /* number of abs values per setting */
/* abs value offset */
#define CY_SIGNAL_OST   0
#define CY_MIN_OST      1
#define CY_MAX_OST      2
#define CY_FUZZ_OST     3
#define CY_FLAT_OST     4
/* axis signal offset */
#define CY_ABS_X_OST    0
#define CY_ABS_Y_OST    1
#define CY_ABS_P_OST    2
#define CY_ABS_W_OST    3
#define CY_ABS_ID_OST   4

#ifdef SH_TPSIF_COMMAND
/* boot mode */
#define SH_BOOT_MODE_HW_CHK	0x41

#define	ADJUST_POINT	6	/* Number of Adjustment points */
#define	AREA_COUNT	(ADJUST_POINT * 2) /* Number of Adjustment Area */
#define DOUBLE_ACCURACY	10000

#define	POS_X0		0
#define	POS_X1		179
#define	POS_X2		539
#define	POS_X3		719
#define	POS_Y0		0
#define	POS_Y1		319
#define	POS_Y2		639
#define	POS_Y3		959
#define	POS_Y4		1279
#define	POS_LIMIT	100
#endif	/* SH_TPSIF_COMMAND */

enum cyttsp4_powerstate {
	CY_IDLE_STATE,		/* IC cannot be reached */
	CY_READY_STATE,		/* pre-operational; ready to go to ACTIVE */
	CY_ACTIVE_STATE,	/* app is running, IC is scanning */
	CY_LOW_PWR_STATE,	/* not currently used  */
	CY_SLEEP_STATE,		/* app is running, IC is idle */
	CY_BL_STATE,		/* bootloader is running */
	CY_LDR_STATE,		/* loader is running */
	CY_SYSINFO_STATE,	/* switching to sysinfo mode */
	CY_CMD_STATE,		/* command initiation mode */
	CY_INVALID_STATE	/* always last in the list */
};

static char *cyttsp4_powerstate_string[] = {
	/* Order must match enum cyttsp4_powerstate above */
	"IDLE",
	"READY",
	"ACTIVE",
	"LOW_PWR",
	"SLEEP",
	"BOOTLOADER",
	"LOADER",
	"SYSINFO",
	"CMD",
	"INVALID"
};

enum cyttsp4_controller_mode {
	CY_MODE_BOOTLOADER,
	CY_MODE_SYSINFO,
	CY_MODE_OPERATIONAL,
	CY_MODE_CONFIG,
	CY_MODE_NUM
};

enum cyttsp4_ic_grpnum {
	CY_IC_GRPNUM_RESERVED = 0,
	CY_IC_GRPNUM_CMD_REGS,
	CY_IC_GRPNUM_TCH_REP,
	CY_IC_GRPNUM_DATA_REC,
	CY_IC_GRPNUM_TEST_REC,
	CY_IC_GRPNUM_PCFG_REC,
	CY_IC_GRPNUM_TCH_PARM_VAL,
	CY_IC_GRPNUM_TCH_PARM_SIZ,
	CY_IC_GRPNUM_RESERVED1,
	CY_IC_GRPNUM_RESERVED2,
	CY_IC_GRPNUM_OPCFG_REC,
	CY_IC_GRPNUM_DDATA_REC,
	CY_IC_GRPNUM_MDATA_REC,
	CY_IC_GRPNUM_TEST_DATA,
	CY_IC_GRPNUM_NUM
};

enum cyttsp4_ic_command {
	CY_GET_PARAM_CMD = 0x02,
	CY_SET_PARAM_CMD = 0x03,
	CY_GET_CFG_BLK_CRC = 0x05,
};

enum cyttsp4_ic_cfg_blkid {
	CY_TCH_PARM_BLKID = 0x00,
	CY_DDATA_BLKID = 0x05,
	CY_MDATA_BLKID = 0x06,
};

enum cyttsp4_flags {
	CY_FLAG_TMA400 = 0x01,
};

/* TMA400A TT_CFG interface definitions */
struct cyttsp4_tma400A_config_crc {
	u8 CONFIG_CRC[4];
};
struct cyttsp4_tma400A_sdk_controller_config {
	u8 SDK_CTRL_CFG_SIZE[4];
	u8 X_LEN_PHY[2];
	u8 Y_LEN_PHY[2];
	u8 HST_MODE0;
	u8 ACT_DIST0;
	u8 SCAN_TYP0;
	u8 ACT_INTRVL0;
	u8 ACT_LFT_INTRVL0;
	u8 Reserved_1;
	u8 TCH_TMOUT0[2];
	u8 LP_INTRVL0[2];
	u8 PWR_CFG;
	u8 INT_CFG;
	u8 INT_PULSE_DATA;
	u8 OPMODE_CFG;
	u8 HANDSHAKE_TIMEOUT[2];
	u8 TIMER_CAL_INTERVAL;
	u8 Reserved_2;
	u8 RP2P_MIN[2];
	u8 ILEAK_MAX[2];
	u8 RFB_P2P[2];
	u8 RFB_EXT[2];
	u8 IDACOPEN_LOW;
	u8 IDACOPEN_HIGH;
	u8 GIDAC_OPEN;
	u8 GAIN_OPEN;
	u8 POST_CFG;
	u8 GESTURE_CFG;
	u8 GEST_EN[32];
	u8 Reserved_align[52];
};

struct cyttsp4_tma400A_tt_cfg {
	struct cyttsp4_tma400A_config_crc config_crc;
	struct cyttsp4_tma400A_sdk_controller_config sdk_controller_config;
};

/* GEN4/SOLO Operational interface definitions */
struct cyttsp4_touch {
	int x;	/* x position */
	int y;	/* y position */
	int p;	/* pressure */
	int t;	/* track id */
	int e;	/* event id */
	int o;	/* object type */
	int w;	/* size */
} __attribute__((packed));

/* TTSP System Information interface definitions */
struct cyttsp4_cydata {
	u8 ttpidh;
	u8 ttpidl;
	u8 fw_ver_major;
	u8 fw_ver_minor;
	u8 revctrl[CY_NUM_REVCTRL];
	u8 blver_major;
	u8 blver_minor;
	u8 jtag_si_id3;
	u8 jtag_si_id2;
	u8 jtag_si_id1;
	u8 jtag_si_id0;
	u8 mfgid_sz;
	u8 mfg_id[CY_NUM_MFGID];
	u8 cyito_idh;
	u8 cyito_idl;
	u8 cyito_verh;
	u8 cyito_verl;
	u8 ttsp_ver_major;
	u8 ttsp_ver_minor;
	u8 device_info;
} __attribute__((packed));

struct cyttsp4_test {
	u8 post_codeh;
	u8 post_codel;
} __attribute__((packed));

struct cyttsp4_pcfg {
	u8 electrodes_x;
	u8 electrodes_y;
	u8 len_xh;
	u8 len_xl;
	u8 len_yh;
	u8 len_yl;
	u8 axis_xh;
	u8 axis_xl;
	u8 axis_yh;
	u8 axis_yl;
	u8 max_zh;
	u8 max_zl;
} __attribute__((packed));

struct cyttsp4_opcfg {
	u8 cmd_ofs;
	u8 rep_ofs;
	u8 rep_szh;
	u8 rep_szl;
	u8 num_btns;
	u8 tt_stat_ofs;
	u8 obj_cfg0;
	u8 max_tchs;
	u8 tch_rec_siz;
	u8 tch_rec_0;	/* x position */
	u8 tch_rec_1;
	u8 tch_rec_2;	/* y position */
	u8 tch_rec_3;
	u8 tch_rec_4;	/* pressure */
	u8 tch_rec_5;
	u8 tch_rec_6;	/* track id */
	u8 tch_rec_7;
	u8 tch_rec_8;	/* event id */
	u8 tch_rec_9;
	u8 tch_rec_10;	/* object type */
	u8 tch_rec_11;
	u8 tch_rec_12;	/* size */
	u8 tch_rec_13;
} __attribute__((packed));

struct cyttsp4_sysinfo_data {
	u8 hst_mode;
	u8 reserved;
	u8 map_szh;
	u8 map_szl;
	u8 cydata_ofsh;
	u8 cydata_ofsl;
	u8 test_ofsh;
	u8 test_ofsl;
	u8 pcfg_ofsh;
	u8 pcfg_ofsl;
	u8 opcfg_ofsh;
	u8 opcfg_ofsl;
	u8 ddata_ofsh;
	u8 ddata_ofsl;
	u8 mdata_ofsh;
	u8 mdata_ofsl;
} __attribute__((packed));

struct cyttsp4_sysinfo_ptr {
	struct cyttsp4_cydata *cydata;
	struct cyttsp4_test *test;
	struct cyttsp4_pcfg *pcfg;
	struct cyttsp4_opcfg *opcfg;
	struct cyttsp4_ddata *ddata;
	struct cyttsp4_mdata *mdata;
} __attribute__((packed));

struct cyttsp4_sysinfo_ofs {
	size_t cmd_ofs;
	size_t rep_ofs;
	size_t rep_sz;
	size_t tt_stat_ofs;
	size_t tch_rec_siz;
	size_t obj_cfg0;
	size_t max_tchs;
	size_t mode_size;
	size_t data_size;
	size_t map_sz;
	size_t cydata_ofs;
	size_t test_ofs;
	size_t pcfg_ofs;
	size_t opcfg_ofs;
	size_t ddata_ofs;
	size_t mdata_ofs;
	size_t cydata_size;
	size_t test_size;
	size_t pcfg_size;
	size_t opcfg_size;
	size_t ddata_size;
	size_t mdata_size;
	size_t tch_rec_x_ofs;	/* x position */
	size_t tch_rec_x_size;
	size_t tch_rec_x_max;
	size_t tch_rec_y_ofs;	/* y position */
	size_t tch_rec_y_size;
	size_t tch_rec_y_max;
	size_t tch_rec_p_ofs;	/* pressure */
	size_t tch_rec_p_size;
	size_t tch_rec_p_max;
	size_t tch_rec_t_ofs;	/* track id */
	size_t tch_rec_t_size;
	size_t tch_rec_t_max;
	size_t tch_rec_e_ofs;	/* event id */
	size_t tch_rec_e_size;
	size_t tch_rec_e_max;
	size_t tch_rec_o_ofs;	/* object type */
	size_t tch_rec_o_size;
	size_t tch_rec_o_max;
	size_t tch_rec_w_ofs;	/* size */
	size_t tch_rec_w_size;
	size_t tch_rec_w_max;
};

/* driver context structure definitions */
#ifdef CONFIG_TOUCHSCREEN_DEBUG
struct cyttsp4_dbg_pkg {
	bool ready;
	int cnt;
	u8 data[CY_MAX_PKG_DATA];
};
#endif
struct cyttsp4 {
	struct device *dev;
	int irq;
	struct input_dev *input;
	struct mutex data_lock;		/* prevent concurrent accesses */
	struct mutex startup_mutex;	/* protect power on sequence */
	struct workqueue_struct		*cyttsp4_wq;
	struct work_struct		cyttsp4_resume_startup_work;
	struct work_struct		cyttsp4_startup_work;
	char phys[32];
	const struct bus_type *bus_type;
	const struct touch_platform_data *platform_data;
	u8 *xy_mode;			/* operational mode and status regs */
	u8 *xy_data;			/* operational touch regs */
	struct cyttsp4_bus_ops *bus_ops;
	struct cyttsp4_sysinfo_data sysinfo_data;
	struct cyttsp4_sysinfo_ptr sysinfo_ptr;
	struct cyttsp4_sysinfo_ofs si_ofs;
	struct completion bl_int_running;
	struct completion si_int_running;
	enum cyttsp4_powerstate power_state;
	enum cyttsp4_controller_mode current_mode;
	bool powered;
	bool cmd_rdy;
	bool hndshk_enabled;
	bool was_suspended;
	u16 flags;
	size_t max_config_bytes;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
	bool bl_ready_flag;
#ifdef CONFIG_TOUCHSCREEN_DEBUG
	bool waiting_for_fw;
	bool debug_upgrade;
	char *fwname;
	bool irq_enabled;
	int ic_grpnum;
	int ic_grpoffset;
	bool ic_grptest;
#endif
#ifdef SH_TPSIF_COMMAND
	bool adjust_enabled;
	bool reverse_y_axis;
#endif	/* SH_TPSIF_COMMAND */
};

#ifdef SH_TPSIF_COMMAND
typedef struct {
	int x;
	int y;
} sh_tpsif_point_t;

typedef struct {
	sh_tpsif_point_t p;		/* Upper left */
	sh_tpsif_point_t q;		/* Upper right */
	sh_tpsif_point_t r;		/* Lower left */
	sh_tpsif_point_t s;		/* Lower right */
} sh_tpsif_area_t;

typedef struct {
	int value;
	int num;
} sh_tpsif_qsort_t;
#endif	/* SH_TPSIF_COMMAND */

#ifdef SH_TPSIF_COMMAND
static struct cyttsp4 *g_tpsif_ts;
static wait_queue_head_t sh_tpsif_wq;
static int sh_tpsif_event;
static struct hrtimer sh_tpsif_polling_timer;
static struct work_struct sh_tpsif_polling_work;

/* Coordinates of six criteria points for adjusting */
static const sh_tpsif_point_t sh_tpsif_base_point[ADJUST_POINT] = {
	{POS_X1, POS_Y1}, {POS_X2, POS_Y1},
	{POS_X1, POS_Y2}, {POS_X2, POS_Y2},
	{POS_X1, POS_Y3}, {POS_X2, POS_Y3},
};
/* Coordinates of the split area for adjusting */
static const sh_tpsif_area_t sh_tpsif_area_rect[AREA_COUNT] = {
	{{POS_X0, POS_Y0}, {POS_X1, POS_Y0}, {POS_X0, POS_Y1}, {POS_X1, POS_Y1}},
	{{POS_X1, POS_Y0}, {POS_X2, POS_Y0}, {POS_X1, POS_Y1}, {POS_X2, POS_Y1}},
	{{POS_X2, POS_Y0}, {POS_X3, POS_Y0}, {POS_X2, POS_Y1}, {POS_X3, POS_Y1}},
	{{POS_X0, POS_Y1}, {POS_X1, POS_Y1}, {POS_X0, POS_Y2}, {POS_X1, POS_Y2}},
	{{POS_X1, POS_Y1}, {POS_X2, POS_Y1}, {POS_X1, POS_Y2}, {POS_X2, POS_Y2}},
	{{POS_X2, POS_Y1}, {POS_X3, POS_Y1}, {POS_X2, POS_Y2}, {POS_X3, POS_Y2}},
	{{POS_X0, POS_Y2}, {POS_X1, POS_Y2}, {POS_X0, POS_Y3}, {POS_X1, POS_Y3}},
	{{POS_X1, POS_Y2}, {POS_X2, POS_Y2}, {POS_X1, POS_Y3}, {POS_X2, POS_Y3}},
	{{POS_X2, POS_Y2}, {POS_X3, POS_Y2}, {POS_X2, POS_Y3}, {POS_X3, POS_Y3}},
	{{POS_X0, POS_Y3}, {POS_X1, POS_Y3}, {POS_X0, POS_Y4}, {POS_X1, POS_Y4}},
	{{POS_X1, POS_Y3}, {POS_X2, POS_Y3}, {POS_X1, POS_Y4}, {POS_X2, POS_Y4}},
	{{POS_X2, POS_Y3}, {POS_X3, POS_Y3}, {POS_X2, POS_Y4}, {POS_X3, POS_Y4}},
};
static sh_tpsif_area_t sh_tpsif_area_diff[AREA_COUNT];
static sh_tpsif_point_t sh_tpsif_adjust_param[ADJUST_POINT];
#endif	/* SH_TPSIF_COMMAND */


static int _cyttsp4_load_app(struct cyttsp4 *ts, const u8 *fw, int fw_size);
static int _cyttsp4_startup(struct cyttsp4 *ts);
static int _cyttsp4_calc_data_crc(struct cyttsp4 *ts,
	size_t ndata, u8 *pdata, u8 *crc_h, u8 *crc_l, const char *name);
static int _cyttsp4_get_ic_crc(struct cyttsp4 *ts,
	u8 blkid, u8 *crc_h, u8 *crc_l);
static int _cyttsp4_set_config_mode(struct cyttsp4 *ts);
static int _cyttsp4_ldr_exit(struct cyttsp4 *ts);
static irqreturn_t cyttsp4_irq(int irq, void *handle);
static int _cyttsp4_set_operational_mode(struct cyttsp4 *ts);
#ifdef CONFIG_TOUCHSCREEN_DEBUG
static int _cyttsp4_calc_ic_crc_tma400(struct cyttsp4 *ts,
	u8 *calc_ic_crc, size_t num_data);
#endif
#ifdef SH_TPSIF_COMMAND
static void _sh_tpsif_adjust_point(struct cyttsp4 *ts, int *x, int *y);
static void _sh_tpsif_poll_start(struct cyttsp4 *ts);
static void _sh_tpsif_poll_stop(struct cyttsp4 *ts);
static void _sh_tpsif_poll_scan(struct work_struct *work);
static enum hrtimer_restart _sh_tpsif_poll_timer_handler(struct hrtimer *timer);
#if 0
static unsigned short _sh_tpsif_get_bootmode(void);
#endif	/* 0 */
static int sh_tpsif_hw_reset(struct cyttsp4 *ts);
static int sh_tpsif_sw_reset(struct cyttsp4 *ts);
static int sh_tpsif_hw_reset_startup(struct cyttsp4 *ts);
static int sh_tpsif_sw_reset_startup(struct cyttsp4 *ts);
static int sh_tpsif_calibration_idac(struct cyttsp4 *ts);
static int _sh_tpsif_firmware_version(struct cyttsp4 *ts, u32 *version);
#endif	/* SH_TPSIF_COMMAND */


static void cyttsp4_pr_state(struct cyttsp4 *ts)
{
	pr_info("%s: %s\n", __func__,
		ts->power_state < CY_INVALID_STATE ?
		cyttsp4_powerstate_string[ts->power_state] :
		"INVALID");
}

static int cyttsp4_read_block_data(struct cyttsp4 *ts, u16 command,
	size_t length, void *buf, int i2c_addr, bool use_subaddr)
{
	int retval;
	int tries;

	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

	if (!buf || !length)
		return -EIO;

	for (tries = 0, retval = -1;
		tries < CY_NUM_RETRY && (retval < 0);
		tries++) {
		retval = ts->bus_ops->read(ts->bus_ops, command, length, buf,
			i2c_addr, use_subaddr);
		if (retval)
			udelay(500);
	}

	if (retval < 0) {
		pr_err("%s: I2C read block data fail (ret=%d)\n",
			__func__, retval);
	}
	return retval;
}

static int cyttsp4_write_block_data(struct cyttsp4 *ts, u16 command,
	size_t length, const void *buf, int i2c_addr, bool use_subaddr)
{
	int retval;
	int tries;

	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

	if (!buf || !length)
		return -EIO;

	for (tries = 0, retval = -1;
		tries < CY_NUM_RETRY && (retval < 0);
		tries++) {
		retval = ts->bus_ops->write(ts->bus_ops, command, length, buf,
			i2c_addr, use_subaddr);
		if (retval)
			udelay(500);
	}

	if (retval < 0) {
		pr_err("%s: I2C write block data fail (ret=%d)\n",
			__func__, retval);
	}
	return retval;
}

static void _cyttsp4_pr_buf(struct cyttsp4 *ts,
	u8 *dptr, int size, const char *data_name)
{
#ifdef CONFIG_TOUCHSCREEN_DEBUG
	int i;
	int max = (CY_MAX_PRBUF_SIZE - 1) - sizeof(" truncated...");
	char *pbuf;

	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

	if (ts->bus_ops->tsdebug >= CY_DBG_LVL_2) {
		pbuf = kzalloc(CY_MAX_PRBUF_SIZE, GFP_KERNEL);
		if (pbuf) {
			for (i = 0; i < size && i < max; i++)
				sprintf(pbuf, "%s %02X", pbuf, dptr[i]);
			pr_info("%s:  %s[0..%d]=%s%s\n", __func__,
				data_name, size-1, pbuf,
				size <= max ?
				"" : " truncated...");
			kfree(pbuf);
		} else
			pr_err("%s: buf allocation error\n", __func__);
	}
#endif
	return;
}

static int _cyttsp4_get_block_size(struct cyttsp4 *ts, size_t *block_size)
{
	int tries;
	int tmp_state;
	int retval = 0;
	u8 cmd_dat[CY_NUM_DAT + 1];	/* +1 for cmd byte */

	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

	memset(cmd_dat, 0, sizeof(cmd_dat));
	cmd_dat[0] = 0x02;	/* get block size command */

	tmp_state = ts->power_state;
	ts->power_state = CY_CMD_STATE;
	cyttsp4_pr_state(ts);
	ts->cmd_rdy = false;
	retval = cyttsp4_write_block_data(ts, ts->si_ofs.cmd_ofs,
		sizeof(cmd_dat), cmd_dat,
		ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
	if (retval) {
		pr_err("%s: Fail writing get block size cmd r=%d\n",
			__func__, retval);
		goto _cyttsp4_get_block_size_exit;
	}

	tries = 0;
	while (tries++ < 10000) {
		if (mutex_is_locked(&ts->data_lock)) {
			mutex_unlock(&ts->data_lock);
			udelay(1000);
			mutex_lock(&ts->data_lock);
		} else {
			udelay(1000);
		}

		if (ts->cmd_rdy) {
			retval = cyttsp4_read_block_data(ts, ts->si_ofs.cmd_ofs,
				sizeof(cmd_dat), cmd_dat,
				ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
			_cyttsp4_pr_buf(ts, cmd_dat, sizeof(cmd_dat),
				"get_block_size read->cmd_dat[]");
			if (retval < 0) {
				pr_err("%s: fail read host mode"
					"r=%d\n", __func__, retval);
				goto _cyttsp4_get_block_size_exit;
			} else if (cmd_dat[0] & CY_CMD_RDY_BIT) {
				break;
			} else {
				/* not our interrupt */
				ts->cmd_rdy = false;
			}
		}
	}

	if (tries >= 10000) {
		pr_err("%s: timeout waiting for cmd ready interrupt\n",
			__func__);
		retval = -ETIMEDOUT;
		goto _cyttsp4_get_block_size_exit;
	}

	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: check cmd ready tries=%d ret=%d cmd=%02X %02X %02X\n",
		__func__, tries, retval, cmd_dat[0], cmd_dat[1], cmd_dat[2]);

	*block_size = (cmd_dat[1] * 256) + cmd_dat[2];

_cyttsp4_get_block_size_exit:
	ts->power_state = tmp_state;
	cyttsp4_pr_state(ts);
	return retval;
}

static int _cyttsp4_get_block_data(struct cyttsp4 *ts,
	u8 block_id, size_t block_size, u8 *pdata)
{
	int tries;
	int tmp_state;
	int retval = 0;
	u8 cmd_dat[CY_NUM_DAT + 1];	/* +1 for cmd byte */

	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

	memset(cmd_dat, 0, sizeof(cmd_dat));
	cmd_dat[0] = 0x03;	/* get block data command */
	cmd_dat[1] = block_id;

	tmp_state = ts->power_state;
	ts->power_state = CY_CMD_STATE;
	cyttsp4_pr_state(ts);
	ts->cmd_rdy = false;
	retval = cyttsp4_write_block_data(ts, ts->si_ofs.cmd_ofs,
		sizeof(cmd_dat), cmd_dat,
		ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
	if (retval) {
		pr_err("%s: Fail writing get block data cmd r=%d\n",
			__func__, retval);
		goto _cyttsp4_get_block_data_exit;
	}

	tries = 0;
	while (tries++ < 10000) {
		if (mutex_is_locked(&ts->data_lock)) {
			mutex_unlock(&ts->data_lock);
			udelay(1000);
			mutex_lock(&ts->data_lock);
		} else {
			udelay(1000);
		}

		if (ts->cmd_rdy) {
			retval = cyttsp4_read_block_data(ts, ts->si_ofs.cmd_ofs,
				sizeof(cmd_dat), cmd_dat,
				ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
			if (retval < 0) {
				pr_err("%s: Fail read host mode"
					"r=%d\n", __func__, retval);
				goto _cyttsp4_get_block_data_exit;
			} else if (cmd_dat[0] & CY_CMD_RDY_BIT) {
				break;
			} else {
				/* not our interrupt */
				ts->cmd_rdy = false;
			}
		}
	}

	if (tries >= 10000) {
		pr_err("%s: timeout waiting for cmd ready interrupt\n",
			__func__);
		retval = -ETIMEDOUT;
		goto _cyttsp4_get_block_data_exit;
	}

	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: check cmd ready tries=%d ret=%d cmd=%02X %02X %02X\n",
		__func__, tries, retval, cmd_dat[0], cmd_dat[1], cmd_dat[2]);

	if (cmd_dat[1]) {
		pr_err("%s: Fail get block command err=0x%02X\n",
			__func__, cmd_dat[1]);
		retval = -EIO;
	} else {
		retval = cyttsp4_read_block_data(ts, ts->si_ofs.cmd_ofs + 2,
			block_size, pdata,
			ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
		if (retval < 0) {
			pr_err("%s: fail get block=%d data r=%d\n",
				__func__, block_id, retval);
			retval = -EIO;
			goto _cyttsp4_get_block_data_exit;
		} else
			retval = 0;
	}

_cyttsp4_get_block_data_exit:
	ts->power_state = tmp_state;
	cyttsp4_pr_state(ts);
	return retval;
}

static u8 cyttsp4_security_key[] = {
	0xA5, 0x01, 0x02, 0x03, 0xFF, 0xFE, 0xFD, 0x5A
};

static int _cyttsp4_put_block_data(struct cyttsp4 *ts,
	u8 block_id, size_t block_size, u8 *pdata)
{
	int tries;
	int tmp_state;
	int retval = 0;
	u8 *out_data = NULL;
	size_t out_data_size;
	u8 cmd_dat[CY_NUM_DAT + 1];	/* +1 for cmd byte */

	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

	memset(cmd_dat, 0, sizeof(cmd_dat));

	tmp_state = ts->power_state;
	ts->power_state = CY_CMD_STATE;
	cyttsp4_pr_state(ts);

	out_data_size = 2 + block_size + sizeof(cyttsp4_security_key);
	out_data = kzalloc(out_data_size, GFP_KERNEL);
	if (out_data == NULL) {
		pr_err("%s: Fail alloc out_data buffer\n", __func__);
		goto _cyttsp4_put_block_data_exit;
	}
	out_data[0] = 0x04;	/* put block data command */
	out_data[1] = block_id;
	memcpy(&out_data[2], pdata, block_size);
	memcpy(&out_data[2 + block_size],
		cyttsp4_security_key, sizeof(cyttsp4_security_key));
	ts->cmd_rdy = false;
	retval = cyttsp4_write_block_data(ts, ts->si_ofs.cmd_ofs,
		out_data_size, out_data,
		ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
	if (retval) {
		pr_err("%s: Fail writing put block=%d data cmd r=%d\n",
			__func__, block_id, retval);
		goto _cyttsp4_put_block_data_exit;
	}

	tries = 0;
	while (tries++ < 10000) {
		if (mutex_is_locked(&ts->data_lock)) {
			mutex_unlock(&ts->data_lock);
			udelay(1000);
			mutex_lock(&ts->data_lock);
		} else {
			udelay(1000);
		}

		if (ts->cmd_rdy) {
			retval = cyttsp4_read_block_data(ts, ts->si_ofs.cmd_ofs,
				sizeof(cmd_dat), cmd_dat,
				ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
			_cyttsp4_pr_buf(ts, cmd_dat, sizeof(cmd_dat),
				"put_block_data read->cmd_dat[]");
			if (retval < 0) {
				pr_err("%s: fail read host mode"
					"r=%d\n", __func__, retval);
				goto _cyttsp4_put_block_data_exit;
			} else if (cmd_dat[0] & CY_CMD_RDY_BIT) {
				break;
			} else {
				/* not our interrupt */
				ts->cmd_rdy = false;
			}
		}
	}

	if (tries >= 10000) {
		pr_err("%s: timeout waiting for cmd ready interrupt\n",
			__func__);
		retval = -ETIMEDOUT;
		goto _cyttsp4_put_block_data_exit;
	}

	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: check cmd ready tries=%d ret=%d cmd=%02X %02X %02X\n",
		__func__, tries, retval, cmd_dat[0], cmd_dat[1], cmd_dat[2]);

_cyttsp4_put_block_data_exit:
	ts->power_state = tmp_state;
	cyttsp4_pr_state(ts);
	return retval;
}

static int _cyttsp4_put_all_params_tma400(struct cyttsp4 *ts)
{
	size_t block_size;
	int block_id = 0;
	int num_blocks = 0;
	u8 *pdata = NULL;
	int retval = 0;

	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

	retval = _cyttsp4_set_config_mode(ts);
	if (retval < 0) {
		pr_err("%s: Fail switch to config mode r=%d\n",
			__func__, retval);
		goto _cyttsp4_put_all_params_tma400_err;
	}

	retval = _cyttsp4_get_block_size(ts, &block_size);
	if (retval < 0) {
		pr_err("%s: Fail get block size r=%d\n", __func__, retval);
		goto _cyttsp4_put_all_params_tma400_exit;
	}

	if (ts->platform_data->sett[CY_IC_GRPNUM_TCH_PARM_VAL]->data == NULL) {
		pr_err("%s: NULL parameter values table\n", __func__);
		goto _cyttsp4_put_all_params_tma400_exit;
	}

	num_blocks = ts->platform_data->sett
		[CY_IC_GRPNUM_TCH_PARM_VAL]->size / block_size;
	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: num_blocks=%d block_size=%d table_size=%d\n", __func__,
		num_blocks, block_size,
		ts->platform_data->sett[CY_IC_GRPNUM_TCH_PARM_VAL]->size);
	pdata = (u8 *)ts->platform_data->sett[CY_IC_GRPNUM_TCH_PARM_VAL]->data;
	for (block_id = 0; block_id < num_blocks; block_id++) {
		cyttsp4_dbg(ts, CY_DBG_LVL_3,
			"%s: block=%d pdata=%p\n", __func__, block_id, pdata);
		retval = _cyttsp4_put_block_data(ts,
			block_id, block_size, pdata);
		if (retval < 0) {
			pr_err("%s: Fail put block=%d r=%d\n",
				__func__, block_id, retval);
			goto _cyttsp4_put_all_params_tma400_exit;
		}
		pdata += block_size;
	}

_cyttsp4_put_all_params_tma400_exit:
	retval = _cyttsp4_set_operational_mode(ts);
	if (retval < 0) {
		pr_err("%s: Fail switch to operational mode r=%d\n",
			__func__, retval);
	}

_cyttsp4_put_all_params_tma400_err:
	return retval;
}

static int cyttsp4_hndshk(struct cyttsp4 *ts, u8 hst_mode)
{
	int retval = 0;
	u8 cmd;

	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

	if (ts->hndshk_enabled) {
		cmd = hst_mode & CY_HNDSHK_BIT ?
			hst_mode & ~CY_HNDSHK_BIT :
			hst_mode | CY_HNDSHK_BIT;

		retval = cyttsp4_write_block_data(ts, CY_REG_BASE,
			sizeof(cmd), (u8 *)&cmd,
			ts->platform_data->addr[CY_TCH_ADDR_OFS], true);

		if (retval < 0) {
			pr_err("%s: I2C write fail on handshake (ret=%d)\n",
				__func__, retval);
		}
	}

	return retval;
}

static int _cyttsp4_hndshk_enable(struct cyttsp4 *ts)
{
	int tries;
	int tmp_state;
	int retval = 0;
	u8 cmd_dat[CY_NUM_DAT + 1];	/* +1 for cmd byte */
	int block_id;
	size_t block_size;
	u8 *pdata = NULL;
	struct cyttsp4_tma400A_tt_cfg *tt_cfg;

	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

	if (ts->flags & CY_FLAG_TMA400) {
		/*
		 * TODO: look at:
		 * TT_CFG.OPMODE_CFG & 0x03 == 0x01(edge) or 0x02(level)
		 */
		ts->hndshk_enabled = false;	/* default */
		retval = _cyttsp4_get_block_size(ts, &block_size);
		if (retval < 0) {
			pr_err("%s: Fail get block size r=%d\n",
				__func__, retval);
			retval = -EIO;
			goto _cyttsp4_set_hndshk_enable_tma400_exit1;
		}

		pdata = kzalloc(block_size, GFP_KERNEL);
		if (pdata == NULL) {
			pr_err("%s: Fail allocate block buffer\n", __func__);
			retval = -ENOMEM;
			goto _cyttsp4_set_hndshk_enable_tma400_exit1;
		}

		block_id = 0;
		retval = _cyttsp4_get_block_data(ts,
			block_id, block_size, pdata);
		if (retval < 0) {
			pr_err("%s: Fail get block=%d data r=%d\n",
				__func__, block_id, retval);
			retval = -EIO;
			goto _cyttsp4_set_hndshk_enable_tma400_exit1;
		}

		tt_cfg = (struct cyttsp4_tma400A_tt_cfg *)pdata;
		if (tt_cfg->sdk_controller_config.OPMODE_CFG & 0x03)
			ts->hndshk_enabled = true;

		cyttsp4_dbg(ts, CY_DBG_LVL_2,
			"%s: opmode_cfg=%02X hndshk_enabled=%d\n",
			__func__, tt_cfg->sdk_controller_config.OPMODE_CFG,
			(int)ts->hndshk_enabled);

_cyttsp4_set_hndshk_enable_tma400_exit1:
		if (pdata != NULL)
			kfree(pdata);
		goto _cyttsp4_set_hndshk_enable_tma400_exit;
	}

	memset(cmd_dat, 0, sizeof(cmd_dat));
	cmd_dat[0] = 0x26;	/* handshake enable operational command */
	cmd_dat[1] = 0x02;	/* synchronous edge handshake */

	tmp_state = ts->power_state;
	ts->power_state = CY_CMD_STATE;
	cyttsp4_pr_state(ts);
	ts->cmd_rdy = false;
	retval = cyttsp4_write_block_data(ts, ts->si_ofs.cmd_ofs,
		sizeof(cmd_dat), cmd_dat,
		ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
	if (retval) {
		pr_err("%s: Fail writing enable handshake cmd r=%d\n",
			__func__, retval);
		return retval;
	}

	tries = 0;
	while (tries++ < 10000) {
		if (mutex_is_locked(&ts->data_lock)) {
			mutex_unlock(&ts->data_lock);
			udelay(1000);
			mutex_lock(&ts->data_lock);
		} else {
			udelay(1000);
		}

		if (ts->cmd_rdy) {
			retval = cyttsp4_read_block_data(ts, ts->si_ofs.cmd_ofs,
				sizeof(cmd_dat), cmd_dat,
				ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
			if (retval < 0) {
				pr_err("%s: Fail read host mode"
					"r=%d\n", __func__, retval);
				goto _cyttsp4_set_hndshk_enable_exit;
			} else if (cmd_dat[0] & CY_CMD_RDY_BIT) {
				break;
			} else {
				/* not our interrupt */
				ts->cmd_rdy = false;
			}
		}
	}

	if (tries >= 10000) {
		pr_err("%s: timeout waiting for cmd ready interrupt\n",
			__func__);
		retval = -ETIMEDOUT;
		goto _cyttsp4_set_hndshk_enable_exit;
	}

	if (cmd_dat[6] == cmd_dat[1])
		ts->hndshk_enabled = true;
	else
		ts->hndshk_enabled = false;

	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: check cmd ready hdshk=%d tries=%d r=%d"
		" cmd[]=%02X %02X %02X %02X %02X %02X %02X\n",
		__func__, (int)ts->hndshk_enabled, tries, retval,
		cmd_dat[0], cmd_dat[1], cmd_dat[2], cmd_dat[3],
		cmd_dat[4], cmd_dat[5], cmd_dat[6]);

_cyttsp4_set_hndshk_enable_exit:
	ts->power_state = tmp_state;
	cyttsp4_pr_state(ts);
_cyttsp4_set_hndshk_enable_tma400_exit:
	return retval;
}

static int _cyttsp4_set_operational_mode(struct cyttsp4 *ts)
{
	int retval = 0;
	int tries = 0;
	int tmp_state = 0;
	u8 cmd = 0;

	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

	cmd = CY_OPERATE_MODE + CY_MODE_CHANGE;

	tmp_state = ts->power_state;
	ts->power_state = CY_CMD_STATE;
	cyttsp4_pr_state(ts);
	ts->cmd_rdy = false;
	retval = cyttsp4_write_block_data(ts, CY_REG_BASE,
		sizeof(cmd), &cmd,
		ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
	if (retval) {
		pr_err("%s: Error writing hst_mode reg (ret=%d)\n",
			__func__, retval);
		return retval;
	}

	tries = 0;
	cmd = 0;
	while (tries++ < 10000) {
		if (mutex_is_locked(&ts->data_lock)) {
			mutex_unlock(&ts->data_lock);
			udelay(1000);
			mutex_lock(&ts->data_lock);
		} else {
			udelay(1000);
		}

		if (ts->cmd_rdy) {
			retval = cyttsp4_read_block_data(ts, CY_REG_BASE,
				sizeof(cmd), &cmd,
				ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
			if (retval < 0) {
				pr_err("%s: fail read host mode"
					"r=%d\n", __func__, retval);
				goto _cyttsp4_set_operational_mode_exit;
			} else if (!(cmd & CY_MODE_CHANGE)) {
				break;
			} else {
				/* not our interrupt */
				ts->cmd_rdy = false;
			}
		}
	}

	if (tries >= 10000) {
		/* try another read in case we missed our interrupt */
		retval = cyttsp4_read_block_data(ts, CY_REG_BASE,
			sizeof(cmd), &cmd,
			ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
		if (retval < 0) {
			pr_err("%s: fail read host mode after time out"
				"r=%d\n", __func__, retval);
			goto _cyttsp4_set_operational_mode_exit;
		}
		if (cmd & CY_MODE_CHANGE) {
			/* the device did not clear the command change bit */
			pr_err("%s: timeout waiting for op mode"
				" ready interrupt\n", __func__);
			retval = -ETIMEDOUT;
			goto _cyttsp4_set_operational_mode_exit;
		}
	}

	if (cmd != CY_OPERATE_MODE) {
		pr_err("%s: failed to switch to operational mode\n", __func__);
		retval = -EIO;
	} else
		ts->current_mode = CY_MODE_OPERATIONAL;

	cyttsp4_dbg(ts, CY_DBG_LVL_1,
		"%s: check op ready tries=%d ret=%d host_mode=%02X\n",
		__func__, tries, retval, cmd);

_cyttsp4_set_operational_mode_exit:
	ts->power_state = tmp_state;
	cyttsp4_pr_state(ts);
	return retval;
}

static int _cyttsp4_set_sysinfo_mode(struct cyttsp4 *ts)
{
	int retval = 0;
	int tries = 0;
	int tmp_state = 0;
	u8 cmd = 0;

	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

	cmd = CY_SYSINFO_MODE + CY_MODE_CHANGE;

	tmp_state = ts->power_state;
	ts->power_state = CY_CMD_STATE;
	cyttsp4_pr_state(ts);
	ts->cmd_rdy = false;
	retval = cyttsp4_write_block_data(ts, CY_REG_BASE,
		sizeof(cmd), &cmd,
		ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
	if (retval) {
		pr_err("%s: Error writing hst_mode reg (ret=%d)\n",
			__func__, retval);
		return retval;
	}

	tries = 0;
	cmd = 0;
	while (tries++ < 10000) {
		if (mutex_is_locked(&ts->data_lock)) {
			mutex_unlock(&ts->data_lock);
			udelay(1000);
			mutex_lock(&ts->data_lock);
		} else {
			udelay(1000);
		}

		if (ts->cmd_rdy) {
			retval = cyttsp4_read_block_data(ts, CY_REG_BASE,
				sizeof(cmd), &cmd,
				ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
			if (retval < 0) {
				pr_err("%s: fail read host mode"
					"r=%d\n", __func__, retval);
				goto _cyttsp4_set_sysinfo_mode_exit;
			} else if (!(cmd & CY_MODE_CHANGE)) {
				break;
			} else {
				/* not our interrupt */
				ts->cmd_rdy = false;
			}
		}
	}

	if (tries >= 10000) {
		/* try another read in case we missed our interrupt */
		retval = cyttsp4_read_block_data(ts, CY_REG_BASE,
			sizeof(cmd), &cmd,
			ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
		if (retval < 0) {
			pr_err("%s: fail read host mode after time out"
				"r=%d\n", __func__, retval);
			goto _cyttsp4_set_sysinfo_mode_exit;
		}
		if (cmd & CY_MODE_CHANGE) {
			pr_err("%s: timeout waiting for sysinfo mode"
				" ready interrupt\n", __func__);
			retval = -ETIMEDOUT;
			goto _cyttsp4_set_sysinfo_mode_exit;
		}
	}

	if (cmd != CY_SYSINFO_MODE) {
		pr_err("%s: failed to switch to sysinfo mode\n", __func__);
		retval = -EIO;
	} else
		ts->current_mode = CY_MODE_SYSINFO;

	cyttsp4_dbg(ts, CY_DBG_LVL_1,
		"%s: check sysinfo ready tries=%d ret=%d host_mode=%02X\n",
		__func__, tries, retval, cmd);

_cyttsp4_set_sysinfo_mode_exit:
	ts->power_state = tmp_state;
	cyttsp4_pr_state(ts);
	return retval;
}

static int _cyttsp4_set_config_mode(struct cyttsp4 *ts)
{
	int retval = 0;
	int tries = 0;
	int tmp_state = 0;
	u8 cmd = 0;

	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

	cmd = CY_CONFIG_MODE + CY_MODE_CHANGE;

	tmp_state = ts->power_state;
	ts->power_state = CY_CMD_STATE;
	cyttsp4_pr_state(ts);
	ts->cmd_rdy = false;
	retval = cyttsp4_write_block_data(ts, CY_REG_BASE,
		sizeof(cmd), &cmd,
		ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
	if (retval) {
		pr_err("%s: Error writing hst_mode reg (ret=%d)\n",
			__func__, retval);
		return retval;
	}

	tries = 0;
	cmd = 0;
	while (tries++ < 10000) {
		if (mutex_is_locked(&ts->data_lock)) {
			mutex_unlock(&ts->data_lock);
			udelay(1000);
			mutex_lock(&ts->data_lock);
		} else {
			udelay(1000);
		}

		if (ts->cmd_rdy) {
			retval = cyttsp4_read_block_data(ts, CY_REG_BASE,
				sizeof(cmd), &cmd,
				ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
			if (retval < 0) {
				pr_err("%s: fail read host mode"
					"r=%d\n", __func__, retval);
				goto _cyttsp4_set_config_mode_exit;
			} else if (!(cmd & CY_MODE_CHANGE)) {
				break;
			} else {
				/* not our interrupt */
				ts->cmd_rdy = false;
			}
		}
	}

	if (tries >= 10000) {
		/* try another read in case we missed our interrupt */
		retval = cyttsp4_read_block_data(ts, CY_REG_BASE,
			sizeof(cmd), &cmd,
			ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
		if (retval < 0) {
			pr_err("%s: fail read host mode after time out"
				"r=%d\n", __func__, retval);
			goto _cyttsp4_set_config_mode_exit;
		}
		if (cmd & CY_MODE_CHANGE) {
			pr_err("%s: timeout waiting for config mode"
				" ready interrupt\n", __func__);
			retval = -ETIMEDOUT;
			goto _cyttsp4_set_config_mode_exit;
		}
	}

	if (cmd != CY_CONFIG_MODE) {
		pr_err("%s: failed to switch to config mode\n", __func__);
		retval = -EIO;
	} else
		ts->current_mode = CY_MODE_CONFIG;

	cyttsp4_dbg(ts, CY_DBG_LVL_1,
		"%s: check config ready tries=%d ret=%d host_mode=%02X\n",
		__func__, tries, retval, cmd);

_cyttsp4_set_config_mode_exit:
	ts->power_state = tmp_state;
	cyttsp4_pr_state(ts);
	return retval;
}

static int _cyttsp4_write_config_block(struct cyttsp4 *ts, u8 blockid,
	const u8 *pdata, size_t ndata, u8 crc_h, u8 crc_l, const char *name)
{
	int retval = 0;
	uint8_t *buf = NULL;
	size_t buf_size;
	int tmp_state = 0;
	int tries = 0;
	uint8_t response[2] = {0x00, 0x00};

	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

	tmp_state = ts->power_state;
	ts->power_state = CY_CMD_STATE;
	cyttsp4_pr_state(ts);

	ts->cmd_rdy = false;

	/* pre-amble (10) + data (122) + crc (2) + key (8) */
	buf_size = sizeof(uint8_t) * 142;
	buf = kzalloc(buf_size, GFP_KERNEL);
	if (buf == NULL) {
		pr_err("%s: Failed to allocate buffer for %s\n",
			__func__, name);
		retval = -ENOMEM;
		goto _cyttsp4_write_config_block_exit;
	}

	if (pdata == NULL) {
		pr_err("%s: bad data pointer\n", __func__);
		retval = -ENXIO;
		goto _cyttsp4_write_config_block_exit;
	}

	if (ndata > 122) {
		pr_err("%s: %s is too large n=%d size=%d\n",
			__func__, name, ndata, 122);
		retval = -EOVERFLOW;
		goto _cyttsp4_write_config_block_exit;
	}

	/* Set command bytes */
	buf[0] = 0x04; /* cmd */
	buf[1] = 0x00; /* row offset high */
	buf[2] = 0x00; /* row offset low */
	buf[3] = 0x00; /* write block length high */
	buf[4] = 0x80; /* write block length low */
	buf[5] = blockid; /* write block id */
	buf[6] = 0x00; /* num of config bytes + 4 high */
	buf[7] = 0x7E; /* num of config bytes + 4 low */
	buf[8] = 0x00; /* max block size w/o crc high */
	buf[9] = 0x7E; /* max block size w/o crc low */

	/* Copy platform data */
	memcpy(&(buf[10]), pdata, ndata);

	/* Copy block CRC */
	buf[132] = crc_h;
	buf[133] = crc_l;

	/* Set key bytes */
	buf[134] = 0x45;
	buf[135] = 0x63;
	buf[136] = 0x36;
	buf[137] = 0x6F;
	buf[138] = 0x34;
	buf[139] = 0x38;
	buf[140] = 0x73;
	buf[141] = 0x77;

	/* Write config block */
	_cyttsp4_pr_buf(ts, buf, buf_size, name);

	retval = cyttsp4_write_block_data(ts, 0x03, 141, &(buf[1]),
		ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
	if (retval < 0) {
		pr_err("%s: Failed to write config %s r=%d\n",
			__func__, name, retval);
		goto _cyttsp4_write_config_block_exit;
	}

	/* Write command */
	retval = cyttsp4_write_block_data(ts, 0x02, 1, &(buf[0]),
		ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
	if (retval < 0) {
		pr_err("%s: Failed to write config command r=%d\n",
			__func__, retval);
		goto _cyttsp4_write_config_block_exit;
	}

	/* Wait for cmd rdy interrupt */
	while (tries++ < 10000) {
		if (mutex_is_locked(&ts->data_lock)) {
			mutex_unlock(&ts->data_lock);
			udelay(1000);
			mutex_lock(&ts->data_lock);
		} else {
			udelay(1000);
		}

		if (ts->cmd_rdy) {
			memset(response, 0, sizeof(response));
			retval = cyttsp4_read_block_data(ts, ts->si_ofs.cmd_ofs,
				sizeof(response), &(response[0]),
				ts->platform_data->addr[CY_TCH_ADDR_OFS],
				true);
			if (retval < 0) {
				pr_err("%s: fail read cmd status"
					" r=%d\n", __func__, retval);
				goto _cyttsp4_write_config_block_exit;
			} else if (response[0] & CY_CMD_RDY_BIT) {
				break;
			} else {
				/* not our interrupt */
				ts->cmd_rdy = false;
			}
		}
	}

	if (tries >= 10000) {
		pr_err("%s: command write timeout\n", __func__);
		retval = -ETIMEDOUT;
		goto _cyttsp4_write_config_block_exit;
	}

	if (response[1] != 0x00) {
		pr_err("%s: Write config block command failed"
			" response=%02X %02X tries=%d\n",
			__func__, response[0], response[1], tries);
		retval = -EIO;
	}

_cyttsp4_write_config_block_exit:
	kfree(buf);
	ts->power_state = tmp_state;
	cyttsp4_pr_state(ts);
	return retval;
}

#ifdef CONFIG_TOUCHSCREEN_DEBUG
static int _cyttsp4_read_config_block(struct cyttsp4 *ts, u8 blockid,
	u8 *pdata, size_t ndata, const char *name)
{
	int retval = 0;
	int tmp_state = 0;
	int tries = 0;
	u8 cmd[CY_NUM_DAT+1];

	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

	tmp_state = ts->power_state;
	ts->power_state = CY_CMD_STATE;
	cyttsp4_pr_state(ts);


	/* Set command bytes */
	cmd[0] = 0x03; /* cmd */
	cmd[1] = 0x00; /* row offset high */
	cmd[2] = 0x00; /* row offset low */
	cmd[3] = ndata / 256; /* write block length high */
	cmd[4] = ndata % 256; /* write block length low */
	cmd[5] = blockid; /* read block id */
	cmd[6] = 0x00; /* blank fill */

	/* Write config block */
	_cyttsp4_pr_buf(ts, cmd, sizeof(cmd), name);

	/* write the read config block command */
	ts->cmd_rdy = false;
	retval = cyttsp4_write_block_data(ts, ts->si_ofs.cmd_ofs,
		sizeof(cmd), cmd,
		ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
	if (retval < 0) {
		pr_err("%s: Failed to write read config command%s r=%d\n",
			__func__, name, retval);
		goto _cyttsp4_read_config_block_exit;
	}

	/* Wait for cmd rdy interrupt */
	tries = 0;
	while (tries++ < 10000) {
		if (mutex_is_locked(&ts->data_lock)) {
			mutex_unlock(&ts->data_lock);
			udelay(1000);
			mutex_lock(&ts->data_lock);
		} else {
			udelay(1000);
		}

		if (ts->cmd_rdy) {
			memset(pdata, 0, ndata);
			retval = cyttsp4_read_block_data(ts, ts->si_ofs.cmd_ofs,
				ndata, pdata,
				ts->platform_data->addr[CY_TCH_ADDR_OFS],
				true);
			if (retval < 0) {
				pr_err("%s: fail read cmd status"
					" r=%d\n", __func__, retval);
				goto _cyttsp4_read_config_block_exit;
			} else if (pdata[0] & CY_CMD_RDY_BIT) {
				break;
			} else {
				/* not our interrupt */
				ts->cmd_rdy = false;
			}
		}
	}

	if (tries >= 10000) {
		pr_err("%s: command write timeout\n", __func__);
		retval = -ETIMEDOUT;
		goto _cyttsp4_read_config_block_exit;
	}

	/* write the returned raw read config block data */
	_cyttsp4_pr_buf(ts, pdata, ndata, name);

	if (pdata[1] != 0x00) {
		pr_err("%s: Read config block command failed"
			" response=%02X %02X tries=%d\n",
			__func__, pdata[0], pdata[1], tries);
		retval = -EIO;
	}

_cyttsp4_read_config_block_exit:
	ts->power_state = tmp_state;
	cyttsp4_pr_state(ts);
	return retval;
}
#endif

static int _cyttsp4_set_op_params(struct cyttsp4 *ts, u8 crc_h, u8 crc_l)
{
	int retval = 0;

	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

	if (ts->platform_data->sett[CY_IC_GRPNUM_TCH_PARM_VAL] == NULL) {
		pr_err("%s: Missing Platform Touch Parameter"
			" values table\n", __func__);
		retval = -ENXIO;
		goto _cyttsp4_set_op_params_exit;
	}

	if ((ts->platform_data->sett
		[CY_IC_GRPNUM_TCH_PARM_VAL]->data == NULL) ||
		(ts->platform_data->sett
		[CY_IC_GRPNUM_TCH_PARM_VAL]->size == 0)) {
		pr_err("%s: Missing Platform Touch Parameter"
			" values table data\n", __func__);
		retval = -ENXIO;
		goto _cyttsp4_set_op_params_exit;
	}

	/* Change to Config Mode */
	retval = _cyttsp4_set_config_mode(ts);
	if (retval < 0) {
		pr_err("%s: Failed to switch to config mode"
			" for touch params\n", __func__);
		goto _cyttsp4_set_op_params_exit;
	}
	retval = _cyttsp4_write_config_block(ts, CY_TCH_PARM_BLKID,
		ts->platform_data->sett[CY_IC_GRPNUM_TCH_PARM_VAL]->data,
		ts->platform_data->sett[CY_IC_GRPNUM_TCH_PARM_VAL]->size,
		crc_h, crc_l, "platform_touch_param_data");

_cyttsp4_set_op_params_exit:
	return retval;
}

static int _cyttsp4_set_data_block(struct cyttsp4 *ts, u8 blkid, u8 *pdata,
	size_t ndata, const char *name, bool force, bool *data_updated)
{
	u8 data_crc[2];
	u8 ic_crc[2];
	int retval = 0;

	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

	memset(data_crc, 0, sizeof(data_crc));
	memset(ic_crc, 0, sizeof(ic_crc));
	*data_updated = false;

	_cyttsp4_pr_buf(ts, pdata, ndata, name);

	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: calc %s crc\n", __func__, name);
	retval = _cyttsp4_calc_data_crc(ts, ndata, pdata,
		&data_crc[0], &data_crc[1],
		name);
	if (retval < 0) {
		pr_err("%s: fail calc crc for %s (0x%02X%02X) r=%d\n",
			__func__, name,
			data_crc[0], data_crc[1],
			retval);
		goto _cyttsp_set_data_block_exit;
	}

	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: get ic %s crc\n", __func__, name);
	retval = _cyttsp4_set_operational_mode(ts);
	if (retval < 0) {
		pr_err("%s: Failed to switch to operational mode\n", __func__);
		goto _cyttsp_set_data_block_exit;
	}
	retval = _cyttsp4_get_ic_crc(ts, blkid,
		&ic_crc[0], &ic_crc[1]);
	if (retval < 0) {
		pr_err("%s: fail get ic crc for %s (0x%02X%02X) r=%d\n",
			__func__, name,
			ic_crc[0], ic_crc[1],
			retval);
		goto _cyttsp_set_data_block_exit;
	}

	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: %s calc_crc=0x%02X%02X ic_crc=0x%02X%02X\n",
		__func__, name,
		data_crc[0], data_crc[1],
		ic_crc[0], ic_crc[1]);
	if ((data_crc[0] != ic_crc[0]) || (data_crc[1] != ic_crc[1]) || force) {
		/* Change to Config Mode */
		retval = _cyttsp4_set_config_mode(ts);
		if (retval < 0) {
			pr_err("%s: Failed to switch to config mode"
				" for sysinfo regs\n", __func__);
			goto _cyttsp_set_data_block_exit;
		}
		retval = _cyttsp4_write_config_block(ts, blkid, pdata,
			ndata, data_crc[0], data_crc[1], name);
		if (retval < 0) {
			pr_err("%s: fail write %s config block r=%d\n",
				__func__, name, retval);
			goto _cyttsp_set_data_block_exit;
		}

		cyttsp4_dbg(ts, CY_DBG_LVL_3,
			"%s: write %s config block ok\n", __func__, name);
		*data_updated = true;
	}

_cyttsp_set_data_block_exit:
	return retval;
}

static int _cyttsp4_set_sysinfo_regs(struct cyttsp4 *ts, bool *updated)
{
	bool ddata_updated = false;
	bool mdata_updated = false;
	size_t num_data;
	u8 *pdata;
	int retval = 0;

	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

	pdata = kzalloc(CY_NUM_MDATA, GFP_KERNEL);
	if (pdata == NULL) {
		pr_err("%s: fail allocate set sysinfo regs buffer\n", __func__);
		retval = -ENOMEM;
		goto _cyttsp4_set_sysinfo_regs_err;
	}

	/* check for missing DDATA */
	if (ts->platform_data->sett[CY_IC_GRPNUM_DDATA_REC] == NULL) {
		cyttsp4_dbg(ts, CY_DBG_LVL_3,
			"%s: No platform_ddata table\n", __func__);
		cyttsp4_dbg(ts, CY_DBG_LVL_3,
			"%s: Use a zero filled array to compare with device\n",
			__func__);
		goto _cyttsp4_set_sysinfo_regs_set_ddata_block;
	}
	if ((ts->platform_data->sett[CY_IC_GRPNUM_DDATA_REC]->data == NULL) ||
		(ts->platform_data->sett[CY_IC_GRPNUM_DDATA_REC]->size == 0)) {
		cyttsp4_dbg(ts, CY_DBG_LVL_3,
			"%s: No platform_ddata table data\n", __func__);
		cyttsp4_dbg(ts, CY_DBG_LVL_3,
			"%s: Use a zero filled array to compare with device\n",
			__func__);
		goto _cyttsp4_set_sysinfo_regs_set_ddata_block;
	}

	/* copy platform data design data to the device eeprom */
	num_data = ts->platform_data->sett
		[CY_IC_GRPNUM_DDATA_REC]->size < CY_NUM_DDATA ?
		ts->platform_data->sett
		[CY_IC_GRPNUM_DDATA_REC]->size : CY_NUM_DDATA;
	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: copy %d bytes from platform data to ddata array\n",
		__func__, num_data);
	memcpy(pdata, ts->platform_data->sett[CY_IC_GRPNUM_DDATA_REC]->data,
		num_data);

_cyttsp4_set_sysinfo_regs_set_ddata_block:
	/* set data block will check CRC match/nomatch */
	retval = _cyttsp4_set_data_block(ts, CY_DDATA_BLKID, pdata,
		CY_NUM_DDATA, "platform_ddata", false, &ddata_updated);
	if (retval < 0) {
		pr_err("%s: Fail while writing platform_ddata"
			" block to ic r=%d\n", __func__, retval);
	}

	/* check for missing MDATA */
	if (ts->platform_data->sett[CY_IC_GRPNUM_MDATA_REC] == NULL) {
		cyttsp4_dbg(ts, CY_DBG_LVL_3,
			"%s: No platform_mdata table\n", __func__);
		cyttsp4_dbg(ts, CY_DBG_LVL_3,
			"%s: Use a zero filled array to compare with device\n",
			__func__);
		goto _cyttsp4_set_sysinfo_regs_set_mdata_block;
	}
	if ((ts->platform_data->sett[CY_IC_GRPNUM_MDATA_REC]->data == NULL) ||
		(ts->platform_data->sett[CY_IC_GRPNUM_MDATA_REC]->size == 0)) {
		cyttsp4_dbg(ts, CY_DBG_LVL_3,
			"%s: No platform_mdata table data\n", __func__);
		cyttsp4_dbg(ts, CY_DBG_LVL_3,
			"%s: Use a zero filled array to compare with device\n",
			__func__);
		goto _cyttsp4_set_sysinfo_regs_set_mdata_block;
	}

	/* copy platform manufacturing data to the device eeprom */
	num_data = ts->platform_data->sett
		[CY_IC_GRPNUM_MDATA_REC]->size < CY_NUM_MDATA ?
		ts->platform_data->sett
		[CY_IC_GRPNUM_MDATA_REC]->size : CY_NUM_MDATA;
	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: copy %d bytes from platform data to mdata array\n",
		__func__, num_data);
	memcpy(pdata, ts->platform_data->sett[CY_IC_GRPNUM_MDATA_REC]->data,
		num_data);

_cyttsp4_set_sysinfo_regs_set_mdata_block:
	/* set data block will check CRC match/nomatch */
	retval = _cyttsp4_set_data_block(ts, CY_MDATA_BLKID, pdata,
		CY_NUM_MDATA, "platform_mdata", false, &mdata_updated);
	if (retval < 0) {
		pr_err("%s: Fail while writing platform_mdata"
			" block to ic r=%d\n", __func__, retval);
	}

	kfree(pdata);
_cyttsp4_set_sysinfo_regs_err:
	*updated = ddata_updated || mdata_updated;
	return retval;
}

static int _cyttsp4_bits_2_bytes(struct cyttsp4 *ts, int nbits, int *max)
{
	int nbytes;

	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

	*max = 1 << nbits;

	for (nbytes = 0; nbits > 0;) {
		cyttsp4_dbg(ts, CY_DBG_LVL_3,
			"%s: nbytes=%d nbits=%d\n", __func__, nbytes, nbits);
		nbytes++;
		if (nbits > 8)
			nbits -= 8;
		else
			nbits = 0;
		cyttsp4_dbg(ts, CY_DBG_LVL_3,
			"%s: nbytes=%d nbits=%d\n", __func__, nbytes, nbits);
	}

	return nbytes;
}

static int _cyttsp4_get_sysinfo_regs(struct cyttsp4 *ts)
{
	int retval = 0;

	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

	/* get the sysinfo data offsets */
	retval = cyttsp4_read_block_data(ts, CY_REG_BASE,
		sizeof(ts->sysinfo_data), &(ts->sysinfo_data),
		ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
	if (retval < 0) {
		pr_err("%s: fail read sysinfo data offsets r=%d\n",
			__func__, retval);
		goto _cyttsp4_get_sysinfo_regs;
	} else {
		/* Print sysinfo data offsets */
		_cyttsp4_pr_buf(ts, (u8 *)&ts->sysinfo_data,
			sizeof(ts->sysinfo_data), "sysinfo_data_offsets");

		/* convert sysinfo data offset bytes into integers */
		ts->si_ofs.map_sz = (ts->sysinfo_data.map_szh * 256) +
			ts->sysinfo_data.map_szl;
		ts->si_ofs.cydata_ofs = (ts->sysinfo_data.cydata_ofsh * 256) +
			ts->sysinfo_data.cydata_ofsl;
		ts->si_ofs.test_ofs = (ts->sysinfo_data.test_ofsh * 256) +
			ts->sysinfo_data.test_ofsl;
		ts->si_ofs.pcfg_ofs = (ts->sysinfo_data.pcfg_ofsh * 256) +
			ts->sysinfo_data.pcfg_ofsl;
		ts->si_ofs.opcfg_ofs = (ts->sysinfo_data.opcfg_ofsh * 256) +
			ts->sysinfo_data.opcfg_ofsl;
		ts->si_ofs.ddata_ofs = (ts->sysinfo_data.ddata_ofsh * 256) +
			ts->sysinfo_data.ddata_ofsl;
		ts->si_ofs.mdata_ofs = (ts->sysinfo_data.mdata_ofsh * 256) +
			ts->sysinfo_data.mdata_ofsl;
	}

	/* get the sysinfo cydata */
	ts->si_ofs.cydata_size = ts->si_ofs.test_ofs - ts->si_ofs.cydata_ofs;
	ts->sysinfo_ptr.cydata = kzalloc(ts->si_ofs.cydata_size, GFP_KERNEL);
	if (ts->sysinfo_ptr.cydata == NULL) {
		retval = -ENOMEM;
		pr_err("%s: fail alloc cydata memory r=%d\n",
			__func__, retval);
		goto _cyttsp4_get_sysinfo_regs;
	} else {
		memset(ts->sysinfo_ptr.cydata, 0, ts->si_ofs.cydata_size);
		retval = cyttsp4_read_block_data(ts, ts->si_ofs.cydata_ofs,
			ts->si_ofs.cydata_size, ts->sysinfo_ptr.cydata,
			ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
		if (retval < 0) {
			pr_err("%s: fail read cydata r=%d\n",
				__func__, retval);
			goto _cyttsp4_get_sysinfo_regs;
		}
		/* Print sysinfo cydata */
		_cyttsp4_pr_buf(ts, (u8 *)ts->sysinfo_ptr.cydata,
			ts->si_ofs.cydata_size, "sysinfo_cydata");
	}
	/* get the sysinfo test data */
	ts->si_ofs.test_size = ts->si_ofs.pcfg_ofs - ts->si_ofs.test_ofs;
	ts->sysinfo_ptr.test = kzalloc(ts->si_ofs.test_size, GFP_KERNEL);
	if (ts->sysinfo_ptr.test == NULL) {
		retval = -ENOMEM;
		pr_err("%s: fail alloc test memory r=%d\n",
			__func__, retval);
		goto _cyttsp4_get_sysinfo_regs;
	} else {
		memset(ts->sysinfo_ptr.test, 0, ts->si_ofs.test_size);
		retval = cyttsp4_read_block_data(ts, ts->si_ofs.test_ofs,
			ts->si_ofs.test_size, ts->sysinfo_ptr.test,
			ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
		if (retval < 0) {
			pr_err("%s: fail read test data r=%d\n",
				__func__, retval);
			goto _cyttsp4_get_sysinfo_regs;
		}
		/* Print sysinfo test data */
		_cyttsp4_pr_buf(ts, (u8 *)ts->sysinfo_ptr.test,
			ts->si_ofs.test_size, "sysinfo_test_data");
	}
	/* get the sysinfo pcfg data */
	ts->si_ofs.pcfg_size = ts->si_ofs.opcfg_ofs - ts->si_ofs.pcfg_ofs;
	ts->sysinfo_ptr.pcfg = kzalloc(ts->si_ofs.pcfg_size, GFP_KERNEL);
	if (ts->sysinfo_ptr.pcfg == NULL) {
		retval = -ENOMEM;
		pr_err("%s: fail alloc pcfg memory r=%d\n",
			__func__, retval);
		goto _cyttsp4_get_sysinfo_regs;
	} else {
		memset(ts->sysinfo_ptr.pcfg, 0, ts->si_ofs.pcfg_size);
		retval = cyttsp4_read_block_data(ts, ts->si_ofs.pcfg_ofs,
			ts->si_ofs.pcfg_size, ts->sysinfo_ptr.pcfg,
			ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
		if (retval < 0) {
			pr_err("%s: fail read pcfg data r=%d\n",
				__func__, retval);
			goto _cyttsp4_get_sysinfo_regs;
		}
		/* Print sysinfo pcfg data */
		_cyttsp4_pr_buf(ts, (u8 *)ts->sysinfo_ptr.pcfg,
			ts->si_ofs.pcfg_size, "sysinfo_pcfg_data");
	}
	/* get the sysinfo opcfg data */
	ts->si_ofs.cmd_ofs = 0;
	ts->si_ofs.rep_ofs = 0;
	ts->si_ofs.rep_sz = 0;
	ts->si_ofs.tt_stat_ofs = 0;
	ts->si_ofs.max_tchs = 0;
	ts->si_ofs.tch_rec_siz = 0;
	ts->si_ofs.tch_rec_x_ofs = 0;	/* x position */
	ts->si_ofs.tch_rec_x_size = 0;
	ts->si_ofs.tch_rec_y_ofs = 0;	/* y position */
	ts->si_ofs.tch_rec_y_size = 0;
	ts->si_ofs.tch_rec_p_ofs = 0;	/* pressure */
	ts->si_ofs.tch_rec_p_size = 0;
	ts->si_ofs.tch_rec_t_ofs = 0;	/* track id */
	ts->si_ofs.tch_rec_t_size = 0;
	ts->si_ofs.tch_rec_e_ofs = 0;	/* event id */
	ts->si_ofs.tch_rec_e_size = 0;
	ts->si_ofs.tch_rec_o_ofs = 0;	/* object type */
	ts->si_ofs.tch_rec_o_size = 0;
	ts->si_ofs.tch_rec_w_ofs = 0;	/* size */
	ts->si_ofs.tch_rec_w_size = 0;
	ts->si_ofs.mode_size = 0;
	ts->si_ofs.data_size = 0;
	ts->si_ofs.opcfg_size = ts->si_ofs.ddata_ofs - ts->si_ofs.opcfg_ofs;
	ts->sysinfo_ptr.opcfg = kzalloc(ts->si_ofs.opcfg_size, GFP_KERNEL);
	if (ts->sysinfo_ptr.opcfg == NULL) {
		retval = -ENOMEM;
		pr_err("%s: fail alloc opcfg memory r=%d\n",
			__func__, retval);
		goto _cyttsp4_get_sysinfo_regs;
	} else {
		memset(ts->sysinfo_ptr.opcfg, 0, ts->si_ofs.opcfg_size);
		retval = cyttsp4_read_block_data(ts, ts->si_ofs.opcfg_ofs,
			ts->si_ofs.opcfg_size, ts->sysinfo_ptr.opcfg,
			ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
		if (retval < 0) {
			pr_err("%s: fail read opcfg data r=%d\n",
				__func__, retval);
			goto _cyttsp4_get_sysinfo_regs;
		}
		ts->si_ofs.cmd_ofs = ts->sysinfo_ptr.opcfg->cmd_ofs;
		ts->si_ofs.rep_ofs = ts->sysinfo_ptr.opcfg->rep_ofs;
		ts->si_ofs.rep_sz = (ts->sysinfo_ptr.opcfg->rep_szh * 256) +
			ts->sysinfo_ptr.opcfg->rep_szl;
		ts->si_ofs.tt_stat_ofs = ts->sysinfo_ptr.opcfg->tt_stat_ofs;
		ts->si_ofs.obj_cfg0 = ts->sysinfo_ptr.opcfg->obj_cfg0;
		ts->si_ofs.max_tchs = ts->sysinfo_ptr.opcfg->max_tchs;
		ts->si_ofs.tch_rec_siz = ts->sysinfo_ptr.opcfg->tch_rec_siz;
		ts->si_ofs.tch_rec_x_ofs = ts->sysinfo_ptr.opcfg->tch_rec_0;
		ts->si_ofs.tch_rec_x_size =
			_cyttsp4_bits_2_bytes(ts,
			ts->sysinfo_ptr.opcfg->tch_rec_1,
			 &ts->si_ofs.tch_rec_x_max);
		ts->si_ofs.tch_rec_y_ofs = ts->sysinfo_ptr.opcfg->tch_rec_2;
		ts->si_ofs.tch_rec_y_size =
			_cyttsp4_bits_2_bytes(ts,
			ts->sysinfo_ptr.opcfg->tch_rec_3,
			 &ts->si_ofs.tch_rec_y_max);
		ts->si_ofs.tch_rec_p_ofs = ts->sysinfo_ptr.opcfg->tch_rec_4;
		ts->si_ofs.tch_rec_p_size =
			_cyttsp4_bits_2_bytes(ts,
			ts->sysinfo_ptr.opcfg->tch_rec_5,
			 &ts->si_ofs.tch_rec_p_max);
		ts->si_ofs.tch_rec_t_ofs = ts->sysinfo_ptr.opcfg->tch_rec_6;
		ts->si_ofs.tch_rec_t_size =
			_cyttsp4_bits_2_bytes(ts,
			ts->sysinfo_ptr.opcfg->tch_rec_7,
			 &ts->si_ofs.tch_rec_t_max);
		ts->si_ofs.tch_rec_e_ofs = ts->sysinfo_ptr.opcfg->tch_rec_8;
		ts->si_ofs.tch_rec_e_size =
			_cyttsp4_bits_2_bytes(ts,
			ts->sysinfo_ptr.opcfg->tch_rec_9,
			 &ts->si_ofs.tch_rec_e_max);
		ts->si_ofs.tch_rec_o_ofs = ts->sysinfo_ptr.opcfg->tch_rec_10;
		ts->si_ofs.tch_rec_o_size =
			_cyttsp4_bits_2_bytes(ts,
			ts->sysinfo_ptr.opcfg->tch_rec_11,
			 &ts->si_ofs.tch_rec_o_max);
		ts->si_ofs.tch_rec_w_ofs = ts->sysinfo_ptr.opcfg->tch_rec_12;
		ts->si_ofs.tch_rec_w_size =
			_cyttsp4_bits_2_bytes(ts,
			ts->sysinfo_ptr.opcfg->tch_rec_13,
			 &ts->si_ofs.tch_rec_w_max);
		ts->si_ofs.mode_size = ts->si_ofs.tt_stat_ofs + 1;
		ts->si_ofs.data_size = ts->si_ofs.max_tchs *
			ts->sysinfo_ptr.opcfg->tch_rec_siz;
		/* Print sysinfo opcfg data */
		_cyttsp4_pr_buf(ts, (u8 *)ts->sysinfo_ptr.opcfg,
			ts->si_ofs.opcfg_size, "sysinfo_opcfg_data");
	}
	/* get the sysinfo ddata data */
	ts->si_ofs.ddata_size = ts->si_ofs.mdata_ofs - ts->si_ofs.ddata_ofs;
	ts->sysinfo_ptr.ddata = kzalloc(ts->si_ofs.ddata_size, GFP_KERNEL);
	if (ts->sysinfo_ptr.ddata == NULL) {
		retval = -ENOMEM;
		pr_err("%s: fail alloc ddata memory r=%d\n",
			__func__, retval);
		goto _cyttsp4_get_sysinfo_regs;
	} else {
		memset(ts->sysinfo_ptr.ddata, 0, ts->si_ofs.ddata_size);
		retval = cyttsp4_read_block_data(ts, ts->si_ofs.ddata_ofs,
			ts->si_ofs.ddata_size, ts->sysinfo_ptr.ddata,
			ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
		if (retval < 0) {
			pr_err("%s: fail read ddata data r=%d\n",
				__func__, retval);
			goto _cyttsp4_get_sysinfo_regs;
		}
		/* Print sysinfo ddata */
		_cyttsp4_pr_buf(ts, (u8 *)ts->sysinfo_ptr.ddata,
			ts->si_ofs.ddata_size, "sysinfo_ddata");
	}
	/* get the sysinfo mdata data */
	ts->si_ofs.mdata_size = ts->si_ofs.map_sz - ts->si_ofs.mdata_ofs;
	ts->sysinfo_ptr.mdata = kzalloc(ts->si_ofs.mdata_size, GFP_KERNEL);
	if (ts->sysinfo_ptr.mdata == NULL) {
		retval = -ENOMEM;
		pr_err("%s: fail alloc mdata memory r=%d\n",
			__func__, retval);
		goto _cyttsp4_get_sysinfo_regs;
	} else {
		memset(ts->sysinfo_ptr.mdata, 0, ts->si_ofs.mdata_size);
		retval = cyttsp4_read_block_data(ts, ts->si_ofs.mdata_ofs,
			ts->si_ofs.mdata_size, ts->sysinfo_ptr.mdata,
			ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
		if (retval < 0) {
			pr_err("%s: fail read mdata data r=%d\n",
				__func__, retval);
			goto _cyttsp4_get_sysinfo_regs;
		}
		/* Print sysinfo mdata */
		_cyttsp4_pr_buf(ts, (u8 *)ts->sysinfo_ptr.mdata,
			ts->si_ofs.mdata_size, "sysinfo_mdata");
	}

	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: cydata_ofs =%4d siz=%4d\n", __func__,
		ts->si_ofs.cydata_ofs, ts->si_ofs.cydata_size);
	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: test_ofs   =%4d siz=%4d\n", __func__,
		ts->si_ofs.test_ofs, ts->si_ofs.test_size);
	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: pcfg_ofs   =%4d siz=%4d\n", __func__,
		ts->si_ofs.pcfg_ofs, ts->si_ofs.pcfg_size);
	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: opcfg_ofs  =%4d siz=%4d\n", __func__,
		ts->si_ofs.opcfg_ofs, ts->si_ofs.opcfg_size);
	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: ddata_ofs  =%4d siz=%4d\n", __func__,
		ts->si_ofs.ddata_ofs, ts->si_ofs.ddata_size);
	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: mdata_ofs  =%4d siz=%4d\n", __func__,
		ts->si_ofs.mdata_ofs, ts->si_ofs.mdata_size);
	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: cmd_ofs    =%4d\n", __func__, ts->si_ofs.cmd_ofs);
	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: rep_ofs    =%4d\n", __func__, ts->si_ofs.rep_ofs);
	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: rep_sz     =%4d\n", __func__, ts->si_ofs.rep_sz);
	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: tt_stat_ofs=%4d\n", __func__, ts->si_ofs.tt_stat_ofs);
	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: tch_rec_siz=%4d\n", __func__, ts->si_ofs.tch_rec_siz);
	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: max_tchs   =%4d\n", __func__, ts->si_ofs.max_tchs);
	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: mode_siz   =%4d\n", __func__, ts->si_ofs.mode_size);
	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: data_siz   =%4d\n", __func__, ts->si_ofs.data_size);
	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: map_sz     =%4d\n", __func__, ts->si_ofs.map_sz);
	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: tch_rec_x_ofs=%2d\n", __func__, ts->si_ofs.tch_rec_x_ofs);
	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: tch_rec_x_siz=%2d\n", __func__, ts->si_ofs.tch_rec_x_size);
	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: tch_rec_x_max=%2d\n", __func__, ts->si_ofs.tch_rec_x_max);
	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: tch_rec_y_ofs=%2d\n", __func__, ts->si_ofs.tch_rec_y_ofs);
	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: tch_rec_y_siz=%2d\n", __func__, ts->si_ofs.tch_rec_y_size);
	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: tch_rec_y_max=%2d\n", __func__, ts->si_ofs.tch_rec_y_max);
	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: tch_rec_p_ofs=%2d\n", __func__, ts->si_ofs.tch_rec_p_ofs);
	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: tch_rec_p_siz=%2d\n", __func__, ts->si_ofs.tch_rec_p_size);
	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: tch_rec_p_max=%2d\n", __func__, ts->si_ofs.tch_rec_p_max);
	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: tch_rec_t_ofs=%2d\n", __func__, ts->si_ofs.tch_rec_t_ofs);
	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: tch_rec_t_siz=%2d\n", __func__, ts->si_ofs.tch_rec_t_size);
	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: tch_rec_t_max=%2d\n", __func__, ts->si_ofs.tch_rec_t_max);
	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: tch_rec_e_ofs=%2d\n", __func__, ts->si_ofs.tch_rec_e_ofs);
	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: tch_rec_e_siz=%2d\n", __func__, ts->si_ofs.tch_rec_e_size);
	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: tch_rec_e_max=%2d\n", __func__, ts->si_ofs.tch_rec_e_max);
	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: tch_rec_o_ofs=%2d\n", __func__, ts->si_ofs.tch_rec_o_ofs);
	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: tch_rec_o_siz=%2d\n", __func__, ts->si_ofs.tch_rec_o_size);
	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: tch_rec_o_max=%2d\n", __func__, ts->si_ofs.tch_rec_o_max);
	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: tch_rec_w_ofs=%2d\n", __func__, ts->si_ofs.tch_rec_w_ofs);
	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: tch_rec_w_siz=%2d\n", __func__, ts->si_ofs.tch_rec_w_size);
	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: tch_rec_w_max=%2d\n", __func__, ts->si_ofs.tch_rec_w_max);

	if (ts->xy_mode == NULL)
		ts->xy_mode = kzalloc(ts->si_ofs.mode_size, GFP_KERNEL);
	if (ts->xy_data == NULL)
		ts->xy_data = kzalloc(ts->si_ofs.data_size, GFP_KERNEL);
	if ((ts->xy_mode == NULL) || (ts->xy_data == NULL)) {
		retval = -ENOMEM;
		pr_err("%s: fail memory alloc xy_mode=%p xy_data=%p\n",
			__func__, ts->xy_mode, ts->xy_data);
	}

_cyttsp4_get_sysinfo_regs:
	return retval;
}

#define CY_IRQ_DEASSERT	1
#define CY_IRQ_ASSERT	0
static void _cyttsp4_get_touch_axis(struct cyttsp4 *ts,
	int *axis, int size, int max, u8 *xy_data)
{
	int nbyte;
	int next;

	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

	for (nbyte = 0, *axis = 0, next = 0; nbyte < size; nbyte++) {
		cyttsp4_dbg(ts, CY_DBG_LVL_3,
			"%s: *axis=%02X(%d) size=%d max=%08X xy_data=%p"
			" xy_data[%d]=%02X(%d)\n",
			__func__, *axis, *axis, size, max, xy_data, next,
			xy_data[next], xy_data[next]);
		*axis = (*axis * 256) + xy_data[next++];
	}

	*axis &= max - 1;

	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: *axis=%02X(%d) size=%d max=%08X xy_data=%p"
		" xy_data[%d]=%02X(%d)\n",
		__func__, *axis, *axis, size, max, xy_data, next,
		xy_data[next], xy_data[next]);
}

static void _cyttsp4_get_touch(struct cyttsp4 *ts,
	struct cyttsp4_touch *touch, u8 *xy_data)
{

	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

	_cyttsp4_get_touch_axis(ts, &touch->x,
		ts->si_ofs.tch_rec_x_size, ts->si_ofs.tch_rec_x_max,
		xy_data + ts->si_ofs.tch_rec_x_ofs);
	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: get x=%08X(%d) rec_x_size=%d"
		" rec_x_ofs=%d xy_data+ofs=%p\n",
		__func__, touch->x, touch->x, ts->si_ofs.tch_rec_x_size,
		ts->si_ofs.tch_rec_x_ofs, xy_data + ts->si_ofs.tch_rec_x_ofs);
	_cyttsp4_get_touch_axis(ts, &touch->y,
		ts->si_ofs.tch_rec_y_size, ts->si_ofs.tch_rec_y_max,
		xy_data + ts->si_ofs.tch_rec_y_ofs);
	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: get y=%08X(%d) rec_y_size=%d"
		" rec_y_ofs=%d xy_data+ofs=%p\n",
		__func__, touch->y, touch->y, ts->si_ofs.tch_rec_y_size,
		ts->si_ofs.tch_rec_y_ofs, xy_data + ts->si_ofs.tch_rec_y_ofs);
	_cyttsp4_get_touch_axis(ts, &touch->p,
		ts->si_ofs.tch_rec_p_size, ts->si_ofs.tch_rec_p_max,
		xy_data + ts->si_ofs.tch_rec_p_ofs);
	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: get p=%08X(%d) rec_p_size=%d"
		" rec_p_ofs=%d xy_data+ofs=%p\n",
		__func__, touch->p, touch->p, ts->si_ofs.tch_rec_p_size,
		ts->si_ofs.tch_rec_p_ofs, xy_data + ts->si_ofs.tch_rec_p_ofs);
	_cyttsp4_get_touch_axis(ts, &touch->t,
		ts->si_ofs.tch_rec_t_size, ts->si_ofs.tch_rec_t_max,
		xy_data + ts->si_ofs.tch_rec_t_ofs);
	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: get t=%08X(%d) rec_t_size=%d"
		" rec_t_ofs=%d rec_t_max=%d xy_data+ofs=%p\n",
		__func__, touch->t, touch->t, ts->si_ofs.tch_rec_t_size,
		ts->si_ofs.tch_rec_t_ofs, ts->si_ofs.tch_rec_t_max,
		xy_data + ts->si_ofs.tch_rec_t_ofs);
	if (!(ts->flags & CY_FLAG_TMA400)) {
		_cyttsp4_get_touch_axis(ts, &touch->w,
			ts->si_ofs.tch_rec_w_size, ts->si_ofs.tch_rec_w_max,
			xy_data + ts->si_ofs.tch_rec_w_ofs);
		_cyttsp4_get_touch_axis(ts, &touch->e,
			ts->si_ofs.tch_rec_e_size, ts->si_ofs.tch_rec_e_max,
			xy_data + ts->si_ofs.tch_rec_e_ofs);
		_cyttsp4_get_touch_axis(ts, &touch->o,
			ts->si_ofs.tch_rec_o_size, ts->si_ofs.tch_rec_o_max,
			xy_data + ts->si_ofs.tch_rec_o_ofs);
	} else {
		touch->w = 0;
		touch->e = 0;
		touch->o = 0;
	}
}

/* read xy_data for all current touches */
#ifdef MULTI_TOUCH_PROTOCOL_B
static int _cyttsp4_xy_worker(struct cyttsp4 *ts)
{
	struct cyttsp4_touch touch;
	u8 num_cur_tch = 0;
	u8 hst_mode = 0;
	u8 rep_len = 0;
	u8 rep_stat = 0;
	u8 tt_stat = 0;
	int i;
	int signal;
	int retval;

	int id;
	static int num_pre_tch = 0;
	static struct cyttsp4_touch pre_touch[] = {
		/* {x, y, p, t, e, o, w} */
		{0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 1, 0, 0, 0},
		{0, 0, 0, 2, 0, 0, 0},
		{0, 0, 0, 3, 0, 0, 0},
		{0, 0, 0, 4, 0, 0, 0},
		{0, 0, 0, 5, 0, 0, 0},
		{0, 0, 0, 6, 0, 0, 0},
		{0, 0, 0, 7, 0, 0, 0},
		{0, 0, 0, 8, 0, 0, 0},
		{0, 0, 0, 9, 0, 0, 0},
	};


	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

	/*
	 * Get event data from CYTTSP device.
	 * The event data includes all data
	 * for all active touches.
	 */
	memset(ts->xy_mode, 0, ts->si_ofs.mode_size);
	retval = cyttsp4_read_block_data(ts, CY_REG_BASE,
					ts->si_ofs.mode_size, ts->xy_mode,
					ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
	if (retval < 0) {
		/* I2C bus failure implies bootloader running */
		pr_err("%s: read fail on mode regs r=%d\n",
			__func__, retval);
		queue_work(ts->cyttsp4_wq,
			&ts->cyttsp4_resume_startup_work);
		pr_err("%s: startup queued\n", __func__);
		retval = IRQ_HANDLED;
		goto _cyttsp4_xy_worker_exit;
	}

	hst_mode = ts->xy_mode[CY_REG_BASE];
	rep_len = ts->xy_mode[ts->si_ofs.rep_ofs];
	rep_stat = ts->xy_mode[ts->si_ofs.rep_ofs + 1];
	tt_stat = ts->xy_mode[ts->si_ofs.tt_stat_ofs];
	cyttsp4_dbg(ts, CY_DBG_LVL_1,
		"%s: hst_mode=%02X rep_len=%d rep_stat=%02X tt_stat=%02X\n",
		__func__, hst_mode, rep_len, rep_stat, tt_stat);

	if (rep_len == 0) {
		pr_err("%s: report length error rep_len=%d\n",
			__func__, rep_len);
		retval = -EIO;
		goto _cyttsp4_xy_worker_exit;
	}

	retval = cyttsp4_read_block_data(ts, ts->si_ofs.tt_stat_ofs + 1,
					rep_len, ts->xy_data,
					ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
	if (retval < 0) {
		pr_err("%s: read fail on touch regs r=%d\n",
			__func__, retval);
		goto _cyttsp4_xy_worker_exit;
	}

	/* print xy data */
	_cyttsp4_pr_buf(ts, ts->xy_data, rep_len, "xy_data");

	/* provide flow control handshake */
	if (cyttsp4_hndshk(ts, hst_mode)) {
		pr_err("%s: handshake fail on operational reg\n",
			__func__);
		ts->power_state = CY_IDLE_STATE;
		cyttsp4_pr_state(ts);
		retval = -EIO;
		goto _cyttsp4_xy_worker_exit;
	}

	/* if in System Information mode, then switch to operational mode */
	if (GET_HSTMODE(hst_mode) == GET_HSTMODE(CY_SYSINFO_MODE)) {
		pr_err("%s: Host mode detected in op state (hm=0x%02X)\n",
			__func__, hst_mode);
		retval = _cyttsp4_set_operational_mode(ts);
		if (retval < 0) {
			ts->power_state = CY_IDLE_STATE;
			cyttsp4_pr_state(ts);
			pr_err("%s: Fail set operational mode (r=%d)\n",
				__func__, retval);
		} else {
			ts->power_state = CY_ACTIVE_STATE;
			cyttsp4_pr_state(ts);
			cyttsp4_dbg(ts, CY_DBG_LVL_3,
				"%s: enable handshake\n", __func__);
			retval = _cyttsp4_hndshk_enable(ts);
			if (retval < 0) {
				pr_err("%s: fail enable handshake r=%d",
					__func__, retval);
			}
		}
		goto _cyttsp4_xy_worker_exit;
	}

	/* determine number of currently active touches */
	num_cur_tch = GET_NUM_TOUCHES(tt_stat);

	/* check for any error conditions */
	if (ts->power_state == CY_IDLE_STATE) {
		pr_err("%s: IDLE STATE detected\n", __func__);
		retval = 0;
		goto _cyttsp4_xy_worker_exit;
	} else if (IS_BAD_PKT(rep_stat)) {
		pr_err("%s: Invalid buffer detected\n", __func__);
		retval = 0;
		goto _cyttsp4_xy_worker_exit;
	} else if (IS_LARGE_AREA(tt_stat)) {
		/* terminate all active tracks */
		num_cur_tch = 0;
		pr_err("%s: Large area detected\n", __func__);
		retval = -EIO;
		goto _cyttsp4_xy_worker_exit;
	} else if (num_cur_tch > ts->si_ofs.max_tchs) {
		if (num_cur_tch == 0x1F) {
			/* terminate all active tracks */
			pr_err("%s: Num touch err detected (n=%d)\n",
				__func__, num_cur_tch);
			num_cur_tch = 0;
		} else {
			pr_err("%s: too many tch; set to max tch (n=%d c=%d)\n",
				__func__, num_cur_tch, CY_NUM_TCH_ID);
			num_cur_tch = CY_NUM_TCH_ID;
		}
	}

	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: num_cur_tch=%d\n", __func__, num_cur_tch);

	/* extract xy_data for all currently reported touches */
	if (num_pre_tch || num_cur_tch) {
		for (id = 0; id < MAX_TOUCH_NUM; id++) {
			for (i = 0; i < num_cur_tch; i++) {
				_cyttsp4_get_touch(ts, &touch,
						ts->xy_data + (i * ts->si_ofs.tch_rec_siz));
				if (touch.t == id)
					break;
			}
			if (i >= num_cur_tch) {
				if (pre_touch[id].p == 0)
					continue;
				pre_touch[id].p = 0;

				input_mt_slot(ts->input, id);
				input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, false);
				continue;
			}

			/* adjusting the coordinates */
			if (ts->adjust_enabled == true)
				_sh_tpsif_adjust_point(ts, &(touch.x), &(touch.y));

			/* report touch event */
			if (((touch.t == 0) &&
					(ts->platform_data->frmwrk->abs
						[(CY_ABS_ID_OST*CY_NUM_ABS_SET)+CY_MIN_OST]
						> 0)) ||
				(touch.t > ts->platform_data->frmwrk->abs
					[(CY_ABS_ID_OST*CY_NUM_ABS_SET)+CY_MAX_OST])) {
				pr_err("%s: touch=%d has bad track_id=%d"
					"max_id=%d\n",
					__func__, i, touch.t,
					ts->platform_data->frmwrk->abs
					[(CY_ABS_ID_OST*CY_NUM_ABS_SET)+
						CY_MAX_OST]);
			} else {
				/* use 0 based track id's */
				signal = ts->platform_data->frmwrk->abs
					[(CY_ABS_ID_OST*CY_NUM_ABS_SET)+0];
				if (signal != CY_IGNORE_VALUE) {
					if (ts->platform_data->frmwrk->abs
						[(CY_ABS_ID_OST*CY_NUM_ABS_SET)+
							CY_MIN_OST] > 0)
						input_mt_slot(ts->input, touch.t - 1);
					else
						input_mt_slot(ts->input, touch.t);
					input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, true);
				}

				signal = ts->platform_data->frmwrk->abs
					[(CY_ABS_X_OST*CY_NUM_ABS_SET)+0];
				if (signal != CY_IGNORE_VALUE)
					input_report_abs(ts->input,
							signal, touch.x);

				signal = ts->platform_data->frmwrk->abs
					[(CY_ABS_Y_OST*CY_NUM_ABS_SET)+0];
				if (ts->reverse_y_axis) {
					if (signal != CY_IGNORE_VALUE) {
						input_report_abs(ts->input,
								signal,
								ts->platform_data->frmwrk->abs[(CY_ABS_Y_OST*CY_NUM_ABS_SET)+CY_MAX_OST] +
								ts->platform_data->frmwrk->abs[(CY_ABS_Y_OST*CY_NUM_ABS_SET)+CY_MIN_OST] -
								touch.y);
					}
				} else {
					if (signal != CY_IGNORE_VALUE)
						input_report_abs(ts->input,
								signal, touch.y);
				}

				signal = ts->platform_data->frmwrk->abs
					[(CY_ABS_P_OST*CY_NUM_ABS_SET)+0];
				if (signal != CY_IGNORE_VALUE)
					input_report_abs(ts->input,
							signal, touch.p);

				if (!(ts->flags & CY_FLAG_TMA400)) {
					signal = ts->platform_data->frmwrk->abs
						[(CY_ABS_W_OST*CY_NUM_ABS_SET)
							+0];
					if (signal != CY_IGNORE_VALUE)
						input_report_abs(ts->input,
								signal, touch.w);
				}
			}
			cyttsp4_dbg(ts, CY_DBG_LVL_1,
				"%s: t-1=%d x=%04X y=%04X z=%02X"
				" w=%02X e=%02X o=%02X\n", __func__,
				touch.t - 1, touch.x, touch.y,
				touch.p, touch.w, touch.e, touch.o);

		}

		input_sync(ts->input);

	} else if (num_cur_tch == 0) {
		/* input_sync(ts->input); */
	}

	/* store current touch */
	for (i = 0; i < num_cur_tch; i++) {
		_cyttsp4_get_touch(ts, &touch,
				ts->xy_data + (i * ts->si_ofs.tch_rec_siz));
		if (touch.t < MAX_TOUCH_NUM)
			pre_touch[touch.t] = touch;
	}
	num_pre_tch = num_cur_tch;

	cyttsp4_dbg(ts, CY_DBG_LVL_1, "%s:\n", __func__);
	retval = 0;
_cyttsp4_xy_worker_exit:
	return retval;
}
#else
#ifdef REPORT_TOUCH_UP_EVENT
static int _cyttsp4_xy_worker(struct cyttsp4 *ts)
{
	struct cyttsp4_touch touch;
	u8 num_cur_tch = 0;
	u8 hst_mode = 0;
	u8 rep_len = 0;
	u8 rep_stat = 0;
	u8 tt_stat = 0;
	int i;
	int signal;
	int retval;

	int id;
	static int num_pre_tch = 0;
	static struct cyttsp4_touch pre_touch[] = {
		/* {x, y, p, t, e, o, w} */
		{0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 1, 0, 0, 0},
		{0, 0, 0, 2, 0, 0, 0},
		{0, 0, 0, 3, 0, 0, 0},
		{0, 0, 0, 4, 0, 0, 0},
		{0, 0, 0, 5, 0, 0, 0},
		{0, 0, 0, 6, 0, 0, 0},
		{0, 0, 0, 7, 0, 0, 0},
		{0, 0, 0, 8, 0, 0, 0},
		{0, 0, 0, 9, 0, 0, 0},
	};


	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

	/*
	 * Get event data from CYTTSP device.
	 * The event data includes all data
	 * for all active touches.
	 */
	memset(ts->xy_mode, 0, ts->si_ofs.mode_size);
	retval = cyttsp4_read_block_data(ts, CY_REG_BASE,
					ts->si_ofs.mode_size, ts->xy_mode,
					ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
	if (retval < 0) {
		/* I2C bus failure implies bootloader running */
		pr_err("%s: read fail on mode regs r=%d\n",
			__func__, retval);
		queue_work(ts->cyttsp4_wq,
			&ts->cyttsp4_resume_startup_work);
		pr_err("%s: startup queued\n", __func__);
		retval = IRQ_HANDLED;
		goto _cyttsp4_xy_worker_exit;
	}

	hst_mode = ts->xy_mode[CY_REG_BASE];
	rep_len = ts->xy_mode[ts->si_ofs.rep_ofs];
	rep_stat = ts->xy_mode[ts->si_ofs.rep_ofs + 1];
	tt_stat = ts->xy_mode[ts->si_ofs.tt_stat_ofs];
	cyttsp4_dbg(ts, CY_DBG_LVL_1,
		"%s: hst_mode=%02X rep_len=%d rep_stat=%02X tt_stat=%02X\n",
		__func__, hst_mode, rep_len, rep_stat, tt_stat);

	if (rep_len == 0) {
		pr_err("%s: report length error rep_len=%d\n",
			__func__, rep_len);
		retval = -EIO;
		goto _cyttsp4_xy_worker_exit;
	}

	retval = cyttsp4_read_block_data(ts, ts->si_ofs.tt_stat_ofs + 1,
					rep_len, ts->xy_data,
					ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
	if (retval < 0) {
		pr_err("%s: read fail on touch regs r=%d\n",
			__func__, retval);
		goto _cyttsp4_xy_worker_exit;
	}

	/* print xy data */
	_cyttsp4_pr_buf(ts, ts->xy_data, rep_len, "xy_data");

	/* provide flow control handshake */
	if (cyttsp4_hndshk(ts, hst_mode)) {
		pr_err("%s: handshake fail on operational reg\n",
			__func__);
		ts->power_state = CY_IDLE_STATE;
		cyttsp4_pr_state(ts);
		retval = -EIO;
		goto _cyttsp4_xy_worker_exit;
	}

	/* if in System Information mode, then switch to operational mode */
	if (GET_HSTMODE(hst_mode) == GET_HSTMODE(CY_SYSINFO_MODE)) {
		pr_err("%s: Host mode detected in op state (hm=0x%02X)\n",
			__func__, hst_mode);
		retval = _cyttsp4_set_operational_mode(ts);
		if (retval < 0) {
			ts->power_state = CY_IDLE_STATE;
			cyttsp4_pr_state(ts);
			pr_err("%s: Fail set operational mode (r=%d)\n",
				__func__, retval);
		} else {
			ts->power_state = CY_ACTIVE_STATE;
			cyttsp4_pr_state(ts);
			cyttsp4_dbg(ts, CY_DBG_LVL_3,
				"%s: enable handshake\n", __func__);
			retval = _cyttsp4_hndshk_enable(ts);
			if (retval < 0) {
				pr_err("%s: fail enable handshake r=%d",
					__func__, retval);
			}
		}
		goto _cyttsp4_xy_worker_exit;
	}

	/* determine number of currently active touches */
	num_cur_tch = GET_NUM_TOUCHES(tt_stat);

	/* check for any error conditions */
	if (ts->power_state == CY_IDLE_STATE) {
		pr_err("%s: IDLE STATE detected\n", __func__);
		retval = 0;
		goto _cyttsp4_xy_worker_exit;
	} else if (IS_BAD_PKT(rep_stat)) {
		pr_err("%s: Invalid buffer detected\n", __func__);
		retval = 0;
		goto _cyttsp4_xy_worker_exit;
	} else if (IS_LARGE_AREA(tt_stat)) {
		/* terminate all active tracks */
		num_cur_tch = 0;
		pr_err("%s: Large area detected\n", __func__);
		retval = -EIO;
		goto _cyttsp4_xy_worker_exit;
	} else if (num_cur_tch > ts->si_ofs.max_tchs) {
		if (num_cur_tch == 0x1F) {
			/* terminate all active tracks */
			pr_err("%s: Num touch err detected (n=%d)\n",
				__func__, num_cur_tch);
			num_cur_tch = 0;
		} else {
			pr_err("%s: too many tch; set to max tch (n=%d c=%d)\n",
				__func__, num_cur_tch, CY_NUM_TCH_ID);
			num_cur_tch = CY_NUM_TCH_ID;
		}
	}

	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: num_cur_tch=%d\n", __func__, num_cur_tch);

	/* extract xy_data for all currently reported touches */
	if (num_pre_tch || num_cur_tch) {
		for (id = 0; id < MAX_TOUCH_NUM; id++) {
			for (i = 0; i < num_cur_tch; i++) {
				_cyttsp4_get_touch(ts, &touch,
						ts->xy_data + (i * ts->si_ofs.tch_rec_siz));
				if (touch.t == id)
					break;
			}
			if (i >= num_cur_tch) {
				if (pre_touch[id].p == 0)
					continue;
				pre_touch[id].p = 0;
				touch = pre_touch[id];
			}

			/* adjusting the coordinates */
			if (ts->adjust_enabled == true)
				_sh_tpsif_adjust_point(ts, &(touch.x), &(touch.y));

			/* report touch event */
			if (((touch.t == 0) &&
					(ts->platform_data->frmwrk->abs
						[(CY_ABS_ID_OST*CY_NUM_ABS_SET)+CY_MIN_OST]
						> 0)) ||
				(touch.t > ts->platform_data->frmwrk->abs
					[(CY_ABS_ID_OST*CY_NUM_ABS_SET)+CY_MAX_OST])) {
				pr_err("%s: touch=%d has bad track_id=%d"
					"max_id=%d\n",
					__func__, i, touch.t,
					ts->platform_data->frmwrk->abs
					[(CY_ABS_ID_OST*CY_NUM_ABS_SET)+
						CY_MAX_OST]);
				input_mt_sync(ts->input);
			} else {
				/* use 0 based track id's */
				signal = ts->platform_data->frmwrk->abs
					[(CY_ABS_ID_OST*CY_NUM_ABS_SET)+0];
				if (signal != CY_IGNORE_VALUE) {
					if (ts->platform_data->frmwrk->abs
						[(CY_ABS_ID_OST*CY_NUM_ABS_SET)+
							CY_MIN_OST] > 0)
						input_report_abs(ts->input,
								signal, touch.t - 1);
					else
						input_report_abs(ts->input,
								signal, touch.t);
				}

				signal = ts->platform_data->frmwrk->abs
					[(CY_ABS_X_OST*CY_NUM_ABS_SET)+0];
				if (signal != CY_IGNORE_VALUE)
					input_report_abs(ts->input,
							signal, touch.x);

				signal = ts->platform_data->frmwrk->abs
					[(CY_ABS_Y_OST*CY_NUM_ABS_SET)+0];
				if (ts->reverse_y_axis) {
					if (signal != CY_IGNORE_VALUE) {
						input_report_abs(ts->input,
								signal,
								ts->platform_data->frmwrk->abs[(CY_ABS_Y_OST*CY_NUM_ABS_SET)+CY_MAX_OST] +
								ts->platform_data->frmwrk->abs[(CY_ABS_Y_OST*CY_NUM_ABS_SET)+CY_MIN_OST] -
								touch.y);
					}
				} else {
					if (signal != CY_IGNORE_VALUE)
						input_report_abs(ts->input,
								signal, touch.y);
				}

				signal = ts->platform_data->frmwrk->abs
					[(CY_ABS_P_OST*CY_NUM_ABS_SET)+0];
				if (signal != CY_IGNORE_VALUE)
					input_report_abs(ts->input,
							signal, touch.p);

				if (!(ts->flags & CY_FLAG_TMA400)) {
					signal = ts->platform_data->frmwrk->abs
						[(CY_ABS_W_OST*CY_NUM_ABS_SET)
							+0];
					if (signal != CY_IGNORE_VALUE)
						input_report_abs(ts->input,
								signal, touch.w);
				}

				input_mt_sync(ts->input);

			}
			cyttsp4_dbg(ts, CY_DBG_LVL_1,
				"%s: t-1=%d x=%04X y=%04X z=%02X"
				" w=%02X e=%02X o=%02X\n", __func__,
				touch.t - 1, touch.x, touch.y,
				touch.p, touch.w, touch.e, touch.o);

		}

		input_sync(ts->input);

	} else if (num_cur_tch == 0) {
		input_mt_sync(ts->input);
		input_sync(ts->input);
	}

	/* store current touch */
	for (i = 0; i < num_cur_tch; i++) {
		_cyttsp4_get_touch(ts, &touch,
				ts->xy_data + (i * ts->si_ofs.tch_rec_siz));
		if (touch.t < MAX_TOUCH_NUM)
			pre_touch[touch.t] = touch;
	}
	num_pre_tch = num_cur_tch;

	cyttsp4_dbg(ts, CY_DBG_LVL_1, "%s:\n", __func__);
	retval = 0;
_cyttsp4_xy_worker_exit:
	return retval;
}
#else
static int _cyttsp4_xy_worker(struct cyttsp4 *ts)
{
	struct cyttsp4_touch touch;
	u8 num_cur_tch = 0;
	u8 hst_mode = 0;
	u8 rep_len = 0;
	u8 rep_stat = 0;
	u8 tt_stat = 0;
	int i;
	int signal;
	int retval;

	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

	/*
	 * Get event data from CYTTSP device.
	 * The event data includes all data
	 * for all active touches.
	 */
	memset(ts->xy_mode, 0, ts->si_ofs.mode_size);
	retval = cyttsp4_read_block_data(ts, CY_REG_BASE,
					ts->si_ofs.mode_size, ts->xy_mode,
					ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
	if (retval < 0) {
		/* I2C bus failure implies bootloader running */
		pr_err("%s: read fail on mode regs r=%d\n",
			__func__, retval);
		queue_work(ts->cyttsp4_wq,
			&ts->cyttsp4_resume_startup_work);
		pr_err("%s: startup queued\n", __func__);
		retval = IRQ_HANDLED;
		goto _cyttsp4_xy_worker_exit;
	}

	hst_mode = ts->xy_mode[CY_REG_BASE];
	rep_len = ts->xy_mode[ts->si_ofs.rep_ofs];
	rep_stat = ts->xy_mode[ts->si_ofs.rep_ofs + 1];
	tt_stat = ts->xy_mode[ts->si_ofs.tt_stat_ofs];
	cyttsp4_dbg(ts, CY_DBG_LVL_1,
		"%s: hst_mode=%02X rep_len=%d rep_stat=%02X tt_stat=%02X\n",
		__func__, hst_mode, rep_len, rep_stat, tt_stat);

	if (rep_len == 0) {
		pr_err("%s: report length error rep_len=%d\n",
			__func__, rep_len);
		retval = -EIO;
		goto _cyttsp4_xy_worker_exit;
	}

	retval = cyttsp4_read_block_data(ts, ts->si_ofs.tt_stat_ofs + 1,
					rep_len, ts->xy_data,
					ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
	if (retval < 0) {
		pr_err("%s: read fail on touch regs r=%d\n",
			__func__, retval);
		goto _cyttsp4_xy_worker_exit;
	}

	/* print xy data */
	_cyttsp4_pr_buf(ts, ts->xy_data, rep_len, "xy_data");

	/* provide flow control handshake */
	if (cyttsp4_hndshk(ts, hst_mode)) {
		pr_err("%s: handshake fail on operational reg\n",
			__func__);
		ts->power_state = CY_IDLE_STATE;
		cyttsp4_pr_state(ts);
		retval = -EIO;
		goto _cyttsp4_xy_worker_exit;
	}

	/* if in System Information mode, then switch to operational mode */
	if (GET_HSTMODE(hst_mode) == GET_HSTMODE(CY_SYSINFO_MODE)) {
		pr_err("%s: Host mode detected in op state (hm=0x%02X)\n",
			__func__, hst_mode);
		retval = _cyttsp4_set_operational_mode(ts);
		if (retval < 0) {
			ts->power_state = CY_IDLE_STATE;
			cyttsp4_pr_state(ts);
			pr_err("%s: Fail set operational mode (r=%d)\n",
				__func__, retval);
		} else {
			ts->power_state = CY_ACTIVE_STATE;
			cyttsp4_pr_state(ts);
			cyttsp4_dbg(ts, CY_DBG_LVL_3,
				"%s: enable handshake\n", __func__);
			retval = _cyttsp4_hndshk_enable(ts);
			if (retval < 0) {
				pr_err("%s: fail enable handshake r=%d",
					__func__, retval);
			}
		}
		goto _cyttsp4_xy_worker_exit;
	}

	/* determine number of currently active touches */
	num_cur_tch = GET_NUM_TOUCHES(tt_stat);

	/* check for any error conditions */
	if (ts->power_state == CY_IDLE_STATE) {
		pr_err("%s: IDLE STATE detected\n", __func__);
		retval = 0;
		goto _cyttsp4_xy_worker_exit;
	} else if (IS_BAD_PKT(rep_stat)) {
		pr_err("%s: Invalid buffer detected\n", __func__);
		retval = 0;
		goto _cyttsp4_xy_worker_exit;
	} else if (IS_LARGE_AREA(tt_stat)) {
		/* terminate all active tracks */
		num_cur_tch = 0;
		pr_err("%s: Large area detected\n", __func__);
		retval = -EIO;
		goto _cyttsp4_xy_worker_exit;
	} else if (num_cur_tch > ts->si_ofs.max_tchs) {
		if (num_cur_tch == 0x1F) {
			/* terminate all active tracks */
			pr_err("%s: Num touch err detected (n=%d)\n",
				__func__, num_cur_tch);
			num_cur_tch = 0;
		} else {
			pr_err("%s: too many tch; set to max tch (n=%d c=%d)\n",
				__func__, num_cur_tch, CY_NUM_TCH_ID);
			num_cur_tch = CY_NUM_TCH_ID;
		}
	}

	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: num_cur_tch=%d\n", __func__, num_cur_tch);

	/* extract xy_data for all currently reported touches */
	if (num_cur_tch) {
		for (i = 0; i < num_cur_tch; i++) {
			_cyttsp4_get_touch(ts, &touch,
					ts->xy_data + (i * ts->si_ofs.tch_rec_siz));

			/* adjusting the coordinates */
			if (ts->adjust_enabled == true)
				_sh_tpsif_adjust_point(ts, &(touch.x), &(touch.y));

			if (((touch.t == 0) &&
					(ts->platform_data->frmwrk->abs
						[(CY_ABS_ID_OST*CY_NUM_ABS_SET)+CY_MIN_OST]
						> 0)) ||
				(touch.t > ts->platform_data->frmwrk->abs
					[(CY_ABS_ID_OST*CY_NUM_ABS_SET)+CY_MAX_OST])) {
				pr_err("%s: touch=%d has bad track_id=%d"
					"max_id=%d\n",
					__func__, i, touch.t,
					ts->platform_data->frmwrk->abs
					[(CY_ABS_ID_OST*CY_NUM_ABS_SET)+
						CY_MAX_OST]);
				input_mt_sync(ts->input);
			} else {
				/* use 0 based track id's */
				signal = ts->platform_data->frmwrk->abs
					[(CY_ABS_ID_OST*CY_NUM_ABS_SET)+0];
				if (signal != CY_IGNORE_VALUE) {
					if (ts->platform_data->frmwrk->abs
						[(CY_ABS_ID_OST*CY_NUM_ABS_SET)+
							CY_MIN_OST] > 0)
						input_report_abs(ts->input,
								signal, touch.t - 1);
					else
						input_report_abs(ts->input,
								signal, touch.t);
				}

				signal = ts->platform_data->frmwrk->abs
					[(CY_ABS_X_OST*CY_NUM_ABS_SET)+0];
				if (signal != CY_IGNORE_VALUE)
					input_report_abs(ts->input,
							signal, touch.x);

				signal = ts->platform_data->frmwrk->abs
					[(CY_ABS_Y_OST*CY_NUM_ABS_SET)+0];
				if (ts->reverse_y_axis) {
					if (signal != CY_IGNORE_VALUE) {
						input_report_abs(ts->input,
								signal,
								ts->platform_data->frmwrk->abs[(CY_ABS_Y_OST*CY_NUM_ABS_SET)+CY_MAX_OST] +
								ts->platform_data->frmwrk->abs[(CY_ABS_Y_OST*CY_NUM_ABS_SET)+CY_MIN_OST] -
								touch.y);
					}
				} else {
					if (signal != CY_IGNORE_VALUE)
						input_report_abs(ts->input,
								signal, touch.y);
				}

				signal = ts->platform_data->frmwrk->abs
					[(CY_ABS_P_OST*CY_NUM_ABS_SET)+0];
				if (signal != CY_IGNORE_VALUE)
					input_report_abs(ts->input,
							signal, touch.p);

				if (!(ts->flags & CY_FLAG_TMA400)) {
					signal = ts->platform_data->frmwrk->abs
						[(CY_ABS_W_OST*CY_NUM_ABS_SET)
							+0];
					if (signal != CY_IGNORE_VALUE)
						input_report_abs(ts->input,
								signal, touch.w);
				}

				input_mt_sync(ts->input);

			}
			cyttsp4_dbg(ts, CY_DBG_LVL_1,
				"%s: t-1=%d x=%04X y=%04X z=%02X"
				" w=%02X e=%02X o=%02X\n", __func__,
				touch.t - 1, touch.x, touch.y,
				touch.p, touch.w, touch.e, touch.o);
		}
		input_sync(ts->input);
	} else {
		input_mt_sync(ts->input);
		input_sync(ts->input);
	}

	cyttsp4_dbg(ts, CY_DBG_LVL_1, "%s:\n", __func__);
	retval = 0;
_cyttsp4_xy_worker_exit:
	return retval;
}
#endif	/* REPORT_TOUCH_UP_EVENT */
#endif	/* MULTI_TOUCH_PROTOCOL_B */

static int _cyttsp4_soft_reset(struct cyttsp4 *ts)
{
	u8 cmd = CY_SOFT_RESET_MODE;

	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

	return cyttsp4_write_block_data(ts, CY_REG_BASE,
		sizeof(cmd), &cmd,
		ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
}

static int _cyttsp4_reset(struct cyttsp4 *ts)
{
	bool soft_reset = false;
	int retval = 0;

	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

	if (ts->platform_data->hw_reset) {
		retval = ts->platform_data->hw_reset();
		if (retval == -ENOSYS) {
			soft_reset = true;
			retval = _cyttsp4_soft_reset(ts);
		}
	} else {
		soft_reset = true;
		retval = _cyttsp4_soft_reset(ts);
	}

	if (soft_reset) {
		cyttsp4_dbg(ts, CY_DBG_LVL_2,
			"%s: Used Soft Reset - %s (r=%d)\n", __func__,
			retval ? "Fail" : "Ok", retval);
	} else {
		cyttsp4_dbg(ts, CY_DBG_LVL_2,
			"%s: Used Hard Reset - %s (r=%d)\n", __func__,
			retval ? "Fail" : "Ok", retval);
	}

	if (retval < 0)
		return retval;
	else {
		ts->current_mode = CY_MODE_BOOTLOADER;
		return retval;
	}
}

static void cyttsp4_ts_work_func(struct work_struct *work)
{
	struct cyttsp4 *ts =
		container_of(work, struct cyttsp4, cyttsp4_resume_startup_work);
	int retval = 0;

	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

	mutex_lock(&ts->data_lock);

	retval = _cyttsp4_startup(ts);
	if (retval < 0) {
		pr_err("%s: Startup failed with error code %d\n",
			__func__, retval);
		ts->power_state = CY_IDLE_STATE;
		cyttsp4_pr_state(ts);
	}

	mutex_unlock(&ts->data_lock);

	return;
}

static void cyttsp4_ts_startup_work_func(struct work_struct *work)
{
	struct cyttsp4 *ts =
		container_of(work, struct cyttsp4, cyttsp4_startup_work);
	int retval = 0;

	mutex_lock(&ts->data_lock);

	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

	retval = _cyttsp4_startup(ts);
	if (retval < 0) {
		pr_err("%s: Startup failed with error code %d\n",
			__func__, retval);
		ts->power_state = CY_IDLE_STATE;
		cyttsp4_pr_state(ts);
	}

	mutex_unlock(&ts->data_lock);

	return;
}

static int _cyttsp4_resume_sleep(struct cyttsp4 *ts)
{
	int retval = 0;
	uint8_t sleep = CY_DEEP_SLEEP_MODE;

	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: Put the part back to sleep\n", __func__);

	retval = cyttsp4_write_block_data(ts, CY_REG_BASE,
		sizeof(sleep), &sleep,
		ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
	if (retval < 0) {
		pr_err("%s: Failed to write sleep bit r=%d\n",
			__func__, retval);
	}

	ts->power_state = CY_SLEEP_STATE;
	cyttsp4_pr_state(ts);

	return retval;
}

#if defined(CONFIG_PM) || defined(CONFIG_HAS_EARLYSUSPEND)
int cyttsp4_resume(void *handle)
{
	struct cyttsp4 *ts = handle;
	int tries = 0;
	int retval = 0;
	u8 dummy = 0x00;
	int wake = 99;

	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

	mutex_lock(&ts->data_lock);

	cyttsp4_dbg(ts, CY_DBG_LVL_2, "%s: Resuming...\n", __func__);

	switch (ts->power_state) {
	case CY_SLEEP_STATE:
		/* Any I2C line traffic will wake the part */
		/*
		 * This write should always fail because the part is not
		 * ready to respond.
		 */
		ts->cmd_rdy = false;
		ts->power_state = CY_CMD_STATE;
		if (ts->platform_data->hw_recov == NULL) {
			cyttsp4_dbg(ts, CY_DBG_LVL_3,
				"%s: no hw_recov function\n", __func__);
			retval = -ENOSYS;
		} else {
			retval = ts->platform_data->hw_recov(wake);
			if (retval < 0) {
				if (retval == -ENOSYS) {
					cyttsp4_dbg(ts, CY_DBG_LVL_3,
						"%s: no hw_recov wake=%d"
						" function\n", __func__, wake);
				} else {
					pr_err("%s: fail hw_recov(wake=%d)"
						" function r=%d\n",
						__func__, wake, retval);
					retval = -ENOSYS;
				}
			}
		}
		if (retval == -ENOSYS) {
			retval = cyttsp4_read_block_data(ts, CY_REG_BASE,
				sizeof(dummy), &dummy,
				ts->platform_data->addr[CY_TCH_ADDR_OFS],
				false);
			if (retval < 0) {
				pr_info("%s: Read failure -121 is normal"
					" here--%s\n", __func__,
					"the I2C lines just need to toggle");
			} else {
				/* IC is already awake, so resume is done */
				ts->power_state = CY_ACTIVE_STATE;
				cyttsp4_pr_state(ts);
				break;
			}
		} else
			retval = 0;

		/* Wait for cmd rdy interrupt to signal device wake */
		tries = 0;
		do {
			mutex_unlock(&ts->data_lock);
			msleep(CY_DELAY_DFLT);
			mutex_lock(&ts->data_lock);
		} while (!ts->cmd_rdy && (tries++ < CY_DELAY_MAX));

		retval = cyttsp4_read_block_data(ts,
			CY_REG_BASE,
			sizeof(dummy), &dummy,
			ts->platform_data->addr
			[CY_TCH_ADDR_OFS],
			false);

		if (retval < 0) {
			pr_err("%s: failed to resume or in bootloader (r=%d)"
				" tries=%d\n", __func__, retval, tries);
			ts->power_state = CY_BL_STATE;
			cyttsp4_pr_state(ts);
			mutex_unlock(&ts->data_lock);
			queue_work(ts->cyttsp4_wq,
				&ts->cyttsp4_resume_startup_work);
			pr_info("%s: startup queued\n", __func__);
			retval = 0;
			goto cyttsp4_resume_exit;
		} else {
			cyttsp4_dbg(ts, CY_DBG_LVL_3,
				"%s: resume in tries=%d\n", __func__, tries);
			retval = cyttsp4_hndshk(ts, dummy);
			if (retval < 0) {
				pr_err("%s: fail resume INT handshake (r=%d)\n",
					__func__, retval);
				/* continue */
			}
			ts->power_state = CY_ACTIVE_STATE;
			cyttsp4_pr_state(ts);
		}
		break;
	case CY_IDLE_STATE:
	case CY_READY_STATE:
	case CY_ACTIVE_STATE:
	case CY_LOW_PWR_STATE:
	case CY_BL_STATE:
	case CY_LDR_STATE:
	case CY_SYSINFO_STATE:
	case CY_CMD_STATE:
	case CY_INVALID_STATE:
	default:
		pr_err("%s: Already in %s state\n", __func__,
			cyttsp4_powerstate_string[ts->power_state]);
		break;
	}

	mutex_unlock(&ts->data_lock);
cyttsp4_resume_exit:
	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: exit Resume r=%d\n", __func__, retval);
	return retval;
}
EXPORT_SYMBOL_GPL(cyttsp4_resume);

int cyttsp4_suspend(void *handle)
{
	int retval = 0;
	struct cyttsp4 *ts = handle;

	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

	switch (ts->power_state) {
	case CY_ACTIVE_STATE:
		mutex_lock(&ts->data_lock);

		cyttsp4_dbg(ts, CY_DBG_LVL_2, "%s: Suspending...\n", __func__);

		retval = _cyttsp4_resume_sleep(ts);
		if (retval < 0) {
			pr_err("%s: fail enter sleep r=%d\n",
				__func__, retval);
		} else {
			ts->power_state = CY_SLEEP_STATE;
			cyttsp4_pr_state(ts);
		}

		mutex_unlock(&ts->data_lock);
		break;
	case CY_SLEEP_STATE:
		pr_err("%s: already in Sleep state\n", __func__);
		break;
	case CY_LDR_STATE:
	case CY_CMD_STATE:
	case CY_SYSINFO_STATE:
	case CY_READY_STATE:
		retval = -EBUSY;
		pr_err("%s: Suspend Blocked while in %s state\n", __func__,
			cyttsp4_powerstate_string[ts->power_state]);
		break;
	case CY_IDLE_STATE:
	case CY_LOW_PWR_STATE:
	case CY_BL_STATE:
	case CY_INVALID_STATE:
	default:
		retval = -EINVAL;
		pr_err("%s: Cannot enter suspend from %s state\n", __func__,
			cyttsp4_powerstate_string[ts->power_state]);
		break;
	}

	return retval;
}
EXPORT_SYMBOL_GPL(cyttsp4_suspend);
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
void cyttsp4_early_suspend(struct early_suspend *h)
{
	struct cyttsp4 *ts = container_of(h, struct cyttsp4, early_suspend);
	int retval = 0;

	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

	retval = cyttsp4_suspend(ts);
	if (retval < 0) {
		pr_err("%s: Early suspend failed with error code %d\n",
			__func__, retval);
	}
}

void cyttsp4_late_resume(struct early_suspend *h)
{
	struct cyttsp4 *ts = container_of(h, struct cyttsp4, early_suspend);
	int retval = 0;

	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

	retval = cyttsp4_resume(ts);
	if (retval < 0) {
		pr_err("%s: Late resume failed with error code %d\n",
			__func__, retval);
	}
}
#endif

static int _cyttsp4_boot_loader(struct cyttsp4 *ts, bool *upgraded)
{
	bool new_vers = false;
	int retval = 0;
	int i = 0;
	u32 fw_vers_platform = 0;
	u32 fw_vers_img = 0;
	u32 fw_revctrl_platform_h = 0;
	u32 fw_revctrl_platform_l = 0;
	u32 fw_revctrl_img_h = 0;
	u32 fw_revctrl_img_l = 0;
	bool new_fw_vers = false;
	bool new_fw_revctrl = false;

	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

	*upgraded = false;
	if (ts->power_state == CY_SLEEP_STATE) {
		pr_err("%s: cannot load firmware in sleep state\n",
			__func__);
		retval = 0;
	} else if ((ts->platform_data->fw->ver == NULL) ||
		(ts->platform_data->fw->img == NULL)) {
		pr_err("%s: empty version list or no image\n",
			__func__);
		retval = 0;
	} else if (ts->platform_data->fw->vsize != CY_BL_VERS_SIZE) {
		pr_err("%s: bad fw version list size=%d\n",
			__func__, ts->platform_data->fw->vsize);
		retval = 0;
	} else {
		/* automatically update firmware if new version detected */
		new_vers = false;
		fw_vers_img = (ts->sysinfo_ptr.cydata->fw_ver_major * 256);
		fw_vers_img += ts->sysinfo_ptr.cydata->fw_ver_minor;
		fw_vers_platform = ts->platform_data->fw->ver[2] * 256;
		fw_vers_platform += ts->platform_data->fw->ver[3];
		if (fw_vers_platform > fw_vers_img)
			new_fw_vers = true;
		else
			new_fw_vers = false;
		cyttsp4_dbg(ts, CY_DBG_LVL_3,
			"%s: fw_vers_platform=%04X fw_vers_img=%04X\n",
			__func__, fw_vers_platform, fw_vers_img);

		fw_revctrl_img_h = ts->sysinfo_ptr.cydata->revctrl[0];
		fw_revctrl_img_l = ts->sysinfo_ptr.cydata->revctrl[4];
		fw_revctrl_platform_h = ts->platform_data->fw->ver[4];
		fw_revctrl_platform_l = ts->platform_data->fw->ver[8];
		for (i = 1; i < 4; i++) {
			fw_revctrl_img_h = (fw_revctrl_img_h * 256) +
				ts->sysinfo_ptr.cydata->revctrl[0+i];
			fw_revctrl_img_l = (fw_revctrl_img_l * 256) +
				ts->sysinfo_ptr.cydata->revctrl[4+i];
			fw_revctrl_platform_h = (fw_revctrl_platform_h * 256) +
				ts->platform_data->fw->ver[4+i];
			fw_revctrl_platform_l = (fw_revctrl_platform_l * 256) +
				ts->platform_data->fw->ver[8+i];
		}
		if (fw_revctrl_platform_h > fw_revctrl_img_h)
			new_fw_revctrl = true;
		else if (fw_revctrl_platform_h == fw_revctrl_img_h) {
			if (fw_revctrl_platform_l > fw_revctrl_img_l)
				new_fw_revctrl = true;
			else
				new_fw_revctrl = false;
		} else
			new_fw_revctrl = false;

		if (new_fw_vers || new_fw_revctrl)
			new_vers = true;

		cyttsp4_dbg(ts, CY_DBG_LVL_3,
			"%s: fw_revctrl_platform_h=%08X"
			" fw_revctrl_img_h=%08X\n", __func__,
			fw_revctrl_platform_h, fw_revctrl_img_h);
		cyttsp4_dbg(ts, CY_DBG_LVL_3,
			"%s: fw_revctrl_platform_l=%08X"
			" fw_revctrl_img_l=%08X\n", __func__,
			fw_revctrl_platform_l, fw_revctrl_img_l);
		cyttsp4_dbg(ts, CY_DBG_LVL_3,
			"%s: new_fw_vers=%d new_fw_revctrl=%d new_vers=%d\n",
			__func__,
			(int)new_fw_vers, (int)new_fw_revctrl, (int)new_vers);

		if (new_vers) {
			pr_info("%s: upgrading firmware...\n", __func__);
			*upgraded = true;
			retval = _cyttsp4_reset(ts);
			if (retval < 0) {
				pr_err("%s: fail to reset to bootloader r=%d\n",
					__func__, retval);
				goto _cyttsp4_boot_loader_exit;
			}
			retval = _cyttsp4_load_app(ts,
				ts->platform_data->fw->img,
				ts->platform_data->fw->size);
			if (retval) {
				pr_err("%s: I2C fail on load fw (r=%d)\n",
					__func__, retval);
				ts->power_state = CY_IDLE_STATE;
				retval = -EIO;
				cyttsp4_pr_state(ts);
			} else {
				/* reset TTSP Device back to bootloader mode */
				retval = _cyttsp4_reset(ts);
			}
		} else {
			cyttsp4_dbg(ts, CY_DBG_LVL_2,
				"%s: No auto firmware upgrade required\n",
				__func__);
		}
	}

_cyttsp4_boot_loader_exit:
	return retval;
}

static ssize_t cyttsp4_ic_ver_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyttsp4 *ts = dev_get_drvdata(dev);

	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

	return sprintf(buf, "%s: 0x%02X 0x%02X\n%s: 0x%02X\n%s: 0x%02X\n%s: "
		"0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n",
		"TrueTouch Product ID",
		ts->sysinfo_ptr.cydata->ttpidh,
		ts->sysinfo_ptr.cydata->ttpidl,
		"Firmware Major Version", ts->sysinfo_ptr.cydata->fw_ver_major,
		"Firmware Minor Version", ts->sysinfo_ptr.cydata->fw_ver_minor,
		"Revision Control Number", ts->sysinfo_ptr.cydata->revctrl[0],
		ts->sysinfo_ptr.cydata->revctrl[1],
		ts->sysinfo_ptr.cydata->revctrl[2],
		ts->sysinfo_ptr.cydata->revctrl[3],
		ts->sysinfo_ptr.cydata->revctrl[4],
		ts->sysinfo_ptr.cydata->revctrl[5],
		ts->sysinfo_ptr.cydata->revctrl[6],
		ts->sysinfo_ptr.cydata->revctrl[7]);
}
static DEVICE_ATTR(ic_ver, S_IRUSR | S_IWUSR,
	cyttsp4_ic_ver_show, NULL);

/* Driver version */
static ssize_t cyttsp4_drv_ver_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, CY_MAX_PRBUF_SIZE,
		"Driver: %s\nVersion: %s\nDate: %s\n",
		CY_SPI_NAME, CY_DRIVER_VERSION, CY_DRIVER_DATE);
}
static DEVICE_ATTR(drv_ver, S_IRUGO, cyttsp4_drv_ver_show, NULL);


/* Driver status */
static ssize_t cyttsp4_drv_stat_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyttsp4 *ts = dev_get_drvdata(dev);

	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

	return snprintf(buf, CY_MAX_PRBUF_SIZE,
		"Driver state is %s\n",
		cyttsp4_powerstate_string[ts->power_state]);
}
static DEVICE_ATTR(drv_stat, S_IRUGO, cyttsp4_drv_stat_show, NULL);

#ifdef CONFIG_TOUCHSCREEN_DEBUG
static ssize_t cyttsp_ic_irq_stat_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int retval;
	struct cyttsp4 *ts = dev_get_drvdata(dev);

	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

	if (ts->platform_data->irq_stat) {
		retval = ts->platform_data->irq_stat();
		switch (retval) {
		case 0:
			return snprintf(buf, CY_MAX_PRBUF_SIZE,
				"Interrupt line is LOW.\n");
		case 1:
			return snprintf(buf, CY_MAX_PRBUF_SIZE,
				"Interrupt line is HIGH.\n");
		default:
			return snprintf(buf, CY_MAX_PRBUF_SIZE,
				"Function irq_stat() returned %d.\n", retval);
		}
	} else {
		return snprintf(buf, CY_MAX_PRBUF_SIZE,
			"Function irq_stat() undefined.\n");
	}
}
static DEVICE_ATTR(hw_irqstat, S_IRUSR | S_IWUSR,
	cyttsp_ic_irq_stat_show, NULL);
#endif

#ifdef CONFIG_TOUCHSCREEN_DEBUG
/* Disable Driver IRQ */
static ssize_t cyttsp4_drv_irq_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	static const char *fmt_disabled = "Driver interrupt is DISABLED\n";
	static const char *fmt_enabled = "Driver interrupt is ENABLED\n";
	struct cyttsp4 *ts = dev_get_drvdata(dev);

	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

	if (ts->irq_enabled == false)
		return snprintf(buf, strlen(fmt_disabled)+1, fmt_disabled);
	else
		return snprintf(buf, strlen(fmt_enabled)+1, fmt_enabled);
}
static ssize_t cyttsp4_drv_irq_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int retval = 0;
	struct cyttsp4 *ts = dev_get_drvdata(dev);
	unsigned long value;

	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

	mutex_lock(&(ts->data_lock));

	if (size > 2) {
		pr_err("%s: Err, data too large\n", __func__);
		retval = -EOVERFLOW;
		goto cyttsp4_drv_irq_store_error_exit;
	}

	retval = strict_strtoul(buf, 10, &value);
	if (retval < 0) {
		pr_err("%s: Failed to convert value\n", __func__);
		goto cyttsp4_drv_irq_store_error_exit;
	}

	if (ts->irq_enabled == false) {
		if (value == 1) {
			/* Enable IRQ */
			enable_irq(ts->irq);
			pr_info("%s: Driver IRQ now enabled\n", __func__);
			ts->irq_enabled = true;
		} else {
			pr_info("%s: Driver IRQ already disabled\n", __func__);
		}
	} else {
		if (value == 0) {
			/* Disable IRQ */
			disable_irq_nosync(ts->irq);
			pr_info("%s: Driver IRQ now disabled\n", __func__);
			ts->irq_enabled = false;
		} else {
			pr_info("%s: Driver IRQ already enabled\n", __func__);
		}
	}

	retval = size;

cyttsp4_drv_irq_store_error_exit:
	mutex_unlock(&(ts->data_lock));
	return retval;
}
static DEVICE_ATTR(drv_irq, S_IRUSR | S_IWUSR,
	cyttsp4_drv_irq_show, cyttsp4_drv_irq_store);
#endif

#ifdef CONFIG_TOUCHSCREEN_DEBUG
/* Driver debugging */
static ssize_t cyttsp4_drv_debug_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyttsp4 *ts = dev_get_drvdata(dev);

	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

	return snprintf(buf, CY_MAX_PRBUF_SIZE,
		"Debug Setting: %u\n", ts->bus_ops->tsdebug);
}
static ssize_t cyttsp4_drv_debug_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct cyttsp4 *ts = dev_get_drvdata(dev);
	int retval = 0;
	unsigned long value = 0;

	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

	retval = strict_strtoul(buf, 10, &value);
	if (retval < 0) {
		pr_err("%s: Failed to convert value\n", __func__);
		goto cyttsp4_drv_debug_store_exit;
	}

	switch (value) {
	case CY_DBG_LVL_0:
	case CY_DBG_LVL_1:
	case CY_DBG_LVL_2:
	case CY_DBG_LVL_3:
	case CY_DBG_LVL_4:
		pr_info("%s: Debug setting=%d\n", __func__, (int)value);
		ts->bus_ops->tsdebug = value;
		break;
#ifdef CY_USE_DEBUG_TOOLS
	case CY_DBG_SUSPEND:
		pr_info("%s: SUSPEND (ts=%p)\n", __func__, ts);
		cyttsp4_suspend(ts);
		break;
	case CY_DBG_RESUME:
		pr_info("%s: RESUME (ts=%p)\n", __func__, ts);
		cyttsp4_resume(ts);
		break;
	case CY_DBG_RESET:
		pr_info("%s: RESET (ts=%p)\n",
			__func__, ts);
		mutex_lock(&ts->data_lock);
		retval = _cyttsp4_startup(ts);
		mutex_unlock(&ts->data_lock);
		pr_info("%s: return from _cyttsp_startup test r=%d\n",
			__func__, retval);
		break;
	case CY_DBG_PUT_ALL_PARAMS:
		if (ts->flags & CY_FLAG_TMA400) {
			pr_info("%s: PUT_ALL_PARAMS (ts=%p)\n",
				__func__, ts);
			mutex_lock(&ts->data_lock);
			retval = _cyttsp4_put_all_params_tma400(ts);
			mutex_unlock(&ts->data_lock);
		} else {
			pr_err("%s: INVALID debug setting=%d\n",
				__func__, (int)value);
		}
		break;
#endif
	default:
		pr_err("%s: INVALID debug setting=%d\n",
			__func__, (int)value);
		break;
	}

	retval = size;

cyttsp4_drv_debug_store_exit:
	return retval;
}
static DEVICE_ATTR(drv_debug, S_IWUSR | S_IRUGO,
	cyttsp4_drv_debug_show, cyttsp4_drv_debug_store);
#endif

#ifdef CONFIG_TOUCHSCREEN_DEBUG
/* Group Number */
static ssize_t cyttsp4_ic_grpnum_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyttsp4 *ts = dev_get_drvdata(dev);

	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

	return snprintf(buf, CY_MAX_PRBUF_SIZE,
		"Current Group: %d\n", ts->ic_grpnum);
}
static ssize_t cyttsp4_ic_grpnum_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct cyttsp4 *ts = dev_get_drvdata(dev);
	unsigned long value = 0;
	int retval = 0;

	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

	mutex_lock(&(ts->data_lock));
	retval = strict_strtoul(buf, 10, &value);
	if (retval < 0) {
		pr_err("%s: Failed to convert value\n", __func__);
		goto cyttsp4_ic_grpnum_store_error_exit;
	}

	if (value > 0xFF) {
		value = 0xFF;
		pr_err("%s: value is greater than max;"
			" set to %d\n", __func__, (int)value);
	}
	ts->ic_grpnum = value;

	cyttsp4_dbg(ts, CY_DBG_LVL_2,
		"%s: grpnum=%d\n", __func__, ts->ic_grpnum);

cyttsp4_ic_grpnum_store_error_exit:
	retval = size;
	mutex_unlock(&(ts->data_lock));
	return retval;
}
static DEVICE_ATTR(ic_grpnum, S_IRUSR | S_IWUSR,
	cyttsp4_ic_grpnum_show, cyttsp4_ic_grpnum_store);

/* Group Offset */
static ssize_t cyttsp4_ic_grpoffset_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyttsp4 *ts = dev_get_drvdata(dev);

	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

	return snprintf(buf, CY_MAX_PRBUF_SIZE,
		"Current Offset: %u\n", ts->ic_grpoffset);
}
static ssize_t cyttsp4_ic_grpoffset_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct cyttsp4 *ts = dev_get_drvdata(dev);
	unsigned long value;
	int retval = 0;

	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

	mutex_lock(&(ts->data_lock));
	retval = strict_strtoul(buf, 10, &value);
	if (retval < 0) {
		pr_err("%s: Failed to convert value\n", __func__);
		goto cyttsp4_ic_grpoffset_store_error_exit;
	}

	if (ts->flags & CY_FLAG_TMA400) {
		if (value > 0xFFFF) {
			value = 0xFFFF;
			pr_err("%s: value is greater than max;"
				" set to %d\n", __func__, (int)value);
		}
	} else {
		if (value > 0xFF) {
			value = 0xFF;
			pr_err("%s: value is greater than max;"
				" set to %d\n", __func__, (int)value);
		}
	}
	ts->ic_grpoffset = value;

	cyttsp4_dbg(ts, CY_DBG_LVL_2,
		"%s: grpoffset=%d\n", __func__, ts->ic_grpoffset);

cyttsp4_ic_grpoffset_store_error_exit:
	retval = size;
	mutex_unlock(&(ts->data_lock));
	return retval;
}
static DEVICE_ATTR(ic_grpoffset, S_IRUSR | S_IWUSR,
	cyttsp4_ic_grpoffset_show, cyttsp4_ic_grpoffset_store);

/* Group Data */
static int _cyttsp4_tma400_show_tch_param(struct cyttsp4 *ts,
	u8 *ic_buf, size_t *num_data)
{
	/*
	 * get data from ts->ic_grpoffset to
	 * end of block containing ts->ic_grpoffset
	 */
	int retval = 0;
	int start_addr;
	int block_id;
	size_t block_size = 0;
	u8 *pdata;

	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

	retval = _cyttsp4_set_config_mode(ts);
	if (retval < 0) {
		pr_err("%s: Fail switch to config mode r=%d\n",
			__func__, retval);
		goto _cyttsp4_tma400_show_tch_param_err;
	}

	retval = _cyttsp4_get_block_size(ts, &block_size);
	if (retval < 0) {
		pr_err("%s: Fail get block size r=%d\n", __func__, retval);
		goto _cyttsp4_tma400_show_tch_param_exit;
	}

	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: read blocksize=%d pdata=%p\r",
		__func__, block_size, pdata);

	pdata = kzalloc(block_size, GFP_KERNEL);
	if (pdata == NULL) {
		pr_err("%s: Fail allocate block buffer\n", __func__);
		retval = -ENOMEM;
		goto _cyttsp4_tma400_show_tch_param_exit;
	}

	start_addr = ts->ic_grpoffset;
	block_id = start_addr / block_size;
	start_addr %= block_size;

	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: read block=%d size=%d pdata=%p\r",
		__func__, block_id, block_size, pdata);

	retval = _cyttsp4_get_block_data(ts, block_id, block_size, pdata);
	if (retval < 0) {
		pr_err("%s: Fail get block=%d r=%d\n",
			__func__, block_id, retval);
		goto _cyttsp4_tma400_show_tch_param_exit;
	}

	*num_data = block_size - start_addr;
	memcpy(&ic_buf[0], &pdata[start_addr], *num_data);

_cyttsp4_tma400_show_tch_param_exit:
	retval = _cyttsp4_set_operational_mode(ts);
	if (retval < 0) {
		pr_err("%s: Fail switch to operational mode r=%d\n",
			__func__, retval);
	}
	if (pdata != NULL)
		kfree(pdata);
_cyttsp4_tma400_show_tch_param_err:
	return retval;
}

static ssize_t _cyttsp4_ic_grpdata_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyttsp4 *ts = dev_get_drvdata(dev);
	int i;
	size_t ndata = 0;
	int retval = 0;
	size_t num_read = 0;
	u8 *ic_buf;
	u8 *pdata;
	u8 blockid = 0;

	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

	ic_buf = kzalloc(CY_MAX_PRBUF_SIZE, GFP_KERNEL);
	if (ic_buf == NULL) {
		pr_err("%s: Failed to allocate buffer for %s\n",
			__func__, "ic_grpdata_show");
		return snprintf(buf, CY_MAX_PRBUF_SIZE,
			"Group %d buffer allocation error.\n",
			ts->ic_grpnum);
	}
	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: grpnum=%d grpoffset=%u\n",
		__func__, ts->ic_grpnum, ts->ic_grpoffset);

	if (ts->ic_grpnum >= CY_IC_GRPNUM_NUM) {
		pr_err("%s: Group %d does not exist.\n",
			__func__, ts->ic_grpnum);
		kfree(ic_buf);
		return snprintf(buf, CY_MAX_PRBUF_SIZE,
			"Group %d does not exist.\n",
			ts->ic_grpnum);
	}

	switch (ts->ic_grpnum) {
	case CY_IC_GRPNUM_RESERVED:
		goto cyttsp4_ic_grpdata_show_grperr;
		break;
	case CY_IC_GRPNUM_CMD_REGS:
		num_read = ts->si_ofs.rep_ofs - ts->si_ofs.cmd_ofs;
		cyttsp4_dbg(ts, CY_DBG_LVL_3,
			"%s: GRP=CMD_REGS: num_read=%d at ofs=%d + grpofs=%d\n",
			__func__, num_read,
			ts->si_ofs.cmd_ofs, ts->ic_grpoffset);
		if (ts->ic_grpoffset >= num_read)
			goto cyttsp4_ic_grpdata_show_ofserr;
		else {
			num_read -= ts->ic_grpoffset;
			retval = cyttsp4_read_block_data(ts, ts->ic_grpoffset +
				ts->si_ofs.cmd_ofs, num_read, ic_buf,
				ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
			if (retval < 0)
				goto cyttsp4_ic_grpdata_show_prerr;
		}
		break;
	case CY_IC_GRPNUM_TCH_REP:
		num_read = ts->si_ofs.rep_sz;
		cyttsp4_dbg(ts, CY_DBG_LVL_3,
			"%s: GRP=TCH_REP: num_read=%d at ofs=%d + grpofs=%d\n",
			__func__, num_read,
			ts->si_ofs.rep_ofs, ts->ic_grpoffset);
		if (ts->ic_grpoffset >= num_read)
			goto cyttsp4_ic_grpdata_show_ofserr;
		else {
			num_read -= ts->ic_grpoffset;
			retval = cyttsp4_read_block_data(ts, ts->ic_grpoffset +
				ts->si_ofs.rep_ofs, num_read, ic_buf,
				ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
			if (retval < 0)
				goto cyttsp4_ic_grpdata_show_prerr;
		}
		break;
	case CY_IC_GRPNUM_DATA_REC:
		num_read = ts->si_ofs.cydata_size;
		cyttsp4_dbg(ts, CY_DBG_LVL_3,
			"%s: GRP=DATA_REC: num_read=%d at ofs=%d + grpofs=%d\n",
			__func__, num_read,
			ts->si_ofs.cydata_ofs, ts->ic_grpoffset);
		if (ts->ic_grpoffset >= num_read)
			goto cyttsp4_ic_grpdata_show_ofserr;
		else {
			num_read -= ts->ic_grpoffset;
			retval = _cyttsp4_set_sysinfo_mode(ts);
			if (retval < 0) {
				pr_err("%s: Fail enter Sysinfo mode r=%d\n",
					__func__, retval);
				goto cyttsp4_ic_grpdata_show_data_rderr;
			}
			retval = cyttsp4_read_block_data(ts, ts->ic_grpoffset +
				ts->si_ofs.cydata_ofs, num_read, ic_buf,
				ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
			if (retval < 0) {
				pr_err("%s: Fail read Sysinfo ddata r=%d\n",
					__func__, retval);
				goto cyttsp4_ic_grpdata_show_data_rderr;
			}
			retval = _cyttsp4_set_operational_mode(ts);
			if (retval < 0) {
				pr_err("%s: Fail enter Operational mode r=%d\n",
					__func__, retval);
			}
		}
		break;
cyttsp4_ic_grpdata_show_data_rderr:
		pr_err("%s: Fail read cydata record\n", __func__);
		goto cyttsp4_ic_grpdata_show_prerr;
		break;
	case CY_IC_GRPNUM_TEST_REC:
		num_read =  ts->si_ofs.test_size;
		cyttsp4_dbg(ts, CY_DBG_LVL_3,
			"%s: GRP=TEST_REC: num_read=%d at ofs=%d + grpofs=%d\n",
			__func__, num_read,
			ts->si_ofs.test_ofs, ts->ic_grpoffset);
		if (ts->ic_grpoffset >= num_read)
			goto cyttsp4_ic_grpdata_show_ofserr;
		else {
			num_read -= ts->ic_grpoffset;
			retval = _cyttsp4_set_sysinfo_mode(ts);
			if (retval < 0) {
				pr_err("%s: Fail enter Sysinfo mode r=%d\n",
					__func__, retval);
				goto cyttsp4_ic_grpdata_show_test_rderr;
			}
			retval = cyttsp4_read_block_data(ts, ts->ic_grpoffset +
				ts->si_ofs.test_ofs, num_read, ic_buf,
				ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
			if (retval < 0) {
				pr_err("%s: Fail read Sysinfo ddata r=%d\n",
					__func__, retval);
				goto cyttsp4_ic_grpdata_show_test_rderr;
			}
			retval = _cyttsp4_set_operational_mode(ts);
			if (retval < 0) {
				pr_err("%s: Fail enter Operational mode r=%d\n",
					__func__, retval);
			}
		}
		break;
cyttsp4_ic_grpdata_show_test_rderr:
		pr_err("%s: Fail read test record\n", __func__);
		goto cyttsp4_ic_grpdata_show_prerr;
		break;
	case CY_IC_GRPNUM_PCFG_REC:
		num_read = ts->si_ofs.pcfg_size;
		cyttsp4_dbg(ts, CY_DBG_LVL_3,
			"%s: GRP=PCFG_REC: num_read=%d at ofs=%d + grpofs=%d\n",
			__func__, num_read,
			ts->si_ofs.pcfg_ofs, ts->ic_grpoffset);
		if (ts->ic_grpoffset >= num_read)
			goto cyttsp4_ic_grpdata_show_ofserr;
		else {
			num_read -= ts->ic_grpoffset;
			retval = _cyttsp4_set_sysinfo_mode(ts);
			if (retval < 0) {
				pr_err("%s: Fail enter Sysinfo mode r=%d\n",
					__func__, retval);
				goto cyttsp4_ic_grpdata_show_pcfg_rderr;
			}
			retval = cyttsp4_read_block_data(ts, ts->ic_grpoffset +
				ts->si_ofs.pcfg_ofs, num_read, ic_buf,
				ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
			if (retval < 0) {
				pr_err("%s: Fail read Sysinfo ddata r=%d\n",
					__func__, retval);
				goto cyttsp4_ic_grpdata_show_pcfg_rderr;
			}
			retval = _cyttsp4_set_operational_mode(ts);
			if (retval < 0) {
				pr_err("%s: Fail enter Operational mode r=%d\n",
					__func__, retval);
			}
		}
		break;
cyttsp4_ic_grpdata_show_pcfg_rderr:
		pr_err("%s: Fail read pcfg record\n", __func__);
		goto cyttsp4_ic_grpdata_show_prerr;
		break;
	case CY_IC_GRPNUM_OPCFG_REC:
		num_read = ts->si_ofs.opcfg_size;
		cyttsp4_dbg(ts, CY_DBG_LVL_3,
			"%s: GRP=OPCFG_REC:"
			" num_read=%d at ofs=%d + grpofs=%d\n",
			__func__, num_read,
			ts->si_ofs.opcfg_ofs, ts->ic_grpoffset);
		if (ts->ic_grpoffset >= num_read)
			goto cyttsp4_ic_grpdata_show_ofserr;
		else {
			num_read -= ts->ic_grpoffset;
			retval = _cyttsp4_set_sysinfo_mode(ts);
			if (retval < 0) {
				pr_err("%s: Fail enter Sysinfo mode r=%d\n",
					__func__, retval);
				goto cyttsp4_ic_grpdata_show_opcfg_rderr;
			}
			retval = cyttsp4_read_block_data(ts, ts->ic_grpoffset +
				ts->si_ofs.opcfg_ofs, num_read, ic_buf,
				ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
			if (retval < 0) {
				pr_err("%s: Fail read Sysinfo ddata r=%d\n",
					__func__, retval);
				goto cyttsp4_ic_grpdata_show_opcfg_rderr;
			}
			retval = _cyttsp4_set_operational_mode(ts);
			if (retval < 0) {
				pr_err("%s: Fail enter Operational mode r=%d\n",
					__func__, retval);
			}
		}
		break;
cyttsp4_ic_grpdata_show_opcfg_rderr:
		pr_err("%s: Fail read opcfg record\n", __func__);
		goto cyttsp4_ic_grpdata_show_prerr;
		break;
	case CY_IC_GRPNUM_TCH_PARM_VAL:
		if (ts->flags & CY_FLAG_TMA400) {
			retval = _cyttsp4_tma400_show_tch_param(ts,
				ic_buf, &num_read);
			if (retval < 0) {
				pr_err("%s: Fail show Touch Parameters"
					" for TMA400 r=%d\n", __func__, retval);
				goto cyttsp4_ic_grpdata_show_tch_rderr;
			} else
				retval = 0;
			goto cyttsp4_ic_grpdata_show_tch_rdexit;
		}
		ndata = CY_NUM_CONFIG_BYTES;
		/* do not show cmd, block size and end of block bytes */
		num_read = ndata - (6+4+6);
		cyttsp4_dbg(ts, CY_DBG_LVL_3,
			"%s: GRP=PARM_VAL: num_read=%d at ofs=%d + grpofs=%d\n",
			__func__, num_read,
			0, ts->ic_grpoffset);
		if (ts->ic_grpoffset >= num_read)
			goto cyttsp4_ic_grpdata_show_ofserr;
		else {
			blockid = CY_TCH_PARM_BLKID;
			pdata = kzalloc(ndata, GFP_KERNEL);
			if (pdata == NULL) {
				pr_err("%s: Failed to allocate read buffer"
					" for %s\n",
					__func__, "platform_touch_param_data");
				retval = -ENOMEM;
				goto cyttsp4_ic_grpdata_show_tch_rderr;
			}
			cyttsp4_dbg(ts, CY_DBG_LVL_3,
				"%s: read config block=0x%02X\n",
				__func__, blockid);
			retval = _cyttsp4_set_config_mode(ts);
			if (retval < 0) {
				pr_err("%s: Failed to switch to config mode\n",
					__func__);
				goto cyttsp4_ic_grpdata_show_tch_rderr;
			}
			retval = _cyttsp4_read_config_block(ts,
				blockid, pdata, ndata,
				"platform_touch_param_data");
			if (retval < 0) {
				pr_err("%s: Failed read config block %s r=%d\n",
					__func__, "platform_touch_param_data",
					retval);
				goto cyttsp4_ic_grpdata_show_tch_rderr;
			}
			retval = _cyttsp4_set_operational_mode(ts);
			if (retval < 0) {
				ts->power_state = CY_IDLE_STATE;
				cyttsp4_pr_state(ts);
				pr_err("%s: Fail set operational mode (r=%d)\n",
					__func__, retval);
				goto cyttsp4_ic_grpdata_show_tch_rderr;
			}
			cyttsp4_dbg(ts, CY_DBG_LVL_3,
				"%s: memcpy config block=0x%02X\n",
				__func__, blockid);
			num_read -= ts->ic_grpoffset;
			/*
			 * cmd+rdy_bit, status, ebid, lenh, lenl, reserved,
			 * data[0] .. data[ndata-6]
			 * skip data[0] .. data[3] - block size bytes
			 */
			memcpy(ic_buf,
				&pdata[6+4] + ts->ic_grpoffset, num_read);
			kfree(pdata);
		}
		break;
cyttsp4_ic_grpdata_show_tch_rderr:
		if (pdata != NULL)
			kfree(pdata);
		goto cyttsp4_ic_grpdata_show_prerr;
cyttsp4_ic_grpdata_show_tch_rdexit:
		break;
	case CY_IC_GRPNUM_TCH_PARM_SIZ:
		if (ts->platform_data->sett
			[CY_IC_GRPNUM_TCH_PARM_SIZ] == NULL) {
			pr_err("%s: Missing platform data"
				" Touch Parameters Sizes table\n", __func__);
			goto cyttsp4_ic_grpdata_show_prerr;
		}
		if (ts->platform_data->sett
			[CY_IC_GRPNUM_TCH_PARM_SIZ]->data == NULL) {
			pr_err("%s: Missing platform data"
				" Touch Parameters Sizes table data\n",
				__func__);
			goto cyttsp4_ic_grpdata_show_prerr;
		}
		num_read = ts->platform_data->sett
			[CY_IC_GRPNUM_TCH_PARM_SIZ]->size;
		cyttsp4_dbg(ts, CY_DBG_LVL_2,
			"%s: GRP=PARM_SIZ:"
			" num_read=%d at ofs=%d + grpofs=%d\n",
			__func__, num_read,
			0, ts->ic_grpoffset);
		if (ts->ic_grpoffset >= num_read)
			goto cyttsp4_ic_grpdata_show_ofserr;
		else {
			num_read -= ts->ic_grpoffset;
			memcpy(ic_buf, (u8 *)ts->platform_data->sett
				[CY_IC_GRPNUM_TCH_PARM_SIZ]->data +
				ts->ic_grpoffset, num_read);
		}
		break;
	case CY_IC_GRPNUM_DDATA_REC:
		num_read = ts->si_ofs.ddata_size;
		cyttsp4_dbg(ts, CY_DBG_LVL_3,
			"%s: GRP=DDATA_REC:"
			" num_read=%d at ofs=%d + grpofs=%d\n",
			__func__, num_read,
			ts->si_ofs.ddata_ofs, ts->ic_grpoffset);
		if (ts->ic_grpoffset >= num_read)
			goto cyttsp4_ic_grpdata_show_ofserr;
		else {
			num_read -= ts->ic_grpoffset;
			retval = _cyttsp4_set_sysinfo_mode(ts);
			if (retval < 0) {
				pr_err("%s: Fail enter Sysinfo mode r=%d\n",
					__func__, retval);
				goto cyttsp4_ic_grpdata_show_ddata_rderr;
			}
			retval = cyttsp4_read_block_data(ts, ts->ic_grpoffset +
				ts->si_ofs.ddata_ofs, num_read, ic_buf,
				ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
			if (retval < 0) {
				pr_err("%s: Fail read Sysinfo ddata r=%d\n",
					__func__, retval);
				goto cyttsp4_ic_grpdata_show_ddata_rderr;
			}
			retval = _cyttsp4_set_operational_mode(ts);
			if (retval < 0) {
				pr_err("%s: Fail enter Operational mode r=%d\n",
					__func__, retval);
			}
		}
		break;
cyttsp4_ic_grpdata_show_ddata_rderr:
		pr_err("%s: Fail read ddata\n", __func__);
		goto cyttsp4_ic_grpdata_show_prerr;
		break;
	case CY_IC_GRPNUM_MDATA_REC:
		num_read = ts->si_ofs.mdata_size;
		cyttsp4_dbg(ts, CY_DBG_LVL_3,
			"%s: GRP=MDATA_REC:"
			" num_read=%d at ofs=%d + grpofs=%d\n",
			__func__, num_read,
			ts->si_ofs.mdata_ofs, ts->ic_grpoffset);
		if (ts->ic_grpoffset >= num_read)
			goto cyttsp4_ic_grpdata_show_ofserr;
		else {
			num_read -= ts->ic_grpoffset;
			retval = _cyttsp4_set_sysinfo_mode(ts);
			if (retval < 0) {
				pr_err("%s: Fail enter Sysinfo mode r=%d\n",
					__func__, retval);
				goto cyttsp4_ic_grpdata_show_mdata_rderr;
			}
			retval = cyttsp4_read_block_data(ts, ts->ic_grpoffset +
				ts->si_ofs.mdata_ofs, num_read, ic_buf,
				ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
			if (retval < 0) {
				pr_err("%s: Fail read Sysinfo regs r=%d\n",
					__func__, retval);
				goto cyttsp4_ic_grpdata_show_mdata_rderr;
			}
			retval = _cyttsp4_set_operational_mode(ts);
			if (retval < 0) {
				pr_err("%s: Fail enter Operational mode r=%d\n",
					__func__, retval);
			}
		}
		break;
cyttsp4_ic_grpdata_show_mdata_rderr:
		pr_err("%s: Fail read mdata\n", __func__);
		goto cyttsp4_ic_grpdata_show_prerr;
		break;
	default:
		goto cyttsp4_ic_grpdata_show_grperr;
		break;
	}

	snprintf(buf, CY_MAX_PRBUF_SIZE,
		"Group %d, Offset %u:\n", ts->ic_grpnum, ts->ic_grpoffset);
	for (i = 0; i < num_read; i++) {
		snprintf(buf, CY_MAX_PRBUF_SIZE,
			"%s0x%02X\n", buf, ic_buf[i]);
	}
	kfree(ic_buf);
	return snprintf(buf, CY_MAX_PRBUF_SIZE,
		"%s(%d bytes)\n", buf, num_read);

cyttsp4_ic_grpdata_show_ofserr:
	pr_err("%s: Group Offset=%d exceeds Group Read Length=%d\n",
		__func__, ts->ic_grpoffset, num_read);
	kfree(ic_buf);
	snprintf(buf, CY_MAX_PRBUF_SIZE,
		"Cannot read Group %d Data.\n",
		ts->ic_grpnum);
	return snprintf(buf, CY_MAX_PRBUF_SIZE,
		"%sGroup Offset=%d exceeds Group Read Length=%d\n",
		buf, ts->ic_grpoffset, num_read);
cyttsp4_ic_grpdata_show_prerr:
	pr_err("%s: Cannot read Group %d Data.\n",
		__func__, ts->ic_grpnum);
	kfree(ic_buf);
	return snprintf(buf, CY_MAX_PRBUF_SIZE,
		"Cannot read Group %d Data.\n",
		ts->ic_grpnum);
cyttsp4_ic_grpdata_show_grperr:
	pr_err("%s: Group %d does not exist.\n",
		__func__, ts->ic_grpnum);
	kfree(ic_buf);
	return snprintf(buf, CY_MAX_PRBUF_SIZE,
		"Group %d does not exist.\n",
		ts->ic_grpnum);
}
static ssize_t cyttsp4_ic_grpdata_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyttsp4 *ts = dev_get_drvdata(dev);
	ssize_t retval = 0;

	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

	mutex_lock(&ts->data_lock);
	if (ts->power_state == CY_SLEEP_STATE) {
		pr_err("%s: Group Show Test blocked: IC suspended\n", __func__);
		retval = snprintf(buf, CY_MAX_PRBUF_SIZE,
			"Group %d Show Test blocked: IC suspended\n",
			ts->ic_grpnum);
	} else
		retval = _cyttsp4_ic_grpdata_show(dev, attr, buf);
	mutex_unlock(&ts->data_lock);

	return retval;
}

static int _cyttsp4_tma400_store_tch_param(struct cyttsp4 *ts,
	u8 *ic_buf, size_t length)
{
	int retval = 0;
	int next_data;
	int num_data;
	int start_addr;
	int end_addr;
	int start_block;
	int end_block;
	int block_id;
	int num_block;
	size_t block_size;
	u8 calc_ic_crc[4];
	u8 *pdata;

	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

	retval = _cyttsp4_set_config_mode(ts);
	if (retval < 0) {
		pr_err("%s: Fail switch to config mode r=%d\n",
			__func__, retval);
		goto _cyttsp4_tma400_store_tch_param_err;
	}

	retval = _cyttsp4_get_block_size(ts, &block_size);
	if (retval < 0) {
		pr_err("%s: Fail get block size r=%d\n", __func__, retval);
		goto _cyttsp4_tma400_store_tch_param_exit;
	}

	start_addr = ts->ic_grpoffset;
	next_data = 0;
	end_addr = start_addr + length;
	start_block = start_addr / block_size;
	start_addr %= block_size;
	end_block = end_addr / block_size;
	end_addr %= block_size;
	num_block = end_block - start_block + 1;

	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: start_addr=0x%04X(%d) size=%d start_block=%d end_block=%d"
		" end_addr=%04X(%d) num_block=%d\n",
		__func__,
		start_addr, start_addr, block_size, start_block,
		end_block, end_addr, end_addr, num_block);

	pdata = kzalloc(block_size, GFP_KERNEL);
	if (pdata == NULL) {
		pr_err("%s: Fail allocate block buffer\n", __func__);
		retval = -ENOMEM;
		goto _cyttsp4_tma400_store_tch_param_exit;
	}

	for (block_id = start_block;
		block_id < start_block + num_block; block_id++) {
		cyttsp4_dbg(ts, CY_DBG_LVL_3,
			"%s: get block=%d\n", __func__, block_id);
		retval = _cyttsp4_get_block_data(ts,
			block_id, block_size, pdata);
		if (retval < 0) {
			pr_err("%s: Fail get block=%d r=%d\n",
				__func__, block_id, retval);
			goto _cyttsp4_tma400_store_tch_param_exit;
		}
		num_data = block_size - start_addr;
		if (block_id == end_block)
			num_data -= block_size - end_addr;
		memcpy(&pdata[start_addr], &ic_buf[next_data], num_data);
		next_data += num_data;
		cyttsp4_dbg(ts, CY_DBG_LVL_2,
			"%s: put_block=%d size=%d pdata=%p start_addr=%04X"
			" &pdata[start_addr]=%p num_data=%d\n", __func__,
			block_id, block_size, pdata, start_addr,
			&pdata[start_addr], num_data);
		_cyttsp4_pr_buf(ts, &pdata[start_addr], num_data, "put_block");
		_cyttsp4_pr_buf(ts, pdata, block_size, "print_block");
		retval = _cyttsp4_put_block_data(ts,
			block_id, block_size, pdata);
		if (retval < 0) {
			pr_err("%s: Fail put block=%d r=%d\n",
				__func__, block_id, retval);
			goto _cyttsp4_tma400_store_tch_param_exit;
		}

		start_addr = 0;
		ts->ic_grptest = true;
	}

	/* Update CRC bytes to force restore on reboot */
	if (ts->ic_grptest) {
		memset(calc_ic_crc, 0, sizeof(calc_ic_crc));
		cyttsp4_dbg(ts, CY_DBG_LVL_3,
			"%s: Calc IC CRC values\n", __func__);
		retval = _cyttsp4_calc_ic_crc_tma400(ts,
			calc_ic_crc, sizeof(calc_ic_crc));
		if (retval < 0) {
			pr_err("%s: Fail calc ic crc r=%d\n",
				__func__, retval);
		}
		block_id = 0;
		retval = _cyttsp4_get_block_data(ts,
			block_id, block_size, pdata);
		if (retval < 0) {
			pr_err("%s: Fail get block=%d r=%d\n",
				__func__, block_id, retval);
			goto _cyttsp4_tma400_store_tch_param_exit;
		}
		memcpy(pdata, calc_ic_crc, sizeof(calc_ic_crc));
		retval = _cyttsp4_put_block_data(ts,
			block_id, block_size, pdata);
		if (retval < 0) {
			pr_err("%s: Fail put block=%d r=%d\n",
				__func__, block_id, retval);
			goto _cyttsp4_tma400_store_tch_param_exit;
		}
	}

_cyttsp4_tma400_store_tch_param_exit:
	retval = _cyttsp4_set_operational_mode(ts);
	if (retval < 0) {
		pr_err("%s: Fail switch to operational mode r=%d\n",
			__func__, retval);
	}
	if (pdata != NULL)
		kfree(pdata);
_cyttsp4_tma400_store_tch_param_err:
	return retval;
}

static int _cyttsp4_write_mddata(struct cyttsp4 *ts, size_t write_length,
	size_t mddata_length, u8 blkid, size_t mddata_ofs,
	u8 *ic_buf, const char *mddata_name)
{
	bool mddata_updated = false;
	u8 *pdata;
	int retval = 0;

	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

	pdata = kzalloc(CY_MAX_PRBUF_SIZE, GFP_KERNEL);
	if (pdata == NULL) {
		pr_err("%s: Fail allocate data buffer\n", __func__);
		retval = -ENOMEM;
		goto cyttsp4_write_mddata_exit;
	}
	if (ts->current_mode != CY_MODE_OPERATIONAL) {
		pr_err("%s: Must be in operational mode to start write of"
			" %s (current mode=%d)\n",
			__func__, mddata_name, ts->current_mode);
		retval = -EPERM;
		goto cyttsp4_write_mddata_exit;
	}
	if ((write_length + ts->ic_grpoffset) > mddata_length) {
		pr_err("%s: Requested length(%d) is greater than"
			" %s size(%d)\n", __func__,
			write_length, mddata_name, mddata_length);
		retval = -EINVAL;
		goto cyttsp4_write_mddata_exit;
	}
	retval = _cyttsp4_set_sysinfo_mode(ts);
	if (retval < 0) {
		pr_err("%s: Fail to enter Sysinfo mode r=%d\n",
			__func__, retval);
		goto cyttsp4_write_mddata_exit;
	}
	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: blkid=%02X mddata_ofs=%d mddata_length=%d"
		" mddata_name=%s write_length=%d grpofs=%d\n",
		__func__, blkid, mddata_ofs, mddata_length, mddata_name,
		write_length, ts->ic_grpoffset);
	cyttsp4_read_block_data(ts, mddata_ofs, mddata_length, pdata,
		ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
	if (retval < 0) {
		pr_err("%s: Fail to read %s regs r=%d\n",
			__func__, mddata_name, retval);
		goto cyttsp4_write_mddata_exit;
	}
	memcpy(pdata + ts->ic_grpoffset, ic_buf, write_length);
	_cyttsp4_set_data_block(ts, blkid, pdata,
		mddata_length, mddata_name, true, &mddata_updated);
	if ((retval < 0) || !mddata_updated) {
		pr_err("%s: Fail while writing %s block r=%d updated=%d\n",
			__func__, mddata_name, retval, (int)mddata_updated);
	}
	retval = _cyttsp4_set_operational_mode(ts);
	if (retval < 0) {
		pr_err("%s: Fail to enter Operational mode r=%d\n",
			__func__, retval);
	}

cyttsp4_write_mddata_exit:
	kfree(pdata);
	return retval;
}
static ssize_t _cyttsp4_ic_grpdata_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct cyttsp4 *ts = dev_get_drvdata(dev);
	unsigned long value;
	int retval = 0;
	const char *pbuf = buf;
	int i, j;
	char *scan_buf = NULL;
	u8 *ic_buf = NULL;
	u8 *pdata = NULL;
	size_t length;
	size_t mddata_length, ndata;
	u8 blockid = 0;
	bool mddata_updated = false;
	const char *mddata_name = "invalid name";

	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

	scan_buf = kzalloc(CY_MAX_PRBUF_SIZE, GFP_KERNEL);
	if (scan_buf == NULL) {
		pr_err("%s: Failed to allocate scan buffer for"
			" Group Data store\n", __func__);
		goto cyttsp4_ic_grpdata_store_exit;
	}
	ic_buf = kzalloc(CY_MAX_PRBUF_SIZE, GFP_KERNEL);
	if (ic_buf == NULL) {
		pr_err("%s: Failed to allocate ic buffer for"
			" Group Data store\n", __func__);
		goto cyttsp4_ic_grpdata_store_exit;
	}
	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: grpnum=%d grpoffset=%u\n",
		__func__, ts->ic_grpnum, ts->ic_grpoffset);

	if (ts->ic_grpnum >= CY_IC_GRPNUM_NUM) {
		pr_err("%s: Group %d does not exist.\n",
			__func__, ts->ic_grpnum);
		retval = size;
		goto cyttsp4_ic_grpdata_store_exit;
	}
	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: pbuf=%p buf=%p size=%d sizeof(scan_buf)=%d buf=%s\n",
		__func__, pbuf, buf, size, sizeof(scan_buf), buf);

	i = 0;
	while (pbuf <= (buf + size)) {
		while (((*pbuf == ' ') || (*pbuf == ',')) &&
			(pbuf < (buf + size)))
			pbuf++;
		if (pbuf < (buf + size)) {
			memset(scan_buf, 0, CY_MAX_PRBUF_SIZE);
			for (j = 0; j < sizeof("0xHH") &&
				*pbuf != ' ' && *pbuf != ','; j++)
				scan_buf[j] = *pbuf++;
			retval = strict_strtoul(scan_buf, 16, &value);
			if (retval < 0) {
				pr_err("%s: Invalid data format. "
					"Use \"0xHH,...,0xHH\" instead.\n",
					__func__);
				retval = size;
				goto cyttsp4_ic_grpdata_store_exit;
			} else {
				if (i >= ts->max_config_bytes) {
					pr_err("%s: Max command size exceeded"
					" (size=%d max=%d)\n", __func__,
					i, ts->max_config_bytes);
					goto cyttsp4_ic_grpdata_store_exit;
				}
				ic_buf[i] = value;
				cyttsp4_dbg(ts, CY_DBG_LVL_3,
					"%s: ic_buf[%d] = 0x%02X\n",
					__func__, i, ic_buf[i]);
				i++;
			}
		} else
			break;
	}
	length = i;

	/* write ic_buf to log */
	_cyttsp4_pr_buf(ts, ic_buf, length, "ic_buf");

	switch (ts->ic_grpnum) {
	case CY_IC_GRPNUM_CMD_REGS:
		if ((length + ts->ic_grpoffset + ts->si_ofs.cmd_ofs) >
			ts->si_ofs.rep_ofs) {
			pr_err("%s: Length(%d) + offset(%d) + cmd_offset(%d)"
				" is beyond cmd reg space[%d..%d]\n", __func__,
				length, ts->ic_grpoffset, ts->si_ofs.cmd_ofs,
				ts->si_ofs.cmd_ofs, ts->si_ofs.rep_ofs - 1);
			goto cyttsp4_ic_grpdata_store_exit;
		}
		retval = cyttsp4_write_block_data(ts, ts->ic_grpoffset +
			ts->si_ofs.cmd_ofs, length, ic_buf,
			ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
		if (retval < 0) {
			pr_err("%s: Fail write command regs r=%d\n",
				__func__, retval);
		}
		if (!ts->ic_grptest) {
			pr_info("%s: Disabled settings checksum verifications"
				" until next boot.\n", __func__);
			ts->ic_grptest = true;
		}
		break;
	case CY_IC_GRPNUM_TCH_PARM_VAL:
		if (ts->flags & CY_FLAG_TMA400) {
			retval = _cyttsp4_tma400_store_tch_param(ts,
				ic_buf, length);
			if (retval < 0) {
				pr_err("%s: Fail store Touch Parameters"
					" for TMA400 r=%d\n", __func__, retval);
			} else
				retval = 0;
			goto cyttsp4_ic_grpdata_store_exit;
		}
		mddata_name = "Touch Parameters";
		ndata = CY_NUM_CONFIG_BYTES;
		blockid = CY_TCH_PARM_BLKID;
		/* do not show cmd, block size and end of block bytes */
		mddata_length = ndata - (6+4+6);
		cyttsp4_dbg(ts, CY_DBG_LVL_3,
			"%s: GRP=PARM_VAL: write length=%d at ofs=%d +"
			" grpofs=%d\n", __func__, length,
			0, ts->ic_grpoffset);
		if ((length + ts->ic_grpoffset) > mddata_length) {
			pr_err("%s: Requested length(%d) is greater than"
				" %s size(%d)\n", __func__,
				length, mddata_name, mddata_length);
			retval = -EINVAL;
			goto cyttsp4_ic_grpdata_store_tch_wrerr;
		}
		pdata = kzalloc(ndata, GFP_KERNEL);
		if (pdata == NULL) {
			pr_err("%s: Failed to allocate read/write buffer"
				" for %s\n",
				__func__, "platform_touch_param_data");
			retval = -ENOMEM;
			goto cyttsp4_ic_grpdata_store_tch_wrerr;
		}
		cyttsp4_dbg(ts, CY_DBG_LVL_3,
			"%s: read config block=0x%02X\n",
			__func__, blockid);
		retval = _cyttsp4_set_config_mode(ts);
		if (retval < 0) {
			pr_err("%s: Failed to switch to config mode\n",
				__func__);
			goto cyttsp4_ic_grpdata_store_tch_wrerr;
		}
		retval = _cyttsp4_read_config_block(ts,
			blockid, pdata, ndata,
			"platform_touch_param_data");
		if (retval < 0) {
			pr_err("%s: Failed read config block %s r=%d\n",
				__func__, "platform_touch_param_data",
				retval);
			goto cyttsp4_ic_grpdata_store_tch_wrerr;
		}
		/*
		 * cmd+rdy_bit, status, ebid, lenh, lenl, reserved,
		 * data[0] .. data[ndata-6]
		 * skip data[0] .. data[3] - block size bytes
		 */
		memcpy(&pdata[6+4+ts->ic_grpoffset], ic_buf, length);
		_cyttsp4_set_data_block(ts, blockid, &pdata[6+4],
			mddata_length, mddata_name, true, &mddata_updated);
		if ((retval < 0) || !mddata_updated) {
			pr_err("%s: Fail while writing %s block r=%d"
				" updated=%d\n", __func__,
				mddata_name, retval, (int)mddata_updated);
		}
		if (!ts->ic_grptest) {
			pr_info("%s: Disabled settings checksum verifications"
				" until next boot.\n", __func__);
			ts->ic_grptest = true;
		}
		retval = _cyttsp4_startup(ts);
		if (retval < 0) {
			pr_err("%s: Fail restart after writing params r=%d\n",
				__func__, retval);
		}
cyttsp4_ic_grpdata_store_tch_wrerr:
		kfree(pdata);
		break;
	case CY_IC_GRPNUM_DDATA_REC:
		if (ts->flags & CY_FLAG_TMA400) {
			pr_info("%s: store Design Data not supported"
				" for TMA400 (use store Touch Parameters)\n",
				__func__);
			goto cyttsp4_ic_grpdata_store_exit;
		}
		mddata_length = ts->si_ofs.ddata_size;
		cyttsp4_dbg(ts, CY_DBG_LVL_3,
			"%s: DDATA_REC length=%d mddata_length=%d blkid=%02X"
			" ddata_ofs=%d name=%s\n", __func__, length,
			mddata_length, CY_DDATA_BLKID, ts->si_ofs.ddata_ofs,
			"Design Data");
		_cyttsp4_pr_buf(ts, ic_buf, length, "Design Data");
		retval = _cyttsp4_write_mddata(ts, length, mddata_length,
			CY_DDATA_BLKID, ts->si_ofs.ddata_ofs, ic_buf,
			"Design Data");
		if (retval < 0) {
			pr_err("%s: Fail writing Design Data\n",
				__func__);
		} else if (!ts->ic_grptest) {
			pr_info("%s: Disabled settings checksum verifications"
				" until next boot.\n", __func__);
			ts->ic_grptest = true;
		}
		break;
	case CY_IC_GRPNUM_MDATA_REC:
		if (ts->flags & CY_FLAG_TMA400) {
			pr_info("%s: store Manufacturing Data not supported"
				" for TMA400\n", __func__);
			goto cyttsp4_ic_grpdata_store_exit;
		}
		mddata_length = ts->si_ofs.mdata_size;
		cyttsp4_dbg(ts, CY_DBG_LVL_3,
			"%s: MDATA_REC length=%d mddata_length=%d blkid=%02X"
			" ddata_ofs=%d name=%s\n", __func__, length,
			mddata_length, CY_MDATA_BLKID, ts->si_ofs.mdata_ofs,
			"Manufacturing Data");
		_cyttsp4_pr_buf(ts, ic_buf, length, "Manufacturing Data");
		retval = _cyttsp4_write_mddata(ts, length, mddata_length,
			CY_MDATA_BLKID, ts->si_ofs.mdata_ofs, ic_buf,
			"Manufacturing Data");
		if (retval < 0) {
			pr_err("%s: Fail writing Manufacturing Data\n",
				__func__);
		} else if (!ts->ic_grptest) {
			pr_info("%s: Disabled settings checksum verifications"
				" until next boot.\n", __func__);
			ts->ic_grptest = true;
		}
		break;
	default:
		pr_err("%s: Group=%d is read only\n",
			__func__, ts->ic_grpnum);
		break;
	}

cyttsp4_ic_grpdata_store_exit:
	kfree(scan_buf);
	kfree(ic_buf);
	return size;
}
static ssize_t cyttsp4_ic_grpdata_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct cyttsp4 *ts = dev_get_drvdata(dev);
	ssize_t retval = 0;

	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

	mutex_lock(&ts->data_lock);
	if (ts->power_state == CY_SLEEP_STATE) {
		pr_err("%s: Group Store Test blocked: IC suspended\n",
			__func__);
		retval = size;
	} else
		retval = _cyttsp4_ic_grpdata_store(dev, attr, buf, size);
	mutex_unlock(&ts->data_lock);

	return retval;
}
static DEVICE_ATTR(ic_grpdata, S_IRUSR | S_IWUSR,
	cyttsp4_ic_grpdata_show, cyttsp4_ic_grpdata_store);

static ssize_t cyttsp4_drv_flags_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyttsp4 *ts = dev_get_drvdata(dev);

	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

	return snprintf(buf, CY_MAX_PRBUF_SIZE,
		"Current Driver Flags: 0x%04X\n", ts->flags);
}
static ssize_t cyttsp4_drv_flags_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct cyttsp4 *ts = dev_get_drvdata(dev);
	unsigned long value = 0;
	ssize_t retval = 0;

	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

	mutex_lock(&(ts->data_lock));
	retval = strict_strtoul(buf, 16, &value);
	if (retval < 0) {
		pr_err("%s: Failed to convert value\n", __func__);
		goto cyttsp4_drv_flags_store_error_exit;
	}

	if (value > 0xFFFF) {
		pr_err("%s: value=%lu is greater than max;"
			" drv_flags=0x%04X\n", __func__, value, ts->flags);
	} else {
		ts->flags = value;
	}

	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: drv_flags=0x%04X\n", __func__, ts->flags);

cyttsp4_drv_flags_store_error_exit:
	retval = size;
	mutex_unlock(&(ts->data_lock));
	return retval;
}
static DEVICE_ATTR(drv_flags, S_IRUSR | S_IWUSR,
	cyttsp4_drv_flags_show, cyttsp4_drv_flags_store);

static ssize_t cyttsp4_hw_power_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct cyttsp4 *ts = dev_get_drvdata(dev);
	int is_on;
	char* stat = "Unknown";

	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

	if (ts->platform_data->hw_power) {
		is_on = ts->platform_data->hw_power(99);
		stat = (is_on) ? "On" : "Off";
	}

	return snprintf(buf, CY_MAX_PRBUF_SIZE, "Power: %s\n", stat);
}
static ssize_t cyttsp4_hw_power_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct cyttsp4 *ts = dev_get_drvdata(dev);
	unsigned long value = 0;
	ssize_t retval = 0;

	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);


	mutex_lock(&(ts->data_lock));
	retval = strict_strtoul(buf, 10, &value);
	if (retval < 0) {
		pr_err("%s: Failed to convert value\n", __func__);
		goto cyttsp4_hw_power_error_exit;
	}

	if (value < 0 || value > 1) {
		pr_err("%s: Invalid value %lu\n", __func__, value);
	} else {
		if (ts->platform_data->hw_power)
			retval = ts->platform_data->hw_power((int)value);
	}

	if (retval < 0) {
		pr_err("%s: fail hw_power r=%d\n", __func__, retval);
	}

cyttsp4_hw_power_error_exit:
	retval = size;
	mutex_unlock(&(ts->data_lock));
	return retval;
}
static DEVICE_ATTR(hw_power, S_IRUSR | S_IWUSR,
		cyttsp4_hw_power_show, cyttsp4_hw_power_store);

static ssize_t cyttsp4_hw_reset_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct cyttsp4 *ts = dev_get_drvdata(dev);
	ssize_t retval = 0;

	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

	mutex_lock(&(ts->data_lock));
	retval = _cyttsp4_startup(ts);
	mutex_unlock(&(ts->data_lock));
	if (retval < 0) {
		pr_err("%s: fail hw_reset device restart r=%d\n",
			__func__, retval);
	}

	retval = size;
	return retval;
}
static DEVICE_ATTR(hw_reset, S_IWUSR, NULL, cyttsp4_hw_reset_store);

static ssize_t cyttsp4_hw_recov_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct cyttsp4 *ts = dev_get_drvdata(dev);
	unsigned long value = 0;
	ssize_t retval = 0;

	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

	mutex_lock(&(ts->data_lock));
	retval = strict_strtoul(buf, 10, &value);
	if (retval < 0) {
		pr_err("%s: Failed to convert value\n", __func__);
		goto cyttsp4_hw_recov_store_error_exit;
	}

	if (ts->platform_data->hw_recov == NULL) {
		pr_err("%s: no hw_recov function\n", __func__);
		goto cyttsp4_hw_recov_store_error_exit;
	}

	retval = ts->platform_data->hw_recov((int)value);
	if (retval < 0) {
		pr_err("%s: fail hw_recov(value=%d) function r=%d\n",
			__func__, (int)value, retval);
	}

cyttsp4_hw_recov_store_error_exit:
	retval = size;
	mutex_unlock(&(ts->data_lock));
	return retval;
}
static DEVICE_ATTR(hw_recov, S_IWUSR, NULL, cyttsp4_hw_recov_store);
#endif

#define CY_CMD_I2C_ADDR					0
#define CY_STATUS_SIZE_BYTE				1
#define CY_STATUS_TYP_DELAY				2
#define CY_CMD_TAIL_LEN					3
#define CY_CMD_BYTE					1
#define CY_STATUS_BYTE					1
#define CY_MAX_STATUS_SIZE				32
#define CY_MIN_STATUS_SIZE				5
#define CY_START_OF_PACKET				0x01
#define CY_END_OF_PACKET				0x17
#define CY_DATA_ROW_SIZE				128
//#define CY_PACKET_DATA_LEN				96
#define CY_PACKET_DATA_LEN				64
#define CY_MAX_PACKET_LEN				512
#define CY_COMM_BUSY					0xFF
#define CY_CMD_BUSY					0xFE
#define CY_SEPARATOR_OFFSET				0
#define CY_ARRAY_ID_OFFSET				0
#define CY_ROW_NUM_OFFSET				1
#define CY_ROW_SIZE_OFFSET				3
#define CY_ROW_DATA_OFFSET				5
#define CY_FILE_SILICON_ID_OFFSET			0
#define CY_FILE_REV_ID_OFFSET				4
#define CY_CMD_LDR_ENTER				0x38
#define CY_CMD_LDR_ENTER_STAT_SIZE			15
#define CY_CMD_LDR_ENTER_DLY				20
#define CY_CMD_LDR_ERASE_ROW				0x34
#define CY_CMD_LDR_ERASE_ROW_STAT_SIZE			7
#define CY_CMD_LDR_ERASE_ROW_DLY			20
#define CY_CMD_LDR_SEND_DATA				0x37
#define CY_CMD_LDR_SEND_DATA_STAT_SIZE			7
#define CY_CMD_LDR_SEND_DATA_DLY			5
#define CY_CMD_LDR_PROG_ROW				0x39
#define CY_CMD_LDR_PROG_ROW_STAT_SIZE			7
#define CY_CMD_LDR_PROG_ROW_DLY				30
#define CY_CMD_LDR_VERIFY_ROW				0x3A
#define CY_CMD_LDR_VERIFY_ROW_STAT_SIZE			8
#define CY_CMD_LDR_VERIFY_ROW_DLY			20
#define CY_CMD_LDR_VERIFY_CHKSUM			0x31
#define CY_CMD_LDR_VERIFY_CHKSUM_STAT_SIZE		8
#define CY_CMD_LDR_VERIFY_CHKSUM_DLY			20
#define CY_CMD_LDR_EXIT					0x3B
#define CY_CMD_LDR_EXIT_STAT_SIZE			7
#define CY_CMD_LDR_EXIT_DLY				0
#define CY_CMD_LDR_INIT					0x48
#define CY_CMD_LDR_INIT_STAT_SIZE			7
#define CY_CMD_LDR_INIT_DLY				20
/* {i2c_addr, status_size, status delay, cmd...} */
static u8 ldr_enter_cmd[] = {
	0x01, CY_CMD_LDR_ENTER,
	0x00, 0x00, 0xC7, 0xFF, 0x17
};
static u8 ldr_erase_row_cmd[] = {
	0x01, CY_CMD_LDR_ERASE_ROW,
	0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17
};
static u8 ldr_send_data_cmd[] = {
	0x01, CY_CMD_LDR_SEND_DATA
};
static u8 ldr_prog_row_cmd[] = {
	0x01, CY_CMD_LDR_PROG_ROW
};
static u8 ldr_verify_row_cmd[] = {
	0x01, CY_CMD_LDR_VERIFY_ROW,
	0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17
};
static u8 ldr_verify_chksum_cmd[] = {
	0x01, CY_CMD_LDR_VERIFY_CHKSUM,
	0x00, 0x00, 0xCE, 0xFF, 0x17
};
static u8 ldr_exit_cmd[] = {
	0x01, CY_CMD_LDR_EXIT,
	0x00, 0x00, 0xC4, 0xFF, 0x17
};
static u8 ldr_init_cmd[] = {
	0x01, CY_CMD_LDR_INIT,
	0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17
};

enum ldr_status {
	ERROR_SUCCESS = 0,
	ERROR_COMMAND = 1,
	ERROR_FLASH_ARRAY = 2,
	ERROR_PACKET_DATA = 3,
	ERROR_PACKET_LEN = 4,
	ERROR_PACKET_CHECKSUM = 5,
	ERROR_FLASH_PROTECTION = 6,
	ERROR_FLASH_CHECKSUM = 7,
	ERROR_VERIFY_IMAGE = 8,
	ERROR_UKNOWN1 = 9,
	ERROR_UKNOWN2 = 10,
	ERROR_UKNOWN3 = 11,
	ERROR_UKNOWN4 = 12,
	ERROR_UKNOWN5 = 13,
	ERROR_UKNOWN6 = 14,
	ERROR_INVALID_COMMAND = 15,
	ERROR_INVALID
};

#ifdef CONFIG_TOUCHSCREEN_DEBUG
static const char *ldr_status_string[] = {
	/* Order must match enum ldr_status above */
	"Error Success",
	"Error Command",
	"Error Flash Array",
	"Error Packet Data",
	"Error Packet Length",
	"Error Packet Checksum",
	"Error Flash Protection",
	"Error Flash Checksum",
	"Error Verify Image",
	"Error Invalid Command",
	"Error Invalid Command",
	"Error Invalid Command",
	"Error Invalid Command",
	"Error Invalid Command",
	"Error Invalid Command",
	"Error Invalid Command",
	"Invalid Error Code"
};
#endif

#ifdef CONFIG_TOUCHSCREEN_DEBUG
static void cyttsp4_pr_status(struct cyttsp4 *ts, int level, int status)
{
	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

	if (status > ERROR_INVALID)
		status = ERROR_INVALID;
	cyttsp4_dbg(ts, level,
		"%s: status error(%d)=%s\n",
		__func__, status, ldr_status_string[status]);
}
#endif

static u16 cyttsp4_get_short(u8 *buf)
{
	return ((u16)(*buf) << 8) + *(buf+1);
}

static u8 *cyttsp4_get_row(struct cyttsp4 *ts,
	u8 *row_buf, u8 *image_buf, int size)
{
	int i;

	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

	for (i = 0; i < size; i++) {
		/* copy a row from the image */
		row_buf[i] = image_buf[i];
	}

	image_buf = image_buf + size;
	return image_buf;
}

static u16 cyttsp4_compute_crc(struct cyttsp4 *ts, u8 *buf, int size)
{
	u16 crc = 0xffff;
	u16 tmp;
	int i;

	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

	/* RUN CRC */

	if (size == 0) {
		crc = ~crc;
		goto cyttsp4_compute_crc_exit;
	}

	do {
		for (i = 0, tmp = 0x00ff & *buf++; i < 8;
			i++, tmp >>= 1) {
			if ((crc & 0x0001) ^ (tmp & 0x0001))
				crc = (crc >> 1) ^ 0x8408;
			else
				crc >>= 1;
		}
	} while (--size);

	crc = ~crc;
	tmp = crc;
	crc = (crc << 8) | (tmp >> 8 & 0xFF);

cyttsp4_compute_crc_exit:
	return crc;
}

static u8 ok_status[] = {
	CY_START_OF_PACKET, ERROR_SUCCESS,
	0, 0, 0xFF, 0xFF, CY_END_OF_PACKET
};

static int cyttsp4_get_status(struct cyttsp4 *ts, u8 *buf, int size, int delay)
{
	int tries;
	u16 crc = 0;
	int retval = 0;

	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

	if (!delay) {
		memcpy(buf, ok_status, sizeof(ok_status));
		crc = cyttsp4_compute_crc(ts, ldr_enter_cmd,
			sizeof(ldr_enter_cmd) - CY_CMD_TAIL_LEN);
		ok_status[4] = (u8)crc;
		ok_status[5] = (u8)(crc >> 8);
		retval = 0;
		goto cyttsp4_get_status_exit;
	}

	/* wait until status ready interrupt or timeout occurs */
	tries = 0;
	while (!ts->bl_ready_flag && (tries++ < (delay*2))) {
		if (mutex_is_locked(&ts->data_lock)) {
			mutex_unlock(&ts->data_lock);
			udelay(1000);
			mutex_lock(&ts->data_lock);
		} else
			udelay(1000);
	};

	/* read the status packet */
	retval = cyttsp4_read_block_data(ts, CY_REG_BASE,
		size, buf, ts->platform_data->addr[CY_LDR_ADDR_OFS], false);
	/* retry if bus read error or status byte shows not ready */
	tries = 0;
	while (((retval < 0) ||
		(!(retval < 0) &&
		((buf[1] == CY_COMM_BUSY) ||
		(buf[1] == CY_CMD_BUSY)))) &&
		(tries++ < CY_DELAY_DFLT)) {
		mdelay(delay);
		retval = cyttsp4_read_block_data(ts, CY_REG_BASE,
			size, buf,
			ts->platform_data->addr[CY_LDR_ADDR_OFS], false);
	}
	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: tries=%d ret=%d status=%02X\n",
		__func__, tries, retval, buf[1]);

cyttsp4_get_status_exit:
	return retval;
}

static u8 *cyttsp4_send_cmd(struct cyttsp4 *ts,	const u8 *cmd_buf, int cmd_size,
	int status_size, int status_delay)
{
	int retval;
	u8 *pdata = NULL;
	u8 *status_buf = kzalloc(CY_MAX_STATUS_SIZE, GFP_KERNEL);

	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

	if (status_buf == NULL) {
		pr_err("%s: Fail alloc status buffer=%p\n",
			__func__, status_buf);
		goto cyttsp4_send_cmd_exit;
	}

	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: cmd=%02X %02X %02X %02X %02X\n", __func__,
		cmd_buf[0], cmd_buf[1], cmd_buf[2], cmd_buf[3], cmd_buf[4]);

	if (!cmd_size) {
		pr_err("%s: bad command size=%d\n", __func__, cmd_size);
		goto cyttsp4_send_cmd_exit;
	}

	/* write the command */
	ts->bl_ready_flag = false;
	pdata = kzalloc(cmd_size + 2, GFP_KERNEL);
	if (pdata == NULL) {
		pr_err("%s: Fail get cmd buffer\n", __func__);
		kfree(status_buf);
		status_buf = NULL;
		goto cyttsp4_send_cmd_exit;
	} else if (ts->flags & CY_FLAG_TMA400) {
#ifdef CONFIG_SHTPS_TMA4XX_TMA443
		pdata[0] = 0xFF;
		memcpy(&pdata[1], cmd_buf, cmd_size);
		cmd_size += 1;
#else
		pdata[0] = 0x00;
		pdata[1] = 0xFF;
		memcpy(&pdata[2], cmd_buf, cmd_size);
		cmd_size += 2;
#endif	/* CONFIG_SHTPS_TMA4XX_TMA443 */
	} else
		memcpy(pdata, cmd_buf, cmd_size);

	retval = cyttsp4_write_block_data(ts, CY_REG_BASE,
		cmd_size, pdata,
		ts->platform_data->addr[CY_LDR_ADDR_OFS], false);
	if (retval < 0) {
		pr_err("%s: Fail writing command=%02X\n",
			__func__, cmd_buf[CY_CMD_BYTE]);
		kfree(status_buf);
		status_buf = NULL;
		goto cyttsp4_send_cmd_exit;
	}

	/* get the status */
	retval = cyttsp4_get_status(ts, status_buf, status_size, status_delay);
	if ((retval < 0) || (status_buf[0] != CY_START_OF_PACKET)) {
		pr_err("%s: Error getting status r=%d status_buf[0]=%02X\n",
			__func__, retval, status_buf[0]);
		kfree(status_buf);
		status_buf = NULL;
		goto cyttsp4_send_cmd_exit;
	}

cyttsp4_send_cmd_exit:
	return status_buf;
}

struct cyttsp4_dev_id {
	u32 silicon_id;
	u8 rev_id;
	u32 bl_ver;
};

static int cyttsp4_ldr_enter(struct cyttsp4 *ts, struct cyttsp4_dev_id *dev_id)
{
	u16 crc;
	u8 *status_buf;
	u8 status = 0;
	int retval = 0;

	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

	dev_id->bl_ver = 0;
	dev_id->rev_id = 0;
	dev_id->silicon_id = 0;

	/* write the command */
	crc = cyttsp4_compute_crc(ts, ldr_enter_cmd,
		sizeof(ldr_enter_cmd) - CY_CMD_TAIL_LEN);
	ldr_enter_cmd[4] = (u8)crc;
	ldr_enter_cmd[5] = (u8)(crc >> 8);
	ts->bl_ready_flag = false;
	status_buf = cyttsp4_send_cmd(ts, ldr_enter_cmd,
				sizeof(ldr_enter_cmd),
				CY_CMD_LDR_ENTER_STAT_SIZE,
				CY_CMD_LDR_ENTER_DLY);
	if (status_buf == NULL) {
		pr_err("%s: write block failed %d\n", __func__, retval);
		goto cyttsp4_ldr_enter_exit;
	} else {
		status = status_buf[CY_STATUS_BYTE];
		if (status == ERROR_SUCCESS) {
			dev_id->bl_ver =
				status_buf[11] << 16 |
				status_buf[10] <<  8 |
				status_buf[9] <<  0;
			dev_id->rev_id =
				status_buf[8] <<  0;
			dev_id->silicon_id =
				status_buf[7] << 24 |
				status_buf[6] << 16 |
				status_buf[5] <<  8 |
				status_buf[4] <<  0;
		}
		kfree(status_buf);
#ifdef CONFIG_TOUCHSCREEN_DEBUG
		cyttsp4_pr_status(ts, CY_DBG_LVL_3, status);
#endif
		cyttsp4_dbg(ts, CY_DBG_LVL_3,
			"%s: status=%d "
			"bl_ver=%08X rev_id=%02X silicon_id=%08X\n",
			__func__, status,
			dev_id->bl_ver, dev_id->rev_id, dev_id->silicon_id);
		retval = status;
	}

cyttsp4_ldr_enter_exit:
	return retval;
}

struct cyttsp4_hex_image {
	u8 array_id;
	u16 row_num;
	u16 row_size;
	u8 row_data[CY_DATA_ROW_SIZE];
} __attribute__((packed));

static int cyttsp4_ldr_erase_row(struct cyttsp4 *ts,
	struct cyttsp4_hex_image *row_image)
{
	u16 crc;
	u8 *status_buf;
	u8 status;
	int retval = 0;

	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

	ldr_erase_row_cmd[4] = row_image->array_id;
	ldr_erase_row_cmd[5] = (u8)row_image->row_num;
	ldr_erase_row_cmd[6] = (u8)(row_image->row_num >> 8);
	crc = cyttsp4_compute_crc(ts, ldr_erase_row_cmd,
		sizeof(ldr_erase_row_cmd) - CY_CMD_TAIL_LEN);
	ldr_erase_row_cmd[7] = (u8)crc;
	ldr_erase_row_cmd[8] = (u8)(crc >> 8);

	status_buf = cyttsp4_send_cmd(ts, ldr_erase_row_cmd,
		sizeof(ldr_erase_row_cmd),
		CY_CMD_LDR_ERASE_ROW_STAT_SIZE,
		CY_CMD_LDR_ERASE_ROW_DLY);

	if (status_buf == NULL) {
		status = ERROR_INVALID;
		retval = -EIO;
	} else {
		status = status_buf[CY_STATUS_BYTE];
		kfree(status_buf);
		if (status == ERROR_SUCCESS)
			retval = 0;
		else
			retval = -EIO;
	}

	return retval;
}

static int cyttsp4_ldr_parse_row(u8 *row_buf,
	struct cyttsp4_hex_image *row_image)
{
	u16 i, j;
	int retval = 0;

	if (!row_buf) {
		pr_err("%s parse row error - buf is null\n", __func__);
		retval = -EINVAL;
		goto cyttsp4_ldr_parse_row_exit;
	}

	row_image->array_id = row_buf[CY_ARRAY_ID_OFFSET];
	row_image->row_num = cyttsp4_get_short(&row_buf[CY_ROW_NUM_OFFSET]);
	row_image->row_size = cyttsp4_get_short(&row_buf[CY_ROW_SIZE_OFFSET]);

	if (row_image->row_size > ARRAY_SIZE(row_image->row_data)) {
		pr_err("%s: row data buffer overflow\n", __func__);
		retval = -EOVERFLOW;
		goto cyttsp4_ldr_parse_row_exit;
	}

	for (i = 0, j = CY_ROW_DATA_OFFSET;
		i < row_image->row_size; i++)
		row_image->row_data[i] = row_buf[j++];

	retval = 0;

cyttsp4_ldr_parse_row_exit:
	return retval;
}

static u16 g_row_checksum;

static int cyttsp4_ldr_prog_row(struct cyttsp4 *ts,
	struct cyttsp4_hex_image *row_image)
{
	u8 *status_buf;
	int status;
	u16 crc;
	u16 i, j, k, l, m;
	u16 row_sum = 0;
	int retval = 0;

	u8 *cmd = kzalloc(CY_MAX_PACKET_LEN, GFP_KERNEL);

	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

	g_row_checksum = 0;

	if (cmd) {
		i = 0;
		status = 0;
		row_sum = 0;

		for (l = 0; l < (CY_DATA_ROW_SIZE/CY_PACKET_DATA_LEN)-1; l++) {
			cmd[0] = ldr_send_data_cmd[0];
			cmd[1] = ldr_send_data_cmd[1];
			cmd[2] = (u8)CY_PACKET_DATA_LEN;
			cmd[3] = (u8)(CY_PACKET_DATA_LEN >> 8);
			j = 4;
			m = 4;

			for (k = 0; k < CY_PACKET_DATA_LEN; k++) {
				cmd[j] = row_image->row_data[i];
				row_sum += cmd[j];
				i++;
				j++;
			}

			crc = cyttsp4_compute_crc(ts, cmd,
				CY_PACKET_DATA_LEN+m);
			cmd[CY_PACKET_DATA_LEN+m+0] = (u8)crc;
			cmd[CY_PACKET_DATA_LEN+m+1] = (u8)(crc >> 8);
			cmd[CY_PACKET_DATA_LEN+m+2] = CY_END_OF_PACKET;

			status_buf = cyttsp4_send_cmd(ts, cmd,
				CY_PACKET_DATA_LEN+m+3,
				CY_CMD_LDR_SEND_DATA_STAT_SIZE,
				CY_CMD_LDR_SEND_DATA_DLY);

			if (status_buf == NULL) {
				status = ERROR_INVALID;
				retval = -EIO;
			} else {
				status = status_buf[CY_STATUS_BYTE];
				kfree(status_buf);
				if (status == ERROR_SUCCESS)
					retval = 0;
				else
					retval = -EIO;
			}
			if (retval < 0) {
				pr_err("%s: send row segment %d"
					" fail status=%d\n",
					__func__, l, status);
				goto cyttsp4_ldr_prog_row_exit;
			}
		}

		cmd[0] = ldr_prog_row_cmd[0];
		cmd[1] = ldr_prog_row_cmd[1];
		/*
		 * include array id size and row id size in CY_PACKET_DATA_LEN
		 */
		cmd[2] = (u8)(CY_PACKET_DATA_LEN+3);
		cmd[3] = (u8)((CY_PACKET_DATA_LEN+3) >> 8);
		cmd[4] = row_image->array_id;
		cmd[5] = (u8)row_image->row_num;
		cmd[6] = (u8)(row_image->row_num >> 8);
		j = 7;
		m = 7;

		for (k = 0; k < CY_PACKET_DATA_LEN; k++) {
			cmd[j] = row_image->row_data[i];
			row_sum += cmd[j];
			i++;
			j++;
		}

		crc = cyttsp4_compute_crc(ts, cmd,
			CY_PACKET_DATA_LEN+m);
		cmd[CY_PACKET_DATA_LEN+m+0] = (u8)crc;
		cmd[CY_PACKET_DATA_LEN+m+1] = (u8)(crc >> 8);
		cmd[CY_PACKET_DATA_LEN+m+2] = CY_END_OF_PACKET;

		status_buf = cyttsp4_send_cmd(ts, cmd,
			CY_PACKET_DATA_LEN+m+3,
			CY_CMD_LDR_PROG_ROW_STAT_SIZE,
			CY_CMD_LDR_PROG_ROW_DLY);

		g_row_checksum = 1 + ~row_sum;

		if (status_buf == NULL) {
			status = ERROR_INVALID;
			retval = -EIO;
		} else {
			status = status_buf[CY_STATUS_BYTE];
			kfree(status_buf);
			if (status == ERROR_SUCCESS)
				retval = 0;
			else
				retval = -EIO;
		}
		if (retval < 0) {
			pr_err("%s: prog row fail status=%d\n",
				__func__, status);
			goto cyttsp4_ldr_prog_row_exit;
		}

	} else {
		pr_err("%s prog row error - cmd buf is NULL\n", __func__);
		status = -EIO;
	}

cyttsp4_ldr_prog_row_exit:
	if (cmd != NULL)
		kfree(cmd);
	return retval;
}

static u8 g_verify_checksum;

static int cyttsp4_ldr_verify_row(struct cyttsp4 *ts,
	struct cyttsp4_hex_image *row_image)
{
	u16 crc;
	u8 *status_buf;
	u8 status;
	int retval = 0;

	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

	g_verify_checksum = 0;

	ldr_verify_row_cmd[4] = row_image->array_id;
	ldr_verify_row_cmd[5] = (u8)row_image->row_num;
	ldr_verify_row_cmd[6] = (u8)(row_image->row_num >> 8);
	crc = cyttsp4_compute_crc(ts, ldr_verify_row_cmd,
		sizeof(ldr_verify_row_cmd) - CY_CMD_TAIL_LEN);
	ldr_verify_row_cmd[7] = (u8)crc;
	ldr_verify_row_cmd[8] = (u8)(crc >> 8);

	status_buf = cyttsp4_send_cmd(ts, ldr_verify_row_cmd,
		sizeof(ldr_verify_row_cmd),
		CY_CMD_LDR_VERIFY_ROW_STAT_SIZE,
		CY_CMD_LDR_VERIFY_ROW_DLY);

	status_buf = cyttsp4_send_cmd(ts, ldr_verify_row_cmd,
		sizeof(ldr_verify_row_cmd),
		CY_CMD_LDR_VERIFY_ROW_STAT_SIZE,
		CY_CMD_LDR_VERIFY_ROW_DLY);

	if (status_buf == NULL) {
		status = ERROR_INVALID;
		retval = -EIO;
	} else {
		status = status_buf[CY_STATUS_BYTE];
		g_verify_checksum = status_buf[4];
		kfree(status_buf);
		if (status == ERROR_SUCCESS)
			retval = 0;
		else
			retval = -EIO;
	}
	if (retval < 0) {
		pr_err("%s: verify row fail status=%d\n",
			__func__, status);
	}

	return retval;
}

static int cyttsp4_ldr_verify_chksum(struct cyttsp4 *ts, u8 *app_chksum)
{
	u16 crc;
	u8 *status_buf;
	u8 status;
	int retval = 0;

	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

	*app_chksum = 0;

	crc = cyttsp4_compute_crc(ts, ldr_verify_chksum_cmd,
		sizeof(ldr_verify_chksum_cmd) - CY_CMD_TAIL_LEN);
	ldr_verify_chksum_cmd[4] = (u8)crc;
	ldr_verify_chksum_cmd[5] = (u8)(crc >> 8);
	status_buf = cyttsp4_send_cmd(ts, ldr_verify_chksum_cmd,
		sizeof(ldr_verify_chksum_cmd),
		CY_CMD_LDR_VERIFY_CHKSUM_STAT_SIZE,
		CY_CMD_LDR_VERIFY_CHKSUM_DLY);

	if (status_buf == NULL) {
		status = ERROR_INVALID;
		retval = -EIO;
	} else {
		status = status_buf[CY_STATUS_BYTE];
		*app_chksum = status_buf[4];
		kfree(status_buf);
		if (status == ERROR_SUCCESS)
			retval = 0;
		else
			retval = -EIO;
	}
	if (retval < 0) {
		pr_err("%s: verify checksum fail status=%d\n",
			__func__, status);
	}

	return retval;
}

static int _cyttsp4_ldr_exit(struct cyttsp4 *ts)
{
	u16 crc;
	u8 *status_buf;
	u8 status;
	int retval = 0;

	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

	crc = cyttsp4_compute_crc(ts, ldr_exit_cmd,
		sizeof(ldr_exit_cmd) - CY_CMD_TAIL_LEN);
	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: CRC=0x%04X\n", __func__, crc);
	ldr_exit_cmd[4] = (u8)crc;
	ldr_exit_cmd[5] = (u8)(crc >> 8);
	status_buf = cyttsp4_send_cmd(ts, ldr_exit_cmd,
		sizeof(ldr_exit_cmd),
		CY_CMD_LDR_EXIT_STAT_SIZE,
		CY_CMD_LDR_EXIT_DLY);

	if (status_buf == NULL) {
		status = ERROR_INVALID;
		retval = -EIO;
	} else {
		status = status_buf[CY_STATUS_BYTE];
		kfree(status_buf);
		if (status == ERROR_SUCCESS)
			retval = 0;
		else
			retval = -EIO;
	}
	if (retval < 0) {
		pr_err("%s: loader exit fail status=%d\n",
			__func__, status);
	}

	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: Exit Bootloader status=%d\n",
		__func__, status);

	return retval;
}

static int cyttsp4_ldr_initiate(struct cyttsp4 *ts)
{
	u16 crc;
	u8 *status_buf;
	u8 status;
	int retval = 0;

	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

	memcpy(ldr_init_cmd + 4, cyttsp4_security_key, sizeof(cyttsp4_security_key));
	crc = cyttsp4_compute_crc(ts, ldr_init_cmd,
		sizeof(ldr_init_cmd) - CY_CMD_TAIL_LEN);
	ldr_init_cmd[12] = (u8)crc;
	ldr_init_cmd[13] = (u8)(crc >> 8);

	status_buf = cyttsp4_send_cmd(ts, ldr_init_cmd,
		sizeof(ldr_init_cmd),
		CY_CMD_LDR_INIT_STAT_SIZE,
		CY_CMD_LDR_INIT_DLY);

	if (status_buf == NULL) {
		status = ERROR_INVALID;
		retval = -EIO;
	} else {
		status = status_buf[CY_STATUS_BYTE];
		kfree(status_buf);
		if (status == ERROR_SUCCESS)
			retval = 0;
		else
			retval = -EIO;
	}

	return retval;
}

static int _cyttsp4_load_app(struct cyttsp4 *ts, const u8 *fw, int fw_size)
{
	u8 *p;
	u8 tries;
	int ret;
	int retval;	/* need separate return value at exit stage */
	struct cyttsp4_dev_id *file_id = NULL;
	struct cyttsp4_dev_id *dev_id = NULL;
	struct cyttsp4_hex_image *row_image = NULL;
	u8 app_chksum;
	u8 *row_buf = NULL;
	size_t row_buf_size = 1024 > CY_MAX_PRBUF_SIZE ?
		1024 : CY_MAX_PRBUF_SIZE;
#ifdef CONFIG_TOUCHSCREEN_DEBUG
	int row_count = 0;
#endif

	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

	if (!fw_size || (fw_size % sizeof(struct cyttsp4_hex_image) != 0)) {
		pr_err("%s: Firmware image is misaligned\n", __func__);
		retval = -EINVAL;
		goto _cyttsp4_load_app_exit;
	}

	pr_info("%s: start load app\n", __func__);
	ts->power_state = CY_BL_STATE;
	cyttsp4_pr_state(ts);

	row_buf = kzalloc(row_buf_size, GFP_KERNEL);
	row_image = kzalloc(sizeof(struct cyttsp4_hex_image), GFP_KERNEL);
	file_id = kzalloc(sizeof(struct cyttsp4_dev_id), GFP_KERNEL);
	dev_id = kzalloc(sizeof(struct cyttsp4_dev_id), GFP_KERNEL);
	if ((row_buf == NULL) || (row_image == NULL) ||
		(file_id == NULL) || (dev_id == NULL)) {
		pr_err("%s: Unable to alloc row buffers(%p %p %p %p)\n",
			__func__, row_buf, row_image, file_id, dev_id);
		retval = -ENOMEM;
		goto _cyttsp4_load_app_error_exit;
	}

	p = (u8 *)fw;
	/* Enter Loader and return Silicon ID and Rev */

	INIT_COMPLETION(ts->bl_int_running);
	_cyttsp4_reset(ts);


	if (mutex_is_locked(&ts->data_lock)) {
		mutex_unlock(&ts->data_lock);
		retval = wait_for_completion_interruptible_timeout(
			&ts->bl_int_running,
			msecs_to_jiffies(CY_DELAY_DFLT * CY_DELAY_MAX * 10));
		mutex_lock(&ts->data_lock);
	} else {
		retval = wait_for_completion_interruptible_timeout(&ts->
			bl_int_running,
			msecs_to_jiffies(CY_DELAY_DFLT * CY_DELAY_MAX * 10));
	}
	if (retval == 0) {
		pr_err("%s: time out waiting for bootloader interrupt\n",
			__func__);
		retval = -ETIMEDOUT;
		goto _cyttsp4_load_app_exit;
	}

	ts->power_state = CY_LDR_STATE;
	retval = cyttsp4_ldr_enter(ts, dev_id);
	if (retval) {
		pr_err("%s: Error cannot start PSOC3 Loader (ret=%d)\n",
			__func__, retval);

		goto _cyttsp4_load_app_error_exit;
	} else {
		cyttsp4_dbg(ts, CY_DBG_LVL_3,
			"%s: dev: silicon id=%08X rev=%02X bl=%08X\n",
			__func__,
			dev_id->silicon_id, dev_id->rev_id, dev_id->bl_ver);
	}

	/* Initiate Bootloader */
	retval = cyttsp4_ldr_initiate(ts);
	if (retval) {
		pr_err("%s: Error cannot initiate bootload  (ret=%d)\n",
			__func__, retval);
		goto bl_exit;
	}

	while (p < (fw + fw_size)) {
		/* Get row */
		cyttsp4_dbg(ts, CY_DBG_LVL_1,
			"%s: read row=%d\n", __func__, ++row_count);
		memset(row_buf, 0, row_buf_size);
		p = cyttsp4_get_row(ts, row_buf, p,
			sizeof(struct cyttsp4_hex_image));

		/* Parse row */
		cyttsp4_dbg(ts, CY_DBG_LVL_3,
			"%s: p=%p buf=%p buf[0]=%02X\n", __func__,
			p, row_buf, row_buf[0]);
		retval = cyttsp4_ldr_parse_row(row_buf, row_image);
		cyttsp4_dbg(ts, CY_DBG_LVL_2,
			"%s: array_id=%02X row_num=%04X(%d)"
				" row_size=%04X(%d)\n", __func__,
			row_image->array_id,
			row_image->row_num, row_image->row_num,
			row_image->row_size, row_image->row_size);
		if (retval) {
			pr_err("%s: Parse Row Error "
				"(a=%d r=%d ret=%d\n",
				__func__, row_image->array_id,
				row_image->row_num,
				retval);
			goto bl_exit;
		} else {
			cyttsp4_dbg(ts, CY_DBG_LVL_3,
				"%s: Parse Row "
				"(a=%d r=%d ret=%d\n",
				__func__, row_image->array_id,
				row_image->row_num, retval);
		}

		/* erase row */
		tries = 0;
		do {
			retval = cyttsp4_ldr_erase_row(ts, row_image);
			if (retval) {
				pr_err("%s: Erase Row Error "
					"(array=%d row=%d ret=%d try=%d)\n",
					__func__, row_image->array_id,
					row_image->row_num, retval, tries);
			}
		} while (retval && tries++ < 5);
		if (retval)
			goto _cyttsp4_load_app_error_exit;

		/* program row */
		retval = cyttsp4_ldr_prog_row(ts, row_image);
		if (retval) {
			pr_err("%s: Program Row Error "
				"(array=%d row=%d ret=%d)\n",
				__func__, row_image->array_id,
				row_image->row_num, retval);
		}
		if (retval)
			goto _cyttsp4_load_app_error_exit;

		/* verify row */
		retval = cyttsp4_ldr_verify_row(ts, row_image);
		if (retval) {
			pr_err("%s: Verify Row Error "
				"(array=%d row=%d ret=%d)\n",
				__func__, row_image->array_id,
				row_image->row_num, retval);
			goto _cyttsp4_load_app_error_exit;
		}

		/* verify flash checksum */
		cyttsp4_dbg(ts, CY_DBG_LVL_3,
			"%s: row=%d flashchk=%02X\n",
			__func__, row_count, g_verify_checksum);
		cyttsp4_dbg(ts, CY_DBG_LVL_3,
			"%s: array=%d row_cnt=%d row_num=%04X "
			"row_chksum=%04X ver_chksum=%02X\n",
			__func__, row_image->array_id, row_count,
			row_image->row_num,
			g_row_checksum, g_verify_checksum);
		if ((u8)g_row_checksum !=
			g_verify_checksum) {
			pr_err("%s: Verify Checksum Error "
				"(array=%d row=%d "
				"rchk=%04X vchk=%02X)\n",
				__func__,
				row_image->array_id, row_image->row_num,
				g_row_checksum, g_verify_checksum);
			retval = -EIO;
			goto _cyttsp4_load_app_error_exit;
		}
	}

	/* verify app checksum */
	retval = cyttsp4_ldr_verify_chksum(ts, &app_chksum);
	cyttsp4_dbg(ts, CY_DBG_LVL_1,
		"%s: Application Checksum = %02X r=%d\n",
		__func__, app_chksum, retval);
	if (retval) {
		pr_err("%s: ldr_verify_chksum fail r=%d\n", __func__, retval);
		retval = 0;
	}

	/* exit loader */
bl_exit:
	ret = _cyttsp4_ldr_exit(ts);
	if (ret) {
		pr_err("%s: Error on exit Loader (ret=%d)\n",
			__func__, ret);
		retval = ret;
		goto _cyttsp4_load_app_error_exit;
	}

	/*
	 * this is a temporary parking state;
	 * the driver will always run startup
	 * after the loader has completed
	 */
	ts->power_state = CY_READY_STATE;
	goto _cyttsp4_load_app_exit;

_cyttsp4_load_app_error_exit:
	ts->power_state = CY_BL_STATE;
_cyttsp4_load_app_exit:
	kfree(row_buf);
	kfree(row_image);
	kfree(file_id);
	kfree(dev_id);
	return retval;
}

#ifdef CONFIG_TOUCHSCREEN_DEBUG
/* Force firmware upgrade */
static void cyttsp4_firmware_cont(const struct firmware *fw, void *context)
{
	int retval = 0;
	struct device *dev = context;
	struct cyttsp4 *ts = dev_get_drvdata(dev);
	u8 header_size = 0;

	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

	mutex_lock(&ts->data_lock);

	if (fw == NULL) {
		pr_err("%s: Firmware not found\n", __func__);
		goto cyttsp4_firmware_cont_exit;
	}

	if ((fw->data == NULL) || (fw->size == 0)) {
		pr_err("%s: No firmware received\n", __func__);
		goto cyttsp4_firmware_cont_release_exit;
	}

	header_size = fw->data[0];
	if (header_size >= (fw->size + 1)) {
		pr_err("%s: Firmware format is invalid\n", __func__);
		goto cyttsp4_firmware_cont_release_exit;
	}

	retval = _cyttsp4_load_app(ts, &(fw->data[header_size + 1]),
		fw->size - (header_size + 1));
	if (retval) {
		pr_err("%s: Firmware update failed with error code %d\n",
			__func__, retval);
		ts->power_state = CY_IDLE_STATE;
		cyttsp4_pr_state(ts);
		retval = -EIO;
		goto cyttsp4_firmware_cont_release_exit;
	}

	ts->debug_upgrade = true;

	retval = _cyttsp4_startup(ts);
	if (retval < 0) {
		pr_err("%s: Failed to restart IC with error code %d\n",
			__func__, retval);
		ts->power_state = CY_IDLE_STATE;
		cyttsp4_pr_state(ts);
	}

cyttsp4_firmware_cont_release_exit:
	release_firmware(fw);

cyttsp4_firmware_cont_exit:
	ts->waiting_for_fw = false;
	mutex_unlock(&ts->data_lock);
	return;
}
static ssize_t cyttsp4_ic_reflash_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	static const char *wait_fw_ld = "Driver is waiting for firmware load\n";
	static const char *no_fw_ld = "No firmware loading in progress\n";
	struct cyttsp4 *ts = dev_get_drvdata(dev);

	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

	if (ts->waiting_for_fw)
		return snprintf(buf, strlen(wait_fw_ld)+1, wait_fw_ld);
	else
		return snprintf(buf, strlen(no_fw_ld)+1, no_fw_ld);
}
static ssize_t cyttsp4_ic_reflash_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int i;
	int retval = 0;
	struct cyttsp4 *ts = dev_get_drvdata(dev);

	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

	if (ts->waiting_for_fw) {
		pr_err("%s: Driver is already waiting for firmware\n",
			__func__);
		retval = -EALREADY;
		goto cyttsp4_ic_reflash_store_exit;
	}

	/*
	 * must configure FW_LOADER in .config file
	 * CONFIG_HOTPLUG=y
	 * CONFIG_FW_LOADER=y
	 * CONFIG_FIRMWARE_IN_KERNEL=y
	 * CONFIG_EXTRA_FIRMWARE=""
	 * CONFIG_EXTRA_FIRMWARE_DIR=""
	 */

	if (size > CY_BL_FW_NAME_SIZE) {
		pr_err("%s: Filename too long\n", __func__);
		retval = -ENAMETOOLONG;
		goto cyttsp4_ic_reflash_store_exit;
	} else {
		/*
		 * name string must be in alloc() memory
		 * or is lost on context switch
		 * strip off any line feed character(s)
		 * at the end of the buf string
		 */
		for (i = 0; buf[i]; i++) {
			if (buf[i] < ' ')
				ts->fwname[i] = 0;
			else
				ts->fwname[i] = buf[i];
		}
	}

	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: Enabling firmware class loader\n", __func__);

	retval = request_firmware_nowait(THIS_MODULE,
		FW_ACTION_NOHOTPLUG, (const char *)ts->fwname, ts->dev,
		GFP_KERNEL, ts->dev, cyttsp4_firmware_cont);
	if (retval) {
		pr_err("%s: Fail request firmware class file load\n",
			__func__);
		ts->waiting_for_fw = false;
		goto cyttsp4_ic_reflash_store_exit;
	} else {
		ts->waiting_for_fw = true;
		retval = size;
	}

cyttsp4_ic_reflash_store_exit:
	return retval;
}
static DEVICE_ATTR(ic_reflash, S_IRUSR | S_IWUSR,
	cyttsp4_ic_reflash_show, cyttsp4_ic_reflash_store);
#endif

static int _cyttsp4_calc_data_crc(struct cyttsp4 *ts, size_t ndata, u8 *pdata,
	u8 *crc_h, u8 *crc_l, const char *name)
{
	int retval = 0;
	u16 crc = 0x0000;
	u8 *buf = NULL;
	int i = 0;
	int j = 0;

	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

	*crc_h = 0;
	*crc_l = 0;

	buf = kzalloc(sizeof(uint8_t) * 126, GFP_KERNEL);
	if (buf == NULL) {
		pr_err("%s: Failed to allocate buf\n", __func__);
		retval = -ENOMEM;
		goto _cyttsp4_calc_data_crc_exit;
	}

	if (pdata == NULL) {
		pr_err("%s: bad data pointer\n", __func__);
		retval = -ENXIO;
		goto _cyttsp4_calc_data_crc_exit;
	}

	if (ndata > 122) {
		pr_err("%s: %s is too large n=%d size=%d\n",
			__func__, name, ndata, 126);
		retval = -EOVERFLOW;
		goto _cyttsp4_calc_data_crc_exit;
	}

	buf[0] = 0x00; /* num of config bytes + 4 high */
	buf[1] = 0x7E; /* num of config bytes + 4 low */
	buf[2] = 0x00; /* max block size w/o crc high */
	buf[3] = 0x7E; /* max block size w/o crc low */

	/* Copy platform data */
	memcpy(&(buf[4]), pdata, ndata);

	/* Calculate CRC */
	crc = 0xFFFF;
	for (i = 0; i < 126; i++) {
		crc ^= (buf[i] << 8);

		for (j = 8; j > 0; --j) {
			if (crc & 0x8000)
				crc = (crc << 1) ^ 0x1021;
			else
				crc = crc << 1;
		}
	}

	*crc_h = crc / 256;
	*crc_l = crc % 256;

	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: crc=%02X%02X\n", __func__, *crc_h, *crc_l);

_cyttsp4_calc_data_crc_exit:
	kfree(buf);
	return retval;
}

#ifdef CONFIG_TOUCHSCREEN_DEBUG
static int _cyttsp4_calc_ic_crc_tma400(struct cyttsp4 *ts,
	u8 *calc_ic_crc, size_t num_data)
{
	u16 crc = 0x0000;
	int retval = 0;
	size_t block_size;
	int block_id;
	int num_block;
	int i, j;
	u8 *pdata = NULL;

	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

	retval = _cyttsp4_get_block_size(ts, &block_size);
	if (retval < 0) {
		pr_err("%s: Fail get block size r=%d\n",
			__func__, retval);
		retval = -EIO;
		goto _cyttsp4_calc_ic_crc_tma400_exit;
	}

	pdata = kzalloc(block_size, GFP_KERNEL);
	if (pdata == NULL) {
		pr_err("%s: Fail allocate block buffer\n", __func__);
		retval = -ENOMEM;
		goto _cyttsp4_calc_ic_crc_tma400_exit;
	}

	if (ts->platform_data->sett
		[CY_IC_GRPNUM_TCH_PARM_VAL] == NULL) {
		pr_err("%s: missing param table\n", __func__);
		goto _cyttsp4_calc_ic_crc_tma400_exit;
	}
	if (ts->platform_data->sett
		[CY_IC_GRPNUM_TCH_PARM_VAL]->data == NULL) {
		pr_err("%s: missing param table data\n", __func__);
		goto _cyttsp4_calc_ic_crc_tma400_exit;
	}
	if (ts->platform_data->sett
		[CY_IC_GRPNUM_TCH_PARM_VAL]->size == 0) {
		pr_err("%s: empty param table\n", __func__);
		goto _cyttsp4_calc_ic_crc_tma400_exit;
	}
	num_block = ts->platform_data->sett
		[CY_IC_GRPNUM_TCH_PARM_VAL]->size / block_size;
	block_id = 0;
	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: Get CRC bytes for block=%d\n", __func__, block_id);
	retval = _cyttsp4_get_block_data(ts,
		block_id, block_size, pdata);
	if (retval < 0) {
		pr_err("%s: Fail get block=%d data r=%d\n",
			__func__, block_id, retval);
		retval = -EIO;
		goto _cyttsp4_calc_ic_crc_tma400_exit;
	}

	/* Calculate CRC */
	crc = 0xFFFF;
	for (i = 4; i < block_size; i++) {
		crc ^= ((u16)pdata[i] << 8);

		for (j = 8; j > 0; --j) {
			if (crc & 0x8000)
				crc = (crc << 1) ^ 0x1021;
			else
				crc = crc << 1;
		}
	}

	for (block_id = 1; block_id < num_block; block_id++) {
		cyttsp4_dbg(ts, CY_DBG_LVL_3,
			"%s: Get CRC bytes for block=%d num_block=%d\n",
			__func__, block_id, num_block);
		retval = _cyttsp4_get_block_data(ts,
			block_id, block_size, pdata);
		if (retval < 0) {
			pr_err("%s: Fail get block=%d data r=%d\n",
				__func__, block_id, retval);
			retval = -EIO;
			goto _cyttsp4_calc_ic_crc_tma400_exit;
		}
		for (i = 0; i < block_size; i++) {
			crc ^= ((u16)pdata[i] << 8);

			for (j = 8; j > 0; --j) {
				if (crc & 0x8000)
					crc = (crc << 1) ^ 0x1021;
				else
					crc = crc << 1;
			}
		}
	}

	calc_ic_crc[0] = crc % 256;	/* lsb first */
	calc_ic_crc[1] = crc / 256;
	calc_ic_crc[2] = ~calc_ic_crc[0];
	calc_ic_crc[3] = ~calc_ic_crc[1];

_cyttsp4_calc_ic_crc_tma400_exit:
	if (pdata != NULL)
		kfree(pdata);
	return retval;
}
#endif

static int _cyttsp4_calc_settings_crc(struct cyttsp4 *ts, u8 *crc_h, u8 *crc_l)
{
	int retval = 0;
	u16 crc = 0x0000;
	u8 *buf = NULL;
	int i = 0;
	int j = 0;
	u8 size = 0;

	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

	buf = kzalloc(sizeof(uint8_t) * 126, GFP_KERNEL);
	if (buf == NULL) {
		pr_err("%s: Failed to allocate buf\n", __func__);
		retval = -ENOMEM;
		goto _cyttsp4_calc_settings_crc_exit;
	}

	if (ts->platform_data->sett[CY_IC_GRPNUM_TCH_PARM_VAL] == NULL) {
		pr_err("%s: Missing Platform Touch Parameter"
			" values table\n",  __func__);
		retval = -ENXIO;
		goto _cyttsp4_calc_settings_crc_exit;
	}
	if ((ts->platform_data->sett
		[CY_IC_GRPNUM_TCH_PARM_VAL]->data == NULL) ||
		(ts->platform_data->sett
		[CY_IC_GRPNUM_TCH_PARM_VAL]->size == 0)) {
		pr_err("%s: Missing Platform Touch Parameter"
			" values table data\n", __func__);
		retval = -ENXIO;
		goto _cyttsp4_calc_settings_crc_exit;
	}

	size = ts->platform_data->sett[CY_IC_GRPNUM_TCH_PARM_VAL]->size;

	if (size > 122) {
		pr_err("%s: Platform data is too large\n", __func__);
		retval = -EOVERFLOW;
		goto _cyttsp4_calc_settings_crc_exit;
	}

	buf[0] = 0x00; /* num of config bytes + 4 high */
	buf[1] = 0x7E; /* num of config bytes + 4 low */
	buf[2] = 0x00; /* max block size w/o crc high */
	buf[3] = 0x7E; /* max block size w/o crc low */

	/* Copy platform data */
	memcpy(&(buf[4]),
		ts->platform_data->sett[CY_IC_GRPNUM_TCH_PARM_VAL]->data,
		size);

	/* Calculate CRC */
	crc = 0xFFFF;
	for (i = 0; i < 126; i++) {
		crc ^= (buf[i] << 8);

		for (j = 8; j > 0; --j) {
			if (crc & 0x8000)
				crc = (crc << 1) ^ 0x1021;
			else
				crc = crc << 1;
		}
	}

	*crc_h = crc / 256;
	*crc_l = crc % 256;

_cyttsp4_calc_settings_crc_exit:
	kfree(buf);
	return retval;
}

static int _cyttsp4_read_ic_crc_tma400(struct cyttsp4 *ts,
	u8 *read_ic_crc, size_t num_data)
{
	int retval = 0;
	int block_id;
	size_t block_size;
	u8 *pdata = NULL;

	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

	retval = _cyttsp4_get_block_size(ts, &block_size);
	if (retval < 0) {
		pr_err("%s: Fail get block size r=%d\n",
			__func__, retval);
		retval = -EIO;
		goto _cyttsp4_read_ic_crc_tma400_exit;
	}

	pdata = kzalloc(block_size, GFP_KERNEL);
	if (pdata == NULL) {
		pr_err("%s: Fail allocate block buffer\n", __func__);
		retval = -ENOMEM;
		goto _cyttsp4_read_ic_crc_tma400_exit;
	}

	block_id = 0;
	retval = _cyttsp4_get_block_data(ts,
		block_id, block_size, pdata);
	if (retval < 0) {
		pr_err("%s: Fail get block=%d data r=%d\n",
			__func__, block_id, retval);
		retval = -EIO;
		goto _cyttsp4_read_ic_crc_tma400_exit;
	}

	memcpy(read_ic_crc, pdata, num_data);

_cyttsp4_read_ic_crc_tma400_exit:
	if (pdata != NULL)
		kfree(pdata);
	return retval;
}

static int _cyttsp4_get_ic_crc(struct cyttsp4 *ts,
	u8 blkid, u8 *crc_h, u8 *crc_l)
{
	int retval = 0;
	u8 cmd_dat[CY_NUM_DAT + 1];	/* +1 for cmd byte */
	int tries;
	int tmp_state;

	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

	tmp_state = ts->power_state;
	ts->power_state = CY_CMD_STATE;
	cyttsp4_pr_state(ts);

	memset(cmd_dat, 0, sizeof(cmd_dat));
	/* pack cmd */
	cmd_dat[0] = CY_GET_CFG_BLK_CRC;
	/* pack blockid */
	cmd_dat[1] = blkid;

	/* send cmd */
	cyttsp4_dbg(ts, CY_DBG_LVL_2, "%s: Get CRC cmd=%02X\n",
		__func__, cmd_dat[0]);
	/* wait rdy */
	ts->cmd_rdy = false;
	tries = 0;

	retval = cyttsp4_write_block_data(ts, ts->si_ofs.cmd_ofs,
		sizeof(cmd_dat), cmd_dat,
		ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
	if (retval < 0) {
		pr_err("%s: write get crc err r=%d\n",
			__func__, retval);
		goto _cyttsp4_get_ic_crc_exit;
	}

	/* wait for cmd rdy interrupt */
	memset(cmd_dat, 0, sizeof(cmd_dat));
	while (tries++ < 10000) {
		if (mutex_is_locked(&ts->data_lock)) {
			mutex_unlock(&ts->data_lock);
			udelay(1000);
			mutex_lock(&ts->data_lock);
		} else {
			udelay(1000);
		}

		if (ts->cmd_rdy) {
			retval = cyttsp4_read_block_data(ts, ts->si_ofs.cmd_ofs,
				sizeof(cmd_dat), cmd_dat,
				ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
			if (retval < 0) {
				pr_err("%s: fail read cmd status"
					"r=%d\n", __func__, retval);
				goto _cyttsp4_get_ic_crc_exit;
			} else if (cmd_dat[0] & CY_CMD_RDY_BIT) {
				break;
			} else {
				/* not our interrupt */
				ts->cmd_rdy = false;
			}
		}
	}

	if (tries >= 10000) {
		pr_err("%s: command write timeout for crc request\n",
			__func__);
		retval = -ETIMEDOUT;
		goto _cyttsp4_get_ic_crc_exit;
	}

	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: get ic crc cmd[0..%d]="
		"%02X %02X %02X %02X %02X %02X %02X\n",
		__func__, sizeof(cmd_dat), cmd_dat[0],
		cmd_dat[1], cmd_dat[2], cmd_dat[3],
		cmd_dat[4], cmd_dat[5], cmd_dat[6]);

	/* Check CRC status and assign values */
	if (cmd_dat[1] != 0) {
		pr_err("%s: Get CRC command failed with code 0x%02X\n",
			__func__, cmd_dat[1]);
		retval = -EIO;
		goto _cyttsp4_get_ic_crc_exit;
	}

	*crc_h = cmd_dat[2];
	*crc_l = cmd_dat[3];

_cyttsp4_get_ic_crc_exit:
	ts->power_state = tmp_state;
	cyttsp4_pr_state(ts);
	return retval;
}

static int _cyttsp4_startup_tma400(struct cyttsp4 *ts)
{
	int i, j, k;
	int tries;
	int retval = 0;
	u8 read_ic_crc[4];
	u8 calc_ic_crc[4];
	u8 table_crc[4];
	u8 stat_regs[3];
#ifdef SH_TPSIF_COMMAND
	u32 version;
#endif	/* SH_TPSIF_COMMAND */
	bool put_all_params_done = true;
	/* 
	 * FIXME
	 *
	 * Default value of "put_all_params_done" is false.  Now
	 * changed to true in order to check firmware update function.
	 * 
	 * Don't forget to return to false in software for production.
	 *
	 */

	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

_cyttsp4_startup_tma400_restart:
	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: enter power_state=%d\n", __func__, ts->power_state);
	ts->power_state = CY_BL_STATE;
	ts->current_mode = CY_MODE_BOOTLOADER;
	cyttsp4_pr_state(ts);
	_cyttsp4_reset(ts);
	/* wait for first interrupt */
	INIT_COMPLETION(ts->bl_int_running);
	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: wait for first bootloader interrupt\n", __func__);
	if (mutex_is_locked(&ts->data_lock)) {
		mutex_unlock(&ts->data_lock);
		retval = wait_for_completion_interruptible_timeout(
			&ts->bl_int_running,
			msecs_to_jiffies(CY_DELAY_DFLT * CY_DELAY_MAX * 10));
		mutex_lock(&ts->data_lock);
	} else {
		retval = wait_for_completion_interruptible_timeout(
			&ts->bl_int_running,
			msecs_to_jiffies(CY_DELAY_DFLT * CY_DELAY_MAX * 10));
	}
	if (retval == 0) {
		pr_err("%s: timeout waiting for bootloader interrupt\n",
			__func__);
		retval = -ETIMEDOUT;
		goto _cyttsp4_startup_tma400_exit;
	}

	msleep(1000);

	/* wait for second interrupt */
	INIT_COMPLETION(ts->bl_int_running);
	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: wait for second bootloader interrupt\n", __func__);
	if (mutex_is_locked(&ts->data_lock)) {
		mutex_unlock(&ts->data_lock);
		retval = wait_for_completion_interruptible_timeout(
			&ts->bl_int_running,
			msecs_to_jiffies(CY_DELAY_DFLT * CY_DELAY_MAX * 10));
		mutex_lock(&ts->data_lock);
	} else {
		retval = wait_for_completion_interruptible_timeout(
			&ts->bl_int_running,
			msecs_to_jiffies(CY_DELAY_DFLT * CY_DELAY_MAX * 10));
	}
	if (retval == 0) {
		pr_err("%s: timeout waiting for second bootloader interrupt\n",
			__func__);
		retval = -ETIMEDOUT;
		goto _cyttsp4_startup_tma400_exit;
	}

	retval = _cyttsp4_ldr_exit(ts);
	if (retval < 0) {
		pr_err("%s: Fail exit bootloader r=%d\n",
			__func__, retval);
		ts->power_state = CY_IDLE_STATE;
		cyttsp4_pr_state(ts);
		goto _cyttsp4_startup_tma400_exit;
	}

	/* wait for app ready interrupt: switch to application */
	ts->power_state = CY_SYSINFO_STATE;
	ts->current_mode = CY_MODE_SYSINFO;
	INIT_COMPLETION(ts->si_int_running);
	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: wait for application ready interrupt\n", __func__);
	if (mutex_is_locked(&ts->data_lock)) {
		mutex_unlock(&ts->data_lock);
		retval = wait_for_completion_interruptible_timeout(
			&ts->si_int_running,
			msecs_to_jiffies(CY_DELAY_DFLT * CY_DELAY_MAX * 20));
		mutex_lock(&ts->data_lock);
	} else {
		retval = wait_for_completion_interruptible_timeout(
			&ts->si_int_running,
			msecs_to_jiffies(CY_DELAY_DFLT * CY_DELAY_MAX * 20));
	}
	if (retval == 0) {
		pr_err("%s: timeout waiting for application ready interrupt\n",
			__func__);
		retval = -ETIMEDOUT;
		goto _cyttsp4_startup_tma400_exit;
	}

	msleep(5000);	/* get past ratty int edges; remove when int fixed */

	/* Wait for IRQ to toggle high */
	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: wait for irq toggle high\n", __func__);
	retval = -ETIMEDOUT;
	for (i = 0, j = 0, k = 0; i < CY_DELAY_MAX * 10 * 5 * 10; i++, j++) {
		if (ts->platform_data->irq_stat() == CY_IRQ_DEASSERT) {
			retval = 0;
			break;
		}
		mdelay(CY_DELAY_DFLT);
		retval = 0;
		if (j >= 4 * CY_DELAY_MAX) {
			/* test to see if int line stuck; try to release */
			retval = cyttsp4_read_block_data(ts, CY_REG_BASE,
				sizeof(stat_regs), stat_regs,
				ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
			if (retval < 0) {
				cyttsp4_dbg(ts, CY_DBG_LVL_3,
					"%s: read attempt=%d fail\n",
					__func__, k);
				j = 0;
				k++;
			} else {
				cyttsp4_dbg(ts, CY_DBG_LVL_3,
					"%s: read attempt=%d pass\n",
					__func__, k);
				break;
			}
			retval = 0;
		}
	}
	if (retval < 0) {
		pr_err("%s: timeout waiting for irq to de-assert\n",
			__func__);
		/* try running anyway; continue */
	}

	tries = 0;
	while (tries++ < CY_NUM_RETRY) {
		cyttsp4_dbg(ts, CY_DBG_LVL_3,
			"%s: Read Sysinfo regs and get rev numbers try=%d\n",
			__func__, tries);
		retval = _cyttsp4_get_sysinfo_regs(ts);
		if (retval < 0) {
			pr_err("%s: Read Block fail -get sys regs (r=%d)\n",
				__func__, retval);
			msleep(1000);
		} else
			break;
	}

#ifdef SH_TPSIF_COMMAND
	if (_sh_tpsif_firmware_version(ts, &version) < 0) {
		pr_err("%s: Touch Panel Firmware Version = Unknown\n", __func__);
	} else {
		pr_info("%s: Touch Panel Firmware Version = %02X.%02X.%02X%02X\n",
			__func__, ((u8 *)&version)[3], ((u8 *)&version)[2],
			((u8 *)&version)[1], ((u8 *)&version)[0]);
		if (((u8 *)&version)[3] == 0x00 && ((u8 *)&version)[2] == 0x24) {
			pr_info("%s: Firmware for ES10.2\n", __func__);
			if (((u8 *)&version)[1] == 0x13 && ((u8 *)&version)[0] == 0xEC) {
				pr_info("%s: Inverted Y-axis\n", __func__);
				ts->reverse_y_axis = true;
			}
		} else {
			pr_info("%s: Firmware for ES100\n", __func__);
		}
	}
#endif	/* SH_TPSIF_COMMAND */

	retval = _cyttsp4_set_config_mode(ts);
	if (retval < 0) {
		pr_err("%s: Failed to switch to config mode\n",
			__func__);
		goto _cyttsp4_startup_tma400_bypass_crc_check;
	}
#ifdef CONFIG_TOUCHSCREEN_DEBUG
	if (ts->ic_grptest)
		goto _cyttsp4_startup_tma400_bypass_tbl_crc_chk;
#endif
	memset(read_ic_crc, 0, sizeof(read_ic_crc));
	memset(calc_ic_crc, 0, sizeof(calc_ic_crc));
	memset(table_crc, 0, sizeof(table_crc));
	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: Read IC CRC values\n", __func__);
	retval = _cyttsp4_read_ic_crc_tma400(ts,
		read_ic_crc, sizeof(read_ic_crc));
	if (retval < 0) {
		pr_err("%s: Fail read ic crc r=%d\n",
			__func__, retval);
	}

	_cyttsp4_pr_buf(ts, read_ic_crc, sizeof(read_ic_crc), "read_ic_crc");
	_cyttsp4_pr_buf(ts, calc_ic_crc, sizeof(calc_ic_crc), "calc_ic_crc");

	if (!put_all_params_done) {
		if (ts->platform_data->sett
			[CY_IC_GRPNUM_TCH_PARM_VAL]->data == NULL) {
			pr_err("%s: missing param table\n", __func__);
			goto _cyttsp4_startup_tma400_bypass_tbl_crc_chk;
		}
		memcpy(table_crc, ts->platform_data->sett
			[CY_IC_GRPNUM_TCH_PARM_VAL]->data, sizeof(table_crc));
		_cyttsp4_pr_buf(ts, table_crc, sizeof(calc_ic_crc),
			"table_crc");
		if ((read_ic_crc[0] != table_crc[0]) ||
			(read_ic_crc[1] != table_crc[1])) {
			retval = _cyttsp4_put_all_params_tma400(ts);
			if (retval < 0) {
				pr_err("%s: Fail put all params r=%d\n",
					__func__, retval);
				goto _cyttsp4_startup_tma400_bypass_tbl_crc_chk;
			}
			put_all_params_done = true;
			goto _cyttsp4_startup_tma400_restart;
		}
	}

_cyttsp4_startup_tma400_bypass_tbl_crc_chk:
	/* get handshake settings in config mode */
	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: enable handshake\n", __func__);
	retval = _cyttsp4_hndshk_enable(ts);
	if (retval < 0)
		pr_err("%s: fail enable handshake r=%d", __func__, retval);

_cyttsp4_startup_tma400_bypass_crc_check:
	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: enter operational mode\n", __func__);
	retval = _cyttsp4_set_operational_mode(ts);
	if (retval < 0) {
		pr_err("%s: Fail set operational mode (r=%d)\n",
			__func__, retval);
		ts->power_state = CY_IDLE_STATE;
		cyttsp4_pr_state(ts);
		goto _cyttsp4_startup_tma400_exit;
	}

	ts->power_state = CY_ACTIVE_STATE;
	cyttsp4_pr_state(ts);

_cyttsp4_startup_tma400_exit:
	return retval;
}

static int _cyttsp4_startup(struct cyttsp4 *ts)
{
	int retval = 0;
	int i = 0;
	u8 pdata_crc[2];
	u8 ic_crc[2];
	bool upgraded = false;
	bool mddata_updated = false;
	bool wrote_sysinfo_regs = false;
	bool wrote_settings = false;
	int tries = 0;

	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

	if (ts->flags & CY_FLAG_TMA400) {
		retval = _cyttsp4_startup_tma400(ts);
		if (retval < 0) {
			pr_err("%s: Fail startup TMA400 r=%d\n",
				__func__, retval);
		}
		goto _cyttsp4_startup_exit;
	}

_cyttsp4_startup_start:
	memset(pdata_crc, 0, sizeof(pdata_crc));
	memset(ic_crc, 0, sizeof(ic_crc));
	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: enter power_state=%d\n", __func__, ts->power_state);
	ts->power_state = CY_BL_STATE;
	cyttsp4_pr_state(ts);

	_cyttsp4_reset(ts);

	/* wait for interrupt to set ready completion */
	tries = 0;
_cyttsp4_startup_wait_bl_irq:
	INIT_COMPLETION(ts->bl_int_running);
	if (mutex_is_locked(&ts->data_lock)) {
		mutex_unlock(&ts->data_lock);
		retval = wait_for_completion_interruptible_timeout(
			&ts->bl_int_running,
			msecs_to_jiffies(CY_DELAY_DFLT * CY_DELAY_MAX * 10));
		mutex_lock(&ts->data_lock);
	} else {
		retval = wait_for_completion_interruptible_timeout(
			&ts->bl_int_running,
			msecs_to_jiffies(CY_DELAY_DFLT * CY_DELAY_MAX * 10));
	}
	if (retval == 0) {
		pr_err("%s: timeout waiting for bootloader interrupt\n",
			__func__);
		retval = -ETIMEDOUT;
		goto _cyttsp4_startup_exit;
	}

	retval = _cyttsp4_ldr_exit(ts);
	if (retval) {
		if (tries == 0) {
			tries++;
			goto _cyttsp4_startup_wait_bl_irq;
		}
		pr_err("%s: Cannot exit bl mode r=%d\n",
			__func__, retval);
		ts->power_state = CY_BL_STATE;
		cyttsp4_pr_state(ts);
		goto _cyttsp4_startup_exit;
	}

#ifdef CONFIG_TOUCHSCREEN_DEBUG
	if (retval) {
		cyttsp4_pr_status(ts, CY_DBG_LVL_3, retval);
		retval = 0;
	}
#endif

	/* wait for first interrupt: switch to application interrupt */
	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: wait for first interrupt\n", __func__);
	ts->power_state = CY_SYSINFO_STATE;
	ts->current_mode = CY_SYSINFO_MODE;
	cyttsp4_pr_state(ts);
	INIT_COMPLETION(ts->si_int_running);
	if (mutex_is_locked(&ts->data_lock)) {
		mutex_unlock(&ts->data_lock);
		retval = wait_for_completion_interruptible_timeout(
			&ts->si_int_running,
			msecs_to_jiffies(CY_DELAY_DFLT * CY_DELAY_MAX * 10));
		mutex_lock(&ts->data_lock);
	} else {
		retval = wait_for_completion_interruptible_timeout(
			&ts->si_int_running,
			msecs_to_jiffies(CY_DELAY_DFLT * CY_DELAY_MAX * 10));
	}
	if (retval == 0) {
		pr_err("%s: timeout waiting for application interrupt\n",
			__func__);
		retval = -ETIMEDOUT;
		goto _cyttsp4_startup_exit;
	}

	/*
	 * TODO: remove this wait for toggle high when
	 * startup from ES10 firmware is no longer required
	 */
	/* Wait for IRQ to toggle high */
	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: wait for irq toggle high\n", __func__);
	retval = -ETIMEDOUT;
	for (i = 0; i < CY_DELAY_MAX * 10 * 5; i++) {
		if (ts->platform_data->irq_stat() == CY_IRQ_DEASSERT) {
			retval = 0;
			break;
		}
		mdelay(CY_DELAY_DFLT);
	}
	if (retval < 0) {
		pr_err("%s: timeout waiting for irq to de-assert\n",
			__func__);
		goto _cyttsp4_startup_exit;
	}

	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: read sysinfo 1\n", __func__);
	memset(&ts->sysinfo_data, 0,
		sizeof(struct cyttsp4_sysinfo_data));
	retval = cyttsp4_read_block_data(ts, CY_REG_BASE,
		sizeof(struct cyttsp4_sysinfo_data),
		&ts->sysinfo_data,
		ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
	if (retval < 0) {
		pr_err("%s: Fail to switch from Bootloader "
			"to Application r=%d\n",
			__func__, retval);

		ts->power_state = CY_BL_STATE;
		cyttsp4_pr_state(ts);

		if (upgraded) {
			pr_err("%s: app failed to launch after"
				" platform firmware upgrade\n", __func__);
			retval = -EIO;
			goto _cyttsp4_startup_exit;
		}

		pr_info("%s: attempting to reflash IC...\n", __func__);
		if (ts->platform_data->fw->img == NULL ||
			ts->platform_data->fw->size == 0) {
			pr_err("%s: no platform firmware available"
				" for reflashing\n", __func__);
			retval = -ENODATA;
			ts->power_state = CY_INVALID_STATE;
			cyttsp4_pr_state(ts);
			goto _cyttsp4_startup_exit;
		}
		retval = _cyttsp4_load_app(ts,
			ts->platform_data->fw->img,
			ts->platform_data->fw->size);
		if (retval) {
			pr_err("%s: failed to reflash IC (r=%d)\n",
				__func__, retval);
			ts->power_state = CY_INVALID_STATE;
			cyttsp4_pr_state(ts);
			retval = -EIO;
			goto _cyttsp4_startup_exit;
		}
		pr_info("%s: resetting IC after reflashing\n", __func__);
		goto _cyttsp4_startup_start; /* Reset the part */
	}

	/*
	 * read system information registers
	 * get version numbers and fill sysinfo regs
	 */
	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: Read Sysinfo regs and get version numbers\n", __func__);
	retval = _cyttsp4_get_sysinfo_regs(ts);
	if (retval < 0) {
		pr_err("%s: Read Block fail -get sys regs (r=%d)\n",
			__func__, retval);
		ts->power_state = CY_IDLE_STATE;
		cyttsp4_pr_state(ts);
		goto _cyttsp4_startup_exit;
	}

#ifdef CONFIG_TOUCHSCREEN_DEBUG
	if (!ts->ic_grptest && !(ts->debug_upgrade)) {
		retval = _cyttsp4_boot_loader(ts, &upgraded);
		if (retval < 0) {
			pr_err("%s: fail boot loader r=%d)\n",
				__func__, retval);
			ts->power_state = CY_IDLE_STATE;
			cyttsp4_pr_state(ts);
			goto _cyttsp4_startup_exit;
		}
		if (upgraded)
			goto _cyttsp4_startup_start;
	}
#else
	retval = _cyttsp4_boot_loader(ts, &upgraded);
	if (retval < 0) {
		pr_err("%s: fail boot loader r=%d)\n",
			__func__, retval);
		ts->power_state = CY_IDLE_STATE;
		cyttsp4_pr_state(ts);
		goto _cyttsp4_startup_exit;
	}
	if (upgraded)
		goto _cyttsp4_startup_start;
#endif

	if (!wrote_sysinfo_regs) {
#ifdef CONFIG_TOUCHSCREEN_DEBUG
		if (ts->ic_grptest)
			goto _cyttsp4_startup_set_sysinfo_done;
#endif
		cyttsp4_dbg(ts, CY_DBG_LVL_3,
			"%s: Set Sysinfo regs\n", __func__);
		retval = _cyttsp4_set_sysinfo_mode(ts);
		if (retval < 0) {
			pr_err("%s: Set SysInfo Mode fail r=%d\n",
				__func__, retval);
			ts->power_state = CY_IDLE_STATE;
			cyttsp4_pr_state(ts);
			goto _cyttsp4_startup_exit;
		}
		retval = _cyttsp4_set_sysinfo_regs(ts, &mddata_updated);
		if (retval < 0) {
			pr_err("%s: Set SysInfo Regs fail r=%d\n",
				__func__, retval);
			ts->power_state = CY_IDLE_STATE;
			cyttsp4_pr_state(ts);
			goto _cyttsp4_startup_exit;
		} else
			wrote_sysinfo_regs = true;
	}

#ifdef CONFIG_TOUCHSCREEN_DEBUG
_cyttsp4_startup_set_sysinfo_done:
#endif
	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: enter operational mode\n", __func__);
	retval = _cyttsp4_set_operational_mode(ts);
	if (retval < 0) {
		ts->power_state = CY_IDLE_STATE;
		cyttsp4_pr_state(ts);
		pr_err("%s: Fail set operational mode (r=%d)\n",
			__func__, retval);
		goto _cyttsp4_startup_exit;
	} else {
#ifdef CONFIG_TOUCHSCREEN_DEBUG
		if (ts->ic_grptest)
			goto _cyttsp4_startup_settings_valid;
#endif
		/* Calculate settings CRC from platform settings */
		cyttsp4_dbg(ts, CY_DBG_LVL_3,
			"%s: Calculate settings CRC and get IC CRC\n",
			__func__);
		retval = _cyttsp4_calc_settings_crc(ts,
			&pdata_crc[0], &pdata_crc[1]);
		if (retval < 0) {
			pr_err("%s: Unable to calculate settings CRC\n",
				__func__);
			goto _cyttsp4_startup_exit;
		}

		/* Get settings CRC from touch IC */
		retval = _cyttsp4_get_ic_crc(ts, CY_TCH_PARM_BLKID,
			&ic_crc[0], &ic_crc[1]);
		if (retval < 0) {
			pr_err("%s: Unable to get settings CRC\n", __func__);
			goto _cyttsp4_startup_exit;
		}

		/* Compare CRC values */
		cyttsp4_dbg(ts, CY_DBG_LVL_3,
			"%s: PDATA CRC = 0x%02X%02X, IC CRC = 0x%02X%02X\n",
			__func__, pdata_crc[0], pdata_crc[1],
			ic_crc[0], ic_crc[1]);

		if ((pdata_crc[0] == ic_crc[0]) &&
			(pdata_crc[1] == ic_crc[1]))
			goto _cyttsp4_startup_settings_valid;

		/* Update settings */
		pr_info("%s: Updating IC settings...\n", __func__);

		if (wrote_settings) {
			pr_err("%s: Already updated IC settings\n",
				__func__);
			goto _cyttsp4_startup_settings_valid;
		}

		retval = _cyttsp4_set_op_params(ts, pdata_crc[0], pdata_crc[1]);
		if (retval < 0) {
			pr_err("%s: Set Operational Params fail r=%d\n",
				__func__, retval);
			goto _cyttsp4_startup_exit;
		}

		wrote_settings = true;
	}

_cyttsp4_startup_settings_valid:
	if (mddata_updated || wrote_settings) {
		pr_info("%s: Resetting IC after writing settings\n",
			__func__);
		mddata_updated = false;
		wrote_settings = false;
		goto _cyttsp4_startup_start; /* Reset the part */
	}
	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: enable handshake\n", __func__);
	retval = _cyttsp4_hndshk_enable(ts);
	if (retval < 0)
		pr_err("%s: fail enable handshake r=%d", __func__, retval);

	ts->power_state = CY_ACTIVE_STATE;
	cyttsp4_pr_state(ts);

	if (ts->was_suspended) {
		ts->was_suspended = false;
		retval = _cyttsp4_resume_sleep(ts);
		if (retval < 0) {
			pr_err("%s: fail resume sleep r=%d\n",
				__func__, retval);
		}
	}

_cyttsp4_startup_exit:
	return retval;
}

static irqreturn_t cyttsp4_irq(int irq, void *handle)
{
	struct cyttsp4 *ts = handle;
	int retval;

	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: GOT IRQ ps=%d\n", __func__, ts->power_state);
	mutex_lock(&ts->data_lock);

	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: DO IRQ ps=%d\n", __func__, ts->power_state);
	switch (ts->power_state) {
	case CY_BL_STATE:
		complete(&ts->bl_int_running);
		break;
	case CY_SYSINFO_STATE:
		complete(&ts->si_int_running);
		break;
	case CY_LDR_STATE:
		ts->bl_ready_flag = true;
		break;
	case CY_CMD_STATE:
		ts->cmd_rdy = true;
		break;
	case CY_SLEEP_STATE:
		pr_err("%s: IRQ in sleep state\n", __func__);
		/* Service the interrupt as if active (if possible) */
		ts->power_state = CY_ACTIVE_STATE;
		cyttsp4_pr_state(ts);
		retval = _cyttsp4_xy_worker(ts);
		if (retval == IRQ_HANDLED) {
			ts->was_suspended = true;
			break;
		} else if (retval < 0) {
			pr_err("%s: Sleep XY Worker fail r=%d\n",
				__func__, retval);
			ts->was_suspended = true;
			_cyttsp4_startup(ts);
		}
		/* Put the part back to sleep */
		retval = _cyttsp4_resume_sleep(ts);
		if (retval < 0) {
			pr_err("%s: fail resume sleep r=%d\n",
				__func__, retval);
		}
		break;
	case CY_IDLE_STATE:
		break;
	case CY_ACTIVE_STATE:
		/* process the touches */
		retval = _cyttsp4_xy_worker(ts);
		if (retval < 0) {
			pr_err("%s: XY Worker fail r=%d\n",
				__func__, retval);
			queue_work(ts->cyttsp4_wq, &ts->cyttsp4_startup_work);
		}
		break;
	default:
		break;
	}

	mutex_unlock(&ts->data_lock);
	return IRQ_HANDLED;
}

static void cyttsp4_ldr_init(struct cyttsp4 *ts)
{

	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

#ifdef CONFIG_TOUCHSCREEN_DEBUG
	if (device_create_file(ts->dev, &dev_attr_drv_debug))
		pr_err("%s: Error, could not create drv_debug\n", __func__);

	if (device_create_file(ts->dev, &dev_attr_drv_flags))
		pr_err("%s: Error, could not create drv_flags\n", __func__);

	if (device_create_file(ts->dev, &dev_attr_drv_irq))
		pr_err("%s: Error, could not create drv_irq\n", __func__);
#endif

	if (device_create_file(ts->dev, &dev_attr_drv_stat))
		pr_err("%s: Error, could not create drv_stat\n", __func__);

	if (device_create_file(ts->dev, &dev_attr_drv_ver))
		pr_err("%s: Error, could not create drv_ver\n", __func__);

#ifdef CONFIG_TOUCHSCREEN_DEBUG
	if (device_create_file(ts->dev, &dev_attr_hw_irqstat))
		pr_err("%s: Error, could not create hw_irqstat\n", __func__);

	if (device_create_file(ts->dev, &dev_attr_hw_reset))
		pr_err("%s: Error, could not create hw_reset\n", __func__);

	if (device_create_file(ts->dev, &dev_attr_hw_recov))
		pr_err("%s: Error, could not create hw_recov\n", __func__);

	if (device_create_file(ts->dev, &dev_attr_hw_power))
		pr_err("%s: Error, could not create hw_power\n", __func__);

	if (device_create_file(ts->dev, &dev_attr_ic_grpnum))
		pr_err("%s: Error, could not create ic_grpnum\n", __func__);

	if (device_create_file(ts->dev, &dev_attr_ic_grpoffset))
		pr_err("%s: Error, could not create ic_grpoffset\n", __func__);

	if (device_create_file(ts->dev, &dev_attr_ic_grpdata))
		pr_err("%s: Error, could not create ic_grpdata\n", __func__);

	if (device_create_file(ts->dev, &dev_attr_ic_reflash))
		pr_err("%s: Error, could not create ic_reflash\n", __func__);
#endif

	if (device_create_file(ts->dev, &dev_attr_ic_ver))
		pr_err("%s: Cannot create ic_ver\n", __func__);

	return;
}

static void cyttsp4_ldr_free(struct cyttsp4 *ts)
{
	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

	device_remove_file(ts->dev, &dev_attr_drv_ver);
	device_remove_file(ts->dev, &dev_attr_drv_stat);
	device_remove_file(ts->dev, &dev_attr_ic_ver);
#ifdef CONFIG_TOUCHSCREEN_DEBUG
	device_remove_file(ts->dev, &dev_attr_ic_grpnum);
	device_remove_file(ts->dev, &dev_attr_ic_grpoffset);
	device_remove_file(ts->dev, &dev_attr_ic_grpdata);
	device_remove_file(ts->dev, &dev_attr_hw_irqstat);
	device_remove_file(ts->dev, &dev_attr_drv_irq);
	device_remove_file(ts->dev, &dev_attr_drv_debug);
	device_remove_file(ts->dev, &dev_attr_ic_reflash);
	device_remove_file(ts->dev, &dev_attr_drv_flags);
	device_remove_file(ts->dev, &dev_attr_hw_reset);
	device_remove_file(ts->dev, &dev_attr_hw_recov);
	device_remove_file(ts->dev, &dev_attr_hw_power);
#endif
}

static int cyttsp4_power_on(struct cyttsp4 *ts)
{
	int retval = 0;

	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

	mutex_lock(&ts->data_lock);
	retval = _cyttsp4_startup(ts);
	mutex_unlock(&ts->data_lock);
	if (retval < 0) {
		pr_err("%s: startup fail at power on r=%d\n", __func__, retval);
		ts->power_state = CY_IDLE_STATE;
		cyttsp4_pr_state(ts);
	}

	return retval;
}

static int cyttsp4_open(struct input_dev *dev)
{
	int retval = 0;

	struct cyttsp4 *ts = input_get_drvdata(dev);

	pr_info("%s: start\n", __func__); //DEBUG
	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

	mutex_lock(&ts->startup_mutex);
	if (!ts->powered) {

#if 1
		pr_info("%s: power on start\n", __func__); //DEBUG
		/* Power on */
		if (ts->platform_data->hw_power == NULL) {
			pr_err("%s: no hw_power function\n", __func__);
		} else {
			retval = ts->platform_data->hw_power(1); // power on
			if(retval < 0) {
				pr_err("%s: failed power on\n", __func__);
			}
		}
		pr_info("%s: power on end\n", __func__); //DEBUG
#endif

		retval = cyttsp4_power_on(ts);

		/* powered if no hard failure */
		if (retval < 0)
			ts->powered = false;
		else
			ts->powered = true;
		pr_info("%s: Powered ON(%d) r=%d\n",
			__func__, (int)ts->powered, retval);
#ifdef SH_TPSIF_COMMAND
#ifdef STARTUP_CALIBRATION
#if 0
		if (ts->powered) {
			if (_sh_tpsif_get_bootmode() == SH_BOOT_MODE_HW_CHK)
				sh_tpsif_calibration_idac(ts);
		}
#else
		sh_tpsif_calibration_idac(ts);
#endif	/* 0 */
#endif	/* STARTUP_CALIBRATION */
#endif	/* SH_TPSIF_COMMAND */
	}
	mutex_unlock(&ts->startup_mutex);
	pr_info("%s: end\n", __func__); //DEBUG
	return 0;
}

static void cyttsp4_close(struct input_dev *dev)
{
	/*
	 * close() normally powers down the device
	 * this call simply returns unless power
	 * to the device can be controlled by the driver
	 */
	return;
}

void cyttsp4_core_release(void *handle)
{
	struct cyttsp4 *ts = handle;

	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

	if (ts) {
#ifdef CONFIG_HAS_EARLYSUSPEND
		unregister_early_suspend(&ts->early_suspend);
#endif
		cyttsp4_ldr_free(ts);
		mutex_destroy(&ts->data_lock);
		mutex_destroy(&ts->startup_mutex);
		free_irq(ts->irq, ts);
#ifdef MULTI_TOUCH_PROTOCOL_B
		if (ts->input->mt)
			input_mt_destroy_slots(ts->input);
#endif	/* MULTI_TOUCH_PROTOCOL_B */
		input_unregister_device(ts->input);
		if (ts->cyttsp4_wq) {
			destroy_workqueue(ts->cyttsp4_wq);
			ts->cyttsp4_wq = NULL;
		}

		if (ts->sysinfo_ptr.cydata != NULL)
			kfree(ts->sysinfo_ptr.cydata);
		if (ts->sysinfo_ptr.test != NULL)
			kfree(ts->sysinfo_ptr.test);
		if (ts->sysinfo_ptr.pcfg != NULL)
			kfree(ts->sysinfo_ptr.pcfg);
		if (ts->sysinfo_ptr.opcfg != NULL)
			kfree(ts->sysinfo_ptr.opcfg);
		if (ts->sysinfo_ptr.ddata != NULL)
			kfree(ts->sysinfo_ptr.ddata);
		if (ts->sysinfo_ptr.mdata != NULL)
			kfree(ts->sysinfo_ptr.mdata);

		/* Power off */
		if (ts->platform_data->hw_power == NULL) {
			pr_err("%s: no hw_power function\n", __func__);
		} else {
			if (ts->platform_data->hw_power(0) < 0) // power off
				pr_err("%s: failed power off\n", __func__);
		}

		kfree(ts);
	}
}
EXPORT_SYMBOL_GPL(cyttsp4_core_release);

void *cyttsp4_core_init(struct cyttsp4_bus_ops *bus_ops,
	struct device *dev, int irq, char *name)
{
	int i;
	u16 signal;
	int retval = 0;
	struct input_dev *input_device;
	struct cyttsp4 *ts = kzalloc(sizeof(*ts), GFP_KERNEL);

	pr_info("%s: start\n", __func__); //DEBUG

	if (!ts) {
		pr_err("%s: Error, kzalloc\n", __func__);
		goto error_alloc_data;
	}

#ifdef SH_TPSIF_COMMAND
	g_tpsif_ts = ts;
	init_waitqueue_head(&sh_tpsif_wq);
	sh_tpsif_event = 0;
	hrtimer_init(&sh_tpsif_polling_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	sh_tpsif_polling_timer.function = _sh_tpsif_poll_timer_handler;
	INIT_WORK(&sh_tpsif_polling_work, _sh_tpsif_poll_scan);
#endif	/* SH_TPSIF_COMMAND */

	ts->cyttsp4_wq =
		create_singlethread_workqueue("cyttsp4_resume_startup_wq");
	if (ts->cyttsp4_wq == NULL) {
		pr_err("%s: No memory for cyttsp4_resume_startup_wq\n",
			__func__);
		retval = -ENOMEM;
		goto err_alloc_wq_failed;
	}

#ifdef CONFIG_TOUCHSCREEN_DEBUG
	ts->fwname = kzalloc(CY_BL_FW_NAME_SIZE, GFP_KERNEL);
	if ((ts->fwname == NULL) || (dev == NULL) || (bus_ops == NULL)) {
		pr_err("%s: Error, dev, bus_ops, or fwname null\n",
			__func__);
		kfree(ts);
		ts = NULL;
		goto error_alloc_data;
	}
	ts->waiting_for_fw = false;
	ts->debug_upgrade = false;

#endif
	ts->powered = false;
	ts->cmd_rdy = false;
	ts->hndshk_enabled = false;
	ts->was_suspended = false;

	ts->xy_data = NULL;
	ts->xy_mode = NULL;

#ifdef CONFIG_TOUCHSCREEN_DEBUG
	ts->ic_grpnum = CY_IC_GRPNUM_RESERVED;
	ts->ic_grpoffset = 0;
	ts->ic_grptest = false;
#endif

#ifdef SH_TPSIF_COMMAND
	ts->adjust_enabled = false;
	ts->reverse_y_axis = false;
#endif	/* SH_TPSIF_COMMAND */

	mutex_init(&ts->data_lock);
	mutex_init(&ts->startup_mutex);
	ts->power_state = CY_INVALID_STATE;
	ts->current_mode = CY_MODE_BOOTLOADER;
	ts->dev = dev;
	ts->platform_data = dev->platform_data;
	ts->bus_ops = bus_ops;
#ifdef CONFIG_TOUCHSCREEN_DEBUG
	ts->bus_ops->tsdebug = CY_DBG_LVL_0;
#endif

	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: Initialize irq flags\n", __func__);
	init_completion(&ts->bl_int_running);
	init_completion(&ts->si_int_running);
	ts->flags = ts->platform_data->flags;
	if (ts->flags & CY_FLAG_TMA400)
		ts->max_config_bytes = CY_TMA400_MAX_BYTES;
	else
		ts->max_config_bytes = CY_TMA884_MAX_BYTES;

	ts->irq = irq;
	if (ts->irq <= 0) {
		cyttsp4_dbg(ts, CY_DBG_LVL_3,
			"%s: Error, failed to allocate irq\n", __func__);
			goto error_init;
	}

	ts->bl_ready_flag = false;

	/* Create the input device and register it. */
	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: Create the input device and register it\n", __func__);
	input_device = input_allocate_device();
	if (!input_device) {
		pr_err("%s: Error, failed to allocate input device\n",
			__func__);
		goto error_input_allocate_device;
	}
	ts->input = input_device;
	input_device->name = name;
	snprintf(ts->phys, sizeof(ts->phys), "%s", dev_name(dev));
	input_device->phys = ts->phys;
	input_device->dev.parent = ts->dev;
	ts->bus_type = bus_ops->dev->bus;

	input_device->open = cyttsp4_open;
	input_device->close = cyttsp4_close;
	input_set_drvdata(input_device, ts);
	dev_set_drvdata(dev, ts);

	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: Initialize event signals\n", __func__);
	__set_bit(EV_ABS, input_device->evbit);

#ifdef MULTI_TOUCH_PROTOCOL_B
	input_mt_init_slots(input_device, MAX_TOUCH_NUM);
	if (input_device->mt == NULL) {
		pr_err("%s: Error, failed to init input slots\n",
			__func__);
		goto error_input_register_device;
	}
#endif	/* MULTI_TOUCH_PROTOCOL_B */

	for (i = 0; i < (ts->platform_data->frmwrk->size/CY_NUM_ABS_SET); i++) {
		signal = ts->platform_data->frmwrk->abs[
			(i*CY_NUM_ABS_SET)+CY_SIGNAL_OST];
		if (signal != CY_IGNORE_VALUE
#ifdef MULTI_TOUCH_PROTOCOL_B
			&& signal != ABS_MT_TRACKING_ID
#endif	/* MULTI_TOUCH_PROTOCOL_B */
			) {
			input_set_abs_params(input_device,
				signal,
				ts->platform_data->frmwrk->abs[
					(i*CY_NUM_ABS_SET)+CY_MIN_OST],
				ts->platform_data->frmwrk->abs[
					(i*CY_NUM_ABS_SET)+CY_MAX_OST],
				ts->platform_data->frmwrk->abs[
					(i*CY_NUM_ABS_SET)+CY_FUZZ_OST],
				ts->platform_data->frmwrk->abs[
					(i*CY_NUM_ABS_SET)+CY_FLAT_OST]);
		}
	}

	input_set_events_per_packet(input_device, 6 * CY_NUM_TCH_ID);
	if (ts->platform_data->frmwrk->enable_vkeys)
		input_set_capability(input_device, EV_KEY, KEY_PROG1);
	pr_info("%s: input_register_device\n", __func__); //DEBUG
	if (input_register_device(input_device)) {
		pr_err("%s: Error, failed to register input device\n",
			__func__);
		goto error_input_register_device;
	}
	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: Initialize irq\n", __func__);
	retval = request_threaded_irq(ts->irq, NULL, cyttsp4_irq,
		IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
		ts->input->name, ts);
	if (retval < 0) {
#ifdef CONFIG_TOUCHSCREEN_DEBUG
		ts->irq_enabled = false;
#endif
		pr_err("%s: failed to init irq r=%d name=%s\n",
			__func__, retval, ts->input->name);
		goto error_input_register_device;
	} else {
#ifdef CONFIG_TOUCHSCREEN_DEBUG
		ts->irq_enabled = true;
#endif
	}

	/* add /sys files */
	cyttsp4_ldr_init(ts);

#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = cyttsp4_early_suspend;
	ts->early_suspend.resume = cyttsp4_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif

	INIT_WORK(&ts->cyttsp4_resume_startup_work, cyttsp4_ts_work_func);
	INIT_WORK(&ts->cyttsp4_startup_work, cyttsp4_ts_startup_work_func);

#if 0
	pr_info("%s: power on start\n", __func__); //DEBUG
	/* Power on */
	if (ts->platform_data->hw_power == NULL) {
		pr_err("%s: no hw_power function\n", __func__);
		goto error_power_on;
	} else {
		retval = ts->platform_data->hw_power(1); // power on
		if(retval < 0) {
			pr_err("%s: failed power on\n", __func__);
			goto error_power_on;
		}
	}
	pr_info("%s: power on end\n", __func__); //DEBUG
#endif

	goto no_error;

#if 0
error_power_on:
#endif

error_input_register_device:
#ifdef MULTI_TOUCH_PROTOCOL_B
	if (input_device->mt)
		input_mt_destroy_slots(input_device);
#endif	/* MULTI_TOUCH_PROTOCOL_B */
	input_free_device(input_device);

error_input_allocate_device:

error_init:
	mutex_destroy(&ts->data_lock);
	mutex_destroy(&ts->startup_mutex);
	if (ts->cyttsp4_wq) {
		destroy_workqueue(ts->cyttsp4_wq);
		ts->cyttsp4_wq = NULL;
	}
#ifdef CONFIG_TOUCHSCREEN_DEBUG
	kfree(ts->fwname);
#endif
err_alloc_wq_failed:
	kfree(ts);
	ts = NULL;
error_alloc_data:
	pr_err("%s: Failed Initialization\n", __func__);
no_error:
	pr_info("%s: end\n", __func__); //DEBUG
	return ts;
}
EXPORT_SYMBOL_GPL(cyttsp4_core_init);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Cypress TrueTouch(R) Standard touchscreen driver core");
MODULE_AUTHOR("Cypress");


#ifdef SH_TPSIF_COMMAND
#include <linux/cdev.h>		/* cdev */
#include <asm/uaccess.h>	/* copy_from_user, copy_to_user */
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/wait.h>
#if 0
#include <sharp/sh_smem.h>	/* sh_smem_get_common_address */
#endif

/* power */
#define SH_TPSIF_HW_POWER_ON	1
#define SH_TPSIF_HW_POWER_OFF	0

#define SH_TPSIF_POLL_INTERVAL 100 /* [ms] */

static dev_t sh_tpsif_dev;
static struct cdev sh_tpsif_cdev;
static struct class *sh_tpsif_class;

static struct cyttsp4_scan_data sh_tpsif_scan_data[2];
static int sh_tpsif_scan_type = TPSIF_SCAN_TYPE_MUTUAL_RAW;
static int sh_tpsif_polling_loop = 0;

#if 0
static unsigned short _sh_tpsif_get_bootmode(void)
{
	sharp_smem_common_type *p_sharp_smem_common_type;

	p_sharp_smem_common_type = sh_smem_get_common_address();
	if (p_sharp_smem_common_type != NULL) {
		pr_info("%s: boot mode = %d\n", __func__, (int) p_sharp_smem_common_type->sh_boot_mode); //DEBUG
		return p_sharp_smem_common_type->sh_boot_mode;
	}
	else {
		pr_info("%s: p_sharp_smem_common_type = NULL\n", __func__); //DEBUG
		return 0;
	}
}
#endif	/* 0 */

static void _sh_tpsif_adjust_point(struct cyttsp4 *ts, int *x, int *y)
{
	int i;
	long l_xpq;
	long l_xrs;
	long l_x;
	long l_ypr;
	long l_yqs;
	long l_y;

	pr_info("%s: before (%d, %d)\n", __func__, *x, *y); //DEBUG

	/* divide the area */
	for (i = 0; i < AREA_COUNT; i++) {
		if (sh_tpsif_area_rect[i].p.x <= *x && sh_tpsif_area_rect[i].s.x > *x &&
			sh_tpsif_area_rect[i].p.y <= *y && sh_tpsif_area_rect[i].s.y > *y)
			break;
	}
	/* if not belong to any area, do not adjust */
	if (i != AREA_COUNT) {
		/* do an adjustment of coordinate */

		l_xpq = (((sh_tpsif_area_diff[i].q.x * DOUBLE_ACCURACY) -
				(sh_tpsif_area_diff[i].p.x * DOUBLE_ACCURACY)) /
			(sh_tpsif_area_rect[i].q.x - sh_tpsif_area_rect[i].p.x)) *
			(*x - sh_tpsif_area_rect[i].p.x) +
			(sh_tpsif_area_diff[i].p.x * DOUBLE_ACCURACY);
		l_xrs = (((sh_tpsif_area_diff[i].s.x * DOUBLE_ACCURACY) -
				(sh_tpsif_area_diff[i].r.x * DOUBLE_ACCURACY)) /
			(sh_tpsif_area_rect[i].s.x - sh_tpsif_area_rect[i].r.x)) *
			(*x - sh_tpsif_area_rect[i].r.x) +
			(sh_tpsif_area_diff[i].r.x * DOUBLE_ACCURACY);
		l_x   = ((l_xrs - l_xpq) / (sh_tpsif_area_rect[i].r.y - sh_tpsif_area_rect[i].p.y)) *
			(*y - sh_tpsif_area_rect[i].p.y) + l_xpq;
		l_ypr = (((sh_tpsif_area_diff[i].r.y * DOUBLE_ACCURACY) -
				(sh_tpsif_area_diff[i].p.y * DOUBLE_ACCURACY)) /
			(sh_tpsif_area_rect[i].r.y - sh_tpsif_area_rect[i].p.y)) *
			(*y - sh_tpsif_area_rect[i].p.y) +
			(sh_tpsif_area_diff[i].p.y * DOUBLE_ACCURACY);
		l_yqs = (((sh_tpsif_area_diff[i].s.y * DOUBLE_ACCURACY) -
				(sh_tpsif_area_diff[i].q.y * DOUBLE_ACCURACY)) /
			(sh_tpsif_area_rect[i].s.y - sh_tpsif_area_rect[i].q.y)) *
			(*y - sh_tpsif_area_rect[i].q.y) +
			(sh_tpsif_area_diff[i].q.y * DOUBLE_ACCURACY);
		l_y   = ((l_yqs - l_ypr) / (sh_tpsif_area_rect[i].q.x - sh_tpsif_area_rect[i].p.x)) *
			(*x - sh_tpsif_area_rect[i].p.x) + l_ypr;
		*x = *x - (int)(l_x / DOUBLE_ACCURACY);
		*y = *y - (int)(l_y / DOUBLE_ACCURACY);
	}

	/* to be adjusted inside the range */
	*x = MINMAX(0, ts->platform_data->frmwrk->abs[(CY_ABS_X_OST*CY_NUM_ABS_SET)+CY_MAX_OST], *x);
	*y = MINMAX(0, ts->platform_data->frmwrk->abs[(CY_ABS_Y_OST*CY_NUM_ABS_SET)+CY_MAX_OST], *y);

	pr_info("%s: after (%d, %d)\n", __func__, *x, *y); //DEBUG
}

static void _sh_tpsif_qsort(sh_tpsif_qsort_t *table, int top, int end)
{
	int i, j;
	int center;
	sh_tpsif_qsort_t swap;

	i = top;
	j = end;

	center = table[(top + end) / 2].value;

	while (1) {
		while (table[i].value < center)
			i++;
		while (center < table[j].value)
			j--;
		if (i >= j)
			break;
		memcpy(&swap, &table[i], sizeof(sh_tpsif_qsort_t));
		memcpy(&table[i], &table[j], sizeof(sh_tpsif_qsort_t));
		memcpy(&table[j], &swap, sizeof(sh_tpsif_qsort_t));
		i++;
		j--;
	}
	if (top < i - 1)
		_sh_tpsif_qsort(table, top, i - 1);
	if (j + 1 < end)
		_sh_tpsif_qsort(table, j + 1, end);
}

static void _sh_tpsif_round_value(int *value)
{
	sh_tpsif_qsort_t table[6];
	int i;

	for (i = 0; i < 6; i++) {
		table[i].num = i;
		table[i].value = value[i];
	}
	_sh_tpsif_qsort(table, 0, 5);
	value[table[0].num] = value[table[1].num];
	value[table[5].num] = value[table[4].num];
}

static int sh_tpsif_set_adjust_param(struct cyttsp4 *ts, u16 *param)
{
	int i;
	sh_tpsif_point_t sd[ADJUST_POINT];
	int diff[2][6];
	int retval = 0;

	/* adjustment of coordinate is invalid */
	if (param == NULL) {
		ts->adjust_enabled = false;

		pr_info("%s: param is NULL\n", __func__);

		retval = sh_tpsif_calibration_idac(ts);
		return retval;
	}

	pr_info("%s: (%4d, %4d) (%4d, %4d)\n", __func__, param[0], param[1], param[2], param[3]);
	pr_info("%s: (%4d, %4d) (%4d, %4d)\n", __func__, param[4], param[5], param[6], param[7]);
	pr_info("%s: (%4d, %4d) (%4d, %4d)\n", __func__, param[8], param[9], param[10], param[11]);

	/* parameter check */
	for (i = 0; i < ADJUST_POINT; i++) {
		if (param[i * 2 + 0] > sh_tpsif_base_point[i].x + POS_LIMIT ||
			param[i * 2 + 0] < sh_tpsif_base_point[i].x - POS_LIMIT)
			return -EINVAL;
		if (param[i * 2 + 1] > sh_tpsif_base_point[i].y + POS_LIMIT ||
			param[i * 2 + 1] < sh_tpsif_base_point[i].y - POS_LIMIT)
			return -EINVAL;
	}

	/* save parameters */
	SET_POINT(sh_tpsif_adjust_param[0], param[ 0], param[ 1]);
	SET_POINT(sh_tpsif_adjust_param[1], param[ 2], param[ 3]);
	SET_POINT(sh_tpsif_adjust_param[2], param[ 4], param[ 5]);
	SET_POINT(sh_tpsif_adjust_param[3], param[ 6], param[ 7]);
	SET_POINT(sh_tpsif_adjust_param[4], param[ 8], param[ 9]);
	SET_POINT(sh_tpsif_adjust_param[5], param[10], param[11]);

#if 0	/* changing the calculation method of diff */
	/* calculate diff value */
	for (i = 0; i < ADJUST_POINT; i++) {
		sd[i].x = (sh_tpsif_adjust_param[i].x - sh_tpsif_base_point[i].x) * 3 / 4;
		sd[i].y = (sh_tpsif_adjust_param[i].y - sh_tpsif_base_point[i].y) * 3 / 4;
	}
#else
	for (i = 0; i < ADJUST_POINT; i++) {
		diff[0][i] = (sh_tpsif_adjust_param[i].x - sh_tpsif_base_point[i].x);
		diff[1][i] = (sh_tpsif_adjust_param[i].y - sh_tpsif_base_point[i].y);
	}

	/* truncate the maximum and minimum */
	_sh_tpsif_round_value(diff[0]); /* X */
	_sh_tpsif_round_value(diff[1]); /* Y */

	for (i = 0; i < ADJUST_POINT; i++) {
		sd[i].x = diff[0][i] * 75 / 100;
		sd[i].y = diff[1][i] * 75 / 100;
	}
#endif	/* changing the calculation method of diff */

	/* store the blurring value of each four area corners */
	/*                     |-------p-------| |-------q-------| |-------r-------| |-------s-------|*/
	SET_AREA(sh_tpsif_area_diff[ 0], 0      , 0      , sd[0].x, 0      , 0      , sd[0].y, sd[0].x, sd[0].y);
	SET_AREA(sh_tpsif_area_diff[ 1], sd[0].x, 0      , sd[1].x, 0      , sd[0].x, sd[0].y, sd[1].x, sd[1].y);
	SET_AREA(sh_tpsif_area_diff[ 2], sd[1].x, 0      , 0      , 0      , sd[1].x, sd[1].y, 0      , sd[1].y);
	SET_AREA(sh_tpsif_area_diff[ 3], 0      , sd[0].y, sd[0].x, sd[0].y, 0      , sd[2].y, sd[2].x, sd[2].y);
	SET_AREA(sh_tpsif_area_diff[ 4], sd[0].x, sd[0].y, sd[1].x, sd[1].y, sd[2].x, sd[2].y, sd[3].x, sd[3].y);
	SET_AREA(sh_tpsif_area_diff[ 5], sd[1].x, sd[1].y, 0      , sd[1].y, sd[3].x, sd[3].y, 0      , sd[3].y);
	SET_AREA(sh_tpsif_area_diff[ 6], 0      , sd[2].y, sd[2].x, sd[2].y, 0      , sd[4].y, sd[4].x, sd[4].y);
	SET_AREA(sh_tpsif_area_diff[ 7], sd[2].x, sd[2].y, sd[3].x, sd[3].y, sd[4].x, sd[4].y, sd[5].x, sd[5].y);
	SET_AREA(sh_tpsif_area_diff[ 8], sd[3].x, sd[3].y, 0      , sd[3].y, sd[5].x, sd[5].y, 0      , sd[5].y);
	SET_AREA(sh_tpsif_area_diff[ 9], 0      , sd[4].y, sd[4].x, sd[4].y, 0      , 0      , sd[4].x, 0      );
	SET_AREA(sh_tpsif_area_diff[10], sd[4].x, sd[4].y, sd[5].x, sd[5].y, sd[4].x, 0      , sd[5].x, 0      );
	SET_AREA(sh_tpsif_area_diff[11], sd[5].x, sd[5].y, 0      , sd[5].y, sd[5].x, 0      , 0      , 0      );

	/* to valid an adjustment of coordinate */
	ts->adjust_enabled = true;

	return retval;
}

static int sh_tpsif_hw_power(struct cyttsp4 *ts, int on)
{
	int retval = 0;

	mutex_lock(&ts->data_lock);

	if (ts->platform_data->hw_power)
		retval = ts->platform_data->hw_power(on);
	else
		retval = -ENOSYS;

	mutex_unlock(&ts->data_lock);

	return retval;
}

static int sh_tpsif_hw_reset(struct cyttsp4 *ts)
{
	int retval = 0;

	mutex_lock(&ts->data_lock);

	if (ts->platform_data->hw_reset)
		retval = ts->platform_data->hw_reset();
	else
		retval = -ENOSYS;

	if (retval == 0) {
		/* ts->power_state = CY_INVALID_STATE; */
		ts->current_mode = CY_MODE_BOOTLOADER;
	}

	mutex_unlock(&ts->data_lock);

	return retval;
}

static int _sh_tpsif_sw_reset(struct cyttsp4 *ts)
{
	int retval = 0;

	retval = _cyttsp4_soft_reset(ts);
	if (retval == 0) {
		/* ts->power_state = CY_INVALID_STATE; */
		ts->current_mode = CY_MODE_BOOTLOADER;
	}

	return retval;
}

static int sh_tpsif_sw_reset(struct cyttsp4 *ts)
{
	int retval = 0;

	mutex_lock(&ts->data_lock);
	retval = _sh_tpsif_sw_reset(ts);
	mutex_unlock(&ts->data_lock);

	return retval;
}

/* copied from _cyttsp4_startup_tma400() */
static int _sh_tpsif_startup_sw_reset_tma400(struct cyttsp4 *ts)
{
	int i, j, k;
	int tries;
	int retval = 0;
	u8 read_ic_crc[4];
	u8 calc_ic_crc[4];
	u8 table_crc[4];
	u8 stat_regs[3];
#ifdef SH_TPSIF_COMMAND
	u32 version;
#endif	/* SH_TPSIF_COMMAND */
	bool put_all_params_done = true;
	/* 
	 * FIXME
	 *
	 * Default value of "put_all_params_done" is false.  Now
	 * changed to true in order to check firmware update function.
	 * 
	 * Don't forget to return to false in software for production.
	 *
	 */

	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

_cyttsp4_startup_tma400_restart:
	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: enter power_state=%d\n", __func__, ts->power_state);
	ts->power_state = CY_BL_STATE;
	ts->current_mode = CY_MODE_BOOTLOADER;
	cyttsp4_pr_state(ts);
	_sh_tpsif_sw_reset(ts);	/* Soft Reset */
	/* wait for first interrupt */
	INIT_COMPLETION(ts->bl_int_running);
	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: wait for first bootloader interrupt\n", __func__);
	if (mutex_is_locked(&ts->data_lock)) {
		mutex_unlock(&ts->data_lock);
		retval = wait_for_completion_interruptible_timeout(
			&ts->bl_int_running,
			msecs_to_jiffies(CY_DELAY_DFLT * CY_DELAY_MAX * 10));
		mutex_lock(&ts->data_lock);
	} else {
		retval = wait_for_completion_interruptible_timeout(
			&ts->bl_int_running,
			msecs_to_jiffies(CY_DELAY_DFLT * CY_DELAY_MAX * 10));
	}
	if (retval == 0) {
		pr_err("%s: timeout waiting for bootloader interrupt\n",
			__func__);
		retval = -ETIMEDOUT;
		goto _cyttsp4_startup_tma400_exit;
	}

	msleep(1000);

	/* wait for second interrupt */
	INIT_COMPLETION(ts->bl_int_running);
	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: wait for second bootloader interrupt\n", __func__);
	if (mutex_is_locked(&ts->data_lock)) {
		mutex_unlock(&ts->data_lock);
		retval = wait_for_completion_interruptible_timeout(
			&ts->bl_int_running,
			msecs_to_jiffies(CY_DELAY_DFLT * CY_DELAY_MAX * 10));
		mutex_lock(&ts->data_lock);
	} else {
		retval = wait_for_completion_interruptible_timeout(
			&ts->bl_int_running,
			msecs_to_jiffies(CY_DELAY_DFLT * CY_DELAY_MAX * 10));
	}
	if (retval == 0) {
		pr_err("%s: timeout waiting for second bootloader interrupt\n",
			__func__);
		retval = -ETIMEDOUT;
		goto _cyttsp4_startup_tma400_exit;
	}

	retval = _cyttsp4_ldr_exit(ts);
	if (retval < 0) {
		pr_err("%s: Fail exit bootloader r=%d\n",
			__func__, retval);
		ts->power_state = CY_IDLE_STATE;
		cyttsp4_pr_state(ts);
		goto _cyttsp4_startup_tma400_exit;
	}

	/* wait for app ready interrupt: switch to application */
	ts->power_state = CY_SYSINFO_STATE;
	ts->current_mode = CY_MODE_SYSINFO;
	INIT_COMPLETION(ts->si_int_running);
	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: wait for application ready interrupt\n", __func__);
	if (mutex_is_locked(&ts->data_lock)) {
		mutex_unlock(&ts->data_lock);
		retval = wait_for_completion_interruptible_timeout(
			&ts->si_int_running,
			msecs_to_jiffies(CY_DELAY_DFLT * CY_DELAY_MAX * 20));
		mutex_lock(&ts->data_lock);
	} else {
		retval = wait_for_completion_interruptible_timeout(
			&ts->si_int_running,
			msecs_to_jiffies(CY_DELAY_DFLT * CY_DELAY_MAX * 20));
	}
	if (retval == 0) {
		pr_err("%s: timeout waiting for application ready interrupt\n",
			__func__);
		retval = -ETIMEDOUT;
		goto _cyttsp4_startup_tma400_exit;
	}

	msleep(5000);	/* get past ratty int edges; remove when int fixed */

	/* Wait for IRQ to toggle high */
	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: wait for irq toggle high\n", __func__);
	retval = -ETIMEDOUT;
	for (i = 0, j = 0, k = 0; i < CY_DELAY_MAX * 10 * 5 * 10; i++, j++) {
		if (ts->platform_data->irq_stat() == CY_IRQ_DEASSERT) {
			retval = 0;
			break;
		}
		mdelay(CY_DELAY_DFLT);
		retval = 0;
		if (j >= 4 * CY_DELAY_MAX) {
			/* test to see if int line stuck; try to release */
			retval = cyttsp4_read_block_data(ts, CY_REG_BASE,
				sizeof(stat_regs), stat_regs,
				ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
			if (retval < 0) {
				cyttsp4_dbg(ts, CY_DBG_LVL_3,
					"%s: read attempt=%d fail\n",
					__func__, k);
				j = 0;
				k++;
			} else {
				cyttsp4_dbg(ts, CY_DBG_LVL_3,
					"%s: read attempt=%d pass\n",
					__func__, k);
				break;
			}
			retval = 0;
		}
	}
	if (retval < 0) {
		pr_err("%s: timeout waiting for irq to de-assert\n",
			__func__);
		/* try running anyway; continue */
	}

	tries = 0;
	while (tries++ < CY_NUM_RETRY) {
		cyttsp4_dbg(ts, CY_DBG_LVL_3,
			"%s: Read Sysinfo regs and get rev numbers try=%d\n",
			__func__, tries);
		retval = _cyttsp4_get_sysinfo_regs(ts);
		if (retval < 0) {
			pr_err("%s: Read Block fail -get sys regs (r=%d)\n",
				__func__, retval);
			msleep(1000);
		} else
			break;
	}

#ifdef SH_TPSIF_COMMAND
	if (_sh_tpsif_firmware_version(ts, &version) < 0) {
		pr_err("%s: Touch Panel Firmware Version = Unknown\n", __func__);
	} else {
		pr_info("%s: Touch Panel Firmware Version = %02X.%02X.%02X%02X\n",
			__func__, ((u8 *)&version)[3], ((u8 *)&version)[2],
			((u8 *)&version)[1], ((u8 *)&version)[0]);
		if (((u8 *)&version)[3] == 0x00 && ((u8 *)&version)[2] == 0x24) {
			pr_info("%s: Firmware for ES10.2\n", __func__);
		} else {
			pr_info("%s: Firmware for ES100\n", __func__);
		}
	}
#endif	/* SH_TPSIF_COMMAND */

	retval = _cyttsp4_set_config_mode(ts);
	if (retval < 0) {
		pr_err("%s: Failed to switch to config mode\n",
			__func__);
		goto _cyttsp4_startup_tma400_bypass_crc_check;
	}
#ifdef CONFIG_TOUCHSCREEN_DEBUG
	if (ts->ic_grptest)
		goto _cyttsp4_startup_tma400_bypass_tbl_crc_chk;
#endif
	memset(read_ic_crc, 0, sizeof(read_ic_crc));
	memset(calc_ic_crc, 0, sizeof(calc_ic_crc));
	memset(table_crc, 0, sizeof(table_crc));
	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: Read IC CRC values\n", __func__);
	retval = _cyttsp4_read_ic_crc_tma400(ts,
		read_ic_crc, sizeof(read_ic_crc));
	if (retval < 0) {
		pr_err("%s: Fail read ic crc r=%d\n",
			__func__, retval);
	}

	_cyttsp4_pr_buf(ts, read_ic_crc, sizeof(read_ic_crc), "read_ic_crc");
	_cyttsp4_pr_buf(ts, calc_ic_crc, sizeof(calc_ic_crc), "calc_ic_crc");

	if (!put_all_params_done) {
		if (ts->platform_data->sett
			[CY_IC_GRPNUM_TCH_PARM_VAL]->data == NULL) {
			pr_err("%s: missing param table\n", __func__);
			goto _cyttsp4_startup_tma400_bypass_tbl_crc_chk;
		}
		memcpy(table_crc, ts->platform_data->sett
			[CY_IC_GRPNUM_TCH_PARM_VAL]->data, sizeof(table_crc));
		_cyttsp4_pr_buf(ts, table_crc, sizeof(calc_ic_crc),
			"table_crc");
		if ((read_ic_crc[0] != table_crc[0]) ||
			(read_ic_crc[1] != table_crc[1])) {
			retval = _cyttsp4_put_all_params_tma400(ts);
			if (retval < 0) {
				pr_err("%s: Fail put all params r=%d\n",
					__func__, retval);
				goto _cyttsp4_startup_tma400_bypass_tbl_crc_chk;
			}
			put_all_params_done = true;
			goto _cyttsp4_startup_tma400_restart;
		}
	}

_cyttsp4_startup_tma400_bypass_tbl_crc_chk:
	/* get handshake settings in config mode */
	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: enable handshake\n", __func__);
	retval = _cyttsp4_hndshk_enable(ts);
	if (retval < 0)
		pr_err("%s: fail enable handshake r=%d", __func__, retval);

_cyttsp4_startup_tma400_bypass_crc_check:
	cyttsp4_dbg(ts, CY_DBG_LVL_3,
		"%s: enter operational mode\n", __func__);
	retval = _cyttsp4_set_operational_mode(ts);
	if (retval < 0) {
		pr_err("%s: Fail set operational mode (r=%d)\n",
			__func__, retval);
		ts->power_state = CY_IDLE_STATE;
		cyttsp4_pr_state(ts);
		goto _cyttsp4_startup_tma400_exit;
	}

	ts->power_state = CY_ACTIVE_STATE;
	cyttsp4_pr_state(ts);

_cyttsp4_startup_tma400_exit:
	return retval;
}

static int _sh_tpsif_startup_sw_reset(struct cyttsp4 *ts)
{
	int retval = 0;

	if (ts->flags & CY_FLAG_TMA400) {
		retval = _sh_tpsif_startup_sw_reset_tma400(ts);
		if (retval < 0) {
			pr_err("%s: Fail startup TMA400 r=%d\n",
				__func__, retval);
		}
	}

	return retval;
}

static int sh_tpsif_hw_reset_startup(struct cyttsp4 *ts)
{
	int retval = 0;

	mutex_lock(&ts->data_lock);
	retval = _cyttsp4_startup(ts);
	mutex_unlock(&ts->data_lock);

	return retval;
}

static int sh_tpsif_sw_reset_startup(struct cyttsp4 *ts)
{
	int retval = 0;

	mutex_lock(&ts->data_lock);
	retval = _sh_tpsif_startup_sw_reset(ts);
	mutex_unlock(&ts->data_lock);

	return retval;
}

static int sh_tpsif_change_mode(struct cyttsp4 *ts, int mode)
{
	int retval = -EINVAL;

	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

	mutex_lock(&ts->data_lock);

	switch (mode) {
	case TPSIF_MODE_OPERATE:
		retval = _cyttsp4_set_operational_mode(ts);
		break;
	case TPSIF_MODE_SYSINFO:
		retval = _cyttsp4_set_sysinfo_mode(ts);
		break;
	case TPSIF_MODE_CONFIG:
		retval = _cyttsp4_set_config_mode(ts);
		break;
	}

	if (retval < 0) {
		ts->power_state = CY_IDLE_STATE;
		cyttsp4_pr_state(ts);
		pr_err("%s: Fail change mode (r=%d)\n", __func__, retval);
	}

	mutex_unlock(&ts->data_lock);
	return retval;
}

static int _sh_tpsif_read_scan_result(struct cyttsp4 *ts, int type, struct cyttsp4_scan_data *data)
{
	int retval = 0;
	uint8_t response[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

	retval = cyttsp4_read_block_data(ts, ts->si_ofs.cmd_ofs,
					sizeof(response), &(response[0]),
					ts->platform_data->addr[CY_TCH_ADDR_OFS],
					true);
	if (retval < 0) {
		pr_err("%s: fail read cmd status"
			" r=%d\n", __func__, retval);
		goto _sh_tpsif_read_scan_result_error;
	}

	pr_info("%s: Status         = 0x%02X\n", __func__, response[1]);//DEBUG
	pr_info("%s: Data ID        = 0x%02X\n", __func__, response[2]);//DEBUG
	pr_info("%s: Num Elements 1 = 0x%02X\n", __func__, response[3]);//DEBUG
	pr_info("%s: Num Elements 2 = 0x%02X\n", __func__, response[4]);//DEBUG
	pr_info("%s: Data format    = 0x%02X\n", __func__, response[5]);//DEBUG

	data->data_num = ((u16)response[3] << 8) | response[4];
	data->data_size = response[5] & 0x07;
	data->matrix_mapping = (response[5] & 0x08) >> 3;
	data->endian = (response[5] & 0x10) >> 4;
	data->sign_type = (response[5] & 0x20) >> 5;
	data->scan_type = type;

	data->buf_size = sizeof(u8) * data->data_size * data->data_num;

	retval = cyttsp4_read_block_data(ts, ts->si_ofs.cmd_ofs + 6,
					data->buf_size, data->buf,
					ts->platform_data->addr[CY_TCH_ADDR_OFS],
					true);
	if (retval < 0) {
		pr_err("%s: fail read scan result"
			" r=%d\n", __func__, retval);
		goto _sh_tpsif_read_scan_result_error;
	}

	goto _sh_tpsif_read_scan_result_exit;

_sh_tpsif_read_scan_result_error:

_sh_tpsif_read_scan_result_exit:

	return retval;
}

static int _sh_tpsif_retrieve_panel_scan(struct cyttsp4 *ts, int type)
{
	int retval = 0;
	uint8_t *buf = NULL;
	size_t buf_size;
	int tmp_state = 0;
	int tries = 0;
	uint8_t response[2] = {0x00, 0x00};

	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

	tmp_state = ts->power_state;
	ts->power_state = CY_CMD_STATE;
	cyttsp4_pr_state(ts);

	ts->cmd_rdy = false;

	buf_size = sizeof(uint8_t) * 6;
	buf = kzalloc(buf_size, GFP_KERNEL);
	if (buf == NULL) {
		pr_err("%s: Failed to allocate buffer\n", __func__);
		retval = -ENOMEM;
		goto _sh_tpsif_retrieve_panel_scan_exit;
	}

	if (type < 0 || type >= TPSIF_SCAN_TYPE_NUM) {
		pr_err("%s: Ivalid scan type %d\n", __func__, type);
		retval = -EINVAL;
		goto _sh_tpsif_retrieve_panel_scan_exit;
	}

	/* Set command bytes */
	buf[0] = 0x0C; /* Command */
	buf[1] = 0x00; /* Offset */
	buf[2] = 0x00; /* Offset */
	buf[3] = 0x00; /* (12 * 21) high */  //FIXME
	buf[4] = 0xFC; /* (12 * 21) low */  //FIXME
	buf[5] = type;

	retval = cyttsp4_write_block_data(ts, 0x03, 5, &(buf[1]),
		ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
	if (retval < 0) {
		pr_err("%s: Failed to write parameters for retrieve command r=%d\n",
			__func__, retval);
		goto _sh_tpsif_retrieve_panel_scan_exit;
	}

	/* Write command */
	retval = cyttsp4_write_block_data(ts, 0x02, 1, &(buf[0]),
		ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
	if (retval < 0) {
		pr_err("%s: Failed to write retrieve command r=%d\n",
			__func__, retval);
		goto _sh_tpsif_retrieve_panel_scan_exit;
	}

	/* Wait for cmd rdy interrupt */
	while (tries++ < 10000) {
		if (mutex_is_locked(&ts->data_lock)) {
			mutex_unlock(&ts->data_lock);
			udelay(1000);
			mutex_lock(&ts->data_lock);
		} else {
			udelay(1000);
		}

		if (ts->cmd_rdy) {
			memset(response, 0, sizeof(response));
			retval = cyttsp4_read_block_data(ts, ts->si_ofs.cmd_ofs,
				sizeof(response), &(response[0]),
				ts->platform_data->addr[CY_TCH_ADDR_OFS],
				true);
			if (retval < 0) {
				pr_err("%s: fail read cmd status"
					" r=%d\n", __func__, retval);
				goto _sh_tpsif_retrieve_panel_scan_exit;
			} else if (response[0] & CY_CMD_RDY_BIT) {
				break;
			} else {
				/* not our interrupt */
				ts->cmd_rdy = false;
			}
		}
	}

	if (tries >= 10000) {
		pr_err("%s: Retrieve panel scan command timeout\n", __func__);
		retval = -ETIMEDOUT;
		goto _sh_tpsif_retrieve_panel_scan_exit;
	}

	if (response[1] != 0x00) {
		pr_err("%s: Retrieve panel scan command failed"
			" response=%02X %02X tries=%d\n",
			__func__, response[0], response[1], tries);
		retval = -EIO;
	}

_sh_tpsif_retrieve_panel_scan_exit:
	kfree(buf);
	ts->power_state = tmp_state;
	cyttsp4_pr_state(ts);
	return retval;
}

static int _sh_tpsif_execute_panel_scan(struct cyttsp4 *ts)
{
	int retval = 0;
	uint8_t *buf = NULL;
	size_t buf_size;
	int tmp_state = 0;
	int tries = 0;
	uint8_t response[2] = {0x00, 0x00};

	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

	tmp_state = ts->power_state;
	ts->power_state = CY_CMD_STATE;
	cyttsp4_pr_state(ts);

	ts->cmd_rdy = false;

	buf_size = sizeof(uint8_t) * 1;
	buf = kzalloc(buf_size, GFP_KERNEL);
	if (buf == NULL) {
		pr_err("%s: Failed to allocate buffer\n", __func__);
		retval = -ENOMEM;
		goto _sh_tpsif_execute_panel_scan_exit;
	}

	/* Set command bytes */
	buf[0] = 0x0B; /* Command */

	/* Write command */
	retval = cyttsp4_write_block_data(ts, 0x02, 1, &(buf[0]),
		ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
	if (retval < 0) {
		pr_err("%s: Failed to write execute panel scan command r=%d\n",
			__func__, retval);
		goto _sh_tpsif_execute_panel_scan_exit;
	}

	/* Wait for cmd rdy interrupt */
	while (tries++ < 10000) {
		if (mutex_is_locked(&ts->data_lock)) {
			mutex_unlock(&ts->data_lock);
			udelay(1000);
			mutex_lock(&ts->data_lock);
		} else {
			udelay(1000);
		}

		if (ts->cmd_rdy) {
			memset(response, 0, sizeof(response));
			retval = cyttsp4_read_block_data(ts, ts->si_ofs.cmd_ofs,
				sizeof(response), &(response[0]),
				ts->platform_data->addr[CY_TCH_ADDR_OFS],
				true);
			if (retval < 0) {
				pr_err("%s: fail read cmd status"
					" r=%d\n", __func__, retval);
				goto _sh_tpsif_execute_panel_scan_exit;
			} else if (response[0] & CY_CMD_RDY_BIT) {
				break;
			} else {
				/* not our interrupt */
				ts->cmd_rdy = false;
			}
		}
	}

	if (tries >= 10000) {
		pr_err("%s: Execute panel scan command timeout\n", __func__);
		retval = -ETIMEDOUT;
		goto _sh_tpsif_execute_panel_scan_exit;
	}

	if (response[1] != 0x00) {
		pr_err("%s: Execute panel scan command failed"
			" response=%02X %02X tries=%d\n",
			__func__, response[0], response[1], tries);
		retval = -EIO;
	}

_sh_tpsif_execute_panel_scan_exit:
	kfree(buf);
	ts->power_state = tmp_state;
	cyttsp4_pr_state(ts);
	return retval;
}

static int sh_tpsif_scan_rawdata(struct cyttsp4 *ts, int type, struct cyttsp4_scan_data *data)
{
	int retval = 0;

	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

	mutex_lock(&ts->data_lock);

	/* Switch to config mode */
	if (ts->current_mode != CY_MODE_CONFIG) {
		retval = _cyttsp4_set_config_mode(ts);
		if (retval < 0) {
			pr_err("%s: Failed to switch to config mode"
				" for sysinfo regs\n", __func__);
			goto sh_tpsif_scan_rawdata_exit;
		}
	}

	/* Execute panel scan */
	retval = _sh_tpsif_execute_panel_scan(ts);
	if (retval < 0) {
		pr_err("%s: Fail execute panel scan r=%d\n", __func__, retval);
		sh_tpsif_polling_loop = 0;
		goto sh_tpsif_scan_rawdata_op_mode;
	}

	/* Retrieve panel scan */
	retval = _sh_tpsif_retrieve_panel_scan(ts, type);
	if (retval < 0) {
		pr_err("%s: Fail retrieve panel scan r=%d\n", __func__, retval);
		sh_tpsif_polling_loop = 0;
		goto sh_tpsif_scan_rawdata_op_mode;
	}

	/* Read scan result */
	retval = _sh_tpsif_read_scan_result(ts, type, data);
	if (retval < 0) {
		pr_err("%s: Fail read scan result r=%d\n", __func__, retval);
		sh_tpsif_polling_loop = 0;
		goto sh_tpsif_scan_rawdata_op_mode;
	}

sh_tpsif_scan_rawdata_op_mode:

#if 0
	retval = _cyttsp4_set_operational_mode(ts);
	if (retval < 0) {
		ts->power_state = CY_IDLE_STATE;
		cyttsp4_pr_state(ts);
		pr_err("%s: Fail set operational mode (r=%d)\n", __func__, retval);
	}
#endif

sh_tpsif_scan_rawdata_exit:

	mutex_unlock(&ts->data_lock);

	return retval;
}

static int _sh_tpsif_calibration(struct cyttsp4 *ts, u8 mode)
{
	int retval = 0;
	uint8_t *buf = NULL;
	size_t buf_size;
	int tmp_state = 0;
	int tries = 0;
	uint8_t response[2] = {0x00, 0x00};

	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

	tmp_state = ts->power_state;
	ts->power_state = CY_CMD_STATE;
	cyttsp4_pr_state(ts);

	ts->cmd_rdy = false;

	buf_size = sizeof(uint8_t) * 4;
	buf = kzalloc(buf_size, GFP_KERNEL);
	if (buf == NULL) {
		pr_err("%s: Failed to allocate buffer\n", __func__);
		retval = -ENOMEM;
		goto _sh_tpsif_calibration_exit;
	}

	/* Set command bytes */
	buf[0] = 0x09; /* Command */
	buf[1] = mode; /* Mode */
	buf[2] = 0x00; /* Force / Gain */
	buf[3] = 0x00; /* Global IDAC */

	retval = cyttsp4_write_block_data(ts, 0x03, 3, &(buf[1]),
		ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
	if (retval < 0) {
		pr_err("%s: Failed to write calibration parameters r=%d\n",
			__func__, retval);
		goto _sh_tpsif_calibration_exit;
	}

	/* Write command */
	retval = cyttsp4_write_block_data(ts, 0x02, 1, &(buf[0]),
		ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
	if (retval < 0) {
		pr_err("%s: Failed to write calibration command r=%d\n",
			__func__, retval);
		goto _sh_tpsif_calibration_exit;
	}

	/* Wait for cmd rdy interrupt */
	while (tries++ < 10000) {
		if (mutex_is_locked(&ts->data_lock)) {
			mutex_unlock(&ts->data_lock);
			udelay(1000);
			mutex_lock(&ts->data_lock);
		} else {
			udelay(1000);
		}

		if (ts->cmd_rdy) {
			memset(response, 0, sizeof(response));
			retval = cyttsp4_read_block_data(ts, ts->si_ofs.cmd_ofs,
				sizeof(response), &(response[0]),
				ts->platform_data->addr[CY_TCH_ADDR_OFS],
				true);
			if (retval < 0) {
				pr_err("%s: fail read cmd status"
					" r=%d\n", __func__, retval);
				goto _sh_tpsif_calibration_exit;
			} else if (response[0] & CY_CMD_RDY_BIT) {
				break;
			} else {
				/* not our interrupt */
				ts->cmd_rdy = false;
			}
		}
	}

	if (tries >= 10000) {
		pr_err("%s: command calibration timeout\n", __func__);
		retval = -ETIMEDOUT;
		goto _sh_tpsif_calibration_exit;
	}

	if (response[1] != 0x00) {
		pr_err("%s: Calibration command failed"
			" response=%02X %02X tries=%d\n",
			__func__, response[0], response[1], tries);
		retval = -EIO;
	}

_sh_tpsif_calibration_exit:
	kfree(buf);
	ts->power_state = tmp_state;
	cyttsp4_pr_state(ts);
	return retval;
}

#if 0
static int sh_tpsif_calibration_idac_mode(struct cyttsp4 *ts, u8 mode)
{
	int retval = 0;

	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

	mutex_lock(&ts->data_lock);

	retval = _cyttsp4_set_config_mode(ts);
	if (retval < 0) {
		pr_err("%s: Failed to switch to config mode"
			" for sysinfo regs\n", __func__);
		goto sh_tpsif_calibration_idac_exit;
	}

	retval = _sh_tpsif_calibration(ts, mode);
	if (retval < 0) {
		pr_err("%s: Fail calibration mode=%d, r=%d\n", __func__, mode, retval);
		goto sh_tpsif_calibration_idac_op_mode;
	}

sh_tpsif_calibration_idac_op_mode:

	retval = _cyttsp4_set_operational_mode(ts);
	if (retval < 0) {
		ts->power_state = CY_IDLE_STATE;
		cyttsp4_pr_state(ts);
		pr_err("%s: Fail set operational mode (r=%d)\n", __func__, retval);
	}

sh_tpsif_calibration_idac_exit:

	mutex_unlock(&ts->data_lock);

	return retval;
}
#endif

static int sh_tpsif_calibration_idac(struct cyttsp4 *ts)
{
	int retval = 0;

	cyttsp4_dbg(ts, CY_DBG_LVL_4, "%s: start\n", __func__);

	mutex_lock(&ts->data_lock);

	retval = _cyttsp4_set_config_mode(ts);
	if (retval < 0) {
		pr_err("%s: Failed to switch to config mode"
			" for sysinfo regs\n", __func__);
		goto sh_tpsif_calibration_idac_exit;
	}

	retval = _sh_tpsif_calibration(ts, TPSIF_CALIB_MODE_MUTUAL_FINE);
	if (retval < 0) {
		pr_err("%s: Fail mutal calibration r=%d\n", __func__, retval);
		goto sh_tpsif_calibration_idac_op_mode;
	}

	retval = _sh_tpsif_calibration(ts, TPSIF_CALIB_MODE_SELF);
	if (retval < 0) {
		pr_err("%s: Fail self calibration r=%d\n", __func__, retval);
		goto sh_tpsif_calibration_idac_op_mode;
	}

sh_tpsif_calibration_idac_op_mode:

	retval = _cyttsp4_set_operational_mode(ts);
	if (retval < 0) {
		ts->power_state = CY_IDLE_STATE;
		cyttsp4_pr_state(ts);
		pr_err("%s: Fail set operational mode (r=%d)\n", __func__, retval);
	}

sh_tpsif_calibration_idac_exit:

	mutex_unlock(&ts->data_lock);

	return retval;
}

static int _sh_tpsif_firmware_version(struct cyttsp4 *ts, u32 *version)
{
	int retval = 0;
	u8 ver_major, ver_minor;
	u8 config_verh, config_verl;

	if (ts->sysinfo_ptr.cydata == NULL || ts->sysinfo_ptr.pcfg == NULL) {
		retval = -EINVAL;
		goto _sh_tpsif_firmware_version_exit;
	}

	ver_major = ts->sysinfo_ptr.cydata->fw_ver_major;
	ver_minor = ts->sysinfo_ptr.cydata->fw_ver_minor;

	if (ver_major == 0x00 && ver_minor == 0x24) {
		/* ES10.2 */
		config_verh = ts->sysinfo_ptr.pcfg->len_xh;
		config_verl = ts->sysinfo_ptr.pcfg->len_xl;
	} else {
		/* ES100 */
		config_verh = ts->sysinfo_ptr.cydata->cyito_verh;
		config_verl = ts->sysinfo_ptr.cydata->cyito_verl;
	}

	*version = (ver_major << 24) | (ver_minor << 16)
		| (config_verh << 8) | (config_verl << 0);

_sh_tpsif_firmware_version_exit:

	return retval;
}

static int _sh_tpsif_firmware_update(struct cyttsp4 *ts, u8 *data, size_t size)
{
	int retval = 0;
	u8 header_size = 0;

	mutex_lock(&ts->data_lock);

	if ((data == NULL) || (size == 0)) {
		dev_err(ts->dev, "%s: No firmware received\n", __func__);
		goto _sh_tpsif_firmware_update_exit;
	}

	header_size = data[0];
	if (header_size >= (size + 1)) {
		dev_err(ts->dev, "%s: Firmware format is invalid\n", __func__);
		goto _sh_tpsif_firmware_update_exit;
	}

	retval = _cyttsp4_load_app(ts, &(data[header_size + 1]),
		size - (header_size + 1));
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: Firmware update failed with error code %d\n",
			__func__, retval);
		ts->power_state = CY_IDLE_STATE;
		cyttsp4_pr_state(ts);
		retval = -EIO;
		goto _sh_tpsif_firmware_update_exit;
	}

	ts->debug_upgrade = true;

	retval = _cyttsp4_startup(ts);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: Failed to restart IC with error code %d\n",
			__func__, retval);
		ts->power_state = CY_IDLE_STATE;
		cyttsp4_pr_state(ts);
	}

_sh_tpsif_firmware_update_exit:
	mutex_unlock(&ts->data_lock);
	return retval;
}

static int sh_tpsif_start_firmware_update(struct cyttsp4 *ts, u8 *data, size_t size)
{
	int retval = -EFAULT;

	if (ts->waiting_for_fw) {
		dev_err(ts->dev, "%s: Driver is already waiting for firmware\n",
			__func__);
		retval = -EALREADY;
		goto sh_tpsif_start_firmware_update_exit;
	}

	dev_dbg(ts->dev,
		"%s: Enabling firmware class loader\n", __func__);

	ts->waiting_for_fw = true;
	retval = _sh_tpsif_firmware_update(ts, data, size);
	ts->waiting_for_fw = false;
	if (retval < 0) {
		dev_err(ts->dev, "%s: Fail request firmware class file load\n",
			__func__);
	} else {
		retval = size;
	}

sh_tpsif_start_firmware_update_exit:
	return retval;
}

static void _sh_tpsif_poll_start(struct cyttsp4 *ts)
{
	pr_info("%s: start\n", __func__);//DEBUG

	hrtimer_cancel(&sh_tpsif_polling_timer);
	hrtimer_start(&sh_tpsif_polling_timer, ktime_set(0, SH_TPSIF_POLL_INTERVAL * 1000 * 1000), HRTIMER_MODE_REL);

	sh_tpsif_polling_loop = 1;
}

static void _sh_tpsif_poll_stop(struct cyttsp4 *ts)
{
	pr_info("%s: start\n", __func__);//DEBUG

	sh_tpsif_polling_loop = 0;

	hrtimer_try_to_cancel(&sh_tpsif_polling_timer);
}

static void _sh_tpsif_poll_scan(struct work_struct *work)
{
	//struct cyttsp4 *ts = container_of(work, struct cyttsp4, sh_tpsif_polling_work);
	struct cyttsp4 *ts = g_tpsif_ts;

	pr_info("%s: start\n", __func__);//DEBUG

	if (sh_tpsif_scan_type == TPSIF_SCAN_TYPE_MUTUAL_RAW_AND_BASE) {
		sh_tpsif_scan_rawdata(ts, TPSIF_SCAN_TYPE_MUTUAL_RAW, &sh_tpsif_scan_data[0]);
		sh_tpsif_scan_rawdata(ts, TPSIF_SCAN_TYPE_MUTUAL_BASE, &sh_tpsif_scan_data[1]);
	} else if (sh_tpsif_scan_type == TPSIF_SCAN_TYPE_SELF_RAW_AND_BASE) {
		sh_tpsif_scan_rawdata(ts, TPSIF_SCAN_TYPE_SELF_RAW, &sh_tpsif_scan_data[0]);
		sh_tpsif_scan_rawdata(ts, TPSIF_SCAN_TYPE_SELF_BASE, &sh_tpsif_scan_data[1]);
	} else
		sh_tpsif_scan_rawdata(ts, sh_tpsif_scan_type, &sh_tpsif_scan_data[0]);

	sh_tpsif_event = 1;
	wake_up_interruptible(&sh_tpsif_wq);

	if (sh_tpsif_polling_loop)
		_sh_tpsif_poll_start(ts);
}

static enum hrtimer_restart _sh_tpsif_poll_timer_handler(struct hrtimer *timer)
{
	schedule_work(&sh_tpsif_polling_work);

	return HRTIMER_NORESTART;
}

static int sh_tpsif_command(struct cyttsp4 *ts, unsigned int cmd, unsigned long arg)
{
	int retval = 0;
	void __user *argp = (void __user *)arg;
	u8 *fw;
	static u16 addr = 0x00;
	u8 data;
	u32 version;
	int mode;
	int scan_type;
	u16 param[AREA_COUNT];

	switch (cmd) {
	case TPSIF_POWER_ON:
		retval = sh_tpsif_hw_power(ts, SH_TPSIF_HW_POWER_ON);
		break;
	case TPSIF_POWER_OFF:
		retval = sh_tpsif_hw_power(ts, SH_TPSIF_HW_POWER_OFF);
		break;
	case TPSIF_HWRESET:
		retval = sh_tpsif_hw_reset(ts);
		break;
	case TPSIF_SWRESET:
		retval = sh_tpsif_sw_reset(ts);
		break;
	case TPSIF_SLEEP_ON:
		retval = cyttsp4_suspend(ts);
		break;
	case TPSIF_SLEEP_OFF:
		retval = cyttsp4_resume(ts);
		break;
	case TPSIF_FW_VERSION:
		retval = _sh_tpsif_firmware_version(ts, &version);
		if (retval < 0)
			dev_err(ts->dev, "%s: fail get version\n", __func__);
		if (copy_to_user((u32 __user *)argp, &version, sizeof(version))) {
			return -EFAULT;
		}
		break;
	case TPSIF_FW_UPDATE:
		fw = kzalloc(TPSIF_FW_IMG_SIZE, GFP_KERNEL);
		if (fw  == NULL) {
			dev_err(ts->dev, "%s: fail allocate memory for firmware\n", __func__);
			return -ENOMEM;
		}
		if (copy_from_user(fw, (u8 __user *)argp, TPSIF_FW_IMG_SIZE)) {
			kfree(fw);
			return -EFAULT;
		}
		retval = sh_tpsif_start_firmware_update(ts, fw, TPSIF_FW_IMG_SIZE);
		kfree(fw);
		break;
	case TPSIF_DRV_STAT:
		/* TODO */
		break;
	case TPSIF_DRV_VER:
		/* TODO */
		break;
	case TPSIF_IRQ_STAT:
		/* TODO */
		break;
	case TPSIF_CALIBRATION_IDAC:
		retval = sh_tpsif_calibration_idac(ts);
		if (retval < 0)
			dev_err(ts->dev, "%s: fail calibration\n", __func__);
		break;
	case TPSIF_SET_REG_ADDR:
		if (copy_from_user(&addr, (u16 __user *)argp, sizeof(addr))) {
			return -EFAULT;
		}
		break;
	case TPSIF_READ_REG:
		retval = cyttsp4_read_block_data(ts, addr, sizeof(data), &data,
						ts->platform_data->addr[CY_TCH_ADDR_OFS], false);
		if (retval < 0)
			dev_err(ts->dev, "%s: fail read register %04x\n", __func__, addr);
		if (copy_to_user((u8 __user *)argp, &data, sizeof(data))) {
			return -EFAULT;
		}
		break;
	case TPSIF_WRITE_REG:
		if (copy_from_user(&data, (u8 __user *)argp, sizeof(data))) {
			return -EFAULT;
		}
		retval = cyttsp4_write_block_data(ts, addr, sizeof(data), &data,
						ts->platform_data->addr[CY_TCH_ADDR_OFS], false);
		if (retval < 0)
			dev_err(ts->dev, "%s: fail write register %04x\n", __func__, addr);
		break;
	case TPSIF_SWITCH_MODE:
		if (copy_from_user(&mode, (u8 __user *)argp, sizeof(mode))) {
			return -EFAULT;
		}
		_sh_tpsif_poll_stop(ts);
		retval = sh_tpsif_change_mode(ts, mode);
		break;
	case TPSIF_START_SCAN:
                if (copy_from_user(&scan_type, (int __user *)argp, sizeof(scan_type)))
                        return -EFAULT;
                if (scan_type < 0 || scan_type >= TPSIF_SCAN_TYPE_NUM)
                        return -EFAULT;
                _sh_tpsif_poll_stop(ts);
                sh_tpsif_scan_type = scan_type;
                _sh_tpsif_poll_start(ts);
                break;
	case TPSIF_HWRESET_STARTUP:
		retval = sh_tpsif_hw_reset_startup(ts);
		break;
	case TPSIF_SWRESET_STARTUP:
		retval = sh_tpsif_sw_reset_startup(ts);
		break;
	case TPSIF_CALIBRATION_PARAM:
		if (argp == NULL) {
			retval = sh_tpsif_set_adjust_param(ts, NULL);
		} else {
			if (copy_from_user(param, (u16 __user *)argp, sizeof(param))) {
				return -EFAULT;
			}
			retval = sh_tpsif_set_adjust_param(ts, param);
		}
		break;
	default:
		return -ENOTTY;
	}
	return retval;
}

static int sh_tpsif_open(struct inode *inode, struct file *filp)
{
	return 0;
}

static int sh_tpsif_release(struct inode *inode, struct file *filp)
{
	struct cyttsp4 *ts = g_tpsif_ts;

	sh_tpsif_event = 0;
	_sh_tpsif_poll_stop(ts);

	return 0;
}

static long sh_tpsif_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct cyttsp4 *ts = g_tpsif_ts;

	return (long) sh_tpsif_command(ts, cmd, arg);
}

static ssize_t sh_tpsif_read(struct file *filp, char *buf, size_t count, loff_t *pos)
{
	struct cyttsp4_scan_data data[2];
	ssize_t size;

	wait_event_interruptible(sh_tpsif_wq, sh_tpsif_event == 1);

	if (sh_tpsif_scan_type == TPSIF_SCAN_TYPE_MUTUAL_RAW_AND_BASE ||
		sh_tpsif_scan_type == TPSIF_SCAN_TYPE_SELF_RAW_AND_BASE) {
		memcpy(&data, &sh_tpsif_scan_data, sizeof(data));

		if (copy_to_user((u8 __user *)buf, &data, sizeof(data)))
			return -EFAULT;

		size = sizeof(data);
	} else {
		memcpy(&data[0], &sh_tpsif_scan_data[0], sizeof(data[0]));

		if (copy_to_user((u8 __user *)buf, &data[0], sizeof(data[0])))
			return -EFAULT;

		size = sizeof(data[0]);
	}

	sh_tpsif_event = 0;

	return size;
}

static unsigned int sh_tpsif_poll(struct file *filp, poll_table *wait)
{
	int retval;

	retval = wait_event_interruptible_timeout(sh_tpsif_wq,
						sh_tpsif_event == 1,
						msecs_to_jiffies(SH_TPSIF_POLL_INTERVAL));

	if(retval)
		return POLLIN | POLLRDNORM;

	return 0;
}

static const struct file_operations sh_tpsif_fops =
{
	.owner		= THIS_MODULE,
	.open		= sh_tpsif_open,
	.release	= sh_tpsif_release,
	.read		= sh_tpsif_read,
	.unlocked_ioctl = sh_tpsif_ioctl,
	.poll		= sh_tpsif_poll,
};

int __init sh_tpsif_init(void)
{
	int ret;
	dev_t major = 0;
	dev_t minor = 0;

	ret = alloc_chrdev_region(&sh_tpsif_dev, 0, 1, TPSIF_DEV_NAME);
	if (!ret) {
		major = MAJOR(sh_tpsif_dev);
		minor = MINOR(sh_tpsif_dev);
	} else
		goto sh_tpsif_err_exit;

	cdev_init(&sh_tpsif_cdev, &sh_tpsif_fops);

	sh_tpsif_cdev.owner = THIS_MODULE;
	sh_tpsif_cdev.ops = &sh_tpsif_fops;

	ret = cdev_add(&sh_tpsif_cdev, sh_tpsif_dev, 1);
	if (ret)
		goto sh_tpsif_err_add;

	sh_tpsif_class = class_create(THIS_MODULE, TPSIF_DEV_NAME);
	if (IS_ERR(sh_tpsif_class))
		goto sh_tpsif_err_add;

	device_create(sh_tpsif_class, NULL,
		sh_tpsif_dev, &sh_tpsif_cdev, TPSIF_DEV_NAME);

	return 0;

sh_tpsif_err_add:
	cdev_del(&sh_tpsif_cdev);

sh_tpsif_err_exit:
	return -1;
}
module_init(sh_tpsif_init);

void __exit sh_tpsif_exit(void)
{
	device_destroy(sh_tpsif_class, sh_tpsif_dev);
	class_destroy(sh_tpsif_class);
	cdev_del(&sh_tpsif_cdev);

	return;
}
module_exit(sh_tpsif_exit);

#endif	/* SH_TPSIF_COMMAND */

void msm_tps_setsleep(int on)
{
	return;
}
EXPORT_SYMBOL(msm_tps_setsleep);
