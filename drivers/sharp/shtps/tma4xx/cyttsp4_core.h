/*
 * Header file for:
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

#ifndef __CYTTSP4_CORE_H__
#define __CYTTSP4_CORE_H__

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/err.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#define CY_NUM_RETRY                10 /* max retries for rd/wr ops */

#define CY_I2C_NAME                 "cyttsp4-i2c"
#define CY_SPI_NAME                 "SH_touchpanel"
#define CY_DRIVER_VERSION           "Rev4-2M-23"
#define CY_DRIVER_DATE              "2011-09-30"

//#define STARTUP_CALIBRATION
#define MULTI_TOUCH_PROTOCOL_B
//#define REPORT_TOUCH_UP_EVENT
//#define REVERSE_Y_AXIS
#define MAX_TOUCH_NUM 10 //TODO: should use (CY_ABS_MAX_T - CY_ABS_MIN_T + 1).  defined in board-sharp.c
#define SH_TPSIF_COMMAND

/* use the following define to enable special debug tools for test only
#define CY_USE_DEBUG_TOOLS
 */
#define CY_USE_DEBUG_TOOLS
#define CONFIG_TOUCHSCREEN_DEBUG
#ifdef CONFIG_TOUCHSCREEN_DEBUG
/* use the following defines for dynamic debug printing */
/*
 * Level 0: Default Level
 * All debug (cyttsp_dbg) prints turned off
 */
#define CY_DBG_LVL_0                    0
/*
 * Level 1:  Used to verify driver and IC are working
 *    Input from IC, output to event queue
 */
#define CY_DBG_LVL_1                    1
/*
 * Level 2:  Used to further verify/debug the IC
 *    Output to IC
 */
#define CY_DBG_LVL_2                    2
/*
 * Level 3:  Used to further verify/debug the driver
 *    Driver internals
 */
#define CY_DBG_LVL_3                    3
/*
 * Level 4:  Used to further output verbose message
 *
 */
#define CY_DBG_LVL_4                    4

#ifdef CY_USE_DEBUG_TOOLS
#if 0 // commented out because of adding CY_DBG_LVL_4
#define CY_DBG_SUSPEND                  4
#define CY_DBG_RESUME                   5
#define CY_DBG_PUT_ALL_PARAMS		6
#define CY_DBG_RESET                    99
#endif
#define CY_DBG_SUSPEND                  5
#define CY_DBG_RESUME                   6
#define CY_DBG_PUT_ALL_PARAMS		7
#define CY_DBG_RESET                    99
#endif

#endif

#ifdef CONFIG_TOUCHSCREEN_DEBUG
#define cyttsp4_dbg(ts, l, f, a...) {\
	if (ts->bus_ops->tsdebug >= (l))\
		pr_info(f, ## a);\
}
#else
#define cyttsp4_dbg(ts, l, f, a...)
#endif

struct cyttsp4_bus_ops {
	s32 (*write)(void *handle, u16 subaddr, size_t length,
		const void *values, int i2c_addr, bool use_subaddr);
	s32 (*read)(void *handle, u16 subaddr, size_t length,
		void *values, int i2c_addr, bool use_subaddr);
	struct device *dev;
#ifdef CONFIG_TOUCHSCREEN_DEBUG
	u8 tsdebug;
#endif
};

void *cyttsp4_core_init(struct cyttsp4_bus_ops *bus_ops,
	struct device *dev, int irq, char *name);

void cyttsp4_core_release(void *handle);
#if defined(CONFIG_PM) || defined(CONFIG_HAS_EARLY_SUSPEND)
int cyttsp4_resume(void *handle);
int cyttsp4_suspend(void *handle);
#endif

#endif /* __CYTTSP4_CORE_H__ */
