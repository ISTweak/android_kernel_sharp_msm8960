/* drivers/video/msm/mipi_sharp_video_takt_qhd_pt.c  (Display Driver)
 *
 * Copyright (C) 2011 SHARP CORPORATION
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

#include "msm_fb.h"
#include "mipi_dsi.h"
#include "mipi_sharp.h"

static struct msm_panel_info pinfo;

static struct mipi_dsi_phy_ctrl dsi_video_mode_phy_db = {
	/* regulator */
	{0x03, 0x01, 0x01, 0x00, 0x20},
	/* timing   */
	{0x56, 0x1D, 0x15, 0x00, 0x58, 0x2D, 0x19,
	 0x1F, 0x18, 0x03, 0x04, 0x00},
	/* phy ctrl */
	{0x7F, 0x00, 0x00, 0x10},
	/* strength */
	{0xEE, 0x02, 0x86, 0x00},
	/* pll control */
	{0x00, 0xC8, 0x30, 0xCF, 0x00, 0x50, 0x48, 0x63,
	 0x31, 0x0F, 0x01, /*  --> Four lane configuration */
	 0x05, 0x14, 0x03, 0x00, 0x02, 0x54, 0x06, 0x10, 0x04, 0x0},
};

static int __init mipi_video_sharp_takt_qhd_pt_init(void)
{
	int ret;

#ifdef CONFIG_FB_MSM_MIPI_PANEL_DETECT
	if (msm_fb_detect_client("mipi_video_sharp_takt_qhd"))
		return 0;
#endif

	pinfo.xres = 540;
	pinfo.yres = 960;
	pinfo.type = MIPI_VIDEO_PANEL;
	pinfo.pdest = DISPLAY_1;
	pinfo.wait_cycle = 0;
	pinfo.bpp = 24;
	pinfo.lcdc.h_back_porch = 10;
	pinfo.lcdc.h_front_porch = 166;
	pinfo.lcdc.h_pulse_width = 8;
	pinfo.lcdc.v_back_porch = 2;
	pinfo.lcdc.v_front_porch = 6;
	pinfo.lcdc.v_pulse_width = 2;
	pinfo.lcdc.border_clr = 0;			/* blk */
	pinfo.lcdc.underflow_clr = 0;		/* blk */
	pinfo.lcdc.hsync_skew = 0;
	pinfo.bl_max = 15;
	pinfo.bl_min = 1;
	pinfo.fb_num = 2;

	pinfo.mipi.mode = DSI_VIDEO_MODE;
	pinfo.mipi.pulse_mode_hsa_he = FALSE;
	pinfo.mipi.hfp_power_stop = TRUE;
	pinfo.mipi.hbp_power_stop = FALSE;
	pinfo.mipi.hsa_power_stop = FALSE;
	pinfo.mipi.eof_bllp_power_stop = TRUE;
	pinfo.mipi.bllp_power_stop = TRUE;
	pinfo.mipi.traffic_mode = DSI_NON_BURST_SYNCH_EVENT;
	pinfo.mipi.dst_format = DSI_VIDEO_DST_FORMAT_RGB888;
	pinfo.mipi.vc = 0;
	pinfo.mipi.rgb_swap = DSI_RGB_SWAP_RGB;
	pinfo.mipi.esc_byte_ratio = 4;
	pinfo.mipi.data_lane0 = TRUE;
	pinfo.mipi.data_lane1 = TRUE;
	pinfo.mipi.data_lane2 = TRUE;
	pinfo.mipi.data_lane3 = FALSE;

	pinfo.mipi.force_clk_lane_hs = TRUE;

	pinfo.clk_rate = 339190000;
	pinfo.clk_min  = 339190000;
	pinfo.clk_max  = 339190000;

	pinfo.mipi.tx_eot_append = TRUE;
	pinfo.mipi.t_clk_post = 0x00;
	pinfo.mipi.t_clk_pre = 0x13;
	pinfo.mipi.stream = 0; /* dma_p */
	pinfo.mipi.frame_rate = 60;
	pinfo.mipi.dsi_phy_db = &dsi_video_mode_phy_db;

	ret = mipi_sharp_device_register(&pinfo, MIPI_DSI_PRIM,
						MIPI_DSI_PANEL_QHD_PT);
	if (ret)
		pr_err("%s: failed to register device!\n", __func__);

	return ret;
}

module_init(mipi_video_sharp_takt_qhd_pt_init);
