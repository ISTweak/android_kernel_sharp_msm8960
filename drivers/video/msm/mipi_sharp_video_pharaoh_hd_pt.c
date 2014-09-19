/* drivers/video/msm/mipi_sharp_video_pharaoh_hd_pt.c  (Display Driver)
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
#include "mipi_sharp_video.h"
#include <sharp/shdisp_kerl.h>

static struct msm_panel_info pinfo;

static struct mipi_dsi_phy_ctrl dsi_video_mode_phy_db = {
#if defined(CONFIG_MACH_LYNX_DL12) || defined(CONFIG_MACH_LYNX_DL10) || defined(CONFIG_MACH_LYNX_DL15)
	/* regulator */
    {0x03, 0x01, 0x01, 0x00, 0x20},
	/* timing   */
	{0x65, 0x21, 0x19, 0x00, 0x5d, 0x2e, 0x1c,
	0x23, 0x1c, 0x03, 0x04, 0x00},
	/* phy ctrl */
    {0x7f, 0x00, 0x00, 0x10},
	/* strength */
    {0xee, 0x02, 0x86, 0x00},
	/* pll control */
    {0x40, 0x52, 0x31, 0xd6, 0x00, 0x50, 0x48, 0x63,
     0x31, 0x0f, 0x03, /*  --> Four lane configuration */
     0x05, 0x14, 0x03, 0x00, 0x02, 0x54, 0x06, 0x10, 0x04, 0x00},
#else
	/* regulator */
	{0x03, 0x01, 0x01, 0x00, 0x20},
	/* timing   */
	{0x6A, 0x23, 0x1b, 0x00, 0x5f, 0x32, 0x1e,
	0x25, 0x1e, 0x03, 0x04, 0x00},
	/* phy ctrl */
	{0x7f, 0x00, 0x00, 0x10},
	/* strength */
	{0xee, 0x02, 0x86, 0x00},
	/* pll control */
	{0x40, 0x66, 0x31, 0xd6, 0x00, 0x50, 0x48, 0x63,
	 0x31, 0x0f, 0x03, /*  --> Four lane configuration */
	 0x05, 0x14, 0x03, 0x00, 0x02, 0x54, 0x06, 0x10, 0x04, 0x0},
#endif
};

static int __init mipi_video_sharp_pharaoh_hd_pt_init(void)
{
	int ret;

#if defined(CONFIG_SHDISP_PANEL_SWITCH)
    if (shdisp_api_get_panel_info() == 0) {
        return 0;
    }
#endif

#ifdef CONFIG_FB_MSM_MIPI_PANEL_DETECT
	if (msm_fb_detect_client("mipi_video_sharp_pharaoh_hd"))
		return 0;
#endif

#if defined(CONFIG_SHDISP_PANEL_SWITCH)
	printk(KERN_INFO "[SHDISP]%s\n", __func__);
#endif

	pinfo.xres = 720;
	pinfo.yres = 1280;
	pinfo.type = MIPI_VIDEO_PANEL;
	pinfo.pdest = DISPLAY_1;
	pinfo.wait_cycle = 0;
	pinfo.bpp = 24;
	pinfo.lcdc.h_back_porch = 20;
#if defined(CONFIG_MACH_LYNX_DL12) || defined(CONFIG_MACH_LYNX_DL10) || defined(CONFIG_MACH_LYNX_DL15)
	pinfo.lcdc.h_front_porch = 94;
	pinfo.lcdc.v_front_porch = 16;
	pinfo.lcdc.v_back_porch = 4;
	pinfo.lcdc.v_pulse_width = 4;
#else
	pinfo.lcdc.h_front_porch = 144;
	pinfo.lcdc.v_front_porch = 18;
	pinfo.lcdc.v_back_porch = 2;
	pinfo.lcdc.v_pulse_width = 2;
#endif
	pinfo.lcdc.h_pulse_width = 12;
	pinfo.lcdc.border_clr = 0;			/* blk */
	pinfo.lcdc.underflow_clr = 0;		/* blk */
	pinfo.lcdc.hsync_skew = 0;
	pinfo.bl_max = 15;
	pinfo.bl_min = 1;
	pinfo.fb_num = 2;

	pinfo.mipi.mode = DSI_VIDEO_MODE;
	pinfo.mipi.pulse_mode_hsa_he = FALSE;
	pinfo.mipi.hfp_power_stop = FALSE;
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
	pinfo.mipi.data_lane3 = TRUE;

	pinfo.mipi.force_clk_lane_hs = FALSE;

#if defined(CONFIG_MACH_LYNX_DL12) || defined(CONFIG_MACH_LYNX_DL10) || defined(CONFIG_MACH_LYNX_DL15)
	pinfo.clk_rate = 397960000;
	pinfo.clk_min  = 397960000;
	pinfo.clk_max  = 397960000;
#else
	pinfo.clk_rate = 421430000;
	pinfo.clk_min  = 421430000;
	pinfo.clk_max  = 421430000;
#endif

	pinfo.mipi.tx_eot_append = TRUE;

#if defined(CONFIG_MACH_LYNX_DL12) || defined(CONFIG_MACH_LYNX_DL10) || defined(CONFIG_MACH_LYNX_DL15)
	pinfo.mipi.t_clk_post = 0x00;
	pinfo.mipi.t_clk_pre = 0x16;
#else
	pinfo.mipi.t_clk_post = 0x00;
	pinfo.mipi.t_clk_pre = 0x17;
#endif
	pinfo.mipi.stream = 0; /* dma_p */
	pinfo.mipi.frame_rate = 60;
	pinfo.mipi.dsi_phy_db = &dsi_video_mode_phy_db;

	ret = mipi_sharp_video_device_register(&pinfo, MIPI_DSI_PRIM,
						MIPI_DSI_PANEL_WXGA);
	if (ret)
		pr_err("%s: failed to register device!\n", __func__);

	return ret;
}

module_init(mipi_video_sharp_pharaoh_hd_pt_init);
