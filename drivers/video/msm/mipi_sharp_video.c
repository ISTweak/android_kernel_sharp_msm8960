/* drivers/video/msm/mipi_sharp_video.c  (Display Driver)
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

#include "msm_fb.h"
#include "mipi_dsi.h"
#include "mipi_sharp_video.h"
#include <sharp/shdisp_kerl.h>

/* ------------------------------------------------------------------------- */
/* DEBUG MACRAOS                                                             */
/* ------------------------------------------------------------------------- */


static int ch_used[3];

static int mipi_sharp_video_lcd_on(struct platform_device *pdev)
{
    struct msm_fb_data_type *mfd;
    int ret = 0;
    
    mfd = platform_get_drvdata(pdev);
    if (!mfd) {
        return -ENODEV;
    }
    if (mfd->key != MFD_KEY) {
        return -EINVAL;
    }
    return ret;
}

static int mipi_sharp_video_lcd_off(struct platform_device *pdev)
{
    struct msm_fb_data_type *mfd;
    int ret = 0;
    
    mfd = platform_get_drvdata(pdev);
    if (!mfd) {
        return -ENODEV;
    }
    if (mfd->key != MFD_KEY) {
        return -EINVAL;
    }
    shdisp_api_main_lcd_power_off();
    return ret;
}

static void mipi_sharp_video_lcd_set_backlight(struct msm_fb_data_type *mfd)
{
	struct shdisp_main_bkl_ctl bkl;

	if (mfd->bl_level == 0) {
		shdisp_api_main_bkl_off();
	}else{
		bkl.mode = SHDISP_MAIN_BKL_MODE_FIX;
		bkl.param = mfd->bl_level;
		shdisp_api_main_bkl_on(&bkl);
	}
}

static int __devinit mipi_sharp_video_lcd_probe(struct platform_device *pdev)
{
	struct mipi_dsi_panel_platform_data *platform_data;
	struct msm_fb_panel_data            *pdata;
	struct msm_panel_info               *pinfo;

	static u32 width, height;

	if (pdev->id == 0) {
		platform_data = pdev->dev.platform_data;

		if (platform_data) {
			width  = (platform_data->width_in_mm);
			height = (platform_data->height_in_mm);
		}

		return 0;
	}

	pdata = (struct msm_fb_panel_data *)pdev->dev.platform_data;
	pinfo = &(pdata->panel_info);

	pinfo->width_in_mm  = width;
	pinfo->height_in_mm = height;

	msm_fb_add_device(pdev);

	return 0;
}

static struct platform_driver this_driver = {
	.probe  = mipi_sharp_video_lcd_probe,
	.driver = {
		.name   = "mipi_sharp_video",
	},
};

static struct msm_fb_panel_data sharp_video_panel_data = {
	.on		= mipi_sharp_video_lcd_on,
	.off	= mipi_sharp_video_lcd_off,
	.set_backlight = mipi_sharp_video_lcd_set_backlight,
};

int mipi_sharp_video_device_register(struct msm_panel_info *pinfo,
					u32 channel, u32 panel)
{
	struct platform_device *pdev = NULL;
	int ret;

	
	if ((channel >= 3) || ch_used[channel])
		return -ENODEV;

	
	ch_used[channel] = TRUE;

	
	pdev = platform_device_alloc("mipi_sharp_video", (panel << 8)|channel);
	if (!pdev)
		return -ENOMEM;

	sharp_video_panel_data.panel_info = *pinfo;

	
	ret = platform_device_add_data(pdev, &sharp_video_panel_data,
		sizeof(sharp_video_panel_data));
	if (ret) {
		printk(KERN_ERR
		  "%s: platform_device_add_data failed!\n", __func__);
		goto err_device_put;
	}

	ret = platform_device_add(pdev);
	if (ret) {
		printk(KERN_ERR
		  "%s: platform_device_register failed!\n", __func__);
		goto err_device_put;
	}

	return 0;

err_device_put:
	platform_device_put(pdev);
	return ret;
}

static int __init mipi_sharp_video_lcd_init(void)
{
    int ret;
    
    ret = platform_driver_register(&this_driver);
    if( ret < 0 )
    {
        goto mipi_init_err_1;
    }
    return 0;

mipi_init_err_1:
    return -1;
}
module_init(mipi_sharp_video_lcd_init);

static void mipi_sharp_video_lcd_exit(void)
{
    platform_driver_unregister(&this_driver);
    return;
}
module_exit(mipi_sharp_video_lcd_exit);

