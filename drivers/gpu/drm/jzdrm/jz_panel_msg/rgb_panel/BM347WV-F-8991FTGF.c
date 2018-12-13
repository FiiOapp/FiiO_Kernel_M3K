/*
 * Copyright (c) 2016 Ingenic Semiconductor Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */
#include <linux/byd_8991.h>
#include <mach/jzfb.h>
#include "../../jzdrm_drv.h"

#ifdef CONFIG_LCD_BYD_8991FTGF
struct fb_videomode jzdrm_8991_videomode = {
	.name = "480x800",
	.refresh = 60,
	.xres = 480,
	.yres = 800,
	.pixclock = KHZ2PICOS(36000),
	.left_margin = 70,
	.right_margin = 70,
	.upper_margin = 2,
	.lower_margin = 2,
	.hsync_len = 42,
	.vsync_len = 11,
	.sync = ~FB_SYNC_HOR_HIGH_ACT & ~FB_SYNC_VERT_HIGH_ACT,
	.vmode = FB_VMODE_NONINTERLACED,
	.flag = 0,
};

struct jzfb_platform_data jzdrm_pdata = {
	.num_modes = 1,
	.modes = &jzdrm_8991_videomode,
	.lcd_type = LCD_TYPE_GENERIC_24_BIT,
	.bpp = 24,
	.width = 45,
	.height = 75,
	/*this pixclk_edge is for lcd_controller sending data, which is reverse to lcd*/
	.pixclk_falling_edge = 1,
	.data_enable_active_low = 0,
} ;
#endif
