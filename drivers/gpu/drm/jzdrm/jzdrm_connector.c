/*
 * Copyright (c) 2016 Ingenic Semiconductor Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#include <linux/err.h>
#include <linux/clk.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>
#include <linux/of_device.h>
#include <mach/jzfb.h>

#include <drm/drmP.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_edid.h>
#include <drm/drm_encoder_slave.h>

#include "jzdrm_connector.h"

#define print_dbg(f, arg...) printk(KERN_INFO __FILE__ "-- %s:%d-- " f "\n", __func__, __LINE__, ## arg)

void get_previous_mode(struct drm_display_mode *mode,
			      struct lcd_link *lcd_link)
{
	struct drm_device *dev = lcd_link->drm;
	struct platform_device *pdev = dev->platformdev;
	struct jzfb_platform_data *pdata = pdev->dev.platform_data;
	struct fb_videomode *video_modes = pdata->modes;

	mode->clock = PICOS2KHZ(video_modes->pixclock);

	mode->hdisplay = video_modes->xres;
	mode->hsync_start = mode->hdisplay + video_modes->left_margin;
	mode->hsync_end = mode->hsync_start + video_modes->hsync_len;
	mode->htotal = mode->hsync_end + video_modes->right_margin;

	mode->vdisplay = video_modes->yres;
	mode->vsync_start = mode->vdisplay + video_modes->upper_margin;
	mode->vsync_end = mode->vsync_start + video_modes->vsync_len;
	mode->vtotal = mode->vsync_end + video_modes->lower_margin;

	mode->flags = 0;

	if (video_modes->vmode & FB_VMODE_INTERLACED)
		mode->flags |= DRM_MODE_FLAG_INTERLACE;
	if (video_modes->sync& FB_SYNC_HOR_HIGH_ACT)
		mode->flags |= DRM_MODE_FLAG_PHSYNC;
	else
		mode->flags |= DRM_MODE_FLAG_NHSYNC;

	if (video_modes->sync & FB_SYNC_VERT_HIGH_ACT)
		mode->flags |= DRM_MODE_FLAG_PVSYNC;
	else
		mode->flags |= DRM_MODE_FLAG_NVSYNC;
}

struct lcd_link *jzdrm_lcd_register(struct drm_device *dev)
{
	struct platform_device *pdev = dev->platformdev;
	struct jzfb_platform_data *pdata = pdev->dev.platform_data;
	struct lcd_link *lcd_link;
	struct drm_encoder *encoder;
	struct drm_connector *connector;
	struct lcd_manager *subdev;
	int ret;

	lcd_link = kzalloc(sizeof(struct lcd_link), GFP_KERNEL);
	if (!lcd_link)
		return NULL;

	lcd_link->drm = dev;
	encoder = &lcd_link->encoder;
	connector = &lcd_link->connector;

#ifdef CONFIG_LCD_BYD_8991FTGF
	subdev = rgb_lcd_register(lcd_link);
#endif

#ifdef CONFIG_LCD_X163
	subdev = mipi_dsi_lcd_register(lcd_link);
#endif
	if (!subdev)
		return NULL;

	connector->funcs = subdev->jzdrm_connector_funcs;
	encoder->funcs = subdev->jzdrm_encoder_funcs;

	encoder->encoder_type = DRM_MODE_ENCODER_TMDS;
	connector->connector_type = DRM_MODE_CONNECTOR_DisplayPort;
	encoder->possible_crtcs = 1;

	drm_encoder_helper_add(encoder,
			      subdev->jzdrm_encoder_helper_funcs);

	drm_encoder_init(dev, encoder,
			encoder->funcs,
			encoder->encoder_type);

	drm_connector_helper_add(connector,
			subdev->jzdrm_connector_helper_funcs);

	drm_connector_init(dev, connector,
			connector->funcs,
			connector->connector_type);

	connector->encoder = encoder;
	connector->dpms = subdev->dpms;

	ret = drm_sysfs_connector_add(connector);
	if (ret) {
		dev_err(dev->dev,
			"[CONNECTOR:%d] drm_connector_register failed: %d\n",
			connector->base.id, ret);
		return NULL;
	}

	drm_mode_connector_attach_encoder(connector, encoder);

	lcd_link->dpms = subdev->dpms;

	return lcd_link;
}
EXPORT_SYMBOL_GPL(jzdrm_lcd_register);
