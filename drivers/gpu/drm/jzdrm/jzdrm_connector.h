/*
 * Copyright (c) 2016 Ingenic Semiconductor Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#ifndef __JZDRM_CONNECTOR_H__
#define __JZDRM_CONNECTOR_H___
struct lcd_manager {
	struct drm_encoder_funcs *jzdrm_encoder_funcs;

	struct drm_encoder_helper_funcs *jzdrm_encoder_helper_funcs;

	struct drm_connector_funcs *jzdrm_connector_funcs;

	struct drm_connector_helper_funcs *jzdrm_connector_helper_funcs;

	int dpms;
};

struct lcd_link {
	struct drm_connector connector;
	struct drm_encoder encoder;
	struct drm_device *drm;

	struct drm_display_mode previous_mode;

	struct lcd_manager *subdev;

	struct regmap *regmap;

	unsigned long pixel_clk_rate;
	unsigned int sample_rate;

	int dpms;

#ifdef CONFIG_JZ_MIPI_DSI_MODE2
	struct dsi_device *dsi;
#endif
};

struct lcd_link *jzdrm_lcd_register(struct drm_device *dev);

#ifdef CONFIG_LCD_X163
struct lcd_manager *mipi_dsi_lcd_register(struct lcd_link *lcd_link);
#endif

#ifdef CONFIG_LCD_BYD_8991FTGF
struct lcd_manager *rgb_lcd_register(struct lcd_link *lcd_link);
#endif

extern void get_previous_mode(struct drm_display_mode *mode,
	                      struct lcd_link *lcd_link);
#endif /* __DWC_HDMI_H__ */
