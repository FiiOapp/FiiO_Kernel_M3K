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

#include "../jzdrm_connector.h"

#ifdef CONFIG_JZ_MIPI_DSI_MODE2
#include "jz_mipi_dsih_hal.h"
#include "jz_mipi_dsi_regs.h"
#include "jz_mipi_dsi_lowlevel.h"
#include <mach/jz_dsim.h>
extern struct dsi_device * jzdsi_init(struct jzdsi_data *pdata);
extern void jzdsi_remove(struct dsi_device *dsi);
extern void dump_dsi_reg(struct dsi_device *dsi);
int jz_mipi_dsi_set_client(struct dsi_device *dsi, int power);
#endif

#define print_dbg(f, arg...) printk(KERN_INFO __FILE__ "-- %s:%d-- " f "\n", __func__, __LINE__, ## arg)

static void jzdrm_connector_power(struct drm_connector *connector, int mode)
{
	struct lcd_link *lcd_link = container_of(connector, struct lcd_link,
					     connector);
	if (mode != DRM_MODE_DPMS_ON)
		mode = DRM_MODE_DPMS_OFF;

	if (lcd_link->dpms == mode)
		return;

	lcd_link->dpms = mode;

	if(mode == DRM_MODE_DPMS_ON) {
#ifdef CONFIG_JZ_MIPI_DSI_MODE2
		lcd_link->dsi->master_ops->set_blank_mode(lcd_link->dsi,
				                          FB_BLANK_UNBLANK);
#endif
#ifdef CONFIG_JZ_MIPI_DSI_MODE2
	if (lcd_link->dsi->master_ops->ioctl)
	lcd_link->dsi->master_ops->ioctl(lcd_link->dsi,
			                 CMD_MIPI_DISPLAY_ON);
#endif
	}else{
#ifdef CONFIG_JZ_MIPI_DSI_MODE2
		lcd_link->dsi->master_ops->set_blank_mode(lcd_link->dsi,
				                          FB_BLANK_POWERDOWN);
#endif
	}
}

static void jzdrm_connector_dpms(struct drm_connector *connector, int mode)
{
	jzdrm_connector_power(connector, mode);
	drm_helper_connector_dpms(connector, mode);
}
static enum drm_connector_status jzdrm_connector_detect(struct drm_connector
							*connector, bool force)
{
	/* FIXME */
	return connector_status_connected;
}

static void jzdrm_connector_destroy(struct drm_connector *connector)
{
	struct lcd_link *lcd_link = container_of(connector, struct lcd_link,
					     connector);
	drm_sysfs_connector_remove(connector);
	drm_connector_cleanup(connector);

#ifdef CONFIG_JZ_MIPI_DSI_MODE2
	jzdsi_remove(lcd_link->dsi);
#endif

	kfree(lcd_link);
}

static int jzdrm_connector_get_modes(struct drm_connector *connector)
{
	struct lcd_link *lcd_link = container_of(connector, struct lcd_link,
					     connector);
	struct drm_display_mode *mode = drm_mode_create(connector->dev);
	int n;

	get_previous_mode(mode, lcd_link);

	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_set_name(mode);
	drm_mode_probed_add(connector, mode);

	n = 1;

	return n;
}

static int jzdrm_connector_mode_valid(struct drm_connector *connector,
			  struct drm_display_mode *mode)
{
	return MODE_OK;
}

static struct drm_encoder *jzdrm_connector_best_encoder(struct drm_connector
							   *connector)
{
	struct lcd_link *lcd_link = container_of(connector, struct lcd_link,
					     connector);

	return &lcd_link->encoder;
}

static void jzdrm_encoder_mode_set(struct drm_encoder *encoder,
			struct drm_display_mode *mode,
			struct drm_display_mode *adjusted_mode)
{
}

static bool jzdrm_encoder_mode_fixup(struct drm_encoder *encoder,
			const struct drm_display_mode *mode,
			struct drm_display_mode *adjusted_mode)
{
	return true;
}

static void jzdrm_encoder_disable(struct drm_encoder *encoder)
{
}

static void jzdrm_encoder_dpms(struct drm_encoder *encoder, int mode)
{
}

static void jzdrm_encoder_prepare(struct drm_encoder *encoder)
{
}

static void jzdrm_encoder_commit(struct drm_encoder *encoder)
{
}

static void jzdrm_encoder_destroy(struct drm_encoder *encoder)
{
	return;
}

static struct drm_encoder_funcs jzdrm_encoder_funcs = {
	.destroy = jzdrm_encoder_destroy,
};

static struct drm_encoder_helper_funcs jzdrm_encoder_helper_funcs = {
	.dpms = jzdrm_encoder_dpms,
	.prepare = jzdrm_encoder_prepare,
	.commit = jzdrm_encoder_commit,
	.mode_set = jzdrm_encoder_mode_set,
	.mode_fixup = jzdrm_encoder_mode_fixup,
	.disable = jzdrm_encoder_disable,
};

static struct drm_connector_funcs jzdrm_connector_funcs = {
	.dpms = jzdrm_connector_dpms,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.detect = jzdrm_connector_detect,
	.destroy = jzdrm_connector_destroy,
};

static struct drm_connector_helper_funcs jzdrm_connector_helper_funcs = {
	.get_modes = jzdrm_connector_get_modes,
	.mode_valid = jzdrm_connector_mode_valid,
	.best_encoder = jzdrm_connector_best_encoder,
};

struct lcd_manager *mipi_dsi_lcd_register(struct lcd_link *lcd_link)
{
	struct lcd_manager *subdev;
	struct drm_device *dev = lcd_link->drm;
	struct platform_device *pdev = dev->platformdev;
	struct jzfb_platform_data *pdata = pdev->dev.platform_data;

	subdev = kzalloc(sizeof(struct lcd_manager), GFP_KERNEL);
	if (!lcd_link)
		return NULL;

	lcd_link->subdev = subdev;
	subdev->jzdrm_connector_funcs = &jzdrm_connector_funcs;
	subdev->jzdrm_connector_helper_funcs = &jzdrm_connector_helper_funcs;
	subdev->jzdrm_encoder_funcs = &jzdrm_encoder_funcs;
	subdev->jzdrm_encoder_helper_funcs = &jzdrm_encoder_helper_funcs;
	subdev->dpms = DRM_MODE_DPMS_OFF;

#ifdef CONFIG_JZ_MIPI_DSI_MODE2
	lcd_link->dsi = jzdsi_init(pdata->dsi_pdata);
	if (!lcd_link->dsi) {
		dev_err(dev->dev,
			"get mipi dsi device fialed\n");
	}
#endif

	return subdev;
}
EXPORT_SYMBOL_GPL(mipi_dsi_lcd_register);
