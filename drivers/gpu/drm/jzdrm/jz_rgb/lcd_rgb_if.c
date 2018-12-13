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
#include <mach/jzfb.h>

#include <drm/drmP.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_edid.h>
#include <drm/drm_encoder_slave.h>

#include "../jzdrm_connector.h"

#define print_dbg(f, arg...) printk(KERN_INFO __FILE__ "-- %s:%d-- " f "\n", __func__, __LINE__, ## arg)

static void jzdrm_connector_dpms(struct drm_connector *connector, int mode)
{
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

struct lcd_manager *rgb_lcd_register(struct lcd_link *lcd_link)
{
	struct lcd_manager *subdev;

	subdev = kzalloc(sizeof(struct lcd_manager), GFP_KERNEL);
	if (!lcd_link)
		return NULL;

	lcd_link->subdev = subdev;
	subdev->jzdrm_connector_funcs = &jzdrm_connector_funcs;
	subdev->jzdrm_connector_helper_funcs = &jzdrm_connector_helper_funcs;
	subdev->jzdrm_encoder_funcs = &jzdrm_encoder_funcs;
	subdev->jzdrm_encoder_helper_funcs = &jzdrm_encoder_helper_funcs;
	subdev->dpms = DRM_MODE_DPMS_OFF;

	return subdev;
}
EXPORT_SYMBOL_GPL(rgb_lcd_register);
