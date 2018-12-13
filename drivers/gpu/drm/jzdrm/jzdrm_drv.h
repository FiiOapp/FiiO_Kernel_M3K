/*
 * Copyright (c) 2016 Ingenic Semiconductor Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __JZDRM_DRV_H__
#define __JZDRM_DRV_H__

#include <linux/clk.h>
#include <linux/cpufreq.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/list.h>

#include <drm/drmP.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_fb_cma_helper.h>
#include <mach/jzfb.h>

/* Defaulting to maximum capability of JZDRM */
#define JZDRM_DEFAULT_MAX_PIXELCLOCK	200000
#define JZDRM_DEFAULT_MAX_WIDTH	2048
#define JZDRM_DEFAULT_MAX_BANDWIDTH	(1920*1080*60)


struct jzdrm_drm_private {
	void __iomem *mmio;

	struct clk *pwc_lcd;    /* display dpll */
	struct clk *disp_clk;    /* display dpll */
	struct clk *clk;         /* functional clock */
	int  is_clk_en;
	int rev;                /* IP revision */

	/* don't attempt resolutions w/ higher W * H * Hz: */
	uint32_t max_bandwidth;
	/*
	 * Pixel Clock will be restricted to some value as
	 * defined in the device datasheet measured in KHz
	 */
	uint32_t max_pixelclock;
	/*
	 * Max allowable width is limited on a per device basis
	 * measured in pixels
	 */
	uint32_t max_width;

	struct workqueue_struct *wq;

	struct drm_fbdev_cma *fbdev;

	struct drm_crtc *crtc;

	unsigned int num_encoders;
	struct drm_encoder *encoders[8];

	unsigned int num_connectors;
	struct drm_connector *connectors[8];
};

struct drm_crtc *jzdrm_crtc_create(struct drm_device *dev);
void jzdrm_crtc_cancel_page_flip(struct drm_crtc *crtc,
				  struct drm_file *file);
irqreturn_t jzdrm_crtc_irq(struct drm_crtc *crtc);
void jzdrm_crtc_update_clk(struct drm_crtc *crtc);
int jzdrm_crtc_mode_valid(struct drm_crtc *crtc,
			   struct drm_display_mode *mode);
int jzdrm_crtc_max_width(struct drm_crtc *crtc);
extern struct jzfb_platform_data jzdrm_pdata;
#endif /* __jzdrm_DRV_H__ */
