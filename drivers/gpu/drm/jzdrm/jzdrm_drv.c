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

#include "jzdrm_drv.h"
#include "regs.h"

#include "drm_fb_helper.h"
#include "jzdrm_connector.h"
#include "drm_flip_work.h"

#define print_dbg(f, arg...) printk(KERN_INFO __FILE__ "-- %s: " f "\n",__func__, ## arg)

static struct of_device_id jzdrm_of_match[];

static struct drm_framebuffer *jzdrm_fb_create(struct drm_device *dev,
		struct drm_file *file_priv, struct drm_mode_fb_cmd2 *mode_cmd)
{
	return drm_fb_cma_create(dev, file_priv, mode_cmd);
}

static void jzdrm_fb_output_poll_changed(struct drm_device *dev)
{
	struct jzdrm_drm_private *priv = dev->dev_private;

	if (priv->fbdev)
		drm_fbdev_cma_hotplug_event(priv->fbdev);
}

static const struct drm_mode_config_funcs mode_config_funcs = {
	.fb_create = jzdrm_fb_create,
	.output_poll_changed = jzdrm_fb_output_poll_changed,
};

void modeset_init(struct drm_device *dev)
{
	struct jzdrm_drm_private *priv = dev->dev_private;

	priv->crtc = jzdrm_crtc_create(dev);

	dev->mode_config.min_width = 0;
	dev->mode_config.min_height = 0;
	dev->mode_config.max_width = 2048;
	dev->mode_config.max_height = 2048;
	dev->mode_config.funcs = &mode_config_funcs;
}

/*
 * DRM operations:
 */

static int jzdrm_unload(struct drm_device *dev)
{
	struct jzdrm_drm_private *priv = dev->dev_private;

	drm_kms_helper_poll_fini(dev);
	drm_mode_config_cleanup(dev);
	drm_vblank_cleanup(dev);

	pm_runtime_get_sync(dev->dev);
	drm_irq_uninstall(dev);
	pm_runtime_put_sync(dev->dev);

	if (priv->disp_clk)
		clk_put(priv->disp_clk);

	if (priv->clk)
		clk_put(priv->clk);

	if (priv->pwc_lcd)
		clk_put(priv->pwc_lcd);

	if (priv->mmio)
		iounmap(priv->mmio);

	flush_workqueue(priv->wq);
	destroy_workqueue(priv->wq);

	dev->dev_private = NULL;

	pm_runtime_disable(dev->dev);

	kfree(priv);

	return 0;
}

static int jzdrm_load(struct drm_device *dev, unsigned long flags)
{
	struct platform_device *pdev = dev->platformdev;
	struct jzdrm_drm_private *priv;
	struct resource *res;
	struct lcd_link *lcd_link;
	int ret;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		DRM_DEBUG_DRIVER("failed to allocate private data\n");
		return -ENOMEM;
	}

	dev->dev_private = priv;

	priv->wq = alloc_ordered_workqueue("jzdrm", 0);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		DRM_DEBUG_DRIVER("failed to get memory resource\n");
		ret = -EINVAL;
		goto fail;
	}

	priv->mmio = ioremap_nocache(res->start, resource_size(res));
	if (!priv->mmio) {
		DRM_DEBUG_DRIVER("failed to ioremap\n");
		ret = -ENOMEM;
		goto fail;
	}

	priv->pwc_lcd = clk_get(dev->dev, "pwc_lcd");
	if (IS_ERR(priv->pwc_lcd)) {
		DRM_DEBUG_DRIVER("failed to get pwc_lcd clock\n");
		ret = -ENODEV;
		goto fail;
	}

	clk_prepare_enable(priv->pwc_lcd);

	priv->clk = clk_get(dev->dev, "lcd");
	if (IS_ERR(priv->clk)) {
		DRM_DEBUG_DRIVER("failed to get lcd clock\n");
		ret = -ENODEV;
		goto fail;
	}

	clk_prepare_enable(priv->clk);

	priv->disp_clk = clk_get(dev->dev, "cgu_lpc");
	if (IS_ERR(priv->disp_clk)) {
		DRM_DEBUG_DRIVER("failed to get pixel clock\n");
		ret = -ENODEV;
		goto fail;
	}

	clk_prepare_enable(priv->disp_clk);
	priv->is_clk_en = 1;

	priv->max_bandwidth = JZDRM_DEFAULT_MAX_BANDWIDTH;
	priv->max_width = JZDRM_DEFAULT_MAX_WIDTH;
	priv->max_pixelclock = JZDRM_DEFAULT_MAX_PIXELCLOCK;

	drm_mode_config_init(dev);

	lcd_link = jzdrm_lcd_register(dev);
	if(!lcd_link){
		DRM_DEBUG_DRIVER("failed to register lcd device\n");
		ret = -ENODEV;
		goto fail;
	}

	priv->encoders[priv->num_encoders++] = &lcd_link->encoder;
	priv->connectors[priv->num_connectors++] = &lcd_link->connector;

	modeset_init(dev);

	ret = drm_vblank_init(dev, 1);
	if (ret < 0) {
		DRM_DEBUG_DRIVER("failed to initialize vblank\n");
		goto fail;
	}

	pm_runtime_get_sync(dev->dev);
	ret = drm_irq_install(dev);
	pm_runtime_put_sync(dev->dev);
	if (ret < 0) {
		DRM_DEBUG_DRIVER("failed to install IRQ handler\n");
		goto fail;
	}

	platform_set_drvdata(pdev, dev);

	priv->fbdev = drm_fbdev_cma_init(dev, 32,
			dev->mode_config.num_crtc,
			dev->mode_config.num_connector);

	drm_kms_helper_poll_init(dev);

	return 0;

fail:
	jzdrm_unload(dev);
	return ret;
}

static void jzdrm_preclose(struct drm_device *dev, struct drm_file *file)
{
	struct jzdrm_drm_private *priv = dev->dev_private;

	jzdrm_crtc_cancel_page_flip(priv->crtc, file);
}

static void jzdrm_lastclose(struct drm_device *dev)
{
	struct jzdrm_drm_private *priv = dev->dev_private;

	drm_fbdev_cma_restore_mode(priv->fbdev);
}

static irqreturn_t jzdrm_irq(int irq, void *arg)
{
	struct drm_device *dev = arg;
	struct jzdrm_drm_private *priv = dev->dev_private;

	return jzdrm_crtc_irq(priv->crtc);
}

static void enable_vblank(struct drm_device *dev, bool enable)
{
	u32 tmp;

	/* clear previous EOF flag */
	tmp = jzdrm_read(dev, LCDC_STATE);
	jzdrm_write(dev, LCDC_STATE, tmp & ~LCDC_STATE_EOF);

	/* enable end of frame interrupt */
	tmp = jzdrm_read(dev, LCDC_CTRL);
	if (enable)
		jzdrm_write(dev, LCDC_CTRL, tmp | LCDC_CTRL_EOFM);
	else
		jzdrm_write(dev, LCDC_CTRL, tmp & ~LCDC_CTRL_EOFM);

}

static int jzdrm_enable_vblank(struct drm_device *dev, int crtc)
{
	enable_vblank(dev, true);
	return 0;
}

static void jzdrm_disable_vblank(struct drm_device *dev, int crtc)
{
	enable_vblank(dev, false);
}

static struct dma_buf *jzdrm_drm_gem_prime_export(struct drm_device *dev,
                        struct drm_gem_object *obj,
                        int flags)
{
	/* Read/write access required */
	flags |= O_RDWR;
	return drm_gem_prime_export(dev, obj, flags);
}

static const struct file_operations fops = {
	.owner              = THIS_MODULE,
	.open               = drm_open,
	.release            = drm_release,
	.unlocked_ioctl     = drm_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl       = drm_compat_ioctl,
#endif
	.poll               = drm_poll,
	.read               = drm_read,
	.llseek             = no_llseek,
	.mmap               = drm_gem_cma_mmap,
};

static struct drm_driver jzdrm_driver = {
	.driver_features    = DRIVER_HAVE_IRQ | DRIVER_GEM | DRIVER_MODESET |
				DRIVER_PRIME,
	.load               = jzdrm_load,
	.unload             = jzdrm_unload,
	.preclose           = jzdrm_preclose,
	.lastclose          = jzdrm_lastclose,
	.irq_handler        = jzdrm_irq,
	.get_vblank_counter = drm_vblank_count,
	.enable_vblank      = jzdrm_enable_vblank,
	.disable_vblank     = jzdrm_disable_vblank,
	.gem_free_object    = drm_gem_cma_free_object,
	.gem_vm_ops         = &drm_gem_cma_vm_ops,
	.dumb_create        = drm_gem_cma_dumb_create,
	.dumb_map_offset    = drm_gem_cma_dumb_map_offset,
	.dumb_destroy       = drm_gem_cma_dumb_destroy,
	.prime_handle_to_fd = drm_gem_prime_handle_to_fd,
	.prime_fd_to_handle = drm_gem_prime_fd_to_handle,
	.gem_prime_import   = drm_gem_prime_import,
	.gem_prime_export   = jzdrm_drm_gem_prime_export,
//	.gem_prime_get_sg_table = drm_gem_cma_prime_get_sg_table,
//	.gem_prime_import_sg_table = drm_gem_cma_prime_import_sg_table,
//	.gem_prime_vmap     = drm_gem_cma_prime_vmap,
//	.gem_prime_vunmap   = drm_gem_cma_prime_vunmap,
//	.gem_prime_mmap     = drm_gem_cma_prime_mmap,
	.fops               = &fops,
	.name               = "jzdrm",
	.desc               = "Ingenic LCD Controller DRM",
	.date               = "20140623",
	.major              = 1,
	.minor              = 0,
};

/*
 * Platform driver:
 */
static int jzdrm_pdev_probe(struct platform_device *pdev)
{
	struct jzfb_platform_data *pdata = &jzdrm_pdata;

	if (!pdata) {
		DRM_DEBUG_DRIVER("Missing platform data");
		return -ENXIO;
	}

	pdev->dev.platform_data = pdata;

	return drm_platform_init(&jzdrm_driver, pdev);
}

static int jzdrm_pdev_remove(struct platform_device *pdev)
{
	drm_put_dev(platform_get_drvdata(pdev));
	return 0;
}

static struct platform_driver jzdrm_platform_driver = {
	.probe      = jzdrm_pdev_probe,
	.remove     = jzdrm_pdev_remove,
	.driver     = {
		.owner  = THIS_MODULE,
		.name   = "jz-fb",
	},
};

static int __init jzdrm_drm_init(void)
{
	DRM_DEBUG_DRIVER("init");
	return platform_driver_register(&jzdrm_platform_driver);
}

static void __exit jzdrm_drm_fini(void)
{
	DRM_DEBUG_DRIVER("fini");
	platform_driver_unregister(&jzdrm_platform_driver);
}
module_init(jzdrm_drm_init);
module_exit(jzdrm_drm_fini);

MODULE_AUTHOR("Zubair Lutfullah Kakakhel <Zubair.Kakakhel@imgtec.com>");
MODULE_DESCRIPTION("Ingenic JZDRM LCD/HDMI Driver");
MODULE_LICENSE("GPL");
