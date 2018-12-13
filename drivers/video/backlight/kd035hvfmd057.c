/*
 *  LCD control code for KD035HVFMD057
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 */

#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/lcd.h>
#include <linux/spi/spi.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/regulator/consumer.h>
#include <linux/platform_device.h>

#include "../jz_fb_v13/regs.h"

static int uboot_inited;

struct kd035hvfmd057_data {
	int lcd_power;
	struct lcd_device *lcd;
	struct lcd_platform_data *ctrl;
	struct regulator *lcd_vcc_reg;
};

static int kd035hvfmd057_set_power(struct lcd_device *lcd, int power)
{
	struct kd035hvfmd057_data *dev= lcd_get_data(lcd);
	if (!power && dev->lcd_power) {
		if(!regulator_is_enabled(dev->lcd_vcc_reg))
			regulator_enable(dev->lcd_vcc_reg);
		dev->ctrl->power_on(lcd, 1);
	} else if (power && !dev->lcd_power) {
		if (dev->ctrl->reset) {
			dev->ctrl->reset(lcd);
		}
		dev->ctrl->power_on(lcd, 0);
		if(regulator_is_enabled(dev->lcd_vcc_reg))
			regulator_disable(dev->lcd_vcc_reg);
	}
	dev->lcd_power = power;
	return 0;
}

static int kd035hvfmd057_get_power(struct lcd_device *lcd)
{
	struct kd035hvfmd057_data *dev= lcd_get_data(lcd);

	return dev->lcd_power;
}

static int kd035hvfmd057_set_mode(struct lcd_device *lcd, struct fb_videomode *mode)
{
	return 0;
}

static struct lcd_ops kd035hvfmd057_ops = {
	.set_power = kd035hvfmd057_set_power,
	.get_power = kd035hvfmd057_get_power,
	.set_mode = kd035hvfmd057_set_mode,
};
static int lcd_inited_by_uboot( void )
{
	if (*(unsigned int*)(0xb3050000 + LCDC_CTRL) & LCDC_CTRL_ENA)
		        uboot_inited = 1;
	else
		        uboot_inited = 0;
		    /* screen init will set this function first */
	return uboot_inited;
}

static int kd035hvfmd057_probe(struct platform_device *pdev)
{
	int ret;
	struct kd035hvfmd057_data *dev;
	dev = kzalloc(sizeof(struct kd035hvfmd057_data), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	dev->ctrl = pdev->dev.platform_data;
	if (dev->ctrl == NULL) {
		dev_info(&pdev->dev, "no platform data!");
		return -EINVAL;
	}

	dev_set_drvdata(&pdev->dev, dev);

	dev->lcd = lcd_device_register("kd035hvfmd057_slcd", &pdev->dev,
				       dev, &kd035hvfmd057_ops);
	if (IS_ERR(dev->lcd)) {
		ret = PTR_ERR(dev->lcd);
		dev->lcd = NULL;
		dev_info(&pdev->dev, "lcd device register error: %d\n", ret);
	} else {
		dev_info(&pdev->dev, "lcd device(KD035HVFMD057) register success\n");
	}

        if (!lcd_inited_by_uboot())
                if (dev->ctrl->power_on)
                        dev->ctrl->power_on(dev->lcd, 1);
        dev->lcd_power = FB_BLANK_UNBLANK;

	dev->lcd_vcc_reg = regulator_get(NULL,"lcd_3v3");
	regulator_enable(dev->lcd_vcc_reg);
	return 0;
}

static int kd035hvfmd057_remove(struct platform_device *pdev)
{
	struct kd035hvfmd057_data *dev = dev_get_drvdata(&pdev->dev);

	if (dev->lcd_power)
		dev->ctrl->power_on(dev->lcd, 0);
	regulator_put(dev->lcd_vcc_reg);
	lcd_device_unregister(dev->lcd);
	dev_set_drvdata(&pdev->dev, NULL);
	kfree(dev);

	return 0;
}

#ifdef CONFIG_PM
static int kd035hvfmd057_suspend(struct platform_device *pdev,
		pm_message_t state)
{
	return 0;
}

static int kd035hvfmd057_resume(struct platform_device *pdev)
{
	return 0;
}
#else
#define kd035hvfmd057_suspend	NULL
#define kd035hvfmd057_resume	NULL
#endif

static struct platform_driver kd035hvfmd057_driver = {
	.driver		= {
		.name	= "kd035hvfmd057_slcd",
		.owner	= THIS_MODULE,
	},
	.probe		= kd035hvfmd057_probe,
	.remove		= kd035hvfmd057_remove,
	.suspend	= kd035hvfmd057_suspend,
	.resume		= kd035hvfmd057_resume,
};

static int __init kd035hvfmd057_init(void)
{
	return platform_driver_register(&kd035hvfmd057_driver);
}
module_init(kd035hvfmd057_init);

static void __exit kd035hvfmd057_exit(void)
{
	platform_driver_unregister(&kd035hvfmd057_driver);
}
module_exit(kd035hvfmd057_exit);

MODULE_DESCRIPTION("KD035HVFMD057 lcd panel driver");
MODULE_LICENSE("GPL");
