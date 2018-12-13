/*
 * linux/drivers/leds-pwm.c
 *
 * simple PWM based LED control
 *
 * Copyright 2009 Luotao Fu @ Pengutronix (l.fu@pengutronix.de)
 *
 * based on leds-gpio.c by Raphael Assenat <raph@8d.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>
#include <linux/fb.h>
#include <linux/leds.h>
#include <linux/err.h>
#include <linux/pwm.h>
#include <linux/leds_pwm.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <mach/jz_pwm_dev.h>

struct jz_pwm_dev_data {
	struct jz_pwm_classdev	cdev;
	struct pwm_device	*pwm;
	struct work_struct	work;
	unsigned int		active_low;
	unsigned int		period;
	int			duty;
	bool			can_sleep;
};

struct jz_pwm_dev_priv {
	int num_devs;
	struct jz_pwm_dev_data devs[0];
};

static void __jz_pwm_dev_set(struct jz_pwm_dev_data *dev_dat)
{
	int new_duty = dev_dat->duty;
	pwm_disable(dev_dat->pwm);
	pr_info("new_duty is %d ,period is %d ns\n",dev_dat->duty,dev_dat->period);
	pwm_config(dev_dat->pwm, new_duty,dev_dat->period);
	if (new_duty == 0)
		pwm_disable(dev_dat->pwm);
	else
		pwm_enable(dev_dat->pwm);
}

static void jz_pwm_dev_enable(struct jz_pwm_classdev *cdev)
{
	struct jz_pwm_dev_data *dev_dat =
		container_of(cdev, struct jz_pwm_dev_data, cdev);
	if(cdev->enable)
		pwm_enable(dev_dat->pwm);
	else
		pwm_disable(dev_dat->pwm);
}

static void jz_pwm_dev_work(struct work_struct *work)
{
	struct jz_pwm_dev_data *dev_dat =
		container_of(work, struct jz_pwm_dev_data, work);
	__jz_pwm_dev_set(dev_dat);
}

static void jz_pwm_dev_get(struct jz_pwm_classdev *cdev)
{
	struct jz_pwm_dev_data *dev_dat =
		container_of(cdev, struct jz_pwm_dev_data, cdev);
	unsigned int max = dev_dat->cdev.max_duty_ratio;
	unsigned int period = dev_dat->period;
	cdev->period_ns = period;
	pr_info("period is %d ns,duty_ratio is %d ns,duty is %d ns\n",cdev->period_ns,cdev->duty_ratio,dev_dat->duty);
}
static void jz_pwm_dev_set(struct jz_pwm_classdev *cdev)
{
	struct jz_pwm_dev_data *dev_dat =
		container_of(cdev, struct jz_pwm_dev_data, cdev);
	unsigned int max = dev_dat->cdev.max_duty_ratio;
	unsigned int period = cdev->period_ns;
	dev_dat->duty = cdev->duty_ratio * period / max;
	dev_dat->period = cdev->period_ns;
	if (dev_dat->can_sleep){
		schedule_work(&dev_dat->work);
	}else{
		__jz_pwm_dev_set(dev_dat);
	}
}

static inline size_t sizeof_jz_pwm_dev_priv(int num_devs)
{
	return sizeof(struct jz_pwm_dev_priv) +
		      (sizeof(struct jz_pwm_dev_data) * num_devs);
}

static struct jz_pwm_dev_priv *jz_pwm_dev_create_of(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct device_node *child;
	struct jz_pwm_dev_priv *priv;
	int count, ret;

	/* count LEDs in this device, so we know how much to allocate */
	count = of_get_child_count(node);
	if (!count)
		return NULL;

	priv = devm_kzalloc(&pdev->dev, sizeof_jz_pwm_dev_priv(count),
			    GFP_KERNEL);
	if (!priv)
		return NULL;

	for_each_child_of_node(node, child) {
		struct jz_pwm_dev_data *dev_dat = &priv->devs[priv->num_devs];

		dev_dat->cdev.name = of_get_property(child, "label",
						     NULL) ? : child->name;

		dev_dat->pwm = devm_of_pwm_get(&pdev->dev, child, NULL);
		if (IS_ERR(dev_dat->pwm)) {
			dev_err(&pdev->dev, "unable to request PWM for %s\n",
				dev_dat->cdev.name);
			goto err;
		}
		/* Get the period from PWM core when n*/
		dev_dat->period = pwm_get_period(dev_dat->pwm);
		dev_dat->cdev.pwm_set = jz_pwm_dev_set;
		dev_dat->cdev.pwm_get = jz_pwm_dev_get;
		dev_dat->cdev.pwm_enable = jz_pwm_dev_enable;
		dev_dat->cdev.duty_ratio = 100;//DEV_OFF;
		dev_dat->cdev.enable = 1;
		dev_dat->cdev.flags |= JZ_PWM_DEV_CORE_SUSPENDRESUME;
		dev_dat->can_sleep = pwm_can_sleep(dev_dat->pwm);
		if (dev_dat->can_sleep)
			INIT_WORK(&dev_dat->work, jz_pwm_dev_work);

		ret = jz_pwm_classdev_register(&pdev->dev, &dev_dat->cdev);
		if (ret < 0) {
			dev_err(&pdev->dev, "failed to register for %s\n",
				dev_dat->cdev.name);
			of_node_put(child);
			goto err;
		}
		priv->num_devs++;
	}

	return priv;
err:
	while (priv->num_devs--)
		jz_pwm_classdev_unregister(&priv->devs[priv->num_devs].cdev);

	return NULL;
}

static int jz_pwm_dev_probe(struct platform_device *pdev)
{
	struct jz_pwm_dev_platform_data *pdata = pdev->dev.platform_data;
	struct jz_pwm_dev_priv *priv;
	int i, ret = 0;
	if (pdata && pdata->num_devs) {
		priv = devm_kzalloc(&pdev->dev,
				    sizeof_jz_pwm_dev_priv(pdata->num_devs),
				    GFP_KERNEL);
		if (!priv)
			return -ENOMEM;

		for (i = 0; i < pdata->num_devs; i++) {
			struct jz_pwm_dev *cur_pwm = &pdata->devs[i];
			struct jz_pwm_dev_data *dev_dat = &priv->devs[i];

			dev_dat->pwm = devm_pwm_get(&pdev->dev, cur_pwm->name);
			if (IS_ERR(dev_dat->pwm)) {
				dev_dat->pwm = pwm_request(pdata->devs[i].pwm_id,"pwm-dev");
			}

			if (IS_ERR(dev_dat->pwm)) {
				ret = PTR_ERR(dev_dat->pwm);
				dev_err(&pdev->dev,
					"unable to request PWM for %s\n",
					cur_pwm->name);
				goto err;
			}

			dev_dat->cdev.name = cur_pwm->name;
			dev_dat->active_low = cur_pwm->active_low;
			dev_dat->period = cur_pwm->period_ns;
			dev_dat->cdev.pwm_set = jz_pwm_dev_set;
			dev_dat->cdev.pwm_get = jz_pwm_dev_get;
			dev_dat->cdev.pwm_enable = jz_pwm_dev_enable;
			dev_dat->cdev.max_duty_ratio = cur_pwm->max_duty_ratio;
			dev_dat->cdev.duty_ratio = 100;//DEV_OFF;
			dev_dat->cdev.enable = 1;
			dev_dat->cdev.flags |= LED_CORE_SUSPENDRESUME;
			dev_dat->can_sleep = pwm_can_sleep(dev_dat->pwm);
			if (dev_dat->can_sleep)
				INIT_WORK(&dev_dat->work, jz_pwm_dev_work);
			ret = jz_pwm_classdev_register(&pdev->dev, &dev_dat->cdev);
			if (ret < 0)
				goto err;
		}
		priv->num_devs = pdata->num_devs;
	} else {
		priv = jz_pwm_dev_create_of(pdev);
		if (!priv)
			return -ENODEV;
	}

	platform_set_drvdata(pdev, priv);
	return 0;

err:
	while (i--)
		jz_pwm_classdev_unregister(&priv->devs[i].cdev);

	return ret;
}

static int jz_pwm_dev_remove(struct platform_device *pdev)
{
	struct jz_pwm_dev_priv *priv = platform_get_drvdata(pdev);
	int i;

	for (i = 0; i < priv->num_devs; i++) {
		jz_pwm_classdev_unregister(&priv->devs[i].cdev);
		if (priv->devs[i].can_sleep)
			cancel_work_sync(&priv->devs[i].work);
	}

	return 0;
}

static const struct of_device_id of_jz_pwm_dev_match[] = {
	{ .compatible = "pwm-devs", },
	{},
};
MODULE_DEVICE_TABLE(of, of_jz_pwm_dev_match);

static struct platform_driver jz_pwm_dev_driver = {
	.probe		= jz_pwm_dev_probe,
	.remove		= jz_pwm_dev_remove,
	.driver		= {
		.name	= "jz_pwm_dev",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(of_jz_pwm_dev_match),
	},
};

module_platform_driver(jz_pwm_dev_driver);

MODULE_AUTHOR("jxsun <jingxin.sun@ingenic.com>");
MODULE_DESCRIPTION("PWM Device Driver,such as LED,Beeper");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:jz-pwm-dev");
