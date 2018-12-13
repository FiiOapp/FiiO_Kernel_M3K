/*
 *  Class Core
 *
 * Copyright (C) 2005 John Lenz <lenz@cs.wisc.edu>
 * Copyright (C) 2005-2007 Richard Purdie <rpurdie@openedhand.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/device.h>
#include <linux/timer.h>
#include <linux/err.h>
#include <linux/ctype.h>
#include <mach/jz_pwm_dev.h>
#include <../../arch/mips/xburst/soc-x1000/chip-x1000/halley2/halley2_v10/board.h>

DECLARE_RWSEM(jz_pwm_dev_list_lock);
EXPORT_SYMBOL_GPL(jz_pwm_dev_list_lock);

LIST_HEAD(jz_pwm_dev_list);
EXPORT_SYMBOL_GPL(jz_pwm_dev_list);

static struct class *jz_pwm_class;

static void jz_pwm_update(struct jz_pwm_classdev *cdev)
{
	if (cdev->pwm_get)
		cdev->pwm_get(cdev);
}

static void jz_pwm_set(struct jz_pwm_classdev *cdev,int value ,int flag)
{
	if(flag == SET_DUTY_RATIO){
		if (value > cdev->max_duty_ratio)
			value = cdev->max_duty_ratio;
		else if(value < 0)
			value = 0;
		cdev->duty_ratio = value;
	}else{
		if(value < PWM_PERIOD_MIN)
			value = PWM_PERIOD_MIN;
		else if(value > PWM_PERIOD_MAX)
			value = PWM_PERIOD_MAX;
		cdev->period_ns = value;
	}

	if(cdev->duty_ratio == 0)
		cdev->enable = 0;
	else
		cdev->enable = 1;

	if (!(cdev->flags & JZ_PWM_DEV_SUSPENDED))
		cdev->pwm_set(cdev,value,flag);
}

static ssize_t jz_pwm_dev_duty_ratio_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct jz_pwm_classdev *cdev = dev_get_drvdata(dev);
	/* no lock needed for this */
	jz_pwm_update(cdev);
	return sprintf(buf, "%u\n", cdev->duty_ratio);
}

static ssize_t jz_pwm_dev_duty_ratio_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct jz_pwm_classdev *cdev = dev_get_drvdata(dev);
	unsigned long state;
	ssize_t ret = -EINVAL;

	ret = kstrtoul(buf, 10, &state);
	pr_info("%s ,buf is %s state is %ld\n",buf,state);
	if (ret)
		return ret;
	jz_pwm_set(cdev, state,SET_DUTY_RATIO);
	return size;
}

static ssize_t jz_pwm_dev_period_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct jz_pwm_classdev *cdev = dev_get_drvdata(dev);
	/* no lock needed for this */
	jz_pwm_update(cdev);
	return sprintf(buf, "%u\n", cdev->period_ns);
}

static ssize_t jz_pwm_dev_period_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct jz_pwm_classdev *cdev = dev_get_drvdata(dev);
	unsigned long state;
	ssize_t ret = -EINVAL;

	ret = kstrtoul(buf, 10, &state);
	if (ret)
		return ret;
	jz_pwm_set(cdev, state,SET_PERIOD);
	return size;
}

static ssize_t jz_pwm_dev_max_duty_ratio_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct jz_pwm_classdev *cdev = dev_get_drvdata(dev);
	jz_pwm_update(cdev);
	return sprintf(buf, "%u\n", cdev->max_duty_ratio);
}

static ssize_t jz_pwm_dev_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct jz_pwm_classdev *cdev = dev_get_drvdata(dev);
	return sprintf(buf,"%u\n",cdev->enable);
}

static ssize_t jz_pwm_dev_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct jz_pwm_classdev *cdev = dev_get_drvdata(dev);
	unsigned long state;
	ssize_t ret = -EINVAL;
	ret = kstrtoul(buf, 10, &state);
	if (ret)
		return ret;
	cdev->enable = state;
	if(cdev->pwm_enable)
		cdev->pwm_enable(cdev);
	return size;
}

static struct device_attribute jz_pwm_class_attrs[] = {
	__ATTR(period, 0644, jz_pwm_dev_period_show, jz_pwm_dev_period_store),
	__ATTR(max_duty_ratio, 0444, jz_pwm_dev_max_duty_ratio_show, NULL),
	__ATTR(duty_ratio, 0644, jz_pwm_dev_duty_ratio_show, jz_pwm_dev_duty_ratio_store),
	__ATTR(enable,0644,jz_pwm_dev_enable_show,jz_pwm_dev_enable_store),
	__ATTR_NULL,
};


/**
 * jz_pwm_dev_classdev_suspend - suspend an jz_pwm_classdev.
 * @cdev: the jz_pwm_classdev to suspend.
 */
void jz_pwm_classdev_suspend(struct jz_pwm_classdev *cdev)
{
	cdev->flags |= JZ_PWM_DEV_SUSPENDED;
	cdev->pwm_set(cdev,DEV_OFF,SET_DUTY_RATIO);
	//by pillar
	cdev->enable =0;
                cdev->pwm_enable(cdev);
}
EXPORT_SYMBOL_GPL(jz_pwm_classdev_suspend);

/**
 * jz_pwm_classdev_resume - resume an jz_pwm_classdev.
 * @cdev: the jz_pwm_classdev to resume.
 */
void jz_pwm_classdev_resume(struct jz_pwm_classdev *cdev)
{
	cdev->pwm_set(cdev,cdev->duty_ratio,SET_DUTY_RATIO);
	cdev->flags &= ~JZ_PWM_DEV_SUSPENDED;
}
EXPORT_SYMBOL_GPL(jz_pwm_classdev_resume);

static int jz_pwm_dev_suspend(struct device *dev, pm_message_t state)
{
	struct jz_pwm_classdev *cdev = dev_get_drvdata(dev);
	//cdev->pwm_set(cdev,DEV_OFF,SET_DUTY_RATIO);
        //by pillar
	printk("PM:jz_pwm_dev_suspend\n");
        cdev->enable =0;
                cdev->pwm_enable(cdev);
	if (cdev->flags & JZ_PWM_DEV_CORE_SUSPENDRESUME)
		jz_pwm_classdev_suspend(cdev);
	return 0;
}

static int jz_pwm_dev_resume(struct device *dev)
{
	struct jz_pwm_classdev *cdev = dev_get_drvdata(dev);

	if (cdev->flags & JZ_PWM_DEV_CORE_SUSPENDRESUME)
		jz_pwm_classdev_resume(cdev);
	return 0;
}

/**
 * jz_pwm_classdev_register - register a new object of jz_pwm_classdev class.
 * @parent: The device to register.
 * @cdev: the jz_pwm_classdev structure for this device.
 */
int jz_pwm_classdev_register(struct device *parent, struct jz_pwm_classdev *cdev)
{
	cdev->dev = device_create(jz_pwm_class, parent, 0, cdev,
				      "%s", cdev->name);
	if (IS_ERR(cdev->dev))
		return PTR_ERR(cdev->dev);

	/* add to the list of jz_pwm_devs */
	down_write(&jz_pwm_dev_list_lock);
	list_add_tail(&cdev->node, &jz_pwm_dev_list);
	up_write(&jz_pwm_dev_list_lock);

	if (!cdev->max_duty_ratio)
		cdev->max_duty_ratio = DEV_FULL;

	jz_pwm_update(cdev);

	cdev->pwm_set(cdev, 100,SET_DUTY_RATIO);
        cdev->enable =1;
	cdev->pwm_enable(cdev);
//	INIT_WORK(&cdev->set_brightness_work, set_brightness_delayed);

	dev_dbg(parent, "Registered jz pwm generic device: %s\n",
			cdev->name);
	return 0;
}
EXPORT_SYMBOL_GPL(jz_pwm_classdev_register);

/**
 * jz_pwm_classdev_unregister - unregisters a object of jz_pwm_dev_properties class.
 * @cdev: the jz pwm device to unregister
 *
 * Unregisters a previously registered via jz_pwm_classdev_register object.
 */
void jz_pwm_classdev_unregister(struct jz_pwm_classdev *cdev)
{

//	cancel_work_sync(&led_cdev->set_brightness_work);

	jz_pwm_set(cdev, DEV_OFF,SET_DUTY_RATIO);
	device_unregister(cdev->dev);
	down_write(&jz_pwm_dev_list_lock);
	list_del(&cdev->node);
	up_write(&jz_pwm_dev_list_lock);
}
EXPORT_SYMBOL_GPL(jz_pwm_classdev_unregister);

static int __init jz_pwm_dev_init(void)
{
	
	int ret;
	ret = gpio_request(PWM4, "pwm4");
        if (ret) {
            printk(KERN_ERR "can's request pwm4\n");
            //return ret;
        }
	
	/*
	ret = gpio_request(GPIO_LCD_PWM, "pwm0");
        if (ret) {
            printk(KERN_ERR "can's request pwm0\n");
            //return ret;
        }
	*/
	gpio_set_func(pwm4_jz, GPIO_FUNC_0, g4->pins);
	//gpio_set_func(pwm0_jz, GPIO_OUTPUT0, g3->pins);
	jz_pwm_class = class_create(THIS_MODULE, "jz_pwm_dev");
	if (IS_ERR(jz_pwm_class))
		return PTR_ERR(jz_pwm_class);
	jz_pwm_class->suspend = jz_pwm_dev_suspend;
	jz_pwm_class->resume = jz_pwm_dev_resume;
	jz_pwm_class->dev_attrs = jz_pwm_class_attrs;
	return 0;
}

static void __exit jz_pwm_dev_exit(void)
{
	class_destroy(jz_pwm_class);
}

subsys_initcall(jz_pwm_dev_init);
module_exit(jz_pwm_dev_exit);

MODULE_AUTHOR("jingxin.sun@ingenic.com");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("JZ PWM Class Interface");
