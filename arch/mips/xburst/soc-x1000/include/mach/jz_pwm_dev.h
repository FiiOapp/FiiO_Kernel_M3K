/*
 * Driver model for leds and led triggers
 *
 * Copyright (C) 2005 John Lenz <lenz@cs.wisc.edu>
 * Copyright (C) 2005 Richard Purdie <rpurdie@openedhand.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#ifndef __MACH_JZ_PWM_DEV_H
#define __MACH_JZ_PWM_DEV_H

#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/rwsem.h>
#include <linux/timer.h>
#include <linux/workqueue.h>


#define SET_DUTY_RATIO	0
#define SET_PERIOD	1

#define PWM_PERIOD_MIN	200
#define PWM_PERIOD_MAX	100000000

enum dev_duty_ratio{
	DEV_OFF = 0,
	DEV_HALF = 127,
	DEV_FULL = 255,
};

struct jz_pwm_dev {
	const char      *name;
	unsigned        pwm_id __deprecated;
	u8              active_low;
	unsigned        max_duty_ratio;
	unsigned       	period_ns;
};

struct jz_pwm_dev_platform_data {
	int	num_devs;
	struct jz_pwm_dev  *devs;
};

struct jz_pwm_classdev {
	const char		*name;
	int			period_ns;
	int			duty_ratio;
	int			max_duty_ratio;
	int			flags;
	int			enable;
#define JZ_PWM_DEV_SUSPENDED	(1 << 0)
#define JZ_PWM_DEV_CORE_SUSPENDRESUME  (1 << 16)
	void		(*pwm_set)(struct jz_pwm_classdev *cdev,int val,int flag);
	int		(*pwm_get)(struct jz_pwm_classdev *cdev);
	int		(*pwm_enable)(struct jz_pwm_classdev *cdev);
	struct device		*dev;
	struct list_head	 node;			/* LED Device list */
	struct work_struct	jz_pwm_dev_work;
	int			delayed_set_value;
};

extern int jz_pwm_classdev_register(struct device *parent,
				 struct jz_pwm_classdev *jz_pwm_cdev);
extern void jz_pwm_classdev_unregister(struct jz_pwm_classdev *jz_pwm_cdev);
extern void jz_pwm_classdev_suspend(struct jz_pwm_classdev *jz_pwm_cdev);
extern void jz_pwm_classdev_resume(struct jz_pwm_classdev *jz_pwm_cdev);

#endif		/* __MACH_JZ_PWM__DEV_H */
