/*
 * [board]-cim.c - This file defines camera host driver (cim) on the board.
 *
 * Copyright (C) 2012 Ingenic Semiconductor Co., Ltd.
 * Author: YeFei <feiye@ingenic.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/platform_device.h>
#include <linux/delay.h>
#include <media/soc_camera.h>
#include <mach/platform.h>
#include <mach/jz_camera.h>
#include <linux/regulator/machine.h>
#include <gpio.h>
#include "board.h"

extern struct i2c_board_info jz_i2c0_camera_devs[];
extern struct i2c_board_info jz_i2c1_camera_devs[];

#ifdef CONFIG_VIDEO_JZ_CIM_HOST_V11

#ifdef CONFIG_SOC_JZ_CIM0
struct jz_camera_pdata back_camera_pdata = {
	.mclk_10khz = 2400,
	.flags = 0 ,
	.cam_sensor_pdata[BACK_CAMERA_INDEX] = {
		.gpio_rst = BACK_CAMERA_SENSOR_RESET,
		.gpio_power = BACK_CAMERA_SENSOR_EN,
	},
};

static int back_camera_sensor_reset(struct device *dev) {
	gpio_set_value(BACK_CAMERA_SENSOR_RESET, 0);
	msleep(10);
	gpio_set_value(BACK_CAMERA_SENSOR_RESET, 1);
	msleep(10);

	return 0;
}

static int back_camera_sensor_power(struct device *dev, int on) {
	/* enable or disable the camera */
	gpio_set_value(BACK_CAMERA_SENSOR_EN, on ? 0 : 1);
	msleep(10);

	return 0;
}

static struct soc_camera_link iclink_back = {
	.bus_id		= 0,		/* Must match with the camera ID */
	.board_info	= &jz_i2c0_camera_devs[BACK_CAMERA_INDEX],
	.i2c_adapter_id	= 0,
	.power = back_camera_sensor_power,
	.reset = back_camera_sensor_reset,
};

struct platform_device back_camera_sensor = {
	.name	= "soc-camera-pdrv",
	.id	= 0,
	.dev	= {
		.platform_data = &iclink_back,
	},
};
#endif	/* CONFIG_SOC_JZ_CIM0 */

#ifdef CONFIG_SOC_JZ_CIM1
struct jz_camera_pdata front_camera_pdata = {
	.mclk_10khz = 2400,
	.flags = 0 ,
	.cam_sensor_pdata[FRONT_CAMERA_INDEX] = {
		.gpio_rst = FRONT_CAMERA_SENSOR_RESET,
		.gpio_power = FRONT_CAMERA_SENSOR_EN,
	},
};
static int front_camera_sensor_reset(struct device *dev) {
	gpio_set_value(FRONT_CAMERA_SENSOR_RESET, 0);
	msleep(10);
	gpio_set_value(FRONT_CAMERA_SENSOR_RESET, 1);
	msleep(10);

	return 0;
}
static int front_camera_sensor_power(struct device *dev, int on) {
	/* enable or disable the camera */
	gpio_set_value(FRONT_CAMERA_SENSOR_EN, on ? 0 : 1);
	msleep(10);

	return 0;
}
static struct soc_camera_link iclink_front = {
	.bus_id		= 1,		/* Must match with the camera ID */
	.board_info	= &jz_i2c1_camera_devs[FRONT_CAMERA_INDEX],
	.i2c_adapter_id	= 1,
	.power = front_camera_sensor_power,
	.reset = front_camera_sensor_reset,
};

struct platform_device front_camera_sensor = {
	.name	= "soc-camera-pdrv",
	.id	= 1,
	.dev	= {
		.platform_data = &iclink_front,
	},
};
#endif	/* CONFIG_SOC_JZ_CIM1 */


#endif
static int __init mensa_board_cim_init(void) {
	/* camera host */
#ifdef CONFIG_VIDEO_JZ_CIM_HOST_V11
#ifdef CONFIG_SOC_JZ_CIM0
	jz_device_register(&jz_cim0_device, &back_camera_pdata);
#endif
#ifdef CONFIG_SOC_JZ_CIM1
	jz_device_register(&jz_cim1_device, &front_camera_pdata);
#endif
#endif
	/* camera sensor */
#ifdef CONFIG_SOC_JZ_CIM0
	platform_device_register(&back_camera_sensor);
#endif
#ifdef CONFIG_SOC_JZ_CIM1
	platform_device_register(&front_camera_sensor);
#endif
	return 0;
}

core_initcall(mensa_board_cim_init);
