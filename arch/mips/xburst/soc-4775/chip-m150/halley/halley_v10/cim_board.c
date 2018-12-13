/*
 * [board]-cim.c - This file defines camera host driver (cim) on the board.
 *
 * Copyright (C) 2012 Ingenic Semiconductor Co., Ltd.
 * Author: xiaoyangfu <xyfu@ingenic.com>
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
#include <media/ov772x.h>

extern struct i2c_board_info jz_i2c0_devs[];
extern struct i2c_board_info jz_i2c1_devs[];
#define CIM0 0
#define CIM1 1
#ifdef CONFIG_VIDEO_JZ_CIM_HOST_V11
#ifdef CONFIG_SOC_JZ_CIM0
struct jz_camera_pdata camera_pdata_front = {
	.mclk_10khz = 2400,
	.flags = 0,
	.cam_sensor_pdata[FRONT_CAMERA_INDEX] = {
		.gpio_rst = FRONT_CAMERA_SENSOR_RESET,
		.gpio_power = FRONT_CAMERA_SENSOR_EN,
	},
};

static int camera_sensor_reset_front(struct device *dev) {
	gpio_direction_output(FRONT_CAMERA_SENSOR_RESET, 0);
	mdelay(250);
	gpio_direction_output(FRONT_CAMERA_SENSOR_RESET, 1);
	mdelay(250);
	return 0;
}

static int front_camera_sensor_power(struct device *dev, int on) {
	/* enable or disable the camera */
	gpio_direction_output(FRONT_CAMERA_SENSOR_EN, on ? 0 : 1);
	mdelay(250);

	return 0;
}

struct ov772x_camera_info ov772x_priv_front = {
	.flags = OV772X_FLAG_8BIT,
	.edgectrl = {
		.strength = OV772X_EDGE_STRENGTH_MASK & 0x01,
		.threshold = OV772X_EDGE_STRENGTH_MASK & 0x04,
		.upper = OV772X_EDGE_UPPER_MASK & 0x10,
		.lower = OV772X_EDGE_LOWER_MASK & 0x02,
	},
};

struct soc_camera_desc icdesc_front = {
	.host_desc = {
		.bus_id = CIM0,
		.board_info	= &jz_i2c0_devs[FRONT_CAMERA_INDEX],
		.i2c_adapter_id	= 0,
	},
	.subdev_desc = {
		.power = front_camera_sensor_power,
		.reset = camera_sensor_reset_front,
		.drv_priv = &ov772x_priv_front,
	},
};


struct platform_device camera_sensor_front = {
	.name	= "soc-camera-pdrv",
	.id	= 0,
	.dev	= {
		.platform_data = &icdesc_front,
	},
};
#endif

#ifdef CONFIG_SOC_JZ_CIM1
struct jz_camera_pdata camera_pdata_back = {
	.mclk_10khz = 2400,
	.flags = 0,
	.cam_sensor_pdata[BACK_CAMERA_INDEX] = {
		.gpio_rst = BACK_CAMERA_SENSOR_RESET,
		.gpio_power = BACK_CAMERA_SENSOR_EN,
	},
};

static int camera_sensor_reset_back(struct device *dev) {
	gpio_direction_output(BACK_CAMERA_SENSOR_RESET, 0);
	mdelay(250);
	gpio_direction_output(BACK_CAMERA_SENSOR_RESET, 1);
	mdelay(250);
	return 0;
}

static int camera_sensor_power_back(struct device *dev, int on) {
	/* enable or disable the camera */
	gpio_direction_output(BACK_CAMERA_SENSOR_EN, on ? 0 : 1);
	mdelay(250);

	return 0;
}

struct ov772x_camera_info ov772x_priv_back = {
	.flags = OV772X_FLAG_8BIT,
	.edgectrl = {
		.strength = OV772X_EDGE_STRENGTH_MASK & 0x01,
		.threshold = OV772X_EDGE_STRENGTH_MASK & 0x04,
		.upper = OV772X_EDGE_UPPER_MASK & 0x10,
		.lower = OV772X_EDGE_LOWER_MASK & 0x02,
	},
};

struct soc_camera_desc icdesc_back = {
	.host_desc = {
		.bus_id = CIM1,
		.board_info	= &jz_i2c1_devs[BACK_CAMERA_INDEX],
		.i2c_adapter_id	= 1,
	},
	.subdev_desc = {
		.power = camera_sensor_power_back,
		.reset = camera_sensor_reset_back,
		.drv_priv = &ov772x_priv_back,
	},
};


struct platform_device camera_sensor_back = {
	.name	= "soc-camera-pdrv",
	.id	= 1,
	.dev	= {
		.platform_data = &icdesc_back,
	},
};

#endif
#endif
static int __init mensa_board_cim_init(void) {
	/* camera host */

#ifdef CONFIG_VIDEO_JZ_CIM_HOST_V11
#ifdef CONFIG_SOC_JZ_CIM0
	jz_device_register(&jz_cim0_device, &camera_pdata_front);
#endif

#ifdef	CONFIG_SOC_JZ_CIM1
	jz_device_register(&jz_cim1_device, &camera_pdata_back);
#endif
#endif
	/* camera sensor */
#ifdef CONFIG_SOC_CAMERA
#ifdef CONFIG_SOC_JZ_CIM0
	platform_device_register(&camera_sensor_front);
#endif

#ifdef	CONFIG_SOC_JZ_CIM1
	platform_device_register(&camera_sensor_back);
#endif
#endif

	return 0;
}

//arch_initcall(mensa_board_cim_init);
core_initcall(mensa_board_cim_init);
