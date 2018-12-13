/* include/linux/regulator/axp173.h
 *
 * Copyright (C) 2011 ROCKCHIP, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#ifndef __LINUX_REGULATOR_axp173_H
#define __LINUX_REGULATOR_axp173_H
#include <../../arch/mips/xburst/soc-x1000/chip-x1000/halley2/halley2_v10/board.h>
#include <linux/regulator/machine.h>

/* 定义幻数 */
#define MEMDEV_IOC_MAGIC  '3'

/* 定义命令 */
#define MEMDEV_IOCPRINT   _IO(MEMDEV_IOC_MAGIC, 0x20)
#define MEMDEV_IOCGETDATA _IOR(MEMDEV_IOC_MAGIC, 0x21, int)
#define MEMDEV_IOCSETDATA _IOW(MEMDEV_IOC_MAGIC, 0x22, int)
#define AXP173_SHUTDOWN    _IO(MEMDEV_IOC_MAGIC,0x23)
#define BLED_OPEN _IO(MEMDEV_IOC_MAGIC,0x24)
#define BLED_CLOSE _IO(MEMDEV_IOC_MAGIC,0x25)
#define MEMDEV_IOC_MAXNR 3
/*
struct _headset {
// struct switch_dev sdev;
 struct input_dev *input;
 //struct _headset_gpio detect;
 //struct _headset_gpio button;
 int headphone;
};*/
extern struct axp173;
extern struct axp173 *axp173;
//extern int axp173_i2c_read(struct axp173 *bq, u8 reg);
extern int axp173_i2c_write_bit(struct axp173 *bq, u8 reg, bool val, u8 bit);
extern bool m3k_usb_det;
extern bool m3k_gadget_sign;
/*
#define AXP173_NUM_REGULATORS 4
struct axp173;

int axp173_device_shutdown(void);

struct axp173_regulator_subdev {
	int id;
	struct regulator_init_data *initdata;
};

struct axp173_platform_data {
	int num_regulators;
	int (*set_init)(struct axp173 *axp173);
	struct axp173_regulator_subdev *regulators;
};
*/
#endif

