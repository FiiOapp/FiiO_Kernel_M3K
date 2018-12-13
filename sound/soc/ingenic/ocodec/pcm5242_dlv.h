/*
 * sound/soc/ingenic/icodec/icdc_d3.h
 * ALSA SoC Audio driver -- ingenic internal codec (icdc_d3) driver

 * Copyright 2015 Ingenic Semiconductor Co.,Ltd
 *	cli <chen.li@ingenic.com>
 *
 * Note: icdc_d3 is an internal codec for jz SOC
 *	 used for jz4775 m150 and so on
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */


#ifndef __PCM5242_DLV_H__
#define __PCM5242_DLV_H__
#include <linux/regulator/machine.h>
#include <linux/skytc/eq/eq.h>
#include <../../arch/mips/xburst/soc-x1000/chip-x1000/halley2/halley2_v10/board.h>
/* 定义幻数 */
#define MEMDEV_IOC_MAGIC2  '4'

/* 定义命令 */
#define MEMDEV_IOCPRINT2   _IO(MEMDEV_IOC_MAGIC2, 0x1c)
#define MEMDEV_IOCGETDATA2 _IOR(MEMDEV_IOC_MAGIC2, 0x1d, int)
#define MEMDEV_IOCSETDATA2 _IOW(MEMDEV_IOC_MAGIC2, 0x1e, int)
#define PCM5242_SET    _IO(MEMDEV_IOC_MAGIC2,0x1f)
#define PCM5242_SET1    _IO(MEMDEV_IOC_MAGIC2,0x20)
#define PCM5242_SET2    _IO(MEMDEV_IOC_MAGIC2,0x21)
#define PCM5242_SET3    _IO(MEMDEV_IOC_MAGIC2,0x22)
#define PCM5242_SET4    _IO(MEMDEV_IOC_MAGIC2,0x23)
#define PCM5242_SET5    _IO(MEMDEV_IOC_MAGIC2,0x24)
#define PCM5242_SET6    _IO(MEMDEV_IOC_MAGIC2,0x25)
#define PCM5242_SET7    _IO(MEMDEV_IOC_MAGIC2,0x27)
#define PCM5242_SET8    _IO(MEMDEV_IOC_MAGIC2,0x28)
#define PCM5242_SET9    _IO(MEMDEV_IOC_MAGIC2,0x29)
#define MEMDEV_IOC_MAXNR2 3

static int jz_eq_debug = 1;
#define i2s_info(msg...)			\
	do {					\
		if (jz_eq_debug)		\
			printk("EQ: " msg);	\
	} while(0)

extern bool global_test_eq_on_off;
 void register_write(int page,int addr,int value);
 void transmit_registers(cfg_reg *r, int n);
 void eq_registers_read(cfg_reg *r, int n);
int register_read(int page,int addr);
void fm_mode_gpio(void);
void dac_mode_gpio(void);
#endif	/* __ICDC_D3_REG_H__ */
