/*
 * include/linux/sky1311t.h
 *
 * Skyrelay sky1311t contactless reader IC driver.
 * This driver support for Ingenic X1000 SoC.
 *
 * Copyright 2016, <qiuwei.wang@ingenic.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#ifndef __SKY1311T_H
#define __SKY1311T_H

#define SKY1311T_DEV_NAME    "sky1311t"

struct sky1311t_platform_data {
    unsigned int spi_cs_pin;
    unsigned int ic_int_pin;  /* IRQ signal from IC to CPU */
    unsigned int ic_pd2_pin;
};

#endif
