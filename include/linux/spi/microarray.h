/*
 * This file is part of MicroArray FingerPrint driver
 *
 * Copyright (C) 2005-2017 Ingenic  Semiconductor Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 */

#ifndef _LINUX_SPI_MICROARRAY_H
#define _LINUX_SPI_MICROARRAY_H

#define MICROARRAY_DRV_NAME             "madev0"

struct microarray_platform_data {
    int     power_2v8;
    int     power_1v8;
    int     power_en;
    int     gpio_int;
    int     reset;

    int (*fingerprint_power_on)(void);
    int (*fingerprint_power_off)(void);
};

#endif
