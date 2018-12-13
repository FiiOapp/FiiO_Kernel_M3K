/*
 * kernel/arch/mips/mach-jz4780/boards/aurora/aurora-dmmu.c
 *
 * Copyright (c) 2012 Engenic Semiconductor Co., Ltd.
 *              http://www.engenic.com/
 *
 * JZ4780 aurora board lcd setup routines.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/ioport.h>
#include <linux/mm.h>
#include <linux/console.h>
#include <linux/delay.h>
#include <linux/wakelock.h>

#include <asm/cpu.h>
#include <asm/bootinfo.h>
#include <asm/mipsregs.h>
#include <asm/reboot.h>

#include "jz_dmmu_v10.h"

#define DEV_NAME "dmmu"

extern struct platform_device jz_dmmu_device;

static struct jz_dmmu_platform_data jz_dmmu_platform_info = {
	.name = DEV_NAME,
	.page_table_pool_init_capacity = 4,
	.start = 0,
	.cached = 0,
	.reserved_size = 0,
};

static int __init jz_dmmu_init(void)
{
	platform_device_add_data(&jz_dmmu_device, &jz_dmmu_platform_info, sizeof(struct jz_dmmu_platform_data));

	return 0;
}

arch_initcall(jz_dmmu_init);
