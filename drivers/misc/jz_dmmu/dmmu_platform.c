/*
 * kernel/arch/mips/jz4780/aurora/common/dmmu_platform.c
 *
 * Copyright (c) 2012 Engenic Semiconductor Co., Ltd.
 *              http://www.engenic.com/
 *
 * Platform device support for JZ4780 SOC.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/


#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/resource.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_gpio.h>

#define DMMU_IOBASE	0x13089000
#define IRQ_DMMU    61

/*** LCD controller ***/
static struct resource jz_dmmu_resources[] = {
	[0] = {
		.start          = DMMU_IOBASE,
		.end            = DMMU_IOBASE + 0x2 - 1,
		.flags          = IORESOURCE_MEM,
	},
	[1] = {
		.start          = IRQ_DMMU,
		.end            = IRQ_DMMU,
		.flags          = IORESOURCE_IRQ,
	}
};

static u64 jz_dmmu_dmamask = ~(u32)0;

struct platform_device jz_dmmu_device = {
	.name           = "jz-dmmu",
	.id             = 0,
	.dev = {
		.dma_mask               = &jz_dmmu_dmamask,
		.coherent_dma_mask      = 0xffffffff,
	},
	.num_resources  = ARRAY_SIZE(jz_dmmu_resources),
	.resource       = jz_dmmu_resources,
};

static struct platform_device *jz_platform_devices[] __initdata = {
	&jz_dmmu_device,
};

static int __init jz_platform_init(void)
{
	int ret = platform_add_devices(jz_platform_devices, ARRAY_SIZE(jz_platform_devices));

	printk("jz_platform_init--->dmmu\n");
	return ret;
}

arch_initcall(jz_platform_init);
