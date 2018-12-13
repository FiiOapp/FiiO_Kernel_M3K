/*
 * Copyright (C) 2016 ingenic, Inc.
 * Author: chen.li <chen.li@ingenic.com>
 * Baseon: arch/mips/include/asm/mach-loongson/loongson.h
 *
 * This program is free software; you can redistribute	it and/or modify it
 * under  the terms of	the GNU General	 Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef __XBURST_VPCI_H
#define __XBURST_VPCI_H

#include <linux/io.h>
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/kconfig.h>

/* loongson internal northbridge initialization */
extern void xburst_vpci_irq_init(void);

/* irq operation functions */
extern void __init xburst_vpci_irq_init(void);
extern void xburst_vpci_irqdispatch(void);


#define XBURST_VPCIREG_BASE		0x1fe00000
#define XBURST_VPCIREG_SIZE		0x00100000
#define XBURST_VPCIREG_TOP		(XBURST_VPCIREG_BASE+XBURST_VPCIREG_SIZE-1)

#define XBURST_VPCIREG(x) \
	(*(volatile u32 *)((char *)CKSEG1ADDR(XBURST_VPCIREG_BASE) + (x)))

#define XBURST_VPCILO_BASE		0x14000000
#define XBURST_VPCILO_SIZE		0x08000000	/*64M * 2*/
#define XBURST_VPCIIO_BASE		0x1c000000
#define XBURST_VPCIIO_SIZE		0x00008000	/*32K*/

/*PCI CONFIG address*/
#define XBURST_VPCICFG_BASE	0x1fe80000
#define XBURST_VPCICFG_SIZE	0x00000800	/* 2K */
#define XBURST_VPCICFG_TOP		(XBURST_VPCICFG_BASE+XBURST_VPCICFG_SIZE-1)

/* Loongson Registe Bases */
#define XBURST_VPCI_REGBASE	0x100
/* PCI address map ontrol */
#define XBURST_VPCIMAP_CFG		XBURST_VPCIREG(XBURST_VPCI_REGBASE + 0x18)
/* ICU Enable Regs  IntEn & IntISR are r/o. */
#define XBURST_VPCI_INTENSET		XBURST_VPCIREG(XBURST_VPCI_REGBASE + 0x30)
#define XBURST_VPCI_INTENCLR		XBURST_VPCIREG(XBURST_VPCI_REGBASE + 0x34)
#define XBURST_VPCI_INTEN			XBURST_VPCIREG(XBURST_VPCI_REGBASE + 0x38)
#define XBURST_VPCI_INTISR			XBURST_VPCIREG(XBURST_VPCI_REGBASE + 0x3c)
#endif /* __XBURST_VPCI_H */
