/*
 *	Author: chen.li chen.li@ingenic.com
 *	Baseon: arch/mips/loongson/common/pci.c
 *			arch/mips/loongson/common/bonito-irq.c
 *
 *  This program is free software; you can redistribute	 it and/or modify it
 *  under  the terms of	 the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the	License, or (at your
 *  option) any later version.
 */
#include <linux/interrupt.h>
#include <linux/pci.h>
#include <soc/pci.h>

static inline void xburst_vpci_irq_enable(struct irq_data *d)
{
	XBURST_VPCI_INTENSET = (1 << (d->irq - IRQ_PCI_BASE));
	mmiowb();
}

static inline void xburst_vpci_irq_disable(struct irq_data *d)
{
	XBURST_VPCI_INTENCLR = (1 << (d->irq - IRQ_PCI_BASE));
	mmiowb();
}

static struct irq_chip xburst_vpci_irq_type = {
	.name		= "xburst_vpci_irq",
	.irq_mask	= xburst_vpci_irq_disable,
	.irq_unmask	= xburst_vpci_irq_enable,
};

void xburst_vpci_irqdispatch(void)
{
	u32 int_status;
	int i;

	/* Get pending sources, masked by current enables */
	int_status = XBURST_VPCI_INTISR & XBURST_VPCI_INTEN;

	if (int_status) {
		i = __ffs(int_status);
		do_IRQ(IRQ_PCI_BASE + i);
	}
}
EXPORT_SYMBOL(xburst_vpci_irqdispatch);

void xburst_vpci_irq_init(void)
{
	u32 i;
	for (i = IRQ_PCI_BASE; i < IRQ_PCI_BASE + 32; i++)
		irq_set_chip_and_handler(i, &xburst_vpci_irq_type,
					 handle_level_irq);
}
EXPORT_SYMBOL(xburst_vpci_irq_init);

extern struct pci_ops xburst_vir_pci_ops;

static struct resource pci_mem_resource = {
	.name	= "pci memory space",
	.start	= XBURST_VPCILO_BASE,
	.end	= XBURST_VPCILO_BASE + XBURST_VPCILO_SIZE - 1,
	.flags	= IORESOURCE_MEM,
};

static struct resource pci_io_resource = {
	.name	= "pci io space",
	.start	= XBURST_VPCIIO_BASE,
	.end	= XBURST_VPCIIO_BASE + XBURST_VPCIIO_SIZE -1,
	.flags	= IORESOURCE_IO,
};

static struct pci_controller pci_controller = {
	.pci_ops	= &xburst_vir_pci_ops,
	.io_resource	= &pci_io_resource,
	.mem_resource	= &pci_mem_resource,
	.mem_offset	= 0x00000000UL,
	.io_offset	= 0x00000000UL,
};

static int __init pcibios_init(void)
{
	pci_controller.io_map_base = (unsigned long)ioremap(XBURST_VPCIIO_BASE, XBURST_VPCIIO_SIZE);

	register_pci_controller(&pci_controller);
	return 0;
}
arch_initcall(pcibios_init);
