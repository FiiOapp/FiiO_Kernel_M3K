/*
 *	BaseOn  arch/mips/pci/fuloong2e.c
 *
 *  This program is free software; you can redistribute	 it and/or modify it
 *  under  the terms of	 the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the	License, or (at your
 *  option) any later version.
 */
#include <linux/init.h>
#include <linux/pci.h>
#include <soc/irq.h>

int __init pcibios_map_irq(const struct pci_dev *dev, u8 slot, u8 pin)
{
#define PIN_MAX (4)
	int irq = (slot + (pin - 1) + PIN_MAX * dev->bus->number) % 32;
	return irq + IRQ_PCI_BASE;
#undef PIN_MAX
}

/* Do platform specific device initialization at pci_enable_device() time */
int pcibios_plat_dev_init(struct pci_dev *dev)
{
	return 0;
}
