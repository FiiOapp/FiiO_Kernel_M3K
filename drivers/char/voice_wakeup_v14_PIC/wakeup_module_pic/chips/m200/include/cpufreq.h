#ifndef __CPUFREQ_H__
#define __CPUFREQ_H__
#include <common.h>
#include <uart.h>
#include <cpm.h>

void scale_cpu_freq(int status, unsigned int *cpccr)
{
	unsigned int val;

	if(status == SCALE) {
		/*
		 * (1) SCL_SRC source clock changes APLL to EXCLK
		 * (2) AH0/2 source clock changes MPLL to EXCLK
		 * (3) set PDIV H2DIV H0DIV L2CDIV CDIV = 0
		 */
		val = cpm_inl(CPM_CPCCR);
		*cpccr = val;

		val &= ~((3 << 30) | (0xF << 20) | (0x3F << 24));
		val |=  (2 << 30)  | (1 << 22) | (1 << 28) | (0x5 << 24) | (3 << 20);
		cpm_outl(val,CPM_CPCCR);
		while((cpm_inl(CPM_CPCSR) & 7))
			serial_putc('w');

		val &= ~(0xFF);
		val |=  0x00;
		cpm_outl(val,CPM_CPCCR);
		while((cpm_inl(CPM_CPCSR) & 7))
			serial_putc('g');
		val &= ~(0xFFF00);
		val |= 0x00000;
		cpm_outl(val,CPM_CPCCR);
		while((cpm_inl(CPM_CPCSR) & 7))
			serial_putc('g');
	} else if (status == RESTORE){
		val = cpm_inl(CPM_CPCCR);
		val &= ~(0xfffff);
		val |= *cpccr & 0xfffff;
		cpm_outl(val,CPM_CPCCR);

		cpm_outl(*cpccr | (7 << 20),CPM_CPCCR);
		while((cpm_inl(CPM_CPCSR) & 7))
			serial_putc('w');
	}
}

#endif	/* __CPUFREQ_H__ */
