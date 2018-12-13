#include "jz_cpm.h"
#include "common.h"
#include "print.h"
#include "pm_ops.h"


static inline void flush_dcache_all()
{
	u32 addr;

	for (addr = 0x80000000; addr < 0x80000000 + CONFIG_SYS_DCACHE_SIZE; \
	     addr += CONFIG_SYS_CACHELINE_SIZE) {
		cache_op(0x01, addr); /* Index Invalid. */
	}
}

void cpu_deep_sleep(void)
{
	unsigned int opcr;
	unsigned int val;
	unsigned int lcr;

	TCSM_PCHAR('D');
	TCSM_PCHAR('s');
	TCSM_PCHAR('l');
	TCSM_PCHAR('p');
	TCSM_PCHAR('\r');
	TCSM_PCHAR('\n');

	/* cpu enter sleep */
	lcr = REG32(CPM_IOBASE + CPM_LCR);
	lcr &= ~(3|(0xfff<<8));
	lcr |= 0xfff << 8;	/* power stable time */
	lcr |= 1;
	REG32(CPM_IOBASE + CPM_LCR) = lcr;

	opcr = (CPM_IOBASE + CPM_OPCR);
	opcr &= ~((1 << 7) | (1 << 6) | (1 << 4) | (0xfff << 8) | (1 << 22) | (1 << 25) | (1 << 28)| (1 << 3));
	opcr |= (1 << 31) | (1 << 30) | (1 << 25) | (1 << 23) | (0xfff << 8) | (1 << 2) | (1 << 3) | (1 << 4);
	REG32(CPM_IOBASE + CPM_OPCR) = opcr;
	opcr = REG32(CPM_IOBASE + CPM_OPCR);

	cache_prefetch_voice(LABLE1,512);
LABLE1:
	flush_dcache_all();
	//while(!cpu_should_sleep());

	val = REG32(0xb0000000);
	val &= ~(0xff);
	val |= 0x73;
	REG32(0xb0000000) = val;
	while((REG32(0xB00000D4) & 7))
		TCSM_PCHAR('A');

	__asm__ volatile(".set mips32\n\t"
			"nop\n\t"
			"nop\n\t"
			"nop\n\t"
			"wait\n\t"
			"nop\n\t"
			"nop\n\t"
			"nop\n\t"
			"nop\n\t"
			".set mips32 \n\t");
	/* can't be here ,it's err occurs!!! */
	while(1) {
		TCSM_PCHAR('S');
		TCSM_PCHAR('E');
		TCSM_PCHAR('R');
		TCSM_PCHAR('R');
		TCSM_PCHAR(' ');
	}
}

void cpu_normal_sleep(void)
{
	unsigned int opcr;
	unsigned int lcr;

	TCSM_PCHAR('N');
	TCSM_PCHAR('s');
	TCSM_PCHAR('l');
	TCSM_PCHAR('p');
	TCSM_PCHAR('\r');
	TCSM_PCHAR('\n');

	/* cpu enter sleep */
	lcr = REG32(CPM_IOBASE + CPM_LCR);
	lcr &= ~(3|(0xfff<<8));
	lcr |= 0xfff << 8;	/* power stable time */
	lcr |= 1;
	REG32(CPM_IOBASE + CPM_LCR) = lcr;

	opcr = (CPM_IOBASE + CPM_OPCR);
	opcr &= ~((1 << 7) | (1 << 6) | (1 << 4) | (0xfff << 8) | (1 << 22) | (1 << 25) | (1 << 28)| (1 << 3));
	opcr |= (1 << 31) | (1 << 30) | (1 << 25) | (1 << 23) | (0xfff << 8) | (1 << 2) | (1 << 4);
	REG32(CPM_IOBASE + CPM_OPCR) = opcr;
	opcr = REG32(CPM_IOBASE + CPM_OPCR);

	cache_prefetch_voice(LABLE1,32);
LABLE1:
	__asm__ volatile(".set mips32\n\t"
			"nop\n\t"
			"nop\n\t"
			"wait\n\t"
			"nop\n\t"
			"nop\n\t"
			"nop\n\t"
			".set mips32 \n\t");
//	TCSM_PCHAR('s');
}

void cpu_idle(void)
{
	unsigned int val;

	/* TCSM_PCHAR('I'); */
	/* TCSM_PCHAR('D'); */
	/* TCSM_PCHAR('L'); */
	/* TCSM_PCHAR('E'); */
	/* TCSM_PCHAR('\r'); */
	/* TCSM_PCHAR('\n'); */

	/* cpu enter sleep */
	val = REG32(CPM_IOBASE + CPM_LCR);
	val &= ~(3);
	REG32(CPM_IOBASE + CPM_LCR) = val;

	val = (CPM_IOBASE + CPM_OPCR);
	val &= ~((1 << 30) | (1 << 28) | (1 << 22) | (0xfff << 8) | (1 << 7) | (1 << 6) |  (1 << 3) | (1 << 2));
	val |=  (1 << 31) | (1 << 25) | (1 << 23) | (0xfff << 8) | (1 << 4);
	REG32(CPM_IOBASE + CPM_OPCR) = val;
	val = REG32(CPM_IOBASE + CPM_OPCR);

	__asm__ volatile(".set mips32\n\t"
			"nop\n\t"
			"wait\n\t"
			"nop\n\t"
			"nop\n\t"
			".set mips32\n\t");
//	TCSM_PCHAR('I');
}
