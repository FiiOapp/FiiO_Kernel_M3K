#ifndef __UART_H__
#define __UART_H__

#define PRINT_DEBUG

#ifdef PRINT_DEBUG
#define	UART2_IOBASE	0x10032000
#define OFF_TDR         (0x00)
#define OFF_LCR         (0x0C)
#define OFF_LSR         (0x14)

#define LSR_TDRQ        (1 << 5)
#define LSR_TEMT        (1 << 6)
#define UART_IOBASE (UART2_IOBASE + 0xa0000000)
#define TCSM_PCHAR(x)							\
	*((volatile unsigned int*)(UART_IOBASE+OFF_TDR)) = x;		\
	while ((*((volatile unsigned int*)(UART_IOBASE + OFF_LSR)) & (LSR_TDRQ | LSR_TEMT)) != (LSR_TDRQ | LSR_TEMT))

static inline void uart_put_hex(unsigned int x)
{
	int i;
	unsigned int d;
	for(i = 7;i >= 0;i--) {
		d = (x  >> (i * 4)) & 0xf;
		if(d < 10) d += '0';
		else d += 'A' - 10;
		TCSM_PCHAR(d);
	}
}


#else
#define TCSM_PCHAR(x)
#endif

#define TCSM_DELAY(x)						\
	do{							\
		register unsigned int i = x;			\
		while(i--)					\
			__asm__ volatile(".set mips32\n\t"	\
					 "nop\n\t"		\
					 ".set mips32");	\
	}while(0)

void serial_putc(char x);
void serial_put_hex(unsigned int x);
void serial_setid(int uart_id);
#endif	/* __UART_H__ */
