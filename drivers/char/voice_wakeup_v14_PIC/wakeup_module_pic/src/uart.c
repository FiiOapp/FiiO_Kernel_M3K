#include <common.h>
#include <uart.h>
//#include <cpm.h>

void udelay(int d)
{
	do{
		__asm__ __volatile__ ("nop \n\t");
		d -= (1000 + 23)/ 24 * 2;
	}while(d > 0);

}

#ifdef DEBUG

#define OFF_TDR         (0x00)
#define OFF_LCR         (0x0C)
#define OFF_LSR         (0x14)

#define LSR_TDRQ        (1 << 5)
#define LSR_TEMT        (1 << 6)

static unsigned int U_IOBASE = (0x10030000 + 0xa0000000);
static unsigned int U_ADDR = 0;

void serial_setid(int uart_id)
{
	/* [2016-08-20 12:34:23.004] S
[2016-08-20 12:34:23.005] 01
[2016-08-20 12:34:23.005] i 000000FF u B012F000
[2016-08-20 12:34:23.007] 12
[2016-08-20 12:34:23.007] C B0032000 B012F0001
 */
	/* TCSM_PCHAR('\r');TCSM_PCHAR('\n'); */
	/* TCSM_PCHAR('i'); */
	/* TCSM_PCHAR(' '); */
	/* uart_put_hex(uart_id); */

	U_ADDR = U_IOBASE + uart_id * 0x1000;

	/* TCSM_PCHAR(' '); */
	/* TCSM_PCHAR('u'); */
	/* TCSM_PCHAR(' '); */
	/* uart_put_hex(U_ADDR); */
	/* TCSM_PCHAR('\r');TCSM_PCHAR('\n'); */
}
void serial_putc(char x)
{
	//TCSM_PCHAR('C'); TCSM_PCHAR('\r');TCSM_PCHAR('\n');
#if 0
	TCSM_PCHAR('\r');TCSM_PCHAR('\n');
	TCSM_PCHAR('C');
	TCSM_PCHAR(' ');
	uart_put_hex(UART_IOBASE);
	TCSM_PCHAR(' ');
	uart_put_hex(U_ADDR);
	TCSM_PCHAR(' ');
#endif
#if 1
	REG32(U_ADDR + OFF_TDR) = x;
	while ((REG32(U_ADDR + OFF_LSR) & (LSR_TDRQ | LSR_TEMT)) != (LSR_TDRQ | LSR_TEMT));
#else
	TCSM_PCHAR(x);
#endif
	//TCSM_PCHAR(' ');
	//TCSM_PCHAR('2');
}
void serial_put_hex(unsigned int x)
{
	int i;
	unsigned int d;
	for(i = 7;i >= 0;i--) {
		d = (x  >> (i * 4)) & 0xf;
		if(d < 10) d += '0';
		else d += 'A' - 10;
		serial_putc(d);
	}
}
#else
void serial_setid(int uart_id)
{
}
void serial_putc(char x)
{
}
void serial_put_hex(unsigned int x)
{
}
#endif
