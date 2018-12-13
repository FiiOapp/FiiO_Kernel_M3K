#ifndef __PRINT_H__
#define __PRINT_H__

enum debug_print_level {
	LOG_DEFAULT = 1,
	LOG_VERBOSE,
	LOG_DEBUG,
	LOG_INFO,
	LOG_WARN,
	LOG_ERROR,
	LOG_FATAL,
};



#if 1				/* debug */
#define OFF_TDR         (0x00)
#define OFF_LCR         (0x0C)
#define OFF_LSR         (0x14)

#define LSR_TDRQ        (1 << 5)
#define LSR_TEMT        (1 << 6)

#define UART2_IOBASE    0x10032000
#define UART_IOBASE (UART2_IOBASE + 0xa0000000)
#define TCSM_PCHAR(x)							\
	do {								\
		while ((*((volatile unsigned int*)(UART_IOBASE + OFF_LSR)) & (LSR_TDRQ | LSR_TEMT)) != (LSR_TDRQ | LSR_TEMT)); \
		*((volatile unsigned int*)(UART_IOBASE+OFF_TDR)) = x;	\
	}while(0)

static inline void serial_put_hex(unsigned int x) {
	int i;
	unsigned int d;
	for(i = 7;i >= 0;i--) {
		d = (x  >> (i * 4)) & 0xf;
		if(d < 10) d += '0';
		else d += 'A' - 10;
		TCSM_PCHAR(d);
	}
}

/* voice trigger wakeup print */
extern int vtw_print_str(int debug_level, const char *fmt, ...);
extern int vtw_print_str2(const char *fmt, ...);
extern int vtw_print_hex(int debug_level, unsigned int x);
extern int vtw_print_hex2(unsigned int x);


extern int set_debug_print_level(int);
extern int get_debug_print_level(void);
int set_printk_callback(void *handler);

#else
#define TCSM_PCHAR(x)	do {} while(0)
#define serial_put_hex(x) do {} while(0)
#define vtw_print_str(x, ...) do {} while(0)
#define vtw_print_hex(x) do {} while(0)
#endif

#define vtw_print vtw_print_str
#define vtw_print2 vtw_print_str2
#define printk vtw_print_str2

#endif	/* __PRINT_H__ */
