
/*
  cp /pdev/chains/mips-gcc472-glibc216/lib/gcc/mips-linux-gnu/4.7.2/include/stdarg.h ../include/
 */
#include <stdarg.h>
#include "interface.h"

#include "print.h"

static int g_debug_level = 0;

int (*printk_callback)(const char *fmt, ...);


int set_printk_callback(void *handler)
{
	printk_callback = (int(*)(const char *fmt, ...)) handler;

	return 0;
}


int set_debug_print_level(int l)
{
	g_debug_level = l;
	return l;
}

int get_debug_print_level(void)
{
	return g_debug_level;
}



/* voice trigger wakeup print */
int vtw_print_str(int debug_level, const char *fmt, ...)
{
	va_list args;
	int state;

	if ( debug_level < g_debug_level ) {
		return 0;
		//goto end;
	}

	va_start(args, fmt);

	state = get_voice_wakeup_state();

	if (state == DEEP_SLEEP) {
		char c;
		while ((c = *fmt++) != '\0') {
			TCSM_PCHAR(c);
		}

	}
	else {
		printk_callback(fmt, args);
		//printk_callback(fmt);
	}

	va_end(args);

	return 0;
}

/* voice trigger wakeup print */
int vtw_print_str2(const char *fmt, ...)
{
	va_list args;
	int state;

	if ( g_debug_level > LOG_INFO ) {
		return 0;
		//goto end;
	}

	va_start(args, fmt);
	//vtw_print_str(LOG_INFO, fmt, args);

	state = get_voice_wakeup_state();

	if (state == DEEP_SLEEP) {
		char c;
		while ((c = *fmt++) != '\0') {
			TCSM_PCHAR(c);
		}

	}
	else {
		printk_callback(fmt, args);
		//printk_callback(args);
	}


	va_end(args);
	return 0;
}

int vtw_print_hex(int debug_level, unsigned int x)
{
	int state;

	if ( debug_level < g_debug_level ) {
		return 0;
		//goto end;
	}

	state = get_voice_wakeup_state();

	if (state == DEEP_SLEEP) {
		int i;
		unsigned int d;
		for(i = 7;i >= 0;i--) {
			d = (x  >> (i * 4)) & 0xf;
			if(d < 10) d += '0';
			else d += 'A' - 10;
			TCSM_PCHAR(d);
		}
	}
	else {
		printk_callback(" %08X", x);
	}

	return 0;
}
