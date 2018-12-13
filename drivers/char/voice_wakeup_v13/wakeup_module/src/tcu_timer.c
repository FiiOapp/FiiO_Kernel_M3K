#include "interface.h"
#include "tcu_timer.h"
#include <jz_tcu.h>
#include <jz_cpm.h>
#include <common.h>

//#define TCU_VOICE_DEBUG


/*
  If x1000 rtc clock source, set 30ms, actually time is 20ms.
  If x1000 ext clock source, set 30ms, actually time is 31 ms.

  tcu use ext clk, otherwise tcu clock source use rtc clock.
*/
#define TCU_USE_EXT_CLK


static int tcu_channel = 2; /*default 2*/
unsigned int clk_enabled;
#ifdef TCU_VOICE_DEBUG
unsigned long time = 0;
#endif

#define TIME_1S		(1000)

#define CSRDIV(x)      ({int n = 0;int d = x; while(d){ d >>= 2;n++;};(n-1) << 3;})


#ifdef TCU_USE_EXT_CLK
/* ext clock source: 24M Hz */
#define TCU_CLK_FREQ (24000000)
#define CLK_DIV         1024
/* TIME_US_PER_COUNT:  1000000 us / (24000000/1024)  = 42.667 us/cycle */
#else
/* rtc clock source: 32k Hz */
#define TCU_CLK_FREQ (32768)
#define CLK_DIV         64
/*  TIME_US_PER_COUNT  1000000 us / (32768/64) = 1953.125 us/cycle */
#endif


#define US_PER_SECOND (1000000)
#define TIME_US_PER_COUNT	(US_PER_SECOND/((TCU_CLK_FREQ)/CLK_DIV))	/* 42 us */
#define TCU_TIMER_MAX_MS	((0xFFFF*(TIME_US_PER_COUNT))/1000)


unsigned int save_tcsr;

static inline void tcu_save(void)
{
	save_tcsr = tcu_readl(CH_TCSR(tcu_channel));
}
static inline void tcu_restore(void)
{
	tcu_writel(CH_TCSR(tcu_channel), save_tcsr);
}

#ifdef TCU_VOICE_DEBUG
static void tcu_dump_reg(void)
{
	/* TCU Ext clock 24M Hz, DIV1024, 60ms:
		*********** tcu_dump_reg() ***********
		TCU_TCSR:0000002c
		TCU_TDFR:00000595
		TCU_TDHR:00000001
		TCU_TCNT:00000134
		TCU_TER:00008000
		TCU_TFR:00018001
		TCU_TMR:00ff80ff
		TCU_TSR:00000000
		TCU_TSTR:00040000
	*/

	printk("*********** tcu_dump_reg() ***********\n");
	printk("TCU_TCSR:%08x\n", tcu_readl(CH_TCSR(tcu_channel)));
	printk("TCU_TDFR:%08x\n", tcu_readl(CH_TDFR(tcu_channel)));
	printk("TCU_TDHR:%08x\n", tcu_readl(CH_TDHR(tcu_channel)));
	printk("TCU_TCNT:%08x\n", tcu_readl(CH_TCNT(tcu_channel)));
	printk("TCU_TER:%08x\n", tcu_readl(TCU_TER));
	printk("TCU_TFR:%08x\n", tcu_readl(TCU_TFR));
	printk("TCU_TMR:%08x\n", tcu_readl(TCU_TMR));
	printk("TCU_TSR:%08x\n", tcu_readl(TCU_TSR));
	printk("TCU_TSTR:%08x\n", tcu_readl(TCU_TSTR));

}

static void tcu_dump_reg_hex(void)
{
	/* TCU Ext clock 24M Hz, DIV1024, 60ms:
	   [14.681352 0.062945] irp 04000000 00000000
	   G 0000002C 00000595 000002CA 0000003C 00008020 00218001 00FF80DF 00000000 00040000 H
	*/
	TCSM_PCHAR('G');
	TCSM_PCHAR(' '); serial_put_hex(tcu_readl(CH_TCSR(tcu_channel)));
	TCSM_PCHAR(' '); serial_put_hex(tcu_readl(CH_TDFR(tcu_channel)));
	TCSM_PCHAR(' '); serial_put_hex(tcu_readl(CH_TDHR(tcu_channel)));
	TCSM_PCHAR(' '); serial_put_hex(tcu_readl(CH_TCNT(tcu_channel)));
	TCSM_PCHAR(' '); serial_put_hex(tcu_readl(TCU_TER));
	TCSM_PCHAR(' '); serial_put_hex(tcu_readl(TCU_TFR));
	TCSM_PCHAR(' '); serial_put_hex(tcu_readl(TCU_TMR));
	TCSM_PCHAR(' '); serial_put_hex(tcu_readl(TCU_TSR));
	TCSM_PCHAR(' '); serial_put_hex(tcu_readl(TCU_TSTR));
	TCSM_PCHAR(' '); TCSM_PCHAR('H');
}
#endif
static inline void stop_timer()
{
	/* disable tcu n */
	tcu_writel(TCU_TECR,(1 << tcu_channel));
	tcu_writel(TCU_TFCR , (1 << tcu_channel));
}
static inline void start_timer()
{
	tcu_writel(TCU_TESR, (1 << tcu_channel));
}

static void reset_timer(int count)
{
//	unsigned int tcsr = tcu_readl(CH_TCSR(tcu_channel));

	/* set count */
	tcu_writel(CH_TDFR(tcu_channel),count);
	tcu_writel(CH_TDHR(tcu_channel),count/2);

	tcu_writel(TCU_TMCR , (1 << tcu_channel));
	start_timer();
}


void tcu_timer_del(void)
{
	tcu_writel(TCU_TMSR , (1 << tcu_channel));
	stop_timer();
#ifdef TCU_VOICE_DEBUG
	time = 0;
#endif
}


/* @fn: convert ms to count
 * @ms: input ms
 * @return: count.
 * */
unsigned long ms_to_count(unsigned long ms)
{
	unsigned long count;

	if ( ms > TCU_TIMER_MAX_MS ) {
		TCSM_PCHAR('\r');
		TCSM_PCHAR('\n');
		TCSM_PCHAR('T');
		TCSM_PCHAR('M');
		TCSM_PCHAR('M');
		TCSM_PCHAR('A');
		TCSM_PCHAR('X');
		TCSM_PCHAR(' ');
		serial_put_hex(ms);
		TCSM_PCHAR('>');

		serial_put_hex(TCU_TIMER_MAX_MS);
		TCSM_PCHAR(' ');
	}

	 /* one count is about 1.953 ms */
	count = (ms * 1000 + TIME_US_PER_COUNT - 1) / TIME_US_PER_COUNT;
	return count;
}

/* @fn: mod timer.
 * @timer_cnt: cnt to be written to register.
 * */
unsigned int tcu_timer_mod(unsigned long timer_cnt)
{
	int count;
	int current_count;
	count = timer_cnt;
	current_count = tcu_readl(CH_TCNT(tcu_channel));

	tcu_writel(CH_TCNT(tcu_channel), 0);

	tcu_writel(TCU_TSCR , (1 << tcu_channel));
	tcu_writel(TCU_TECR,(1 << tcu_channel));

	reset_timer(count);

#ifdef TCU_VOICE_DEBUG
	if(time >= TIME_1S) {
		time = 0;
		TCSM_PCHAR('.');
	}
	time += TCU_TIMER_MS;

	tcu_dump_reg_hex();
#endif
	return current_count;
}


/*
 * @fn: request a tcu channel.
 * @tcu_chan: channel to request.
 * */
void tcu_timer_request(int tcu_chan)
{
#ifdef TCU_VOICE_DEBUG
	if (1) {
		TCSM_PCHAR('\r');
		TCSM_PCHAR('\n');
		TCSM_PCHAR('t');
		TCSM_PCHAR('c');
		TCSM_PCHAR('u');
		TCSM_PCHAR('u');
		TCSM_PCHAR('s');
		TCSM_PCHAR(' ');
		serial_put_hex(TIME_US_PER_COUNT);
		TCSM_PCHAR(' ');

		serial_put_hex(TCU_TIMER_MAX_MS);
		TCSM_PCHAR(' ');
	}
#endif

	tcu_channel = tcu_chan;
	REG32(CPM_IOBASE + CPM_CLKGR0) &= ~(1<<30);
#ifdef TCU_VOICE_DEBUG
	tcu_dump_reg();
#endif
	tcu_save();
	/* stop clear */
	tcu_writel(TCU_TSCR,(1 << tcu_channel));

	tcu_writel(TCU_TECR,(1 << tcu_channel));

	tcu_writel(TCU_TMSR,(1 << tcu_channel)|(1 << (tcu_channel + 16)));

	tcu_writel(TCU_TFCR, (1 << tcu_channel) | (1 << (tcu_channel + 16)));

	tcu_writel(CH_TDHR(tcu_channel), 1);
	/* Mask interrupt */

	/* RTC CLK, 32768
	 * DIV:     64.
	 * TCOUNT:  1: 1.953125ms
	 * */
#ifdef TCU_USE_EXT_CLK
	tcu_writel(CH_TCSR(tcu_channel),CSRDIV(CLK_DIV) | CSR_EXT_EN);
#else
	tcu_writel(CH_TCSR(tcu_channel),CSRDIV(CLK_DIV) | CSR_RTC_EN);
#endif


#ifdef TCU_VOICE_DEBUG
	tcu_dump_reg();
	tcu_dump_reg_hex();
#endif

	return ;
}

/*
 * @fn: release a tcu timer. this should close tcu channel.
 * @tcu_chan: channel to release.
 * */
void tcu_timer_release(int tcu_chan)
{
	tcu_writel(TCU_TMSR , (1 << tcu_channel));
	stop_timer();
	tcu_restore();

	REG32(CPM_IOBASE + CPM_CLKGR0) |= 1<<30;
}

/*
 * @fn: tcu timer handler, called when tcu int happens. only
 *		in cpu idle mode.
 * */

void tcu_timer_handler(void)
{
	int ctrlbit = 1 << (tcu_channel);

	if(tcu_readl(TCU_TFR) & ctrlbit) {
		/* CLEAR INT */
		tcu_writel(TCU_TFCR,ctrlbit);
		tcu_timer_mod(ms_to_count(TCU_TIMER_MS));
	}
}

