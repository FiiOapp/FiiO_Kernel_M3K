#include <jz_cpm.h>
#include <jz_gpio.h>
#include "dmic_config.h"
#include "dmic_ops.h"
#include "voice_wakeup.h"
#include "interface.h"
#include "trigger_value_adjust.h"
#include "tcu_timer.h"
#include "dma_ops.h"


extern unsigned int _dma_channel;
int dmic_current_state;
unsigned int cur_thr_value;
unsigned int wakeup_failed_times;
unsigned int dmic_recommend_thr;
unsigned int last_dma_count;

//#define DMIC_FIFO_THR	(48) /* 48 * 4Bytes, or 48 * 2 Bytes.*/
#define DMIC_FIFO_THR	(48) /* 48 * 4Bytes, or 48 * 2 Bytes.*/
//#define DMIC_FIFO_THR	(128)


#define __dmic_reset()									\
	do {												\
		REG_DMIC_CR0 |= (1<< DMICCR0_RESET);		\
		while (REG_DMIC_CR0 & (1 << DMICCR0_RESET));		\
	} while (0)
#define __dmic_reset_tri()			\
	do {							\
		REG_DMIC_CR0 |= (1 << DMICCR0_RESET_TRI);		\
		while(REG_DMIC_CR0 & (1<<DMICCR0_RESET_TRI));	\
	} while(0)

#ifdef DMIC_VOICE_DEBUG
void dump_dmic_regs(void)
{
	printk("REG_DMIC_CR0			%08x\n",	REG_DMIC_CR0		);
	printk("REG_DMIC_GCR            %08x\n",    REG_DMIC_GCR		);
	printk("REG_DMIC_IMR            %08x\n",    REG_DMIC_IMR		);
	printk("REG_DMIC_ICR            %08x\n",    REG_DMIC_ICR		);
	printk("REG_DMIC_TRICR          %08x\n",    REG_DMIC_TRICR	);
	printk("REG_DMIC_THRH           %08x\n",    REG_DMIC_THRH		);
	printk("REG_DMIC_THRL           %08x\n",    REG_DMIC_THRL		);
	printk("REG_DMIC_TRIMMAX        %08x\n",    REG_DMIC_TRIMMAX	);
	printk("REG_DMIC_TRINMAX        %08x\n",    REG_DMIC_TRINMAX	);
	printk("REG_DMIC_DR             %08x\n",    REG_DMIC_DR		);
	printk("REG_DMIC_FCR            %08x\n",    REG_DMIC_FCR		);
	printk("REG_DMIC_FSR            %08x\n",    REG_DMIC_FSR		);
	printk("REG_DMIC_CGDIS          %08x\n",    REG_DMIC_CGDIS	);
}


void dump_gpio()
{
	printk("REG_GPIO_PXINT(5)%08x\n", REG_GPIO_PXINT(5));
	printk("REG_GPIO_PXMASK(5):%08x\n", REG_GPIO_PXMASK(5));
	printk("REG_GPIO_PXPAT1(5):%08x\n", REG_GPIO_PXPAT1(5));
	printk("REG_GPIO_PXPAT0(5):%08x\n", REG_GPIO_PXPAT0(5));
}
void dump_cpm_reg()
{
	printk("CPM_LCR:%08x\n", REG32(CPM_IOBASE + CPM_LCR));
	printk("CPM_CLKGR0:%08x\n", REG32(CPM_IOBASE + CPM_CLKGR0));
	printk("I2S_DEVCLK:%08x\n", REG32(CPM_IOBASE + CPM_I2SCDR));
}
#endif
void dmic_clk_config(void)
{
	/*1. cgu config 24MHz , extern clock */
	/* dmic need a 24MHZ clock from cpm */
	REG32(CPM_IOBASE + CPM_I2SCDR) = 0;
	REG32(CPM_IOBASE + CPM_I2SCDR) |= 2 << 30;
	REG32(CPM_IOBASE + CPM_I2SCDR) |= ((1 << 13) | 1);
	REG32(CPM_IOBASE + CPM_I2SCDR) |= 1 << 29;
	REG32(CPM_IOBASE + CPM_I2SCDR1) |= 1;

	/*2.gate clk*/
	REG32(CPM_IOBASE + CPM_CLKGR0) &= ~(1 << 17); /* clk on */

	/*3.extern clk sleep mode enable*/
	REG32(CPM_IOBASE + CPM_OPCR) |= 1 << 4;
	/*4. l2 cache power on */
	REG32(CPM_IOBASE + CPM_OPCR) &= ~(1 << 25);
#ifdef DMIC_VOICE_DEBUG
	dump_cpm_reg();
#endif
}
void gpio_as_dmic(void)
{
	/* dmic 0, PB21,PB22, func 0*/
	REG_GPIO_PXINTC(1)	|= 3 << 21;
	REG_GPIO_PXMASKC(1) |= 3 << 21;
	REG_GPIO_PXPAT1C(1) |= 3 << 21;
	REG_GPIO_PXPAT0C(1) |= 3 << 21;

	/* dmic 1, PB05 func 1*/
	REG_GPIO_PXINTC(1)	|= 1 << 5;
	REG_GPIO_PXMASKC(1) |= 1 << 5;
	REG_GPIO_PXPAT1C(1) |= 1 << 5;
	REG_GPIO_PXPAT0S(1) |= 1 << 5;
#ifdef DMIC_VOICE_DEBUG
	dump_gpio();
#endif
}

int dmic_set_channel(unsigned long channels)
{
	int ret;
	if(channels > 0 && channels < 4) {
		REG_DMIC_CR0 &= ~(0x7<<16);
		REG_DMIC_CR0 |= (--channels) << 16;
		ret = 0;
	} else {
		printk("err channel num :%d\n", channels);
		ret = -1;
	}

	return ret;

}
void dmic_init(void)
{
	gpio_as_dmic();
	dmic_clk_config();
	__dmic_reset();
	__dmic_reset_tri();
}

int cpu_should_sleep(void)
{
	return ((REG_DMIC_FSR & 0x7f) < (DMIC_FIFO_THR - 10)) &&(last_dma_count != REG_DMADTC(_dma_channel)) ? 1 : 0;
}

int dmic_init_mode(int mode)
{

	dmic_init();

	switch (mode) {
		case EARLY_SLEEP:

			break;
		case DEEP_SLEEP:
			dmic_current_state = WAITING_TRIGGER;
			wakeup_failed_times = 0;
			cur_thr_value = thr_table[0];
			dmic_recommend_thr = 0;

			REG_DMIC_CR0 = 0;
			REG_DMIC_CR0 |= 1<<6;
			REG_DMIC_CR0 |= 0 << 16; /* mono 0 channel */
#if DMIC_FIFO_BIT_WIDTH == FIFO_32BIT
			/*PACK_EN, UNPACK_DIS, UNPACK_MSB*/
			REG_DMIC_CR0 |= (1<<13) | (1<<12) | (1<<8); /* 32 bit data port(2x16bit) */
#else
			REG_DMIC_CR0 |= (1<<8); /* data 16 bit once */
#endif
			REG_DMIC_CR0 |= 1<<2; /*hpf1en*/

			dmic_set_channel(1); /* mono channel */

#if DMIC_USE_DMA
			/* dma burst 128bytes = 32 DATA (32bit) = 16 fifo (64bit) */
			REG_DMIC_FCR = (1<<31) | 48;
#else
			REG_DMIC_FCR = 48;
#endif
			REG_DMIC_IMR |= 0x3f; /*mask all ints*/
			REG_DMIC_GCR = 10;


			REG_DMIC_CR0 |= 1<<1; /*ENABLE DMIC tri*/
			REG_DMIC_ICR |= 0x3f; /*clear all ints*/
			REG_DMIC_IMR &= ~(1 << 0);/*enable tri int*/
			REG_DMIC_IMR &= ~(1 << 4);/*enable tri int*/

			//dmic_set_samplerate(16000);
			REG_DMIC_CR0 |= 1<<6; /* 16k samplerate */

			/* ******************************************** */
			/* trigger application path */

			__dmic_reset();
			//__dmic_reset_tri();

			dmic_enable();

			/* trigger configuration */
			REG_DMIC_THRL = cur_thr_value;/*SET A MIDDLE THR_VALUE, AS INIT */
			REG_DMIC_TRICR = 0; /* clear first */
			REG_DMIC_TRICR	|= 1 <<3; /*hpf2 en*/
			REG_DMIC_TRICR	|= 2 << 1; /* prefetch 16k*/
			//REG_DMIC_TRICR	|= 1 << 1; /* prefetch 8k */
			//REG_DMIC_TRICR	|= 0 << 1; /* disable prefetch */

			//REG_DMIC_TRICR &= ~(0xf << 16);
			REG_DMIC_TRICR |= 2<<16; /* trigger mode 2 */
			REG_DMIC_TRINMAX = 5;
			REG_DMIC_TRIMMAX = 3000000;

			/* disable data path */
			REG_DMIC_CR0 |= (3<<6);
			/* data path always on ???*/

			/* reset */
			__dmic_reset();
			__dmic_reset_tri();


			break;
		case NORMAL_RECORD:
			REG_DMIC_CR0 = 0;
			REG_DMIC_CR0 |= 1<<6;
			/* channel = 1 */
			REG_DMIC_CR0 |= 0 << 16; /* mono 0 channel */
#if DMIC_FIFO_BIT_WIDTH == FIFO_32BIT
			/*PACK_EN, UNPACK_DIS, UNPACK_MSB*/
			REG_DMIC_CR0 |= (1<<13) | (1<<12) | (1<<8); /* 32 bit data port(2x16bit) */
#else
			REG_DMIC_CR0 |= (1<<8); /* data 16 bit once */
#endif

			/* dma burst 128bytes = 32 DATA (32bit) = 16 fifo (64bit) */
#if DMIC_FIFO_BIT_WIDTH == FIFO_32BIT
			REG_DMIC_FCR = (1<<31) | 16;
#else
			REG_DMIC_FCR = (1<<31) | 32;
#endif

			//REG_DMIC_IMR &= ~(0x1f);
			REG_DMIC_IMR |= 0x3f; /*mask all ints*/
			REG_DMIC_ICR |= 0x3f; /*mask all ints*/
			REG_DMIC_GCR = 9;

			REG_DMIC_CR0 |= 1<<2; /*hpf1en*/

			REG_DMIC_TRICR = 0; /* clear first */
			REG_DMIC_TRICR &= ~(0xf << 16);
			REG_DMIC_TRICR |= 0 << 16; /*trigger mode*/
			REG_DMIC_TRICR	|= 1 <<3; /*hpf2 en*/
			REG_DMIC_TRICR	|= 2 << 1; /* prefetch 16k*/
			dmic_set_channel(1);

			break;
		default:
			break;

	}

	return 0;
}

int dmic_enable(void)
{
	REG_DMIC_CR0 |= 1 << 0; /*ENABLE DMIC*/
#ifdef DMIC_VOICE_DEBUG
	dump_dmic_regs();
#endif
	return 0;
}

int dmic_disable_tri(void)
{
	REG_DMIC_CR0 &= ~(1<<1);
	REG_DMIC_ICR |= 0x3f;
	REG_DMIC_IMR |= 0x3f;

	return 0;
}
int dmic_disable(void)
{
	REG_DMIC_CR0 &= ~(1 << 0); /* DISABLE DMIC */
	return 0;
}



void reconfig_thr_value()
{

	TCSM_PCHAR('\r');
	TCSM_PCHAR('\n');
	TCSM_PCHAR('t');
	TCSM_PCHAR('h');
	TCSM_PCHAR('r');
	TCSM_PCHAR(' ');
	serial_put_hex(dmic_current_state);
	TCSM_PCHAR(' ');
	serial_put_hex(dmic_recommend_thr);
	if(dmic_current_state == WAITING_TRIGGER) {
		TCSM_PCHAR('T');
		TCSM_PCHAR('T');
		/* called by rtc timer, or last wakeup failed .*/
		if(dmic_recommend_thr != 0) {
			cur_thr_value = dmic_recommend_thr;
			REG_DMIC_THRL = cur_thr_value;
			dmic_recommend_thr = 0;
		} else {
			REG_DMIC_THRL = cur_thr_value;
		}
	} else if(dmic_current_state == WAITING_DATA) {
		/* called only by rtc timer, we can not change thr here.
		 * should wait dmic waiting trigger state.
		 * */
		TCSM_PCHAR('T');
		TCSM_PCHAR('D');
		dmic_recommend_thr = adjust_trigger_value(wakeup_failed_times+1, cur_thr_value);

		wakeup_failed_times = 0;
	}
	TCSM_PCHAR(' ');
}

int dmic_set_samplerate(unsigned long rate)
{
	int ret;
	if(rate == 8000) {
		REG_DMIC_CR0 &= ~(3<<6);
		ret = 0;
	} else if(rate == 16000) {
		REG_DMIC_CR0 &= ~(3<<6);
		REG_DMIC_CR0 |= (1<<6);
		ret = 0;
	} else {
		printk("dmic does not support samplerate:%d\n", rate);
		ret = -1;
	}

	return ret;

}
int dmic_ioctl(int cmd, unsigned long args)
{
	switch (cmd) {
		case DMIC_IOCTL_SET_SAMPLERATE:
			dmic_set_samplerate(args);
			break;
		case DMIC_IOCTL_SET_CHANNEL:
			dmic_set_channel(args);
			break;
		default:
			break;
	}

	return 0;
}

int dmic_make_fifo_empty(void)
{

	TCSM_PCHAR('f');
	TCSM_PCHAR('i');
	TCSM_PCHAR('f');
	TCSM_PCHAR('o');
	TCSM_PCHAR('-');
	TCSM_PCHAR('E');
	TCSM_PCHAR('M');
	TCSM_PCHAR('P');
	TCSM_PCHAR('T');
	TCSM_PCHAR('\r');
	TCSM_PCHAR('\n');

	while ( REG_DMIC_FSR & 0X3F ) {
		volatile unsigned int data;
		data = REG_DMIC_DR;
		/* warning: variable 'data' set but not used [-Wunused-but-set-variable] */
		data = data;
	}

	return 0;
}



int dmic_store_data_from_16bit_fifo_to_memory(char * buffer, int size)
{
	int read_bytes;
	int fifo_cnt;
	int iii;
	/* unsigned int fifo_control; */
	unsigned int fifo_state;
	unsigned short * data_in_word; /* 16bit data */

#if 0
	/* fifo controll */
	fifo_control = REG_DMIC_FCR;
	TCSM_PCHAR('C');
	serial_put_hex(fifo_control);
#endif
	/* fifo state */
	fifo_state = REG_DMIC_FSR;
	/* TCSM_PCHAR('S'); */
	/* serial_put_hex(fifo_state); */
	/* one channel, 16bit */

	fifo_cnt = fifo_state & 0x3f;

	/* overflow check, max 0x3B? */
	if (fifo_cnt > 0x30 || (fifo_state&(1<<19))) { /* bit-19 */
		/* fifo-over 0004003B */
		TCSM_PCHAR('f');
		TCSM_PCHAR('i');
		TCSM_PCHAR('f');
		TCSM_PCHAR('o');
		TCSM_PCHAR('-');
		TCSM_PCHAR('o');
		TCSM_PCHAR('v');
		TCSM_PCHAR('e');
		TCSM_PCHAR('r');
		TCSM_PCHAR(' ');
		serial_put_hex(fifo_state);
		TCSM_PCHAR('\r');
		TCSM_PCHAR('\n');
	}

	fifo_cnt <<= 1;

	read_bytes = fifo_cnt<<1; /* fifo 16bit sample data */
	if (read_bytes > size) {
		read_bytes = size;
		fifo_cnt = read_bytes>>1;
		read_bytes = fifo_cnt<<1;
	}

	data_in_word = (unsigned short *)buffer;

	/* TCSM_PCHAR(' '); */
	/* TCSM_PCHAR('r'); */
	/* TCSM_PCHAR('c'); */
	/* TCSM_PCHAR('n'); */
	/* TCSM_PCHAR('t'); */
	/* serial_put_hex(fifo_cnt); */
	/* TCSM_PCHAR('\r'); TCSM_PCHAR('\n'); */

	/* one channel, 16bit */
	for (iii=0; iii<fifo_cnt; iii++) {
		/* bytes order, endian? */
		*data_in_word = REG_DMIC_DR;
		/* TCSM_PCHAR('\t'); serial_put_hex(*data_in_word); */
		/* TCSM_PCHAR('\r'); TCSM_PCHAR('\n'); */
		data_in_word++;
	}

#if 0
	TCSM_PCHAR(' ');
	TCSM_PCHAR('I');
	TCSM_PCHAR('C');
	TCSM_PCHAR('R');
	serial_put_hex(REG_DMIC_ICR);
	if (1) {
	TCSM_PCHAR('\r');
	TCSM_PCHAR('\n');
		fifo_cnt = 10;
		while (fifo_cnt--) {
			/* fifo state */
			fifo_state = REG_DMIC_FSR;
			TCSM_PCHAR('S');
			serial_put_hex(fifo_state);
			TCSM_PCHAR('\r');
			TCSM_PCHAR('\n');
		}
		TCSM_PCHAR('\r');
		TCSM_PCHAR('\n');
	}

	TCSM_PCHAR(' ');
	TCSM_PCHAR('I');
	TCSM_PCHAR('C');
	TCSM_PCHAR('R');
	serial_put_hex(REG_DMIC_ICR);

	TCSM_PCHAR(' ');
	TCSM_PCHAR('I');
	TCSM_PCHAR('M');
	TCSM_PCHAR('R');
	serial_put_hex(REG_DMIC_IMR);

#endif
	return read_bytes;
}

int dmic_store_data_from_32bit_fifo_to_memory(char * buffer, int size)
{
	int read_bytes;
	int fifo_cnt;
	int iii;
	/* unsigned int fifo_control; */
	unsigned int fifo_state;
	unsigned int * data_in_word; /* 16bit data */

#if 0
	/* fifo controll */
	fifo_control = REG_DMIC_FCR;
	TCSM_PCHAR('C');
	serial_put_hex(fifo_control);
#endif
	/* fifo state */
	fifo_state = REG_DMIC_FSR;
	/* TCSM_PCHAR('S'); */
	/* serial_put_hex(fifo_state); */
	/* one channel, 16bit */

	fifo_cnt = fifo_state & 0x3f;

	/* overflow check, max 0x3B? */
	if (fifo_cnt > 0x30 || (fifo_state&(1<<19))) { /* bit-19 */
		/* fifo-over 0004003B */
		TCSM_PCHAR('f');
		TCSM_PCHAR('i');
		TCSM_PCHAR('f');
		TCSM_PCHAR('o');
		TCSM_PCHAR('-');
		TCSM_PCHAR('o');
		TCSM_PCHAR('v');
		TCSM_PCHAR('e');
		TCSM_PCHAR('r');
		TCSM_PCHAR(' ');
		serial_put_hex(fifo_state);
		TCSM_PCHAR('\r');
		TCSM_PCHAR('\n');
	}

	fifo_cnt<<=1;		/* 64bit fifo to 32bit data */

	read_bytes = fifo_cnt<<2; /* fifo 32bit sample data */
	if (read_bytes > size) {
		read_bytes = size;
		fifo_cnt = read_bytes>>2;
		read_bytes = fifo_cnt<<2;
	}

	data_in_word = (unsigned int *)buffer;

	/* TCSM_PCHAR(' '); */
	/* TCSM_PCHAR('r'); */
	/* TCSM_PCHAR('c'); */
	/* TCSM_PCHAR('n'); */
	/* TCSM_PCHAR('t'); */
	/* serial_put_hex(fifo_cnt); */
	/* TCSM_PCHAR('\r'); TCSM_PCHAR('\n'); */

	/* one channel, 16bit */
	for (iii=0; iii<fifo_cnt; iii++) {
		unsigned int data;
		/* bytes order, endian? */
		data = REG_DMIC_DR;
		//data = ((data&0x0000ffff) <<16) | ((data &0xffff0000)>>16);
		/* TCSM_PCHAR('\t'); serial_put_hex(data); */
		/* TCSM_PCHAR('\r'); TCSM_PCHAR('\n'); */

		*data_in_word = data;
		data_in_word++;
	}

#if 0
	TCSM_PCHAR(' ');
	TCSM_PCHAR('I');
	TCSM_PCHAR('C');
	TCSM_PCHAR('R');
	serial_put_hex(REG_DMIC_ICR);
	if (1) {
	TCSM_PCHAR('\r');
	TCSM_PCHAR('\n');
		fifo_cnt = 10;
		while (fifo_cnt--) {
			/* fifo state */
			fifo_state = REG_DMIC_FSR;
			TCSM_PCHAR('S');
			serial_put_hex(fifo_state);
			TCSM_PCHAR('\r');
			TCSM_PCHAR('\n');
		}
		TCSM_PCHAR('\r');
		TCSM_PCHAR('\n');
	}

	TCSM_PCHAR(' ');
	TCSM_PCHAR('I');
	TCSM_PCHAR('C');
	TCSM_PCHAR('R');
	serial_put_hex(REG_DMIC_ICR);

	TCSM_PCHAR(' ');
	TCSM_PCHAR('I');
	TCSM_PCHAR('M');
	TCSM_PCHAR('R');
	serial_put_hex(REG_DMIC_IMR);

#endif
	return read_bytes;
}

int dmic_store_data_from_fifo_to_memory(char * buffer, int size)
{
#if DMIC_FIFO_BIT_WIDTH == FIFO_32BIT
	/* 32bit fifo */
	return dmic_store_data_from_32bit_fifo_to_memory(buffer, size);
#else
	/* 16bit fifo */
	return dmic_store_data_from_16bit_fifo_to_memory(buffer, size);
#endif
}




#define is_int_rtc(in)		(in & (1 << 0))

int dmic_handler(int pre_ints)
{
	volatile int ret;
	unsigned int dmic_icr;

	dmic_icr = REG_DMIC_ICR;
#if 0
	TCSM_PCHAR(' ');
	TCSM_PCHAR('I');
	TCSM_PCHAR('C');
	TCSM_PCHAR('R');
	serial_put_hex(dmic_icr);

	TCSM_PCHAR(' ');
	TCSM_PCHAR('I');
	TCSM_PCHAR('M');
	TCSM_PCHAR('R');
	serial_put_hex(REG_DMIC_IMR);
	TCSM_PCHAR('\t');
#endif
	//TCSM_PCHAR(' ');

	/* wakeuped by voice trigger */
	//if ( (dmic_icr & 0x11) == 0x11) {
	if ( (dmic_icr & 0x01) ) { /* check trigger flag */
		TCSM_PCHAR(' ');
		TCSM_PCHAR('T');
		TCSM_PCHAR('R');
		TCSM_PCHAR('G');
		TCSM_PCHAR(' ');
		/* turn on data path */

		/* data path always on */
		dmic_set_samplerate(16000);

		/* reset dmic */
		//__dmic_reset(); /* after reset dmic, cost 40ms waiting fifo level. */

		/* start dma */
		dma_start(_dma_channel);

		REG_DMIC_CR0 &= ~(1<<1);     /* disable trigger function */

		/* use 40ms timer wakeup cpu, so mask all dmic irqs */

		//REG_DMIC_IMR |= 1<<0 | 1<<4; /* mask wakeup ints and trigger ints */
		REG_DMIC_IMR = 0x3f; /* mask all ints */

		REG_DMIC_ICR = 0x3f; /* clear irq flags */

		/* wait fifo num > 0 */
		if (0) {
			unsigned int fifo_state;
			do {
				fifo_state = REG_DMIC_FSR;
				TCSM_PCHAR('\r');
				TCSM_PCHAR('\n');
				TCSM_PCHAR(' ');
				TCSM_PCHAR('S');
				serial_put_hex(fifo_state);
			} while ( (fifo_state&0x3f) < 10 );
			TCSM_PCHAR('\r');
			TCSM_PCHAR('\n');
		}

		if( dmic_current_state == WAITING_TRIGGER) {
			dmic_current_state = WAITING_DATA;
			/*change trigger value to 0, make dmic wakeup cpu all the time*/
		}
#ifdef CONFIG_TCU_TIMER_WAKEUP
		tcu_timer_mod(ms_to_count(TCU_TIMER_MS)); /* start a timer */
#endif
	}
	else {
		if( dmic_current_state == WAITING_TRIGGER) {
			return SYS_WAKEUP_FAILED;
		}
	}

	last_dma_count = REG_DMADTC(_dma_channel);


	ret = process_dma_data_3();
	/* ret = process_dma_data_2(); */

	if(ret == SYS_WAKEUP_OK) {
		REG_DMIC_IMR = 0x3f; /* mask all ints */
		REG_DMIC_ICR = 0x3f; /* clear irq flags */

		return SYS_WAKEUP_OK;

	} else if(ret == SYS_NEED_DATA) {
#ifndef CONFIG_TCU_TIMER_WAKEUP
		/* wakeup cpu by fifo trigger */
		REG_DMIC_IMR &= ~( 1<<5); /* fifo trigger irq */
		REG_DMIC_ICR = 0x3f; /* clear irq flags */
#endif
		last_dma_count = REG_DMADTC(_dma_channel);
		return SYS_NEED_DATA;
	} else if( ret == SYS_WAKEUP_FAILED) {
		/*
		 * if current wakeup operation failed. we need reconfig dmic
		 * to work at appropriate mode.
		 * */
		dmic_current_state = WAITING_TRIGGER;
		wakeup_failed_times++;
		TCSM_PCHAR(' ');
		TCSM_PCHAR('F');
		TCSM_PCHAR('A');
		TCSM_PCHAR('I');
		TCSM_PCHAR('L');
		TCSM_PCHAR('E');
		TCSM_PCHAR('D');
		TCSM_PCHAR(' ');

		/* ******************************************** */
		/* trigger application path */

		/* why voice wakeup failed when deep sleep again??? */
		if (1) {
			//dmic_init_mode(DEEP_SLEEP);
		} else {
			/* ******************************************** */
			/* trigger application path */
			__dmic_reset();

			REG_DMIC_CR0 |= 3<<6; /* disable data path*/

			reconfig_thr_value();

			/* change trigger mode to > N times*/
			//REG_DMIC_TRICR |= 2 << 16;
			REG_DMIC_TRINMAX = 5;

			REG_DMIC_TRICR |= 1<<0; /*clear trigger and reset trigger */

			REG_DMIC_ICR |= 0x3f;
			REG_DMIC_IMR &= ~(1<<0 | 1<<4); /* enable wakeup irq */
		}
		return SYS_WAKEUP_FAILED;
	}

	return SYS_WAKEUP_FAILED;
}



int dmic_handler_cpu_mode(int pre_ints)
{
	volatile int ret;
	unsigned int dmic_icr;

	dmic_icr = REG_DMIC_ICR;

	/* TCSM_PCHAR(' '); */
	/* TCSM_PCHAR('I'); */
	/* TCSM_PCHAR('C'); */
	/* TCSM_PCHAR('R'); */
	/* serial_put_hex(dmic_icr); */

	/* TCSM_PCHAR(' '); */
	/* TCSM_PCHAR('I'); */
	/* TCSM_PCHAR('M'); */
	/* TCSM_PCHAR('R'); */
	/* serial_put_hex(REG_DMIC_IMR); */

	/* wrong intterrupt */
	if ( (dmic_icr & 0xff) == 0x37) {
		TCSM_PCHAR(' '); TCSM_PCHAR('F'); TCSM_PCHAR('S'); TCSM_PCHAR('R'); serial_put_hex(REG_DMIC_FSR);
		REG_DMIC_ICR |= 0x3f;
		return SYS_WAKEUP_FAILED;
	}

	//TCSM_PCHAR(' ');
	/* wakeuped by voice trigger */
	if ( (dmic_icr & 0x11) == 0x11) {
		TCSM_PCHAR('1');
		TCSM_PCHAR('1');
		TCSM_PCHAR(' ');
		/* turn on data path */

		//REG_DMIC_CR0 |= 3<<6; /* disable data path */
		dmic_set_samplerate(16000);

		/* reset dmic */
		//__dmic_reset();

		//REG_DMIC_IMR |= 1<<0 | 1<<4; /* mask wakeup ints and trigger ints */

		REG_DMIC_IMR = 0x3f; /* mask wakeup ints and trigger ints */

		REG_DMIC_CR0 &= ~(1<<1);     /* disable trigger function */

		REG_DMIC_ICR = 0xff;

		//dmic_enable();
	}

	/* wait fifo num > 0 */
	if (0) {
		unsigned int fifo_state;
		do {
			fifo_state = REG_DMIC_FSR;
			TCSM_PCHAR(' ');
			TCSM_PCHAR('S');
			serial_put_hex(fifo_state);
		} while ( (fifo_state&0x3f) < 1 );
	}




	//REG_DMIC_ICR |= 0x3f;

	if(0 && dmic_current_state == WAITING_TRIGGER) {
		if(is_int_rtc(pre_ints)) {
//			TCSM_PCHAR('F');
			REG_DMIC_ICR |= 0x3f;
			REG_DMIC_IMR &= ~(1<<0 | 1<<4);
			return SYS_WAKEUP_FAILED;
		}

		dmic_current_state = WAITING_DATA;
		/*change trigger value to 0, make dmic wakeup cpu all the time*/
#ifdef CONFIG_TCU_TIMER_WAKEUP
		tcu_timer_mod(ms_to_count(TCU_TIMER_MS)); /* start a timer */
#else
		REG_DMIC_THRL = 0;
#endif
	} else if (dmic_current_state == WAITING_DATA){

	}

	/* TCSM_PCHAR('\r'); */
	/* TCSM_PCHAR('\n'); */
	/* TCSM_PCHAR('C'); */
	/* TCSM_PCHAR('P'); */
	/* TCSM_PCHAR('U'); */
	ret = voice_wakeup_process_data_cpu_mode(NULL);
	/* TCSM_PCHAR('\r'); */
	/* TCSM_PCHAR('\n'); */

	/* TCSM_PCHAR(' '); */
	/* TCSM_PCHAR('I'); */
	/* TCSM_PCHAR('C'); */
	/* TCSM_PCHAR('R'); */
	/* serial_put_hex(REG_DMIC_ICR); */


	/* dmic_make_fifo_empty(); */


	//REG_DMIC_IMR = 0x11;
	//REG_DMIC_ICR |= 0x3f;

	/* TCSM_PCHAR(' '); */
	/* TCSM_PCHAR('I'); */
	/* TCSM_PCHAR('M'); */
	/* TCSM_PCHAR('R'); */
	/* serial_put_hex(REG_DMIC_IMR); */
	/* TCSM_PCHAR('\r'); */
	/* TCSM_PCHAR('\n'); */


	/* *************************** */
	/* *************************** */
	/* *************************** */
	return ret;
	/* *************************** */
	/* *************************** */
	/* *************************** */
	/* *************************** */

}
