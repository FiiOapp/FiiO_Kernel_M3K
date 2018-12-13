#include <jz_cpm.h>
#include <jz_gpio.h>
#include "dmic_config.h"
#include "dmic_ops.h"
#include "recognization.h"
#include "interface.h"
#include "trigger_value_adjust.h"
#include "tcu_timer.h"
#include "print.h"
#include "irq.h"
#include "mem_buffer.h"
#include "dma_ops.h"
#include "cpm_ops.h"


static int dmic_current_state;
static unsigned int cur_thr_value;
static int dmic_trigger_irq_arrival;


static int dmic_irq_handler(int irq, void * dev);


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

#endif

static void gpio_as_dmic(void)
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

static int dmic_set_channel(unsigned long channels)
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

static void dmic_init(void)
{
	gpio_as_dmic();
	//dmic_clk_config();
	cpm_config_dmic_clk();
	__dmic_reset();
	__dmic_reset_tri();
}


int dmic_close(int mode)
{

	dmic_disable();
	dma_close();

	return 0;
}

int dmic_init_normal_mode(int mode)
{

	dmic_init();

	{
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
	}


	dma_config_normal();
	dma_start();

	dmic_enable();

	return 0;
}

int dmic_init_trigger_mode(int mode)
{

	dmic_init();
	cpm_config_suspend_dmic_clk();

	dmic_current_state = WAITING_TRIGGER;
	//wakeup_failed_times = 0;
	cur_thr_value = thr_table[0];

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
	REG_DMIC_FCR = (1<<31) | 16;
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



void reconfig_thr_value(void)
{

	vtw_print(LOG_DEBUG, "reconfig_thr_value()\r\n");
	return;
#if 0
	if(dmic_current_state == WAITING_TRIGGER) {
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
		dmic_recommend_thr = adjust_trigger_value(wakeup_failed_times+1, cur_thr_value);

		wakeup_failed_times = 0;
	}
#endif
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
	vtw_print(LOG_VERBOSE, "make_fifo_empty()\r\n");

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

		vtw_print(LOG_INFO, "fifo-overflow()\t");
		serial_put_hex(fifo_state);
		vtw_print(LOG_INFO, "\r\n");
	}

	fifo_cnt <<= 1;

	read_bytes = fifo_cnt<<1; /* fifo 16bit sample data */
	if (read_bytes > size) {
		read_bytes = size;
		fifo_cnt = read_bytes>>1;
		read_bytes = fifo_cnt<<1;
	}

	data_in_word = (unsigned short *)buffer;

	/* vtw_print(LOG_DEBUG, "fifo_cnt "); */
	/* serial_put_hex(fifo_cnt); */
	/* vtw_print(LOG_DEBUG, "\r\n"); */

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
		vtw_print(LOG_INFO, "fifo-overflow()\t");
		serial_put_hex(fifo_state);
		vtw_print(LOG_INFO, "\r\n");
	}

	fifo_cnt<<=1;		/* 64bit fifo to 32bit data */

	read_bytes = fifo_cnt<<2; /* fifo 32bit sample data */
	if (read_bytes > size) {
		read_bytes = size;
		fifo_cnt = read_bytes>>2;
		read_bytes = fifo_cnt<<2;
	}

	data_in_word = (unsigned int *)buffer;

	/* vtw_print(LOG_DEBUG, "fifo_cnt "); */
	/* serial_put_hex(fifo_cnt); */
	/* vtw_print(LOG_DEBUG, "\r\n"); */

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

static int get_valid_bytes()
{
	unsigned int dma_addr;
	int nbytes;
	struct circ_buf *xfer;
	struct tx_buffer *rx_fifo;

	rx_fifo = get_rx_fifo();

	xfer = &rx_fifo->xfer;
	/* update buffer head */
	//dma_addr = pdma_trans_addr(_dma_channel, DMA_DEV_TO_MEM);
	dma_addr = dma_get_dma_address();
	xfer->head = (char *)(dma_addr | 0xA0000000) - xfer->buf;

	nbytes = CIRC_CNT(xfer->head, xfer->tail, rx_fifo->n_size);
#if 0
	vtw_print(LOG_VERBOSE, "get_valid_bytes()\t");
	vtw_print(LOG_VERBOSE, " xfer->head ");
	vtw_print_hex(LOG_VERBOSE, xfer->head);
	vtw_print(LOG_VERBOSE, " xfer->tail ");
	vtw_print_hex(LOG_VERBOSE, xfer->tail);
	vtw_print(LOG_VERBOSE, " nbytes ");
	vtw_print_hex(LOG_VERBOSE, nbytes);
	vtw_print(LOG_VERBOSE, "\r\n");
#endif
	return nbytes;
}

/*
 * return sample data length in bytes
 */
int dmic_check_and_receive_data()
{
	int length;
	/* update circle head */
	length = get_valid_bytes();

	return length;
}

/* dma loop back? */
int dmic_get_voice_data(unsigned char ** buf, int length)
{
	struct circ_buf * xfer;
	struct tx_buffer *rx_fifo;

	rx_fifo = get_rx_fifo();
	xfer = &rx_fifo->xfer;
	if ( !buf )
		return 0;

	*buf = (unsigned char *)xfer->buf + xfer->tail;
	/* if ( length == 0 ) */
	/* 	return 0; */

	/* xfer->nsize = VOICE_TCSM_DATA_BUF_SIZE */
	/* check buffer end */
	if ( xfer->tail + length > rx_fifo->n_size ) {
		//if ( xfer->head < xfer->tail ) {
		vtw_print(LOG_VERBOSE, "dmic_get_voice_data()\t");
		vtw_print(LOG_VERBOSE, " dma loop back xfer->head ");
		vtw_print_hex(LOG_VERBOSE, xfer->head);
		vtw_print(LOG_VERBOSE, " xfer->tail ");
		vtw_print_hex(LOG_VERBOSE, xfer->tail);
		vtw_print(LOG_VERBOSE, "+ length ");
		vtw_print_hex(LOG_VERBOSE, length);
		vtw_print(LOG_VERBOSE, " > MAX_BUF_SIZE ");
		vtw_print_hex(LOG_VERBOSE, rx_fifo->n_size);
		vtw_print(LOG_VERBOSE, "\r\n");

		length = CIRC_CNT_TO_END(xfer->head, xfer->tail, rx_fifo->n_size);
		vtw_print(LOG_VERBOSE, "new length ");
		vtw_print_hex(LOG_VERBOSE, length);
		vtw_print(LOG_VERBOSE, "\r\n");
	}

	/* update circle tail */
	xfer->tail += length;
	xfer->tail %= rx_fifo->n_size;

	/* vtw_print(LOG_VERBOSE, "dmic_get_voice_data()\t"); */
	/* vtw_print(LOG_VERBOSE, " xfer->tail "); */
	/* vtw_print_hex(LOG_VERBOSE, xfer->tail); */
	/* vtw_print(LOG_VERBOSE, "\r\n"); */

	return length;
}

int reset_dmic_trigger_irq_arrival_state(void)
{
	dmic_trigger_irq_arrival = 0;
	return 0;
}

int is_dmic_trigger_irq_arrival(void)
{
	return dmic_trigger_irq_arrival;
}


int is_dmic_waiting_data(void)
{
	return (dmic_current_state == WAITING_DATA);
}


static int dmic_irq_handler(int irq, void * dev)
{
	unsigned int dmic_icr;

	dmic_icr = REG_DMIC_ICR;

#if 1
	vtw_print(LOG_DEBUG, "DMIC_ICR\t");
	vtw_print_hex(LOG_DEBUG, dmic_icr);

	vtw_print(LOG_DEBUG, " DMIC_IMR\t");
	vtw_print_hex(LOG_DEBUG, REG_DMIC_IMR);
#endif

	/* wakeuped by voice trigger */
	//if ( (dmic_icr & 0x11) == 0x11) {
	if ( (dmic_icr & 0x01) ) { /* check trigger flag */

		vtw_print(LOG_INFO, "\tTRGR\r\n");
		/* turn on data path */

		/* data path always on */
		dmic_set_samplerate(16000);

		/* reset dmic */
		//__dmic_reset(); /* after reset dmic, cost 40ms waiting fifo level. */

		/* start dma */
		dma_start();

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
				vtw_print(LOG_DEBUG, "\r\nDMIC_FSR ");
				serial_put_hex(fifo_state);
			} while ( (fifo_state&0x3f) < 10 );
			vtw_print(LOG_DEBUG, "\r\n");
		}

		dmic_trigger_irq_arrival = 1;

		if( dmic_current_state == WAITING_TRIGGER) {
			dmic_current_state = WAITING_DATA;
			/*change trigger value to 0, make dmic wakeup cpu all the time*/
		}
#ifdef CONFIG_TCU_TIMER_WAKEUP
		tcu_timer_mod(ms_to_count(TCU_TIMER_MS)); /* start a timer */
#endif
	}
	else {
		unsigned int dmic_fsr;
		dmic_fsr = REG_DMIC_FSR;
		vtw_print(LOG_INFO, "DMIC_ICR\t");
		serial_put_hex(dmic_icr);
		vtw_print(LOG_INFO, "DMIC_FSR\t");
		serial_put_hex(dmic_fsr);

		if( dmic_current_state == WAITING_TRIGGER) {
			vtw_print(LOG_DEBUG, "error *** dmic_current_state == WAITING_TRIGGER\r\n");
		}
	}

	return IRQ_HANDLED;
}


int dmic_module_reset_voice_trigger(int dma)
{

	reset_dmic_trigger_irq_arrival_state();

	dmic_init_trigger_mode(DEEP_SLEEP);

	if (1 || dma == DMA_MODE) { /* DMIC_USE_DMA	dma */
		/* reset dma, stop dma first. */
		dma_stop( );
		dma_config_normal();
		mem_buffer_reset_rx_fifo();

		/* WARNING:
		 * Do not start dma before cpu resume.
		 * So move dma_start() to dmic trigger handler.
		 */
		//dma_start( );

	}

	return 0;
}


int dmic_module_suspend_enter(int use_dma)
{
	/* stop dma */
	dma_stop();

	/* save resource */

	/* request irq */
	irq_request(INTC0, INTC_DMIC, dmic_irq_handler, "dmic irq", NULL);


	return 0;
}


int dmic_module_suspend_exit(int use_dma)
{
	/* restore resource */


	/* request irq */
	irq_free(INTC0, INTC_DMIC, NULL);

	//dmic_ioctl(DMIC_IOCTL_SET_SAMPLERATE, 16000);

	dmic_disable_tri();

	/* resume dma */
	if ( !is_dmic_waiting_data() ) { /*  */
		dmic_init_normal_mode(NORMAL_RECORD);
		dma_config_normal();
		mem_buffer_reset_rx_fifo();

		dma_start();
		dmic_enable();
	}

	return 0;
}

