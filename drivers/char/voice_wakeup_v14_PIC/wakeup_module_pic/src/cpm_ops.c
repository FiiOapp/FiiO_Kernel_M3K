#include "common.h"
#include "jz_cpm.h"
#include "print.h"
#include "cpm_ops.h"



int cpm_init_save(int ch)
{
	/* save clock gate register */
	return 0;
}

int cpm_exit_restore(int ch)
{

	return 0;
}


int cpm_start_tcu_clock(int ch)
{
	REG32(CPM_IOBASE + CPM_CLKGR0) &= ~(1<<18);
	return 0;
}

int cpm_stop_tcu_clock(int ch)
{
	REG32(CPM_IOBASE + CPM_CLKGR0) |= 1<<18;
	return 0;
}

int cpm_start_dma_clock(int ch)
{
	REG32(CPM_IOBASE + CPM_CLKGR0) &= ~(1<<21); /* pdma clock gate */
	return 0;
}

int cpm_stop_dma_clock(int ch)
{
	REG32(CPM_IOBASE + CPM_CLKGR0) |= 1<<21;
	return 0;
}


/* bit17 */
int cpm_start_dmic_clock(int ch)
{
	REG32(CPM_IOBASE + CPM_CLKGR0) &= ~(1<<17); /* dmic clock gate */
	return 0;
}

int cpm_stop_dmic_clock(int ch)
{
	REG32(CPM_IOBASE + CPM_CLKGR0) |= (1<<17); /* dmic clock gate */
	return 0;
}

#ifdef DMIC_VOICE_DEBUG
static void dump_cpm_reg(void)
{
	printk("CPM_LCR:%08x\n", REG32(CPM_IOBASE + CPM_LCR));
	printk("CPM_CLKGR0:%08x\n", REG32(CPM_IOBASE + CPM_CLKGR0));
	printk("I2S_DEVCLK:%08x\n", REG32(CPM_IOBASE + CPM_I2SCDR));
}
#endif

void cpm_config_dmic_clk(void)
{
	unsigned long i2scdr;

	/*1. cgu config 24MHz, extern clock */
	/* dmic need a 24MHZ clock from cpm */
	i2scdr = 0;
	i2scdr |= 2 << 30;
	i2scdr |= ((1 << 13) | 1);
	i2scdr |= 1 << 29;
	REG32(CPM_IOBASE + CPM_I2SCDR) = i2scdr;
	REG32(CPM_IOBASE + CPM_I2SCDR1) |= 1;

	/*2.gate clk*/
	//REG32(CPM_IOBASE + CPM_CLKGR0) &= ~(1 << 17); /* clk on */
	cpm_start_dmic_clock(0);


	/*3.extern clk sleep mode enable*/
	//REG32(CPM_IOBASE + CPM_OPCR) |= 1 << 4;
	/*4. l2 cache power on */
	//REG32(CPM_IOBASE + CPM_OPCR) &= ~(1 << 25);
#ifdef DMIC_VOICE_DEBUG
	//dump_cpm_reg();
#endif
}

void cpm_config_suspend_dmic_clk(void)
{
	unsigned long opcr;
	opcr = REG32(CPM_IOBASE + CPM_OPCR);

	/*
	  [2016-08-15 16:09:01.915] cpm_config_suspend_dmic_clk() opcr: C28FFF26
	  [2016-08-15 16:09:01.919] cpm_config_suspend_dmic_clk() opcr: C28FFF36
	*/
	//vtw_print(LOG_INFO, "cpm_config_suspend_dmic_clk() opcr: ");vtw_print_hex(LOG_INFO, opcr);vtw_print(LOG_INFO, "\r\n");

	/*3.extern clk sleep mode enable*/
	opcr |= 1 << 4;

	/*4. l2 cache power on */
	//opcr &= ~(1 << 25);	/* X1000 does not have L2C_PD */

	REG32(CPM_IOBASE + CPM_OPCR) = opcr;

	//vtw_print(LOG_INFO, "cpm_config_suspend_dmic_clk() opcr: ");vtw_print_hex(LOG_INFO, opcr);vtw_print(LOG_INFO, "\r\n");
	return;
}
