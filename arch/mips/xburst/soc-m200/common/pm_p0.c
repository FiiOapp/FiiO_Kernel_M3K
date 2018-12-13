/*
 * linux/arch/mips/xburst/soc-m200/common/pm_p0.c
 *
 *  M200 Power Management Routines
 *  Copyright (C) 2006 - 2012 Ingenic Semiconductor Inc.
 *
 *  This program is free software; you can distribute it and/or modify it
 *  under the terms of the GNU General Public License (Version 2) as
 *  published by the Free Software Foundation.
 *
 *  This program is distributed in the hope it will be useful, but WITHOUT
 *  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 *  for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  59 Temple Place - Suite 330, Boston MA 02111-1307, USA.
 *
 */

#include <linux/init.h>
#include <linux/pm.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/suspend.h>
#include <linux/proc_fs.h>
#include <linux/syscalls.h>
#include <linux/fs.h>
#include <linux/sysctl.h>
#include <linux/delay.h>
#include <asm/fpu.h>
#include <linux/syscore_ops.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>
#include <linux/notifier.h>
#ifdef CONFIG_CPU_FREQ
#include <linux/cpufreq.h>
#endif
#include <asm/cacheops.h>
#include <soc/cache.h>
#include <asm/r4kcache.h>
#include <soc/base.h>
#include <soc/cpm.h>
#include <soc/gpio.h>
#include <soc/ddr.h>
#include <tcsm.h>
#include <smp_cp0.h>

#include <soc/tcsm_layout.h>

#ifdef CONFIG_JZ_DMIC_WAKEUP
#include <linux/voice_wakeup_module.h>
#endif

#ifdef CONFIG_TEST_SECOND_REFRESH
#include <linux/second_refresh.h>
#endif

extern long long save_goto(unsigned int);
extern int restore_goto(void);
extern unsigned int get_pmu_slp_gpio_info(void);

extern void dump_clk(void);

extern int jz_clocks_show(void* args);

unsigned int pm_firmware_new[] ={
#include "core_sleep.hex"
};
void load_pm_firmware_new(unsigned int addr)
{
	void (*func)(unsigned int addr,unsigned int to);
	unsigned int firmware_size = sizeof(pm_firmware_new);

	if(firmware_size > TCSM_BANK_LEN * 1024)
		printk("WARN: firmware_size %d bigger than" \
		       "TCSM_BANK_LEN %d\n", firmware_size, TCSM_BANK_LEN * 1024);
	func = (void (*)(unsigned int,unsigned int))addr;
	memcpy((void *)addr,pm_firmware_new,firmware_size);
	func(addr,0);
}
struct sleep_param
{
	unsigned int  pm_core_enter;
	unsigned char pmu_i2c_scl;           //default 0xff
	unsigned char pmu_i2c_sda;           //default 0xff
	unsigned char pmu_addr;               //default 0xff
	unsigned char pmu_reg;                //default 0xff
	unsigned char pmu_register_val;

	unsigned char pmu_pin;               //default 0xff
	unsigned char pmu_pin_func;          //default 0xff
	unsigned char uart_id;          //default 0xff

	unsigned int  prev_resume_pc;  //ddr is self-reflash default 0xffffffff
	unsigned int  post_resume_pc;  //ddr is ok. default 0xffffffff
	unsigned int  prev_sleep_pc;   //after flush cache. default 0xffffffff
	unsigned int  post_sleep_pc;   //before wait. default 0xffffffff

};
struct sleep_save_register
{
	unsigned int lcr;
	unsigned int opcr;
	unsigned int spcr0;
	unsigned int gate;
	/* unsigned int sleep_voice_enable; */
	unsigned int ddr_training_space[20];
};

static struct sleep_save_register s_reg;

struct sleep_param *sleep_param;
void local_flush_tlb_all(void);
extern int rtc_is_enabled(void);

static int soc_pm_enter(suspend_state_t state)
{
	unsigned int lcr,opcr;
	unsigned int gate,spcr0;
	unsigned int core_ctrl;
	unsigned int scpu_start_addr;
	unsigned int pmu_slp_gpio_info = -1;

	memcpy(&s_reg.ddr_training_space,(void*)0x80000000,sizeof(s_reg.ddr_training_space));
	s_reg.opcr = cpm_inl(CPM_OPCR);
	s_reg.lcr = cpm_inl(CPM_LCR);
	s_reg.spcr0 = cpm_inl(CPM_SPCR0);
	s_reg.gate = cpm_inl(CPM_CLKGR);

#ifdef CONFIG_JZ_DMIC_WAKEUP
	/* if voice identified before deep sleep. just return to wakeup system. */
	if(wakeup_module_get_sleep_process() == SYS_WAKEUP_OK) {
		return 0;
	}
#endif
	disable_fpu();
#ifdef DDR_MEM_TEST
	test_ddr_data_init();
#endif

#ifdef PRINT_DEBUG
	jz_clocks_show(NULL);
#endif /* PRINT_DEBUG */
	load_pm_firmware_new(SLEEP_TCSM_SPACE);
	sleep_param = (struct sleep_param *)SLEEP_TCSM_SPACE;

	pmu_slp_gpio_info = get_pmu_slp_gpio_info();
	if(pmu_slp_gpio_info != -1) {
		sleep_param->pmu_pin = pmu_slp_gpio_info & 0xffff;
		sleep_param->pmu_pin_func = pmu_slp_gpio_info >> 16;
	}
	sleep_param->post_resume_pc = (unsigned int)restore_goto;
	sleep_param->uart_id = -1;

	/*
	 *   set OPCR.
	 */
	opcr = s_reg.opcr;
	lcr = s_reg.lcr;
	spcr0 = s_reg.lcr;
	gate = s_reg.gate;

	lcr &= ~3;
	lcr |= LCR_LPM_SLEEP;
	lcr |= (1 << 2);

	/* OPCR.MASK_INT bit30*/
	/* set Oscillator Stabilize Time bit8*/
	/* disable externel clock Oscillator in sleep mode bit4*/
	/* select 32K crystal as RTC clock in sleep mode bit2*/
	opcr &= ~((1 << 7) | (1 << 6) | (1 << 4));
	opcr |= (1<<30) | (1 << 27) | (1 << 23) | (0xff << 8)  | (1 << 2) ;

	/*
	 * set sram pdma_ds & open nfi
	 */
	spcr0 &= ~((1 << 31) | (1 << 15) | (1 << 27) | (1 << 2));

	gate &= ~(1 << 21 | 0x7);

	cpm_outl(opcr,CPM_OPCR);
	cpm_outl(lcr,CPM_LCR);
	cpm_outl(spcr0,CPM_SPCR0);
	cpm_outl(gate,CPM_CLKGR);


	core_ctrl = get_smp_ctrl();
	set_smp_ctrl(core_ctrl & ~(3 << 8));
	scpu_start_addr = get_smp_reim();

	printk("#####lcr:%08x\n", cpm_inl(CPM_LCR));
	printk("#####gate:%08x\n", cpm_inl(CPM_CLKGR));
	printk("#####gate1:%08x\n", cpm_inl(CPM_CLKGR1));
	printk("#####opcr:%08x\n", cpm_inl(CPM_OPCR));
	printk("#####cpccr:%08x\n", cpm_inl(CPM_CPCCR));
	printk("#####INT_MASK0:%08x\n", *(volatile unsigned int*)(0xB0001004));
	printk("#####INT_MASK1:%08x\n", *(volatile unsigned int*)(0xB0001024));
	printk("#####CPM_DDRCDR:0x%08x\n",cpm_inl(CPM_DDRCDR));
	printk("#####DDRC_AUTOSR_EN: %x\n",ddr_readl(DDRC_AUTOSR_EN));
	printk("#####DDRC_DLP: %x\n",ddr_readl(DDRC_DLP));
	printk("#####ddr cs %x\n",ddr_readl(DDRP_DX0GSR));

#ifdef CONFIG_JZ_DMIC_WAKEUP
	wakeup_module_open(DEEP_SLEEP);
	if(wakeup_module_is_wakeup_enabled()) {
		sleep_param->prev_sleep_pc = (unsigned int)wakeup_module_cache_prefetch;
		sleep_param->prev_resume_pc = *(unsigned int*)WAKEUP_HANDLER_ADDR;
	}
#endif

	mb();
	save_goto((unsigned int)sleep_param->pm_core_enter);
	mb();

	memcpy((void*)0x80000000,&s_reg.ddr_training_space,sizeof(s_reg.ddr_training_space));
	dma_cache_wback_inv(0x80000000,sizeof(s_reg.ddr_training_space));
	cpm_outl(s_reg.lcr,CPM_LCR);
	cpm_outl(s_reg.opcr,CPM_OPCR);
	cpm_outl(s_reg.spcr0,CPM_SPCR0);
	cpm_outl(s_reg.gate,CPM_CLKGR);

#ifdef CONFIG_JZ_DMIC_WAKEUP
	wakeup_module_close(DEEP_SLEEP);
#endif
	set_smp_ctrl(core_ctrl);
	set_smp_reim(scpu_start_addr);
	__write_32bit_c0_register($12, 7, 0);
	printk("#####cpccr:%08x\n", cpm_inl(CPM_CPCCR));

	return 0;
}

//#define SLEEP_CHANGE_CORE_VCC
static struct m200_early_sleep_t {
#ifdef SLEEP_CHANGE_CORE_VCC
	struct regulator*  core_vcc;
#endif
	struct clk *cpu_clk;
	unsigned int rate_hz;
	unsigned int real_hz;
	unsigned int vol_uv;

}m200_early_sleep;
const unsigned int sleep_rate_hz = 120*1000*1000;
#ifdef SLEEP_CHANGE_CORE_VCC
const unsigned int sleep_vol_uv = 975 * 1000;
#endif
static int soc_prepare(void)
{
#ifdef CONFIG_CPU_FREQ
	struct cpufreq_policy *policy;

	policy = cpufreq_cpu_get(0);
	if(policy) {
		policy->governor->governor(policy, CPUFREQ_GOV_STOP);
		cpufreq_cpu_put(policy);
	}
#endif
#ifdef SLEEP_CHANGE_CORE_VCC
	if(m200_early_sleep.core_vcc == NULL) {
		m200_early_sleep.core_vcc = regulator_get(NULL,"cpu_core_slp");
	}
#endif
	m200_early_sleep.rate_hz = clk_get_rate(m200_early_sleep.cpu_clk);
	clk_set_rate(m200_early_sleep.cpu_clk,sleep_rate_hz);
	m200_early_sleep.real_hz = clk_get_rate(m200_early_sleep.cpu_clk);
#ifdef SLEEP_CHANGE_CORE_VCC
	if(!IS_ERR(m200_early_sleep.core_vcc)) {
		m200_early_sleep.vol_uv = regulator_get_voltage(m200_early_sleep.core_vcc);
		regulator_set_voltage(m200_early_sleep.core_vcc,sleep_vol_uv,sleep_vol_uv);
	}
#endif
	return 0;
}
static void soc_finish(void)
{
#ifdef CONFIG_CPU_FREQ
	struct cpufreq_policy *policy;
#endif
	unsigned int rate;
	rate = clk_get_rate(m200_early_sleep.cpu_clk);
	if(rate != m200_early_sleep.real_hz) {
		printk("warn! current cpu clk %d is not deep sleep set cpu clk %d!\n",
		       rate, m200_early_sleep.real_hz);
	}
#ifdef SLEEP_CHANGE_CORE_VCC
	if(!IS_ERR(m200_early_sleep.core_vcc)) {
		regulator_set_voltage(m200_early_sleep.core_vcc,m200_early_sleep.vol_uv,m200_early_sleep.vol_uv);
	}
#endif
	clk_set_rate(m200_early_sleep.cpu_clk,m200_early_sleep.rate_hz);
#ifdef CONFIG_CPU_FREQ
	policy = cpufreq_cpu_get(0);
	if(policy) {
		policy->governor->governor(policy, CPUFREQ_GOV_START);
		cpufreq_cpu_put(policy);
	}
#endif
}
/*
 * Initialize power interface
 */
struct platform_suspend_ops pm_ops = {
	.valid = suspend_valid_only_mem,
	.enter = soc_pm_enter,
	.prepare = soc_prepare,
	.finish = soc_finish,
};
//extern void ddr_retention_exit(void);
//extern void ddr_retention_entry(void);

int __init soc_pm_init(void)
{
	volatile unsigned int lcr,opcr;//,i;

	suspend_set_ops(&pm_ops);

	/* init opcr and lcr for idle */
	lcr = cpm_inl(CPM_LCR);
	lcr &= ~(0x7);		/* LCR.SLEEP.DS=1'b0,LCR.LPM=2'b00*/
	lcr |= 0xff << 8;	/* power stable time */
	cpm_outl(lcr,CPM_LCR);

	opcr = cpm_inl(CPM_OPCR);
	opcr &= ~(2 << 25);	/* OPCR.PD=2'b00 */
	opcr |= 0xff << 8;	/* EXCLK stable time */
	cpm_outl(opcr,CPM_OPCR);

	m200_early_sleep.cpu_clk = clk_get(NULL, "cclk");
	if (IS_ERR(m200_early_sleep.cpu_clk)) {
		printk("ERROR:cclk request fail!\n");
		suspend_set_ops(NULL);
		return -1;
	}
	/* sysfs */
	return 0;
}

arch_initcall(soc_pm_init);
