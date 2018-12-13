/*
 * JZSOC Clock and Power Manager
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2006 Ingenic Semiconductor Inc.
 */
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/proc_fs.h>
#include <linux/ioport.h>
#include <linux/slab.h>

#include <asm/string.h>
#include <soc/cpm.h>
#include <soc/base.h>
#include <soc/extal.h>
#include <mach/jzcpm_pwc.h>
#include <mach/platform.h>
#include <uapi/asm/setup.h>

#define DDR_SIZE_64M	64
extern int reset_keep_power(void);
extern char __initdata arcs_cmdline[COMMAND_LINE_SIZE];
static int g_is_use_rtc;
int __init check_socid(void);
int __init soc_is_x1000e(int);

int rtc_is_enabled(void)
{
	return !g_is_use_rtc;
}

void __init cpm_reset(void)
{
	unsigned int cpm_clkgr;

	cpm_clkgr = cpm_inl(CPM_CLKGR);
	cpm_clkgr |= 0x75e3ffe;
	/* default open pdma clk gate */
	cpm_clkgr &= ~(1 << 21);
	/* default open tcu clk gate */
	cpm_clkgr &= ~(1 << 18);
#ifdef CONFIG_TIMER_SYS_OST
	cpm_clkgr &= ~(1 << 20);
#endif
	cpm_outl(cpm_clkgr, CPM_CLKGR);
	/* default close otg clk gate */
	cpm_set_bit(3, CPM_CLKGR);

	cpm_outl(0,CPM_PSWC0ST);
	cpm_outl(16,CPM_PSWC1ST);
	cpm_outl(24,CPM_PSWC2ST);
	cpm_outl(8,CPM_PSWC3ST);
}
int __init setup_init(void)
{
	cpm_reset();
	g_is_use_rtc = reset_keep_power();
	return 0;
}

void __cpuinit jzcpu_timer_setup(void);
void __cpuinit jz_clocksource_init(void);
void __init init_all_clk(void);
///* used by linux-mti code */
//int coherentio;			/* init to 0, no DMA cache coherency */
//int hw_coherentio;		/* init to 0, no HW DMA cache coherency */

static void __init ddr_param_change(char *param_addr)
{
	char param_start_buf[50] = {0};
	char *param_last_addr = NULL;
	char param_buf[100] = {0};
	param_last_addr = strchr(param_addr, 'M');
	strncpy(param_start_buf, param_addr, param_last_addr - param_addr - 2);

	sprintf(param_buf, "%s%d%s", param_start_buf,
		DDR_SIZE_64M, param_last_addr);
	strcpy(param_addr, param_buf);

}

void __init plat_mem_setup(void)
{
	/* jz mips cpu special */
	__asm__ (
		"li    $2, 0xa9000000 \n\t"
		"mtc0  $2, $5, 4      \n\t"
		"nop                  \n\t"
		::"r"(2));

	/* use IO_BASE, so that we can use phy addr on hard manual
	 * directly with in(bwlq)/out(bwlq) in io.h.
	 */
	set_io_port_base(IO_BASE);
	ioport_resource.start	= 0x00000000;
	ioport_resource.end	= 0xffffffff;
	iomem_resource.start	= 0x00000000;
	iomem_resource.end	= 0xffffffff;
	setup_init();
	init_all_clk();

	if(!soc_is_x1000e(check_socid())) {
		ddr_param_change(arcs_cmdline);
	}
	return;
}

void __init plat_time_init(void)
{
	jzcpu_timer_setup();
	jz_clocksource_init();
}
