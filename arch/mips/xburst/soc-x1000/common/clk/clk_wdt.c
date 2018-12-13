#include <linux/spinlock.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>

#include <soc/cpm.h>
#include <soc/base.h>
#include <soc/extal.h>
#include <soc/tcu.h>
#include <jz_notifier.h>


#include "clk.h"

static DEFINE_SPINLOCK(cpm_wdt_lock);

static int cpm_wdt_enable(struct clk *clk,int on)
{
	unsigned long flags;
	volatile void * reg_addr;

	spin_lock_irqsave(&cpm_wdt_lock,flags);
	if(on) {
		reg_addr = (volatile void *)(TCU_IOBASE + TCU_TSCR + 0xa0000000);
		writel(BIT(16), reg_addr);
	} else {
		reg_addr = (volatile void *)(TCU_IOBASE + TCU_TSSR + 0xa0000000);
		writel(BIT(16), reg_addr);
	}
	spin_unlock_irqrestore(&cpm_wdt_lock,flags);
	return 0;
}
static unsigned long opcr_get_rate(struct clk *clk)
{
	unsigned long rtc_clk = 0;
	struct clk *wdt_clk = NULL;

	if(cpm_inl(CPM_OPCR) & (1 << 2)) {
		wdt_clk = clk_get(NULL, "rtc");
		if (IS_ERR(wdt_clk)) {
			printk("cannot find RTC clock\n");
			return 0;
		}
		rtc_clk = clk_get_rate(wdt_clk);
	} else {
		wdt_clk = clk_get(NULL, "ext1");
		if (IS_ERR(wdt_clk)) {
			printk("cannot find EXT1 clock\n");
			return 0;
		}
		rtc_clk = clk_get_rate(wdt_clk) / 512;
	}
	clk_put(wdt_clk);

	return rtc_clk;
}
static struct clk_ops clk_wdt_ops = {
	.enable	= cpm_wdt_enable,
	.get_rate = opcr_get_rate,
};

void __init init_wdt_clk(struct clk *clk)
{
	clk->rate = 0;
	clk->ops = &clk_wdt_ops;
}
