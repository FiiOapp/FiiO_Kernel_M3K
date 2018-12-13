#include <common.h>
#include "interface.h"
#include "rtc_ops.h"
#include "rtc-jz.h"
#include "recognization.h"
#include "print.h"
#include "irq.h"
#include "dmic_ops.h"
#include "trigger_value_adjust.h"

#define TRIGGER_CHANGE_TIME		60*5		/*60s * 5*/

static int is_period_rtc = 0;
static int g_is_os_rtc_alarm_occur = 0;
struct rtc_config {
	unsigned long RTCCR;
	unsigned long RTCSR;
	unsigned long RTCSAR;
	unsigned long HWCR;
	//unsigned long alarm_val;
	unsigned int alarm_enabled;
	unsigned int alarm_pending;
	unsigned int alarm_int_en;
	unsigned int os_alarm_enabled;
};

static struct rtc_config old_config;
//static struct rtc_config rtc_config;


int rtc_irq_handler(int irq, void * dev);

static unsigned int jzrtc_readl(int offset)
{
	unsigned int data, timeout = 0x100000;

	do {
		data = REG32(RTC_IOBASE + offset);
	} while (REG32(RTC_IOBASE + offset) != data && timeout--);

	if (timeout <= 0)
		printk("RTC : rtc_read_reg timeout!\n");
	return data;
}

static inline void wait_write_ready()
{
	int timeout = 0x100000;

	while (!(jzrtc_readl(RTC_RTCCR) & RTCCR_WRDY) && timeout--);
	if (timeout <= 0)
		printk("RTC : %s timeout!\n",__func__);
}

static void jzrtc_writel(int offset, unsigned int value)
{
	int timeout = 0x100000;

	REG32(RTC_IOBASE + RTC_WENR) = WENR_WENPAT_WRITABLE;
	wait_write_ready();

	while (!(jzrtc_readl(RTC_WENR) & WENR_WEN) && timeout--);
	if (timeout <= 0)
		printk("RTC :  wait_writable timeout!\n");

	wait_write_ready();
	REG32(RTC_IOBASE + offset) = value;
	wait_write_ready();
}

static inline void jzrtc_clrl(int offset, unsigned int value)
{
	jzrtc_writel(offset, jzrtc_readl(offset) & ~(value));
}

static inline void jzrtc_setl(int offset, unsigned int value)
{
	jzrtc_writel(offset,jzrtc_readl(offset) | (value));
}

#ifdef RTC_VOICE_DEBUG
static void dump_rtc_regs(void)
{
	 printk("*******************************************************************\n");
	 printk("******************************jz_rtc_dump**********************\n\n");
	 printk("jz_rtc_dump-----RTC_RTCCR is --0x%08x--\n",jzrtc_readl( RTC_RTCCR));
	 printk("jz_rtc_dump-----RTC_RTCSR is --0x%08x--\n",jzrtc_readl( RTC_RTCSR));
	 printk("jz_rtc_dump-----RTC_RTCSAR is --0x%08x--\n",jzrtc_readl(RTC_RTCSAR));
	 printk("jz_rtc_dump-----RTC_RTCGR is --0x%08x--\n",jzrtc_readl( RTC_RTCGR));
	 printk("jz_rtc_dump-----RTC_HCR is --0x%08x--\n",jzrtc_readl( RTC_HCR));
	 printk("jz_rtc_dump-----RTC_HWFCR is --0x%08x--\n",jzrtc_readl( RTC_HWFCR));
	 printk("jz_rtc_dump-----RTC_HRCR is --0x%08x--\n",jzrtc_readl( RTC_HRCR));
	 printk("jz_rtc_dump-----RTC_HWCR is --0x%08x--\n",jzrtc_readl( RTC_HWCR));
	 printk("jz_rtc_dump-----RTC_HWRSR is --0x%08x--\n",jzrtc_readl(RTC_HWRSR));
	 printk("jz_rtc_dump-----RTC_HSPR is --0x%08x--\n",jzrtc_readl( RTC_HSPR));
	 printk("jz_rtc_dump-----RTC_WENR is --0x%08x--\n",jzrtc_readl( RTC_WENR));
	 printk("jz_rtc_dump-----RTC_CKPCR is --0x%08x--\n",jzrtc_readl(RTC_CKPCR));
	 printk("jz_rtc_dump-----RTC_PWRONCR is -0x%08x-\n",jzrtc_readl(RTC_PWRONCR));
	 printk("***************************jz_rtc_dump***************************\n");
	 printk("*******************************************************************\n\n");

}

static int dump_rtc_config(struct rtc_config *c)
{
	if (!c)
		return 0;

	vtw_print(LOG_WARN, "dump_rtc_config():\r\n");
	vtw_print(LOG_WARN, "\tRTCCR: "); vtw_print_hex(LOG_WARN, c->RTCCR);
	vtw_print(LOG_WARN, "\tRTCSR:");  vtw_print_hex(LOG_WARN, c->RTCSR);
	vtw_print(LOG_WARN, "\tRTCSAR:"); vtw_print_hex(LOG_WARN, c->RTCSAR);
	vtw_print(LOG_WARN, "\tHWCR: ");  vtw_print_hex(LOG_WARN, c->HWCR);
	vtw_print(LOG_WARN, "\r\n");

	return 0;
}
#endif

int rtc_save(void)
{
	unsigned int rtc_rcr;

	old_config.RTCCR = jzrtc_readl(RTC_RTCCR);
	old_config.HWCR = jzrtc_readl(RTC_HWCR);
	old_config.RTCSR = jzrtc_readl(RTC_RTCSR);
	old_config.RTCSAR = jzrtc_readl(RTC_RTCSAR);

	if(jzrtc_readl(RTC_RTCSAR) < jzrtc_readl(RTC_RTCSR)) {
		/* alarm value < current second, then systimer not set.*/
		//old_config.alarm_val = 0;
		old_config.os_alarm_enabled = 0;
	} else {
		//old_config.alarm_val = jzrtc_readl(RTC_RTCSAR);
		old_config.os_alarm_enabled = 1;
	}

	rtc_rcr = old_config.RTCCR;
	old_config.alarm_enabled  = (rtc_rcr & RTCCR_AE) ? 1 : 0;
	old_config.alarm_int_en = (rtc_rcr & RTCCR_AIE) ? 1 : 0;
	if (!old_config.alarm_enabled || !old_config.alarm_int_en) {
		old_config.os_alarm_enabled = 0;
	}
	old_config.alarm_pending = (rtc_rcr & RTCCR_AF) ? 1 : 0;

#ifdef RTC_VOICE_DEBUG
	dump_rtc_config(&old_config);
#endif
	return 0;
}

int rtc_restore(void)
{
	unsigned long val;

	jzrtc_writel(RTC_RTCSAR, old_config.RTCSAR);
	jzrtc_writel(RTC_HWCR, old_config.HWCR);

	//if ( old_config.os_alarm_enabled) {
	if (1) {
		val = jzrtc_readl(RTC_RTCCR);
		/* do not clear AF flag */
		val &= RTCCR_AF;
		val |= old_config.RTCCR;
		jzrtc_writel(RTC_RTCCR,val);
	}
	else {
	}

	return 0;
}

static int rtc_clear_alarm_flag(void)
{
	unsigned int temp;

	temp = jzrtc_readl(RTC_RTCCR);
	temp &= ~RTCCR_AF;
	jzrtc_writel(RTC_RTCCR, temp);

	return 0;
}


int rtc_set_alarm(unsigned long alarm_seconds, int is_period)
{
	unsigned int temp;

	alarm_seconds += jzrtc_readl(RTC_RTCSR);

	/* check with old_config */
	if ( old_config.os_alarm_enabled && alarm_seconds >= old_config.RTCSAR ) {
		alarm_seconds = old_config.RTCSAR;
	}

	jzrtc_writel(RTC_RTCSAR, alarm_seconds);

	temp = jzrtc_readl(RTC_RTCCR);
	temp &= ~RTCCR_AF;
	temp |= RTCCR_AIE | RTCCR_AE;
	jzrtc_writel(RTC_RTCCR, temp);

	/* RTC Alarm wakeup enable. */
	jzrtc_setl(RTC_HWCR, HWCR_EALM);

	is_period_rtc = is_period;
	return 0;
}



int rtc_init(void)
{
	g_is_os_rtc_alarm_occur = 0;
	rtc_set_alarm(ALARM_VALUE, 1);
//	jzrtc_setl(RTC_HWCR, 1);

#ifdef RTC_VOICE_DEBUG
	dump_rtc_regs();
#endif

	irq_request(INTC1, INTC_RTC, rtc_irq_handler, "rtc irq", NULL);

	return 0;
}

int rtc_exit(void)
{
	irq_free(INTC1, INTC_RTC, NULL);
	//rtc_restore();

	return 0;
}


void process_dmic_timer()
{
	reconfig_thr_value();
}

int is_os_rtc_alarm_occur(void)
{

	return g_is_os_rtc_alarm_occur;
}


int rtc_irq_handler(int irq, void * dev)
{
	unsigned long alarm_seconds;

	vtw_print(LOG_DEBUG, "rtc_irq_handler()\r\n");

	alarm_seconds = jzrtc_readl(RTC_RTCSAR);
	/* old_config */
	if ( old_config.os_alarm_enabled && alarm_seconds == old_config.RTCSAR ) {
		g_is_os_rtc_alarm_occur = 1;
		return IRQ_HANDLED;
	}

	/* clear rtc alarm flag */
	rtc_clear_alarm_flag();

	if ( 1 || is_period_rtc ) {
		rtc_set_alarm(ALARM_VALUE, 1);
	}
	else {
		/* disable rtc alarm */
	}

	return IRQ_HANDLED;
}

