#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/jz_dwc.h>
#include <soc/base.h>
#include <soc/extal.h>
#include <soc/cpm.h>

#ifndef BIT
#define BIT(nr)  (1UL << nr)
#endif

/*USB Parameter Control Register*/
#define USBPCR_USB_MODE			BIT(31)
#define USBPCR_AVLD_REG			BIT(30)
#define USBPCR_IDPULLUP_MASK_BIT	28
#define USBPCR_IDPULLUP_MASK_MSK		(0x3 << USBPCR_IDPULLUP_MASK_BIT)
#define USBPCR_IDPULLUP_OTG				(0x0 << USBPCR_IDPULLUP_MASK_BIT)
#define USBPCR_IDPULLUP_ALWAYS_SUSPEND	(0x1 << USBPCR_IDPULLUP_MASK_BIT)
#define USBPCR_IDPULLUP_ALWAYS			(0x2 << USBPCR_IDPULLUP_MASK_BIT)
#define USBPCR_INCR_MASK		BIT(27)
#define USBPCR_TXRISETUNE		BIT(26)		/*0*/
#define USBPCR_COMMONONN		BIT(25)
#define USBPCR_VBUSVLDEXT		BIT(24)
#define USBPCR_VBUSVLDEXTSEL	BIT(23)
#define USBPCR_POR_BIT			22
#define USBPCR_POR				BIT(USBPCR_POR_BIT)
#define USBPCR_SIDDQ			BIT(21)
#define USBPCR_OTG_DISABLE		BIT(20)
#define USBPCR_COMPDISTUNE_BIT	17
#define USBPCR_COMPDISTUNE_MSK	(0x7 << USBPCR_COMPDISTUNE_BIT)
#define USBPCR_COMPDISTUNE(x)   (((x) << USBPCR_COMPDISTUNE_BIT) & USBPCR_COMPDISTUNE_MSK)  /*4*/
#define USBPCR_OTGTUNE_BIT		14
#define USBPCR_OTGTUNE_MSK		(0x7 << USBPCR_OTGTUNE_BIT)
#define USBPCR_OTGTUNE(x)		(((x) << USBPCR_OTGTUNE_BIT) & USBPCR_OTGTUNE_MSK)			/*4*/
#define USBPCR_SQRXTUNE_BIT		11
#define USBPCR_SQRXTUNE_MSK		(0x7 << USBPCR_SQRXTUNE_BIT)
#define USBPCR_SQRXTUNE(x)		(((x) << USBPCR_SQRXTUNE_BIT) & USBPCR_SQRXTUNE_MSK)		/*3*/
#define USBPCR_TXFSLSTUNE_BIT	7
#define USBPCR_TXFSLSTUNE_MSK	(0xf << USBPCR_TXFSLSTUNE_BIT)
#define USBPCR_TXFSLSTUNE(x)	(((x) << USBPCR_TXFSLSTUNE_BIT) & USBPCR_TXFSLSTUNE_MSK)	/*2*/
#define USBPCR_TXPREEMPHTUNE	BIT(6)														/*0*/
#define USBPCR_TXHSXVTUNE_BIT	4
#define USBPCR_TXHSXVTUNE_MSK	(0x3 << USBPCR_TXHSXVTUNE_BIT)
#define USBPCR_TXHSXVTUNE(x)	(((x) << USBPCR_TXHSXVTUNE_BIT) & USBPCR_TXHSXVTUNE_MSK)	/*3*/
#define USBPCR_TXVREFTUNE_BIT	0
#define USBPCR_TXVREFTUNE_MSK	(0xf << USBPCR_TXVREFTUNE_BIT)
#define USBPCR_TXVREFTUNE(x)	(((x) << USBPCR_TXVREFTUNE_BIT) & USBPCR_TXVREFTUNE_MSK)    /*4*/

/*USB Reset Detect Timer Register*/
#define USBRDT_HB_MASK			BIT(26)
#define USBRDT_VBFIL_LD_EN		BIT(25)
#define USBRDT_IDDIG_EN			24
#define USBRDT_IDDIG_REG		23
#define USBRDT_USBRDT_MSK		(0x7fffff)
#define USBRDT_USBRDT(x)		((x) & USBRDT_USBRDT_MSK)

/*USB VBUS Jitter Filter Register*/
#define USBVBFIL_USBVBFIL(x)	((x) & 0xffff)
#define USBVBFIL_IDDIGFIL(x)	((x) & (0xffff << 16))

/*USB Parameter Control Register1*/
#define USBPCR1_BVLD_REG		BIT(31)
#define USBPCR1_REFCLKSEL		(0x3 << 26)
#define USBPCR1_REFCLKDIV_MSK	(0x3 << 24)
#define USBPCR1_REFCLKDIV(x)	(((x) & 0x3) << 24)
#define USBPCR1_REFCLKDIV_48M   (0x2 << 24)
#define USBPCR1_REFCLKDIV_24M	(0x1 << 24)
#define USBPCR1_REFCLKDIV_12M	(0x0 << 24)
#define USBPCR1_PORT_RST		BIT(21)
#define USBPCR1_WORD_IF_16BIT	BIT(19)

#define OPCR_SPENDN_BIT			7
#define OPCR_GATE_USBPHY_CLK_BIT	23

#define SRBC_USB_SR			12


void jz_otg_ctr_reset(void)
{
	printk ("\njz_otg_ctr_reset!!!!\n\n");

	cpm_set_bit(SRBC_USB_SR, CPM_SRBC);
	udelay(10);
	cpm_clear_bit(SRBC_USB_SR, CPM_SRBC);
}
EXPORT_SYMBOL(jz_otg_ctr_reset);

void jz_otg_phy_init(otg_mode_t mode)
{
	unsigned int usbclk_sel;
	unsigned int usbpcr1, usbrdt, usbpcr = 0;

	printk ("\njz_otg_phy_init , mode = %d!!!!\n\n", mode);

	/*phy clk*/
	cpm_clear_bit(OPCR_GATE_USBPHY_CLK_BIT, CPM_OPCR);

	/*unsuspend*/
	cpm_set_bit(OPCR_SPENDN_BIT, CPM_OPCR);
	udelay(45);

	/* fil */
	cpm_outl(0, CPM_USBVBFIL);

	/* rdt */
	usbrdt = USBRDT_USBRDT(0x96) | USBRDT_VBFIL_LD_EN;
	cpm_outl(usbrdt, CPM_USBRDT);

	/*pcr*/
	usbpcr = USBPCR_COMMONONN | USBPCR_VBUSVLDEXT | USBPCR_VBUSVLDEXTSEL |
		USBPCR_SQRXTUNE(7) | USBPCR_TXPREEMPHTUNE | USBPCR_TXHSXVTUNE(1) |
		USBPCR_TXVREFTUNE(7);
	switch (CONFIG_EXTAL_CLOCK) {
	case 12:
	case 48:
		usbclk_sel = CONFIG_EXTAL_CLOCK/24;
		break;
	default:
	case 24:
		usbclk_sel = 0x1;
		break;
	}
	usbpcr1 = USBPCR1_REFCLKSEL | USBPCR1_REFCLKDIV(usbclk_sel) | USBPCR1_WORD_IF_16BIT;
	if (mode == DEVICE_ONLY) {
		pr_info("DWC PHY IN DEVICE ONLY MODE\n");
		usbpcr1 |= USBPCR1_BVLD_REG;
		usbpcr |= USBPCR_OTG_DISABLE;
	} else {
		pr_info("DWC PHY IN OTG MODE\n");
		usbpcr |= USBPCR_USB_MODE;
	}
	cpm_outl(usbpcr, CPM_USBPCR);
	cpm_outl(usbpcr1, CPM_USBPCR1);

	cpm_set_bit(USBPCR_POR_BIT, CPM_USBPCR);
	mdelay(1);
	cpm_clear_bit(USBPCR_POR_BIT, CPM_USBPCR);
	mdelay(1);
}
EXPORT_SYMBOL(jz_otg_phy_init);

int jz_otg_phy_is_suspend(void)
{
	return (!(cpm_test_bit(7, CPM_OPCR)));
}
EXPORT_SYMBOL(jz_otg_phy_is_suspend);

void jz_otg_phy_suspend(int suspend)
{
	if (!suspend && jz_otg_phy_is_suspend()) {
		cpm_set_bit(OPCR_SPENDN_BIT, CPM_OPCR);
		udelay(45);
		printk("\nEN PHY  !!!!!\n\n");
	} else if (suspend && !jz_otg_phy_is_suspend()) {
		cpm_clear_bit(OPCR_SPENDN_BIT, CPM_OPCR);
		udelay(5);
		printk("\nDIS PHY !!!!!!\n\n");
	}
}
EXPORT_SYMBOL(jz_otg_phy_suspend);

void jz_otg_phy_powerdown(void)
{
	unsigned usbpcr = cpm_inl(CPM_USBPCR);

	printk ("\njz_otg_phy_powerdown !!!!\n\n");

	usbpcr |= (USBPCR_OTG_DISABLE| USBPCR_SIDDQ);
	cpm_outl(usbpcr, CPM_USBPCR);
	cpm_set_bit(OPCR_GATE_USBPHY_CLK_BIT, CPM_OPCR);
}
EXPORT_SYMBOL(jz_otg_phy_powerdown);

