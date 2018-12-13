#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <mach/jz_efuse.h>

#define	EFUSE_IOBASE	(volatile void *)0xb3540000
#define CPM_IOBASE	(volatile void *)0xb0000000
#define CLKGATE		0x20
#define CTRL_OFF	0x0
#define CFG_OFF		0x4
#define STATUS_OFF	0x8
#define DATA0_OFF	0xc

enum socid {
	X1000 = 0xff00,
	X1000E = 0xff01,
	X1500 = 0xff02,
	X1000_NEW = 0xff08,
	X1000E_NEW = 0xff09,
	X1500_NEW = 0xff0a,
};

static uint32_t efuse_read(int off) {

	return readl(EFUSE_IOBASE + off);
}

static void efuse_write(uint32_t val, int off) {

	writel(val, EFUSE_IOBASE + off);
}

static uint32_t cpm_read(int off) {

	return readl(CPM_IOBASE + off);
}

static void cpm_write(uint32_t val, int off) {

	writel(val, CPM_IOBASE + off);
}

static int read_socid(void) {

	uint32_t val = 0, timeout = 10;
	uint32_t clkgat = 0;
	int ret = 0;

	clkgat = cpm_read(CLKGATE);
	cpm_write(clkgat & ~(0x1 << 1), CLKGATE);

	efuse_write(0, STATUS_OFF);
	/*efuse ctrl*/
	val = 0x3c << 21 | 1 << 16 | 1;
	efuse_write(val, CTRL_OFF);

	while(!(efuse_read(STATUS_OFF) & 1)) {
		timeout --;
		if(!timeout) {
			ret = -EBUSY;
			goto efuse_fail;
		}
	}

	ret = efuse_read(DATA0_OFF);

efuse_fail:
	cpm_write(clkgat, CLKGATE);
	return ret;
}

int __init check_socid(void) {

	int socid = read_socid();
	if (socid < 0) {
		pr_err("socid: efuse busy !\n");
		return -EBUSY;
	}
	switch(socid) {
	    case X1000:
	    case X1500:
	    case X1000E:
	    case X1000_NEW:
	    case X1000E_NEW:
	    case X1500_NEW:
	    case 0:
		break;
	    default:
		pr_err("socid: unknown x1000 socid !\n");
		return -ENODEV;
	}

	return socid;
}

int __init soc_is_x1000e(int socid) {

	if(socid == X1000E || socid == X1000E_NEW)
		return 0;
	else
		return -1;
}

MODULE_DESCRIPTION("x1000 socid driver special used by itself");
