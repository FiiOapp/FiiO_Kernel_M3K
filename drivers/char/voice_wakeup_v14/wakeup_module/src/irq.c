#include "common.h"
#include "print.h"
#include "irq.h"

//#define DEBUG_IRQ

#ifdef DEBUG_IRQ
#define LOG_VERBOSE LOG_INFO
#define LOG_DEBUG LOG_INFO
#endif


#define INTC_GROUP_NUM 2
#define IRQ_ARRAY_SIZE 10


struct jz_irq_register {
	int group;
	int irq_no;
	irq_handler_t handler;
	char * name;
	void * dev;
} jz_irq_register_array[IRQ_ARRAY_SIZE];

unsigned long intc_mask[INTC_GROUP_NUM];



#define INTC_BASE_ADDR 0xb0001000

#define INTC_ICSR  (0x00)
#define INTC_ICMR  (0x04)
#define INTC_ICMSR (0x08)
#define INTC_ICMCR (0x0c)
#define INTC_ICPR  (0x10)



static int intc_writel(int group, unsigned int reg, unsigned int value)
{
	REG32(INTC_BASE_ADDR + group*0x20 + reg) = value;
	return 0;
}

static unsigned int intc_readl(int group, unsigned int reg)
{
	return REG32(INTC_BASE_ADDR + group*0x20 + reg);
}

#ifdef DEBUG_IRQ
void intc_dump_regs(void)
{
	vtw_print(LOG_INFO, "intc_dump_reg_hex()\r\n: ");
	vtw_print_hex(LOG_INFO, intc_readl(0, INTC_ICMR));
	vtw_print(LOG_INFO, "\t");
	vtw_print_hex(LOG_INFO, intc_readl(1, INTC_ICMR));
	vtw_print(LOG_INFO, "\r\n");

	return;
}
#endif
/*
  save INTC MASK register
 */
int intc_save(int intc)
{
	int g;
	int iii;

#ifdef DEBUG_IRQ
	intc_dump_regs();
#endif
	for (g=0; g<INTC_GROUP_NUM; g++) {
		intc_mask[g] = intc_readl(g, INTC_ICMR);
	}


	for (iii=0; iii<IRQ_ARRAY_SIZE; iii++) {
		struct jz_irq_register *irq;
		irq = &jz_irq_register_array[iii];
		irq->group = irq->irq_no = -1;
	}

	return 0;
}


int intc_restore(int intc)
{
	int g;

#ifdef DEBUG_IRQ
	intc_dump_regs();
#endif
	/* restore irq mask */
	for (g=0; g<INTC_GROUP_NUM; g++) {
		intc_writel(g, INTC_ICMSR, intc_mask[g]);
		intc_writel(g, INTC_ICMCR, ~intc_mask[g]);
	}

#ifdef DEBUG_IRQ
	intc_dump_regs();
#endif
	return 0;
}


int irq_request(int group, int irq_no, irq_handler_t handler, const char *name, void * dev)
{
	int iii;
	for (iii=0; iii<IRQ_ARRAY_SIZE; iii++) {
		struct jz_irq_register *irq;
		int ok;
		ok = 0;

		irq = &jz_irq_register_array[iii];
		if ( irq->group == group && irq->irq_no == irq_no) {
			vtw_print(LOG_INFO, "WARNING: irq already registed.\r\n");
			ok = 1;
		}

		if ( irq->group == -1 && irq->irq_no == -1) {
			ok = 1;
		}

		if (ok) {
			vtw_print(LOG_DEBUG, "irq request ok: ");
			vtw_print_hex(LOG_DEBUG, group);
			vtw_print_hex(LOG_DEBUG, irq_no);
			vtw_print(LOG_DEBUG, "\t");
			vtw_print(LOG_DEBUG, name);
			vtw_print(LOG_DEBUG, "\r\n");

			irq->group = group;
			irq->irq_no = irq_no;
			irq->handler = handler;
			irq->name = (char *)name;
			irq->dev = dev;

			/* disable irq mask */
			intc_writel(group, INTC_ICMCR, 1<<irq_no);

			return irq_no;
		}
	}

	vtw_print(LOG_ERROR, "irq register failed: ");
	vtw_print_hex(LOG_ERROR, group);
	vtw_print_hex(LOG_ERROR, irq_no);
	vtw_print(LOG_ERROR, "\r\n");

	return -1;
}


int irq_free(int group, int irq_no, void*dev)
{
	int iii;
	for (iii=0; iii<IRQ_ARRAY_SIZE; iii++) {
		struct jz_irq_register *irq;
		irq = &jz_irq_register_array[iii];
		if ( irq->group == group && irq->irq_no == irq_no) {
			vtw_print(LOG_DEBUG, "irq free ok: ");
			vtw_print_hex(LOG_DEBUG, group);
			vtw_print_hex(LOG_DEBUG, irq_no);
			vtw_print(LOG_DEBUG, "\t");
			vtw_print(LOG_DEBUG, irq->name);
			vtw_print(LOG_DEBUG, "\r\n");

			irq->group = -1;
			irq->irq_no = -1;
			irq->handler = NULL;
			irq->name = NULL;
			irq->dev = NULL;

			/* mask irq */
			intc_writel(group, INTC_ICMSR, 1<<irq_no);

			return irq_no;
		}
	}

	/* irq not registed? */
	vtw_print(LOG_INFO, "irq free failed: ");
	vtw_print_hex(LOG_INFO, group);
	vtw_print_hex(LOG_INFO, irq_no);
	vtw_print(LOG_INFO, "\t not registed?\r\n");


	return -1;
}

/* return value: 0, not handled, 1, handled; */
static int do_irq(int group, int irq_index)
{
	int iii;

	vtw_print(LOG_VERBOSE, "do_irq(): ");
	vtw_print_hex(LOG_VERBOSE, group);
	vtw_print_hex(LOG_VERBOSE, irq_index);
	vtw_print(LOG_VERBOSE, "\r\n");

	for (iii=0; iii<IRQ_ARRAY_SIZE; iii++) {
		struct jz_irq_register *irq;
		irq = &jz_irq_register_array[iii];
		if ( group == irq->group && irq_index == irq->irq_no) {
			vtw_print(LOG_VERBOSE, "do irq handler: ");
			vtw_print_hex(LOG_VERBOSE, irq_index);
			vtw_print(LOG_VERBOSE, " ");
			vtw_print(LOG_VERBOSE, irq->name);
			vtw_print(LOG_VERBOSE, "\r\n");

			/* #define IRQ_HANDLED 1 */
			/* do irq handler */
			irq->handler(irq_index, irq->dev);
			return 1;
		}
	}

	return 0;
}


/* return value: 
   0, all irq handled,
   1, there are irqs not handled;
*/
int do_irq_handler(int intc)
{
	int g, j;

	unsigned long irq_pendings[INTC_GROUP_NUM];

	vtw_print(LOG_VERBOSE, "do_irq_handler()\r\n");
	for (g=0; g<INTC_GROUP_NUM; g++) {
		unsigned long pending;

		/* irq pending */
		pending = intc_readl(g, INTC_ICPR);
		vtw_print(LOG_VERBOSE, "pending: \t");
		vtw_print_hex(LOG_VERBOSE, pending);
		vtw_print(LOG_VERBOSE, "\r\n");

		irq_pendings[g] = pending;
		for (j=0; j<32; j++) {
			if (pending & (1<<j)) {
				if ( do_irq(g,j) ) {
					/* clear irq handled */
					irq_pendings[g] &= ~(1<<j);
				}
			}
		}
	}

	/* check irq that not handled */
	for (g=0; g<INTC_GROUP_NUM; g++) {
		if ( irq_pendings[g] )
			return 1;
	}

	return 0;
}
