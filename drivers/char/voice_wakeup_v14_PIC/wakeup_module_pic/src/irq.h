#ifndef __IRQ_H__
#define __IRQ_H__


#define IRQ_HANDLED 1

typedef int (*irq_handler_t)(int, void *);

int irq_request(int group, int irq_no, irq_handler_t handler, const char *name, void * dev);
int irq_free(int group, int irq_no, void*);



/* return value: 
   0, all irq handled,
   1, there are irqs not handled;
*/
int do_irq_handler(int intc);

int intc_save(int intc);
int intc_restore(int intc);



#define INTC0 0			/* intc group 0 */
#define INTC1 1			/* intc group 0 */

#define INTC_DMIC 0
#define INTC_TCU1 26

#define INTC_RTC  0



#endif	/* __IRQ_H__ */
