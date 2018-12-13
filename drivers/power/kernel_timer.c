
#include <linux/module.h>
#include <linux/timer.h>
#include <linux/jiffies.h>

 

struct timer_list mytimer;

 

static void myfunc(unsigned long data)

{

        printk("%s/n", (char *)data);

        mod_timer(&mytimer, jiffies + 2*HZ);

}

 

static int __init mytimer_init(void)

{

        setup_timer(&mytimer, myfunc, (unsigned long)"Hello, world!");

        mytimer.expires = jiffies + HZ;

        add_timer(&mytimer);

 

        return 0;

}

 

static void __exit mytimer_exit(void)

{

        del_timer(&mytimer);

}

 

module_init(mytimer_init);

module_exit(mytimer_exit); 
