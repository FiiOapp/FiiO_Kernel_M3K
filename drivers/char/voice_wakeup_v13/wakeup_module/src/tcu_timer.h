#ifndef __TCU_TIMER_H__
#define __TCU_TIMER_H__


//#define TCU_TIMER_MS	(30)  /* if x1000 rtc clock source, actually 20ms */


/*
  process_dma_data_3()
     #define PROCESS_BYTES_PER_TIME	(1024)
     1024 bytes = 512 sample.
     512 *( 1000ms / 16K) = 32 ms;
  So, set timer 40ms.
 */
#define TCU_TIMER_MS	(40) /* ext clk, actually 40ms */


extern unsigned long ms_to_count(unsigned long ms);
extern unsigned int tcu_timer_mod(unsigned long timer_cnt);
extern void tcu_timer_request(int tcu_chan);
extern void tcu_timer_release(int tcu_chan);
extern void tcu_timer_handler(void);
extern void tcu_timer_del(void);

#endif
