/*
 * common.h
 */
#ifndef __COMMON_H__
#define __COMMON_H__

#include "type.h"

#define SYS_WAKEUP_OK	0x1
#define SYS_WAKEUP_FAILED 0x2
#define SYS_NEED_DATA	0x3

enum wakeup_source {
	WAKEUP_BY_OTHERS = 1,
	WAKEUP_BY_DMIC
};

/* 
 * config tcu timer wakeup when dmic working in dma mode 
 * other wise wakeuped by dmic fifo trigger.
 */
#define CONFIG_TCU_TIMER_WAKEUP

#define TCU_CHANNEL 5

/*
  0: use cpu process data in deep sleep (not complemented)
  1: use dma process data in deep sleep
 */
//#define DMIC_USE_DMA 0
#define DMIC_USE_DMA 1

enum {
	CPU_MODE,
	DMA_MODE
};

//extern int g_dma_mode;

#endif /* __COMMON_H__ */
