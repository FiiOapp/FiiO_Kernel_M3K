#ifndef __DMA_OPS_H__
#define __DMA_OPS_H__

#include "interface.h"

#define DMA_CHANNEL     (5)
#define DMIC_REQ_TYPE	(5)

#define NR_DESC         (4)
#define DMA_DESC_SIZE	(NR_DESC * 8)
#define DMIC_RX_FIFO    (DMIC_BASE_ADDR + DMICDR)




extern void dma_init_for_dmic(void);

/*void dma_open(void);*/
void dma_close(void);
void dma_start();
void dma_stop();
void dma_config_normal(void);
void dma_config_early_sleep(struct sleep_buffer *);
void dma_set_channel(int channel);
unsigned int dma_get_dma_address(void);

void early_sleep_dma_config(unsigned char *buffer, unsigned long len);
#endif




