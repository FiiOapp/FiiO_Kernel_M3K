#ifndef __MEM_BUFFER_H__
#define __MEM_BUFFER_H__

#include "circ_buf.h"


struct tx_buffer {
	struct circ_buf xfer;
	u32	n_size;
};

extern struct tx_buffer *get_rx_fifo(void);

/* ********************* recognization.c ******************* */
#define SR_IVWOBJ_SIZE (20*1024)	/* 20 KB */
#define SR_RESIDENT_RAM_SIZE (38)	/* 38 */

extern void *get_pIvwObjBuf(void);
extern void *get_nReisdentBuf(void);
extern void *get_wakeup_key_resource_address(void);
/* ********************* recognization.c ******************* */


/* ********************* dma_ops.c ************************* */
#define DMA_DESC_ADDR		(VOICE_TCSM_DMA_DESC_ADDR)
extern void * get_dma_desc_addr(void);

/* ********************* dma_ops.c ************************* */


extern int set_mem_buffer_reserved(char * mem, int size);
extern int mem_buffer_init(int mode);
extern int mem_buffer_reset_rx_fifo(void);


#endif	/*  __MEM_BUFFER_H__ */
