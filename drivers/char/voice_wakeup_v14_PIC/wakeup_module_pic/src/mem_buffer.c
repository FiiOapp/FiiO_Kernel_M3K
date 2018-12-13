#include <tcsm_layout.h>
#include "common.h"
#include "print.h"
#include "mem_buffer.h"


/* ********************************************************* */
/* ********************* recognization.c ******************* */

/* target.ld
 * 96Kbytes: .resource.
 * voice_res(rw): ORIGIN = 0x81F28000, LENGTH = 0x18000
 */
//static unsigned int __res_mem mem_buffer_reserved[] = {
//static char * __res_mem mem_buffer_reserved[] = {
	//#include "ivModel_v21.data"
//};

char * mem_buffer_reserved;
int mem_buffer_size;


struct mem_buffer {
	void * wakeup_key_resource;
	unsigned char * pIvwObjBuf;	/* __attribute__((aligned(32))) */
	unsigned char * nReisdentBuf;	/* __attribute__((aligned(32))) */
	struct tx_buffer rx_buffer;
	void *dma_desc_addr;
} g_mem_buffer_alloc[1];

static struct mem_buffer * g_mem;
/* ********************* recognization.c ******************* */

int set_mem_buffer_reserved(char * mem, int size)
{
	mem_buffer_reserved = mem;
	mem_buffer_size = size;
	return 0;
}


int mem_buffer_init(int mode)
{
	int mem_offset;
	struct tx_buffer *rx_fifo;
	struct circ_buf *xfer;

	g_mem = &g_mem_buffer_alloc[0];
#if 1
	/* setup memory buffer space */
	vtw_print(LOG_INFO, "mem_buffer_init() mem_buffer_reserved:\t");
	vtw_print_hex(LOG_INFO, (unsigned int)mem_buffer_reserved);
	vtw_print(LOG_INFO, " mem_buffer_size:\t");
	vtw_print_hex(LOG_INFO, mem_buffer_size);
	vtw_print(LOG_INFO, "\r\n");
#endif
	mem_offset = 0;
	g_mem->pIvwObjBuf = (void *)(mem_buffer_reserved + mem_offset);
	mem_offset += SR_IVWOBJ_SIZE;

	g_mem->nReisdentBuf = (void *)(mem_buffer_reserved + mem_offset);
	mem_offset += SR_RESIDENT_RAM_SIZE;

	g_mem->wakeup_key_resource = (void *)(mem_buffer_reserved + mem_offset);

	/* DMA_DESC_ADDR */
	g_mem->dma_desc_addr = (void* )DMA_DESC_ADDR;

	/* setup rx_fifo */
	rx_fifo = get_rx_fifo();
	xfer = &rx_fifo->xfer;
	rx_fifo->n_size	= VOICE_TCSM_DATA_BUF_SIZE; /*tcsm 4kBytes*/
	xfer->buf = (char *)VOICE_TCSM_DATA_BUF;
	xfer->tail = xfer->head = 0;


	return 0;
}

int mem_buffer_reset_rx_fifo(void)
{
	struct tx_buffer *rx_fifo;
	struct circ_buf *xfer;

	rx_fifo = get_rx_fifo();
	xfer = &rx_fifo->xfer;
	xfer->tail = xfer->head = 0;

	return 0;
}

struct tx_buffer *get_rx_fifo(void)
{
	return &g_mem->rx_buffer;
}

void * get_dma_desc_addr(void)
{
	return (void *) g_mem->dma_desc_addr;
}

void * get_pIvwObjBuf(void)
{
	return (void *) g_mem->pIvwObjBuf;
}

void * get_nReisdentBuf(void)
{
	return (void *) g_mem->nReisdentBuf;
}

void * get_wakeup_key_resource_address(void)
{
	return (void *) g_mem->wakeup_key_resource;
}

