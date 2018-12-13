/*
 * SFC controller for SPI protocol, use FIFO and DMA;
 *
 * Copyright (c) 2015 Ingenic
 * Author: <xiaoyang.fu@ingenic.com>
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
*/

#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>

#include <asm/uaccess.h>
#include <asm/irq.h>
#include <asm/io.h>

#include "jz_sfc_common.h"


//#define SFC_DEBUG


#define GET_PHYADDR(a)  \
({						\
	unsigned int v;        \
	if (unlikely((unsigned int)(a) & 0x40000000)) {    \
	v = page_to_phys(vmalloc_to_page((const void *)(a))) | ((unsigned int)(a) & ~PAGE_MASK); \
	} else     \
	v = ((unsigned int)(a) & 0x1fffffff);                   \
	v;                                             \
 })
static inline void sfc_writel(struct sfc *sfc, unsigned short offset, u32 value)
{
	writel(value, sfc->iomem + offset);
}

static inline unsigned int sfc_readl(struct sfc *sfc, unsigned short offset)
{
	return readl(sfc->iomem + offset);
}

#ifdef SFC_DEBUG
void dump_sfc_reg(struct sfc *sfc)
{
	int i = 0;
	printk("SFC_GLB			:%08x\n", sfc_readl(sfc, SFC_GLB ));
	printk("SFC_DEV_CONF	:%08x\n", sfc_readl(sfc, SFC_DEV_CONF ));
	printk("SFC_DEV_STA_EXP	:%08x\n", sfc_readl(sfc, SFC_DEV_STA_EXP));
	printk("SFC_DEV_STA_RT	:%08x\n", sfc_readl(sfc, SFC_DEV_STA_RT ));
	printk("SFC_DEV_STA_MSK	:%08x\n", sfc_readl(sfc, SFC_DEV_STA_MSK ));
	printk("SFC_TRAN_LEN		:%08x\n", sfc_readl(sfc, SFC_TRAN_LEN ));

	for(i = 0; i < 6; i++)
		printk("SFC_TRAN_CONF(%d)	:%08x\n", i,sfc_readl(sfc, SFC_TRAN_CONF(i)));

	for(i = 0; i < 6; i++)
		printk("SFC_DEV_ADDR(%d)	:%08x\n", i,sfc_readl(sfc, SFC_DEV_ADDR(i)));

	printk("SFC_MEM_ADDR :%08x\n", sfc_readl(sfc, SFC_MEM_ADDR ));
	printk("SFC_TRIG	 :%08x\n", sfc_readl(sfc, SFC_TRIG));
	printk("SFC_SR		 :%08x\n", sfc_readl(sfc, SFC_SR));
	printk("SFC_SCR		 :%08x\n", sfc_readl(sfc, SFC_SCR));
	printk("SFC_INTC	 :%08x\n", sfc_readl(sfc, SFC_INTC));
	printk("SFC_FSM		 :%08x\n", sfc_readl(sfc, SFC_FSM ));
	printk("SFC_CGE		 :%08x\n", sfc_readl(sfc, SFC_CGE ));
//	printk("SFC_RM_DR 	 :%08x\n", sfc_readl(spi, SFC_RM_DR));
}
static void dump_reg(struct sfc *sfc)
{
	printk("SFC_GLB = %08x\n",sfc_readl(sfc,0x0000));
	printk("SFC_DEV_CONF = %08x\n",sfc_readl(sfc,0x0004));
	printk("SFC_DEV_STA_EXP = %08x\n",sfc_readl(sfc,0x0008));
	printk("SFC_DEV_STA_RT	 = %08x\n",sfc_readl(sfc,0x000c));
	printk("SFC_DEV_STA_MASK = %08x\n",sfc_readl(sfc,0x0010));
	printk("SFC_TRAN_CONF0 = %08x\n",sfc_readl(sfc,0x0014));
	printk("SFC_TRAN_LEN = %08x\n",sfc_readl(sfc,0x002c));
	printk("SFC_DEV_ADDR0 = %08x\n",sfc_readl(sfc,0x0030));
	printk("SFC_DEV_ADDR_PLUS0 = %08x\n",sfc_readl(sfc,0x0048));
	printk("SFC_MEM_ADDR = %08x\n",sfc_readl(sfc,0x0060));
	printk("SFC_TRIG = %08x\n",sfc_readl(sfc,0x0064));
	printk("SFC_SR = %08x\n",sfc_readl(sfc,0x0068));
	printk("SFC_SCR = %08x\n",sfc_readl(sfc,0x006c));
	printk("SFC_INTC = %08x\n",sfc_readl(sfc,0x0070));
	printk("SFC_FSM = %08x\n",sfc_readl(sfc,0x0074));
	printk("SFC_CGE = %08x\n",sfc_readl(sfc,0x0078));
//	printk("SFC_DR = %08x\n",sfc_readl(sfc,0x1000));
}
#endif

static inline void sfc_init(struct sfc *sfc)
{
	sfc_writel(sfc, SFC_TRIG, TRIG_STOP);
	sfc_writel(sfc, SFC_DEV_CONF, 0);

	/* X1000 need set to 0,but X2000 can be set to 1*/
	sfc_writel(sfc, SFC_CGE, 0);

}

static inline void sfc_start(struct sfc *sfc)
{
	uint32_t tmp;
	tmp = sfc_readl(sfc, SFC_TRIG);
	tmp |= TRIG_START;
	sfc_writel(sfc, SFC_TRIG, tmp);
}

static inline void sfc_flush_fifo(struct sfc *sfc)
{
	unsigned int tmp;
	tmp = sfc_readl(sfc, SFC_TRIG);
	tmp |= TRIG_FLUSH;
	sfc_writel(sfc, SFC_TRIG, tmp);
}
static inline void  sfc_clear_end_intc(struct sfc *sfc)
{
	sfc_writel(sfc, SFC_SCR, CLR_END);
}

static inline void sfc_clear_treq_intc(struct sfc *sfc)
{
	sfc_writel(sfc, SFC_SCR, CLR_TREQ);
}

static inline void sfc_clear_rreq_intc(struct sfc *sfc)
{
	sfc_writel(sfc, SFC_SCR, CLR_RREQ);
}

static inline void sfc_clear_over_intc(struct sfc *sfc)
{
	sfc_writel(sfc, SFC_SCR, CLR_OVER);
}

static inline void sfc_clear_under_intc(struct sfc *sfc)
{
	sfc_writel(sfc, SFC_SCR, CLR_UNDER);
}

static inline void sfc_clear_all_intc(struct sfc *sfc)
{
	sfc_writel(sfc, SFC_SCR, 0x1f);
}

static inline void sfc_mask_all_intc(struct sfc *sfc)
{
	sfc_writel(sfc, SFC_INTC, 0x1f);
}

static void sfc_set_phase_num(struct sfc *sfc, int32_t num)
{
	uint32_t tmp;

	tmp = sfc_readl(sfc, SFC_GLB);
	tmp &= ~GLB_PHASE_NUM_MSK;
	tmp |= num << GLB_PHASE_NUM_OFFSET;
	sfc_writel(sfc, SFC_GLB, tmp);
}

static void sfc_dev_hw_init(struct sfc *sfc)
{
	uint32_t tmp;
	tmp = sfc_readl(sfc, SFC_DEV_CONF);

	/*cpha bit:0 , cpol bit:0 */
	tmp &= ~(DEV_CONF_CPHA | DEV_CONF_CPOL);
	/*ce_dl bit:1, hold bit:1,wp bit:1*/
	tmp |= (DEV_CONF_CEDL | DEV_CONF_HOLDDL | DEV_CONF_WPDL);
	sfc_writel(sfc, SFC_DEV_CONF, tmp);

}

static void sfc_threshold(struct sfc *sfc, uint32_t value)
{
	uint32_t tmp = sfc_readl(sfc, SFC_GLB);
	tmp &= ~GLB_THRESHOLD_MSK;
	tmp |= value << GLB_THRESHOLD_OFFSET;
	sfc_writel(sfc, SFC_GLB, tmp);
}

static void sfc_smp_delay(struct sfc *sfc, uint8_t value)
{
	uint32_t tmp;
	tmp = sfc_readl(sfc, SFC_DEV_CONF);
	tmp &= ~DEV_CONF_SMP_DELAY_MSK;
	tmp |= value << DEV_CONF_SMP_DELAY_OFFSET;
	sfc_writel(sfc, SFC_DEV_CONF, tmp);
}

int32_t set_flash_timing(struct sfc *sfc, uint32_t t_hold, uint32_t t_setup, uint32_t t_shslrd, uint32_t t_shslwr)
{
	uint32_t c_hold = 0;
	uint32_t c_setup = 0;
	uint32_t t_in = 0, c_in = 0;
	uint32_t tmp;
	unsigned long cycle;
	unsigned long long ns;

	ns = 1000000000ULL;
	cycle = do_div(ns, sfc->src_clk);
	cycle = ns;

	tmp = sfc_readl(sfc, SFC_DEV_CONF);
	tmp &= ~(DEV_CONF_THOLD_MSK | DEV_CONF_TSETUP_MSK | DEV_CONF_TSH_MSK);

	c_hold = t_hold / cycle;
	if(c_hold > 0)
		c_hold -= 1;

	c_setup = t_setup / cycle;
	if(c_setup > 0)
		c_setup -= 1;

	t_in = max(t_shslrd, t_shslwr);
	c_in = t_in / cycle;
	if(c_in > 0)
		c_in -= 1;

	tmp |= (c_hold << DEV_CONF_THOLD_OFFSET) | \
		  (c_setup << DEV_CONF_TSETUP_OFFSET) | \
		  (c_in << DEV_CONF_TSH_OFFSET);

	sfc_writel(sfc, SFC_DEV_CONF, tmp);
	return 0;
}

static void sfc_set_length(struct sfc *sfc, uint32_t value)
{
	sfc_writel(sfc, SFC_TRAN_LEN, value);
}

static inline void sfc_transfer_mode(struct sfc *sfc, uint8_t value)
{
	uint32_t tmp;
	tmp = sfc_readl(sfc, SFC_GLB);
	if(value == 0)
		tmp &= ~GLB_OP_MODE;
	else
		tmp |= GLB_OP_MODE;
	sfc_writel(sfc, SFC_GLB, tmp);
}

static void sfc_read_data(struct sfc *sfc, uint32_t *value)
{
	*value = sfc_readl(sfc, SFC_RM_DR);
}

static void sfc_write_data(struct sfc *sfc, uint32_t value)
{
	sfc_writel(sfc, SFC_RM_DR, value);
}

static void cpu_read_rxfifo(struct sfc *sfc)
{
	int32_t i = 0;
	uint32_t align_len = 0;
	uint32_t fifo_num = 0;
	uint32_t last_word = 0;
	uint32_t unalign_data;
	uint8_t *c;

	align_len = ALIGN(sfc->transfer->len, 4);

	if(((align_len - sfc->transfer->cur_len) / 4) > sfc->threshold) {
		fifo_num = sfc->threshold;
		last_word = 0;
	} else {
		/* last aligned THRESHOLD data*/
		if(sfc->transfer->len % 4) {
			fifo_num = (align_len - sfc->transfer->cur_len) / 4 - 1;
			last_word = 1;
		} else {
			fifo_num = (align_len - sfc->transfer->cur_len) / 4;
			last_word = 0;
		}
	}

	if ((uint32_t)sfc->transfer->data & 0x3) {
		/* addr not align */
		for (i = 0; i < fifo_num; i++) {
			sfc_read_data(sfc, &unalign_data);
			c = sfc->transfer->data;
			c[0] = (unalign_data >> 0) & 0xff;
			c[1] = (unalign_data >> 8) & 0xff;
			c[2] = (unalign_data >> 16) & 0xff;
			c[3] = (unalign_data >> 24) & 0xff;

			sfc->transfer->data += 4;
			sfc->transfer->cur_len += 4;
		}
	} else {
		/* addr align */
		for (i = 0; i < fifo_num; i++) {
			sfc_read_data(sfc, (uint32_t *)sfc->transfer->data);
			sfc->transfer->data += 4;
			sfc->transfer->cur_len += 4;
		}
	}

	/* last word */
	if(last_word == 1) {
		sfc_read_data(sfc, &unalign_data);
		c = sfc->transfer->data;

		for(i = 0; i < sfc->transfer->len % 4; i++) {
			c[i] = (unalign_data >> (i * 8)) & 0xff;
		}

		sfc->transfer->data += sfc->transfer->len % 4;
		sfc->transfer->cur_len += sfc->transfer->len % 4;
	}

}

static void cpu_write_txfifo(struct sfc *sfc)
{
	uint32_t align_len = 0;
	uint32_t fifo_num = 0;
	uint32_t data = 0;
	uint32_t i;
	uint32_t nbytes = sfc->transfer->len % 4;

	align_len = sfc->transfer->len / 4 * 4;

	if (((align_len - sfc->transfer->cur_len) / 4) >= sfc->threshold) {
		fifo_num = sfc->threshold;
		nbytes = 0;
	} else {
		fifo_num = (align_len - sfc->transfer->cur_len) / 4;
	}

	if ((uint32_t)sfc->transfer->data & 3) {
		/* addr not align */
		for(i = 0; i < fifo_num; i++) {
			data = sfc->transfer->data[3] << 24 | sfc->transfer->data[2] << 16 | sfc->transfer->data[1] << 8 | sfc->transfer->data[0];
			sfc_write_data(sfc, data);
			sfc->transfer->data += 4;
			sfc->transfer->cur_len += 4;
		}
	} else {
		/* addr align */
		for(i = 0; i < fifo_num; i++) {
			sfc_write_data(sfc, *(uint32_t *)sfc->transfer->data);
			sfc->transfer->data += 4;
			sfc->transfer->cur_len += 4;
		}
	}

	if(nbytes) {
		data = 0;
		for(i = 0; i < nbytes; i++)
			data |= sfc->transfer->data[i] << i * 8;
		sfc_write_data(sfc, data);
		sfc->transfer->cur_len += nbytes;
	}

}

uint32_t sfc_get_sta_rt(struct sfc *sfc)
{
	return sfc_readl(sfc, SFC_DEV_STA_RT);
}

static void sfc_dev_sta_exp(struct sfc *sfc, uint32_t value)
{
	sfc_writel(sfc, SFC_DEV_STA_EXP, value);
}

static void sfc_dev_sta_msk(struct sfc *sfc, uint32_t value)
{
	sfc_writel(sfc, SFC_DEV_STA_MSK, value);
}

static void sfc_enable_all_intc(struct sfc *sfc)
{
	sfc_writel(sfc, SFC_INTC, 0);
}

static void sfc_set_mem_addr(struct sfc *sfc, unsigned int addr)
{
	sfc_writel(sfc, SFC_MEM_ADDR, addr);
}

#define SFC_TRANSFER_TIMEOUT	3000	//3000ms for timeout
static int32_t sfc_start_transfer(struct sfc *sfc)
{
	int32_t err;
	sfc_clear_all_intc(sfc);
	sfc_enable_all_intc(sfc);
	sfc_start(sfc);
	err = wait_for_completion_timeout(&sfc->done, msecs_to_jiffies(SFC_TRANSFER_TIMEOUT));
	if (!err) {
		sfc_mask_all_intc(sfc);
		sfc_clear_all_intc(sfc);
		printk("line:%d Timeout for ACK from SFC device\n",__LINE__);
		return -ETIMEDOUT;
	}
	return 0;
}

static void sfc_set_tran_config(struct sfc *sfc, struct sfc_transfer *transfer, int channel)
{
	uint32_t tmp = 0;

	tmp = (transfer->sfc_mode << TRAN_CONF_TRAN_MODE_OFFSET)		\
		| (transfer->addr_len << ADDR_WIDTH_OFFSET)			\
		| (transfer->cmd_info.pollen << TRAN_CONF_POLL_OFFSET)		\
		| (TRAN_CONF_CMDEN)						\
		| (0 << TRAN_CONF_FMAT_OFFSET)					\
		| (transfer->data_dummy_bits << DMYBITS_OFFSET)			\
		| (transfer->cmd_info.dataen << TRAN_CONF_DATEEN_OFFSET)	\
		| transfer->cmd_info.cmd;

	sfc_writel(sfc, SFC_TRAN_CONF(channel), tmp);
}

static void sfc_phase_transfer(struct sfc *sfc, struct sfc_transfer *
		transfer, int channel)
{
	sfc_writel(sfc, SFC_DEV_ADDR(channel), transfer->addr);    //set addr
	sfc_writel(sfc, SFC_DEV_ADDR_PLUS(channel), transfer->addr_plus);  //set plus addr
	sfc_set_tran_config(sfc, transfer, channel);
}

static void sfc_set_glb_config(struct sfc *sfc, struct sfc_transfer *transfer)
{
	uint32_t tmp = 0;

	tmp = sfc_readl(sfc, SFC_GLB);

	if (transfer->direction == GLB_TRAN_DIR_READ)
		tmp &= ~GLB_TRAN_DIR;
	else
		tmp |= GLB_TRAN_DIR;

	if (transfer->ops_mode == DMA_OPS)
		tmp |= GLB_OP_MODE;
	else
		tmp &= ~GLB_OP_MODE;

	sfc_writel(sfc, SFC_GLB, tmp);
}

static void sfc_glb_info_config(struct sfc *sfc, struct sfc_transfer *transfer)
{
	sfc_set_length(sfc, transfer->len);
	if((transfer->ops_mode == DMA_OPS)){
		if(transfer->direction == GLB_TRAN_DIR_READ)
			dma_cache_sync(NULL, (void *)transfer->data, transfer->len, DMA_FROM_DEVICE);
		else
			dma_cache_sync(NULL, (void *)transfer->data, transfer->len, DMA_TO_DEVICE);
		sfc_set_mem_addr(sfc, GET_PHYADDR(transfer->data));
	}else{
		sfc_set_mem_addr(sfc, 0);
	}
	sfc_set_glb_config(sfc, transfer);
}
#ifdef SFC_DEBUG
static void  dump_transfer(struct sfc_transfer *xfer,int num)
{
	printk("\n");
	printk("cmd[%d].cmd = 0x%02x\n",num,xfer->cmd_info->cmd);
	printk("cmd[%d].addr_len = %d\n",num,xfer->addr_len);
	printk("cmd[%d].dummy_byte = %d\n",num,xfer->data_dummy_bits);
	printk("cmd[%d].dataen = %d\n",num,xfer->cmd_info->dataen);
	printk("cmd[%d].sta_exp = %d\n",num,xfer->cmd_info->sta_exp);
	printk("cmd[%d].sta_msk = %d\n",num,xfer->cmd_info->sta_msk);


	printk("transfer[%d].addr = 0x%08x\n",num,xfer->addr);
	printk("transfer[%d].len = %d\n",num,xfer->len);
	printk("transfer[%d].data = 0x%p\n",num,xfer->data);
	printk("transfer[%d].direction = %d\n",num,xfer->direction);
	printk("transfer[%d].sfc_mode = %d\n",num,xfer->sfc_mode);
	printk("transfer[%d].ops_mode = %d\n",num,xfer->ops_mode);
}
#endif

int sfc_sync(struct sfc *sfc, struct sfc_transfer *head)
{
	int phase_num = 0;
	struct sfc_transfer *xfer = head;

	sfc_flush_fifo(sfc);
	sfc_set_length(sfc, 0);
	do {

		sfc_phase_transfer(sfc, xfer, phase_num);    //set phase
		if(xfer->cmd_info.pollen) {			     //set polling
			sfc_dev_sta_exp(sfc, xfer->cmd_info.sta_exp);
			sfc_dev_sta_msk(sfc, xfer->cmd_info.sta_msk);
		}

		if(xfer->cmd_info.dataen && xfer->len) {     //set mem
			sfc_glb_info_config(sfc, xfer);
			if(xfer->ops_mode == CPU_OPS)
				sfc->transfer = xfer;
			if(xfer->ops_mode == DMA_OPS)
				xfer->cur_len = xfer->len;
		}

		phase_num++;
	} while (&xfer->list != head->list.prev &&
		(xfer = list_entry(xfer->list.next, typeof(*xfer), list)));

	sfc_set_phase_num(sfc, phase_num);
	return sfc_start_transfer(sfc);
}

void sfc_transfer_del(struct sfc_transfer *entry)
{
	list_del(&entry->list);
}

void sfc_list_add_tail(struct sfc_transfer *new, struct sfc_transfer *head)
{
	list_add_tail(&new->list, &head->list);
}

void sfc_list_init(struct sfc_transfer *head)
{
	INIT_LIST_HEAD(&head->list);
}

static irqreturn_t jz_sfc_pio_irq_callback(int32_t irq, void *dev)
{
	struct sfc *sfc = dev;
	uint32_t val;

	val = sfc_readl(sfc, SFC_SR) & 0x1f;
	switch(val) {
		case CLR_RREQ:
			sfc_clear_rreq_intc(sfc);
			cpu_read_rxfifo(sfc);
			break;
		case CLR_TREQ:
			sfc_clear_treq_intc(sfc);
			cpu_write_txfifo(sfc);
			break;
		case CLR_END:
			sfc_mask_all_intc(sfc);
			sfc_clear_end_intc(sfc);
			complete(&sfc->done);
			break;
		case CLR_OVER:
			sfc_clear_over_intc(sfc);
			printk("sfc OVER !\n");
			complete(&sfc->done);
			break;
		case CLR_UNDER:
			sfc_clear_under_intc(sfc);
			printk("sfc UNDR !\n");
			complete(&sfc->done);
			break;
		default:
			printk("current staus %x not support \n", sfc_readl(sfc, SFC_SR));
			printk("entry handler staus %x  \n", val);
			return IRQ_NONE;
	}
	return IRQ_HANDLED;
}

static void jz_sfc_init_setup(struct sfc *sfc)
{
	sfc_init(sfc);
	sfc_threshold(sfc, sfc->threshold);
	sfc_dev_hw_init(sfc);

	sfc_transfer_mode(sfc, SLAVE_MODE);
	if(sfc->src_clk >= 100000000){
		sfc_smp_delay(sfc, DEV_CONF_HALF_CYCLE_DELAY);
	}
}

struct sfc *sfc_res_init(struct platform_device *pdev)
{
	struct sfc *sfc;
	struct resource *res;
	int32_t err;
	sfc = kzalloc(sizeof(struct sfc), GFP_KERNEL);
	if (!sfc) {
		printk("ERROR: %s %d kzalloc() error !\n",__func__,__LINE__);
		return ERR_PTR(-ENOMEM);
	}
	/* find and map our resources */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "Cannot get IORESOURCE_MEM\n");
		err = -ENOENT;
		goto err_no_iores;
	}

	sfc->ioarea = request_mem_region(res->start, resource_size(res),
					pdev->name);
	if (sfc->ioarea == NULL) {
		dev_err(&pdev->dev, "Cannot reserve iomem region\n");
		err = -ENXIO;
		goto err_mem_region;
	}

	sfc->phys = res->start;

	sfc->iomem = ioremap(res->start, (res->end - res->start) + 1);
	if (sfc->iomem == NULL) {
		dev_err(&pdev->dev, "Cannot map IO\n");
		err = -ENXIO;
		goto err_no_iomap;
	}

	sfc->irq = platform_get_irq(pdev, 0);
	if (sfc->irq < 0) {
		dev_err(&pdev->dev, "No IRQ specified\n");
		err = -ENOENT;
		goto err_no_irq;
	}

	sfc->clk = clk_get(&pdev->dev, "cgu_sfc");
	if (IS_ERR(sfc->clk)) {
		dev_err(&pdev->dev, "Cannot get cgu_sfc clock\n");
		err = -ENOENT;
		goto err_no_clk;
	}

	sfc->clk_gate = clk_get(&pdev->dev, "sfc");
	if (IS_ERR(sfc->clk_gate)) {
		dev_err(&pdev->dev, "Cannot get sfc clock\n");
		err = -ENOENT;
		goto err_no_clk_gate;
	}

	res = platform_get_resource(pdev, IORESOURCE_BUS, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "Cannot get IORESOURCE_BUS\n");
		err = -ENOENT;
		goto err_no_iores_bus;
	}

	sfc->src_clk = res->start * 1000000;

	if (clk_get_rate(sfc->clk) >= sfc->src_clk) {
		clk_set_rate(sfc->clk, sfc->src_clk);
	} else {
		clk_set_rate(sfc->clk, sfc->src_clk);
	}

	clk_enable(sfc->clk);
	clk_enable(sfc->clk_gate);

	sfc->threshold = THRESHOLD;
	/* request SFC irq */
	err = request_irq(sfc->irq, jz_sfc_pio_irq_callback, 0, pdev->name, sfc);
	if (err) {
		dev_err(&pdev->dev, "Cannot claim IRQ\n");
		err = -EINVAL;
		goto err_request_irq;
	}

	/* SFC controller initializations for SFC */
	jz_sfc_init_setup(sfc);
	init_completion(&sfc->done);
	return sfc;

err_request_irq:
err_no_iores_bus:
	clk_put(sfc->clk_gate);
err_no_clk_gate:
	clk_put(sfc->clk);
err_no_clk:
err_no_irq:
	iounmap(sfc->iomem);
err_no_iomap:
	release_mem_region(res->start, resource_size(res));
err_mem_region:
	release_resource(res);
err_no_iores:
	kfree(sfc);
	return ERR_PTR(err);
}

void sfc_res_deinit(struct sfc *sfc)
{
	clk_put(sfc->clk_gate);
	clk_put(sfc->clk);
	free_irq(sfc->irq, sfc);
	iounmap(sfc->iomem);
	release_resource(sfc->ioarea);
	kfree(sfc->ioarea);
	kfree(sfc);
}
