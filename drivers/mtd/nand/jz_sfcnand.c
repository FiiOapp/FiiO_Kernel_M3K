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
#include <linux/spi/spi.h>
#include <linux/spi/spi_bitbang.h>
#include <linux/module.h>
#include <linux/mutex.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/nand.h>

#include <asm/uaccess.h>
#include <asm/irq.h>
#include <asm/io.h>

#include"jz_sfcnand.h"

#define SFC_SWAP_BUF_SIZE (2 * 1024)
#define GET_PHYADDR(a)                                          \
	({                                              \
	 unsigned int v;                                        \
	 if (unlikely((unsigned int)(a) & 0x40000000)) {                            \
	 v = page_to_phys(vmalloc_to_page((const void *)(a))) | ((unsigned int)(a) & ~PAGE_MASK); \
	 } else                                         \
	 v = ((int)(a) & 0x1fffffff);                   \
	 v;                                             \
	 })



#ifdef SFC_DEGUG
#define  print_dbg(format,arg...)	printk(format,## arg)
#else
#define  print_dbg(format,arg...)
#endif

#define MAX_ADDR_LEN 4
#define THRESHOLD                       31

#define STATUS_SUSPND    (1<<0)
#define R_MODE                  0x1
#define W_MODE                  0x2
#define RW_MODE                 (R_MODE | W_MODE)
#define THRESHOLD                       31


struct mtd_partition *jz_mtd_partition;
struct spi_nor_platform_data *board_info;
static int quad_mode = 0;


struct jz_sfc_nand *to_jz_spi_nand(struct mtd_info *mtd_info)
{
	return container_of(mtd_info, struct jz_sfc_nand, mtd);
}
int sfc_nand_plat_resource_init(struct jz_sfc_nand *flash,struct platform_device *pdev)
{
	struct resource *res;
	int err;
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "Cannot get IORESOURCE_MEM\n");
		err = -1;
		return err;
	}
	flash->ioarea = request_mem_region(res->start, resource_size(res),pdev->name);
	if (flash->ioarea == NULL) {
		dev_err(&pdev->dev, "Cannot reserve iomem region\n");
		err = -2;
		return err;
	}
	flash->phys = res->start;
	flash->iomem = ioremap(res->start, (res->end - res->start)+1);
	if (flash->iomem == NULL){
		dev_err(&pdev->dev, "Cannot map IO\n");
		err = -3;
		return err;
	}
	flash->irq = platform_get_irq(pdev, 0);
	if (flash->irq <= 0) {
		dev_err(&pdev->dev, "No IRQ specified\n");
		err = -4;
		return err;
	}
	flash->clk = clk_get(&pdev->dev, "cgu_ssi");
	if (IS_ERR(flash->clk)) {
		dev_err(&pdev->dev, "Cannot get ssi clock\n");
		err=-5;
		return err;
	}
	flash->clk_gate = clk_get(&pdev->dev, "sfc");
	if (IS_ERR(flash->clk_gate)) {
		dev_err(&pdev->dev, "Cannot get sfc clock\n");
		err=-6;
		return err;
	}
	res = platform_get_resource(pdev, IORESOURCE_BUS, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "Cannot get IORESOURCE_BUS\n");
		err = -7;
		return err;
	}

	flash->src_clk = res->start * 1000000;

	if (clk_get_rate(flash->clk) >= flash->src_clk) {
		clk_set_rate(flash->clk, flash->src_clk);
	} else {
		clk_set_rate(flash->clk, flash->src_clk);
	}

	clk_enable(flash->clk);
	clk_enable(flash->clk_gate);

	platform_set_drvdata(pdev, flash);
	return 0;
}
static int sfc_pio_transfer(struct jz_sfc_nand *flash)
{
	print_dbg("function : %s, line : %d\n", __func__, __LINE__);
	sfc_flush_fifo(flash);
	sfc_set_length(flash, flash->sfc_tran->len);
	/*use one phase for transfer*/
	sfc_set_addr_length(flash, 0, flash->sfc_tran->sfc_cmd.addr_len);//
	sfc_cmd_en(flash, 0, 0x1);
	sfc_write_cmd(flash, 0, flash->sfc_tran->sfc_cmd.cmd);
	sfc_data_en(flash, 0, flash->sfc_tran->date_en);
	sfc_mode(flash,0,flash->sfc_tran->sfc_cmd.transmode);
	sfc_dev_addr(flash, 0, flash->sfc_tran->sfc_cmd.addr_low);
	sfc_dev_addr_plus(flash, 0, flash->sfc_tran->sfc_cmd.addr_high);
	sfc_dev_addr_dummy_bytes(flash,0,flash->sfc_tran->sfc_cmd.dummy_byte);
	//sfc_dev_pollen(flash,0,flash->sfc_tran->pollen);
	if(flash->sfc_tran->rw_mode & R_MODE)
	{
		if((flash->use_dma ==1)&&(flash->sfc_tran->dma_mode)){
			dma_cache_sync(NULL, (void *)(flash->sfc_tran->rx_buf), flash->sfc_tran->len, DMA_FROM_DEVICE);
			sfc_set_mem_addr(flash, GET_PHYADDR(flash->sfc_tran->rx_buf));
			sfc_transfer_mode(flash, DMA_MODE);
		}else{
			sfc_transfer_mode(flash, SLAVE_MODE);
		}
		sfc_transfer_direction(flash, GLB_TRAN_DIR_READ);

	}else if(flash->sfc_tran->rw_mode & W_MODE)
	{
		if((flash->use_dma ==1)&&(flash->sfc_tran->dma_mode)){
			dma_cache_sync(NULL, (void *)(flash->sfc_tran->tx_buf), flash->sfc_tran->len,DMA_TO_DEVICE);
			sfc_set_mem_addr(flash, GET_PHYADDR(flash->sfc_tran->tx_buf));
			sfc_transfer_mode(flash, DMA_MODE);
		}else{
			sfc_transfer_mode(flash, SLAVE_MODE);
		}
		sfc_transfer_direction(flash, GLB_TRAN_DIR_WRITE);
	}
	else {
		sfc_transfer_mode(flash, SLAVE_MODE);
		sfc_transfer_direction(flash, GLB_TRAN_DIR_WRITE);
	}
	sfc_enable_all_intc(flash);
	sfc_start(flash);
	return 0;
}

static int jz_sfc_pio_txrx(struct jz_sfc_nand *flash, struct sfc_transfer_nand *t)
{
	int ret;
	unsigned long flags;
	int err = 0;
	print_dbg("function : %s, line : %d\n", __func__, __LINE__);
	if((t->tx_buf == NULL) && (t->rx_buf == NULL)&&(t->date_en)) {
		dev_info(flash->dev, "the tx and rx buf of spi_transfer is NULL !\n");
		return -EINVAL;
	}
	if((!t->rx_buf)&&(t->rw_mode==R_MODE)){
		printk("please check the read buf is NULL\n");
		return -EINVAL;
	}
	if((t->tx_buf==NULL)&&(t->rw_mode==W_MODE))
	{
		printk("please check the write buf is NULL\n");
		return -EINVAL;
	}
	spin_lock_irqsave(&flash->lock_rxtx, flags);
	flash->sfc_tran=t;
	ret = sfc_pio_transfer(flash);
	if (ret < 0) {
		dev_err(flash->dev,"data transfer error!,please check the cmd,and the driver do not support spi nand      flash\n");
		sfc_mask_all_intc(flash);
		sfc_clear_all_intc(flash);
		spin_unlock_irqrestore(&flash->lock_rxtx, flags);
		return ret;
	}
	spin_unlock_irqrestore(&flash->lock_rxtx, flags);
	err = wait_for_completion_timeout(&flash->done,10*HZ);
	if (!err) {
		dev_err(flash->dev, "Timeout for ACK from SFC device\n");
		dump_sfc_reg(flash);
		return -ETIMEDOUT;
	}
	/*fix the cache line problem,when use jffs2 filesystem must be flush cache twice*/
	if(flash->sfc_tran->rw_mode & R_MODE)
		dma_cache_sync(NULL, (void *)flash->sfc_tran->rx_buf, flash->sfc_tran->len, DMA_FROM_DEVICE);

	if(flash->use_dma == 1)
	{
		flash->sfc_tran->finally_len=flash->sfc_tran->len;
	}
	return 0;
}

static irqreturn_t jz_sfc_pio_irq_callback(struct jz_sfc_nand *flash)
{
	print_dbg("function : %s, line : %d\n", __func__, __LINE__);
	if (ssi_underrun(flash)) {
		sfc_clear_under_intc(flash);
		dev_err(flash->dev, "sfc UNDR !\n");
		complete(&flash->done);
		return IRQ_HANDLED;
	}
	if (ssi_overrun(flash)) {
		sfc_clear_over_intc(flash);
		dev_err(flash->dev, "sfc OVER !\n");
		complete(&flash->done);
		return IRQ_HANDLED;
	}
	if (rxfifo_rreq(flash)) {
		sfc_clear_rreq_intc(flash);
		return IRQ_HANDLED;
	}
	if (txfifo_treq(flash)) {
		sfc_clear_treq_intc(flash);
		return IRQ_HANDLED;
	}
	if(sfc_end(flash)){
		sfc_clear_end_intc(flash);
		complete(&flash->done);
		return IRQ_HANDLED;
	}
	/*err_no_clk:
	  clk_put(flash->clk_gate);
	  clk_put(flash->clk);
err_no_irq:
free_irq(flash->irq, flash);
err_no_iomap:
iounmap(flash->iomem);
err_no_iores:
err_no_pdata:
release_resource(flash->ioarea);
kfree(flash->ioarea);
	//return err;
	*/
}
static irqreturn_t jz_sfc_irq(int irq, void *dev)
{
	struct jz_sfc_nand *flash = dev;

	//printk("function : %s, line : %d\n", __func__, __LINE__);

	return flash->irq_callback(flash);
}

unsigned int jz_sfc_detect_id(struct jz_sfc_nand *flash)
{
    int detect_again_flag = 1;
    struct jz_spi_support *jz_spi_nand_support_table
        =((struct jz_spi_nand_platform_data *)(flash->pdata->board_info))->jz_spi_support;
    int number_spi_flash = ((struct jz_spi_nand_platform_data *)(flash->pdata->board_info))->num_spi_flash;
    struct sfc_transfer_nand transfer[1];
    struct jz_spi_support *params;
    unsigned int id = 0;
    unsigned char rx_chipid[4];
    int i, ret;
detect_again:
    transfer[0].sfc_cmd.cmd=CMD_RDID;
    transfer[0].sfc_cmd.addr_low=0;
    transfer[0].sfc_cmd.addr_high=0;
    transfer[0].sfc_cmd.dummy_byte=0;
    if (!detect_again_flag)
        transfer[0].sfc_cmd.addr_len=0;
    else
        transfer[0].sfc_cmd.addr_len=1;
    transfer[0].sfc_cmd.cmd_len=1;
    transfer[0].sfc_cmd.transmode=0;
    flash->use_dma=1;

    transfer[0].tx_buf  = NULL;
    transfer[0].rx_buf =  rx_chipid;
    transfer[0].len = 2;
    transfer[0].date_en = 1;
    transfer[0].dma_mode = DMA_MODE;
    transfer[0].pollen = 0;
    transfer[0].rw_mode = R_MODE;
    transfer[0].sfc_mode = 0;
    ret = jz_sfc_pio_txrx(flash,transfer);
    if(ret < 0){
        printk("error in spi transfer.%s %s %d \n",__FILE__,__func__,__LINE__);
        params = NULL;
        goto detect_id_exit;
    }
    for (i = 0; i < number_spi_flash; i++) {
        params = &jz_spi_nand_support_table[i];
        if ( (params->id_manufactory == rx_chipid[0])&&(params->id_device == rx_chipid[1]) ){
            id = (rx_chipid[3]<<24 | rx_chipid[2]<<16 | rx_chipid[1]<<8 | rx_chipid[0]);
            break;
        }
    }
    if (i >= number_spi_flash) {
        if (detect_again_flag){
            detect_again_flag--;
            goto detect_again;
        }
        goto detect_id_exit;
    }
detect_id_exit:
    return id;
}

struct jz_spi_support *jz_sfc_nand_probe(struct jz_sfc_nand *flash)
{
	int ret,i;
	unsigned char rx_chipid[4];
	struct sfc_transfer_nand transfer[1];
	struct jz_spi_support *params;
	struct jz_spi_support *jz_spi_nand_support_table;
	int detect_again_flag = 1;
	mutex_lock(&flash->lock);
	int number_spi_flash;
	number_spi_flash=((struct jz_spi_nand_platform_data *)(flash->pdata->board_info))->num_spi_flash;
	jz_spi_nand_support_table=((struct jz_spi_nand_platform_data *)(flash->pdata->board_info))->jz_spi_support;
detect_again:
	transfer[0].sfc_cmd.cmd=CMD_RDID;
	transfer[0].sfc_cmd.addr_low=0;
	transfer[0].sfc_cmd.addr_high=0;
	transfer[0].sfc_cmd.dummy_byte=0;
	if (!detect_again_flag)
	    transfer[0].sfc_cmd.addr_len=0;
	else
	    transfer[0].sfc_cmd.addr_len=1;
	transfer[0].sfc_cmd.cmd_len=1;
	transfer[0].sfc_cmd.transmode=0;
	flash->use_dma=1;

	transfer[0].tx_buf  = NULL;
	transfer[0].rx_buf =  rx_chipid;
	transfer[0].len=2;
	transfer[0].date_en=1;
	transfer[0].dma_mode=DMA_MODE;
	transfer[0].pollen=0;
	transfer[0].rw_mode=R_MODE;
	transfer[0].sfc_mode=0;
	ret=jz_sfc_pio_txrx(flash,transfer);
	if(ret<0){
		printk("error in spi transfer.%s %s %d \n",__FILE__,__func__,__LINE__);
		params=NULL;
		goto probe_exit;
	}
	printk("sfcnand num=%d\n",number_spi_flash);
	for (i = 0; i < number_spi_flash; i++) {
		params = &jz_spi_nand_support_table[i];
		if ( (params->id_manufactory == rx_chipid[0])&&(params->id_device==rx_chipid[1]) ){
			break;
		}
	}
	if (i >= number_spi_flash) {
	    if (detect_again_flag){
	        detect_again_flag--;
	        printk("sfc nand probe: detect id again\n");
	        goto detect_again;
	    }
		printk("ingenic: Unsupported ID %02x,%02x\n", rx_chipid[0],rx_chipid[1]);
		goto probe_exit;
	}
#if 0
	printk("**************************************************************\n");
	printk("id_manufactory=0x%08x\n",params->id_manufactory);
	printk("name=%s\n",params->name);
	printk("page_size=%d\n",params->page_size);
	printk("oob_size=%d\n",params->oobsize);
	printk("sector_size=%d\n",params->sector_size);
	printk("block_size=%d\n",params->block_size);
	printk("size=%d\n",params->size);
	printk("page_num=%d\n",params->page_num);
	printk("tRD_maxbusy=%d\n",params->tRD_maxbusy);
	printk("tPROG_maxbusy=%d\n",params->tPROG_maxbusy);
	printk("tBERS_maxbusy=%d\n",params->tBERS_maxbusy);
	printk("column_cmdaddr_bits=%d\n",params->column_cmdaddr_bits);
	printk("**************************************************************\n");
#endif
probe_exit:
	mutex_unlock(&flash->lock);
	return params;
}
static int jz_sfc_init_setup(struct jz_sfc_nand *flash)
{
	sfc_init(flash);
	sfc_stop(flash);
	/*set hold high*/
	sfc_hold_invalid_value(flash, 1);
	/*set wp high*/
	sfc_wp_invalid_value(flash, 1);
	sfc_clear_all_intc(flash);
	sfc_mask_all_intc(flash);

	sfc_threshold(flash, flash->threshold);
	/*config the sfc pin init state*/
	sfc_clock_phase(flash, 0);
	sfc_clock_polarity(flash, 0);
	sfc_ce_invalid_value(flash, 1);
	sfc_transfer_mode(flash, SLAVE_MODE);
	if(flash->src_clk >= 100000000){
		sfc_smp_delay(flash,DEV_CONF_HALF_CYCLE_DELAY);
	}
	flash->swap_buf = kmalloc(SFC_SWAP_BUF_SIZE ,GFP_KERNEL);
	if(flash->swap_buf == NULL){
		dev_err(flash->dev,"alloc mem error\n");
		return ENOMEM;
	}
#if defined(CONFIG_SFC_DMA)
	flash->use_dma = 1;
#endif
	flash->irq_callback = &jz_sfc_pio_irq_callback;
	return 0;
}
static int jz_sfc_nand_pro_random_cmd(struct jz_sfc_nand *flash,char *buf,int column,int len)
{
	struct sfc_transfer_nand transfer[1];
	int ret;
	memset(transfer,0,sizeof(struct sfc_transfer_nand));
#ifdef CONFIG_SPI_QUAD
	transfer[0].sfc_cmd.cmd=SPINAND_CMD_PLRd_X4;
	transfer[0].sfc_cmd.transmode=TRAN_SPI_QUAD;
#else
	transfer[0].sfc_cmd.cmd=SPINAND_CMD_PLRd;
	transfer[0].sfc_cmd.transmode=0;
#endif
	transfer[0].sfc_cmd.addr_low=column;
	transfer[0].sfc_cmd.addr_high=0;
	transfer[0].sfc_cmd.dummy_byte=0;
	transfer[0].sfc_cmd.addr_len=2;
	transfer[0].sfc_cmd.cmd_len=1;
	transfer[0].finally_len=0;

	transfer[0].tx_buf = buf;
	transfer[0].rx_buf = NULL;
	transfer[0].len=len;
	transfer[0].date_en=1;
	transfer[0].dma_mode=DMA_MODE;
	transfer[0].pollen=0;
	transfer[0].rw_mode=W_MODE;
	transfer[0].sfc_mode=0;
	ret=jz_sfc_pio_txrx(flash,transfer);
	return ret;
}
static int jz_sfc_nand_pro_en_cmd(struct jz_sfc_nand *flash,int page)
{
	struct sfc_transfer_nand transfer[1];
	int ret;
	memset(transfer,0,sizeof(struct sfc_transfer_nand));
	transfer[0].sfc_cmd.cmd=SPINAND_CMD_PRO_EN;
	transfer[0].sfc_cmd.addr_low=page;
	transfer[0].sfc_cmd.addr_high=0;
	transfer[0].sfc_cmd.dummy_byte=0;
	transfer[0].sfc_cmd.addr_len=3;
	transfer[0].sfc_cmd.cmd_len=1;
	transfer[0].sfc_cmd.transmode=0;
	transfer[0].finally_len=0;

	transfer[0].tx_buf = NULL;
	transfer[0].rx_buf = NULL;
	transfer[0].len=0;
	transfer[0].date_en=0;
	transfer[0].dma_mode=SLAVE_MODE;
	transfer[0].pollen=0;
	transfer[0].rw_mode=0;
	transfer[0].sfc_mode=0;
	ret=jz_sfc_pio_txrx(flash,transfer);
	return ret;
}
static int jz_sfc_nand_write_cmd(struct jz_sfc_nand *flash,int column,int wlen)
{
	struct sfc_transfer_nand transfer[1];
	int ret;
	memset(transfer,0,sizeof(struct sfc_transfer_nand));
#ifdef CONFIG_SPI_QUAD
	transfer[0].sfc_cmd.cmd=SPINAND_CMD_PRO_LOAD_X4;
	transfer[0].sfc_cmd.transmode=TRAN_SPI_QUAD;
#else
	transfer[0].sfc_cmd.cmd=SPINAND_CMD_PRO_LOAD;
	transfer[0].sfc_cmd.transmode=0;
#endif
	transfer[0].sfc_cmd.addr_low=column;
	transfer[0].sfc_cmd.addr_high=0;
	transfer[0].sfc_cmd.dummy_byte=0;
	transfer[0].sfc_cmd.addr_len=2;
	transfer[0].sfc_cmd.cmd_len=1;
	transfer[0].finally_len=0;

	transfer[0].tx_buf = flash->swap_buf;
	transfer[0].rx_buf = NULL;
	transfer[0].len=wlen;
	transfer[0].date_en=1;
	transfer[0].dma_mode=DMA_MODE;
	transfer[0].pollen=0;
	transfer[0].rw_mode=W_MODE;
	transfer[0].sfc_mode=0;
	ret=jz_sfc_pio_txrx(flash,transfer);
	return ret;
}
static int jz_sfc_nand_loadpage_cmd(struct jz_sfc_nand *flash,int page)
{
	struct sfc_transfer_nand transfer[1];
	int ret;
	memset(transfer,0,sizeof(struct sfc_transfer_nand));
	transfer[0].sfc_cmd.cmd=SPINAND_CMD_PARD;
	transfer[0].sfc_cmd.addr_low=page;
	transfer[0].sfc_cmd.addr_high=0;
	transfer[0].sfc_cmd.dummy_byte=0;
	transfer[0].sfc_cmd.addr_len=3;
	transfer[0].sfc_cmd.cmd_len=1;
	transfer[0].sfc_cmd.transmode=0;
	transfer[0].finally_len=0;

	transfer[0].tx_buf = NULL;
	transfer[0].rx_buf = NULL;
	transfer[0].len=0;
	transfer[0].date_en=0;
	transfer[0].dma_mode=SLAVE_MODE;
	transfer[0].pollen=0;
	transfer[0].rw_mode=0;
	transfer[0].sfc_mode=0;
	ret=jz_sfc_pio_txrx(flash,transfer);
	return ret;
}
static int jz_sfc_nand_read_cmd(struct jz_sfc_nand *flash,char *buf,int column,unsigned int len)
{
	struct sfc_transfer_nand transfer[1];
	int ret;
	memset(transfer,0,sizeof(struct sfc_transfer_nand));
	switch(flash->column_cmdaddr_bits){
		case 24:
#ifdef CONFIG_SPI_QUAD
			transfer[0].sfc_cmd.cmd=SPINAND_CMD_RDCH_X4;
#else
			transfer[0].sfc_cmd.cmd=SPINAND_CMD_RDCH;
#endif
			transfer[0].sfc_cmd.addr_len=3;
			break;
		case 32:
#ifdef CONFIG_SPI_QUAD
			transfer[0].sfc_cmd.cmd=SPINAND_CMD_RDCH_X4;
#else
			transfer[0].sfc_cmd.cmd=SPINAND_CMD_FRCH;
#endif
			transfer[0].sfc_cmd.addr_len=4;
			break;
		default:
			pr_info("can't support the format of column addr ops !!\n");
			ret = -EINVAL;
			return ret;
			break;
	}
	transfer[0].sfc_cmd.addr_low=(column<<8)& 0xffffff00;
#ifdef CONFIG_SPI_QUAD
	transfer[0].sfc_cmd.transmode=TRAN_SPI_QUAD;
#else
	transfer[0].sfc_cmd.transmode=0;
#endif
	transfer[0].sfc_cmd.cmd_len=1;
	transfer[0].rx_buf = buf;
	transfer[0].len=len;
	transfer[0].date_en=1;
	transfer[0].dma_mode=DMA_MODE;
	transfer[0].rw_mode=R_MODE;
	ret=jz_sfc_pio_txrx(flash,transfer);
	return ret;
}
static int jz_sfc_nandflash_write_enable(struct jz_sfc_nand *flash)
{

	int ret;
	struct sfc_transfer_nand transfer[1];
	transfer[0].sfc_cmd.cmd=SPINAND_CMD_WREN;
	transfer[0].sfc_cmd.addr_low=0;
	transfer[0].sfc_cmd.addr_high=0;
	transfer[0].sfc_cmd.dummy_byte=0;
	transfer[0].sfc_cmd.addr_len=0;
	transfer[0].sfc_cmd.cmd_len=1;
	transfer[0].sfc_cmd.transmode=0;

	transfer[0].dma_mode=0;
	transfer[0].tx_buf  = NULL;
	transfer[0].rx_buf =  NULL;
	transfer[0].len=0;
	transfer[0].date_en=0;
	transfer[0].dma_mode=SLAVE_MODE;
	transfer[0].pollen=0;
	transfer[0].rw_mode=0;
	transfer[0].sfc_mode=0;
	ret=jz_sfc_pio_txrx(flash,transfer);
	return ret;
}

#ifdef CONFIG_SPI_QUAD
static int jz_sfc_nandflash_get_feature(struct jz_sfc_nand *flash, u8 addr, u8 *status)
{
	int ret = 0;
	struct sfc_transfer_nand transfer[1];

	transfer[0].sfc_cmd.cmd=SPINAND_CMD_GET_FEATURE;
	transfer[0].sfc_cmd.addr_low=addr;
	transfer[0].sfc_cmd.addr_high=0;
	transfer[0].sfc_cmd.dummy_byte=0;
	transfer[0].sfc_cmd.addr_len=1;
	transfer[0].sfc_cmd.cmd_len=1;
	transfer[0].sfc_cmd.transmode=0;
	flash->use_dma=1;

	transfer[0].tx_buf  = NULL;
	transfer[0].rx_buf =  status;
	transfer[0].len=1;
	transfer[0].date_en=1;
	transfer[0].dma_mode=DMA_MODE;
	transfer[0].pollen=0;
	transfer[0].rw_mode=R_MODE;
	transfer[0].sfc_mode=0;
	transfer[0].finally_len=0;
	ret = jz_sfc_pio_txrx(flash,transfer);
	return ret;
}

static int jz_sfc_nandflash_set_quad_mode(struct jz_sfc_nand *flash)
{
	u8 status;
	int ret;
	struct sfc_transfer_nand transfer[1];

	jz_sfc_nandflash_get_feature(flash, SPINAND_ADDR_FEATURE, &status);
	status |= 0x01; // set FEATURE_REG QE bit
	transfer[0].sfc_cmd.cmd=SPINAND_CMD_SET_FEATURE;
	transfer[0].sfc_cmd.addr_low=0xb000|status; //SPINAND_FEATURE_ADDR + value;
	transfer[0].sfc_cmd.addr_high=0;
	transfer[0].sfc_cmd.dummy_byte=0;
	transfer[0].sfc_cmd.addr_len=2;
	transfer[0].sfc_cmd.cmd_len=1;
	transfer[0].sfc_cmd.transmode=0;

	transfer[0].tx_buf = NULL;
	transfer[0].rx_buf = NULL;
	transfer[0].len=0;
	transfer[0].date_en=0;
	transfer[0].dma_mode=SLAVE_MODE;
	transfer[0].pollen=0;
	transfer[0].rw_mode=0;
	transfer[0].sfc_mode=0;
	transfer[0].finally_len=0;
	ret=jz_sfc_pio_txrx(flash,transfer);

#if 0 //debug
	jz_sfc_nandflash_get_feature(flash, SPINAND_ADDR_FEATURE, &status);
	if(status & 0x01)
		quad_mode = 1;
#endif
	quad_mode = 1;
	return ret;
}
#endif

static int jz_sfc_nandflash_get_status(struct jz_sfc_nand *flash,u8 *status)
{
	int ret;
	struct sfc_transfer_nand transfer[1];
	transfer[0].sfc_cmd.cmd=SPINAND_CMD_GET_FEATURE;
	transfer[0].sfc_cmd.addr_low=SPINAND_ADDR_STATUS;
	transfer[0].sfc_cmd.addr_high=0;
	transfer[0].sfc_cmd.dummy_byte=0;
	transfer[0].sfc_cmd.addr_len=1;
	transfer[0].sfc_cmd.cmd_len=1;
	transfer[0].sfc_cmd.transmode=0;
	flash->use_dma=1;

	transfer[0].tx_buf  = NULL;
	transfer[0].rx_buf =  status;
	transfer[0].len=1;
	transfer[0].date_en=1;
	transfer[0].dma_mode=DMA_MODE;
	transfer[0].pollen=0;
	transfer[0].rw_mode=R_MODE;
	transfer[0].sfc_mode=0;
	transfer[0].finally_len=0;
	ret=jz_sfc_pio_txrx(flash,transfer);

	return ret;
}
static int jz_sfc_nandflash_erase_blk(struct jz_sfc_nand *flash,uint32_t addr)
{
	int ret;
	u8 status;
	struct sfc_transfer_nand transfer[1];
	int page = addr / flash->mtd.writesize;
	int timeout = 2000;
	jz_sfc_nandflash_write_enable(flash);
	switch(flash->mtd.erasesize){
		case SPINAND_OP_BL_128K:
			transfer[0].sfc_cmd.cmd = SPINAND_CMD_ERASE_128K;
			break;
		default:
			pr_info("Don't support the blksize to erase ! \n");
			break;
	}

	transfer[0].sfc_cmd.addr_low=page;
	transfer[0].sfc_cmd.addr_high=0;
	transfer[0].sfc_cmd.dummy_byte=0;
	transfer[0].sfc_cmd.addr_len=3;
	transfer[0].sfc_cmd.cmd_len=1;
	transfer[0].sfc_cmd.transmode=0;
	flash->use_dma=1;

	transfer[0].tx_buf  = NULL;
	transfer[0].rx_buf =  NULL;
	transfer[0].len=0;
	transfer[0].date_en=0;
	transfer[0].dma_mode=SLAVE_MODE;
	transfer[0].pollen=0;
	transfer[0].rw_mode=0;
	transfer[0].sfc_mode=0;
	ret=jz_sfc_pio_txrx(flash,transfer);
	if(ret<0)
		return ret;
	//	msleep((flash->tBERS + 999) / 1000);
	do{
		ret=jz_sfc_nandflash_get_status(flash,&status);
		if(ret<0)return ret;
		timeout--;
	}while((status & SPINAND_IS_BUSY) && (timeout > 0));
	if(timeout<=0)return -ETIMEDOUT;
	if(status & E_FALI){
		pr_info("Erase error,get state error ! %s %s %d \n",__FILE__,__func__,__LINE__);
		return -EIO;
	}
	return 0;

}
static int jz_sfc_nandflash_erase(struct mtd_info *mtd, struct erase_info *instr)
{
	int ret;
	uint32_t addr, end;
	int check_addr;
	struct jz_sfc_nand *flash;
	flash=to_jz_spi_nand(mtd);
	check_addr = ((unsigned int)instr->addr) % (mtd->erasesize);
	if (check_addr) {
		pr_info("%s line %d eraseaddr no align\n", __func__,__LINE__);
		return -EINVAL;
	}
	addr = (uint32_t)instr->addr;
	end = addr + (uint32_t)instr->len;
	instr->state = MTD_ERASING;;
	mutex_lock(&flash->lock);
	while (addr < end) {
		ret = jz_sfc_nandflash_erase_blk(flash, addr);
		if (ret) {
			pr_info("spi nand erase error blk id  %d !\n",addr / mtd->erasesize);
			instr->state = MTD_ERASE_FAILED;
			goto erase_exit;
		}
		addr += mtd->erasesize;
	}

	instr->state = MTD_ERASE_DONE;
erase_exit:
	mtd_erase_callback(instr);
	mutex_unlock(&flash->lock);
	return ret;
}
size_t jz_sfc_nandflash_read_ops(struct jz_sfc_nand *flash,u_char *buffer,int page, int column,size_t rlen,size_t *rel_rlen)
{
	int ret,timeout = 2000;
	int read_ret=0;
	struct sfc_transfer_nand transfer[1];
	u8 status;
	ret=jz_sfc_nand_loadpage_cmd(flash,page);
	if(ret<0)
		return ret;
	do{
		ret = jz_sfc_nandflash_get_status(flash,&status);
		if(ret<0)
			return ret;
		timeout--;
	}while((status & SPINAND_IS_BUSY) && (timeout > 0));
	if(timeout <=0){
		pr_info(" unknow timeout error sfc nand %d column = %d !!! %s %s %d \n",page,column,__FILE__,__func__,__LINE__);
		return -ETIMEDOUT;
	}
	status=(status>>4)&0x03;
	if(status == 0x2) {
		pr_info("spi nand read error page %d column = %d ret = %02x !!! %s %s %d \n",page,column,ret,__FILE__,__func__,__LINE__);
		status=-EBADMSG;
	}
	else if(status==3)status--;
	ret=jz_sfc_nand_read_cmd(flash,buffer,column,rlen);
	*rel_rlen=rlen;
	if(ret<0){
		pr_info("sfc nand read ops error !!! %s %s %d \n",__FILE__,__func__,__LINE__);
		return ret;
	}else
		return status;
}

static int jz_sfc_nandflash_read(struct mtd_info *mtd,loff_t addr,size_t len,u_char *buf,size_t *retlen)
{
	struct jz_sfc_nand *flash;
	int ret;
	int column;
	unsigned int rlen,page=0;
	int page_size = mtd->writesize;
	u_char *buffer = buf;
	unsigned int  ops_addr=(unsigned int)addr;
	size_t rel_rlen = 0;
	size_t ops_len = len;

	flash=to_jz_spi_nand(mtd);
	mutex_lock(&flash->lock);
	while(1)
	{
		column=ops_addr%page_size;
		page=ops_addr/page_size;
		rlen=min_t(unsigned int,ops_len,page_size-column);
		ret = jz_sfc_nandflash_read_ops(flash,flash->swap_buf,page,column,rlen,&rel_rlen);
		memcpy(buffer,flash->swap_buf,rlen);
		if(ret<0){
			break;
		}
		ops_len=ops_len-rlen;
		ops_addr=ops_addr+rlen;
		buffer=buffer+rlen;
		*retlen+=rlen;                  //check use ret_rlen
		if(ops_len==0)break;
		else if (ops_len<0){printk("text read error!\n");break;}
	}
	mutex_unlock(&flash->lock);
	return ret;
}

static size_t jz_sfc_nandflash_write(struct mtd_info *mtd,loff_t addr,size_t len,u_char *buf,int *retlen)
{
	int page_size=mtd->writesize;
	int wlen = 0;
	struct jz_sfc_nand *flash;
	int column;
	int ops_addr=(unsigned int) addr;
	int ops_len=len;
	u_char *buffer = buf;
	int page=0;
	int ret,timeout = 2000;
	u8 status;
	if(addr & (mtd->writesize - 1)){
		pr_info("wirte add don't align ,error ! %s %s %d \n",__FILE__,__func__,__LINE__);
		return -EINVAL;
	}
	*retlen=0;
	flash = to_jz_spi_nand(mtd);
	mutex_lock(&flash->lock);
	while(1){
		page=ops_addr/page_size;
		column=ops_addr%page_size;
		wlen=min_t(int,page_size-column,ops_len);
		memcpy(flash->swap_buf,buffer,wlen);

		jz_sfc_nandflash_write_enable(flash);

		ret=jz_sfc_nand_write_cmd(flash,column,wlen);
		if(ret<0){
			pr_info("spi nand write fail %s %s %d\n",__FILE__,__func__,__LINE__);
			break;
		}

		ret=jz_sfc_nand_pro_en_cmd(flash,page);
		if(ret<0){
			pr_info("spi nand write fail %s %s %d\n",__FILE__,__func__,__LINE__);
			break;
		}
		udelay(flash->tPROG);
		do{
			ret = jz_sfc_nandflash_get_status(flash,&status);
			if(ret<0){
				pr_info("spi nand write fail %s %s %d\n",__FILE__,__func__,__LINE__);
				break;
			}
			timeout--;
		}while((status & SPINAND_IS_BUSY) && (timeout > 0));
		if(status & p_FAIL){
			pr_info("spi nand write fail %s %s %d\n",__FILE__,__func__,__LINE__);
			ret= -EIO;
		}
		*retlen += wlen;
		ops_len -= wlen;
		ops_addr+=wlen;
		buffer += wlen;
		if(ret<0)	//write fail
			break;
		if(ops_len<=0){
			ret=0;
			break;
		}
	}
	mutex_unlock(&flash->lock);
	return ret;
}
static int jz_sfcnand_read_oob(struct mtd_info *mtd,loff_t addr,struct mtd_oob_ops *ops)
{
	struct jz_sfc_nand *flash;
	int column = mtd->writesize;
	int page = (unsigned int)addr / mtd->writesize;
	struct sfc_transfer_nand transfer[1];
	int ret,timeout = 2000;
	int readlen=0;

	u8 status;
	flash = to_jz_spi_nand(mtd);
	mutex_lock(&flash->lock);
	ret=jz_sfc_nand_loadpage_cmd(flash,page);
	if(ret<0){
		pr_info("spi nand read oob error !!! %s %s %d \n",__FILE__,__func__,__LINE__);
		goto read_oob_exit;
	}

	do{
		ret = jz_sfc_nandflash_get_status(flash,&status);
		if(ret<0)goto read_oob_exit;
		timeout--;
	}while((status & SPINAND_IS_BUSY) && (timeout > 0));
	if(timeout<=0){
		ret=-ETIMEDOUT;
		pr_info("spi nand read oob timeout !!! %s %s %d \n",__FILE__,__func__,__LINE__);
		goto read_oob_exit;
	}
	status=(status>>4)&0x03;
	if(status == 0x20) {
		pr_info("spi nand read oob error page %d column = %d ret = %02x !!! %s %s %d \n",__FILE__,__func__,__LINE__);
		status=-EBADMSG;
	}
	else if(status==3)status--;
	if(ops->datbuf){
		ret=jz_sfc_nand_read_cmd(flash,flash->swap_buf,0,mtd->writesize);
		readlen=mtd->writesize;
		memcpy(ops->datbuf,flash->swap_buf,readlen);
	}
	jz_sfc_nand_read_cmd(flash,flash->swap_buf+readlen,mtd->writesize+ops->ooboffs,ops->ooblen);
	memcpy(ops->oobbuf,flash->swap_buf+readlen,ops->ooblen);

read_oob_exit:
	mutex_unlock(&flash->lock);
	if(ret<0){
		pr_info("spi nand read error %s %s %d \n",__FILE__,__func__,__LINE__);
		return ret;
	}
	else
		return status;
}


static int jz_sfcnand_write_oob(struct mtd_info *mtd,loff_t addr,struct mtd_oob_ops *ops)
{
	struct sfc_transfer_nand transfer[1];
	struct jz_sfc_nand *flash;
	int ret,timeout=2000;
	int column = mtd->writesize;
	int page = ((unsigned int)addr) / mtd->writesize;
	flash = to_jz_spi_nand(mtd);
	u8 status;
	mutex_lock(&flash->lock);
	memcpy(flash->swap_buf,ops->oobbuf,ops->ooblen);

	ret=jz_sfc_nand_loadpage_cmd(flash,page);
	if(ret<0){
		pr_info("spi nand write oob fail %s %s %d\n",__FILE__,__func__,__LINE__);
		goto write_oob_exit;
	}
	memset(transfer,0,sizeof(struct sfc_transfer_nand));

	jz_sfc_nandflash_write_enable(flash);

	ret=jz_sfc_nand_write_cmd(flash,column,ops->ooblen);
	if(ret<0)
		goto write_oob_exit;

	ret=jz_sfc_nand_pro_en_cmd(flash,page);
	if(ret<0){
		pr_info("spi nand write oob fail %s %s %d\n",__FILE__,__func__,__LINE__);
		goto write_oob_exit;
	}
	udelay(flash->tPROG);

	do{
		ret = jz_sfc_nandflash_get_status(flash,&status);
		if(ret<0)goto write_oob_exit;
		timeout--;
	}while((status & SPINAND_IS_BUSY) && (timeout > 0));
	if(status & p_FAIL){
		pr_info("spi nand write oob fail %s %s %d\n",__FILE__,__func__,__LINE__);
		ret = -EIO;
		goto write_oob_exit;
	}
	ops->retlen=ops->ooblen;
write_oob_exit:
	mutex_unlock(&flash->lock);
	return ret;
}

static int jz_sfcnand_read(struct mtd_info *mtd, loff_t from,size_t len, size_t *retlen, unsigned char *buf)
{
	int ret;
	ret = jz_sfc_nandflash_read(mtd,from,len,buf,retlen);
	return ret;
}
static int jz_sfcnand_write(struct mtd_info *mtd, loff_t to, size_t len,size_t *retlen,u_char *buf)
{
	size_t ret;
	ret = jz_sfc_nandflash_write(mtd,to,len,buf,retlen);
	return ret;
}

static int jz_sfc_nandflash_block_checkbad(struct mtd_info *mtd, loff_t ofs,int getchip,int allowbbt)
{
	struct nand_chip *chip = mtd->priv;
	if (!chip->bbt)
		return chip->block_bad(mtd, ofs,getchip);
	/* Return info from the table */
	return nand_isbad_bbt(mtd, ofs, allowbbt);
}
static int jz_sfcnand_block_isbab(struct mtd_info *mtd,loff_t ofs)
{
	int ret;
	ret = jz_sfc_nandflash_block_checkbad(mtd, ofs,1, 0);
	return ret;
}
static int jz_sfcnand_block_markbad(struct mtd_info *mtd,loff_t ofs)
{
	struct nand_chip *chip = mtd->priv;
	int ret;
	pr_info("jz_sfcnand_block_markbad\n");
	ret = jz_sfcnand_block_isbab(mtd, ofs);
	if (ret) {
		/* If it was bad already, return success and do nothing */
		if (ret > 0)
			return 0;
		return ret;
	}
	return chip->block_markbad(mtd, ofs);
}
static int badblk_check(int len,unsigned char *buf)
{
	int i,bit0_cnt = 0;
	unsigned short *check_buf = (unsigned short *)buf;

	if(check_buf[0] != 0xff){
		for(i = 0; i < len * 8; i++){
			if(!((check_buf[0] >> 1) & 0x1))
				bit0_cnt++;
		}
	}
	if(bit0_cnt > 6 * len)
		return 1; // is bad blk
	return 0;
}
static int jz_sfcnand_block_bad_check(struct mtd_info *mtd, loff_t ofs,int getchip)
{
	int check_len = 2;
	unsigned char check_buf[] = {0xaa,0xaa};
	struct mtd_oob_ops ops;
	ops.oobbuf = check_buf;
	ops.ooblen = check_len;
	jz_sfcnand_read_oob(mtd,ofs,&ops);
	if(badblk_check(check_len,check_buf))
		return 1;
	return 0;
}
static int jz_sfcnand_erase(struct mtd_info *mtd, struct erase_info *instr)
{
	int addr,ret;
	addr = instr->addr;

	ret = jz_sfc_nandflash_erase(mtd,instr);
	if(ret){
		pr_info("WARNING: block %d erase fail !\n",addr / mtd->erasesize);

		ret = jz_sfcnand_block_markbad(mtd,addr);
		if(ret){
			pr_info("mark bad block error, there will occur error,so exit !\n");
			return -1;
		}
	}
	instr->state = MTD_ERASE_DONE;
	return 0;
}
static int jz_sfc_nandflash_block_markbad(struct mtd_info *mtd, loff_t ofs)
{
	struct nand_chip *chip = mtd->priv;
	uint8_t buf[2] = { 0, 0 };
	int  res, ret = 0, i = 0;
	int write_oob = !(chip->bbt_options & NAND_BBT_NO_OOB_BBM);

	/* Write bad block marker to OOB */
	if (write_oob) {
		struct mtd_oob_ops ops;
		loff_t wr_ofs = ofs;
		ops.datbuf = NULL;
		ops.oobbuf = buf;
		ops.ooboffs = chip->badblockpos;
		if (chip->options & NAND_BUSWIDTH_16) {
			ops.ooboffs &= ~0x01;
			ops.len = ops.ooblen = 2;
		} else {
			ops.len = ops.ooblen = 1;
		}
		ops.mode = MTD_OPS_PLACE_OOB;

		/* Write to first/last page(s) if necessary */
		if (chip->bbt_options & NAND_BBT_SCANLASTPAGE)
			wr_ofs += mtd->erasesize - mtd->writesize;
		do {
			res = jz_sfcnand_write_oob(mtd, wr_ofs, &ops);
			if (!ret)
				wr_ofs += mtd->writesize;
		} while ((chip->bbt_options & NAND_BBT_SCAN2NDPAGE) && i < 2);
	}
	/* Update flash-based bad block table */
	if (chip->bbt_options & NAND_BBT_USE_FLASH) {
		res = nand_update_bbt(mtd, ofs);
		if (!ret)
			ret = res;
	}
	if (!ret)
		mtd->ecc_stats.badblocks++;

	return ret;
}
static int jz_sfc_nand_ext_init(struct jz_sfc_nand *flash)
{
	int ret;
	u8 status;
	struct sfc_transfer_nand transfer[1];
	unsigned int id;

    id = jz_sfc_detect_id(flash);

	transfer[0].sfc_cmd.cmd=SPINAND_CMD_SET_FEATURE;
    transfer[0].sfc_cmd.addr_low=0xa000;
    transfer[0].sfc_cmd.addr_high=0;
    transfer[0].sfc_cmd.dummy_byte=0;
    transfer[0].sfc_cmd.addr_len=2;
    transfer[0].sfc_cmd.cmd_len=1;
    transfer[0].sfc_cmd.transmode=0;
    transfer[0].dma_mode=0;
    transfer[0].tx_buf = NULL;
    transfer[0].rx_buf = NULL;
    transfer[0].len=0;
    transfer[0].date_en=0;
    transfer[0].dma_mode=SLAVE_MODE;
    transfer[0].pollen=0;
    transfer[0].rw_mode=0;
    transfer[0].sfc_mode=0;
    ret=jz_sfc_pio_txrx(flash,transfer);
    if (ret < 0)
        return ret;

	transfer[0].sfc_cmd.cmd=SPINAND_CMD_SET_FEATURE;
	transfer[0].sfc_cmd.addr_low=0xb010;
	transfer[0].sfc_cmd.addr_high=0;
	transfer[0].sfc_cmd.dummy_byte=0;
	transfer[0].sfc_cmd.addr_len=2;
	transfer[0].sfc_cmd.cmd_len=1;
	transfer[0].sfc_cmd.transmode=0;
	if ((id&0xff) == 0xef){
	    transfer[0].sfc_cmd.addr_low |= 1<<3;
	}
	transfer[0].dma_mode=0;
	transfer[0].tx_buf = NULL;
	transfer[0].rx_buf = NULL;
	transfer[0].len=0;
	transfer[0].date_en=0;
	transfer[0].dma_mode=SLAVE_MODE;
	transfer[0].pollen=0;
	transfer[0].rw_mode=0;
	transfer[0].sfc_mode=0;
	ret=jz_sfc_pio_txrx(flash,transfer);
    if (ret < 0)
        return ret;

	return 0;
}
static int get_pagesize_from_nand(struct jz_sfc_nand *flash,int page,int column)
{
	char *buffer=NULL;
	int page_size=0;
	int rlen;
	buffer=kzalloc(100,GFP_KERNEL);
	if(!buffer)return -ENOMEM;
	jz_sfc_nandflash_read_ops(flash,buffer,page,column,100,&rlen);
	page_size=buffer[SPL_TYPE_FLAG_LEN+5]*1024;
	kfree(buffer);
	return page_size;
}
static void convert_burner_to_driver_use(struct jz_spi_support *spinand,struct jz_spi_support_from_burner *burner,int param_num)
{
	int i=0;
	for(i=0;i<param_num;i++){
		spinand[i].id_manufactory=(burner[i].chip_id>>8)&0xff;
		spinand[i].id_device=(burner[i].chip_id)&0x000000ff;
		memcpy(spinand[i].name,burner[i].name,SIZEOF_NAME);
		spinand[i].page_size=burner[i].page_size;
		spinand[i].oobsize=burner[i].oobsize;
		spinand[i].sector_size=burner[i].sector_size;
		spinand[i].block_size=burner[i].block_size;
		spinand[i].size=burner[i].size;
		spinand[i].page_num=burner[i].page_num;
		spinand[i].tRD_maxbusy=burner[i].tRD_maxbusy;
		spinand[i].tPROG_maxbusy=burner[i].tPROG_maxbusy;
		spinand[i].tBERS_maxbusy=burner[i].tBERS_maxbusy;
		spinand[i].column_cmdaddr_bits=burner[i].column_cmdaddr_bits;
	}
}

static int transfer_to_mtddriver_struct(struct get_chip_param *param,struct jz_spi_nand_platform_data **change)
{
	*change=kzalloc(sizeof(struct jz_spi_nand_platform_data),GFP_KERNEL);
	if(!*change)
		return -ENOMEM;
	(*change)->num_spi_flash=param->para_num;
	(*change)->jz_spi_support=kzalloc(param->para_num*sizeof(struct jz_spi_support),GFP_KERNEL);
	if(!(*change)->jz_spi_support)
		return -ENOMEM;
	memcpy((*change)->jz_spi_support,param->addr,param->para_num*sizeof(struct jz_spi_support));
	convert_burner_to_driver_use((*change)->jz_spi_support,param->addr,param->para_num);

	(*change)->num_partitions=param->partition_num;
	(*change)->mtd_partition=kzalloc(param->partition_num*sizeof(struct mtd_partition),GFP_KERNEL);
	if(!(*change)->mtd_partition){
		return -ENOMEM;
	}
	int i=0;
	for(i=0;i<(*change)->num_partitions;i++)
	{
		(*change)->mtd_partition[i].name=kzalloc(32*sizeof(char),GFP_KERNEL);
		if(!(*change)->mtd_partition[i].name)
			return -ENOMEM;
		memcpy((*change)->mtd_partition[i].name,param->partition[i].name,32);
		(*change)->mtd_partition[i].size=param->partition[i].size;
		(*change)->mtd_partition[i].offset=param->partition[i].offset;
		(*change)->mtd_partition[i].mask_flags=param->partition[i].mask_flags;

	}
	return 0;
}
static int jz_get_sfcnand_param(struct jz_spi_nand_platform_data **param,struct jz_sfc_nand *flash,int *nand_magic)
{
	//first get pagesize
	int rlen;
	int page_size;
	struct get_chip_param param_from_burner;
	char *buffer=NULL;
	char *member_addr=NULL;
	int i=0;
	for(i=0;i<2;i++){
		flash->column_cmdaddr_bits=24;
		if(i==1)
			flash->column_cmdaddr_bits=32;
		*nand_magic=0;
		page_size=get_pagesize_from_nand(flash,0,0);
		if(page_size>0&&page_size<4000)
			buffer=kzalloc(page_size,GFP_KERNEL);
		else
			continue;
		if(!buffer)
			return -ENOMEM;
		jz_sfc_nandflash_read_ops(flash,buffer,SPIFLASH_PARAMER_OFFSET/page_size,SPIFLASH_PARAMER_OFFSET%page_size,
				page_size,&rlen);
		*nand_magic=*(int32_t *)(buffer);
		printk("nand_magic=0x%x",*nand_magic);
		if(*nand_magic!=0x6e616e64){
			kfree(buffer);
			if(i==1)
				return 0;
		}else
			break;
	}
	if(i>=2){
		printk("error in read magic\n");
		return -1;
	}
	member_addr=buffer+sizeof(int32_t);
	param_from_burner.version=*(int *)member_addr;
	member_addr+=sizeof(param_from_burner.version);
	param_from_burner.flash_type=*(int *)member_addr;
	member_addr+=sizeof(param_from_burner.flash_type);
	param_from_burner.para_num=*(int *)member_addr;
	member_addr+=sizeof(param_from_burner.para_num);
	param_from_burner.addr=(struct jz_spi_support_from_burner *)member_addr;
	member_addr+=param_from_burner.para_num*sizeof(struct jz_spi_support_from_burner);
	param_from_burner.partition_num=*(int *)(member_addr);
	member_addr+=sizeof(param_from_burner.partition_num);
	param_from_burner.partition=(struct jz_spinand_partition *)member_addr;
	transfer_to_mtddriver_struct(&param_from_burner,param);
	return 0;
}
static int __init jz_sfc_probe(struct platform_device *pdev)
{
	struct jz_sfc_nand *flash;//flash->board_info是jz_spi_nand_platform_data
	const char *jz_probe_types[] = {"cmdlinepart",NULL};
	struct jz_spi_support *spi_flash;

	int err = 0,ret = 0;
	struct nand_chip *chip;
	struct mtd_partition *mtd_sfcnand_partition;
	int num_partitions;
	struct jz_spi_nand_platform_data *param;
	int nand_magic= 0;
	chip = kzalloc(sizeof(struct nand_chip),GFP_KERNEL);
	if(!chip)
		return -ENOMEM;

	print_dbg("function : %s, line : %d\n", __func__, __LINE__);

	flash = kzalloc(sizeof(struct jz_sfc_nand), GFP_KERNEL);
	if (!flash) {
		printk("%s---%s---%d\n", __FILE__, __func__, __LINE__);
		printk("kzalloc() error !\n");
		kfree(chip);
		return -ENOMEM;
	}

	flash->dev = &pdev->dev;
	flash->pdata = pdev->dev.platform_data;
	/*	if (flash->pdata == NULL) {
		dev_err(&pdev->dev, "No platform data supplied\n");
		goto err_no_pdata;
		}
		*/	flash->threshold = THRESHOLD;
	ret=sfc_nand_plat_resource_init(flash,pdev);
	if(ret<0)
		switch (ret)
		{
			case -1:goto err_no_iores;break;
			case -2:goto err_no_iores;break;
			case -3:goto err_no_iomap;break;
			case -4:goto err_no_irq;break;
			case -5:goto err_no_clk;break;
			case -6:goto err_no_iomap;break;
			case -7:goto err_no_iores;break;
		};
	flash->chnl= flash->pdata->chnl;
	flash->tx_addr_plus = 0;
	flash->rx_addr_plus = 0;
	flash->use_dma = 0;
	err=0;

	/* find and map our resources */
	/* SFC controller initializations for SFC */
	jz_sfc_init_setup(flash);
	platform_set_drvdata(pdev, flash);
	init_completion(&flash->done);
	spin_lock_init(&flash->lock_rxtx);
	spin_lock_init(&flash->lock_status);
	mutex_init(&flash->lock);
	flash->threshold = THRESHOLD;
	err = request_irq(flash->irq, jz_sfc_irq, 0, pdev->name, flash);
	if (err) {
		dev_err(&pdev->dev, "Cannot claim IRQ\n");
	}
	jz_sfc_nand_ext_init(flash);
#ifdef CONFIG_SPI_QUAD
	jz_sfc_nandflash_set_quad_mode(flash);
#endif
	jz_get_sfcnand_param(&param,flash,&nand_magic);
	if(nand_magic==0x6e616e64){
		(flash->pdata)->board_info = (void *)param;
	}else{
		param=(flash->pdata)->board_info;
	}
	mtd_sfcnand_partition=param->mtd_partition;
	num_partitions=param->num_partitions;
#if 0
	printk("-----------------test param-------------------------------------\n");
	printk("param number=%d\n",param->num_spi_flash);
	printk("partition number=%d\n",param->num_partitions);
	printk("chip id=%x\n",param->jz_spi_support->id_manufactory);
	printk("chip name=%s\n",param->jz_spi_support->name);
	printk("partition0 name=%s\n",param->mtd_partition[0].name);
	printk("partition1 name=%s\n",param->mtd_partition[1].name);
	printk("partition2 name=%s\n",param->mtd_partition[2].name);
	printk("partition3 name=%s\n",param->mtd_partition[3].name);
	printk("-------------------end------------------------------------------\n");
#endif
	if (flash->pdata == NULL) {
		dev_err(&pdev->dev, "No platform data supplied\n");
		goto err_no_pdata;
	}

	spi_flash = jz_sfc_nand_probe(flash);
	flash->mtd.name = "sfc_nand";
	flash->mtd.owner = THIS_MODULE;
	flash->mtd.type = MTD_NANDFLASH;
	flash->mtd.flags |= MTD_CAP_NANDFLASH;
	flash->mtd.erasesize = spi_flash->block_size;
	flash->mtd.writesize = spi_flash->page_size;
	flash->mtd.size = spi_flash->size;
	flash->mtd.oobsize = spi_flash->oobsize;
	flash->mtd.writebufsize = flash->mtd.writesize;
	flash->column_cmdaddr_bits = spi_flash->column_cmdaddr_bits;
	flash->tRD = spi_flash->tRD_maxbusy;
	flash->tPROG = spi_flash->tPROG_maxbusy;
	flash->tBERS = spi_flash->tBERS_maxbusy;
	flash->mtd.bitflip_threshold = flash->mtd.ecc_strength = 2;
	chip->select_chip = NULL;
	chip->badblockbits = 8;
	chip->scan_bbt = nand_default_bbt;
	chip->block_bad = jz_sfcnand_block_bad_check;
	chip->block_markbad = jz_sfc_nandflash_block_markbad;
	//chip->ecc.layout= &gd5f_ecc_layout_128; // for erase ops
	chip->bbt_erase_shift = chip->phys_erase_shift = ffs(flash->mtd.erasesize) - 1;
	if (!(chip->options & NAND_OWN_BUFFERS))
		chip->buffers = kmalloc(sizeof(*chip->buffers), GFP_KERNEL);

	/* Set the bad block position */
	if (flash->mtd.writesize > 512 || (chip->options & NAND_BUSWIDTH_16))
		chip->badblockpos = NAND_LARGE_BADBLOCK_POS;
	else
		chip->badblockpos = NAND_SMALL_BADBLOCK_POS;

	flash->mtd.priv = chip;

	flash->mtd._erase = jz_sfcnand_erase;
	flash->mtd._read = jz_sfcnand_read;
	flash->mtd._write = jz_sfcnand_write;
	flash->mtd._read_oob = jz_sfcnand_read_oob;
	flash->mtd._write_oob = jz_sfcnand_write_oob;
	flash->mtd._block_isbad = jz_sfcnand_block_isbab;
	flash->mtd._block_markbad = jz_sfcnand_block_markbad;

	chip->scan_bbt(&flash->mtd);
	ret = mtd_device_parse_register(&flash->mtd,jz_probe_types,NULL, mtd_sfcnand_partition, num_partitions);
	if (ret) {
		kfree(flash);
		return -ENODEV;
	}
	/*	unsigned char user_buf[2]={};
		int rrrr=0,llen;
		struct erase_info ssa={
		.addr=0x900000,
		.len=flash->mtd.erasesize,
		.mtd=&(flash->mtd),
		.callback=0,
		};
		jz_sfcnand_erase(&(flash->mtd), &ssa);
		unsigned char  *bufff=(unsigned char *)kmalloc(2048*3,GFP_KERNEL);
		for(rrrr=0;rrrr<2049;rrrr++)
		bufff[rrrr]=rrrr;
		ret=jz_sfcnand_write(&(flash->mtd),0x900002, 2049,&llen, bufff);
		memset(bufff,0,3*2048);
		jz_sfcnand_read(&(flash->mtd),0x900000,2080, &rrrr, bufff);
		printk("-----------readbuff-------------\n");
		for(rrrr=0;rrrr<2080;)
		{
		printk("%x,",bufff[rrrr]);
		rrrr++;
		if(rrrr%16==0)printk("\n");
		}
		*/
	return 0;
err_no_clk:
	clk_put(flash->clk_gate);
	clk_put(flash->clk);
err_no_irq:
	free_irq(flash->irq, flash);
err_no_iomap:
	iounmap(flash->iomem);
err_no_iores:
err_no_pdata:
	release_resource(flash->ioarea);
	kfree(flash->ioarea);
	return err;
}

static int __exit jz_sfc_remove(struct platform_device *pdev)
{
	struct jz_sfc_nand *flash = platform_get_drvdata(pdev);


	clk_disable(flash->clk_gate);
	clk_put(flash->clk_gate);

	clk_disable(flash->clk);
	clk_put(flash->clk);

	free_irq(flash->irq, flash);

	iounmap(flash->iomem);

	release_mem_region(flash->resource->start, resource_size(flash->resource));

	platform_set_drvdata(pdev, NULL);

	return 0;
}

static int jz_sfc_suspend(struct platform_device *pdev, pm_message_t msg)
{
	unsigned long flags;
	struct jz_sfc_nand *flash = platform_get_drvdata(pdev);

	spin_lock_irqsave(&flash->lock_status, flags);
	flash->status |= STATUS_SUSPND;
	disable_irq(flash->irq);
	spin_unlock_irqrestore(&flash->lock_status, flags);

	clk_disable(flash->clk_gate);

	clk_disable(flash->clk);

	return 0;
}

static int jz_sfc_resume(struct platform_device *pdev)
{
	unsigned long flags;
	struct jz_sfc_nand *flash = platform_get_drvdata(pdev);

	clk_enable(flash->clk);

	clk_enable(flash->clk_gate);

	spin_lock_irqsave(&flash->lock_status, flags);
	flash->status &= ~STATUS_SUSPND;
	enable_irq(flash->irq);
	spin_unlock_irqrestore(&flash->lock_status, flags);

	return 0;
}

void jz_sfc_shutdown(struct platform_device *pdev)
{
	unsigned long flags;
	struct jz_sfc_nand *flash = platform_get_drvdata(pdev);

	spin_lock_irqsave(&flash->lock_status, flags);
	flash->status |= STATUS_SUSPND;
	disable_irq(flash->irq);
	spin_unlock_irqrestore(&flash->lock_status, flags);

	clk_disable(flash->clk_gate);

	clk_disable(flash->clk);

	return ;
}

static struct platform_driver jz_sfcdrv = {
	.driver		= {
		.name	= "jz-sfc",
		.owner	= THIS_MODULE,
	},
	.remove		= jz_sfc_remove,
	.suspend	= jz_sfc_suspend,
	.resume		= jz_sfc_resume,
	.shutdown	= jz_sfc_shutdown,
};

static int __init jz_sfc_init(void)
{
	print_dbg("function : %s, line : %d\n", __func__, __LINE__);
	return platform_driver_probe(&jz_sfcdrv, jz_sfc_probe);
}

static void __exit jz_sfc_exit(void)
{
	print_dbg("function : %s, line : %d\n", __func__, __LINE__);
	platform_driver_unregister(&jz_sfcdrv);
}

module_init(jz_sfc_init);
module_exit(jz_sfc_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("JZ SFC Driver");
