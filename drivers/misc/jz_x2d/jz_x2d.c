/*
 * linux/drivers/misc/jz_x2d_core.c -- Ingenic EXtreme 2D driver
 *
 * Copyright (C) 2005-2012, Ingenic Semiconductor Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/major.h>
#include <linux/string.h>
#include <linux/fcntl.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/spinlock.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/major.h>
#include <linux/version.h>
#include <linux/io.h>
#include <linux/memory.h>
/*#include <linux/earlysuspend.h>*/
#include <linux/dma-mapping.h>

#include <asm/cacheflush.h>
#include <asm/mipsregs.h>
#include <asm/mipsmtregs.h>
#include <asm/pgtable.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <asm/thread_info.h>

#include <mach/jzcpm_pwc.h>
#include "jz_x2d.h"
#include "jz_x2d_reg.h"

#define USE_DMMU_TLB
#define X2D_PAGE_SIZE 0x1000
#define X2D_PAGE_VALID_BIT 0x1
#define X2D_TLB_TABLE_SIZE 0x00200000   // 2M
#define X2D_MAX_ADDR_RECORD_NUM 1024
#define DUMMY_TLB_ADDR 0x8f000001
#define JZ4780_X2D_WTHDOG_1S 0xa0000000
#define X2D_LAYER_OFFSET 0x1000

#define X2D_SCALE_FACTOR 512

//#define X2D_DEBUG
//#define CLEAR_DST

#ifdef X2D_DEBUG
#define X2D_SLV_GLB_TIME 0xf01c
#define X2D_SLV_BWR_DATA 0xf020
#define X2D_SLV_BWR_CYC 0xf024
#define X2D_SLV_BRD_DATA 0xf028
#define X2D_SLV_BRD_CYC 0xf02c
#endif

/**************************************global parameters********************************/
#define X2D_NAME        "x2d"

struct x2d_device {
	int irq;
	int proc_num;
	pid_t hold_proc;

	struct file *cur_filp;

	void __iomem *base;
	struct device *dev;
	struct resource * mem;
	struct miscdevice misc_dev;
	struct clk *x2d_clk;
	void* cpm_pwc;
	unsigned int wd_cnt;

	enum jz_x2d_state state;
	enum jz_x2d_errcode errcode;

	x2d_chain_info * chain_p;
	wait_queue_head_t set_wait_queue;
	int wake_up_condition;

	struct list_head proc_list;
/*	struct early_suspend early_suspend;*/
	struct mutex x2d_lock;
	struct mutex compose_lock;
};

/************************************function declare******************************************/
static int x2d_open(struct inode *inode, struct file *filp);
static int x2d_release(struct inode *inode, struct file *filp);
static ssize_t x2d_read(struct file *filp, char *buf, size_t size, loff_t *l);
static ssize_t x2d_write(struct file *filp, const char *buf, size_t size, loff_t *l);
static long x2d_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);

/***********************************internal interface*****************************************/
static unsigned long reg_read(struct x2d_device *jz_x2d, int offset)
{
	return readl(jz_x2d->base + offset);
}

static void reg_write(struct x2d_device *jz_x2d, int offset, unsigned long val)
{
	writel(val, jz_x2d->base + offset);
}

static void bit_set(struct x2d_device *jz_x2d, int offset, int bit)
{
	int val = 0;

	val = reg_read(jz_x2d, offset);
	val |= (1<<bit);
	reg_write(jz_x2d, offset, val);
}

#if 0 //not use
static void bit_clr(struct x2d_device *jz_x2d, int offset, int bit)
{
	int val = 0;

	val = reg_read(jz_x2d, offset);
	val &= ~(1<<bit);
	reg_write(jz_x2d, offset, val);
}

static void bit_clr(struct x2d_device *jz_x2d, int offset, int bit)
{
	int val = 0;
	val = reg_read(jz_x2d, offset);
	val &= ~(1<<bit);
	reg_write(jz_x2d, offset, val);
}

static int get_srcfmt_bpp(int format)
{
	switch (format) {
		case X2D_INFORMAT_ARGB888:
			return 32;
		case X2D_INFORMAT_RGB555:
		case X2D_INFORMAT_RGB565:
			return 16;
		case X2D_INFORMAT_YUV420SP:
		case X2D_INFORMAT_TILE420:
		case X2D_INFORMAT_NV21:
		case X2D_INFORMAT_NV12:
			return 12;
		default:
			pr_err("Error!!unknow src format %d \n",format);
			return -1;
	}
}

static int get_dstfmt_bpp(int format)
{
	if ( format & NEW_DST_FORMAT_BIT) {
		format = (format>>BIT_X2D_DST_RGB_FORMAT)&3;
	}

	switch (format) {
		case X2D_OUTFORMAT_ARGB888:
		case X2D_OUTFORMAT_XARGB888:
			return 32;
		case X2D_OUTFORMAT_RGB565:
		case X2D_OUTFORMAT_RGB555:
			return 16;
		default:
			pr_err("Error!!unknow dst format %d\n",format);
			return -1;
	}
}

int x2d_get_state(void)
{
	return jz_x2d->state;
}

int x2d_get_errcode(void)
{
	return jz_x2d->errcode;
}
#endif

static unsigned int get_phy_addr(unsigned int vaddr)
{
	unsigned int addr = vaddr & (PAGE_SIZE-1);
	pgd_t   *pgdir;
#ifdef CONFIG_PGTABLE_4
	pud_t	*pudir;
#endif
	pmd_t   *pmdir;
	pte_t   *pte;

	pgdir = pgd_offset(current->mm, vaddr);
	if (pgd_none(*pgdir) || pgd_bad(*pgdir))
		return -EINVAL;

#ifdef CONFIG_PGTABLE_4
	pudir = pud_offset(pgdir, vaddr);
	if (pud_none(*pudir) || pud_bad(*pudir))
		return -EINVAL;

	pmdir = pmd_offset(pudir, vaddr);
	if (pmd_none(*pmdir) || pmd_bad(*pmdir))
		return -EINVAL;
#else
	pmdir = pmd_offset((pud_t *)pgdir, vaddr);
	if (pmd_none(*pmdir) || pmd_bad(*pmdir))
		return -EINVAL;
#endif

	pte = pte_offset(pmdir, vaddr);

	if (pte_present(*pte)) {
		return addr | (pte_pfn(*pte)<<PAGE_SHIFT);
		//pte_page(*pte);
	}
	return -EINVAL;
}

/**************************************tlb interface************************************/
static int if_recorded_in_list(int vaddr, struct x2d_proc_info* proc)
{
	int i;
	int find_flag = 0;

	for(i = 0; i < proc->record_addr_num; i++) {
		if(vaddr == proc->record_addr_list[i] && 1) {   /*valid page*/
			find_flag = 1;
			break;
		}
	}
	return find_flag;
}

void fill_tlb_address(int vaddr, int lenth, struct x2d_proc_info* proc)
{

	int page_num,i,addr,paddr,tlb_pos,s_pos;
	if(if_recorded_in_list(vaddr,proc))
		return;

	proc->record_addr_list[proc->record_addr_num] = vaddr;
	proc->record_addr_num++;

	addr = (vaddr >> 12)<<12;
	page_num = ((vaddr - addr) + lenth + X2D_PAGE_SIZE - 1) >> 12;
	tlb_pos = (vaddr >> 12 ) << 2;

	s_pos = tlb_pos;

	for (i = 0; i < page_num; i++) {
		int *p_tlb;
		paddr = get_phy_addr(addr);
		p_tlb = (int *)(proc->tlb_base + tlb_pos);
		*p_tlb = paddr + X2D_PAGE_VALID_BIT;
		tlb_pos+=4;
		addr += X2D_PAGE_SIZE;
	}
	dma_cache_wback_inv(proc->tlb_base + s_pos,page_num * 4);
	dma_cache_wback_inv(vaddr,lenth);
}

#ifndef USE_DMMU_TLB
void transfer_config_address(struct x2d_proc_info* proc)
{
	int i = 0;
	int lenth = 0;

	/***********************dst address transfer*********************/
	lenth = proc->configs.dst_width * proc->configs.dst_height *
		get_dstfmt_bpp(proc->configs.dst_format)/8;
	fill_tlb_address(proc->configs.dst_address, lenth,  proc);

	/**********************src address transfer**********************/
	for(i = 0; i < proc->configs.layer_num; i++) {
		lenth = proc->configs.lay[i].in_width * proc->configs.lay[i].in_height *
			get_srcfmt_bpp(proc->configs.lay[i].format)/8;
		fill_tlb_address(proc->configs.lay[i].addr, lenth, proc);
	}
}
#endif

/*******************************process functions*********************************/
static struct x2d_proc_info* x2d_index_procinfo(struct x2d_device *jz_x2d, struct file *filp)
{
	struct x2d_proc_info *x2d_proc = NULL;

	list_for_each_entry(x2d_proc, &jz_x2d->proc_list, list) {
		if(filp == x2d_proc->x2d_filp) {
			dev_dbg(jz_x2d->dev, "%s %p", __func__, filp);
			return x2d_proc;
		}
	}

	dev_err(jz_x2d->dev,"cannot find the proc %p from chain", filp);

	return NULL;
}

#ifndef USE_DMMU_TLB
static int create_tlb_table(struct x2d_proc_info* proc)
{
	int i = 0;
	int *tlb_p = NULL;
	int *addr_list = NULL;

	tlb_p = kzalloc(X2D_TLB_TABLE_SIZE,GFP_KERNEL);  //alloc 2M tlb mem
	if (!tlb_p) {
		pr_err("malloc tlb table fail\n");
		return -1;
	}

	for (i = 0; i < (X2D_TLB_TABLE_SIZE>>2); i++)   //avoid x2d  hardware bug,must do it!
		*(tlb_p + i) = DUMMY_TLB_ADDR;

	addr_list = kzalloc(GFP_KERNEL, X2D_MAX_ADDR_RECORD_NUM*sizeof(int));
	if (!addr_list) {
		pr_err("malloc addr list fail\n");
		return -1;
	}

	proc->tlb_base = (int)tlb_p;
	proc->record_addr_list = addr_list;
	proc->record_addr_num = 0;

	return 0;
}

static int free_tlb_table(struct x2d_proc_info* proc)
{
	kzfree((int *)proc->tlb_base);
	kzfree(proc->record_addr_list);
	proc->tlb_base = 0;
	proc->record_addr_list = NULL;

	return 0;
}
#endif

static int x2d_create_procinfo(struct x2d_device *jz_x2d, struct file *filp)
{
	struct x2d_proc_info *x2d_proc = NULL;

	x2d_proc = kzalloc(sizeof(struct x2d_proc_info), GFP_KERNEL);
	if (!x2d_proc) {
		dev_err(jz_x2d->dev,"malloc for struct proc_info fail\n");
		return -EFAULT;
	}
	memset(x2d_proc, 0, sizeof(struct x2d_proc_info));
	x2d_proc->pid = current->pid;
	x2d_proc->x2d_filp = filp;
	list_add_tail(&x2d_proc->list,&jz_x2d->proc_list);

#ifndef USE_DMMU_TLB
	ret = create_tlb_table(x2d_proc);
	if(ret != 0) {
		dev_err(jz_x2d->dev,"creat tlb for proc %d  fail\n",p->pid);
		return -1;
	}
#endif
	jz_x2d->proc_num++;

	dev_dbg(jz_x2d->dev,"X2d has opened by %d processes\n",jz_x2d->proc_num);
	return 0;
}

static int x2d_free_procinfo(struct x2d_device *jz_x2d, struct file *filp)
{
	struct x2d_proc_info *x2d_proc = NULL;

	x2d_proc = x2d_index_procinfo(jz_x2d, filp);
	if (!x2d_proc) {
		dev_err(jz_x2d->dev,"free_tlb_table  cannot find proc %p\n", filp);
		return -EFAULT;
	}

#ifndef USE_DMMU_TLB
	free_tlb_table(x2d_proc);
#endif

	list_del_init(&x2d_proc->list);
	kfree(x2d_proc);
	x2d_proc = NULL;
	jz_x2d->proc_num--;

	dev_dbg(jz_x2d->dev,"A process is freed,now x2d has %d process\n",jz_x2d->proc_num);

	return 0;
}

#ifdef X2D_DEBUG
static void x2d_dump_config(struct x2d_device *jz_x2d, struct x2d_proc_info *p)
{
	int i;
	dev_info(jz_x2d->dev, "x2d_process_info  pid: %d \n tlb_base: %08x\n record_addr_num: %d\n",
			 p->pid, p->tlb_base, p->record_addr_num);
	dev_info(jz_x2d->dev, "watchdog_cnt: %d\ntlb_base:%08x\ndst_address:%08x\n",
			 p->configs.watchdog_cnt, p->configs.tlb_base, p->configs.dst_address);
#ifdef X2D_USE_PHY_ADDR
	dev_info(jz_x2d->dev, "dst_address_p:%08x\n",
			p->configs.dst_address_p);
#endif
	dev_info(jz_x2d->dev, "dst_alpha_val: %d\n dst_stride:%d\n dst_mask_val:%08x\n",
			 p->configs.dst_alpha_val, p->configs.dst_stride, p->configs.dst_mask_val);
	dev_info(jz_x2d->dev, "dst_width: %d\n dst_height:%d\n dst_bcground:%08x\n",
			 p->configs.dst_width, p->configs.dst_height, p->configs.dst_bcground);
	dev_info(jz_x2d->dev, "dst_format: %d\n dst_back_en:%08x\n dst_preRGB_en:%08x\n",
			 p->configs.dst_format, p->configs.dst_back_en, p->configs.dst_preRGB_en);
	dev_info(jz_x2d->dev, "dst_glb_alpha_en: %d\n dst_backpure_en:%08x\n configs.layer_num: %d\n",
			 p->configs.dst_glb_alpha_en, p->configs.dst_mask_en, p->configs.layer_num);

	for (i = 0; i < p->configs.layer_num; i++) {
		dev_info(jz_x2d->dev,"layer[%d]: ======================================\n", i);
		dev_info(jz_x2d->dev, "format: %d\n, transform: %d\n, global_alpha_val: %d\n, argb_order: %d\n",
				 p->configs.lay[i].format, p->configs.lay[i].transform,
				 p->configs.lay[i].global_alpha_val, p->configs.lay[i].argb_order);
		dev_info(jz_x2d->dev, "osd_mode: %d\n preRGB_en: %d\n glb_alpha_en: %d\n mask_en: %d\n msk_val=%x\n",
				 p->configs.lay[i].osd_mode, p->configs.lay[i].preRGB_en,
				 p->configs.lay[i].glb_alpha_en, p->configs.lay[i].mask_en,p->configs.lay[i].msk_val);
		dev_info(jz_x2d->dev, "color_cov_en: %d\n in_width: %d\n in_height: %d\n out_width: %d\n",
				 p->configs.lay[i].color_cov_en, p->configs.lay[i].in_width,
				 p->configs.lay[i].in_height, p->configs.lay[i].out_width);
		dev_info(jz_x2d->dev, "out_height: %d\n out_w_offset: %d\n out_h_offset: %d\n v_scale_ratio: %d\n",
				 p->configs.lay[i].out_height, p->configs.lay[i].out_w_offset,
				 p->configs.lay[i].out_h_offset, p->configs.lay[i].v_scale_ratio);
		dev_info(jz_x2d->dev, "h_scale_ratio: %d\n yuv address addr: %08x\n u_addr: %08x\n v_addr: %08x\n",
				 p->configs.lay[i].h_scale_ratio, p->configs.lay[i].addr,
				 p->configs.lay[i].u_addr, p->configs.lay[i].v_addr);
#ifdef X2D_USE_PHY_ADDR
		dev_info(jz_x2d->dev, "yuv phy addr addr: %08x\n u_addr_p: %08x\n v_addr_p: %08x\n",
				 p->configs.lay[i].addr_p, p->configs.lay[i].u_addr_p, p->configs.lay[i].v_addr_p);
#endif

		dev_info(jz_x2d->dev, "y_stride: %d\n v_stride: %d\n",
				 p->configs.lay[i].y_stride, p->configs.lay[i].v_stride);
	}
}
#endif

static void x2d_dump_reg(struct x2d_device *jz_x2d, struct x2d_proc_info* p)
{
	int j =0;

	dev_info(jz_x2d->dev,"pid is %d   current is %d\n", p->pid,current->pid);
	dev_info(jz_x2d->dev,"REG_X2D_GLB_CTRL %lx\n", reg_read(jz_x2d,REG_X2D_GLB_CTRL));
	dev_info(jz_x2d->dev,"REG_X2D_GLB_STATUS %lx\n", reg_read(jz_x2d,REG_X2D_GLB_STATUS));
	dev_info(jz_x2d->dev,"REG_X2D_GLB_TRIG %lx\n", reg_read(jz_x2d,REG_X2D_GLB_TRIG));
	dev_info(jz_x2d->dev,"REG_X2D_DHA %lx\n", reg_read(jz_x2d,REG_X2D_DHA));
	dev_info(jz_x2d->dev,"REG_X2D_TLB_BASE %lx\n", reg_read(jz_x2d,REG_X2D_TLB_BASE));
	dev_info(jz_x2d->dev,"REG_X2D_WDOG_CNT %lx\n", reg_read(jz_x2d,REG_X2D_WDOG_CNT));
	dev_info(jz_x2d->dev,"REG_X2D_LAY_GCTRL %lx\n", reg_read(jz_x2d,REG_X2D_LAY_GCTRL));
	dev_info(jz_x2d->dev,"REG_X2D_DST_BASE %lx\n", reg_read(jz_x2d,REG_X2D_DST_BASE));
	dev_info(jz_x2d->dev,"REG_X2D_DST_CTRL_STR %lx\n", reg_read(jz_x2d,REG_X2D_DST_CTRL_STR));
	dev_info(jz_x2d->dev,"REG_X2D_DST_GS %lx\n", reg_read(jz_x2d,REG_X2D_DST_GS));
	dev_info(jz_x2d->dev,"REG_X2D_DST_MSK_ARGB %lx\n", reg_read(jz_x2d,REG_X2D_DST_MSK_ARGB));
	dev_info(jz_x2d->dev,"REG_X2D_DST_FMT %lx\n", reg_read(jz_x2d,REG_X2D_DST_FMT));
	for (j = 0; j < 4; j++) {
		dev_info(jz_x2d->dev,"REG_X2D_LAY%d_CTRL  %lx\n", j, reg_read(jz_x2d,REG_X2D_LAY0_CTRL + j*0x1000));
		dev_info(jz_x2d->dev,"REG_X2D_LAY%d_Y_ADDR  %lx\n", j, reg_read(jz_x2d,REG_X2D_LAY0_Y_ADDR+ j*0x1000));
		dev_info(jz_x2d->dev,"REG_X2D_LAY%d_U_ADDR  %lx\n", j, reg_read(jz_x2d,REG_X2D_LAY0_U_ADDR+ j*0x1000));
		dev_info(jz_x2d->dev,"REG_X2D_LAY%d_V_ADDR  %lx\n", j, reg_read(jz_x2d,REG_X2D_LAY0_V_ADDR+ j*0x1000));
		dev_info(jz_x2d->dev,"REG_X2D_LAY%d_IN_FM_GS %lx\n", j, reg_read(jz_x2d,REG_X2D_LAY0_IN_FM_GS+ j*0x1000));
		dev_info(jz_x2d->dev,"REG_X2D_LAY%d_STRIDE  %lx\n", j, reg_read(jz_x2d,REG_X2D_LAY0_STRIDE+ j*0x1000));
		dev_info(jz_x2d->dev,"REG_X2D_LAY%d_OUT_GS  %lx\n", j, reg_read(jz_x2d,REG_X2D_LAY0_OUT_GS+ j*0x1000));
		dev_info(jz_x2d->dev,"REG_X2D_LAY%d_OOSFT  %lx\n", j, reg_read(jz_x2d,REG_X2D_LAY0_OOSFT+ j*0x1000));
		dev_info(jz_x2d->dev,"REG_X2D_LAY%d_RSZ_COEF  %lx\n", j, reg_read(jz_x2d,REG_X2D_LAY0_RSZ_COEF+ j*0x1000));
		dev_info(jz_x2d->dev,"REG_X2D_LAY%d_BK_ARGB %lx\n", j, reg_read(jz_x2d,REG_X2D_LAY0_BK_ARGB+ j*0x1000));
	}
}

static inline void x2d_set_wake_up_condition(struct x2d_device *jz_x2d)
{
	jz_x2d->wake_up_condition = 1;
}

static inline void x2d_clear_wake_up_condition(struct x2d_device *jz_x2d)
{
	jz_x2d->wake_up_condition = 0;
}

static int x2d_check_wake_up_condition(struct x2d_device *jz_x2d)
{
	return jz_x2d->wake_up_condition;
}

/**************************************main function******************************************/
static int jz_x2d_start_compose(struct x2d_device *jz_x2d, struct file *filp)
{
	int i = 0,dst_stride = 0;
	struct x2d_proc_info * x2d_proc = NULL;

	mutex_lock(&jz_x2d->compose_lock);

	jz_x2d->state = x2d_state_calc;
	x2d_proc = x2d_index_procinfo(jz_x2d, filp);
	if (!x2d_proc) {
		dev_err(jz_x2d->dev, "%s x2d_proc is NULL!", __func__);
		mutex_unlock(&jz_x2d->compose_lock);
		return -EFAULT;
	}
	jz_x2d->cur_filp = filp;

#ifdef X2D_DEBUG
	x2d_dump_config(jz_x2d, x2d_proc);
#endif

	clk_enable(jz_x2d->x2d_clk);
	__x2d_reset_trig();
	//udelay(1);
#ifdef X2D_USE_PHY_ADDR
	__x2d_setup_default();
#else
	__x2d_setup_default_vaddr();
#endif
	__x2d_enable_dma();

#ifdef USE_DMMU_TLB
	if (x2d_proc->configs.tlb_base) {
		reg_write(jz_x2d, REG_X2D_TLB_BASE, x2d_proc->configs.tlb_base);
	} else {
		dev_err(jz_x2d->dev, "x2d_proc->configs.tlb_base is zero");
		clk_disable(jz_x2d->x2d_clk);
		mutex_unlock(&jz_x2d->compose_lock);
		return -EFAULT;
	}
#else
	reg_write(jz_x2d, REG_X2D_TLB_BASE,
			  (unsigned long)virt_to_phys((void *)x2d_proc->tlb_base));
#endif

//	reg_write(jz_x2d, REG_X2D_WDOG_CNT, JZ4780_X2D_WTHDOG_1S);
	reg_write(jz_x2d, REG_X2D_WDOG_CNT, jz_x2d->wd_cnt);
	__x2d_enable_wthdog();

	reg_write(jz_x2d, REG_X2D_LAY_GCTRL, x2d_proc->configs.layer_num);
	reg_write(jz_x2d, REG_X2D_DHA, virt_to_phys(jz_x2d->chain_p));

	memset(jz_x2d->chain_p, 0, sizeof(x2d_chain_info));

#if 0
	x2d_proc->configs.dst_back_en = 0;
	x2d_proc->configs.dst_glb_alpha_en = 1;
	x2d_proc->configs.dst_preRGB_en  = 0;
	x2d_proc->configs.dst_mask_en = 1;
	x2d_proc->configs.dst_alx2d_procha_val = 0x80;
#endif
#ifdef X2D_USE_PHY_ADDR
	if((unsigned int)x2d_proc->configs.dst_address != 0){
		__x2d_set_dst_tlb();
		jz_x2d->chain_p->dst_addr = x2d_proc->configs.dst_address;
	}else if((unsigned int)x2d_proc->configs.dst_address_p != 0){
		__x2d_clr_dst_tlb();
		jz_x2d->chain_p->dst_addr = x2d_proc->configs.dst_address_p;
	}else{
		dev_err(jz_x2d->dev,"x2d dst addr is error!!!");
		return -1;
	}
#else
	jz_x2d->chain_p->dst_addr = x2d_proc->configs.dst_address;
#endif

	/*dst_stride must 8 aligned*/
	if(x2d_proc->configs.dst_stride % 8 != 0){
		dev_err(jz_x2d->dev,"Notes: dst stride (%d) should be 8 aligned\n",x2d_proc->configs.dst_stride);
	}
	dst_stride = (x2d_proc->configs.dst_stride) & (~0x7);

	jz_x2d->chain_p->dst_ctrl_str = (dst_stride << BIT_X2D_DST_STRIDE) \
					|(x2d_proc->configs.dst_back_en << BIT_X2D_DST_BG_EN) \
					|(x2d_proc->configs.dst_glb_alpha_en << BIT_X2D_DST_GLB_ALPHA_EN)\
					|(x2d_proc->configs.dst_preRGB_en << BIT_X2D_DST_PREM_EN)\
					|(x2d_proc->configs.dst_mask_en << BIT_X2D_DST_MSK_EN)\
					|(x2d_proc->configs.dst_alpha_val << BIT_X2D_DST_GLB_ALPHA_VAL);
	jz_x2d->chain_p->dst_height = (uint16_t)x2d_proc->configs.dst_height;
	jz_x2d->chain_p->dst_width = (uint16_t)x2d_proc->configs.dst_width;
	jz_x2d->chain_p->overlay_num = x2d_proc->configs.layer_num;
	jz_x2d->chain_p->dst_tile_en  = 0;

	if ( x2d_proc->configs.dst_format & NEW_DST_FORMAT_BIT ) {
		jz_x2d->chain_p->dst_fmt = x2d_proc->configs.dst_format & ~(NEW_DST_FORMAT_BIT);
	}
	else {
		jz_x2d->chain_p->dst_fmt = (X2D_ALPHA_POSHIGH << BIT_X2D_DST_ALPHA_POS)\
			|(x2d_proc->configs.dst_format << BIT_X2D_DST_RGB_FORMAT)\
			|(X2D_RGBORDER_RGB << BIT_X2D_DST_RGB_ORDER); // orig code
		//|(X2D_RGBORDER_BGR << BIT_X2D_DST_RGB_ORDER); // abgr
	}
	//dev_err(jz_x2d->dev, "jz_x2d->chain_p->dst_fmt=0x%x, x2d_proc->configs.dst_format=%#x\n", jz_x2d->chain_p->dst_fmt, x2d_proc->configs.dst_format);
	jz_x2d->chain_p->dst_argb = x2d_proc->configs.dst_bcground;

	for (i = 0; i < x2d_proc->configs.layer_num; i++) {
#ifdef CLEAR_DST
		static int bg_count;
		x2d_proc->configs.lay[i].mask_en = 1;
		if (bg_count % 2) {
			x2d_proc->configs.lay[i].msk_val = 0xffff0000;
		} else {
			x2d_proc->configs.lay[i].msk_val = 0xff0000ff;
		}
		bg_count++;
#endif


#if 0 //def SRC_ALPHA_TEST
		p->configs.lay[i].msk_val = 0;//0xff0000ff;
		if (i == 1) {
			x2d_proc->configs.lay[i].glb_alpha_en = 1;
			x2d_proc->configs.lay[i].global_alpha_val = 0x80;
		}
		x2d_proc->configs.lay[i].preRGB_en = 0;
#endif

#ifdef X2D_USE_PHY_ADDR
		if((uint32_t)x2d_proc->configs.lay[i].addr != 0){
			__x2d_set_lay_tlb(i);
			jz_x2d->chain_p->x2d_lays[i].y_addr = (uint32_t)x2d_proc->configs.lay[i].addr;
			jz_x2d->chain_p->x2d_lays[i].v_addr = (uint32_t)x2d_proc->configs.lay[i].v_addr;
			jz_x2d->chain_p->x2d_lays[i].u_addr = (uint32_t)x2d_proc->configs.lay[i].u_addr;
		}else if((uint32_t)x2d_proc->configs.lay[i].addr_p != 0){
			__x2d_clr_lay_tlb(i);
			jz_x2d->chain_p->x2d_lays[i].y_addr = (uint32_t)x2d_proc->configs.lay[i].addr_p;
			jz_x2d->chain_p->x2d_lays[i].v_addr = (uint32_t)x2d_proc->configs.lay[i].v_addr_p;
			jz_x2d->chain_p->x2d_lays[i].u_addr = (uint32_t)x2d_proc->configs.lay[i].u_addr_p;
		}else{
			dev_err(jz_x2d->dev,"x2d layer[%d] addr is error!!!",i);
			return -1;
		}
#else
		jz_x2d->chain_p->x2d_lays[i].y_addr = (uint32_t)x2d_proc->configs.lay[i].addr;
		jz_x2d->chain_p->x2d_lays[i].v_addr = (uint32_t)x2d_proc->configs.lay[i].v_addr;
		jz_x2d->chain_p->x2d_lays[i].u_addr = (uint32_t)x2d_proc->configs.lay[i].u_addr;
#endif
		//the CSCM_EN mau affects the color of video for x2d
		jz_x2d->chain_p->x2d_lays[i].lay_ctrl =(x2d_proc->configs.lay[i].glb_alpha_en << BIT_X2D_LAY_GLB_ALPHA_EN)\
			|(x2d_proc->configs.lay[i].mask_en << BIT_X2D_LAY_MSK_EN)			\
			/* |((x2d_proc->configs.lay[i].format > X2D_INFORMAT_RGB565) << BIT_X2D_LAY_CSCM_EN)*/ \
			|(x2d_proc->configs.lay[i].preRGB_en << BIT_X2D_LAY_PREM_EN)		\
			|(x2d_proc->configs.lay[i].format << BIT_X2D_LAY_INPUT_FORMAT)		\
			;

		jz_x2d->chain_p->x2d_lays[i].lay_galpha =(uint8_t)x2d_proc->configs.lay[i].global_alpha_val;
		jz_x2d->chain_p->x2d_lays[i].rom_ctrl = (uint8_t)x2d_proc->configs.lay[i].transform;
		jz_x2d->chain_p->x2d_lays[i].RGBM = (uint8_t)x2d_proc->configs.lay[i].argb_order \
			|(x2d_proc->configs.lay[i].osd_mode << 4);

		if (x2d_proc->configs.lay[i].format == Tile_YUV420) {
			jz_x2d->chain_p->x2d_lays[i].swidth = (uint16_t)x2d_proc->configs.lay[i].in_width & ~0xf;
			jz_x2d->chain_p->x2d_lays[i].sheight = (uint16_t)x2d_proc->configs.lay[i].in_height & ~0xf;
		}else {
			jz_x2d->chain_p->x2d_lays[i].swidth = (uint16_t)x2d_proc->configs.lay[i].in_width;
			jz_x2d->chain_p->x2d_lays[i].sheight = (uint16_t)x2d_proc->configs.lay[i].in_height;
		}
		jz_x2d->chain_p->x2d_lays[i].ystr = (uint16_t)x2d_proc->configs.lay[i].y_stride;
		jz_x2d->chain_p->x2d_lays[i].uvstr = (uint16_t)x2d_proc->configs.lay[i].v_stride;
		jz_x2d->chain_p->x2d_lays[i].owidth = (uint16_t)x2d_proc->configs.lay[i].out_width;
		jz_x2d->chain_p->x2d_lays[i].oheight= (uint16_t)x2d_proc->configs.lay[i].out_height;
		jz_x2d->chain_p->x2d_lays[i].oxoffset= (uint16_t)x2d_proc->configs.lay[i].out_w_offset;
		jz_x2d->chain_p->x2d_lays[i].oyoffset= (uint16_t)x2d_proc->configs.lay[i].out_h_offset;

		if (x2d_proc->configs.lay[i].format == Tile_YUV420) {
			switch (x2d_proc->configs.lay[i].transform) {
				case X2D_H_MIRROR:
				case X2D_V_MIRROR:
				case X2D_ROTATE_0:
				case X2D_ROTATE_180:
					jz_x2d->chain_p->x2d_lays[i].rsz_hcoef = \
						(uint16_t)(((uint32_t)jz_x2d->chain_p->x2d_lays[i].swidth * X2D_SCALE_FACTOR) \
						/(uint16_t)jz_x2d->chain_p->x2d_lays[i].owidth);
					jz_x2d->chain_p->x2d_lays[i].rsz_vcoef = \
						(uint16_t)(((uint32_t)jz_x2d->chain_p->x2d_lays[i].sheight * X2D_SCALE_FACTOR) \
						/(uint16_t)jz_x2d->chain_p->x2d_lays[i].oheight);
					if(jz_x2d->chain_p->x2d_lays[i].swidth < jz_x2d->chain_p->x2d_lays[i].owidth)
						jz_x2d->chain_p->x2d_lays[i].rsz_hcoef -= 1;
					if(jz_x2d->chain_p->x2d_lays[i].sheight < jz_x2d->chain_p->x2d_lays[i].oheight)
						jz_x2d->chain_p->x2d_lays[i].rsz_vcoef -= 1;
					break;
				case X2D_ROTATE_90:
				case X2D_ROTATE_270:
					jz_x2d->chain_p->x2d_lays[i].rsz_hcoef = \
						(uint16_t)(((uint32_t)jz_x2d->chain_p->x2d_lays[i].swidth * X2D_SCALE_FACTOR) \
						/(uint16_t)jz_x2d->chain_p->x2d_lays[i].oheight);
					jz_x2d->chain_p->x2d_lays[i].rsz_vcoef = \
						(uint16_t)(((uint32_t)jz_x2d->chain_p->x2d_lays[i].sheight * X2D_SCALE_FACTOR) \
						/(uint16_t)jz_x2d->chain_p->x2d_lays[i].owidth);
					if(jz_x2d->chain_p->x2d_lays[i].swidth < jz_x2d->chain_p->x2d_lays[i].oheight)
						jz_x2d->chain_p->x2d_lays[i].rsz_hcoef -= 1;
					if(jz_x2d->chain_p->x2d_lays[i].sheight < jz_x2d->chain_p->x2d_lays[i].owidth)
						jz_x2d->chain_p->x2d_lays[i].rsz_vcoef -= 1;
					break;
				default:
					dev_err(jz_x2d->dev,"undefined rotation degree!!!!");
			}
		} else {
			jz_x2d->chain_p->x2d_lays[i].rsz_hcoef = (uint16_t)x2d_proc->configs.lay[i].h_scale_ratio;
			jz_x2d->chain_p->x2d_lays[i].rsz_vcoef = (uint16_t)x2d_proc->configs.lay[i].v_scale_ratio;
		}
		jz_x2d->chain_p->x2d_lays[i].bk_argb = x2d_proc->configs.lay[i].msk_val;
	}
	dma_cache_wback_inv((unsigned long)jz_x2d->chain_p,sizeof(x2d_chain_info));

#ifdef X2D_DEBUG
	//__x2d_enable_wthdog();
	x2d_dump_reg(jz_x2d,x2d_proc);
#endif
	x2d_clear_wake_up_condition(jz_x2d);
	__x2d_enable_irq();
	__x2d_start_trig();

	if(!wait_event_timeout(jz_x2d->set_wait_queue, \
				x2d_check_wake_up_condition(jz_x2d), HZ/5)) {
		dev_err(jz_x2d->dev,"wait queue time out  %lx\n", reg_read(jz_x2d,REG_X2D_GLB_STATUS));
		__x2d_stop_trig();
		x2d_dump_reg(jz_x2d,x2d_proc);
		clk_disable(jz_x2d->x2d_clk);
		mutex_unlock(&jz_x2d->compose_lock);
		dev_err(jz_x2d->dev,"wait queue time out  %lx\n", reg_read(jz_x2d,REG_X2D_GLB_STATUS));
		return -1;
	}

#ifdef X2D_DEBUG
	uint32_t consum_time = reg_read(jz_x2d, X2D_SLV_GLB_TIME);
    int cycle_per_pixel = (consum_time)/(x2d_proc->configs.dst_height*x2d_proc->configs.dst_width+1);
	int taget_cycle_per_pixel = (400*1000000)/(1920*1280*30);

	uint32_t wdata_trans_num = reg_read(jz_x2d, X2D_SLV_BWR_DATA);
	uint32_t wdata_trans_cycle = reg_read(jz_x2d, X2D_SLV_BWR_CYC);

	uint32_t rdata_trans_num = reg_read(jz_x2d, X2D_SLV_BRD_DATA);
	uint32_t rdata_trans_cycle = reg_read(jz_x2d, X2D_SLV_BRD_CYC);

	dev_info(jz_x2d->dev, " > GKER_W --cycle: %d time:%dns/per_pix target: %dns %d%% \n",
			 consum_time, cycle_per_pixel,
		   taget_cycle_per_pixel, cycle_per_pixel/taget_cycle_per_pixel);
	dev_info(jz_x2d->dev, " > write data num: %d, cycle: %d, effect: %d%%\n",
			 wdata_trans_num, wdata_trans_cycle, wdata_trans_num/(wdata_trans_cycle+1));
	dev_info(jz_x2d->dev, " > read data num: %d, cycle: %d, effect: %d%%\n",
			 rdata_trans_num, rdata_trans_cycle, rdata_trans_num/(rdata_trans_cycle+1));
#endif

#ifdef X2D_DEBUG
	x2d_dump_reg(jz_x2d,x2d_proc);
	dev_info(jz_x2d->dev, "+++++++++<================================>\n");
#endif

	clk_disable(jz_x2d->x2d_clk);
	mutex_unlock(&jz_x2d->compose_lock);

	return 0;
}

static int jz_x2d_set_config(struct x2d_device *jz_x2d,
							 struct jz_x2d_config *config, struct file *filp)
{
	struct x2d_proc_info *x2d_proc = NULL;

	mutex_lock(&jz_x2d->x2d_lock);
	x2d_proc = x2d_index_procinfo(jz_x2d, filp);
	if (!x2d_proc) {
		mutex_unlock(&jz_x2d->x2d_lock);
		dev_err(jz_x2d->dev, "%s x2d_index_procinfo Failed!", __func__);
		return -EFAULT;
	}

	if (copy_from_user(&x2d_proc->configs, (void *)config, sizeof(struct jz_x2d_config))) {
		mutex_unlock(&jz_x2d->x2d_lock);
		dev_err(jz_x2d->dev, "copy_from_user Failed!!!");
		return -EFAULT;
	}

#ifdef X2D_DEBUG
	x2d_dump_config(jz_x2d, x2d_proc);
#endif

#ifndef USE_DMMU_TLB
	transfer_config_address(x2d_proc);
#endif

	dev_dbg(jz_x2d->dev, "function: %s line: %d---->\n", __func__, __LINE__);
	mutex_unlock(&jz_x2d->x2d_lock);

	return 0;
}

int jz_x2d_get_proc_config(struct x2d_device *jz_x2d,
						   struct jz_x2d_config* config, struct file *filp)
{
	struct x2d_proc_info *x2d_proc = NULL;

	x2d_proc = x2d_index_procinfo(jz_x2d, filp);
	if (!x2d_proc) {
		dev_err(jz_x2d->dev, "%s x2d_index_procinfo failed!", __func__);
		return -EFAULT;
	}

	if (copy_to_user((void *)config, &x2d_proc->configs, sizeof(struct jz_x2d_config))) {
		dev_err(jz_x2d->dev, "%s copy_to_user failed!", __func__);
		return -EFAULT;
	}

	return 0;
}

int jz_x2d_stop_calc(struct x2d_device *jz_x2d, struct file *filp)
{
	if(filp == jz_x2d->cur_filp) {
		__x2d_stop_trig();
		dev_dbg(jz_x2d->dev,"proc %p stop x2d calculating\n",jz_x2d->cur_filp);
	} else {
		dev_err(jz_x2d->dev,"proc %p want to stop x2d ,hold proc is %p\n",
				filp, jz_x2d->cur_filp);
		return -EFAULT;
	}

	return 0;
}

/***************************************irq handle******************************************/
static irqreturn_t x2d_irq_handler(int irq, void *dev_id)
{
	struct x2d_device *jz_x2d = (struct x2d_device *)dev_id;
	unsigned int status_reg = 0;

	status_reg = reg_read(jz_x2d,REG_X2D_GLB_STATUS);

	if (status_reg & X2D_WTDOG_ERR) {
		dev_err(jz_x2d->dev, "Error:x2d watch dog time out!!!!");
		jz_x2d->errcode = error_wthdog;
	} else if (!(status_reg & X2D_BUSY)) {
		jz_x2d->errcode = error_none;
	}

	x2d_set_wake_up_condition(jz_x2d);

	if (jz_x2d->state == x2d_state_suspend) {
		dev_info(jz_x2d->dev, "%s x2d already suspend!", __func__);
		__x2d_stop_trig();
		__x2d_clear_irq();
		clk_disable(jz_x2d->x2d_clk);
		wake_up(&jz_x2d->set_wait_queue);
		clk_enable(jz_x2d->x2d_clk);
	}

	jz_x2d->state = x2d_state_complete;
	__x2d_clear_irq();
	wake_up(&jz_x2d->set_wait_queue);

	return IRQ_HANDLED;
}
#if 0
/*****************************suspend  resume********************************/
static void x2d_early_suspend(struct early_suspend *handler)
{
	struct x2d_device *jz_x2d = container_of(handler, struct x2d_device, early_suspend);
	jz_x2d->state = x2d_state_suspend;
}

static void x2d_early_resume(struct early_suspend *handler)
{
	struct x2d_device *jz_x2d = container_of(handler, struct x2d_device, early_suspend);
	jz_x2d->state = x2d_state_idle;
}
#endif
/****************************sys call functions******************************/
static inline struct x2d_device *file_to_x2d(struct file *file)
{
	struct miscdevice *dev = file->private_data;

	return container_of(dev, struct x2d_device, misc_dev);
}

static int x2d_open(struct inode *inode, struct file *filp)
{
	int ret = 0;
	struct x2d_device *jz_x2d = NULL;
	if (filp == NULL) {
		printk("error: filp == NULL");
		return -EFAULT;
	}

	jz_x2d = file_to_x2d(filp);
	if (jz_x2d == NULL) {
		printk("can't open x2d to filp: %p\n", filp);
		return -EFAULT;
	}
/*	dev_info(jz_x2d->dev, "x2d open struct file: %p", filp);*/

	mutex_lock(&jz_x2d->x2d_lock);
	ret = x2d_create_procinfo(jz_x2d, filp);
	if (ret < 0) {
		mutex_unlock(&jz_x2d->x2d_lock);
		dev_err(jz_x2d->dev, "x2d_create_procinfo failed!");
		return -EFAULT;
	}
	mutex_unlock(&jz_x2d->x2d_lock);

	return ret;
}

static int x2d_release(struct inode *inode, struct file *filp)
{
	int ret = 0;
	struct x2d_device *jz_x2d = NULL;

	jz_x2d = file_to_x2d(filp);
/*	dev_info(jz_x2d->dev, "x2d release struct file: %p", filp);*/

	mutex_lock(&jz_x2d->x2d_lock);
	ret = x2d_free_procinfo(jz_x2d, filp);
	if (ret < 0) {
		mutex_unlock(&jz_x2d->x2d_lock);
		dev_err(jz_x2d->dev, "x2d_free_procinfo failed!");
		return -EFAULT;
	}

	if (0 == jz_x2d->proc_num)
		clk_disable(jz_x2d->x2d_clk);  //stop x2d hardware
	mutex_unlock(&jz_x2d->x2d_lock);

	return 0;
}

/**************************
 *     IOCTL Handlers     *
 **************************/
static long x2d_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int retval = 0;
	struct x2d_device *jz_x2d = NULL;

	jz_x2d = file_to_x2d(filp);

	if (_IOC_TYPE(cmd) != X2D_IOCTL_MAGIC) {
		dev_err(jz_x2d->dev, "invalid cmd!\n");
		return -EFAULT;
	}

	switch (cmd) {
		case IOCTL_X2D_SET_CONFIG:
			retval = jz_x2d_set_config(jz_x2d, (struct jz_x2d_config*)arg, filp);
			break;
		case IOCTL_X2D_START_COMPOSE:
			retval = jz_x2d_start_compose(jz_x2d, filp);
			break;
		case IOCTL_X2D_GET_SYSINFO:
			retval = jz_x2d_get_proc_config(jz_x2d,(void *)arg, filp);
			break;
		case IOCTL_X2D_STOP:
			retval = jz_x2d_stop_calc(jz_x2d, filp);
			break;
		case IOCTL_X2D_MAP_GRAPHIC_BUF:
			//retval = jz_x2d_map_graphic_buf();
			break;
		case IOCTL_X2D_FREE_GRAPHIC_BUF:
			//retval = jz_x2d_free_graphic_buf();
			break;
		default:
			dev_info(jz_x2d->dev, "Not supported command: 0x%x\n", cmd);
			return -EINVAL;
	}

	return retval;
}

static ssize_t x2d_read(struct file *filp, char *buf, size_t size, loff_t *l)
{
	return -1;
}

static ssize_t x2d_write(struct file *filp, const char *buf, size_t size, loff_t *l)
{
	return -1;
}

static struct file_operations x2d_fops =
{
			open:			x2d_open,
			release:		x2d_release,
			read:			x2d_read,
			write:			x2d_write,
			unlocked_ioctl:	x2d_ioctl,
			//mmap:			x2d_mmap,
};

static int x2d_probe(struct platform_device *pdev)
{
	int ret = 0;
	unsigned short x2d_id = 0;
	struct resource *mem = NULL;
	struct x2d_device *jz_x2d = NULL;
	struct clk *clk_h0;

	jz_x2d = kzalloc(sizeof(struct x2d_device),GFP_KERNEL);
	if (jz_x2d == NULL) {
		dev_err(&pdev->dev, "Error alloc jz_x2d no memory!\n");
		return -ENOMEM;
	}

	jz_x2d->misc_dev.minor     = MISC_DYNAMIC_MINOR;
	jz_x2d->misc_dev.name      = "x2d";
	jz_x2d->misc_dev.fops      = &x2d_fops;
	jz_x2d->dev = &pdev->dev;

	jz_x2d->chain_p = kzalloc(sizeof(x2d_chain_info), GFP_KERNEL);
	if (jz_x2d->chain_p == NULL) {
		dev_err(&pdev->dev, "Error alloc jz_x2d->chain_p no memory!\n");
		return -ENOMEM;
	}
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem) {
		dev_err(&pdev->dev, "Failed to get register memory resource\n");
		ret = -ENXIO;
		goto err_exit;
	}
	mem = request_mem_region(mem->start, resource_size(mem), pdev->name);
	if (!mem) {
		dev_err(&pdev->dev, "Failed to request register memory region\n");
		ret = -EBUSY;
		goto err_exit;
	}
	jz_x2d->mem = mem;
	jz_x2d->base = ioremap(mem->start, resource_size(mem));
	if (!jz_x2d->base) {
		dev_err(&pdev->dev, "Failed to ioremap register memory region\n");
		ret = -EBUSY;
		goto err_exit;
	}

	jz_x2d->x2d_clk = clk_get(&pdev->dev, "x2d");
	if (IS_ERR(jz_x2d->x2d_clk)) {
		ret = PTR_ERR(jz_x2d->x2d_clk );
		dev_err(&pdev->dev, "Failed to get x2d clock: %d\n", ret);
		goto err_exit;
	}

	clk_h0 = clk_get(NULL,"h0clk");
	if (!IS_ERR(clk_h0))
		jz_x2d->wd_cnt = clk_get_rate(clk_h0);
	else
		jz_x2d->wd_cnt = JZ4780_X2D_WTHDOG_1S;

#ifdef CONFIG_SOC_4775
	jz_x2d->cpm_pwc = cpm_pwc_get(PWC_X2D);
	if(jz_x2d->cpm_pwc == NULL) {
		dev_err(&pdev->dev, "get %s fail!\n",PWC_X2D);
		goto err_exit;
	}
#else
	jz_x2d->cpm_pwc = NULL;
#endif
	if(jz_x2d->cpm_pwc)
		cpm_pwc_enable(jz_x2d->cpm_pwc);

	//jz_x2d->platdev = pdev;
	clk_enable(jz_x2d->x2d_clk);

	/* the hardware don't support x2d clock frequency set, so we don't need to set its rate */
	//clk_set_rate(jz_x2d->x2d_clk, 300000000);

	x2d_id = __x2d_read_devid();
	if (x2d_id != X2D_ID) {
		dev_err(&pdev->dev, "invalid x2d ID 0x%x\n",x2d_id);
		ret = -EINVAL;
		goto err_exit;
	}
	jz_x2d->irq = platform_get_irq(pdev, 0);
	if (request_irq(jz_x2d->irq, x2d_irq_handler, IRQF_SHARED,pdev->name, jz_x2d)) {
		dev_err(&pdev->dev,  "request_irq return error, ret=%d\n", ret);
		dev_err(&pdev->dev,  "X2d could not get IRQ\n");
		ret = -EINVAL;
		goto err_exit;
	}

	dev_set_drvdata(&pdev->dev, jz_x2d);

	ret = misc_register(&jz_x2d->misc_dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "misc_register failed!");
		goto err_exit;;
	}

	jz_x2d->state = x2d_state_idle;
	jz_x2d->errcode = error_none;
	INIT_LIST_HEAD(&jz_x2d->proc_list);
	jz_x2d->proc_num = 0;
	init_waitqueue_head(&jz_x2d->set_wait_queue);
	mutex_init(&jz_x2d->compose_lock);
	mutex_init(&jz_x2d->x2d_lock);

#ifdef CONFIG_HAS_EARLYSUSPEND
/*	jz_x2d->early_suspend.suspend = x2d_early_suspend;
	jz_x2d->early_suspend.resume = x2d_early_resume;
	jz_x2d->early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB;
	register_early_suspend(&jz_x2d->early_suspend);*/
#endif

	clk_disable(jz_x2d->x2d_clk);
	dev_info(&pdev->dev, "Virtual Driver of JZ X2D registered\n");

	printk("Virtual Driver of JZ X2D registered\n");

	return 0;

err_exit:
	if (!IS_ERR(jz_x2d->x2d_clk))
		clk_put(jz_x2d->x2d_clk);
	if (mem)
		release_mem_region(mem->start, resource_size(mem));
	if (jz_x2d->irq)
		free_irq(jz_x2d->irq, jz_x2d);
	if (jz_x2d->base)
		iounmap(jz_x2d->base);
	if (jz_x2d->chain_p)
		kzfree(jz_x2d->chain_p);
	if (jz_x2d)
		kzfree(jz_x2d);
	return ret;
}

static int x2d_remove(struct platform_device *pdev)
{
	struct x2d_device *jz_x2d = platform_get_drvdata(pdev);

	if(jz_x2d->cpm_pwc)
		cpm_pwc_put(jz_x2d->cpm_pwc);
	iounmap(jz_x2d->base);
	free_irq(jz_x2d->irq,jz_x2d);
	release_mem_region(jz_x2d->mem->start, resource_size(jz_x2d->mem));
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&jz_x2d->early_suspend);
#endif
	kzfree(jz_x2d->chain_p);
	kzfree(jz_x2d);
	misc_deregister(&jz_x2d->misc_dev);

	return 0;
}

static int x2d_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct x2d_device *jz_x2d = platform_get_drvdata(pdev);

	clk_disable(jz_x2d->x2d_clk);
	return 0;
}

static int x2d_resume(struct platform_device *pdev)
{
	struct x2d_device *jz_x2d = platform_get_drvdata(pdev);

	clk_enable(jz_x2d->x2d_clk);
	return 0;
}

static struct platform_driver x2d_driver = {
	.driver.name	= "x2d",
	.driver.owner	= THIS_MODULE,
	.probe		= x2d_probe,
	.remove		= x2d_remove,
	.suspend	= x2d_suspend,
	.resume		= x2d_resume,
};

static int __init x2d_init(void)
{
	return platform_driver_register(&x2d_driver);
}

static void __exit x2d_exit(void)
{
	platform_driver_unregister(&x2d_driver);
}

late_initcall(x2d_init);// x2d driver should not be inited before PMU driver inited.
module_exit(x2d_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("cheer_fu<cfu@ingenic.cn>");
MODULE_DESCRIPTION("SOC Jz478x EXtreme 2D Module Driver");

