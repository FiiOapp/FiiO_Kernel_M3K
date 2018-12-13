/* drivers/misc/dmmu_jz4780.c
 *
 * DMMU: Device MMU, TLB TABLE for JZ4780: X2D,IPU,CIM,VIDEO-DEC,VIDEO-ENC
 *
 * Copyright (C) 2012 Ingenic
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/mm.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/debugfs.h>
#include <linux/mempolicy.h>
#include <linux/sched.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/cacheflush.h>

#include "jz_dmmu_v10.h"

//#define DMMU_DEBUG

#if 1
#ifdef dev_info
#undef dev_info
#endif
#define dev_info(sss, aaa...)                   \
    do {                                        \
    } while(0)
#endif

#define PAGE_VALID_BIT 0x1
#define TABLE_ID_BITMAP_NONE (0)
#define TABLE_ID_BITMAP_USER_SET (~0)

#define VIDEO_TABLE_FLAGE 1

struct buffer_heap_info {
	struct list_head list;

	void *vaddr;
	unsigned int size;    /* total size of the buffer */
	unsigned int page_count;

	void *pages_phys_table;                /* point to the allocated phys_table base */
//	unsigned int *page_table_addr_start;
//	unsigned int *page_table_addr_end;

	unsigned int start_offset;
	unsigned int end_offset;
};

struct dup_page_info {
	struct list_head list;

	unsigned int phys_addr;
	unsigned int count;
};

struct proc_page_tab_data {
	int pid;					/* process id(tgid) */
	int tid;					/* thread id */
	int table_flag;

	void *global_data;			    /* point to global_data */
	unsigned int base;              /* physical start address of the remaped dmmu space */
	unsigned char __iomem *vbase; 	/* vitual start address of the remaped dmmu space */
	unsigned int size;              /* total size of the dmmu space */

	unsigned int bitmap_table_id;
	struct mutex buffer_list_lock;

	struct list_head buffer_heap_list; 	/* mapped buffer list */
	struct list_head dup_page_list;  /* duplicate pages list */
};

struct dmmu_global_data {
	struct device *dev;
	struct miscdevice misc_dev;

	unsigned int base;  /* physical start address of the remaped dmmu space */
	unsigned char __iomem *vbase;  /* vitual start address of the remaped dmmu space */
	unsigned int size;  /* total size of the dmmu space */
	void * dummy_page_vbase;  /* dummy page,  point to a 4k area used to fix a bug of X2D */
	unsigned int dummy_page_phys_addr;  /* dummy page physical address, point to a 4k area used to fix a bug of X2D */

	unsigned int page_table_pool_init_capacity;  /* init capacity */
	struct mutex page_table_pool_lock;

	struct mutex map_lock;

	unsigned int table_pool_bitmap;  /* table pool bitmap */
};

static struct dmmu_global_data jz_dmmu;

static int dmmu_alloc_page_table(struct proc_page_tab_data *table);
static int dmmu_free_page_table(struct proc_page_tab_data *table);
//static int table_free_buffer_heap_list(struct proc_page_tab_data *table);
//static int check_pid(struct proc_page_tab_data *table);

static int dmmu_release(struct inode *, struct file *);
static int dmmu_open(struct inode *, struct file *);
static long dmmu_ioctl(struct file *, unsigned int, unsigned long);
//static int dmmu_ioctl(struct file *, unsigned int, unsigned long);

struct file_operations dmmu_fops = {
	.owner       =    THIS_MODULE,
	.open        =    dmmu_open,
	.release     =    dmmu_release,
	.unlocked_ioctl =    dmmu_ioctl,
};

/* transform the user's virturl addr to phys */
static unsigned int get_phy_addr(unsigned int vaddr, int map)
{
	unsigned int addr = vaddr & (PAGE_SIZE-1);
	pgd_t *pgdir;
#ifdef CONFIG_PGTABLE_4
	pud_t *pudir;
#endif
	pmd_t *pmdir;
	pte_t *pte;

	pgdir = pgd_offset(current->mm, vaddr);
	if(pgd_none(*pgdir) || pgd_bad(*pgdir))
		return 0;

#ifdef CONFIG_PGTABLE_4
	pudir = pud_offset(pgdir, vaddr);
	if (pud_none(*pudir) || pud_bad(*pudir))
		return 0;
	pmdir = pmd_offset(pudir, vaddr);
	if (pmd_none(*pmdir) || pmd_bad(*pmdir))
		return 0;
#else
	pmdir = pmd_offset((pud_t *)pgdir, vaddr);
	if(pmd_none(*pmdir) || pmd_bad(*pmdir))
		return 0;
#endif
	pte = pte_offset(pmdir,vaddr);
	if (pte_present(*pte)) {
#ifdef CONFIG_COMPACTION // CONFIG_COMPACTION=y CONFIG_MIGRATION=y
		struct page *page = pte_page(*pte);
		if (map)
			SetPageUnevictable(page);
		else
			ClearPageUnevictable(page);
#endif
		return addr | (pte_pfn(*pte) << PAGE_SHIFT);
	}

	return 0;
}

#if 0
static int traverse_dup_pages(struct list_head *head, unsigned int paddr)
{
	int found;
	struct list_head *pos;
	struct dup_page_info *dup_page;
	struct dup_page_info *page_node;

	found = 0;
	list_for_each(pos, head) {
		dup_page = list_entry(pos, struct dup_page_info, list);
		if (dup_page->phys_addr == paddr) {
			found = 1;
			dup_page->count++;
			dev_dbg(jz_dmmu.dev, "%d dup_page->count: %d\n", __LINE__, dup_page->count);
			break;
		}
	}
	if (!found) {
		page_node = (struct dup_page_info *)kzalloc(sizeof(struct dup_page_info), GFP_KERNEL);
		if (page_node == NULL) {
			dev_err(jz_dmmu.dev, "kzalloc page_node failed!\n");
			return -EFAULT;
		}
		page_node->phys_addr = paddr;
		page_node->count++;
		list_add_tail(&(page_node->list), head);
	}

	return 0;
}
#endif

/* transform a buffer */
static int fill_tlb_address(void *page_base, struct dmmu_mem_info *mem,
							struct proc_page_tab_data *proc)
{
	int page_num, i, tlb_pos;
	unsigned int paddr;
	int s_pos;
	void *end, *addr;

//	struct list_head *head = &proc->dup_page_list;

	addr = (void *)(((unsigned int)mem->vaddr >> PAGE_SHIFT) << PAGE_SHIFT);
	page_num = (unsigned int)((mem->vaddr-addr) + mem->size+PAGE_SIZE-1) >> PAGE_SHIFT;
	end = (void *)(((unsigned int)mem->vaddr + (mem->size-1)) & (~(PAGE_SIZE-1)));
	tlb_pos = ((unsigned int)mem->vaddr >> PAGE_SHIFT) << 2;

	mem->start_offset = (unsigned int)mem->vaddr - (unsigned int)addr;
	mem->end_offset = ((unsigned int)mem->vaddr + mem->size) - (unsigned int)end;

	s_pos = tlb_pos;

	down_write(&current->mm->mmap_sem);
	for (i=0; i < page_num; i++) {
		int *p_tlb;
again:
		paddr = get_phy_addr((unsigned int)addr, 1);
		if (!paddr) {
			void *tmp_vaddr = jz_dmmu.dummy_page_vbase;
			memcpy(tmp_vaddr, addr, PAGE_SIZE>>4);
			goto again;
		}
		if (i == 0)
			mem->paddr = paddr;
#if 0
		if (i == 0) {
			mem->paddr = paddr;
			dev_dbg(jz_dmmu.dev, "%s %d i: %d paddr: 0x%08x\n", __func__, __LINE__, i, paddr);
			if (mem->start_offset) {
				ret = traverse_dup_pages(head, paddr);
				if (ret < 0) {
					dev_err(jz_dmmu.dev, "%d traverse_dup_pages failed!\n", __LINE__);
					return -EFAULT;
				}
			}
		}
		if (i == page_num-1) {
			if (mem->end_offset) {
				dev_dbg(jz_dmmu.dev, "%s %d i: %d paddr: 0x%08x\n", __func__, __LINE__, i, paddr);
				ret = traverse_dup_pages(head, paddr);
				if (ret < 0) {
					dev_err(jz_dmmu.dev, "%d traverse_dup_pages failed!\n", __LINE__);
					return -EFAULT;
				}
			}
		}
#endif
		p_tlb = (int *)(proc->vbase + tlb_pos);
		*p_tlb = paddr + PAGE_VALID_BIT;
#ifdef DMMU_DEBUG
		*((int *)page_base) = paddr;
		page_base += 4;
#endif
		dev_dbg(jz_dmmu.dev, "%d tlb_pos: %d, addr: %p, page_base: %p\n", __LINE__, tlb_pos, addr, page_base);
		tlb_pos += 4;
		addr += PAGE_SIZE;
		dev_dbg(jz_dmmu.dev, "%d tlb_pos: %d, addr: %p, page_base: %p\n", __LINE__, tlb_pos, addr, page_base);
	}
	up_write(&current->mm->mmap_sem);

	dma_cache_wback((unsigned int)(proc->vbase + s_pos), page_num * 4);

	return 0;
}

#if 0
static void free_dup_pages(struct list_head *head, int *p_tlb)
{
	int found;
	struct list_head *pos;
	struct dup_page_info *dup_page;

	found = 0;
	dev_dbg(jz_dmmu.dev, "*p_tlb: 0x%08x\n", *p_tlb);
	list_for_each(pos, head) {
		dup_page = list_entry(pos, struct dup_page_info, list);
		if ((dup_page->phys_addr+1) == *p_tlb) {
			found = 1;
			dup_page->count--;
			if (!dup_page->count) {
				dev_dbg(jz_dmmu.dev, "%s %d\n",__func__,__LINE__);
				*p_tlb = jz_dmmu.dummy_page_phys_addr;
				list_del_init(pos);
			}
			break;
		}
	}
	if (!found) {
		dev_dbg(jz_dmmu.dev, "%s %d i == 0 not found\n",__func__,__LINE__);
	}

	return;
}
#endif

static int free_tlb_address(struct dmmu_mem_info *mem, struct proc_page_tab_data *proc)
{
	void *vaddr;
	int page_num, tlb_pos;
	int s_pos, i;

//	struct list_head *head = &proc->dup_page_list;

	vaddr = (void *)(((unsigned int)mem->vaddr >> PAGE_SHIFT) << PAGE_SHIFT);
	page_num = ((mem->vaddr - vaddr) + mem->size + PAGE_SIZE - 1) >> PAGE_SHIFT;
	tlb_pos = ((unsigned int)mem->vaddr >> PAGE_SHIFT) << 2;
	s_pos = tlb_pos;

	down_write(&current->mm->mmap_sem);
	for (i = 0; i < page_num; i++) {
		int *p_tlb;

		get_phy_addr((unsigned int)vaddr, 0);
		vaddr += PAGE_SIZE;

		p_tlb = (int *)(proc->vbase + tlb_pos);

        /* find && release possible dup pages */
#if 0
		if (i == 0 || i == page_num-1)
			free_dup_pages(head, p_tlb);
		else
#endif
		*p_tlb = jz_dmmu.dummy_page_phys_addr;
		tlb_pos += 4;
	}
	up_write(&current->mm->mmap_sem);

	return 0;
}

static int dmmu_release(struct inode *inode, struct file *file)
{
	struct proc_page_tab_data *table = (struct proc_page_tab_data *)file->private_data;
	int ret = 0;

	/* release page table space */
	dmmu_free_page_table(table);

	file->private_data = NULL;

	kfree(table);
	dev_info(jz_dmmu.dev, "%d dmmu_release success!\n", __LINE__);

	return ret;
}

static int dmmu_open(struct inode *inode, struct file *file)
{
	struct proc_page_tab_data *table;
	int ret = 0;

	dev_info(jz_dmmu.dev, "current %u (tgid=%d)file %p(%ld)\n",
			current->pid, current->tgid, file, file_count(file));

	/* setup file->private_data to indicate its unmapped */
	/* you can only open a dmmu device one time */
#if 0
	if (file->private_data != NULL)
		return -1;
#else
	if (file_count(file) > 1) {
		dev_err(jz_dmmu.dev, "you can only open a dmmu device one time!\n");
		return -1;
	}
#endif
	table = kzalloc(sizeof(struct proc_page_tab_data), GFP_KERNEL);
	if (table == NULL) {
		dev_err(jz_dmmu.dev, "dmmu: unable to allocate memory for dmmu metadata.\n");
		return -1;
	}
	mutex_init(&table->buffer_list_lock);
	ret = dmmu_alloc_page_table(table);
	if (ret < 0) {
		dev_err(jz_dmmu.dev, "dmmu_alloc_page_table failed!!!");
		return -EFAULT;
	}

	dev_info(jz_dmmu.dev, "base: 0x%08x, vbase: %p, size: %d\n",
			 table->base, table->vbase, table->size);
	dev_info(jz_dmmu.dev, "pid: %d, tgid: %d!\n", table->pid, table->tid);

	INIT_LIST_HEAD(&table->buffer_heap_list);
	INIT_LIST_HEAD(&table->dup_page_list);

	file->private_data = table;

	return ret;
}

#if 0
static int check_pid(struct proc_page_tab_data *table)
{
	if (table == NULL) {
		dev_err(jz_dmmu.dev, "table is NULL\n");
		return -EFAULT;
	}

	dev_dbg(jz_dmmu.dev, "table->pid=%d, current->pid=%d, current->tgid=%d\n",
		 table->pid, current->pid, current->tgid);

	if (table->pid == current->tgid) {
		return 0;
	}

	dev_err(jz_dmmu.dev, "table->pid=%d, current->pid=%d, current->tgid=%d\n",
		   table->pid, current->pid, current->tgid);

	return -EFAULT;
}
#endif

static int dmmu_alloc_page_table(struct proc_page_tab_data *table)
{
	struct dmmu_global_data *global_data = &jz_dmmu;
	unsigned int bitmap;
	int i;

	if (table == NULL) {
		dev_err(jz_dmmu.dev, "table is NULL!\n");
		return -EFAULT;
	}

	mutex_lock(&table->buffer_list_lock);

	if (table->base) {
		dev_err(jz_dmmu.dev, "already dmmu_alloc_page_table()! base=0x%08x, vbase=%p\n",
			   table->base, table->vbase);
		return -EFAULT;
	}

	mutex_lock(&global_data->page_table_pool_lock);

	bitmap = global_data->table_pool_bitmap;
	if (!bitmap) {
		dev_err(jz_dmmu.dev, "bitmap is zero, no table!\n");
		goto no_table;
	}
	for (i=0; i < global_data->page_table_pool_init_capacity; i++) {
		if (bitmap & (1<<i)) {
			dev_info(jz_dmmu.dev, "bitmap i: %d\n", i);
			break;
		}
	}

	if (i == global_data->page_table_pool_init_capacity) {
		dev_err(jz_dmmu.dev, "i == global_data->page_table_pool_init_capacity!\n");
		goto no_table;
	}

	table->bitmap_table_id = (1<<i);
	global_data->table_pool_bitmap &= ~(table->bitmap_table_id);
	mutex_unlock(&global_data->page_table_pool_lock);

	/* get page table */
	table->base = global_data->base + DEFAULT_PAGE_TABLE_SIZE * i;
	table->vbase = (unsigned char *)((unsigned int)global_data->vbase + DEFAULT_PAGE_TABLE_SIZE*i);

	table->size = DEFAULT_PAGE_TABLE_SIZE;
	table->pid = current->tgid;
	table->tid = current->pid;
	/* init table */
	//init_page_table(table);

	mutex_unlock(&table->buffer_list_lock);

	return 0;

no_table:
	dev_err(jz_dmmu.dev, "no page table left.\n");
	mutex_unlock(&global_data->page_table_pool_lock);
	mutex_unlock(&table->buffer_list_lock);
	return -EFAULT;
}

/*
 * dmmu_free_page_table()
 * Case 1: free pre-allocated tabel space
 * Case 2: free user set tabel space
 *
 */
static int dmmu_free_page_table(struct proc_page_tab_data *table)
{
	struct dmmu_global_data *global_data = &jz_dmmu;

	if (table == NULL) {
		dev_err(jz_dmmu.dev, "table is NULL!\n");
		return -EFAULT;
	}

	if ((table->bitmap_table_id == TABLE_ID_BITMAP_NONE)
		 || table->base == 0 || table->vbase == NULL) {
		dev_err(jz_dmmu.dev,
				"dmmu_free_page_table(), table->bitmap_table_id=%x, base=%x, vbase=%p\n",
				table->bitmap_table_id, table->base, table->vbase);
		return -EFAULT;
	}

	mutex_lock(&table->buffer_list_lock);
	mutex_lock(&global_data->page_table_pool_lock);

	if (table->bitmap_table_id != TABLE_ID_BITMAP_USER_SET) {
		if (table->bitmap_table_id & global_data->table_pool_bitmap) {
			dev_err(jz_dmmu.dev,
					"same thing error, table->bitmap_table_id=%x, global_table->table_pool_bitmap=%x\n",
					table->bitmap_table_id, global_data->table_pool_bitmap);
			goto no_table;
		}

		global_data->table_pool_bitmap |= table->bitmap_table_id;
	}

	mutex_unlock(&global_data->page_table_pool_lock);

	table->bitmap_table_id = TABLE_ID_BITMAP_NONE;	/* clear table id */
	/* get page table */
	table->base = 0;
	table->vbase = NULL;
	table->size = 0;

	/* clear buffer_heap_list? */
	//table_free_buffer_heap_list(table);

	mutex_unlock(&table->buffer_list_lock);

	return 0;

no_table:
	dev_err(jz_dmmu.dev, "no page table left.\n");
	mutex_unlock(&global_data->page_table_pool_lock);
	mutex_unlock(&table->buffer_list_lock);
	return -EFAULT;
}

static int dmmu_get_page_table_base_phys(struct proc_page_tab_data *table, unsigned int *pbase)
{
	if (table == NULL || pbase == NULL) {
		dev_err(jz_dmmu.dev, "table is NULL or pbase is NULL!\n");
		return -EFAULT;
	}

	*pbase = table->base;
	return 0;
}

#ifdef DMMU_DEBUG
static int is_addr_mapped(struct dmmu_mem_info *mem, struct list_head *head)
{
	int mapped = 0;
	struct list_head *pos = NULL;
	struct buffer_heap_info *buffer_map = NULL;

	list_for_each(pos, head) {
		buffer_map = list_entry(pos, struct buffer_heap_info, list);
		if (buffer_map) {
			if ((buffer_map->vaddr == mem->vaddr) && (buffer_map->page_count >= mem->page_count)) {
				mapped = 1;
				break;
			}
		}
	}

	return mapped;
}
#endif

static int dmmu_map_user_mem(struct proc_page_tab_data *table, struct dmmu_mem_info *mem)
{
	int ret = 0;
	void *page_base = NULL;

	if (table == NULL || mem == NULL) {
		dev_err(jz_dmmu.dev, "table is NULL or mem is NULL!\n");
		return -EFAULT;
	}

	mutex_lock(&jz_dmmu.map_lock);

#ifdef DMMU_DEBUG
	int page_count = 0;
	void *tmp_page_base = NULL;
	struct list_head *head = NULL;
	struct buffer_heap_info *buffer = NULL;

	head = &table->buffer_heap_list;
	page_count = mem->page_count;
	page_base = kzalloc(page_count * sizeof(int), GFP_KERNEL);
	dev_dbg(jz_dmmu.dev, "<-----page_base: %p\n", page_base);
	if (page_base == NULL) {
		dev_err(jz_dmmu.dev, "kzalloc page_base failed");
		mutex_unlock(&jz_dmmu.map_lock);
		return -EFAULT;
	}
	tmp_page_base = page_base;
#endif

	/* set mem pages to page table */
	if ((ret = fill_tlb_address(page_base, mem, table)) < 0) {
		dev_err(jz_dmmu.dev, "fill_tlb_address failed!!!");
#ifdef DMMU_DEBUG
		kfree(page_base);
#endif
		mutex_unlock(&jz_dmmu.map_lock);
		return -EFAULT;
	}

#ifdef DMMU_DEBUG
	if (!(ret = is_addr_mapped(mem, head))) {
		/* create buffer_heap_info, added to buffer_heap_list */
		buffer = (struct buffer_heap_info *)kzalloc(sizeof(struct buffer_heap_info), GFP_KERNEL);
		if (buffer == NULL) {
			dev_err(jz_dmmu.dev, "kzalloc buffer_heap_info failed!\n");
			mutex_unlock(&jz_dmmu.map_lock);
			return -EFAULT;
		}

		buffer->vaddr = mem->vaddr;
		buffer->size = mem->size;
		buffer->page_count = mem->page_count;
		buffer->pages_phys_table = tmp_page_base;
		buffer->start_offset = mem->start_offset;
		buffer->end_offset = mem->end_offset;

		list_add_tail(&(buffer->list), head);
	} else {
		kfree(page_base);
	}
#endif

	mutex_unlock(&jz_dmmu.map_lock);

	return 0;
}

static int dmmu_unmap_user_mem(struct proc_page_tab_data *table, struct dmmu_mem_info *mem)
{
#ifdef DMMU_DEBUG
	int found = 0;
	struct list_head *pos = NULL;
	struct list_head *head = NULL;
	struct buffer_heap_info *buffer = NULL;

	head = &table->buffer_heap_list;
	dump_mem_info(mem, NULL);
#endif

	/* free tlb phys address */
	mutex_lock(&jz_dmmu.map_lock);
	free_tlb_address(mem, table);

	/* delete buffer_heap_list */
#ifdef DMMU_DEBUG
	list_for_each(pos, head) {
		buffer = list_entry(pos, struct buffer_heap_info, list);
		if (buffer) {
			if (buffer->vaddr == mem->vaddr) {
				found = 1;
				kfree(buffer->pages_phys_table);
				list_del_init(pos);
				kfree(buffer);
				break;
			}
		}
	}
	if (!found)
		dev_err(jz_dmmu.dev, "%s buffer->vaddr: %p", __func__, buffer->vaddr);
#endif

	mutex_unlock(&jz_dmmu.map_lock);

	return 0;
}

static int get_user_mem_pages_phys_addr_table(struct proc_page_tab_data *table, struct dmmu_mem_info *mem)
{
	void *vaddr;
	unsigned int page_num, tlb_pos;
	unsigned int *p_tlb;
	dev_dbg(jz_dmmu.dev, "*********** get_user_mem_pages_phys_addr_table (**************\n");
	if ( mem->pages_phys_addr_table == NULL )
		return -EINVAL;

	/* aligned vaddr */
	vaddr = (void *)(((unsigned int)mem->vaddr >> PAGE_SHIFT) << PAGE_SHIFT);
	page_num = ((mem->vaddr - vaddr) + mem->size + PAGE_SIZE - 1) >> PAGE_SHIFT;
	tlb_pos = ((unsigned int)mem->vaddr >> PAGE_SHIFT) << 2;

	p_tlb = (unsigned int *)(table->vbase + tlb_pos);

	if (page_num > mem->page_count)
		page_num = mem->page_count;

	if (copy_to_user(mem->pages_phys_addr_table, p_tlb,
					 page_num*sizeof(int))) {
		dev_err(jz_dmmu.dev, "copy_to_user failed!\n");
		return -EFAULT;
	}

	/* dump phys table */
	{
		dev_info(jz_dmmu.dev, "dump phys table, p_tlb=%p, page_num=%d\n", p_tlb, page_num);
		while (page_num--) {
			dev_info(jz_dmmu.dev, "%08X\n", *p_tlb++);
		}
	}

	return 0;
}


static long dmmu_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	int flag = 0;
	unsigned int pbase;
	void __user *argp = (void __user *)arg;

	struct list_head *pos;
	struct list_head *head;
	struct buffer_heap_info *buffer = NULL;

	struct dmmu_mem_info mem;
	struct proc_page_tab_data *table = (struct proc_page_tab_data *)file->private_data;

	if (_IOC_TYPE(cmd) != DMMU_IOCTL_MAGIC) {
		dev_err(jz_dmmu.dev, "invalid cmd!\n");
		return -EFAULT;
	}

	switch (cmd) {
		case DMMU_GET_PAGE_TABLE_BASE_PHYS:
			dev_dbg(jz_dmmu.dev, "DMMU_GET_PAGE_TABLE_BASE_PHYS\n");
		dmmu_get_page_table_base_phys(table, &pbase);
		if (copy_to_user(argp, &pbase, sizeof(unsigned int))) {
			dev_err(jz_dmmu.dev, "copy_to_user failed!\n");
			return -EFAULT;
		}
		break;
	case DMMU_MAP_USER_MEM:
		if (copy_from_user(&mem, argp, sizeof(struct dmmu_mem_info))) {
			dev_err(jz_dmmu.dev, "copy_from_user failed!\n");
			return -EFAULT;
		}
		ret = dmmu_map_user_mem(table, &mem);
		if (ret < 0) {
			dev_err(jz_dmmu.dev, "dmmu_map_user_mem failed!\n");
			return ret;
		}
		if (copy_to_user(((int *)argp+2), &mem.paddr, sizeof(int))) {
			dev_err(jz_dmmu.dev, "copy_to_user failed\n");
			return -EFAULT;
		}
		break;
	case DMMU_UNMAP_USER_MEM:
		if (copy_from_user(&mem, argp, sizeof(struct dmmu_mem_info))) {
			dev_err(jz_dmmu.dev, "copy_from_user failed!\n");
			return -EFAULT;
		}
		ret = dmmu_unmap_user_mem(table, &mem);
		break;
	case DMMU_GET_TLB_PHYS:
		head = &table->buffer_heap_list;
		if (copy_from_user(&mem, argp, sizeof(struct dmmu_mem_info))) {
			dev_err(jz_dmmu.dev, "copy_from_user failed!\n");
			return -EFAULT;
		}
		list_for_each(pos, head) {
			buffer = list_entry(pos, struct buffer_heap_info, list);
			if (buffer->vaddr == mem.vaddr) {
				printk("%s %d\n",__func__,__LINE__);
				break;
			}
		}

		get_user_mem_pages_phys_addr_table(table, &mem);
		return 0;
#ifdef DMMU_DEBUG
		if (copy_to_user(mem.pages_phys_addr_table, buffer->pages_phys_table,
						 buffer->page_count*sizeof(int))) {
			dev_err(jz_dmmu.dev, "copy_to_user failed!\n");
			return -EFAULT;
		}
#endif
		break;
	case DMMU_FLUSH_CACHE:
		/* flush cache all. this is for testing */
		//return -EINVAL;
		break;
	case DMMU_ALLOC_PAGE_TABLE:
		ret = dmmu_alloc_page_table(table);
		break;
	case DMMU_FREE_PAGE_TABLE:
		ret = dmmu_free_page_table(table);
		break;
	case DMMU_SET_PAGE_TABLE:
		/* struct dmmu_page_table_info table; */
		/* if (copy_from_user(&table, (void __user *)arg, */
		/* 				   sizeof(struct dmmu_page_table_info))) */
		/* 	return -EFAULT; */
		//return dmmu_set_page_table(table, &table);
		//return -EINVAL;
		break;
	case DMMU_SET_TABLE_FLAG:
		if (copy_from_user(&flag, argp, sizeof(int))) {
				dev_err(jz_dmmu.dev, "copy_from_user failed!\n");
				return -EFAULT;
		}
		table->table_flag = flag;
		break;
	default:
		dev_err(jz_dmmu.dev, "unknown command!\n");
		return -EINVAL;
	}

	return ret;
}

static inline void * i_memset32(void *vbase, unsigned int value32, int total_size)
{
    unsigned int * s32;
    int iii, isize;
    s32 = (unsigned int *)vbase;
    isize = total_size/(sizeof(unsigned int));
    for (iii=0; iii<isize; iii++) {
        *s32++ = value32;
    }
    return (void*)s32;
}

static int dmmu_probe(struct platform_device *pdev)
{
	int err = 0;
	int total_size = 0, num;
	void *vaddr;
	struct jz_dmmu_platform_data *pdata = pdev->dev.platform_data;

	if (!pdev || !pdata) {
		dev_err(&pdev->dev, "Unable to probe dmmu!\n");
		return -EINVAL;
	}

	pdata = pdev->dev.platform_data;
	jz_dmmu.page_table_pool_init_capacity = pdata->page_table_pool_init_capacity;
	jz_dmmu.base = pdata->start;
	jz_dmmu.size = pdata->size;
	jz_dmmu.dev = &pdev->dev;

	jz_dmmu.misc_dev.name = pdata->name;
	jz_dmmu.misc_dev.minor = MISC_DYNAMIC_MINOR;
	jz_dmmu.misc_dev.fops = &dmmu_fops;

	/* get dummy_base phys addr */
	vaddr = (void *)__get_free_page(GFP_KERNEL);
	if (!vaddr) {
		dev_err(&pdev->dev, "__get_free_page failed!\n");
		goto err_get_page;
	}
	jz_dmmu.dummy_page_vbase = (vaddr);
	jz_dmmu.dummy_page_phys_addr = virt_to_phys(vaddr);
	jz_dmmu.dummy_page_phys_addr |= PAGE_VALID_BIT;
	if (jz_dmmu.base) {
		if (pdata->cached) {
			jz_dmmu.vbase = ioremap_cachable(jz_dmmu.base, jz_dmmu.size);
		} else {
			jz_dmmu.vbase = ioremap(jz_dmmu.base, jz_dmmu.size);
		}
		if (jz_dmmu.vbase == 0) {
			dev_err(&pdev->dev, "get jz_dmmu vbase failed!\n");
			iounmap(jz_dmmu.vbase);
		}
	} else {
		total_size = DEFAULT_PAGE_TABLE_SIZE * jz_dmmu.page_table_pool_init_capacity;
		jz_dmmu.vbase = kmalloc(total_size, GFP_KERNEL); /* cachable ??*/
		if (jz_dmmu.vbase == NULL) {
			dev_err(&pdev->dev, "get jz_dmmu vbase failed!\n");
			goto err_kmalloc;
		}

		/* page aligned */
		jz_dmmu.vbase = (void*)(((unsigned int)jz_dmmu.vbase + PAGE_SIZE-1) & (~(PAGE_SIZE-1)));
		jz_dmmu.base = virt_to_phys(jz_dmmu.vbase);
	}

        //memset(jz_dmmu.vbase, jz_dmmu.dummy_base, total_size);
        i_memset32(jz_dmmu.vbase, jz_dmmu.dummy_page_phys_addr, total_size);

	/* init mutex */
	mutex_init(&jz_dmmu.page_table_pool_lock);
	mutex_init(&jz_dmmu.map_lock);

	/* init bitmap */
	num = jz_dmmu.page_table_pool_init_capacity;
	jz_dmmu.table_pool_bitmap = (1<<num) - 1;

        dev_err(&pdev->dev, "page_table_pool_init_capacity=%d table_pool_bitmap=%#x\n", num, jz_dmmu.table_pool_bitmap);

	err = misc_register(&jz_dmmu.misc_dev);
	if (err < 0) {
		dev_err(&pdev->dev, "Unable to register dmmu driver!\n");
		goto err_register_device;
	}

	return 0;

err_register_device:
	kfree(jz_dmmu.vbase);
err_kmalloc:
	kfree(vaddr);
err_get_page:
	return -1;
}


static int dmmu_remove(struct platform_device *pdev)
{
	void *vaddr;
	vaddr = jz_dmmu.dummy_page_vbase;

	kfree(vaddr);
	kfree(jz_dmmu.vbase);

	misc_deregister(&jz_dmmu.misc_dev);

	return 0;
}

static struct platform_driver dmmu_driver = {
	.probe = dmmu_probe,
	.remove = dmmu_remove,
	.driver = {
		.name = "jz-dmmu"
	}
};


static int __init dmmu_init(void)
{
	return platform_driver_register(&dmmu_driver);
}

static void __exit dmmu_exit(void)
{
	platform_driver_unregister(&dmmu_driver);
}

module_init(dmmu_init);
module_exit(dmmu_exit);
