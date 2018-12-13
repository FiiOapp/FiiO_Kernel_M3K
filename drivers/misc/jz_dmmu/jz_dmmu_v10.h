#ifndef _JZ_DMMU_JZ4780_H_
#define _JZ_DMMU_JZ4780_H_


#define DMMU_DEV_NAME "/dev/dmmu"

#define DMMU_IOCTL_MAGIC 'd'


#define DEFAULT_PAGE_TABLE_SIZE 	0x00200000 /* 2MB: 512K page phys addr x4KB = map user space 2GB */
#define DEFAULT_PAGE_TABLE_NUM 		2 /* 2 PAGE_TABLE: SurfaceFlinger, MediaServer */


struct jz_dmmu_platform_data
{
	const char *name;
	int page_table_pool_init_capacity;  /* 2 PAGE_TABLE: SurfaceFlinger, MediaServer */
	unsigned int start;  /* starting physical address of memory region */
	unsigned int size;  /* size of memory region */

	/* set to indicate maps of this region should be cached, if a mix of
	 * cached and uncached is desired, set this and open the device with
	 * O_SYNC to get an uncached region */
	unsigned int cached;
	unsigned int reserved_size;
};

struct dmmu_mem_info {
	int size;
	int page_count;

	unsigned int paddr;

	void *vaddr;
	void *pages_phys_addr_table;

	unsigned int start_offset;
	unsigned int end_offset;
};

static inline int get_page_count(void *vaddr, int size)
{
	int page_count;
	unsigned int start;			/* page start */
	unsigned int end;			/* page end */

	start = ((unsigned int)vaddr) & (~(PAGE_SIZE-1));
	end = ((unsigned int)vaddr + (size-1)) & (~(PAGE_SIZE-1));
	page_count = (end-start) / (PAGE_SIZE) + 1;

	return page_count;
}


static inline int dump_mem_info(struct dmmu_mem_info *mem, char *description)
{
	if (mem == NULL) {
		printk("mem not alloc yet!\n");
		return -1;
	}
#define printf printk
	{
		printf("mem: %p, \t%s\n", mem, description ? description : "");
		printf("\tvaddr= %p,\n", mem->vaddr);
		printf("\tsize= %d (0x%x)\n", mem->size, mem->size);
		printf("\tpaddr= 0x%08x\n", mem->paddr);
		printf("\tpage_count= %d\n", mem->page_count);
		printf("\tpages_phys_addr_table=%p\n", mem->pages_phys_addr_table);
	}
#undef printf
	return 0;
}

#define DMMU_GET_PAGE_TABLE_BASE_PHYS		_IOW(DMMU_IOCTL_MAGIC, 0x01, unsigned int)
#define DMMU_GET_BASE_PHYS                  _IOR(DMMU_IOCTL_MAGIC, 0x02, unsigned int)
#define DMMU_MAP_USER_MEM		            _IOWR(DMMU_IOCTL_MAGIC, 0x11, struct dmmu_mem_info)
#define DMMU_UNMAP_USER_MEM		            _IOW(DMMU_IOCTL_MAGIC, 0x12, struct dmmu_mem_info)
#define DMMU_GET_TLB_PHYS                   _IOWR(DMMU_IOCTL_MAGIC, 0x13, struct dmmu_mem_info)
#define DMMU_FLUSH_CACHE		            _IOW(DMMU_IOCTL_MAGIC, 0x21, unsigned int)
#define DMMU_ALLOC_PAGE_TABLE	            _IOW(DMMU_IOCTL_MAGIC, 0x31, unsigned int)
#define DMMU_FREE_PAGE_TABLE	            _IOW(DMMU_IOCTL_MAGIC, 0x32, unsigned int)
#define DMMU_SET_PAGE_TABLE 	            _IOW(DMMU_IOCTL_MAGIC, 0x33, unsigned int)
#define DMMU_SET_TABLE_FLAG 	            _IOW(DMMU_IOCTL_MAGIC, 0x34, int)

#endif	/*  _JZ_DMMU_JZ4780_H_ */
