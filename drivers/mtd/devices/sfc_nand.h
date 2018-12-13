#ifndef __SFC_NAND_H__
#define __SFC_NAND_H__


#define SPIFLASH_PARAMER_OFFSET 0x3c00
#define SPI_NORFLASH_PART_OFFSET 0x3c6c

#define NORFLASH_PART_RW		0
#define NORFLASH_PART_WO		1
#define NORFLASH_PART_RO		2

#undef  NOR_MAJOR_VERSION_NUMBER
#undef  NOR_MINOR_VERSION_NUMBER
#undef  NOR_REVERSION_NUMBER
#undef  NOR_VERSION
#define NOR_MAJOR_VERSION_NUMBER	1
#define NOR_MINOR_VERSION_NUMBER	0
#define NOR_REVERSION_NUMBER	0
#define NOR_VERSION		(NOR_MAJOR_VERSION_NUMBER | (NOR_MINOR_VERSION_NUMBER << 8) | (NOR_REVERSION_NUMBER << 16))

#define NOR_MAGIC	0x726f6e	//ascii "nor"

struct sfc_nor_info {
	u8 cmd;
	u8 addr_len;
	u8 daten;
	u8 pollen;
	u8 M7_0;// some cmd must be send the M7-0
	u8 dummy_byte;
	u8 dma_mode;
};

struct spi_nor_block_info {
	u32 blocksize;
	u8 cmd_blockerase;
	/* MAX Busytime for block erase, unit: ms */
	u32 be_maxbusy;
};

struct spi_quad_mode {
	u8 dummy_byte;
	u8 RDSR_CMD;
	u8 WRSR_CMD;
	unsigned int RDSR_DATE;//the data is write the spi status register for QE bit
	unsigned int RD_DATE_SIZE;//the data is write the spi status register for QE bit
	unsigned int WRSR_DATE;//this bit should be the flash QUAD mode enable
	unsigned int WD_DATE_SIZE;//the data is write the spi status register for QE bit
	u8 cmd_read;
	u8 sfc_mode;
};

#define SIZEOF_NAME         32
#define NOR_PART_NUM	10
struct norflash_params {
	char name[SIZEOF_NAME];
	u32 pagesize;
	u32 sectorsize;
	u32 chipsize;
	u32 erasesize;
	int id;
	/* Flash Address size, unit: Bytes */
	int addrsize;

	/* MAX Busytime for page program, unit: ms */
	u32 pp_maxbusy;
	/* MAX Busytime for sector erase, unit: ms */
	u32 se_maxbusy;
	/* MAX Busytime for chip erase, unit: ms */
	u32 ce_maxbusy;

	/* Flash status register num, Max support 3 register */
	int st_regnum;
	/* Some NOR flash has different blocksize and block erase command,
	 *          * One command with One blocksize. */
	struct spi_nor_block_info block_info;
	struct spi_quad_mode quad_mode;
};



struct nor_sharing_params {
	uint32_t magic;
	uint32_t version;
	struct norflash_params norflash_params;
	struct norflash_partitions norflash_partitions;
};


struct spi_nor_platform_data {
	char *name;
	u32 pagesize;
	u32 sectorsize;
	u32 chipsize;
	u32 erasesize;
	int id;
	/* Some NOR flash has different blocksize and block erase command,
	 *          * One command with One blocksize. */
	struct spi_nor_block_info *block_info;
	int num_block_info;

	/* Flash Address size, unit: Bytes */
	int addrsize;

	/* MAX Busytime for page program, unit: ms */
	u32 pp_maxbusy;
	/* MAX Busytime for sector erase, unit: ms */
	u32 se_maxbusy;
	/* MAX Busytime for chip erase, unit: ms */
	u32 ce_maxbusy;

	/* Flash status register num, Max support 3 register */
	int st_regnum;
	struct mtd_partition *mtd_partition;
	struct spi_quad_mode *quad_mode;
	int num_partition_info;
};

struct jz_spi_info {
	u8	chnl;				/* the chanel of SSI controller */
	u16	bus_num;			/* spi_master.bus_num */
	unsigned is_pllclk:1;			/* source clock: 1---pllclk;0---exclk */
	unsigned long	max_clk;
	unsigned long	board_size;		/* spi_master.num_chipselect */
	struct spi_board_info	*board_info; 	/* link to spi devices info */
	u32	 num_chipselect;
	u32	 allow_cs_same;
	unsigned int chipselect[2];

	void (*set_cs)(struct jz_spi_info *spi, u8 cs,unsigned int pol); /* be defined by spi devices driver user */
	void (*pins_config)(void);		/* configure spi function pins (CLK,DR,RT) by user if need. */
};


struct jz_sfc_nand_info{
	u8      chnl;                           /* the chanel of SSI controller */
	u16     bus_num;                        /* spi_master.bus_num */
	unsigned is_pllclk:1;                   /* source clock: 1---pllclk;0---exclk */
	unsigned long   board_size;             /* spi_master.num_chipselect */
	u32      num_chipselect;
	u32      allow_cs_same;
	void  *board_info;
	u32  board_info_size;
};
#define SPL_TYPE_FLAG_LEN 6
struct jz_spi_support {
	unsigned int id_manufactory;
	unsigned char id_device;
	char name[SIZEOF_NAME];
	int page_size;
	int oobsize;
	int sector_size;
	int block_size;
	int size;
	int page_num;

	/* MAX Busytime for page read, unit: us */
	u32 tRD_maxbusy;
	/* MAX Busytime for Page Program, unit: us */
	u32 tPROG_maxbusy;
	/* MAX Busytime for block erase, unit: us */
	u32 tBERS_maxbusy;

	unsigned short column_cmdaddr_bits;/* read from cache ,the bits of cmd + addr */

};

struct jz_spi_support_from_burner {
        unsigned int chip_id;
        unsigned char id_device;
        char name[32];
        int page_size;
        int oobsize;
        int sector_size;
        int block_size;
        int size;
        int page_num;
        uint32_t tRD_maxbusy;
        uint32_t tPROG_maxbusy;
        uint32_t tBERS_maxbusy;
        unsigned short column_cmdaddr_bits;

};
struct jz_spinand_partition {
	char name[32];         /* identifier string */
	uint32_t size;          /* partition size */
	uint32_t offset;        /* offset within the master MTD space */
	u_int32_t mask_flags;       /* master MTD flags to mask out for this partition */
	u_int32_t manager_mode;         /* manager_mode mtd or ubi */
};
struct get_chip_param {
	int version;
	int flash_type;
	int para_num;
	struct jz_spi_support_from_burner *addr;
	int partition_num;
	struct jz_spinand_partition *partition;
};
struct jz_spi_nand_platform_data {
	struct jz_spi_support *jz_spi_support;
	int num_spi_flash;
	struct mtd_partition *mtd_partition;
	int num_partitions;
};
#endif
