#ifndef __JZSSI__
#define __JZSSI__


/*SPI NOR FLASH Instructions*/
#define CMD_WREN        0x06 /* Write Enable */
#define CMD_WRDI        0x04 /* Write Disable */
#define CMD_RDSR        0x05 /* Read Status Register */
#define CMD_RDSR_1      0x35 /* Read Status1 Register */
#define CMD_RDSR_2      0x15 /* Read Status2 Register */
#define CMD_WRSR        0x01 /* Write Status Register */
#define CMD_WRSR_1      0x31 /* Write Status1 Register */
#define CMD_WRSR_2      0x11 /* Write Status2 Register */
#define CMD_READ        0x03 /* Read Data */
#define CMD_DUAL_READ   0x3b /* DUAL Read Data */
#define CMD_QUAD_READ   0x6b /* QUAD Read Data */
#define CMD_QUAD_IO_FAST_READ   0xeb /* QUAD FAST Read Data */
#define CMD_QUAD_IO_WORD_FAST_READ   0xe7 /* QUAD IO WORD Read Data */
#define CMD_FAST_READ   0x0B /* Read Data at high speed */
#define CMD_PP          0x02 /* Page Program(write data) */
#define CMD_QPP         0x32 /* QUAD Page Program(write data) */
#define CMD_SE          0x20 /* Sector Erase */
#define CMD_BE_32K      0x52 /* Block Erase */
#define CMD_BE_64K		0XD8 /* Block Erase */
#define CMD_CE          0xC7 /* Bulk or Chip Erase */
#define CMD_DP          0xB9 /* Deep Power-Down */
#define CMD_RES         0xAB /* Release from Power-Down and Read Electronic Signature */
#define CMD_REMS        0x90 /* Read Manufacture ID/ Device ID */
#define CMD_RDID        0x9F /* Read Identification */
#define CMD_NON         0x00 /* Read Identification */
#define CMD_EN4B		0xB7 /* Enter 4 bytes address mode */
#define CMD_EX4B		0xE9 /* Exit 4 bytes address mode */

/* SPI NAND Flash Instructions */
//#define CMD_WREN    0x06    /* Write Enable */
//#define CMD_WRDI    0x04    /* Write Disable */
#define CMD_G_FEATURE   0x0F    /* Get Feature */
#define CMD_S_FEATURE   0x1F    /* Set Feature */
#define CMD_R_PAGE  0x13    /* Page Read (to cache) */
#define CMD_R_CACHE 0x03    /* Read From Cache */
#define CMD_P_LOAD  0x02    /* Program Load(write data to cache) */
#define CMD_P_EXECUTE   0x10    /* Program Execute(write data) */
#define CMD_E_BLOCK 0xD8    /* Block Erase */
//#define CMD_RDID    0x9F    /* Read ID */
#define CMD_RESET   0xFF    /* Reset */

/*for sfc register config*/
#define TRAN_SPI_QUAD   (0x5 )
#define TRAN_SPI_IO_QUAD   (0x6 )


#define SPIFLASH_PARAMER_OFFSET 0x3c00
#define SPI_NORFLASH_PART_OFFSET 0x3c6c

#define NORFLASH_PART_RW		0
#define NORFLASH_PART_WO		1
#define NORFLASH_PART_RO		2

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

struct nor_partition {
	char name[SIZEOF_NAME];
	uint32_t size;
	uint32_t offset;
	uint32_t mask_flags;//bit0-1 mask the partition RW mode, 0:RW  1:WO  2:RO
	uint32_t manager_mode;
};

struct norflash_partitions {
	struct nor_partition nor_partition[NOR_PART_NUM];
	uint32_t num_partition_info;
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

struct jz_sfc_info {
	u8	chnl;				/* the chanel of SSI controller */
	u16	bus_num;			/* spi_master.bus_num */
	unsigned is_pllclk:1;			/* source clock: 1---pllclk;0---exclk */
	unsigned long	board_size;		/* spi_master.num_chipselect */
	u32	 num_chipselect;
	u32	 allow_cs_same;
	void  *board_info;
	u32  board_info_size;
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
