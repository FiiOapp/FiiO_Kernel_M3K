#ifndef __TCSM_LAYOUT_H__
#define __TCSM_LAYOUT_H__

/**
 *      |-------------|
 *      |   BANK 0    |
 *	|-------------| <--- VOICE_TCSM_DATA_BUFFER
 *	|    ...      |
 *	|   BUFFER    |
 *	|    ...      |
 *	|-------------|
 *
 *      |-------------|
 *      |    BANK 1   |
 *      |-------------| <--- SLEEP_TCSM_BOOTCODE_TEXT
 *      |  BOOT CODE  |
 *      |-------------| <--- SLEEP_TCSM_RESUMECODE_TEXT
 *      |    ...      |
 *	| RESUME CODE |
 *      |    ...      |
 *      |-------------| <--- SLEEP_TCSM_RESUME_DATA
 *      | RESUME DATA |
 *      |_____________|
 *
 *      |-------------|
 *      |   BANK 2    |
 *      |-------------| <--- VOICE_DMA_DRSC_ADDR
 *      |   DMA DESC  |
 *      |-------------|
 *
 *      |-------------|
 *      |   BANK 3    |
 *      |-------------|
 *      |-------------| <--- CPU_RESMUE_SP
 *      |  RESUME SP  |
 *      |-------------|
 *
 */


#define SLEEP_TCSM_SPACE           0xb3422000
#define TCSM_BANK_LEN             4096


#define VOICE_TCSM_DATA_BUF		(0xb3423000)
#define VOICE_TCSM_DATA_BUF_SIZE	TCSM_BANK_LEN
#define VOICE_TCSM_DESC_ADDR		(0xb3424000) /* bank2 start */
#define CPU_RESUME_SP               0xb3425FF0
#define CPU_SAVE_SP               0xb3425FFC

#endif
