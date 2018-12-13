#ifndef __EQ_H__
#define __EQ_H__


#define CODEC_SET_EQ				106
#define CODEC_SET_FILTER			107
#define CODEC_FIXED_GAIN			108

#define MAX_EQ_NUM		16
extern unsigned int po_temp;
enum sampling_rate {
	Rate_192K = 192000, 
	Rate_96K   = 96000, 
	Rate_48K   = 48000,
};


typedef unsigned char cfg_u8;
typedef union {
    struct {
        cfg_u8 offset;
        cfg_u8 value;
    };
    struct {
        cfg_u8 command;
        cfg_u8 param;
    };
} cfg_reg;

typedef struct mixer_eq_table 
{
  char eq_type[32];
  int eq_62;
  int eq_160;
  int eq_400;
  int eq_1K;
  int eq_3K;
  int eq_8K;
  int eq_16K;
}mixer_eq_table;

extern mixer_eq_table mixer_eq_tables[MAX_EQ_NUM];
extern bool global_replay_change;

static struct mixer_eq_list {
  char num;	/* Index to volume table */
  char eq_type[32];
} mixer_eq[MAX_EQ_NUM]= {
  {0, "close"},
  {1, "rock"},
  {2, "classical"},
  {3, "jazz"},
  {4, "pop"},
  {5, "dance"},	
  {6, "vocal"},
  {7, "metal"},
  {8, "custom"}, 
  {9, "62Hz"},
  {10, "160Hz"},
  {11, "400Hz"},
  {12, "1KHz"},
  {13, "3KHz"},  
  {14, "8KHz"},  
  {15, "16KHz"},
};


#define SOUND_MIXER_SET_EQ				_SIOR ('M', 119, mixer_eq_table)
#define SOUND_MIXER_SET_FILTER				_SIOR ('M', 120, int)
#define SOUND_MIXER_SET_FIXED_GAIN				_SIOR ('M', 121, int)

#if 0
#define TESTEQ 		0
#define YAOGUN 		1
#define GUDIAN 		2
#define JUESHI 		3
#define LIUXING 	4
#define RENGSHENG 	5
#define WUQU 		6
#define JINGSHU 	7
#endif

#endif  /* __EQ_H__ */

