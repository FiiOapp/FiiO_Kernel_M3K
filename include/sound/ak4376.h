/* include/sound/ak4376.h
 *
 * Copyright (C) 2011 ROCKCHIP, Inc.
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
#ifndef __SOUND_ak4376_H
#define __SOUND_ak4376_H
#define  uint8   unsigned char
#define  uint16  unsigned short
#define  int16   short
#define  uint32  unsigned int
#define  int32   int
#define  uint64  unsigned long

extern struct ak4376 *ak4376;
extern int ak4376_i2c_read(struct ak4376 *bq, u8 reg);
extern int ak4376_i2c_write(struct ak4376 *bq, u8 reg, u8 val);
extern unsigned char AK4376_ReadOneByte(uint8 ReadAddr);
extern void AK4376_WriteOneByte(uint8 WriteAddr,uint8 DataToWrite);
extern unsigned char volumeL;
extern unsigned char volumeR;
extern int work_sign;
extern void ak4376_cache_reg_init(void);
extern void ak4376_reg_init(void);
extern bool ak4376_power_mode;
#define AK4376_IOC_MAGIC  '4'
#define AK4376_POWER_ON  _IO(AK4376_IOC_MAGIC,0x24)
#define AK4376_POWER_DOWN  _IO(AK4376_IOC_MAGIC,0x25)

#endif

