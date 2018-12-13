/*
 * common.h
 */
#ifndef __COMMON_H__
#define __COMMON_H__

#include <tcsm_layout.h>

#ifndef noinline
#define noinline __attribute__((noinline))
#endif

#ifndef __section
# define __section(S) __attribute__((__section__(#S)))
#endif

#define __res_mem		__section(.voice_res)

#define ALIGN_ADDR_WORD(addr)	(void *)((((unsigned int)(addr) >> 2) + 1) << 2)



#define NULL (void *)0


typedef		char s8;
typedef	unsigned char	u8;
typedef		short s16;
typedef unsigned short	u16;
typedef		int s32;
typedef unsigned int	u32;

#define REG8(addr)  *((volatile u8 *)(addr))
#define REG16(addr) *((volatile u16 *)(addr))
#define REG32(addr) *((volatile u32 *)(addr))

#define get_cp0_ebase()		__read_32bit_c0_register($15, 1)

#define __read_32bit_c0_register(source, sel)               \
	({ int __res;                               \
	 if (sel == 0)                           \
	 __asm__ __volatile__(                   \
		 "mfc0\t%0, " #source "\n\t"         \
		 : "=r" (__res));                \
	 else                                \
	 __asm__ __volatile__(                   \
		 ".set\tmips32\n\t"              \
		 "mfc0\t%0, " #source ", " #sel "\n\t"       \
		 ".set\tmips0\n\t"               \
		 : "=r" (__res));                \
	 __res;                              \
	 })


#define CONFIG_SYS_DCACHE_SIZE  (32*1024)
#define CONFIG_SYS_ICACHE_SIZE  (32*1024)
#define CONFIG_SYS_SCACHE_SIZE  (512*1024)
#define CONFIG_SYS_CACHELINE_SIZE   32

#define cache_op(op, addr)      \
	__asm__ __volatile__(       \
			".set   push\n"     \
			".set   noreorder\n"    \
			".set   mips3\n"    \
			"cache  %0, %1\n"   \
			".set   pop\n"      \
			:           \
			: "i" (op), "R" (*(unsigned char *)(addr))  \
			: "memory"                  \
			)

#define _cpu_switch_restore(freq)						\
	do {											\
		int val = 0;								\
		val = freq;									\
		val |= (7 << 20);							\
		REG32(0xb0000000) = val;					\
		while((REG32(0xB00000D4) & 7))				\
			TCSM_PCHAR('w');						\
	} while(0)

#define _cpu_switch_24MHZ()				\
	do {								\
		int val = 0;				\
		val = REG32(0xb0000000);	\
		val &= ~(0xfff << 20);		\
		val |= (0x95 << 24);		\
		REG32(0xb0000000) = val;	\
		val = REG32(0xb0000000);	\
		val &= ~(0xfffff);			\
		REG32(0xb0000000) = val;	\
		val = REG32(0xb0000000);	\
		val |= 7 << 20;				\
		REG32(0xb0000000) = val;	\
		while((REG32(0xB00000D4) & 7))	\
			    TCSM_PCHAR('w');		\
	} while(0)



/* Choose One Work Mode: */
/* #define CONFIG_CPU_IDLE_SLEEP */
#define CONFIG_CPU_SWITCH_FREQUENCY





#endif /* __COMMON_H__ */
