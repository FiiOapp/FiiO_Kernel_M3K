/*
 * JZSOC GPIO port, usually used in arch code.
 *
 * Copyright (C) 2010 Ingenic Semiconductor Co., Ltd.
 */

#ifndef __JZSOC_JZ_GPIO_H__
#define __JZSOC_JZ_GPIO_H__

enum gpio_function {
	GPIO_FUNC_0 	= 0x00,  //0000, GPIO as function 0 / device 0
	GPIO_FUNC_1 	= 0x01,  //0001, GPIO as function 1 / device 1
	GPIO_FUNC_2 	= 0x02,  //0010, GPIO as function 2 / device 2
	GPIO_FUNC_3 	= 0x03,  //0011, GPIO as function 3 / device 3
	GPIO_OUTPUT0 	= 0x04,  //0100, GPIO output low  level
	GPIO_OUTPUT1 	= 0x05,  //0101, GPIO output high level
	GPIO_INPUT 	= 0x06,  //0110, GPIO as input
	GPIO_INT_LO 	= 0x08,  //1000, Low  Level trigger interrupt
	GPIO_INT_HI 	= 0x09,  //1001, High Level trigger interrupt
	GPIO_INT_FE 	= 0x0a,  //1010, Fall Edge trigger interrupt
	GPIO_INT_RE 	= 0x0b,  //1011, Rise Edge trigger interrupt
	GPIO_PULL 	= 0x10,  //10000, GPIO enable pull
	GPIO_INPUT_PULL	= 0x16,  //0001 0110, GPIO as input and enable pull
};
#define GPIO_AS_FUNC(func)  (! ((func) & 0xc))

enum gpio_port {
	GPIO_PORT_A, GPIO_PORT_B,
	GPIO_PORT_C, GPIO_PORT_D,
	/* this must be last */
	GPIO_NR_PORTS,
};
/*
struct jzgpio_state {
        unsigned int pxint;
        unsigned int pxmsk;
        unsigned int pxpat1;
        unsigned int pxpat0;
        unsigned int pxpen;
        unsigned int pxignore;
};


struct jzgpio_chip {
        void __iomem *reg;
        void __iomem *shadow_reg;
        int irq_base;
        spinlock_t gpio_lock;
        DECLARE_BITMAP(dev_map, 32);
        DECLARE_BITMAP(gpio_map, 32);
        DECLARE_BITMAP(irq_map, 32);
        DECLARE_BITMAP(out_map, 32);
        struct gpio_chip gpio_chip;
        struct irq_chip  irq_chip;
        unsigned int wake_map;
        struct jzgpio_state sleep_state;
        unsigned int save[5];
        unsigned int *mcu_gpio_reg;
};
*/

struct jz_gpio_func_def {
	char *name;
	int port;
	int func;
	unsigned long pins;
};

struct gpio_reg_func {
	unsigned int save[5];
};

/* PHY hard reset */
struct jz_gpio_phy_reset {
	unsigned int		gpio;
	int                     active_level;
	enum gpio_port		crtl_port;
	unsigned int		crtl_pins;
	enum gpio_function	set_func;
	struct gpio_reg_func	func;
	unsigned int		delaytime_msec;
};

/*
 * must define this array in board special file.
 * define the gpio pins in this array, use GPIO_DEF_END
 * as end of array. it will inited in function
 * setup_gpio_pins()
 */
//extern struct jzgpio_chip;
extern struct jz_gpio_func_def platform_devio_array[];
extern int platform_devio_array_size;
extern struct jz_gpio_func_def platform_current_array[];
//extern struct jz_gpio_func_def platform_msc0_array[];
extern struct jzgpio_chip *slcd_jz;
extern struct jzgpio_chip *msc0_jz;
extern struct jzgpio_chip *pwm0_jz;
extern struct jzgpio_chip *pwm4_jz;
//extern struct jzgpio_chip *slcd_jz2;
extern struct jz_gpio_func_def *g1;
extern struct jz_gpio_func_def *g2;
extern struct jz_gpio_func_def *g3;
extern struct jz_gpio_func_def *g4;
//extern struct jzgpio_chip jz_gpio_chips[];
/* This functions are used in special driver which need
 * operate the device IO pin.
 */
int jzgpio_set_func(enum gpio_port port,
		    enum gpio_function func,unsigned long pins);

int jz_gpio_set_func(int gpio, enum gpio_function func);

int jzgpio_ctrl_pull(enum gpio_port port, int enable_pull,
		     unsigned long pins);

int jz_gpio_save_reset_func(enum gpio_port port, enum gpio_function dst_func,
			    unsigned long pins, struct gpio_reg_func *rfunc);
int jz_gpio_restore_func(enum gpio_port port,
			 unsigned long pins, struct gpio_reg_func *rfunc);
int jzgpio_phy_reset(struct jz_gpio_phy_reset *gpio_phy_reset);
int mcu_gpio_register(unsigned int reg);
void gpio_set_func(struct jzgpio_chip *chip,
                          enum gpio_function func, unsigned int pins);
#endif
