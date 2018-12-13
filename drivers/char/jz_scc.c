/*
 * kernel/drivers/char/jz_scc.c
 *
 * Copyright (c) 2012 Ingenic Semiconductor Co., Ltd.
 *              http://www.ingenic.com/
 *
 * Core file for Ingenic Smart Card Controller  driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/clk.h>
#include <linux/mutex.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/regulator/consumer.h>
#include <linux/fcntl.h>
#include <linux/memory.h>
#include <linux/mm.h>
#include <soc/gpio.h>

/* #define DEBUG */

#define check_state(str) while((str) && timeout_count--)

/* #define CARD_CHECK_GPIO GPIO_PB(xxx) */
#define RESET_IO GPIO_PA(6)

#ifdef CARD_CHECK_GPIO
  #define CHECK_CARD_BY_IRQ
#endif

#define SCC_BASE 0xb0040000

#define SCCDR   0x00
#define SCCFDR  0x04
#define SCCCR   0x08
#define ENABLE_SCC (1<<31)
#define TRANSMODE (1<<30)
#define EMPTYFIFO (1<<23)
#define SCCSR   0x0c
#define TRANSEND (1<<7)
#define ECNTO (1<<0)
#define SCCTFR  0x10
#define SCCEGTR 0x14
#define SCCECR  0x18
#define SCCRTOR 0x1c

enum {
        RX,
        TX,
};

struct scc_dev {
	int major;
	int minor;
	int nr_devs;

	void __iomem *regbase;
        struct mutex run_lock;
        unsigned char info[100];
        int have_card;

	struct regulator * vcc_scc;
        struct clk *scc_clk;

	struct class *class;
	struct cdev cdev;
	struct device *dev;
};

static inline unsigned long reg_read(struct scc_dev *scc, int offset)
{
	return readl(scc->regbase + offset);
}

static inline void reg_write(struct scc_dev *scc, int offset, unsigned long val)
{
	writel(val, scc->regbase + offset);
}

static inline void reg_or(struct scc_dev *scc, int offset, unsigned long val)
{
        unsigned long tmp = readl(scc->regbase + offset);
	writel(tmp | val, scc->regbase + offset);
}

static inline void reg_and(struct scc_dev *scc, int offset, unsigned long val)
{
        unsigned long tmp = readl(scc->regbase + offset);
	writel(tmp & val, scc->regbase + offset);
}

static int scc_open(struct inode *inode, struct file *filp)
{
	/* struct cdev *cdev = inode->i_cdev; */
	/* struct scc_dev *scc = container_of(cdev, struct scc_dev, cdev); */

	return 0;
}
static int scc_read(struct file *filp, char *buf, size_t count, loff_t *f_pos)
{
	return 0;
}
static int scc_write(struct file *filp, const char *buf, size_t count, loff_t *f_pos)
{
	return count;
}
static int scc_close(struct inode *inode, struct file *filp)
{
	return 0;
}

static long scc_ioctl(struct file *filp, unsigned int cmd, unsigned long args)
{
	return 0;
}

static struct file_operations scc_ops = {
	.owner = THIS_MODULE,
	.write = scc_write,
	.read = scc_read,
	.open = scc_open,
	.release = scc_close,
	.unlocked_ioctl = scc_ioctl,
};

static void dump_scc_register(struct scc_dev *scc)
{
	dev_info(scc->dev, "REG_SCC_DR:0x%lx\n",   reg_read(scc, SCCDR));
	dev_info(scc->dev, "REG_SCC_FDR:0x%lx\n",  reg_read(scc, SCCFDR));
	dev_info(scc->dev, "REG_SCC_CR:0x%lx\n",   reg_read(scc, SCCCR));
	dev_info(scc->dev, "REG_SCC_SR:0x%lx\n",   reg_read(scc, SCCSR));
	dev_info(scc->dev, "REG_SCC_TFR:0x%lx\n",  reg_read(scc, SCCTFR));
	dev_info(scc->dev, "REG_SCC_EGTR:0x%lx\n", reg_read(scc, SCCEGTR));
	dev_info(scc->dev, "REG_SCC_ECR:0x%lx\n",  reg_read(scc, SCCECR));
	dev_info(scc->dev, "REG_SCC_RTOR:0x%lx\n", reg_read(scc, SCCRTOR));
}

static void scc_function_set(void)
{
	gpio_request_one(GPIO_PA(6), GPIOF_IN, "");
	jz_gpio_set_func(GPIO_PB(23), GPIO_FUNC_1);
	jz_gpio_set_func(GPIO_PB(24), GPIO_FUNC_1);
}

static int char_encode(const unsigned char *a, unsigned char *h, int size)
{
	int i = 0;
	int j = 0;

	if(size > 100)
		return -ENOMEM;

	while(i < size)
	{
		if((a[i] >= '0') && (a[i] <= '9'))
		{
			h[j] = a[i] - '0';
		}
		else if((a[i] >= 'a') && (a[i] <= 'f'))
		{
			h[j] = a[i] - 'a' + (unsigned char)10;
		}
		else if((a[i] >= 'A') && (a[i] <= 'F'))
		{
			h[j] = a[i] - 'A' + (unsigned char)10;
		}
		else
		{
			printk("error code[%d]:%x\n",i,a[i]);
			return -1;
		}
		if((i%2) == 0)
		{
			h[j] <<= 4;
			h[j] &= 0xf0;
			j++;
		}
		else if((i%2) == 1)
		{
			h[j-1] += h[j];
		}
		i++;
	}

	if((size%2) == 1)
	{
		h[j-1] >>= 4;
		h[j-1] &= 0x0f;
	}

	return size / 2;
}

static void char_decode(const unsigned char *in, unsigned char *out, int size) // out buffer need twice bigger than in buffer!
{
        int i;
        unsigned char tmp;

	memset(out, 0, size * 2);

        for (i=0; i < size; i++) {
                tmp = (in[i] >> 4) & 0xf;
                if (tmp < 10) {
                        out[2*i] = tmp + '0';
                } else {
                        out[2*i] = tmp - 10 + 'a';
                }

                tmp = in[i] & 0xf;
                if (tmp < 10) {
                        out[2*i+1] = tmp + '0';
                } else {
                        out[2*i+1] = tmp - 10 + 'a';
                }
        }
        out[2*i] = '\n';
}

static int scc_reset(struct scc_dev *scc)
{
	int timeout_count;

	arch_local_irq_disable();
	scc_function_set();

	/*set etu counter as wait time value = 40000 clk*/
        reg_write(scc, SCCECR, 0xe8);

	gpio_direction_output(RESET_IO, 0);
        reg_or(scc, SCCCR, EMPTYFIFO);
        reg_and(scc, SCCSR, ~ECNTO);
        reg_and(scc, SCCCR, ~TRANSMODE);
        reg_or(scc, SCCCR,ENABLE_SCC);
        mdelay(10);
	gpio_direction_output(RESET_IO, 1);

        for (timeout_count = 10000; (!(reg_read(scc, SCCSR) & ECNTO) || reg_read(scc, SCCFDR)) && timeout_count--; udelay(100)) {
                (reg_read(scc, SCCFDR)) ?
#ifdef DEBUG
                        printk("fdr:%04lx\tsr:%02lx\tdr:%02lx\n", reg_read(scc, SCCFDR), reg_read(scc, SCCSR),reg_read(scc, SCCDR)),
#else
                        reg_read(scc, SCCDR),
#endif
                        reg_and(scc, SCCSR, ~ECNTO)
                        : udelay(10);
	}
	if (timeout_count < 0) {
		dev_err(scc->dev, "wrong\n");
                return -EIO;
	}
        return 0;
}

void scc_enable(struct scc_dev *scc)
{
        clk_enable(scc->scc_clk);
        scc_reset(scc);
}
void scc_disable(struct scc_dev *scc)
{
        reg_and(scc, SCCCR,~ENABLE_SCC);
        clk_disable(scc->scc_clk);
}

void set_mode(struct scc_dev *scc, int mode)
{
        if (mode == RX) {
                reg_or(scc, SCCCR, EMPTYFIFO);
                reg_and(scc, SCCCR, ~TRANSMODE);
                reg_write(scc, SCCECR, 0xfff);
                reg_and(scc, SCCSR, ~ECNTO);
        } else if (mode == TX) {
                reg_or(scc, SCCCR, EMPTYFIFO);
                reg_write(scc, SCCECR, 0x0);
                reg_and(scc, SCCSR, ~ECNTO);
                reg_or(scc, SCCCR, TRANSMODE);
        }
}

static int read_from_card(struct scc_dev *scc, const char *str_in, size_t size)
{
	int timeout_count, i, hcount;
	unsigned char hex_buffer[100];

	memset(hex_buffer, 0, sizeof(hex_buffer));

	hcount = char_encode((const unsigned char*)str_in, hex_buffer, size - 1);
        if (hcount < 0) {
                dev_err(scc->dev, "size out of memory!\n");
                return -1;
        }

        scc_enable(scc);

#ifdef DEBUG
	printk("count = %d, hex_buffer = ", hcount);
        for (i=0; i<hcount;)
		printk("%02x",hex_buffer[i++]);
	printk("\n");
        dump_scc_register(scc);
#endif


	/*set transmition mode*/
        set_mode(scc, TX);

        for (i=0; i<hcount;)
                if (reg_read(scc, SCCFDR) < 10) reg_write(scc, SCCDR, hex_buffer[i++]);

        for (timeout_count = 10000; (!(reg_read(scc, SCCSR) & TRANSEND)) && timeout_count--; udelay(10));
	if (timeout_count < 0) {
		dev_err(scc->dev, "transmition error: wait transend timeout!\n");
                return -EIO;
	}

	/*convert receive mode*/
        set_mode(scc, RX);

	memset(hex_buffer, 0, sizeof(hex_buffer));
        for (i=0; !(reg_read(scc, SCCSR) & ECNTO);) {   // || REG_SCCFDR)
		if(reg_read(scc, SCCFDR)) {
			hex_buffer[i++] = reg_read(scc, SCCDR);
                        reg_and(scc, SCCSR, ~ECNTO);
		}
	}
        reg_or(scc, SCCCR, EMPTYFIFO);
#ifdef DEBUG
        printk("i == %d\n", i);
        printk("0 = %02x\n", hex_buffer[0]);
        printk("1 = %02x\n", hex_buffer[1]);
#endif
        if (i)
                char_decode(hex_buffer, scc->info, i);
        else
                strcpy(scc->info, "error\n");
        scc_disable(scc);

	return 0;
}

static int write_to_card(struct scc_dev *scc, char *str_in, int str_line, char *str_out, int str_size, char *buf)
{
	return 0;
}

static ssize_t write_card(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
        int blank_counter = 0;
        int blank_pos = 0;
        int i = 0;
        struct scc_dev *scc = dev_get_drvdata(dev);

#if 0
        /* for write */
        for (i=0; i<count; i++) {
                if (*(buf+i) == ' ') {
                        blank_counter++;
                        blank_pos = i;
                }
        }
        printk("blank_pos = %d, blank_counter = %d\n", blank_pos, blank_counter);
#endif

        mutex_lock(&scc->run_lock);
        if (blank_counter == 1) {
#if 0
                /* for write */
                char addr[100];
                char value[100];
                memset(addr, 0, 100);
                memset(value, 0, 100);

                memcpy(addr, buf, blank_pos);
                printk("a = %s, count = %d\n", addr, count);
                memcpy(value, buf + blank_pos + 1, count - blank_pos - 2);
                printk("b = %s, count = %d\n", value, count);

                mutex_unlock(&scc->run_lock);
                return count;
                write_to_card(scc, addr, blank_pos, value, count - blank_pos, scc->info);
#endif
        } else if (blank_counter == 0) {
                if (read_from_card(scc, buf, count)) {
                        dev_err(scc->dev, "read error!\n");
                }
        }
        mutex_unlock(&scc->run_lock);
        return count;
}

static ssize_t read_card(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
        int len;
        struct scc_dev *scc = dev_get_drvdata(dev);

        mutex_lock(&scc->run_lock);

        len = sprintf(buf, scc->info);

        mutex_unlock(&scc->run_lock);

        return len + 1;
}

static DEVICE_ATTR(card, 0666, read_card, write_card);

static struct attribute *scc_attributes[] = {
	&dev_attr_card.attr,
	NULL
};

static const struct attribute_group scc_attr_group = {
	.attrs = scc_attributes,
};

static int __init scc_init(void)
{
	dev_t dev = 0;
	int ret, dev_no;
	struct scc_dev *scc;

	scc = kzalloc(sizeof(struct scc_dev), GFP_KERNEL);
	if(!scc) {
		printk("scc dev alloc failed\n");
		goto __err_scc;
	}

        scc->regbase = (void __iomem *)SCC_BASE;
	scc->class = class_create(THIS_MODULE, "jz-scc");
	scc->minor = 0;
	scc->nr_devs = 1;
	ret = alloc_chrdev_region(&dev, scc->minor, scc->nr_devs, "jz-scc");
	if(ret) {
		printk("alloc chrdev failed\n");
		goto __err_scc;
	}
	scc->major = MAJOR(dev);

	dev_no = MKDEV(scc->major, scc->minor);
	cdev_init(&scc->cdev, &scc_ops);
	scc->cdev.owner = THIS_MODULE;
	cdev_add(&scc->cdev, dev_no, 1);

	scc->dev = device_create(scc->class, NULL, dev_no, NULL, "jz-scc");

	scc->vcc_scc = regulator_get(scc->dev,"vcc_scc");
        if (!scc->vcc_scc) {
		printk("get scc card power error!\n");
		/* goto __err_regulator; */
        } else
                regulator_enable(scc->vcc_scc);

	scc->scc_clk = clk_get(NULL, "scc");
        if (!scc->scc_clk) {
		printk("get scc clock error!\n");
		/* goto __err_clk; */
        }

	ret = sysfs_create_group(&scc->dev->kobj, &scc_attr_group);
	if (ret < 0) {
		    printk("cannot create sysfs interface\n");
	}

        mutex_init(&scc->run_lock);

	dev_set_drvdata(scc->dev, scc);

        printk("the SCC init success!\n");
	return 0;

__err_clk:
	regulator_disable(scc->vcc_scc);
__err_regulator:
__err_scc:
	kfree(scc);

	return -EFAULT;
}
static void __exit scc_exit(void)
{

}

module_init(scc_init);
module_exit(scc_exit);

MODULE_AUTHOR("gaowei<wei.gao@ingenic.com>");
MODULE_DESCRIPTION("smart card controller driver");
MODULE_LICENSE("GPL");
