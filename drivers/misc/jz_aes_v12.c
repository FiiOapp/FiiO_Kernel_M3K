/*
 * linux/drivers/misc/jz_aes_v12.c - Ingenic aes driver
 *
 * Copyright (C) 2012 Ingenic Semiconductor Co., Ltd.
 * Author: qipengzhen <aric.pzqi@ingenic.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */



#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/dma-mapping.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/delay.h>

#include <linux/clk.h>

#include <soc/base.h>
#include <soc/irq.h>
#include "jz_aes_v12.h"




struct aes_dma {
#define MAX_DMA_BUFFER	(2 * PAGE_SIZE)		/* 8Kbyte for input dma, 8Kbyte for output dma in one operation */

	unsigned char *src;
	dma_addr_t src_addr;

	unsigned char *dst;
	dma_addr_t dst_addr;

	unsigned int max_len;
};

struct jz_aes {
	struct miscdevice mdev;
	struct device *dev;

	/* Soc resources */
	void __iomem *iomem;
	int irq;
	struct clk *clk;

	/* Controller Desc */
	int aesmode;
	int bitmode;

	bool pio_mode;	/* against with dma mode, which only used for test. */

	struct aes_dma dma;
	unsigned int max_dma_len;	/* max dma process len in bytes for one operation */

	bool busy;			/* show status, do not support for twice open */

	struct mutex mutex;

	wait_queue_head_t       wq;


	int dma_done;
	int key_done;
};


#define ASCR 		(0x0000)	/* AES control register 0x00000000 0x0000 32		*/
#define ASSR 		(0x0004)	/* AES status register 0x00000000 0x0004 32		*/
#define ASINTM 		(0x0008)	/* AES interrupt mask register 0x00000000 0x0008 32	*/
#define ASSA 		(0x000c)	/* AES DMA source address 0x00000000 0x000c 32		*/
#define ASDA 		(0x0010)	/* AES DMA destine address 0x00000000 0x0010 32		*/
#define ASTC 		(0x0014)	/* AES DMA transfer count 0x00000000 0x0014 32		*/
#define ASDI 		(0x0018)	/* AES data input 0x00000000 0x0018 32			*/
#define ASDO 		(0x001c)	/* AES data output 0x00000000 0x001c 32			*/
#define ASKY 		(0x0020)	/* AES key input 0x00000000 0x0020 32			*/
#define ASIV 		(0x0024)	/* AES IV input 0x00000000 0x0024 32			*/


#define ASCR_CLR	(1<<10)
#define ASCR_DMAS	(1<<9)
#define ASCR_DMAE	(1<<8)
#define ASCR_KEYL_BIT	(6)
#define ASCR_KEYL_MASK	(0x3<<6)
#define ASCR_MODE	(1<<5)
#define ASCR_DECE	(1<<4)
#define ASCR_AESS	(1<<3)
#define ASCR_KEYS	(1<<2)
#define ASCR_INIT_IV	(1<<1)
#define ASCR_EN		(1<<0)

#define ASSR_DMAD	(1<<2)
#define ASSR_AESD	(1<<1)
#define ASSR_KEYD	(1<<0)


#define ASINTM_DMA_INT	(1<<2)
#define ASINTM_AES_INT	(1<<1)
#define ASINTM_KEY_INT	(1<<0)


#define mcu_soft_reset()	\
	do {			\
	*(volatile unsigned int *)0xb3421030 &= ~1; \
	} while(0)

#define TIMEOUT_MS	(2000)

static int inline aes_readl(struct jz_aes *aes, unsigned int offset)
{
	return readl(aes->iomem + offset);
}

static void inline aes_writel(struct jz_aes *aes, unsigned int offset, unsigned int val)
{
	writel(val, aes->iomem + offset);
}

static void aes_dump(struct jz_aes *aes)
{

	printk("ASCR 	: 0x%08x\n",  aes_readl(aes, ASCR 	));
	printk("ASSR 	: 0x%08x\n",  aes_readl(aes, ASSR 	));
	printk("ASINTM 	: 0x%08x\n",  aes_readl(aes, ASINTM 	));
	printk("ASSA 	: 0x%08x\n",  aes_readl(aes, ASSA 	));
	printk("ASDA 	: 0x%08x\n",  aes_readl(aes, ASDA 	));
	printk("ASTC 	: 0x%08x\n",  aes_readl(aes, ASTC 	));
	printk("ASDI 	: 0x%08x\n",  aes_readl(aes, ASDI 	));
	printk("ASDO 	: 0x%08x\n",  aes_readl(aes, ASDO 	));
	printk("ASKY 	: 0x%08x\n",  aes_readl(aes, ASKY 	));
	printk("ASIV 	: 0x%08x\n",  aes_readl(aes, ASIV 	));
}

static int aes_load_iv(struct jz_aes *aes, char *iv, unsigned int len)
{
	int word_len = len / 4;
	int i;

	unsigned int *ptr = (unsigned int *)iv;
	for(i = 0; i < word_len; i++) {
		aes_writel(aes, ASIV, ptr[i]);
	}

	return 0;
}


static int aes_load_key(struct jz_aes *aes, struct aes_key *aes_key)
{
	int word_len;
	int i, ret;
	unsigned int key[MAX_KEY_LEN_INWORD];
	unsigned int iv[IV_LEN_INWORD];
	unsigned int ascr, asintm;

	mutex_lock(&aes->mutex);
	word_len = aes_key->keylen / 4;

	if(aes_key->keylen > MAX_KEY_LEN_INWORD * 4) {
		dev_err(aes->dev, "Aes Key Too long, max support is %d bits, real %d bits\n",
				MAX_KEY_LEN_INWORD * 4 * 8, aes_key->keylen * 8);
		ret = -EINVAL;
		goto err;
	}

	if(copy_from_user(key, aes_key->key, aes_key->keylen)) {
		dev_err(aes->dev, "Failed to copy key from user!\n");
		ret = -EFAULT;
		goto err;
	}

	aes->aesmode = aes_key->aesmode;
	aes->bitmode = aes_key->bitmode;

	aes->key_done = 0;

	ascr = aes_readl(aes, ASCR);

	/* enable and clear  */
	ascr |= ASCR_EN;
	ascr |= ASCR_CLR;
	aes_writel(aes, ASCR, ascr);


	ascr = aes_readl(aes, ASCR);
	if(aes->aesmode == AES_MODE_CBC) {
		if(copy_from_user(iv, aes_key->iv, aes_key->ivlen)) {
			dev_err(aes->dev, "Failed to copy iv from user!\n");
			ret = EFAULT;
			goto err;
		}

		aes_load_iv(aes, (char *)iv, aes_key->ivlen);

		/* write initial IV */

		ascr |= (ASCR_MODE);
		ascr |= ASCR_INIT_IV;

	} else {
		ascr &= ~(ASCR_MODE);
		ascr &= ~(ASCR_INIT_IV);
	}

	for(i = 0; i < word_len; i++) {
		aes_writel(aes, ASKY, key[i]);
	}


	/* bit mode 128,192,256 */
	ascr &= ~(ASCR_KEYL_MASK);
	ascr |= (aes->bitmode << ASCR_KEYL_BIT);


	ascr |= ASCR_KEYS;
	aes_writel(aes, ASCR, ascr);


	asintm = aes_readl(aes, ASINTM);
	asintm |= ASINTM_KEY_INT;
	aes_writel(aes, ASINTM, asintm);



	ret = wait_event_interruptible_timeout(aes->wq, aes->key_done == 1, msecs_to_jiffies(TIMEOUT_MS));
	if(ret == 0) {
		dev_err(aes->dev, "Key Done Timeout\n");
		ret = -EFAULT;
		goto err;
	}


	mutex_unlock(&aes->mutex);
	return 0;
err:
	mutex_unlock(&aes->mutex);
	return ret;

}



static int dump_data(void *addr, unsigned int len)
{
	unsigned int *data = (unsigned int *)addr;

	int i;
	for(i = 0; i < len/4 ; i++) {
		printk("data[%d]: %08x\n", i, data[i]);
	}
	return 0;
}

static int aes_do_crypt(struct jz_aes *aes, struct aes_data *data)
{

	int encrypt = data->encrypt;
	int ret;
	unsigned int ascr = aes_readl(aes, ASCR);
	unsigned int dma_count;
	int len;
	int aes_len;
	unsigned char *data_input = data->input;
	unsigned char *data_output = data->output;

	mutex_lock(&aes->mutex);
	/* encrypt or decrypt */
	if(encrypt == 0) {
		ascr &= ~(ASCR_DECE);
	} else {
		ascr |= (ASCR_DECE);
	}

	len = data->input_len;
	while(len > 0) {

		if(len > aes->dma.max_len) {
			aes_len = aes->dma.max_len;
		} else {
			aes_len = len;
		}

		/* copy input data to dma src*/
		if(copy_from_user(aes->dma.src, data_input, aes_len)) {
			dev_err(aes->dev, "Failed to load data from user!\n");
			ret = -EFAULT;
			goto err;
		}


		/* dma_cache_sync(src)  DMA_TO_DEVICE */
		dma_cache_sync(aes->dev, aes->dma.src, aes_len, DMA_TO_DEVICE);

		/* enable DMA Int */
		aes_writel(aes, ASINTM, ASINTM_DMA_INT);

		dma_count = data->input_len * 8 / 128;
		aes_writel(aes, ASTC, dma_count);
		aes_writel(aes, ASSA, aes->dma.src_addr);
		aes_writel(aes, ASDA, aes->dma.dst_addr);

		ascr |= ASCR_DMAE;
		ascr &= ~(ASCR_AESS);
		ascr |= ASCR_DMAS;
		/* start encrypt/decrypt */
		aes_writel(aes, ASCR, ascr);

		ret = wait_event_interruptible_timeout(aes->wq, aes->dma_done == 1, msecs_to_jiffies(TIMEOUT_MS));
		if(ret == 0) {
			dev_err(aes->dev, "Encrypt or Decrypt Timeout!\n");
			ret = -EFAULT;
			goto err;
		}

		/* dma_cache_sync() DMA_FROM_DEVICE */
		dma_cache_sync(aes->dev, aes->dma.dst, data->input_len, DMA_FROM_DEVICE);

		/* copy data to output buffer. */
		if(copy_to_user(data_output, aes->dma.dst, aes_len)) {
			dev_err(aes->dev, "Failed to copy data to user!\n");
			ret = -EFAULT;
			goto err;
		}


		len -= aes_len;
		data_input += aes_len;
		data_output += aes_len;
	}


	mutex_unlock(&aes->mutex);
	return 0;

err:
	mutex_unlock(&aes->mutex);
	return ret;

}



static int aes_open(struct inode *inode, struct file *filp)
{
	struct miscdevice *mdev = filp->private_data;
	struct jz_aes *aes = container_of(mdev, struct jz_aes, mdev);

	mutex_lock(&aes->mutex);
	if(aes->busy) {
		dev_err(aes->dev, "Device is busy!\n");
		mutex_unlock(&aes->mutex);
		return -EBUSY;
	}

	aes->busy = 1;

	clk_enable(aes->clk);

	mcu_soft_reset();

	mutex_unlock(&aes->mutex);

	return 0;
}
static int aes_close(struct inode *inode, struct file *filp)
{
	struct miscdevice *mdev = filp->private_data;
	struct jz_aes *aes = container_of(mdev, struct jz_aes, mdev);

	mutex_lock(&aes->mutex);

	aes->busy = 0;

	clk_disable(aes->clk);

	mutex_unlock(&aes->mutex);

	return 0;
}

static long aes_ioctl(struct file *filp, unsigned int cmd, unsigned long args)
{
	struct miscdevice *mdev = filp->private_data;
	struct jz_aes *aes = container_of(mdev, struct jz_aes, mdev);
	struct aes_key aes_key;
	struct aes_data aes_data;
	int ret = 0;


	switch(cmd) {
		case AES_LOAD_KEY:
			if(copy_from_user(&aes_key, (void *)args, sizeof(struct aes_key))) {
				dev_err(aes->dev, "Failed to copy aes_key from user!\n");
				ret = -EFAULT;
				goto err;
			}


			ret = aes_load_key(aes, &aes_key);


			break;
		case AES_DO_CRYPT:
			if(aes->aesmode != AES_MODE_CBC && aes->aesmode != AES_MODE_ECB) {
				dev_err(aes->dev, "Aes Enc/Decrypt Mode not set yet!\n");
				ret = -EINVAL;
				goto err;
			}
			if(copy_from_user(&aes_data, (void *)args, sizeof(struct aes_data))) {
				dev_err(aes->dev, "Could not copy aes_data from user!\n");
				ret = -EFAULT;
				goto err;
			}

			ret = aes_do_crypt(aes, &aes_data);
			if(ret < 0) {
				dev_err(aes->dev, "Failed to encrypt data!\n");
				goto err;
			}

			break;
		default:
			dev_err(aes->dev, "Unspport IO cmd: %x\n", cmd);
	}

	return 0;
err:
	return ret;
}

static struct file_operations aes_misc_fops = {
	.owner = THIS_MODULE,
	.open = aes_open,
	.release = aes_close,
	.unlocked_ioctl = aes_ioctl,
};


static irqreturn_t aes_interrupt(int irq, void *data)
{
	struct jz_aes *aes = (struct jz_aes *)data;

	if(aes_readl(aes, ASSR) & ASSR_DMAD) {
		aes_writel(aes, ASSR, ASSR_DMAD | aes_readl(aes, ASSR));
		aes->dma_done = 1;
	} else if(aes_readl(aes, ASSR) & ASSR_KEYD) {
		aes_writel(aes, ASSR, ASSR_KEYD | aes_readl(aes, ASSR));
		aes->key_done = 1;
	}

	wake_up(&aes->wq);

	return IRQ_HANDLED;
}


static int aes_dma_init(struct jz_aes * aes)
{

	struct aes_dma *dma = &aes->dma;

	dma->max_len = MAX_DMA_BUFFER;
	dma->src = dma_alloc_noncoherent(aes->dev, dma->max_len, &dma->src_addr, DMA_TO_DEVICE);
	if(IS_ERR_OR_NULL(dma->src)) {
		return -ENOMEM;
	}

	dma->dst = dma_alloc_noncoherent(aes->dev, dma->max_len, &dma->dst_addr, DMA_FROM_DEVICE);
	if(IS_ERR_OR_NULL(dma->dst)) {
		dma_free_noncoherent(aes->dev, dma->max_len, dma->src, dma->src_addr);
	}

	return 0;
}

static int aes_dma_deinit(struct jz_aes *aes)
{
	struct aes_dma * dma = &aes->dma;
	dma_free_noncoherent(aes->dev, dma->max_len, dma->src, dma->src_addr);
	dma_free_noncoherent(aes->dev, dma->max_len, dma->dst, dma->dst_addr);
	return 0;
}


static int jz_aes_probe(struct platform_device * pdev)
{
	int ret = 0;
	struct jz_aes * aes;
	struct resource *regs;

	aes = kzalloc(sizeof(struct jz_aes), GFP_KERNEL);
	if(IS_ERR_OR_NULL(aes)) {
		pr_err("Failed to alloc mem for jz_aes!\n");
		ret = -ENOMEM;
		goto err;
	}

	aes->dev = &pdev->dev;

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if(IS_ERR_OR_NULL(regs)) {
		dev_err(aes->dev, "No iomem resource!\n");
		ret = -ENOMEM;
		goto err1;
	}

	aes->irq = platform_get_irq(pdev, 0);
	if(aes->irq < 0) {
		dev_err(&pdev->dev, "No irq resource!\n");
		ret = -ENOMEM;
		goto err1;
	}

	aes->clk = clk_get(&pdev->dev, "aes");
	if(IS_ERR_OR_NULL(aes->clk)) {
		dev_err(&pdev->dev, "Failed to get aes clk!\n");
		ret = -ENOMEM;
		goto err1;
	}

	aes->iomem = devm_ioremap(aes->dev, regs->start, resource_size(regs));
	if(IS_ERR_OR_NULL(aes->iomem)) {
		dev_err(&pdev->dev, "Failed to remap io resources!\n");
		ret = -ENOMEM;
		goto err1;
	}

	ret = devm_request_irq(&pdev->dev, aes->irq, aes_interrupt, IRQF_DISABLED, dev_name(&pdev->dev), aes);
	if(ret < 0) {
		dev_err(&pdev->dev, "Failed to request irq!\n");
		goto err_request_irq;
	}

	aes->aesmode = AES_MODE_NONE;
	aes->pio_mode = false;



	init_waitqueue_head(&aes->wq);

	platform_set_drvdata(pdev, aes);

	if(aes_dma_init(aes) < 0) {
		goto err_dma_init;
	}

	mutex_init(&aes->mutex);


	aes->mdev.minor = MISC_DYNAMIC_MINOR;
	aes->mdev.name = dev_name(&pdev->dev);
	aes->mdev.fops = &aes_misc_fops;
	ret = misc_register(&aes->mdev);
	if(ret < 0) {
		dev_err(&pdev->dev, "Failed to register misc driver!\n");
		goto err_misc_register;
	}

	return 0;

err_misc_register:
	mutex_destroy(&aes->mutex);
	aes_dma_deinit(aes);
err_dma_init:

err_request_irq:

	devm_iounmap(aes->dev, aes->iomem);



err1:
	kfree(aes);
err:
	return ret;

}

static int jz_aes_remove(struct platform_device * pdev)
{
	struct jz_aes *aes = platform_get_drvdata(pdev);

	misc_deregister(&aes->mdev);
	aes_dma_deinit(aes);

	devm_iounmap(aes->dev, aes->iomem);

	kfree(aes);
	aes = NULL;


	return 0;

}

void jz_aes_release(struct device *dev)
{
	return;
}


struct platform_driver jz_aes_driver = {
	.driver = {
		.name = "jz-aes",
		.owner = THIS_MODULE,
	},
	.probe = jz_aes_probe,
	.remove = jz_aes_remove,
};


static int __init jz_aes_init(void)
{
	int ret = 0;


	ret = platform_driver_register(&jz_aes_driver);
	if(ret < 0) {
		pr_info("Failed to register platform driver jz_aes_driver !\n");
		goto out;
	}

	return 0;
out:
	return ret;

}

static void __exit jz_aes_exit(void)
{
	platform_driver_unregister(&jz_aes_driver);

}


module_init(jz_aes_init);
module_exit(jz_aes_exit);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("qipengzhen <aric.pzqi@ingenic.com>");
MODULE_DESCRIPTION("a simple driver for AES controller on ingenic SOC.");
