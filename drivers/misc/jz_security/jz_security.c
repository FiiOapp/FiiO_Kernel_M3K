/*
 * linux/drivers/misc/jz_security/jz_security.c - Ingenic security driver
 *
 * Copyright (C) 2015 Ingenic Semiconductor Co., Ltd.
 * Author: liu yang <king.lyang@ingenic.com>.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/module.h>
#include <linux/vmalloc.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/miscdevice.h>
#include <linux/sched.h>
#include <linux/ctype.h>
#include <linux/fs.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/clk.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/dma-mapping.h>

#include <soc/base.h>
#include <jz_proc.h>
#include "jz_security.h"
#include "pdma.h"

extern int init_seboot(void);
extern int change_aes_key(struct change_key_param *);
extern void cpu_do_aes(struct aes_param *, unsigned int dmamode,unsigned int cbcmode);
extern int do_rsa(struct rsa_param *);
extern int aes_use_internal_key(void *input, void *output, unsigned int len, unsigned int key, unsigned int crypt);

struct jz_security {
	struct device      *dev;
	struct miscdevice  mdev;
	struct mutex       lock;
	void __iomem       *iomem;
	unsigned int       open_count;
};
struct jz_security *security;
struct security_info *sec_info;

static int security_open(struct inode *inode, struct file *filp)
{
	struct miscdevice *dev = filp->private_data;
	struct jz_security *security = container_of(dev,struct jz_security, mdev);

	mutex_lock(&security->lock);
	security->open_count++;

	sec_info = vzalloc(sizeof(struct security_info));
	if(!PTR_ERR((void *)sec_info)){
		dev_err(security->dev, "vzalloc mem error!!!,[%s,%d]\n",__func__,__LINE__);
		return -ENOMEM;
	}
	if(security->open_count == 1){
		init_seboot();
	}else {
		mutex_unlock(&security->lock);
		return -EBUSY;
	}
	mutex_unlock(&security->lock);

	return 0;
}

static int security_release(struct inode *inode, struct file *filp)
{
	struct miscdevice *dev = filp->private_data;
	struct jz_security *security = container_of(dev,struct jz_security, mdev);

	mutex_lock(&security->lock);
	security->open_count--;
	if(PTR_ERR((void*)sec_info))
		vfree(sec_info);
	mutex_unlock(&security->lock);

	return 0;
}

static long security_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct miscdevice *dev = filp->private_data;

	void __user *argp = (void __user *)arg;
	struct aes_internal_key *aes_internal_p;
	struct change_key_param *key_param;
	struct aes_param  *aes_p;
	struct rsa_param  *rsa_p;
	unsigned int *user_output;
	unsigned int *user_input;
	int ret = 0;
	switch (cmd) {
	case SECURITY_INTERNAL_CHANGE_KEY:
		key_param = (struct change_key_param*)argp;
			if(copy_from_user(sec_info->ker_rsa_enc_data,key_param->rsa_enc_data ,sizeof(sec_info->ker_rsa_enc_data))){
				dev_err(security->dev, "copy_from_user error!!!,[%s,%d]\n",__func__,__LINE__);
				return -EFAULT;
			}
			if(copy_from_user(sec_info->ker_nku_kr, key_param->nku_kr, sizeof(sec_info->ker_nku_kr))){
				dev_err(security->dev, "copy_from_user error!!!,[%s,%d]\n",__func__,__LINE__);
				return -EFAULT;
			}

		key_param->rsa_enc_data = sec_info->ker_rsa_enc_data;
		key_param->nku_kr = sec_info->ker_nku_kr;

		ret = change_aes_key(key_param);
		break;
	case SECURITY_INTERNAL_AES:
		aes_p = (struct aes_param *)argp;
		user_output = aes_p->output;

			if(copy_from_user(sec_info->ker_input,aes_p->input,sizeof(sec_info->ker_input))){
				dev_err(security->dev, "copy_from_user error!!!,[%s,%d]\n",__func__,__LINE__);
				return -EFAULT;
			}
		aes_p->input = sec_info->ker_input;
		aes_p->output = sec_info->ker_output;
		cpu_do_aes(aes_p, 0, 0);
			if(copy_to_user(user_output,sec_info->ker_output,sizeof(sec_info->ker_output))){
				dev_err(security->dev, "copy_to_user error!!!,[%s,%d]\n",__func__,__LINE__);
				return -EFAULT;
			}
		break;
	case SECURITY_RSA:
		rsa_p = (struct rsa_p *)argp;
		user_output = rsa_p->output;
			if(copy_from_user(sec_info->ker_input, rsa_p->input,sizeof(sec_info->ker_input))){
				dev_err(security->dev, "copy_from_user error!!!,[%s,%d]\n",__func__,__LINE__);
				return -EFAULT;
			}
			if(copy_from_user(sec_info->ker_k, rsa_p->key, sizeof(sec_info->ker_k))){
				dev_err(security->dev, "copy_from_user error!!!,[%s,%d]\n",__func__,__LINE__);
				return -EFAULT;
			}
			if(copy_from_user(sec_info->ker_n,rsa_p->n,sizeof(sec_info->ker_n))){
				dev_err(security->dev, "copy_from_user error!!!,[%s,%d]\n",__func__,__LINE__);
				return -EFAULT;
			}
		rsa_p->input = sec_info->ker_input;
		rsa_p->output = sec_info->ker_output;
		rsa_p->key = sec_info->ker_k;
		rsa_p->n = sec_info->ker_n;
		do_rsa(rsa_p);
			if(copy_to_user(user_output, sec_info->ker_output, sizeof(sec_info->ker_output))){
				dev_err(security->dev, "copy_to_user error!!!,[%s,%d]\n",__func__,__LINE__);
				return -EFAULT;
			}
		break;
	case SECURITY_INTERNAL_AES_KEY:
		aes_internal_p = (struct aes_internal_key *)argp;
			user_input = (unsigned int *)(MCU_TCSM_INDATA);
			user_output = (unsigned int *)(MCU_TCSM_OUTDATA);

			if(aes_internal_p->len > AES_MAX_DATA_SIZE){
				dev_err(security->dev,"data len error, should < %d bytes\n",AES_MAX_DATA_SIZE);
				return -EFAULT;
			}
			if(copy_from_user(user_input,aes_internal_p->input,aes_internal_p->len)){
				dev_err(security->dev, "copy_from_user error!!!,[%s,%d]\n",__func__,__LINE__);
				return -EFAULT;
			}
			aes_use_internal_key(user_input,user_output,aes_internal_p->len,AES_BY_UKEY,aes_internal_p->crypt);
			if(copy_to_user(aes_internal_p->output,user_output,aes_internal_p->len)){
				dev_err(security->dev, "copy_to_user error!!!,[%s,%d]\n",__func__,__LINE__);
				return -EFAULT;
			}
		break;
	default:
		ret = -1;
		printk("no support other cmd\n");
	}
	return ret;
}
static struct file_operations security_misc_fops = {
	.open		= security_open,
	.release	= security_release,
	.unlocked_ioctl	= security_ioctl,
};

static int jz_security_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct clk *devclk;

	security= kzalloc(sizeof(struct jz_security), GFP_KERNEL);
	if (!security) {
		printk("security:malloc faile\n");
		return -ENOMEM;
	}

	security->dev = &pdev->dev;

	devclk = clk_get(NULL, "aes");
	if (IS_ERR(devclk)) {
		dev_err(security->dev, "get devclk rate fail!\n");
		return -1;
	}
	clk_enable(devclk);

	security->iomem = ioremap(SECURITY_IOBASE, 0x44);
	if (!security->iomem) {
		dev_err(security->dev, "ioremap failed!\n");
		ret = -EBUSY;
		goto fail_free_io;
	}

	security->mdev.minor = MISC_DYNAMIC_MINOR;
	security->mdev.name =  DRV_NAME;
	security->mdev.fops = &security_misc_fops;

	ret = misc_register(&security->mdev);
	if (ret < 0) {
		dev_err(security->dev, "misc_register failed\n");
		goto fail;
	}
	platform_set_drvdata(pdev, security);

	mutex_init(&security->lock);

	dev_info(security->dev, "ingenic security interface module registered success.\n");

	return 0;
fail:
fail_free_io:
	iounmap(security->iomem);

	return ret;
}


static int jz_security_remove(struct platform_device *dev)
{
	struct jz_security *security = platform_get_drvdata(dev);


	misc_deregister(&security->mdev);
	iounmap(security->iomem);
	kfree(security);

	return 0;
}

static struct platform_driver jz_security_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
	},
	.probe		= jz_security_probe,
	.remove		= jz_security_remove,
};

static int __init jz_security_init(void)
{
	return platform_driver_register(&jz_security_driver);
}

static void __exit jz_security_exit(void)
{
	platform_driver_unregister(&jz_security_driver);
}


module_init(jz_security_init);
module_exit(jz_security_exit);

MODULE_DESCRIPTION("X1000 security driver");
MODULE_AUTHOR("liu yang <king.lyang@ingenic.com>");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("20151028");
