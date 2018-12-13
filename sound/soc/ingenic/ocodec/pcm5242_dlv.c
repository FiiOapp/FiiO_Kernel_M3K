/*
 * Linux/sound/oss/jz_dlv.c
 *
 * DLV CODEC driver for Ingenic Jz4750 MIPS processor
 *
 * 2009-12-xx	Steven <dsqiu@ingenic.cn>
 * 2010-01-xx	Jason <xwang@ingenic.cn>
 *
 * Copyright (c) Ingenic Semiconductor Co., Ltd.
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>

#include <linux/sound.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/initval.h>
#include <linux/proc_fs.h>
#include <linux/soundcard.h>
#include <linux/dma-mapping.h>
#include <linux/mutex.h>
#include <linux/mm.h>
//#include <../../arch/mips/xburst/soc-x1000/chip-x1000/halley2/common/board_base.h> 


#include <asm/hardirq.h>
#include "pcm5242_dlv.h"

//#include <asm/jzsoc.h>
bool global_test_eq_on_off=false;
static char driver_name[] = "pcm5242";		


int pcm5242_spi_gpio_init(void)
{
    int ret ;
    printk("======pcm5242_gpio_init==============\n");
/*
    if(DAC_SPI_CS2 > 0){
        ret = gpio_request(DAC_SPI_CS2, "pcm5242 cs2");
        if (ret) {
            printk(KERN_ERR "can's request pcm5242 cs2\n");
            return ret;
        }
    }
    */
     if(DAC_SPI_CS1> 0){
        ret = gpio_request(DAC_SPI_CS1, "pcm5242 cs1");
        if (ret) {
            printk(KERN_ERR "can's request pcm5242 cs1\n");
            return ret;
        }
    }
      if(DAC_SPI_CLK > 0){
        ret = gpio_request(DAC_SPI_CLK, "pcm5242 clk");
        if (ret) {
            printk(KERN_ERR "can's request pcm5242 clk\n");
            return ret;
        }
    }
      if(DAC_SPI_DATA > 0){
        ret = gpio_request(DAC_SPI_DATA, "pcm5242 data");
        if (ret) {
            printk(KERN_ERR "can's request pcm5242 data\n");
            return ret;
        }
    }
       if(DAC_SPI_DO > 0){
        ret = gpio_request(DAC_SPI_DO, "pcm5242 do");
        if (ret) {
            printk(KERN_ERR "can's request pcm5242 do\n");
            return ret;
        }
    }
       if(SF_LC_CE > 0){
        ret = gpio_request(SF_LC_CE, "SF_LC_CE");
        if (ret) {
            printk(KERN_ERR "can's request SF_LC_CE\n");
            return ret;
        }
    }
       if(SF_LC_SEL > 0){
        ret = gpio_request(SF_LC_SEL, "SF_LC_SEL");
        if (ret) {
            printk(KERN_ERR "can's request SF_LC_SEL\n");
            return ret;
        }
    }
       if(I2S_EN> 0){
       ret = gpio_request(I2S_EN, "I2S_EN");
        if (ret) {
            printk(KERN_ERR "can's request I2S_EN\n");
            return ret;
        }
    }
       if(I2S_SET > 0){
        ret = gpio_request(I2S_SET, "I2S_SET");
        if (ret) {
            printk(KERN_ERR "can's request I2S_SET\n");
            return ret;
        }
    }
 /*   if(AMP_MUTE > 0){
        ret = gpio_request(AMP_MUTE, "AMP_MUTE");
        if (ret) {
            printk(KERN_ERR "can's request AMP_MUTE\n");
            return ret;
        }
    }
 */
     if(DAC_MUTE > 0){
        ret = gpio_request(DAC_MUTE, "DAC_MUTE");
        if (ret) {
            printk(KERN_ERR "can's request DAC_MUTE\n");
            return ret;
        }
    }
     if(LO_EN > 0){
        ret = gpio_request(LO_EN, "LO_EN");
        if (ret) {
            printk(KERN_ERR "can's request LO_EN\n");
           return ret;
        }
	//gpio_direction_output(LO_EN, 1);
	gpio_direction_output(LO_EN, 0);
    }
    /*printk("======pcm5242_gpio_init=======10=======\n");
    if(PO_EN > 0){
        ret = gpio_request(PO_EN, "PO_EN");
        if (ret) {
            printk(KERN_ERR "can's request PO_EN\n");
           return ret;
        }
	//gpio_direction_output(PO_EN, 1);
	gpio_direction_output(PO_EN, 0);
    }*/
 /*
    if(DAC_EN > 0){
        ret = gpio_request(DAC_EN, "DAC_EN");
        if (ret) {
            printk(KERN_ERR "can's request DAC_EN\n");
            return ret;
        }
    }
    
    if(AMP_EN > 0){
        ret = gpio_request(AMP_EN, "AMP_EN");
        if (ret) {
            printk(KERN_ERR "can's request AMP_EN\n");
            return ret;
        }
    }
*/ 
     if(LPO_SET > 0){
        ret = gpio_request(LPO_SET, "LPO_SET");
        if (ret) {
            printk(KERN_ERR "can's request LPO_SET\n");
            return ret;
        }
	gpio_direction_output(LPO_SET, 1);
	//gpio_direction_output(LPO_SET, 0);
    }
/*    
    if(LPO_EN > 0){
        ret = gpio_request(LPO_EN, "LPO_EN");
        if (ret) {
            printk(KERN_ERR "can's request LPO_EN\n");
            return ret;
        }
    }
*/ 
     if(LPO_MUTE > 0){
        ret = gpio_request(LPO_MUTE, "LPO_MUTE");
        if (ret) {
            printk(KERN_ERR "can's request LPO_MUTE\n");
            return ret;
        }
	gpio_direction_output(LPO_MUTE, 1);
	//gpio_direction_output(LPO_MUTE, 0);
    }
 /* printk("======pcm5242_gpio_init=======13=======\n");
    if(FM_EN > 0){
        ret = gpio_request(FM_EN, "FM_EN");
        if (ret) {
            printk(KERN_ERR "can's request FM_EN\n");
            return ret;
        }
    }*/
    return 0;
}
/*
static void spi_send_(int data)
{
	int loop;
	int send_data;

	send_data = data;
	gpio_direction_output(DAC_SPI_CS2, 0);//CS=2	
	ndelay(50);
	
	for(loop=0;loop<16;loop++)
	{
		
		gpio_direction_output(DAC_SPI_CLK, 1);	//CLK=1
		
		if((send_data&0x8000)!=0)
		{
			gpio_direction_output(DAC_SPI_DATA, 1);
		}
		else
		{
			gpio_direction_output(DAC_SPI_DATA, 0);
		}
		
		ndelay(100);
		gpio_direction_output(DAC_SPI_CLK, 0);  //CLK=0
		ndelay(100);
		
		send_data = send_data<<1;
	}
	
	gpio_direction_output(DAC_SPI_CS2, 1);
	gpio_direction_output(DAC_SPI_CLK, 0);
	gpio_direction_output(DAC_SPI_DATA, 0);
	ndelay(100);
}

static int spi_read_(int addr)
{
    int loop;
    int read_data;

    gpio_direction_output(DAC_SPI_CS2, 0);//CS1=0
    gpio_direction_input(DAC_SPI_DO);
    ndelay(50);

    for(loop=0;loop<8;loop++)
    {

        gpio_direction_output(DAC_SPI_CLK, 1);	//CLK=1

        if((addr&0x80)!=0)
        {
            gpio_direction_output(DAC_SPI_DATA, 1);
        }
        else
        {
            gpio_direction_output(DAC_SPI_DATA, 0);
        }

        ndelay(100);
        gpio_direction_output(DAC_SPI_CLK, 0);  //CLK=0
        ndelay(100);

        addr = addr<<1;
    }
    for(loop=0;loop<8;loop++)
    {

        gpio_direction_output(DAC_SPI_CLK, 1);	//CLK=1

        if(gpio_get_value(DAC_SPI_DO))
        {
            read_data|0x0001;
        }

        ndelay(100);
        gpio_direction_output(DAC_SPI_CLK, 0);  //CLK=0
        ndelay(100);

        read_data = read_data<<1;
    }

    gpio_direction_output(DAC_SPI_CS2,1);
    gpio_direction_output(DAC_SPI_CLK, 0);
    gpio_direction_output(DAC_SPI_DATA, 0);
    ndelay(100);
    return read_data;
}

static void register_write_(int page,int addr,int value)
{
	int send_data=0;
	send_data = (addr << 1)<<8 | value;

	spi_send_(page);
	spi_send_(send_data);

	return;
}

int register_read_(int page,int addr)
{
    int addr_read;
    addr_read = (addr << 1)+1;

    spi_send_(page);


    return spi_read_(addr_read);
}
EXPORT_SYMBOL(register_read_);

static void transmit_registers_(cfg_reg *r, int n)
{
	int i = 0;
	int page = 0;


	while (i < n)
	{
		if(0x00 == r[i].command)
			page = r[i].param;
		
		i++;
		
		if(0x00 == r[i].command)
			continue;
		
		register_write_(page, r[i].command, r[i].param);	
		//codec_rwreg("%s====>>>> page 0x%x reg[0x%x]=0x%x\n", __func__, page, r[i].command, r[i].param );

	}
	msleep(30);
}
*/
static void spi_send(int data)
{
	int loop;
	int send_data;

	send_data = data;
	gpio_direction_output(DAC_SPI_CS1, 0);//CS1=0	
	ndelay(50);
	
	for(loop=0;loop<16;loop++)
	{
		
		gpio_direction_output(DAC_SPI_CLK, 1);	//CLK=1
		
		if((send_data&0x8000)!=0)
		{
			gpio_direction_output(DAC_SPI_DATA, 1);
		}
		else
		{
			gpio_direction_output(DAC_SPI_DATA, 0);
		}
		
		ndelay(100);
		gpio_direction_output(DAC_SPI_CLK, 0);  //CLK=0
		ndelay(100);
		
		send_data = send_data<<1;
	}
	
	gpio_direction_output(DAC_SPI_CS1,1);
	gpio_direction_output(DAC_SPI_CLK, 0);
	gpio_direction_output(DAC_SPI_DATA, 0);
	ndelay(100);
}

static  int spi_read(int addr)
{
    int loop;
    int  read_data =0;

    gpio_direction_output(DAC_SPI_CS1, 0);//CS1=0
    gpio_direction_input(DAC_SPI_DO);
    ndelay(50);

    for(loop=0;loop<8;loop++)
    {

        gpio_direction_output(DAC_SPI_CLK, 1);	//CLK=1

        if((addr&0x80)!=0)
        {
            gpio_direction_output(DAC_SPI_DATA, 1);
        }
        else
        {
            gpio_direction_output(DAC_SPI_DATA, 0);
        }

        ndelay(100);
        gpio_direction_output(DAC_SPI_CLK, 0);  //CLK=0
        ndelay(100);

        addr = addr<<1;
    }
    for(loop=0;loop<8;loop++)
    {

        gpio_direction_output(DAC_SPI_CLK, 1);	//CLK=1

        if(gpio_get_value(DAC_SPI_DO))
        {
            read_data = read_data+1;
        }

        ndelay(100);
        gpio_direction_output(DAC_SPI_CLK, 0);  //CLK=0
        ndelay(100);
		if(loop<7)
        read_data = read_data<<1;
    }

    gpio_direction_output(DAC_SPI_CS1,1);
    gpio_direction_output(DAC_SPI_CLK, 0);
    gpio_direction_output(DAC_SPI_DATA, 0);
    ndelay(100);
    return read_data;
}

 void register_write(int page,int addr,int value)
{
	int send_data=0;
	send_data = (addr << 1)<<8 | value;
		
	spi_send(page);
	spi_send(send_data);

	return;
}
EXPORT_SYMBOL(register_write);

int register_read(int page,int addr)
{
    int addr_read;
    addr_read = (addr << 1)+1;

    spi_send(page);


    return spi_read(addr_read);
}
EXPORT_SYMBOL(register_read);
void transmit_registers(cfg_reg *r, int n)
{
	int i = 0;
	int page = 0;


	while (i < n) {
		
		if(0x00 == r[i].command)
			page = r[i].param;
		
		i++;
		
		if(0x00 == r[i].command)
			continue;
		
		register_write(page, r[i].command, r[i].param);
		//register_write_(page, r[i].command, r[i].param);


		//printk("%s====>>>> page 0x%x reg[0x%x]=0x%x\n", __func__, page, r[i].command, r[i].param );

	}
	msleep(30);
	printk("%s====>>>> done\n", __func__);
}
EXPORT_SYMBOL(transmit_registers);

void eq_registers_read(cfg_reg *r, int n)
{
	int i = 0;
	int page = 0;


	while (i < n) {
		
		if(0x00 == r[i].command)
			page = r[i].param;
		
		i++;
		
		if(0x00 == r[i].command)
			continue;
		
		printk("page 0x%x,reg:0x%x,value=0x%x\n",page,r[i].command,register_read(page, r[i].command));
		//register_write(page, r[i].command, r[i].param);
		//register_write_(page, r[i].command, r[i].param);


		//codec_rwreg("%s====>>>> page 0x%x reg[0x%x]=0x%x\n", __func__, page, r[i].command, r[i].param );

	}
	msleep(30);
}
EXPORT_SYMBOL(eq_registers_read);

static void transmit_registers_l(cfg_reg *r, int n)
{
	int i = 0;
	int page = 0;


	while (i < n) {
		
		if(0x00 == r[i].command)
			page = r[i].param;
		
		i++;
		
		if(0x00 == r[i].command)
			continue;
		
		register_write(page, r[i].command, r[i].param);	
		//codec_rwreg("%s====>>>> page 0x%x reg[0x%x]=0x%x\n", __func__, page, r[i].command, r[i].param );

	}
	msleep(30);
}
void fm_mode_gpio(void)
{
    //gpio_direction_output(SF_LC_CE, 0);
    //gpio_direction_output(SF_LC_SEL,1);
    //gpio_direction_output(I2S_EN,0);
    gpio_direction_output(I2S_SET,1);
    //gpio_direction_output(DAC_MUTE, 1);
    //gpio_direction_output(LO_EN, 1);
   //gpio_direction_output(PO_EN,1);
    //gpio_direction_output(LPO_SET, 1);
    //gpio_direction_output(LPO_MUTE,1);
    //gpio_direction_output(FM_EN,1);
    printk("DAC master!\n");
     register_write(0,9,17); //mater mode 0x11
       register_write(0,32, 0); //64fs 48khz
        register_write(0,33, 127);
        register_write(0,12, 3);

}
EXPORT_SYMBOL(fm_mode_gpio);
void dac_mode_gpio(void)
{
    //gpio_direction_output(SF_LC_CE, 0);
    //gpio_direction_output(SF_LC_SEL,1);
    //gpio_direction_output(I2S_EN,0);
    gpio_direction_output(I2S_SET,0);
    //gpio_direction_output(DAC_MUTE, 1);
    //gpio_direction_output(LO_EN, 1);
    //gpio_direction_output(PO_EN,1);
    //gpio_direction_output(LPO_SET, 1);
    //gpio_direction_output(LPO_MUTE,1);
    //gpio_direction_output(FM_EN,0);
     printk("DAC slave!\n");
     register_write(0,9,0);
     printk("==> start MUTE");
      register_write(0, 0x03, (1 <<  4)|(1 <<  0));
}
EXPORT_SYMBOL(dac_mode_gpio);
/*
static void transmit_registers_r(cfg_reg *r, int n)
{
	int i = 0;
	int page = 0;


	while (i < n) {
		
		if(0x00 == r[i].command)
			page = r[i].param;
		
		i++;
		
		if(0x00 == r[i].command)
			continue;
		
		register_write_(page, r[i].command, r[i].param);	
		//codec_rwreg("%s====>>>> page 0x%x reg[0x%x]=0x%x\n", __func__, page, r[i].command, r[i].param );

	}
	msleep(30);
}
*/

static ssize_t test_eq_show(struct device *device, struct device_attribute *attr, char *buf)
{
	
	return sprintf(buf,"test_eq %s\n", global_test_eq_on_off?"true":"false");

}

static ssize_t test_eq_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
int page, reg, value;
	char *end;
	char *temp;
	char *temp1;
	unsigned long n;
	ssize_t ret = -EINVAL;
	
	// echo 1 > test_eq
	n = simple_strtoul(buf, &end, 0);

	if (n==1) {
		global_test_eq_on_off = true;
		i2s_info("%s eq true \n", __func__);
	}else{
		global_test_eq_on_off = false;
		i2s_info("%s eq fasle\n", __func__);
	}
	return ret;

}

static ssize_t reg_show(struct device *device, struct device_attribute *attr, char *buf)
{
	return sprintf(buf,"%s\n","registers ok");

}
static ssize_t reg_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
int page, reg, value;
	char *end;
	char *temp;
	char *temp1;
	unsigned long n;
	ssize_t ret = -EINVAL;
	
	// echo 0x0 0x3d 0x3c > reg
	n = simple_strtoul(buf, &end, 0);

	if (*buf ) {
		page = n;
		n=0;
		n = simple_strtoul(end+1, &temp, 0);
		reg =n;

		n=0;
		n=simple_strtoul(temp+1, &temp1, 0);
		value=n;
		ret = count;
				
		register_write(page,  reg, value); 
		
		i2s_info("%s page=0x%x reg=0x%x value=0x%x \n", __func__, page, reg, value);
	}
	
	return ret;

}			
static struct device_attribute pcm5242_device_attrs[] = {
	__ATTR(reg, S_IRUGO | S_IWUSR, reg_show, reg_store),
		
	__ATTR(test_eq, S_IRUGO | S_IWUSR, test_eq_show, test_eq_store),
};

static int pcm5242_device_attr_register(struct miscdevice  *pcm5242_misc_opt)
{
	int error = 0;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(pcm5242_device_attrs)-1; i++) {
		error = device_create_file(pcm5242_misc_opt->this_device, &pcm5242_device_attrs[i]);

		if (error)
			break;
	}

	if (error) {
		while (--i >= 0)
			device_remove_file(pcm5242_misc_opt->this_device, &pcm5242_device_attrs[i]);
	}

	return 0;
}

static int pcm5242_device_attr_unregister(struct miscdevice  *pcm5242_misc_opt)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(pcm5242_device_attrs); i++)
		device_remove_file(pcm5242_misc_opt->this_device, &pcm5242_device_attrs[i]);

	return 0;
}

static int pcm5242_open(struct inode *inode, struct file *file)
{
	return 0;
}
static int pcm5242_release(struct inode *inode, struct file *file)
{
	return 0;
}

static int pcm5242_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	 int err = 0;
    int ret = 0;
    int ioarg = 0;
	
    printk("enter  pcm5242_ioctl\n");
	    /* 检测命令的有效性 */
    if (_IOC_TYPE(cmd) != MEMDEV_IOC_MAGIC2) 
     {printk(" pcm5242  cmd error1\n");
        //return -EINVAL;
        }
     if (_IOC_NR(cmd) > MEMDEV_IOC_MAXNR2) 
           {  printk(" pcm5242  cmd error2\n");
        //return -EINVAL;
	}
	 /* 根据命令，执行相应的操作 */
    switch(cmd) {

      /* 打印当前设备信息 */
      case MEMDEV_IOCPRINT2:
        //  printk("<--- begin shut down do2--->\n\n");
          //  axp173_i2c_write_bit(axp173,0x10,1,2);
        break;
      /* 获取参数 */
      case MEMDEV_IOCGETDATA2: 
      printk(" pcm5242  cmd MEMDEV_IOCGETDATA\n");
        ioarg = 1101;
        ret = __put_user(ioarg, (int *)arg);
        break;
      
      /* 设置参数 */
      case MEMDEV_IOCSETDATA2: 
       printk(" pcm5242  cmd MEMDEV_IOCSETDATA\n");
        ret = __get_user(ioarg, (int *)arg);
        printk("<--- In Kernel MEMDEV_IOCSETDATA ioarg = %d --->\n\n",ioarg);
        break;

      /* shut down pcm5242 */
      case PCM5242_SET:
       {
         //  printk("<--- begin shut down --->\n\n");
             //axp173_i2c_write_bit(axp173,0x10,1,2);
		   printk("begin set pcm5242\n");
		   register_write(0,61, 0xbf); //left volume
		   register_write(0,60, 0x01); //left volume
         // axp173_i2c_write_bit(axp173,0x32,1,7);
          break;
	  }
	  case PCM5242_SET1:
	  {
		  printk("begin  pcm5242_set1\n");
		  gpio_direction_output(SF_LC_CE, 1);
		  break;
	 }
	 case PCM5242_SET2:
	 {
		 printk("begin  pcm5242_set2\n");
		gpio_direction_output(SF_LC_SEL,0);
			break;
		 }
		 case PCM5242_SET3:
	 {
		 printk("begin  pcm5242_set3\n");
		gpio_direction_output(I2S_EN,0);
			break;
		 }
		case PCM5242_SET4:
	 {
		 printk("begin  pcm5242_set4\n");
		gpio_direction_output(I2S_SET,1);
			break;
		 }
		 case PCM5242_SET5:
	 {
		 printk("begin  pcm5242_set5\n");
		gpio_direction_output(DAC_MUTE, 0);
			break;
		 }
		 case PCM5242_SET6:
	 {
		 printk("begin  pcm5242_set6\n");
		 gpio_direction_output(LO_EN, 0);
			break;
		 }
		 case PCM5242_SET7:
	 {
		 printk("begin  pcm5242_set7\n");
		gpio_direction_output(PO_EN,1);
			break;
		 }
		 case PCM5242_SET8:
	 {
		 printk("begin  pcm5242_set8\n");
		gpio_direction_output(LPO_SET, 0);
			break;
		 }
		  case PCM5242_SET9:
	 {
		 printk("begin  pcm5242_set9\n");
		gpio_direction_output(LPO_MUTE,0);
			break;
		 }
      default:  
        return -EINVAL;
    }
    return ret;
}

static struct file_operations pcm5242_opt_fops = {
	.owner	=	THIS_MODULE,
	.open	=	pcm5242_open,
	.unlocked_ioctl		=	pcm5242_ioctl,
	.release	= 	pcm5242_release,
};

static struct miscdevice pcm5242_misc_opt = {
	.minor	=	MISC_DYNAMIC_MINOR,
	.name	= 	(char *)driver_name,
	.fops	=	&pcm5242_opt_fops,
};

/**
 * Module init
 */
static int __init init_dlv(void)
{
	int retval;
	printk("enter pcm5242 dlv init");
	//codec_dbg("\n");
	//register_jz_codecs((void *)jzdlv_ioctl);
	pcm5242_spi_gpio_init();
    //gpio_direction_output(DAC_SPI_CS2, 1);//CS1=0
	//mdelay(200);
	//transmit_registers(initpcm, sizeof(initpcm)/sizeof(initpcm[0]));
   // transmit_registers(close_48K_eq, sizeof(close_48K_eq)/sizeof(close_48K_eq[0]));
           
			//register_write(0,61, 107); //left volume
			//register_write(0,62, 107); //left volume
			
			register_write(0,62, 0); //left volume
			register_write(0,61, 0); //left volume
			//register_write(0,62, 0xBf); //left volume
			//register_write_(0,61, 0); //left volume
			//register_write(0,61, 0); //left volume
	
	printk("pcm5242 L rge60=%d\n",register_read(0,60)); //left volume
    printk("pcm5242 L rge61=%d\n",register_read(0,61)); //left volume
    printk("pcm5242 L reg62=%d\n",register_read(0,62)); //left volume
    
   // printk("pcm5242 R rge60=%x\n",register_read_(0,60)); //left volume
   // printk("pcm5242 R rge61=%x\n",register_read_(0,61)); //left volume
    //printk("pcm5242 R reg62=%x\n",register_read_(0,62)); //left volume
			//register_write_(0,61, 0x00); //right volume
			//register_write_(0,62, 0x00); //right volume
    gpio_direction_output(DAC_SPI_CS1, 0);//CS1=0
    //open i2s data line
/* DAC */
/*
    gpio_direction_output(SF_LC_CE, 0);
    gpio_direction_output(SF_LC_SEL,1);
    gpio_direction_output(I2S_EN,0);
    gpio_direction_output(I2S_SET,0);
   // gpio_direction_output(AMP_MUTE, 1);//AMP open
    gpio_direction_output(DAC_MUTE, 1);
    gpio_direction_output(LO_EN, 1);
    gpio_direction_output(PO_EN,1);
   // gpio_direction_output(DAC_EN, 1);
    //gpio_direction_output(AMP_EN, 1);//AMP open
    gpio_direction_output(LPO_SET, 1);
    //gpio_direction_output(LPO_EN, 1);
    gpio_direction_output(LPO_MUTE,1);
    // gpio_direction_output(FM_EN,0);
*/
/* test */
/*
gpio_direction_output(SF_LC_CE, 0);
    gpio_direction_output(SF_LC_SEL,0);
    gpio_direction_output(I2S_EN,0);
    gpio_direction_output(I2S_SET,0);
   // gpio_direction_output(AMP_MUTE, 1);//AMP open
    gpio_direction_output(DAC_MUTE, 0);
    gpio_direction_output(LO_EN, 0);
    gpio_direction_output(PO_EN,0);
   // gpio_direction_output(DAC_EN, 1);
    //gpio_direction_output(AMP_EN, 1);//AMP open
    gpio_direction_output(LPO_SET, 0);
    //gpio_direction_output(LPO_EN, 1);
    gpio_direction_output(LPO_MUTE,0);
    // gpio_direction_output(FM_EN,0);
*/

/* FM */
  /*  gpio_direction_output(SF_LC_CE, 0);
    gpio_direction_output(SF_LC_SEL,1);
    gpio_direction_output(I2S_EN,0);
    gpio_direction_output(I2S_SET,1);
   // gpio_direction_output(AMP_MUTE, 1);//AMP open
    gpio_direction_output(DAC_MUTE, 1);
    gpio_direction_output(LO_EN, 1);
    gpio_direction_output(PO_EN,1);
   // gpio_direction_output(DAC_EN, 1);
    //gpio_direction_output(AMP_EN, 1);//AMP open
    gpio_direction_output(LPO_SET, 1);
    //gpio_direction_output(LPO_EN, 1);
    gpio_direction_output(LPO_MUTE,1);
    // gpio_direction_output(FM_EN,1);
*/
      register_write(1,2, 17); // -6db 0x11
/*      
      printk("begin to set pcm5242 i2s master mode!\n");
      register_write(0,9,17); //mater mode 0x11
       register_write(0,32, 0); //64fs 48khz
        register_write(0,33, 127); 
        register_write(0,12, 3); 
        printk("pcm5242  rge9=%d\n",register_read(0,9)); //left volume
    printk("pcm5242  rge32=%d\n",register_read(0,32)); //left volume
    printk("pcm5242 reg33=%d\n",register_read(0,33)); //left volume
    printk("pcm5242 reg12=%d\n",register_read(0,12)); //left volume
*/        
	retval = misc_register(&pcm5242_misc_opt);
	if (retval < 0)
	{
		printk(KERN_ERR"register misc device opt failed.\n");
		return retval;
	}
	pcm5242_device_attr_register(&pcm5242_misc_opt);

	return 0;
}

/**
 * Module exit
 */
static void __exit cleanup_dlv(void)
{

	pcm5242_device_attr_unregister(&pcm5242_misc_opt);
    misc_deregister(&pcm5242_misc_opt);
    gpio_free(DAC_SPI_CS1);
    //gpio_free(DAC_SPI_CS2);
    gpio_free(DAC_SPI_CLK);
    gpio_free(DAC_SPI_DATA);
    gpio_free(DAC_SPI_DO);
    gpio_free(SF_LC_CE);
    gpio_free(SF_LC_SEL);
    gpio_free(I2S_EN);
    gpio_free(I2S_SET);
    //gpio_free(AMP_MUTE);
    gpio_free(DAC_MUTE);
    gpio_free(LO_EN);
    //gpio_free(PO_EN);
   // gpio_free(DAC_EN);
   // gpio_free(AMP_EN);
    gpio_free(LPO_SET);
    //gpio_free(LPO_EN);
    gpio_free(LPO_MUTE);

}

module_init(init_dlv);
module_exit(cleanup_dlv);

MODULE_AUTHOR("zhengpz@fiio.cn");
MODULE_DESCRIPTION("pcm5242 driver");
MODULE_LICENSE("GPL");
