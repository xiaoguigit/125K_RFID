#include <linux/module.h>
#include <linux/kernel.h>
#include <asm/io.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/time.h>


static int rfid_get_card_id(uint64_t *pl_card_id);

static int major;
static dev_t devid;
static struct cdev io_cdev;
static struct class *cls;


#define IO_IDCARD_IN	0x16
#define A33_PL11    363
#define A33_PL11_IRQ 	(__gpio_to_irq(A33_PL11))
#define BELOW1BIT	128		//this is 128us (128*1us)
#define ABOVE1BIT	384		//this is 384us (384*1us)
#define ABOVE2BITS	640		//this is 640us (640*1us)
#define ERRORCOUNTS 20


struct sunxi_pwm_regs {
	unsigned long	pwm_ctrl_reg;	
	unsigned long	pwm_ch0_period;	
	unsigned long	pwm_ch1_period;	
};


static volatile struct sunxi_pwm_regs* pt_sunxi_pwm_regs;
static volatile unsigned long *p_pio_phcfg0;
static uint64_t card_id;
static struct timeval pre_time;
static unsigned char  RFIDBuf128[16];
static unsigned char RFIDDecodeOK;
static unsigned char RFIDCustomerID;
static unsigned long RFIDLastCardID;
static unsigned char const RFIDMask[8] = {0x80,0x40,0x20,0x10,0x08,0x04,0x02,0x01};
static unsigned char tmp_bit1, tmp_bit2, DataOK,ExtCardReader;
static unsigned long  Data;


//static irqreturn_t rfid_data_intterupt_handler(int irq, void *dev_id)
//{

//	return IRQ_RETVAL(IRQ_HANDLED);
//}


static unsigned char rfid_read_bit(void)
{
	return gpio_get_value(A33_PL11)? 1: 0;
}

static void pwm_ch1_enable(void)
{
	pt_sunxi_pwm_regs->pwm_ctrl_reg |= (1 << 19);
	printk("%s ok!\n", __FUNCTION__);
}

static void pwm_ch1_disable(void)
{
	pt_sunxi_pwm_regs->pwm_ctrl_reg &= (0 << 19);
	printk("%s ok!\n", __FUNCTION__);
}

static void out_125KHz(void)
{
	pwm_ch1_disable();
	if((1 << 29) & pt_sunxi_pwm_regs->pwm_ctrl_reg){
		printk("PWM1 is busy now\n");
	}
	pt_sunxi_pwm_regs->pwm_ctrl_reg = 0x003f8000;// 24MHz / 1  == 24MHz  				
	pt_sunxi_pwm_regs->pwm_ch1_period &= 0;
	pt_sunxi_pwm_regs->pwm_ch1_period |= (191 << 16);// SET DUTY 125k    (理论值为192，实际测出191较为合适)
	pt_sunxi_pwm_regs->pwm_ch1_period |= (95<< 0);  // 50 %
	pwm_ch1_enable();
}

static int io_open(struct inode *inode, struct file *file)
{
	printk("io_open ok.\n");
	return 0;
}


static unsigned char _crotl__(u8 value,u8 count)
{
	unsigned int buff,buff1;
	unsigned char a,b;
	buff1 = value;
	buff = buff1 << count;

	a = buff & 0xff;
	b = buff >> 8;
	// printk("%x,%x,%x,%x\n",buff1,buff,a,b);
	return a | b;
}
static void RFIDBuf64Shift(void)
{
	unsigned char d,i;
	
	d = (RFIDBuf128[0]>>7) & 0x01;
	for(i=0;i<8;i++)
		RFIDBuf128[i] = _crotl__(RFIDBuf128[i],1);
	for(i=0;i<7;i++)
		RFIDBuf128[i] = (RFIDBuf128[i] & 0xFE) | (RFIDBuf128[i+1] & 0x01);
	RFIDBuf128[7] = (RFIDBuf128[7] & 0xFE) | d;
}


static unsigned char CheckParity(void)//uchar CheckParity(void)
{
	unsigned char  i,d;
	unsigned long ld;
	
	ld = Data;
	d = 0;
	for(i=0;i<13;i++)
	{
		if ( ld % 2 ) d++;
		ld >>= 1;
	}
	if ( !(d % 2) ) return 1;
	
	d = 0;
	for(i=0;i<13;i++)
	{
		if ( ld % 2 ) d++;
		ld >>= 1;
	}
	if ( d % 2 )
		return 1;
	else
		return 0;
}

static unsigned char RFIDParityCheck(void)
{
	unsigned char i,t1,t2;
	unsigned char bits;
	unsigned char ok;
	
	for(i=0;i<64;i++)
	{
		ok = 1;
		if ( (RFIDBuf128[0]!=0xFF) || ((RFIDBuf128[1]&0x80)!=0x80) || (RFIDBuf128[7] & 0x01) )
		{
			ok = 0;
			RFIDBuf64Shift();
			continue;
		}
		for(t1=0;t1<10;t1++)	//10 rows parity check
		{
			bits = t1*5+9;
			tmp_bit1 = 0;
			for(t2=0;t2<5;t2++)
			{
				if ( RFIDBuf128[(bits+t2)>>3] & RFIDMask[(bits+t2)&0x07] ) tmp_bit1 = ~tmp_bit1;
			}
			if ( tmp_bit1 )
			{
				ok = 0;
				break;
			}
		}
		if ( !ok )
		{
			RFIDBuf64Shift();
			continue;
		}
		
		for(t1=0;t1<4;t1++)	//4 columns parity check
		{
			bits = t1+9;
			tmp_bit1 = 0;
			for(t2=bits;t2<bits+55;t2+=5)
			{
				if ( RFIDBuf128[t2>>3] & RFIDMask[t2&0x07] ) tmp_bit1 = ~tmp_bit1;
			}
			if ( tmp_bit1 )
			{
				ok = 0;
				break;
			}
		}
		if ( !ok )
		{
			RFIDBuf64Shift();
			continue;
		}
		ok = 1;
		break;
	}
	return ok;
}
	


static unsigned int get_time_use(void)
{
	struct timeval now_time, temp;
	do_gettimeofday(&now_time);
	if ((now_time.tv_usec-pre_time.tv_usec)<0) {
		temp.tv_sec = now_time.tv_sec-pre_time.tv_sec-1;
		temp.tv_usec = 1000000+now_time.tv_usec-pre_time.tv_usec;
	} else {
		temp.tv_sec = now_time.tv_sec-pre_time.tv_sec;
	 	temp.tv_usec = now_time.tv_usec-pre_time.tv_usec;
	}

	//printk("pre: %ld s  : %ld us\n", pre_time.tv_sec * 1000000 , pre_time.tv_usec);
	//printk("now: %ld s  : %ld us\n", now_time.tv_sec * 1000000, now_time.tv_usec);

	//printk("use: %ld us\n", temp.tv_sec * 1000000 + temp.tv_usec);

	return (temp.tv_sec * 1000000 + temp.tv_usec);
}


static void rfid_decode(void)
{
	unsigned int use_time, bits;

	tmp_bit1 = rfid_read_bit();
	printk("START BIT1 : %d\n",tmp_bit1);
	do_gettimeofday( &pre_time);

#if 0
	unsigned int test, i;
	i = 0;
	test =0;
	while(1){
		tmp_bit2 = rfid_read_bit();
		if(test != tmp_bit2){
			test = tmp_bit2;
			get_time_use();
			do_gettimeofday( &pre_time);
			if(i++ == 200)
				return;
		}
	}
#endif

#if 1
	while (1)
	{
		tmp_bit2 = rfid_read_bit();
		
		if(tmp_bit1 != tmp_bit2){
			break;
		}
		use_time = get_time_use();
		if(use_time > ABOVE2BITS)
		{
			printk("640us timeout[%d] : %d\n", __LINE__, use_time);
			RFIDDecodeOK = 0;
			return;
		}
	}

	do_gettimeofday(&pre_time);
	use_time = 0;

	for(bits = 0; bits < 128; bits++)
	{
		if ( tmp_bit2 )
		{
			while(1)
			{
				if(use_time > ABOVE2BITS)
				{
					printk("640us timeout[%d] : %d\n", __LINE__, use_time);
					RFIDDecodeOK = 0;
					return;
				}

				use_time = get_time_use();
				if ( rfid_read_bit() == 1){
					continue;
				}else
				{
					tmp_bit2 = 0;
					do_gettimeofday(&pre_time);
					if ( use_time < BELOW1BIT )
					{
						printk("error :BELOW1BIT[%d] : %d\n", __LINE__, use_time);
						RFIDDecodeOK = 0;
						return;
					}
					if ( use_time >= ABOVE1BIT )
					{
						RFIDBuf128[bits>>3] |= RFIDMask[bits & 0x07];	//save double bits of 1
						if(bits<127)
						{
							bits++;
						}
					}
					RFIDBuf128[bits>>3] |= RFIDMask[bits&0x07];	//save single bit of 1
					break;
				}
			}
		}
		else
		{	
			while(1)
			{
				if ( use_time >=ABOVE2BITS )
				{
					printk("640us timeout[%d] : %d\n", __LINE__, use_time);
					RFIDDecodeOK = 0;
					return;
				}

				use_time = get_time_use();
				if ( rfid_read_bit() == 0 )
				{
					continue;
				}
				else	
				{
					tmp_bit2 = 1;
					do_gettimeofday(&pre_time);
					if ( use_time < BELOW1BIT )
					{
						printk("error :BELOW1BIT[%d] : %d\n", __LINE__, use_time);
						RFIDDecodeOK = 0;
						return;
					}
					if ( use_time>=ABOVE1BIT )
					{
						RFIDBuf128[bits>>3] &= ~RFIDMask[bits&0x07];	//save double bits of 0
						bits++;
					}
					RFIDBuf128[bits>>3] &= ~RFIDMask[bits&0x07];		//save single bits of 0
					break;
				}
			}
		}
	}

	for(bits=0;bits<64;bits++)
	{
		if ( RFIDBuf128[bits>>2] & RFIDMask[(bits<<1)&0x07] )
			RFIDBuf128[bits>>3] |= RFIDMask[bits&0x07];
		else
			RFIDBuf128[bits>>3] &= ~RFIDMask[bits&0x07];
	}
	RFIDDecodeOK = 1;

#endif
}

static int rfid_get_card_id(uint64_t *pl_card_id)
{
	unsigned char  i,t1,t2;
	printk("rfid_get_card_id() \n");
	for(i=0;i<ERRORCOUNTS;i++)
	{
		//printk("error count : %d\n", i);
		rfid_decode();
		if ( !RFIDDecodeOK ) 
			continue;
		if ( !RFIDParityCheck() )
		{
			for(t1=0;t1<8;t1++) 
				RFIDBuf128[t1] = ~RFIDBuf128[t1];	//to invert the data bits
			if ( !RFIDParityCheck() ) 
				continue;
		}
		for(t1=0;t1<7;t1++)			//to remove start bits (9 bits)
		{
			RFIDBuf128[t1] = RFIDBuf128[t1+1];
		}
		RFIDBuf64Shift();
		
		RFIDCustomerID = RFIDBuf128[0] & 0xF0;	//to get customer ID
		for(t1=0;t1<5;t1++) 
			RFIDBuf64Shift();
		RFIDCustomerID = RFIDCustomerID | (RFIDBuf128[0]>>4);
		
		*pl_card_id = 0L;			//to get Card ID
		for(t1=0;t1<8;t1++)
		{
			for(t2=0;t2<5;t2++) RFIDBuf64Shift();
			*pl_card_id = (*pl_card_id<<4) | (RFIDBuf128[0]>>4);
		}
		
		if ( *pl_card_id==RFIDLastCardID )
		{
			return 0;
		}
		else  
		{
			RFIDLastCardID = *pl_card_id;
			return 1;
		}
	}

	RFIDLastCardID = 0;
	
	if ( !DataOK ) 
		return 0;
	DataOK = 0;
	if ( !CheckParity() )
	{
		*pl_card_id = (Data>>1);
		ExtCardReader = 1;	
		
		return 1;
	}
	else
	{
		return 0;
	}
}

static long io_ioctl(struct file *filep, unsigned int cmd, unsigned long arg)
{
	int ret;
  	switch(cmd)
  	{
		case IO_IDCARD_IN:
			rfid_get_card_id(&card_id);
			printk("card id : %llu\n", card_id);
			ret = copy_to_user( (void*)arg, &card_id, sizeof(card_id) );
			if(ret < 0)
				return -1;
			else
				card_id = 0;
			break;
		default:
			break;
	}
	return 0;
}


ssize_t io_read(struct file *file, char __user *buf, size_t size, loff_t *ppos)
{
	int val;
	int err;
	if(__gpio_get_value(A33_PL11) )
		val = 1;
	else
		val = 0;
	err = copy_to_user(buf, &val, 1);
	if(err < 0)
		return -1;
	else
		return 0;
}	


static struct file_operations io_fops = {
	.owner = THIS_MODULE,
	.open  = io_open,
	.unlocked_ioctl = io_ioctl,
	.read = io_read,	   
};



static int __init IODev_init(void)
{
	int err;
	
	if (major) {
		devid = MKDEV(major, 0);
		register_chrdev_region(devid, 1, "io");  
	} else {
		alloc_chrdev_region(&devid, 0, 1, "io"); 
		major = MAJOR(devid);                     
	}
	
	cdev_init(&io_cdev, &io_fops);
	cdev_add(&io_cdev, devid, 1);

	cls = class_create(THIS_MODULE, "io");
	device_create(cls, NULL, devid, NULL, "io"); /* for operaton   -->   /dev/io */


	//request and configure the rfid data io
	err = gpio_request(A33_PL11,"RFIDData");
	if(err){
		printk("Cannot Request the gpio of  %d\n", A33_PL11);
		goto out;
	}
		
	err = gpio_direction_input(A33_PL11);
	if (err < 0) {
		printk("Cannot set the gpio to input mode. \n");
		gpio_free(A33_PL11);
		goto out;
	}

	//err = request_irq(A33_PL11_IRQ, rfid_data_intterupt_handler, (IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING), "RFIDData", NULL);
	//if (err < 0) {
	//	printk("Cannot Request the irq, ERROR =  %d\n", err);
	//	gpio_free(A33_PL11);
	//	goto out;
	//}
	//disable_irq(A33_PL11_IRQ);
	//ioremap the register
	pt_sunxi_pwm_regs = ioremap(0x01C21400, sizeof(struct sunxi_pwm_regs));
	p_pio_phcfg0 = (volatile unsigned long *)ioremap(0x01C20800+0xfc, sizeof( unsigned long));

	 //set PH1 to PWM1 mode
	*p_pio_phcfg0 &= 0xffffff8f;
	*p_pio_phcfg0 |= (0x02 << 4);  
	
	
	out_125KHz();   // initial

	return 0;

out:
	device_destroy(cls, devid);
	class_destroy(cls);
	cdev_del(&io_cdev);
	unregister_chrdev_region(devid, 1);
	return -1;
}

static void __exit IODev_exit(void)
{
	printk("IODev_exit :");
	pwm_ch1_disable();
	pt_sunxi_pwm_regs->pwm_ctrl_reg = 0;

	if((1 << 29) & pt_sunxi_pwm_regs->pwm_ctrl_reg){
		printk("exit :PWM1 is busy now\n");
	}

	gpio_free(A33_PL11);
	//free_irq(A33_PL11_IRQ, NULL);
	device_destroy(cls, devid);
	class_destroy(cls);

	cdev_del(&io_cdev);
	unregister_chrdev_region(devid, 1);
	iounmap(pt_sunxi_pwm_regs);
	iounmap(p_pio_phcfg0);
}

module_init(IODev_init);
module_exit(IODev_exit);


MODULE_AUTHOR ("www.gzseeing.com");
MODULE_DESCRIPTION("IO DRIVER");
MODULE_LICENSE("GPL");



