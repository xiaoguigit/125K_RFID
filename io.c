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
#include <linux/kthread.h>
#include <linux/sched.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>


//#define __DEBUG_PRINTK__

#ifdef __DEBUG_PRINTK__ 
#define debug(format,...) printk("Line: %05d: "format"\n", __LINE__, ##__VA_ARGS__)
#else 
#define debug(format,...)   
#endif 



static int major;
static dev_t devid;
static struct cdev io_cdev;
static struct class *cls;
static DECLARE_WAIT_QUEUE_HEAD(get_id_waitq);
static volatile int ev_get_id = 0;

static int cnt = 0;
static int v1, v2;
static struct timespec start_uptime;  
static int start_here = 0;
static unsigned long used_time;
static int bits = 0;
static uint64_t my_pl_card_id;
static uint64_t real_card_id;

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
static unsigned char  RFIDBuf128[16];
static unsigned char RFIDDecodeOK;
static unsigned char RFIDCustomerID;
static unsigned long RFIDLastCardID;
static unsigned char const RFIDMask[8] = {0x80,0x40,0x20,0x10,0x08,0x04,0x02,0x01};
static unsigned char tmp_bit1;


static unsigned char rfid_read_bit(void)
{
	return gpio_get_value(A33_PL11)? 1: 0;
}

static void pwm_ch1_enable(void)
{
	pt_sunxi_pwm_regs->pwm_ctrl_reg |= (1 << 19);
	debug("%s ok!\n", __FUNCTION__);
}

static void pwm_ch1_disable(void)
{
	pt_sunxi_pwm_regs->pwm_ctrl_reg &= (0 << 19);
	debug("%s ok!\n", __FUNCTION__);
}

static void out_125KHz(void)
{
	pwm_ch1_disable();
	if((1 << 29) & pt_sunxi_pwm_regs->pwm_ctrl_reg){
		debug("PWM1 is busy now\n");
	}
	pt_sunxi_pwm_regs->pwm_ctrl_reg = 0x003f8000;// 24MHz / 1  == 24MHz  				
	pt_sunxi_pwm_regs->pwm_ch1_period &= 0;
	pt_sunxi_pwm_regs->pwm_ch1_period |= (191 << 16);// SET DUTY 125k    (理论值为192，实际测出191较为合适)
	pt_sunxi_pwm_regs->pwm_ch1_period |= (95<< 0);  // 50 %
	pwm_ch1_enable();
}

static int io_open(struct inode *inode, struct file *file)
{
	debug("io_open ok.\n");
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
	// debug("%x,%x,%x,%x\n",buff1,buff,a,b);
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

#if 0
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

#endif

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
	


static unsigned long get_time_use(void)
{
        struct timespec uptime, temp;  
        getrawmonotonic(&uptime);  	
        if ((uptime.tv_nsec-start_uptime.tv_nsec)<0) {
		temp.tv_sec = uptime.tv_sec-start_uptime.tv_sec-1;
		temp.tv_nsec = 1000000000+uptime.tv_nsec-start_uptime.tv_nsec;
	} else {
		temp.tv_sec = uptime.tv_sec-start_uptime.tv_sec;
	 	temp.tv_nsec = uptime.tv_nsec-start_uptime.tv_nsec;
	}
	return (temp.tv_sec * 1000000000 + temp.tv_nsec)/1000;  /* unit us */
}

static long io_ioctl(struct file *filep, unsigned int cmd, unsigned long arg)
{
	int ret;
  	switch(cmd)
  	{
		case IO_IDCARD_IN:
			//rfid_get_card_id(&card_id);
			debug("card id : %llu\n", card_id);
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
	int ret;
	char tmp_buf[32];
        //enable_irq(A33_PL11_IRQ);
        wait_event_interruptible(get_id_waitq, ev_get_id);
        ev_get_id = 0;
	 sprintf(tmp_buf, "%llu", real_card_id);
        //printk("read card : %s    len = %d\n", tmp_buf,  strlen(tmp_buf));
	ret = copy_to_user( buf, tmp_buf, strlen(tmp_buf) );
	if(ret < 0)
		return ret;
	else
		card_id = 0;

        memset(tmp_buf, 0, sizeof(tmp_buf));
	return 0;


}	


static struct file_operations io_fops = {
	.owner = THIS_MODULE,
	.open  = io_open,
	.unlocked_ioctl = io_ioctl,
	.read = io_read,	   
};



int get_start_action(void)
{
    int ret = -1;
    if(cnt == 0){
         v1 = rfid_read_bit();
    }else{
        v2 = rfid_read_bit();
    }

    cnt++;

    if(cnt == 2){
        cnt = 0;
        if(v1 != v2){
            ret = 0;
        }else{
            ret =  -1;
        }
    }
    return ret;
}


static void start_to_decode(void)
{
    unsigned char  t1,t2;

    if(start_here){
        used_time = get_time_use();
        if(used_time > ABOVE2BITS){
            RFIDDecodeOK = 0;
            start_here = 0;
            bits = 0;
        }else{
            if ( used_time < BELOW1BIT ){
                RFIDDecodeOK = 0;
                start_here = 0;
                bits = 0;
            }

            if(rfid_read_bit()){
                if ( used_time>=ABOVE1BIT ){
                    RFIDBuf128[bits>>3] &= ~RFIDMask[bits&0x07];	//save double bits of 0
                    if(bits < 127){
                        bits++;
                    }
                }
                RFIDBuf128[bits>>3] &= ~RFIDMask[bits&0x07];		//save single bits of 0
            }else{
                if ( used_time >= ABOVE1BIT ){
                    RFIDBuf128[bits >>3] |= RFIDMask[bits & 0x07];	//save double bits of 1
                    if(bits < 127){
                        bits++;
                    }
                } 
                RFIDBuf128[bits>>3] |= RFIDMask[bits&0x07];	//save single bit of 1
            }
            bits++;
        }

        if(bits == 128){
            for(bits=0;bits<64;bits++){
                if ( RFIDBuf128[bits>>2] & RFIDMask[(bits<<1)&0x07] )
                    RFIDBuf128[bits>>3] |= RFIDMask[bits&0x07];
                else
                    RFIDBuf128[bits>>3] &= ~RFIDMask[bits&0x07];
            }
            RFIDDecodeOK = 1;
        }

        if ( RFIDDecodeOK && !RFIDParityCheck() ){
            for(t1=0;t1<8;t1++) 
                RFIDBuf128[t1] = ~RFIDBuf128[t1];	//to invert the data bits
            if ( !RFIDParityCheck() ) {
                //printk("RFIDParityCheck   error\n");
                RFIDDecodeOK = 0;
                start_here = 0;
                bits = 0;
            }
        }

        if(RFIDDecodeOK){
            for(t1=0;t1<7;t1++)	{               //to remove start bits (9 bits)
                RFIDBuf128[t1] = RFIDBuf128[t1+1];
            }

            RFIDBuf64Shift();
		
            RFIDCustomerID = RFIDBuf128[0] & 0xF0;	//to get customer ID
            for(t1=0;t1<5;t1++) 
			RFIDBuf64Shift();

            RFIDCustomerID = RFIDCustomerID | (RFIDBuf128[0]>>4);

            my_pl_card_id = 0L;			//to get Card ID
            for(t1=0;t1<8;t1++){
                for(t2=0;t2<5;t2++) 
                    RFIDBuf64Shift();
                my_pl_card_id = (my_pl_card_id <<4) | (RFIDBuf128[0]>>4);
            }
		
            if ( my_pl_card_id ==RFIDLastCardID ){
                //printk("card id : %llu\n", my_pl_card_id);
                real_card_id = my_pl_card_id;
                ev_get_id = 1;
                wake_up_interruptible(&get_id_waitq);  
                start_here = 0;
                bits = 0;
                RFIDDecodeOK = 0;
                //disable_irq(A33_PL11_IRQ);

            }else{
                RFIDLastCardID = my_pl_card_id;
                RFIDDecodeOK = 0;
                start_here = 0;
                bits = 0;
            }
        }

        getrawmonotonic(&start_uptime);  	
    }

}


static irqreturn_t rfid_data_intterupt_handler(int irq, void *dev_id)
{
#if 1
    if(start_here == 0){
        if(0 == get_start_action()){
            start_here = 1;
            bits = 0;
            getrawmonotonic(&start_uptime);  	
            return IRQ_RETVAL(IRQ_HANDLED);
        }
    }

    start_to_decode();
#endif

	return IRQ_RETVAL(IRQ_HANDLED);
}

       

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
		debug("Cannot Request the gpio of  %d\n", A33_PL11);
		goto out;
	}
		
	err = gpio_direction_input(A33_PL11);
	if (err < 0) {
		debug("Cannot set the gpio to input mode. \n");
		gpio_free(A33_PL11);
		goto out;
	}

	err = request_irq(A33_PL11_IRQ, rfid_data_intterupt_handler, (IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING), "RFIDData", NULL);
	if (err < 0) {
		debug("Cannot Request the irq, ERROR =  %d\n", err);
		gpio_free(A33_PL11);
		goto out;
        }
    
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
	debug("IODev_exit :");
	pwm_ch1_disable();
	pt_sunxi_pwm_regs->pwm_ctrl_reg = 0;

	if((1 << 29) & pt_sunxi_pwm_regs->pwm_ctrl_reg){
		debug("exit :PWM1 is busy now\n");
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



