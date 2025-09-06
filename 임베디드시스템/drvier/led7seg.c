#include <linux/atomic.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/printk.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/version.h>

#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/errno.h>

static int open(struct inode *, struct file *);
static int release(struct inode * , struct file *);
static ssize_t write(struct file *, const char __user * , size_t, loff_t *);


#define SUCCESS 0
#define DEVICE_NAME "led7seg"
static int major;
static struct class *cls;

enum {
    CEDV_NOT_USED =0,
    CEDV_EXCLUSIVE_OPEN =1
};

static atomic_t alredy_open = ATOMIC_INIT(CEDV_NOT_USED);


#define GPIO_BASE_ADDR 0xFE200000

static volatile uint32_t *gpsel1;
static volatile uint32_t *gpsel2;
static volatile uint32_t *gpset0;
static volatile uint32_t *gpclr0;

static struct file_operations chardev_fops = {
    .open = open,
    .release = release,
    .write = write,
    
};

static int open(struct inode *, struct file *){
    if(atomic_cmpxchg(&alredy_open, CEDV_NOT_USED,CEDV_EXCLUSIVE_OPEN))
        return -EBUSY;

    try_module_get(THIS_MODULE);

    return SUCCESS;
}

static int release(struct inode * , struct file *)
{
    atomic_set(&alredy_open, CEDV_NOT_USED);

    module_put(THIS_MODULE);

    return SUCCESS;
}


static ssize_t write(struct file *filep, const char __user * buffer, size_t len, loff_t * off){
    
    int i;
    uint8_t byteval, gp17val;

    static uint8_t dec_to_seg[] = {0x3f,0x06,0x5b,0x4f};
    static uint8_t dec_to_led[] = {0x01,0x02,0x04,0x08};
    
    
    
    if(get_user(byteval, buffer) != 0) {
        pr_alert("Copy from user failde \n");
        return -EFAULT;
    }
     pr_info("user input : 0x&%X\n",byteval);

     for(i = 0; i < 8; i++){
        gp17val =(0x80 & (dec_to_seg[byteval] << i)) >> 7;
        *gpset0 = gp17val << 17;
        *gpclr0 = (0x1 & !gp17val) << 17;

        *gpset0 = 1 << 27;
        udelay(1);
        *gpclr0 =  1 << 27;
        *gpset0 = 1<< 18;
        udelay(1);
        *gpclr0 = 1 << 18;
     }
    i = 0;
     for(i = 0; i < 8; i++){
        gp17val =(0x80 & (dec_to_led[byteval] << i)) >> 7;
        *gpset0 = gp17val << 17;
        *gpclr0 = (0x1 & !gp17val) << 17;

        *gpset0 = 1 << 27;
        udelay(1);
        *gpclr0 =  1 << 27;
        *gpset0 = 1<< 18;
        udelay(1);
        *gpclr0 = 1 << 18;
     }

    return len;
}


static int __init led7seg_init(void)
{
    major = register_chrdev(0,DEVICE_NAME,&chardev_fops);
    if(major < 0){
        pr_info("Registering char device failed with %d\n",major);
        return major;
    }
    pr_info("Major number : %d\n", major);

    
    cls = class_create(DEVICE_NAME);

    struct device *dev = device_create(cls,NULL, MKDEV(major,0),NULL,DEVICE_NAME);

    int err = PTR_ERR(dev);
    if(err < 0) {
        pr_info("Creating char device with %d\n",err);
        return err;
    }
    pr_info("/dev/%s\n",DEVICE_NAME);

    gpsel1 = (uint32_t*)ioremap(GPIO_BASE_ADDR | 0x04,4);
    gpsel2 = (uint32_t*)ioremap(GPIO_BASE_ADDR | 0x08,4);
    gpset0 = (uint32_t*)ioremap(GPIO_BASE_ADDR | 0x1C,4);
    gpclr0 = (uint32_t*)ioremap(GPIO_BASE_ADDR | 0x28,4);

    *gpsel1= (*gpsel1 & ~(0x7 << 21)) | (1<<21);
    *gpsel1= (*gpsel1 & ~(0x7 << 24)) | (1<<24);
    *gpsel2= (*gpsel2 & ~(0x7 << 21)) | (1<<21);
    *gpclr0 = (1<<17) | (1 << 18) | (1 << 27);
    return SUCCESS;
}

static void __exit led7seg_exit(void){
    device_destroy(cls,MKDEV(major,0));
    class_destroy(cls);
    unregister_chrdev(major,DEVICE_NAME);


    iounmap(gpsel1);
    iounmap(gpsel2);
    iounmap(gpset0);
    iounmap(gpclr0);
    
}


module_init(led7seg_init);
module_exit(led7seg_exit);

MODULE_LICENSE("GPL");

