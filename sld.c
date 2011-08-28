#include <linux/kernel.h> /* printk() */
#include <linux/slab.h> /* kmalloc() */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <asm/uaccess.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/version.h>

#define DEVICE_NAME "lcd driver"
#define SLD "sld"

#define LINES 2
#define CHARS_PER_LINE 16

#define PIN_RS		AT91_PIN_PB3
#define PIN_RW		AT91_PIN_PB1
#define PIN_E		AT91_PIN_PA6
#define PIN_D0		AT91_PIN_PA9
#define PIN_D1		AT91_PIN_PA31
#define PIN_D2		AT91_PIN_PB11
#define PIN_D3		AT91_PIN_PB13
#define PIN_D4		AT91_PIN_PB21
#define PIN_D5		AT91_PIN_PB31
#define PIN_D6		AT91_PIN_PC1
#define PIN_D7		AT91_PIN_PC3

#define RS_INST		0
#define RS_DATA		1

#define RW_WRITE	0
#define RW_READ		1

#define LINE_0		0x00
#define LINE_1		0x40

#define LED_OFF		0
#define LED_ON		(!LED_OFF)

#define DISPLAY_OFF	0
#define DISPLAY_ON	(!DISPLAY_OFF)

static ssize_t sld_write(struct file *filp, const char __user *buff,
			 size_t count, loff_t *offp);
static ssize_t sld_read(struct file *filp, char __user *buff,
			size_t count, loff_t *offp);
static loff_t sld_llseek(struct file *filp, loff_t off, int whence);

static int sld_hw_write(unsigned char *buffer, size_t count);
static int sld_hw_write_diff(unsigned char *buffer, size_t count,
			     unsigned char *buffer_diff);
static int sld_hw_init(void);
static void sld_hw_dbus(unsigned char c);
static void sld_hw_e(void);
static void sld_hw_line(int);
static void sld_hw_clear(void);
static int sld_hw_busy(void);
static void sld_hw_display(int);

static void sld_led(int);

static void delay_1us(void);
static void delay_1ms(void);
static void delay_5ms(void);

static void sld_do_write(void);
static void sld_wq_write(struct work_struct*);

static const size_t BUFFER_SIZE = LINES * CHARS_PER_LINE;
//#define BUFFER_SIZE LINES * CHARS_PER_LINE
static char sld_buffer_1[LINES * CHARS_PER_LINE];
static char sld_buffer_2[LINES * CHARS_PER_LINE];
static char *sld_buffer = sld_buffer_1;
static char *sld_buffer_back = sld_buffer_2;
static struct semaphore sld_semaphore;
struct workqueue_struct *sld_workqueue;

#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,16)
//DECLARE_WORK(sld_work_write, sld_workqueue_write);
#else
static DECLARE_WORK(sld_work_write, sld_wq_write);
#endif

/* Needs putting dev fs */
static unsigned int dbus_to_pin[8] = {
	PIN_D0,
	PIN_D1,
	PIN_D2,
	PIN_D3,
	PIN_D4,
	PIN_D5,
	PIN_D6,
	PIN_D7,
};

static int sld_major;
static struct class *sld_class;
static struct device *sld_device;
static struct file_operations sld_fops = {
	.write = sld_write,
	.read = sld_read,
	.llseek = sld_llseek,
};
static struct cdev *sld_cdev;
static dev_t sld_dev;

#define SLD_COUNT 1
#define SLD_FIRST_MINOR 0
#define STATIC_MAJOR 0
#define SLD_WORKQUEUE "sld_workqueue"

static ssize_t sld_write(struct file *filp, const char __user *buff,
			 size_t count, loff_t *offp)

{
	int ret = 0;
	int len = min(count, BUFFER_SIZE - (size_t)*offp);

	if (down_interruptible(&sld_semaphore))
		return -ERESTARTSYS;

	if (copy_from_user(sld_buffer + *offp, buff, len)) {
		ret = -EFAULT;
		goto out;
	}

	if (count > len) {
		ret = ENOBUFS;
		goto out;
	} else {
		*offp += count;
		ret = count;
	}
	if (filp->f_flags==O_NONBLOCK || filp->f_flags==O_NDELAY) {
		queue_work(sld_workqueue, &sld_work_write);
	} else {
		sld_wq_write(NULL);
	}

	printk(KERN_INFO "sld_buffer = %s\n",  sld_buffer);	
out:
	up(&sld_semaphore);

	return ret;
}

static void sld_do_write(void)
{
	printk(KERN_INFO "%s\n", __FUNCTION__);
	sld_hw_clear();
	sld_hw_line(LINE_0);
	sld_hw_write(sld_buffer, 16);
	sld_hw_line(LINE_1);
	sld_hw_write(sld_buffer + 16, 16);
	/*
	  sld_hw_clear();
	  sld_hw_line(LINE_0);
	  sld_hw_write(sld_buffer, 16, sld_buffer_back);
	  sld_hw_line(LINE_1);
	  sld_hw_write(sld_buffer + 16, 16, sld_buffer_back);

	 */
}

static void sld_wq_write(struct work_struct *work)
{
	sld_do_write();
	/*
	static char *tmp_buffer = sld_buffer_back;
	sld_buffer_back = sld_buffer;
	sld_buffer = tmp_buffer;
	*/
}


static ssize_t sld_read(struct file *filp, char __user *buff,
			size_t count, loff_t *offp)
{
	return ENOSYS; /*not implemented*/
/*
  int ret = 0;
	if (copy_to_user(buff, "not implemented\n", 16)) {
		ret = -EFAULT;
		goto out;
	}
	if (*offp == 0) {
		*offp += 16;
		ret = 16;
	} else {
		ret = 0;
	}
out:	
	return ret;
*/
}
 
static loff_t sld_llseek(struct file *filp, loff_t off, int whence)
{
	loff_t newpos;

	switch(whence) {
	case 0: /* SEEK_SET */
		newpos = off;
		break;

	case 1: /* SEEK_CUR */
		newpos = filp->f_pos + off;
		break;

	case 2: /* SEEK_END */
		newpos = BUFFER_SIZE - 1 - off;
		break;

	default: /* can't happen */
		return -EINVAL;
	}
	
	if (newpos < 0 || newpos >= BUFFER_SIZE)
		return -EINVAL;
	filp->f_pos = newpos;
	
	return newpos;
}


static void sld_hw_line(int line)
{
	at91_set_gpio_value(PIN_RS, RS_INST);
	at91_set_gpio_value(PIN_RW, RW_WRITE);

	sld_hw_dbus(0x80 | line);
	sld_hw_e();
	delay_1ms();

}

static int sld_hw_write(unsigned char *buffer, size_t count)
{
	return sld_hw_write_diff(buffer, count, buffer);
}

static int sld_hw_write_diff(unsigned char *buffer, size_t count,
			     unsigned char *buffer_diff)
{
	int i;
	int k = 40;
	at91_set_gpio_value(PIN_RW, RW_WRITE);
	at91_set_gpio_value(PIN_RS, RS_DATA);
	delay_1us();
	for (i = 0; i < count; i++) {
		/*
		if (!(buffer[i]-buffer_diff[i])) {
			sld_inc_addr(); is it worth ?
			continue;
		}
		*/
		sld_hw_dbus(buffer[i]);
		sld_hw_e();
		k = 40;
		while(k--)
			delay_1us();
	}
		
	return 0;
}


static void sld_hw_dbus(unsigned char c)
{
	int i;
	for (i = 0; i < 8; i++) {
		at91_set_gpio_value(dbus_to_pin[i], c&0x01);
		c >>= 1;
	}
}
/*
static unsigned char sld_hw_dbus_read()
{
	int i;
	for (i = 0; i < 8; i++) {
		at91_set_gpio_value(dbus_to_pin[i], c&0x01);
		c >>= 1;
	}
}
*/

static void sld_hw_e(void)
{
	at91_set_gpio_value(PIN_E, 1);
	delay_1us();
	at91_set_gpio_value(PIN_E, 0);
	delay_1us();
}

static int sld_hw_init(void)
{
	at91_set_gpio_output(PIN_RS, RS_INST);
	at91_set_gpio_output(PIN_RW, RW_WRITE);
	at91_set_gpio_output(PIN_E, 0);

	at91_set_gpio_output(PIN_D7, 0);
	at91_set_gpio_output(PIN_D6, 0);
	at91_set_gpio_output(PIN_D5, 0);
	at91_set_gpio_output(PIN_D4, 0);
	at91_set_gpio_output(PIN_D3, 0);
	at91_set_gpio_output(PIN_D2, 0);//5x7
	at91_set_gpio_output(PIN_D1, 0);
	at91_set_gpio_output(PIN_D0, 0);

	// sld_hw_function_set()
	// sld_hw_cursor()
	// sld_hw_clear()
	// sld_hw_entry_mode()
// function set	
	sld_hw_dbus(0x38);
	sld_hw_e();
	delay_5ms();
	
// coursor
/*
	at91_set_gpio_value(PIN_D2, 1); // display
	at91_set_gpio_value(PIN_D1, 1); // cursor 
	at91_set_gpio_value(PIN_D0, 0); // blink  
*/	
	sld_hw_dbus(0x0E);
	sld_hw_e();
	delay_5ms();

// clear
	sld_hw_dbus(0x01);
	sld_hw_e();
	delay_5ms();
	
// entry mode
	sld_hw_dbus(0x02);
	sld_hw_e();
	delay_5ms();
	
	return 0;
}

static void sld_hw_clear(void)
{
	at91_set_gpio_value(PIN_RS, RS_INST);
	at91_set_gpio_value(PIN_RW, RW_WRITE);
	sld_hw_dbus(0x01);
	sld_hw_e();
	delay_1ms();
	if (sld_hw_busy()) {
		delay_1ms();
	}
}

static int sld_hw_busy(void)
{
	int val = 0;
/*
  not sure how it goes...
	at91_set_gpio_value(PIN_RS, RS_INST);
	at91_set_gpio_value(PIN_RW, RW_READ);
	at91_set_gpio_input(PIN_D7);
	sld_hw_e();
	val = at91_get_gpio_value(PIN_D7);
	at91_set_gpio_output(PIN_D7, 0);
*/	
	return val;
}

static void sld_hw_display(int show)
{
/*	
	int status = 0;
	at91_set_gpio_value(PIN_RS, RS_INST);
	at91_set_gpio_value(PIN_RW, RW_WRITE);
	
	sld_hw_dbus(show?0x08 | 0x40: 0x0);
	sld_hw_e();
	delay_1ms();
*/	
}

static void sld_led(int status)
{
	at91_set_gpio_value(AT91_PIN_PB19, 0);
	at91_set_gpio_value(AT91_PIN_PC9, status);
}

static void delay_1us(void)
{
	udelay(1);
}

static void delay_1ms(void)
{
	msleep(1);
}
static void delay_5ms(void)
{
	msleep(5);
}

static int __init sld_init(void)
{
	int res = 0;
	printk(KERN_INFO "Loading "DEVICE_NAME".\n");

#if STATIC_MAJOR
	//statically for major = 244, minor = 1
	sld_dev = MKDEV(STATIC_MAJOR, 0);
	register_chrdev_region(sld_dev, 1, SLD);
	if (res < 0) {
		printk(KERN_ERR "Failed on: register_chrdev_region()\n");
	} else {
		printk(KERN_INFO "Device allocated with major = %d\n",
			   MAJOR(sld_dev));
	}
#else
	sld_major = alloc_chrdev_region(&sld_dev, SLD_FIRST_MINOR,
					    SLD_COUNT, SLD);
	if (res < 0) {
		printk(KERN_ERR "Failed on: alloc_chrdev_region()\n");
	} else {
		printk(KERN_INFO "Device allocated with major = %d\n",
		       MAJOR(sld_dev));
	}
#endif	
	sld_major = MAJOR(sld_dev);
	sld_cdev = cdev_alloc();
	sld_cdev->owner = THIS_MODULE;
	sld_cdev->ops = &sld_fops;
	cdev_add(sld_cdev, sld_dev, 1);
	
	sld_class = class_create(THIS_MODULE, SLD);
	sld_device = device_create(sld_class, NULL,
				       sld_dev, "%s", SLD);

	sema_init(&sld_semaphore, 1);
/*
  struct device_driver sld_driver {
 .name = SLD;
 .bus = NULL;
 .owner = THIS_MODULE;
  
  .probe = NULL;
  .remove = NULL;
  
  .suspend = NULL;
  .resume = NULL;
  };
  driver_register(&sld_driver);
  DEVICE_ATTR(sld, 0644, show_sld, store_sld);

  device_register(device);
  device_create_file(device, &dev_attr_sld);
*/
	sld_workqueue = create_singlethread_workqueue(SLD_WORKQUEUE);
	sld_hw_init();
	sld_hw_line(LINE_1);
	
/*
  	sld_hw_display(DISPLAY_OFF);
*/	
	sld_hw_write(" Basiu kochana!#", 16);
/*	
	sld_hw_display(DISPLAY_ON);
*/
	sld_led(LED_ON);	

	return 0;
}

static void __exit sld_exit(void)
{
	destroy_workqueue(sld_workqueue);
//	void device_unregister(struct device *dev);
	device_destroy(sld_class, sld_dev);
	class_destroy(sld_class);
	cdev_del(sld_cdev);
	unregister_chrdev_region(sld_dev, SLD_COUNT);
	sld_hw_clear();
	sld_led(LED_OFF);
	printk(KERN_INFO "Unloading "DEVICE_NAME".\n");
}

module_init(sld_init);
module_exit(sld_exit);

MODULE_AUTHOR("Jakub Palider <jpalider@gmail.com>");
MODULE_DESCRIPTION("Simple Lcd Driver HD4478 alike displays");
MODULE_LICENSE("GPL");
