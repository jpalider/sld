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
#define SLD "lcd"

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
static void sld_hw_dbus_write(unsigned char c);
static unsigned char sld_hw_dbus_read(void);
static void sld_hw_set_addr(int address);
static void sld_hw_e(void);
static void sld_hw_line(int);
static void sld_hw_clear(void);
static int sld_hw_busy(void);
static void sld_hw_show_display(int);

static void sld_led(int);

static void delay_1us(void);
static void delay_1ms(void);
static void delay_5ms(void);

static void sld_do_write(void);
static void sld_wq_write(struct work_struct*);
static void sld_switch_buffers(void);

static const size_t BUFFER_SIZE = LINES * CHARS_PER_LINE;
static char sld_buffer_1[LINES * CHARS_PER_LINE];
static char sld_buffer_2[LINES * CHARS_PER_LINE];
static char *sld_buffer = sld_buffer_1;
static char *sld_buffer_back = sld_buffer_2;
static struct semaphore sld_semaphore;
struct workqueue_struct *sld_workqueue;

static DECLARE_WORK(sld_work_write, sld_wq_write);

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
	if (filp->f_flags==O_NONBLOCK || filp->f_flags==O_NDELAY) {
		if (down_trylock(&sld_semaphore)) {
			return -EAGAIN;
		}
	} else 	if (down_interruptible(&sld_semaphore)) {
		return -ERESTARTSYS;
	}

	if (copy_from_user(sld_buffer + *offp, buff, len)) {
		ret = -EFAULT;
		goto out;
	}

	if (count > len) {  /* ||*offt + len > BUFFER_SIZE */
		ret = ENOBUFS;
	} else {
		*offp += count;
		ret = count;
	}
	queue_work(sld_workqueue, &sld_work_write);
out:
	up(&sld_semaphore);

	return ret;
}

static void sld_do_write(void)
{
#if 0
	printk(KERN_INFO "%s\n", __FUNCTION__);
	sld_hw_clear();
	sld_hw_line(LINE_0);
	sld_hw_write(sld_buffer, 16);
	sld_hw_line(LINE_1);
	sld_hw_write(sld_buffer + 16, 16);
#else
	sld_hw_clear();
	sld_hw_line(LINE_0);
	sld_hw_write_diff(sld_buffer, CHARS_PER_LINE, sld_buffer_back);
	sld_hw_line(LINE_1);
	sld_hw_write_diff(sld_buffer + CHARS_PER_LINE, CHARS_PER_LINE, sld_buffer_back);

#endif
}

static void sld_wq_write(struct work_struct *work)
{
	sld_do_write();
	sld_switch_buffers();
}

static void sld_switch_buffers(void)
{
	char *tmp_buffer;
	tmp_buffer = sld_buffer_back;
	sld_buffer_back = sld_buffer;
	sld_buffer = tmp_buffer;
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
	gpio_set_value(PIN_RS, RS_INST);
	gpio_set_value(PIN_RW, RW_WRITE);

	sld_hw_dbus_write(0x80 | line);
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
	int jump = 0;
	int k = 40;
	gpio_set_value(PIN_RW, RW_WRITE);
	gpio_set_value(PIN_RS, RS_DATA);
	delay_1us();
	for (i = 0; i < count; i++) {
		if (!(buffer[i]-buffer_diff[i])) {
			jump++;			
			continue;
		}
		sld_hw_set_addr(i+jump);
		printk(KERN_ERR "jumped: %d\n", jump);
		jump = 0;
		sld_hw_dbus_write(buffer[i]);
		k = 40;
		while(k--) {
			/* sld_hw_busy(); */
			delay_1us();
		}
	}
	return 0;
}

static void sld_hw_dbus_write(unsigned char c)
{
	int i;
	for (i = 0; i < 8; i++) {
		gpio_set_value(dbus_to_pin[i], c&0x01);
		c >>= 1;
	}
	sld_hw_e();

}

static unsigned char sld_hw_dbus_read()
{
	int i;
	unsigned char val;
	unsigned char out = 0;

	for (i = 0; i < 8; i++) {
		gpio_direction_input(dbus_to_pin[i]);
	}
	
	delay_1us();
	gpio_set_value(PIN_E, 1);
	delay_1us();

	for (i = 0; i < 8; i++) {
		val = gpio_get_value(dbus_to_pin[i]) ? 1 : 0;
		out |= val << i;
		gpio_direction_output(dbus_to_pin[i], 0);
	}

	gpio_set_value(PIN_E, 0);
	delay_1us();

	return out;
}

static void sld_hw_set_addr(int address)
{
	gpio_set_value(PIN_RS, RS_INST);
	gpio_set_value(PIN_RW, RW_WRITE);
	sld_hw_dbus_write(0x80 | address);
	delay_1ms();
}

static void sld_hw_e(void)
{
	gpio_set_value(PIN_E, 1);
	delay_1us();
	gpio_set_value(PIN_E, 0);
	delay_1us();
}

static int sld_hw_init(void)
{
	gpio_direction_output(PIN_RS, RS_INST);
	gpio_direction_output(PIN_RW, RW_WRITE);
	gpio_direction_output(PIN_E, 0);

	gpio_direction_output(PIN_D7, 0);
	gpio_direction_output(PIN_D6, 0);
	gpio_direction_output(PIN_D5, 0);
	gpio_direction_output(PIN_D4, 0);
	gpio_direction_output(PIN_D3, 0);
	gpio_direction_output(PIN_D2, 0); /* 5x7 */
	gpio_direction_output(PIN_D1, 0);
	gpio_direction_output(PIN_D0, 0);

	/* sld_hw_function_set() */
	/* sld_hw_cursor() */
	/* sld_hw_clear() */
	/* sld_hw_entry_mode() */
/* function set	 */
	sld_hw_dbus_write(0x38);
	delay_1ms();
	
/* coursor */
	gpio_set_value(PIN_D2, 1); /* display */
	gpio_set_value(PIN_D1, 1); /* cursor */
	gpio_set_value(PIN_D0, 0); /* blink */  	
	sld_hw_dbus_write(0x0C);
	delay_1ms();

/* clear */
	sld_hw_dbus_write(0x01);
	delay_1ms();
	
/* entry mode */
	sld_hw_dbus_write(0x02);
	delay_1ms();
	
	return 0;
}

static void sld_hw_clear(void)
{
	int k = 40;
	unsigned char c = 0;
	gpio_set_value(PIN_RS, RS_INST);
	gpio_set_value(PIN_RW, RW_WRITE);
	sld_hw_dbus_write(0x01);

	if (sld_hw_busy()) {
		delay_1ms();
	}
}

static int sld_hw_busy(void)
{
	int val = 0;

	gpio_set_value(PIN_RS, RS_INST);
	gpio_set_value(PIN_RW, RW_READ);

	val = sld_hw_dbus_read();

	return val & PIN_D7;
}

static void sld_hw_show_display(int show)
{
	/* int status = 0; */
	/* unsigned char bus = 0; */

	/* gpio_set_value(PIN_RS, RS_DATA); */
	/* gpio_set_value(PIN_RW, RW_READ); */

	/* bus = sld_hw_dbus_read(); */
	
	/* sld_hw_dbus_write(show?0x08 | 0x40: 0x0); */
	/* delay_1ms(); */
}

static void sld_led(int status)
{
	gpio_set_value(AT91_PIN_PB19, 0);
	gpio_set_value(AT91_PIN_PC9, status);
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
	/* statically for major = 244, minor = 1 */
	sld_dev = MKDEV(STATIC_MAJOR, 0);
	res = register_chrdev_region(sld_dev, 1, SLD);
	if (res < 0) {
		printk(KERN_ERR "Failed on: register_chrdev_region()\n");
		return -1;
	} else {
		printk(KERN_INFO "Device allocated with major = %d\n",
		       MAJOR(sld_dev));
	}
	sld_major = MAJOR(sld_dev);
#else
	sld_major = alloc_chrdev_region(&sld_dev, SLD_FIRST_MINOR,
					SLD_COUNT, SLD);
	if (sld_major < 0) {
		printk(KERN_ERR "Failed on: alloc_chrdev_region()\n");
		return -1;
	} else {
		printk(KERN_INFO "Device allocated with major = %d\n",
		       MAJOR(sld_dev));
	}
#endif
	
	sld_cdev = cdev_alloc();
	sld_cdev->owner = THIS_MODULE;
	sld_cdev->ops = &sld_fops;
	cdev_add(sld_cdev, sld_dev, 1);
	
	sld_class = class_create(THIS_MODULE, SLD);
	sld_device = device_create(sld_class, NULL, sld_dev, "%s", SLD);

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

	gpio_set_value(PIN_RS, RS_INST);
	gpio_set_value(PIN_RW, RW_WRITE);
	sld_hw_dbus_write(0x14);
	delay_1ms();	
	
	sld_hw_set_addr(0x06);
	sld_hw_write("To jest tekst!#", 16);
	sld_hw_set_addr(0x00);
	sld_hw_write("To jest tekst!#", 5);

/*	
	sld_hw_display(DISPLAY_ON);
	sld_hw_display(DISPLAY_OFF);
*/
	sld_led(LED_ON);
	return 0;
}

static void __exit sld_exit(void)
{
	destroy_workqueue(sld_workqueue);
	/* void device_unregister(struct device *dev); */
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
