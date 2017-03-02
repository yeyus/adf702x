#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <asm/uaccess.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/stat.h>
#include <linux/moduleparam.h>

#include "bit_circ_buf.h"

void push_bit_to_cbuf(struct bit_circ_buf *cb, char value) {
  if ( value & 1 ) {
    cb->buf[cb->head / 8] |= 1 << (7 - (cb->head % 8));
  } else {
    cb->buf[cb->head / 8] &= ~(1 << (7 - (cb->head % 8)));
  }
  cb->head = (cb->head + 1) % cb->size;
}

ssize_t cbuf_size(struct bit_circ_buf *cb) {
  ssize_t nbytes = 0;
  if (cb->head > cb->tail) {
    nbytes = (cb->head / 8) - (cb->tail / 8);
  } else if (cb->head < cb->tail) {
    nbytes = ((cb->size - cb->tail) + cb->head) / 8;
  }
  return nbytes;
}

#define DRIVER_AUTHOR "Jesus Trujillo <jesusftrujillor@gmail.com>"
#define DRIVER_DESC   "ADF702x"

#define DEVICE_NAME   "adf702x"
#define TAG DEVICE_NAME ": "

// Module params section
static int gpio_txrxclk_pin = -EINVAL;
static int gpio_txrxdata_pin = -EINVAL;
static int gpio_txrx_sw_pin = -EINVAL;
static int gpio_sle_pin = -EINVAL;
static int gpio_sdata_pin = -EINVAL;
static int gpio_sread_pin = -EINVAL;
static int gpio_sclk_pin = -EINVAL;
static int gpio_ce_pin = -EINVAL;

module_param(gpio_txrxclk_pin, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(gpio_txrxclk_pin, "adf702x transceiver clock pin");
module_param(gpio_txrxdata_pin, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(gpio_txrxdata_pin, "adf702x transceiver data pin");
module_param(gpio_txrx_sw_pin, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(gpio_txrx_sw_pin, "adf702x transmit/receive switch pin");
module_param(gpio_sle_pin, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(gpio_sle_pin, "adf702x serial load enable pin");
module_param(gpio_sdata_pin, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(gpio_sdata_pin, "adf702x serial data pin");
module_param(gpio_sread_pin, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(gpio_sread_pin, "adf702x serial read pin");
module_param(gpio_sclk_pin, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(gpio_sclk_pin, "adf702x serial clock pin");
module_param(gpio_ce_pin, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(gpio_ce_pin, "adf702x chip eneable pin");

// we want GPIO_4 (pin 7 on P1 pinout raspberry pi rev. 2 board)
// to generate interrupt
//#define GPIO_TXRXCLK_GPIO                4   // pin 7 on P1
//#define GPIO_TXRXDATA_GPIO               17  // pin 11 on P1
//#define GPIO_TXRX_SW_GPIO                25  // pin 22 on P1
//#define GPIO_SLE_GPIO                    18  // pin 12 on P1
//#define GPIO_SDATA_GPIO                  21  // pin 13 on P1 (rpi1 only)
//#define GPIO_SREAD_GPIO                  22  // pin 15 on P1
//#define GPIO_SCLK_GPIO                   23  // pin 16 on P1
//#define GPIO_CE_GPIO                     24  // pin 18 on P1

// text below will be seen in 'cat /proc/interrupt' command
#define GPIO_TXRXCLK_GPIO_DESC           "ADF702x transceiver CLK"
#define GPIO_TXRXCLK_GPIO_DEVICE_DESC    "adf702x"

static struct gpio adf702x_gpios[] = {
  { 0,     GPIOF_IN,          "adf702x transceiver clock"  },
  { 0,     GPIOF_IN,          "adf702x transceiver data"   },
  { 0,     GPIOF_INIT_LOW,    "adf702x TX/RX switch"       },
  { 0,     GPIOF_INIT_LOW,    "adf702x serial load enable" },
  { 0,     GPIOF_INIT_LOW,    "adf702x serial data out"    },
  { 0,     GPIOF_IN,          "adf702x serial data in"     },
  { 0,     GPIOF_INIT_LOW,    "adf702x serial clock"       },
  { 0,     GPIOF_INIT_LOW,    "adf702x chip enable"        }
};

/****************************************************************************/
/* IOCTL definitions                                                        */
/****************************************************************************/

#define ADF702X_IOC_MAGIC 190

#define ADF702X_IOC_COMMAND     _IOW(ADF702X_IOC_MAGIC,   1, unsigned long)
#define ADF702X_IOC_READBACK    _IOWR(ADF702X_IOC_MAGIC,  2, unsigned long)
#define ADF702X_IOC_ENABLE      _IOW(ADF702X_IOC_MAGIC,   3, unsigned long)
#define ADF702X_IOC_DISABLE     _IOW(ADF702X_IOC_MAGIC,   4, unsigned long)
#define ADF702X_IOC_TEST        _IOWR(ADF702X_IOC_MAGIC,  5, unsigned long)

#define ADF702X_IOC_MAXNR 5

#define T1_DELAY_US 10
#define T6_DELAY_US 20

/****************************************************************************/
/* Interrupts variables block                                               */
/****************************************************************************/
short int irq_txrxclk_gpio    = 0;

/****************************************************************************/
/* Char device variables block                                              */
/****************************************************************************/
#define BUFFER_SIZE 1024

static DECLARE_WAIT_QUEUE_HEAD(readers_wait_q);

static int major_number;

static int     adf702x_open(struct inode *, struct file *);
static int     adf702x_release(struct inode *, struct file *);
static ssize_t adf702x_read(struct file *, char *, size_t, loff_t *);
static ssize_t adf702x_write(struct file *, const char *, size_t, loff_t *);
static long    adf702x_ioctl(struct file *, unsigned int, unsigned long);

struct adf702x_state {
  struct bit_circ_buf *rcv_buf;
};

static char adf702x_rcv_buf[BUFFER_SIZE];
static struct bit_circ_buf adf702x_state_rcv = {
  .buf = adf702x_rcv_buf,
  .size = sizeof(adf702x_rcv_buf) * 8,
  .head = 0,
  .tail = 0,
};

static struct adf702x_state state = {
  .rcv_buf = &adf702x_state_rcv,
};

static struct file_operations fops =
  {
    .open = adf702x_open,
    .read = adf702x_read,
    .write = adf702x_write,
    .release = adf702x_release,
    .unlocked_ioctl = adf702x_ioctl,
  };

/****************************************************************************/
/* IRQ handler - fired on interrupt                                         */
/****************************************************************************/
static irqreturn_t r_irq_handler(int irq, void *dev_id, struct pt_regs *regs) {

  //unsigned long flags;
  
  // push data bit 
  int gpio_state = gpio_get_value(gpio_txrxdata_pin);
  push_bit_to_cbuf(state.rcv_buf, gpio_state);

  if (state.rcv_buf->head % 8 == 0) {
    wake_up_interruptible(&readers_wait_q);
  }
  
  // disable hard interrupts (remember them in flag 'flags')
  //local_irq_save(flags);

  // NOTE:
  // Anonymous Sep 17, 2013, 3:16:00 PM:
  // You are putting printk while interupt are disabled. printk can block.
  // It's not a good practice.
  //
  // hardware.coder:
  // http://stackoverflow.com/questions/8738951/printk-inside-an-interrupt-handler-is-it-really-that-bad

  //printk(KERN_NOTICE "Interrupt [%d] for device %s was triggered !, gpio_value= %d.\n",
  //	 irq, (char *) dev_id, gpio_state);

  // restore hard interrupts
  //local_irq_restore(flags);

  return IRQ_HANDLED;
}


/****************************************************************************/
/* This function releases interrupts.                                       */
/****************************************************************************/
void r_int_release(void) {
  free_irq(irq_txrxclk_gpio, GPIO_TXRXCLK_GPIO_DEVICE_DESC);
  gpio_free_array(adf702x_gpios, ARRAY_SIZE(adf702x_gpios));
  
  return;
}


/****************************************************************************/
/* This function configures gpios and interrupts.                           */
/****************************************************************************/
int r_int_config(void) {

  int ret = 0;
  int i = 0;

  // Setting pin numbers coming from module params
  adf702x_gpios[0].gpio = gpio_txrxclk_pin;
  adf702x_gpios[1].gpio = gpio_txrxdata_pin;
  adf702x_gpios[2].gpio = gpio_txrx_sw_pin;
  adf702x_gpios[3].gpio = gpio_sle_pin;
  adf702x_gpios[4].gpio = gpio_sdata_pin;
  adf702x_gpios[5].gpio = gpio_sread_pin;
  adf702x_gpios[6].gpio = gpio_sclk_pin;
  adf702x_gpios[7].gpio = gpio_ce_pin;

  // print pin descriptions and assignments
  for (i = 0; i < ARRAY_SIZE(adf702x_gpios); i++) {
    printk(KERN_INFO TAG "pin %s = %d", adf702x_gpios[i].label, adf702x_gpios[i].gpio);
  }

  // request GPIOs
  if ( (ret = gpio_request_array(adf702x_gpios, ARRAY_SIZE(adf702x_gpios))) < 0 ) {
    printk(KERN_ERR TAG "GPIO failure requesting pin, errno=%d\n", ret);
    goto err;
  }

  // set interrupt handler for TxRxCLK
  if ( (irq_txrxclk_gpio = gpio_to_irq(gpio_txrxclk_pin)) < 0 ) {
    printk(KERN_ERR TAG "GPIO to IRQ mapping faiure %s\n", GPIO_TXRXCLK_GPIO_DESC);
    ret = irq_txrxclk_gpio;
    goto err;
  }

  printk(KERN_NOTICE TAG "mapped int %d\n", irq_txrxclk_gpio);
  if ( (ret = request_irq(irq_txrxclk_gpio,
		  (irq_handler_t ) r_irq_handler,
		  IRQF_TRIGGER_RISING, // | IRQF_TRIGGER_FALLING,
		  GPIO_TXRXCLK_GPIO_DESC,
			  GPIO_TXRXCLK_GPIO_DEVICE_DESC)) < 0 ) {
    printk(KERN_ERR TAG "irq Request failure\n");
    goto err;
  }

  init_waitqueue_head(&readers_wait_q);

  return ret;
  
 err:
  r_int_release();
  return ret;
}

/****************************************************************************/
/* Module init / cleanup block.                                             */
/****************************************************************************/
static int __init r_init(void) {
  int err = 0;
  
  printk(KERN_NOTICE "Hello adf702x!\n");
  if ( (err = r_int_config()) < 0) {
    printk(KERN_ERR TAG "unable to get usage of gpio pin");
    goto err;
  }

  if ( (major_number = register_chrdev(0, DEVICE_NAME, &fops)) < 0 ) {
    printk(KERN_ERR TAG "unable to get major number");
    err = -EIO;
    goto err;
  }

  printk(KERN_NOTICE TAG "got major_number=%d", major_number);
  return 0;
  
 err:
  return err;
}

void r_cleanup(void) {
  printk(KERN_NOTICE TAG "goodbye\n");

  unregister_chrdev(major_number, DEVICE_NAME);
  r_int_release();
}

static int adf702x_open(struct inode *inodep, struct file *filep) {
  // TODO implement file open.
  // use filep->private_data to hold status my view of the circular buffer
  printk(KERN_NOTICE TAG "opened\n");
  return 0;
}

static ssize_t adf702x_read(struct file *filep, char *buffer, size_t len, loff_t *offset) {
  // TODO implement file read
  struct bit_circ_buf *cb = state.rcv_buf;
  ssize_t nbytes;
  ssize_t tail_bytes;
  long ret = 0;

  if (cbuf_size(cb) < 1) {
    if (filep->f_flags & O_NONBLOCK) {
      return -EAGAIN;
    }
    if (wait_event_interruptible(readers_wait_q, cbuf_size(cb) != 0)) {
      return -ERESTARTSYS;
    }
  }

  nbytes = min(cbuf_size(cb), (ssize_t)len);
  tail_bytes = (cb->size - cb->tail) / 8;
  
  if ( cb->head > cb->tail ) {
    ret = copy_to_user(buffer, &cb->buf[cb->tail / 8], nbytes);
  } else if ( cb->head < cb->tail ) {
    ret = copy_to_user(buffer, &cb->buf[cb->tail / 8], min(nbytes, tail_bytes));
    if (nbytes > tail_bytes) {
      ret += copy_to_user(buffer + min(nbytes, tail_bytes), &cb->buf[0], nbytes - min(nbytes, tail_bytes));
    }
  }
  //printk(KERN_INFO "cbuf_size is %d, requested length is %d\n", cbufsize, len);
  //printk(KERN_INFO "circ_buf before status-> head=%d tail=%d size=%d\n", cb->head, cb->tail, cb->size);
  
  if ( ret != 0 ) {
    printk(KERN_INFO TAG "failed to send %d characters to the user\n", nbytes);
    return -EFAULT;
  }

  cb->tail = (cb->tail + ((nbytes - ret) * 8)) % cb->size;
  //printk(KERN_INFO "adf702x: read %d bytes\n", nbytes);
  //printk(KERN_INFO "circ_buf after  status-> head=%d tail=%d size=%d\n", cb->head, cb->tail, cb->size);

  return nbytes;
}

static ssize_t adf702x_write(struct file *filep, const char *buffer, size_t len, loff_t *offset) {
  int i = 0;
  
  // TODO implement file write
  printk(KERN_ALERT TAG "sorry write operation isn't supported yet");

  for ( i = 0; i < (len * 8); i++ ) {
    char bit_value = (buffer[i / 8] >> (i % 8)) & 0x1;
    push_bit_to_cbuf(state.rcv_buf, bit_value);
  }

  printk(KERN_INFO TAG "wrote %d bytes", len);
  return len;
}

static long adf702x_ioctl(struct file *filep, unsigned int cmd, unsigned long arg) {
  int ret_val = -EFAULT;
  int i, bit;
  
  switch (cmd)
  {
    case ADF702X_IOC_COMMAND:
      printk(KERN_INFO TAG "ioctl: command value: %08lx", arg & 0xFFFFFFFF);
      // SLE down
      gpio_set_value(gpio_sle_pin, 0);
      for (i = 31; i>=0; i--) {
	// SCLK down
	gpio_set_value(gpio_sclk_pin, 0);
	// SDATA set bit
	gpio_set_value(gpio_sdata_pin, ((arg & 0xFFFFFFFF) >> i) & 1);
	// delay t1= >10ns
	udelay(T1_DELAY_US);
	// SCLK up
	gpio_set_value(gpio_sclk_pin, 1);
	// delay t1= >10ns
	udelay(T1_DELAY_US);	
      }
      // SCLK down
      gpio_set_value(gpio_sclk_pin, 0);
      // delay t1= >10ns
      udelay(T1_DELAY_US);	
      // SLE up
      gpio_set_value(gpio_sle_pin, 1);
      // delay t6= >20ns
      udelay(T6_DELAY_US);
      // SLE down
      gpio_set_value(gpio_sle_pin, 0);
      return 0;

    case ADF702X_IOC_READBACK:
      printk(KERN_INFO TAG "ioctl: readback value: %08lx", arg & 0x01FF);
      ret_val = 0;
      // SLE down
      gpio_set_value(gpio_sle_pin, 0);
      for (i = 8; i>=0; i--) {
	// SCLK down
	gpio_set_value(gpio_sclk_pin, 0);
	// SDATA set bit
	gpio_set_value(gpio_sdata_pin, ((arg & 0xFF) >> i) & 1);
	// delay t1= >10ns
	udelay(T1_DELAY_US);
	// SCLK up
	gpio_set_value(gpio_sclk_pin, 1);
	// delay t1= >10ns
	udelay(T1_DELAY_US);	
      }
      // SCLK down
      gpio_set_value(gpio_sclk_pin, 0);
      // delay t1= >10ns
      udelay(T1_DELAY_US);	
      // SLE up
      gpio_set_value(gpio_sle_pin, 1);
      // delay t1 = >10ns
      udelay(T1_DELAY_US);
      for (i = 15; i>=0; i--) {
	// SCLK down
	gpio_set_value(gpio_sclk_pin, 0);
	// SREAD get bit
	bit = gpio_get_value(gpio_sread_pin);
	if (bit) {
	  ret_val |= (1 << i);
	}
	// delay t1= >10ns
	udelay(T1_DELAY_US);
	// SCLK up
	gpio_set_value(gpio_sclk_pin, 1);
	// delay t1= >10ns
	udelay(T1_DELAY_US);	
      }
      // SCLK down
      gpio_set_value(gpio_sclk_pin, 0);
      // delay t1= >10ns
      udelay(T1_DELAY_US);	
      // SLE down
      gpio_set_value(gpio_sle_pin, 0);
      printk(KERN_INFO TAG "ioctl: readback returns: %04x", ret_val & 0xFFFF);
      return ret_val & 0xFFFF;
      
    case ADF702X_IOC_ENABLE:
      printk(KERN_INFO TAG "ioctl: enable");
      gpio_set_value(gpio_ce_pin, 1);
      return 0;
 
    case ADF702X_IOC_DISABLE:
      printk(KERN_INFO TAG "ioctl: disable");
      gpio_set_value(gpio_ce_pin, 0);
      return 0;

    case ADF702X_IOC_TEST:
      printk(KERN_INFO TAG "ioctl: test value: %ld", arg);
      return arg + 1;

    default:
      printk(KERN_ERR TAG "unknown ioctl with %u", cmd);
      return -EINVAL;
  }
  
  return -EINVAL;
}

static int adf702x_release(struct inode *inodep, struct file *filep) {
  // TODO implement file release

  printk(KERN_INFO TAG "released");
  return 0;
}

module_init(r_init);
module_exit(r_cleanup);


/****************************************************************************/
/* Module licensing/description block.                                      */
/****************************************************************************/
MODULE_LICENSE("GPL");
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
