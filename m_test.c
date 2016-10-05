#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <asm/uaccess.h>

#include "bit_circ_buf.h"

void push_bit_to_cbuf(struct bit_circ_buf *cb, char value) {
  int pos = (cb->head % cb->size);
  if ( value ) {
    cb->buf[cb->head / 8] |= 1 << (cb->head % 8);
  } else {
    cb->buf[cb->head / 8] &= ~(1 << (cb->head % 8));
  }
  cb->head = (cb->head + 1) % cb->size;
}

int cbuf_size(struct bit_circ_buf *cb) {
  int nbytes = 0;
  if (cb->head > cb->tail) {
    nbytes = (cb->head / 8) - (cb->tail / 8);
  } else if (cb->head < cb->tail) {
    nbytes = (cb->size * 8) - (cb->tail / 8) - (cb->head / 8);
  }
  return nbytes;
}

#define DRIVER_AUTHOR "Jesus Trujillo <jesusftrujillor@gmail.com>"
#define DRIVER_DESC   "ADF702x"

#define DEVICE_NAME   "adf702x"

// we want GPIO_4 (pin 7 on P1 pinout raspberry pi rev. 2 board)
// to generate interrupt
#define GPIO_TXRXCLK_GPIO                4   // pin 7 on P1
#define GPIO_TXRXDATA_GPIO               17  // pin 11 on P1
#define GPIO_TXRX_SW_GPIO                25  // pin 22 on P1
#define GPIO_SLE_GPIO                    18  // pin 12 on P1
#define GPIO_SDATA_GPIO                  21  // pin 13 on P1 (rpi1 only)
#define GPIO_SREAD_GPIO                  22  // pin 15 on P1
#define GPIO_SCLK_GPIO                   23  // pin 16 on P1
#define GPIO_CE_GPIO                     24  // pin 18 on P1

// text below will be seen in 'cat /proc/interrupt' command
#define GPIO_TXRXCLK_GPIO_DESC           "ADF702x transceiver CLK"

// below is optional, used in more complex code, in our case, this could be
// NULL
#define GPIO_TXRXCLK_GPIO_DEVICE_DESC    "adf702x"

static struct gpio adf702x_gpios[] = {
  { GPIO_TXRXCLK_GPIO,     GPIOF_IN,          "adf702x transceiver clock"  },
  { GPIO_TXRXDATA_GPIO,    GPIOF_IN,          "adf702x transceiver data"   },
  { GPIO_TXRX_SW_GPIO,     GPIOF_INIT_LOW,    "adf702x TX/RX switch"       },
  { GPIO_SLE_GPIO,         GPIOF_INIT_LOW,    "adf702x serial load enable" },
  { GPIO_SDATA_GPIO,       GPIOF_INIT_LOW,    "adf702x serial data out"    },
  { GPIO_SREAD_GPIO,       GPIOF_IN,          "adf702x serial data in"     },
  { GPIO_SCLK_GPIO,        GPIOF_INIT_LOW,    "adf702x serial clock"       },
  { GPIO_CE_GPIO,          GPIOF_INIT_LOW,    "adf702x chip enable"        }
};

/****************************************************************************/
/* Interrupts variables block                                               */
/****************************************************************************/
short int irq_txrxclk_gpio    = 0;

/****************************************************************************/
/* Char device variables block                                              */
/****************************************************************************/
#define BUFFER_SIZE 64

static int major_number;

static int     adf702x_open(struct inode *, struct file *);
static int     adf702x_release(struct inode *, struct file *);
static ssize_t adf702x_read(struct file *, char *, size_t, loff_t *);
static ssize_t adf702x_write(struct file *, const char *, size_t, loff_t *);

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
  };

/****************************************************************************/
/* IRQ handler - fired on interrupt                                         */
/****************************************************************************/
static irqreturn_t r_irq_handler(int irq, void *dev_id, struct pt_regs *regs) {

  unsigned long flags;

  // push data bit 
  push_bit_to_cbuf(state.rcv_buf, gpio_get_value(GPIO_TXRXDATA_GPIO));
  
  // disable hard interrupts (remember them in flag 'flags')
  local_irq_save(flags);

  // NOTE:
  // Anonymous Sep 17, 2013, 3:16:00 PM:
  // You are putting printk while interupt are disabled. printk can block.
  // It's not a good practice.
  //
  // hardware.coder:
  // http://stackoverflow.com/questions/8738951/printk-inside-an-interrupt-handler-is-it-really-that-bad

  printk(KERN_NOTICE "Interrupt [%d] for device %s was triggered !.\n",
	 irq, (char *) dev_id);

  // restore hard interrupts
  local_irq_restore(flags);

  return IRQ_HANDLED;
}


/****************************************************************************/
/* This function configures interrupts.                                     */
/****************************************************************************/
void r_int_config(void) {

  // request GPIOs
  int ret = 0;
  if ( (ret = gpio_request_array(adf702x_gpios, ARRAY_SIZE(adf702x_gpios))) < 0 ) {
    printk("GPIO failure requesting pin, errno=%d\n", ret);
    return;
  }

  // set interrupt handler for TxRxCLK
  if ( (irq_txrxclk_gpio = gpio_to_irq(GPIO_TXRXCLK_GPIO)) < 0 ) {
    printk("GPIO to IRQ mapping faiure %s\n", GPIO_TXRXCLK_GPIO_DESC);
    return;
  }

  printk(KERN_NOTICE "Mapped int %d\n", irq_txrxclk_gpio);

  if (request_irq(irq_txrxclk_gpio,
		  (irq_handler_t ) r_irq_handler,
		  IRQF_TRIGGER_RISING, // | IRQF_TRIGGER_FALLING,
		  GPIO_TXRXCLK_GPIO_DESC,
		  GPIO_TXRXCLK_GPIO_DEVICE_DESC)) {
    printk("Irq Request failure\n");
    return;
  }

  return;
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
/* Module init / cleanup block.                                             */
/****************************************************************************/
static int __init r_init(void) {
  int err = 0;
  
  printk(KERN_NOTICE "Hello adf702x!\n");
  r_int_config();

  if ( (major_number = register_chrdev(0, DEVICE_NAME, &fops)) < 0 ) {
    printk(KERN_ERR "adf702x: unable to get major number");
    err = -EIO;
    goto err;
  }

  return 0;
  
 err:
  return err;
}

void r_cleanup(void) {
  printk(KERN_NOTICE "Goodbye\n");

  unregister_chrdev(major_number, DEVICE_NAME);
  r_int_release();
}

static int adf702x_open(struct inode *inodep, struct file *filep) {
  // TODO implement file open.
  // use filep->private_data to hold status my view of the circular buffer
  printk(KERN_NOTICE "adf702x: opened");
  return 0;
}

static ssize_t adf702x_read(struct file *filep, char *buffer, size_t len, loff_t *offset) {
  // TODO implement file read
  ssize_t nbytes = 0;

  if (cbuf_size(state.rcv_buf) < len) {
    nbytes = cbuf_size(state.rcv_buf);
  } else {
    nbytes = len;
  }

  if ( copy_to_user(buffer, &state.rcv_buf->buf[state.rcv_buf->tail / 8], nbytes) != 0 ) {
    printk(KERN_INFO "adf702x: failed to send %d characters to the user\n", nbytes);
    return -EFAULT;
  }

  state.rcv_buf->tail = (state.rcv_buf->tail + (nbytes * 8)) % state.rcv_buf->size;
  printk(KERN_INFO "adf702x: read %d bytes", nbytes);
  
  return nbytes;
}

static ssize_t adf702x_write(struct file *filep, const char *buffer, size_t len, loff_t *offset) {
  int i = 0;
  
  // TODO implement file write
  printk(KERN_ALERT "adf702x: sorry write operation isn't supported yet");

  for ( i = 0; i < (len * 8); i++ ) {
    char bit_value = (buffer[i / 8] >> (i % 8)) & 0x1;
    push_bit_to_cbuf(state.rcv_buf, bit_value);
  }

  printk(KERN_INFO "adf702x: wrote %d bytes", len);
  return len;
}

static int adf702x_release(struct inode *inodep, struct file *filep) {
  // TODO implement file release

  printk(KERN_INFO "adf702x: released");
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
