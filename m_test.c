#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>

#include <linux/interrupt.h>
#include <linux/gpio.h>


#define DRIVER_AUTHOR "Jesus Trujillo <jesusftrujillor@gmail.com>"
#define DRIVER_DESC   "ADF702x"

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
/* IRQ handler - fired on interrupt                                         */
/****************************************************************************/
static irqreturn_t r_irq_handler(int irq, void *dev_id, struct pt_regs *regs) {

  unsigned long flags;

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
		  IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
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
int r_init(void) {

  printk(KERN_NOTICE "Hello adf702x!\n");
  r_int_config();

  return 0;
}

void r_cleanup(void) {
  printk(KERN_NOTICE "Goodbye\n");
  r_int_release();

  return;
}


module_init(r_init);
module_exit(r_cleanup);


/****************************************************************************/
/* Module licensing/description block.                                      */
/****************************************************************************/
MODULE_LICENSE("GPL");
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
