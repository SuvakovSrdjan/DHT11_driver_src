// DHT11 driver
// http://www.uugear.com/portfolio/dht11-humidity-temperature-sensor-module/
// --sajt za dht11
/*#include <asm/uaccess.h>
#include <linux/errno.h>
#include <linux/fcntl.h>
#include <linux/fs.h>
#include <linux/hrtimer.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/types.h>
*/
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/errno.h>
#include <linux/fcntl.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/hrtimer.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/types.h>
MODULE_LICENSE("Dual BSD/GPL");

#define BUF_LEN 80

/* GPIO registers base address. */
#define GPIO_BASE_ADDR (0x3F200000)
//--

// 000 = GPIO Pin 'x' is an input
// 001 = GPIO Pin 'x' is an output
// By default GPIO pin is being used as an input
#define GPIO_DIRECTION_IN (0)
#define GPIO_DIRECTION_OUT (1)
//--

// Handle GPIO: 0-9
/* GPIO Function Select 0. */
#define GPFSEL0_BASE_ADDR (GPIO_BASE_ADDR + 0x00000000)

// Handle GPIO: 10-19
/* GPIO Function Select 1. */
#define GPFSEL1_BASE_ADDR (GPIO_BASE_ADDR + 0x00000004)

// Handle GPIO: 20-29
/* GPIO Function Select 2. */
#define GPFSEL2_BASE_ADDR (GPIO_BASE_ADDR + 0x00000008)

// Handle GPIO: 30-39
/* GPIO Function Select 3. */
#define GPFSEL3_BASE_ADDR (GPIO_BASE_ADDR + 0x0000000C)

// Handle GPIO: 40-49
/* GPIO Function Select 4. */
#define GPFSEL4_BASE_ADDR (GPIO_BASE_ADDR + 0x00000010)

// Handle GPIO: 50-53
/* GPIO Function Select 5. */
#define GPFSEL5_BASE_ADDR (GPIO_BASE_ADDR + 0x00000014)
//--

// GPIO: 0-31
/* GPIO Pin Output Set 0. */
#define GPSET0_BASE_ADDR (GPIO_BASE_ADDR + 0x0000001C)

// GPIO: 32-53
/* GPIO Pin Output Set 1. */
#define GPSET1_BASE_ADDR (GPIO_BASE_ADDR + 0x00000020)

//--
// GPIO: 0-31
/* GPIO Pin Output Clear 0. */
#define GPCLR0_BASE_ADDR (GPIO_BASE_ADDR + 0x00000028)

// GPIO: 32-53
/* GPIO Pin Output Clear 1. */
#define GPCLR1_BASE_ADDR (GPIO_BASE_ADDR + 0x0000002C)

//--
// GPIO: 0-31
/* GPIO Pin Level 0. */
#define GPLEV0_BASE_ADDR (GPIO_BASE_ADDR + 0x00000034)

// GPIO: 32-53
/* GPIO Pin Level 1. */
#define GPLEV1_BASE_ADDR (GPIO_BASE_ADDR + 0x00000038)
//--

// GPIO: 0-53
/* GPIO Pin Pull-up/down Enable. */
#define GPPUD_BASE_ADDR (GPIO_BASE_ADDR + 0x00000094)

// GPIO: 0-31
/* GPIO Pull-up/down Clock Register 0. */
#define GPPUDCLK0_BASE_ADDR (GPIO_BASE_ADDR + 0x00000098)

// GPIO: 32-53
/* GPIO Pull-up/down Clock Register 1. */
#define GPPUDCLK1_BASE_ADDR (GPIO_BASE_ADDR + 0x0000009C)

/* PUD - GPIO Pin Pull-up/down */
#define PULL_NONE (0)
#define PULL_DOWN (1)
#define PULL_UP (2)

/* Avilable GPIO pins. */
#define GPIO_02 (2)
#define GPIO_03 (3)
#define GPIO_04 (4)
#define GPIO_05 (5)
#define GPIO_06 (6)
#define GPIO_07 (7)
#define GPIO_08 (8)
#define GPIO_09 (9)
#define GPIO_10 (10)
#define GPIO_11 (11)
#define GPIO_12 (12)
#define GPIO_13 (13)
#define GPIO_14 (14)
#define GPIO_15 (15)
#define GPIO_16 (16)
#define GPIO_17 (17)
#define GPIO_18 (18)
#define GPIO_19 (19)
#define GPIO_20 (20)
#define GPIO_21 (21)
#define GPIO_22 (22)
#define GPIO_23 (23)
#define GPIO_24 (24)
#define GPIO_25 (25)
#define GPIO_26 (26)
#define GPIO_27 (27)

struct data_format {
  char data[5];
};
// A data buffer
struct data_format DHT11_data_buffer[BUF_LEN];
int DHT11_sending_buffer[BUF_LEN];
int dht11_dat[5] = { 0, 0, 0, 0, 0 };

// DHT11 driver function declarations
int DHT11_driver_init(void);
void DHT11_driver_exit(void);
static int DHT11_driver_open(struct inode *, struct file *);
static int DHT11_driver_release(struct inode *, struct file *);
void DHT11_driver_write_to_buffer(char pin);
static ssize_t DHT11_driver_write_buffer_to_file(struct file *, const char *buf,
                                                 size_t, loff_t *);

struct file_operations DHT11_driver_fops = {
  open : DHT11_driver_open,
  release : DHT11_driver_release,
  // read : DHT11_driver_read,
  write : DHT11_driver_write_buffer_to_file
};

/* Declaration of the init and exit functions. */
module_init(DHT11_driver_init);
module_exit(DHT11_driver_exit);

// Driver major number
int DHT11_device_major_number;

/* Blink timer vars. */
static struct hrtimer blink_timer;


#define MS_TO_NS(x) ((x)*1E6L)
#define TIMER_SEC 0
#define TIMER_40_us_SEC 50 * 1000        /* 40 uS delay za DHT11 */
#define TIMER_70_us_SEC 80 * 1000        /* 70 uS */
#define TIMER_18_us_SEC 25 * 1000        /* 18 uS */
#define TIMER_24_us_SEC 30 * 1000        /* 24 uS */
#define TIMER_54_us_SEC 60 * 1000        /* 54 uS */
#define TIMER_18_ms_SEC 20 * 1000 * 1000 /* 18 mS */
#define TIMER_1_us_SEC 1 * 1000 	     // 1 uS

unsigned int GetGPFSELReg(char pin) {
  unsigned int addr;

  if (pin >= 0 && pin < 10)
    addr = GPFSEL0_BASE_ADDR;
  else if (pin >= 10 && pin < 20)
    addr = GPFSEL1_BASE_ADDR;
  else if (pin >= 20 && pin < 30)
    addr = GPFSEL2_BASE_ADDR;
  else if (pin >= 30 && pin < 40)
    addr = GPFSEL3_BASE_ADDR;
  else if (pin >= 40 && pin < 50)
    addr = GPFSEL4_BASE_ADDR;
  else /*if(pin >= 50 && pin <53) */
    addr = GPFSEL5_BASE_ADDR;

  return addr;
}

char GetGPIOPinOffset(char pin) {
  if (pin >= 0 && pin < 10)
    pin = pin;
  else if (pin >= 10 && pin < 20)
    pin -= 10;
  else if (pin >= 20 && pin < 30)
    pin -= 20;
  else if (pin >= 30 && pin < 40)
    pin -= 30;
  else if (pin >= 40 && pin < 50)
    pin -= 40;
  else /*if(pin >= 50 && pin <53) */
    pin -= 50;

  return pin;
}

static enum hrtimer_restart blink_timer_callback(struct hrtimer *param) {  
    //hrtimer_forward(&blink_timer, ktime_get(), kt);
    //return HRTIMER_RESTART;
	return HRTIMER_NORESTART;
}

void timer_delay(ktime_t timer , int time_to_wait) {

  
  timer = ktime_set(TIMER_SEC, time_to_wait);
  blink_timer.function = &blink_timer_callback;
  hrtimer_start(&blink_timer, timer, HRTIMER_MODE_REL);

}

char GetGpioPinValue(char pin) {
  void *addr = NULL;
  unsigned int tmp;
  unsigned int mask;

  /* Get base address of gpio level register. */
  addr = (pin < 32) ? (void *)GPLEV0_BASE_ADDR : (void *)GPLEV1_BASE_ADDR;
  pin = (pin < 32) ? pin : pin - 32;

  /* Read gpio pin level. */
  addr = ioremap((unsigned long)addr, 4);
  tmp = ioread32(addr);
  mask = 0x1 << pin;
  tmp &= mask;

  return (tmp >> pin);
}

void ClearGpioPin(char pin) {
  void *addr = NULL;
  unsigned int tmp;

  /* Get base address of gpio clear register. */
  addr = (pin < 32) ? (void *)GPCLR0_BASE_ADDR : (void *)GPCLR1_BASE_ADDR;
  pin = (pin < 32) ? pin : pin - 32;

  /* Clear gpio. */
  addr = ioremap((unsigned long)addr, 4);
  tmp = 0x1 << pin;
  iowrite32(tmp, addr);
}

void SetGpioPin(char pin) {
  void *addr = NULL;
  unsigned int tmp;

  /* Get base address of gpio set register. */
  addr = (pin < 32) ? (void *)GPSET0_BASE_ADDR : (void *)GPSET1_BASE_ADDR;
  pin = (pin < 32) ? pin : pin - 32;

  /* Set gpio. */
  addr = ioremap((unsigned long)addr, 4);
  tmp = 0x1 << pin;
  iowrite32(tmp, addr);
}

void SetInternalPullUpDown(char pin, char value) {
  unsigned int base_addr_gppud;
  unsigned int base_addr_gppudclk;
  void *addr = NULL;
  unsigned int tmp;
  unsigned int mask;

  /* Get base address of GPIO Pull-up/down Register (GPPUD). */
  base_addr_gppud = GPPUD_BASE_ADDR;

  /* Get base address of GPIO Pull-up/down Clock Register (GPPUDCLK). */
  base_addr_gppudclk = (pin < 32) ? GPPUDCLK0_BASE_ADDR : GPPUDCLK1_BASE_ADDR;

  /* Get pin offset in register . */
  pin = (pin < 32) ? pin : pin - 32;

  /* Write to GPPUD to set the required control signal (i.e. Pull-up or
     Pull-Down or neither
     to remove the current Pull-up/down). */
  addr = ioremap(base_addr_gppud, 4);
  iowrite32(value, addr);

  /* Wait 150 cycles  this provides the required set-up time for the control
   * signal */

  /* Write to GPPUDCLK0/1 to clock the control signal into the GPIO pads you
     wish to
     modify  NOTE only the pads which receive a clock will be modified, all
     others will
     retain their previous state. */
  addr = ioremap(base_addr_gppudclk, 4);
  tmp = ioread32(addr);
  mask = 0x1 << pin;
  tmp |= mask;
  iowrite32(tmp, addr);

  /* Wait 150 cycles  this provides the required hold time for the control
   * signal */

  /* Write to GPPUD to remove the control signal. */
  addr = ioremap(base_addr_gppud, 4);
  iowrite32(PULL_NONE, addr);

  /* Write to GPPUDCLK0/1 to remove the clock. */
  addr = ioremap(base_addr_gppudclk, 4);
  tmp = ioread32(addr);
  mask = 0x1 << pin;
  tmp &= (~mask);
  iowrite32(tmp, addr);
}

void SetGpioPinDirection(char pin, char direction) {
  unsigned int base_addr;
  void *addr = NULL;
  unsigned int tmp;
  unsigned int mask;

  /* Get base address of function selection register. */
  base_addr = GetGPFSELReg(pin);

  /* Calculate gpio pin offset. */
  pin = GetGPIOPinOffset(pin);

  /* Set gpio pin direction. */
  addr = ioremap(base_addr, 4);
  tmp = ioread32(addr);
  if (direction) { // set as output: set 1
    mask = 0x1 << (pin * 3);
    tmp |= mask;
  } else { // set as input: set 0
    mask = ~(0x1 << (pin * 3));
    tmp &= mask;
  }
  iowrite32(tmp, addr);
}

// Function to be called before writing data to the data buffer
void SendInitSequenceAndReciveData(char pin) {
  uint8_t i , j = 0;
  uint8_t laststate = PULL_UP;
  uint8_t counter = 0;
  static ktime_t kt1 , kt2;
  static ktime_t kt[85];

  dht11_dat[0] = dht11_dat[1] = dht11_dat[2] = dht11_dat[3] = dht11_dat[4] = 0;

  /* Send init sequence to GPIO_04 */
  // Init GPIO_04
  // Set GPIO_04 direction to out to send init sequence
  SetGpioPinDirection(pin, GPIO_DIRECTION_OUT);
  //ClearGpioPin(pin);
  SetInternalPullUpDown(pin , PULL_DOWN);
  timer_delay(kt1 , TIMER_18_ms_SEC);
  SetInternalPullUpDown(pin , PULL_UP);
  timer_delay(kt2 , TIMER_40_us_SEC);
  SetGpioPinDirection(pin, GPIO_DIRECTION_IN);
  printk(KERN_INFO "Device is now sending data to GPIO_04 PIN\n");

  //Detect change and read data
  for(i = 0; i < 85; i++) {
      counter = 0;

      while(GetGpioPinValue(pin) == laststate) {
          counter++;
          timer_delay(kt[i] , TIMER_1_us_SEC);
          if(counter == 225) {
              break;
          }
          //printk(KERN_INFO "Current state of GPIO_4: %c" , GetGpioPinValue(pin));
      }
      laststate = GetGpioPinValue(pin);

      if(counter == 225)
        break;

      //Ignore the first 3 transitions
      if( (i >= 4) && (i % 2 == 0)) {
        dht11_dat[j/8] <<= 1;
        if(counter > 16)
            dht11_dat[j/8] |= 1;
        j++;
      }
  }

  if ( (j >= 40) &&
      (dht11_dat[4] == ( (dht11_dat[0] + dht11_dat[1] + dht11_dat[2] + dht11_dat[3]) & 0xFF) ) )
  {
     printk(KERN_INFO "Humidity = %d.%d Temperature = %d.%d *C\n", dht11_dat[0], dht11_dat[1], dht11_dat[2], dht11_dat[3] );
  } else  {
      printk(KERN_INFO "Data not good, skip\n" );
  }
}

void ClearDataBuffer(void) {
  int i = 0, j = 0;

  for (i = 0; i < BUF_LEN; i++) {
    for (j = 0; j < 5; j++) {
      DHT11_data_buffer[i].data[j] = 0;
    }
  }
}

// Functon operations:
// - Clear data buffer
// - Init timer
// - Init GPIO Pins
int DHT11_driver_init(void) {
  int result = -1;
  //int i = 0;

  // Registering the device
  result = register_chrdev(0, "DHT11_driver", &DHT11_driver_fops);
  if (result < 0) {
    printk(KERN_INFO "DHT11_driver: cannot obtain device major number %d\n",
           DHT11_device_major_number);
    return result;
  }
  DHT11_device_major_number = result;
  printk(KERN_INFO "Successfuly obtained device major number\n");

  // Init data buffer
  // TODO: Check if you didnt fuck up
  /*  DHT11_data_buffer = kmalloc(BUF_LEN, GFP_KERNEL);
    if (!DHT11_data_buffer) {
      result = -ENOMEM;
      goto fail;
    }
  */
  memset(DHT11_data_buffer, 0, BUF_LEN);
  printk(KERN_INFO "Successfuly allocated memory for ring buffer\n");

  // Initialize high resolution timer.
  //hrtimer_init(&blink_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
  //kt = ktime_set(TIMER_SEC, TIMER_40_us_SEC);
  //blink_timer.function = &blink_timer_callback;
  //hrtimer_start(&blink_timer, kt, HRTIMER_MODE_REL);

  //printk(KERN_INFO "So far , so good\n");
  hrtimer_init(&blink_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
  // Initialize the device to send data
  //for(i = 0; i < 85; i++) {
  	SendInitSequenceAndReciveData(GPIO_04);
  //}
  // Fill the data buffer here
  // TODO: odraditi u funkciji DHT11_driver_write_to_buffer(...);

  return 0;
}

// Cleanup:
// - Free memory from the buffer
// - Reset timer
// - Clear GPIO pin
void DHT11_driver_exit() {
  // Freeing the major number
  unregister_chrdev(DHT11_device_major_number, "DHT11_driver");

  printk(KERN_INFO "Removig DHT11_driver module\n");

  // Release high resolution timer
  hrtimer_cancel(&blink_timer);

  // Clear GPIO pins
  ClearGpioPin(GPIO_04);
  SetGpioPinDirection(GPIO_04 , GPIO_DIRECTION_IN);
}

// A function that fills the data buffer
/*void DHT11_driver_write_to_buffer(char pin) {
  int i = 0;

  for (i = 0; i < BUF_LEN; i++) {
    // Now device is sending data
    // TODO: Fill the data buffer acording to the specification
    while (GetGpioPinValue(GPIO_04) == 0) {
      // tmp << 1;
    }
    // 0 - 54us PULL_DOWN & 24us PULL_UP
    // 1 - 54us PULL_DOWN & 70us PULL_UP
    // END - 54us PULL_DOWN & >70US PULL_UP
  }
}*/

/* File open function. */
static int DHT11_driver_open(struct inode *inode, struct file *filp) {
  /* Initialize driver variables here. */

  /* Reset the device here. */

  /* Success. */
  return 0;
}

/* File close function. */
static int DHT11_driver_release(struct inode *inode, struct file *filp) {
  /* Success. */
  return 0;
}

// Fucntion that send buffer data to user space using function copy_to_user
// Asuming the driver is allready filled
static ssize_t DHT11_driver_write_buffer_to_file(struct file *filp,
                                                 const char *buf, size_t len,
                                                 loff_t *f_pos) {
  int data_size = BUF_LEN;
  int i = 0;
  for (i = 0; i < BUF_LEN; i++) {
    DHT11_sending_buffer[i] =
        DHT11_data_buffer[i].data[0] & DHT11_data_buffer[i].data[1] &
        DHT11_data_buffer[i].data[2] & DHT11_data_buffer[i].data[3];
  }

  if (*f_pos == 0) {
    // Getting size of valid data
    // data_size = strlen(DHT11_data_buffer);

    // Send data to user space
    if (copy_to_user(DHT11_sending_buffer, buf, data_size) != 0) {
      return -EFAULT;
    } else {
      ClearDataBuffer();
      (*f_pos) += data_size;
      return data_size;
    }
  } else {
    return 0;
  }
}
