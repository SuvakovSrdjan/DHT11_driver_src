//DHT11 driver
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/fcntl.h>
#include <linux/proc_fs.h>
#include <linux/string.h>
#include <linux/ioport.h>
#include <linux/ktime.h>
#include <linux/hrtimer.h>
#include <asm/io.h>
#include <asm/uaccess.h>



MODULE_LICENSE("Dual BSD/GPL");

#define BUF_LEN 80

/* GPIO registers base address. */
#define GPIO_BASE_ADDR (0x3F200000)
//--

//000 = GPIO Pin 'x' is an input
//001 = GPIO Pin 'x' is an output
// By default GPIO pin is being used as an input
#define GPIO_DIRECTION_IN  (0)
#define GPIO_DIRECTION_OUT (1)
//--

//Handle GPIO: 0-9
/* GPIO Function Select 0. */
#define GPFSEL0_BASE_ADDR (GPIO_BASE_ADDR + 0x00000000)

//Handle GPIO: 10-19
/* GPIO Function Select 1. */
#define GPFSEL1_BASE_ADDR (GPIO_BASE_ADDR + 0x00000004)

//Handle GPIO: 20-29
/* GPIO Function Select 2. */
#define GPFSEL2_BASE_ADDR (GPIO_BASE_ADDR + 0x00000008)

//Handle GPIO: 30-39
/* GPIO Function Select 3. */
#define GPFSEL3_BASE_ADDR (GPIO_BASE_ADDR + 0x0000000C)

//Handle GPIO: 40-49
/* GPIO Function Select 4. */
#define GPFSEL4_BASE_ADDR (GPIO_BASE_ADDR + 0x00000010)

//Handle GPIO: 50-53
/* GPIO Function Select 5. */
#define GPFSEL5_BASE_ADDR (GPIO_BASE_ADDR + 0x00000014)
//--

//GPIO: 0-31
/* GPIO Pin Output Set 0. */
#define GPSET0_BASE_ADDR (GPIO_BASE_ADDR + 0x0000001C)

//GPIO: 32-53
/* GPIO Pin Output Set 1. */
#define GPSET1_BASE_ADDR (GPIO_BASE_ADDR + 0x00000020)

//--
//GPIO: 0-31
/* GPIO Pin Output Clear 0. */
#define GPCLR0_BASE_ADDR (GPIO_BASE_ADDR + 0x00000028)

//GPIO: 32-53
/* GPIO Pin Output Clear 1. */
#define GPCLR1_BASE_ADDR (GPIO_BASE_ADDR + 0x0000002C)

//--
//GPIO: 0-31
/* GPIO Pin Level 0. */
#define GPLEV0_BASE_ADDR (GPIO_BASE_ADDR + 0x00000034)

//GPIO: 32-53
/* GPIO Pin Level 1. */
#define GPLEV1_BASE_ADDR (GPIO_BASE_ADDR + 0x00000038)
//--

//GPIO: 0-53
/* GPIO Pin Pull-up/down Enable. */
#define GPPUD_BASE_ADDR (GPIO_BASE_ADDR + 0x00000094)

//GPIO: 0-31
/* GPIO Pull-up/down Clock Register 0. */
#define GPPUDCLK0_BASE_ADDR (GPIO_BASE_ADDR + 0x00000098)

//GPIO: 32-53
/* GPIO Pull-up/down Clock Register 1. */
#define GPPUDCLK1_BASE_ADDR (GPIO_BASE_ADDR + 0x0000009C)

/* PUD - GPIO Pin Pull-up/down */
#define PULL_NONE (0)
#define PULL_DOWN (1)
#define PULL_UP   (2)

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

/* Structure that declares the usual file access functions. */
struct file_operations DHT11_driver_fops =
{
    open    :   gpio_driver_open,
    release :   gpio_driver_release,
    read    :   gpio_driver_read,
    write   :   gpio_driver_write
};

//DHT11 driver function declarations
int DHT11_driver_init(void);
void DHT11_driver_exit(void);
void DHT11_driver_write_to_buffer(char *, size_t);
void DHT11_driver_write_buffer_to_file(struct file* , char *, size_t , loff_t *);


/* Declaration of the init and exit functions. */
module_init(DHT11_driver_init);
module_exit(DHT11_driver_exit);

//Driver major number
int DHT11_device_major_number;

//A data buffer
char DHT11_data_buffer[BUF_LEN][5];
int DHT11_sending_buffer[BUF_LEN];

/* Blink timer vars. */
static struct hrtimer blink_timer;
static ktime_t kt;

#define MS_TO_NS(x) ((x) * 1E6L)
#define TIMER_SEC    0
#define TIMER_NANO_SEC  250*1000*1000 /* 250ms */

char GetGPIOPinOffset(char pin)
{
    if(pin >= 0 && pin <10)
        pin = pin;
    else if(pin >= 10 && pin <20)
        pin -= 10;
    else if(pin >= 20 && pin <30)
        pin -= 20;
    else if(pin >= 30 && pin <40)
        pin -= 30;
    else if(pin >= 40 && pin <50)
        pin -= 40;
    else /*if(pin >= 50 && pin <53) */
        pin -= 50;

    return pin;
}

char GetGPIOPinOffset(char pin)
{
    if(pin >= 0 && pin <10)
        pin = pin;
    else if(pin >= 10 && pin <20)
        pin -= 10;
    else if(pin >= 20 && pin <30)
        pin -= 20;
    else if(pin >= 30 && pin <40)
        pin -= 30;
    else if(pin >= 40 && pin <50)
        pin -= 40;
    else /*if(pin >= 50 && pin <53) */
        pin -= 50;

    return pin;
}

//TODO: Modify the function for it to work on GPIO_04
static enum hrtimer_restart blink_timer_callback(struct hrtimer *param)
{
#ifdef TEST
    static char power = 0x0;

    power ^= 0x1;

    if (power)
        SetGpioPin(GPIO_04);
    else
        ClearGpioPin(GPIO_04);
#endif

    hrtimer_forward(&blink_timer, ktime_get(), kt);

    return HRTIMER_RESTART;
}

static irqreturn_t h_irq_gpio3(int irq, void *data)
{
    static char value = -1;

    printk("Interrupt from IRQ 0x%x\n", irq);

    value = GetGpioPinValue(GPIO_12);

    printk("GPIO_12 level = 0x%x\n", value);

    return IRQ_HANDLED;
}

char GetGpioPinValue(char pin)
{
    void *addr = NULL;
    unsigned int tmp;
    unsigned int mask;

    /* Get base address of gpio level register. */
    addr = (pin < 32) ? (void *) GPLEV0_BASE_ADDR : (void *)GPLEV1_BASE_ADDR;
    pin = (pin < 32) ? pin : pin - 32;

    /* Read gpio pin level. */
    addr = ioremap((unsigned long)addr, 4);
    tmp = ioread32(addr);
    mask = 0x1 << pin;
    tmp &= mask;

    return (tmp >> pin);


void ClearGpioPin(char pin)
{
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


void SetGpioPin(char pin)
{
    void *addr = NULL;
    unsigned int tmp;

    /* Get base address of gpio set register. */
    addr = (pin < 32) ? (void *) GPSET0_BASE_ADDR : (void *)GPSET1_BASE_ADDR;
    pin = (pin < 32) ? pin : pin - 32;

    /* Set gpio. */
    addr = ioremap((unsigned long)addr, 4);
    tmp = 0x1 << pin;
    iowrite32(tmp, addr);
}

void SetInternalPullUpDown(char pin, char value)
{
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

    /* Write to GPPUD to set the required control signal (i.e. Pull-up or Pull-Down or neither
       to remove the current Pull-up/down). */
    addr = ioremap(base_addr_gppud, 4);
    iowrite32(value, addr);

    /* Wait 150 cycles  this provides the required set-up time for the control signal */

    /* Write to GPPUDCLK0/1 to clock the control signal into the GPIO pads you wish to
       modify  NOTE only the pads which receive a clock will be modified, all others will
       retain their previous state. */
    addr = ioremap(base_addr_gppudclk, 4);
    tmp = ioread32(addr);
    mask = 0x1 << pin;
    tmp |= mask;
    iowrite32(tmp, addr);

    /* Wait 150 cycles  this provides the required hold time for the control signal */

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

void SetGpioPinDirection(char pin, char direction)
{
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
    if(direction)
    { //set as output: set 1
      mask = 0x1 << (pin*3);
      tmp |= mask;
    }
    else
    { //set as input: set 0
      mask = ~(0x1 << (pin*3));
      tmp &= mask;
    }
    iowrite32(tmp, addr);
}


//Function to be called before writing data to the data buffer
void SendInitSequence(char pin) {
  /* Send init sequence to GPIO_04 */
  //Init GPIO_04
  //Set GPIO_04 direction to out to send init sequence
  SetGpioPinDirection(pin , GPIO_DIRECTION_OUT);
  //Send init sequence
  //TODO: for 18us ClearGpioPinDirecton(GPIO_04 , PULL_DOWN); for 40us SetGpioPinDirection(GPIO_04 , PULL_UP);
  //After init sequence the device is now sending data to GPIO_04
  //Set GPIO_04 directon ti GPIO_DIRECTION_IN
  SetGpioPinDirection(pin , GPIO_DIRECTION_IN);
  printk(KERN_INFO "Device is now sending data to GPIO_04 PIN\n");
}

void ClearDataBuffer() {
    int i = 0 , j = 0;

    for(i = 0; i < BUF_LEN; i++) {
        for(j = 0; j < 5; j++) {
            DHT11_data_buffer[i][j] = 0;
        }
    }
}

//Functon operations:
// - Clear data buffer
// - Init timer
// - Init GPIO Pins
int DHT11_driver_init(void) {
    int result = -1;

    //Registering the device
    result = register_chardev(0 , "DHT11_driver" , &DHT11_driver_fops);
    if(result < 0) {
        printk(KERN_INFO "DHT11_driver: cannot obtain device major number %d\n" , DHT11_device_major_number);
        return result;
    }
    DHT11_device_major_number = result;
    printk(KERN_INFO "Successfuly obtained device major number\n");

    //Init data buffer
    //TODO: Check if you didnt fuck up
    DHT11_data_buffer = kmallock(BUF_LEN , GFP_KERNEL);
    if(!DHT11_data_buffer) {
      result = -ENOMEM:
      goto fail;
    }

    memset(DHT11_data_buffer , 0 , BUF_LEN);
    printk(KERN_INFO "Successfuly allocated memory for ring buffer\n");

    /* Initialize high resolution timer. */
    hrtimer_init(&blink_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    kt = ktime_set(TIMER_SEC, TIMER_NANO_SEC);
    blink_timer.function = &blink_timer_callback;
    hrtimer_start(&blink_timer, kt, HRTIMER_MODE_REL);

    /* Initialize gpio 3 ISR. */
    gpio_request_one(GPIO_03, GPIOF_IN, "irq_gpio3");
    irq_gpio3 = gpio_to_irq(GPIO_03);
    if(request_irq(irq_gpio3, h_irq_gpio3,
      IRQF_TRIGGER_FALLING, "irq_gpio3", (void *)(h_irq_gpio3))) != 0)
    {
        printk("Error: ISR not registered!\n");
    } 

    //Initialize the device to send data
    SendInitSequence(GPIO_04);
    //Fill the data buffer here
    DHT11_driver_write_to_buffer(GPIO_4);

    return 0;
fail:
    DHT11_driver_exit();
    return result;
}

//Cleanup:
// - Free memory from the buffer
// - Reset timer
// - Clear GPIO pin
void DHT11_driver_exit() {
  //Freeing the major number
  unregister_chardev(DHT11_device_major_number , "DHT11_driver");

  printk(KERN_INFO "Removig DHT11_driver module\n");

  //Release high resolution timer
  hrtimer_cancel(&blink_timer);

  //Clear GPIO pins
  SetInternalPullUpDown(GPIO_04 , PULL_NONE);
}

//A function that fills the data buffer
void DHT11_driver_write_to_buffer(char pin) {
    int i = 0 , j = 0;
    char temp = NULL;

    for(i = 0; i < BUF_LEN; i++) {
        for(j = 0; j < 5; j++) {

          //Now device is sending data
          //TODO: Fill the data buffer acording to the specification
          // 0 - 54us PULL_DOWN & 24us PULL_UP
          // 1 - 54us PULL_DOWN & 70us PULL_UP
          // END - 54us PULL_DOWN & >70US PULL_UP

          //if(GetGpioPinValue(pin) == 0 && time <= 54us)
          // reset temp
          //if(GetGpioPinValue(pin) == 1 && time <= 24)
          //  write 0 to register
          //if(GetGpioPinValue(pin) == 1 && time <= 70)
          //  write 1 to register
          //if(GetGpioPinValue(pin) == 1 && time > 70)
          //  ENDE
          DHT11_data_buffer[i][j] = GetGpioPinValue(GPIO_04);
        }
    }
}

//Fucntion that send buffer data to user space using function copy_to_user
//Asuming the driver is allready filled
void DHT11_driver_write_buffer_to_file(struct file *filp, char *buf, size_t len, loff_t *f_pos) {
    int data_size = 0;
    int i = 0;

    for(i = 0; i < BUF_LEN; i++) {
      DHT11_sending_buffer[i] = DHT11_data_buffer[i][0] & DHT11_data_buffer[i][1]
                  & DHT11_data_buffer[i][2] & DHT11_data_buffer[i][3];
    }

    if(*f_pos == 0) {
        //Getting size of valid data
        data_size = strlen(DHT11_sending_buffer);

        //Send data to user space
        if (copy_to_user(buf, DHT11_sending_buffer, data_size) != 0)  {
            return -EFAULT;
        }
        else
        {
            (*f_pos) += data_size;
            return data_size;
        }
    } else {
        return 0;
    }
}
