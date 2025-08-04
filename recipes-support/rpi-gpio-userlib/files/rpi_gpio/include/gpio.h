#ifndef GPIO_H
#define GPIO_H

#include <stdint.h>

// start address of the I/O peripheral register space on the VideoCore bus
#define BUS_REG_BASE 0x7E000000
// start address of the I/O peripheral register space seen from the CPU bus
#define PHYS_REG_BASE 0xFE000000
// start address of the GPIO register space on the VideoCore bus
#define GPIO_BASE 0x7E200000

// address offsets for the individual registers
#define GPIO_FSEL0 0x00 // mode selection GPIO 0-9
#define GPIO_FSEL1 0x04 // mode selection GPIO 10-19
#define GPIO_FSEL2 0x08 // mode selection GPIO 20-29
#define GPIO_SET0 0x1C  // set outputs to '1' GPIO 0-31
#define GPIO_CLR0 0x28  // set outputs to '0' GPIO 0-31
#define GPIO_LEV0 0x34  // get input states GPIO 0-31

// GPIO modes
#define GPIO_INPUT 0b000
#define GPIO_OUTPUT 0b001

// Function prototypes to initialize GPIO
int gpio_init();
void gpio_close();
void gpio_set_mode(int gpio, int mode);
void gpio_write(int gpio, int value);
int gpio_read(int gpio);

#endif // GPIO_H
