#include "gpio.h"
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

static volatile uint32_t *gpio_base = NULL;
static int fd = -1;

int gpio_init()
{
    uint32_t gpio_phy_addr = GPIO_BASE - BUS_REG_BASE + PHYS_REG_BASE;

    fd = open("/dev/mem", O_RDWR | O_SYNC | __O_CLOEXEC);
    if (fd < 0)
    {
        perror("Failed to open /dev/mem");
        return -1;
    }

    gpio_base = (volatile uint32_t *)mmap(NULL, 0x1000, PROT_READ | PROT_WRITE, MAP_SHARED, fd, gpio_phy_addr);
    if (gpio_base == MAP_FAILED)
    {
        perror("mmap");
        close(fd);
        return -1;
    }
    return 0;
}

void gpio_close()
{
    if (gpio_base)
    {
        munmap((void *)gpio_base, 0x1000);
        gpio_base = NULL;
    }
    if (fd >= 0)
    {
        close(fd);
        fd = -1;
    }
}

void gpio_set_mode(int gpio, int mode) {
    int reg = gpio / 10;
    int shift = (gpio % 10) * 3;

    volatile uint32_t *gpfsel = &gpio_base[reg];
    uint32_t val = *gpfsel;
    val &= ~(0b111 << shift);
    val |= (mode << shift);
    *gpfsel = val;
}

void gpio_write(int gpio, int value) {
    if (value)
        gpio_base[GPIO_SET0 / 4] = (1 << gpio);
    else
        gpio_base[GPIO_CLR0 / 4] = (1 << gpio);
}

int gpio_read(int gpio) {
    return (gpio_base[GPIO_LEV0 / 4] & (1 << gpio)) ? 1 : 0;
}