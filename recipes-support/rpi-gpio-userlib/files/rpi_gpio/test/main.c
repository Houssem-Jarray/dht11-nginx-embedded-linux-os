#include <stdio.h>
#include <unistd.h>
#include "gpio.h"

int main() {
    if (gpio_init() != 0) return 1;

    int pin = 27;

    gpio_set_mode(pin, GPIO_OUTPUT);
    gpio_write(pin, 0);
    sleep(1);
    gpio_write(pin, 1);
    sleep(1);
    gpio_write(pin, 0);

    gpio_close();
    return 0;
}
