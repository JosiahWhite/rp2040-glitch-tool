#pragma once
#include <stdint.h>
#define __not_in_flash_func(name) name
uint64_t time_us_64(void);
#define NUM_BANK0_GPIOS 30
void gpio_init(unsigned pin);
void gpio_disable_pulls(unsigned pin);
