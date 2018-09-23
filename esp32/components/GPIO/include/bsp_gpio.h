#ifndef _BSP_GPIO_H_
#define _BSP_GPIO_H_
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

void bsp_gpio_init(gpio_num_t set_gpio_pin);

#ifdef __cplusplus
}
#endif

#endif