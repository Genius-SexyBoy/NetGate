


#include "bsp_gpio.h"
#include "esp_err.h"

void bsp_gpio_init(gpio_num_t set_gpio_pin)
{
  gpio_config_t gpio_conf;
  gpio_conf.pin_bit_mask = (1ULL<<set_gpio_pin);
  gpio_conf.mode = GPIO_MODE_DEF_OUTPUT;
  gpio_conf.pull_up_en = GPIO_PULLUP_ENABLE;
  gpio_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  gpio_conf.intr_type = GPIO_INTR_DISABLE;
  
  gpio_set_level(GPIO_NUM_12, 1);
  ESP_ERROR_CHECK(gpio_config(&gpio_conf));
}