#pragma once
#include <cstdint>
using gpio_num_t = int;
enum { GPIO_NUM_1=1, GPIO_NUM_2=2, GPIO_NUM_15=15, GPIO_NUM_16=16,
       GPIO_NUM_18=18, GPIO_NUM_40=40, GPIO_NUM_41=41, GPIO_NUM_42=42,
       GPIO_MODE_INPUT=1, GPIO_INTR_DISABLE=0 };
struct gpio_config_t { uint64_t pin_bit_mask=0; int mode=0, intr_type=0; };
int gpio_config(const gpio_config_t*);
int gpio_get_level(gpio_num_t);
