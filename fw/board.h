#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <libopencm3/stm32/gpio.h>

#include "gpsdo.h"
#include "sampler.h"


extern uint32_t samples;
extern Gpsdo gpsdo;
extern Sampler sampler;

void gpio_setup(void);
void usart_setup(void);
void timer_sync_start(void);
