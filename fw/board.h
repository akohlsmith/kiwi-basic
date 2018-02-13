#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <libopencm3/stm32/gpio.h>

#include "gpsdo.h"
#include "sampler.h"
#include "uxb_waveform.h"


#define FIRMWARE_VERSION "1.0.0-a.1"
#define HARDWARE_VERSION "1.0.0+20170723"

extern uint32_t samples;
extern Gpsdo gpsdo;
extern Sampler sampler;

void gpio_setup(void);
void usart_setup(void);
void timer_sync_start(void);
