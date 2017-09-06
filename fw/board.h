#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <libopencm3/stm32/gpio.h>


extern uint32_t samples;

void gpio_setup(void);
void usart_setup(void);
void print_buffer(void);
void timer_sync_start(void);
void dma_setup(void);
