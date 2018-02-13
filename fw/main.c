#include "stdio.h"
#include <stdint.h>
#include <stdbool.h>

#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/dac.h>
#include <libopencm3/stm32/dma.h>

#include "board.h"
#include "uxb_slave.h"
#include "gpsdo.h"
#include "sampler.h"


static void delay_simple(uint32_t d) {
	for (uint32_t i = 0; i < d; i++) {
		__asm__("nop");
	}
}


int main(void) {
	gpio_setup();
	usart_setup();
	uxb_slave_init();

	/* Setup the GPSDO. */
	gpsdo_init(&gpsdo, TIM2, TIM_IC2, 60000000, CHANNEL_1);
	gpsdo_sync_enable(&gpsdo, TIM_OC1);
	gpsdo_housekeeping_enable(&gpsdo, TIM_OC3);

	delay_simple(1000000);

	sampler_init(
		&sampler,
		60000000,    /* Base frequency of the system and timers. */
		2000000,     /* Desired sampling frequency. */
		3072,        /* Delay before stopping after the trigger has occurred. */
		500          /* Trigger signal value (lower or higher than 0). */
	);
	sampler_start(&sampler);

	/* For debugging purposes only. */
	while (true) {

		uint16_t c = usart_recv_blocking(USART1);

		if (c == 's') {
			sampler_stop(&sampler);
			sampler_print_buffer(&sampler);
			sampler_save_buffer(&sampler);
			sampler_start(&sampler);
		}

		if (c == 'b') {

			for (size_t i = 0; i < 4; i++) {
				uint32_t now = TIM2_CNT;
				uint32_t start = sampler.buffer_second_time;
				uint32_t pos = ADC_BUFFER_SIZE - DMA_CNDTR(DMA1, DMA_CHANNEL1);
				printf("now = %lu buf[%u].second_time = %lu pos = %lu dma pos = %lu\n", now, i, start, (now - start) / 30, pos);
			}

		}

	}

	return 0;
}
