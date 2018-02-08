#include "stdio.h"
#include <stdint.h>
#include <stdbool.h>


#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/dac.h>
#include <libopencm3/stm32/dma.h>
#include "board.h"


static void delay_simple(uint32_t d) {
	for (uint32_t i = 0; i < d; i++) {
		__asm__("nop");
	}
}


int main(void) {
	gpio_setup();
	usart_setup();

	/* Setup the GPSDO. */
	gpsdo_init(&gpsdo, TIM2, TIM_IC2, 60000000, CHANNEL_1);
	gpsdo_sync_enable(&gpsdo, TIM_OC1);
	gpsdo_housekeeping_enable(&gpsdo, TIM_OC3);


	delay_simple(1000000);

	sampler_init(&sampler, 60000000, 2000000);
	sampler_start(&sampler);

	while (true) {

		uint16_t c = usart_recv_blocking(USART1);

		if (c == 't') {
			uint32_t cnt1 = TIM1_CNT;
			uint32_t cnt2 = TIM2_CNT;
			uint32_t cnt3 = TIM3_CNT;
			uint32_t pos = ADC_BUFFER_SIZE - DMA_CNDTR(DMA1, DMA_CHANNEL1);

			// printf("tim2 mod 30 = %lu tim3 = %lu diff = %ld\n", cnt2 % 30, cnt3, (int32_t)(cnt2 % 30 - cnt3));
			printf("tim2 div 30 mod SIZE = %lu DMA pos = %lu diff = %ld\n", (cnt2 / 30) % ADC_BUFFER_SIZE, pos, (int32_t)((cnt2 / 30) % ADC_BUFFER_SIZE - pos));
			printf("DMA_ISR = %08x\n", DMA_ISR(DMA1));
			printf("ADC_ISR = %08x\n", ADC_ISR(ADC1));

		}

		if (c == 's') {
			sampler_stop(&sampler);
			sampler_print_buffer(&sampler);
			sampler_start(&sampler);
		}

		if (c == 'b') {

			for (size_t i = 0; i < 4; i++) {
				uint32_t now = TIM2_CNT;
				uint32_t start = sampler.buf[i].buffer_start_time;
				uint32_t pos = ADC_BUFFER_SIZE - DMA_CNDTR(DMA1, DMA_CHANNEL1);
				printf("now = %lu buf[%u].start_time = %lu pos = %lu dma pos = %lu\n", now, i, start, (now - start) / 30, pos);
			}

		}

	}

	return 0;
}
