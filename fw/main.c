#include "stdio.h"
#include <stdint.h>
#include <stdbool.h>


#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/usart.h>
#include "board.h"


static void delay_simple(uint32_t d) {
	for (uint32_t i = 0; i < d; i++) {
		__asm__("nop");
	}
}


int main(void) {
	gpio_setup();
	usart_setup();
	timer_setup();

	printf("som tu\n");

	delay_simple(1000000);
	while (true) {

		uint16_t c = usart_recv_blocking(USART1);

		if (c == 't') {
			// gpio_set(GPIOA, GPIO0);
			// gpio_set(GPIOC, GPIO3);
			// delay_simple(40);
			// gpio_clear(GPIOA, GPIO0);
			// gpio_clear(GPIOC, GPIO3);
			// delay_simple(2000000);

			uint32_t cnt1 = TIM1_CNT;
			uint32_t cnt2 = TIM2_CNT;
			uint32_t cnt3 = TIM3_CNT;

			/* Re-sync. */
			// timer_disable_counter(TIM3);
			// timer_set_oc_value(TIM2, TIM_OC1, cnt2 + 1000);

			printf("tim2=%u tim3=%u tim1=%u samples=%u\n", cnt2, cnt3, cnt1, samples);
			// ADC1_IER |= ADC_IER_AWD2IE;

			// for (size_t i = 0; i < ADC_BUFFER_SIZE; i++) {
				// if (adc_buffer[i] > 3000) {
					// printf("t\n");
					// break;
				// }
				// printf("%d ", adc_buffer[i]);
			// }

		}

		if (c == 's') {
			print_buffer();

			timer_set_counter(TIM1, 0);
			timer_enable_irq(TIM1, TIM_DIER_CC1IE);

			dma_setup();
			timer_sync_start();
		}


	}

	return 0;
}
