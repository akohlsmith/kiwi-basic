#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <stdint.h>
#include <stdbool.h>

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/exti.h>

#include "board.h"
#include "gpsdo.h"
#include "sampler.h"
#include "uxb_slave.h"


Gpsdo gpsdo;
Sampler sampler;


void gpio_setup(void) {

	rcc_osc_on(RCC_HSE);
	rcc_wait_for_osc_ready(RCC_HSE);

	rcc_osc_off(RCC_PLL);
	rcc_wait_for_osc_not_ready(RCC_PLL);
	rcc_set_prediv(RCC_CFGR2_PREDIV_NODIV);
	rcc_set_pll_source(RCC_CFGR_PLLSRC_HSE_PREDIV);
	rcc_set_pll_multiplier(RCC_CFGR_PLLMUL_PLL_IN_CLK_X3);
	rcc_osc_on(RCC_PLL);
	rcc_wait_for_osc_ready(RCC_PLL);
	rcc_set_hpre(RCC_CFGR_HPRE_DIV_NONE);
	rcc_set_ppre2(RCC_CFGR_PPRE1_DIV_2);
	rcc_set_ppre1(RCC_CFGR_PPRE2_DIV_NONE);
	flash_set_ws(FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY_2WS);
	rcc_set_sysclk_source(RCC_CFGR_SW_PLL);
	rcc_wait_for_sysclk_status(RCC_PLL);

	rcc_ahb_frequency  = 60000000;
	rcc_apb1_frequency = 30000000;
	rcc_apb2_frequency = 30000000;

	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOC);
	rcc_periph_clock_enable(RCC_GPIOD);
	rcc_periph_clock_enable(RCC_GPIOE);
	rcc_periph_clock_enable(RCC_USART1);
	rcc_periph_clock_enable(RCC_TIM2);
	rcc_periph_clock_enable(RCC_DAC1);

	nvic_enable_irq(NVIC_TIM1_CC_IRQ);
	nvic_enable_irq(NVIC_TIM2_IRQ);
	nvic_enable_irq(NVIC_TIM3_IRQ);
	nvic_enable_irq(NVIC_ADC1_2_IRQ);

	/* Unused pins. */
	gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN,
		GPIO0 |
		GPIO2 |
		GPIO6 |
		GPIO11 |
		GPIO12
	);
	gpio_mode_setup(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN,
		GPIO0 |
		GPIO1 |
		GPIO2 |
		GPIO6 |
		GPIO7 |
		GPIO8 |
		GPIO10 |
		GPIO11 |
		GPIO12 |
		GPIO13 |
		GPIO14 |
		GPIO15
	);
	gpio_mode_setup(GPIOC, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN,
		GPIO0 |
		GPIO1 |
		GPIO2 |
		GPIO3 |
		GPIO4 |
		GPIO5 |
		GPIO6 |
		GPIO7 |
		GPIO8 |
		GPIO13
	);
	gpio_mode_setup(GPIOD, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN,
		GPIO0 |
		GPIO1 |
		GPIO2 |
		GPIO3 |
		GPIO4 |
		GPIO5 |
		GPIO6 |
		GPIO7 |
		GPIO11 |
		GPIO12 |
		GPIO13 |
		GPIO14 |
		GPIO15
	);
	gpio_mode_setup(GPIOE, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN,
		GPIO0 |
		GPIO1 |
		GPIO3 |
		GPIO4 |
		GPIO7 |
		GPIO8 |
		GPIO10 |
		GPIO11 |
		GPIO12 |
		GPIO13 |
		GPIO14
	);

	// gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO5);
	gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO3);

	/* Timer 2, IC2 */
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO1);
	gpio_set_af(GPIOA, GPIO_AF1, GPIO1);

	/* USART2 */
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9 | GPIO10);
	gpio_set_af(GPIOA, GPIO_AF7, GPIO9 | GPIO10);

	/* ADC1, channel 4, no filter, fast channel. */
	gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO3);

	/* ADC2, channel 4, no filter, fast channel. */
	gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO7);

	/* ADC3, channel 2, no filter, fast channel. */
	gpio_mode_setup(GPIOE, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO9);

	/* ADC4, channel 2, no filter, fast channel. */
	gpio_mode_setup(GPIOE, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO15);

	/* VCTCXO steering, DAC output. */
	gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO4);



}



/* Debug output over USART1. */
void usart_setup(void) {
	usart_set_baudrate(USART1, 460800);
	usart_set_databits(USART1, 8);
	usart_set_stopbits(USART1, USART_STOPBITS_1);
	usart_set_mode(USART1, USART_MODE_TX_RX);
	usart_set_parity(USART1, USART_PARITY_NONE);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
	usart_enable(USART1);
}


int _write(int file, char *ptr, int len) {
	int i;

	if (file == STDOUT_FILENO || file == STDERR_FILENO) {
		for (i = 0; i < len; i++) {
			if (ptr[i] == '\n') {
				usart_send_blocking(USART1, '\r');
			}
			usart_send_blocking(USART1, ptr[i]);
		}
		return i;
	}
	errno = EIO;
	return -1;
}



void tim1_cc_isr(void) {
	if (TIM_SR(TIM1) & TIM_SR_CC1IF) {
		sampler_stop(&sampler);
		sampler_save_buffer(&sampler);
		/* Do not start again, otherwise we overwrite old data.
		 * There is no buffer management yet. */
	}
}


void tim2_isr(void) {
	if (TIM_SR(TIM2) & TIM_SR_CC2IF) {
		timer_clear_flag(TIM2, TIM_SR_CC2IF);

		gpsdo_1pps_irq_handler(&gpsdo);

		// printf("%u ticks=%u out=%d error=%d phase_error=%d pps_setpoint=%u\n", timer_get_counter(TIM2), gpsdo.pps_time_ticks, gpsdo.pps_steer, (int32_t)gpsdo.pps_error, (int32_t)gpsdo.phase_error, gpsdo.pps_setpoint);
	}
	if (TIM_SR(TIM2) & TIM_SR_CC3IF) {
		timer_clear_flag(TIM2, TIM_SR_CC3IF);

		gpsdo_housekeeping_irq_handler(&gpsdo);
		struct gpsdo_sync_stats stats;
		gpsdo_get_sync_status(&gpsdo, &stats);

		/*
		printf("gpsdo sync stats:\n");

		printf("  phase error: %d ns\n", stats.phase_error);
		printf("  period error: %d ns\n", stats.period_error);

		const char *status_str = "?";
		switch (stats.status) {
			case GPSDO_SYNC_STATUS_UNKNOWN: status_str = "UNKNOWN"; break;
			case GPSDO_SYNC_STATUS_ADJUSTING: status_str = "ADJUSTING"; break;
			case GPSDO_SYNC_OK: status_str = "OK"; break;
		}
		printf("  sync status: %s\n", status_str);
		*/

	}

}

/*
void tim3_isr(void) {
	if (TIM_SR(TIM3) & TIM_SR_UIF) {
		timer_clear_flag(TIM3, TIM_SR_UIF);

		adc_start_conversion_regular(ADC1);
		gpio_set(GPIOC, GPIO3);
		while (!adc_eoc(ADC1)) {
			;
		}
		gpio_clear(GPIOC, GPIO3);


	}
}
*/

void dma1_channel1_isr(void) {
	if (dma_get_interrupt_flag(DMA1, DMA_CHANNEL1, DMA_TCIF)) {
		dma_clear_interrupt_flags(DMA1, DMA_CHANNEL1, DMA_TCIF);
		sampler_dma_completed_handler(&sampler);
	}
}
