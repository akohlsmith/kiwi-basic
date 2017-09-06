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

#include "board.h"

uint32_t ccr_old = 0;

#define ADC_BUFFER_SIZE 4096
uint16_t adc_buffer[ADC_BUFFER_SIZE];

uint32_t samples = 0;

static void delay_simple(uint32_t d) {
	for (uint32_t i = 0; i < d; i++) {
		__asm__("nop");
	}
}


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
	rcc_periph_clock_enable(RCC_USART1);
	rcc_periph_clock_enable(RCC_TIM2);
	rcc_periph_clock_enable(RCC_TIM3);
	rcc_periph_clock_enable(RCC_DMA1);
	rcc_periph_clock_enable(RCC_TIM1);
	rcc_periph_clock_enable(RCC_TIM8);

	nvic_enable_irq(NVIC_TIM1_CC_IRQ);
	nvic_enable_irq(NVIC_TIM2_IRQ);
	nvic_enable_irq(NVIC_TIM3_IRQ);
	nvic_enable_irq(NVIC_ADC1_2_IRQ);
	nvic_enable_irq(NVIC_DMA1_CHANNEL1_IRQ);

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

}


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


void dma_setup(void) {

	/* DMA1 configuration, channel 1 for ADC1. */
	dma_set_priority(DMA1, DMA_CHANNEL1, DMA_CCR_PL_VERY_HIGH);

	dma_set_memory_size(DMA1, DMA_CHANNEL1, DMA_CCR_MSIZE_16BIT);
	dma_set_peripheral_size(DMA1, DMA_CHANNEL1, DMA_CCR_PSIZE_16BIT);

	dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL1);
	dma_disable_peripheral_increment_mode(DMA1, DMA_CHANNEL1);

 	dma_set_read_from_peripheral(DMA1, DMA_CHANNEL1);

 	dma_set_peripheral_address(DMA1, DMA_CHANNEL1, (uint32_t)(&ADC1_DR));
	dma_set_memory_address(DMA1, DMA_CHANNEL1, (uint32_t)adc_buffer);

	dma_set_number_of_data(DMA1, DMA_CHANNEL1, ADC_BUFFER_SIZE);
	dma_enable_circular_mode(DMA1, DMA_CHANNEL1);

	dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL1);
	dma_enable_channel(DMA1, DMA_CHANNEL1);

}


void timer_setup(void) {
	timer_reset(TIM2);
	timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
	timer_continuous_mode(TIM2);
	timer_direction_up(TIM2);
	timer_enable_break_main_output(TIM2);
	timer_enable_update_event(TIM2);
	timer_disable_preload(TIM2);
	timer_set_prescaler(TIM2, 0);
	timer_set_period(TIM2, ~(uint32_t)0);

	/* GPS 1pps input used to measure the pulse period and adjust the VC/TCXO. */
	timer_ic_set_input(TIM2, TIM_IC2, TIM_IC_IN_TI2);
	timer_ic_set_filter(TIM2, TIM_IC2, TIM_IC_OFF);
	/* Detecting 1PPS on the rising edge. */
	timer_ic_set_polarity(TIM2, TIM_IC2, TIM_IC_RISING);
	timer_ic_set_prescaler(TIM2, TIM_IC2, TIM_IC_PSC_OFF);
	timer_ic_enable(TIM2, TIM_IC2);

	/* CC2 irq to print 1PPS pulse period and adjust the VCTCXO. */
	timer_enable_irq(TIM2, TIM_DIER_CC2IE);



	timer_disable_oc_preload(TIM2, TIM_OC1);
	timer_set_oc_mode(TIM2, TIM_OC1, TIM_OCM_PWM2);
	timer_set_oc_value(TIM2, TIM_OC1, 1);
	/* Not needed for synchronization. */
	// timer_enable_oc_output(TIM2, TIM_OC1);

	/* Synchronize TIM3 to a exact time. There is a 2 clock cycles delay between
	 * OC3REF and TIM3 start. */
	timer_set_master_mode(TIM2, TIM_CR2_MMS_COMPARE_OC1REF);

	/* Initialize the TIM3, but does not enable it. */
	timer_reset(TIM3);
	timer_set_mode(TIM3, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
	timer_continuous_mode(TIM3);
	timer_direction_up(TIM3);
	timer_disable_preload(TIM3);
	timer_enable_update_event(TIM3);
	timer_set_prescaler(TIM3, 0);
	timer_set_period(TIM3, 29); /* 14 */
	// timer_enable_irq(TIM3, TIM_DIER_UIE);

	/* Configure it as a slave instead and wait for TIM2. */
	timer_slave_set_polarity(TIM3, TIM_ET_RISING);
	timer_slave_set_trigger(TIM3, TIM_SMCR_TS_ITR1);
	timer_slave_set_mode(TIM3, TIM_SMCR_SMS_TM);

	/* Set update event as a TRGO output on TIM3 - this will be
	 * the trigger used to do ADC conversions. */
	timer_set_master_mode(TIM3, TIM_CR2_MMS_UPDATE);






	/* Configure timer 1 and timer 8 to receive events from the analog watchdogs. */
	timer_reset(TIM1);
	timer_set_mode(TIM1, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
	timer_continuous_mode(TIM1);
	timer_direction_up(TIM1);
	timer_disable_preload(TIM1);
	timer_enable_update_event(TIM1);
	timer_set_prescaler(TIM1, 0);
	timer_set_period(TIM1, 65535);
	/* Do not enable the timer, configure enable slave mode instead. */
	timer_slave_set_polarity(TIM1, TIM_ET_RISING);
	TIM1_SMCR |= TIM_SMCR_ETF_DTS_DIV_4_N_6;
	timer_slave_set_trigger(TIM1, TIM_SMCR_TS_ETRF);
	timer_slave_set_mode(TIM1, TIM_SMCR_SMS_TM);
	TIM1_OR |= TIM1_ETR_ADC1_RMP_AWD1;
	// timer_enable_counter(TIM1);

	timer_disable_oc_preload(TIM1, TIM_OC1);
	timer_set_oc_value(TIM1, TIM_OC1, 30 * 2048 - 1);
	timer_enable_irq(TIM1, TIM_DIER_CC1IE);




	/* ADC configuration. */
	rcc_periph_clock_enable(RCC_ADC12);

	adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_1DOT5CYC);
	uint8_t channels[16] = {4};
	adc_set_regular_sequence(ADC1, 1, channels);
	/* Do not use ADc interrupt, use DMA instead. */
	// ADC1_IER |= ADC_IER_EOSIE;
	ADC12_CCR |= ADC_CCR_CKMODE_DIV1;
	ADC1_CFGR |= ADC_CFGR_EXTEN_RISING_EDGE;
	ADC1_CFGR |= ADC_CFGR_EXTSEL_EXT4;

	/* Enable DMA circular mode. */
	ADC1_CFGR |= ADC_CFGR_DMACFG | ADC_CFGR_DMAEN;

	/* Configure the analog watchdog. */
	ADC1_TR1 = (uint32_t)((2100 << 16) | 1800);
	ADC1_CFGR |= ADC_CFGR_AWD1EN;
	// ADC1_IER |= ADC_IER_AWD1IE;


	dma_setup();

	adc_power_on(ADC1);
	ADC1_CR |= ADC_CR_ADSTART;


	timer_enable_counter(TIM2);

}


void timer_sync_start(void) {
	uint32_t current = timer_get_counter(TIM2);
	uint32_t next = (current / ADC_BUFFER_SIZE / 30) * 30 * ADC_BUFFER_SIZE + 2 * 30 * ADC_BUFFER_SIZE;
	printf("current=%d next=%d\n", current, next);
	timer_set_oc_value(TIM2, TIM_OC1, next);


}

void print_buffer(void) {
	uint32_t pos = ADC_BUFFER_SIZE - DMA_CNDTR(DMA1, DMA_CHANNEL1);

	printf("buffer \n");

	for (size_t i = pos + 1; i < ADC_BUFFER_SIZE; i++) {
		printf("%d ", adc_buffer[i]);
	}
	for (size_t i = 0; i < pos; i++) {
		printf("%d ", adc_buffer[i]);
	}
	printf("\n");

}

void tim1_cc_isr(void) {
	if (TIM_SR(TIM1) & TIM_SR_CC1IF) {
		/* Disable triggering of the ADC. */
		timer_disable_counter(TIM3);

		timer_disable_irq(TIM1, TIM_DIER_CC1IE);
		timer_clear_flag(TIM1, TIM_SR_CC1IF);
		timer_disable_counter(TIM1);

		print_buffer();

		timer_set_counter(TIM1, 0);
		timer_enable_irq(TIM1, TIM_DIER_CC1IE);

		dma_setup();
		timer_sync_start();
	}
}


void tim2_isr(void) {
	if (TIM_SR(TIM2) & TIM_SR_CC2IF) {
		timer_clear_flag(TIM2, TIM_SR_CC2IF);
		uint32_t ccr_new = TIM_CCR2(TIM2);
		if (ccr_old == 0) {
			ccr_old = ccr_new;
			return;
		}

		uint32_t ccr_sample = ccr_new - ccr_old;
		ccr_old = ccr_new;

		printf("%u %u sps = %u\n", (unsigned int)ccr_new, (unsigned int)ccr_sample, (unsigned int)samples);
		samples = 0;
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

void adc1_2_isr(void) {

	// printf("%08x\n", ADC1_ISR);
	if (ADC1_ISR & ADC_ISR_AWD1) {
		ADC1_ISR |= ADC_ISR_AWD1;
		printf("awd %d\n", ADC1_DR);
	}

	// if (ADC1_ISR & ADC_ISR_EOS) {
		// ADC1_ISR |= ADC_ISR_EOS;
		// printf("%u\n", (unsigned int)adc_read_regular(ADC1));
	// }

}


void dma1_channel1_isr(void) {
	if (dma_get_interrupt_flag(DMA1, DMA_CHANNEL1, DMA_TCIF)) {
		dma_clear_interrupt_flags(DMA1, DMA_CHANNEL1, DMA_TCIF);

		// printf("buffer = ");
		// for (size_t i = 0; i < ADC_BUFFER_SIZE; i++) {
			// if (adc_buffer[i] > 3000) {
				// printf("t\n");
				// break;
			// }
			// printf("%d ", adc_buffer[i]);
		// }
		samples += ADC_BUFFER_SIZE;
		// printf("\n");
	}

}
