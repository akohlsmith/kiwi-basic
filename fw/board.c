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
#include "uxb_locm3.h"

uint32_t ccr_old = 0;

#define ADC_BUFFER_SIZE 4096
uint16_t adc_buffer[ADC_BUFFER_SIZE];

uint32_t samples = 0;

Gpsdo gpsdo;
UxbMasterLocm3 uxb;
UxbInterface iface1;
UxbSlot slot1;
UxbSlot descriptor_slot;
uint8_t slot1_buffer[1024];

const char *uxb_descriptor[] = {
	"device=kiwi-basic",
	"hw-version=1.0.0+20170723",
	"fw-version=1.0.0",
	"cspeed=4",
	"dspeed=4",
	"slot=1,bootloader,1.0.0",
	"slot=2,rtc,1.0.0",
	"slot=3,hrtime,1.0.0",
	"slot=4,waveform,1.0.0",
	NULL,
};

uint8_t descriptor_slot_buffer[64];

static void delay_simple(uint32_t d) {
	for (uint32_t i = 0; i < d; i++) {
		__asm__("nop");
	}
}


static uxb_master_locm3_ret_t uxb_read_descriptor(void *context, uint8_t *buf, size_t len) {
	uint8_t zero = 0;

	if (len != 1) {
		return UXB_MASTER_LOCM3_RET_FAILED;
	}
	if (buf[0] == 0) {
		/* Send the 0 back. */
		uxb_slot_send_data(&descriptor_slot, &zero, 1, true);
	} else {
		uint8_t descriptor_index = buf[0] - 1;
		if (uxb_descriptor[descriptor_index] == NULL) {
			uxb_slot_send_data(&descriptor_slot, &zero, 1, true);
		} else {
			uxb_slot_send_data(&descriptor_slot, uxb_descriptor[descriptor_index], strlen(uxb_descriptor[descriptor_index]) + 1, true);
		}
	}

	return UXB_MASTER_LOCM3_RET_OK;
}


static uxb_master_locm3_ret_t uxb_data_received(void *context, uint8_t *buf, size_t len) {
	// usart_send_blocking(USART1, 'A');
	uxb_slot_send_data(&slot1, (uint8_t *)adc_buffer, 1024, true);
	return UXB_MASTER_LOCM3_RET_OK;
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
	rcc_periph_clock_enable(RCC_DAC1);

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

	/* VCTCXO steering, DAC output. */
	gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO4);

	/* Initialize the UXB bus. */
	rcc_periph_clock_enable(RCC_SPI1);
	rcc_periph_clock_enable(RCC_TIM7);

	/* Setup a timer for precise UXB protocol delays. */
	timer_reset(TIM7);
	timer_set_mode(TIM7, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
	timer_continuous_mode(TIM7);
	timer_direction_up(TIM7);
	timer_disable_preload(TIM7);
	timer_enable_update_event(TIM7);
	timer_set_prescaler(TIM7, (rcc_ahb_frequency / 1000000) - 1);
	timer_set_period(TIM7, 65535);
	timer_enable_counter(TIM7);

	uxb_master_locm3_init(&uxb, &(struct uxb_master_locm3_config) {
		.spi_port = SPI1,
		.spi_af = GPIO_AF5,
		.sck_port = GPIOB, .sck_pin = GPIO3,
		.miso_port = GPIOB, .miso_pin = GPIO4,
		.mosi_port = GPIOB, .mosi_pin = GPIO5,
		.frame_port = GPIOA, .frame_pin = GPIO15,
		.id_port = GPIOC, .id_pin = GPIO10,
		.delay_timer = TIM7,
		.delay_timer_freq_mhz = 1,
		.control_prescaler = SPI_CR1_BR_FPCLK_DIV_16,
		.data_prescaler = SPI_CR1_BR_FPCLK_DIV_16,
	});

	uxb_interface_init(&iface1);
	uxb_interface_set_address(
		&iface1,
		(uint8_t[]){0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04},
		(uint8_t[]){0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
	);
	uxb_master_locm3_add_interface(&uxb, &iface1);

	uxb_slot_init(&slot1);
	uxb_slot_set_slot_number(&slot1, 5);
	uxb_slot_set_slot_buffer(&slot1, slot1_buffer, 1024);
	uxb_slot_set_data_received(&slot1, uxb_data_received, NULL);
	uxb_interface_add_slot(&iface1, &slot1);

	uxb_slot_init(&descriptor_slot);
	uxb_slot_set_slot_number(&descriptor_slot, 0);
	uxb_slot_set_slot_buffer(&descriptor_slot, descriptor_slot_buffer, sizeof(descriptor_slot_buffer));
	uxb_slot_set_data_received(&descriptor_slot, uxb_read_descriptor, NULL);
	uxb_interface_add_slot(&iface1, &descriptor_slot);

	/* Setup uxb exti interrupts. */
	nvic_enable_irq(NVIC_EXTI15_10_IRQ);
	rcc_periph_clock_enable(RCC_SYSCFG);
	exti_select_source(EXTI15, GPIOA);
	exti_set_trigger(EXTI15, EXTI_TRIGGER_FALLING);
	exti_enable_request(EXTI15);

}


void exti15_10_isr(void) {
	exti_reset_request(EXTI15);

	exti_disable_request(EXTI15);
	uxb_master_locm3_frame_irq(&uxb);
	exti_enable_request(EXTI15);


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


void timer_sync_start(void) {
	uint32_t current = timer_get_counter(TIM2);
	uint32_t next = (current / ADC_BUFFER_SIZE / 30) * 30 * ADC_BUFFER_SIZE + 2 * 30 * ADC_BUFFER_SIZE;
	printf("current=%d next=%d\n", current, next);
	gpsdo_sync_start(&gpsdo, next);
	// timer_set_oc_value(TIM2, TIM_OC1, next);


}


void timer_setup(void) {

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

	timer_sync_start();





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

	adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_4DOT5CYC);
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
	ADC1_TR1 = (uint32_t)((3000 << 16) | 1000);
	ADC1_CFGR |= ADC_CFGR_AWD1EN;
	// ADC1_IER |= ADC_IER_AWD1IE;


	dma_setup();

	adc_power_on(ADC1);
	ADC1_CR |= ADC_CR_ADSTART;


	timer_enable_counter(TIM2);

}


void print_buffer(void) {
	uint32_t pos = ADC_BUFFER_SIZE - DMA_CNDTR(DMA1, DMA_CHANNEL1);

	printf("buffer=");

	for (size_t i = pos + 1; i < ADC_BUFFER_SIZE; i++) {
		printf("%04x", adc_buffer[i]);
	}
	for (size_t i = 0; i < pos; i++) {
		printf("%04x", adc_buffer[i]);
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

		gpsdo_1pps_irq_handler(&gpsdo);

		// printf("%u ticks=%u out=%d error=%d phase_error=%d pps_setpoint=%u\n", timer_get_counter(TIM2), gpsdo.pps_time_ticks, gpsdo.pps_steer, (int32_t)gpsdo.pps_error, (int32_t)gpsdo.phase_error, gpsdo.pps_setpoint);
	}
	if (TIM_SR(TIM2) & TIM_SR_CC3IF) {
		timer_clear_flag(TIM2, TIM_SR_CC3IF);

		gpsdo_housekeeping_irq_handler(&gpsdo);
		struct gpsdo_sync_stats stats;
		gpsdo_get_sync_status(&gpsdo, &stats);
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
