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
#include "sampler.h"

Gpsdo gpsdo;
Sampler sampler;

/** @todo sampler, move */
uint32_t ccr_old = 0;

/** @todo UXB related, move */
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


/** @todo UXB related, move */
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


/** @todo UXB related, move */
static uxb_master_locm3_ret_t uxb_data_received(void *context, uint8_t *buf, size_t len) {
	// usart_send_blocking(USART1, 'A');
	uxb_slot_send_data(&slot1, (uint8_t *)&sampler.buf[0].adc_buffer, 1024, true);
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
	rcc_periph_clock_enable(RCC_GPIOD);
	rcc_periph_clock_enable(RCC_GPIOE);
	rcc_periph_clock_enable(RCC_USART1);
	rcc_periph_clock_enable(RCC_TIM2);
	rcc_periph_clock_enable(RCC_DAC1);

	nvic_enable_irq(NVIC_TIM1_CC_IRQ);
	nvic_enable_irq(NVIC_TIM2_IRQ);
	nvic_enable_irq(NVIC_TIM3_IRQ);
	nvic_enable_irq(NVIC_ADC1_2_IRQ);

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
		sampler_print_buffer(&sampler);
		sampler_start(&sampler);
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

void adc1_2_isr(void) {

	/* No ADC watchdog interrupt, not needed. */
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
		sampler_dma_completed_handler(&sampler, 0);
	}
}

void dma2_channel1_isr(void) {
	if (dma_get_interrupt_flag(DMA2, DMA_CHANNEL1, DMA_TCIF)) {
		dma_clear_interrupt_flags(DMA2, DMA_CHANNEL1, DMA_TCIF);
		sampler_dma_completed_handler(&sampler, 1);
	}
}

void dma2_channel5_isr(void) {
	if (dma_get_interrupt_flag(DMA2, DMA_CHANNEL5, DMA_TCIF)) {
		dma_clear_interrupt_flags(DMA2, DMA_CHANNEL5, DMA_TCIF);
		sampler_dma_completed_handler(&sampler, 2);
	}
}

void dma2_channel2_isr(void) {
	if (dma_get_interrupt_flag(DMA2, DMA_CHANNEL2, DMA_TCIF)) {
		dma_clear_interrupt_flags(DMA2, DMA_CHANNEL2, DMA_TCIF);
		sampler_dma_completed_handler(&sampler, 3);
	}
}
