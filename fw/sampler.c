/*
 * Copyright (c) 2017, Marek Koza (qyx@krtko.org)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


#include "stdint.h"
#include "stdbool.h"
#include "stdlib.h"
#include "stdio.h"

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/dma.h>

#include "sampler.h"
#include "board.h"



sampler_ret_t sampler_init(Sampler *self, uint32_t timer_freq_hz, uint32_t adc_freq_hz) {
	if (self == NULL) {
		return SAMPLER_RET_FAILED;
	}

	/* Enable DMA clocks. */
	rcc_periph_clock_enable(RCC_DMA1);
	rcc_periph_clock_enable(RCC_DMA2);

	nvic_enable_irq(NVIC_DMA1_CHANNEL1_IRQ);
	nvic_enable_irq(NVIC_DMA2_CHANNEL1_IRQ);
	nvic_enable_irq(NVIC_DMA2_CHANNEL5_IRQ);
	nvic_enable_irq(NVIC_DMA2_CHANNEL2_IRQ);


	/* Enable timer clocks. */
	rcc_periph_clock_enable(RCC_TIM3);
	rcc_periph_clock_enable(RCC_TIM1);
	rcc_periph_clock_enable(RCC_TIM8);

	self->adc_freq_hz = adc_freq_hz;
	self->timer_freq_hz = timer_freq_hz;

	/* Initialize the TIM3, but does not enable it. TIM3 will be used to
	 * trigger ADc conversions on all channels simultaneously. */
	timer_reset(TIM3);
	timer_set_mode(TIM3, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
	timer_continuous_mode(TIM3);
	timer_direction_up(TIM3);
	timer_disable_preload(TIM3);
	timer_enable_update_event(TIM3);

	/* We are counting up at the maximum speed, no prescaler. */
	timer_set_prescaler(TIM3, 0);

	/* Set the timer period to be the same as the ADC sampling frequency. */
	timer_set_period(TIM3, self->timer_freq_hz / self->adc_freq_hz - 1);

	/** @todo do we need the update interrupt? disabled now. */
	// timer_enable_irq(TIM3, TIM_DIER_UIE);

	/* Configure it as a slave instead and wait for TIM2. TIM2 is run by the
	 * GPSDO and synced to the GPS clock. */
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

	/* We are counting at the sampling frequency. */
	/** @todo wtf, prescaler setting doesn't not work. */
	timer_set_prescaler(TIM1, 0);
	timer_set_period(TIM1, 65535);

	/* Do not enable the timer, configure slave mode instead. The timer waits with
	 * its counter at 0 for an external event. It is started when the analog watchdog
	 * detects signal with configured parameters. */
	timer_slave_set_polarity(TIM1, TIM_ET_RISING);
	TIM1_SMCR |= TIM_SMCR_ETF_DTS_DIV_4_N_6;
	timer_slave_set_trigger(TIM1, TIM_SMCR_TS_ETRF);
	timer_slave_set_mode(TIM1, TIM_SMCR_SMS_TM);
	TIM1_OR |= TIM1_ETR_ADC1_RMP_AWD1;

	/* When the timer is started, wait until a desired number of samples
	 * is available in the buffer and then stop the ADC in the interrupt handler. */
	timer_disable_oc_preload(TIM1, TIM_OC1);
	/** @todo the time instant should be configurable */
	timer_set_oc_value(TIM1, TIM_OC1, 2047 * 30);
	timer_enable_irq(TIM1, TIM_DIER_CC1IE);

	/* Enable ADC1 for channels 1 and 2. */
	rcc_periph_clock_enable(RCC_ADC12);
	ADC12_CCR |= ADC_CCR_CKMODE_DIV1;

	{
		/* Setup channel 1. */
		adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_4DOT5CYC);
		uint8_t channels[16] = {4};
		adc_set_regular_sequence(ADC1, 1, channels);

		/* Select ADC conversion trigger by TIM3 and enable the DMA. */
		ADC1_CFGR |= ADC_CFGR_EXTEN_RISING_EDGE;
		ADC1_CFGR |= ADC_CFGR_EXTSEL_EXT4;
		ADC1_CFGR |= ADC_CFGR_DMACFG | ADC_CFGR_DMAEN;

		/* Configure ADC1 analog watchdog for channel 1. */
		ADC1_TR1 = (uint32_t)((3000 << 16) | 1000);
		ADC1_CFGR |= ADC_CFGR_AWD1EN;

		/* And start the ADC1. */
		adc_power_on(ADC1);
		ADC1_CR |= ADC_CR_ADSTART;
	}

	{
		/* Setup channel 2. */
		adc_set_sample_time_on_all_channels(ADC2, ADC_SMPR_SMP_4DOT5CYC);
		uint8_t channels[16] = {4};
		adc_set_regular_sequence(ADC2, 1, channels);

		/* Select ADC conversion trigger by TIM3 and enable the DMA. */
		ADC2_CFGR |= ADC_CFGR_EXTEN_RISING_EDGE;
		ADC2_CFGR |= ADC_CFGR_EXTSEL_EXT4;
		ADC2_CFGR |= ADC_CFGR_DMACFG | ADC_CFGR_DMAEN;

		/* Configure ADC2 analog watchdog for channel 1. */
		ADC2_TR1 = (uint32_t)((2100 << 16) | 1900);
		ADC2_CFGR |= ADC_CFGR_AWD1EN;

		/* And start the ADC2. */
		adc_power_on(ADC2);
		ADC2_CR |= ADC_CR_ADSTART;
	}

	/* Channels 3 and 4. */
	rcc_periph_clock_enable(RCC_ADC34);
	ADC34_CCR |= ADC_CCR_CKMODE_DIV1;

	{
		/* Setup channel 3. */
		adc_set_sample_time_on_all_channels(ADC3, ADC_SMPR_SMP_4DOT5CYC);
		uint8_t channels[16] = {2};
		adc_set_regular_sequence(ADC3, 1, channels);

		/* Select ADC conversion trigger by TIM3 and enable the DMA. */
		ADC3_CFGR |= ADC_CFGR_EXTEN_RISING_EDGE;
		ADC3_CFGR |= ADC_CFGR_EXTSEL_EXT11;
		ADC3_CFGR |= ADC_CFGR_DMACFG | ADC_CFGR_DMAEN;

		/* Configure ADC3 analog watchdog for channel 1. */
		ADC3_TR1 = (uint32_t)((2100 << 16) | 1900);
		ADC3_CFGR |= ADC_CFGR_AWD1EN;

		/* And start the ADC3. */
		adc_power_on(ADC3);
		ADC3_CR |= ADC_CR_ADSTART;
	}

	{
		/* Setup channel 4. */
		adc_set_sample_time_on_all_channels(ADC4, ADC_SMPR_SMP_4DOT5CYC);
		uint8_t channels[16] = {2};
		adc_set_regular_sequence(ADC4, 1, channels);

		/* Select ADC conversion trigger by TIM3 and enable the DMA. */
		ADC4_CFGR |= ADC_CFGR_EXTEN_RISING_EDGE;
		ADC4_CFGR |= ADC_CFGR_EXTSEL_EXT11;
		ADC4_CFGR |= ADC_CFGR_DMACFG | ADC_CFGR_DMAEN;

		/* Configure ADC3 analog watchdog for channel 1. */
		ADC4_TR1 = (uint32_t)((2100 << 16) | 1900);
		ADC4_CFGR |= ADC_CFGR_AWD1EN;

		/* And start the ADC3. */
		adc_power_on(ADC4);
		ADC4_CR |= ADC_CR_ADSTART;
	}

	return SAMPLER_RET_OK;
}


sampler_ret_t sampler_start(Sampler *self) {
	if (self == NULL) {
		return SAMPLER_RET_FAILED;
	}

	/* Reset analog watchdog delay timers and enable their interrupts.
	 * They will be triggered by the watchdogs. */
	timer_set_counter(TIM1, 0);
	timer_enable_irq(TIM1, TIM_DIER_CC1IE);

	/* Setup and enable DMA for all channels. */
	for (uint8_t i = 0; i < 4; i++) {
		sampler_dma_enable(&sampler, i);
	}

	/* Start the TIM3 to enable ADC sampling. We need to do this synchronously
	 * to the TIM2 in order to know the exact position within a single second. */
	uint32_t current = timer_get_counter(TIM2);

	/* Prepare the starting position sufficiently in the future. */
	uint32_t next = (current / ADC_BUFFER_SIZE / 30) * 30 * ADC_BUFFER_SIZE + 2 * 30 * ADC_BUFFER_SIZE;

	for (uint8_t i = 0; i < 4; i++) {
		self->buf[i].buffer_start_time = next;
	}
	/* Ask the GPSDO to make a trigger pulse to enable the TIM3. */
	gpsdo_sync_start(&gpsdo, next);

	return SAMPLER_RET_OK;
}


sampler_ret_t sampler_stop(Sampler *self) {
	if (self == NULL) {
		return SAMPLER_RET_FAILED;
	}

	/* Disable triggering of the ADC. */
	timer_disable_counter(TIM3);

	/* Disable watchdog delay timers and clear any pending interrupts. */
	timer_disable_irq(TIM1, TIM_DIER_CC1IE);
	timer_clear_flag(TIM1, TIM_SR_CC1IF);
	timer_disable_counter(TIM1);

	for (size_t i = 0; i < 4; i++) {
		/* Position in the buffer AFTER the ADC is stopped. */
		self->buf[i].buffer_second_time = self->buf[i].buffer_start_time % self->timer_freq_hz;
		self->buf[i].buffer_pos = ADC_BUFFER_SIZE - DMA_CNDTR(DMA1, DMA_CHANNEL1);
		self->buf[i].trigger_pos = self->buf[i].buffer_pos - 2048;
		self->buf[i].trigger_time = self->buf[i].buffer_second_time + self->buf[i].trigger_pos * 30;

		/* Triggered in the brevious second. */
		if (self->buf[i].trigger_time < 0) {
			self->buf[i].trigger_time += self->timer_freq_hz;
		}
	}

	return SAMPLER_RET_OK;
}


sampler_ret_t sampler_dma_enable(Sampler *self, uint8_t sampler_channel) {
	if (self == NULL) {
		return SAMPLER_RET_FAILED;
	}

	uint32_t dma = 0;
	uint32_t dma_channel = 0;
	uint32_t adc_dr = 0;
	switch (sampler_channel) {
		case 0:
			dma = DMA1;
			dma_channel = DMA_CHANNEL1;
			adc_dr = (uint32_t)&ADC1_DR;
			break;
		case 1:
			dma = DMA2;
			dma_channel = DMA_CHANNEL1;
			adc_dr = (uint32_t)&ADC2_DR;
			break;
		case 2:
			dma = DMA2;
			dma_channel = DMA_CHANNEL5;
			adc_dr = (uint32_t)&ADC3_DR;
			break;
		case 3:
			dma = DMA2;
			dma_channel = DMA_CHANNEL2;
			adc_dr = (uint32_t)&ADC4_DR;
			break;
		default:
			return SAMPLER_RET_FAILED;
	}

	dma_disable_channel(dma, dma_channel);
	dma_set_priority(dma, dma_channel, DMA_CCR_PL_HIGH);

	dma_set_memory_size(dma, dma_channel, DMA_CCR_MSIZE_16BIT);
	dma_set_peripheral_size(dma, dma_channel, DMA_CCR_PSIZE_16BIT);

	dma_enable_memory_increment_mode(dma, dma_channel);
	dma_disable_peripheral_increment_mode(dma, dma_channel);

 	dma_set_read_from_peripheral(dma, dma_channel);

 	dma_set_peripheral_address(dma, dma_channel, adc_dr);
	dma_set_memory_address(dma, dma_channel, (uint32_t)&(self->buf[sampler_channel].adc_buffer));

	dma_set_number_of_data(dma, dma_channel, ADC_BUFFER_SIZE);
	dma_enable_circular_mode(dma, dma_channel);

	dma_enable_transfer_complete_interrupt(dma, dma_channel);
	dma_enable_channel(dma, dma_channel);

	return SAMPLER_RET_OK;
}


sampler_ret_t sampler_print_buffer(Sampler *self) {

	for (size_t i = 0; i < 1; i++) {
		printf("channel %u:", i + 1);
		// printf("  buffer_start_time: %lu (%lu us)", self->buf[i].buffer_start_time, self->buf[i].buffer_start_time / 60);
		// printf("  buffer_pos: %lu", self->buf[i].buffer_pos);
		// printf("  trigger_pos: %lu", self->buf[i].trigger_pos);
		printf("  buffer_second_time: %10lu (%10lu us)", self->buf[i].buffer_second_time, self->buf[i].buffer_second_time / 60);
		printf("  trigger_time: %10ld (%10ld us)", self->buf[i].trigger_time, self->buf[i].trigger_time / 60);

		printf("\n");
	}

	// printf("buffer=");
	// for (size_t i = self->buf[0].buffer_pos + 1; i < ADC_BUFFER_SIZE; i++) {
		// printf("%04x", (uint16_t)self->buf[3].adc_buffer[i]);
	// }
	// for (size_t i = 0; i < self->buf[0].buffer_pos; i++) {
		// printf("%04x", (uint16_t)self->buf[3].adc_buffer[i]);
	// }
	// printf("\n");

}


sampler_ret_t sampler_dma_completed_handler(Sampler *self, uint8_t channel) {
	if (self == NULL) {
		return SAMPLER_RET_FAILED;
	}

	self->buf[channel].buffer_start_time = (self->buf[channel].buffer_start_time + 30 * ADC_BUFFER_SIZE);
	if (self->buf[channel].buffer_start_time > gpsdo.timer_period) {
		self->buf[channel].buffer_start_time -= gpsdo.timer_period;
	}

	return SAMPLER_RET_OK;
}
