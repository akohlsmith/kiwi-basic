/*
 * Copyright (c) 2018, Marek Koza (qyx@krtko.org)
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

/* Documentation in this file is very verbose as the synchronized sampling
 * is quite complex. */

/** @todo The sampler does not use any fifo buffer to save the sampled data yet. */


static sampler_ret_t sampler_init_clocks(Sampler *self) {
	if (self == NULL) {
		return SAMPLER_RET_BAD_ARG;
	}

	/* Enable DMA clocks. */
	rcc_periph_clock_enable(RCC_DMA1);
	rcc_periph_clock_enable(RCC_DMA2);

	/* Using only DMA1/channel1 transfer completed interrupt. Other channel interrupts
	 * are not needed because all channels run in sync. */
	nvic_enable_irq(NVIC_DMA1_CHANNEL1_IRQ);

	/* Enable timer clocks. TIM3 is used as a trigger for ADC1, ADC2, ADC3 and ADC4. */
	rcc_periph_clock_enable(RCC_TIM3);
	/* TIM1 and TIM8 are triggered by the analog watchdog(s) and their capture/compare
	 * interrupt stops sampling. */
	rcc_periph_clock_enable(RCC_TIM1);
	rcc_periph_clock_enable(RCC_TIM8);

	/* Cannot return error. */
	return SAMPLER_RET_OK;
}


static sampler_ret_t sampler_init_adc_trigger_clock(Sampler *self) {

	/* Initialize the TIM3, but does not enable it. TIM3 will be used to
	 * trigger ADC conversions on all channels simultaneously. */
	timer_reset(TIM3);
	timer_set_mode(TIM3, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
	timer_continuous_mode(TIM3);
	timer_direction_up(TIM3);

	/* We are counting up at the maximum speed (self->timer_freq_hz), no prescaler. */
	timer_set_prescaler(TIM3, 0);

	/* Set the timer period to be the same as the ADC sampling frequency. */
	timer_set_period(TIM3, self->timer_freq_hz / self->adc_freq_hz - 1);

	/* Generate an update event. */
	TIM1_EGR |= TIM_EGR_UG;

	/* Do not enable the timer. Configure it as a slave instead and wait for TIM2.
	 * TIM2 is run by the GPSDO and synced to the GPS clock. */
	timer_slave_set_polarity(TIM3, TIM_ET_RISING);
	timer_slave_set_trigger(TIM3, TIM_SMCR_TS_ITR1);
	timer_slave_set_mode(TIM3, TIM_SMCR_SMS_TM);

	/* Set update event as a TRGO output on TIM3 - this will be
	 * the trigger used to do ADC conversions. */
	timer_set_master_mode(TIM3, TIM_CR2_MMS_UPDATE);

	return SAMPLER_RET_OK;
}


/* Configure timer 1 and timer 8 to receive events from the analog watchdogs. */
static sampler_ret_t sampler_init_triggers(Sampler *self) {
	if (self == NULL) {
		return SAMPLER_RET_BAD_ARG;
	}

	/* TIM1 is counting up at the sampling speed. */
	timer_reset(TIM1);
	timer_set_mode(TIM1, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
	timer_continuous_mode(TIM1);
	timer_direction_up(TIM1);
	timer_set_prescaler(TIM1, self->timer_freq_hz / self->adc_freq_hz - 1);
	/* The maximum usable period is actually the same as the buffer size but
	 * we are using 2^16-1. */
	timer_set_period(TIM1, 65535);

	/* And update from shadow registers. */
	TIM1_EGR |= TIM_EGR_UG;

	/* Do not enable the timer, configure slave mode instead. The timer waits with
	 * its counter at 0 for an external event. It is started when the analog watchdog
	 * detects a signal with configured parameters. */
	timer_slave_set_polarity(TIM1, TIM_ET_RISING);

	/* The ETR input supports some basic filters. */
	/** @todo make the filter configurable */
	// TIM1_SMCR |= TIM_SMCR_ETF_DTS_DIV_4_N_6;
	timer_slave_set_trigger(TIM1, TIM_SMCR_TS_ETRF);
	timer_slave_set_mode(TIM1, TIM_SMCR_SMS_TM);
	TIM1_OR |= TIM1_ETR_ADC1_RMP_AWD1;

	/* When the timer is started, wait until a desired number of samples
	 * is available in the buffer and then stop the ADC in the capture/compare
	 * interrupt handler. */
	timer_disable_oc_preload(TIM1, TIM_OC1);
	timer_set_oc_value(TIM1, TIM_OC1, self->trigger_delay - 1);

	return SAMPLER_RET_OK;
}


static sampler_ret_t sampler_start_triggers(Sampler *self) {
	(void)self;

	/** @todo the trigger is enabled whenever the ADC is running. It basically cannot
	 *        be disabled as it is configured in slave mode and is enabled by the
	 *        trigger input. One way how to disable it is to disable the trigger input
	 *        or disable analog watchdogs.
	 *        Also, the trigger and the timer can be left enabled and we may disable
	 *        the CC interrupt only. */

	/* Always start from the zero. */
	timer_set_counter(TIM1, 0);
	timer_enable_irq(TIM1, TIM_DIER_CC1IE);

	return SAMPLER_RET_OK;
}


static sampler_ret_t sampler_stop_triggers(Sampler *self) {
	(void)self;

	/* As the timer is run in slave mode and is enabled by an external trigger,
	 * disabling the timer is not enough. The CC interrupt must be disabled too. */
	timer_disable_counter(TIM1);
	timer_disable_irq(TIM1, TIM_DIER_CC1IE);
	timer_clear_flag(TIM1, TIM_SR_CC1IF);

	return SAMPLER_RET_OK;
}


static sampler_ret_t sampler_init_adc(Sampler *self, uint32_t adc, uint32_t adc_channel) {
	if (self == NULL) {
		return SAMPLER_RET_BAD_ARG;
	}

	if (adc == ADC1 || adc == ADC2) {
		rcc_periph_clock_enable(RCC_ADC12);
		ADC12_CCR |= ADC_CCR_CKMODE_DIV1;
	} else if (adc == ADC3 || adc == ADC4) {
		rcc_periph_clock_enable(RCC_ADC34);
		ADC34_CCR |= ADC_CCR_CKMODE_DIV1;
	} else {
		return SAMPLER_RET_BAD_ARG;
	}

	/* Perform ADC calibration once. */
	ADC_CR(adc) |= ADC_CR_ADCAL;
	while (ADC_CR(adc) & ADC_CR_ADCAL) {
		;
	}

	adc_set_sample_time_on_all_channels(adc, ADC_SMPR_SMP_4DOT5CYC);
	uint8_t channels[16] = {adc_channel};
	adc_set_regular_sequence(adc, 1, channels);

	/* Select ADC conversion trigger by TIM3 and enable the DMA. */
	ADC_CFGR(adc) |= ADC_CFGR_EXTEN_RISING_EDGE;

	if (adc == ADC1 || adc == ADC2) {
		ADC_CFGR(adc) |= ADC_CFGR_EXTSEL_EXT4;
	} else {
		ADC_CFGR(adc) |= ADC_CFGR_EXTSEL_EXT11;
	}
	ADC_CFGR(adc) |= ADC_CFGR_DMACFG | ADC_CFGR_DMAEN;

	/* Configure the analog watchdog. */
	ADC_TR1(adc) = (uint32_t)(((2048 + self->trigger_value) << 16) | (2048 - self->trigger_value));
	ADC_CFGR(adc) |= ADC_CFGR_AWD1EN;

	/* And start the ADC. */
	adc_power_on(adc);
	ADC_CR(adc) |= ADC_CR_ADSTART;

	return SAMPLER_RET_OK;
}


/* DMA data for all 4 channels. */
static const uint32_t dma_controllers[4] = {DMA1, DMA2, DMA2, DMA2};
static const uint32_t dma_channels[4] = {DMA_CHANNEL1, DMA_CHANNEL1, DMA_CHANNEL5, DMA_CHANNEL2};
static const uint32_t adc_registers[4] = {
	(uint32_t)&ADC1_DR,
	(uint32_t)&ADC2_DR,
	(uint32_t)&ADC3_DR,
	(uint32_t)&ADC4_DR,
};


static sampler_ret_t sampler_dma_enable(Sampler *self, uint8_t sampler_channel) {
	if (self == NULL) {
		return SAMPLER_RET_FAILED;
	}

	uint32_t dma = dma_controllers[sampler_channel];
	uint32_t dma_channel = dma_channels[sampler_channel];
	uint32_t adc_dr = adc_registers[sampler_channel];

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


sampler_ret_t sampler_init(
	Sampler *self,
	uint32_t timer_freq_hz,
	uint32_t adc_freq_hz,
	uint32_t trigger_delay,
	uint16_t trigger_value
) {
	if (self == NULL) {
		return SAMPLER_RET_FAILED;
	}

	self->adc_freq_hz = adc_freq_hz;
	self->timer_freq_hz = timer_freq_hz;
	self->trigger_delay = trigger_delay;
	self->trigger_value = trigger_value;

	sampler_init_clocks(self);
	sampler_init_adc_trigger_clock(self);
	sampler_init_triggers(self);

	sampler_init_adc(self, ADC1, 4);
	sampler_init_adc(self, ADC2, 4);
	sampler_init_adc(self, ADC3, 2);
	sampler_init_adc(self, ADC4, 2);

	return SAMPLER_RET_OK;
}


sampler_ret_t sampler_start(Sampler *self) {
	if (self == NULL) {
		return SAMPLER_RET_FAILED;
	}

	/* Reset analog watchdog delay timers and enable their interrupts.
	 * They will be triggered by the watchdogs. */
	sampler_start_triggers(self);

	/* Setup and enable DMA for all channels. */
	for (uint8_t i = 0; i < 4; i++) {
		sampler_dma_enable(&sampler, i);
	}

	/* Start the TIM3 to enable ADC sampling. We need to do this synchronously
	 * to the TIM2 in order to know the exact position within a single second. */
	uint32_t current = timer_get_counter(TIM2);

	/* Prepare the starting position sufficiently in the future. */
	uint32_t tics_per_sample = self->timer_freq_hz / self->adc_freq_hz;
	uint32_t buffer_size_tics = ADC_BUFFER_SIZE * tics_per_sample;
	uint32_t next = (current / buffer_size_tics) * buffer_size_tics + 2 * buffer_size_tics;
	self->buffer_second_time = next % self->timer_freq_hz;
	self->buffer_time = next / self->timer_freq_hz;

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
	sampler_stop_triggers(self);

	/* Position in the buffer AFTER the ADC is stopped (in samples). */
	self->buffer_pos = ADC_BUFFER_SIZE - DMA_CNDTR(DMA1, DMA_CHANNEL1);

	/* Trigger position is trigger_delay earlier. It may be negative! */
	int32_t trigger_pos = self->buffer_pos - self->trigger_delay;

	self->trigger_time = self->buffer_time;
	self->trigger_second_time = self->buffer_second_time + trigger_pos * (self->timer_freq_hz / self->adc_freq_hz);

	/* Buffer start is in this second but stop was triggered in the previous second. */
	if (self->trigger_second_time < 0) {
		self->trigger_second_time += self->timer_freq_hz;
		self->trigger_time -= 1;
	}
	/* Buffer start is in this second but stop was triggered in the next second. */
	if (self->trigger_second_time >= (int32_t)self->timer_freq_hz) {
		self->trigger_second_time -= self->timer_freq_hz;
		self->trigger_time += 1;
	}

	return SAMPLER_RET_OK;
}


/* Normally the sampler is triggered by the analog watchdog. This function allows
 * it to be triggered manually when needed. */
sampler_ret_t sampler_trigger(Sampler *self) {
	if (self == NULL) {
		return SAMPLER_RET_BAD_ARG;
	}

	/** @todo not implemented */

	return SAMPLER_RET_OK;
}


sampler_ret_t sampler_print_buffer(Sampler *self) {
	if (self == NULL) {
		return SAMPLER_RET_BAD_ARG;
	}

	// printf("  buffer_start_time: %lu (%lu us)", self->buf[i].buffer_start_time, self->buf[i].buffer_start_time / 60);
	// printf("  buffer_pos: %lu", self->buf[i].buffer_pos);
	// printf("  trigger_pos: %lu", self->buf[i].trigger_pos);
	printf("  time: %lu.%07lu", self->buffer_time, self->buffer_second_time / 6);
	printf("  trigger:  %ld.%07ld", self->trigger_time, self->trigger_second_time / 6);

	printf("\n");

	printf("buffer=");
	for (size_t i = self->buffer_pos + 1; i < ADC_BUFFER_SIZE; i++) {
		printf("%04x", (uint16_t)self->buf[0].adc_buffer[i]);
	}
	for (size_t i = 0; i < self->buffer_pos; i++) {
		printf("%04x", (uint16_t)self->buf[0].adc_buffer[i]);
	}
	printf("\n");

	return SAMPLER_RET_OK;
}


sampler_ret_t sampler_save_buffer(Sampler *self) {
	if (self == NULL) {
		return SAMPLER_RET_BAD_ARG;
	}

	self->buffer_read_pos = 0;
	self->buffer_not_empty = true;

	return SAMPLER_RET_OK;
}


sampler_ret_t sampler_dma_completed_handler(Sampler *self) {
	if (self == NULL) {
		return SAMPLER_RET_BAD_ARG;
	}

	/* After the DMA cycle is completed and the buffer is full (4096 samples of data),
	 * increase the buffer starting mark by buffer_size * samples_per_second (duration
	 * of sampling one single buffer). If the value overflows one second (timer_freq_hz),
	 * crop it.
	 * This does not introduce any delay in data processing.
	 */
	/** @todo there might be a race condition when the interrupt delay is longer than
	 *        the processing speed if the signal is sampled right at the buffer boundary.
	 *        The buffer_second_time might not be incremented in this case.
	 */

	uint32_t tics_per_sample = self->timer_freq_hz / self->adc_freq_hz;
	self->buffer_second_time = self->buffer_second_time + tics_per_sample * ADC_BUFFER_SIZE;
	if (self->buffer_second_time >= self->timer_freq_hz) {
		self->buffer_second_time -= self->timer_freq_hz;
		self->buffer_time += 1;
	}
	return SAMPLER_RET_OK;
}
