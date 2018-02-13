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

#pragma once

#include "stdint.h"
#include "stdbool.h"
#include "stdlib.h"


/* Size of the sampler buffer in samples. */
#define ADC_BUFFER_SIZE 4096


typedef enum {
	SAMPLER_RET_OK = 0,
	SAMPLER_RET_FAILED,
	SAMPLER_RET_BAD_ARG,
} sampler_ret_t;


struct sampler_buffer {
	uint16_t adc_buffer[ADC_BUFFER_SIZE];
};

typedef struct {
	/* Frequency of the timers (AHB frequency). */
	uint32_t timer_freq_hz;

	/* Sampling frequency of the ADC. */
	uint32_t adc_freq_hz;

	/* After the trigger occurs, we wait trigger_delay samples
	 * before stopping the ADC and saving the result. */
	uint32_t trigger_delay;

	/* APPROXIMATE time in seconds when the trigger occured. */
	int32_t trigger_time;

	/* APPROXIMATE time in second fractions when the trigger
	 * occured. There are timer_freq_hz fractions within a second. */
	int32_t trigger_second_time;

	/* Value of the signal (amplitude) which triggers a buffer
	 * saving. Both positive and negative value applies. */
	uint16_t trigger_value;

	/* Position of the DMA transfer within the buffer (in samples). */
	uint32_t buffer_pos;

	/* ne buffer for every channel. */
	struct sampler_buffer buf[4];

	/* EXACT time in seconds when the buffer starts. */
	uint32_t buffer_second_time;

	/* EXACT time in second fractions when the buffer starts. */
	uint32_t buffer_time;

	/* Temporary handling of buffer readings. */
	size_t buffer_read_pos;
	bool buffer_not_empty;
} Sampler;


sampler_ret_t sampler_init(
	Sampler *self,
	uint32_t timer_freq_hz,
	uint32_t adc_freq_hz,
	uint32_t trigger_delay,
	uint16_t trigger_value
);
sampler_ret_t sampler_print_buffer(Sampler *self);
sampler_ret_t sampler_start(Sampler *self);
sampler_ret_t sampler_stop(Sampler *self);
sampler_ret_t sampler_dma_completed_handler(Sampler *self);
