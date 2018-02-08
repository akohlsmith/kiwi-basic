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

#pragma once

#include "stdint.h"
#include "stdbool.h"
#include "stdlib.h"


#define ADC_BUFFER_SIZE 4096


typedef enum {
	SAMPLER_RET_OK = 0,
	SAMPLER_RET_FAILED,
} sampler_ret_t;


struct sampler_buffer {
	uint16_t adc_buffer[ADC_BUFFER_SIZE];
	uint32_t buffer_start_time;
	uint32_t buffer_second_time;
	uint32_t buffer_pos;
	int32_t trigger_pos;
	int32_t trigger_time;
};

typedef struct {
	uint32_t timer_freq_hz;
	uint32_t adc_freq_hz;

	struct sampler_buffer buf[4];

} Sampler;



sampler_ret_t sampler_init(Sampler *self, uint32_t timer_freq_hz, uint32_t adc_freq_hz);
sampler_ret_t sampler_dma_enable(Sampler *self, uint8_t sampler_channel);
sampler_ret_t sampler_print_buffer(Sampler *self);

sampler_ret_t sampler_start(Sampler *self);
sampler_ret_t sampler_stop(Sampler *self);
sampler_ret_t sampler_dma_completed_handler(Sampler *self, uint8_t channel);
