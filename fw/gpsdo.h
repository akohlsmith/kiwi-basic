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

/*
 * This file is part of the kiwi board GPS synchronized signal digitizer GPSDO
 * subsystem. It steers a VCTCXO oscillator using feedback from the GPS 1PPS
 * output to provide an accurate clock for ADC synchronization.
 */

#pragma once

#include "stdint.h"
#include "stdbool.h"
#include "stdlib.h"


typedef enum {
	GPSDO_RET_OK = 0,
	GPSDO_RET_FAILED,
} gpsdo_ret_t;


typedef struct {
	/* Gpsdo runs on a 32bit timer. A libopencm3 timer reference is needed. */
	uint32_t timer;
	uint32_t timer_freq_hz;

	/* GPS 1PPS signal is connected to a input capture line. */
	uint32_t pps_ic;
	uint32_t pps_ccie;
	uint32_t pps_ic_in;

	/* Frequency of the VCTCXO is adjusted using a DAC output. */
	uint32_t dac;
	uint32_t dac_channel;

	/* Sync output. */
	bool sync_enabled;
	uint32_t sync_oc;
	uint32_t sync_oc_ref;


} Gpsdo;


gpsdo_ret_t gpsdo_init(Gpsdo *self, uint32_t timer, uint32_t pps_ic, uint32_t timer_freq_hz, uint32_t dac);
gpsdo_ret_t gpsdo_sync_enable(Gpsdo *self);
gpsdo_ret_t gpsdo_sync_start(Gpsdo *self, uint32_t time_tick);
uint32_t gpsdo_counter(Gpsdo *self);
