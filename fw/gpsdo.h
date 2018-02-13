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

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include <libopencm3/stm32/dac.h>


#define GPSDO_PID_KP 0.2
#define GPSDO_PID_KI 0.5
#define GPSDO_PID_KD 0.1

#define GPSDO_PHASE_KP 0.1
#define GPSDO_PHASE_KI 0.0
#define GPSDO_PHASE_KD 0.1

#define GPSDO_PERIOD_SYNC_RANGE_NS 1000
#define GPSDO_PHASE_SYNC_RANGE_NS 1000


typedef enum {
	GPSDO_RET_OK = 0,
	GPSDO_RET_FAILED,
} gpsdo_ret_t;


enum gpsdo_fix_status {
	GPSDO_STATUS_NO_FIX = 0,
	GPSDO_STATUS_FIX_2D,
	GPSDO_STATUS_FIX_3D,
};


enum gpsdo_pps_status {
	GPSDO_PPS_NONE = 0,
	GPSDO_PPS_OK,
};


enum gpsdo_sync_status {
	/* 1PPS signal is unavailable or we has not attempted any sync yet. */
	GPSDO_SYNC_STATUS_UNKNOWN = 0,

	/* 1PPS signal is available but we are out of the OK range. */
	GPSDO_SYNC_STATUS_ADJUSTING,

	/* The clock is synced within the allowable range. */
	GPSDO_SYNC_OK,
};


struct gpsdo_sync_stats {
	enum gpsdo_sync_status status;
	int32_t phase_error;
	int32_t period_error;
};


typedef struct {
	/* Gpsdo runs on a 32bit timer. A libopencm3 timer reference is needed. */
	uint32_t timer;
	uint32_t timer_freq_hz;
	uint32_t timer_period;

	/* GPS 1PPS signal is connected to a input capture line. */
	uint32_t pps_ic;
	uint32_t pps_ccie;
	uint32_t pps_ic_in;

	enum gpsdo_pps_status pps_status;

	/* Frequency of the VCTCXO is adjusted using a DAC output. */
	data_channel dac;

	/* Last value applied to the DAC converter to steer the VCTCXO. */
	int32_t pps_steer;

	/* Difference between the actual and desired timer ticks. */
	float pps_error;
	float pps_error_last;
	float pps_integral;
	uint32_t pps_setpoint;

	/* Sync output. */
	bool sync_enabled;
	uint32_t sync_oc;
	uint32_t sync_oc_ref;

	/* Housekeeping output compare. */
	bool housekeeping_enabled;
	uint32_t housekeeping_oc;
	uint32_t housekeeping_ccie;

	/* Sample of the CC register when the 1PPS signal was last received. */
	uint32_t pps_new;
	/* Previously sample 1PPS signal counter value. The variable is zero if the
	 * 1PPS signal is invalid (it is cleared if no 1PPS signal comes during the housekeeping
	 * period). */
	uint32_t pps_last;

	/* Actual number of timer ticks between the current and the last 1PPS pulse. */
	uint32_t pps_time_ticks;

	/* Phase error is the mismatch between the timer running from the VCTCXO
	 * and the precise GPS clock. The value is in timer ticks. */
	float phase_error;
	float phase_error_last;
	float phase_integral;


} Gpsdo;


gpsdo_ret_t gpsdo_init(Gpsdo *self, uint32_t timer, uint32_t pps_ic, uint32_t timer_freq_hz, data_channel dac);
gpsdo_ret_t gpsdo_sync_enable(Gpsdo *self, uint32_t sync_oc);
gpsdo_ret_t gpsdo_sync_start(Gpsdo *self, uint32_t time_tick);
gpsdo_ret_t gpsdo_housekeeping_enable(Gpsdo *self, uint32_t housekeeping_oc);
uint32_t gpsdo_counter(Gpsdo *self);
gpsdo_ret_t gpsdo_1pps_irq_handler(Gpsdo *self);

/**
 * @brief Schedule a housekeeping check in the future
 */
gpsdo_ret_t gpsdo_housekeeping_schedule(Gpsdo *self, uint32_t time_ms);
gpsdo_ret_t gpsdo_housekeeping_irq_handler(Gpsdo *self);
gpsdo_ret_t gpsdo_get_sync_status(Gpsdo *self, struct gpsdo_sync_stats *stats);
