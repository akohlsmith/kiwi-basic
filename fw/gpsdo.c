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

#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/dac.h>

#include "gpsdo.h"

/**
 * @todo
 *
 * - add housekeeping OC + interrupt (every 2sec) (DONE)
 * - DAC output (DONE)
 * - add 1PPS IC interrupt handler with PID to steer the VCTVXO (clock adjust) (DONE)
 * - status output (DONE)
 * - clock step if the phase difference is greater than a configurable value (DONE)
 * - add proper error handling
 */

gpsdo_ret_t gpsdo_init(Gpsdo *self, uint32_t timer, uint32_t pps_ic, uint32_t timer_freq_hz, data_channel dac) {
	if (self == NULL || timer_freq_hz == 0) {
		return GPSDO_RET_FAILED;
	}

	self->timer = timer;
	self->pps_ic = pps_ic;
	self->timer_freq_hz = timer_freq_hz;
	self->dac = dac;
	self->pps_setpoint = timer_freq_hz;

	/* Initialize the VCTCXO steering PID controller. */
	self->pps_error = 0;
	self->pps_error_last = 0;
	self->pps_integral = 0;

	/* Initialize the timer, count up, continuously. */
	timer_reset(self->timer);
	timer_set_mode(self->timer, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
	timer_continuous_mode(self->timer);
	timer_direction_up(self->timer);
	timer_enable_break_main_output(self->timer);
	timer_enable_update_event(self->timer);
	timer_disable_preload(self->timer);
	timer_set_prescaler(self->timer, 0);

	/* Set the maximum possible period. */
	timer_set_period(self->timer, (~(uint32_t)0) / self->timer_freq_hz * self->timer_freq_hz);

	switch (self->pps_ic) {
		case TIM_IC1:
			self->pps_ccie = TIM_DIER_CC1IE;
			self->pps_ic_in = TIM_IC_IN_TI1;
			break;

		case TIM_IC2:
			self->pps_ccie = TIM_DIER_CC2IE;
			self->pps_ic_in = TIM_IC_IN_TI2;
			break;

		case TIM_IC3:
			self->pps_ccie = TIM_DIER_CC3IE;
			self->pps_ic_in = TIM_IC_IN_TI3;
			break;

		case TIM_IC4:
			self->pps_ccie = TIM_DIER_CC4IE;
			self->pps_ic_in = TIM_IC_IN_TI4;
			break;

		default:
			return GPSDO_RET_FAILED;
	};

	timer_ic_set_input(self->timer, self->pps_ic, self->pps_ic_in);
	timer_ic_set_filter(self->timer, self->pps_ic, TIM_IC_OFF);

	/* Detecting 1PPS on the rising edge. */
	timer_ic_set_polarity(self->timer, self->pps_ic, TIM_IC_RISING);
	timer_ic_set_prescaler(self->timer, self->pps_ic, TIM_IC_PSC_OFF);
	timer_ic_enable(self->timer, self->pps_ic);

	/* CC2 irq to print 1PPS pulse period and adjust the VCTCXO. */
	timer_enable_irq(self->timer, self->pps_ccie);

	/* Enable the DAC. */
	dac_enable(self->dac);
	dac_load_data_buffer_single(0, RIGHT12, self->dac);

	return GPSDO_RET_OK;
}


gpsdo_ret_t gpsdo_sync_enable(Gpsdo *self, uint32_t sync_oc) {
	if (self == NULL) {
		return GPSDO_RET_FAILED;
	}

	self->sync_oc = sync_oc;
	switch (self->sync_oc) {
		case TIM_OC1:
			self->sync_oc_ref = TIM_CR2_MMS_COMPARE_OC1REF;
			break;

		case TIM_OC2:
			self->sync_oc_ref = TIM_CR2_MMS_COMPARE_OC2REF;
			break;

		case TIM_OC3:
			self->sync_oc_ref = TIM_CR2_MMS_COMPARE_OC3REF;
			break;

		case TIM_OC4:
			self->sync_oc_ref = TIM_CR2_MMS_COMPARE_OC4REF;
			break;

		default:
			return GPSDO_RET_FAILED;
	};

	/* Enable sync output. */
	timer_disable_oc_preload(self->timer, self->sync_oc);
	timer_set_oc_mode(self->timer, self->sync_oc, TIM_OCM_PWM2);
	timer_set_oc_value(self->timer, self->sync_oc, 0);
	timer_set_master_mode(self->timer, self->sync_oc_ref);

	self->sync_enabled = true;
	return GPSDO_RET_OK;
}


gpsdo_ret_t gpsdo_housekeeping_enable(Gpsdo *self, uint32_t housekeeping_oc) {
	if (self == NULL) {
		return GPSDO_RET_FAILED;
	}

	self->housekeeping_oc = housekeeping_oc;
	switch (self->housekeeping_oc) {
		case TIM_OC1:
			self->housekeeping_ccie = TIM_DIER_CC1IE;
			break;

		case TIM_OC2:
			self->housekeeping_ccie = TIM_DIER_CC2IE;
			break;

		case TIM_OC3:
			self->housekeeping_ccie = TIM_DIER_CC3IE;
			break;

		case TIM_OC4:
			self->housekeeping_ccie = TIM_DIER_CC4IE;
			break;

		default:
			return GPSDO_RET_FAILED;
	};

	timer_disable_oc_preload(self->timer, self->housekeeping_oc);
	timer_set_oc_value(self->timer, self->housekeeping_oc, 1);
	timer_enable_irq(self->timer, self->housekeeping_ccie);

	self->housekeeping_enabled = true;
	return GPSDO_RET_OK;
}


gpsdo_ret_t gpsdo_sync_start(Gpsdo *self, uint32_t time_tick) {
	if (self == NULL || self->sync_enabled == false) {
		return GPSDO_RET_FAILED;
	}

	timer_set_oc_value(self->timer, self->sync_oc, time_tick);

	return GPSDO_RET_OK;
}


uint32_t gpsdo_counter(Gpsdo *self) {
	if (self == NULL) {
		return 0;
	}

	return timer_get_counter(self->timer);
}


gpsdo_ret_t gpsdo_1pps_irq_handler(Gpsdo *self) {
	if (self == NULL) {
		return GPSDO_RET_FAILED;
	}

	/* Capture the timer counter value when the 1PPS signal arrives. */
	switch (self->pps_ic) {
		case TIM_IC1:
			self->pps_new = TIM_CCR1(self->timer);
			break;
		case TIM_IC2:
			self->pps_new = TIM_CCR2(self->timer);
			break;
		case TIM_IC3:
			self->pps_new = TIM_CCR3(self->timer);
			break;
		case TIM_IC4:
			self->pps_new = TIM_CCR4(self->timer);
			break;
		default:
			return GPSDO_RET_FAILED;
	}

	/* 1PPS signal becomes valid (pps_last was invalid). Save the actual
	 * time and do nothing else. We cannot compute or adjust anything. */
	if (self->pps_last == 0) {
		self->pps_last = self->pps_new;
		return GPSDO_RET_OK;
	}

	/* Calculate the number of local timer ticks from the last 1PPS signal. */
	uint32_t pps_time_ticks = self->pps_new - self->pps_last;
	self->pps_last = self->pps_new;

	/* Now try to maintain the second phase between the precise GPS clock
	 * and the local timer running from the VCTCXO. There is a PID regulator
	 * for this purpose. */

	/* 1 second phase PID. Compute the expected timer counter value which would
	 * be sampled if the local timer was running precisely. */
	uint32_t go_to = (self->pps_new + self->timer_freq_hz / 2) / self->timer_freq_hz * self->timer_freq_hz;

	/* Determine if there is any phase error, positive or negative. */
	self->phase_error =  (float)((int64_t)go_to - (int64_t)self->pps_new);

	if (abs(self->phase_error) > 10000) {
		/* If the phase error is that big that it is not feasible to
		 * fine-tune the timer, adjust the timer value by setting
		 * it to something close to the expected value. */
		timer_set_counter(self->timer, (uint32_t)(timer_get_counter(TIM2) + self->phase_error));

		/* And reset the PID because we need to start over. */
		self->phase_error_last = 0;
		self->phase_integral = 0;
		self->pps_last = (uint32_t)(timer_get_counter(TIM2) + self->phase_error);

		/* Also, reschedule the housekeeping as the counter value has changed. */
		gpsdo_housekeeping_schedule(self, 2000);

		/* There is not much to do actually, continue when the next
		 * 1PPS signal arrives. */
		return GPSDO_RET_OK;
	} else {
		/* If the phase error is reasonably small, try to adjust the phase
		 * by fine-tuning the 1PPS setpoint value (another PID regulator). */
		self->phase_integral += self->phase_error;
		float phase_derivative = self->phase_error - self->phase_error_last;
		self->pps_setpoint = self->timer_freq_hz + (int32_t)(GPSDO_PHASE_KP * self->phase_error + GPSDO_PHASE_KI * self->phase_integral + GPSDO_PHASE_KD * phase_derivative);
		self->phase_error_last = self->phase_error;
	}


	/* Now we are going to control the actual period of local "second"
	 * (and not the phase itself). Maintaining a correct second period doesn't
	 * help us much (as the phase may be off significantly), but we are
	 * manipulating the period setpoint to achieve the correct phase. */

	/* Check if the 1PPS signal is within a reasonable range (+- 1000pps).
	 * Just to be sure we don't have bogus configuration. The VCTCXO with
	 * steering completely off should not be THAT imprecise. */
	if (pps_time_ticks > (self->timer_freq_hz - self->timer_freq_hz / 1000) &&
	    pps_time_ticks < (self->timer_freq_hz + self->timer_freq_hz / 1000)) {
		self->pps_time_ticks = pps_time_ticks;
	} else {
		/* 1PPS is invalid. Keep the VCTCXO running at the last frequency. */
		self->pps_time_ticks = 0;
		return GPSDO_RET_FAILED;
	}

	/* Now if the period of the 1PPS pulse is known, compare it to the
	 * actually counted timer ticks and steer the VCTCXO accordingly
	 * (this is a simple PID regulator). */
	self->pps_error = (float)((int64_t)self->pps_setpoint - (int64_t)self->pps_time_ticks);

	self->pps_integral += self->pps_error;
	float pps_derivative = self->pps_error - self->pps_error_last;
	self->pps_steer = GPSDO_PID_KP * self->pps_error + GPSDO_PID_KI * self->pps_integral + GPSDO_PID_KD * pps_derivative;
	self->pps_error_last = self->pps_error;

	/* Sanitize the output. */
	if (self->pps_steer < 0) {
		self->pps_steer = 0;
	}
	if (self->pps_steer > 4095) {
		self->pps_steer = 4095;
	}

	/* And apply the PID output as a value to the VC node of the VCTCXO. */
	dac_load_data_buffer_single(self->pps_steer, RIGHT12, self->dac);

	return GPSDO_RET_OK;
}


gpsdo_ret_t gpsdo_housekeeping_schedule(Gpsdo *self, uint32_t time_ms) {
	if (time_ms < 1000) {
		/* The housekeeping interval is smaller than the 1PPS signal period. */
		return GPSDO_RET_FAILED;
	}

	uint32_t counter = timer_get_counter(self->timer);

	/* Go to the future. */
	counter += self->timer_freq_hz / 1000 * time_ms;

	timer_set_oc_value(self->timer, self->housekeeping_oc,  counter);

	return GPSDO_RET_OK;
}


gpsdo_ret_t gpsdo_housekeeping_irq_handler(Gpsdo *self) {
	/* Schedule the next check in 2 seconds. */
	gpsdo_housekeeping_schedule(self, 2000);

	if (self->pps_new == 0) {
		self->pps_status = GPSDO_PPS_NONE;
	} else {
		self->pps_status = GPSDO_PPS_OK;
	}

	/* Erase the pps_new variable to be able to detect if a 1PPS pulse
	 * arrived when the housekeeping is called the next time. */
	self->pps_new = 0;
}


gpsdo_ret_t gpsdo_get_sync_status(Gpsdo *self, struct gpsdo_sync_stats *stats) {
	if (self == NULL) {
		return GPSDO_RET_FAILED;
	}

	if (self->pps_status == GPSDO_PPS_NONE) {
		stats->status = GPSDO_SYNC_STATUS_UNKNOWN;
		stats->phase_error = 0;
		stats->period_error = 0;
		return GPSDO_RET_OK;
	}

	/* Convert to nanoseconds. */
	stats->phase_error = self->phase_error * 1000 / (self->timer_freq_hz / 1000000);
	stats->period_error = self->pps_error * 1000 / (self->timer_freq_hz / 1000000);

	if (abs(stats->phase_error) <= GPSDO_PHASE_SYNC_RANGE_NS && abs(stats->period_error) <= GPSDO_PERIOD_SYNC_RANGE_NS) {
		stats->status = GPSDO_SYNC_OK;
	} else {
		stats->status = GPSDO_SYNC_STATUS_ADJUSTING;
	}

	return GPSDO_RET_OK;
}
