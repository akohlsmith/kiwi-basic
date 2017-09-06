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

#include "gpsdo.h"


gpsdo_ret_t gpsdo_init(Gpsdo *self, uint32_t timer, uint32_t pps_ic, uint32_t timer_freq_hz, uint32_t dac) {
	if (self == NULL || timer_freq_hz == 0) {
		return GPSDO_RET_FAILED;
	}

	self->timer = timer;
	self->pps_ic = pps_ic;
	self->timer_freq_hz = timer_freq_hz;
	self->dac = dac;

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
	timer_set_period(self->timer, ~(uint32_t)0);

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
	timer_enable_irq(TIM2, self->pps_ccie);

	return GPSDO_RET_OK;
}


gpsdo_ret_t gpsdo_sync_enable(Gpsdo *self) {
	if (self == NULL) {
		return GPSDO_RET_FAILED;
	}

	/* Sync output is on the OC1 by default. */
	if (self->pps_ic == TIM_IC1) {
		self->sync_oc = TIM_OC2;
	} else {
		self->sync_oc = TIM_OC1;
	}

	switch (self->sync_oc) {
		case TIM_OC1:
			self->sync_oc_ref = TIM_CR2_MMS_COMPARE_OC1REF;
			break;

		case TIM_OC2:
			self->sync_oc_ref = TIM_CR2_MMS_COMPARE_OC2REF;
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
