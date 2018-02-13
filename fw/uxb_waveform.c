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


#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "uxb_waveform.h"
#include "waveform.pb.h"
#include "pb_encode.h"
#include "pb_decode.h"


uxb_waveform_ret_t uxb_waveform_init(UxbWaveform *self, LibUxbSlot *slot, Sampler *sampler) {
	if (self == NULL || slot == NULL || sampler == NULL) {
		return UXB_WAVEFORM_RET_FAILED;
	}

	self->slot = slot;
	self->sampler = sampler;

	return UXB_WAVEFORM_RET_OK;
}


static uxb_waveform_ret_t parse_message(UxbWaveform *self, const uint8_t *buf, size_t len, WaveformRequest *msg) {
	(void)self;

	pb_istream_t stream;
        stream = pb_istream_from_buffer(buf, len);

	if (!pb_decode(&stream, WaveformRequest_fields, msg)) {
		return UXB_WAVEFORM_RET_FAILED;
	}

	return UXB_WAVEFORM_RET_OK;
}


static uxb_waveform_ret_t send_message(UxbWaveform *self, const WaveformResponse *msg) {
	uint8_t tx[128];
	pb_ostream_t stream;
	stream = pb_ostream_from_buffer(tx, sizeof(tx));
	if (!pb_encode(&stream, WaveformResponse_fields, msg)) {
	       return UXB_WAVEFORM_RET_FAILED;;
	}

	libuxb_slot_send_data(self->slot, tx, stream.bytes_written, true);
	return UXB_WAVEFORM_RET_OK;
}


static uxb_waveform_ret_t send_start_recording_response(UxbWaveform *self) {
	memset(&self->resp, 0, sizeof(WaveformResponse));
	self->resp.which_content = WaveformResponse_start_recording_response_tag;
	self->resp.result = Result_OK;
	self->resp.buffer_stats.free = 0;
	self->resp.buffer_stats.occupied = 0;
	self->resp.buffer_stats.size = ADC_BUFFER_SIZE;

	send_message(self, &self->resp);

	return UXB_WAVEFORM_RET_OK;
}


static uxb_waveform_ret_t process_start_recording(UxbWaveform *self, const StartRecording *msg) {
	(void)self;
	(void)msg;

	/* Clear the buffer first. */
	self->sampler->buffer_read_pos = 0;
	self->sampler->buffer_not_empty = false;
	sampler_start(self->sampler);
	send_start_recording_response(self);

	return UXB_WAVEFORM_RET_OK;
}


static uxb_waveform_ret_t send_stop_recording_response(UxbWaveform *self) {
	memset(&self->resp, 0, sizeof(WaveformResponse));
	self->resp.which_content = WaveformResponse_stop_recording_response_tag;
	self->resp.result = Result_OK;
	self->resp.buffer_stats.free = 0;
	self->resp.buffer_stats.occupied = 0;
	self->resp.buffer_stats.size = ADC_BUFFER_SIZE;

	send_message(self, &self->resp);

	return UXB_WAVEFORM_RET_OK;
}


static uxb_waveform_ret_t process_stop_recording(UxbWaveform *self, const StopRecording *msg) {
	(void)self;
	(void)msg;

	sampler_stop(self->sampler);
	send_stop_recording_response(self);

	return UXB_WAVEFORM_RET_OK;
}


static uxb_waveform_ret_t send_trigger_response(UxbWaveform *self) {
	memset(&self->resp, 0, sizeof(WaveformResponse));
	self->resp.which_content = WaveformResponse_trigger_response_tag;
	self->resp.result = Result_OK;
	self->resp.buffer_stats.free = 0;
	self->resp.buffer_stats.occupied = ADC_BUFFER_SIZE;
	self->resp.buffer_stats.size = ADC_BUFFER_SIZE;

	send_message(self, &self->resp);

	return UXB_WAVEFORM_RET_OK;
}


static uxb_waveform_ret_t process_trigger(UxbWaveform *self, const Trigger *msg) {
	(void)self;
	(void)msg;

	sampler_stop(self->sampler);
	self->sampler->buffer_read_pos = 0;
	self->sampler->buffer_not_empty = true;
	send_trigger_response(self);

	return UXB_WAVEFORM_RET_OK;
}


static uxb_waveform_ret_t send_get_data_response(UxbWaveform *self) {
	memset(&self->resp, 0, sizeof(WaveformResponse));
	self->resp.which_content = WaveformResponse_get_data_response_tag;

	if (self->sampler->buffer_not_empty == false) {
		self->resp.result = Result_FAILED;
		self->resp.buffer_stats.free = ADC_BUFFER_SIZE;
		self->resp.buffer_stats.occupied = 0;
		self->resp.buffer_stats.size = ADC_BUFFER_SIZE;
	} else {
		self->resp.result = Result_OK;
		self->resp.buffer_stats.free = self->sampler->buffer_read_pos;
		self->resp.buffer_stats.occupied = ADC_BUFFER_SIZE - self->sampler->buffer_read_pos;
		self->resp.buffer_stats.size = ADC_BUFFER_SIZE;

		if (self->sampler->buffer_read_pos == 0) {
			self->resp.content.get_data_response.has_new_frame = true;
			self->resp.content.get_data_response.new_frame = true;
		}

		self->resp.content.get_data_response.has_data = true;
		self->resp.content.get_data_response.data.size = 64;

		for (size_t i = 0; i < 16; i++) {
			for (size_t j = 0; j < 4; j++) {
				/* MSB first (big endian). */
				self->resp.content.get_data_response.data.bytes[i * 8 + j * 2] = self->sampler->buf[j].adc_buffer[i] >> 8;
				/* LSB. */
				self->resp.content.get_data_response.data.bytes[i * 8 + j * 2 + 1] = self->sampler->buf[j].adc_buffer[i] & 0xff;
			}
		}
		self->sampler->buffer_read_pos += 128;
		if (self->sampler->buffer_read_pos == ADC_BUFFER_SIZE) {
			self->sampler->buffer_not_empty = false;
		}

		if (self->sampler->buffer_read_pos >= ADC_BUFFER_SIZE) {
			self->resp.content.get_data_response.has_finish_frame = true;
			self->resp.content.get_data_response.finish_frame = true;
		}

	}

	send_message(self, &self->resp);

	return UXB_WAVEFORM_RET_OK;
}


static uxb_waveform_ret_t process_get_data(UxbWaveform *self, const GetData *msg) {
	(void)self;
	(void)msg;

	send_get_data_response(self);

	return UXB_WAVEFORM_RET_OK;
}


uxb_waveform_ret_t uxb_waveform_message_received(UxbWaveform *self, const uint8_t *buf, size_t len) {
	if (self == NULL || buf == NULL || len == 0) {
		return UXB_WAVEFORM_RET_FAILED;
	}

	if (parse_message(self, buf, len, &self->req) != UXB_WAVEFORM_RET_OK) {
		return UXB_WAVEFORM_RET_FAILED;
	}

	switch (self->req.which_content) {
		case WaveformRequest_start_recording_tag:
			process_start_recording(self, &self->req.content.start_recording);
			break;
		case WaveformRequest_stop_recording_tag:
			process_stop_recording(self, &self->req.content.stop_recording);
			break;
		case WaveformRequest_trigger_tag:
			process_trigger(self, &self->req.content.trigger);
			break;
		case WaveformRequest_get_data_tag:
			process_get_data(self, &self->req.content.get_data);
			break;
		default:
			return UXB_WAVEFORM_RET_FAILED;
	}

	return UXB_WAVEFORM_RET_OK;
}
