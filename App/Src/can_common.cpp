/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2026 Marc Kleine-Budde <kernel@pengutronix.de>
 * Copyright (c) 2016 Hubert Denkmair
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#include "config.h"
#include "can_common.h"
#include "led.h"
#include "timer.h"

#ifndef CONFIG_CANFD
const gs_device_bt_const_extended CAN_btconst_ext{};
#endif

#define CAN_CHANNEL_BUS_OFF_RESTART_DISABLED 0

const gs_device_bt_const CAN_btconst = {
	.feature =
		GS_CAN_FEATURE_LISTEN_ONLY |
		GS_CAN_FEATURE_LOOP_BACK |
		GS_CAN_FEATURE_ONE_SHOT |
		GS_CAN_FEATURE_HW_TIMESTAMP |
		GS_CAN_FEATURE_IDENTIFY |
		GS_CAN_FEATURE_PAD_PKTS_TO_MAX_PKT_SIZE |
		(IS_ENABLED(CONFIG_TERMINATION) ?
		 GS_CAN_FEATURE_TERMINATION : 0) |
		GS_CAN_FEATURE_BERR_REPORTING |
		GS_CAN_FEATURE_GET_STATE |
		0,
	.fclk_can = CAN_CLOCK_SPEED,
	.btc = {
		.tseg1_min = 1,
		.tseg1_max = 16,
		.tseg2_min = 1,
		.tseg2_max = 8,
		.sjw_max = 4,
		.brp_min = 1,
		.brp_max = 1024,
		.brp_inc = 1,
	},
};

bool CAN_Channel::check_bittiming_ok(const can_bittiming_const *btc, const gs_device_bittiming *timing) {
	const uint32_t tseg1 = timing->prop_seg + timing->phase_seg1;

	if (tseg1 < btc->tseg1_min ||
	    tseg1 > btc->tseg1_max ||
	    timing->phase_seg2 < btc->tseg2_min ||
	    timing->phase_seg2 > btc->tseg2_max ||
	    timing->sjw > btc->sjw_max ||
	    timing->brp < btc->brp_min ||
	    timing->brp > btc->brp_max)
		return false;

	return true;
}

void CAN_Channel::can_schedule_bus_off_recovery(const uint32_t delay_ms) {
	bus_off_restart = HAL_GetTick() + delay_ms;

	if (bus_off_restart == CAN_CHANNEL_BUS_OFF_RESTART_DISABLED)
		bus_off_restart++;
}

bool CAN_Channel::set_bittiming(const gs_device_bittiming *timing) {
	if (!check_bittiming_ok(&CAN_btconst.btc, timing)) return false;
	bittiming = *timing;
	return true;
}

bool CAN_Channel::set_data_bittiming(const gs_device_bittiming *timing) {
	if (!check_bittiming_ok(&CAN_btconst.btc, timing)) return false;
#ifdef CONFIG_CANFD
	channel->data_bittiming = *timing;
	return true;
#endif
	return false;
}

bool CAN_Channel::is_enabled() const {
	return state < GS_CAN_STATE_STOPPED;
}

bool CAN_Channel::enable(const uint32_t feature_) {
	xQueueReset(xQueue_from_host);
	led.mode = signal_led::Mode::NORMAL;
	feature = feature_;
	state = GS_CAN_STATE_ERROR_ACTIVE;
	drv_enable();
	return true;
}

void CAN_Channel::disable() {
	drv_disable();
	bus_off_restart = CAN_CHANNEL_BUS_OFF_RESTART_DISABLED;
	state = GS_CAN_STATE_STOPPED;
	feature = 0;

	led.mode = signal_led::Mode::OFF;
}

void CAN_Channel::CAN_SendFrame() {
	static bool datain_request = false;
	gs_host_frame_object rx_frame{};
	if (!datain_request && xQueueReceive(xQueue_from_host, rx_frame._buf, 0) == pdPASS) {
		datain_request = true;
		led.blink_request(signal_led::LED_TX);
	}
	if (datain_request == true) {
		if (can_send(&rx_frame.frame)) {
			datain_request = false;
		}
	}
	// Echo sent frame back to host
	rx_frame.frame.reserved = 0x0;
	if (IS_ENABLED(CONFIG_CANFD) && rx_frame.frame.flags & GS_CAN_FLAG_FD)
		rx_frame.frame.canfd_ts->timestamp_us = timer_get();
	else
		rx_frame.frame.classic_can_ts->timestamp_us = timer_get();
	// TODO: 返回给上位机
	// list_add_tail_locked(&frame_object->list, plist_to_host);
}

void CAN_Channel::CAN_HandleError(xQueueHandle xQueue_to_host) {
	if (is_rx_pending()) return;

	can_drv_read_reg_status();

	if (can_state_change_pending()) {
		can_handle_state_change(xQueue_to_host);
	} else if (can_bus_error_pending()) {
		can_handle_bus_error(xQueue_to_host);
	} else if (can_bus_off_recovery_pending()) {
		can_handle_bus_off_recovery(xQueue_to_host);
	}
}

bool CAN_Channel::can_is_lec_error(const uint8_t lec) {
	if (lec == CAN_LEC_NO_ERROR || lec == CAN_LEC_SOFTWARE)
		return false;

	return true;
}

bool CAN_Channel::can_state_change_pending() {
	if (!is_enabled()) return false;

	const gs_can_state new_state = drv_get_state();
	if (state == new_state) return false;

	state = new_state;

	return true;
}


void CAN_Channel::can_handle_state_change(xQueueHandle xQueue_to_host) {
	gs_host_frame_object frame_object;

	gs_host_frame *frame = &frame_object.frame;
	can_prepare_error_frame(frame);

	if (state == GS_CAN_STATE_BUS_OFF) {
		frame->can_id |= CAN_ERR_BUSOFF;

		/* host is not taking care of CAN bus of recovery */
		can_schedule_bus_off_recovery(CAN_BUS_OFF_RESTART_DELAY_MS);
	} else {
		frame->can_id |= CAN_ERR_CRTL | CAN_ERR_CNT;
		can_drv_handle_state_change(frame);
		can_drv_handle_bus_error(frame);
	}
	xQueueSendToBack(xQueue_to_host, &frame_object._buf, 0);
}

void CAN_Channel::can_prepare_error_frame(gs_host_frame *frame) const {
	frame->echo_id = GS_HOST_FRAME_ECHO_ID_RX;
	frame->can_id = CAN_ERR_FLAG;
	frame->can_dlc = CAN_ERR_DLC;
	frame->channel = nr;
	frame->flags = 0;
	frame->reserved = 0;
	*frame->classic_can = classic_can{};

	frame->classic_can_ts->timestamp_us = timer_get();
}

bool CAN_Channel::can_bus_error_pending() {
	if (!(feature & GS_CAN_FEATURE_BERR_REPORTING))
		return false;

	return can_drv_bus_error_pending();
}

void CAN_Channel::can_handle_bus_error(xQueueHandle xQueue_to_host) {
	gs_host_frame_object frame_object{};

	gs_host_frame *frame = &frame_object.frame;

	can_prepare_error_frame(frame);
	if (can_drv_handle_bus_error(frame)) {
		xQueueSendToBack(xQueue_to_host, &frame_object._buf, 0);
	}
}

bool CAN_Channel::can_bus_off_recovery_pending() const {
	if (bus_off_restart == CAN_CHANNEL_BUS_OFF_RESTART_DISABLED)
		return false;

	const uint32_t now = HAL_GetTick();

	return static_cast<int32_t>(now - bus_off_restart) >= 0;
}

void CAN_Channel::can_handle_bus_off_recovery(xQueueHandle xQueue_to_host) {
	can_drv_handle_bus_off_recovery();

	bus_off_restart = CAN_CHANNEL_BUS_OFF_RESTART_DISABLED;

	gs_host_frame_object frame_object{};

	gs_host_frame *frame = &frame_object.frame;
	can_prepare_error_frame(frame);

	frame->can_id |= CAN_ERR_RESTARTED;

	xQueueSendToBack(xQueue_to_host, &frame_object._buf, 0);
}
