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

#include "can_common.h"
#include "led.h"
#include "timer.h"

#ifndef CONFIG_CANFD
const struct gs_device_bt_const_extended CAN_btconst_ext{};
#endif

static gs_host_frame_object* gs_host_frame_object_get_locked(list_head *plist) {
	const bool was_irq_enabled = disable_irq();
	gs_host_frame_object *frame_object =
		list_first_entry_or_null(plist, struct gs_host_frame_object, list);

	if (frame_object) list_del(&frame_object->list);

	restore_irq(was_irq_enabled);
	return frame_object;
}

static bool can_check_bittiming_ok(const struct can_bittiming_const *btc,
                                   const struct gs_device_bittiming *timing)
{
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

bool can_set_bittiming(struct can_channel *channel, const struct gs_device_bittiming *timing) {
	if (!can_check_bittiming_ok(&CAN_btconst.btc, timing)) return false;
	channel->bittiming = *timing;
	return true;
}

bool can_set_data_bittiming(struct can_channel *channel, const struct gs_device_bittiming *timing) {
	if (!can_check_bittiming_ok(&CAN_btconst.btc, timing)) return false;
#ifdef CONFIG_CANFD
	channel->data_bittiming = *timing;
	return true;
#endif
	return false;
}

bool can_is_enabled(const struct can_channel *channel)
{
	return channel->state < GS_CAN_STATE_STOPPED;
}

bool can_enable(can_channel *channel, const uint32_t feature) {
	led.mode = signal_led::Mode::NORMAL;

	channel->feature = feature;
	channel->state = GS_CAN_STATE_ERROR_ACTIVE;

	can_drv_enable(channel);
	return true;
}

static void usbd_gs_can_purge_from_host_list_by_channel(list_head *plist, can_channel *channel) {
	list_splice_tail_init_locked(&channel->list_from_host, plist);
}

static void usbd_gs_can_purge_to_host_list_by_channel(list_head *plist_to_host,list_head *plist_frame_pool, const can_channel *channel) {
	/*
	 * If we only have one channel, all object of hcan->list_to_host belong to this one channel.
	 * Move the complete list to the frame pool.
	 */
	if (NUM_CAN_CHANNEL == 1) {
		list_splice_tail_init_locked(plist_to_host, plist_frame_pool);

		return;
	}

	/*
	 * For more than one channel, iterate over each object in hcan->list_to_host.
	 */
	gs_host_frame_object *iter, *next;
	const bool was_irq_enabled = disable_irq();

	list_for_each_entry_safe(iter, next, plist_to_host, list) {
		if (iter->frame.channel == channel->nr) {
			list_move_tail(&iter->list, plist_frame_pool);
		}
	}

	restore_irq(was_irq_enabled);
}

void can_disable(list_head *plist_to_host,list_head *plist_frame_pool, can_channel *channel) {
	can_drv_disable(channel);

	usbd_gs_can_purge_from_host_list_by_channel(plist_frame_pool, channel);
	usbd_gs_can_purge_to_host_list_by_channel(plist_to_host, plist_frame_pool, channel);

	channel->bus_off_restart = CAN_CHANNEL_BUS_OFF_RESTART_DISABLED;
	channel->state = GS_CAN_STATE_STOPPED;
	channel->feature = 0;

	led.mode = signal_led::Mode::OFF;
}

void CAN_SendFrame(list_head *plist_to_host, can_data_t *channel) {
	const bool was_irq_enabled = disable_irq();
	gs_host_frame_object *frame_object =
		list_first_entry_or_null(&channel->list_from_host, struct gs_host_frame_object, list);

	if (!frame_object) {
		restore_irq(was_irq_enabled);
		return;
	}

	list_del(&frame_object->list);
	restore_irq(was_irq_enabled);

	gs_host_frame *frame = &frame_object->frame;

	if (!can_send(channel, frame)) {
		list_add_locked(&frame_object->list, &channel->list_from_host);
		return;
	}

	// Echo sent frame back to host
	frame->reserved = 0x0;
	if (IS_ENABLED(CONFIG_CANFD) && frame->flags & GS_CAN_FLAG_FD)
		frame->canfd_ts->timestamp_us = timer_get();
	else
		frame->classic_can_ts->timestamp_us = timer_get();

	list_add_tail_locked(&frame_object->list, plist_to_host);

	led.blink_request(signal_led::LED_TX);
}

void CAN_ReceiveFrame(list_head *plist_to_host,list_head *plist_frame_pool, can_data_t *channel) {
	gs_host_frame_object *frame_object;

	if (!can_is_rx_pending(channel)) return;

	frame_object = gs_host_frame_object_get_locked(plist_frame_pool);
	if (!frame_object) return;

	gs_host_frame *frame = &frame_object->frame;

	if (!can_receive(channel, frame)) {
		list_add_tail_locked(&frame_object->list, plist_frame_pool);
		return;
	}

	frame->echo_id = GS_HOST_FRAME_ECHO_ID_RX; // not an echo frame
	frame->reserved = 0;

	list_add_tail_locked(&frame_object->list, plist_to_host);

	led.blink_request(signal_led::LED_RX);
}

void can_get_device_state(const can_channel *channel, gs_device_state *state) {
	const uint32_t reg_esr = channel->reg_status.esr;

	state->state = can_drv_get_state(channel);
	state->rxerr = FIELD_GET(CAN_ESR_REC, reg_esr);
	state->txerr = FIELD_GET(CAN_ESR_TEC, reg_esr);
}

static void can_prepare_error_frame(const can_channel *channel, gs_host_frame *frame) {
	frame->echo_id = GS_HOST_FRAME_ECHO_ID_RX;
	frame->can_id = CAN_ERR_FLAG;
	frame->can_dlc = CAN_ERR_DLC;
	frame->channel = channel->nr;
	frame->flags = 0;
	frame->reserved = 0;
	*frame->classic_can = classic_can{0};

	frame->classic_can_ts->timestamp_us = timer_get();
}

gs_can_state can_err_to_state(const uint16_t err) {
	if (err < CAN_ERROR_WARNING_THRESHOLD)
		return GS_CAN_STATE_ERROR_ACTIVE;
	if (err < CAN_ERROR_PASSIVE_THRESHOLD)
		return GS_CAN_STATE_ERROR_WARNING;
	if (err < CAN_BUS_OFF_THRESHOLD)
		return GS_CAN_STATE_ERROR_PASSIVE;

	return GS_CAN_STATE_BUS_OFF;
}

uint8_t gs_can_tx_state_to_frame(const gs_can_state state) {
	switch (state) {
		case GS_CAN_STATE_ERROR_ACTIVE:
			return CAN_ERR_CRTL_ACTIVE;
		case GS_CAN_STATE_ERROR_WARNING:
			return CAN_ERR_CRTL_TX_WARNING;
		case GS_CAN_STATE_ERROR_PASSIVE:
			return CAN_ERR_CRTL_TX_PASSIVE;
		default:
			return 0;
	}
}

uint8_t gs_can_rx_state_to_frame(const gs_can_state state) {
	switch (state) {
		case GS_CAN_STATE_ERROR_ACTIVE:
			return CAN_ERR_CRTL_ACTIVE;
		case GS_CAN_STATE_ERROR_WARNING:
			return CAN_ERR_CRTL_RX_WARNING;
		case GS_CAN_STATE_ERROR_PASSIVE:
			return CAN_ERR_CRTL_RX_PASSIVE;
		default:
			return 0;
	}
}

void can_lec_error_to_frame(gs_host_frame *frame, const uint8_t lec) {
	switch (lec) {
		case CAN_LEC_STUFF_ERROR:
			frame->classic_can->data[2] |= CAN_ERR_PROT_STUFF;
			break;
		case CAN_LEC_FORM_ERROR:
			frame->classic_can->data[2] |= CAN_ERR_PROT_FORM;
			break;
		case CAN_LEC_ACK_ERROR:
			frame->can_id |= CAN_ERR_ACK;
			break;
		case CAN_LEC_REC_ERROR:
			frame->classic_can->data[2] |= CAN_ERR_PROT_BIT1;
			break;
		case CAN_LEC_DOM_ERROR:
			frame->classic_can->data[2] |= CAN_ERR_PROT_BIT0;
			break;
		case CAN_LEC_CRC_ERROR:
			frame->classic_can->data[3] |= CAN_ERR_PROT_LOC_CRC_SEQ;
			break;
		default:
			break;
	}
}

static void can_handle_bus_error(list_head *plist_to_host, list_head *plist_frame_pool, const can_channel *channel) {
	gs_host_frame_object *frame_object = gs_host_frame_object_get_locked(plist_frame_pool);
	if (!frame_object) return;

	gs_host_frame *frame = &frame_object->frame;

	can_prepare_error_frame(channel, frame);
	bool handled = can_drv_handle_bus_error(channel, frame);
	if (handled) {
		list_add_tail_locked(&frame_object->list, plist_to_host);
	} else {
		list_add_locked(&frame_object->list, plist_frame_pool);
	}
}

static bool can_bus_error_pending(const can_channel *channel) {
	if (!(channel->feature & GS_CAN_FEATURE_BERR_REPORTING))
		return false;

	return can_drv_bus_error_pending(channel);
}

void can_schedule_bus_off_recovery(can_channel *channel, const uint32_t delay_ms) {
	channel->bus_off_restart = HAL_GetTick() + delay_ms;

	if (channel->bus_off_restart == CAN_CHANNEL_BUS_OFF_RESTART_DISABLED)
		channel->bus_off_restart++;
}

static void can_handle_state_change(list_head *plist_to_host, list_head *plist_frame_pool, can_channel *channel) {
	gs_host_frame_object *frame_object = gs_host_frame_object_get_locked(plist_frame_pool);
	if (!frame_object) return;

	gs_host_frame *frame = &frame_object->frame;
	can_prepare_error_frame(channel, frame);

	if (channel->state == GS_CAN_STATE_BUS_OFF) {
		frame->can_id |= CAN_ERR_BUSOFF;

		/* host is not taking care of CAN bus of recovery */
		can_schedule_bus_off_recovery(channel, CAN_BUS_OFF_RESTART_DELAY_MS);
	} else {
		frame->can_id |= CAN_ERR_CRTL | CAN_ERR_CNT;
		can_drv_handle_state_change(channel, frame);
		can_drv_handle_bus_error(channel, frame);
	}

	list_add_tail_locked(&frame_object->list, plist_to_host);
}

static bool can_state_change_pending(can_channel *channel) {
	if (!can_is_enabled(channel)) return false;

	const gs_can_state new_state = can_drv_get_state(channel);
	if (channel->state == new_state) return false;

	channel->state = new_state;

	return true;
}

static void can_handle_bus_off_recovery(list_head *plist_to_host, list_head *plist_frame_pool, can_channel *channel) {
	can_drv_handle_bus_off_recovery(channel);

	channel->bus_off_restart = CAN_CHANNEL_BUS_OFF_RESTART_DISABLED;

	gs_host_frame_object *frame_object = gs_host_frame_object_get_locked(plist_frame_pool);
	if (!frame_object)
		return;

	gs_host_frame *frame = &frame_object->frame;
	can_prepare_error_frame(channel, frame);

	frame->can_id |= CAN_ERR_RESTARTED;

	list_add_tail_locked(&frame_object->list, plist_to_host);
}

static bool can_bus_off_recovery_pending(const can_channel *channel) {
	if (channel->bus_off_restart == CAN_CHANNEL_BUS_OFF_RESTART_DISABLED)
		return false;

	const uint32_t now = HAL_GetTick();

	return channel->bus_off_restart - now < 0;
}

// If there are frames to receive, don't report any error frames. The
// best we can localize the errors to is "after the last successfully
// received frame", so wait until we get there. LEC will hold some error
// to report even if multiple pass by.
void CAN_HandleError(list_head *plist_to_host, list_head *plist_frame_pool, can_data_t *channel) {
	if (can_is_rx_pending(channel)) return;

	can_drv_read_reg_status(channel);

	if (can_state_change_pending(channel)) {
		can_handle_state_change(plist_to_host, plist_frame_pool, channel);
	} else if (can_bus_error_pending(channel)) {
		can_handle_bus_error(plist_to_host, plist_frame_pool, channel);
	} else if (can_bus_off_recovery_pending(channel)) {
		can_handle_bus_off_recovery(plist_to_host, plist_frame_pool, channel);
	}
}
