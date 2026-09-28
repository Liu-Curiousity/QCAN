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

#pragma once

#include <stdint.h>

#include "bxcan.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CAN_LEC_NO_ERROR	0
#define CAN_LEC_STUFF_ERROR 1
#define CAN_LEC_FORM_ERROR	2
#define CAN_LEC_ACK_ERROR	3
#define CAN_LEC_REC_ERROR	4
#define CAN_LEC_DOM_ERROR	5
#define CAN_LEC_CRC_ERROR	6
#define CAN_LEC_SOFTWARE	7

#ifdef __cplusplus
}

#ifdef CONFIG_CANFD
#define GS_HOST_FRAME_SIZE struct_size((struct gs_host_frame *)NULL, canfd_ts, 1)
#else
#define GS_HOST_FRAME_SIZE struct_size((struct gs_host_frame *)NULL, classic_can_ts, 1)
#endif

struct gs_host_frame_object {
    list_head list;

    union {
        uint8_t _buf[GS_HOST_FRAME_SIZE];
        gs_host_frame frame;
    };
};

bool can_set_bittiming(can_channel *channel, const gs_device_bittiming *timing);
bool can_set_data_bittiming(can_channel *channel, const gs_device_bittiming *timing);

bool can_is_enabled(const can_channel *channel);
bool can_enable(can_channel *channel, uint32_t feature);
void can_disable(list_head *plist_to_host, list_head *plist_frame_pool, can_channel *channel);
void can_get_device_state(const can_channel *channel, gs_device_state *state);

gs_can_state can_err_to_state(uint16_t err);
uint8_t gs_can_tx_state_to_frame(const gs_can_state state);
uint8_t gs_can_rx_state_to_frame(const gs_can_state state);
void can_lec_error_to_frame(gs_host_frame *frame, uint8_t lec);

void can_schedule_bus_off_recovery(can_channel *channel, uint32_t delay_ms);

void CAN_SendFrame(list_head *plist_to_host, can_data_t *channel);
void CAN_ReceiveFrame(list_head *plist_to_host, list_head *plist_frame_pool, can_data_t *channel);
void CAN_HandleError(list_head *plist_to_host, list_head *plist_frame_pool, can_data_t *channel);
#endif
