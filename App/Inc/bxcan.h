/*
 * The MIT License (MIT)
 *
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
#include <stdbool.h>

#include "config.h"
#include "gs_usb.h"
#include "main.h"
// #include "led.h"
#define new new_
#include "list.h"
#undef new

#ifdef __cplusplus
extern "C" {
#endif

struct can_drv_reg_status {
#if defined (CONFIG_BXCAN)
	uint32_t esr;
#elif defined(CONFIG_M_CAN)
	uint32_t ecr;
	uint32_t psr;
#endif
};

#define CAN_CHANNEL_BUS_OFF_RESTART_DISABLED 0

typedef struct can_channel {
#if defined (CONFIG_BXCAN)
	CAN_HandleTypeDef *hcan;
#elif defined(CONFIG_M_CAN)
	FDCAN_HandleTypeDef *hfdcan;
#endif
	struct can_drv_reg_status reg_status;
	struct list_head list_from_host;		// 待发送到CAN总线的帧队列
	uint32_t feature;
	enum gs_can_state state;
	uint32_t bus_off_restart;
	struct gs_device_bittiming bittiming;
#ifdef CONFIG_CANFD
	struct gs_device_bittiming data_bittiming;
#endif
	uint8_t nr;
} can_data_t;

extern const struct gs_device_bt_const CAN_btconst;
extern const struct gs_device_bt_const_extended CAN_btconst_ext;

bool can_receive(can_data_t *channel, struct gs_host_frame *rx_frame);
bool can_is_rx_pending(can_data_t *channel);

bool can_send(can_data_t *channel, struct gs_host_frame *frame);

void can_drv_enable(struct can_channel *channel);
void can_drv_disable(struct can_channel *channel);

void can_drv_read_reg_status(struct can_channel *channel);

bool can_drv_bus_error_pending(const struct can_channel *channel);
bool can_drv_handle_bus_error(const struct can_channel *channel, struct gs_host_frame *frame);

enum gs_can_state can_drv_get_state(const struct can_channel *channel);
void can_drv_handle_state_change(const struct can_channel *channel, struct gs_host_frame *frame);

void can_drv_handle_bus_off_recovery(struct can_channel *channel);

#ifdef __cplusplus
}
#endif
