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

#include "bxcan.h"
#include "can_common.h"
#include "config.h"
#include "gs_usb.h"
#include "main.h"
#include "timer.h"

const struct gs_device_bt_const CAN_btconst = {
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

void can_drv_enable(struct can_channel *channel)
{
	HAL_CAN_DeInit(channel->hcan);

	// Completely reset while being of the bus
	__HAL_RCC_CAN1_FORCE_RESET();
	__HAL_RCC_CAN1_RELEASE_RESET();
	CAN_HandleTypeDef *hcan = channel->hcan;
	const uint32_t feature = channel->feature;
	hcan->Init.Prescaler = channel->bittiming.brp;
	hcan->Init.Mode = (feature & GS_CAN_FEATURE_LISTEN_ONLY ? CAN_MODE_SILENT : CAN_MODE_NORMAL) |
	                  (feature & GS_CAN_FEATURE_LOOP_BACK ? CAN_MODE_LOOPBACK : CAN_MODE_NORMAL);
	hcan->Init.SyncJumpWidth = FIELD_PREP(CAN_BTR_SJW, channel->bittiming.sjw - 1);
	hcan->Init.TimeSeg1 = FIELD_PREP(CAN_BTR_TS1, channel->bittiming.prop_seg + channel->bittiming.phase_seg1 - 1);
	hcan->Init.TimeSeg2 = FIELD_PREP(CAN_BTR_TS2, channel->bittiming.phase_seg2 - 1);
	hcan->Init.TimeTriggeredMode = DISABLE;
	hcan->Init.AutoBusOff = DISABLE;
	hcan->Init.AutoWakeUp = DISABLE;
	hcan->Init.AutoRetransmission = feature & GS_CAN_FEATURE_ONE_SHOT ? DISABLE : ENABLE;
	hcan->Init.ReceiveFifoLocked = DISABLE;
	hcan->Init.TransmitFifoPriority = ENABLE;
	if (HAL_CAN_Init(hcan) != HAL_OK)
	{
		Error_Handler();
	}

	CAN_FilterTypeDef filter_config = {
		.FilterBank = 0,
		.FilterMode = CAN_FILTERMODE_IDMASK,
		.FilterScale = CAN_FILTERSCALE_32BIT,
		.FilterIdHigh = 0x0000,
		.FilterIdLow = 0x0000,
		.FilterMaskIdHigh = 0x0000,
		.FilterMaskIdLow = 0x0000,
		.FilterFIFOAssignment = CAN_RX_FIFO0,
		.FilterActivation = ENABLE,
	};
	if (HAL_CAN_ConfigFilter(channel->hcan, &filter_config) != HAL_OK) {
		Error_Handler();
	}

	HAL_CAN_Start(channel->hcan);
}

void can_drv_disable(struct can_channel *channel) {
	HAL_CAN_Stop(channel->hcan);
}

bool can_is_rx_pending(can_data_t *channel) {
	CAN_TypeDef *can = channel->hcan->Instance;

	return (can->RF0R & CAN_RF0R_FMP0) != 0;
}

bool can_receive(can_data_t *channel, struct gs_host_frame *rx_frame) {
	if (HAL_CAN_GetRxFifoFillLevel(channel->hcan,CAN_RX_FIFO0)) {
		CAN_RxHeaderTypeDef rx_header;
		HAL_CAN_GetRxMessage(channel->hcan,CAN_RX_FIFO0, &rx_header, rx_frame->classic_can->data);
		rx_frame->classic_can_ts->timestamp_us = timer_get();
		rx_frame->can_dlc = rx_header.DLC;
		rx_frame->can_id = rx_header.IDE & CAN_ID_EXT ? rx_header.ExtId : rx_header.StdId;
		if (rx_header.IDE & CAN_ID_EXT) rx_frame->can_id |= CAN_EFF_FLAG;
		if (rx_header.RTR & CAN_RTR_REMOTE) rx_frame->can_id |= CAN_RTR_REMOTE;
		rx_frame->channel = channel->nr;
		rx_frame->flags = 0;
		return true;
	} else {
		return false;
	}
}

bool can_send(can_data_t *channel, struct gs_host_frame *frame) {
	CAN_TxHeaderTypeDef tx_header;
	uint32_t tx_mailbox;

	tx_header.StdId = frame->can_id & 0x7FF;
	tx_header.ExtId = frame->can_id & 0x1FFFFFFF;
	tx_header.IDE = frame->can_id & CAN_EFF_FLAG ? CAN_ID_EXT : CAN_RTR_DATA;
	tx_header.RTR = frame->can_id & CAN_RTR_FLAG ? CAN_RTR_REMOTE : CAN_ID_STD;
	tx_header.DLC = frame->can_dlc;
	tx_header.TransmitGlobalTime = DISABLE;

	const HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(channel->hcan, &tx_header, frame->classic_can->data, &tx_mailbox);
	return status == HAL_OK ? true : false;
}

static inline bool can_is_lec_error(const uint8_t lec) {
	if (lec == CAN_LEC_NO_ERROR || lec == CAN_LEC_SOFTWARE)
		return false;

	return true;
}

bool can_drv_bus_error_pending(const struct can_channel *channel) {
	const uint32_t reg_esr = channel->reg_status.esr;
	const uint8_t lec = FIELD_GET(CAN_ESR_LEC, reg_esr);

	return can_is_lec_error(lec);
}

void can_drv_read_reg_status(struct can_channel *channel) {
	channel->reg_status.esr = channel->hcan->Instance->ESR;

	if (can_drv_bus_error_pending(channel)) {
		/* mark as handled by software */
		channel->hcan->Instance->ESR |= FIELD_PREP(CAN_ESR_LEC, CAN_LEC_SOFTWARE);
	}
}

bool can_drv_handle_bus_error(const struct can_channel *channel, struct gs_host_frame *frame) {
	const uint32_t reg_esr = channel->reg_status.esr;

	const uint8_t tx_err = FIELD_GET(CAN_ESR_TEC, reg_esr);
	const uint8_t rx_err = FIELD_GET(CAN_ESR_REC, reg_esr);

	if (tx_err == 0 && rx_err == 0) {
		return false;
	}

	frame->classic_can->data[6] = tx_err;
	frame->classic_can->data[7] = rx_err;

	frame->can_id |= CAN_ERR_PROT | CAN_ERR_BUSERROR | CAN_ERR_CNT;

	can_lec_error_to_frame(frame, FIELD_GET(CAN_ESR_LEC, reg_esr));

	return true;
}

enum gs_can_state can_drv_get_state(const struct can_channel *channel) {
	const uint32_t reg_esr = channel->reg_status.esr;

	if (!(reg_esr & (CAN_ESR_BOFF | CAN_ESR_EPVF | CAN_ESR_EWGF))) {
		return GS_CAN_STATE_ERROR_ACTIVE;
	}

	if (reg_esr & CAN_ESR_BOFF) {
		return GS_CAN_STATE_BUS_OFF;
	}

	if (reg_esr & CAN_ESR_EPVF) {
		return GS_CAN_STATE_ERROR_PASSIVE;
	}

	return GS_CAN_STATE_ERROR_WARNING;
}

void can_drv_handle_state_change(const struct can_channel *channel, struct gs_host_frame *frame) {
	const uint32_t reg_esr = channel->reg_status.esr;

	const uint8_t tx_err = FIELD_GET(CAN_ESR_TEC, reg_esr);
	const uint8_t rx_err = FIELD_GET(CAN_ESR_REC, reg_esr);

	const enum gs_can_state tx_state = can_err_to_state(tx_err);
	const enum gs_can_state rx_state = can_err_to_state(rx_err);

	if (tx_state >= rx_state) {
		frame->classic_can->data[1] |= gs_can_tx_state_to_frame(tx_state);
	}

	if (tx_state <= rx_state) {
		frame->classic_can->data[1] |= gs_can_rx_state_to_frame(rx_state);
	}
}

void can_drv_handle_bus_off_recovery(struct can_channel *channel) {
	can_drv_disable(channel);
	can_drv_enable(channel);
}
