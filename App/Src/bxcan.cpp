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
#include "timer.h"

void bxcan_::drv_enable()
{
	HAL_CAN_DeInit(hcan);

	// Completely reset while being of the bus
	__HAL_RCC_CAN1_FORCE_RESET();
	__HAL_RCC_CAN1_RELEASE_RESET();
	hcan->Init.Prescaler = bittiming.brp;
	hcan->Init.Mode = (feature & GS_CAN_FEATURE_LISTEN_ONLY ? CAN_MODE_SILENT : CAN_MODE_NORMAL) |
	                  (feature & GS_CAN_FEATURE_LOOP_BACK ? CAN_MODE_LOOPBACK : CAN_MODE_NORMAL);
	hcan->Init.SyncJumpWidth = FIELD_PREP(CAN_BTR_SJW, bittiming.sjw - 1);
	hcan->Init.TimeSeg1 = FIELD_PREP(CAN_BTR_TS1, bittiming.prop_seg + bittiming.phase_seg1 - 1);
	hcan->Init.TimeSeg2 = FIELD_PREP(CAN_BTR_TS2, bittiming.phase_seg2 - 1);
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
		.FilterIdHigh = 0x0000,
		.FilterIdLow = 0x0000,
		.FilterMaskIdHigh = 0x0000,
		.FilterMaskIdLow = 0x0000,
		.FilterFIFOAssignment = CAN_RX_FIFO0,
		.FilterBank = 0,
		.FilterMode = CAN_FILTERMODE_IDMASK,
		.FilterScale = CAN_FILTERSCALE_32BIT,
		.FilterActivation = ENABLE,
	};
	if (HAL_CAN_ConfigFilter(hcan, &filter_config) != HAL_OK) {
		Error_Handler();
	}

	if (HAL_CAN_Start(hcan) != HAL_OK ||
	    HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
		Error_Handler();
	}
}

void bxcan_::drv_disable() {
	HAL_CAN_Stop(hcan);
}

bool bxcan_::is_rx_pending() {
	return (hcan->Instance->RF0R & CAN_RF0R_FMP0) != 0;
}

bool bxcan_::can_send(gs_host_frame *frame) {
	CAN_TxHeaderTypeDef tx_header;
	uint32_t tx_mailbox;

	tx_header.StdId = frame->can_id & 0x7FF;
	tx_header.ExtId = frame->can_id & 0x1FFFFFFF;
	tx_header.IDE = frame->can_id & CAN_EFF_FLAG ? CAN_ID_EXT : CAN_RTR_DATA;
	tx_header.RTR = frame->can_id & CAN_RTR_FLAG ? CAN_RTR_REMOTE : CAN_ID_STD;
	tx_header.DLC = frame->can_dlc;
	tx_header.TransmitGlobalTime = DISABLE;

	const HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(hcan, &tx_header, frame->classic_can->data, &tx_mailbox);
	return status == HAL_OK ? true : false;
}

void bxcan_::can_drv_read_reg_status() {
	reg_status.esr = hcan->Instance->ESR;

	rx_error_count = FIELD_GET(CAN_ESR_REC, reg_status.esr);
	tx_error_count = FIELD_GET(CAN_ESR_TEC, reg_status.esr);
	const uint8_t lec = FIELD_GET(CAN_ESR_LEC, reg_status.esr);
	error_pending = lec != CAN_LEC_NO_ERROR && lec != CAN_LEC_SOFTWARE;

	if (error_pending) hcan->Instance->ESR |= FIELD_PREP(CAN_ESR_LEC, CAN_LEC_SOFTWARE);
}

void bxcan_::can_drv_handle_bus_error(gs_host_frame *frame) {
	frame->classic_can->data[6] = tx_error_count;
	frame->classic_can->data[7] = rx_error_count;

	frame->can_id |= CAN_ERR_PROT | CAN_ERR_BUSERROR | CAN_ERR_CNT;

	switch (FIELD_GET(CAN_ESR_LEC, reg_status.esr)) {
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

gs_can_state bxcan_::drv_get_state() {
	const uint32_t reg_esr = reg_status.esr;

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

void bxcan_::can_drv_handle_bus_off_recovery() {
	drv_disable();
	drv_enable();
}

void bxcan_::get_device_state(gs_device_state *state) {
	state->state = drv_get_state();
	state->rxerr = rx_error_count;
	state->txerr = tx_error_count;
}
