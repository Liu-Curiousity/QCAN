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

#include "config.h"
#include "gs_usb.h"
#include "main.h"
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

#ifdef __cplusplus
}
#include "can_common.h"
class bxcan_ : public CAN_Channel {
public:
	bxcan_(CAN_HandleTypeDef *hcan, const uint8_t nr) : CAN_Channel(nr), hcan(hcan) {}

	gs_can_state err_to_state(const uint16_t err) {
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

	void can_drv_read_reg_status();

	bool can_drv_bus_error_pending();

private:
	CAN_HandleTypeDef *hcan;
	can_drv_reg_status reg_status{};

	void drv_enable() override;
	void drv_disable() override;
	bool is_rx_pending() override;
	bool can_send(gs_host_frame *frame) override;
	bool can_drv_handle_bus_error(gs_host_frame *frame) override;
	gs_can_state drv_get_state() override;
	void can_drv_handle_state_change(gs_host_frame *frame) override;
	void can_drv_handle_bus_off_recovery() override;
	void get_device_state(gs_device_state *state) override;
};
#endif
