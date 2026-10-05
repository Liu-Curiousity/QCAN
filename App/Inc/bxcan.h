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

#ifdef __cplusplus

#include "gs_usb.h"
#include "main.h"
#include "can_common.h"

class bxcan_ : public CAN_Channel {
public:
    bxcan_(const uint8_t nr, CAN_HandleTypeDef *hcan, signal_led& led,
           const gs_device_bt_const& bt_const, gs_device_bt_const_extended bt_const_ext = {}) :
        CAN_Channel(nr, led, bt_const, bt_const_ext), hcan(hcan) {}

private:
    constexpr static auto CAN_LEC_NO_ERROR = 0;
    constexpr static auto CAN_LEC_STUFF_ERROR = 1;
    constexpr static auto CAN_LEC_FORM_ERROR = 2;
    constexpr static auto CAN_LEC_ACK_ERROR = 3;
    constexpr static auto CAN_LEC_REC_ERROR = 4;
    constexpr static auto CAN_LEC_DOM_ERROR = 5;
    constexpr static auto CAN_LEC_CRC_ERROR = 6;
    constexpr static auto CAN_LEC_SOFTWARE = 7;

    CAN_HandleTypeDef *hcan;
    uint32_t esr{};

    void drv_enable() override;
    void drv_disable() override;
    bool drv_is_rx_pending() override;
    bool drv_send(gs_host_frame *frame) override;
    void drv_handle_bus_error(gs_host_frame *frame) override;
    void drv_handle_bus_off_recovery() override;
    gs_can_state drv_read_status() override;
};
#endif
