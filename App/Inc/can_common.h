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

#define CAN_ERROR_WARNING_THRESHOLD	 96
#define CAN_ERROR_PASSIVE_THRESHOLD	 128
#define CAN_BUS_OFF_THRESHOLD		 256

#ifdef __cplusplus

#ifdef CONFIG_CANFD
#define GS_HOST_FRAME_SIZE struct_size((struct gs_host_frame *)NULL, canfd_ts, 1)
#else
#define GS_HOST_FRAME_SIZE struct_size((struct gs_host_frame *)NULL, classic_can_ts, 1)
#endif

#include "gs_usb.h"
#include "led.h"
#include "FreeRTOS.h"
#include "queue.h"

struct gs_host_frame_object {
    union {
        uint8_t _buf[GS_HOST_FRAME_SIZE];
        gs_host_frame frame;
    };
};

class CAN_Channel {
public:
    virtual ~CAN_Channel() {
        vQueueDelete(xQueue_from_host);
    };

    CAN_Channel(const uint8_t nr, const signal_led& led, const gs_device_bt_const& bt_const,
                gs_device_bt_const_extended bt_const_ext = {}) :
        nr(nr), led(led), bt_const(bt_const), bt_const_ext(bt_const_ext) {
        xQueue_from_host = xQueueCreate(20, GS_HOST_FRAME_SIZE);
    }

    xQueueHandle xQueue_from_host; // 待发送到CAN总线的帧队列
    uint32_t feature{};
    uint8_t nr;
    signal_led led;
    gs_device_bt_const bt_const;
    gs_device_bt_const_extended bt_const_ext;

    gs_device_state get_device_state() const;

    bool set_bittiming(const gs_device_bittiming *timing);

    bool set_data_bittiming(const gs_device_bittiming *timing);

    [[nodiscard]] bool is_enabled() const;

    bool enable(uint32_t feature);

    void disable();

    void CAN_SendFrame(xQueueHandle xQueue_to_host);

    void CAN_HandleError(xQueueHandle xQueue_to_host);

protected:
    uint8_t tx_error_count{};
    uint8_t rx_error_count{};
    bool error_pending{};
    gs_device_bittiming bittiming{};
    gs_device_bittiming data_bittiming{};

private:
    constexpr static uint32_t CAN_CHANNEL_BUS_OFF_RESTART_DISABLED = 0;
    constexpr static uint32_t CAN_BUS_OFF_RESTART_DELAY_MS = 100;

    uint32_t bus_off_restart{CAN_CHANNEL_BUS_OFF_RESTART_DISABLED};
    gs_can_state state{GS_CAN_STATE_STOPPED};

    virtual void drv_enable() = 0;
    virtual void drv_disable() = 0;
    virtual bool drv_is_rx_pending() = 0;
    virtual bool drv_send(gs_host_frame *frame) = 0;
    virtual void drv_handle_bus_error(gs_host_frame *frame) = 0;
    virtual void drv_handle_bus_off_recovery() = 0;
    virtual gs_can_state drv_read_status() = 0;

    static bool check_bittiming_ok(const can_bittiming_const *btc, const gs_device_bittiming *timing);

    uint8_t controller_error_flags() const;

    void schedule_bus_off_recovery(uint32_t delay_ms);

    bool bus_off_recovery_pending() const;
};

#endif
