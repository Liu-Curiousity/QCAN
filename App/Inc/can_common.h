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

#include "compiler.h"
#include "main.h"

#define CAN_ERROR_WARNING_THRESHOLD	 96
#define CAN_ERROR_PASSIVE_THRESHOLD	 128
#define CAN_BUS_OFF_THRESHOLD		 256

#define CAN_BUS_OFF_RESTART_DELAY_MS 100

#define CAN_LEC_NO_ERROR	0
#define CAN_LEC_STUFF_ERROR 1
#define CAN_LEC_FORM_ERROR	2
#define CAN_LEC_ACK_ERROR	3
#define CAN_LEC_REC_ERROR	4
#define CAN_LEC_DOM_ERROR	5
#define CAN_LEC_CRC_ERROR	6
#define CAN_LEC_SOFTWARE	7

#ifdef __cplusplus

#ifdef CONFIG_CANFD
#define GS_HOST_FRAME_SIZE struct_size((struct gs_host_frame *)NULL, canfd_ts, 1)
#else
#define GS_HOST_FRAME_SIZE struct_size((struct gs_host_frame *)NULL, classic_can_ts, 1)
#endif

#define new new_
#include "list.h"
#undef new
#include "gs_usb.h"

extern const gs_device_bt_const CAN_btconst;
extern const gs_device_bt_const_extended CAN_btconst_ext;

struct gs_host_frame_object {
    list_head list;

    union {
        uint8_t _buf[GS_HOST_FRAME_SIZE];
        gs_host_frame frame;
    };
};

class CAN_Channel {
public:
    virtual ~CAN_Channel() = default;

    CAN_Channel(uint8_t nr) : nr(nr) {
        INIT_LIST_HEAD(&list_from_host);
    }

    list_head list_from_host{}; // 待发送到CAN总线的帧队列
    uint32_t feature{};
    gs_can_state state{GS_CAN_STATE_STOPPED};
    uint32_t bus_off_restart{};
    gs_device_bittiming bittiming{};
    gs_device_bittiming data_bittiming{};
    uint8_t nr;
    virtual void get_device_state(gs_device_state *state) = 0;

    bool set_bittiming(const gs_device_bittiming *timing);

    bool set_data_bittiming(const gs_device_bittiming *timing);

    [[nodiscard]] bool is_enabled() const;

    bool enable(uint32_t feature_);

    void disable(list_head *plist_to_host, list_head *plist_frame_pool);

    void CAN_SendFrame(list_head *plist_to_host);

    void CAN_HandleError(list_head *plist_to_host, list_head *plist_frame_pool);

    static bool can_is_lec_error(uint8_t lec);

private:
    virtual void drv_enable() = 0;
    virtual void drv_disable() = 0;
    virtual bool is_rx_pending() = 0;
    virtual bool can_send(gs_host_frame *frame) = 0;
    virtual gs_can_state drv_get_state() = 0;
    virtual void can_drv_handle_state_change(gs_host_frame *frame) = 0;
    virtual bool can_drv_handle_bus_error(gs_host_frame *frame) = 0;
    virtual void can_drv_handle_bus_off_recovery() = 0;
    virtual void can_drv_read_reg_status() = 0;
    virtual bool can_drv_bus_error_pending() = 0;

    static bool check_bittiming_ok(const can_bittiming_const *btc, const gs_device_bittiming *timing);

    void can_schedule_bus_off_recovery(uint32_t delay_ms);

    static gs_host_frame_object* gs_host_frame_object_get_locked(const list_head *plist);

    bool can_state_change_pending();

    void can_handle_state_change(list_head *plist_to_host, list_head *plist_frame_pool);

    void can_prepare_error_frame(gs_host_frame *frame) const;

    bool can_bus_error_pending();

    void can_handle_bus_error(list_head *plist_to_host, list_head *plist_frame_pool);

    bool can_bus_off_recovery_pending() const;

    void can_handle_bus_off_recovery(list_head *plist_to_host, list_head *plist_frame_pool);
};

#endif
