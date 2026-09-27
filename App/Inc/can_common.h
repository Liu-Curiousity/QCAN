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

    CAN_Channel(CAN_HandleTypeDef *hcan, uint8_t nr) : hcan(hcan), nr(nr) {
        INIT_LIST_HEAD(&list_from_host);
    }

    CAN_HandleTypeDef *hcan;
    can_drv_reg_status reg_status{};
    list_head list_from_host{}; // 待发送到CAN总线的帧队列
    uint32_t feature{};
    gs_can_state state{GS_CAN_STATE_STOPPED};
    uint32_t bus_off_restart{};
    gs_device_bittiming bittiming{};
    gs_device_bittiming data_bittiming{};
    uint8_t nr;

    [[nodiscard]] bool is_enabled() const {
        return state < GS_CAN_STATE_STOPPED;
    }


    bool enable(const uint32_t feature_) {
        feature = feature_;
        state = GS_CAN_STATE_ERROR_ACTIVE;
        drv_enable();
        return true;
    }

    void disable(list_head *plist_to_host, list_head *plist_frame_pool) {
        drv_disable();

        list_splice_tail_init_locked(&list_from_host, plist_frame_pool);

        if (NUM_CAN_CHANNEL == 1) {
            list_splice_tail_init_locked(plist_to_host, plist_frame_pool);
            return;
        }
        gs_host_frame_object *iter, *next;
        const bool was_irq_enabled = disable_irq();

        list_for_each_entry_safe(iter, next, plist_to_host, list) {
            if (iter->frame.channel == nr) {
                list_move_tail(&iter->list, plist_frame_pool);
            }
        }
        restore_irq(was_irq_enabled);

        bus_off_restart = CAN_CHANNEL_BUS_OFF_RESTART_DISABLED;
        state = GS_CAN_STATE_STOPPED;
        feature = 0;
    }

    void get_device_state(gs_device_state *state) {
        const uint32_t reg_esr = reg_status.esr;

        state->state = drv_get_state();
        state->rxerr = FIELD_GET(CAN_ESR_REC, reg_esr);
        state->txerr = FIELD_GET(CAN_ESR_TEC, reg_esr);
    }

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

    void can_schedule_bus_off_recovery(const uint32_t delay_ms) {
        bus_off_restart = HAL_GetTick() + delay_ms;

        if (bus_off_restart == CAN_CHANNEL_BUS_OFF_RESTART_DISABLED)
            bus_off_restart++;
    }

    void CAN_SendFrame(list_head *plist_to_host) {
        bool was_irq_enabled = disable_irq();
        gs_host_frame_object *frame_object =
            list_first_entry_or_null(&list_from_host, struct gs_host_frame_object, list);

        if (!frame_object) {
            restore_irq(was_irq_enabled);
            return;
        }

        list_del(&frame_object->list);
        restore_irq(was_irq_enabled);

        gs_host_frame *frame = &frame_object->frame;

        if (!can_send(frame)) {
            list_add_locked(&frame_object->list, &list_from_host);
            return;
        }

        // Echo sent frame back to host
        frame->reserved = 0x0;
        if (IS_ENABLED(CONFIG_CANFD) && frame->flags & GS_CAN_FLAG_FD)
            // frame->canfd_ts->timestamp_us = timer_get();
            ;
        else
            // frame->classic_can_ts->timestamp_us = timer_get();
            ;

        list_add_tail_locked(&frame_object->list, plist_to_host);

        // led.blink_request(signal_led::LED_TX);
    }


    void CAN_ReceiveFrame(list_head *plist_to_host,list_head *plist_frame_pool) {
        gs_host_frame_object *frame_object;

        if (!can_is_rx_pending()) return;

        frame_object = gs_host_frame_object_get_locked(plist_frame_pool);
        if (!frame_object) return;

        gs_host_frame *frame = &frame_object->frame;

        if (!can_receive(frame)) {
            list_add_tail_locked(&frame_object->list, plist_frame_pool);
            return;
        }

        frame->echo_id = GS_HOST_FRAME_ECHO_ID_RX; // not an echo frame
        frame->reserved = 0;

        list_add_tail_locked(&frame_object->list, plist_to_host);

        // led.blink_request(signal_led::LED_RX);
    }

    void CAN_HandleError(list_head *plist_to_host, list_head *plist_frame_pool) {
        if (can_is_rx_pending()) return;

        can_drv_read_reg_status();

        if (can_state_change_pending()) {
            can_handle_state_change(plist_to_host, plist_frame_pool);
        } else if (can_bus_error_pending()) {
            can_handle_bus_error(plist_to_host, plist_frame_pool);
        } else if (can_bus_off_recovery_pending()) {
            can_handle_bus_off_recovery(plist_to_host, plist_frame_pool);
        }
    }

private:
    virtual void drv_enable() = 0;
    virtual void drv_disable() = 0;
    virtual bool can_send(gs_host_frame *frame) = 0;
    virtual bool can_receive(gs_host_frame *frame) = 0;
    virtual bool can_is_rx_pending() =0;
    virtual gs_can_state drv_get_state() = 0;
    virtual void can_drv_handle_state_change(gs_host_frame *frame) =0;
    virtual bool can_drv_handle_bus_error(gs_host_frame *frame) =0;
    virtual void can_drv_handle_bus_off_recovery() =0;

    static gs_host_frame_object* gs_host_frame_object_get_locked(list_head *plist) {
        const bool was_irq_enabled = disable_irq();
        gs_host_frame_object *frame_object =
            list_first_entry_or_null(plist, struct gs_host_frame_object, list);

        if (frame_object) list_del(&frame_object->list);

        restore_irq(was_irq_enabled);
        return frame_object;
    }

    void can_drv_read_reg_status() {
        reg_status.esr = hcan->Instance->ESR;

        if (can_drv_bus_error_pending()) {
            /* mark as handled by software */
            hcan->Instance->ESR |= FIELD_PREP(CAN_ESR_LEC, CAN_LEC_SOFTWARE);
        }
    }

    bool can_drv_bus_error_pending() const {
        const uint32_t reg_esr = reg_status.esr;
        const uint8_t lec = FIELD_GET(CAN_ESR_LEC, reg_esr);

        return can_is_lec_error(lec);
    }


    static bool can_is_lec_error(const uint8_t lec) {
        if (lec == CAN_LEC_NO_ERROR || lec == CAN_LEC_SOFTWARE)
            return false;

        return true;
    }

    bool can_state_change_pending() {
        if (!is_enabled()) return false;

        const gs_can_state new_state = can_drv_get_state();
        if (state == new_state) return false;

        state = new_state;

        return true;
    }

    gs_can_state can_drv_get_state() {
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

    void can_handle_state_change(list_head *plist_to_host, list_head *plist_frame_pool) {
        gs_host_frame_object *frame_object = gs_host_frame_object_get_locked(plist_frame_pool);
        if (!frame_object) return;

        gs_host_frame *frame = &frame_object->frame;
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

        list_add_tail_locked(&frame_object->list, plist_to_host);
    }


    void can_prepare_error_frame(gs_host_frame *frame) {
        frame->echo_id = GS_HOST_FRAME_ECHO_ID_RX;
        frame->can_id = CAN_ERR_FLAG;
        frame->can_dlc = CAN_ERR_DLC;
        frame->channel = nr;
        frame->flags = 0;
        frame->reserved = 0;
        *frame->classic_can = (struct classic_can){0};

        // frame->classic_can_ts->timestamp_us = timer_get();
    }

    bool can_bus_error_pending() {
        if (!(feature & GS_CAN_FEATURE_BERR_REPORTING))
            return false;

        return can_drv_bus_error_pending();
    }

    void can_handle_bus_error(list_head *plist_to_host, list_head *plist_frame_pool) {
        gs_host_frame_object *frame_object = gs_host_frame_object_get_locked(plist_frame_pool);
        if (!frame_object) return;

        gs_host_frame *frame = &frame_object->frame;

        can_prepare_error_frame(frame);
        bool handled = can_drv_handle_bus_error(frame);
        if (handled) {
            list_add_tail_locked(&frame_object->list, plist_to_host);
        } else {
            list_add_locked(&frame_object->list, plist_frame_pool);
        }
    }

    bool can_bus_off_recovery_pending() {
        if (bus_off_restart == CAN_CHANNEL_BUS_OFF_RESTART_DISABLED)
            return false;

        const uint32_t now = HAL_GetTick();

        return bus_off_restart - now < 0;
    }

    void can_handle_bus_off_recovery(list_head *plist_to_host, list_head *plist_frame_pool) {
        can_drv_handle_bus_off_recovery();

        bus_off_restart = CAN_CHANNEL_BUS_OFF_RESTART_DISABLED;

        gs_host_frame_object *frame_object = gs_host_frame_object_get_locked(plist_frame_pool);
        if (!frame_object)
            return;

        gs_host_frame *frame = &frame_object->frame;
        can_prepare_error_frame(frame);

        frame->can_id |= CAN_ERR_RESTARTED;

        list_add_tail_locked(&frame_object->list, plist_to_host);
    }
};

class bxcan_ : public CAN_Channel {
public:
    bxcan_(CAN_HandleTypeDef *hcan, uint8_t nr) : CAN_Channel(hcan, nr) {}

private:
    void drv_enable() override {}
    void drv_disable() override {}
    bool can_send(gs_host_frame *frame) override {}
    bool can_receive(gs_host_frame *frame) override {}
    bool can_is_rx_pending() override {}
    gs_can_state drv_get_state() override {}
    void can_drv_handle_state_change(gs_host_frame *frame) override {}
    bool can_drv_handle_bus_error(gs_host_frame *frame) override {}
    void can_drv_handle_bus_off_recovery() override {}
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
