#include "usbd_gs_can.h"

#include <cstring>
#include <usbd_core.h>

#include "can_common.h"
#include "timer.h"
#include "led.h"
#include "util.h"

extern USBD_GS_CAN_HandleTypeDef hGS_CAN;

uint8_t USBD_GS_CAN_HandleTypeDef::PrepareReceive(USBD_HandleTypeDef *pdev) {
    return USBD_LL_PrepareReceive(
        pdev,
        GSUSB_ENDPOINT_OUT,
        reinterpret_cast<uint8_t *>(&from_host_buf.frame),
        struct_size(&from_host_buf.frame, canfd_ts, 1)
    );
}

uint8_t USBD_GS_CAN_HandleTypeDef::Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx) {
    USBD_LL_OpenEP(pdev, GSUSB_ENDPOINT_IN, USBD_EP_TYPE_BULK, CAN_DATA_MAX_PACKET_SIZE);
    USBD_LL_OpenEP(pdev, GSUSB_ENDPOINT_OUT, USBD_EP_TYPE_BULK, CAN_DATA_MAX_PACKET_SIZE);
    xQueueReset(xQueue_to_host);
    PrepareReceive(pdev);
    return USBD_OK;
}

uint8_t USBD_GS_CAN_HandleTypeDef::DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx) {
    USBD_LL_CloseEP(pdev, GSUSB_ENDPOINT_IN);
    USBD_LL_CloseEP(pdev, GSUSB_ENDPOINT_OUT);
    return USBD_OK;
}

uint8_t USBD_GS_CAN_HandleTypeDef::Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req) {
    if (!IsRequest(req)) return USBD_OK;

    CAN_Channel *channel = nullptr;
    const void *src = nullptr;
    size_t len;

    if (!(req->bRequest == GS_USB_BREQ_HOST_FORMAT || req->bRequest == GS_USB_BREQ_DEVICE_CONFIG)) {
        channel = GetChannel(req->wValue);
        if (!channel) {
            goto out_fail;
        }
    }

    if (!IS_ENABLED(CONFIG_CANFD)) {
        switch (req->bRequest) {
            case GS_USB_BREQ_DATA_BITTIMING:
            case GS_USB_BREQ_BT_CONST_EXT:
                goto out_fail;
        }
    }

    if (!IS_ENABLED(CONFIG_TERMINATION)) {
        switch (req->bRequest) {
            case GS_USB_BREQ_SET_TERMINATION:
            case GS_USB_BREQ_GET_TERMINATION:
                goto out_fail;
        }
    }

    switch (req->bRequest) {
        // Host -> Device
        case GS_USB_BREQ_HOST_FORMAT:
            len = sizeof(ep0.config);
            break;
        case GS_USB_BREQ_BITTIMING:
            len = sizeof(ep0.bittiming);
            break;
        case GS_USB_BREQ_MODE:
            len = sizeof(ep0.mode);
            break;
        case GS_USB_BREQ_BT_CONST:
            src = &channel->bt_const;
            len = sizeof(channel->bt_const);
            break;
        case GS_USB_BREQ_DEVICE_CONFIG:
            src = &USBD_GS_CAN_dconf;
            len = sizeof(USBD_GS_CAN_dconf);
            break;
        case GS_USB_BREQ_TIMESTAMP:
            src = &sof_timestamp_us;
            len = sizeof(sof_timestamp_us);
            break;
        case GS_USB_BREQ_IDENTIFY:
            len = sizeof(ep0.identify_mode);
            break;
        case GS_USB_BREQ_DATA_BITTIMING:
            len = sizeof(ep0.bittiming);
            break;
        case GS_USB_BREQ_BT_CONST_EXT:
            src = &channel->bt_const_ext;
            len = sizeof(channel->bt_const_ext);
            break;
        case GS_USB_BREQ_SET_TERMINATION:
            len = sizeof(ep0.term_state);
            break;
        case GS_USB_BREQ_GET_TERMINATION:
            // TODO: 后续支持
            ep0.term_state.state = GS_CAN_TERMINATION_UNSUPPORTED;
            src = &ep0.term_state;
            len = sizeof(ep0.term_state);
            break;
        case GS_USB_BREQ_GET_STATE:
            ep0.state = channel->get_device_state();
            src = &ep0.state;
            len = sizeof(ep0.state);
            break;
        default:
            goto out_fail;
    }

    if (req->wLength < len) {
        goto out_fail;
    }

    switch (req->bRequest) {
        case GS_USB_BREQ_HOST_FORMAT:
        case GS_USB_BREQ_BITTIMING:
        case GS_USB_BREQ_MODE:
        case GS_USB_BREQ_IDENTIFY:
        case GS_USB_BREQ_DATA_BITTIMING:
        case GS_USB_BREQ_SET_TERMINATION:
            if (req->wLength > sizeof(ep0)) {
                goto out_fail;
            }

            last_setup_request = *req;
            USBD_CtlPrepareRx(pdev, ep0.buf, req->wLength);
            break;

        // Device -> Host
        case GS_USB_BREQ_BT_CONST:
        case GS_USB_BREQ_DEVICE_CONFIG:
        case GS_USB_BREQ_TIMESTAMP:
        case GS_USB_BREQ_BT_CONST_EXT:
        case GS_USB_BREQ_GET_TERMINATION:
        case GS_USB_BREQ_GET_STATE:
            USBD_CtlSendData(pdev, (uint8_t *)src, len);
            break;
        default:
            goto out_fail;
    }

    return USBD_OK;

out_fail:
    USBD_CtlError(pdev, req);
    return USBD_FAIL;
}

uint8_t USBD_GS_CAN_HandleTypeDef::EP0_RxReady(USBD_HandleTypeDef *pdev) {
    /* Use the current transfer, not a saved request from a previous CAN write. */
    if (!IsRequest(&pdev->request)) return USBD_OK;

    CAN_Channel *channel = nullptr;
    USBD_SetupReqTypedef *req = &last_setup_request;

    if (!(req->bRequest == GS_USB_BREQ_HOST_FORMAT || req->bRequest == GS_USB_BREQ_DEVICE_CONFIG)) {
        channel = GetChannel(req->wValue);
        if (!channel) {
            goto out_fail;
        }
    }

    if (!IS_ENABLED(CONFIG_CANFD)) {
        switch (req->bRequest) {
            case GS_USB_BREQ_DATA_BITTIMING:
                goto out_fail;
        }
    }

    if (!IS_ENABLED(CONFIG_TERMINATION)) {
        switch (req->bRequest) {
            case GS_USB_BREQ_SET_TERMINATION:
                goto out_fail;
        }
    }

    switch (req->bRequest) {
        case GS_USB_BREQ_HOST_FORMAT:
            /*
             * The firmware on the original USB2CAN by Geschwister Schneider
             * Technologie Entwicklungs- und Vertriebs UG exchanges all data
             * between the host and the device in host byte order. This is done
             * with the struct gs_host_config::byte_order member, which is sent
             * first to indicate the desired byte order.
             *
             * The widely used open source firmware candleLight doesn't support
             * this feature and exchanges the data in little endian byte order.
             */
            break;
        case GS_USB_BREQ_BITTIMING: {
            if (!channel->set_bittiming(&ep0.bittiming)) goto out_fail;
            break;
        }
        case GS_USB_BREQ_MODE: {
            const gs_device_mode *mode = &ep0.mode;

            if (mode->mode == GS_CAN_MODE_RESET) {
                channel->disable();
            } else if (mode->mode == GS_CAN_MODE_START) {
                if (!channel->enable(mode->feature)) goto out_fail;
            }
            break;
        }
        case GS_USB_BREQ_IDENTIFY: {
            if (ep0.identify_mode.mode) {
                channel->led.mode = signal_led::Mode::CONTINUES;
            } else {
                channel->led.mode = channel->is_enabled() ? signal_led::Mode::NORMAL : signal_led::Mode::OFF;
            }
            break;
        }
        case GS_USB_BREQ_DATA_BITTIMING: {
            if (!channel->set_data_bittiming(&ep0.bittiming)) goto out_fail;
            break;
        }
        case GS_USB_BREQ_SET_TERMINATION: {
            // TODO: 后续支持
            // set_term(channel, (gs_can_termination_state)ep0.term_state.state);
            break;
        }
        default:
            break;
    }

    req->bRequest = 0xFF;
    return USBD_OK;

out_fail:
    return USBD_FAIL;
}

uint8_t USBD_GS_CAN_HandleTypeDef::DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum) {
    return USBD_OK;
}

// Note that the return value is completely ignored by the stack.
uint8_t USBD_GS_CAN_HandleTypeDef::DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum) {
    if (epnum != GSUSB_ENDPOINT_OUT) return USBD_OK;

    uint32_t rxlen = USBD_LL_GetRxDataSize(pdev, epnum);
    if (rxlen < struct_size(&from_host_buf.frame, classic_can, 1)) {
        PrepareReceive(pdev);
        return USBD_OK;
    }

    CAN_Channel *channel = GetChannel(from_host_buf.frame.channel);
    if (!channel) {
        PrepareReceive(pdev);
        return USBD_OK;
    }

    if (IS_ENABLED(CONFIG_CANFD) &&
        from_host_buf.frame.flags & GS_CAN_FLAG_FD &&
        rxlen < struct_size(&from_host_buf.frame, canfd, 1)) {
        PrepareReceive(pdev);
        return USBD_OK;
    }

    // Enqueue the frame we just received.
    // TODO: 未处理溢出
    bool queue_overflow = xQueueSendToBack(channel->xQueue_from_host, &from_host_buf._buf, 0) != pdPASS;

#if defined(USB) || defined(USB_DRD_FS)
    // Not all RX buffers are ready. We got enough in the
    // pre-allocated reserve to ensure callbacks which are going
    // to be called due to USB frames being received into double
    // buffer, but disable further RX. If double buffer not in use,
    // RX would have already been disabled by USB HW.
    PrepareReceive(pdev);
    if (queue_overflow)
        PCD_SET_EP_RX_STATUS(((PCD_HandleTypeDef *)pdev->pData)->Instance,
                         GSUSB_ENDPOINT_OUT,
                         USB_EP_RX_NAK);
#endif

    return USBD_OK;
}

uint8_t USBD_GS_CAN_HandleTypeDef::SOF(USBD_HandleTypeDef *pdev) {
    sof_timestamp_us = timer_get();
    return USBD_OK;
}

USBD_ClassTypeDef USBD_GS_CAN = {
    .Init = [](USBD_HandleTypeDef *pdev, uint8_t cfgidx) -> uint8_t {
        return hGS_CAN.Init(pdev, cfgidx);
    },
    .DeInit = [](USBD_HandleTypeDef *pdev, uint8_t cfgidx) -> uint8_t {
        return hGS_CAN.DeInit(pdev, cfgidx);
    },
    .Setup = [](USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req) -> uint8_t {
        return hGS_CAN.Setup(pdev, req);
    },
    .EP0_RxReady = [](USBD_HandleTypeDef *pdev) -> uint8_t {
        return hGS_CAN.EP0_RxReady(pdev);
    },
    .DataIn = [](USBD_HandleTypeDef *pdev, uint8_t epnum) -> uint8_t {
        return hGS_CAN.DataIn(pdev, epnum);
    },
    .DataOut = [](USBD_HandleTypeDef *pdev, uint8_t epnum) -> uint8_t {
        return hGS_CAN.DataOut(pdev, epnum);
    },
    .SOF = [](USBD_HandleTypeDef *pdev) -> uint8_t {
        return hGS_CAN.SOF(pdev);
    },
};

void USBD_GS_CAN_ReceiveFromHost(USBD_HandleTypeDef *pdev) {
    // hGS_CAN.PrepareReceive(pdev);
}

static uint8_t USBD_GS_CAN_Transmit(USBD_HandleTypeDef *pdev, uint8_t *buf, uint16_t len) {
    if (pdev->dev_state == USBD_STATE_CONFIGURED) {
        return USBD_LL_Transmit(pdev, GSUSB_ENDPOINT_IN, buf, len);
    }
    return USBD_FAIL;
}

void USBD_GS_CAN_SendFrame(USBD_HandleTypeDef *pdev) {
    static bool datain_request = false;
    static gs_host_frame_object rx_frame{};

    if (!datain_request && xQueueReceive(hGS_CAN.xQueue_to_host, rx_frame._buf, 0) == pdPASS) {
        datain_request = true;
    }
    if (datain_request == false) return;

    const CAN_Channel *channel = &hGS_CAN.channels[rx_frame.frame.channel];
    const gs_host_frame *frame = &rx_frame.frame;
    static uint8_t buf[CAN_DATA_MAX_PACKET_SIZE];
    uint8_t *send_addr = (uint8_t *)frame;
    size_t len;

    if (IS_ENABLED(CONFIG_CANFD) &&
        frame->flags & GS_CAN_FLAG_FD) {
        if (channel->feature & GS_CAN_FEATURE_HW_TIMESTAMP) {
            len = struct_size(frame, canfd_ts, 1);
        } else {
            len = struct_size(frame, canfd, 1);
        }
    } else {
        if (channel->feature & GS_CAN_FEATURE_HW_TIMESTAMP) {
            len = struct_size(frame, classic_can_ts, 1);
        } else {
            len = struct_size(frame, classic_can, 1);
        }
    }

    /*
     * When talking to WinUSB it seems to help a lot if the size of
     * packet you send equals the max packet size. In this mode, fill
     * packets out to max packet size and then send.
     *
     * Don't know if the above observation is still true for CAN-FD
     * (CAN-FD frames don't fit into a single the full speed USB
     * packet of 64 byte), so don't do any padding for CAN-FD frames
     * for now.
     */
    if (channel->feature & GS_CAN_FEATURE_PAD_PKTS_TO_MAX_PKT_SIZE &&
        !((IS_ENABLED(CONFIG_CANFD) && frame->flags & GS_CAN_FLAG_FD))) {
        memcpy(buf, frame, len);

        // zero rest of buffer
        memset(buf + len, 0, sizeof(buf) - len);
        send_addr = buf;
        len = sizeof(buf);
    }

    if (USBD_GS_CAN_Transmit(pdev, send_addr, len) == USBD_OK) {
        datain_request = false;
    }
}