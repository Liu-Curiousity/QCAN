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
    queue_to_host.reset();
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

    if ((req->bmRequest & 0x80U) != 0U) {
        if (Control(req->bRequest, req->wValue) != USBD_OK) {
            USBD_CtlError(pdev, req);
            return USBD_FAIL;
        }
        USBD_CtlSendData(pdev, ep0.buf, req->wLength);
    } else {
        if (req->wLength > sizeof(ep0)) {
            USBD_CtlError(pdev, req);
            return USBD_FAIL;
        }

        last_setup_request = *req;
        USBD_CtlPrepareRx(pdev, ep0.buf, req->wLength);
    }

    return USBD_OK;
}

uint8_t USBD_GS_CAN_HandleTypeDef::EP0_RxReady(USBD_HandleTypeDef *pdev) {
    uint8_t ret = USBD_OK;
    if (!IsRequest(&pdev->request)) return USBD_OK;

    USBD_SetupReqTypedef *req = &last_setup_request;

    ret = Control(req->bRequest, req->wValue);

    req->bRequest = 0xFF;
    return ret;
}

uint8_t USBD_GS_CAN_HandleTypeDef::DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum) {
    if (epnum != (GSUSB_ENDPOINT_IN & EP_ADDR_MSK)) return USBD_OK;
    to_host_sending = false;
    return USBD_OK;
}

// Note that the return value is completely ignored by the stack.
uint8_t USBD_GS_CAN_HandleTypeDef::DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum) {
    if (epnum != (GSUSB_ENDPOINT_OUT & EP_ADDR_MSK)) return USBD_OK;

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

    // TODO:队列满是否要NAK
    channel->queue_from_host.push(from_host_buf);
    PrepareReceive(pdev);
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

uint8_t USBD_GS_CAN_HandleTypeDef::Control(uint8_t bRequest, uint16_t wValue) {
    CAN_Channel *channel = nullptr;
    if (!(bRequest == GS_USB_BREQ_HOST_FORMAT || bRequest == GS_USB_BREQ_DEVICE_CONFIG)) {
        channel = GetChannel(wValue);
        if (!channel) return USBD_FAIL;
    }

    switch (bRequest) {
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
        case GS_USB_BREQ_BITTIMING:
            if (!channel->set_bittiming(&ep0.bittiming)) return USBD_FAIL;
            break;
        case GS_USB_BREQ_MODE: {
            const gs_device_mode *mode = &ep0.mode;

            if (mode->mode == GS_CAN_MODE_RESET) {
                channel->disable();
            } else if (mode->mode == GS_CAN_MODE_START) {
                if (!channel->enable(mode->feature)) return USBD_FAIL;
            }
            break;
        }
        case GS_USB_BREQ_BT_CONST:
            ep0.bt_const = channel->bt_const;
            break;
        case GS_USB_BREQ_DEVICE_CONFIG:
            ep0.dev_config = dev_config;
            break;
        case GS_USB_BREQ_TIMESTAMP:
            ep0.timestamp = sof_timestamp_us;
            break;
        case GS_USB_BREQ_IDENTIFY:
            if (ep0.identify_mode.mode) {
                channel->led.mode = signal_led::Mode::CONTINUES;
            } else {
                channel->led.mode = channel->is_enabled() ? signal_led::Mode::NORMAL : signal_led::Mode::OFF;
            }
            break;
        case GS_USB_BREQ_DATA_BITTIMING:
            if constexpr (!IS_ENABLED(CONFIG_CANFD)) return USBD_FAIL;
            if (!channel->set_data_bittiming(&ep0.bittiming)) return USBD_FAIL;
            break;
        case GS_USB_BREQ_BT_CONST_EXT:
            if constexpr (!IS_ENABLED(CONFIG_CANFD)) return USBD_FAIL;
            ep0.bt_const_ext = channel->bt_const_ext;
            break;
        case GS_USB_BREQ_SET_TERMINATION:
            if constexpr (!IS_ENABLED(CONFIG_TERMINATION)) return USBD_FAIL;
            // TODO: 后续支持
            // set_term(channel, (gs_can_termination_state)ep0.term_state.state);
            break;
        case GS_USB_BREQ_GET_TERMINATION:
            if constexpr (!IS_ENABLED(CONFIG_TERMINATION)) return USBD_FAIL;
            // TODO: 后续支持
            ep0.term_state.state = GS_CAN_TERMINATION_UNSUPPORTED;
            break;
        case GS_USB_BREQ_GET_STATE:
            ep0.state = channel->get_device_state();
            break;
        default:
            break;
    }
    return USBD_OK;
}

void USBD_GS_CAN_ReceiveFromHost(USBD_HandleTypeDef *pdev) {
    // hGS_CAN.PrepareReceive(pdev);
}

static uint8_t USBD_GS_CAN_Transmit(USBD_HandleTypeDef *pdev, uint8_t *buf, uint16_t len) {
    if (pdev->dev_state == USBD_STATE_CONFIGURED) {
        return USBD_LL_Transmit(pdev, GSUSB_ENDPOINT_IN, buf, len);
    }
    return USBD_FAIL;
}

void USBD_GS_CAN_HandleTypeDef::USBD_GS_CAN_SendFrame(USBD_HandleTypeDef *pdev) {
    static gs_host_frame_object rx_frame{};
    if (to_host_sending) return;
    // TODO:可能被USB中断打断，导致为空时front和pop
    if (queue_to_host.empty()) return;
    rx_frame = queue_to_host.front();

    if (queue_to_host.is_overflow()) rx_frame.frame.flags |= GS_CAN_FLAG_OVERFLOW;

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

    to_host_sending = true;
    if (USBD_GS_CAN_Transmit(pdev, send_addr, len) == USBD_OK) {
        queue_to_host.pop();
    }
}
