#include "usbd_cdc_if.h"
#include "usbd_core.h"

USBD_CDC_HandleTypeDef hCDC;

uint8_t USBD_CDC_HandleTypeDef::Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx) {
    USBD_LL_OpenEP(pdev, CDC_IN_EP, USBD_EP_TYPE_BULK, CDC_DATA_MAX_PACKET_SIZE);
    USBD_LL_OpenEP(pdev, CDC_OUT_EP, USBD_EP_TYPE_BULK, CDC_DATA_MAX_PACKET_SIZE);
    USBD_LL_OpenEP(pdev, CDC_CMD_EP, USBD_EP_TYPE_INTR, CDC_CMD_PACKET_SIZE);
    pdev->ep_in[CDC_IN_EP & 0xFU].is_used = 1U;
    pdev->ep_out[CDC_OUT_EP & 0xFU].is_used = 1U;
    pdev->ep_in[CDC_CMD_EP & 0xFU].is_used = 1U;
    USBD_LL_PrepareReceive(pdev, CDC_OUT_EP, rx_buffer, CDC_DATA_MAX_PACKET_SIZE);
    device = pdev;
    return USBD_OK;
}

uint8_t USBD_CDC_HandleTypeDef::DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx) {
    device = pdev;
    USBD_LL_CloseEP(pdev, CDC_IN_EP);
    USBD_LL_CloseEP(pdev, CDC_OUT_EP);
    USBD_LL_CloseEP(pdev, CDC_CMD_EP);
    pdev->ep_in[CDC_IN_EP & 0xFU].is_used = 0;
    pdev->ep_out[CDC_OUT_EP & 0xFU].is_used = 0;
    pdev->ep_in[CDC_CMD_EP & 0xFU].is_used = 0;
    transmitting = zlp_request = false;
    return USBD_OK;
}

uint8_t USBD_CDC_HandleTypeDef::Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req) {
    USBD_StatusTypeDef ret = USBD_OK;

    if (!((req->bmRequest & 0x1FU) == USB_REQ_RECIPIENT_INTERFACE &&
          (req->wIndex == CDC_CONTROL_INTERFACE || req->wIndex == CDC_DATA_INTERFACE)))
        return USBD_OK;

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

    return (uint8_t)ret;

fail:
    USBD_CtlError(pdev, req);
    return USBD_FAIL;
}

uint8_t USBD_CDC_HandleTypeDef::EP0_RxReady(USBD_HandleTypeDef *pdev) {
    uint8_t ret = USBD_OK;

    if (!((pdev->request.bmRequest & 0x1F) == USB_REQ_RECIPIENT_INTERFACE &&
          (pdev->request.wIndex == CDC_CONTROL_INTERFACE || pdev->request.wIndex == CDC_DATA_INTERFACE)))
        return USBD_OK;

    if (pdev->request.bmRequest != 0x21 || pdev->request.bRequest != 0x20 ||
        pdev->request.wIndex != CDC_CONTROL_INTERFACE || pdev->request.wLength != 7)
        return USBD_FAIL;

    USBD_SetupReqTypedef *req = &last_setup_request;

    ret = Control(req->bRequest, req->wValue);

    req->bRequest = 0xFF;
    return ret;
}

uint8_t USBD_CDC_HandleTypeDef::DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum) {
    if (epnum != (CDC_IN_EP & EP_ADDR_MSK)) return USBD_OK;

    if (zlp_request) {
        zlp_request = false;
        USBD_LL_Transmit(pdev, epnum, NULL, 0U);
    } else {
        transmitting = false;
        // TODO:发送完成回调
    }
    return USBD_OK;
}

uint8_t USBD_CDC_HandleTypeDef::DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum) {
    if (epnum != (CDC_OUT_EP & EP_ADDR_MSK)) return USBD_OK;

    // TODO:处理接收到的数据
    // CDC_Read_FS(rx_buffer, USBD_LL_GetRxDataSize(pdev, epnum));

    USBD_LL_PrepareReceive(pdev, CDC_OUT_EP, rx_buffer, CDC_DATA_MAX_PACKET_SIZE);

    return USBD_OK;
}

USBD_ClassTypeDef USBD_CDC = {
    .Init = [](USBD_HandleTypeDef *pdev, uint8_t cfgidx) -> uint8_t {
        return hCDC.Init(pdev, cfgidx);
    },
    .DeInit = [](USBD_HandleTypeDef *pdev, uint8_t cfgidx) -> uint8_t {
        return hCDC.DeInit(pdev, cfgidx);
    },
    .Setup = [](USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req) -> uint8_t {
        return hCDC.Setup(pdev, req);
    },
    .EP0_RxReady = [](USBD_HandleTypeDef *pdev) -> uint8_t {
        return hCDC.EP0_RxReady(pdev);
    },
    .DataIn = [](USBD_HandleTypeDef *pdev, uint8_t epnum) -> uint8_t {
        return hCDC.DataIn(pdev, epnum);
    },
    .DataOut = [](USBD_HandleTypeDef *pdev, uint8_t epnum) -> uint8_t {
        return hCDC.DataOut(pdev, epnum);
    },
};

uint8_t USBD_CDC_HandleTypeDef::Control(uint8_t bRequest, uint16_t wValue) {
    switch (bRequest) {
        case CDC_SET_LINE_CODING:
            line_coding = ep0.line_coding;
        case CDC_GET_LINE_CODING:
            ep0.line_coding = line_coding;
        default:
            return USBD_OK;
            break;
    }
    return USBD_OK;
}

uint8_t USBD_CDC_HandleTypeDef::CDC_Transmit_FS(uint8_t *buffer, uint16_t length) {
    if (length > CDC_DATA_MAX_PACKET_SIZE || (buffer == nullptr && length != 0))
        return USBD_FAIL;

    if (!transmitting) {
        transmitting = true;
        if (length % CDC_DATA_MAX_PACKET_SIZE == 0U) zlp_request = true;
        return USBD_LL_Transmit(device, CDC_IN_EP, buffer, length);
    }
    return USBD_BUSY;
}
