#include "usbd_cdc_if.h"
#include "usbd_core.h"
#include "DualBuffer.h"
#include "usart.h"

extern USBD_CDC_HandleTypeDef hCDC;

static uint8_t uart_rx_buffer[64];

TxDualBuffer<uint8_t, 512> device_to_host{
    [](uint8_t *buffer, uint16_t length) -> uint8_t {
        return hCDC.CDC_Transmit_FS(buffer, length);
    }
};

TxDualBuffer<uint8_t, 512> host_to_device{
    [](uint8_t *buffer, uint16_t length) -> uint8_t {
        return HAL_UART_Transmit_DMA(&huart1, buffer, length);
    }
};

uint8_t USBD_CDC_HandleTypeDef::Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx) {
    USBD_LL_OpenEP(pdev, CDC_IN_EP, USBD_EP_TYPE_BULK, CDC_DATA_MAX_PACKET_SIZE);
    USBD_LL_OpenEP(pdev, CDC_OUT_EP, USBD_EP_TYPE_BULK, CDC_DATA_MAX_PACKET_SIZE);
    USBD_LL_OpenEP(pdev, CDC_CMD_EP, USBD_EP_TYPE_INTR, CDC_CMD_PACKET_SIZE);
    pdev->ep_in[CDC_IN_EP & 0xFU].is_used = 1U;
    pdev->ep_out[CDC_OUT_EP & 0xFU].is_used = 1U;
    pdev->ep_in[CDC_CMD_EP & 0xFU].is_used = 1U;
    USBD_LL_PrepareReceive(pdev, CDC_OUT_EP, rx_buffer, CDC_DATA_MAX_PACKET_SIZE);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart_rx_buffer, sizeof(uart_rx_buffer));
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
    device = pdev;
    return USBD_OK;
}

uint8_t USBD_CDC_HandleTypeDef::DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx) {
    device = nullptr;
    HAL_UART_AbortReceive_IT(&huart1);
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
    if (!((req->bmRequest & 0x1FU) == USB_REQ_RECIPIENT_INTERFACE &&
          (req->wIndex == CDC_CONTROL_INTERFACE || req->wIndex == CDC_DATA_INTERFACE)))
        return USBD_OK;

    if (req->wLength != 0U) {
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
    } else {
        if (Control(req->bRequest, req->wValue) != USBD_OK) {
            USBD_CtlError(pdev, req);
            return USBD_FAIL;
        }
    }

    return USBD_OK;
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
        // 发送完成回调
        device_to_host.transmitComplete();
    }
    return USBD_OK;
}

uint8_t USBD_CDC_HandleTypeDef::DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum) {
    if (epnum != (CDC_OUT_EP & EP_ADDR_MSK)) return USBD_OK;

    // TODO:处理接收到的数据
    led.blink_request(signal_led::LED_TX);
    host_to_device.inBuffer(rx_buffer, USBD_LL_GetRxDataSize(pdev, epnum));

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
            break;
        case CDC_GET_LINE_CODING:
            ep0.line_coding = line_coding;
            break;
        default:
            return USBD_OK;
            break;
    }
    return USBD_OK;
}

uint8_t USBD_CDC_HandleTypeDef::CDC_Transmit_FS(uint8_t *buffer, uint16_t length) {
    if (device == nullptr || (buffer == nullptr && length != 0))
        return USBD_FAIL;

    if (!transmitting) {
        transmitting = true;
        led.blink_request(signal_led::LED_RX);
        if (length % CDC_DATA_MAX_PACKET_SIZE == 0U) zlp_request = true;
        return USBD_LL_Transmit(device, CDC_IN_EP, buffer, length);
    }
    return USBD_BUSY;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart1) {
        host_to_device.transmitComplete();
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart == &huart1) {
        device_to_host.inBuffer(uart_rx_buffer, Size);
        HAL_UARTEx_ReceiveToIdle_DMA(huart, uart_rx_buffer, sizeof(uart_rx_buffer));
        __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart1) {
        __HAL_UNLOCK(huart);
        HAL_UARTEx_ReceiveToIdle_DMA(huart, uart_rx_buffer, sizeof(uart_rx_buffer));
        __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
    }
}
