#include "usbd_cdc_if.h"
#include "usbd_core.h"
#include "usbd_ctlreq.h"
#include "usbd_ioreq.h"
#include "stdbool.h"
#include "util.h"
#include <string.h>

static USBD_HandleTypeDef *device;
static uint8_t rx_buffer[CDC_DATA_MAX_PACKET_SIZE];
static uint8_t tx_buffer[CDC_DATA_MAX_PACKET_SIZE];
static USBD_CDC_LineCodingTypeDef pending_line_coding[7];
static uint16_t rx_length{0}, rx_offset{0}, control_lines{0};
static bool tx_busy{false}, tx_zlp{false};
static uint8_t alternate_setting;
static uint16_t interface_status;
static USBD_CDC_LineCodingTypeDef line_coding{.bitrate = 115200, .format = 0, .paritytype = 0, .datatype = 8};

USBD_CDC_HandleTypeDef hCDC;

uint8_t USBD_CDC_HandleTypeDef::Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx) {
    rx_length = rx_offset = control_lines = 0;
    tx_busy = tx_zlp = false;
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
    device = NULL;
    USBD_LL_CloseEP(pdev, CDC_IN_EP);
    USBD_LL_CloseEP(pdev, CDC_OUT_EP);
    USBD_LL_CloseEP(pdev, CDC_CMD_EP);
    pdev->ep_in[CDC_IN_EP & 0xFU].is_used = 0;
    pdev->ep_out[CDC_OUT_EP & 0xFU].is_used = 0;
    pdev->ep_in[CDC_CMD_EP & 0xFU].is_used = 0;
    rx_length = rx_offset = control_lines = 0;
    tx_busy = tx_zlp = false;
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

static uint8_t USBD_CDC_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum) {
    if (epnum != (CDC_IN_EP & EP_ADDR_MSK)) return USBD_OK;

    if (tx_zlp) {
        tx_zlp = false;
        uint8_t result = USBD_LL_Transmit(pdev, CDC_IN_EP, tx_buffer, 0);
        if (result != USBD_OK)
            tx_busy = false;
        return result;
    }
    tx_busy = false;
    return USBD_OK;
}

static uint8_t USBD_CDC_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum) {
    if (epnum != (CDC_OUT_EP & EP_ADDR_MSK)) return USBD_OK;

    rx_offset = 0;
    rx_length = (uint16_t)USBD_LL_GetRxDataSize(pdev, CDC_OUT_EP);
    if (rx_length == 0)
        return USBD_LL_PrepareReceive(pdev, CDC_OUT_EP, rx_buffer, sizeof(rx_buffer));
    /* Leave OUT NAKed until the application consumes this packet. */
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
    .DataIn = USBD_CDC_DataIn,
    .DataOut = USBD_CDC_DataOut,
};

uint8_t USBD_CDC_HandleTypeDef::Control(uint8_t bRequest, uint16_t wValue) {
    switch (bRequest) {
        case CDC_SET_LINE_CODING:
            line_coding = ep0.line_coding;
        case CDC_SET_CONTROL_LINE_STATE:

        default:
            return USBD_OK;
            break;
    }
    return USBD_OK;
}

uint16_t CDC_Read_FS(uint8_t *buffer, uint16_t capacity) {
    if (buffer == NULL || capacity == 0)
        return 0;
    bool irq = disable_irq();
    uint16_t length = rx_length - rx_offset;
    if (length > capacity)
        length = capacity;
    if (device != NULL && device->dev_state == USBD_STATE_CONFIGURED && length != 0) {
        memcpy(buffer, &rx_buffer[rx_offset], length);
        rx_offset += length;
        if (rx_offset == rx_length) {
            rx_length = rx_offset = 0;
            USBD_LL_PrepareReceive(device, CDC_OUT_EP, rx_buffer, sizeof(rx_buffer));
        }
    } else {
        length = 0;
    }
    restore_irq(irq);
    return length;
}

uint8_t CDC_Transmit_FS(const uint8_t *buffer, uint16_t length) {
    if (length > CDC_DATA_MAX_PACKET_SIZE || (buffer == NULL && length != 0))
        return USBD_FAIL;
    bool irq = disable_irq();
    uint8_t result = USBD_FAIL;
    if (device != NULL && device->dev_state == USBD_STATE_CONFIGURED) {
        result = USBD_BUSY;
        if (!tx_busy) {
            if (length != 0)
                memcpy(tx_buffer, buffer, length);
            tx_busy = true;
            tx_zlp = length == CDC_DATA_MAX_PACKET_SIZE;
            result = USBD_LL_Transmit(device, CDC_IN_EP, tx_buffer, length);
            if (result != USBD_OK)
                tx_busy = tx_zlp = false;
        }
    }
    restore_irq(irq);
    return result;
}


uint16_t CDC_GetControlLineState_FS(void) {
    bool irq = disable_irq();
    uint16_t result = control_lines;
    restore_irq(irq);
    return result;
}
