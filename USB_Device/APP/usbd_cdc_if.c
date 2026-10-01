#include "usbd_cdc_if.h"
#include "usbd_core.h"
#include "usbd_ctlreq.h"
#include "usbd_ioreq.h"
#include "stdbool.h"
#include "util.h"
#include <string.h>

static USBD_HandleTypeDef *device;
static uint8_t rx_buffer[CDC_PACKET_SIZE];
static uint8_t tx_buffer[CDC_PACKET_SIZE];
static uint8_t line_coding[7];
static uint8_t pending_line_coding[7];
static uint16_t rx_length, rx_offset, control_lines;
static bool tx_busy, tx_zlp;
static uint8_t alternate_setting;
static uint16_t interface_status;

static uint8_t USBD_CDC_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx) {
    device = NULL;
    USBD_LL_CloseEP(pdev, CDC_IN_EP);
    USBD_LL_CloseEP(pdev, CDC_OUT_EP);
    USBD_LL_CloseEP(pdev, CDC_CMD_EP);
    pdev->ep_in[CDC_IN_EP & 0x7f].is_used = 0;
    pdev->ep_out[CDC_OUT_EP].is_used = 0;
    pdev->ep_in[CDC_CMD_EP & 0x7f].is_used = 0;
    rx_length = rx_offset = control_lines = 0;
    tx_busy = tx_zlp = false;
    return USBD_OK;
}

uint8_t USBD_CDC_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx) {
    static const uint8_t defaults[7] = {0x00, 0xc2, 0x01, 0x00, 0, 0, 8};
    memcpy(line_coding, defaults, sizeof(line_coding)); /* 115200 8N1 */
    rx_length = rx_offset = control_lines = 0;
    tx_busy = tx_zlp = false;
    if (USBD_LL_OpenEP(pdev, CDC_IN_EP, USBD_EP_TYPE_BULK, CDC_PACKET_SIZE) != USBD_OK)
        return USBD_FAIL;
    pdev->ep_in[CDC_IN_EP & 0x7f].is_used = 1;
    if (USBD_LL_OpenEP(pdev, CDC_OUT_EP, USBD_EP_TYPE_BULK, CDC_PACKET_SIZE) != USBD_OK)
        goto fail;
    pdev->ep_out[CDC_OUT_EP].is_used = 1;
    if (USBD_LL_OpenEP(pdev, CDC_CMD_EP, USBD_EP_TYPE_INTR, CDC_CMD_PACKET_SIZE) != USBD_OK)
        goto fail;
    pdev->ep_in[CDC_CMD_EP & 0x7f].is_used = 1;
    if (USBD_LL_PrepareReceive(pdev, CDC_OUT_EP, rx_buffer, sizeof(rx_buffer)) != USBD_OK)
        goto fail;
    device = pdev;
    return USBD_OK;
fail:
    USBD_CDC_DeInit(pdev, cfgidx);
    return USBD_FAIL;
}

static uint8_t USBD_CDC_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req) {
    if (!((req->bmRequest & 0x1F) == USB_REQ_RECIPIENT_INTERFACE &&
          (req->wIndex == CDC_CONTROL_INTERFACE || req->wIndex == CDC_DATA_INTERFACE)))
        return USBD_OK;

    if (pdev->dev_state != USBD_STATE_CONFIGURED)
        goto fail;
    if (req->bmRequest == 0x21 && req->wIndex == CDC_CONTROL_INTERFACE) {
        switch (req->bRequest) {
            case 0x20: /* SET_LINE_CODING */
                if (req->wValue != 0 || req->wLength != 7)
                    goto fail;
                return USBD_CtlPrepareRx(pdev, pending_line_coding, 7);
            case 0x22: /* SET_CONTROL_LINE_STATE */
                if (req->wLength != 0 || (req->wValue & ~3U) != 0)
                    goto fail;
                control_lines = req->wValue;
                return USBD_OK;
            default:
                goto fail;
        }
    }
    if (req->bmRequest == 0xa1 && req->wIndex == CDC_CONTROL_INTERFACE &&
        req->bRequest == 0x21 && req->wValue == 0 && req->wLength == 7)
        return USBD_CtlSendData(pdev, line_coding, sizeof(line_coding));

    if ((req->bmRequest & USB_REQ_TYPE_MASK) == USB_REQ_TYPE_STANDARD) {
        if (req->bmRequest == 0x81 && req->bRequest == USB_REQ_GET_INTERFACE &&
            req->wValue == 0 && req->wLength == 1)
            return USBD_CtlSendData(pdev, &alternate_setting, 1);
        if (req->bmRequest == 0x01 && req->bRequest == USB_REQ_SET_INTERFACE &&
            req->wValue == 0 && req->wLength == 0)
            return USBD_OK;
        if (req->bmRequest == 0x81 && req->bRequest == USB_REQ_GET_STATUS &&
            req->wValue == 0 && req->wLength == 2)
            return USBD_CtlSendData(pdev, (uint8_t *)&interface_status, 2);
    }
fail:
    USBD_CtlError(pdev, req);
    return USBD_FAIL;
}

static uint8_t USBD_CDC_EP0_RxReady(USBD_HandleTypeDef *pdev) {
    if (!((pdev->request.bmRequest & 0x1F) == USB_REQ_RECIPIENT_INTERFACE &&
          (pdev->request.wIndex == CDC_CONTROL_INTERFACE || pdev->request.wIndex == CDC_DATA_INTERFACE)))
        return USBD_OK;

    if (pdev->request.bmRequest != 0x21 || pdev->request.bRequest != 0x20 ||
        pdev->request.wIndex != CDC_CONTROL_INTERFACE || pdev->request.wLength != 7)
        return USBD_FAIL;
    memcpy(line_coding, pending_line_coding, sizeof(line_coding));
    return USBD_OK;
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
    .Init = USBD_CDC_Init,
    .DeInit = USBD_CDC_DeInit,
    .Setup = USBD_CDC_Setup,
    .EP0_RxReady = USBD_CDC_EP0_RxReady,
    .DataIn = USBD_CDC_DataIn,
    .DataOut = USBD_CDC_DataOut,
};

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
    if (length > CDC_PACKET_SIZE || (buffer == NULL && length != 0))
        return USBD_FAIL;
    bool irq = disable_irq();
    uint8_t result = USBD_FAIL;
    if (device != NULL && device->dev_state == USBD_STATE_CONFIGURED) {
        result = USBD_BUSY;
        if (!tx_busy) {
            if (length != 0)
                memcpy(tx_buffer, buffer, length);
            tx_busy = true;
            tx_zlp = length == CDC_PACKET_SIZE;
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
