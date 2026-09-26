#include "usbd_dfu.h"
#include <usbd_ioreq.h>

static USBD_DFU_HandleTypeDef hDFU;

static uint8_t USBD_DFU_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req) {
    if (!((req->bmRequest & 0x1F) == USB_REQ_RECIPIENT_INTERFACE &&     // recipient: interface
          (req->bmRequest & USB_REQ_TYPE_MASK) == USB_REQ_TYPE_CLASS && // class request
          req->wIndex == DFU_INTERFACE_NUM))
        return USBD_OK;

    switch (req->bRequest) {
        case USB_DFU_REQ_DETACH:
            hDFU.dfu_detach_requested = true;
            break;
        case USB_DFU_REQ_GETSTATUS: {
            struct dfu_status *status = &hDFU.ep0.dfu_status;

            status->status = ERR_OK;
            status->poll_timeout[0] = 0x0;
            status->poll_timeout[1] = 0x0;
            status->poll_timeout[2] = 0x0;
            status->state = APP_IDLE;
            status->stringidx = 0xff; // status string descriptor index

            USBD_CtlSendData(pdev, hDFU.ep0.buf, sizeof(hDFU.ep0.dfu_status));
            break;
        }
        default:
            USBD_CtlError(pdev, req);
    }

    return USBD_OK;
}

USBD_ClassTypeDef USBD_DFU = {
    .Setup = USBD_DFU_Setup,
};

bool USBD_DFU_DfuDetachRequested() {
    return hDFU.dfu_detach_requested;
}
