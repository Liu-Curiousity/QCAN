#include "usbd_dfu.h"

#include <usbd_ioreq.h>
#include "usbd_composite.h"

static uint8_t USBD_DFU_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
    USBD_COMPOSITE_HandleTypeDef *hcan = pdev->pClassData;
    union ep0 *ep0 = &hcan->ep0;

    switch (req->bRequest) {
        case USB_DFU_REQ_DETACH:
            hcan->dfu_detach_requested = true;
            break;
        case USB_DFU_REQ_GETSTATUS: {
            struct dfu_status *status = &ep0->data.dfu_status;

            status->status = ERR_OK;
            status->poll_timeout[0] = 0x0;
            status->poll_timeout[1] = 0x0;
            status->poll_timeout[2] = 0x0;
            status->state = APP_IDLE;
            status->stringidx = 0xff; // status string descriptor index

            USBD_CtlSendData(pdev, ep0->buf, sizeof(ep0->data.dfu_status));
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
