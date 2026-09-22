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

#include <stdlib.h>
#include <string.h>

#include "can.h"
#include "can_common.h"
#include "compiler.h"
#include "config.h"
#include "gs_usb.h"
#include "usbd_core.h"
#include "usbd_ctlreq.h"
#include "usbd_def.h"
#include "usbd_desc.h"
#include "usbd_dfu.h"
#include "usbd_composite.h"

#include "usbd_cdc_if.h"
#include "usbd_gs_can.h"
#include "usbd_ioreq.h"
#include "util.h"

#define CDC_INTERFACE_STRING_FS	 "qcan cdc"

/* Configuration Descriptor */
__ALIGN_BEGIN static uint8_t USBD_COMPOSITE_CfgDesc[USB_CAN_CONFIG_DESC_SIZ] __ALIGN_END =
{
	/*---------------------------------------------------------------------------*/
	/* Configuration Descriptor */
	0x09,                             /* bLength */
	USB_DESC_TYPE_CONFIGURATION,      /* bDescriptorType */
	USB_CAN_CONFIG_DESC_SIZ,          /* wTotalLength */
	0x00,
	0x04,                             /* bNumInterfaces */
	0x01,                             /* bConfigurationValue */
	USBD_IDX_CONFIG_STR,              /* iConfiguration */
	0x80,                             /* bmAttributes */
	0x4B,                             /* MaxPower 150 mA */
	/*---------------------------------------------------------------------------*/

	/*---------------------------------------------------------------------------*/
	/* GS_USB Interface Descriptor */
	0x09,                             /* bLength */
	USB_DESC_TYPE_INTERFACE,          /* bDescriptorType */
	0x00,                             /* bInterfaceNumber */
	0x00,                             /* bAlternateSetting */
	0x02,                             /* bNumEndpoints */
	0xFF,                             /* bInterfaceClass: Vendor Specific*/
	0xFF,                             /* bInterfaceSubClass: Vendor Specific */
	0xFF,                             /* bInterfaceProtocol: Vendor Specific */
	0x00,                             /* iInterface */
	/*---------------------------------------------------------------------------*/

	/*---------------------------------------------------------------------------*/
	/* EP1 descriptor */
	0x07,                             /* bLength */
	USB_DESC_TYPE_ENDPOINT,           /* bDescriptorType */
	GSUSB_ENDPOINT_IN,                /* bEndpointAddress */
	0x02,                             /* bmAttributes: bulk */
	LOBYTE(CAN_DATA_MAX_PACKET_SIZE), /* wMaxPacketSize */
	HIBYTE(CAN_DATA_MAX_PACKET_SIZE),
	0x00,                             /* bInterval: */
	/*---------------------------------------------------------------------------*/

	/*---------------------------------------------------------------------------*/
	/* EP2 descriptor */
	0x07,                             /* bLength */
	USB_DESC_TYPE_ENDPOINT,           /* bDescriptorType */
	GSUSB_ENDPOINT_OUT,               /* bEndpointAddress */
	0x02,                             /* bmAttributes: bulk */
	LOBYTE(CAN_DATA_MAX_PACKET_SIZE), /* wMaxPacketSize */
	HIBYTE(CAN_DATA_MAX_PACKET_SIZE),
	0x00,                             /* bInterval: */
	/*---------------------------------------------------------------------------*/

	/*---------------------------------------------------------------------------*/
	/* DFU Interface Descriptor */
	/*---------------------------------------------------------------------------*/
	0x09,                             /* bLength */
	USB_DESC_TYPE_INTERFACE,          /* bDescriptorType */
	DFU_INTERFACE_NUM,                /* bInterfaceNumber */
	0x00,                             /* bAlternateSetting */
	0x00,                             /* bNumEndpoints */
	0xFE,                             /* bInterfaceClass: Vendor Specific*/
	0x01,                             /* bInterfaceSubClass */
	0x01,                             /* bInterfaceProtocol : Runtime mode */
	DFU_INTERFACE_STR_INDEX,          /* iInterface */

	/*---------------------------------------------------------------------------*/
	/* Run-Time DFU Functional Descriptor */
	/*---------------------------------------------------------------------------*/
	0x09,                             /* bLength */
	0x21,                             /* bDescriptorType: DFU FUNCTIONAL */
	0x0B,                             /* bmAttributes: detach, upload, download */
	0xFF, 0x00,                       /* wDetachTimeOut */
	0x00, 0x08,                       /* wTransferSize */
	0x1a, 0x01,                       /* bcdDFUVersion: 1.1a */

	/* CDC ACM: IAD groups interfaces 2 (control) and 3 (data). */
	0x08,                             /* bLength */
	0x0B,
	CDC_CONTROL_INTERFACE,
	0x02,
	0x02,
	0x02,
	0x01,
	0x00,

	0x09,                             /* bLength */
	USB_DESC_TYPE_INTERFACE,		  /* bDescriptorType */
	CDC_CONTROL_INTERFACE,			  /* bInterfaceNumber */
	0x00,							  /* bAlternateSetting */
	0x01,							  /* bNumEndpoints */
	0x02,							  /* bInterfaceClass */
	0x02,							  /* bInterfaceSubClass */
	0x01,							  /* bInterfaceProtocol */
	CDC_INTERFACE_STR_INDEX,		  /* iInterface: */

	/* Header Functional Descriptor */
	0x05,							  /* bLength: Endpoint Descriptor size */
	0x24,							  /* bDescriptorType: CS_INTERFACE */
	0x00,							  /* bDescriptorSubtype: Header Func Desc */
	0x10,							  /* bcdCDC: spec release number */
	0x01,

	/* CDC header, version 1.10 */
	0x05,
	0x24,
	0x01,
	0x00,
	CDC_DATA_INTERFACE,

	/* Call management */
	0x04,
	0x24,
	0x02,
	0x02,

	/* ACM: line coding and control line state */
	0x05,
	0x24,
	0x06,
	CDC_CONTROL_INTERFACE,
	CDC_DATA_INTERFACE,

	/* Union */
	0x07,
	USB_DESC_TYPE_ENDPOINT,
	CDC_CMD_EP,
	0x03,
	CDC_CMD_PACKET_SIZE,
	0x00,
	0x10,

	0x09,
	USB_DESC_TYPE_INTERFACE,
	CDC_DATA_INTERFACE,
	0x00,
	0x02,
	0x0A,
	0x00,
	0x00,
	0x00,

	0x07,
	USB_DESC_TYPE_ENDPOINT,
	CDC_OUT_EP,
	0x02,
	CDC_PACKET_SIZE,
	0x00,
	0x00,

	0x07,
	USB_DESC_TYPE_ENDPOINT,
	CDC_IN_EP,
	0x02,
	CDC_PACKET_SIZE,
	0x00,
	0x00,
};

/* Microsoft OS String Descriptor */
__ALIGN_BEGIN static uint8_t USBD_WINUSB_STR[] __ALIGN_END =
{
	0x12,                    /* length */
	0x03,                    /* descriptor type == string */
	0x4D, 0x00, 0x53, 0x00,  /* signature: "MSFT100" */
	0x46, 0x00, 0x54, 0x00,
	0x31, 0x00, 0x30, 0x00,
	0x30, 0x00,
	USBD_GS_CAN_VENDOR_CODE, /* vendor code */
	0x00                     /* padding */
};

/* Microsoft Compatible ID Feature Descriptor */
__ALIGN_BEGIN static uint8_t USBD_MS_COMP_ID_FEATURE_DESC[] __ALIGN_END = {
	0x40, 0x00, 0x00, 0x00, /* length */
	0x00, 0x01,             /* version 1.0 */
	0x04, 0x00,             /* descr index (0x0004) */
	0x02,                   /* number of sections */
	0x00, 0x00, 0x00, 0x00, /* reserved */
	0x00, 0x00, 0x00,
	0x00,                   /* interface number */
	0x01,                   /* reserved */
	0x57, 0x49, 0x4E, 0x55, /* compatible ID ("WINUSB\0\0") */
	0x53, 0x42, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, /* sub-compatible ID */
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, /* reserved */
	0x00, 0x00,
	0x01,                   /* interface number */
	0x01,                   /* reserved */
	0x57, 0x49, 0x4E, 0x55, /* compatible ID ("WINUSB\0\0") */
	0x53, 0x42, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, /* sub-compatible ID */
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, /* reserved */
	0x00, 0x00
};

/* Microsoft Extended Properties Feature Descriptor */
__ALIGN_BEGIN static uint8_t USBD_MS_EXT_PROP_FEATURE_DESC[] __ALIGN_END= {
	0x92, 0x00, 0x00, 0x00, /* length */
	0x00, 0x01,             /* version 1.0 */
	0x05, 0x00,             /* descr index (0x0005) */
	0x01, 0x00,             /* number of sections */
	0x88, 0x00, 0x00, 0x00, /* property section size */
	0x07, 0x00, 0x00, 0x00, /* property data type 7: Unicode REG_MULTI_SZ */
	0x2a, 0x00,             /* property name length */

	0x44, 0x00, 0x65, 0x00, /* property name "DeviceInterfaceGUIDs" */
	0x76, 0x00, 0x69, 0x00,
	0x63, 0x00, 0x65, 0x00,
	0x49, 0x00, 0x6e, 0x00,
	0x74, 0x00, 0x65, 0x00,
	0x72, 0x00, 0x66, 0x00,
	0x61, 0x00, 0x63, 0x00,
	0x65, 0x00, 0x47, 0x00,
	0x55, 0x00, 0x49, 0x00,
	0x44, 0x00, 0x73, 0x00,
	0x00, 0x00,

	0x50, 0x00, 0x00, 0x00, /* property data length */

	0x7b, 0x00, 0x63, 0x00, /* property name: "{c15b4308-04d3-11e6-b3ea-6057189e6443}\0\0" */
	0x31, 0x00, 0x35, 0x00,
	0x62, 0x00, 0x34, 0x00,
	0x33, 0x00, 0x30, 0x00,
	0x38, 0x00, 0x2d, 0x00,
	0x30, 0x00, 0x34, 0x00,
	0x64, 0x00, 0x33, 0x00,
	0x2d, 0x00, 0x31, 0x00,
	0x31, 0x00, 0x65, 0x00,
	0x36, 0x00, 0x2d, 0x00,
	0x62, 0x00, 0x33, 0x00,
	0x65, 0x00, 0x61, 0x00,
	0x2d, 0x00, 0x36, 0x00,
	0x30, 0x00, 0x35, 0x00,
	0x37, 0x00, 0x31, 0x00,
	0x38, 0x00, 0x39, 0x00,
	0x65, 0x00, 0x36, 0x00,
	0x34, 0x00, 0x34, 0x00,
	0x33, 0x00, 0x7d, 0x00,
	0x00, 0x00, 0x00, 0x00
};

static uint8_t USBD_COMPOSITE_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx) {
	uint8_t ret = USBD_OK;
	if (USBD_GS_CAN.Init != NULL) ret |= USBD_GS_CAN.Init(pdev, cfgidx);
	if (USBD_CDC.Init != NULL) ret |= USBD_CDC.Init(pdev, cfgidx);
	if (USBD_DFU.Init != NULL) ret |= USBD_DFU.Init(pdev, cfgidx);
	return ret;
}

static uint8_t USBD_COMPOSITE_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx) {
	uint8_t ret = USBD_OK;
	if (USBD_GS_CAN.DeInit != NULL) ret |= USBD_GS_CAN.DeInit(pdev, cfgidx);
	if (USBD_CDC.DeInit != NULL) ret |= USBD_CDC.DeInit(pdev, cfgidx);
	if (USBD_DFU.DeInit != NULL) ret |= USBD_DFU.DeInit(pdev, cfgidx);
	return ret;
}

static uint8_t USBD_COMPOSITE_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req) {
	if ((req->bmRequest & 0x1F) == USB_REQ_RECIPIENT_INTERFACE &&
	    (req->wIndex == CDC_CONTROL_INTERFACE || req->wIndex == CDC_DATA_INTERFACE)) {
		if (USBD_CDC.Setup != NULL) return USBD_CDC.Setup(pdev, req);
	} else if ((req->bmRequest & 0x1F) == USB_REQ_RECIPIENT_INTERFACE &&     // recipient: interface
	           (req->bmRequest & USB_REQ_TYPE_MASK) == USB_REQ_TYPE_CLASS && // class request
	           req->wIndex == DFU_INTERFACE_NUM) {
		if (USBD_DFU.Setup != NULL) return USBD_DFU.Setup(pdev, req);
	} else {
		if (USBD_GS_CAN.Setup != NULL) return USBD_GS_CAN.Setup(pdev, req);
	}
	return USBD_OK;
}

static uint8_t USBD_COMPOSITE_EP0_RxReady(USBD_HandleTypeDef *pdev) {
	if ((pdev->request.bmRequest & 0x1F) == USB_REQ_RECIPIENT_INTERFACE &&
	    (pdev->request.wIndex == CDC_CONTROL_INTERFACE || pdev->request.wIndex == CDC_DATA_INTERFACE))
		if (USBD_CDC.EP0_RxReady != NULL) return USBD_CDC.EP0_RxReady(pdev);
	if (USBD_GS_CAN.EP0_RxReady != NULL) return USBD_GS_CAN.EP0_RxReady(pdev);
	return USBD_OK;
}

static uint8_t USBD_COMPOSITE_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum) {
	if (epnum == (CDC_IN_EP & 0x7F))
		if (USBD_CDC.DataIn != NULL) return USBD_CDC.DataIn(pdev, epnum);
	if (epnum == (GSUSB_ENDPOINT_IN & 0x7F))
		if (USBD_GS_CAN.DataIn != NULL) return USBD_GS_CAN.DataIn(pdev, epnum);
	return USBD_OK;
}

static uint8_t USBD_COMPOSITE_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum) {
	if (epnum == CDC_OUT_EP)
		if (USBD_CDC.DataOut != NULL) return USBD_CDC.DataOut(pdev, epnum);
	if (epnum == GSUSB_ENDPOINT_OUT)
		if (USBD_GS_CAN.DataOut != NULL) return USBD_GS_CAN.DataOut(pdev, epnum);
	return USBD_OK;
}

static uint8_t USBD_COMPOSITE_SOF(struct _USBD_HandleTypeDef *pdev) {
	if (USBD_GS_CAN.SOF != NULL) return USBD_GS_CAN.SOF(pdev);
	if (USBD_CDC.SOF != NULL) return USBD_CDC.SOF(pdev);
	if (USBD_DFU.SOF != NULL) return USBD_DFU.SOF(pdev);
	return USBD_OK;
}

static uint8_t *USBD_COMPOSITE_GetCfgDesc(uint16_t *length)
{
	*length = (uint16_t)sizeof(USBD_COMPOSITE_CfgDesc);
	return USBD_COMPOSITE_CfgDesc;
}

static uint8_t *USBD_COMPOSITE_GetStrDesc(USBD_HandleTypeDef *pdev, uint8_t index, uint16_t *length)
{
	switch (index) {
		case DFU_INTERFACE_STR_INDEX:
			USBD_GetString((uint8_t *)DFU_INTERFACE_STRING_FS, USBD_StrDesc, length);
			return USBD_StrDesc;
		case CDC_INTERFACE_STR_INDEX:
			USBD_GetString((uint8_t *)CDC_INTERFACE_STRING_FS, USBD_StrDesc, length);
			return USBD_StrDesc;
		case 0xEE:
			*length = (uint16_t)sizeof(USBD_WINUSB_STR);
			return USBD_WINUSB_STR;
		default:
			*length = 0;
			USBD_CtlError(pdev, 0);
			return 0;
	}
}

/* CAN interface class callbacks structure */
USBD_ClassTypeDef USBD_COMPOSITE = {
	.Init = USBD_COMPOSITE_Init,
	.DeInit = USBD_COMPOSITE_DeInit,
	.Setup = USBD_COMPOSITE_Setup,
	.EP0_RxReady = USBD_COMPOSITE_EP0_RxReady,
	.DataIn = USBD_COMPOSITE_DataIn,
	.DataOut = USBD_COMPOSITE_DataOut,
	.SOF = USBD_COMPOSITE_SOF,
	.GetHSConfigDescriptor = USBD_COMPOSITE_GetCfgDesc,
	.GetFSConfigDescriptor = USBD_COMPOSITE_GetCfgDesc,
	.GetOtherSpeedConfigDescriptor = USBD_COMPOSITE_GetCfgDesc,
	.GetUsrStrDescriptor = USBD_COMPOSITE_GetStrDesc,
};

bool USBD_CustomDeviceRequest(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
	/* CDC SET_LINE_CODING also uses request 0x20. Only intercept vendor IN. */
	if ((req->bmRequest & 0xE0) == 0xC0 && req->bRequest == USBD_GS_CAN_VENDOR_CODE) {
		switch (req->wIndex) {
			case 0x0004:
				USBD_CtlSendData(pdev, USBD_MS_COMP_ID_FEATURE_DESC, MIN(sizeof(USBD_MS_COMP_ID_FEATURE_DESC), req->wLength));
				return true;
			case 0x0005:
				if (req->wValue == 0) { // only return our GUID for interface #0
					USBD_CtlSendData(pdev, USBD_MS_EXT_PROP_FEATURE_DESC, MIN(sizeof(USBD_MS_EXT_PROP_FEATURE_DESC), req->wLength));
					return true;
				}
				break;
		}
	}

	return false;
}

bool USBD_CustomInterfaceRequest(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
	return USBD_CustomDeviceRequest(pdev, req);
}
