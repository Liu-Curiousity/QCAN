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
#include "usbd_composite.h"

#include <stdlib.h>
#include <string.h>

#include "config.h"
#include "gs_usb.h"
#include "usbd_core.h"
#include "usbd_ctlreq.h"
#include "usbd_def.h"
#include "usbd_desc.h"

#include "usbd_dfu.h"
#include "usbd_cdc_if.h"
#include "usbd_gs_can.h"

USBD_COMPOSITE_HandleTypeDef hcomposite = {
	.pClass = {&USBD_CDC, &USBD_DFU, &USBD_GS_CAN},
};

/* Configuration Descriptor */
__ALIGN_BEGIN static uint8_t USBD_COMPOSITE_CfgDesc[] __ALIGN_END ={
	/*---------------------------------------------------------------------------*/
	/* Configuration Descriptor */
	0x09,								/* bLength */
	USB_DESC_TYPE_CONFIGURATION,      	/* bDescriptorType */
	USB_CAN_CONFIG_DESC_SIZ,          	/* wTotalLength */
	0x00,
	0x04,                             	/* bNumInterfaces */
	0x01,                             	/* bConfigurationValue */
	USBD_IDX_CONFIG_STR,              	/* iConfiguration */
	0x80,                             	/* bmAttributes */
	0x4B,                             	/* MaxPower 150 mA */
	/*---------------------------------------------------------------------------*/

	/*---------------------------------------------------------------------------*/
	/* GS_USB Interface Descriptor */
	0x09,                             	/* bLength */
	USB_DESC_TYPE_INTERFACE,          	/* bDescriptorType */
	GS_CAN_INTERFACE_NUM,             	/* bInterfaceNumber */
	0x00,                             	/* bAlternateSetting */
	0x02,                             	/* bNumEndpoints */
	0xFF,                             	/* bInterfaceClass: Vendor Specific*/
	0xFF,                             	/* bInterfaceSubClass: Vendor Specific */
	0xFF,                             	/* bInterfaceProtocol: Vendor Specific */
	GS_CAN_INTERFACE_STR_INDEX,		  	/* iInterface */
	/*---------------------------------------------------------------------------*/

	/*---------------------------------------------------------------------------*/
	/* EP1 descriptor */
	0x07,                             	/* bLength */
	USB_DESC_TYPE_ENDPOINT,           	/* bDescriptorType */
	GSUSB_ENDPOINT_IN,                	/* bEndpointAddress */
	0x02,                             	/* bmAttributes: bulk */
	LOBYTE(CAN_DATA_MAX_PACKET_SIZE), 	/* wMaxPacketSize */
	HIBYTE(CAN_DATA_MAX_PACKET_SIZE),
	0x00,								/* bInterval: */
	/*---------------------------------------------------------------------------*/

	/*---------------------------------------------------------------------------*/
	/* EP2 descriptor */
	0x07,                             	/* bLength */
	USB_DESC_TYPE_ENDPOINT,           	/* bDescriptorType */
	GSUSB_ENDPOINT_OUT,               	/* bEndpointAddress */
	0x02,                             	/* bmAttributes: bulk */
	LOBYTE(CAN_DATA_MAX_PACKET_SIZE), 	/* wMaxPacketSize */
	HIBYTE(CAN_DATA_MAX_PACKET_SIZE),
	0x00,								/* bInterval: */
	/*---------------------------------------------------------------------------*/

	/*---------------------------------------------------------------------------*/
	/* DFU Interface Descriptor */
	0x09,                             	/* bLength */
	USB_DESC_TYPE_INTERFACE,          	/* bDescriptorType */
	DFU_INTERFACE_NUM,                	/* bInterfaceNumber */
	0x00,                             	/* bAlternateSetting */
	0x00,                             	/* bNumEndpoints */
	0xFE,                             	/* bInterfaceClass: Vendor Specific*/
	0x01,                             	/* bInterfaceSubClass */
	0x01,                             	/* bInterfaceProtocol : Runtime mode */
	DFU_INTERFACE_STR_INDEX,          	/* iInterface */
	/*---------------------------------------------------------------------------*/

	/*---------------------------------------------------------------------------*/
	/* Run-Time DFU Functional Descriptor */
	0x09,                             	/* bLength */
	0x21,                             	/* bDescriptorType: DFU FUNCTIONAL */
	0x0B,                             	/* bmAttributes: detach, upload, download */
	0xFF, 0x00,                       	/* wDetachTimeOut */
	0x00, 0x08,                       	/* wTransferSize */
	0x1a, 0x01,                       	/* bcdDFUVersion: 1.1a */
	/*---------------------------------------------------------------------------*/

	/*---------------------------------------------------------------------------*/
	/* CDC ACM: IAD groups interfaces 2 (control) and 3 (data). */
	0x08,                             	/* bLength */
	0x0B,							  	/* bDescriptorType: USB_DESC_TYPE_IAD */
	CDC_CONTROL_INTERFACE,			  	/* bFirstInterface */
	0x02,							  	/* bInterfaceCount */
	0x02,							  	/* bFunctionClass */
	0x02,							  	/* bFunctionSubClass */
	0x01,							  	/* bFunctionProtocol */
	0x00,							  	/* iFunction: */
	/*---------------------------------------------------------------------------*/

	/*---------------------------------------------------------------------------*/
	/* CDC CMD Interface Descriptor */
	0x09,                             	/* bLength */
	USB_DESC_TYPE_INTERFACE,		  	/* bDescriptorType */
	CDC_CONTROL_INTERFACE,			  	/* bInterfaceNumber */
	0x00,							  	/* bAlternateSetting */
	0x01,							  	/* bNumEndpoints */
	0x02,							  	/* bInterfaceClass */
	0x02,							  	/* bInterfaceSubClass */
	0x01,							  	/* bInterfaceProtocol */
	CDC_INTERFACE_STR_INDEX,		  	/* iInterface: */
	/*---------------------------------------------------------------------------*/

	/*---------------------------------------------------------------------------*/
	/* Header Functional Descriptor */
	0x05,							  	/* bLength: Endpoint Descriptor size */
	0x24,							  	/* bDescriptorType: CS_INTERFACE */
	0x00,							  	/* bDescriptorSubtype: Header Func Desc */
	0x10,							  	/* bcdCDC: spec release number */
	0x01,
	/*---------------------------------------------------------------------------*/

	/*---------------------------------------------------------------------------*/
	/* Call Management Functional Descriptor */
	0x05,							  	/* bFunctionLength */
	0x24,							  	/* bDescriptorType: CS_INTERFACE */
	0x01,							  	/* bDescriptorSubtype: Call Management Func Desc */
	0x00,							  	/* bmCapabilities: D0+D1 */
	CDC_DATA_INTERFACE,				  	/* bDataInterface: 1 */
	/*---------------------------------------------------------------------------*/

	/*---------------------------------------------------------------------------*/
	/* ACM Functional Descriptor */
	0x04,							  	/* bFunctionLength */
	0x24,							  	/* bDescriptorType: CS_INTERFACE */
	0x02,							  	/* bDescriptorSubtype: Abstract Control Management desc */
	0x02,							  	/* bmCapabilities */
	/*---------------------------------------------------------------------------*/

	/*---------------------------------------------------------------------------*/
	/* Union Functional Descriptor */
	0x05,							  	/* bFunctionLength */
	0x24,							  	/* bDescriptorType: CS_INTERFACE */
	0x06,							  	/* bDescriptorSubtype: Union func desc */
	CDC_CONTROL_INTERFACE,			  	/* bMasterInterface: Communication class interface */
	CDC_DATA_INTERFACE,				  	/* bSlaveInterface0: Data Class Interface */
	/*---------------------------------------------------------------------------*/

	/*---------------------------------------------------------------------------*/
	/* Endpoint CMD Endpoint */
	0x07,								/* bLength: Endpoint Descriptor size */
	USB_DESC_TYPE_ENDPOINT,				/* bDescriptorType: Endpoint */
	CDC_CMD_EP,							/* bEndpointAddress */
	0x03,								/* bmAttributes: Interrupt */
	LOBYTE(CDC_CMD_PACKET_SIZE),		/* wMaxPacketSize: */
	HIBYTE(CDC_CMD_PACKET_SIZE),
	CDC_BINTERVAL,						/* bInterval: */
	/*---------------------------------------------------------------------------*/

	/*---------------------------------------------------------------------------*/
	/* CDC DATA Interface Descriptor */
	0x09,								/* bLength: Endpoint Descriptor size */
	USB_DESC_TYPE_INTERFACE,			/* bDescriptorType: */
	CDC_DATA_INTERFACE,					/* bInterfaceNumber: Number of Interface */
	0x00,								/* bAlternateSetting: Alternate setting */
	0x02,								/* bNumEndpoints: Two endpoints used */
	0x0A,								/* bInterfaceClass: CDC */
	0x00,								/* bInterfaceSubClass: */
	0x00,								/* bInterfaceProtocol: */
	0x00,								/* iInterface: */
	/*---------------------------------------------------------------------------*/

	/*---------------------------------------------------------------------------*/
	/* Endpoint OUT Descriptor */
	0x07,								/* bLength: Endpoint Descriptor size */
	USB_DESC_TYPE_ENDPOINT,				/* bDescriptorType: Endpoint */
	CDC_OUT_EP,							/* bEndpointAddress */
	0x02,								/* bmAttributes: Bulk */
	LOBYTE(CDC_DATA_MAX_PACKET_SIZE),	/* wMaxPacketSize: */
	HIBYTE(CDC_DATA_MAX_PACKET_SIZE),
	0x00,								/* bInterval: ignore for Bulk transfer */
	/*---------------------------------------------------------------------------*/

	/*---------------------------------------------------------------------------*/
	/* Endpoint IN Descriptor */
	0x07,								/* bLength: Endpoint Descriptor size */
	USB_DESC_TYPE_ENDPOINT,				/* bDescriptorType: Endpoint */
	CDC_IN_EP,							/* bEndpointAddress */
	0x02,								/* bmAttributes: Bulk */
	LOBYTE(CDC_DATA_MAX_PACKET_SIZE),	/* wMaxPacketSize: */
	HIBYTE(CDC_DATA_MAX_PACKET_SIZE),
	0x00,								/* bInterval: ignore for Bulk transfer */
	/*---------------------------------------------------------------------------*/
};

/* Default configuration: function subsets keep WinUSB off the CDC interfaces. */
__ALIGN_BEGIN static uint8_t USBD_MS_OS_20_DESC[] __ALIGN_END = {
	/* Descriptor set header: Windows 8.1 and later. */
	0x0A, 0x00,							/* wLength */
	0x00, 0x00,							/* wDescriptorType */
	0x00, 0x00, 0x03, 0x06,				/* dwWindowsVersion */
	LOBYTE(USBD_MS_OS_20_DESC_SIZE), HIBYTE(USBD_MS_OS_20_DESC_SIZE),/* wDescriptorSetTotalLength */

	/* CAN function subset: 8 + 20 + 132 bytes. */
	0x08, 0x00,							/* wLength */
	0x02, 0x00,							/* wDescriptorType */
	0x00,								/* bFirstInterface */
	0x00,								/* bReserved */
	0xA0, 0x00,							/* wSubsetLength */

	/* Compatible ID: WINUSB */
	0x14, 0x00,							/* wLength */
	0x03, 0x00,							/* wDescriptorType */
	'W', 'I', 'N', 'U', 'S', 'B', 0x00, 0x00,/* CompatibleID */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,/* SubCompatibleID */

	/* Registry property: DeviceInterfaceGUIDs, REG_MULTI_SZ (UTF-16LE). */
	0x84, 0x00,							/* wLength */
	0x04, 0x00,							/* wDescriptorType */
	0x07, 0x00,							/* wPropertyDataType */
	0x2A, 0x00,							/* wPropertyNameLength */
	/* "DeviceInterfaceGUIDs" */
	'D', 0x00, 'e', 0x00, 'v', 0x00, 'i', 0x00,
	'c', 0x00, 'e', 0x00, 'I', 0x00, 'n', 0x00,
	't', 0x00, 'e', 0x00, 'r', 0x00, 'f', 0x00,
	'a', 0x00, 'c', 0x00, 'e', 0x00, 'G', 0x00,
	'U', 0x00, 'I', 0x00, 'D', 0x00, 's', 0x00,
	0x00, 0x00,
	0x50, 0x00,							/* wPropertyDataLength */
	/* "{c15b4308-04d3-11e6-b3ea-6057189e6443}\0\0" */
	0x7b, 0x00, 0x63, 0x00, 0x31, 0x00, 0x35, 0x00,
	0x62, 0x00, 0x34, 0x00, 0x33, 0x00, 0x30, 0x00,
	0x38, 0x00, 0x2d, 0x00, 0x30, 0x00, 0x34, 0x00,
	0x64, 0x00, 0x33, 0x00, 0x2d, 0x00, 0x31, 0x00,
	0x31, 0x00, 0x65, 0x00, 0x36, 0x00, 0x2d, 0x00,
	0x62, 0x00, 0x33, 0x00, 0x65, 0x00, 0x61, 0x00,
	0x2d, 0x00, 0x36, 0x00, 0x30, 0x00, 0x35, 0x00,
	0x37, 0x00, 0x31, 0x00, 0x38, 0x00, 0x39, 0x00,
	0x65, 0x00, 0x36, 0x00, 0x34, 0x00, 0x34, 0x00,
	0x33, 0x00, 0x7d, 0x00, 0x00, 0x00, 0x00, 0x00,

	/* DFU function subset: WinUSB, without the CAN interface GUID. */
	0x08, 0x00, 						/* wLength */
	0x02, 0x00, 						/* wDescriptorType */
	DFU_INTERFACE_NUM,					/* bFirstInterface */
	0x00,								/* bReserved */
	0x1C, 0x00,							/* wSubsetLength */

	0x14, 0x00, 						/* wLength */
	0x03, 0x00,							/* wDescriptorType */
	'W', 'I', 'N', 'U', 'S', 'B', 0x00, 0x00,/* CompatibleID */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,/* SubCompatibleID */
};

static uint8_t USBD_COMPOSITE_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx) {
	uint8_t ret = USBD_OK;
	const USBD_COMPOSITE_HandleTypeDef *device = pdev->pClassData;
	for (int i = 0; i < COMPOSITE_DEVICE_NUM; i++) {
		if (device->pClass[i] != NULL && device->pClass[i]->Init != NULL) {
			ret |= device->pClass[i]->Init(pdev, cfgidx);
		}
	}
	return ret;
}

static uint8_t USBD_COMPOSITE_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx) {
	uint8_t ret = USBD_OK;
	const USBD_COMPOSITE_HandleTypeDef *device = pdev->pClassData;
	for (int i = 0; i < COMPOSITE_DEVICE_NUM; i++) {
		if (device->pClass[i] != NULL && device->pClass[i]->DeInit != NULL) {
			ret |= device->pClass[i]->DeInit(pdev, cfgidx);
		}
	}
	return ret;
}

static uint8_t USBD_COMPOSITE_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req) {
	uint8_t ret = USBD_OK;
	if ((req->bmRequest & 0x1F) == USB_REQ_RECIPIENT_DEVICE &&
	    req->bRequest == USBD_GS_CAN_VENDOR_CODE) {
		USBD_CtlSendData(pdev, USBD_MS_OS_20_DESC, MIN(sizeof(USBD_MS_OS_20_DESC), req->wLength));
		return USBD_OK;
	}
	const USBD_COMPOSITE_HandleTypeDef *device = pdev->pClassData;
	for (int i = 0; i < COMPOSITE_DEVICE_NUM; i++) {
		if (device->pClass[i] != NULL && device->pClass[i]->Setup != NULL) {
			ret |= device->pClass[i]->Setup(pdev, req);
		}
	}
	return ret;
}

static uint8_t USBD_COMPOSITE_EP0_RxReady(USBD_HandleTypeDef *pdev) {
	uint8_t ret = USBD_OK;
	const USBD_COMPOSITE_HandleTypeDef *device = pdev->pClassData;
	for (int i = 0; i < COMPOSITE_DEVICE_NUM; i++) {
		if (device->pClass[i] != NULL && device->pClass[i]->EP0_RxReady != NULL) {
			ret |= device->pClass[i]->EP0_RxReady(pdev);
		}
	}
	return ret;
}

static uint8_t USBD_COMPOSITE_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum) {
	uint8_t ret = USBD_OK;
	const USBD_COMPOSITE_HandleTypeDef *device = pdev->pClassData;
	for (int i = 0; i < COMPOSITE_DEVICE_NUM; i++) {
		if (device->pClass[i] != NULL && device->pClass[i]->DataIn != NULL) {
			ret |= device->pClass[i]->DataIn(pdev, epnum);
		}
	}
	return ret;
}

static uint8_t USBD_COMPOSITE_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum) {
	uint8_t ret = USBD_OK;
	const USBD_COMPOSITE_HandleTypeDef *device = pdev->pClassData;
	for (int i = 0; i < COMPOSITE_DEVICE_NUM; i++) {
		if (device->pClass[i] != NULL && device->pClass[i]->DataOut != NULL) {
			ret |= device->pClass[i]->DataOut(pdev, epnum);
		}
	}
	return ret;
}

static uint8_t USBD_COMPOSITE_SOF(struct _USBD_HandleTypeDef *pdev) {
	uint8_t ret = USBD_OK;
	const USBD_COMPOSITE_HandleTypeDef *device = pdev->pClassData;
	for (int i = 0; i < COMPOSITE_DEVICE_NUM; i++) {
		if (device->pClass[i] != NULL && device->pClass[i]->SOF != NULL) {
			ret |= device->pClass[i]->SOF(pdev);
		}
	}
	return ret;
}

static uint8_t *USBD_COMPOSITE_GetCfgDesc(uint16_t *length)
{
	*length = (uint16_t)sizeof(USBD_COMPOSITE_CfgDesc);
	return USBD_COMPOSITE_CfgDesc;
}

static uint8_t *USBD_COMPOSITE_GetStrDesc(USBD_HandleTypeDef *pdev, uint8_t index, uint16_t *length)
{
	switch (index) {
		case GS_CAN_INTERFACE_STR_INDEX:
			USBD_GetString((uint8_t *)GS_CAN_INTERFACE_STRING_FS, USBD_StrDesc, length);
			return USBD_StrDesc;
		case DFU_INTERFACE_STR_INDEX:
			USBD_GetString((uint8_t *)DFU_INTERFACE_STRING_FS, USBD_StrDesc, length);
			return USBD_StrDesc;
		case CDC_INTERFACE_STR_INDEX:
			USBD_GetString((uint8_t *)CDC_INTERFACE_STRING_FS, USBD_StrDesc, length);
			return USBD_StrDesc;
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