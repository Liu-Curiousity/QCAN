/**
 * @brief 		usbd_gs_usb.h库文件
 * @detail
 * @author 	    Haoqi Liu
 * @date        2026/9/22
 * @version 	V1.0.0
 * @note 		
 * @warning	    
 * @par 		历史版本
                V1.0.0创建于2026/9/22
 * */

#ifndef QCAN_USBD_GS_USB_H
#define QCAN_USBD_GS_USB_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include "usbd_def.h"

extern USBD_ClassTypeDef USBD_GS_CAN;

void USBD_GS_CAN_SuspendCallback(USBD_HandleTypeDef *pdev);
void USBD_GS_CAN_ResumeCallback(USBD_HandleTypeDef *pdev);
void USBD_GS_CAN_ReceiveFromHost(USBD_HandleTypeDef *pdev);
void USBD_GS_CAN_SendToHost(USBD_HandleTypeDef *pdev);
bool USBD_CustomDeviceRequest(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
bool USBD_CustomInterfaceRequest(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);

bool USBD_GS_CAN_DfuDetachRequested(USBD_HandleTypeDef *pdev);

#ifdef __cplusplus
};
#endif

#endif //QCAN_USBD_GS_USB_H
