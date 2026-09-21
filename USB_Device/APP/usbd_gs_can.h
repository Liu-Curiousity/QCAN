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
#include <stdint.h>

#include "usbd_def.h"

#include "usbd_composite.h"

uint8_t USBD_GS_CAN_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
uint8_t USBD_GS_CAN_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
uint8_t USBD_GS_CAN_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
uint8_t USBD_GS_CAN_EP0_RxReady(USBD_HandleTypeDef *pdev);
uint8_t USBD_GS_CAN_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum);
uint8_t USBD_GS_CAN_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum);


uint8_t USBD_GS_CAN_Init_(USBD_COMPOSITE_HandleTypeDef *hcan, USBD_HandleTypeDef *pdev);
void USBD_GS_CAN_SuspendCallback(USBD_HandleTypeDef *pdev);
void USBD_GS_CAN_ResumeCallback(USBD_HandleTypeDef *pdev);
void USBD_GS_CAN_ReceiveFromHost(USBD_HandleTypeDef *pdev);
void USBD_GS_CAN_SendToHost(USBD_HandleTypeDef *pdev);
bool USBD_CustomDeviceRequest(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
bool USBD_CustomInterfaceRequest(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);

bool USBD_GS_CAN_DfuDetachRequested(USBD_HandleTypeDef *pdev);


void usbd_gs_can_purge_from_host_list_by_channel(USBD_COMPOSITE_HandleTypeDef *hcan,
                                                 struct can_channel *channel);

void usbd_gs_can_purge_to_host_list_by_channel(USBD_COMPOSITE_HandleTypeDef *hcan,
                                               const struct can_channel *channel);

#ifdef __cplusplus
};
#endif

#endif //QCAN_USBD_GS_USB_H
