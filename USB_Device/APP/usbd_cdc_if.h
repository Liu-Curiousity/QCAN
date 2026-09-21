#pragma once

#include "usbd_def.h"

#define CDC_CONTROL_INTERFACE 2U
#define CDC_DATA_INTERFACE    3U
#define CDC_IN_EP             0x83U
#define CDC_OUT_EP            0x03U
#define CDC_CMD_EP            0x84U
#define CDC_PACKET_SIZE       64U
#define CDC_CMD_PACKET_SIZE   8U
#define CDC_CONFIG_DESC_SIZE  66U

/* Nonblocking, copied TX: 0..64 bytes, USBD_BUSY means retry later.
 * Read returns bytes consumed; unread data NAKs the host instead of dropping.
 * Call from the main loop. Buffers may be reused immediately after return. */
uint8_t CDC_Transmit_FS(const uint8_t *buffer, uint16_t length);
uint16_t CDC_Read_FS(uint8_t *buffer, uint16_t capacity);
/* Bit 0 = DTR, bit 1 = RTS; line coding is virtual (no UART reconfiguration). */
uint16_t CDC_GetControlLineState_FS(void);

/* Composite class callbacks; pClassData remains owned by GS_CAN. */
uint8_t QCAN_CDC_Init(USBD_HandleTypeDef *pdev);
void QCAN_CDC_DeInit(USBD_HandleTypeDef *pdev);
uint8_t QCAN_CDC_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
uint8_t QCAN_CDC_EP0_RxReady(USBD_HandleTypeDef *pdev);
uint8_t QCAN_CDC_DataIn(USBD_HandleTypeDef *pdev);
uint8_t QCAN_CDC_DataOut(USBD_HandleTypeDef *pdev);
