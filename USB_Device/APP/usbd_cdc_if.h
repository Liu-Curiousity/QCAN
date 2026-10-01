#pragma once

#include "usbd_def.h"

#define CDC_CONTROL_INTERFACE       2
#define CDC_DATA_INTERFACE          3
#define CDC_IN_EP                   0x83
#define CDC_OUT_EP                  0x03
#define CDC_CMD_EP                  0x84
#define CDC_PACKET_SIZE             64
#define CDC_CMD_PACKET_SIZE         8
#define CDC_CONFIG_DESC_SIZE        66
#define CDC_INTERFACE_STR_INDEX	    0xE2
#define CDC_INTERFACE_STRING_FS	    "Serial Interface"

/* Nonblocking, copied TX: 0..64 bytes, USBD_BUSY means retry later.
 * Read returns bytes consumed; unread data NAKs the host instead of dropping.
 * Call from the main loop. Buffers may be reused immediately after return. */
uint8_t CDC_Transmit_FS(const uint8_t *buffer, uint16_t length);
uint16_t CDC_Read_FS(uint8_t *buffer, uint16_t capacity);
/* Bit 0 = DTR, bit 1 = RTS; line coding is virtual (no UART reconfiguration). */
uint16_t CDC_GetControlLineState_FS(void);

extern USBD_ClassTypeDef USBD_CDC;