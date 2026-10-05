#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "usbd_def.h"

#define CDC_CONTROL_INTERFACE       2
#define CDC_DATA_INTERFACE          3
#define CDC_IN_EP                   0x83
#define CDC_OUT_EP                  0x03
#define CDC_CMD_EP                  0x84

#define CDC_BINTERVAL               0x10    // Interrupt polling interval in ms

#define CDC_DATA_MAX_PACKET_SIZE    64
#define CDC_CMD_PACKET_SIZE         8
#define CDC_INTERFACE_STR_INDEX	    0xE2
#define CDC_INTERFACE_STRING_FS	    "Serial Interface"

/*---------------------------------------------------------------------*/
/*  CDC definitions                                                    */
/*---------------------------------------------------------------------*/
#define CDC_SEND_ENCAPSULATED_COMMAND               0x00U
#define CDC_GET_ENCAPSULATED_RESPONSE               0x01U
#define CDC_SET_COMM_FEATURE                        0x02U
#define CDC_GET_COMM_FEATURE                        0x03U
#define CDC_CLEAR_COMM_FEATURE                      0x04U
#define CDC_SET_LINE_CODING                         0x20U
#define CDC_GET_LINE_CODING                         0x21U
#define CDC_SET_CONTROL_LINE_STATE                  0x22U
#define CDC_SEND_BREAK                              0x23U

typedef struct {
    uint32_t bitrate;
    uint8_t format;
    uint8_t paritytype;
    uint8_t datatype;
} USBD_CDC_LineCodingTypeDef;

extern USBD_ClassTypeDef USBD_CDC;

#ifdef __cplusplus
}

#include <algorithm>

class USBD_CDC_HandleTypeDef {
public:
    uint8_t Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx);

    uint8_t DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx);

    uint8_t Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);

    uint8_t EP0_RxReady(USBD_HandleTypeDef *pdev);

    uint8_t DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum);

    uint8_t DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum);

    uint8_t Control(uint8_t bRequest, uint16_t wValue);

    uint8_t CDC_Transmit_FS(uint8_t *buffer, uint16_t length);

private:
    union ep0 {
        // Device -> Host

        // Host -> Device

        // Device <-> Host
        USBD_CDC_LineCodingTypeDef line_coding;

        alignas(4) uint8_t buf[std::max({
            sizeof(line_coding),
        })];
    } ep0{};

    bool transmitting{false};
    bool zlp_request{false};
    uint8_t rx_buffer[CDC_DATA_MAX_PACKET_SIZE]{};
    USBD_HandleTypeDef *device;

    USBD_CDC_LineCodingTypeDef line_coding{.bitrate = 115200, .format = 0, .paritytype = 0, .datatype = 8};

    USBD_SetupReqTypedef last_setup_request{};
};

/* Nonblocking, copied TX: 0..64 bytes, USBD_BUSY means retry later.
 * Read returns bytes consumed; unread data NAKs the host instead of dropping.
 * Call from the main loop. Buffers may be reused immediately after return. */
uint8_t CDC_Transmit_FS(const uint8_t *buffer, uint16_t length);

#endif
