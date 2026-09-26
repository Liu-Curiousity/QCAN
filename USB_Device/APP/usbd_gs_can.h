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
#include "bxcan.h"

/* Define these here so they can be referenced in other files */
#ifdef CONFIG_CANFD
#define CAN_DATA_MAX_PACKET_SIZE 64    /* Endpoint IN & OUT Packet size */
#else
#define CAN_DATA_MAX_PACKET_SIZE 32    /* Endpoint IN & OUT Packet size */
#endif

#ifdef CONFIG_CANFD
#define GS_HOST_FRAME_SIZE struct_size((struct gs_host_frame *)NULL, canfd_ts, 1)
#else
#define GS_HOST_FRAME_SIZE struct_size((struct gs_host_frame *)NULL, classic_can_ts, 1)
#endif

// When using double buffer for RX, this needs to be at least 2 to
// ensure there is always an RX buffer ready to receive the
// RX frames.
#if defined(USB) || defined(USB_DRD_FS)
#define USBD_GS_CAN_RX_BUFFER_COUNT 2
#else
#define USBD_GS_CAN_RX_BUFFER_COUNT 1
#endif

struct gs_host_frame_object {
    struct list_head list;

    union {
        uint8_t _buf[GS_HOST_FRAME_SIZE];
        struct gs_host_frame frame;
    };
};

union ep0 {
    union ep0_data {
        // Device -> Host
        struct gs_device_state state;

        // Host -> Device
        struct gs_host_config config;
        struct gs_device_bittiming bittiming;
        struct gs_device_mode mode;
        struct gs_identify_mode identify_mode;

        // Device <-> Host
        struct gs_device_termination_state term_state;
    } data;

    uint8_t __aligned(4) buf[sizeof(union ep0_data)];
};

typedef struct {
    union ep0 ep0;

    USBD_SetupReqTypedef last_setup_request;

    struct list_head list_frame_pool; // 共享空闲队列
    struct list_head list_to_host;    // 向主机发送帧的队列

    struct gs_host_frame_object *from_host_buf[USBD_GS_CAN_RX_BUFFER_COUNT]; // 从主机接收到帧后使用的双缓冲指针
    struct gs_host_frame_object *to_host_buf;                                // 向主机发送帧时使用的缓冲指针

    can_data_t channels[NUM_CAN_CHANNEL];

    uint32_t sof_timestamp_us;

    struct gs_host_frame_object msgbuf[CAN_QUEUE_SIZE]; // 共享池实际储存位置

} USBD_GS_CAN_HandleTypeDef __attribute__ ((aligned (4)));


extern USBD_ClassTypeDef USBD_GS_CAN;

void USBD_GS_CAN_SuspendCallback(USBD_HandleTypeDef *pdev);
void USBD_GS_CAN_ResumeCallback(USBD_HandleTypeDef *pdev);
void USBD_GS_CAN_ReceiveFromHost(USBD_HandleTypeDef *pdev);
void USBD_GS_CAN_SendToHost(USBD_HandleTypeDef *pdev);
bool USBD_CustomDeviceRequest(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
bool USBD_CustomInterfaceRequest(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
#ifdef __cplusplus
};
#endif

#endif //QCAN_USBD_GS_USB_H
