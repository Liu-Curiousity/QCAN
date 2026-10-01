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

#include "config.h"
#include "usbd_def.h"
#include "bxcan.h"

#define GS_CAN_INTERFACE_NUM		 0
#define GS_CAN_INTERFACE_STR_INDEX	 0xE0
#define GS_CAN_INTERFACE_STRING_FS	 "canable???"

/* Define these here so they can be referenced in other files */
#ifdef CONFIG_CANFD
#define CAN_DATA_MAX_PACKET_SIZE 64    /* Endpoint IN & OUT Packet size */
#else
#define CAN_DATA_MAX_PACKET_SIZE 32    /* Endpoint IN & OUT Packet size */
#endif

#ifdef __cplusplus

#include <algorithm>
#include "can_common.h"

class USBD_GS_CAN_HandleTypeDef {
public:
    // template <std::size_t N>
    explicit USBD_GS_CAN_HandleTypeDef(CAN_Channel& channels) :
        channels(&channels), channel_count(1), sof_timestamp_us(0) {
        xQueue_to_host = xQueueCreate(10, GS_HOST_FRAME_SIZE);
    }

    ~USBD_GS_CAN_HandleTypeDef() {
        vQueueDelete(xQueue_to_host);
    }

    xQueueHandle xQueue_to_host; // 向主机发送帧的队列

    CAN_Channel *channels;
    size_t channel_count{};

    [[nodiscard]] CAN_Channel* GetChannel(const uint16_t ch) const {
        return ch < channel_count ? &channels[ch] : nullptr;
    }

    uint8_t PrepareReceive(USBD_HandleTypeDef *pdev);

    uint8_t Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx);

    uint8_t DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx);

    uint8_t Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);

    uint8_t EP0_RxReady(USBD_HandleTypeDef *pdev);

    uint8_t DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum);

    uint8_t DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum);

    uint8_t SOF(USBD_HandleTypeDef *pdev);

    /* Composite control callbacks are broadcast. Only handle gs_usb vendor
     * requests, retaining support for device-recipient requests. */
    static bool IsRequest(const USBD_SetupReqTypedef *req) {
        return (req->bmRequest & USB_REQ_TYPE_MASK) == USB_REQ_TYPE_VENDOR &&
               ((req->bmRequest & 0x1F) == USB_REQ_RECIPIENT_DEVICE ||
                ((req->bmRequest & 0x1F) == USB_REQ_RECIPIENT_INTERFACE && req->wIndex == 0));
    }

private:
    union ep0 {
        // Device -> Host
        gs_device_state state;

        // Host -> Device
        gs_host_config config;
        gs_device_bittiming bittiming;
        gs_device_mode mode;
        gs_identify_mode identify_mode;

        // Device <-> Host
        gs_device_termination_state term_state;

        alignas(4) uint8_t buf[std::max({
            sizeof(gs_device_state),
            sizeof(gs_host_config),
            sizeof(gs_device_bittiming),
            sizeof(gs_device_mode),
            sizeof(gs_identify_mode),
            sizeof(gs_device_termination_state)
        })];
    } ep0{};

    uint32_t sof_timestamp_us;
    USBD_SetupReqTypedef last_setup_request{};
    gs_host_frame_object from_host_buf{}; // 从主机接收到帧后使用的双缓冲指针

    // device info
    const gs_device_config USBD_GS_CAN_dconf = {
        .icount = NUM_CAN_CHANNEL - 1,
        .sw_version = 2,
        .hw_version = 1,
    };
} __attribute__ ((aligned (4)));

extern "C" {
#endif

extern USBD_ClassTypeDef USBD_GS_CAN;

#ifdef __cplusplus
};
#endif

#endif //QCAN_USBD_GS_USB_H
