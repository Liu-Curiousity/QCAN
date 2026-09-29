#include "can.h"
#include "usbd_composite.h"
#include "can_common.h"
#include "led.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_gs_can.h"
#include "task_public.h"
#include "timer.h"

#include "FreeRTOS.h"
#include "queue.h"
#include <cstring>

extern USBD_COMPOSITE_HandleTypeDef hcomposite;
bxcan_ can_data{&hcan, 0};
USBD_GS_CAN_HandleTypeDef hGS_CAN{can_data};
static USBD_HandleTypeDef hUSB;

uint8_t USBD_GS_CAN_SendFrame(USBD_HandleTypeDef *pdev, gs_host_frame_object *frame_object);

void StartDebugTask(void *argument) {
    gs_host_frame_object rx_frame{};

    USBD_Init(&hUSB, &FS_Desc, DEVICE_FS);
    USBD_RegisterClass(&hUSB, &USBD_COMPOSITE);
    hUSB.pClassData = &hcomposite;
    USBD_Start(&hUSB);
    static bool datain_request = false;
    while (true) {
        for (unsigned int i = 0; i < hGS_CAN.channel_count; i++) {
            hGS_CAN.channels[i].CAN_SendFrame();
        }

        /*==========================从上位机接收帧============================*/
        USBD_GS_CAN_ReceiveFromHost(&hUSB);

        /*==========================向上位机发送帧============================*/
        if (!datain_request && xQueueReceive(hGS_CAN.xQueue_to_host, rx_frame._buf, 0) == pdPASS) {
            datain_request = true;
            led.blink_request(signal_led::LED_RX);
        }
        if (datain_request == true) {
            if (USBD_GS_CAN_SendFrame(&hUSB, &rx_frame) == USBD_OK) {
                datain_request = false;
            }
        }

        for (unsigned int i = 0; i < hGS_CAN.channel_count; i++) {
            // hGS_CAN.channels[i].CAN_HandleError(&hGS_CAN.list_to_host, &hGS_CAN.list_frame_pool);
            led.update();
        }
    }
}

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (hcan == &::hcan) {
        static bool queue_overflow = false;
        while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0)) {
            gs_host_frame_object rx_object{};
            gs_host_frame& rx_frame = rx_object.frame;
            CAN_RxHeaderTypeDef rx_header{};
            if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_frame.classic_can->data) != HAL_OK) {
                break;
            }
            rx_frame.echo_id = GS_HOST_FRAME_ECHO_ID_RX;
            rx_frame.classic_can_ts->timestamp_us = timer_get();
            rx_frame.can_dlc = rx_header.DLC;
            rx_frame.can_id = rx_header.IDE == CAN_ID_EXT ? rx_header.ExtId : rx_header.StdId;
            if (rx_header.IDE == CAN_ID_EXT) rx_frame.can_id |= CAN_EFF_FLAG;
            if (rx_header.RTR == CAN_RTR_REMOTE) rx_frame.can_id |= CAN_RTR_FLAG;
            rx_frame.channel = can_data.nr;
            rx_frame.flags = queue_overflow ? GS_CAN_FLAG_OVERFLOW : 0;

            queue_overflow = xQueueSendToBackFromISR(hGS_CAN.xQueue_to_host, rx_object._buf, &xHigherPriorityTaskWoken) != pdPASS;
        }
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}
