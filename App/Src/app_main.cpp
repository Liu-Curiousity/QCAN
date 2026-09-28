#include "can.h"
#include "usbd_composite.h"
#include "can_common.h"
#include "led.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_gs_can.h"

extern USBD_COMPOSITE_HandleTypeDef hcomposite;
can_data_t can_data[1]{{.hcan = &hcan}};
USBD_GS_CAN_HandleTypeDef hGS_CAN{can_data};
static USBD_HandleTypeDef hUSB;

extern "C" void app_main() {
    for (unsigned int i = 0; i < hGS_CAN.channel_count; i++) {
        can_disable(&hGS_CAN.list_to_host, &hGS_CAN.list_frame_pool, &hGS_CAN.channels[i]);
    }

    USBD_Init(&hUSB, &FS_Desc, DEVICE_FS);
    USBD_RegisterClass(&hUSB, &USBD_COMPOSITE);
    hUSB.pClassData = &hcomposite;
    USBD_Start(&hUSB);
    while (true) {
        for (unsigned int i = 0; i < hGS_CAN.channel_count; i++) {
            can_data_t *channel = &hGS_CAN.channels[i];

            CAN_SendFrame(&hGS_CAN.list_to_host, channel);
        }

        USBD_GS_CAN_ReceiveFromHost(&hUSB);
        USBD_GS_CAN_SendToHost(&hUSB);

        for (unsigned int i = 0; i < hGS_CAN.channel_count; i++) {
            can_data_t *channel = &hGS_CAN.channels[i];

            CAN_ReceiveFrame(&hGS_CAN.list_to_host, &hGS_CAN.list_frame_pool, channel);
            CAN_HandleError(&hGS_CAN.list_to_host, &hGS_CAN.list_frame_pool, channel);

            led.update();
        }
    }
}
