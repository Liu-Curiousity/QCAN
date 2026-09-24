#include "can.h"
#include "usbd_composite.h"
#include "can_common.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_gs_can.h"

static USBD_COMPOSITE_HandleTypeDef hGS_CAN;
static USBD_HandleTypeDef hUSB;

void app_main() {
    INIT_LIST_HEAD(&hGS_CAN.list_frame_pool);
    INIT_LIST_HEAD(&hGS_CAN.list_to_host);

    for (unsigned i = 0; i < ARRAY_SIZE(hGS_CAN.msgbuf); i++) {
        list_add_tail(&hGS_CAN.msgbuf[i].list, &hGS_CAN.list_frame_pool);
    }

    for (unsigned int i = 0; i < ARRAY_SIZE(hGS_CAN.channels); i++) {
        can_data_t *channel = &hGS_CAN.channels[i];

        channel->nr = i;

        INIT_LIST_HEAD(&channel->list_from_host);

        led_init(&channel->leds,
                 LED_RX_GPIO_Port, LED_RX_Pin, 1,
                 LED_TX_GPIO_Port, LED_RX_Pin, 1);

        channel->hcan = &hcan;
        can_disable(&hGS_CAN, channel);
    }

    USBD_Init(&hUSB, &FS_Desc, DEVICE_FS);
    USBD_RegisterClass(&hUSB, &USBD_COMPOSITE);
    USBD_GS_CAN_Init_(&hGS_CAN, &hUSB);
    USBD_Start(&hUSB);
    while (true) {
        for (unsigned int i = 0; i < ARRAY_SIZE(hGS_CAN.channels); i++) {
            can_data_t *channel = &hGS_CAN.channels[i];

            CAN_SendFrame(&hGS_CAN, channel);
        }

        USBD_GS_CAN_ReceiveFromHost(&hUSB);
        USBD_GS_CAN_SendToHost(&hUSB);

        for (unsigned int i = 0; i < ARRAY_SIZE(hGS_CAN.channels); i++) {
            can_data_t *channel = &hGS_CAN.channels[i];

            CAN_ReceiveFrame(&hGS_CAN, channel);
            CAN_HandleError(&hGS_CAN, channel);

            led_update(&channel->leds);
        }
    }
}
