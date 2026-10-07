#include "can.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_composite.h"
#include "usbd_gs_can.h"
#include "usbd_cdc_if.h"
#include "timer.h"
#include "usbd_dfu.h"
#include "util.h"

extern USBD_COMPOSITE_HandleTypeDef hcomposite;

void USBD_GS_CAN_ReceiveFromHost(USBD_HandleTypeDef *pdev);

static signal_led led{
    {LED_RX_GPIO_Port, LED_RX_Pin, true},
    {LED_TX_GPIO_Port, LED_TX_Pin, true}
};

USBD_DFU_HandleTypeDef hDFU{};
USBD_CDC_HandleTypeDef hCDC{led};

static bxcan_ can_channel_0{
    0,
    &hcan,
    led,
    gs_device_bt_const{
        .feature =
        GS_CAN_FEATURE_LISTEN_ONLY |
        GS_CAN_FEATURE_LOOP_BACK |
        GS_CAN_FEATURE_ONE_SHOT |
        GS_CAN_FEATURE_HW_TIMESTAMP |
        GS_CAN_FEATURE_IDENTIFY |
        GS_CAN_FEATURE_PAD_PKTS_TO_MAX_PKT_SIZE |
        (IS_ENABLED(CONFIG_TERMINATION) ?
         GS_CAN_FEATURE_TERMINATION : 0) |
        GS_CAN_FEATURE_BERR_REPORTING |
        GS_CAN_FEATURE_GET_STATE |
        0,
        .fclk_can = CAN_CLOCK_SPEED,
        .btc = {
            .tseg1_min = 1,
            .tseg1_max = 16,
            .tseg2_min = 1,
            .tseg2_max = 8,
            .sjw_max = 4,
            .brp_min = 1,
            .brp_max = 1024,
            .brp_inc = 1,
        },
    }
};
USBD_GS_CAN_HandleTypeDef hGS_CAN{can_channel_0};
static USBD_HandleTypeDef hUSB;

static void USB_Disconnected() {
    __HAL_RCC_USB_FORCE_RESET();
    HAL_Delay(200);
    __HAL_RCC_USB_RELEASE_RESET();

    GPIO_InitTypeDef GPIO_Initure;
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_Initure.Pin = GPIO_PIN_11 | GPIO_PIN_12;
    GPIO_Initure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_Initure.Pull = GPIO_PULLDOWN;
    GPIO_Initure.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_Initure);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
    HAL_Delay(300);
}

extern "C" void app_main() {
    USB_Disconnected();
    USBD_Init(&hUSB, &FS_Desc, DEVICE_FS);
    USBD_RegisterClass(&hUSB, &USBD_COMPOSITE);
    hUSB.pClassData = &hcomposite;
    USBD_Start(&hUSB);
    while (true) {
        /*==========================从上位机接收帧============================*/
        USBD_GS_CAN_ReceiveFromHost(&hUSB);

        /*==========================向上位机发送帧============================*/
        hGS_CAN.USBD_GS_CAN_SendFrame(&hUSB);

        for (unsigned int i = 0; i < hGS_CAN.channel_count; i++) {
            hGS_CAN.channels[i].send_frame(hGS_CAN.queue_to_host);
            hGS_CAN.channels[i].CAN_HandleError(hGS_CAN.queue_to_host);
            hGS_CAN.channels[i].led.update();
        }
    }
}

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    if (hcan == &::hcan) {
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
            rx_frame.channel = can_channel_0.nr;
            rx_frame.flags = 0;

            can_channel_0.led.blink_request(signal_led::LED_RX);
            hGS_CAN.queue_to_host.push(rx_object);
        }
    }
}
