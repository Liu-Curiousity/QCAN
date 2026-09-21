/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "usb.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_composite.h"
#include "can_common.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_gs_can.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static USBD_COMPOSITE_HandleTypeDef hGS_CAN;
static USBD_HandleTypeDef hUSB;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  MX_USB_PCD_Init();
  MX_USART4_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  INIT_LIST_HEAD(&hGS_CAN.list_frame_pool);
  INIT_LIST_HEAD(&hGS_CAN.list_to_host);

  for (unsigned i = 0; i < ARRAY_SIZE(hGS_CAN.msgbuf); i++) {
    list_add_tail(&hGS_CAN.msgbuf[i].list, &hGS_CAN.list_frame_pool);
  }

  for (unsigned int i = 0; i < ARRAY_SIZE(hGS_CAN.channels); i++) {
    can_data_t *channel = &hGS_CAN.channels[i];

    can_channel_set_nr(channel, i);

    INIT_LIST_HEAD(&channel->list_from_host);

    led_init(&channel->leds,
         LEDRX_GPIO_Port, LEDRX_Pin, LEDRX_Active_High,
         LEDTX_GPIO_Port, LEDTX_Pin, LEDTX_Active_High);

    can_init(channel);
    can_disable(&hGS_CAN, channel);
  }

  USBD_Init(&hUSB, (USBD_DescriptorsTypeDef *)&FS_Desc, DEVICE_FS);
  USBD_RegisterClass(&hUSB, &USBD_COMPOSITE);
  USBD_GS_CAN_Init_(&hGS_CAN, &hUSB);
  USBD_Start(&hUSB);

  /* nice wake-up pattern */
  for (uint8_t j = 0; j < 10; j++) {
    HAL_GPIO_TogglePin(LEDRX_GPIO_Port, LEDRX_Pin);
    HAL_Delay(50);
    HAL_GPIO_TogglePin(LEDTX_GPIO_Port, LEDTX_Pin);
  }
  while (1)
  {
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
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
