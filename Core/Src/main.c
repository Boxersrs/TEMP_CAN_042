/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <ds18b20.h>
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
CAN_HandleTypeDef hcan;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t Dev_ID[8][8] = {0};
uint8_t Dev_Cnt;
char str1[60];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
CAN_RxHeaderTypeDef rxHeader;  // CAN Bus Transmit Header
CAN_TxHeaderTypeDef txHeader;  // CAN Bus Receive Header
uint8_t canRx[8];              // CAN Bus Receive Buffer
CAN_FilterTypeDef canfil;      // CAN Bus Filter
uint32_t canMailbox;           // CAN Bus Mail box variable
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
  /* USER CODE BEGIN 1 */
  uint8_t status;
  uint8_t dt[8];
  uint16_t raw_temper;
  float temper;
  char c;
  uint8_t i;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  canfil.FilterBank = 0;
  canfil.FilterMode = CAN_FILTERMODE_IDMASK;
  canfil.FilterFIFOAssignment = CAN_RX_FIFO0;
  canfil.FilterIdHigh = 0x0000;
  canfil.FilterIdLow = 0x0000;
  canfil.FilterMaskIdHigh = 0;
  canfil.FilterMaskIdLow = 0;
  canfil.FilterScale = CAN_FILTERSCALE_32BIT;
  canfil.FilterActivation = ENABLE;
  canfil.SlaveStartFilterBank = 14;

  HAL_CAN_ConfigFilter(&hcan, &canfil);
  HAL_CAN_Start(&hcan);
  HAL_CAN_ActivateNotification(
      &hcan, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY);

  port_init();
  status = ds18b20_init(NO_SKIP_ROM);
  sprintf(str1, "Init Status: %d\r\n", status);
  HAL_UART_Transmit(&huart1, (uint8_t *)str1, strlen(str1), 0x1000);
  sprintf(str1, "Dev count: %d\r\n", Dev_Cnt);
  HAL_UART_Transmit(&huart1, (uint8_t *)str1, strlen(str1), 0x1000);
  for (i = 1; i <= Dev_Cnt; i++) {
    sprintf(str1, "Device %d\r\n", i);
    HAL_UART_Transmit(&huart1, (uint8_t *)str1, strlen(str1), 0x1000);
    sprintf(str1, "ROM RAW: %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
            Dev_ID[i - 1][0], Dev_ID[i - 1][1], Dev_ID[i - 1][2],
            Dev_ID[i - 1][3], Dev_ID[i - 1][4], Dev_ID[i - 1][5],
            Dev_ID[i - 1][6], Dev_ID[i - 1][7]);
    HAL_UART_Transmit(&huart1, (uint8_t *)str1, strlen(str1), 0x1000);
    sprintf(str1, "Family CODE: 0x%02X\r\n", Dev_ID[i - 1][0]);
    HAL_UART_Transmit(&huart1, (uint8_t *)str1, strlen(str1), 0x1000);
    sprintf(str1, "ROM CODE: 0x%02X%02X%02X%02X%02X%02X\r\n", Dev_ID[i - 1][6],
            Dev_ID[i - 1][5], Dev_ID[i - 1][4], Dev_ID[i - 1][3],
            Dev_ID[i - 1][2], Dev_ID[i - 1][1]);
    HAL_UART_Transmit(&huart1, (uint8_t *)str1, strlen(str1), 0x1000);
    sprintf(str1, "CRC: 0x%02X\r\n", Dev_ID[i - 1][7]);
    HAL_UART_Transmit(&huart1, (uint8_t *)str1, strlen(str1), 0x1000);
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    for (i = 1; i <= Dev_Cnt; i++) {
      ds18b20_MeasureTemperCmd(NO_SKIP_ROM, i);
    }
    HAL_Delay(800);
    for (i = 1; i <= Dev_Cnt; i++) {
      ds18b20_ReadStratcpad(NO_SKIP_ROM, dt, i);
      sprintf(str1, "STRATHPAD %d: %02X %02X %02X %02X %02X %02X %02X %02X; ",
              i, dt[0], dt[1], dt[2], dt[3], dt[4], dt[5], dt[6], dt[7]);
      HAL_UART_Transmit(&huart1, (uint8_t *)str1, strlen(str1), 0x1000);
      raw_temper = ((uint16_t)dt[1] << 8) | dt[0];
      if (ds18b20_GetSign(raw_temper))
        c = '-';
      else
        c = '+';
      temper = ds18b20_Convert(raw_temper);
      sprintf(str1, "Raw t: 0x%04X; t: %c%.2f\r\n", raw_temper, c, temper);
      HAL_UART_Transmit(&huart1, (uint8_t *)str1, strlen(str1), 0x1000);
    }
    HAL_Delay(150);
  }
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
  txHeader.DLC = 2;
  txHeader.IDE = CAN_ID_STD;
  txHeader.RTR = CAN_RTR_DATA;
  uint8_t can_send[2] = {1, 0};
  txHeader.StdId = 0x301;
  txHeader.TransmitGlobalTime = DISABLE;
  HAL_CAN_AddTxMessage(&hcan, &txHeader, can_send, &canMailbox);
}
/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType =
      RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief CAN Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN_Init(void) {
  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 16;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan.Init.TimeTriggeredMode = ENABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = ENABLE;
  hcan.Init.AutoRetransmission = ENABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = ENABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {
  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_HalfDuplex_Init(&huart2) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
}

/* USER CODE BEGIN 4 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  // получаем сообщение //
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, canRx) != HAL_OK) {
    // Reception Error //
    Error_Handler();
  }
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
