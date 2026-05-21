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
#include "dma.h"
#include "fdcan.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ws28xx.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define NUM_LEDS      144

#define CHASE_LENGTH  40
#define CHASE_SPEED   20
#define CHASE_R       0
#define CHASE_G       150
#define CHASE_B       255

#define FLASH_ON_MS   1000
#define FLASH_OFF_MS  1000

int borg = 0;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
WS28XX_HandleTypeDef ws28xx_a;
WS28XX_HandleTypeDef ws28xx_b;

FDCAN_TxHeaderTypeDef TxHeader1;
uint8_t TxData1[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE BEGIN 0 */

static void wait_for_strip(WS28XX_HandleTypeDef *ws, uint32_t dma_id) {
    while (ws->hTim->hdma[dma_id]->State != HAL_DMA_STATE_READY);
    while (ws->hTim->State != HAL_TIM_STATE_READY);
    HAL_Delay(1);
}

void chase_mode(void) {
    static uint16_t head_a = 0;
    static uint16_t head_b = 0;

    // Clear all LEDs
    for (uint16_t i = 0; i < NUM_LEDS; i++) {
        WS28XX_SetPixel_RGB(&ws28xx_a, i, 0, 0, 0);
        WS28XX_SetPixel_RGB(&ws28xx_b, i, 0, 0, 0);
    }

    // Draw chase trail for strip A (blue)
    for (int16_t t = CHASE_LENGTH - 1; t >= 0; t--) {
        int16_t pos = (int16_t)head_a - t;
        if (pos < 0) pos += NUM_LEDS;

        uint8_t brightness = 255 - ((uint16_t)t * 255 / CHASE_LENGTH);
        WS28XX_SetPixel_RGB(&ws28xx_a, (uint16_t)pos,
            borg == 0 ? 0 : 255,
            (uint16_t)150 * brightness / 255,
            (uint16_t)255 * brightness / 255);
    }

    // Draw chase trail for strip B (red/orange)
    for (int16_t t = CHASE_LENGTH - 1; t >= 0; t--) {
        int16_t pos = (int16_t)head_b - t;
        if (pos < 0) pos += NUM_LEDS;

        uint8_t brightness = 255 - ((uint16_t)t * 255 / CHASE_LENGTH);
        WS28XX_SetPixel_RGB(&ws28xx_b, (uint16_t)pos,
            (uint16_t)255 * brightness / 255,
            (uint16_t)80  * brightness / 255,
            0);
    }

    // Update strip A
    wait_for_strip(&ws28xx_a, TIM_DMA_ID_CC3);
    WS28XX_Update(&ws28xx_a);
    wait_for_strip(&ws28xx_a, TIM_DMA_ID_CC3);

    // Update strip B
    wait_for_strip(&ws28xx_b, TIM_DMA_ID_CC4);
    WS28XX_Update(&ws28xx_b);
    wait_for_strip(&ws28xx_b, TIM_DMA_ID_CC4);

    head_a = (head_a + 1) % NUM_LEDS;
    head_b = (head_b + 1) % NUM_LEDS;
    HAL_Delay(CHASE_SPEED);
}

void mode_estopped(void) {
    for (int16_t t = 0; t < NUM_LEDS; t++) {
        WS28XX_SetPixel_RGB(&ws28xx_a, t, 255, 255, 255);
        WS28XX_SetPixel_RGB(&ws28xx_b, t, 255, 255, 255);
    }

    wait_for_strip(&ws28xx_a, TIM_DMA_ID_CC3);
    WS28XX_Update(&ws28xx_a);
    wait_for_strip(&ws28xx_a, TIM_DMA_ID_CC3);

    wait_for_strip(&ws28xx_b, TIM_DMA_ID_CC4);
    WS28XX_Update(&ws28xx_b);
    wait_for_strip(&ws28xx_b, TIM_DMA_ID_CC4);
}

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
  MX_DMA_Init();
  MX_FDCAN1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  WS28XX_Init(&ws28xx_a, &htim1, 48, TIM_CHANNEL_3, 144);
  WS28XX_Init(&ws28xx_b, &htim1, 48, TIM_CHANNEL_4, 144);

  HAL_Delay(10);
  __HAL_TIM_MOE_ENABLE(&htim1);

  /* Configure Tx header - Classic CAN frame, 8 bytes, ID 0x11 */
  TxHeader1.Identifier = 0x11;
  TxHeader1.IdType = FDCAN_STANDARD_ID;
  TxHeader1.TxFrameType = FDCAN_DATA_FRAME;
  TxHeader1.DataLength = FDCAN_DLC_BYTES_8;
  TxHeader1.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader1.BitRateSwitch = FDCAN_BRS_OFF;
  TxHeader1.FDFormat = FDCAN_CLASSIC_CAN;        /* Must match FrameFormat in Init */
  TxHeader1.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader1.MessageMarker = 0;

  /* Start FDCAN peripheral - REQUIRED before any Tx/Rx */
  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  FDCAN_FilterTypeDef FilterConfig;
  FilterConfig.IdType = FDCAN_STANDARD_ID;
  FilterConfig.FilterIndex = 0;
  FilterConfig.FilterType = FDCAN_FILTER_MASK;
  FilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  FilterConfig.FilterID1 = 0x014;
  FilterConfig.FilterID2 = 0x7FF;
  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &FilterConfig) != HAL_OK)
  {
      Error_Handler();
  }

  // Reject anything that doesn't match a filter
  HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
     FDCAN_REJECT, FDCAN_REJECT,
	 FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE
  );
  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12))
	  {
		  mode_estopped();
		  continue;
	  }

//	  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader1, TxData1) != HAL_OK)
//	  {
//	      Error_Handler();
//	  }

//	  HAL_Delay(250);
	  chase_mode();
  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* USER CODE END WHILE */
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

  __HAL_FLASH_SET_LATENCY(FLASH_LATENCY_1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    FDCAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];

    if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE)
    {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
        {
            if (RxHeader.Identifier == 0x14)
            {
            	borg = borg == 0 ? 1 : 0;
            }
        }
    }
}
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
#ifdef USE_FULL_ASSERT
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
