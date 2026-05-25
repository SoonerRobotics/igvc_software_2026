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

#define LOADING_LENGTH  30
#define LOADING_SPEED   15
#define LOADING_R       0
#define LOADING_G       150
#define LOADING_B       255
#define STRIP_A_DIR     1
#define STRIP_B_DIR    -1

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
uint8_t TxData1[8] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08 };
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

typedef enum {
	MODE_DEFAULT = 0,
	MODE_CHASING = 1,
	MODE_SOLID = 2,
	MODE_BLINKING = 3,
	MODE_RAINBOW = 4,
} SafetyLightsMode;

typedef struct {
	SafetyLightsMode mode;
	uint8_t r;
	uint8_t g;
	uint8_t b;
	uint16_t speed;
	uint16_t unused;
} SafetyLightsState;

static volatile SafetyLightsState g_lights = { .mode = MODE_DEFAULT, .r = 0,
		.g = 150, .b = 255, .speed = 1000, };

/* USER CODE BEGIN 0 */

static void wait_for_strip(WS28XX_HandleTypeDef *ws, uint32_t dma_id) {
	while (ws->hTim->hdma[dma_id]->State != HAL_DMA_STATE_READY)
		;
	while (ws->hTim->State != HAL_TIM_STATE_READY)
		;
	HAL_Delay(1);
}

static void update_both_strips(void) {
	// Wait for any previous DMA to finish, then fire both transfers
	wait_for_strip(&ws28xx_a, TIM_DMA_ID_CC3);
	wait_for_strip(&ws28xx_b, TIM_DMA_ID_CC4);

	WS28XX_Update(&ws28xx_a);
	WS28XX_Update(&ws28xx_b);

	// Wait for both to complete before next frame
	wait_for_strip(&ws28xx_a, TIM_DMA_ID_CC3);
	wait_for_strip(&ws28xx_b, TIM_DMA_ID_CC4);
}

static void set_all(uint8_t r, uint8_t g, uint8_t b) {
	for (uint16_t i = 0; i < NUM_LEDS; i++) {
		WS28XX_SetPixel_RGB(&ws28xx_a, i, r, g, b);
		WS28XX_SetPixel_RGB(&ws28xx_b, i, r, g, b);
	}
}

void mode_chasing(uint8_t r, uint8_t g, uint8_t b, uint16_t speed) {
	static uint16_t head = 0;

	set_all(0, 0, 0);

	for (int16_t t = CHASE_LENGTH - 1; t >= 0; t--) {
		int16_t pos = ((int16_t) head - t + NUM_LEDS) % NUM_LEDS;
		uint8_t bright = 255 - ((uint16_t) t * 255 / CHASE_LENGTH);

		// Scale the target color by brightness
		uint8_t pr = (uint8_t) ((uint32_t) r * bright / 255);
		uint8_t pg = (uint8_t) ((uint32_t) g * bright / 255);
		uint8_t pb = (uint8_t) ((uint32_t) b * bright / 255);

		WS28XX_SetPixel_RGB(&ws28xx_a, (uint16_t) pos, pr, pg, pb);
		WS28XX_SetPixel_RGB(&ws28xx_b, (uint16_t) pos, pr, pg, pb);
	}

	update_both_strips();
	head = (head + 1) % NUM_LEDS;
	HAL_Delay(speed / NUM_LEDS);
}

void mode_solid(uint8_t r, uint8_t g, uint8_t b) {
	static uint8_t last_r = 0xFF, last_g = 0xFF, last_b = 0xFF;

	// Only re-push if color changed — avoids hammering DMA every loop iteration
	if (r == last_r && g == last_g && b == last_b)
		return;

	last_r = r;
	last_g = g;
	last_b = b;
	set_all(r, g, b);
	update_both_strips();
}

void mode_blinking(uint8_t r, uint8_t g, uint8_t b, uint16_t speed) {
	static uint8_t flash_on = 1;
	static uint32_t last_toggle = 0;

	uint32_t now = HAL_GetTick();
	uint32_t interval = speed / 2;   // speed = full cycle; half on, half off

	if ((now - last_toggle) >= interval) {
		flash_on = !flash_on;
		last_toggle = now;

		if (flash_on)
			set_all(r, g, b);
		else
			set_all(0, 0, 0);

		update_both_strips();
	}
}

void mode_rainbow(uint16_t speed) {
	static uint16_t hue_offset = 0;

	for (uint16_t i = 0; i < NUM_LEDS; i++) {
		uint16_t hue = (hue_offset + (i * 360 / NUM_LEDS)) % 360;

		// Simple HSV→RGB with S=1, V=1
		uint8_t region = hue / 60;
		uint8_t rem = (uint8_t) ((hue % 60) * 255 / 60);
		uint8_t pr, pg, pb;

		switch (region) {
		case 0:
			pr = 255;
			pg = rem;
			pb = 0;
			break;
		case 1:
			pr = 255 - rem;
			pg = 255;
			pb = 0;
			break;
		case 2:
			pr = 0;
			pg = 255;
			pb = rem;
			break;
		case 3:
			pr = 0;
			pg = 255 - rem;
			pb = 255;
			break;
		case 4:
			pr = rem;
			pg = 0;
			pb = 255;
			break;
		default:
			pr = 255;
			pg = 0;
			pb = 255 - rem;
			break;
		}

		WS28XX_SetPixel_RGB(&ws28xx_a, i, pr, pg, pb);
		WS28XX_SetPixel_RGB(&ws28xx_b, i, pr, pg, pb);
	}

	update_both_strips();
	hue_offset = (hue_offset + 2) % 360;
	HAL_Delay(speed / 180);   // ~180 steps for a full cycle
}

void mode_estopped(void) {
	for (int16_t t = 0; t < NUM_LEDS; t++) {
		WS28XX_SetPixel_RGB(&ws28xx_a, t, 255, 255, 255);
		WS28XX_SetPixel_RGB(&ws28xx_b, t, 255, 255, 255);
	}

	update_both_strips();
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

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
	TxHeader1.FDFormat = FDCAN_CLASSIC_CAN; /* Must match FrameFormat in Init */
	TxHeader1.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TxHeader1.MessageMarker = 0;

	/* Start FDCAN peripheral - REQUIRED before any Tx/Rx */
	if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
		Error_Handler();
	}
	FDCAN_FilterTypeDef FilterConfig;
	FilterConfig.IdType = FDCAN_STANDARD_ID;
	FilterConfig.FilterIndex = 0;
	FilterConfig.FilterType = FDCAN_FILTER_MASK;
	FilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	FilterConfig.FilterID1 = 0x014;
	FilterConfig.FilterID2 = 0x7FF;
	if (HAL_FDCAN_ConfigFilter(&hfdcan1, &FilterConfig) != HAL_OK) {
		Error_Handler();
	}

	// Reject anything that doesn't match a filter
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
	FDCAN_REJECT, FDCAN_REJECT,
	FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
z		if (!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12)) {
			mode_estopped();
			continue;
		}

		SafetyLightsMode mode = g_lights.mode;
		uint8_t r = g_lights.r;
		uint8_t g = g_lights.g;
		uint8_t b = g_lights.b;
		uint16_t speed = g_lights.speed;

		switch (mode) {
		case MODE_CHASING:
			mode_chasing(r, g, b, speed);
			break;
		case MODE_SOLID:
			mode_solid(r, g, b);
			break;
		case MODE_BLINKING:
			mode_blinking(r, g, b, speed);
			break;
		case MODE_RAINBOW:
			mode_rainbow(100);
			break;
		case MODE_DEFAULT:
		default:
			mode_rainbow(100);
			break;
		}
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
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	__HAL_FLASH_SET_LATENCY(FLASH_LATENCY_1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
	FDCAN_RxHeaderTypeDef RxHeader;
	uint8_t RxData[8];

	if (!(RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE)) {
		return;
	}

	if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData)
			!= HAL_OK) {
		return;
	}

	if (RxHeader.Identifier != 0x14) {
		return;
	}

	// Unpack SafetyLightsPacket
	g_lights.mode = (SafetyLightsMode) RxData[0];
	g_lights.r = RxData[1];
	g_lights.g = RxData[2];
	g_lights.b = RxData[3];
	g_lights.speed = (uint16_t) (RxData[4] | ((uint16_t) RxData[5] << 8));
	g_lights.speed = (uint16_t) (RxData[6] | ((uint16_t) RxData[7] << 8));
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
