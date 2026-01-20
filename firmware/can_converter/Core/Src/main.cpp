#include "main.h"
#include "Hardware/CanSparkMax.h"
#include "SwerveDrive/SwerveModule.h"
#include "SwerveDrive/SwerveDrive.h"
#include <cmath>
#include "usb_device.h"
#include "usbd_cdc_if.h"
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;
void SystemClock_Config(void);
void serial_debug(SwerveModule& module);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void configFilter(CAN_HandleTypeDef *hcan, uint32_t id, uint32_t bank);
static void SendOdometry(const SwerveDriveState& odom);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
CanSparkMax* s_spark_registry[64] = { nullptr };
SwerveDrive* InitSwerveDrive();
SwerveDrive* g_swerve_drive = nullptr;
SwerveDriveState cmd{};

SwerveDrive* InitSwerveDrive() {

	static SwerveModuleConfig fl_config = {
	        .x_pos =  0.3048,
	        .y_pos =  0.3048,
	        .drive_motor_id = 1,
	        .angle_motor_id = 2,
	        .is_drive_motor_reversed = false,
	        .is_angle_motor_reversed = false
	    };

	    static SwerveModuleConfig fr_config = {
	        .x_pos =  0.3048,
	        .y_pos = -0.3048,
	        .drive_motor_id = 4,
	        .angle_motor_id = 3,
	        .is_drive_motor_reversed = false,
	        .is_angle_motor_reversed = false
	    };

	    static SwerveModuleConfig bl_config = {
	        .x_pos = -0.3048,
	        .y_pos =  0.3048,
	        .drive_motor_id = 5,
	        .angle_motor_id = 6,
	        .is_drive_motor_reversed = false,
	        .is_angle_motor_reversed = false
	    };

	    static SwerveModuleConfig br_config = {
	        .x_pos = -0.3048,
	        .y_pos = -0.3048,
	        .drive_motor_id = 8,
	        .angle_motor_id = 7,
	        .is_drive_motor_reversed = false,
	        .is_angle_motor_reversed = false
	    };
	    static SwerveModule fl_module(fl_config);
	    static SwerveModule fr_module(fr_config);
	    static SwerveModule bl_module(bl_config);
	    static SwerveModule br_module(br_config);
	    static SwerveDriveConfig drive_config = {
	    	.front_left = &fl_module,
			.front_right = &fr_module,
			.back_left = &bl_module,
			.back_right = &br_module
	    };
	    static SwerveDrive drive(drive_config);
	    g_swerve_drive = &drive;
	    return &drive;
}

// motor command stuff
typedef struct {
    int16_t forward_velocity;
    int16_t sideways_velocity;
    int16_t angular_velocity;
} MotorCommandMsg;

// motor odometry stuff
typedef struct {
    int16_t delta_x;
    int16_t delta_y;
    int16_t delta_theta;
} MotorOdometryMsg;

void encode_motor_odom_le(const MotorOdometryMsg *msg, uint8_t *out)
{
    out[0] = msg->delta_x & 0xFF;
    out[1] = (msg->delta_x >> 8) & 0xFF;

    out[2] = msg->delta_y & 0xFF;
    out[3] = (msg->delta_y >> 8) & 0xFF;

    out[4] = msg->delta_theta & 0xFF;
    out[5] = (msg->delta_theta >> 8) & 0xFF;
}

int main(void)
{


  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_USB_DEVICE_Init();
  if (HAL_CAN_Start(&hcan1) != HAL_OK) {
        Error_Handler();  // CAN failed to start
      }
  if (HAL_CAN_Start(&hcan2) != HAL_OK) {
          Error_Handler();  // CAN failed to start
  	  }
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);



  SwerveDrive* swerveDrive = InitSwerveDrive();
  uint32_t last = HAL_GetTick();
  const uint32_t period_ms = 50;

  while (1) {
	  uint32_t now = HAL_GetTick();
	  if ((now - last) >= period_ms) {
		  SwerveDriveState measured = swerveDrive->updateState(cmd);
		  SendOdometry(measured);
		  CanSparkMax* hb_spark = s_spark_registry[1];  // e.g. drive motor ID 1
		  if (hb_spark) {
			  hb_spark->sendHeartbeat();
		  }
		  double a = swerveDrive->front_left_module_.angle_motor_.getAbsolutePosition();
		  double b = swerveDrive->front_left_module_.drive_motor_.getRPM();

		    char msg[96];
		    int len = snprintf(
		        msg,
		        sizeof(msg),
				"RPM=%.3f Abs = %.3f\r\n",
		        b,a
		    );
		    //CDC_Transmit_FS((uint8_t*)msg, len);
		  //swerveDrive->front_left_module_.drive_motor_.getAbsolutePosition;
		  last = now;
	  }



  }
}
static void SendOdometry(const SwerveDriveState& odom)
{
    uint8_t txData[8] = {0};
	MotorOdometryMsg msg = {
		.delta_x = (int16_t)(odom.x_vel / 0.0001f),
		.delta_y = (int16_t)(odom.y_vel / 0.0001f),
		.delta_theta = (int16_t)(odom.angular_vel / 0.0001f)
	};
	encode_motor_odom_le(&msg, txData);

    CAN_TxHeaderTypeDef txHeader;
    txHeader.StdId = 0xB;                 // Odometry Feedback ID (new 2026 id)
    txHeader.IDE   = CAN_ID_STD;
    txHeader.RTR   = CAN_RTR_DATA;
    txHeader.DLC   = 6;
    txHeader.TransmitGlobalTime = DISABLE;

    uint32_t mailbox;
    HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, &mailbox);
}

void serial_debug(SwerveModule& module)
{
    char msg[64];
    float delta = module.getDriveDelta();

    int len = snprintf(msg, sizeof(msg), "Delta: %.3f\r\n", delta);
    CDC_Transmit_FS((uint8_t*)msg, len);
}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if (hcan ->Instance == CAN1) {
		CAN_RxHeaderTypeDef rxHeader;
		uint8_t data[8];
		if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, data) != HAL_OK) {
			return;
		}
		uint32_t can_id = rxHeader.StdId;
		if ((can_id == 0xA) && (rxHeader.DLC == 6)) {
			MotorCommandMsg *msg = (MotorCommandMsg *)data;
	        cmd.x_vel       = msg->forward_velocity * 0.0001;
	        cmd.y_vel       = msg->sideways_velocity * 0.0001;
	        cmd.angular_vel = msg->angular_velocity * 0.001;
	        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
		}


	}
	else if (hcan ->Instance ==CAN2) {
		CAN_RxHeaderTypeDef rxHeader;
		uint8_t data[8];

		if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, data) != HAL_OK) {
			return;
		}
		uint32_t can_id = rxHeader.ExtId;
		uint8_t deviceID  =  can_id        & 0x3F;       // lower 6 bits
		uint8_t api_index = (can_id >> 6)  & 0x0F;       // next 4 bits
		uint8_t api_class = (can_id >> 10) & 0x3F;       // next 6 bits
//		char msg[64];
//		int len = snprintf(
//		    msg,
//		    sizeof(msg),
//		    "ID:%u cls:%u idx:%u\r\n",
//		    deviceID,
//		    api_class,
//		    api_index
//		);
//		CDC_Transmit_FS((uint8_t*)msg, len);
		// Look up the instance
		extern CanSparkMax* s_spark_registry[64];
		CanSparkMax* spark = s_spark_registry[deviceID];
		if (spark) {

			spark->handleFeedback(api_class, api_index, data);
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);
		}
	}
}


static void MX_CAN1_Init(void)
{
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 20;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_15TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = ENABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }


  CAN_FilterTypeDef filter = {0};

  filter.FilterActivation = CAN_FILTER_ENABLE;
  filter.FilterBank = 1;
  filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;

  // Standard 11-bit ID 10 → 0x00A
  // Hardware shifts StdId left by 5 bits for the register
  filter.FilterIdHigh     = (10 << 5);
  filter.FilterIdLow      = 0x0000;

  // Mask 0x7FF makes all 11 bits matter → exact match
  filter.FilterMaskIdHigh = (0x7FF << 5);
  filter.FilterMaskIdLow  = 0x0000;
  filter.SlaveStartFilterBank = 14;
  HAL_CAN_ConfigFilter(&hcan1, &filter);

}

static void MX_CAN2_Init(void)
{
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 2;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_15TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  configFilter(&hcan2,0x02051840,16);
  configFilter(&hcan2,0x02051880,17);
  configFilter(&hcan2,0x02051940,18); //RPM, and encoder feedback ids
}
static void configFilter(CAN_HandleTypeDef *hcan, uint32_t id, uint32_t bank)
{
	const uint32_t m_shift  = (0xFFFFFFC0u << 3);
    CAN_FilterTypeDef f ;
    f.FilterBank = bank;
    f.FilterActivation = CAN_FILTER_ENABLE;
    f.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    f.FilterScale = CAN_FILTERSCALE_32BIT;
    f.FilterMode  = CAN_FILTERMODE_IDMASK;
    f.FilterMaskIdHigh = (uint16_t)(m_shift >> 16);
    f.FilterMaskIdLow  = (uint16_t)(m_shift & 0xFFFF);
    f.FilterMaskIdLow |= (1u << 2) | (1u << 1);
    f.FilterIdHigh = (uint16_t)((id<<3) >> 16);
    f.FilterIdLow  = (uint16_t)((id<<3) & 0xFFFF);
    f.FilterIdLow |= (1u << 2) | (0u << 1);
    HAL_CAN_ConfigFilter(&hcan2, &f);
}


static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  //HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);
 // HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.Prediv1Source = RCC_PREDIV1_SOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL2.PLL2State = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV3;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the Systick interrupt time
  */
  __HAL_RCC_PLLI2S_ENABLE();
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
