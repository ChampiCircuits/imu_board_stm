/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include <stdio.h>
#include "custom_motion_sensors.h"
#include "custom_motion_sensors_ex.h"

#include "MessageRecomposer.h"
#include "ChampiCan.h"

#include "ChampiState.h"

#include <pb_encode.h>
#include <pb_decode.h>
#include "msgs_can.pb.h"
#include "can_ids.hpp"
#include "vector"

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
CRC_HandleTypeDef hcrc;

FDCAN_HandleTypeDef hfdcan1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

ChampiCan champi_can;
ChampiState champi_state;


// On le déclare ici ce buffer, car il va servir tout le temps.
uint8_t buffer_encode_imu_data[60]; // TODO 60 c'est large

CUSTOM_MOTION_SENSOR_Axes_t axes_acc;
CUSTOM_MOTION_SENSOR_Axes_t axes_gyro;
CUSTOM_MOTION_SENSOR_Event_Status_t imu_status;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_CRC_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

void Error_Handler_CAN_ok();
void transmit_imu_data(CUSTOM_MOTION_SENSOR_Axes_t axes);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern "C" {
	int _write(int file, uint8_t  *ptr, int len) {
		for (int DataIdx = 0; DataIdx < len; DataIdx++)
	//        ITM_SendChar(*ptr++);
			HAL_UART_Transmit(&huart2, ptr++, 1, HAL_MAX_DELAY);
		return len;
	}
}

/**
 * @brief Function to send the current velocity of the robot on the CAN bus.
 * @param vel : the current velocity of the robot
 */
void transmit_imu_data(CUSTOM_MOTION_SENSOR_Axes_t data_acc, CUSTOM_MOTION_SENSOR_Axes_t data_gyro) {

    // Init message
    msgs_can_ImuData imu_proto = msgs_can_ImuData_init_zero;
    pb_ostream_t stream = pb_ostream_from_buffer(buffer_encode_imu_data, sizeof(buffer_encode_imu_data));

    // Fill message (convert mg to m/s^2 and mdps to rad/s)
    imu_proto.acc_x = (float)(data_acc.x * 9.8106 / 1000.);
    imu_proto.acc_y = (float)(data_acc.y * 9.8106 / 1000.);
    imu_proto.acc_z = (float)(data_acc.z * 9.8106 / 1000.);
    imu_proto.gyro_x = (float)(data_gyro.x * 0.01745329251 / 1000.);
    imu_proto.gyro_y = (float)(data_gyro.y * 0.01745329251 / 1000.);
    imu_proto.gyro_z = (float)(data_gyro.z * 0.01745329251 / 1000.);

    // Fill has.. fields
    imu_proto.has_acc_x = true;
    imu_proto.has_acc_y = true;
    imu_proto.has_acc_z = true;
    imu_proto.has_gyro_x = true;
    imu_proto.has_gyro_y = true;
    imu_proto.has_gyro_z = true;


    // Encode message
    bool status = pb_encode(&stream, msgs_can_ImuData_fields, &imu_proto);
    size_t message_length = stream.bytes_written;

    // Check for errors
    if (!status) {
        // TODO on peut récupérer un message d'erreur avec PB_GET_ERROR(&stream))
        champi_state.report_status(msgs_can_Status_StatusType_ERROR, msgs_can_Status_ErrorType_PROTO_ENCODE);
        Error_Handler_CAN_ok();
    }

    // Send
    if (champi_can.send_msg(CAN_ID_IMU_DATA, (uint8_t *) buffer_encode_imu_data, message_length) != 0) {
        /* Transmission request Error */
        champi_state.report_status(msgs_can_Status_StatusType_ERROR, msgs_can_Status_ErrorType_CAN_TX);
        Error_Handler_CAN_ok();
    }
}

/**
 * @brief Error handler we call when CAN might still work.
 * It blinks the built-in LED at 1Hz AND sends status on CAN bus.
 */
void Error_Handler_CAN_ok() {

    // Blink the built-in LED at 1Hz
    uint32_t last_time = HAL_GetTick();
    while (true) {
        champi_state.spin_once();
        HAL_Delay(10); // 10ms required to match the main loop frequency (for control)

        if (HAL_GetTick() - last_time > 500) {
            last_time = HAL_GetTick();
            HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        }
    }
}

/**
 * @brief Fonction qui attend que le l'envoi de données sur le CAN fonctionne. Ca envoie un message de test
 * à répétition jusqu'à ce que ça fonctionne.
 * Also blinks the built-in LED at 5 Hz.
 */
void tx_ok_or_reset() {
    uint8_t buff[20] = {0}; // We need a big message to fill the FIFO

    // Send a message to test if the can bus works (at least 1 node up)
    uint32_t ret = champi_can.send_msg(CAN_ID_IMU_TEST, (uint8_t *) buff, 20);

    if(ret==0){
        return;
    }

    // If we get an error, retry doesn't work sometimes. So we reset the stm to try again. Also blink the led 10Hz

    // blink the built-in LED for 1s
    for (int i = 0; i < 10; i++) {
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        HAL_Delay(100);
    }

    // Then reset the stm
    NVIC_SystemReset();

}



void setup() {

    champi_can = ChampiCan(&hfdcan1);


    if (champi_can.start() != 0) {
        // TODO: On a jamais rencontré cette erreur.
        Error_Handler();
    }

    // This is required: when the Raspberry Pi starts up, transmit CAN frames returns error.
    tx_ok_or_reset();

    // Initialize the IMU
    (void) CUSTOM_MOTION_SENSOR_Init(CUSTOM_LSM6DSO_0, MOTION_ACCELERO | MOTION_GYRO);
    (void) CUSTOM_MOTION_SENSOR_Enable_6D_Orientation(CUSTOM_LSM6DSO_0, CUSTOM_MOTION_SENSOR_INT1_PIN);

    champi_state = ChampiState(&champi_can, 500);

    champi_state.report_status(msgs_can_Status_StatusType_INIT, msgs_can_Status_ErrorType_NONE);

    // Switch led ON to indicate that we're running
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);

    champi_state.report_status(msgs_can_Status_StatusType_OK, msgs_can_Status_ErrorType_NONE);

}

/**
 * @brief Main loop.
 */
void loop() {
    if (CUSTOM_MOTION_SENSOR_Get_Event_Status(CUSTOM_LSM6DSO_0, &imu_status) != BSP_ERROR_NONE) {
        Error_Handler();
    }


    if (CUSTOM_MOTION_SENSOR_GetAxes(CUSTOM_LSM6DSO_0, MOTION_ACCELERO, &axes_acc) != BSP_ERROR_NONE) {
        Error_Handler();
    }

    if (CUSTOM_MOTION_SENSOR_GetAxes(CUSTOM_LSM6DSO_0, MOTION_GYRO, &axes_gyro) != BSP_ERROR_NONE) {
        Error_Handler();
    }

    transmit_imu_data(axes_acc, axes_gyro);

    champi_state.spin_once();
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
  MX_FDCAN1_Init();
  MX_CRC_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

    setup();
//setup_calibration();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  unsigned int last_time = HAL_GetTick();

    while (1) {
        if(HAL_GetTick() - last_time > 10) {
            last_time = HAL_GetTick();
             loop();
//            loop_calibration();
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 10;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 14;
  hfdcan1.Init.NominalTimeSeg2 = 2;
  hfdcan1.Init.DataPrescaler = 10;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 14;
  hfdcan1.Init.DataTimeSeg2 = 2;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

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
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
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
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : IMU_INT_Pin */
  GPIO_InitStruct.Pin = IMU_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IMU_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IMU_CS_Pin */
  GPIO_InitStruct.Pin = IMU_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(IMU_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_Pin */
  GPIO_InitStruct.Pin = BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
    while (1) {
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
