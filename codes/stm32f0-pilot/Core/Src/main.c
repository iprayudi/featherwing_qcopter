/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BNO055_DEV_ADDR (0x29<<1) //(0x28<<1)
#define BNO055_REG_CHIP_ID 0x00
#define BNO055_REG_CHIP_ID_VALUE 0xA0
#define BNO055_REG_SW_REV_ID_LSB 0x04
#define BNO055_REG_SW_REV_ID_MSB 0x05
#define BNO055_REG_BL_REV_ID 0x06
#define BNO055_REG_SYS_TRIGGER 0x3F
#define BNO055_REG_PWR_MODE 0x3E
#define BNO055_REG_OPR_MODE 0x3D
#define BNO055_REG_UNIT_SEL 0x3B
#define BNO055_REG_SYS_ERR 0x3A
#define BNO055_REG_SYS_STATUS 0x39
#define BNO055_REG_EUL_PITCH_MSB 0x1F
#define BNO055_REG_EUL_PITCH_LSB 0x1E
#define BNO055_REG_EUL_ROLL_MSB 0x1D
#define BNO055_REG_EUL_ROLL_LSB 0x1C
#define BNO055_REG_EUL_YAW_MSB 0x1B
#define BNO055_REG_EUL_YAW_LSB 0x1A
#define BNO055_REG_GYR_DATA_Z_MSB 0x19
#define BNO055_REG_GYR_DATA_Z_LSB 0x18
#define BNO055_REG_GYR_DATA_Y_MSB 0x17
#define BNO055_REG_GYR_DATA_Y_LSB 0x16
#define BNO055_REG_GYR_DATA_X_MSB 0x15
#define BNO055_REG_GYR_DATA_X_LSB 0x14
#define BNO055_OPR_MODE_IMU 0x08

#define MPL3115A2_DEV_ADDR (0x60 << 1)
#define MPL3115A2_REG_WHO_AM_I 0x0C
#define MPL3115A2_REG_WHO_AM_VALUE 0xC4
#define MPL3115A2_REG_CTRL_REG1 0x26
#define MPL3115A2_REG_CTRL_REG2 0x27
#define MPL3115A2_REG_CTRL_REG3 0x28
#define MPL3115A2_REG_CTRL_REG4 0x29
#define MPL3115A2_REG_CTRL_REG5 0x2A
#define MPL3115A2_REG_STATUS 0x00
#define MPL3115A2_REG_OUT_P_MSB 0x01
#define MPL3115A2_REG_OUT_P_CSB 0x02
#define MPL3115A2_REG_OUT_P_LSB 0x03
#define MPL3115A2_REG_OUT_T_MSB 0x04
#define MPL3115A2_REG_OUT_T_LSB 0x05
#define MPL3115A2_REG_OUT_P_DELTA_MSB 0x07
#define MPL3115A2_REG_OUT_P_DELTA_CSB 0x08
#define MPL3115A2_REG_OUT_P_DELTA_LSB 0x09
#define MPL3115A2_REG_OUT_T_DELTA_MSB 0x0A
#define MPL3115A2_REG_OUT_T_DELTA_LSB 0x0B
#define MPL3115A2_REG_SYSMOD 0x11
#define MPL3115A2_REG_INT_SOURCE 0x12
#define MPL3115A2_REG_PT_DATA_CFG 0x13

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim14;

/* USER CODE BEGIN PV */
uint8_t i2cRxBuffer[12];
uint8_t i2cTxBuffer;
uint16_t euler[3];
uint16_t gyro[3];

int32_t pData;
float pData_f;
int16_t tData;
float tData_f;
char uartTxBuff[64];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM14_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void IMU_Init(void)
{
	// read CHIP_ID register
	if (HAL_I2C_Mem_Read(&hi2c1, BNO055_DEV_ADDR, BNO055_REG_CHIP_ID, 1, i2cRxBuffer, 1, 100)==HAL_OK){
	  if (i2cRxBuffer[0] == BNO055_REG_CHIP_ID_VALUE) {
	  }
	} else {
	}

	// read Bootloader version
	HAL_I2C_Mem_Read(&hi2c1, BNO055_DEV_ADDR, BNO055_REG_BL_REV_ID, 1, &i2cRxBuffer[1], 1, 100);

	// read Software version
	HAL_I2C_Mem_Read(&hi2c1, BNO055_DEV_ADDR, BNO055_REG_SW_REV_ID_LSB, 1, &i2cRxBuffer[2], 1, 100);
	HAL_I2C_Mem_Read(&hi2c1, BNO055_DEV_ADDR, BNO055_REG_SW_REV_ID_MSB, 1, &i2cRxBuffer[3], 1, 100);
	HAL_I2C_Mem_Read(&hi2c1, BNO055_DEV_ADDR, BNO055_REG_SW_REV_ID_LSB, 1, &i2cRxBuffer[4], 2, 100);

	// read System Status
	HAL_I2C_Mem_Read(&hi2c1, BNO055_DEV_ADDR, BNO055_REG_SYS_STATUS, 1, &i2cRxBuffer[1], 1, 10);
	HAL_I2C_Mem_Read(&hi2c1, BNO055_DEV_ADDR, BNO055_REG_SYS_ERR, 1, &i2cRxBuffer[1], 1, 10);

	// set operating mode to IMU mode
	i2cTxBuffer = BNO055_OPR_MODE_IMU;
	HAL_I2C_Mem_Write(&hi2c1, BNO055_DEV_ADDR, BNO055_REG_OPR_MODE, 1, &i2cTxBuffer, 1, 10);
	HAL_I2C_Mem_Read(&hi2c1, BNO055_DEV_ADDR, BNO055_REG_SYS_STATUS, 1, &i2cRxBuffer[1], 1, 10);
}
void readIMU(void)
{
	HAL_I2C_Mem_Read(&hi2c1, BNO055_DEV_ADDR, BNO055_REG_EUL_PITCH_MSB, 1, i2cRxBuffer, 12, 10);
	euler[0] = i2cRxBuffer[0]<<8 | i2cRxBuffer[1];
	euler[1] = i2cRxBuffer[2]<<8 | i2cRxBuffer[3];
	euler[2] = i2cRxBuffer[4]<<8 | i2cRxBuffer[5];
	gyro[0]  = i2cRxBuffer[6]<<8 | i2cRxBuffer[7];
	gyro[1]  = i2cRxBuffer[8]<<8 | i2cRxBuffer[9];
	gyro[2]  = i2cRxBuffer[10]<<8 | i2cRxBuffer[11];
}

void Alti_Init(void)
{
	/* MPL3115A2 INITIALIZATION */

	// check MPL3115A2 sensor by reading WHO_AM_I register
	if (HAL_I2C_Mem_Read(&hi2c1, MPL3115A2_DEV_ADDR, MPL3115A2_REG_WHO_AM_I, 1, i2cRxBuffer, 1, 100) == HAL_OK){
	  if (i2cRxBuffer[0] == MPL3115A2_REG_WHO_AM_VALUE) {
		  //read system mode
		  HAL_I2C_Mem_Read(&hi2c1, MPL3115A2_DEV_ADDR, MPL3115A2_REG_SYSMOD, 1, &i2cRxBuffer[1], 1, 100);
	  } else {
	  }
	}

	i2cTxBuffer=0x07;
	HAL_I2C_Mem_Write(&hi2c1, MPL3115A2_DEV_ADDR, MPL3115A2_REG_PT_DATA_CFG, 1, &i2cTxBuffer, 1, 100);

	i2cTxBuffer=0x80; // Data Ready interrupt enabled
	HAL_I2C_Mem_Write(&hi2c1, MPL3115A2_DEV_ADDR, MPL3115A2_REG_CTRL_REG4, 1, &i2cTxBuffer, 1, 100);

	i2cTxBuffer=0x80; // Interrupt is routed to INT1
	HAL_I2C_Mem_Write(&hi2c1, MPL3115A2_DEV_ADDR, MPL3115A2_REG_CTRL_REG5, 1, &i2cTxBuffer, 1, 100);

	// put sensor to ACTIVE mode
	//  i2cTxBuff=0xB9; // altitude mode
	i2cTxBuffer=0x39; // barometer mode
	HAL_I2C_Mem_Write(&hi2c1, MPL3115A2_DEV_ADDR, MPL3115A2_REG_CTRL_REG1, 1, &i2cTxBuffer, 1, 100);

	//read system mode
	HAL_I2C_Mem_Read(&hi2c1, MPL3115A2_DEV_ADDR, MPL3115A2_REG_SYSMOD, 1, &i2cRxBuffer[1], 1, 100);
}

void readAlti(void)
{

	// read all data at once
	if (HAL_I2C_Mem_Read(&hi2c1, MPL3115A2_DEV_ADDR, MPL3115A2_REG_OUT_P_MSB, 1, i2cRxBuffer, 5, 100) == HAL_OK)
	{

		/*
		 * altitude
		 */

		pData = i2cRxBuffer[0]<<24 | i2cRxBuffer[1]<<16 | i2cRxBuffer[2]<<8;
		pData_f = pData / 65536;

		/*
		 * pressure
		 */

//		pData = i2cRxBuff[0]<<16 | i2cRxBuff[1]<<8 | i2cRxBuff[2]<<8;
//		pData_f = pData / 64;

//		sprintf(uartTxBuff, "baro: %f\r\n", pData_f);
//		HAL_UART_Transmit(&huart3, (uint8_t *) uartTxBuff, strlen(uartTxBuff), 100);

		/*
		 * temperature
		 */

//		tData = i2cRxBuff[3]<<8 | i2cRxBuff[4];
//		tData_f = tData / 256;

//		sprintf(uartTxBuff, "temp: %f\r\n", tData_f);
//		HAL_UART_Transmit(&huart3, (uint8_t *) uartTxBuff, strlen(uartTxBuff), 100);

	}

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
  MX_I2C1_Init();
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */


  /* USER CODE END 2 */
 
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  IMU_Init();
  Alti_Init();

  HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1);
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	  readIMU();
	  readAlti();

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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 48000;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 500;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 100;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim14, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */
  HAL_TIM_MspPostInit(&htim14);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
