/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/*
 * Example used:
 * https://github.com/STMicroelectronics/STMems_Standard_C_drivers/blob/master/lsm6dsr_STdC/examples/lsm6dsr_read_data_polling.c
 * https://docs.google.com/document/d/11LbYRzzlxmaHmQzeuiTY6JHf88UdYm1TuBmDsukHJq8/edit
 */

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include "ism330dlc_reg.h"
#include "stm32f4xx_hal.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart2;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define SENSOR_BUS hi2c1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define    BOOT_TIME            10 //ms
#define TX_BUF_DIM          1000

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

//Necessary for setting up the device
static uint8_t whoamI, rst;
stmdev_ctx_t dev_ctx;

//Necessary for acceleration
static int16_t data_raw_acceleration[3];
static float acceleration_mg[3];

//Necessary for angular rate
static int16_t data_raw_angular_rate[3];
static float angular_rate_mdps[3];

//Necessary for temp
static int16_t data_raw_temperature;
static float temperature_degC;

//Necessary to output data
static uint8_t tx_buffer[TX_BUF_DIM];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/*
 *   WARNING:
 *   Functions declare in this section must be defined at the end of this file
 *
 */
static int32_t write(void *handle, uint8_t reg, const uint8_t *bufp,uint16_t len);
static int32_t read(void *handle, uint8_t reg, uint8_t *bufp,uint16_t len);
static void tx_com( uint8_t *tx_buffer, uint16_t len );
void setup(stmdev_ctx_t* dev_ctx);
void lsm6dsr_read_data_polling(stmdev_ctx_t* dev_ctx);

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

	//If Float error message: go to Project->Properties->C/C++ Build->Settings->
	//MCU Settings->Check the box "Use float with printf"

	/* Main Example --------------------------------------------------------------*/


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
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HAL_UART_Transmit(&huart3,"\n\rMain Loop Starts\n\r", 24, HAL_MAX_DELAY);
	  setup(&dev_ctx);
	  lsm6dsr_read_data_polling(&dev_ctx);
	  HAL_UART_Transmit(&huart3,"Main Loop Terminated\n\r", 24, HAL_MAX_DELAY);
	  break;
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  HAL_UART_Transmit(&huart3,"Program Terminated\n\r", 22, HAL_MAX_DELAY);
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 38400;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(INT1_GPIO_Port, INT1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : INT1_Pin */
  GPIO_InitStruct.Pin = INT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(INT1_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void setup(stmdev_ctx_t *dev_ctx)
	{
	  HAL_UART_Transmit(&huart3,"LISM330DLC Setup Starts\n\r", 27, HAL_MAX_DELAY);
	  /* Initialize mems driver interface */
	  dev_ctx->write_reg = write;
	  dev_ctx->read_reg = read;
	  dev_ctx->handle = &SENSOR_BUS;
	  /* Wait sensor boot time */
	  HAL_Delay(BOOT_TIME);
	  /* Check device ID */
	  ism330dlc_device_id_get(dev_ctx, &whoamI);

	  	  HAL_UART_Transmit(&huart3,"Checking Sensor ID...", 21, HAL_MAX_DELAY);
	  if (whoamI != ISM330DLC_ID){
		  HAL_UART_Transmit(&huart3,"NOT OK\n\r", 10, HAL_MAX_DELAY);
		  HAL_UART_Transmit(&huart3,"This Device is: " , 16, HAL_MAX_DELAY);
		  HAL_UART_Transmit(&huart3,whoamI, 2, HAL_MAX_DELAY);
		  HAL_UART_Transmit(&huart3,"\n\rProgram Terminated\n\r", 26, HAL_MAX_DELAY);
		  while(1);
	  }
	  	  HAL_UART_Transmit(&huart3,"OK\n\r", 6, HAL_MAX_DELAY);

	  /* Restore default configuration */
	  ism330dlc_reset_set(dev_ctx, PROPERTY_ENABLE);

	  do {
		ism330dlc_reset_get(dev_ctx, &rst);
	  } while (rst);


	  /* Enable Block Data Update */
	    ism330dlc_block_data_update_set(dev_ctx, PROPERTY_ENABLE);
	    /* Set Output Data Rate */
	    ism330dlc_xl_data_rate_set(dev_ctx, ISM330DLC_XL_ODR_12Hz5);
	    ism330dlc_gy_data_rate_set(dev_ctx, ISM330DLC_GY_ODR_12Hz5);
	    /* Set full scale */
	    ism330dlc_xl_full_scale_set(dev_ctx, ISM330DLC_2g);
	    ism330dlc_gy_full_scale_set(dev_ctx, ISM330DLC_2000dps);
	    /* Configure filtering chain(No aux interface) */
	    /* Accelerometer - analog filter */
	    ism330dlc_xl_filter_analog_set(dev_ctx, ISM330DLC_XL_ANA_BW_400Hz);
	    /* Accelerometer - LPF1 path ( LPF2 not used )*/
	    //ism330dlc_xl_lp1_bandwidth_set(dev_ctx, ISM330DLC_XL_LP1_ODR_DIV_4);
	    /* Accelerometer - LPF1 + LPF2 path */
	    ism330dlc_xl_lp2_bandwidth_set(dev_ctx,
	                                   ISM330DLC_XL_LOW_NOISE_LP_ODR_DIV_100);
	    /* Accelerometer - High Pass / Slope path */
	    //ism330dlc_xl_reference_mode_set(&dev_ctx, PROPERTY_DISABLE);
	    //ism330dlc_xl_hp_bandwidth_set(&dev_ctx, ISM330DLC_XL_HP_ODR_DIV_100);
	    /* Gyroscope - filtering chain */
	    ism330dlc_gy_band_pass_set(dev_ctx, ISM330DLC_HP_260mHz_LP1_STRONG);
	    HAL_UART_Transmit(&huart3,"LISM330DLC Setup Ends\n\r", 25, HAL_MAX_DELAY);
	}

void lsm6dsr_read_data_polling(stmdev_ctx_t *dev_ctx)
{
	HAL_UART_Transmit(&huart3,"lism330dlc_read_data_polling called\n\r", 39, HAL_MAX_DELAY);
	  /* Read samples in polling mode */
	  	  HAL_UART_Transmit(&huart3,"Data reading loop starts in\n\r", 31, HAL_MAX_DELAY);
	  	  HAL_Delay(1000);
	  	  HAL_UART_Transmit(&huart3,"3", 3, HAL_MAX_DELAY);
	  	  HAL_Delay(1000);
	  	  HAL_UART_Transmit(&huart3,"\r2", 3, HAL_MAX_DELAY);
	  	  HAL_Delay(1000);
	  	  HAL_UART_Transmit(&huart3,"\r1\n\r", 3, HAL_MAX_DELAY);
	  	  HAL_Delay(1000);

	  while (1) {
		  /* Read output only if new value is available */
		  ism330dlc_reg_t reg; //For some reason, this one has to be in the loop
		  ism330dlc_status_reg_get(dev_ctx, &reg.status_reg);

		      if (reg.status_reg.xlda) {
		        /* Read magnetic field data */
		        memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
		        ism330dlc_acceleration_raw_get(dev_ctx, data_raw_acceleration);
		        acceleration_mg[0] = ism330dlc_from_fs2g_to_mg(
		                               data_raw_acceleration[0]);
		        acceleration_mg[1] = ism330dlc_from_fs2g_to_mg(
		                               data_raw_acceleration[1]);
		        acceleration_mg[2] = ism330dlc_from_fs2g_to_mg(
		                               data_raw_acceleration[2]);
		        sprintf((char *)tx_buffer,
		                "Acceleration [mg]:%4.2f\t%4.2f\t%4.2f\r\n",
		                acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
		        //tx_com( tx_buffer, strlen( (char const *)tx_buffer ) );
		      }

		      if (reg.status_reg.gda) {
		        /* Read magnetic field data */
		        memset(data_raw_angular_rate, 0x00, 3 * sizeof(int16_t));
		        ism330dlc_angular_rate_raw_get(dev_ctx, data_raw_angular_rate);
		        angular_rate_mdps[0] = ism330dlc_from_fs2000dps_to_mdps(
		                                 data_raw_angular_rate[0]);
		        angular_rate_mdps[1] = ism330dlc_from_fs2000dps_to_mdps(
		                                 data_raw_angular_rate[1]);
		        angular_rate_mdps[2] = ism330dlc_from_fs2000dps_to_mdps(
		                                 data_raw_angular_rate[2]);
		        sprintf((char *)tx_buffer,
		                "Angular rate [mdps]:%4.2f\t%4.2f\t%4.2f\r\n",
		                angular_rate_mdps[0], angular_rate_mdps[1], angular_rate_mdps[2]);
		        tx_com( tx_buffer, strlen( (char const *)tx_buffer ) );
		      }

		      if (reg.status_reg.tda) {
		        /* Read temperature data */
		        memset( &data_raw_temperature, 0x00, sizeof(int16_t));
		        ism330dlc_temperature_raw_get(dev_ctx, &data_raw_temperature);
		        temperature_degC = ism330dlc_from_lsb_to_celsius(
		                             data_raw_temperature );
		        sprintf((char *)tx_buffer, "Temperature [degC]:%6.2f\r\n",
		                temperature_degC );
		        //tx_com( tx_buffer, strlen( (char const *)tx_buffer ) );
		      }

	    HAL_Delay(100);

	  }
	  HAL_UART_Transmit(&huart3,"Data reading loop ends\n\r", 26, HAL_MAX_DELAY);
	}


/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len)
{
  HAL_I2C_Mem_Write(handle, ISM330DLC_I2C_ADD_L, reg,
                    I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
  return 0;
}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{
  HAL_I2C_Mem_Read(handle, ISM330DLC_I2C_ADD_L, reg,
                   I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
  return 0;
}

/*
 * @brief  Send buffer to console (platform dependent)
 *
 * @param  tx_buffer     buffer to transmit
 * @param  len           number of byte to send
 *
 */
static void tx_com(uint8_t *tx_buffer, uint16_t len)
{
  HAL_UART_Transmit(&huart3, tx_buffer, len, 1000);
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
