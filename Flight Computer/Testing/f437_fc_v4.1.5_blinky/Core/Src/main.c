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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

// usb vcp
#include "usbd_cdc_if.h"
#include "string.h"

// sensors
#include "i2c_sensor_functions.h"
#include "gps.h"
#include "MAX31855.h"
#include "bme280.h"
#include "bme280_defs.h"

#include "sd_card.h"
#include "w25qxx.h" // external flash

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//#define TEST_BLINKY 						// pass
//#define TEST_EJECTION 					// pass
//#define TEST_VR 							// pass
//#define TEST_SD_CARD_ALONE 				// pass
//#define TEST_SD_CARD_DYNAMIC_FILE_NAMES	// pass
//#define TEST_GPS_ALONE					// pass but GPS fix inconsistent sometimes
//#define TEST_FLASH_W25QXX_ALONE			// status?
//#define TEST_I2C_SENSORS_ALONE			// pass
#define TEST_ALL_SENSORS_WITH_SD_CARD		// pass
//#define RECORD_VIDEO_WITH_TEST			// activates video recorder in TEST_ALL_SENSORS_WITH_SD_CARD
//#define OUTPUT_USB_WITH_TEST				// sends string to USB with TEST_ALL_SENSORS_WITH_SD_CARD

//#define TEST_USB_VCP_ALONE				// pass

//#define TEST_I2C_PAYLOAD

//#define TEST_VENT_VALVE
//#define TEST_PRESSURE_TRANSDUCER_ADC
//#define TEST_THERMOCOUPLE_ADC

//#define HOLIDAY_LED_BLINKY

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

SPI_HandleTypeDef hspi4;
SPI_HandleTypeDef hspi5;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
volatile uint8_t start_ejection = 0;
volatile uint8_t timer_actuated_vent_valve = 0;

// i2c sensors
stmdev_ctx_t dev_ctx_lsm;
stmdev_ctx_t dev_ctx_lps;
float acceleration_mg[] = {0, 0, 0};
float angular_rate_mdps[]= {0, 0, 0};
float pressure_hPa = 0;
float temperature_degC = 0;

struct bme280_dev dev_bme280;

// gps data
float latitude;
float longitude;
float time;
static uint8_t gps_fix_lat = 0;
static uint8_t gps_fix_long = 0; // beep when we get fix

// sd card
FATFS FatFs; 	//Fatfs handle
FIL fil; 		//File handle
FRESULT fres; //Result after operations
static uint8_t msg_buffer[1000];

// external flash
extern w25qxx_t w25qxx;
uint8_t flash_write_buffer[256]; // page size is 256 bytes for w25qxx
uint8_t flash_read_buffer[256];
static volatile uint32_t flash_write_address = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI4_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI5_Init(void);
static void MX_I2C3_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

// helpers
void tone(uint32_t duration, uint32_t repeats);

// bme280
void user_delay_ms(uint32_t period, void *intf_ptr);
int8_t user_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);
int8_t user_i2c_write(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);

// flash and sd
void save_flash_to_sd(void);

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
  MX_ADC1_Init();
  MX_SPI4_Init();
  MX_TIM2_Init();
  MX_SPI5_Init();
  MX_FATFS_Init();
  MX_I2C3_Init();
  MX_USART6_UART_Init();
  MX_I2C2_Init();
  MX_USB_DEVICE_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  // reset LEDs
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, RESET);
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, RESET);
  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, RESET);

  // reset recovery pyro pins
  HAL_GPIO_WritePin(Rcov_Arm_GPIO_Port, Rcov_Arm_Pin, RESET);
  HAL_GPIO_WritePin(Rcov_Gate_Drogue_GPIO_Port, Rcov_Gate_Drogue_Pin, RESET);
  HAL_GPIO_WritePin(Rcov_Gate_Main_GPIO_Port, Rcov_Gate_Main_Pin, RESET);

  // reset prop pyro pins
  HAL_GPIO_WritePin(Prop_Pyro_Arming_GPIO_Port, Prop_Pyro_Arming_Pin, RESET);
  HAL_GPIO_WritePin(Prop_Gate_1_GPIO_Port, Prop_Gate_1_Pin, RESET);
  HAL_GPIO_WritePin(Prop_Gate_2_GPIO_Port, Prop_Gate_2_Pin, RESET);

  // reset 12 V buck converter enable pin (disable converter)
  HAL_GPIO_WritePin(PM_12V_EN_GPIO_Port, PM_12V_EN_Pin, RESET);
  HAL_GPIO_WritePin(Vent_Valve_EN_GPIO_Port, Vent_Valve_EN_Pin, RESET);

  // reset payload EN signal
  HAL_GPIO_WritePin(Payload_EN_GPIO_Port, Payload_EN_Pin, RESET);

  // set CS pin for thermocouple chip high
  //	HAL_GPIO_WritePin(TH_CS_1_GPIO_Port, TH_CS_1_Pin, SET);

  // set power off for VR
  HAL_GPIO_WritePin(VR_CTRL_PWR_GPIO_Port, VR_CTRL_PWR_Pin, RESET);
  HAL_GPIO_WritePin(VR_CTRL_REC_GPIO_Port, VR_CTRL_REC_Pin, RESET);

  // FLASH set CS, WP and IO3 pins high
  HAL_GPIO_WritePin(CS_FLASH_GPIO_Port, CS_FLASH_Pin, SET);
  HAL_GPIO_WritePin(FLASH_WP_GPIO_Port, FLASH_WP_Pin, SET);
  HAL_GPIO_WritePin(FLASH_IO3_GPIO_Port, FLASH_IO3_Pin, SET);


  dev_ctx_lsm = lsm6dsl_init();
  dev_ctx_lps = lps22hh_init();

#ifdef HOLIDAY_LED_BLINKY
	// use ejection channels to blink LEDs like Christmas lights
	while (1)
	{
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, SET);
		HAL_GPIO_WritePin(Rcov_Arm_GPIO_Port, Rcov_Arm_Pin, SET);
		HAL_GPIO_WritePin(Prop_Pyro_Arming_GPIO_Port, Prop_Pyro_Arming_Pin, SET);

		HAL_GPIO_WritePin(Prop_Gate_1_GPIO_Port, Prop_Gate_1_Pin, SET);
		HAL_Delay(100);
		HAL_GPIO_WritePin(Prop_Gate_1_GPIO_Port, Prop_Gate_1_Pin, RESET);
		HAL_GPIO_WritePin(Prop_Gate_2_GPIO_Port, Prop_Gate_2_Pin, SET);
		HAL_Delay(100);
		HAL_GPIO_WritePin(Prop_Gate_2_GPIO_Port, Prop_Gate_2_Pin, RESET);
		HAL_GPIO_WritePin(Rcov_Gate_Main_GPIO_Port, Rcov_Gate_Main_Pin, SET);
		HAL_Delay(100);
		HAL_GPIO_WritePin(Rcov_Gate_Main_GPIO_Port, Rcov_Gate_Main_Pin, RESET);
		HAL_GPIO_WritePin(Rcov_Gate_Drogue_GPIO_Port, Rcov_Gate_Drogue_Pin, SET);
		HAL_Delay(100);
		HAL_GPIO_WritePin(Rcov_Gate_Drogue_GPIO_Port, Rcov_Gate_Drogue_Pin, RESET);
		HAL_Delay(100);
	}
#endif

#ifdef TEST_USB_VCP_ALONE

	char txBuf[100];
	uint8_t count = 1;

	while (1)
	{
		HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);

		sprintf(txBuf, "Printing via USB: count = %u\r\n", count);
		count++;
		if (count > 100)
			count = 1;

		CDC_Transmit_FS((uint8_t *)txBuf, strlen(txBuf));
		HAL_Delay(100);
	}
#endif

#ifdef TEST_I2C_PAYLOAD
	char msg_buffer[100]; // for usb vcp

	// testing i2c for payload over long wires
	int8_t rslt = BME280_OK;
	uint8_t dev_addr = BME280_I2C_ADDR_SEC;

	dev_bme280.intf_ptr = &dev_addr;
	dev_bme280.intf = BME280_I2C_INTF;
	dev_bme280.read = user_i2c_read;
	dev_bme280.write = user_i2c_write;
	dev_bme280.delay_us = user_delay_ms;

	rslt = bme280_init(&dev_bme280);
	HAL_Delay(100);

	if (rslt != 0) {
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, SET); // turn LED on to indicate fault
		while (1);
	}
	struct bme280_data comp_data;

	uint8_t settings_sel;

	/* Recommended mode of operation: Indoor navigation */
	dev_bme280.settings.osr_h = BME280_OVERSAMPLING_1X;
	dev_bme280.settings.osr_p = BME280_OVERSAMPLING_16X;
	dev_bme280.settings.osr_t = BME280_OVERSAMPLING_2X;
	dev_bme280.settings.filter = BME280_FILTER_COEFF_16;
	dev_bme280.settings.standby_time = BME280_STANDBY_TIME_62_5_MS;

	settings_sel = BME280_OSR_PRESS_SEL;
	settings_sel |= BME280_OSR_TEMP_SEL;
	settings_sel |= BME280_OSR_HUM_SEL;
	settings_sel |= BME280_STANDBY_SEL;
	settings_sel |= BME280_FILTER_SEL;
	rslt = bme280_set_sensor_settings(settings_sel, &dev_bme280);
	rslt = bme280_set_sensor_mode(BME280_NORMAL_MODE, &dev_bme280);

	while (1)
	{
		int8_t status = bme280_get_sensor_data(BME280_ALL, &comp_data, &dev_bme280);
		if (status != 0)
		{
			HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, SET);
			while (1);
		}

		sprintf((char*)msg_buffer, "Pressure = %d, Temperature = %d, Humidity = %d\r\n", comp_data.pressure, comp_data.temperature, comp_data.humidity);
		CDC_Transmit_FS((uint8_t *)msg_buffer, strlen(msg_buffer));

		HAL_Delay(100);
	}

#endif

#ifdef TEST_I2C_SENSORS_ALONE

	while (1)
	{
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, SET);

		// lsm6dsl data
		get_acceleration(dev_ctx_lsm, acceleration_mg);
		get_angvelocity(dev_ctx_lsm, angular_rate_mdps);

		// lps22hh data
		get_pressure(dev_ctx_lps, &pressure_hPa);
		get_temperature(dev_ctx_lps, &temperature_degC);

		HAL_Delay(100);
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, RESET);
		HAL_Delay(100);
	}
#endif

#ifdef TEST_GPS_ALONE
	while (1)
	{
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, SET);
		GPS_Poll(&latitude, &longitude, &time);
		GPS_check_nonzero_data(latitude, longitude, &gps_fix_lat, &gps_fix_long);

		HAL_Delay(100);
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, RESET);
		HAL_Delay(100);
	}
#endif

#ifdef TEST_FLASH_W25QXX_ALONE

	// easy test
	/*
	uint8_t buffer1[8] = {1, 2, 3, 4, 5, 6, 7, 8};
	uint8_t buffer2[8];

	// init FLASH
	if (W25qxx_Init()) {
		tone(250, 2); // init properly
	} else {
		tone(750, 2);
		Error_Handler(); // hangs and blinks LEDF
	}

	  W25qxx_EraseSector(1);
	  W25qxx_WriteSector(buffer1, 1, 0, 8);
	  W25qxx_ReadSector(buffer2, 1, 0, 8);

	  while (1);
	  */

	// harder test
	// init sd
	char filename[13]; // filename will be of form fc000000.txt which is 13 chars in the array (with null termination)
	fres = sd_init_dynamic_filename("FC", "S,PRESSURE_HPA,TEMP_DEG_C,ACCx,ACCy,ACCz,GYRx,GYRy,GYRz,LAT,LONG,E\r\n", filename);
	if (fres != FR_OK) {
		Error_Handler();
	}

	// init flash
	if (W25qxx_Init()) {
		tone(250, 2);
	} else {
		tone(750, 2);
		Error_Handler(); // hang and blinky
	}

	save_flash_to_sd(); // check if flash empty and write to sd card

	// open sd card file for simultaneous logging
	fres = f_open(&fil, filename, FA_WRITE | FA_OPEN_ALWAYS | FA_CREATE_ALWAYS);

	if (fres == FR_OK) {
		myprintf("I was able to open filename.txt for writing\r\n");
	} else {
		myprintf("f_open error (%i)\r\n", fres);
		return fres;
	}
	// set pointer to end of file to append
	f_lseek(&fil, f_size(&fil));

	// now make fake data
	while (1)
	{
		// flash should be cleared by now
		// create dummy data to be written to the flash chip
		for (uint32_t i = 0; i < 50; i++) // 300 for laughs
		{
//			sprintf((char *)flash_write_buffer, "S,%ld,0.12,253.70,280.00,-840.00,-560.00,0.0,1004.84,45.5052567,-73.5796127,20,10,34,17,%ld,%ldE,\r\n", i, 50-i,i);
			sprintf((char *)flash_write_buffer, "S,%ld,0.12,25.0,20.0,40.20,-5+0.0,0.0,1343204.z4,45.507,-73.5796127,20,10,34,17,%ld+%ld,%ldE,\r\n", i,i, 50-i,i);
			sd_write(&fil, flash_write_buffer);

			// calculate page address and offset based on number of bytes already written
			uint32_t page_address = (int) (flash_write_address / w25qxx.BlockSize);
			uint32_t page_offset = (int) (flash_write_address % w25qxx.BlockSize);

			W25qxx_WriteBlock(flash_write_buffer, page_address, page_offset, strlen((const char *)flash_write_buffer));
//			W25qxx_ReadBlock(flash_read_buffer, page_address, page_offset, strlen((const char *)flash_write_buffer));

			flash_write_address += strlen((const char *)flash_write_buffer);
		}

		f_close(&fil); // close logging file

		while (1)
		{
			HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, SET);
			HAL_Delay(100);
		}
	}
#endif

#ifdef TEST_ALL_SENSORS_WITH_SD_CARD

//	sd_init("fcdata.txt", "S,PRESSURE_HPA,TEMP_DEG_C,ACCx,ACCy,ACCz,GYRx,GYRy,GYRz,LAT,LONG,E\r\n");
	char filename[13]; // filename will be of form fc000000.txt which is 13 chars in the array (with null termination)
	fres = sd_init_dynamic_filename("FC", "S,PRESSURE_HPA,TEMP_DEG_C,ACCx,ACCy,ACCz,GYRx,GYRy,GYRz,LAT,LONG,E\r\n", filename);
	if (fres != FR_OK)
		Error_Handler();

#ifdef RECORD_VIDEO_WITH_TEST
	// successful init, can start video recorder
	// power on
	HAL_GPIO_WritePin(VR_CTRL_PWR_GPIO_Port, VR_CTRL_PWR_Pin, SET);

	HAL_Delay(5000);
	// start recording?
	HAL_GPIO_WritePin(VR_CTRL_REC_GPIO_Port, VR_CTRL_REC_Pin, SET);
	HAL_Delay(400);
	HAL_GPIO_WritePin(VR_CTRL_REC_GPIO_Port, VR_CTRL_REC_Pin, RESET);
	HAL_Delay(400);
	HAL_GPIO_WritePin(VR_CTRL_REC_GPIO_Port, VR_CTRL_REC_Pin, SET);
	HAL_Delay(400);
	HAL_GPIO_WritePin(VR_CTRL_REC_GPIO_Port, VR_CTRL_REC_Pin, RESET);
#endif

	while (1)
	{
		// poll data
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, SET);
		// lsm6dsl data
		get_acceleration(dev_ctx_lsm, acceleration_mg);
		get_angvelocity(dev_ctx_lsm, angular_rate_mdps);

		// lps22hh data
		get_pressure(dev_ctx_lps, &pressure_hPa);
		get_temperature(dev_ctx_lps, &temperature_degC);

		// use beeper for GPS to make sure we know whether coordinates are nonzero
		GPS_Poll(&latitude, &longitude, &time);
		GPS_check_nonzero_data(latitude, longitude, &gps_fix_lat, &gps_fix_long);

		// make buffer with data and save
		sprintf((char *)msg_buffer, "S,%03.2f,%03.2f,%03.2f,%03.2f,%03.2f,%03.2f,%03.2f,%03.2f,%03.7f,%03.7f,E\r\n",
				pressure_hPa, temperature_degC,
				acceleration_mg[0], acceleration_mg[1], acceleration_mg[2],
				angular_rate_mdps[0], angular_rate_mdps[1], angular_rate_mdps[2],
				latitude, longitude);

		HAL_UART_Transmit(&huart3, msg_buffer, strlen(msg_buffer), HAL_MAX_DELAY);

		fres = sd_open_file(filename);
		if (fres == FR_OK) {
			myprintf("I was able to open filename.txt for writing\r\n");
		} else {
			myprintf("f_open error (%i)\r\n", fres);
		}
		sd_write(&fil, msg_buffer);

#ifdef OUTPUT_USB_WITH_TEST
		CDC_Transmit_FS((uint8_t *)msg_buffer, strlen((char *)msg_buffer));
#endif

		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, RESET);
		HAL_Delay(100);

		// close file
		f_close(&fil);

		if (start_ejection) // stop when button pressed
			break;
	}

#ifdef RECORD_VIDEO_WITH_TEST
	// stop recording
	HAL_GPIO_WritePin(VR_CTRL_REC_GPIO_Port, VR_CTRL_REC_Pin, SET);
	HAL_Delay(1000);
	HAL_GPIO_WritePin(VR_CTRL_REC_GPIO_Port, VR_CTRL_REC_Pin, RESET);
#endif

	while (1); // stop here

#endif

	// start timer
#ifdef TEST_VENT_VALVE
	HAL_TIM_Base_Start_IT(&htim2); // doing PWM of the valve
#endif


#ifdef TEST_BLINKY
	while (1)
	{
		HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
		HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
		HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
		HAL_Delay(500);
	}

#endif

#ifdef TEST_SD_CARD_ALONE
	  HAL_Delay(1000); //a short delay is important to let the SD card settle

	  //Open the file system
	  fres = f_mount(&FatFs, "", 1); //1=mount now
	  if (fres != FR_OK) {
		myprintf("f_mount error (%i)\r\n", fres);
		while(1);
	  }

	  //Now let's try to open file "test.txt"
	  fres = f_open(&fil, "test.txt", FA_READ);
	  if (fres != FR_OK) {
		myprintf("f_open error (%i)\r\n");
		while(1);
	  }
	  myprintf("I was able to open 'test.txt' for reading!\r\n");

	  //Read 30 bytes from "test.txt" on the SD card
	  BYTE readBuf[30];

	  //We can either use f_read OR f_gets to get data out of files
	  //f_gets is a wrapper on f_read that does some string formatting for us
	  TCHAR* rres = f_gets((TCHAR*)readBuf, 30, &fil);
	  if(rres != 0) {
		myprintf("Read string from 'test.txt' contents: %s\r\n", readBuf);
	  } else {
		myprintf("f_gets error (%i)\r\n", fres);
	  }

	  //Be a tidy kiwi - don't forget to close your file!
	  f_close(&fil);

	  //Now let's try and write a file "write.txt"
	  fres = f_open(&fil, "write_10.txt", FA_WRITE | FA_OPEN_ALWAYS | FA_CREATE_ALWAYS);
	  if(fres == FR_OK) {
		myprintf("I was able to open 'write.txt' for writing\r\n");
	  } else {
		myprintf("f_open error (%i)\r\n", fres);
	  }

	  //Copy in a string
	  strncpy((char*)readBuf, "a new file is made!", 19);
	  UINT bytesWrote;
	  fres = f_write(&fil, readBuf, 19, &bytesWrote);
	  if(fres == FR_OK) {
		myprintf("Wrote %i bytes to 'write.txt'!\r\n", bytesWrote);
	  } else {
		myprintf("f_write error (%i)\r\n");
	  }

	  //Be a tidy kiwi - don't forget to close your file!
	  f_close(&fil);

	  //We're done, so de-mount the drive
	  f_mount(NULL, "", 0);

	  while (1); // stop here
#endif

#ifdef TEST_SD_CARD_DYNAMIC_FILE_NAMES
	  HAL_Delay(1000); //a short delay is important to let the SD card settle

	  //Open the file system
	  fres = f_mount(&FatFs, "", 1); //1=mount now
	  if (fres != FR_OK) {
		myprintf("f_mount error (%i)\r\n", fres);
		while(1);
	  }

	  char buff[256];
	  strcpy(buff, "/");

	  char filename[13]; // filename will be of form fc000000.txt which is 13 chars in the array (with null termination)
	  sd_init_dynamic_filename("FC", "S,PRESSURE_HPA,TEMP_DEG_C,ACCx,ACCy,ACCz,GYRx,GYRy,GYRz,LAT,LONG,E\r\n", filename);

	  while (1);


#endif

#ifdef TEST_VR
	  // power on
	  HAL_GPIO_WritePin(VR_CTRL_PWR_GPIO_Port, VR_CTRL_PWR_Pin, SET);

	  HAL_Delay(5000);
	  // start recording
	  HAL_GPIO_WritePin(VR_CTRL_REC_GPIO_Port, VR_CTRL_REC_Pin, SET);
	  HAL_Delay(400);
	  HAL_GPIO_WritePin(VR_CTRL_REC_GPIO_Port, VR_CTRL_REC_Pin, RESET);
	  HAL_Delay(400);
	  HAL_GPIO_WritePin(VR_CTRL_REC_GPIO_Port, VR_CTRL_REC_Pin, SET);
	  HAL_Delay(400);
	  HAL_GPIO_WritePin(VR_CTRL_REC_GPIO_Port, VR_CTRL_REC_Pin, RESET);

	  HAL_Delay(15000); // approx 12 seconds of recording

	  // stop recording
	  HAL_GPIO_WritePin(VR_CTRL_REC_GPIO_Port, VR_CTRL_REC_Pin, SET);
	  HAL_Delay(1000);
	  HAL_GPIO_WritePin(VR_CTRL_REC_GPIO_Port, VR_CTRL_REC_Pin, RESET);

	  while (1); // stop here

#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
#ifdef TEST_EJECTION
	  if (start_ejection)
	  {
		  // indicate arming occurred
//		  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
		  HAL_Delay(2000); // wait so i can probe voltage
//		  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
		  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, SET);
		  HAL_GPIO_WritePin(Rcov_Arm_GPIO_Port, Rcov_Arm_Pin, SET);
		  HAL_GPIO_WritePin(Prop_Pyro_Arming_GPIO_Port, Prop_Pyro_Arming_Pin, SET);
		  HAL_GPIO_WritePin(Payload_EN_GPIO_Port, Payload_EN_Pin, SET);

		  HAL_Delay(3000);

		  GPIO_PinState cont_drogue = HAL_GPIO_ReadPin(Rcov_Cont_Drogue_GPIO_Port, Rcov_Cont_Drogue_Pin);
		  GPIO_PinState cont_main = HAL_GPIO_ReadPin(Rcov_Cont_Main_GPIO_Port, Rcov_Cont_Main_Pin);

		  GPIO_PinState cont_prop_1 = HAL_GPIO_ReadPin(Prop_Cont_1_GPIO_Port, Prop_Cont_1_Pin);
		  GPIO_PinState cont_prop_2 = HAL_GPIO_ReadPin(Prop_Cont_2_GPIO_Port, Prop_Cont_2_Pin);

		  HAL_GPIO_WritePin(Rcov_Gate_Drogue_GPIO_Port, Rcov_Gate_Drogue_Pin, SET); // fire drogue and main
		  HAL_GPIO_WritePin(Rcov_Gate_Main_GPIO_Port, Rcov_Gate_Main_Pin, SET);
		  HAL_GPIO_WritePin(Prop_Gate_1_GPIO_Port, Prop_Gate_1_Pin, SET); // prop pyro channels
		  HAL_GPIO_WritePin(Prop_Gate_2_GPIO_Port, Prop_Gate_2_Pin, SET);

		  HAL_Delay(100);

		  HAL_GPIO_WritePin(Rcov_Gate_Drogue_GPIO_Port, Rcov_Gate_Drogue_Pin, RESET);
		  HAL_GPIO_WritePin(Rcov_Gate_Main_GPIO_Port, Rcov_Gate_Main_Pin, RESET);
		  HAL_GPIO_WritePin(Prop_Gate_1_GPIO_Port, Prop_Gate_1_Pin, RESET);
		  HAL_GPIO_WritePin(Prop_Gate_2_GPIO_Port, Prop_Gate_2_Pin, RESET);
		  HAL_GPIO_WritePin(Payload_EN_GPIO_Port, Payload_EN_Pin, RESET);

		  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, SET);

		  HAL_Delay(1000);

		  start_ejection = 0; // allow button press to trigger it again
		  HAL_GPIO_WritePin(Rcov_Arm_GPIO_Port, Rcov_Arm_Pin, RESET);
		  HAL_GPIO_WritePin(Prop_Pyro_Arming_GPIO_Port, Prop_Pyro_Arming_Pin, RESET);
		  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, RESET);
		  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, RESET);

	  }
#endif

//	  HAL_Delay(5000);

#ifdef TEST_VENT_VALVE
	  HAL_GPIO_WritePin(PM_12V_EN_GPIO_Port, PM_12V_EN_Pin, SET);

	  while (0)
	  {
		  GPIO_PinState vent_valve_fb_before = HAL_GPIO_ReadPin(Vent_Valve_FB_GPIO_Port, Vent_Valve_FB_Pin);
		  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, vent_valve_fb_before);
		  HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
		  HAL_GPIO_TogglePin(Vent_Valve_EN_GPIO_Port, Vent_Valve_EN_Pin);
		  GPIO_PinState vent_valve_fb_after = HAL_GPIO_ReadPin(Vent_Valve_FB_GPIO_Port, Vent_Valve_FB_Pin);
		  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, vent_valve_fb_after);
		  HAL_Delay(1);
	  }

	  while (1)
	  {
		  if (timer_actuated_vent_valve)
		  {
			  timer_actuated_vent_valve = 0;
			  HAL_GPIO_TogglePin(Vent_Valve_EN_GPIO_Port, Vent_Valve_EN_Pin);
		  }

	  }
#endif

#ifdef TEST_PRESSURE_TRANSDUCER_ADC
	  HAL_GPIO_WritePin(PM_12V_EN_GPIO_Port, PM_12V_EN_Pin, SET);
	  while (1)
	  {
		  HAL_ADC_Start(&hadc1);
		  HAL_ADC_PollForConversion(&hadc1, 1000);
		  uint32_t pressure_sensor_raw = HAL_ADC_GetValue(&hadc1);
		  HAL_ADC_Stop(&hadc1);

		  float voltage = (float) (pressure_sensor_raw / 4095.0);

		  HAL_Delay(500);
	  }
#endif

#ifdef TEST_THERMOCOUPLE_ADC
	  while (1) {
		  float th_temp = Max31855_Read_Temp();
	  	  HAL_Delay(500);
	  }
#endif
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi4.Init.NSS = SPI_NSS_SOFT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */

}

/**
  * @brief SPI5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI5_Init(void)
{

  /* USER CODE BEGIN SPI5_Init 0 */

  /* USER CODE END SPI5_Init 0 */

  /* USER CODE BEGIN SPI5_Init 1 */

  /* USER CODE END SPI5_Init 1 */
  /* SPI5 parameter configuration*/
  hspi5.Instance = SPI5;
  hspi5.Init.Mode = SPI_MODE_MASTER;
  hspi5.Init.Direction = SPI_DIRECTION_2LINES;
  hspi5.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi5.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi5.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi5.Init.NSS = SPI_NSS_SOFT;
  hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi5.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI5_Init 2 */

  /* USER CODE END SPI5_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 50;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 100;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  huart3.Init.BaudRate = 9600;
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
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 38400;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, PM_12V_EN_Pin|Vent_Valve_EN_Pin|Payload_EN_Pin|TH_CS_1_Pin
                          |TH_CS_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, SD_CS_Pin|Prop_Gate_2_Pin|Prop_Gate_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED1_Pin|LED2_Pin|LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, Prop_Pyro_Arming_Pin|VR_CTRL_PWR_Pin|Rcov_Gate_Main_Pin|Rcov_Gate_Drogue_Pin
                          |Rcov_Arm_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, FLASH_IO3_Pin|FLASH_WP_Pin|CS_FLASH_Pin|VR_CTRL_REC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PM_12V_EN_Pin Vent_Valve_EN_Pin Payload_EN_Pin TH_CS_1_Pin
                           TH_CS_2_Pin */
  GPIO_InitStruct.Pin = PM_12V_EN_Pin|Vent_Valve_EN_Pin|Payload_EN_Pin|TH_CS_1_Pin
                          |TH_CS_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_CS_Pin */
  GPIO_InitStruct.Pin = SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SD_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Button_Pin */
  GPIO_InitStruct.Pin = Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED2_Pin LED3_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : Vent_Valve_FB_Pin */
  GPIO_InitStruct.Pin = Vent_Valve_FB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Vent_Valve_FB_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Prop_Cont_2_Pin */
  GPIO_InitStruct.Pin = Prop_Cont_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Prop_Cont_2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Prop_Gate_2_Pin Prop_Gate_1_Pin */
  GPIO_InitStruct.Pin = Prop_Gate_2_Pin|Prop_Gate_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : Prop_Cont_1_Pin Rcov_Cont_Main_Pin Rcov_Cont_Drogue_Pin */
  GPIO_InitStruct.Pin = Prop_Cont_1_Pin|Rcov_Cont_Main_Pin|Rcov_Cont_Drogue_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : Prop_Pyro_Arming_Pin VR_CTRL_PWR_Pin Rcov_Gate_Main_Pin Rcov_Gate_Drogue_Pin
                           Rcov_Arm_Pin */
  GPIO_InitStruct.Pin = Prop_Pyro_Arming_Pin|VR_CTRL_PWR_Pin|Rcov_Gate_Main_Pin|Rcov_Gate_Drogue_Pin
                          |Rcov_Arm_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : FLASH_IO3_Pin FLASH_WP_Pin CS_FLASH_Pin VR_CTRL_REC_Pin */
  GPIO_InitStruct.Pin = FLASH_IO3_Pin|FLASH_WP_Pin|CS_FLASH_Pin|VR_CTRL_REC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == Button_Pin)
	{
		start_ejection = 1;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

#ifdef TEST_VENT_VALVE
	if (htim == &htim2)
	{
//		HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
//		HAL_GPIO_TogglePin(Vent_Valve_EN_GPIO_Port, Vent_Valve_EN_Pin);
		timer_actuated_vent_valve = 1;
	}
#endif
}


/**
 * reads data out of external FLASH and saves to SD card.
 * erases the used flash after finished.
 *
 * assumes f_mount has already been run.
 * this function does not close the file system.
 * opens a file "datalog.txt" and closes it when finished.
 */
void save_flash_to_sd(void)
{
	// FLASH variables
	uint32_t page_num = 0;
	uint16_t page_bytes = w25qxx.PageSize; // 256 bytes saved per page
	uint8_t readBuf[page_bytes];
	uint16_t page_address;

	// write to file
	fres = f_open(&fil, "flashlog.txt",
			FA_WRITE | FA_OPEN_ALWAYS);

	if (fres == FR_OK) {
		myprintf("I was able to open 'datalog.txt' for writing\r\n");
	} else {
		myprintf("f_open error (%i)\r\n", fres);
	}
	// set pointer to end of file
	f_lseek(&fil, f_size(&fil));

	// print string to indicate new log session
	sprintf((char *)msg_buffer, "\n--- new logging session! ---\r\n");
	sd_write(&fil, msg_buffer);

	for (page_num = 0; page_num < w25qxx.PageCount; page_num++)
	{
//		page_address = page_num * w25qxx.PageSize;

		if (!W25qxx_IsEmptyPage(page_num, 0, page_bytes))
//		if ((!W25qxx_IsEmptyPage(page_num, 0, page_bytes)) || page_num < 5)
		{
			// page not empty, read page out of flash
			W25qxx_ReadPage(readBuf, page_num, 0, page_bytes);

			// save to SD
			int8_t status = sd_write(&fil, readBuf);
			if (status <= 0) {
				HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, SET);
			}
		}
		else break; // page empty, no need to continue
	}

	// close file
	f_close(&fil);

	if (page_num == 0) // nothing saved
	{
		tone(50, 5);
		while (!start_ejection);
		start_ejection = 0;
		tone(50, 3);
	}
	else
	{
		// wait for button press to erase
		tone(250, 3);
		while (!start_ejection);
		start_ejection = 0;

	//	W25qxx_EraseChip();

		// clear the blocks with data
		uint32_t blocks_to_clear = W25qxx_PageToBlock(page_num);
		for (uint32_t block = 0; block <= blocks_to_clear; block++)
		{
			W25qxx_EraseBlock(block);
		}

		// notify that erase is finished
		tone(100, 3);
		while (!start_ejection);
		start_ejection = 0;

		tone(50, 3);
	}

}

void tone(uint32_t duration, uint32_t repeats)
{
	for (uint32_t i = 0; i < repeats; i++)
	{
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
		HAL_Delay(duration); // wait so i can probe voltage
		HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
		HAL_Delay(duration);
	}
}

void user_delay_ms(uint32_t period, void *intf_ptr)
{
    /*
     * Return control or wait,
     * for a period amount of milliseconds
     */
	HAL_Delay(50);
}

int8_t user_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    /* Return 0 for Success, non-zero for failure */

    /*
     * The parameter intf_ptr can be used as a variable to store the I2C address of the device
     */

    /*
     * Data on the bus should be like
     * |------------+---------------------|
     * | I2C action | Data                |
     * |------------+---------------------|
     * | Start      | -                   |
     * | Write      | (reg_addr)          |
     * | Stop       | -                   |
     * | Start      | -                   |
     * | Read       | (reg_data[0])       |
     * | Read       | (....)              |
     * | Read       | (reg_data[len - 1]) |
     * | Stop       | -                   |
     * |------------+---------------------|
     */
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c2, (BME280_I2C_ADDR_SEC << 1), reg_addr, I2C_MEMADD_SIZE_8BIT, reg_data, len, HAL_MAX_DELAY);
    return (status == HAL_OK ? 0 : 1);

}

int8_t user_i2c_write(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    /* Return 0 for Success, non-zero for failure */

    /*
     * The parameter intf_ptr can be used as a variable to store the I2C address of the device
     */

    /*
     * Data on the bus should be like
     * |------------+---------------------|
     * | I2C action | Data                |
     * |------------+---------------------|
     * | Start      | -                   |
     * | Write      | (reg_addr)          |
     * | Write      | (reg_data[0])       |
     * | Write      | (....)              |
     * | Write      | (reg_data[len - 1]) |
     * | Stop       | -                   |
     * |------------+---------------------|
     */

    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&hi2c2, (BME280_I2C_ADDR_SEC << 1), reg_addr, I2C_MEMADD_SIZE_8BIT, reg_data, len, HAL_MAX_DELAY);
	return (status == HAL_OK ? 0 : 1);
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
//  __disable_irq();
  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, SET); // error occurred, fatal

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_Delay(1000);
  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
  while (1)
  {
	  HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin); // error occurred, fatal
	  HAL_Delay(500);
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
