/*
 * MRT_setup.c
 *
 *  Created on: Apr 12, 2022
 *      Author: Jacoby
 */

#include <MRT_setup.h>
#include <main.h>
#include <iwdg.h>

#include <MRT_rtc.h>
#include <MRT_external_flash.h>
#include <MRT_i2c_sensors.h>
#include <sd_card.h>

//**************************************************//
//PRIVATE FUNCTIONS PROTOTYPES

void MRT_Reinitialize_Peripherals(void);



//**************************************************//
//PUBLIC FUNCTIONS

void MRT_Init(void){
	print("\r\n\r\n/********MRT Init********/\r\n");

	MRT_Reinitialize_Peripherals();
	MRT_external_flash_Init();
	MRT_reset_info();

	#if IWDG_ACTIVE
	print("IWDG Init...");
	MX_IWDG_Init();
	print("OK\r\n");
	#endif

	//RTC
	HAL_IWDG_Refresh(&hiwdg);
	MRT_rtc_Init();

	//TODO SD card (doesn't work)
	#if MEMORY_THREAD

		//SD card
		#if SD_CARD_
			HAL_IWDG_Refresh(&hiwdg);
			sd_init_dynamic_filename("FC", "", filename);
		#endif
	#endif


	//Sensors
	#if SENSORS_THREAD

		//Scan I2C buses (only for debug mode)
		#if CHECK_I2C && DEBUG
		  checkForI2CDevices(huart8,hi2c1);
		  checkForI2CDevices(huart8,hi2c2);
		  checkForI2CDevices(huart8,hi2c3);
		#endif

		//LSM6DSR
		#if LSM6DSR_
		HAL_IWDG_Refresh(&hiwdg);
		hlsm6dsr = MRT_LSM6DSR_Setup(&LSM6DSR_I2C, MRT_LSM6DSR_ID);
		#endif

		//LPS22HH
		#if LPS22HH_
		HAL_IWDG_Refresh(&hiwdg);
		hlps22hh = MRT_LPS22HH_Setup(&LPS22HH_I2C, MRT_LPS22HH_ID);
		#endif

		//GPS
		#if GPS_
		HAL_IWDG_Refresh(&hiwdg);
		GPS_init(&GPS_UART, &DEBUG_UART);
		#endif
	#endif


//FOR TESTING

#define TX_BUF_DIM 256
char buffer[TX_BUF_DIM];

	while(1){
		HAL_GPIO_WritePin(OUT_LED2_GPIO_Port, OUT_LED2_Pin, SET);
		HAL_Delay(1000);

		  //GPS
		  GPS_Poll(&gps_latitude, &gps_longitude, &gps_time);

	  	  //LSM6DSR
	  	  MRT_LSM6DSR_getAcceleration(hlsm6dsr,acceleration_mg);
	  	  MRT_LSM6DSR_getAngularRate(hlsm6dsr,angular_rate_mdps);
		  MRT_LSM6DSR_getTemperature(hlsm6dsr,&lsm6dsr_temperature_degC);

		  //LPS22HH
		  MRT_LPS22HH_getTemperature(hlps22hh,&lps22hh_temperature_degC);
		  MRT_LPS22HH_getPressure(hlps22hh, &pressure_hPa);
		  //altitude_m = MRT_get_altitude(pressure_hPa); //Update altitude TODO put somewhere else




		  //GPS
		  memset(buffer, 0, TX_BUF_DIM);
		  sprintf(buffer,"Alt: %.2f   Long: %.2f   Time: %.0f\r\n",gps_latitude, gps_longitude, gps_time);
		  HAL_UART_Transmit(&DEBUG_UART,buffer,strlen(buffer),HAL_MAX_DELAY);

		  //LSM6DSR
		  memset(buffer, 0, TX_BUF_DIM);
		  sprintf(buffer, "Acceleration [mg]:%4.2f\t%4.2f\t%4.2f\r\n",acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
		  HAL_UART_Transmit(&DEBUG_UART, buffer, strlen(buffer), HAL_MAX_DELAY);

		  memset(buffer, 0, TX_BUF_DIM);
		  sprintf(buffer,"Angular rate [mdps]:%4.2f\t%4.2f\t%4.2f\r\n",angular_rate_mdps[0], angular_rate_mdps[1], angular_rate_mdps[2]);
		  HAL_UART_Transmit(&DEBUG_UART, buffer, strlen(buffer), HAL_MAX_DELAY);

		  memset(buffer, 0, TX_BUF_DIM);
		  sprintf(buffer, "Temperature [degC]:%6.2f\r\n", lsm6dsr_temperature_degC);
		  HAL_UART_Transmit(&DEBUG_UART, buffer, strlen(buffer), HAL_MAX_DELAY);


		  //LPS22HH
		  memset(buffer, 0, TX_BUF_DIM);
		  sprintf(buffer,"Pressure [hPa]:%6.2f\r\n",pressure_hPa);
		  HAL_UART_Transmit(&DEBUG_UART, buffer, strlen(buffer), HAL_MAX_DELAY);

		  memset(buffer, 0, TX_BUF_DIM);
		  sprintf(buffer, "Temperature [degC]:%6.2f\r\n", lps22hh_temperature_degC);
		  HAL_UART_Transmit(&DEBUG_UART, buffer, strlen(buffer), HAL_MAX_DELAY);

		HAL_GPIO_WritePin(OUT_LED2_GPIO_Port, OUT_LED2_Pin, RESET);
		HAL_Delay(1000);
		HAL_IWDG_Refresh(&hiwdg);
	}

}


void MRT_reset_info(void){

	  char buffer[100];
	  sprintf(buffer,"Reset: %i,  WU: %i,  IWDG: %i\r\nPrevious RTC time: %i:%i:%i\r\n",reset_flag, wu_flag, iwdg_flag, prev_hours, prev_min, prev_sec);
	  print(buffer);

	  //Check if IWDG is being deactivated
	  if (iwdg_flag==1){
		  print("Deactivating IWDG\r\n");

		  iwdg_flag = 0; //Flip flag

		  //Write new flag to flash memory
		  flash_flags_buffer[IWDG_FLAG_OFFSET] = iwdg_flag;
		  W25qxx_EraseSector(1);
		  W25qxx_WriteSector(flash_flags_buffer, 1, FLAGS_OFFSET, NB_OF_FLAGS);

		  HAL_Delay(1000);

		  //Go to sleep
		  MRT_StandByMode(SLEEP_TIME);
	  }


	  //Check if we are after waking up (and at which wake up we are at)
	  if (wu_flag>0){
		  char buf[30];
		  sprintf(buf, "FC wake up %i\r\n", wu_flag);
		  print(buf);

		  print("Resetting RTC time\r\n");

		  //Clear RTC time (last recorded)
		  W25qxx_EraseSector(2);
		  W25qxx_WriteSector(RTC_TIME_NULL_BUFFER, 2, RTC_TIME_OFFSET, 3);

		  //Update variables (to 0)
		  for (int i = 0; i < 3; i++){
			  *flash_time[i] = 0x0;
		  }

	  }


	  //Check if we start from the beginning
	  if (reset_flag==0){
		  print("FC restarted\r\n");

		  reset_flag = 1; //Flip flag

		  //Write new flag to flash memory
	      flash_flags_buffer[RESET_FLAG_OFFSET] = reset_flag;
		  W25qxx_EraseSector(1);
		  W25qxx_WriteSector(flash_flags_buffer, 1, FLAGS_OFFSET, NB_OF_FLAGS);
	  }


	  //Check if before or after apogee
	  if (apogee_flag == 0){
		  print("Pre-apogee\r\n");
	  }
	  else if(apogee_flag==1){
		  print("Post-apogee\r\n");
	  }


	  //Check ejection stage
	  print("Ejection Stage: ");
	  if (ejection_state_flag==0){
		  print("Pad\r\n");
	  }
	  else if(ejection_state_flag==1){
		  print("Boost\r\n");
	  }
	  else if(ejection_state_flag==2){
		  print("Drogue descent\r\n");
	  }
	  else if(ejection_state_flag==3){
		  print("Main descent\r\n");
	  }
	  else if(ejection_state_flag==4){
		  print("Landed\r\n");
	  }
}






//**************************************************//
//PRIVATE FUNCTIONS

void MRT_Reinitialize_Peripherals(void){
	  /*
	   * Reinitialize all peripherals
	   */

	  print("Reinitializing Peripherals...");

	  // reset LEDs
	  HAL_GPIO_WritePin(OUT_LED1_GPIO_Port, OUT_LED1_Pin, RESET);
	  HAL_GPIO_WritePin(OUT_LED2_GPIO_Port, OUT_LED2_Pin, RESET);
	  HAL_GPIO_WritePin(OUT_LED3_GPIO_Port, OUT_LED3_Pin, RESET);

	  // reset recovery pyro pins
	  HAL_GPIO_WritePin(OUT_EJ_Arming_GPIO_Port, OUT_EJ_Arming_Pin, SET); //PG14 ARMING RCOV
	  HAL_GPIO_WritePin(OUT_EJ_Drogue_Gate_GPIO_Port, OUT_EJ_Drogue_Gate_Pin, RESET); //PG12 DROGUE GATE
	  HAL_GPIO_WritePin(OUT_EJ_Main_Gate_GPIO_Port, OUT_EJ_Main_Gate_Pin, RESET); //PG11 MAIN GATE

	  // reset prop pyro pins
	  HAL_GPIO_WritePin(OUT_PyroValve_Arming_GPIO_Port, OUT_PyroValve_Arming_Pin, SET); //PG1 ARMING_PROP
	  HAL_GPIO_WritePin(OUT_PyroValve_Gate_1_GPIO_Port, OUT_PyroValve_Gate_1_Pin, RESET); //PF15 PROP GATE 1
	  HAL_GPIO_WritePin(OUT_PyroValve_Gate_2_GPIO_Port,OUT_PyroValve_Gate_2_Pin, RESET); //PF14 PROP GATE 2

	  // reset 12 V buck converter enable pin (disable converter)
	  HAL_GPIO_WritePin(EN_12V_Buck_GPIO_Port, EN_12V_Buck_Pin, RESET); //PE2 Buck converter enable

	  // TODO Couldn't find the pin of the vent gate enable
	  //HAL_GPIO_WritePin(Vent_Valve_EN_GPIO_Port, Vent_Valve_EN_Pin, RESET); //This was in the previous code
	  //HAL_GPIO_WritePin(OUT_Prop_ActuatedVent_Gate_GPIO_Port, OUT_Prop_ActuatedVent_Gate_Pin, RESET); //PE7 (MAY NOT BE THE RIGHT ONE)


	  // reset payload EN signal
	  HAL_GPIO_WritePin(PAYLOAD_I2C_EN_GPIO_Port, PAYLOAD_I2C_EN_Pin, RESET); //PE9 Payload I2C enable

	  // set CS pin for thermocouple chip high
	  //	HAL_GPIO_WritePin(TH_CS_1_GPIO_Port, TH_CS_1_Pin, SET);

	  // set power off for VR
	  HAL_GPIO_WritePin(OUT_VR_PWR_GPIO_Port, OUT_VR_PWR_Pin, RESET); //PG9
	  HAL_GPIO_WritePin(OUT_VR_REC_GPIO_Port, OUT_VR_REC_Pin, RESET); //PD7

	  // FLASH set CS, WP and IO3 pins high
	  HAL_GPIO_WritePin(OUT_FLASH_CS_GPIO_Port, OUT_FLASH_CS_Pin, SET);
	  HAL_GPIO_WritePin(OUT_FLASH_WP_GPIO_Port, OUT_FLASH_WP_Pin, SET);
	  HAL_GPIO_WritePin(OUT_FLASH_IO3_GPIO_Port, OUT_FLASH_IO3_Pin, SET);

	  print("OK\r\n");
}




