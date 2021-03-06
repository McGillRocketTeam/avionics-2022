/*
 * MRT_setup.c
 *
 *  Created on: Apr 12, 2022
 *      Author: Jacoby
 */

#include <MRT_setup.h>
#include <MRT_helpers.h>
#include <main.h>
#include <iwdg.h>
#include <MRT_i2c_sensors.h>
#include <MRT_memory.h>
#include <MRT_telemetry.h>
#include <MRT_propulsion.h>
#include <MRT_ejection.h>
#include <MRT_payload.h>

//**************************************************//
//PRIVATE FUNCTIONS PROTOTYPES

void MRT_Reinitialize_Peripherals(void);
void MRT_reset_info(void);



//**************************************************//
//PUBLIC FUNCTIONS

void MRT_Init(void){
	print((char*) "\r\n\r\n/********MRT Init********/\r\n");

	MRT_Reinitialize_Peripherals();

	//Memory
	MRT_MEMORY_Init();
	MRT_reset_info();

	//IWDG
	#if IWDG_ACTIVE
	print((char*) "IWDG Init...");
	MX_IWDG_Init();
	print((char*) "OK\r\n");
	#endif

	//RTC
	HAL_IWDG_Refresh(&hiwdg);
	MRT_rtc_Init();


	//Sensors
	#if SENSORS_THREAD
		//Scan I2C buses (only for debug mode)
		#if CHECK_I2C && DEBUG
		  checkForI2CDevices(huart8,hi2c1);
		  checkForI2CDevices(huart8,hi2c2);
		  checkForI2CDevices(huart8,hi2c3);
		#endif

		MRT_i2c_sensors_Init();
	#endif


	//Telemetry
	#if TELEMETRY_THREAD
		MRT_TELEMETRY_Init();
	#endif

	//Payload
	#if PROPULSION_THREAD
		MRT_payloadInit();
	#endif


	//TODO DISABLE EXTERNAL BUTTON INTERRUPT ONCE ROCKET IS ARMED (or find other way to completely reset the board)


	#if FORCED_APOGEE
		  apogee_flag = 2; //Flag set to 'forced'. The value of 2 makes it such that it doesn't affect the external flash
	#endif

	#if FORCED_EJECTION_STAGE
		   ejection_stage_flag = FORCED_STAGE;
	#endif
}




void MRT_Deinit(void){
	print((char*) "\r\n\r\n/********MRT Deinit********/\r\n");

	MRT_Reinitialize_Peripherals();
	//MRT_external_flash_Deinit(); TODO


	//IWDG
	//NO DEINIT NEEDED HERE

	//RTC
	//NO DEINIT NEEDED HERE


	#if MEMORY_THREAD

		//SD card
		//TODO deinit? "or last save/close it?"
		// YES THERE IS A FUNCTION FOR THAT
	#endif


	//Sensors
	#if SENSORS_THREAD
		MRT_i2c_sensors_Deinit();
	#endif

    //TODO
}




//**************************************************//
//PRIVATE FUNCTIONS


void MRT_reset_info(void){

	  char buffer[100];
	  sprintf(buffer,"Reset: %i,  WU: %i,  IWDG: %i\r\nPrevious RTC time: %i:%i:%i ::%i\r\n",
			  reset_flag, wu_flag, iwdg_flag, prev_hour, prev_min, prev_sec, prev_subsec);
	  print(buffer);

	  //Check if IWDG is being deactivated
	  if (iwdg_flag==1){
		  print((char*) "Deactivating IWDG\r\n");

		  iwdg_flag = 0; //Flip flag

		  //Write new flag to memory
		  rtc_bckp_reg_iwdg = iwdg_flag;
		  ext_flash_iwdg = iwdg_flag;
		  MRT_saveFlagValue(FC_STATE_IWDG);

		  HAL_Delay(500);

		  //Go to sleep
		  MRT_StandByMode(SLEEP_TIME);
	  }


	  //Check if we are after waking up (and at which wake up we are at)
	  if (wu_flag > 0){
		  char buf[30];
		  sprintf(buf, "FC wake up %i\r\n", wu_flag);
		  print(buf);
	  }


	  //Check if we start from the beginning
	  if (reset_flag == 0){
		  print((char*) "FC restarted\r\n");

		  reset_flag = 1; //Flip flag

		  MRT_saveFlagValue(FC_STATE_RESET);
	  }


	  //Check if before or after apogee
	  if (apogee_flag == 0){
		  print((char*) "Pre-apogee\r\n");
	  }
	  else if(apogee_flag==1){
		  print((char*) "Post-apogee\r\n");
	  }


	  //Check ejection stage
	  print((char*)"Ejection Stage: ");
	  if (ejection_stage_flag==PAD){
		  print((char*)"Pad\r\n");
	  }
	  else if(ejection_stage_flag==BOOST){
		  print((char*)"Boost\r\n");
	  }
	  else if(ejection_stage_flag==DROGUE_DESCENT){
		  print((char*)"Drogue descent\r\n");
	  }
	  else if(ejection_stage_flag==MAIN_DESCENT){
		  print((char*)"Main descent\r\n");
	  }
	  else if(ejection_stage_flag==LANDED){
		  print((char*)"Landed\r\n");
	  }
}


void MRT_Reinitialize_Peripherals(void){
	  /*
	   * Reinitialize all peripherals
	   */

	  print((char*)"Reinitializing Peripherals...");

	  // reset LEDs
	  HAL_GPIO_WritePin(OUT_LED1_GPIO_Port, OUT_LED1_Pin, RESET);
	  HAL_GPIO_WritePin(OUT_LED2_GPIO_Port, OUT_LED2_Pin, RESET);
	  HAL_GPIO_WritePin(OUT_LED3_GPIO_Port, OUT_LED3_Pin, RESET);

	  // reset recovery pyro pins
	  HAL_GPIO_WritePin(OUT_EJ_Arming_GPIO_Port, OUT_EJ_Arming_Pin, RESET); //PG14 ARMING RCOV
	  HAL_GPIO_WritePin(OUT_EJ_Drogue_Gate_GPIO_Port, OUT_EJ_Drogue_Gate_Pin, RESET); //PG12 DROGUE GATE
	  HAL_GPIO_WritePin(OUT_EJ_Main_Gate_GPIO_Port, OUT_EJ_Main_Gate_Pin, RESET); //PG11 MAIN GATE

	  // reset prop pyro pins
	  HAL_GPIO_WritePin(OUT_PyroValve_Arming_GPIO_Port, OUT_PyroValve_Arming_Pin, RESET); //PG1 ARMING_PROP
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

	  print((char*)"OK\r\n");
}




