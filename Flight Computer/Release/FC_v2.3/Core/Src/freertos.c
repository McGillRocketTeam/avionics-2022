/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <iwdg.h>
#include <rtc.h>
#include "stm32f4xx_it.h" //Flag A and B for alarms A and B and reset from start function


#include <MRT_setup.h>
#include <MRT_helpers.h>
#include <MRT_ejection.h>
#include <MRT_propulsion.h>
#include <MRT_telemetry.h>
#include <MRT_memory.h>
#include <MRT_i2c_sensors.h>
#include <MRT_iridium.h>
#include <MRT_payload.h>
#include <video_recorder.h>

#include <sx126x.h> //sx126x_cfg_rx_boosted(&SRADIO_SPI,bool);



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
/* USER CODE BEGIN Variables */

float altitude_m = 0; //Altitude of the rocket in meters
uint8_t wd_ejection_flag = 0; //Flag for the watch dog. When set to 1, the watch dog thread save the ejection stage in the external flash
osThreadId_t threadsID[NUMBER_OF_THREADS]; //Thread list accessed by Watch Dog thread

/* USER CODE END Variables */
/* Definitions for Memory0 */
osThreadId_t Memory0Handle;
const osThreadAttr_t Memory0_attributes = {
  .name = "Memory0",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for Ejection1 */
osThreadId_t Ejection1Handle;
const osThreadAttr_t Ejection1_attributes = {
  .name = "Ejection1",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for Telemetry2 */
osThreadId_t Telemetry2Handle;
const osThreadAttr_t Telemetry2_attributes = {
  .name = "Telemetry2",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for Sensors3 */
osThreadId_t Sensors3Handle;
const osThreadAttr_t Sensors3_attributes = {
  .name = "Sensors3",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for WatchDog */
osThreadId_t WatchDogHandle;
const osThreadAttr_t WatchDog_attributes = {
  .name = "WatchDog",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for Propulsion4 */
osThreadId_t Propulsion4Handle;
const osThreadAttr_t Propulsion4_attributes = {
  .name = "Propulsion4",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

void MRT_waitForLaunch(void);
void MRT_checkThreadStates(void);

/* USER CODE END FunctionPrototypes */

void StartMemory0(void *argument);
void StartEjection1(void *argument);
void StartTelemetry2(void *argument);
void StartSensors3(void *argument);
void StartWatchDog(void *argument);
void StartPropulsion4(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationDaemonTaskStartupHook(void);

/* USER CODE BEGIN DAEMON_TASK_STARTUP_HOOK */
void vApplicationDaemonTaskStartupHook(void)
{
	buzz_fc_on();

	MRT_Init();

	println("\r\n\r\n/****Starting FC****/\r\n\r\n");
	HAL_IWDG_Refresh(&hiwdg);
	buzz_startup_success();

	MRT_waitForLaunch();

	print("\tCreating the threads...");

	/* Create the thread(s) */
	/* creation of Sensors3 */
	Sensors3Handle = osThreadNew(StartSensors3, NULL, &Sensors3_attributes);

	/* creation of Memory0 */
	Memory0Handle = osThreadNew(StartMemory0, NULL, &Memory0_attributes);

	/* creation of Ejection1 */
	Ejection1Handle = osThreadNew(StartEjection1, NULL, &Ejection1_attributes);

	/* creation of Telemetry2 */
	Telemetry2Handle = osThreadNew(StartTelemetry2, NULL, &Telemetry2_attributes);

	/* creation of Propulsion4 */
	Propulsion4Handle = osThreadNew(StartPropulsion4, NULL, &Propulsion4_attributes);

	/* creation of WatchDog */
	WatchDogHandle = osThreadNew(StartWatchDog, NULL, &WatchDog_attributes);

	println("OK");
}
/* USER CODE END DAEMON_TASK_STARTUP_HOOK */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

	println("\r\nFREERTOS Init");
	print("\tInitializing the kernel...");
	osKernelInitialize();
	println("OK");

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
	//TODO No thread creation here (check vApplicationDaemonTaskStartupHook)
#if NO_THREAD_CREATION_HERE
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Memory0 */
  Memory0Handle = osThreadNew(StartMemory0, NULL, &Memory0_attributes);

  /* creation of Ejection1 */
  Ejection1Handle = osThreadNew(StartEjection1, NULL, &Ejection1_attributes);

  /* creation of Telemetry2 */
  Telemetry2Handle = osThreadNew(StartTelemetry2, NULL, &Telemetry2_attributes);

  /* creation of Sensors3 */
  Sensors3Handle = osThreadNew(StartSensors3, NULL, &Sensors3_attributes);

  /* creation of WatchDog */
  WatchDogHandle = osThreadNew(StartWatchDog, NULL, &WatchDog_attributes);

  /* creation of Propulsion4 */
  Propulsion4Handle = osThreadNew(StartPropulsion4, NULL, &Propulsion4_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
#endif
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartMemory0 */
/**
  * @brief  Function implementing the Memory0 thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartMemory0 */
void StartMemory0(void *argument)
{
  /* USER CODE BEGIN StartMemory0 */

	//Add thread id to the list
	threadsID[0]=osThreadGetId();

	#if !MEMORY_THREAD
    osThreadExit();
	#endif

    fres = sd_open_file(filename);

    //uint8_t reset_counter = 0;
    uint8_t sync_counter = 0;

  /* Infinite loop */
  for(;;)
  {
	 //Get RTC time
	 HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	 HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

	 //Update global variables
	 prev_hour = sTime.Hours;
	 prev_min = sTime.Minutes;
	 prev_sec = sTime.Seconds;
	 if (__HAL_RTC_SHIFT_GET_FLAG(&hrtc, RTC_FLAG_SHPF)) prev_sec++; //Adjust following the user manual
	 prev_subsec = sTime.SubSeconds;

	// Save to SD card
	#if SD_CARD_
	sync_counter++;
	MRT_formatAvionics();

	if (sd_write(&fil,(uint8_t*) msg_buffer_av)<0){
		println("SD card error, closing and opening");
		f_close(&fil);
		osDelay(100);
		fres = sd_open_file(filename);
	}

	if (ejection_stage_flag < MAIN_DESCENT){
		MRT_formatPropulsion();
		if (sd_write(&fil,(uint8_t*) msg_buffer_pr)<0){
			println("SD card error, closing and opening");
			f_close(&fil);
			osDelay(100);
			fres = sd_open_file(filename);
		}
	}

	if (sync_counter == 50) {
		sync_counter=0;
		f_sync(&fil);
	}
	#endif

	if (ejection_stage_flag >= LANDED){
	  osDelay(1000/POST_LANDED_DATA_FREQ);
	}
	else{
		osDelay(1000/DATA_FREQ);
	}
  }

  //In case it leaves the infinite loop
  println((char*) "Something went wrong with thread 0");
  osThreadExit();

  /* USER CODE END StartMemory0 */
}

/* USER CODE BEGIN Header_StartEjection1 */
/**
* @brief Function implementing the Ejection1 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartEjection1 */
void StartEjection1(void *argument)
{
  /* USER CODE BEGIN StartEjection1 */

	//Add thread id to the list
	threadsID[1]=osThreadGetId();

	#if !EJECTION_THREAD
	osThreadExit();
	#endif

	//Double check the state TODO bad? (say wakeup flag is raised but ground isn't reached yet
	if (ejection_stage_flag >= LANDED)  osThreadExit(); //Ground reached

	uint8_t lsl_counter = 0;
	uint8_t acc_counter = 0;

  /* Infinite loop */
  for(;;)
  {
	  //Check acceleration
	  if (MRT_getAccNorm() < ACC_LIMIT){
		  acc_counter++;
	  }

	  //Update true apogee
	  if (altitude_m > rtc_bckp_reg_alt_true_apogee){
		  rtc_bckp_reg_alt_true_apogee = altitude_m;
		  rtc_bckp_reg_true_apogee_time = 100*prev_min + prev_sec;
	  }
	  else{
		  #if TESTING_EJECTION
		  acc_counter = ACC_COUNTER_THRESH + 1;
		  #endif

		  //Check if we are going down and if almost no acceleration (close to apogee)
		  if(LSLinRegression() < LSL_SLOPE_LIMIT && acc_counter > ACC_COUNTER_THRESH){
			  lsl_counter++;
		  }
		  else{
			  //lsl_counter = 0;
		  }

		  if (lsl_counter >= LSL_COUNTER_THRESHOLD || ejection_stage_flag >= DROGUE_DESCENT){

			  if (ejection_stage_flag < DROGUE_DESCENT){

				  //TODO update value to be saved in rtc bckp registers
				  rtc_bckp_reg_alt_apogee = altitude_m;
				  rtc_bckp_reg_apogee_time = 100*prev_min + prev_sec;

				  //Update state (save the state in WatchDog thread)
				  ejection_stage_flag = DROGUE_DESCENT;
				  apogee_flag = 1; //Apogee reached //TODO is it where we change it???
				  wd_ejection_flag = 1; //Raise the flag

				  println("Eject Drogue");
			  }

			  //Put outside of "if" such that we only need to remember the ejection stage instead if it + arming state
			  //TODO should I put a while loop, a foor loop or just "one time functions"?
			  while(!HAL_GPIO_ReadPin(OUT_EJ_Arming_GPIO_Port, OUT_EJ_Arming_Pin)){
				  HAL_GPIO_WritePin(OUT_EJ_Arming_GPIO_Port, OUT_EJ_Arming_Pin, SET); //PG14 ARMING RCOV
			  }
			  while(!HAL_GPIO_ReadPin(OUT_EJ_Drogue_Gate_GPIO_Port, OUT_EJ_Drogue_Gate_Pin)){
				  HAL_GPIO_WritePin(OUT_EJ_Drogue_Gate_GPIO_Port, OUT_EJ_Drogue_Gate_Pin, SET); //PG12 DROGUE GATE
			  }
			  while(HAL_GPIO_ReadPin(OUT_EJ_Arming_GPIO_Port, OUT_EJ_Arming_Pin)){
				  HAL_GPIO_WritePin(OUT_EJ_Arming_GPIO_Port, OUT_EJ_Arming_Pin, RESET); //PG14 ARMING RCOV
			  }

			  //Minimum delay between drogue and main
			  osDelay(DROGUE_TO_MAIN_DELAY);

			  for(;;){

				  //We reached main deployment altitude
				  if (altitude_m < MAIN_DEPLOY_ALT || ejection_stage_flag >= MAIN_DESCENT){

					  if (ejection_stage_flag < MAIN_DESCENT){

						  //TODO update value to be saved in rtc bckp registers
						  rtc_bckp_reg_alt_main = altitude_m;
						  rtc_bckp_reg_main_time = 100*prev_min + prev_sec;

						  //Update state (save the state in WatchDog thread)
						  ejection_stage_flag = MAIN_DESCENT;
						  wd_ejection_flag = 1; //Raise the flag

						  println("Eject Main");
					  }

					  //Put outside of "if" such that we only need to remember the ejection stage instead if it + arming state
					  //TODO should I put a while loop, a foor loop or just "one time functions"?
					  while(!HAL_GPIO_ReadPin(OUT_EJ_Arming_GPIO_Port, OUT_EJ_Arming_Pin)){
						  HAL_GPIO_WritePin(OUT_EJ_Arming_GPIO_Port, OUT_EJ_Arming_Pin, SET); //PG14 ARMING RCOV
					  }
					  while(!HAL_GPIO_ReadPin(OUT_EJ_Main_Gate_GPIO_Port, OUT_EJ_Main_Gate_Pin)){
						  HAL_GPIO_WritePin(OUT_EJ_Main_Gate_GPIO_Port, OUT_EJ_Main_Gate_Pin, SET); //PG11 MAIN GATE
					  }
					  while(HAL_GPIO_ReadPin(OUT_EJ_Arming_GPIO_Port, OUT_EJ_Arming_Pin)){
						  HAL_GPIO_WritePin(OUT_EJ_Arming_GPIO_Port, OUT_EJ_Arming_Pin, RESET); //PG14 ARMING RCOV
					  }

					  uint8_t prev_altitude = 0;
					  uint8_t cur_altitude = 0;
					  uint8_t counter = 0;
					  while(counter < 5){

						  cur_altitude = altitude_m;

						  if (MAX(cur_altitude - prev_altitude, cur_altitude - prev_altitude) < LANDING_DIFF_LIMIT){
							  counter++;
						  }
						  else{
							  counter = 0;
						  }
						  prev_altitude = cur_altitude;
						  osDelay(1000/EJECTION_FREQ);
					  }

					  //TODO update value to be saved in rtc bckp registers
					  rtc_bckp_reg_alt_landed = altitude_m;
					  rtc_bckp_reg_landed_time = 100*prev_min + prev_sec;

					  //Update state (saved state in WatchDog thread)
					  ejection_stage_flag = LANDED;
					  wd_ejection_flag = 1;

					  VR_Stop_Rec();
					  VR_Power_Off();

					  println("Ground Level Reached");
					  osThreadExit();

				  }
				  osDelay(1000/EJECTION_FREQ);
			  }
		  }
	  }
	  osDelay(1000/EJECTION_FREQ);
  }

  //In case it leaves the infinite loop
  println((char*) "Something went wrong with thread 1");
  osThreadExit();

  /* USER CODE END StartEjection1 */
}

/* USER CODE BEGIN Header_StartTelemetry2 */
/**
* @brief Function implementing the Telemetry2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTelemetry2 */
void StartTelemetry2(void *argument)
{
  /* USER CODE BEGIN StartTelemetry2 */

	//Add thread id to the list
	threadsID[2]=osThreadGetId();

	#if !TELEMETRY_THREAD
	osThreadExit();
	#endif

	char radio_buffer[RADIO_BUFFER_SIZE];
	uint8_t counter = 0;

  /* Infinite loop */
  for(;;)
  {
	  HAL_GPIO_WritePin(OUT_LED3_GPIO_Port, OUT_LED3_Pin, SET);

	  if (ejection_stage_flag >= LANDED){
		  osDelay(1000/POST_LANDED_SEND_FREQ);
	  }
	  else if (apogee_flag){
		  osDelay(1000/POST_APOGEE_SEND_FREQ);
	  }
	  else{ //Only send prop data pre-apogee

		  //Send propulsion data
		  memset(radio_buffer, 0, RADIO_BUFFER_SIZE);
		  MRT_formatPropulsion();
		  memcpy(radio_buffer, msg_buffer_pr, strlen(msg_buffer_pr));
		  MRT_radio_tx((char*) radio_buffer);

		  osDelay(1000/PRE_APOGEE_SEND_FREQ);
	  }

	  if (counter == SENSORS_SEND_FREQ_DIVIDER){
		  counter = 0;


		  //TODO Need to make this 't' variable from the Iridium or convert the seconds from the GPS
		  /*
		  HOUR = t.tm_hour;
		  MIN = t.tm_min;
		  SEC = t.tm_sec;
		  */

		  //From the GPS time value
		  /*
		  float MIN = ((uint8_t) time % 3600) / 60.0; sprintf(&MIN, "%.0f",MIN);
		  float SEC = (uint8_t) time % 60; sprintf(&SEC,"%.0f",SEC);
		  float SUBSEC = time / 3600.0; sprintf(&SUBSEC,"%.0f",SUBSEC);
		  */

	  	  //Send sensors data
		  memset(radio_buffer, 0, RADIO_BUFFER_SIZE);
		  MRT_formatAvionics();
		  memcpy(radio_buffer, msg_buffer_av, strlen(msg_buffer_av));
		  MRT_radio_tx((char*) radio_buffer);
	  }
	  counter++;


	  HAL_GPIO_WritePin(OUT_LED3_GPIO_Port, OUT_LED3_Pin, RESET);
  }

  //In case it leaves the infinite loop
  println((char*) "Something went wrong with thread 2");
  osThreadExit();

  /* USER CODE END StartTelemetry2 */
}

/* USER CODE BEGIN Header_StartSensors3 */
/**
* @brief Function implementing the Sensors3 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSensors3 */
void StartSensors3(void *argument)
{
  /* USER CODE BEGIN StartSensors3 */

	//Add thread id to the list
	threadsID[3]=osThreadGetId();

	#if !SENSORS_THREAD
	osThreadExit();
	#endif


  /* Infinite loop */
  for(;;)
  {
	  HAL_GPIO_WritePin(OUT_LED1_GPIO_Port, OUT_LED1_Pin, SET);

	  //GPS
	  hgps.pollAll();

	  //LSM6DSR
	  hlsm6dsr.pollAll();

	  //LPS22HH
	  hlps22hh.pollAll();

	  //Update altitude
	  //altitude_m = MRT_getAltitude(hlps22hh.pressure_hPa); //TODO changed
	  //altitude_m = runAltitudeMeasurements(HAL_GetTick(), MRT_getAltitude(hlps22hh.pressure_hPa));
	  altitude_m = runAltitudeMeasurements(xTaskGetTickCount(), MRT_getAltitude(hlps22hh.pressure_hPa)); //TODO RTOS equivalent?


	  //Gates continuity
	  gates_continuity = MRT_getContinuity();

	  HAL_GPIO_WritePin(OUT_LED1_GPIO_Port, OUT_LED1_Pin, RESET);
	  println("POLLED");

	  if (ejection_stage_flag >= LANDED){
		  osDelay(1000/POST_LANDED_POLL_FREQ);
	  }
	  else if (apogee_flag){
		  osDelay(1000/POST_APOGEE_POLL_FREQ);
	  }
	  else{
		  osDelay(1000/PRE_APOGEE_POLL_FREQ);
	  }
  }

  //In case it leaves the infinite loop
  println((char*) "Something went wrong with thread 3");
  osThreadExit();

  /* USER CODE END StartSensors3 */
}

/* USER CODE BEGIN Header_StartWatchDog */
/**
* @brief Function implementing the WatchDog thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartWatchDog */
void StartWatchDog(void *argument)
{
  /* USER CODE BEGIN StartWatchDog */

	#if !WATCHDOG_THREAD
	osThreadExit();
	#endif

	char buffer[WD_BUFFER_SIZE];

  /* Infinite loop */
  for(;;)
  {
	 HAL_GPIO_WritePin(OUT_LED2_GPIO_Port, OUT_LED2_Pin, SET);

	 HAL_IWDG_Refresh(&hiwdg);

	#if HARDFAULT_GENERATOR
	 uint64_t* i = 0x20CDCDCD;
	 *i = 10;
	#endif

	 //Check if new ejection stage to save in memory
	 if(wd_ejection_flag == 1){

		wd_ejection_flag = 0;

		//Update ejection stage flag and save it
		rtc_bckp_reg_ejection_stage = ejection_stage_flag;
		ext_flash_ejection_stage = ejection_stage_flag;
		MRT_saveFlagValue(FC_STATE_FLIGHT);

		//If applicable, update apogee flag
		if (ejection_stage_flag >= DROGUE_DESCENT){
			apogee_flag = 1;
			rtc_bckp_reg_apogee = apogee_flag;
			ext_flash_apogee = apogee_flag;
			MRT_saveFlagValue(FC_STATE_APOGEE);
		}

		//TODO TESTING SAVE EVERY ALTITUDE REGISTERS
		MRT_RTC_setBackupReg(FC_STATE_ALT_PAD, rtc_bckp_reg_alt_pad);
		MRT_RTC_setBackupReg(FC_PAD_TIME, rtc_bckp_reg_pad_time);
		MRT_RTC_setBackupReg(FC_STATE_TRUE_APOGEE, rtc_bckp_reg_alt_true_apogee);
		MRT_RTC_setBackupReg(FC_TRUE_APOGEE_TIME, rtc_bckp_reg_true_apogee_time);
		MRT_RTC_setBackupReg(FC_STATE_ALT_APOGEE, rtc_bckp_reg_alt_apogee);
		MRT_RTC_setBackupReg(FC_APOGEE_TIME, rtc_bckp_reg_apogee_time);
		MRT_RTC_setBackupReg(FC_STATE_ALT_MAIN, rtc_bckp_reg_alt_main);
		MRT_RTC_setBackupReg(FC_MAIN_TIME, rtc_bckp_reg_main_time);
		MRT_RTC_setBackupReg(FC_STATE_ALT_LANDED, rtc_bckp_reg_alt_landed);
		MRT_RTC_setBackupReg(FC_LANDED_TIME, rtc_bckp_reg_landed_time);
	 }

	  //Check if it's sleep time
	  if (flagA==1 || flagB==1){
		//Update iwdg_flag
		iwdg_flag = 1;
		rtc_bckp_reg_iwdg = iwdg_flag;
		ext_flash_iwdg = iwdg_flag;
		MRT_saveFlagValue(FC_STATE_IWDG);

		//Save the RTC time
		MRT_saveTotalTime();

		//Reset to deactivate IWDG
		NVIC_SystemReset();
	  }

	  //Save the RTC time
	  MRT_saveTotalTime();


	  //Check for complete restart
	  if(restart_flag == 1){
		  #if !FLIGHT_MODE
		  MRT_resetFromStart();
		  #endif
	  }

	  MRT_checkThreadStates();

	  HAL_IWDG_Refresh(&hiwdg);

	  HAL_GPIO_WritePin(OUT_LED2_GPIO_Port, OUT_LED2_Pin, RESET);

	  osDelay(1000/WD_FREQ);
  }

  //In case it leaves the infinite loop
  println((char*) "Something went wrong with thread WD");
  osThreadExit();

  /* USER CODE END StartWatchDog */
}

/* USER CODE BEGIN Header_StartPropulsion4 */
/**
* @brief Function implementing the Propulsion4 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartPropulsion4 */
void StartPropulsion4(void *argument)
{
  /* USER CODE BEGIN StartPropulsion4 */

	//Add thread id to the list
	threadsID[4]=osThreadGetId();

	#if !PROPULSION_THREAD
	osThreadExit();
	#endif

	#if !IRIDIUM_
	if (apogee_flag || ejection_stage_flag >= DROGUE_DESCENT){
		osThreadExit();
	}
	#endif

	#if TESTING_IRIDIUM
	apogee_flag = 1;
	#endif

	uint8_t timeout_changed = 0;
	uint16_t payload_counter = 0; //Was sending too fast relative to thread


  /* Infinite loop */
  for(;;)
  {
	  //hiridium.getTime();
	  //hiridium.adjustTimeout(200);
	  //hiridium.sendMessage(iridium_buffer);
	  //osDelay(400000);
	  //if between boost and landing send payload data over iridium
	  if (payload_counter>=PAYLOAD_COUNT*4 && ejection_stage_flag < LANDED){
		  payload_counter = 0;
		  if(payload_init_success){
			#if IRIDIUM_ //Iridium send
			if(MRT_payloadPoll() == 1){
				  HAL_GPIO_WritePin(OUT_LEDF_GPIO_Port, OUT_LEDF_Pin, SET);
				  osDelay(50);
				  print("\tPayload sending: ");
				  println(payload_buffer);
				  //hiridium.getTime(); //TODO doesn't cost anything
				  //hiridium.sendMessage(iridium_buffer); //TODO IT COSTS CREDITS WATCH OUT
				  memset(iridium_buffer+1,0,IRIDIUM_BUFFER_SIZE-2); //Everything but the beginning and ending characters
				  HAL_GPIO_WritePin(OUT_LEDF_GPIO_Port, OUT_LEDF_Pin, RESET);
			}
			#endif
		  }
		  else{
			  MRT_payloadInit();
		  }

	  }
	  if (apogee_flag){

			#if IRIDIUM_ //Iridium send

		    payload_counter = PAYLOAD_COUNT*4;

		  	//Adjust timeout when landing
			if (!timeout_changed && ejection_stage_flag == LANDED) {
			  //Set for the rest of the flight
			  timeout_changed = 1;
			  hiridium.adjustTimeout(IRIDIUM_LANDED_TIMEOUT); //Adjust timeout when landed
			}


			if(MRT_formatIridium() == 1){
			  HAL_GPIO_WritePin(OUT_LEDF_GPIO_Port, OUT_LEDF_Pin, SET);
			  osDelay(50);
			  //TODO make a list of latest coordinates retrieved to optimize the credits we use
			  print("\tIridium sending: ");
			  println(iridium_buffer);
			  //hiridium.getTime(); //TODO doesn't cost anything
			  //hiridium.sendMessage(iridium_buffer); //TODO IT COSTS CREDITS WATCH OUT
			  memset(iridium_buffer+1,0,IRIDIUM_BUFFER_SIZE-2); //Everything but the beginning and ending characters
			  HAL_GPIO_WritePin(OUT_LEDF_GPIO_Port, OUT_LEDF_Pin, RESET);
			}

			if (ejection_stage_flag >= LANDED){
			  osDelay(1000/POST_LANDED_SEND_FREQ);
			}
			else{
			osDelay(IRIDIUM_WAIT_TIME);
			}
			//osDelay(1000/POST_APOGEE_POLL_FREQ);
			#else
			osThreadExit();
			#endif

	  }
	  else{
		  //Poll propulsion sensors
		  MRT_pollPropulsion();
		  payload_counter++;
		  osDelay(1000/PRE_APOGEE_POLL_FREQ);
	  }
  }

  //In case it leaves the infinite loop
  println((char*) "Something went wrong with thread 4");
  osThreadExit();

  /* USER CODE END StartPropulsion4 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

void MRT_waitForLaunch(void){

	println("Waiting for launch command from ground station\r\n");

	char radio_buffer[RADIO_BUFFER_SIZE];
	radio_command cmd = -1;

	//Increase radio gain (more power used)
	//sx126x_cfg_rx_boosted(&SRADIO_SPI,true);

	//Reduce power used by radios (reduce gain)
	//sx126x_cfg_rx_boosted(&SRADIO_SPI,false);


	//Open SD card file
	fres = sd_open_file(filename);
	uint8_t sync_counter = 0;

	#if TESTING_IRIDIUM
	ejection_stage_flag = BOOST;
	#endif

	//Poll propulsion until launch command sent
	while((XTEND_ || SRADIO_) && ejection_stage_flag == PAD){
		HAL_GPIO_WritePin(OUT_LED3_GPIO_Port, OUT_LED3_Pin, SET);

		HAL_IWDG_Refresh(&hiwdg);


		//Start measuring altitude
		hlps22hh.getPressure();
	    //altitude_m = MRT_getAltitude(hlps22hh.pressure_hPa); //TODO changed
	    //altitude_m = runAltitudeMeasurements(HAL_GetTick(), MRT_getAltitude(hlps22hh.pressure_hPa));
	    altitude_m = runAltitudeMeasurements(xTaskGetTickCount(), MRT_getAltitude(hlps22hh.pressure_hPa)); //TODO RTOS equivalent?


		//Get RTC time
		HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
		HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

		//Update global variables
		prev_hour = sTime.Hours;
		prev_min = sTime.Minutes;
		prev_sec = sTime.Seconds;
		if (__HAL_RTC_SHIFT_GET_FLAG(&hrtc, RTC_FLAG_SHPF)) prev_sec++; //Adjust following the user manual
		prev_subsec = sTime.SubSeconds;

		//Save the RTC time
	    MRT_saveTotalTime();

		//Poll propulsion sensors
		MRT_pollPropulsion();

		//Send propulsion data
		MRT_formatPropulsion();
		MRT_radio_tx(msg_buffer_pr);


		// Save to SD card
		#if SD_CARD_
		sync_counter++;
		if (sd_write(&fil,(uint8_t*) msg_buffer_pr) < 0){
			f_close(&fil);
			fres = sd_open_file(filename);
		}
		if (sync_counter==10){
			sync_counter=0;
			f_sync(&fil);
		}
		#endif


		//Check for complete restart
		if(restart_flag == 1){
		  #if !FLIGHT_MODE
		  MRT_resetFromStart();
		  #endif
		}


		//Refresh before using radio
		HAL_IWDG_Refresh(&hiwdg);


		#if HALF_BYTE_
		//Check for command (encoded so 1 byte received instead of 2)
		memset(radio_buffer, 0, RADIO_BUFFER_SIZE);
		MRT_radio_rx(radio_buffer, 1, 0x500); //Timeout is about 1.2 sec (should be less than 5 sec)
		#else
		memset(radio_buffer, 0, RADIO_BUFFER_SIZE);
		MRT_radio_rx(radio_buffer, 2, 0x25); //Timeout is about 0.6 sec (should be less than 5 sec)
		#endif
		cmd = radio_parse_command(radio_buffer);


		if (cmd == LAUNCH){
			//Update ejection stage flag and save it
			ejection_stage_flag = BOOST;
			rtc_bckp_reg_ejection_stage = BOOST;
			ext_flash_ejection_stage = BOOST;
			MRT_saveFlagValue(FC_STATE_FLIGHT);

			//Todo to test ejection
			rtc_bckp_reg_alt_pad = altitude_m;
			MRT_RTC_setBackupReg(FC_STATE_ALT_PAD, rtc_bckp_reg_alt_pad);
			rtc_bckp_reg_pad_time = 100*prev_min + prev_sec;
			MRT_RTC_setBackupReg(FC_PAD_TIME, rtc_bckp_reg_pad_time);
		}


		if (cmd>=1 && cmd<=11){
			execute_parsed_command(cmd);
			for(int i=0;i<3;i++){
				MRT_radio_send_ack(cmd);
			}
		}

		HAL_GPIO_WritePin(OUT_LED3_GPIO_Port, OUT_LED3_Pin, RESET);

		//Reset IWDG timer
		HAL_IWDG_Refresh(&hiwdg);

		osDelay(1000/PRE_APOGEE_SEND_FREQ);
	}

	//Close SD card (reopened by FreeRTOS)
	f_close(&fil);


	//Reduce power used by radios (reduce gain)
	//sx126x_cfg_rx_boosted(&SRADIO_SPI,false);
}


void MRT_checkThreadStates(void){
	  //Check each thread state
	  #if THREAD_KEEPER

	  osThreadState_t thread_state;

	  for (int i=0; i < NUMBER_OF_THREADS;i++){

		  thread_state = osThreadGetState(threadsID[i]);

		  if (thread_state == osThreadInactive ||
			  thread_state == osThreadBlocked){

			  //Ejection thread
			  if (i==1 && ejection_stage_flag < LANDED){
				 osThreadResume(threadsID[i]);
			  }

			  //Propulsion thread
			  #if !IRIDIUM_
			  if (i==4 && (apogee_flag || ejection_stage_flag >= DROGUE_DESCENT)){
				  osThreadTerminate(threadsID[i]);
			  }
			  #endif
			  else {
				 //Resume otherwise
				 osThreadResume(threadsID[i]);
			  }
		  }
		  else if (thread_state == osThreadError){
			  //If it's the propulsion thread
			  if (i==4 && (apogee_flag || ejection_stage_flag >= DROGUE_DESCENT)){
				  osThreadTerminate(threadsID[i]);
			  }
			  else{
				 //Reset otherwise
				 NVIC_SystemReset();
			  }
		  }

		  /*
		  else if (thread_state == osThreadReady){ //For loop with a count to check if it's been too long?
		  }
		  else if (thread_state == osThreadRunning){
		  }
		  else if (thread_state == osThreadReserved){ TODO not sure what is this state
		  }
		  else if (thread_state == osThreadTerminated){not useful
		  }
		  */
	  }
	  #endif
}

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
