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
#include "stm32f4xx_it.h" //Flag A for alarm A

#include <MRT_setup.h>
#include <MRT_helpers.h>
#include <MRT_ejection.h>
#include <MRT_propulsion.h>
#include <MRT_telemetry.h>
#include <MRT_external_flash.h>
#include <MRT_i2c_sensors.h>

#include <sd_card.h>



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
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityHigh3,
};
/* Definitions for Ejection1 */
osThreadId_t Ejection1Handle;
const osThreadAttr_t Ejection1_attributes = {
  .name = "Ejection1",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for Telemetry2 */
osThreadId_t Telemetry2Handle;
const osThreadAttr_t Telemetry2_attributes = {
  .name = "Telemetry2",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityHigh1,
};
/* Definitions for Sensors3 */
osThreadId_t Sensors3Handle;
const osThreadAttr_t Sensors3_attributes = {
  .name = "Sensors3",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
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
  .priority = (osPriority_t) osPriorityHigh1,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

void MRT_checkThreadStates(void);

/* USER CODE END FunctionPrototypes */

void StartMemory0(void *argument);
void StartEjection1(void *argument);
void StartTelemetry2(void *argument);
void StartSensors3(void *argument);
void StartWatchDog(void *argument);
void StartPropulsion4(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

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

	print("\tCreating the threads...");
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
  println("OK");
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

	//TODO UNTESTED

	//Add thread id to the list
	threadsID[0]=osThreadGetId();

	#if !MEMORY_THREAD
    osThreadExit();
	#endif

    uint8_t counter = 0;
    sd_open_file(&filename);

  /* Infinite loop */
  for(;;)
  {
	//Write data to sd and flash
	//TODO sprintf((char*)writeBuf, "Data: %f, %f, %f, %f\r\n", hlps22hh.pressure_hPa, MIN, SEC, SUBSEC);
	if (sd_write(&fil, writeBuf) < 0){
		//TODO when the sd card bug, it seems that only removing power from it works to reset it.
		//Otherwise it always give an error. Trying to close and open doesn't work
		println("\tTEST\tTEST\tTEST\tTEST");
		f_close(&fil);
		sd_open_file(&filename);
		sd_write(&fil, writeBuf);
	}

	if (counter == 50) {
		f_sync(&fil);
		counter = 0;
	}
	counter++;

	osDelay(1000/DATA_FREQ);
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
	if (ejection_state_flag >= LANDED)  osThreadExit(); //Ground reached
	if (wu_flag > 0) osThreadExit(); //WHEN WAKING UP

	osDelay(5000); //Let the LPS "warm up" to have a valid pressure_hPa

  /* Infinite loop */
  for(;;)
  {
	  altitude_m = MRT_getAltitude(hlps22hh.pressure_hPa);

	  if (APOGEE_ALT <= altitude_m || ejection_state_flag >= DROGUE_DESCENT){

		  if (ejection_state_flag < DROGUE_DESCENT){

			  //Update state (save the state in WatchDog thread)
			  ejection_state_flag = DROGUE_DESCENT;
			  apogee_flag = 1; //Apogee reached //TODO is it where we change it???
			  wd_ejection_flag = 1; //Raise the flag

			  println("Eject Drogue");

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
		  }

		  for(;;){

			  altitude_m = MRT_getAltitude(hlps22hh.pressure_hPa);

			  //We reached main deployment altitude
			  if (altitude_m < DEPLOY_ALT || ejection_state_flag >= MAIN_DESCENT){


				  if (ejection_state_flag < MAIN_DESCENT){

					  //Update state (save the state in WatchDog thread)
					  ejection_state_flag = MAIN_DESCENT;
					  wd_ejection_flag = 1; //Raise the flag

					  println("Eject Main");

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
				  }

				  uint8_t prev_altitude = 0;
				  uint8_t cur_altitude = 0;
				  uint8_t counter = 0;
				  while(counter < 5){
					  cur_altitude = MRT_getAltitude(hlps22hh.pressure_hPa);
					  if (cur_altitude - prev_altitude == 0){ //TODO might need a bigger range to account for errors (gotta know what we expect to be our slowest dscent speed)
						  counter++;
					  }
					  else{
						  counter = 0;
					  }
					  prev_altitude = cur_altitude;
					  osDelay(100);
				  }

				  //Update state (saved state in WatchDog thread)
				  ejection_state_flag = LANDED;
				  wd_ejection_flag = 1;

				  println("Ground Level Reached");
				  osThreadExit();

			  }

			  osDelay(10);
		  }
	  }

	  osDelay(10);
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
	uint8_t iridium_counter = 0;

	//TODO TEMP
	float ACC[3];
	float ANG[3];
	float TEMP_LSM;
	float TEMP_LPS;

  /* Infinite loop */
  for(;;)
  {
	  HAL_GPIO_WritePin(OUT_LED3_GPIO_Port, OUT_LED3_Pin, SET);

	  if(apogee_flag == 0){ //Only send prop data pre-apogee

		  //Send propulsion data
		  memset(radio_buffer, 0, RADIO_BUFFER_SIZE);
		  sprintf(radio_buffer, "P,%.2f,%.2f, %i,E", transducer_voltage, thermocouple_temperature, valve_status);
		  MRT_radio_tx((char*) radio_buffer);
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
		  sprintf(radio_buffer, "S,%03.2f,%03.2f,%03.2f,%03.2f,%03.2f,%03.2f,%03.2f,%03.7f,%03.7f,%02d,%02d,%lu,%u,%u,E",
				hlsm6dsr.acceleration_mg[0],	hlsm6dsr.acceleration_mg[1],	hlsm6dsr.acceleration_mg[2],
				hlsm6dsr.angular_rate_mdps[0],	hlsm6dsr.angular_rate_mdps[1],	hlsm6dsr.angular_rate_mdps[2],
				hlps22hh.pressure_hPa,	hgps.latitude,	hgps.longitude,
				prev_min, prev_sec, prev_subsec,
				gates_continuity,	ejection_state_flag);
		  MRT_radio_tx((char*) radio_buffer);



		  if(apogee_flag && iridium_counter == IRIDIUM_SEND_FREQ_DIVIDER){
			  iridium_counter = 0;
			  #if IRIDIUM_ //Iridium send
			  MRT_Static_Iridium_getTime(); //TODO doesn't cost anything
			  //MRT_Static_Iridium_sendMessage(msg); TODO IT COSTS CREDITS WATCH OUT
			  #endif
		  }
		  iridium_counter++;
	  }
	  counter++;


	  HAL_GPIO_WritePin(OUT_LED3_GPIO_Port, OUT_LED3_Pin, RESET);

	  if (apogee_flag){
		  osDelay(1000/POST_APOGEE_SEND_FREQ);
	  }
	  else{
		  osDelay(1000/PRE_APOGEE_SEND_FREQ);
	  }
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

	  //Gates continuity
	  gates_continuity = MRT_getContinuity();

	  HAL_GPIO_WritePin(OUT_LED1_GPIO_Port, OUT_LED1_Pin, RESET);

	  if (apogee_flag){
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

	 //Get RTC time
	 HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	 HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

	 //Update global variables
	 prev_hours = sTime.Hours;
	 prev_min = sTime.Minutes;
	 prev_sec = sTime.Seconds;
	 if (__HAL_RTC_SHIFT_GET_FLAG(&hrtc, RTC_FLAG_SHPF)) prev_sec++;
	 prev_subsec = sTime.SubSeconds;

	 //Save the time
	 MRT_saveRTCTime();

	 //TODO remove for comp
	 memset(buffer, 0, WD_BUFFER_SIZE);
	 sprintf(buffer, "Time: %i:%i:%i ::%lu	Altitude: \r\n %f\r\n", prev_hours,prev_min,prev_sec,prev_subsec , altitude_m);
	 println((char*) buffer);


	 //Check if new ejection stage to save on external flash
	 if(wd_ejection_flag){
		 wd_ejection_flag = 0;
		 flash_flags_buffer[EJECTION_STATE_FLAG_OFFSET] = ejection_state_flag;

		 if (ejection_state_flag == DROGUE_DESCENT){
			 apogee_flag = 1;
			 flash_flags_buffer[APOGEE_FLAG_OFFSET] = apogee_flag;
		 }

		 W25qxx_EraseSector(FLAGS_SECTOR);
		 W25qxx_WriteSector(flash_flags_buffer, FLAGS_SECTOR, FLAGS_OFFSET, NB_OF_FLAGS);
	 }

	  //Check if it's sleep time
	  if (flagA==1){
		//Update iwdg_flag
		iwdg_flag = 1;
		flash_flags_buffer[IWDG_FLAG_OFFSET] = iwdg_flag;
		W25qxx_EraseSector(FLAGS_SECTOR);
		W25qxx_WriteSector(flash_flags_buffer, FLAGS_SECTOR, FLAGS_OFFSET, NB_OF_FLAGS);

		//Reset to deactivate IWDG
		NVIC_SystemReset();
	  }

	  MRT_checkThreadStates();

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

	if (apogee_flag || ejection_state_flag >= DROGUE_DESCENT){
		osThreadExit();
	}

  /* Infinite loop */
  for(;;)
  {
	  //Poll propulsion sensors
	  MRT_pollPropulsion();

	  if (apogee_flag){
		  osThreadExit();
	  }
	  else{
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

//TODO private functions

void MRT_checkThreadStates(void){
	  //Check each thread state
	  #if THREAD_KEEPER

	  osThreadState_t thread_state;

	  for (int i=0; i < NUMBER_OF_THREADS;i++){

		  thread_state = osThreadGetState(threadsID[i]);

		  if (thread_state == osThreadInactive ||
			  thread_state == osThreadBlocked  ||
			  thread_state == osThreadTerminated){

			  //Ejection thread
			  if (i==1 && ejection_state_flag < LANDED){
				 osThreadResume(threadsID[i]);
			  }

			  //Propulsion thread
			  if (i==4 && (apogee_flag || ejection_state_flag >= DROGUE_DESCENT)){
				  osThreadTerminate(threadsID[i]);
				  continue;
			  }
			  else {
				 //Resume otherwise
				 osThreadResume(threadsID[i]);
			  }
		  }

		  else if (thread_state == osThreadError){
			  //If it's the propulsion thread
			  if (i==4 && (apogee_flag || ejection_state_flag >= DROGUE_DESCENT)){
				  osThreadTerminate(threadsID[i]);
				  continue;
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
		  */
	  }
	  #endif
}

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
