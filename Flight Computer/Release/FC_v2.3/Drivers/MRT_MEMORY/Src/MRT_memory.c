/*
 * MRT_memory.c
 *
 *  Created on: Apr 29, 2022
 *      Author: Jacoby
 */


#include <MRT_helpers.h>
#include <MRT_setup.h>
#include <MRT_memory.h>
#include <radio_commands.h>
#include <video_recorder.h>
#include <iwdg.h>

//**************************************************//
//VARIABLES AND PROTOTYPES

//Flags
//Flags Variables
uint8_t reset_flag = 0; //if 0 -> start from beginning, if 1 -> random watchdog reset (if 2-> reset after wakeup??)
uint8_t wu_flag = 0; //TODO used to be defined in MRT_RTOS.c
uint8_t iwdg_flag = 0; //if 0 -> normal, if 1 -> try to go into standbymode
uint8_t apogee_flag = 0; //if 0 -> pre-apogee, if 1 -> post-apogee
uint8_t ejection_stage_flag = 0; //if 0-> pad, if 1->boost, if 2->drogue descent, if 3->main descent, if 4-> landed

//Private functions prorotypes
uint8_t MRT_checkFlagsValues(rtc_backup_reg val_index, uint32_t max_val);
void MRT_updateFlagsValues(void);


//RTC time
//Time variables
uint8_t prev_hour = 0; //Last recorded hours
uint8_t prev_min = 0; //Last recorded minutes
uint8_t prev_sec = 0; //Last recorded seconds
uint32_t prev_subsec = 0; //Last recorded subseconds

//Private functions prorotypes
uint8_t MRT_checkTimeValues(rtc_backup_reg val_index, uint32_t max_val);
void MRT_updateTimeValues(void);
void MRT_saveTimeValue(rtc_backup_reg state);


//Misc
// public variables in main to be reset
extern volatile uint32_t flash_write_address;
extern volatile uint8_t state;
extern volatile uint8_t state_arm_rcov;
extern volatile uint8_t state_arm_prop;
extern float alt_ground;
extern float alt_apogee;
extern float alt_prev;

//Private function prototype
void MRT_stateRestoration(void);
void MRT_checkWakeUp(void);



//**************************************************//
//FLAGS

//Public functions

/*
 * Save a flag value (value updated outside the function)
 */
void MRT_saveFlagValue(rtc_backup_reg state){
	//Write new flags to flash memory
	MRT_updateExternalFlashBuffers();
	W25qxx_EraseSector(FLAGS_SECTOR);
	W25qxx_WriteSector(ext_flash_flags_buffer, FLAGS_SECTOR, FLAGS_OFFSET, NB_OF_FLAGS);

	//Write new flag to RTC backup register
	MRT_RTC_setBackupReg(state, *rtc_bckp_regs[state]);
}



//Private functions
/*
 * Update all flag values
 */
void MRT_updateFlagsValues(void){
	//External Flash
	ext_flash_reset = reset_flag;
	ext_flash_wu = wu_flag;
	ext_flash_iwdg = iwdg_flag;
	ext_flash_apogee = apogee_flag;
	ext_flash_ejection_stage = ejection_stage_flag;

	//RTC backup registers
	rtc_bckp_reg_reset = reset_flag;
	rtc_bckp_reg_wu = wu_flag;
	rtc_bckp_reg_iwdg = iwdg_flag;
	rtc_bckp_reg_apogee = apogee_flag;
	rtc_bckp_reg_ejection_stage = ejection_stage_flag;
}


/*
 * Check for memory issues
 */
uint8_t MRT_checkFlagsValues(rtc_backup_reg val_index, uint32_t max_val){

	uint8_t ret = true;

	//Check for a random value (unsigned int so can't be negative)
	if (*rtc_bckp_regs[val_index] > max_val){
		*rtc_bckp_regs[val_index] = 0;
		ret = false;
	}
	if (*ext_flash_flags[val_index] > max_val){
		*ext_flash_flags[val_index] = 0;
		ret = false;
	}

	if (*rtc_bckp_regs[val_index] != *ext_flash_flags[val_index]){
		*rtc_bckp_regs[val_index] = MAX(*rtc_bckp_regs[val_index], *ext_flash_flags[val_index]);
		*ext_flash_flags[val_index] = *rtc_bckp_regs[val_index];
		ret = false;
	}
	return ret;
}

void MRT_resetFlags(void){
	//TODO INCOMPLETE

	/*
	//Clear RTC time (last recorded) in external flash
	W25qxx_EraseSector(RTC_SECTOR);
	W25qxx_WriteSector(RTC_TIME_NULL_BUFFER, RTC_SECTOR, RTC_TIME_OFFSET, RTC_NB_OF_VAR);

	//Clear RTC time in backup registers
	MRT_RTC_setBackupReg(RTC_HOUR, 0);
	MRT_RTC_setBackupReg(RTC_MINUTE, 0);
	MRT_RTC_setBackupReg(RTC_SECOND, 0);
	MRT_RTC_setBackupReg(RTC_SUBSEC, 0);

	//Update variables (to 0)
	for (int i = 0; i < RTC_NB_OF_VAR; i++){
	  *ext_flash_time[i] = 0x0;
	  *rtc_bckp_regs[i + NB_OF_FLAGS] = 0x0;
	}
	*/
}





//**************************************************//
//RTC TIME FLAGS

//Public functions

/*
 * Update every time values in memory
 */
void MRT_saveTotalTime(void){

	//Update variables
	MRT_updateTimeValues();

	//Write new RTC time to external flash
	MRT_updateExternalFlashBuffers();
	W25qxx_EraseSector(RTC_SECTOR);
	W25qxx_WriteSector(ext_flash_time_buffer, RTC_SECTOR, RTC_TIME_OFFSET, RTC_NB_OF_VAR);

	//Write new RTC time to backup registers
	MRT_RTC_setBackupReg(RTC_HOUR, rtc_bckp_reg_hour);
	MRT_RTC_setBackupReg(RTC_MINUTE, rtc_bckp_reg_min);
	MRT_RTC_setBackupReg(RTC_SECOND, rtc_bckp_reg_sec);
	MRT_RTC_setBackupReg(RTC_SUBSEC, rtc_bckp_reg_subsec);
}


void MRT_resetTotalTime(void){

	//Clear RTC time (last recorded) in external flash
	W25qxx_EraseSector(RTC_SECTOR);
	W25qxx_WriteSector(RTC_TIME_NULL_BUFFER, RTC_SECTOR, RTC_TIME_OFFSET, RTC_NB_OF_VAR);

	//Clear RTC time in backup registers
	MRT_RTC_setBackupReg(RTC_HOUR, 0);
	MRT_RTC_setBackupReg(RTC_MINUTE, 0);
	MRT_RTC_setBackupReg(RTC_SECOND, 0);
	MRT_RTC_setBackupReg(RTC_SUBSEC, 0);

	//Update variables (to 0)
	for (int i = 0; i < RTC_NB_OF_VAR; i++){
	  *ext_flash_time[i] = 0x0;
	  *rtc_bckp_regs[i + NB_OF_FLAGS] = 0x0;
	}
}




//Private functions

/*
 * Save an RTC value (value updated outside the function)
 */
void MRT_saveTimeValue(rtc_backup_reg state){

	//Write new RTC time to flash memory
	MRT_updateExternalFlashBuffers();
	W25qxx_EraseSector(RTC_SECTOR);
	W25qxx_WriteSector(ext_flash_time_buffer, RTC_SECTOR, RTC_TIME_OFFSET, RTC_NB_OF_VAR);

	//Write new RTC time to backup registers
	MRT_RTC_setBackupReg(state, *rtc_bckp_regs[state]);
}



/*
 * Update all time values
 */
void MRT_updateTimeValues(void){
	//External Flash
	ext_flash_hour = prev_hour;
	ext_flash_min = prev_min;
	ext_flash_sec = prev_sec;
	ext_flash_subsec = prev_subsec;

	//RTC backup registers
	rtc_bckp_reg_hour = prev_hour;
	rtc_bckp_reg_min = prev_min;
	rtc_bckp_reg_sec = prev_sec;
	rtc_bckp_reg_subsec = prev_subsec;
}


/*
 * Check for memory issues
 */
uint8_t MRT_checkTimeValues(rtc_backup_reg val_index, uint32_t max_val){

	uint8_t ret = true;

	//Check for a random value (unsigned int so can't be negative)
	if (*rtc_bckp_regs[val_index] > max_val){
		*rtc_bckp_regs[val_index] = 0;
		ret = false;
	}
	if (*ext_flash_time[val_index - NB_OF_FLAGS] > max_val){
		*ext_flash_time[val_index - NB_OF_FLAGS] = 0;
		ret = false;
	}

	if (*rtc_bckp_regs[val_index] != *ext_flash_time[val_index - NB_OF_FLAGS]){
		*rtc_bckp_regs[val_index] = MAX(*rtc_bckp_regs[val_index], *ext_flash_time[val_index - NB_OF_FLAGS]);
		*ext_flash_time[val_index - NB_OF_FLAGS] = *rtc_bckp_regs[val_index];
		ret = false;
	}
	return ret;
}





//**************************************************//
//MISC


//Public
void MRT_MEMORY_Init(void){

	//External flash
	MRT_external_flash_Init();

	//Backup registers
	MRT_RTC_backup_regs_Init();

	//Get the previous flags and rtc time from memory
	MRT_stateRestoration();

	//TODO SD card (doesn't work)
	#if MEMORY_THREAD

		//SD card
		#if SD_CARD_
			HAL_IWDG_Refresh(&hiwdg);

			// check if SD card is inserted
			if (HAL_GPIO_ReadPin(IN_SD_CARD_DETECT_GPIO_Port, IN_SD_CARD_DETECT_Pin) == GPIO_PIN_RESET) {
			  // init sd card with dynamic filename
			  fres = sd_init_dynamic_filename("AB", sd_file_header, filename);
			  if (fres != FR_OK) {
					Error_Handler();
			  }
			}
			else {
				println("No SD card inserted");
			  //Error_Handler(); TODO not a good idea if SD card stops working for no reason (will stop the FC)
			}
		#endif
	#endif
}



//Private

//Get the flags values and RTC time
void MRT_stateRestoration(void){

	//Check for a wakeup
	MRT_checkWakeUp();

	//If RTC detected a wake up, update the flash memory and the backup registers
	if (wu_flag == 1){

		//Update values
		ext_flash_wu += 1;
		rtc_bckp_reg_wu += 1;

		//Check for memory issues¸
		MRT_checkFlagsValues(FC_STATE_WU, 2);

		//Update memory
		MRT_saveFlagValue(FC_STATE_WU);
	}
	//Update the wu_flag value
	wu_flag = rtc_bckp_reg_wu;


	//Check flags values

	//Reset flag
	if(MRT_checkFlagsValues(FC_STATE_RESET, 1) == false)	MRT_saveFlagValue(FC_STATE_RESET);
	reset_flag = rtc_bckp_reg_reset;

	//Wake up flag (TODO double check??)
	if(MRT_checkFlagsValues(FC_STATE_WU, 2) == false)	MRT_saveFlagValue(FC_STATE_WU);
	wu_flag = rtc_bckp_reg_wu;

	//IWDG flag
	if(MRT_checkFlagsValues(FC_STATE_IWDG, 1) == false)	MRT_saveFlagValue(FC_STATE_IWDG);
	iwdg_flag = rtc_bckp_reg_iwdg;

	//Apogee flag
	if(MRT_checkFlagsValues(FC_STATE_APOGEE, 1) == false)	MRT_saveFlagValue(FC_STATE_APOGEE);
	apogee_flag = rtc_bckp_reg_apogee;

	//Ejection state flag
	if(MRT_checkFlagsValues(FC_STATE_FLIGHT, 4) == false)	MRT_saveFlagValue(FC_STATE_FLIGHT);
	ejection_stage_flag = rtc_bckp_reg_ejection_stage;


	//Check RTC time values
	//Hours
	if(MRT_checkTimeValues(RTC_HOUR, 23) == false)	MRT_saveTimeValue(RTC_HOUR);
	prev_hour = rtc_bckp_reg_hour;

	//Minutes
	if(MRT_checkTimeValues(RTC_MINUTE, 59) == false)	MRT_saveTimeValue(RTC_MINUTE);
	prev_min = rtc_bckp_reg_min;

	//Seconds
	if(MRT_checkTimeValues(RTC_SECOND, 59) == false)	MRT_saveTimeValue(RTC_SECOND);
	prev_sec = rtc_bckp_reg_sec;

	//Sub-Seconds //Is it that important to know subsec? We lose a register because of it
	if(MRT_checkTimeValues(RTC_SUBSEC, 999) == false)	MRT_saveTimeValue(RTC_SUBSEC);
	prev_subsec = rtc_bckp_reg_subsec;
}


/*
 * Check if FC is back from wakeup
 */
void MRT_checkWakeUp(void){
	//If WU flag set, wake up procedure
	if (__HAL_PWR_GET_FLAG(PWR_FLAG_SB) != RESET)
	{

		wu_flag = 1;

		__HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);  // clear the flag

		println("Wakeup from STANDBY MODE");

		/** Disable the WWAKEUP PIN **/
		HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN1);  // disable PA0

		/** Deactivate the RTC wakeup  **/
		HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
	}

	MRT_clear_alarms_flags();
}













/*
// gets backup register values and performs the appropriate
// restoration action (e.g. arming, altitudes, etc.)
void restore_fc_states(void) {
	// flight state
	state = (uint8_t) (MRT_RTC_getBackupReg(FC_STATE_FLIGHT));

	// video recorder
	if (MRT_RTC_getBackupReg(FC_STATE_VR_POWER)) {
		VR_Power_On();
	} else {
		VR_Power_Off();
	}

	if (MRT_RTC_getBackupReg(FC_STATE_VR_RECORDING)) {
		VR_Start_Rec();
	} else {
		VR_Stop_Rec();
	}

	// altitudes
	alt_ground = (float) (MRT_RTC_getBackupReg(FC_STATE_ALT_GROUND));
	alt_apogee = (float) (MRT_RTC_getBackupReg(FC_STATE_ALT_APOGEE));
	alt_prev = (float) (MRT_RTC_getBackupReg(FC_STATE_ALT_PREV));
}
*/
