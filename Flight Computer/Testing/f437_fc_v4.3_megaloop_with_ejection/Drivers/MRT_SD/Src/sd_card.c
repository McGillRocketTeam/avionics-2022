/*
 * sd_card.c
 *
 *  Created on: Dec 22, 2021
 *      Author: jasper
 */

#include "sd_card.h"
#include "w25qxx.h" // for FLASH

FATFS FatFs;
FIL fil;
FRESULT fres;

const char sd_file_header[] = "S,ACCx,ACCy,ACCz,GYRx,GYRy,GYRz,PRESSURE,LAT,LONG,MIN,SEC,SUBSEC,STATE,CONT,E\r\n"; // printed to top of SD card file

// private functions

uint8_t extract_filename_suffix(char* filename, uint8_t len_prefix, uint32_t* num_value);
void str2upper(char* string, char* upper);

/*
 * initialize file system.
 * mount drive and save header text to file.
 *
 * if file already exists, will append to file. otherwise, will create new file.
 */
FRESULT sd_init_append_file(char *filename, char *header_text)
{
	FRESULT fres = f_mount(&FatFs, "", 1);
	if (fres != FR_OK) {
		Error_Handler();
		return fres;
	}

	// write start to SD card
	fres = f_open(&fil, filename, FA_WRITE | FA_OPEN_ALWAYS);
	if (fres != FR_OK) {
		Error_Handler();
		return fres;
	}

	// set pointer to end of file to append
	f_lseek(&fil, f_size(&fil));

	// save indicate start of new log session
	uint8_t msg_buffer[50];
	sprintf((char *)msg_buffer, "--- new logging session! ---\r\n");
	sd_write(&fil, msg_buffer);

	// save header row to indicate what the data is
	sd_write(&fil, (uint8_t *)header_text);
	f_close(&fil);

	return fres;
}

/*
 * initialize file system.
 * mount drive and save header text to file.
 *
 * dynamically searches through existing files and
 * creates new file of form "[prefix][number].txt"
 * where the string composed of "[prefix][number]" is 8 characters long.
 *
 */
FRESULT sd_init_dynamic_filename(char *prefix, char *header_text, char* return_filename)
{
	FRESULT fres = f_mount(&FatFs, "", 1);
	if (fres != FR_OK) {
		Error_Handler();
		return fres;
	}

	// scan files on drive to figure out what suffix number is appropriate
	uint32_t max_used_value = 0;
	fres = scan_files("", prefix, &max_used_value);

	// create filename (max filename length in char array is 13 without LFN)
	char filename[13];
	sprintf(filename, "%s%06lu.txt", prefix, max_used_value + 1);
	return_filename = strcpy(return_filename, filename); // needed so that other functions can open the file!

	// open file (create file) on SD card
	fres = f_open(&fil, filename, FA_WRITE | FA_OPEN_ALWAYS | FA_CREATE_ALWAYS);

	if (fres != FR_OK) {
		Error_Handler();
		return fres;
	}
	// set pointer to end of file to append
	f_lseek(&fil, f_size(&fil));

	// save indicate start of new log session
	uint8_t msg_buffer[50];
	sprintf((char *)msg_buffer, "--- new logging session! ---\r\n");
	sd_write(&fil, msg_buffer);

	// save header row to indicate what the data is
	sd_write(&fil, (uint8_t *)header_text);
	f_close(&fil);

	if (fres != FR_OK) {
		Error_Handler();
	}

	return fres;
}

/*
 * always open in mode FA_WRITE | FA_OPEN_ALWAYS and then appends.
 */
FRESULT sd_open_file(char *filename)
{
	FRESULT fres = f_open(&fil, filename, FA_WRITE | FA_OPEN_ALWAYS);
	f_lseek(&fil, f_size(&fil)); // set pointer to end of file to append
	return fres;
}

/*
 * @brief  write buffer to file on sd card.
 * @param  fp 		file to save to
 * @param  buffer	data to write to file
 */
int8_t sd_write(FIL* fp, uint8_t* buffer)
{
	UINT bytesWrote;
	FRESULT fres = f_write(fp, buffer, strlen((char const *)buffer), &bytesWrote);
	if (fres == FR_OK) {
		return bytesWrote;
	} else {
		return -1;
	}
}

/**
 * sample code from http://elm-chan.org/fsw/ff/doc/readdir.html
 * scans through all the files on the sd card.
 * searches through files with prefix as specified and finds the
 * maximum appended suffix, e.g. of the form "FC000005.txt".
 * if 000005 is the highest, then will return 5 in the max_used_value argument.
 *
 * note: does not search through nested directories.
 */
FRESULT scan_files (
    char* path,        /* Start node to be scanned (***also used as work area***) */
	char* prefix,	   /* prefix in the filename for our datafiles */
	uint32_t* max_used_value
)
{
    FRESULT res;
    DIR dir;
    static FILINFO fno;

    // does not change so make it static
    uint8_t len_prefix = strlen(prefix);
    char prefix_upper[len_prefix];
	str2upper(prefix, prefix_upper);

	*max_used_value = 0; 		// initialize to known minimum value
	uint32_t num_files_fc = 0; 	// suffix on the files containing fc data already on sd card

    res = f_opendir(&dir, path);                       /* Open the directory */
    if (res == FR_OK) {
        for (;;) {
            res = f_readdir(&dir, &fno);                   /* Read a directory item */
            if (res != FR_OK || fno.fname[0] == 0) break;  /* Break on error or end of dir */
            if (fno.fattrib & AM_DIR) {                    /* It is a directory */
            	continue; // don't enter directory
            } else {                                       /* It is a file. */
            	// check if filename contains parts of our standard prefix "FC000000.txt"
            	// but first convert to uppercase to make case insensitive

            	char fname_upper[strlen((char *)fno.fname)];
            	str2upper((char *)fno.fname, fname_upper);

            	int8_t contains_prefix = strncmp(fno.fname, prefix_upper, len_prefix);

            	if (contains_prefix == 0)
            	{
            		// can do error checking with status if desired
            		uint8_t status = extract_filename_suffix(fname_upper, len_prefix, &num_files_fc);

            		if (num_files_fc > *max_used_value)
            		{
            			*max_used_value = num_files_fc;
            		}
            	}
            }
        }

        f_closedir(&dir);
    }

    return res;
}

/**
 * extracts the text after the specified prefix but before the filename extension,
 * which is assumed to be .txt. assumes that filename contains prefix.
 *
 * returns integer indicating success/fail: 0 = success, 1 = fail
 */
uint8_t extract_filename_suffix(char* filename, uint8_t len_prefix, uint32_t* num_value)
{
	uint8_t len_filename = strlen(filename);

	// add characters between prefix and filename extension to buffer
	uint8_t len_buf = 8;
	char buf[len_buf]; // filenames can't be longer than 8 characters total
	for (uint8_t i = 0; i < len_buf; i++)
	{
		if (len_prefix - 1 + i < len_filename - 1) // go to end of filename
		{
			buf[i] = filename[len_prefix + i];
		}
		else break;
	}

	// change chars to integer, strtol will strip out the .txt
	char *ptr;
	*num_value = strtol(buf, &ptr, 10);

	if (ptr == buf || *num_value == LONG_MIN || *num_value == LONG_MAX)
	{
		return 1;
	}

	return 0;
}


/**
 * reads data out of external FLASH and saves to SD card.
 * erases the used flash after finished.
 * returns:
 * 		-1 : failed to save to SD
 * 		 0 : no data saved
 * 		 n : some positive number, number of pages of FLASH saved
 *
 * assumes f_mount has already been run.
 * this function does not close the file system.
 * opens a file "datalog.txt" and closes it when finished.
 */
int8_t save_flash_to_sd(void) {
	// FLASH variables
	uint32_t page_num = 0;
	uint16_t page_bytes = w25qxx.PageSize; // 256 bytes saved per page
	uint8_t readBuf[page_bytes];

	// write to file
	fres = f_open(&fil, "flashlog.txt", FA_WRITE | FA_OPEN_ALWAYS);
	if (fres != FR_OK) {
		return -1;
	}

	// set pointer to end of file
	f_lseek(&fil, f_size(&fil));

	// print string to indicate new log session
	uint8_t msg_buffer[50];
	sprintf((char *)msg_buffer, "\n--- new logging session! ---\r\n");
	sd_write(&fil, msg_buffer);

	for (page_num = 0; page_num < w25qxx.PageCount; page_num++) {

		if (!W25qxx_IsEmptyPage(page_num, 0, page_bytes)) {
			// page not empty, read page out of flash
			W25qxx_ReadPage(readBuf, page_num, 0, page_bytes);

			// save to SD
			int8_t status = sd_write(&fil, readBuf);
			if (status <= 0) {
				return -1; // failed
			}
		}
		else break; // page empty, no need to continue
	}

	// close file
	f_close(&fil);

	if (page_num == 0) { // nothing saved
		return 0;
	}
	else {
		// clear the blocks with data
		uint32_t blocks_to_clear = W25qxx_PageToBlock(page_num);
		for (uint32_t block = 0; block <= blocks_to_clear; block++) {
			W25qxx_EraseBlock(block);
		}
	}

	return page_num;
}


/**
 * assumes that upper has enough characters in the array
 * to store the uppercase version.
 */
void str2upper(char* string, char* upper)
{
	for (uint8_t i = 0; i < strlen(string); i++) {
		upper[i] = toupper(string[i]);
	}
}
