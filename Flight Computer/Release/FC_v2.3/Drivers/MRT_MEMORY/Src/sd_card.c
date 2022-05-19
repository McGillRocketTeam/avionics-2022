/*
 * sd_card.c
 *
 *  Created on: Dec 22, 2021
 *      Author: JO
 */

#include "MRT_helpers.h"
#include "sd_card.h"
#include "string.h"
#include "main.h"

FATFS FatFs;
FIL fil;
uint8_t msg_buffer[1000];
FRESULT fres; //Result after operations
//char filename[13]; Not needed here
uint8_t writeBuf[1000];
const char sd_file_header[] = "S,ACCx,ACCy,ACCz,GYRx,GYRy,GYRz,PRESSURE,LAT,LONG,MIN,SEC,SUBSEC,STATE,CONT,E\r\n"; // printed to top of SD card file



uint8_t extract_filename_suffix(char* filename, uint8_t len_prefix, uint32_t* num_value);
void str2upper(char* string, char* upper);

void myprintf(const char *fmt, ...) { // currently does nothing, was copied from a tutorial to make the code work
  static char buffer[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);

  //CDC_Transmit_FS((uint8_t *)buffer, strlen(buffer));
  //HAL_UART_Transmit(&DEBUGUART, (uint8_t*)buffer, strlen(buffer), -1);
  print(buffer);
}

/*
 * initialize file system.
 * mount drive and save header text to file.
 *
 * if file already exists, will append to file. otherwise, will create new file.
 */
FRESULT sd_init(char *filename, char *header_text)
{
	FRESULT fres = f_mount(&FatFs, "", 1);
	if (fres != FR_OK) {
		myprintf("f_mount error (%i)\r\n", fres);
		return fres;
	}

	// write start to SD card
	fres = f_open(&fil, filename, FA_WRITE | FA_OPEN_ALWAYS);

	if (fres == FR_OK) {
		myprintf("I was able to open filename.txt for writing\r\n");
	} else {
		myprintf("f_open error (%i)\r\n", fres);
		return fres;
	}
	// set pointer to end of file to append
	f_lseek(&fil, f_size(&fil));

	// save indicate start of new log session
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
		myprintf("f_mount error (%i)\r\n", fres);
		return fres;
	}

	// scan files on drive to figure out what suffix number is appropriate
	uint32_t max_used_value = 0;
	fres = scan_files("", prefix, &max_used_value);

	// create filename (max filename length in char array is 13 without LFN)
	char filename[13];
	sprintf(filename, "%s%06lu.txt", prefix, max_used_value + 1);
	memset(return_filename,0,13); //TODO added by MRT
	return_filename = strcpy(return_filename, filename); // needed so that other functions can open the file!

	// open file (create file) on SD card
	fres = f_open(&fil, filename, FA_WRITE | FA_OPEN_ALWAYS | FA_CREATE_ALWAYS);

	if (fres == FR_OK) {
		myprintf("I was able to open filename.txt for writing\r\n");
	} else {
		myprintf("f_open error (%i)\r\n", fres);
		return fres;
	}
	// set pointer to end of file to append
	f_lseek(&fil, f_size(&fil));

	// save indicate start of new log session
	sprintf((char *)msg_buffer, "--- new logging session! ---\r\n");
	sd_write(&fil, msg_buffer);

	// save header row to indicate what the data is
	sd_write(&fil, (uint8_t *)header_text);
	f_close(&fil);

	return fres;
}

/*
 * always open in mode FA_WRITE | FA_OPEN_ALWAYS and then appends.
 */
FRESULT sd_open_file(char *filename)
{
	// write start to SD card
	FRESULT fres = f_open(&fil, filename, FA_WRITE | FA_OPEN_ALWAYS);

	// set pointer to end of file to append
	f_lseek(&fil, f_size(&fil));

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
		myprintf("Wrote %i bytes to 'write.txt'!\r\n", bytesWrote);
		return bytesWrote;
	} else {
		myprintf("f_write error (%i)\r\n");
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
//    UINT i;
    static FILINFO fno;

    // does not change so make it static
    uint8_t len_prefix = strlen(prefix);
    char prefix_upper[len_prefix];
	str2upper(prefix, prefix_upper);

	*max_used_value = 0; // initialize to known minimum value
	uint32_t num_files_fc = 0; // suffix on the files containing fc data already on sd card

    res = f_opendir(&dir, path);                       /* Open the directory */
    if (res == FR_OK) {
        for (;;) {
            res = f_readdir(&dir, &fno);                   /* Read a directory item */
            if (res != FR_OK || fno.fname[0] == 0) break;  /* Break on error or end of dir */
            if (fno.fattrib & AM_DIR) {                    /* It is a directory */
            	continue; // don't enter directory
//                i = strlen(path);
//                sprintf(&path[i], "/%s", fno.fname);
//                res = scan_files(path, prefix, max_used_value);    /* Enter the directory */
//                if (res != FR_OK) break;
//                path[i] = 0;
            } else {                                       /* It is a file. */
//                printf("%s/%s\n", path, fno.fname);

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
 * assumes that upper has enough characters in the array
 * to store the uppercase version.
 */
void str2upper(char* string, char* upper)
{
	for (uint8_t i = 0; i < strlen(string); i++)
	{
		upper[i] = toupper(string[i]);
	}
}
