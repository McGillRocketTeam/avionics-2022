/*
 * sd_card.h
 *
 *  Created on: Dec 22, 2021
 *      Author: JO
 */

#ifndef INC_SD_CARD_H_
#define INC_SD_CARD_H_

#include "main.h"
#include "fatfs.h"

#include <stdio.h>
#include <string.h>
#include <stdarg.h> //for va_list var arg functions
#include <stdlib.h>
#include <limits.h>
#include <ctype.h> // for toupper()


//Variables
char filename[13];


// functions
void myprintf(const char *fmt, ...);
FRESULT sd_init(char *filename, char *header_text);
FRESULT sd_init_dynamic_filename(char *prefix, char *header_text, char* return_filename);
FRESULT sd_open_file(char *filename);
int8_t sd_write(FIL* fp, uint8_t* buffer);
FRESULT scan_files(char* path, char* prefix, uint32_t* max_used_value);
uint8_t extract_filename_suffix(char* filename, uint8_t len_prefix, uint32_t* num_value);

#endif /* INC_SD_CARD_H_ */
