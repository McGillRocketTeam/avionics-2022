/**
 *  @file types.h
 *
 *  @brief libmaple types
 */

#ifndef _TYPES_H_
#define _TYPES_H_

typedef unsigned char uint8;
typedef unsigned short uint16;
typedef unsigned int uint32;
typedef unsigned long long uint64;

typedef signed char int8;
typedef short int16;
typedef int int32;
typedef long long int64;

typedef void (*voidFuncPtr)(void);

#define __io volatile
#define __attr_flash __attribute__((section (".USER_FLASH")))

#define __always_inline inline __attribute__((always_inline))

#ifndef NULL
#define NULL 0
#endif

#endif
