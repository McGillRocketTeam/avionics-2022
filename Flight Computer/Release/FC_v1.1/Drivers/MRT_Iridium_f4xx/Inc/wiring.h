#ifndef _wiring_H_
#define _wiring_H_

#include <stdlib.h>
#include "memory.h"

/*
#include "wiring_types.h"
#include "boards.h"
#include "io.h"
#include "bits.h"
#include "pwm.h"
#include "interrupts.h"
#include "wiring_debug.h"
#include "WMath.h"

#include "wiring_time.h"
#include <wiring_constants.h>
#include "SPI.h"
#include "HardwareSerial.h"
#include "HardwareTimer.h"
#include "usb_serial.h"
#include "Wire.h"
*/

/* Arduino wiring macros and bit defines  */
#define HIGH 0x1
#define LOW  0x0

#define true 0x1
#define false 0x0

#define lowByte(w)                     ((w) & 0xFF)
#define highByte(w)                    (((w) >> 8) & 0xFF)
#define bitRead(value, bit)            (((value) >> (bit)) & 0x01)
#define bitSet(value, bit)             ((value) |= (1UL << (bit)))
#define bitClear(value, bit)           ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : \
                                                   bitClear(value, bit))
#define bit(b)                         (1UL << (b))

typedef uint8 boolean;
typedef uint8 byte;

#endif
