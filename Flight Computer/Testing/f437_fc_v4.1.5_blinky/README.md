# F437 FC V4.1.5 BLINKY TESTING

This project contains a bunch of testing code for the FC hardware. Just quick tests for functionality, such as reading sensors, firing ejection, etc.
To run each test, uncomment the appropriate line in `main.c` (the following code starts at line 46):

```
/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//#define TEST_BLINKY 						// pass
//#define TEST_EJECTION 					// pass
//#define TEST_VR 							// pass
//#define TEST_SD_CARD_ALONE 				// pass
//#define TEST_SD_CARD_DYNAMIC_FILE_NAMES	// pass
//#define TEST_GPS_ALONE					// pass but GPS fix inconsistent sometimes
//#define TEST_I2C_SENSORS_ALONE			// pass
//#define TEST_ALL_SENSORS_WITH_SD_CARD		// pass
//#define RECORD_VIDEO_WITH_TEST			// activates video recorder in TEST_ALL_SENSORS_WITH_SD_CARD
//#define OUTPUT_USB_WITH_TEST				// sends string to USB with TEST_ALL_SENSORS_WITH_SD_CARD

//#define TEST_USB_VCP_ALONE				// pass

//#define TEST_VENT_VALVE
//#define TEST_PRESSURE_TRANSDUCER_ADC
//#define TEST_THERMOCOUPLE_ADC

//#define HOLIDAY_LED_BLINKY

/* USER CODE END PD */
```

The description of each test is provided below.

| Test Name | Description |
|-----------|-------------|
| blinky    | blinks the LEDs (LED1, LED2, LED3) on the FC board. |
| ejection  | arms and fires the pyro channels for recovery and propulsion. | 
| VR        | starts the video recorder, waits 15 seconds, then stops the video recorder. | 
| SD card alone | mounts the file system, reads a file, and creates and writes to a different file. | 
| sd card dynamic file names | same test as SD card alone but with dynamic file names. | 
| gps alone | reads the gps receiver (NEO-M8Q) for coordinates. | 
| i2c sensors alone | reads data from the lps22hh (barometer) and ism330dlc/lsm6dsr (acc/gyro). | 
| all sensors with sd card | reads all sensors (gps, barometer, acc/gyro), saves data to sd card. | 
| record video with test | flag to record video with the test all sensors with sd card. | 
| output usb with test | prints the data to be saved to the sd card over usb in the test all sensors with sd card. |
| usb vcp alone | test printing over usb as virtual com port (communication device class (CDC)). | 
| vent valve | currently the code opens/closes the "valve" with pwm. can be changed to simple on/off. | 
| pressure transducer adc | reads the adc for the pressure transducer circuit output. |
| thermocouple adc | reads the MAX31855 chip over spi for thermocouple data. | 
| holiday led blinky | blinks the ejection channels in a christmas tree style for the holidays. |  

## Notes About Tests

### Video Recorder (VR)

The video recorder is a RunCam Eagle 3 and the video is saved to the RunCam DVR board. 
To start the recording, the GPIO pin is pulsed high and low for certain durations.
These durations were determined experimentally and there are no datasheets/references (to Jasper's knowledge)
of how long they should be. Currently, the `HAL_Delay()` function is used. Beware of this for RTOS.

### SD Card Dynamic File Names 

Functions were written in `sd_card.c` to do the jobs of initializing the SD card and writing to files.
There are two different initialization functions (`sd_init(char *filename, char *header_text)` and 
`sd_init_dynamic_filename(char *prefix, char *header_text, char* return_filename)`). These functions
both do the same thing, but in `sd_init()`, a fixed filename is specified. The file is opened and 
data is appended. In the dynamic one, the files on the SD card are checked and a new filename is 
determined based on the specified prefix. After opening the file, the function always writes the 
header text (e.g. `"S,PRESSURE_HPA,TEMP_DEG_C,ACCx,ACCy,ACCz,GYRx,GYRy,GYRz,LAT,LONG,E\r\n"`)
to the file.

The prefix in the code is not hardcoded. It can be passed to the initialization function. The 
function will search through the *main directory* in the SD card and figure out the next appropriate name
(see example below). Note that the function does not search through any folders right now. The new name
will be of the form "[prefix][number].txt" where the string composed of "[prefix][number]" is 8 characters long.
The number will be zero-padded.

For example, suppose we initialize the SD card using the dynamic filename function with a prefix of `FC`.
Assume that the SD card has no files of the form `FC000000.txt`. Then the function will create a file
named `FC000001.txt` and write to it. If we assume instead that there are already existing files named
`FC000001.txt` up to `FC000034.txt`, then the function will create a file named `FC000035.txt`.

The prefix can be changed as desired, as long as the total filename (`FC000000.txt`) is less than 12 characters.
To Jasper's knowledge, this is a limitation of the FatFs library or the FAT file system. If we want longer 
filenames, we need to enable LFN (long file name). 

### GPS Alone

The GPS driver in this project was modified from Thomas Jarvis's driver from 2020-21. The only changes made are shown
below (starts on line 83 of `gps.c`):

```
		// from f303 code
//		__HAL_UART_CLEAR_FLAG(GPS_USART, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_PEF | UART_CLEAR_FEF);

		// f437 usart doesnt have these flags in hardware, use software to clear the flags
		// (check docstring for __HAL_UART_CLEAR_FLAG function)
		__HAL_UART_CLEAR_OREFLAG(GPS_USART);
		__HAL_UART_CLEAR_NEFLAG(GPS_USART);
		__HAL_UART_CLEAR_PEFLAG(GPS_USART);
		__HAL_UART_CLEAR_FEFLAG(GPS_USART);
```

The F4xx HAL library does not have the `__HAL_UART_CLEAR_FLAG` function with the options that Thomas used; instead, 
it has dedicated functions to clear those flags.

Note that this driver seems buggy; sometimes, it is very quick to acquire GPS fix. Other times, when I am 100% certain
that there are valid GPS coordinates, the GPS data received by the FC remains as `latitude = 0` and `longitude = 0`.

Currently, the test is setup so that when non-zero latitude *or* longitude data is parsed by the driver, the buzzer
on the FC will beep 4 times; that means, if both latitude and longitude become nonzero, the buzzer will beep 8 times.
LED2 and LED3 will also turn on. If latitude or longitude go back to being zero, then the buzzer will beep twice, 
and LED2 and LED3 will turn off.

### I2C Sensors

The drivers for the I2C sensors were taken from the [STMems Standard C Drivers](https://github.com/STMicroelectronics/STMems_Standard_C_drivers)
and from Jennie's code in the `sensor_functions.c` file from the [FC_V2](https://github.com/McGillRocketTeam/avionics-2021/tree/FlightComputer/Flight%20Computer/Flight%20Computer/FC_V2)
code from 2020-21.


### USB Virtual Com Port

This [tutorial document](https://docs.google.com/document/d/1aUiiYd3HXWhsFOPKkIpWHqmyCKSu7RQLqHodWB1qfmQ/edit#) was extremely helpful.
For the most part, follow the instructions. However, since the FC is self-powered, we do a few extra things:

* In Connectivity --> USB_OTG_FS, mode is Device_Only, and *enable* `Activate_VBUS`
* In Middleware --> USB_Device, in the Class for FS IP dropdown, choose `Communication Device Class (Virtual Com Port)`
  * Under Basic Parameters, make sure that USBD_SELF_POWERED is enabled

If all is done correctly, you should be able to plug in the micro-USB connector on the FC to your computer and see 
stuff printed over a serial monitor.


### Vent Valve

Simple on/off control for propulsion. Just need to check that we can turn it on and read the status of the on/off properly.

### Pressure Transducer ADC

Currently the `ioc` configuration is set to give the ADC the maximum possible conversion time. We should test this to 
determine how much time it actually needs for an accurate reading.

### Thermocouple ADC

The [driver](https://github.com/Bardia-Afshar/MAX31855-MAX6675-STM32) for the MAX31855 chip has a few variables that need to 
be configured in the `MAX31855.h` file for the name of the GPIO used as the chip select pin. Note that you may also 
need to change the `SPI_HandleTypeDef` on line 8 of `MAX31855.c` to `hspi4`.

There is a bunch of commented code in `MAX31855.c` which is from the original driver. I copy-pasted the code from the
Adafruit Arduino driver for their MAX31855 breakout (C++ driver). 

### Holiday LED Blinky

Connect LEDs to the pyro channels, make sure there are no short circuits anywhere, and run the code. Enjoy!



