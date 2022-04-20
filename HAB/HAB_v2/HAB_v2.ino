// libraries
#include <Wire.h>
#include <SPI.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_MS8607.h>
#include <Adafruit_ADXL345_U.h>
#include <RTClib.h>
#include <SD.h>

#include <string.h>
#include "HAB_v2_pins.h"
#include "HAB_v2_config.h"

#ifdef USING_GPS
#include <Adafruit_GPS.h>
#endif

// types
enum error_states {
  ERR_OK = 0,
  ERR_ADXL345_INIT,
  ERR_MS8607_INIT,
  ERR_SD_CARD_INIT,
  ERR_PCF8523_INIT,
  ERR_FT_NO_CONTINUITY,
};

struct ms8607_data {
  float temperature;
  float pressure;
  float humidity;
};

struct adxl345_data {
  float acc_x;
  float acc_y;
  float acc_z;
};

struct pcf8523_data {
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
};


// private function prototypes
void init_pinModes(void);
void init_pinStates(void);
void format_telemetry(void);
void blink_beep(int beeps, long duration);
void blink_beep_success(void);
void blink_beep_failure(void);

void adxl345_init(void);
void ms8607_init(void);
void pcf8523_init(void);
void sd_card_init(void);
void sd_find_dynamic_file_name(char *prefix, char *filename);

void ms8607_poll(struct ms8607_data *data_s);
void adxl345_poll(struct adxl345_data *data_s);
void pcf8523_poll(struct pcf8523_data *data_s);

void ft_init(void);
float read_FT_continuity(int pin);

void Error_Handler(enum error_states err);

// programming variables
float alt_ground = 0;
float alt_agl = 0;
float pressure_ground = 0;
float pressure_current = 0;
uint32_t count = 0;
float alt_previous[ALT_ARRAY_SIZE] = {0};
uint8_t alt_arr_position = 0;
uint8_t launched = 0;

// other global variables
char telemetry_buffer[150] = {0};
const char datafile_header_string = "S,ACCx,ACCy,ACCz,PRESSURE_hPa,TEMPERATURE_C,HUMIDITY_rH,TIME,E";

// objects for sensors
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345); // assign unique ID to sensor
Adafruit_MS8607 ms8607;
RTC_PCF8523 rtc;
File datafile;

// structs for data
struct ms8607_data ms8607_data_struct = {0};
struct adxl345_data adxl345_data_struct = {0};
struct pcf8523_data pcf8523_data_struct = {0};


void setup() {
  Serial.begin(9600);
  Wire.begin();
  
  // init pins
  init_pinModes();
  init_pinStates();
  memset(alt_previous, 0, ALT_ARRAY_SIZE*sizeof(*alt_previous));

  // init sensors
  adxl345_init();
  ms8607_init();
  pcf8523_init();
  sd_card_init();

  // init flight termination logic
  ft_init();

  // indicate successful startup to user
  digitalWrite(LED1, HIGH);
  blink_beep_success();




}

void loop() {
  // put your main code here, to run repeatedly:

}


void init_pinModes(void) {
  pinMode(FT_GATE_1, OUTPUT);
  pinMode(FT_GATE_2, OUTPUT);
  pinMode(FT_GATE_3, OUTPUT);
  pinMode(FT_GATE_4, OUTPUT);

  pinMode(ADXL_INT1, INPUT);
  pinMode(ADXL_INT2, INPUT);

  pinMode(VR_CTRL, OUTPUT);
  pinMode(SD_CS, OUTPUT);
  
  pinMode(BUZZER, OUTPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);

  pinMode(FT_C1, INPUT);
  pinMode(FT_C2, INPUT);
  pinMode(FT_C3, INPUT);
  pinMode(FT_C4, INPUT);
}

void init_pinStates(void) {
  digitalWrite(FT_GATE_1, LOW);
  digitalWrite(FT_GATE_2, LOW);
  digitalWrite(FT_GATE_3, LOW);
  digitalWrite(FT_GATE_4, LOW);

  digitalWrite(SD_CS, HIGH);  // SPI idle is high
  digitalWrite(LED1, HIGH);   // bi-color LED, HIGH = green, LOW = red
  digitalWrite(LED2, LOW);
  digitalWrite(LED3, LOW);

  noTone(BUZZER);
}


void format_telemetry(void) {
//  sprintf(telemetry_buffer, "S, ,E\r\n",
//
//  );
}

// sensor init functions
void adxl345_init(void) {
  if (!accel.begin()) {
    Error_Handler(ERR_ADXL345_INIT);
  }

  accel.setRange(ADXL345_RANGE_8_G);
  Serial.println(F("Sensor init: ADXL345 (accelerometer) ok!"));
}

void ms8607_init(void) {
  if (!ms8607.begin()) {
    Error_Handler(ERR_MS8607_INIT);
  }

  ms8607.setHumidityResolution(MS8607_HUMIDITY_RESOLUTION_OSR_8b);
  ms8607.setPressureResolution(MS8607_PRESSURE_RESOLUTION_OSR_4096);
  Serial.println(F("Sensor init: MS8607 (barometer) ok!"));
}

void sd_card_init(void) {
  if (!SD.begin(SD_CS)) {
    Error_Handler(ERR_SD_CARD_INIT);
  }

  // dynamic file name: inspect SD card contents and automatically create filename
  char filename[13] = {0};
  sd_find_dynamic_file_name("HAB", filename);
  datafile = SD.open(filename, FILE_WRITE);

  // write header to file
  datafile.println(datafile_header_string);
  datafile.flush();
  
  Serial.println(F("Sensor init: SD card ok!"));
}

void pcf8523_init(void) {
  if (!rtc.begin()) {
    Error_Handler(ERR_PCF8523_INIT);
  }

  if (!rtc.initialized()) {
    // set rtc date/time to the date/time that this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  Serial.println(F("Sensor init: PCF8523 (real-time clock) ok!"));
}

void ft_init(void) {
  analogReadResolution(10);

  // read continuity and see if channels are armed
  for (uint8_t i = FT_C1; i < FT_C4; i++) {
    float voltage = read_FT_continuity(i);
    if (voltage < 0.7 * 3.3) {
      Serial.print("Warning: No continuity detected on FT Channel ");
      Serial.println(i - FT_C1 + 1);
    }
  }

  // get ground pressure and average
  float pressure = 0;
  for (uint8_t i = 0; i < ALT_MEAS_AVGING; i++) {
//    ms8607_poll();
//    pressure += ms8607_data.pressure;
  }
  pressure /= ((float) ALT_MEAS_AVGING);
  pressure_ground = pressure;

  Serial.println(F("Flight termination init: ADXL345 ok!"));
}


void sd_find_dynamic_file_name(char *prefix, char *filename) {
  char temp_filename[13];
  for (unsigned int i = 0; ; i++) {
    sprintf(temp_filename, "%s%05du.txt", prefix, i); 
    if (SD.exists(temp_filename)) {
      continue;
    }
    else {
      strcpy(filename, temp_filename);
    }
  }
}

void ms8607_poll(struct ms8607_data *data_s) {
  sensors_event_t temp, pressure, humidity;
  ms8607.getEvent(&pressure, &temp, &humidity);
  
  data_s->temperature = temp.temperature;
  data_s->pressure = pressure.pressure;
  data_s->humidity = humidity.relative_humidity;
}

void adxl345_poll(struct adxl345_data *data_s) {
  sensors_event_t event;
  accel.getEvent(&event);

  data_s->acc_x = event.acceleration.x;
  data_s->acc_y = event.acceleration.y;
  data_s->acc_z = event.acceleration.z;
}

void pcf8523_poll(struct pcf8523_data *data_s) {
  DateTime now = rtc.now();
  data_s->day = now.day();
  data_s->hour = now.hour();
  data_s->minute = now.minute();
  data_s->second = now.second();
}

float read_FT_continuity(int pin) {
  return (analogRead(pin) / 1024.0 * 3.3);
}

// beeps the buzzer for the number of beeps specified,
// for the duration specified. also blinks LED3.
void blink_beep(int beeps, long duration) {

  for (int i = 0 ; i < beeps; i++) { // beep thrice, long
    tone(BUZZER, BUZZER_FREQ);
    digitalWrite(LED3, HIGH);
    delay(duration);
    digitalWrite(LED3, LOW);
    noTone(BUZZER);
    if (beeps > 1) {
      delay(duration);
    }
  }
}

// wrapper function to beep and blink indicating success
void blink_beep_success(void) {
  blink_beep(BEEPS_SUCCESS_NUM, BEEPS_SUCCESS_DUR);
} 

// wrapper function to beep and blink indicating failure
void blink_beep_failure(void) {
  digitalWrite(LED1, LOW); // turns on red LED
  blink_beep(BEEPS_FAILURE_NUM, BEEPS_FAILURE_DUR);
}

void Error_Handler(enum error_states err) {
  digitalWrite(LED1, LOW); 
  switch (err) {
    case ERR_OK:
      return;

    case ERR_ADXL345_INIT:
      Serial.println(F("Error: ADXL345 (accelerometer) initialization problem."));
      break;

    case ERR_MS8607_INIT:
      Serial.println(F("Error: MS8607 (barometer) initialization problem."));
      break;

    case ERR_SD_CARD_INIT:
      Serial.println(F("Error: SD card initialization problem."));
      break;

    case ERR_PCF8523_INIT:
      Serial.println(F("Error: PCF8523 (real-time clock) initialization problem."));
      break;

    case ERR_FT_NO_CONTINUITY:
      Serial.println(F("Error: Flight Termination unit does not detect continuity."));
      break;

    default:
      Serial.println(F("Error: something went wrong. Not exactly sure what."));
      break;
  }

  Serial.println(F("Program will block here. Please fix the error :'("));
  while (1) {
    blink_beep_failure();
  }
}
