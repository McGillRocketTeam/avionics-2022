// libraries
#include <Wire.h>
#include <SPI.h>

#define GPSECHO false       // print raw GPS sentences to Serial terminal
#define TESTING_BYPASS_ALL  // comment out for actual launch

#include <Adafruit_Sensor.h>
#include <Adafruit_MS8607.h>
#include <Adafruit_ADXL345_U.h>
#include <RTClib.h>
#include <SD.h>

#include <string.h>
#include "HAB_v2_pins.h"
#include "HAB_v2_config.h"

#include <Adafruit_GPS.h>
#include <TinyGPS++.h>

#define GPSSerial Serial2

// types
enum error_states {
  ERR_OK = 0,
  ERR_ADXL345_INIT,
  ERR_MS8607_INIT,
  ERR_SD_CARD_INIT,
  ERR_SD_CARD_FILE_INIT,
  ERR_PCF8523_INIT,
  ERR_FT_NO_CONTINUITY,
};

enum flight_states {
  FS_PAD,
  FS_ASCENT,
  FS_DESCENT,
  FS_LANDED
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

struct gps_data {
  uint32_t sats;
  double speed;
  double alt;
  float latitude;
  float longitude;
  float heading;
};


// private function prototypes
void init_pinModes(void);
void init_pinStates(void);
void blink_beep(int beeps, long duration);
void blink_beep_success(void);
void blink_beep_failure(void);

void adxl345_init(void);
void ms8607_init(void);
void pcf8523_init(void);
void gps_init(void);
void sd_card_init(void);
void sd_find_dynamic_file_name(char *prefix, char *filename);

void ms8607_poll(struct ms8607_data *data_s);
void adxl345_poll(struct adxl345_data *data_s);
void pcf8523_poll(struct pcf8523_data *data_s);
void gps_poll(struct gps_data *data_s);

void  ft_init(void);
float read_FT_continuity(int pin);
void  ft_terminate(void);
uint8_t ft_check_for_ft_alt(void);
uint8_t ft_check_for_landing(void);
void  ft_test_on_ground(void);

float get_altitude(void);
float get_altitude(float pressure_hPa);
void  save_previous_altitudes(float alt);

void do_save_telemetry(void);

void video_recorder_start_recording(void);
void video_recorder_stop_recording(void);

void Error_Handler(enum error_states err);

// programming variables
float alt_ground = 0;
float alt_agl = 0;
float pressure_ground = 0;
float pressure_current = 0;
uint32_t count = 0;
float alt_previous[ALT_ARRAY_SIZE] = {0};
uint8_t alt_arr_position = 0;

uint8_t flight_state = FS_PAD;
uint8_t vr_is_recording = 0;

const char datafile_header_string = "S,ACCx_m/s2,ACCy,ACCz,PRESSURE_hPa,TEMPERATURE_C,HUMIDITY_rH,TIME,E";

// objects for sensors
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345); // assign unique ID to sensor
Adafruit_MS8607 ms8607;
RTC_PCF8523 rtc;
File datafile;
Adafruit_GPS GPS(&GPSSerial);
TinyGPSPlus gps;

// structs for data
struct ms8607_data ms8607_ds = {0}; // ds = data struct
struct adxl345_data adxl345_ds = {0};
struct pcf8523_data pcf8523_ds = {0};
struct gps_data gps_ds = {0};


void setup() {
  Serial.begin(9600);
  Wire.begin();
  
  // init pins
  init_pinModes();
  init_pinStates();

  // init sensors
  adxl345_init();
  ms8607_init();
  pcf8523_init();
  sd_card_init();
  gps_init();

  // init flight termination logic
  ft_init();

  // indicate successful startup to user
  digitalWrite(LED1, HIGH);
  blink_beep_success();

  video_recorder_start_recording();
  vr_is_recording = 1;

  #ifdef TESTING_BYPASS_ALL
  ft_test_on_ground();
  while (1);
  #endif  
  
}

void loop() {
  unsigned long loop_start = millis();

  do_save_telemetry();
  update_flight_state();

  if (flight_state == FS_LANDED) {
    unsigned long delay_time = 10000 - (millis() - loop_start);
    delay(delay_time);
  }
  // otherwise run main loop as fast as possible
}


void do_save_telemetry(void) {
  // update sensor data
  ms8607_poll(&ms8607_ds);
  adxl345_poll(&adxl345_ds);
  pcf8523_poll(&pcf8523_ds);
  gps_poll(&gps_ds);

  alt_agl = get_altitude((&ms8607_ds)->pressure) - alt_ground;
  save_previous_altitudes(alt_agl);

  // save to storage
  char telemetry_buffer[150] = {0};
  sprintf(telemetry_buffer, "S,%.4f,%.4f,%.4f,%.4f,%.3f,%.3f,%d,%d,%d,%d,E",
      adxl345_ds.acc_x, adxl345_ds.acc_y, adxl345_ds.acc_z,
      ms8607_ds.pressure, ms8607_ds.temperature, ms8607_ds.humidity,
      pcf8523_ds.day, pcf8523_ds.hour, pcf8523_ds.minute, pcf8523_ds.second
  );
  datafile.println(telemetry_buffer); // save to SD card

  // gps data
  sprintf(telemetry_buffer, "GPS,%l,%f,%f,%f,%f,%f,E",
      gps_ds.sats, gps_ds.speed, gps_ds.alt, 
      gps_ds.latitude, gps_ds.longitude, gps_ds.heading
  );
  datafile.println(telemetry_buffer);
  datafile.flush(); // actually write to SD without closing file

  // in more readable form for serial terminal
  sprintf(telemetry_buffer, 
      "ACCx = %.4f\tACCy = %.4f\tACCz = %.4f\nPRESSURE = %.4f\tTEMP = %.3f\tREL HUM = %.3f\nDAY = %d\tHOUR = %d\tMIN = %d\tSEC = %d",
      adxl345_ds.acc_x, adxl345_ds.acc_y, adxl345_ds.acc_z,
      ms8607_ds.pressure, ms8607_ds.temperature, ms8607_ds.humidity,
      pcf8523_ds.day, pcf8523_ds.hour, pcf8523_ds.minute, pcf8523_ds.second
  );
  Serial.println(telemetry_buffer);
  Serial.println("");
}

void update_flight_state(void) {
  switch (flight_state) {
    case FS_PAD:
      if (alt_agl > LAUNCH_THRESHOLD) {
        flight_state = FS_ASCENT;
        
        datafile.print(F("Launched! Altitude (AGL, ft) = "));
        datafile.println(alt_agl);
        datafile.flush();
      }
      break;

    case FS_ASCENT:
      if (ft_check_for_ft_alt()) {
        flight_state = FS_DESCENT;

        datafile.print(F("Starting Flight Termination! Altitude (AGL, ft) = "));
        datafile.println(alt_agl);
        datafile.flush();
        
        ft_terminate();
      }
      break;

    case FS_DESCENT:
      if (ft_check_for_landing()) {
        flight_state = FS_LANDED;
        datafile.print(F("Landed! Altitude (AGL, ft) = "));
        datafile.println(alt_agl);
        datafile.flush();
      }
      break;

    case FS_LANDED:
      if (vr_is_recording) {
        video_recorder_stop_recording();
        vr_is_recording = 0;
      }
      break;
    
    default:
      // should never use this case
      break;
  }
}

// check if HAB is above the flight termination altitude
uint8_t ft_check_for_ft_alt(void) {
  if (alt_agl > TERMINATION_ALTITUDE) {
    count += 1;

    if (count > TERMINATION_SAMPLES) {
      count = 0;
      return 1;
    }
  } else {
    count = 0;
  }

  return 0;
}

// check if HAB has landed
uint8_t ft_check_for_landing(void) {
  if (alt_agl - alt_previous[ALT_ARRAY_SIZE-1] < LANDING_THRESHOLD) {
    count += 1;

    if (count > LANDING_SAMPLES) {
      count = 0;
      return 1;
    }
  } else {
    count = 0;
  }

  return 0;
}

// terminate flight
void ft_terminate(void) {
  digitalWrite(FT_GATE_1, HIGH);
  digitalWrite(FT_GATE_2, HIGH);
  digitalWrite(FT_GATE_3, HIGH);
  digitalWrite(FT_GATE_4, HIGH);
  delay(FT_DELAY);
  digitalWrite(FT_GATE_1, LOW);
  digitalWrite(FT_GATE_2, LOW);
  digitalWrite(FT_GATE_3, LOW);
  digitalWrite(FT_GATE_4, LOW);
}

// sensor init functions
void adxl345_init(void) {
  if (!accel.begin()) {
    Error_Handler(ERR_ADXL345_INIT);
  }

  accel.setRange(ADXL345_RANGE_8_G);
  accel.setDataRate(ADXL345_DATARATE_100_HZ);
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
//  char filename = "HAB1.txt";
  sd_find_dynamic_file_name("HAB", filename);
  datafile = SD.open(filename, FILE_WRITE);

  // write header to file
  if (datafile) {
    datafile.println("S,ACCx_m/s2,ACCy,ACCz,PRESSURE_hPa,TEMPERATURE_C,HUMIDITY_rH,TIME,E");
    datafile.flush();
  }
  else {
    Error_Handler(ERR_SD_CARD_FILE_INIT);
  }
  
  
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

void gps_init(void) {
  GPSSerial.begin(9600);
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); // turn on RMC and GGA data
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);    // 5 Hz update rate
  
}

void ft_init(void) {
  analogReadResolution(10);

  // read continuity and see if channels are armed
  for (uint8_t i = FT_C1; i <= FT_C4; i++) {
    float voltage = read_FT_continuity(i);
    if (voltage < 0.7 * 3.3) {
      Serial.print("Warning: No continuity detected on FT Channel ");
      Serial.println(i - FT_C1 + 1);
    }
  }

  // get ground pressure and average
  float pressure = 0;
  for (uint8_t i = 0; i < ALT_MEAS_AVGING; i++) {
    ms8607_poll(&ms8607_ds);
    pressure += ms8607_ds.pressure;
  }
  pressure /= ((float) ALT_MEAS_AVGING);
  pressure_ground = pressure;
  alt_ground = get_altitude(pressure_ground);
  
  // now that ground pressure is known, reset array
  memset(alt_previous, alt_ground, ALT_ARRAY_SIZE*sizeof(*alt_previous));

  Serial.println(F("Flight termination init: ok!"));
  Serial.print(F("\tGround altitude (ft): "));
  Serial.println(alt_ground);
  Serial.print(F("\tGround pressure (hPa): "));
  Serial.println(pressure_ground);
}


void sd_find_dynamic_file_name(char *prefix, char *filename) {
  char temp_filename[13];
  for (unsigned int i = 0; i < 1000 ; i++) {
    sprintf(temp_filename, "%s%03d.txt", prefix, i); 
    if (SD.exists(temp_filename)) {
      continue;
    }
    else {
      Serial.print("Filename will be = ");
      Serial.println(temp_filename);
      strcpy(filename, temp_filename);
      return;
    }
  }

  // if get here, then reached i > 100
  strcpy(filename, "HAB001.txt");
  Serial.println("Filename will be HAB001.txt");
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

void gps_poll(struct gps_data *data_s) {
  while (GPSSerial.available()) {
    gps.encode(GPSSerial.read());
  }
  uint32_t chars_processed = gps.charsProcessed(); // not sure what this is for
  
  data_s->sats = gps.satellites.value();
  data_s->longitude = gps.location.lng();
  data_s->latitude = gps.location.lat();
  data_s->heading = gps.course.deg();
  data_s->speed = gps.speed.kmph();
  data_s->alt = gps.altitude.meters();
}

float read_FT_continuity(int pin) {
  return (analogRead(pin) / 1024.0 * 3.3);
}

// converts pressure in hPa to altitude in ft (absolute, not AGL)
float get_altitude(float pressure_hPa) {
  return (145442.1609 * (1.0 - pow(pressure_hPa/LOCAL_PRESSURE_HPA, 0.190266436)));
}

// overloaded function
float get_altitude(void) {
  ms8607_poll(&ms8607_ds);
  return (get_altitude((&ms8607_ds)->pressure));
}

void save_previous_altitudes(float alt) {
  alt_previous[alt_arr_position++] = alt;
  if (alt_arr_position == (ALT_ARRAY_SIZE - 1)) { // circular array
    alt_arr_position = 0;
  }
}

void video_recorder_start_recording(void) {
  digitalWrite(VR_CTRL, HIGH);
  delay(400);
  digitalWrite(VR_CTRL, LOW);
  delay(400);
  digitalWrite(VR_CTRL, HIGH);
  delay(400);
  digitalWrite(VR_CTRL, LOW);
}

void video_recorder_stop_recording(void) {
  digitalWrite(VR_CTRL, HIGH);
  delay(1000);
  digitalWrite(VR_CTRL, LOW);
}

void ft_test_on_ground(void) {
  Serial.println("Testing Flight Termination channels on the ground!");
  delay(1000);
  Serial.print("Starting test...");
  blink_beep(2, 100);
  delay(1000);
  ft_terminate();
  Serial.println("finished.");
  blink_beep(3, 100);
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

// beeps the buzzer for the number of beeps specified,
// for the duration specified. also blinks LED3.
void blink_beep(int beeps, long duration) {

  for (int i = 0 ; i < beeps; i++) { // beep thrice, long
    tone(BUZZER, BUZZER_FREQ);
    digitalWrite(LED2, HIGH);
    delay(duration);
    digitalWrite(LED2, LOW);
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

    case ERR_SD_CARD_FILE_INIT:
      Serial.println(F("Error: SD card could not open file."));
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
