#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <SPI.h>
#include <SD.h>

#define DEBUG_MODE
//#define BYPASS_ALL

// Pin definitions
#define main1   4
#define main2   5
#define drogue1 2
#define drogue2 3
#define buzzer  6
#define led     13

// configurations
#define BUZZER_FREQ             3750        // Hz (buzzer sound frequency)
#define ALT_MEAS_AVGING         100         // number of samples to average
#define ALT_ARRAY_SIZE          10          // number of samples to keep track of
#define LAUNCH_THRESHOLD        500         // change in altitude to detect launch
#define TERMINATION_SAMPLES     10          // samples to detect flight termination
#define LANDING_THRESHOLD       10          // change in altitude to detect landing
#define LANDING_SAMPLES         1000        // samples to detect landing

#define LOCAL_PRESSURE          101200      // Pa
#define THRESHOLD_ALTITUDE      10000       // ft
#define TERMINATION_THRESHOLD   18000       // ft

#define TIME_LOCK_SLEEP         10800000    // ms (3 hr)

#ifdef DEBUG_MODE                           // multimeter needs more time to get accurate current reading
#define DROGUE_DELAY            5000        // ms
#else
#define DROGUE_DELAY            12000       // ms
#endif

Adafruit_BME280 bme; // I2C

const float cutoff = 0.00001; // cutoff freq of low-pass filter (units unknown lol)
const int chipSelect = 10;

// Programming variables
float altitude = 0;
float alt_ground = 0;
float alt_filtered = 0;
uint32_t prevTick = 0;
uint32_t count = 0; // for landing detection
uint32_t time_launch = 0;
float alt_previous[ALT_ARRAY_SIZE];
uint8_t alt_arr_position = 0;
uint8_t launched = 0;

void setup() {
  delay(1000); // wait for SD to settle
  // initialize pin modes
  Serial.begin(9600);
  pinMode(main1, OUTPUT);
  pinMode(main2, OUTPUT);
  pinMode(drogue1, OUTPUT);
  pinMode(drogue2, OUTPUT);
  pinMode(buzzer, OUTPUT);
  pinMode(led, OUTPUT);
  pinMode(chipSelect, OUTPUT);

  // set pin states
  digitalWrite(main1, LOW);
  digitalWrite(main2, LOW);
  digitalWrite(drogue1, LOW);
  digitalWrite(drogue2, LOW);
  digitalWrite(led, HIGH);
  noTone(buzzer);

  // reset array of altitude data
  memset(alt_previous, 0, ALT_ARRAY_SIZE*sizeof(*alt_previous));

  // char array for saving to SD
  char dataString[25];
  
  // initialize BME280 sensor
  if (!bme.begin()) {
    Serial.println(F("Could not find a valid BME280 sensor, check wiring!"));
    tone(buzzer, BUZZER_FREQ);
    while (1);
  }
  else {
    blink_beep_success(3, 50);
  }

  // initialize SD card
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("SD card failed, or not present");
    tone(buzzer, BUZZER_FREQ);
    while (1);
  }
  else {
    blink_beep_success(3, 50);

    char header[] = "--- new HAB session ---\n";
    File dataFile = SD.open("haba.txt", FILE_WRITE); // filename must be shorter than 6 characters or something dumb like that
    // if the file is available, write to it:
    if (dataFile) {
      dataFile.println(header);
      dataFile.close();
    }
    else {
      tone(buzzer, BUZZER_FREQ);
      while (1);
    }
  }

  // get ground level pressure/altitude
  for (int i = 0; i < ALT_MEAS_AVGING; i++) {
    alt_ground += getAltitude(); // takes sea-level pressure and reads alt many times
    Store_Altitude_to_SD(alt_ground);
  }
  alt_ground = alt_ground / ALT_MEAS_AVGING; //average of alt readings
  Store_Altitude_to_SD(-100); // flag
  // reset array of altitude data now that ground altitude is known
  memset(alt_previous, alt_ground, ALT_ARRAY_SIZE*sizeof(*alt_previous));

#ifdef DEBUG_MODE
  Serial.print("----------- GROUND ALTITUDE: ");
  Serial.print(alt_ground);
  Serial.println(" -----------");
#endif

#ifdef BYPASS_ALL
  Serial.println("-------- LAUNCHED --------\n");
  blink_beep_success(1, 10);
  Store_Altitude_to_SD(-1.5f);

  delay(100);

  Serial.print("----------- TERMINATING AT ");
  Serial.print(altitude);
  Serial.println(" ft -----------\n");

  blink_beep_success(2, 30);
  Store_Altitude_to_SD(99999999.99); // indicate apogee reached
  digitalWrite(drogue1, HIGH);
  digitalWrite(drogue2, HIGH);
  digitalWrite(main1, HIGH);
  digitalWrite(main2, HIGH);
  delay(DROGUE_DELAY);
  digitalWrite(drogue1, LOW);
  digitalWrite(drogue2, LOW);
  digitalWrite(main1, LOW);
  digitalWrite(main2, LOW);

  delay(100);
  Store_Altitude_to_SD(-1.5f);

  Serial.println("-------- LANDED --------\n");
  blink_beep_success(4, 20);
  Store_Altitude_to_SD(-1.5f);
  
  while (1) {
    altitude = getAltitude();
    alt_filtered = runAltitudeMeasurements(altitude);
    Serial.println(alt_filtered);
    Serial.println(altitude - alt_ground);
    Serial.println();
//    char alt_char_array[100];
//    sprintf(alt_char_array, "alt_array = %d, %d, %d, %d, %d, %d, %d, %d, %d, %d",
//        alt_previous[0], alt_previous[1], alt_previous[2], alt_previous[3], alt_previous[4], alt_previous[5],
//        alt_previous[6], alt_previous[7], alt_previous[8], alt_previous[9]);
//    Serial.println(alt_char_array);
//    Serial.println(alt_arr_position);
    delay(50);
  }
#endif  // BYPASS_ALL

  // wait for launch
  while (alt_filtered < LAUNCH_THRESHOLD) {
    altitude = getAltitude();
    alt_filtered = runAltitudeMeasurements(altitude);
#ifdef DEBUG_MODE
    Serial.print("altitude read = ");
    Serial.print(altitude);
    Serial.print(" -- ");
    Serial.print("filtered altitude = ");
    Serial.print(alt_filtered);
    Serial.println();
#endif
    delay(15);
  }
  time_launch = millis(); // set current time as launched time

#ifdef DEBUG_MODE
  Serial.println("-------- LAUNCHED --------\n");
  blink_beep_success(1, 100);
#endif
  launched = 1; // release the "lock" on termination
  // detect flight termination threshold
  int count_termination = 0;
  while (count_termination < TERMINATION_SAMPLES && launched) {
    altitude = getAltitude();
    alt_filtered = runAltitudeMeasurements(altitude);
    
    if (alt_filtered > TERMINATION_THRESHOLD) {
      count_termination += 1;
    }
    else {
      count_termination = 0;
    }
  }
  
#ifdef DEBUG_MODE
    Serial.print("altitude read = ");
    Serial.print(altitude);
    Serial.print(" -- ");
    Serial.print("filtered altitude = ");
    Serial.print(alt_filtered);
    Serial.println();
#endif
    delay(50);

#ifdef DEBUG_MODE
  Serial.println("-------- FLIGHT TERMINATED --------\n");
  blink_beep_success(1, 10);
  Serial.print("----------- AT TERMINATION ALTITUDE OF ");
  Serial.print(altitude);
  Serial.println(" ft -----------\n");
  blink_beep_success(2, 250);
#endif
  
  Store_Altitude_to_SD(99999999.99); // indicate apogee reached
  // fire both channels to terminate flight
  digitalWrite(drogue1, HIGH);
  digitalWrite(drogue2, HIGH);
  digitalWrite(main1, HIGH);
  digitalWrite(main2, HIGH);
  delay(DROGUE_DELAY);
  digitalWrite(drogue1, LOW);
  digitalWrite(drogue2, LOW);
  digitalWrite(main1, LOW);
  digitalWrite(main1, LOW);

  // landing detection
  while (count < LANDING_SAMPLES) {
    altitude = getAltitude();
    alt_filtered = runAltitudeMeasurements(altitude);
    if (alt_filtered - alt_previous[ALT_ARRAY_SIZE - 1] < LANDING_THRESHOLD && millis() - time_launch > TIME_LOCK_SLEEP)
      count++;
    else
      count = 0;

    delay(15);
  }

#ifdef DEBUG_MODE
  Serial.println("-------- LANDED --------\n");
  blink_beep_success(4, 1500);
#endif
}

void loop() {
#ifdef DEBUG_MODE
  // blink built-in LED
  digitalWrite(led, HIGH);
  delay(500);
  digitalWrite(led, LOW);
  delay(500);
#endif
  altitude = getAltitude();
  alt_filtered = runAltitudeMeasurements(altitude);

  delay(5000);
}

// ----------------------------------
// ----- FUNCTIONS -----
// ----------------------------------
float runAltitudeMeasurements(float currAlt) {
  storeAltitudeArray(currAlt);
  float alt_filtered = filterAltitude();
  Store_Altitude_to_SD(alt_filtered);
  return alt_filtered;
}

// returns current altitude in feet, unfiltered
float getAltitude() {
  return bme.readAltitude(LOCAL_PRESSURE / 100) * 3.28084;
}

// returns filtered altitude (moving average)
float filterAltitude(void) {
  float sum = 0.0f;
  for (int i = 0; i < ALT_ARRAY_SIZE; i++) {
    sum += alt_previous[i];
  }
  return (sum / ALT_ARRAY_SIZE - alt_ground);
}

// stores raw altitude values to array
void storeAltitudeArray(float new_raw_altitude) {
  alt_previous[alt_arr_position] = new_raw_altitude;
  
  if (alt_arr_position == (ALT_ARRAY_SIZE - 1)) {
    alt_arr_position = 0;
  }
  else {
    alt_arr_position += 1;
  }
}

void Store_Altitude_to_SD(float new_altitude) {
  char dataString[75];
  char alt_chars[50];
  dtostrf(new_altitude, 5, 5, alt_chars);
  sprintf(dataString, "%s\n", alt_chars);

  File dataFile = SD.open("haba.txt", FILE_WRITE);
  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
  } 
  else {
    #ifdef DEBUG_MODE
      blink_beep_success(10, 50);
    #endif
  }
}

void blink_beep_success(int beeps, long duration) {
  // blink built-in LED
  digitalWrite(led, HIGH);
  delay(500);
  digitalWrite(led, LOW);
  delay(500);

  for (int i = 0 ; i < beeps; i++) { // beep thrice, long
    tone(buzzer, BUZZER_FREQ);
    delay(duration);
    noTone(buzzer);
    delay(duration);
  }
}
