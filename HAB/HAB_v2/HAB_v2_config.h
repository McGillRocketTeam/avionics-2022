#ifndef _HAB_V2_CONFIG_H
#define _HAB_V2_CONFIG_H

// configurations
#define BUZZER_FREQ             3750        // Hz (buzzer sound frequency)
#define BEEPS_SUCCESS_NUM       5           // number of beeps
#define BEEPS_SUCCESS_DUR       100         // duration of each beep (ms)
#define BEEPS_FAILURE_NUM       3
#define BEEPS_FAILURE_DUR       1000

#define ALT_MEAS_AVGING         100         // number of samples to average
#define ALT_ARRAY_SIZE          10          // number of samples to keep track of
#define LAUNCH_THRESHOLD        500         // change in altitude to detect launch
#define TERMINATION_SAMPLES     10          // samples to detect flight termination
#define LANDING_THRESHOLD       5           // change in altitude to detect landing
#define LANDING_SAMPLES         1000        // samples to detect landing

#define LOCAL_PRESSURE_HPA      1012        // hPa
#define THRESHOLD_ALTITUDE      10000       // ft
#define TERMINATION_ALTITUDE    18000       // ft

#define TIME_LOCK_SLEEP         10800000    // ms (3 hr)
#define FT_DELAY                12000       // ms

#define LOOP_FREQ               15              // Hz
#define LOOP_TIME               1000/LOOP_FREQ  // ms

#endif // _HAB_V2_CONFIG_H
