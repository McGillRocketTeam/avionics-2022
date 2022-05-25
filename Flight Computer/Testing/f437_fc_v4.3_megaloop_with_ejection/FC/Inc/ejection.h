#ifndef INC_EJECTION_H
#define INC_EJECTION_H


#include "main.h"

// public enum
typedef enum {
	EJ_STATE_PAD = 0,
	EJ_STATE_BOOST_COAST,
	EJ_STATE_DROGUE_DESCENT,
	EJ_STATE_MAIN_DESCENT,
	EJ_STATE_LANDED
} ejection_state;

// Public variables
extern float alt_previous[NUM_MEAS_REG];
extern float time_previous[NUM_MEAS_REG];
extern float timalt_previous[NUM_MEAS_REG];
extern float timsqr_previous[NUM_MEAS_REG];

// TODO: put in struct?
extern volatile ejection_state state_flight;
extern volatile uint8_t state_arm_rcov;
extern volatile uint8_t state_arm_prop;

// tracking altitude for state of flight changing
extern float alt_ground;
extern float alt_current;
extern float alt_prev;
extern float alt_apogee;

// Public function define
float runAltitudeMeasurements(uint32_t currTick, uint16_t currAlt);
float filterAltitude(float altitude);
float LSLinRegression();
void init_ejection_states(void);
void check_ejection_state(void);
uint8_t get_continuity(void);

void test_ematch_firing(void);

#endif /* INC_EJECTION_H */
