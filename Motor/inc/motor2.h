#include "main.h"
#include "math.h"
#include "stdint.h"
#include "stdlib.h"
#include "stdlib.h"
#include "stdio.h"
#include "string.h"
#include "time.h"

//// Motor Specifications: 48V 750W BLDC Servo Motor
#define POLE_PAIRS2 4

#define PHASE_RESISTANCE2 0.0735f         // Reduced by 8.3% (original 0.08f)
#define PHASE_INDUCTANCE2 0.0003f

#define BACK_EMF_CONST2 0.063f            // Reduced by 9.1% (original 0.0693f)
#define TORQUE_CONST2 0.063f
#define ROTOR_INERTIA2 0.00192f
#define VISCOUS_DAMPING2 0.00015f
#define COULOMB_FRICTION2 0.01f
#define MAX_PHASE_CURRENT2 20.0f

#define SUPPLY_VOLTAGE2 48.0f
#define MAX_PHASE_CURRENT2 20.0f

// Field weakening configuration
#define BASE_SPEED2 3300.0f    // RPM (point where field weakening begins)
#define MIN_FW_FACTOR2 0.65f   // Minimum field weakening factor (65% weakening)


//#define POLE_PAIRS2 4
//
//#define PHASE_RESISTANCE2 0.08f
//#define PHASE_INDUCTANCE2 0.0003f
//
//#define BACK_EMF_CONST2 0.0693f
//#define TORQUE_CONST2 0.0693f
//#define ROTOR_INERTIA2 0.00192f
//#define VISCOUS_DAMPING2 0.00015f
//#define COULOMB_FRICTION2 0.01f
//
//#define SUPPLY_VOLTAGE2 48.0f
//#define MAX_PHASE_CURRENT2 20.0f
//
//// Field weakening configuration
//#define BASE_SPEED2 3300.0f    // RPM (point where field weakening begins)
//#define MIN_FW_FACTOR2 0.65f   // Minimum field weakening factor (65% weakening)

// Electrical state variables
typedef struct {
	float theta_elec;       // Electrical angle (rad)
	float i_a, i_b, i_c;    // Phase currents (A)
	float v_a, v_b, v_c;    // Phase voltages (V)
} ElectricalState2;

// Mechanical state variables
typedef struct {
	float theta_mech;       // Mechanical angle (rad)
	float omega;            // Angular velocity (rad/s)
	float torque;           // Electromagnetic torque (Nm)
} MechanicalState2;

// Motor state structure
typedef struct {
	ElectricalState2 elec;
	MechanicalState2 mech;
	uint64_t last_update;
	float prev_duty;
} MotorState2;

uint32_t update_bldc_model2(float duty, uint64_t timestamp);
void init_motor_state2(float initial_speed_rpm);
void calculate_commutation2(float theta_elec, float *c_a, float *c_b, float *c_c);
void TestMaxSpeed2();
void VerifyBaseSpeed2();
