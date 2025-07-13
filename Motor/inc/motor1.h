#include "main.h"
#include "math.h"
#include "stdint.h"
#include "stdlib.h"
#include "stdlib.h"
#include "stdio.h"
#include "string.h"
#include "time.h"

// Motor Specifications: 48V 750W BLDC Servo Motor
#define POLE_PAIRS 4

#define PHASE_RESISTANCE 0.08f
#define PHASE_INDUCTANCE 0.0003f

#define BACK_EMF_CONST 0.0693f
#define TORQUE_CONST 0.0693f
#define ROTOR_INERTIA 0.00192f
#define VISCOUS_DAMPING 0.00015f
#define COULOMB_FRICTION 0.01f

#define SUPPLY_VOLTAGE 48.0f
#define MAX_PHASE_CURRENT 20.0f

// Field weakening configuration
#define BASE_SPEED 3300.0f    // RPM (point where field weakening begins)
#define MIN_FW_FACTOR 0.65f   // Minimum field weakening factor (65% weakening)

// Electrical state variables
typedef struct {
	float theta_elec;       // Electrical angle (rad)
	float i_a, i_b, i_c;    // Phase currents (A)
	float v_a, v_b, v_c;    // Phase voltages (V)
} ElectricalState;

// Mechanical state variables
typedef struct {
	float theta_mech;       // Mechanical angle (rad)
	float omega;            // Angular velocity (rad/s)
	float torque;           // Electromagnetic torque (Nm)
} MechanicalState;

// Motor state structure
typedef struct {
	ElectricalState elec;
	MechanicalState mech;
	uint64_t last_update;
	float prev_duty;
} MotorState;

uint32_t update_bldc_model1(float duty, uint64_t timestamp);
void init_motor_state1(float initial_speed_rpm);
void calculate_commutation1(float theta_elec, float *c_a, float *c_b, float *c_c);
void AnalyzeMotorCharacteristics1();
void TestMaxSpeed1();
void VerifyBaseSpeed1();
