///*
// * BLDC_Right.h
// *
// *  Created on: Feb 14, 2025
// *      Author: User
// */
//
//#ifndef INC_BLDC_PID_TUNING_QPC_H_
//#define INC_BLDC_PID_TUNING_QPC_H_
//
//#include "math.h"
//#include "stdint.h"
//#include "stdlib.h"
//#include "stdlib.h"
//#include "stdio.h"
//#include "string.h"
//#include "time.h"
//#include "uart_comm.h"
//#include <float.h>
//
//#define RIGHT_DRIVE 0
//
//// Define constants and limits
//#define POPULATION_SIZE 50
//#define GENERATIONS 15
//#define MUTATION_RATE 0.1f
//#define PID_MIN_VAL 0.0f
//#define PID_MAX_VAL_KP 1.5f
//#define PID_MAX_VAL_KI 15.0f
//#define PID_MAX_VAL_KD 0.1f
//#define TOP_SPEED 3300
//#define DUTY_MAX 100
//#define W1 0.5f
//#define W2 0.5f
//#define W3 1e3f
//#define W4 1.0f
//#define W5 1e3f
//#define PID_ADJUST_TIME 30
//
//
//// Motor Specifications: 48V 750W BLDC Servo Motor
//#define POLE_PAIRS 4
//#if RIGHT_DRIVE
//#define PHASE_RESISTANCE 0.08f
//#define PHASE_INDUCTANCE 0.0003f
//
//#define BACK_EMF_CONST 0.0693f  // Original calculated value
//#define TORQUE_CONST 0.0693f    // Must match back-EMF constant
//#define ROTOR_INERTIA 0.00192f
//#define VISCOUS_DAMPING 0.00015f  // Reduced
//#define COULOMB_FRICTION 0.01f    // Reduced
//#define SUPPLY_VOLTAGE 48.0f
//#define MAX_PHASE_CURRENT 20.0f
//#else
//// 50% difference from right drive
//#define PHASE_RESISTANCE 0.04f
//#define PHASE_INDUCTANCE 0.00045f
//
//#define BACK_EMF_CONST 0.0693f  // Original calculated value
//#define TORQUE_CONST 0.0693f    // Must match back-EMF constant
//#define ROTOR_INERTIA 0.0015f
//#define VISCOUS_DAMPING 0.00015f  // Reduced
//#define COULOMB_FRICTION 0.021f    // Reduced
//#define SUPPLY_VOLTAGE 48.0f
//#define MAX_PHASE_CURRENT 20.0f
//#endif
//
//// Field weakening configuration
//#define BASE_SPEED 3300.0f    // RPM (point where field weakening begins)
//#define MIN_FW_FACTOR 0.65f   // Minimum field weakening factor (65% weakening)
//
//// Electrical state variables
//typedef struct {
//	float theta_elec;       // Electrical angle (rad)
//	float i_a, i_b, i_c;    // Phase currents (A)
//	float v_a, v_b, v_c;    // Phase voltages (V)
//} ElectricalState;
//
//// Mechanical state variables
//typedef struct {
//	float theta_mech;       // Mechanical angle (rad)
//	float omega;            // Angular velocity (rad/s)
//	float torque;           // Electromagnetic torque (Nm)
//} MechanicalState;
//
//// Motor state structure
//typedef struct {
//	ElectricalState elec;
//	MechanicalState mech;
//	uint64_t last_update;
//	float prev_duty;
//} MotorState;
//
//struct feedback_data_struct {
//	float error;
//	float speed_target;
//	float speed_calc;
//};
//typedef struct {
//	float Kp;
//	float Ki;
//	float Kd;
//} PID_Coefficients;
//
//struct bldc_command {
//	int current_direction;
//	float current_speed;
//};
//
//// Particle structure definition
//typedef struct {
//	float Kp, Ki, Kd;
//	float fitness;
//	float best_Kp, best_Ki, best_Kd;
//	float best_fitness;
//} Particle;
//
//typedef struct {
//	uint32_t target_speed;
//	uint32_t simulation_time;
//} testData;
//
//typedef struct {
//	float rise_time;
//	float settling_time;
//	float max_deviation;
//	float error_sum;
//	int osc_count;
//} fitnessInputs;
//
//// Function prototypes
//void QPSO_PID_Tuning(void);
//void initialize_population(void);
//void initialize_testParam(void);
//void evaluate_population(void);
//void get_best_individual(void);
//void update_particle_positions(void);
//void simulate_motor_behavior(Particle pid_coeffs, uint32_t target_speed, uint32_t simulation_time);
//float calculate_fitness(float rise_time, float settling_time, float max_deviation, float error_sum, int osc_count);
//void set_speed(float duty_cycle);
//float check_error_limit(float error);
//float check_duty_limit(float duty_cycle);
//
//void ExecuteCommand();
//void ExtractCommand();
//
//uint32_t update_bldc_model(float duty, uint64_t timestamp);
//void init_motor_state(float initial_speed_rpm);
//void calculate_commutation(float theta_elec, float *c_a, float *c_b, float *c_c);
//void AnalyzeMotorCharacteristics();
//void TestMaxSpeed();
//void VerifyBaseSpeed();
//
//#endif /* INC_BLDC_H_ */
