/*
 * BLDC_Right.h
 *
 *  Created on: Feb 14, 2025
 *      Author: User
 */

#ifndef INC_BLDC_PID_TUNING_GA_H_
#define INC_BLDC_PID_TUNING_GA_H_

#include "main.h"
#include "math.h"
#include "stdint.h"
#include "stdlib.h"
#include "stdlib.h"
#include "stdio.h"
#include "string.h"
#include "time.h"

#include "motor1.h"
#include "motor2.h"

#define TUNE_MOTOR1 1
#define TUNE_MOTOR2 0
#define TEST_MOTOR1_PID 0
#define TEST_MOTOR2_PID 0



#define PRINT_ALL 0

#define POPULATION_SIZE 50
#define POPULATION_INIT_INTERVAL 10
#define GENERATIONS 120
#define TOURNAMENT_SIZE 4
#define MUTATION_RATE 0.5
#define MUTATION_PERCENTAGE 0.15
#define MOTOR1_TUNE_INTERVAL 5
#define MOTOR2_TUNE_INTERVAL 15

#define DUTY_MAX 100
#define DUTY_MIN 0
#define MAX_SPEED 3300
#define MIN_RISE_TIME 1500
#define MIN_SETTLING_TIME 200
#define SIMULATION_TIME 5000
#define SIMULATION_SPEED 3000

typedef struct {
	float speed_target;
	float speed_calc;
	float position;
} motor_status;

typedef struct {
	float Kp;
	float Ki;
	float Kd;
} PID_Coefficients;

typedef struct {
	PID_Coefficients pid;
	PID_Coefficients sync_pid;
	float integral_preload;
	float fitness;
} Individual;

typedef struct {
	uint32_t initial_speed;
	uint32_t target_speed;
	uint32_t simulation_time;
} testData;

// -------- PID Structure and Functions --------
typedef struct {
	float kp, ki, kd;
	float prev_measured;
	float filtered_deriv;
	float proportional;
	float integral;
	float derivative;
	float error;
} PID_Controller;

typedef struct {
	float max_deviation;
	int rise_threshold_crossed;
	int rise_time;
	int settling_start_flag;
	int consecutive_settled_count;
	int settling_time;
	float initial_speed;
	float error_span;
	float rise_low_bound;
	float rise_high_bound;
} PerformanceMetrics;

void TunePID();
void initialize_population();
void initialize_testParam();
void evaluate_population();
void select_individuals();
void crossover_individuals();
void mutate_individuals();
void get_best_individual();
void SimulateMotors(PID_Coefficients pid_coeffs, PID_Coefficients pid_sync_coeffs, float integral_preload, uint32_t initial_speed,
		uint32_t target_speed, uint32_t simulation_time);
float Check_duty_limit(float right_duty);
void SetSpeed();

void PID_Init(PID_Controller *pid, PID_Coefficients pid_coeff, float FeedForward);
float PID_Compute(PID_Controller *pid, float setpoint, float measured, float max_value, float dt);

void init_performance_metrics(PerformanceMetrics *metrics, float initial_speed, float target_speed);
void update_overshoot(PerformanceMetrics *metrics, float current_speed, float target_speed);
void update_rise_time(PerformanceMetrics *metrics, float current_speed, float target_speed, float current_time);
void update_settling_time(PerformanceMetrics *metrics, float current_speed, float target_speed, float current_time);

void ExecuteCommand();
void ExtractCommand();

#endif /* INC_BLDC_H_ */
