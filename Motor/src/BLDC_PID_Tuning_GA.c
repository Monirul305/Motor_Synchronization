/*
 * BLDC.c
 *
 *  Created on: Feb 14, 2025
 *      Author: Monirul
 */
#include "BLDC_PID_Tuning_GA.h"

motor_status motor1_status = { 0, 0, 0 };
motor_status motor2_status = { 0, 0, 0 };

PID_Controller pid_motor1 = { 0, 0, 0, 0, 0, 0, 0, }, pid_motor2 = { 0, 0, 0, 0,
		0, 0, 0 };
PID_Controller pid_sync = { 0, 0, 0, 0, 0, 0, 0, };
PerformanceMetrics metrics;

// motor1 correction based on speed and small rise time
//PID_Coefficients tuned_pid1 = { 0.253539413, 4.26802111, 0.000151426008 };
//PID_Coefficients tuned_sync_pid = { 9.06193638, 38.6679001, 0.000323431857 };
//float tuned_integral_preload = 0.549200416;

// motor1 correction based on speed and large rise time
PID_Coefficients tuned_pid1 = { 0.0555895865, 1.91909659, 9.55296855e-006 };
PID_Coefficients tuned_sync_pid = { 2.47759199, 176.52774, 5.38743625e-005 };
float tuned_integral_preload = 1.40992403;

// motor1 pid without correction
//PID_Coefficients tuned_pid1 = { 0.0321689285, 1.93102467, 0.000142899764 };
//PID_Coefficients tuned_sync_pid = { 0, 0, 0 };
//float tuned_integral_preload = 0;

//PID_Coefficients tuned_pid2 = { 0.767210543, 6.60639286, 0.013410476 }; // small rise time

PID_Coefficients tuned_pid2 = { 0.082243979, 1.80097032, 0.000155176574 }; // larger rise time

Individual pid_population[POPULATION_SIZE];
Individual pid_sync_population[POPULATION_SIZE];
Individual best_population;
testData testParam[POPULATION_SIZE];

float del_Kp, del_Ki, del_Kd, del_intg;
float del_Kp_sync, del_Ki_sync, del_Kd_sync;

uint32_t time_old = 0, time_new, time_passed = 0;

float duty_cycle1 = 0;
float duty_cycle2 = 0;
float duty_correction1 = 0;

float speed_diff_error;
float position_diff_error;

int generation_num = 0;
int population_num = 0;

void TunePID() {
	initialize_testParam();

#if TUNE_MOTOR1
	best_population.pid = tuned_pid1;
#else
	best_population.pid = tuned_pid2;
#endif
	best_population.sync_pid = tuned_sync_pid;
	best_population.integral_preload = tuned_integral_preload;
	initialize_population();

	for (generation_num = 0; generation_num < GENERATIONS; generation_num++) {
		if ((generation_num > 0)
				&& (generation_num % POPULATION_INIT_INTERVAL == 0)) {
			initialize_population();
		}

		evaluate_population();
		get_best_individual();
		select_individuals();
		crossover_individuals();
		mutate_individuals();
	}

	printf("PID: %f %f %f \n SYNC: %f %f %f \n PRELOAD: %f \n",
			best_population.pid.Kp, best_population.pid.Ki,
			best_population.pid.Kd, best_population.sync_pid.Kp,
			best_population.sync_pid.Ki, best_population.sync_pid.Kd,
			best_population.integral_preload);
}

void initialize_population() {
	srand(HAL_GetTick());  // Seed the random number generator

	for (int population_num = 0; population_num < POPULATION_SIZE;
			population_num++) {

		pid_population[population_num].pid.Kp = ((float) rand() / RAND_MAX) * 2
				* best_population.pid.Kp;
		pid_population[population_num].pid.Ki = ((float) rand() / RAND_MAX) * 2
				* best_population.pid.Ki;
		pid_population[population_num].pid.Kd = ((float) rand() / RAND_MAX) * 2
				* best_population.pid.Kd;

		pid_population[population_num].sync_pid.Kp = ((float) rand() / RAND_MAX)
				* 2 * best_population.sync_pid.Kp;
		pid_population[population_num].sync_pid.Ki = ((float) rand() / RAND_MAX)
				* 2 * best_population.sync_pid.Ki;
		pid_population[population_num].sync_pid.Kd = ((float) rand() / RAND_MAX)
				* 2 * best_population.sync_pid.Kd;

		pid_population[population_num].integral_preload = ((float) rand()
				/ RAND_MAX) * 2 * best_population.integral_preload;

		pid_population[population_num].fitness = 0;
	}

	int idx = rand() % POPULATION_SIZE;
	pid_population[idx].pid.Kp = best_population.pid.Kp;
	pid_population[idx].pid.Ki = best_population.pid.Ki;
	pid_population[idx].pid.Kd = best_population.pid.Kd;

	pid_population[idx].sync_pid.Kp = best_population.sync_pid.Kp;
	pid_population[idx].sync_pid.Ki = best_population.sync_pid.Ki;
	pid_population[idx].sync_pid.Kd = best_population.sync_pid.Kd;

	pid_population[idx].integral_preload = best_population.integral_preload;

	del_Kp = best_population.pid.Kp * MUTATION_PERCENTAGE;
	del_Ki = best_population.pid.Ki * MUTATION_PERCENTAGE;
	del_Kd = best_population.pid.Kd * MUTATION_PERCENTAGE;

	del_Kp_sync = best_population.sync_pid.Kp * MUTATION_PERCENTAGE;
	del_Ki_sync = best_population.sync_pid.Ki * MUTATION_PERCENTAGE;
	del_Kd_sync = best_population.sync_pid.Kd * MUTATION_PERCENTAGE;

	del_intg = best_population.integral_preload * MUTATION_PERCENTAGE;

}

void initialize_testParam() {

	printf("target speed1 speed2 diff \n");

	for (int i = 0; i < POPULATION_SIZE; i++) {
		testParam[i].initial_speed = 0;
		testParam[i].target_speed = SIMULATION_SPEED;
		testParam[i].simulation_time = SIMULATION_TIME;
	}

}

void get_best_individual() {

	Individual best_pid = pid_population[0];

	for (int population_num = 1; population_num < POPULATION_SIZE;
			population_num++) {
		if (pid_population[population_num].fitness > best_pid.fitness) {
			best_pid = pid_population[population_num];
		}
	}

	best_population = best_pid;
}

void select_individuals() {
	Individual selected_pid_population[POPULATION_SIZE];

	for (int i = 0; i < POPULATION_SIZE; i++) {

		int best_idx = rand() % POPULATION_SIZE; // Pick first candidate randomly
		// Select the best out of TOURNAMENT_SIZE individuals
		for (int j = 1; j < TOURNAMENT_SIZE; j++) {

			int idx = rand() % POPULATION_SIZE;
			if (pid_population[idx].fitness
					> pid_population[best_idx].fitness) {
				best_idx = idx;  // Update best candidate
			}
		}
		// Store the selected individual
		selected_pid_population[i] = pid_population[best_idx];
	}

	// Copy the selected individuals back to the main population
	for (int i = 1; i < POPULATION_SIZE; i++) {
		pid_population[i] = selected_pid_population[i];
	}

	pid_population[rand() % POPULATION_SIZE] = best_population;
}

void crossover_individuals() {

	for (int population_num = 0; population_num < POPULATION_SIZE;
			population_num += 2) {
		if (population_num + 1 < POPULATION_SIZE) {

			int idx1 = population_num;
			int idx2 = population_num + 1;

			PID_Coefficients parent1 = pid_population[idx1].pid;
			PID_Coefficients parent2 = pid_population[idx2].pid;

			float alpha = ((float) rand() / RAND_MAX);  // Crossover factor
			pid_population[population_num].pid.Kp = alpha * parent1.Kp
					+ (1 - alpha) * parent2.Kp;
			pid_population[population_num + 1].pid.Kp = alpha * parent2.Kp
					+ (1 - alpha) * parent1.Kp;

			alpha = ((float) rand() / RAND_MAX);
			pid_population[population_num].pid.Ki = alpha * parent1.Ki
					+ (1 - alpha) * parent2.Ki;
			pid_population[population_num + 1].pid.Ki = alpha * parent2.Ki
					+ (1 - alpha) * parent1.Ki;

			alpha = ((float) rand() / RAND_MAX);
			pid_population[population_num].pid.Kd = alpha * parent1.Kd
					+ (1 - alpha) * parent2.Kd;
			pid_population[population_num + 1].pid.Kd = alpha * parent2.Kd
					+ (1 - alpha) * parent1.Kd;

			parent1 = pid_population[idx1].sync_pid;
			parent2 = pid_population[idx2].sync_pid;

			alpha = ((float) rand() / RAND_MAX);  // Crossover factor
			pid_population[population_num].sync_pid.Kp = alpha * parent1.Kp
					+ (1 - alpha) * parent2.Kp;
			pid_population[population_num + 1].sync_pid.Kp = alpha * parent2.Kp
					+ (1 - alpha) * parent1.Kp;

			alpha = ((float) rand() / RAND_MAX);
			pid_population[population_num].sync_pid.Ki = alpha * parent1.Ki
					+ (1 - alpha) * parent2.Ki;
			pid_population[population_num + 1].sync_pid.Ki = alpha * parent2.Ki
					+ (1 - alpha) * parent1.Ki;

			alpha = ((float) rand() / RAND_MAX);
			pid_population[population_num].sync_pid.Kd = alpha * parent1.Kd
					+ (1 - alpha) * parent2.Kd;
			pid_population[population_num + 1].sync_pid.Kd = alpha * parent2.Kd
					+ (1 - alpha) * parent1.Kd;

			float parent_intg1 = pid_population[idx1].integral_preload;
			float parent_intg2 = pid_population[idx2].integral_preload;

			alpha = ((float) rand() / RAND_MAX);  // Crossover factor
			pid_population[population_num].integral_preload = alpha
					* parent_intg1 + (1 - alpha) * parent_intg2;
			pid_population[population_num + 1].integral_preload = alpha
					* parent_intg2 + (1 - alpha) * parent_intg1;

		}
	}

	pid_population[rand() % POPULATION_SIZE] = best_population;
}

void mutate_individuals() {

	for (int population_num = 0; population_num < POPULATION_SIZE;
			population_num++) {

		if (((float) rand() / RAND_MAX) < MUTATION_RATE) {

			pid_population[population_num].pid.Kp = fmax(0,
					pid_population[population_num].pid.Kp
							+ ((float) rand() / RAND_MAX) * del_Kp	- del_Kp / 2);

			pid_population[population_num].pid.Ki = fmax(0,
					pid_population[population_num].pid.Ki
							+ ((float) rand() / RAND_MAX) * del_Ki
							- del_Ki / 2);

			pid_population[population_num].pid.Kd = fmax(0,
					pid_population[population_num].pid.Kd
							+ ((float) rand() / RAND_MAX) * del_Kd
							- del_Kd / 2);

			pid_population[population_num].sync_pid.Kp = fmax(0,
					pid_population[population_num].sync_pid.Kp
							+ ((float) rand() / RAND_MAX) * del_Kp_sync
							- del_Kp_sync / 2);

			pid_population[population_num].sync_pid.Ki = fmax(0,
					pid_population[population_num].sync_pid.Ki
							+ ((float) rand() / RAND_MAX) * del_Ki_sync
							- del_Ki_sync / 2);

			pid_population[population_num].sync_pid.Kd = fmax(0,
					pid_population[population_num].sync_pid.Kd
							+ ((float) rand() / RAND_MAX) * del_Kd_sync
							- del_Kd_sync / 2);

			pid_population[population_num].integral_preload = fmax(0,
					pid_population[population_num].integral_preload
							+ ((float) rand() / RAND_MAX) * del_intg
							- del_intg / 2);
		}
	}

	pid_population[rand() % POPULATION_SIZE] = best_population;
}

void evaluate_population() {

	for (population_num = 0; population_num < POPULATION_SIZE;
			population_num++) {

		SimulateMotors(pid_population[population_num].pid,
				pid_population[population_num].sync_pid,
				pid_population[population_num].integral_preload,
				testParam[population_num].initial_speed,
				testParam[population_num].target_speed,
				testParam[population_num].simulation_time);

	}

}

void SimulateMotors(PID_Coefficients pid_coeffs,
		PID_Coefficients pid_sync_coeffs, float integral_preload,
		uint32_t initial_speed, uint32_t target_speed, uint32_t simulation_time) {

	uint64_t current_time = 0;
	uint64_t old_time1 = 0, old_time2 = 0, old_time3 = 0;
	float dt1 = 0, dt2 = 0, dt3 = 0; // dt1-simulation_time checker.dt2-motor1_pid_interval time checker, dt3-motor2_pid_interval time checker

	old_time1 = HAL_GetTick();
	old_time2 = HAL_GetTick();
	old_time3 = HAL_GetTick();

// Initialize motor model
	init_motor_state1(initial_speed);   // initial_speed in RPM
	init_motor_state2(initial_speed);   // initial_speed in RPM

	motor1_status.speed_target = target_speed;
	motor2_status.speed_target = target_speed;
//	motor1_status.position = 0;
//	motor2_status.position = 0;

#if TEST_MOTOR1_PID

	PID_Init(&pid_motor1, tuned_pid1, 0);
	PID_Init(&pid_motor2, tuned_pid2, 0);
	PID_Init(&pid_sync, tuned_sync_pid, tuned_integral_preload);

#elif TEST_MOTOR2_PID
	PID_Init(&pid_motor2, tuned_pid2, 0);
#elif TUNE_MOTOR1

	float speed_diff_error_sum = 0;

	PID_Init(&pid_motor1, pid_coeffs, 0);
	PID_Init(&pid_motor2, tuned_pid2, 0);
	PID_Init(&pid_sync, pid_sync_coeffs, integral_preload);

#elif TUNE_MOTOR2

	float error_sum = 0;

	const float w1 = 1.5;   //rise time penalty
	const float w2 = 0.75;  //settling time penalty
	const float w3 = 1e6;   // overshoot penalty
	const float w4 = 1.0;   // error weight penalty

	PID_Init(&pid_motor2, pid_coeffs, 0);
#endif
	init_performance_metrics(&metrics, initial_speed, target_speed);

	while (dt1 < simulation_time) {

		current_time = HAL_GetTick();
		dt1 = current_time - old_time1;
		dt2 = current_time - old_time2;
		dt3 = current_time - old_time3;

		if (dt2 >= MOTOR1_TUNE_INTERVAL) {
			old_time2 = current_time;

			// Calculate the speed and error for motor1
			motor1_status.speed_calc = update_bldc_model1(duty_cycle1, dt1);
			motor1_status.position += motor1_status.speed_calc * dt2;

			// Calculate the speed and error for motor2
			motor2_status.speed_calc = update_bldc_model2(duty_cycle2, dt1);
			motor2_status.position += motor2_status.speed_calc * dt2;

			duty_correction1 = PID_Compute(&pid_sync, motor2_status.speed_calc,
					motor1_status.speed_calc, MAX_SPEED, dt2);

			duty_cycle1 = PID_Compute(&pid_motor1, motor1_status.speed_target,
					motor1_status.speed_calc, MAX_SPEED, dt2)
					+ duty_correction1;

			// Calculate motor1 duty
			if (dt3 >= MOTOR2_TUNE_INTERVAL) {
				old_time3 = current_time;

				update_overshoot(&metrics, motor2_status.speed_calc,
						target_speed);
				update_rise_time(&metrics, motor2_status.speed_calc,
						target_speed, dt1);
				update_settling_time(&metrics, motor2_status.speed_calc,
						target_speed, dt1);

				duty_cycle2 = PID_Compute(&pid_motor2,
						motor2_status.speed_target, motor2_status.speed_calc,
						MAX_SPEED, dt3);
			}
#if TEST_MOTOR1_PID||TEST_MOTOR2_PID

#elif TUNE_MOTOR1
			speed_diff_error_sum += fabs(pid_sync.error);
#else
			error_sum += fabs(pid_motor2.error);
#endif

			SetSpeed(duty_cycle1);

#if PRINT_ALL
			printf("%f %f %f %f \n", motor1_status.speed_target, motor1_status.speed_calc, motor2_status.speed_calc,
					motor2_status.speed_calc - motor1_status.speed_calc);
#else
			printf("%f \n",
					motor2_status.speed_calc - motor1_status.speed_calc);
#endif

		}

	}

#if TEST_MOTOR1_PID||TEST_MOTOR2_PID

#elif TUNE_MOTOR1
	pid_population[population_num].fitness = 1 / (1 + speed_diff_error_sum);
#else
	pid_population[population_num].fitness =
			1
					/ (1
							+ w1
									* (metrics.rise_time >= MIN_RISE_TIME ?
											(metrics.rise_time - MIN_RISE_TIME) :
											1e6)
							+ w2
									* (metrics.settling_time
											>= MIN_SETTLING_TIME ?
											(metrics.settling_time
													- MIN_SETTLING_TIME) :
											1e6) + w3 * metrics.max_deviation
							+ w4 * error_sum);
#endif
}

void ExtractCommand() {

}

// Verification of tuned parameter
void ExecuteCommand() {

}

void SetSpeed() {

}

// Initialize PID
void PID_Init(PID_Controller *pid, PID_Coefficients pid_coeff,
		float FeedForward) {
	pid->kp = pid_coeff.Kp;
	pid->ki = pid_coeff.Ki;
	pid->kd = pid_coeff.Kd;
	pid->prev_measured = 0.0f;
	pid->integral = FeedForward;
	pid->filtered_deriv = 0.0f;
	pid->error = 0.0f;
}

// PID Compute
float PID_Compute(PID_Controller *pid, float setpoint, float measured,
		float max_value, float dt) {
	const float MAX_DERIVATIVE_OUTPUT = 0.5 * DUTY_MAX;
	const float MAX_INTEGRAL_OUTPUT = DUTY_MAX;

	pid->error = (setpoint - measured) / max_value;

	pid->proportional = pid->kp * pid->error * DUTY_MAX;

	pid->integral += pid->ki * pid->error * (dt / 1000.0f) * DUTY_MAX;
	if (pid->integral > MAX_INTEGRAL_OUTPUT) {
		pid->integral = MAX_INTEGRAL_OUTPUT;
	} else if (pid->integral < -MAX_INTEGRAL_OUTPUT) {
		pid->integral = -MAX_INTEGRAL_OUTPUT;
	}

// Derivative filtering for noise reduction
	float raw_deriv = (measured - pid->prev_measured) / (dt / 1000.0f);
	pid->filtered_deriv = 0.85f * pid->filtered_deriv + 0.15f * raw_deriv;
	pid->derivative = pid->kd * pid->filtered_deriv * DUTY_MAX / max_value;

// Apply hard limit to derivative output
	if (pid->derivative > MAX_DERIVATIVE_OUTPUT) {
		pid->derivative = MAX_DERIVATIVE_OUTPUT;
	} else if (pid->derivative < -MAX_DERIVATIVE_OUTPUT) {
		pid->derivative = -MAX_DERIVATIVE_OUTPUT;
	}

	float output = pid->proportional + pid->integral + pid->derivative;
	pid->prev_measured = measured;
	return output;
}

// Initialize performance metrics
void init_performance_metrics(PerformanceMetrics *metrics, float initial_speed,
		float target_speed) {
	memset(metrics, 0, sizeof(PerformanceMetrics));
	metrics->initial_speed = initial_speed;
	metrics->error_span = fabs(target_speed - initial_speed);
	metrics->rise_low_bound = initial_speed
			+ 0.05f * metrics->error_span
					* (target_speed > initial_speed ? 1 : -1);
	metrics->rise_high_bound = initial_speed
			+ 0.95f * metrics->error_span
					* (target_speed > initial_speed ? 1 : -1);

}

// Update overshoot/undershoot detection
void update_overshoot(PerformanceMetrics *metrics, float current_speed,
		float target_speed) {
	float current_deviation = 0;
	if (target_speed > metrics->initial_speed) {
		current_deviation = current_speed - target_speed;
	} else {
		current_deviation = target_speed - current_speed;
	}

	if (current_deviation > metrics->max_deviation) {
		metrics->max_deviation = current_deviation;
	}
}

// Update rise time detection
void update_rise_time(PerformanceMetrics *metrics, float current_speed,
		float target_speed, float current_time) {
	if (!metrics->rise_time) {
		if ((target_speed > metrics->initial_speed
				&& current_speed >= metrics->rise_high_bound)
				|| (target_speed < metrics->initial_speed
						&& current_speed <= metrics->rise_high_bound)) {
			if (metrics->rise_threshold_crossed) {
				metrics->rise_time = current_time;
			}
		} else if ((target_speed > metrics->initial_speed
				&& current_speed > metrics->rise_low_bound)
				|| (target_speed < metrics->initial_speed
						&& current_speed < metrics->rise_low_bound)) {
			metrics->rise_threshold_crossed = 1;
		}
	}
}

// Update settling time detection
void update_settling_time(PerformanceMetrics *metrics, float current_speed,
		float target_speed, float current_time) {
	if (metrics->rise_time && !metrics->settling_time) {
		float tolerance = fmaxf(fabs(target_speed) * 0.02f, 0.5f);

		if (fabs(current_speed - target_speed) <= tolerance) {
			if (!metrics->settling_start_flag) {
				metrics->settling_start_flag = 1;
				metrics->consecutive_settled_count = 1;
			} else {
				metrics->consecutive_settled_count++;
			}

			if (metrics->consecutive_settled_count >= 3) {
				metrics->settling_time = current_time - metrics->rise_time;
			}
		} else {
			metrics->settling_start_flag = 0;
			metrics->consecutive_settled_count = 0;
		}
	}
}
