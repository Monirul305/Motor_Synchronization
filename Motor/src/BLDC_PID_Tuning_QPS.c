///*
// * BLDC.c
// *
// *  Created on: Feb 14, 2025
// *      Author: User
// */
//#include "BLDC_PID_Tuning_QPS.h"
//
//struct bldc_command motor_command;
//
//struct feedback_data_struct feedback_tx = { 0, 0, 0 };
//struct feedback_data_struct feedback_rx = { 0, 0, 0 };
//#if RIGHT_DRIVE
//PID_Coefficients tunned_pid = { 0.582898557, 5.96418381, 0.0122240232 };
//#else
//PID_Coefficients tunned_pid = { 0.767210543, 6.60639286, 0.013410476 };
//PID_Coefficients tunned_sync_pid = { 0, 0, 0 };
//#endif
//
//uint32_t time_old = 0, time_new, time_passed = 0;
//
//Particle pid_population[POPULATION_SIZE];
//Particle best_population;
//testData testParam[POPULATION_SIZE];
//fitnessInputs fitness_param[POPULATION_SIZE];
//
//float duty_cycle;
//uint64_t current_time, previous_time, dt;
//
//float proportional = 0, integral = 0, differential = 0;
//int generation_num, population_num;
//
//// Global motor state
//MotorState motor_state = { 0 };
//
//// Main tuning function for QPSO-based PID tuning
//void QPSO_PID_Tuning(void) {
//	UART_Comm_Init(&huart1);
//	printf("Tuning started\n");
//
//	initialize_population();
//	initialize_testParam();
//
//	for (generation_num = 0; generation_num < GENERATIONS; generation_num++) {
//		evaluate_population();
//		get_best_individual();
//		update_particle_positions();
//	}
//
//	printf("Kp: %f , Ki: %f , Kd: %f\n", best_population.Kp, best_population.Ki, best_population.Kd);
//}
//
//// Initialize the PID population
//void initialize_population(void) {
//	srand(HAL_GetTick());  // Seed the random number generator
//	for (int population_num = 0; population_num < POPULATION_SIZE; population_num++) {
//		pid_population[population_num].Kp = ((float) rand() / RAND_MAX) * 0.75f;
//		pid_population[population_num].Ki = ((float) rand() / RAND_MAX) * 5.5f;
//		pid_population[population_num].Kd = ((float) rand() / RAND_MAX) * 0.05f;
//		pid_population[population_num].fitness = 0;
//		pid_population[population_num].best_Kp = pid_population[population_num].Kp;
//		pid_population[population_num].best_Ki = pid_population[population_num].Ki;
//		pid_population[population_num].best_Kd = pid_population[population_num].Kd;
//		pid_population[population_num].best_fitness = FLT_MIN;
//	}
//}
//
//// Initialize the test parameters (target speed, simulation time)
//void initialize_testParam(void) {
//	for (int i = 0; i < POPULATION_SIZE; i++) {
//		testParam[i].target_speed = 3000;
//		testParam[i].simulation_time = 5000;
//	}
//}
//
//// Evaluate the population's fitness
//void evaluate_population(void) {
//	for (population_num = 0; population_num < POPULATION_SIZE; population_num++) {
//		simulate_motor_behavior(pid_population[population_num], testParam[population_num].target_speed,
//				testParam[population_num].simulation_time);
//	}
//}
//
//// Get the best individual (best PID coefficients) in the population
//void get_best_individual(void) {
//	Particle best_pid = pid_population[0];
//	for (int population_num = 1; population_num < POPULATION_SIZE; population_num++) {
//		if (pid_population[population_num].fitness > best_pid.fitness) {
//			best_pid = pid_population[population_num];
//		}
//	}
//	best_population = best_pid;
//}
//
//// Update particle positions based on quantum-inspired rules
//void update_particle_positions(void) {
//	for (int population_num = 0; population_num < POPULATION_SIZE; population_num++) {
//		Particle *p = &pid_population[population_num];
//
//		// Update the particle's position using a quantum-inspired update rule
//		float delta_Kp = best_population.Kp - p->Kp;
//		float delta_Ki = best_population.Ki - p->Ki;
//		float delta_Kd = best_population.Kd - p->Kd;
//
//		// Apply quantum-inspired probabilistic update
//		float quantum_factor = (float) rand() / RAND_MAX;
//		p->Kp += delta_Kp * quantum_factor;
//		p->Ki += delta_Ki * quantum_factor;
//		p->Kd += delta_Kd * quantum_factor;
//
//		// Ensure the PID parameters stay within limits
//		p->Kp = fmaxf(PID_MIN_VAL, fminf(PID_MAX_VAL_KP, p->Kp));
//		p->Ki = fmaxf(PID_MIN_VAL, fminf(PID_MAX_VAL_KI, p->Ki));
//		p->Kd = fmaxf(PID_MIN_VAL, fminf(PID_MAX_VAL_KD, p->Kd));
//
//		// Update the best fitness for each particle
//		p->fitness = calculate_fitness(fitness_param[population_num].rise_time, fitness_param[population_num].settling_time,
//				fitness_param[population_num].max_deviation, fitness_param[population_num].error_sum,
//				fitness_param[population_num].osc_count);
//		if (p->fitness > p->best_fitness) {
//			p->best_Kp = p->Kp;
//			p->best_Ki = p->Ki;
//			p->best_Kd = p->Kd;
//			p->best_fitness = p->fitness;
//		}
//	}
//}
//
//// Simulate the motor behavior for the given PID coefficients
//void simulate_motor_behavior(Particle pid_coeffs, uint32_t target_speed, uint32_t simulation_time) {
//	float error_sum = 0;
//	float error = 0;
//
//	uint64_t current_time2 = 0;
//	uint64_t old_time1 = 0, old_time2 = 0;
//	float dt1 = 0, dt2 = 0;
//
//	uint32_t pid_adjust_time = 30;
//
//	// Initialize these before your simulation loop
//	static float prev_speed = 0;
//	static float filtered_deriv = 0;
//	const float MAX_DERIVATIVE_OUTPUT = 0.5 * DUTY_MAX;  // MAX_DERIVATIVE_OUTPUT <= 0.25 * MAX_INTEGRAL_OUTPUT
//	const float MAX_INTEGRAL_OUTPUT = DUTY_MAX; // MAX_INTEGRAL_OUTPUT + MAX_DERIVATIVE_OUTPUT <= 0.7 * duty_max
//	const float top_speed = 3300;
//
//	float initial_speed = 0;  // Capture initial speed when target is set
//	float max_deviation = 0;
//	int rise_threshold_crossed = 0;
//	int settling_start_flag = 0;
//	int consecutive_settled_count = 0;
//	int rise_time = 0, settling_time = 0;
//
//	// Add oscillation detection variables
//	int prev_error_sign = 0;       // Tracks previous error sign: 0=unset, 1=positive, -1=negative
//	int osc_count = 0;             // Counts significant oscillations
//	const float OSC_TOLERANCE = 0.02f; // 1% tolerance (normalized error)
//
//	old_time1 = HAL_GetTick();
//	old_time2 = HAL_GetTick();
//
//	integral = 0;
//
//	// Initialize motor model
//	init_motor_state(initial_speed);  // initial_speed in RPM
//
//	while (dt1 < simulation_time) {
//		dt1 = HAL_GetTick() - old_time1;
//
//		current_time2 = HAL_GetTick();
//		dt2 = current_time2 - old_time2;
//
//		if (dt2 >= pid_adjust_time) {
//			old_time2 = current_time2;
//
//			// Update motor model with current duty cycle
//			feedback_tx.speed_calc = update_bldc_model(duty_cycle, dt1);
//
//			feedback_tx.speed_target = target_speed;
//
//			// 1. Overshoot/undershoot calculation for both step-up and step-down
//			float current_deviation = 0;
//			if (feedback_tx.speed_target > initial_speed) {
//				// Step-up case: only care about overshoot above target
//				current_deviation = feedback_tx.speed_calc - feedback_tx.speed_target;
//			} else {
//				// Step-down case: only care about undershoot below target
//				current_deviation = feedback_tx.speed_target - feedback_tx.speed_calc;
//			}
//
//			if (current_deviation > max_deviation) {
//				max_deviation = current_deviation;
//			}
//
//			// 2. Fixed rise time detection (10% to 90% band)
//			float error_span = fabs(feedback_tx.speed_target - initial_speed);
//			float rise_low_bound = initial_speed + 0.1f * error_span * (feedback_tx.speed_target > initial_speed ? 1 : -1);
//			float rise_high_bound = initial_speed + 0.9f * error_span * (feedback_tx.speed_target > initial_speed ? 1 : -1);
//
//			if (!rise_time) {
//				if ((feedback_tx.speed_target > initial_speed && feedback_tx.speed_calc >= rise_high_bound)
//						|| (feedback_tx.speed_target < initial_speed && feedback_tx.speed_calc <= rise_high_bound)) {
//					if (rise_threshold_crossed) {
//						rise_time = dt1;
//					}
//				} else if ((feedback_tx.speed_target > initial_speed && feedback_tx.speed_calc > rise_low_bound)
//						|| (feedback_tx.speed_target < initial_speed && feedback_tx.speed_calc < rise_low_bound)) {
//					rise_threshold_crossed = 1;
//				}
//			}
//
//			// 3. Robust settling time detection
//			if (rise_time != 0 && !settling_time) {
//				// Dynamic tolerance (2% of target or absolute minimum of 0.5)
//				float tolerance = fmaxf(fabs(feedback_tx.speed_target) * 0.02f, 0.5f);
//
//				if (fabs(feedback_tx.speed_calc - feedback_tx.speed_target) <= tolerance) {
//					if (!settling_start_flag) {
//						settling_start_flag = 1;
//						consecutive_settled_count = 1;
//					} else {
//						consecutive_settled_count++;
//					}
//
//					if (consecutive_settled_count >= 10) {  // Require 10 consecutive samples
//						settling_time = dt1 - rise_time;
//					}
//				} else {
//					settling_start_flag = 0;
//					consecutive_settled_count = 0;
//				}
//			}
//
//			// --- Begin Oscillation Detection ---
//			// Only monitor after rise time (during settling/steady-state)
//			if (rise_time != 0) {
//				// Determine current error region (with hysteresis tolerance)
//				int current_sign = 0;
//				if (error > OSC_TOLERANCE)
//					current_sign = 1;
//				else if (error < -OSC_TOLERANCE)
//					current_sign = -1;
//
//				// Detect valid zero-crossings
//				if (prev_error_sign != 0 && current_sign != 0) {
//					if ((prev_error_sign == 1 && current_sign == -1) || (prev_error_sign == -1 && current_sign == 1)) {
//						osc_count++;  // Count as one oscillation
//					}
//				}
//
//				// Update sign tracker (ignore near-zero regions)
//				if (current_sign != 0) {
//					prev_error_sign = current_sign;
//				}
//			}
//			// --- End Oscillation Detection ---
//
//			// Calculate the error between target speed and simulated speed
//			error = ((float) feedback_tx.speed_target - (float) feedback_tx.speed_calc) / top_speed;
//			error = check_error_limit(error);
//			feedback_tx.error = error;
//
//			// Calculate PID components
//			proportional = pid_coeffs.Kp * error * DUTY_MAX;
//			integral += pid_coeffs.Ki * error * DUTY_MAX * (dt2 / 1000.0f);
//			if (integral > MAX_INTEGRAL_OUTPUT) {
//				integral = MAX_INTEGRAL_OUTPUT;
//			} else if (integral < -MAX_INTEGRAL_OUTPUT) {
//				integral = -MAX_INTEGRAL_OUTPUT;
//			}
//
//			// Derivative filtering for noise reduction
//			float raw_deriv = (feedback_tx.speed_calc - prev_speed) / (dt2 / 1000.0f);
//			filtered_deriv = 0.85f * filtered_deriv + 0.15f * raw_deriv;
//			prev_speed = feedback_tx.speed_calc;
//			float speed_normalized = fabs(filtered_deriv) / top_speed;
//			differential = -pid_coeffs.Kd * speed_normalized * DUTY_MAX;
//			// Apply hard limit to derivative output
//			if (differential > MAX_DERIVATIVE_OUTPUT) {
//				differential = MAX_DERIVATIVE_OUTPUT;
//			} else if (differential < -MAX_DERIVATIVE_OUTPUT) {
//				differential = -MAX_DERIVATIVE_OUTPUT;
//			}
//
//			duty_cycle = proportional + integral + differential;
//
//			duty_cycle = check_duty_limit(duty_cycle);
//
//			set_speed(duty_cycle);
//			// Accumulate error for fitness calculation
//			error_sum += fabs(error);
//			printf("%.1f %.1f\n", feedback_tx.speed_target, feedback_tx.speed_calc);
//
//		}
//	}
//
//	fitness_param[population_num].error_sum = error_sum;
//	fitness_param[population_num].max_deviation = max_deviation;
//	fitness_param[population_num].osc_count = osc_count;
//	fitness_param[population_num].rise_time = rise_time;
//	fitness_param[population_num].settling_time = settling_time;
//
//}
//
//// Fitness calculation function (based on your error calculation)
//float calculate_fitness(float rise_time, float settling_time, float max_deviation, float error_sum, int osc_count) {
//	return 1 / (1 + W1 * (rise_time != 0 ? rise_time : 1e6) + W2 * (settling_time != 0 ? settling_time : 1e6) +
//	W3 * max_deviation + W4 * error_sum + W5 * osc_count);
//}
//
//void ExtractCommand() {
//
//	uint32_t time_slot = 10000;
//	time_new = HAL_GetTick();  // Capture the current time
//	time_passed = time_new - time_old; // Calculate elapsed time
//
//	if (time_passed < time_slot) {
//		motor_command.current_speed = 2500;
//	} else if (time_passed >= time_slot && time_passed < 2 * time_slot) {
//		motor_command.current_speed = 500;
//	} else if (time_passed >= 2 * time_slot && time_passed < 3 * time_slot) {
//		motor_command.current_speed = 2900;
//	} else if (time_passed >= 3 * time_slot && time_passed < 4 * time_slot) {
//		motor_command.current_speed = 300;
//	} else {
//		time_old = time_new;
//	}
//
//}
//
//// Verification of tunned parameter
//void ExecuteCommand() {
//	float error;
//	uint64_t current_time2 = 0;
//	uint64_t old_time1 = 0;
//	uint64_t old_time2 = 0;
//	float dt1 = 0;
//	float dt2 = 0;
//	uint32_t pid_adjust_time = 30;
//
//	// Initialize these before your simulation loop
//	static float prev_speed = 0;
//	static float filtered_deriv = 0;
//	const float MAX_DERIVATIVE_OUTPUT = 0.5 * DUTY_MAX;  // MAX_DERIVATIVE_OUTPUT <= 0.25 * MAX_INTEGRAL_OUTPUT
//	const float MAX_INTEGRAL_OUTPUT = DUTY_MAX; // MAX_INTEGRAL_OUTPUT + MAX_DERIVATIVE_OUTPUT <= 0.7 * duty_max
//	const float top_speed = 3300;
//
//	float initial_speed = update_bldc_model(duty_cycle, dt1);  // Capture initial speed when target is set
//
//	old_time1 = HAL_GetTick();
//	old_time2 = HAL_GetTick();
//	ExtractCommand();
//	prev_speed = motor_command.current_speed;
//
//	init_motor_state(initial_speed);  // initial_speed in RPM
//
//	while (motor_command.current_speed) {
//
//		ExtractCommand();
//		dt1 = HAL_GetTick() - old_time1;
//
//		current_time2 = HAL_GetTick();
//		dt2 = current_time2 - old_time2;
//
//		if (dt2 >= pid_adjust_time) {
//			old_time2 = current_time2;
//
//			// Update motor model with current duty cycle
//			feedback_tx.speed_calc = update_bldc_model(duty_cycle, dt1);
//
//			feedback_tx.speed_target = motor_command.current_speed;
//
//			// Calculate the error between target speed and simulated speed
//			error = ((float) feedback_tx.speed_target - (float) feedback_tx.speed_calc) / top_speed;
//			error = check_error_limit(error);
//			feedback_tx.error = error;
//
//			// Calculate PID components
//			proportional = tunned_pid.Kp * error * DUTY_MAX;
//			integral += tunned_pid.Ki * error * DUTY_MAX * (dt2 / 1000.0f);
//			if (integral > MAX_INTEGRAL_OUTPUT) {
//				integral = MAX_INTEGRAL_OUTPUT;
//			} else if (integral < -MAX_INTEGRAL_OUTPUT) {
//				integral = -MAX_INTEGRAL_OUTPUT;
//			}
//
//			// Derivative filtering for noise reduction
//			float raw_deriv = (feedback_tx.speed_calc - prev_speed) / (dt2 / 1000.0f);
//			filtered_deriv = 0.85f * filtered_deriv + 0.15f * raw_deriv;
//			prev_speed = feedback_tx.speed_calc;
//			float speed_normalized = fabs(filtered_deriv) / top_speed;
//			differential = -tunned_pid.Kd * speed_normalized * DUTY_MAX;
//			// Apply hard limit to derivative output
//			if (differential > MAX_DERIVATIVE_OUTPUT) {
//				differential = MAX_DERIVATIVE_OUTPUT;
//			} else if (differential < -MAX_DERIVATIVE_OUTPUT) {
//				differential = -MAX_DERIVATIVE_OUTPUT;
//			}
//
//			duty_cycle = proportional + integral + differential;
//
//			duty_cycle = check_duty_limit(duty_cycle);
//
//			set_speed(duty_cycle);
//			// Accumulate error for fitness calculation
//
//			printf("%.1f %.1f\n", feedback_tx.speed_target, feedback_tx.speed_calc);
//
//		}
//
//	}
//}
//
//float check_error_limit(float error) {
//	float max_error = 1;
//	if (error > max_error) {
//		error = max_error;
//	} else if (error < -max_error) {
//		error = -max_error;
//	}
//	return error;
//}
//
//float check_duty_limit(float duty_cycle) {
//// Ensure duty stays within realistic limits
//	if (duty_cycle > DUTY_MAX) {
//		duty_cycle = DUTY_MAX;
//	} else if (duty_cycle < 0) {
//		duty_cycle = 0;
//	}
//	return duty_cycle;
//}
//
//void set_speed(float duty_cycle) {
//
//}
//
//void TestMaxSpeed() {
//	init_motor_state(0);
//	float duty = 100.0f;
//
//	for (int i = 0; i < 100; i++) {
//		uint32_t current_time_ms = HAL_GetTick();
//		uint32_t rpm = update_bldc_model(duty, current_time_ms);
//		printf("Time: %dms, RPM: %lu\n", i * 50, rpm);
//		HAL_Delay(50);
//	}
//}
//
//void VerifyBaseSpeed() {
//	init_motor_state(0);
//	for (int duty = 10; duty <= 100; duty += 10) {
//		uint32_t rpm = update_bldc_model(duty, HAL_GetTick());
//		printf("Duty: %d%%, RPM: %lu\n", duty, rpm);
//		HAL_Delay(1000);  // Allow stabilization
//	}
//}
//
//// Add this helper function
//void calculate_commutation(float theta_elec, float *c_a, float *c_b, float *c_c) {
//	theta_elec = fmodf(theta_elec, 2 * M_PI);
//	int sector = (int) (theta_elec / (M_PI / 3)) % 6;
//	switch (sector) {
//	case 0:
//		*c_a = 1;
//		*c_b = -1;
//		*c_c = 0;
//		break;
//	case 1:
//		*c_a = 1;
//		*c_b = 0;
//		*c_c = -1;
//		break;
//	case 2:
//		*c_a = 0;
//		*c_b = 1;
//		*c_c = -1;
//		break;
//	case 3:
//		*c_a = -1;
//		*c_b = 1;
//		*c_c = 0;
//		break;
//	case 4:
//		*c_a = -1;
//		*c_b = 0;
//		*c_c = 1;
//		break;
//	case 5:
//		*c_a = 0;
//		*c_b = -1;
//		*c_c = 1;
//		break;
//	default:
//		*c_a = 0;
//		*c_b = 0;
//		*c_c = 0;
//		break;
//	}
//}
//
//// Update motor model with new duty cycle
//uint32_t update_bldc_model(float duty, uint64_t timestamp) {
//// Calculate total time step in seconds
//	float dt_total = (timestamp - motor_state.last_update) / 1000.0f;  // ms to seconds
//	motor_state.last_update = timestamp;
//
//// Convert duty cycle to DC bus voltage
//	float v_dc = duty * SUPPLY_VOLTAGE / DUTY_MAX;
//
//// Sub-stepping for stability
//	const float max_sub_dt = 0.001f;  // 1ms max sub-step
//	int steps = (int) (dt_total / max_sub_dt) + 1;
//	float sub_dt = dt_total / steps;
//
//	for (int i = 0; i < steps; i++) {
//		// Calculate electrical angle (radians)
//		motor_state.elec.theta_elec = POLE_PAIRS * motor_state.mech.theta_mech;
//		motor_state.elec.theta_elec = fmodf(motor_state.elec.theta_elec, 2 * M_PI);
//
//		// Get commutation pattern
//		float c_a, c_b, c_c;
//		calculate_commutation(motor_state.elec.theta_elec, &c_a, &c_b, &c_c);
//
//		// Field weakening calculation
//		float omega_mech = motor_state.mech.omega;
//		float base_speed_rads = BASE_SPEED * 0.104719755f;
//		float fw_factor = 1.0f;
//
//		if (omega_mech > base_speed_rads) {
//			fw_factor = fmaxf(MIN_FW_FACTOR, base_speed_rads / omega_mech);
//		}
//
//		// Calculate back-EMF with field weakening
//		float e_a = BACK_EMF_CONST * omega_mech * c_a * fw_factor;
//		float e_b = BACK_EMF_CONST * omega_mech * c_b * fw_factor;
//		float e_c = BACK_EMF_CONST * omega_mech * c_c * fw_factor;
//
//		// Apply correct phase voltages (phase-to-neutral)
//		motor_state.elec.v_a = c_a * (v_dc / 2.0f);
//		motor_state.elec.v_b = c_b * (v_dc / 2.0f);
//		motor_state.elec.v_c = c_c * (v_dc / 2.0f);
//
//		// Update phase currents with current limiting
//		float effective_max_current = MAX_PHASE_CURRENT;
//		if (fabsf(omega_mech) > 250.0f) {
//			float speed_ratio = (fabsf(omega_mech) - 250.0f) / 100.0f;
//			effective_max_current *= fmaxf(1.0f - speed_ratio * 0.05f, 0.7f);
//		}
//
//		// Update phase current
//		motor_state.elec.i_a += (motor_state.elec.v_a - PHASE_RESISTANCE * motor_state.elec.i_a - e_a) / PHASE_INDUCTANCE
//				* sub_dt;
//		motor_state.elec.i_b += (motor_state.elec.v_b - PHASE_RESISTANCE * motor_state.elec.i_b - e_b) / PHASE_INDUCTANCE
//				* sub_dt;
//		motor_state.elec.i_c += (motor_state.elec.v_c - PHASE_RESISTANCE * motor_state.elec.i_c - e_c) / PHASE_INDUCTANCE
//				* sub_dt;
//
//		// Current limiting implementation
//		motor_state.elec.i_a = fmaxf(fminf(motor_state.elec.i_a, effective_max_current), -effective_max_current);
//		motor_state.elec.i_b = fmaxf(fminf(motor_state.elec.i_b, effective_max_current), -effective_max_current);
//		motor_state.elec.i_c = fmaxf(fminf(motor_state.elec.i_c, effective_max_current), -effective_max_current);
//
//		// Torque calculation with field weakening
//		motor_state.mech.torque = TORQUE_CONST * fw_factor
//				* (motor_state.elec.i_a * c_a + motor_state.elec.i_b * c_b + motor_state.elec.i_c * c_c);
//
//		// Enhanced friction model
//		float sign = (omega_mech > 0) ? 1.0f : ((omega_mech < 0) ? -1.0f : 0);
//		float friction_torque = VISCOUS_DAMPING * omega_mech + COULOMB_FRICTION * sign;
//
//		// Update mechanical state
//		float net_torque = motor_state.mech.torque - friction_torque;
//
//		float acceleration = net_torque / ROTOR_INERTIA;
//		motor_state.mech.omega += acceleration * sub_dt;
//
//		// Prevent numerical instability at near-zero speeds
//		if (fabsf(motor_state.mech.omega) < 0.5f && fabsf(net_torque) < 0.005f) {
//			motor_state.mech.omega = 0;
//		}
//
//		// Update rotor position
//		motor_state.mech.theta_mech += motor_state.mech.omega * sub_dt;
//	}
//
//// Convert to RPM with reasonable limits
//	float rpm = fabsf(motor_state.mech.omega) * 9.5492968f;
//	return (uint32_t) rpm;
//}
//
//// Initialize motor state
//void init_motor_state(float initial_speed_rpm) {
//	memset(&motor_state, 0, sizeof(motor_state));
//	motor_state.mech.omega = initial_speed_rpm * 0.104719755f;
//	motor_state.last_update = HAL_GetTick();  // Store milliseconds directly
//// Initialize currents to zero
//	motor_state.elec.i_a = 0;
//	motor_state.elec.i_b = 0;
//	motor_state.elec.i_c = 0;
//}
