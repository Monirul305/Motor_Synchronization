#include "motor1.h"

// Global motor state
MotorState motor_state = { 0 };


void VerifyBaseSpeed1() {
	init_motor_state1(0);
	for (float duty = 0; duty <= 93; duty += 0.5) {
		float rpm = update_bldc_model1(duty, HAL_GetTick());
		printf("%f,%f\n", duty, rpm);
		HAL_Delay(5);
	}
}

// Add this helper function
void calculate_commutation1(float theta_elec, float *c_a, float *c_b, float *c_c) {
	theta_elec = fmodf(theta_elec, 2 * M_PI);
	int sector = (int) (theta_elec / (M_PI / 3)) % 6;
	switch (sector) {
	case 0:
		*c_a = 1;
		*c_b = -1;
		*c_c = 0;
		break;
	case 1:
		*c_a = 1;
		*c_b = 0;
		*c_c = -1;
		break;
	case 2:
		*c_a = 0;
		*c_b = 1;
		*c_c = -1;
		break;
	case 3:
		*c_a = -1;
		*c_b = 1;
		*c_c = 0;
		break;
	case 4:
		*c_a = -1;
		*c_b = 0;
		*c_c = 1;
		break;
	case 5:
		*c_a = 0;
		*c_b = -1;
		*c_c = 1;
		break;
	default:
		*c_a = 0;
		*c_b = 0;
		*c_c = 0;
		break;
	}
}

// Update motor model with new duty cycle
uint32_t update_bldc_model1(float duty, uint64_t timestamp) {
// Calculate total time step in seconds
	float dt_total = (timestamp - motor_state.last_update) / 1000.0f;  // ms to seconds
	motor_state.last_update = timestamp;

// Convert duty cycle to DC bus voltage
	float v_dc = duty * SUPPLY_VOLTAGE / 100.0f;

// Sub-stepping for stability
	const float max_sub_dt = 0.001f;  // 1ms max sub-step
	int steps = (int) (dt_total / max_sub_dt) + 1;
	float sub_dt = dt_total / steps;

	for (int i = 0; i < steps; i++) {
		// Calculate electrical angle (radians)
		motor_state.elec.theta_elec = POLE_PAIRS * motor_state.mech.theta_mech;
		motor_state.elec.theta_elec = fmodf(motor_state.elec.theta_elec, 2 * M_PI);

		// Get commutation pattern
		float c_a, c_b, c_c;
		calculate_commutation1(motor_state.elec.theta_elec, &c_a, &c_b, &c_c);

		// Field weakening calculation
		float omega_mech = motor_state.mech.omega;
		float base_speed_rads = BASE_SPEED * 0.104719755f;
		float fw_factor = 1.0f;

		if (omega_mech > base_speed_rads) {
			fw_factor = fmaxf(MIN_FW_FACTOR, base_speed_rads / omega_mech);
		}

		// Calculate back-EMF with field weakening
		float e_a = BACK_EMF_CONST * omega_mech * c_a * fw_factor;
		float e_b = BACK_EMF_CONST * omega_mech * c_b * fw_factor;
		float e_c = BACK_EMF_CONST * omega_mech * c_c * fw_factor;

		// Apply correct phase voltages (phase-to-neutral)
		motor_state.elec.v_a = c_a * (v_dc / 2.0f);
		motor_state.elec.v_b = c_b * (v_dc / 2.0f);
		motor_state.elec.v_c = c_c * (v_dc / 2.0f);

		// Update phase currents with current limiting
		float effective_max_current = MAX_PHASE_CURRENT;
		if (fabsf(omega_mech) > 250.0f) {
			float speed_ratio = (fabsf(omega_mech) - 250.0f) / 100.0f;
			effective_max_current *= fmaxf(1.0f - speed_ratio * 0.05f, 0.7f);
		}

		// Update phase current
		motor_state.elec.i_a += (motor_state.elec.v_a - PHASE_RESISTANCE * motor_state.elec.i_a - e_a) / PHASE_INDUCTANCE
				* sub_dt;
		motor_state.elec.i_b += (motor_state.elec.v_b - PHASE_RESISTANCE * motor_state.elec.i_b - e_b) / PHASE_INDUCTANCE
				* sub_dt;
		motor_state.elec.i_c += (motor_state.elec.v_c - PHASE_RESISTANCE * motor_state.elec.i_c - e_c) / PHASE_INDUCTANCE
				* sub_dt;

		// Current limiting implementation
		motor_state.elec.i_a = fmaxf(fminf(motor_state.elec.i_a, effective_max_current), -effective_max_current);
		motor_state.elec.i_b = fmaxf(fminf(motor_state.elec.i_b, effective_max_current), -effective_max_current);
		motor_state.elec.i_c = fmaxf(fminf(motor_state.elec.i_c, effective_max_current), -effective_max_current);

		// Torque calculation with field weakening
		motor_state.mech.torque = TORQUE_CONST * fw_factor
				* (motor_state.elec.i_a * c_a + motor_state.elec.i_b * c_b + motor_state.elec.i_c * c_c);

		// Enhanced friction model
		float sign = (omega_mech > 0) ? 1.0f : ((omega_mech < 0) ? -1.0f : 0);
		float friction_torque = VISCOUS_DAMPING * omega_mech + COULOMB_FRICTION * sign;

		// Update mechanical state
		float net_torque = motor_state.mech.torque - friction_torque;

		float acceleration = net_torque / ROTOR_INERTIA;
		motor_state.mech.omega += acceleration * sub_dt;

		// Prevent numerical instability at near-zero speeds
		if (fabsf(motor_state.mech.omega) < 0.5f && fabsf(net_torque) < 0.005f) {
			motor_state.mech.omega = 0;
		}

		// Update rotor position
		motor_state.mech.theta_mech += motor_state.mech.omega * sub_dt;
	}

// Convert to RPM with reasonable limits
	float rpm = fabsf(motor_state.mech.omega) * 9.5492968f;
	return (uint32_t) rpm;
}

// Initialize motor state
void init_motor_state1(float initial_speed_rpm) {
	memset(&motor_state, 0, sizeof(motor_state));
	motor_state.mech.omega = initial_speed_rpm * 0.104719755f;
	motor_state.last_update = HAL_GetTick();  // Store milliseconds directly
// Initialize currents to zero
	motor_state.elec.i_a = 0;
	motor_state.elec.i_b = 0;
	motor_state.elec.i_c = 0;
}
