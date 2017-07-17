/*
 * Copyright (C) Tom van Dijk
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/dead_reckoning/dead_reckoning.c"
 * @author Tom van Dijk
 * Estimate quadrotor position from IMU data.
 *
 * This is an implementation of the (linear fixed-gain) attitude and velocity
 * observer described in Leischman et al. 2014 "Quadrotors and accelerometers:
 * State estimation with an improved dynamic model".
 */

#include "modules/dead_reckoning/dead_reckoning.h"

#include "math/pprz_algebra_int.h"

#include "state.h"
#include "autopilot.h"
#include "generated/modules.h"
#include "subsystems/datalink/telemetry.h"

#include <stdio.h>

/* Tuning */
#ifndef DR_G
#define DR_G 9.81
#endif

#ifndef DR_MU_OVER_M
#define DR_MU_OVER_M 0.5
#endif
float dr_mu_over_m = DR_MU_OVER_M;

#ifndef DR_L
#define DR_L {{0.0, -0.1}, {0.1, 0.0}, {-0.53, 0.0}, {0.0, -0.53}}
#endif

/* Observer state */
struct dr_state_t {
	float phi;
	float theta;
	float u;
	float v;
};
static struct dr_state_t dr; // Note: all values initialized to 0.

/* Update functions */
static void propagate(struct Int32Rates * gyro, float dt);
static void measurement_accel(struct Int32Vect3 * accel);

/* Telemetry */
static float gyro_p;
static float gyro_q;
static float accel_x;
static float accel_y;
static void send_telemetry(struct transport_tx *trans, struct link_device *dev);

void dr_init(void) {
	register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_DEAD_RECKONING,
			send_telemetry);
}

void dr_periodic(void) {
	// Get sensor data
	// The data should NOT assume a body attitude (as that is part of the
	// filter here). According to the AHRS (rates) and INS (accel) sources,
	// it seems that these measurements have not been processed apart from
	// Reset if not in flight
	if (!autopilot_in_flight()) {
		dr.phi = 0;
		dr.theta = 0;
		dr.u = 0;
		dr.v = 0;
		return;
	}
	// bias/orientation corrections, which is ok.
	struct Int32Rates *rates = stateGetBodyRates_i();
	struct Int32Vect3 *accel = stateGetAccelBody_i();
	// Propagate
	propagate(rates, DR_PERIODIC_PERIOD);
	// Measurement update
	measurement_accel(accel);
}


static void propagate(struct Int32Rates * gyro, float dt) {
	printf("cb_gyro enter phi=%+.2f theta=%+.2f u=%+.2f v=%+.2f\n", dr.phi,
			dr.theta, dr.u, dr.v);
	// Propagate internal state
	dr.phi += RATE_FLOAT_OF_BFP(gyro->p) * dt;
	dr.theta += RATE_FLOAT_OF_BFP(gyro->q) * dt;
	dr.u += (-DR_G * dr.theta - dr_mu_over_m * dr.u) * dt;
	dr.v += (DR_G * dr.phi - dr_mu_over_m * dr.v) * dt;
	// Save gyro values for telemetry
	gyro_p = RATE_FLOAT_OF_BFP(gyro->p);
	gyro_q = RATE_FLOAT_OF_BFP(gyro->q);
	printf("cb_gyro leave phi=%+.2f theta=%+.2f u=%+.2f v=%+.2f\n",
			dr.phi, dr.theta, dr.u, dr.v);
}

static void measurement_accel(struct Int32Vect3 * accel) {
	printf("cb_accel enter phi=%+.2f theta=%+.2f u=%+.2f v=%+.2f\n", dr.phi,
			dr.theta, dr.u, dr.v);
	// Update with latest measurement
	// Find error in estimated accelerations
	float ax = ACCEL_FLOAT_OF_BFP(accel->x);
	float ay = ACCEL_FLOAT_OF_BFP(accel->y);
	float ax_est = -dr_mu_over_m * dr.u;
	float ay_est = -dr_mu_over_m * dr.v;
	float ax_error = ax - ax_est;
	float ay_error = ay - ay_est;
	printf("ax  = %+.2f   ay  = %+.2f\n", ax, ay);
	printf("axe = %+.2f   aye = %+.2f\n", ax_est, ay_est);
	// Update estimated state
	static const float L[4][2] = DR_L;
	dr.phi += L[0][0] * ax_error + L[0][1] * ay_error;
	dr.theta += L[1][0] * ax_error + L[1][1] * ay_error;
	dr.u += L[2][0] * ax_error + L[2][1] * ay_error;
	dr.v += L[3][0] * ax_error + L[3][1] * ay_error;
	// Save accel values for telemetry
	accel_x = ax;
	accel_y = ay;
	printf("cb_accel leave phi=%+.2f theta=%+.2f u=%+.2f v=%+.2f\n", dr.phi,
			dr.theta, dr.u, dr.v);
}

static void send_telemetry(struct transport_tx *trans, struct link_device *dev)
{
	struct FloatEulers *att = stateGetNedToBodyEulers_f();
	struct NedCoor_f *pos = stateGetPositionNed_f();
	struct NedCoor_f *vel = stateGetSpeedNed_f();
	pprz_msg_send_DEAD_RECKONING(trans, dev, AC_ID,
			&dr.phi, &dr.theta, &dr.u, &dr.v,
			&dr_mu_over_m,
			&gyro_p, &gyro_q, &accel_x, &accel_y,
			&att->phi, &att->theta, &att->psi,
			&pos->x, &pos->y,
			&vel->x, &vel->y);
}

