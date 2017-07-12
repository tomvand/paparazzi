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
#include "subsystems/abi.h"

/* ABI IDs */
#ifndef DR_GYRO_ID
#define DR_GYRO_ID ABI_BROADCAST
#endif
abi_event ev_gyro;
static void cb_gyro(uint8_t sender_id, uint32_t stamp, struct Int32Rates * gyro);

#ifndef DR_ACCEL_ID
#define DR_ACCEL_ID ABI_BROADCAST
#endif
abi_event ev_accel;
static void cb_accel(
		uint8_t sender_id,
		uint32_t stamp,
		struct Int32Vect3 * accel);

/* Tuning */
#ifndef DR_G
#define DR_G 9.81
#endif

#ifndef DR_MU_OVER_M
#define DR_MU_OVER_M 0.5
#endif
float dr_mu_over_m = DR_MU_OVER_M;

#ifndef DR_L
#define DR_L {{0.0, 1.0}, {-1.0, 0.0}, {7.3435, 0.0}, {0.0, 7.3435}}
#endif

/* Observer state */
struct dr_state_t {
	float phi;
	float theta;
	float u;
	float v;
	uint32_t last_gyro_ts;
};
static struct dr_state_t dr; // Note: all values initialized to 0.

void dr_init(void) {
	AbiBindMsgIMU_GYRO_INT32(DR_GYRO_ID, &ev_gyro, cb_gyro);
	AbiBindMsgIMU_ACCEL_INT32(DR_ACCEL_ID, &ev_accel, cb_accel);
}

void cb_gyro(uint8_t sender_id, uint32_t stamp, struct Int32Rates * gyro) {
	// Propagate internal state
	float dt = (float)(stamp - dr.last_gyro_ts) / 1e6;
	if (dt < 0) return;
	dr.phi += RATE_FLOAT_OF_BFP(gyro->p) * dt;
	dr.theta += RATE_FLOAT_OF_BFP(gyro->q) * dt;
	dr.u += -DR_G * dr.theta - dr_mu_over_m * dr.u;
	dr.v += DR_G * dr.phi - dr_mu_over_m * dr.v;
	dr.last_gyro_ts = stamp;
}

void cb_accel(uint8_t sender_id, uint32_t stamp, struct Int32Vect3 * accel) {
	// Update with latest measurement
	// Find error in estimated accelerations
	float ax = ACCEL_FLOAT_OF_BFP(accel->x);
	float ay = ACCEL_FLOAT_OF_BFP(accel->y);
	float ax_est = -dr_mu_over_m * dr.u;
	float ay_est = -dr_mu_over_m * dr.v;
	float ax_error = ax - ax_est;
	float ay_error = ay - ay_est;
	// Update estimated state
	static const float L[4][2] = DR_L;
	dr.phi += L[0][0] * ax_error + L[0][1] * ay_error;
	dr.theta += L[1][0] * ax_error + L[1][1] * ay_error;
	dr.u += L[2][0] * ax_error + L[2][1] * ay_error;
	dr.v += L[3][0] * ax_error + L[3][1] * ay_error;
}

