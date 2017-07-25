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
#include "firmwares/rotorcraft/stabilization.h"

#include "subsystems/abi.h"
#include "mcu_periph/sys_time.h"

#if DR_TEMP_BIAS
#include "boards/ardrone/navdata.h"
#include "temp_bias_ardrone_vh.h"
#endif

#include <stdio.h>

#ifndef DR_SEND_ABI
#define DR_SEND_ABI 0
#endif

#ifndef DR_VEL_ID
#define DR_VEL_ID ABI_BROADCAST
#endif

/* Tuning */
#ifndef DR_DRAG1
#define DR_DRAG1 0.80
#endif
float dr_drag1 = DR_DRAG1;

#ifndef DR_FILTER
#define DR_FILTER 0.1
#endif
float dr_filter = DR_FILTER;

#ifndef DR_BIAS_X
#define DR_BIAS_X 0.0
#endif
float dr_bias_x = DR_BIAS_X;

#ifndef DR_BIAS_Y
#define DR_BIAS_Y 0.0
#endif
float dr_bias_y = DR_BIAS_Y;

/* Observer state */
struct dr_state_t {
	struct FloatVect2 v;
	float psi;
} dr_state;

/* Calibration */
int dr_calibrate = 0;
static int dr_calibrate_prev = 0;
static int calibration_samples = 0;
static void calibrate(void);

/* Telemetry */
static float gyro_p;
static float gyro_q;
static float accel_x;
static float accel_y;
static void send_telemetry(struct transport_tx *trans, struct link_device *dev);

struct FloatVect2 dr_getBodyVel(void) {
	return dr_state.v;
}

float dr_getHeading(void) {
	return dr_state.psi;
}

void dr_init(void) {
	register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_DEAD_RECKONING,
			send_telemetry);
}

void dr_periodic(void) {
	// Zero estimator when landed
	if (!autopilot_in_flight()) {
		dr_state.v.x = 0;
		dr_state.v.y = 0;
		return;
	}
	// Calibrate if set by user
	if (dr_calibrate) {
		calibrate();
	}
	dr_calibrate_prev = dr_calibrate;

	// Estimate speed directly from accelerometer
	// Get new measurement
	struct Int32Vect3 *acc = stateGetAccelBody_i();
	float ax = ACCEL_FLOAT_OF_BFP(acc->x) + dr_bias_x;
	float ay = ACCEL_FLOAT_OF_BFP(acc->y) + dr_bias_y;
	// Correct bias with temperature model
#if DR_TEMP_BIAS
	uint16_t temp = navdata.measure.temperature_acc;
	ax -= bias_x(temp);
	ay -= bias_y(temp);
#endif
	// Get velocity measurement from drag
	float vx = -ax / dr_drag1;
	float vy = -ay / dr_drag1;
	// Low-pass filter
	dr_state.v.x += dr_filter * (vx - dr_state.v.x);
	dr_state.v.y += dr_filter * (vy - dr_state.v.y);

	// Output velocity estimate ABI message for use in INS
#if DR_SEND_ABI
	uint32_t stamp = get_sys_time_usec();
	AbiSendMsgVELOCITY_ESTIMATE(DR_VEL_ID, stamp, dr_state.v.x, dr_state.v.y, 0,
			0);
#endif
	// Output measurements to telemetry
	struct Int32Rates *gyro = stateGetBodyRates_i();
	gyro_p = RATE_FLOAT_OF_BFP(gyro->p);
	gyro_q = RATE_FLOAT_OF_BFP(gyro->q);
	accel_x = ACCEL_FLOAT_OF_BFP(acc->x);
	accel_y = ACCEL_FLOAT_OF_BFP(acc->y);

}

static void calibrate(void) {
	// Average attitude values and accelerometer readings during stationary
	// hover, in order to find their biases.
	struct Int32Vect3 *acc = stateGetAccelBody_i();
	struct FloatVect2 a_meas = {
			.x = ACCEL_FLOAT_OF_BFP(acc->x),
			.y = ACCEL_FLOAT_OF_BFP(acc->y)
	};
#if DR_TEMP_BIAS
	uint16_t temp = navdata.measure.temperature_acc;
	a_meas.x -= bias_x(temp);
	a_meas.y -= bias_y(temp);
#endif
	if (dr_calibrate_prev == 0) {
		// Just started new calibration
		calibration_samples = 0;
	}
	dr_bias_x = (dr_bias_x * calibration_samples - a_meas.x)
			/ (calibration_samples + 1);
	dr_bias_y = (dr_bias_y * calibration_samples - a_meas.y)
			/ (calibration_samples + 1);
	calibration_samples++;
}

static void send_telemetry(struct transport_tx *trans, struct link_device *dev)
{
	float dummy = 0;
	struct FloatEulers *att = stateGetNedToBodyEulers_f();
	struct NedCoor_f *pos = stateGetPositionNed_f();
	struct NedCoor_f *vel = stateGetSpeedNed_f();
	float ins_u, ins_v;
	ins_u = cos(att->psi) * vel->x + sin(att->psi) * vel->y;
	ins_v = -sin(att->psi) * vel->x + cos(att->psi) * vel->y;

	pprz_msg_send_DEAD_RECKONING(trans, dev, AC_ID,
			&dummy, &dummy,
			&dr_state.v.x, &dr_state.v.y,
			&dummy,
			&gyro_p, &gyro_q, &accel_x, &accel_y,
			&att->phi, &att->theta, &att->psi,
			&pos->x, &pos->y,
			&vel->x, &vel->y,
			&ins_u, &ins_v);
}

