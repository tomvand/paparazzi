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

#include <stdio.h>

#ifndef DR_VEL_ID
#define DR_VEL_ID ABI_BROADCAST
#endif

/* Tuning */
#ifndef DR_DRAG1
#define DR_DRAG1 0.9388
#endif
#ifndef DR_DRAG2
#define DR_DRAG2 0.0
#endif
float dr_drag[] = { DR_DRAG1, DR_DRAG2 };

#ifndef DR_THRUST
#define DR_THRUST 0.0011
#endif
float dr_thrust = DR_THRUST;

#ifndef DR_GAIN
#define DR_GAIN 0.0
#endif
float dr_gain = DR_GAIN;

#ifndef DR_BIAS
#define DR_BIAS { 0.0, 0.0, 0.0, 0.0 }
#endif
float dr_bias[] = DR_BIAS;

/* Observer state */
struct dr_state_t {
	struct FloatVect2 v;
	float psi;
} dr_state;

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
	if (!autopilot_in_flight()) {
		dr_state.v.x = 0;
		dr_state.v.y = 0;
		return;
	}
	// State observer with single gain and quadratic drag model
	// Timestep
	float dt = DR_PERIODIC_PERIOD;
	// Rotation
	struct FloatEulers *att = stateGetNedToBodyEulers_f();
	float new_psi = att->psi;
	float dpsi = new_psi - dr_state.psi;
	dr_state.psi = new_psi;
	struct FloatVect2 v_prev = dr_state.v;
	dr_state.v.x = cos(dpsi) * v_prev.x + sin(dpsi) * v_prev.y;
	dr_state.v.y = -sin(dpsi) * v_prev.x + cos(dpsi) * v_prev.y;
	// Quadratic drag model
	float speed = FLOAT_VECT2_NORM(dr_state.v);
	float Fdm;
	struct FloatVect2 dir;
	if (speed != 0) {
		Fdm = dr_drag[0] * speed + dr_drag[1] * speed * speed;
		dir.x = -dr_state.v.x / speed;
		dir.y = -dr_state.v.y / speed;
	} else {
		Fdm = 0;
		dir.x = 0;
		dir.y = 0;
	}
	// Acceleration
	float phi = att->phi + dr_bias[0];
	float theta = att->theta + dr_bias[1];
	float Ftm = stabilization_cmd[COMMAND_THRUST] * dr_thrust;
	// Time update
	dr_state.v.x += (Ftm * -theta + Fdm * dir.x) * dt;
	dr_state.v.y += (Ftm * phi + Fdm * dir.y) * dt;
	// Measurement update
	struct FloatVect2 a_pred = {
			.x = Fdm * dir.x,
			.y = Fdm * dir.y
	};
	struct Int32Vect3 *acc = stateGetAccelBody_i();
	struct FloatVect2 a_meas = {
			.x = ACCEL_FLOAT_OF_BFP(acc->x) + dr_bias[2],
			.y = ACCEL_FLOAT_OF_BFP(acc->y) + dr_bias[3]
	};
	dr_state.v.x += dr_gain * (a_meas.x - a_pred.x);
	dr_state.v.y += dr_gain * (a_meas.y - a_pred.y);

	// Output velocity estimate ABI message for use in INS
	uint32_t stamp = get_sys_time_usec();
	AbiSendMsgVELOCITY_ESTIMATE(DR_VEL_ID, stamp, dr_state.v.x, dr_state.v.y, 0,
			0);
	// Output measurements to telemetry
	struct Int32Rates *gyro = stateGetBodyRates_i();
	gyro_p = RATE_FLOAT_OF_BFP(gyro->p);
	gyro_q = RATE_FLOAT_OF_BFP(gyro->q);
	accel_x = a_meas.x;
	accel_y = a_meas.y;

}

static void send_telemetry(struct transport_tx *trans, struct link_device *dev)
{
	float dummy = 0;
	struct FloatEulers *att = stateGetNedToBodyEulers_f();
	struct NedCoor_f *pos = stateGetPositionNed_f();
	struct NedCoor_f *vel = stateGetSpeedNed_f();
	pprz_msg_send_DEAD_RECKONING(trans, dev, AC_ID,
			&dummy, &dummy,
			&dr_state.v.x, &dr_state.v.y,
			&dummy,
			&gyro_p, &gyro_q, &accel_x, &accel_y,
			&att->phi, &att->theta, &att->psi,
			&pos->x, &pos->y,
			&vel->x, &vel->y);
}

