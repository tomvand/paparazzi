/*
 * Copyright (C) Tom van Dijk
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

#include "visualhoming_guidance_h.h"

#include "firmwares/rotorcraft/navigation.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "firmwares/rotorcraft/guidance/guidance_v.h"
#include "firmwares/rotorcraft/autopilot_static.h"
#include "firmwares/rotorcraft/autopilot_guided.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
#include "mcu_periph/sys_time.h"

#include "modules/dead_reckoning/dead_reckoning.h"

#include "generated/modules.h"

#include <stdio.h>

#ifndef VH_MAX_YAW_RATE
#define VH_MAX_YAW_RATE 1.0
#endif

#ifndef VH_MAX_BANK
#define VH_MAX_BANK GUIDANCE_H_MAX_BANK
#endif

/** Default speed saturation (taken from guidance_h_ref.h) */
#ifndef GUIDANCE_H_REF_MAX_SPEED
#define GUIDANCE_H_REF_MAX_SPEED 5.
#endif
PRINT_CONFIG_MSG_VALUE("VH MAX SPEED ", GUIDANCE_H_REF_MAX_SPEED);

/** Accel saturation.
 * tanf(RadOfDeg(30.))*9.81 = 5.66
 */
#ifndef GUIDANCE_H_REF_MAX_ACCEL
#define GUIDANCE_H_REF_MAX_ACCEL 5.66
#endif
PRINT_CONFIG_MSG_VALUE("VH MAX ACCEL ", GUIDANCE_H_REF_MAX_ACCEL);

/* Misc */
static const float dt = (1.0 / MODULES_FREQUENCY);
static const float max_pos_error = 16.0;

/* Guidance state */
#ifndef VH_GUIDANCE_H_PGAIN
#define VH_GUIDANCE_H_PGAIN GUIDANCE_H_PGAIN
#endif
#ifndef VH_GUIDANCE_H_IGAIN
#define VH_GUIDANCE_H_IGAIN GUIDANCE_H_IGAIN
#endif
#ifndef VH_GUIDANCE_H_DGAIN
#define VH_GUIDANCE_H_DGAIN GUIDANCE_H_DGAIN
#endif

struct vh_guidance_t vh_guidance = {
		.gains.p = VH_GUIDANCE_H_PGAIN,
		.gains.i = VH_GUIDANCE_H_IGAIN,
		.gains.d = VH_GUIDANCE_H_DGAIN,
		.maneuver_time = 10.0
};

/* Guidance functions */
void vh_guidance_set_pos(float x, float y) {
	vh_guidance.sp.pos.x = x;
	vh_guidance.sp.pos.y = y;
	vh_guidance.sp.pos_set = TRUE;
	vh_guidance.sp.vel_set = FALSE;
}

void vh_guidance_set_pos_vel(float x, float y, float u, float v) {
	vh_guidance.sp.pos.x = x;
	vh_guidance.sp.pos.y = y;
	vh_guidance.sp.pos_set = TRUE;
	vh_guidance.sp.vel.x = u;
	vh_guidance.sp.vel.y = v;
	vh_guidance.sp.vel_set = TRUE;
}

void vh_guidance_set_vel(float u, float v) {
	vh_guidance.sp.pos.x = 0;
	vh_guidance.sp.pos.y = 0;
	vh_guidance.sp.pos_set = TRUE;
	vh_guidance.sp.vel.x = u;
	vh_guidance.sp.vel.y = v;
	vh_guidance.sp.vel_set = TRUE;
}

void vh_guidance_change_heading(float delta_psi) {
	vh_guidance.sp.heading = stateGetNedToBodyEulers_f()->psi + delta_psi;
}


void guidance_h_module_init(void) {
	// Do nothing
}

void guidance_h_module_enter(void) {
	vh_guidance.sp.vel.x = 0;
	vh_guidance.sp.vel.y = 0;
	vh_guidance.integrator.x = 0;
	vh_guidance.integrator.y = 0;
	vh_guidance.cmd.psi = stateGetNedToBodyEulers_f()->psi;
	vh_guidance.sp.heading = stateGetNedToBodyEulers_f()->psi;
}

void guidance_h_module_read_rc(void) {
	// Do nothing
}

// with a pgain of 100 and this scale,
// you get an angle of 5.6 degrees for 1m pos error
// See guidance_h.c
#define VH_GAIN_SCALE 1.0e-3

void guidance_h_module_run(bool in_flight) {
	// Get current velocities
	struct FloatVect2 vel = dr_getBodyVel();

	// Fill unset reference signals
	if (!vh_guidance.sp.pos_set && !vh_guidance.sp.vel_set) {
		vh_guidance.sp.pos.x = 0;
		vh_guidance.sp.pos.y = 0;
		vh_guidance.sp.vel.x = 0;
		vh_guidance.sp.vel.y = 0;
	} else if (!vh_guidance.sp.pos_set) {
		vh_guidance.sp.pos.x = vh_guidance.sp.vel.x * vh_guidance.maneuver_time;
		vh_guidance.sp.pos.y = vh_guidance.sp.vel.y * vh_guidance.maneuver_time;
	} else if (!vh_guidance.sp.vel_set) {
		float diff_x = vh_guidance.sp.pos.x / vh_guidance.maneuver_time
				- vh_guidance.sp.vel.x;
		float diff_y = vh_guidance.sp.pos.y / vh_guidance.maneuver_time
				- vh_guidance.sp.vel.y;
		float diff_magn = sqrt(diff_x * diff_x + diff_y * diff_y);
		float diff_max = (float)GUIDANCE_H_REF_MAX_ACCEL / MODULES_FREQUENCY;
		if (diff_magn > diff_max) {
			diff_x = diff_x / diff_magn * diff_max;
			diff_y = diff_y / diff_magn * diff_max;
		}
		vh_guidance.sp.vel.x += diff_x;
		vh_guidance.sp.vel.y += diff_y;
	}
	// Trim reference velocity
	float vel_magn = sqrt(vh_guidance.sp.vel.x * vh_guidance.sp.vel.x +
			vh_guidance.sp.vel.y * vh_guidance.sp.vel.y);
	if (vel_magn > GUIDANCE_H_REF_MAX_SPEED) {
		vh_guidance.sp.vel.x = vh_guidance.sp.vel.x
				/ vel_magn* GUIDANCE_H_REF_MAX_SPEED;
		vh_guidance.sp.vel.y = vh_guidance.sp.vel.y
				/ vel_magn* GUIDANCE_H_REF_MAX_SPEED;
	}
	// Trim reference position
	float pos_magn = sqrt(vh_guidance.sp.pos.x * vh_guidance.sp.pos.x +
			vh_guidance.sp.pos.y * vh_guidance.sp.pos.y);
	if (pos_magn > max_pos_error) {
		vh_guidance.sp.pos.x = vh_guidance.sp.pos.x
				/ pos_magn * max_pos_error;
		vh_guidance.sp.pos.y = vh_guidance.sp.pos.y
				/ pos_magn * max_pos_error;
	}

	// Compute velocity error
	float vel_err_x = vh_guidance.sp.vel.x - vel.x;
	float vel_err_y = vh_guidance.sp.vel.y - vel.y;

	// Run PD
	float pd_x = vh_guidance.gains.p * vh_guidance.sp.pos.x * VH_GAIN_SCALE +
			vh_guidance.gains.d * vel_err_x * VH_GAIN_SCALE;
	float pd_y = vh_guidance.gains.p * vh_guidance.sp.pos.y * VH_GAIN_SCALE +
			vh_guidance.gains.d * vel_err_y * VH_GAIN_SCALE;
	// Trim PD command
	float pd_magn = sqrt(pd_x * pd_x + pd_y * pd_y);
	if (pd_magn > VH_MAX_BANK / 2) {
		pd_x = pd_x / pd_magn * VH_MAX_BANK / 2;
		pd_y = pd_y / pd_magn * VH_MAX_BANK / 2;
	}

	// Update integrator
	if (in_flight) {
		// Update integrator
		vh_guidance.integrator.x += vh_guidance.gains.i * pd_x * VH_GAIN_SCALE
				* VH_GAIN_SCALE; // Scaled twice as in guidance_h.c
		vh_guidance.integrator.y += vh_guidance.gains.i * pd_y * VH_GAIN_SCALE
				* VH_GAIN_SCALE;
		// Trim
		float int_magn = sqrt(
				vh_guidance.integrator.x * vh_guidance.integrator.x
						+ vh_guidance.integrator.y * vh_guidance.integrator.y);
		if (int_magn > VH_MAX_BANK / 2) {
			vh_guidance.integrator.x = vh_guidance.integrator.x
					/ int_magn * VH_MAX_BANK / 2;
			vh_guidance.integrator.y = vh_guidance.integrator.y
					/ int_magn * VH_MAX_BANK / 2;
		}
		// Add to command
		pd_x += vh_guidance.integrator.x;
		pd_y += vh_guidance.integrator.y;
	} else {
		vh_guidance.integrator.x = 0;
		vh_guidance.integrator.y = 0;
	}

	// Trim setpoints
	float sp_magn = sqrt(pd_x * pd_x + pd_y * pd_y);
	if (sp_magn > VH_MAX_BANK) {
		pd_x = pd_x / sp_magn * VH_MAX_BANK;
		pd_y = pd_y / sp_magn * VH_MAX_BANK;
	}

	// Store setpoints
	vh_guidance.cmd.phi = pd_y;
	vh_guidance.cmd.theta = -pd_x;

	// Calculate yaw command
	float diff_yaw = vh_guidance.sp.heading - vh_guidance.cmd.psi;
	while (diff_yaw < -M_PI)
		diff_yaw += 2 * M_PI;
	while (diff_yaw > M_PI)
		diff_yaw -= 2 * M_PI;
	if (diff_yaw > VH_MAX_YAW_RATE * dt) diff_yaw = VH_MAX_YAW_RATE * dt;
	if (diff_yaw < -VH_MAX_YAW_RATE * dt) diff_yaw = -VH_MAX_YAW_RATE * dt;
	vh_guidance.cmd.psi += diff_yaw;

	// Run stabilization
	struct Int32Eulers rpy;
	EULERS_BFP_OF_REAL(rpy, vh_guidance.cmd);
	stabilization_attitude_set_rpy_setpoint_i(&rpy);
	stabilization_attitude_run(in_flight);

	// Update velocity setpoint
	if (vh_guidance.sp.pos_set) {
		vh_guidance.sp.pos.x -= vel.x * dt;
		vh_guidance.sp.pos.y -= vel.y * dt;
	}
}


/** Switch autopilot mode from flightplan
 *
 * @param ap_mode new autopilot mode.
 * @return FALSE
 */
bool SetAPMode(uint8_t ap_mode) {
	printf("Set autopilot to mode %d\n", ap_mode);
	autopilot_mode_auto2 = ap_mode;
	autopilot_set_mode(ap_mode);
	return FALSE; // Return immediately
}

/**
 * Check whether the visualhoming module is currently controlling the UAV.
 * @return TRUE if controlling, FALSE if not.
 */
int visualhoming_guidance_in_control(void) {
	return (guidance_h.mode == GUIDANCE_H_MODE_MODULE)
			|| (guidance_h.mode == GUIDANCE_H_MODE_GUIDED);
}

