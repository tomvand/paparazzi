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
#include "mcu_periph/sys_time.h"

#include <stdio.h>

// PD tuning
#ifndef VISUALHOMING_GUIDANCE_P
#define VISUALHOMING_GUIDANCE_P 0.30
#endif
#ifndef VISUALHOMING_GUIDANCE_D
#define VISUALHOMING_GUIDANCE_D 0.00
#endif
#ifndef VISUALHOMING_GUIDANCE_FILTER_GAIN
#define VISUALHOMING_GUIDANCE_FILTER_GAIN 0.20
#endif
#ifndef VISUALHOMING_GUIDANCE_MAX_BANK
#define VISUALHOMING_GUIDANCE_MAX_BANK 0.18 // rad
#endif

struct vh_guidance_tuning_t vh_guidance_tuning = {
		.Kp = VISUALHOMING_GUIDANCE_P,
		.Kd = VISUALHOMING_GUIDANCE_D,
		.Kf = VISUALHOMING_GUIDANCE_FILTER_GAIN };

static struct vh_guidance_cmd_t {
	float cmd_phi;		// Roll [rad]
	float cmd_theta;	// Pitch [rad]
	float cmd_psi;		// Yaw [rad]
	float vx; // Last estimated velocity [m/s]
	float vy; // Last estimated velocity [m/s]
} vh_cmd;

/**
 * Set new position error.
 *
 * This function estimates the current velocity using the change in position
 * error and updates the PD controller accordingly.
 * @param dx Position error [m]
 * @param dy Position error [m]
 */
void visualhoming_guidance_set_pos_error(float dx, float dy) {
	static uint32_t prev_ts = 0;
	static float prev_dx = 0;
	static float prev_dy = 0;

	// Estimate current velocity
	uint32_t now_ts = get_sys_time_usec();
	float dt = (now_ts - prev_ts) / 1.0e6;
	if (dt > 0.01 && prev_ts != 0) { // Prevent too small timesteps.
		vh_cmd.vx = vh_guidance_tuning.Kf * ((prev_dx - dx) / dt)
				+ (1 - vh_guidance_tuning.Kf) * vh_cmd.vx;
		vh_cmd.vy = vh_guidance_tuning.Kf * ((prev_dy - dy) / dt)
				+ (1 - vh_guidance_tuning.Kf) * vh_cmd.vy;
		prev_dx = dx;
		prev_dy = dy;
	}

	// Update PD controller
	visualhoming_guidance_set_PD(dx, dy, vh_cmd.vx, vh_cmd.vy);
}

/**
 * Set new position error and update PD controller. Unlike _set_pos_error(),
 * this function accepts external velocity estimates.
 * @param dx Position error [m]
 * @param dy Position error [m]
 * @param vx Current velocity [m/s]
 * @param vy Current velocity [m/s]
 */
void visualhoming_guidance_set_PD(float dx, float dy, float vx, float vy) {
	// Bounded P action
	vh_cmd.cmd_theta = -vh_guidance_tuning.Kp * dx;
	vh_cmd.cmd_phi = vh_guidance_tuning.Kp * dy;
	BoundAbs(vh_cmd.cmd_phi, VISUALHOMING_GUIDANCE_MAX_BANK);
	BoundAbs(vh_cmd.cmd_theta, VISUALHOMING_GUIDANCE_MAX_BANK);
	// D action
	vh_cmd.cmd_theta += vh_guidance_tuning.Kd * vx;
	vh_cmd.cmd_phi += -vh_guidance_tuning.Kd * vy;
	BoundAbs(vh_cmd.cmd_phi, VISUALHOMING_GUIDANCE_MAX_BANK);
	// Final bound
	BoundAbs(vh_cmd.cmd_theta, VISUALHOMING_GUIDANCE_MAX_BANK);
}

void visualhoming_guidance_set_heading_error(float dpsi __attribute__((unused))) {
	// XXX Fix strange rotation bug
//	vh_cmd.cmd_psi = stateGetNedToBodyEulers_f()->psi + dpsi;
//	// Normalize heading
//	while (vh_cmd.cmd_psi > 2 * M_PI)
//		vh_cmd.cmd_psi -= 2. * M_PI;
//	while (vh_cmd.cmd_psi < 0)
//		vh_cmd.cmd_psi += 2. * M_PI;
}

/**
 * Write attitude setpoints in NAV mode.
 */
void visualhoming_guidance_update_nav(void) {
	struct Int32Eulers rpy = {
			.theta = ANGLE_BFP_OF_REAL(vh_cmd.cmd_theta),
			.phi = ANGLE_BFP_OF_REAL(vh_cmd.cmd_phi),
			.psi = ANGLE_BFP_OF_REAL(vh_cmd.cmd_psi) };
	horizontal_mode = HORIZONTAL_MODE_ATTITUDE;
	// XXX Causes sharp attitude steps when changing to/from waypoint mode.
	// Maybe because stabilization_attitude_enter() is not called!
	// TODO Check if horizontal mode changes and reset stabilization by hand
	// at least when starting.
	nav_roll = rpy.phi;
	nav_pitch = rpy.theta;
}

void guidance_h_module_init(void) {
	// Do nothing
}

void guidance_h_module_enter(void) {
	// Set setpoints to zero
	vh_cmd.cmd_theta = 0;
	vh_cmd.cmd_phi = 0;
	vh_cmd.cmd_psi = stateGetNedToBodyEulers_f()->psi;
}

void guidance_h_module_read_rc(void) {
	// Do nothing
}

void guidance_h_module_run(bool in_flight) {
	struct Int32Eulers rpy = {
			.theta = ANGLE_BFP_OF_REAL(vh_cmd.cmd_theta),
			.phi = ANGLE_BFP_OF_REAL(vh_cmd.cmd_phi),
			.psi = ANGLE_BFP_OF_REAL(vh_cmd.cmd_psi) };
	stabilization_attitude_set_rpy_setpoint_i(&rpy);
	stabilization_attitude_run(in_flight);
}

