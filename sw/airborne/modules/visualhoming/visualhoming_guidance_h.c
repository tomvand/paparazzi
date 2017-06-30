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

#include <stdio.h>

// PD tuning
#ifndef VISUALHOMING_GUIDANCE_P
#define VISUALHOMING_GUIDANCE_P 1.00
#endif
#ifndef VISUALHOMING_GUIDANCE_TD
#define VISUALHOMING_GUIDANCE_TD 0.10
#endif
#ifndef VISUALHOMING_GUIDANCE_FILTER_GAIN
#define VISUALHOMING_GUIDANCE_FILTER_GAIN 1.00
#endif
#ifndef VISUALHOMING_GUIDANCE_MAX_BANK
#define VISUALHOMING_GUIDANCE_MAX_BANK 0.18 // rad
#endif
#ifndef VISUALHOMING_VECTOR_SATURATE
#define VISUALHOMING_VECTOR_SATURATE 0.10
#endif
#ifndef VISUALHOMING_CONSTANT_PITCH
#define VISUALHOMING_CONSTANT_PITCH 0.10 // rad
#endif
#ifndef VISUALHOMING_GUIDANCE_HEADING_RATE
#define VISUALHOMING_GUIDANCE_HEADING_RATE 10.0 // rad/s
#endif

struct vh_guidance_tuning_t vh_guidance_tuning = {
		.Kp = VISUALHOMING_GUIDANCE_P,
		.Td = VISUALHOMING_GUIDANCE_TD,
		.Kf = VISUALHOMING_GUIDANCE_FILTER_GAIN };

static struct vh_guidance_cmd_t {
	float cmd_phi;		// Roll [rad]
	float cmd_theta;	// Pitch [rad]
	float cmd_psi;		// Yaw [rad]
	float vx; // Last estimated velocity [m/s]
	float vy; // Last estimated velocity [m/s]
} vh_cmd;

/**
 * Set position setpoint for use in GUIDED mode.
 * @param dx
 * @param dy
 */
void visualhoming_guidance_set_pos_setpoint(float dx, float dy) {
//	autopilot_guided_goto_body_relative(10.0 * dx, 10.0 * dy, 0, 0);
	guidance_h_set_guided_body_vel(3.0 * dx, 3.0 * dy);
	guidance_v_from_nav(1);
}

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
	// Saturate homing vector
	float magn = sqrt(dx * dx + dy * dy);
	if (magn > VISUALHOMING_VECTOR_SATURATE) {
		dx = dx / magn * VISUALHOMING_VECTOR_SATURATE;
		dy = dy / magn * VISUALHOMING_VECTOR_SATURATE;
	}
	// Calculate desired angles
	float ax, ay;
	ax = vh_guidance_tuning.Kp * (dx - vh_guidance_tuning.Td * vx);
	ay = vh_guidance_tuning.Kp * (dy - vh_guidance_tuning.Td * vy);
	// Saturate angles
	magn = sqrt(ax * ax + ay * ay);
	if (magn > VISUALHOMING_GUIDANCE_MAX_BANK) {
		ax = ax / magn * VISUALHOMING_GUIDANCE_MAX_BANK;
		ay = ay / magn * VISUALHOMING_GUIDANCE_MAX_BANK;
	}
	// Write commands
	vh_cmd.cmd_theta = -ax;
	vh_cmd.cmd_phi = ay;
}

/**
 * Move with constant pitch/roll along the homing vector
 * @param dx
 * @param dy
 */
void visualhoming_guidance_set_constant_pitch(float dx, float dy) {
	float magn = sqrt(dx * dx + dy * dy);
	if (magn != 0) {
		vh_cmd.cmd_theta = -dx / magn * VISUALHOMING_CONSTANT_PITCH;
		vh_cmd.cmd_phi = dy / magn * VISUALHOMING_CONSTANT_PITCH;
	} else {
		vh_cmd.cmd_theta = 0;
		vh_cmd.cmd_phi = 0;
	}
}

void visualhoming_guidance_set_heading_rate(float dx, float dy, float dt) {
	float dpsi = visualhoming_guidance_point_at_homingvector(dx, dy);
	if (dpsi > -0.5 && dpsi < 0.5) {
		vh_cmd.cmd_theta = -VISUALHOMING_CONSTANT_PITCH;
	} else {
		vh_cmd.cmd_theta = 0;
	}
	vh_cmd.cmd_phi = 0;
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

float visualhoming_guidance_point_at_homingvector(float dx, float dy) {
	float dpsi = atan2(dy, dx);
	vh_cmd.cmd_psi = stateGetNedToBodyEulers_f()->psi + dpsi;
	while (vh_cmd.cmd_psi > 2 * M_PI) {
		vh_cmd.cmd_psi -= 2 * M_PI;
	}
	while (vh_cmd.cmd_psi < 0) {
		vh_cmd.cmd_psi += 2 * M_PI;
	}
	return dpsi;
}

/**
 * Switch to MODULE mode and update AUTO2 accordingly.
 * @return
 */
bool NavToModule(void) {
	printf("Autopilot to MODULE mode\n");
	autopilot_mode_auto2 = AP_MODE_MODULE;
	autopilot_set_mode(AP_MODE_MODULE);
	return FALSE; // Return immediately
}

/**
 * Switch to NAV mode and update AUTO2 accordingly.
 * @return
 */
bool ModuleToNav(void) {
	printf("Autopilot to NAV mode\n");
	autopilot_mode_auto2 = AP_MODE_NAV;
	autopilot_set_mode(AP_MODE_NAV);
	return FALSE; // Return immediately
}

/**
 * Check whether the visualhoming module is currently controlling the UAV.
 * @return TRUE if controlling, FALSE if not.
 */
int visualhoming_guidance_in_control(void) {
	return (guidance_h.mode == GUIDANCE_H_MODE_MODULE)
			|| (guidance_h.mode == GUIDANCE_H_MODE_GUIDED)
			|| (guidance_h.mode == GUIDANCE_H_MODE_NAV
					&& horizontal_mode == HORIZONTAL_MODE_ATTITUDE);
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

