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

#include <stdio.h>

#ifndef VISUALHOMING_MAX_BANK
#define VISUALHOMING_MAX_BANK 0.18 // rad
#endif

struct vh_guidance_data_t {
	float cmd_phi;
	float cmd_theta;
	float cmd_psi;
	float dx;
	float dy;
	float vx;
	float vy;
	float a_1;
	float b_0;
	float b_1;
};

// Found through MATLAB.
// TODO calculate here or explain
static struct vh_guidance_data_t data = {
	.a_1 = 0.2, .b_0 = 3.983, .b_1 = 3.616
};

/**
 * Set new position error.
 * Should be called at 10 Hz!
 *
 * This function accepts new position error measurements and
 * calculates new attitude setpoints.
 * @param dx
 * @param dy
 */
void visualhoming_guidance_set_pos_error(float dx, float dy) {
	// Calculate new setpoints
//	data.cmd_theta = data.b_0 * (-dx) - data.b_1 * (-data.dx)
//			- data.a_1 * data.cmd_theta;
//	data.cmd_phi = data.b_0 * dy - data.b_1 * data.dy - data.a_1 * data.cmd_phi;

//	data.cmd_theta = -0.10 * dx;
//	data.cmd_phi = 0.10 * dy;

	static const float GAIN = 0.20;
	static const float KP = 0.02;
	static const float KD = 0.10;
	data.vx = GAIN * (10 * (data.dx - dx)) + (1 - GAIN) * data.vx;
	data.vy = GAIN * (10 * (data.dy - dy)) + (1 - GAIN) * data.vy;
	data.cmd_theta = -(KP * dx - KD * data.vx);
	data.cmd_phi = KP * dy - KD * data.vy;

	BoundAbs(data.cmd_phi, VISUALHOMING_MAX_BANK);
	BoundAbs(data.cmd_theta, VISUALHOMING_MAX_BANK);
	printf("\n");
	printf("PITCH: %.1f\n", data.cmd_theta / M_PI * 180.0);
	printf("ROLL:  %.1f\n", data.cmd_phi / M_PI * 180.0);
	printf("\n");
	// Keep track of last error
	data.dx = dx;
	data.dy = dy;
}

void visualhoming_guidance_set_heading_error(float dpsi) {
//	data.cmd_psi = stateGetNedToBodyEulers_f()->psi + dpsi;
}

void guidance_h_module_init(void) {
	// Do nothing
}

void guidance_h_module_enter(void) {
	// Set setpoints to zero
	data.cmd_theta = 0;
	data.cmd_phi = 0;
	data.cmd_psi = stateGetNedToBodyEulers_f()->psi;
	// Set previous error to zero
	data.dx = 0;
	data.dy = 0;
	data.vx = 0;
	data.vy = 0;
}

void guidance_h_module_read_rc(void) {
	// Do nothing
}

void guidance_h_module_run(bool in_flight) {
	struct Int32Eulers rpy = {
			.theta = ANGLE_BFP_OF_REAL(data.cmd_theta),
			.phi = ANGLE_BFP_OF_REAL(data.cmd_phi),
			.psi = ANGLE_BFP_OF_REAL(data.cmd_psi) };
	stabilization_attitude_set_rpy_setpoint_i(&rpy);
	stabilization_attitude_run(in_flight);
}

