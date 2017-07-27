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
#ifndef VISUALHOMING_GUIDANCE_H_H
#define VISUALHOMING_GUIDANCE_H_H

#include "state.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"

struct vh_guidance_tuning_t {
	float Kp; // P-gain [rad/m]
	float Td; // D-gain [rad/(m/s)]
	float Kf; // Filter gain [0..1], where 1 does not use old estimates
};
extern struct vh_guidance_tuning_t vh_guidance_tuning;

//void visualhoming_guidance_set_pos_setpoint(float dx, float dy);
//void visualhoming_guidance_set_pos_error(float dx, float dy);
void visualhoming_guidance_set_PD(float dx, float dy, float vx, float vy);
//void visualhoming_guidance_set_constant_pitch(float dx, float dy);
//void visualhoming_guidance_set_heading_rate(float dx, float dy, float dt);
void visualhoming_guidance_set_heading_error(float dpsi);
//float visualhoming_guidance_point_at_homingvector(float dx, float dy);

/* Mode switching functions for flightplan */
bool SetAPMode(uint8_t ap_mode);

int visualhoming_guidance_in_control(void);

struct FloatEulers visualhoming_guidance_get_command(void);

void guidance_h_module_init(void);
void guidance_h_module_enter(void);
void guidance_h_module_read_rc(void);
void guidance_h_module_run(bool in_flight);

#endif
