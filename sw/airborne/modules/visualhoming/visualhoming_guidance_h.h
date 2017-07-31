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

struct vh_gains_t {
	float p;
	float d;
	float i;
};

struct vh_g_setpoint_t {
	struct FloatVect2 pos; // Position setpoint [m]
	bool pos_set; // Is position set by user?
	struct FloatVect2 vel; // Velocity setpoint [m]
	bool vel_set; // Is velocity set by user?
};

struct vh_guidance_t {
	struct vh_gains_t gains; // Control gains
	struct vh_g_setpoint_t sp; // Setpoints
	struct FloatVect2 integrator; // Integrator state
	struct FloatEulers cmd; // Commanded attitude
	float maneuver_time; // Expected time for feedforward pos/vel signals [s]
};
extern struct vh_guidance_t vh_guidance;

/* Setpoints */
void vh_guidance_set_pos(float x, float y);
void vh_guidance_set_pos_vel(float x, float y, float u, float v);
void vh_guidance_set_vel(float u, float v);
void vh_guidance_change_heading(float delta_psi);

/* Mode switching functions for flightplan */
bool SetAPMode(uint8_t ap_mode);

int visualhoming_guidance_in_control(void);

void guidance_h_module_init(void);
void guidance_h_module_enter(void);
void guidance_h_module_read_rc(void);
void guidance_h_module_run(bool in_flight);

#endif
