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

#include "visualhoming_snapshot.h"

#include "state.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"

// PD tuning
#ifndef VISUALHOMING_GUIDANCE_P
#define VISUALHOMING_GUIDANCE_P 0.10
#endif
#ifndef VISUALHOMING_GUIDANCE_D
#define VISUALHOMING_GUIDANCE_D 0.20
#endif
#ifndef VISUALHOMING_GUIDANCE_FILTER_GAIN
#define VISUALHOMING_GUIDANCE_FILTER_GAIN 0.20
#endif
#ifndef VISUALHOMING_GUIDANCE_MAX_BANK
#define VISUALHOMING_GUIDANCE_MAX_BANK 0.18 // rad
#endif

struct vh_guidance_tuning_t {
	float Kp; // P-gain [rad/m]
	float Kd; // D-gain [rad/(m/s)]
	float Kf; // Filter gain [0..1], where 1 does not use old estimates
};
extern struct vh_guidance_tuning_t vh_guidance_tuning;

struct homingvector_t vh_estimate_velocity(const struct snapshot_t *new_ss);

void visualhoming_guidance_set_pos_error(float dx, float dy);
void visualhoming_guidance_set_PD(float dx, float dy, float vx, float vy);
void visualhoming_guidance_set_heading_error(float dpsi);

void guidance_h_module_init(void);
void guidance_h_module_enter(void);
void guidance_h_module_read_rc(void);
void guidance_h_module_run(bool in_flight);

#endif
