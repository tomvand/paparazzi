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

void visualhoming_guidance_set_pos_error(float dx, float dy);
void visualhoming_guidance_set_heading_error(float dpsi);

void guidance_h_module_init(void);
void guidance_h_module_enter(void);
void guidance_h_module_read_rc(void);
void guidance_h_module_run(bool in_flight);

#endif
