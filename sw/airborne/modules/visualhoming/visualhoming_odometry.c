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

#include "visualhoming_odometry.h"

/**
 * Update odometric vector using the latest body-centric displacement and
 * rotation measurements.
 *
 * The odometric vector is expressed in the *current* body frame and points
 * towards the *starting* location.
 *
 * @param odo Odometric vector to update
 * @param delta_x Position change relative to last measurement
 * @param delta_y Position change relative to last measurement
 * @param delta_psi Heading change relative to last measurement
 */
void odometry_update(
		struct odometry_t *odo,
		float delta_x,
		float delta_y,
		float delta_psi) {
	// Pre-rotate by half delta_psi
	odo->x = cos(delta_psi / 2) * odo->x + sin(delta_psi / 2) * odo->y;
	odo->y = -sin(delta_psi / 2) * odo->x + cos(delta_psi / 2) * odo->y;
	// Apply translation
	odo->x -= delta_x;
	odo->y -= delta_y;
	// Post-rotate by half delta_psi
	odo->x = cos(delta_psi / 2) * odo->x + sin(delta_psi / 2) * odo->y;
	odo->y = -sin(delta_psi / 2) * odo->x + cos(delta_psi / 2) * odo->y;
}
