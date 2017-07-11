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

#include "math/pprz_algebra_float.h"

#include <math.h>

void vh_odometry_reset(struct FloatVect2 *odo) {
	odo->x = 0;
	odo->y = 0;
}

void vh_odometry_update(struct FloatVect2 *odo, float dx, float dy, float dpsi) {
	// Translate
	odo->x -= dx;
	odo->y -= dy;
	// Rotate
	struct FloatVect2 old_odo = *odo;
	odo->x = old_odo.x * cos(dpsi) + old_odo.y * sin(dpsi);
	odo->y = -old_odo.x * sin(dpsi) + old_odo.y * cos(dpsi);
}
