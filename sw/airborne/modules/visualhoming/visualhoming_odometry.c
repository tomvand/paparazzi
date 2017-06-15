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
#include "visualhoming.h"

#include <math.h>

#ifndef VISUALHOMING_ODOMETRY_UPDATE_THRESHOLD
#define VISUALHOMING_ODOMETRY_UPDATE_THRESHOLD 0.10
#endif
float vh_odo_update_threshold = VISUALHOMING_ODOMETRY_UPDATE_THRESHOLD;

static struct snapshot_t ref_ss; // Reference snapshot
static struct odometry_t ref_ss_odo; // Odometric position of reference snapshot

struct odometry_t tel_ss_ref_odo; // For telemetry

void vh_odometry_reset(
		struct odometry_t *odo,
		const struct snapshot_t *current_ss) {
	// Set odometry to zero.
	odo->x = 0;
	odo->y = 0;
	// Store current_ss as latest reference snapshot.
	vh_snapshot_copy(&ref_ss, current_ss);
	// Set reference snapshot odometry to zero.
	ref_ss_odo.x = 0;
	ref_ss_odo.y = 0;
}

void vh_odometry_update(
		struct odometry_t *odo,
		const struct snapshot_t *current_ss) {
	// Calculate pose of reference snapshot relative to current position
	struct homingvector_t pose = vh_snapshot_homingvector(current_ss,
			&ref_ss, NULL, NULL);
	// Add pose to ref snapshot odometry.
	odo->x = vh_environment_radius * pose.x + ref_ss_odo.x * cos(pose.sigma)
			- ref_ss_odo.y * sin(pose.sigma);
	odo->y = vh_environment_radius * pose.y + ref_ss_odo.x * sin(pose.sigma)
			+ ref_ss_odo.y * cos(pose.sigma);
	// If too far from reference snapshot
	if (sqrt(pose.x * pose.x + pose.y * pose.y) > vh_odo_update_threshold) {
		// 	Store current snapshot as new reference.
		vh_snapshot_copy(&ref_ss, current_ss);
		//	Store current odometry as reference odometry.
		ref_ss_odo = *odo;
		tel_ss_ref_odo = ref_ss_odo;
	}
}
