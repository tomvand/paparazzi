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

#include "visualhoming.h"
#include "visualhoming_video.h"
#include "visualhoming_snapshot.h"
#include "visualhoming_map.h"
#include "visualhoming_guidance_h.h"

/* Control mode */
enum visualhoming_mode_t vh_mode = VH_MODE_SNAPSHOT;
int vh_record_cmd = 0;
int vh_drop_cmd = 0;

/* Navigation functions for flightplan */
bool NavVisualHomingDrop(void) {
	vh_drop_cmd = 1;
	return FALSE;
}

bool NavVisualHomingRecord(enum visualhoming_mode_t mode) {
	vh_mode = mode;
	vh_record_cmd = 1;
	return FALSE; // Immediately continue
}

bool NavVisualHomingReturn(void) {
	vh_record_cmd = 0;
	return TRUE; // TODO Report end of procedure.
}

/* Static variables */
// Guidance targets
static struct snapshot_t *target_snapshot = NULL;
static struct odometry_t *target_odometry = NULL;

// Target buffers
static struct snapshot_t current_warped_snapshot;
static struct snapshot_t target_rotated_snapshot;
static struct snapshot_t single_target_snapshot;
static struct odometry_t single_target_odometry;

// Shared measurements
static struct horizon_t horizon;
static struct snapshot_t current_snapshot;
static struct homingvector_t velocity;

/* Static functions */
static void draw_snapshots(struct image_t *img);

/* Module functions */
void visualhoming_init(void) {
	// Initialize submodules
	vh_video_init(); // TODO set draw_snapshots callback
	vh_video_set_callback(draw_snapshots);
	vh_snapshot_init();
}

void visualhoming_periodic(void) {
	// Get current snapshot
	vh_get_current_horizon(horizon);
	vh_snapshot_from_horizon(&current_snapshot, horizon);
	// Estimate velocity
	velocity = vh_estimate_velocity(&current_snapshot);

	// Handle recording and dropping
	switch (vh_mode) {
	case VH_MODE_SNAPSHOT:
		if (vh_record_cmd) {
			vh_snapshot_copy(&single_target_snapshot, &current_snapshot);
			target_snapshot = &single_target_snapshot;
			vh_record_cmd = 0;
		}
		if (vh_drop_cmd) {
			target_snapshot = NULL;
			vh_drop_cmd = 0;
		}
		break;

	case VH_MODE_ODOMETRY:
		if (vh_record_cmd) {
			single_target_odometry.x = 0;
			single_target_odometry.y = 0;
			target_odometry = &single_target_odometry;
			vh_record_cmd = 0;
		}
		if (vh_drop_cmd) {
			target_odometry = NULL;
			vh_drop_cmd = 0;
		}
		break;

	case VH_MODE_ROUTE:
		default:
		// Not implemented
		break;
	}

	// Update odometry
	if (target_odometry) {
		float delta_x = VISUALHOMING_PERIODIC_PERIOD * velocity.x;
		float delta_y = VISUALHOMING_PERIODIC_PERIOD * velocity.y;
		float delta_psi = velocity.sigma;
		vh_odometry_update(target_odometry, delta_x, delta_y, delta_psi);
	}

	// Guidance
	struct homingvector_t vec;
	if (target_odometry && vh_mode != VH_MODE_SNAPSHOT) {
		// Follow odometry
		vec.x = target_odometry->x;
		vec.y = target_odometry->y;
		vec.sigma = 0;
		// TODO If VH_MODE_ROUTE and below threshold
		// target_odometry = NULL;
	} else if (target_snapshot) {
		// Home towards snapshot
		vec = vh_snapshot_homingvector(&current_snapshot, target_snapshot, NULL,
				NULL);
		// TODO If VH_MODE_ROUTE and below threshold
		// Switch to next target if possible
	} else {
		// Maintain zero velocity
		vec.x = 0;
		vec.y = 0;
		vec.sigma = 0;
	}
	visualhoming_guidance_set_PD(vec.x, vec.y, velocity.x, velocity.y);
}

static void draw_snapshots(struct image_t *img) {
	// CAUTION: This function is called from the video thread.
	// Because this function only reads shared data and because the
	// output is only user feedback, no special precautions (mutex)
	// are taken.
	horizon_t hor;
	// Draw current horizon
	for (int y = 0; y < img->h / 5 * 1; y++) {
		for (int x = 0; x < VISUALHOMING_HORIZON_RESOLUTION; x++) {
			PIXEL_UV(img, x, y) = 127;
			PIXEL_Y(img, x, y) = horizon[x];
		}
	}
	// Draw current snapshot
	vh_snapshot_to_horizon(&current_snapshot, hor);
	for (int y = img->h / 5 * 1; y < img->h / 5 * 2; y++) {
		for (int x = 0; x < VISUALHOMING_HORIZON_RESOLUTION; x++) {
			PIXEL_UV(img, x, y) = 127;
			PIXEL_Y(img, x, y) = hor[x];
		}
	}
	if (target_snapshot != NULL) {
		// Draw warped current snapshot
		vh_snapshot_to_horizon(&current_warped_snapshot, hor);
		for (int y = img->h / 5 * 2; y < img->h / 5 * 3; y++) {
			for (int x = 0; x < VISUALHOMING_HORIZON_RESOLUTION; x++) {
				PIXEL_UV(img, x, y) = 127;
				PIXEL_Y(img, x, y) = hor[x];
			}
		}
		// Draw rotated target snapshot
		vh_snapshot_to_horizon(&target_rotated_snapshot, hor);
		for (int y = img->h / 5 * 3; y < img->h / 5 * 4; y++) {
			for (int x = 0; x < VISUALHOMING_HORIZON_RESOLUTION; x++) {
				PIXEL_UV(img, x, y) = 127;
				PIXEL_Y(img, x, y) = hor[x];
			}
		}
		// Draw target snapshot
		vh_snapshot_to_horizon(&target_snapshot, hor);
		for (int y = img->h / 5 * 4; y < img->h / 5 * 5; y++) {
			for (int x = 0; x < VISUALHOMING_HORIZON_RESOLUTION; x++) {
				PIXEL_UV(img, x, y) = 127;
				PIXEL_Y(img, x, y) = hor[x];
			}
		}
	}
}

