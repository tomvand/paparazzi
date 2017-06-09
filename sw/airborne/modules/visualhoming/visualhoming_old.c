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
/**
 * @file "modules/visualhoming/visualhoming.c"
 * @author Tom van Dijk
 * Fourier-based local visual homing.
 */

// Standard headers
#include "visualhoming_old.h"

#include <stdio.h>
#include <sys/fcntl.h>
#include <math.h>
#include <unistd.h>
#include <inttypes.h>
#include <pthread.h>
#include <assert.h>

// Paparazzi headers
#include "math/pprz_algebra_float.h"
#include "math/pprz_geodetic_int.h"
#include "state.h"
#include "subsystems/datalink/telemetry.h"

// Own headers
#include "visualhoming_video.h"
#include "visualhoming_snapshot.h"

// Image formats
#include "modules/computer_vision/cv.h"
#include "modules/computer_vision/lib/vision/image.h"

// Guidance in MODULE mode
#include "subsystems/abi.h"
#include "modules/visualhoming/visualhoming_guidance_h.h"
//#include "modules/guidance_opticflow/guidance_opticflow_hover.h"
//#include "firmwares/rotorcraft/guidance/guidance_h.h"

// Guidance in GUIDED mode
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "firmwares/rotorcraft/guidance/guidance_v.h"

// Default settings
#ifndef VISUALHOMING_ENV_RADIUS
#define VISUALHOMING_ENV_RADIUS 5.0
#endif
#ifndef VISUALHOMING_USE_FRAME_TO_FRAME_VELOCITY
#define VISUALHOMING_USE_FRAME_TO_FRAME_VELOCITY 1
#endif

// Settings
int take_snapshot = 0;
int drop_snapshot = 0;
float environment_radius = VISUALHOMING_ENV_RADIUS;
int use_frame_to_frame_velocity = VISUALHOMING_USE_FRAME_TO_FRAME_VELOCITY;

// Snapshots
static int first_run = 1;
static struct snapshot_t current_snapshot;
static struct snapshot_t previous_snapshot;

static int use_target = 0;
static struct snapshot_t target_snapshot;
static struct snapshot_t *current_warped_snapshot = NULL;
static struct snapshot_t *target_rotated_snapshot = NULL;

// Telemetry data
struct homingvector_t homingvector_telemetry;
struct homingvector_t velocity_telemetry;

// Function declarations
// Telemetry callback
static void send_visualhoming(
		struct transport_tx *trans,
		struct link_device *dev);
// Video callback
static void draw_snapshots(struct image_t *img);

// Function definitions
// Module functions
void visualhoming_init(void) {
	printf("visualhoming_init\n");
	// Initialize video handling
	vh_video_init();
	vh_video_set_callback(draw_snapshots);
	// Initialize snapshots
	vh_snapshot_init();
	// Register telemetry message
	homingvector_telemetry.x = 0.0;
	homingvector_telemetry.y = 0.0;
	homingvector_telemetry.sigma = 0.0;
	register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_VISUALHOMING,
			send_visualhoming);
}

void visualhoming_periodic(void) {
	static struct homingvector_t vel_vec = { 0, 0, 0 };
	struct homingvector_t vec = { 0, 0, 0 };
	struct homingvector_t measured_vel = { 0, 0, 0 };

	// Get current horizon
	horizon_t horizon;
	vh_get_current_horizon(horizon);
	// Transform into snapshot
	vh_snapshot_from_horizon(&current_snapshot, horizon);
	if (first_run) {
		vh_snapshot_from_horizon(&previous_snapshot, horizon);
		first_run = 0;
	}

	// Store or drop target snapshot
	if (take_snapshot) {
		vh_snapshot_copy(&target_snapshot, &current_snapshot);
		use_target = 1;
		take_snapshot = 0;
	}
	if (drop_snapshot) {
		use_target = 0;
		target_rotated_snapshot = NULL;
		current_warped_snapshot = NULL;
		drop_snapshot = 0;
	}

	/*
	 * Navigation
	 */
	// Estimate current velocity
	const float GAIN = 0.20;
	measured_vel = vh_snapshot_homingvector(&previous_snapshot,
			&current_snapshot, NULL, NULL);
	measured_vel.x *= VISUALHOMING_PERIODIC_FREQ * environment_radius;
	measured_vel.y *= VISUALHOMING_PERIODIC_FREQ * environment_radius;
	if (!isnan(measured_vel.x) && !isnan(measured_vel.y)) {
		vel_vec.x = GAIN * measured_vel.x + (1 - GAIN) * vel_vec.x;
		vel_vec.y = GAIN * measured_vel.y + (1 - GAIN) * vel_vec.y;
	}
	printf("VELOCITY: vx = %+.1f, vy = %+.1f\n", vel_vec.x, vel_vec.y);
	// Update previous snapshot
	vh_snapshot_copy(&previous_snapshot, &current_snapshot);

	// Estimate homing vector if req'd
	if (use_target) {
		vec = vh_snapshot_homingvector(&current_snapshot, &target_snapshot,
				&current_warped_snapshot, &target_rotated_snapshot);
		vec.x *= environment_radius;
		vec.y *= environment_radius;
		printf("dX = %+.2f\tdY = %+.2f\tsigma = %+4.0f\n", vec.x, vec.y,
				vec.sigma / M_PI * 180.0);
	}

	/*
	 * GPS-less guidance in MODULE mode.
	 */
	if (use_frame_to_frame_velocity) {
		visualhoming_guidance_set_PD(vec.x, -vec.y, vel_vec.x, -vel_vec.y);
	} else {
		visualhoming_guidance_set_pos_error(vec.x, -vec.y);
	}
	visualhoming_guidance_set_heading_error(vec.sigma);

	/*
	 * GPS-assisted guidance in GUIDED mode.
	 */
	// Set position reference for guided mode.
	// Note: vec = {0, 0, 0} without snapshot.
	float x_ref, y_ref;
	struct NedCoor_f *current_pos = stateGetPositionNed_f();
	struct FloatEulers * current_eulers = stateGetNedToBodyEulers_f();
	float heading = current_eulers->psi;
	printf("Pos = %+.1f, %+.1f\n", current_pos->x, current_pos->y);
	printf("Heading = %+.2f\n", heading);
	x_ref = current_pos->x + (vec.x * cos(heading) + vec.y * sin(heading));
	y_ref = current_pos->y + (vec.x * sin(heading) - vec.y * cos(heading));
	printf("Pos_ref = %+.1f, %+.1f\n", x_ref, y_ref);
	guidance_h_set_guided_pos(x_ref, y_ref);

	/*
	 * Telemetry
	 */
	// Save homing vector for telemetry
	homingvector_telemetry = vec;
	velocity_telemetry = vel_vec;
}

void visualhoming_start(void) {
}

void visualhoming_stop(void) {
}

// Telemetry callback
static void send_visualhoming(
		struct transport_tx *trans,
		struct link_device *dev) {
	pprz_msg_send_VISUALHOMING(trans, dev, AC_ID, &homingvector_telemetry.x,
			&homingvector_telemetry.y, &homingvector_telemetry.sigma,
			&velocity_telemetry.x, &velocity_telemetry.y);
}

// Camera callback
static void draw_snapshots(struct image_t *img) {
	// CAUTION: This function is called from the video thread.
	// Because this function only reads shared data and because the
	// output is only user feedback, no special precautions (mutex)
	// are taken.
	// Draw current horizon
	horizon_t hor;
	vh_get_current_horizon(hor);
	for (int y = 0; y < img->h / 5 * 1; y++) {
		for (int x = 0; x < VISUALHOMING_HORIZON_RESOLUTION; x++) {
			PIXEL_UV(img, x, y) = 127;
			PIXEL_Y(img, x, y) = hor[x];
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
	// Draw warped current snapshot
	if (current_warped_snapshot != NULL) {
		vh_snapshot_to_horizon(current_warped_snapshot, hor);
		for (int y = img->h / 5 * 2; y < img->h / 5 * 3; y++) {
			for (int x = 0; x < VISUALHOMING_HORIZON_RESOLUTION; x++) {
				PIXEL_UV(img, x, y) = 127;
				PIXEL_Y(img, x, y) = hor[x];
			}
		}
	}
	// Draw rotated target snapshot
	if (target_rotated_snapshot != NULL) {
		vh_snapshot_to_horizon(target_rotated_snapshot, hor);
		for (int y = img->h / 5 * 3; y < img->h / 5 * 4; y++) {
			for (int x = 0; x < VISUALHOMING_HORIZON_RESOLUTION; x++) {
				PIXEL_UV(img, x, y) = 127;
				PIXEL_Y(img, x, y) = hor[x];
			}
		}
	}
	if (use_target) {
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
