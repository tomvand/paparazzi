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
#include "visualhoming_odometry.h"

#include "generated/modules.h"
#include "subsystems/datalink/telemetry.h"
#include "mcu_periph/sys_time.h"

#include <stdio.h>

/* Settings */
#ifndef VISUALHOMING_ODOMETRY_ARRIVAL_THRESHOLD
#define VISUALHOMING_ODOMETRY_ARRIVAL_THRESHOLD 0.0
#endif
float vh_odometry_arrival_threshold = VISUALHOMING_ODOMETRY_ARRIVAL_THRESHOLD;

#ifndef VISUALHOMING_SNAPSHOT_ARRIVAL_THRESHOLD
#define VISUALHOMING_SNAPSHOT_ARRIVAL_THRESHOLD 0.01
#endif
float vh_snapshot_arrival_threshold = VISUALHOMING_SNAPSHOT_ARRIVAL_THRESHOLD;

#ifndef VISUALHOMING_SNAPSHOT_INITIAL_THRESHOLD
#define VISUALHOMING_SNAPSHOT_INITIAL_THRESHOLD 0.01
#endif
float vh_snapshot_initial_threshold = VISUALHOMING_SNAPSHOT_INITIAL_THRESHOLD;

#ifndef VISUALHOMING_SNAPSHOT_TRIGGER_THRESHOLD
#define VISUALHOMING_SNAPSHOT_TRIGGER_THRESHOLD 0.2
#endif
float vh_snapshot_trigger_threshold = VISUALHOMING_SNAPSHOT_TRIGGER_THRESHOLD;

#ifndef VISUALHOMING_SNAPSHOT_TRIGGER_FROM_INITIAL
#define VISUALHOMING_SNAPSHOT_TRIGGER_FROM_INITIAL 1
#endif
int vh_snapshot_trigger_from_initial =
VISUALHOMING_SNAPSHOT_TRIGGER_FROM_INITIAL;

#ifndef VISUALHOMING_ENV_RADIUS
#define VISUALHOMING_ENV_RADIUS 1.0
#endif
float vh_environment_radius = VISUALHOMING_ENV_RADIUS;

/* Control mode */
enum visualhoming_mode_t vh_mode_cmd = VH_MODE_NOCMD;
static enum visualhoming_mode_t vh_mode = VH_MODE_STOP;

/* Static variables */
// Snapshot mode buffers
static struct snapshot_t *current_warped_snapshot = NULL;
static struct snapshot_t *target_rotated_snapshot = NULL;
static struct homingvector_t homingvector;
static uint32_t current_ts;
static uint32_t previous_ts;

// Odometry mode buffers
static struct odometry_t single_target_odometry;

// Shared measurements
static horizon_t horizon;
static struct snapshot_t current_snapshot;
static struct homingvector_t velocity;
static int arrival_detected = 0;

// Miscellaneous telemetry data
static uint32_t step_time;
static float tel_dt;
static float tel_angle_diff;

/* Navigation functions for flightplan */
bool VisualHomingTakeSnapshot(void) {
	vh_mode_cmd = VH_MODE_SNAPSHOT;
	return FALSE; // Return immediately
}

bool VisualHomingRecordOdometry(void) {
	vh_mode_cmd = VH_MODE_ODOMETRY;
	return FALSE; // Return immediately
}

bool NavVisualHoming(void) {
	visualhoming_guidance_update_nav();
	return !arrival_detected;
}

/* Static functions */
static struct homingvector_t estimate_velocity(
		const struct snapshot_t *new_ss);
static float homingvector_difference(void);
static void draw_snapshots(struct image_t *img);
static void send_visualhoming(
		struct transport_tx *trans,
		struct link_device *dev);

/* Module functions */
void visualhoming_init(void) {
	// Initialize submodules
	vh_video_init(); // TODO set draw_snapshots callback
	vh_video_set_callback(draw_snapshots);
	vh_snapshot_init();
	// Register telemetry
	register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_VISUALHOMING,
			send_visualhoming);
	register_periodic_telemetry(DefaultPeriodic,
			PPRZ_MSG_ID_VISUALHOMING_MAP_UPDATE,
			send_visualhoming_map_update);
}

void visualhoming_periodic(void) {
	// Check that horizon has been updated
	current_ts = vh_get_current_timestamp();
	if (current_ts == previous_ts) return; // Nothing to do if image hasn't been updated.

	// Get current snapshot
	vh_get_current_horizon(horizon);
	vh_snapshot_from_horizon(&current_snapshot, horizon);
	// Estimate velocity
	velocity = estimate_velocity(&current_snapshot);

	// Handle commands
	if (vh_mode_cmd != VH_MODE_NOCMD) {
		switch (vh_mode_cmd) {
		case VH_MODE_STOP:
			vh_map_clear();
			vh_mode = VH_MODE_STOP;
			break;
		case VH_MODE_SNAPSHOT:
			vh_map_clear();
			vh_map_push(&current_snapshot);
			vh_mode = VH_MODE_SNAPSHOT;
			break;
		case VH_MODE_ODOMETRY:
			vh_odometry_reset(&single_target_odometry, &current_snapshot);
			vh_mode = VH_MODE_ODOMETRY;
			break;
		default:
			printf("[VISUALHOMING] Invalid mode command: %d!\n", vh_mode_cmd);
			break;
		}
		vh_mode_cmd = VH_MODE_NOCMD;
	}

	// Update guidance
	const struct snapshot_t *target_snapshot;
	target_snapshot = vh_map_peek();
	if (vh_mode == VH_MODE_SNAPSHOT && !target_snapshot) {
		vh_mode = VH_MODE_STOP;
	}

	arrival_detected = 0; // Signal to NAV routine to continue flightplan

	switch (vh_mode) {
	case VH_MODE_STOP:
		// Maintain zero velocity
		visualhoming_guidance_set_PD(0, 0, velocity.x, velocity.y);
		break;

	case VH_MODE_SNAPSHOT:
		// Home towards current snapshot
		homingvector = vh_snapshot_homingvector(&current_snapshot,
				target_snapshot, &current_warped_snapshot,
				&target_rotated_snapshot);
		visualhoming_guidance_set_PD(homingvector.x, -homingvector.y,
				velocity.x, velocity.y);
		// Waypoint sequencing
		if (visualhoming_guidance_in_control()) {
			// Inbound flight, detect arrivals
			float remaining;
			remaining = sqrt(
					homingvector.x * homingvector.x
							+ homingvector.y * homingvector.y);
			printf("Remaining: %+.2f\n", remaining);
			if (remaining < vh_snapshot_arrival_threshold) {
				// Arrival detected
				if (vh_map_get_index() > 0) {
					vh_map_pop();
				} else {
					// Final waypoint reached
//					arrival_detected = 1;
				}
			}
		} else {
			// Outbound flight, detect edge of catchment area
			float diff;
			diff = homingvector_difference();
			if (diff > vh_snapshot_trigger_threshold) {
				// Reached edge of catchment area
				vh_map_push(&current_snapshot);
			}
		}
		break;

	case VH_MODE_ODOMETRY:
		// Follow odometric vector
		vh_odometry_update(&single_target_odometry, &current_snapshot);
		visualhoming_guidance_set_PD(single_target_odometry.x,
				-single_target_odometry.y, velocity.x, velocity.y);
		// Detect arrival
		if (visualhoming_guidance_in_control()) {
			float odo_distance = sqrt(
					single_target_odometry.x * single_target_odometry.x
							+ single_target_odometry.y
									* single_target_odometry.y);
			if (odo_distance < vh_odometry_arrival_threshold) {
				arrival_detected = 1;
			}
		}
		break;

	default:
		printf("[VISUALHOMING] Invalid mode: %d!\n", vh_mode);
		break;
	}

	// Update timestamps
	previous_ts = current_ts;

	// Measure run-time of step
	step_time = get_sys_time_usec() - current_ts;
}

/* Static functions */
/**
 * Estimate velocity. Should be called every run with the current snapshot.
 * @param new_ss
 * @return
 */
static struct homingvector_t estimate_velocity(
		const struct snapshot_t *new_ss) {
	static struct homingvector_t velocity;
	static struct snapshot_t previous_snapshot;
	static int first_run = 1;
	if (first_run) {
		vh_snapshot_copy(&previous_snapshot, new_ss);
		first_run = 0;
	}
	// Get instantaneous velocity measurement
	struct homingvector_t measured_vel;
	float dt = (current_ts - previous_ts) / 1e6;
	tel_dt = dt;
	measured_vel = vh_snapshot_homingvector(&previous_snapshot, new_ss, NULL,
	NULL);
	if (dt != 0) {
		measured_vel.x *= vh_environment_radius / dt;
		measured_vel.y *= -vh_environment_radius / dt;
	}
	// Filter velocities
	if (!isnan(measured_vel.x) && !isnan(measured_vel.y)) {
		velocity.x = vh_guidance_tuning.Kf * measured_vel.x
				+ (1 - vh_guidance_tuning.Kf) * velocity.x;
		velocity.y = vh_guidance_tuning.Kf * measured_vel.y
				+ (1 - vh_guidance_tuning.Kf) * velocity.y;
	}
	// Store previous snapshot
	vh_snapshot_copy(&previous_snapshot, new_ss);

	return velocity;
}

static float homingvector_difference(void) {
	static const struct snapshot_t *target_snapshot;
	static struct homingvector_t trigger_vec;

	const struct snapshot_t *new_target_snapshot;
	new_target_snapshot = vh_map_peek();
	if (new_target_snapshot != target_snapshot) {
		// New reference snapshot, reset detection
		trigger_vec.x = 0;
		trigger_vec.y = 0;
	}
	target_snapshot = new_target_snapshot;
	if (!target_snapshot) {
		return 0;
	}

	struct homingvector_t trigger_vec_new = vh_snapshot_homingvector(
			target_snapshot, &current_snapshot, NULL, NULL);
	float angle_diff;

	if (sqrt(trigger_vec_new.x * trigger_vec_new.x
			+ trigger_vec_new.y * trigger_vec_new.y) > 0.05) {
		if (trigger_vec.x == 0 && trigger_vec.y == 0) {
			trigger_vec = trigger_vec_new; // Keep track of initial direction of travel.
		}
		float angle_new = atan2(trigger_vec_new.y, trigger_vec_new.x);
		float angle_old = atan2(trigger_vec.y, trigger_vec.x);
		angle_diff = angle_new - angle_old;
//		printf("Angle new: %+.2f\n", angle_new);
//		printf("Angle old: %+.2f\n", angle_old);
		while (angle_diff > M_PI) {
			angle_diff -= 2 * M_PI;
		}
		while (angle_diff < -M_PI) {
			angle_diff += 2 * M_PI;
		}
		if (angle_diff < 0) angle_diff = -angle_diff;
	} else {
		trigger_vec.x = 0;
		trigger_vec.y = 0;
		angle_diff = 0;
	}
//	printf("Angle dif: %+.2f\n", angle_diff);
	tel_angle_diff = angle_diff;
	if (!vh_snapshot_trigger_from_initial) {
		// Compare to *previous* homing vector instead of initial vector.
		trigger_vec = trigger_vec_new;
	}
	return angle_diff;
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
	if (vh_mode == VH_MODE_SNAPSHOT) {
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
		// Draw target snapshot
		const struct snapshot_t *target_snapshot;
		target_snapshot = vh_map_peek();
		if (target_snapshot) {
			vh_snapshot_to_horizon(target_snapshot, hor);
			for (int y = img->h / 5 * 4; y < img->h / 5 * 5; y++) {
				for (int x = 0; x < VISUALHOMING_HORIZON_RESOLUTION; x++) {
					PIXEL_UV(img, x, y) = 127;
					PIXEL_Y(img, x, y) = hor[x];
				}
			}
		}
	}
}

static void send_visualhoming(
		struct transport_tx *trans,
		struct link_device *dev) {
	int8_t m = vh_mode;
	int8_t in_control = visualhoming_guidance_in_control();
	struct EnuCoor_f *enu = stateGetPositionEnu_f();
	float psi = stateGetNedToBodyEulers_f()->psi;
	pprz_msg_send_VISUALHOMING(trans, dev, AC_ID, &m, &homingvector.x,
			&homingvector.y, &homingvector.sigma,
			&single_target_odometry.x, &single_target_odometry.y,
			&tel_ss_ref_odo.x, &tel_ss_ref_odo.y,
			&enu->x, &enu->y, &psi,
			&velocity.x, &velocity.y,
			&current_ts, &tel_dt, &step_time,
			&tel_angle_diff,
			&in_control);
}

