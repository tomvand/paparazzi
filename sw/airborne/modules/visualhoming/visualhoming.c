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
#include "state.h"

#include "modules/dead_reckoning/dead_reckoning.h"

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
#define VISUALHOMING_SNAPSHOT_TRIGGER_THRESHOLD 0.1
#endif
float vh_snapshot_trigger_threshold = VISUALHOMING_SNAPSHOT_TRIGGER_THRESHOLD;

#ifndef VISUALHOMING_SNAPSHOT_TRIGGER_FROM_INITIAL
#define VISUALHOMING_SNAPSHOT_TRIGGER_FROM_INITIAL 1
#endif
int vh_snapshot_trigger_from_initial =
VISUALHOMING_SNAPSHOT_TRIGGER_FROM_INITIAL;

#ifndef VISUALHOMING_RECORD_PERIOD
#define VISUALHOMING_RECORD_PERIOD 1.0e6 // us
#endif

#ifndef VISUALHOMING_REPLAY_PERIOD
#define VISUALHOMING_REPLAY_PERIOD 5.0e6 // us
#endif

#ifndef VISUALHOMING_ENV_RADIUS
#define VISUALHOMING_ENV_RADIUS 3.0
#endif
float vh_environment_radius = VISUALHOMING_ENV_RADIUS;

/* Control mode */
enum vh_sequencer_mode_t vh_gcs_seq_mode = VH_SEQ_NOCMD;
enum vh_input_mode_t vh_gcs_input_mode = VH_IN_NOCMD;

static enum vh_sequencer_mode_t vh_seq_mode = VH_SEQ_STOP;
static enum vh_input_mode_t vh_input_mode = VH_IN_SNAPSHOT;

static int odo_reqd = 0;

/* Static variables */
// Snapshot buffers
static struct snapshot_t current_snapshot;
static struct snapshot_t current_warped_snapshot;
static struct snapshot_t target_rotated_snapshot;
static struct homingvector_t homingvector;
static uint32_t current_ts;
static uint32_t previous_ts;

// Shared measurements
static horizon_t horizon;
static struct FloatVect2 velocity;
static int arrival_detected = 0;

// Miscellaneous telemetry data
static uint32_t step_time;
static float tel_dt;
static float tel_angle_diff;

/* Navigation functions for flightplan */
bool VisualHomingSetSnapshotMode(void) {
	vh_gcs_input_mode = VH_IN_SNAPSHOT;
	return FALSE;
}
bool VisualHomingSetOdometryMode(void) {
	vh_gcs_input_mode = VH_IN_ODO;
	return FALSE;
}

bool VisualHomingStop(void) {
	vh_gcs_seq_mode = VH_SEQ_STOP;
	return FALSE;
}
bool VisualHomingRecordSingle(void) {
	vh_gcs_seq_mode = VH_SEQ_SINGLE;
	return FALSE;
}
bool VisualHomingRecordRoute(void) {
	vh_gcs_seq_mode = VH_SEQ_ROUTE;
	return FALSE;
}

bool VisualHomingCompleted(void) {
	return arrival_detected;
}

/* Static functions */
static struct homingvector_t estimate_velocity(
		const struct snapshot_t *new_ss);
static float homingvector_difference(struct homingvector_t vec);
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
//	velocity = estimate_velocity(&current_snapshot);
	velocity = dr_getBodyVel();

	// Handle commands
	if (vh_gcs_input_mode != VH_IN_NOCMD) {
		// Input mode button pressed
		vh_input_mode = vh_gcs_input_mode;
		vh_gcs_input_mode = VH_IN_NOCMD;
	}
	if (vh_gcs_seq_mode != VH_SEQ_NOCMD) {
		// Sequencer mode button pressed
		switch (vh_gcs_seq_mode) {
		case VH_SEQ_STOP:
			vh_map_clear();
			break;
		case VH_SEQ_SINGLE:
			case VH_SEQ_ROUTE:
			vh_map_clear();
			vh_map_push(&current_snapshot);
			vh_odometry_reset(vh_map_odometry());
			break;
		default:
			printf("Error: Invalid sequencer command: %d!\n", vh_gcs_seq_mode);
		}
		vh_seq_mode = vh_gcs_seq_mode;
		vh_gcs_seq_mode = VH_SEQ_NOCMD;
	}

	// Update odometry
	struct FloatVect2 *odo = vh_map_odometry();
	if (odo) {
		static float previous_heading = 0.0;

		float heading = dr_getHeading(); // TODO remove?
		float dpsi = heading - previous_heading;
		previous_heading = heading;

		float dt = (float)(current_ts - previous_ts) / 1e6;
		float dx = velocity.x * dt;
		float dy = velocity.y * dt;

		vh_odometry_update(odo, dx, dy, dpsi);
		printf("Odo update: dx = %+.2f, dy = %+.2f, dpsi = %+.2f\n", dx, dy,
				dpsi);
		printf("Odo: x = %+.2f, y = %+.2f\n", odo->x, odo->y);
	}

	// Update guidance
	const struct snapshot_t *target_snapshot;
	target_snapshot = vh_map_peek();

	arrival_detected = 0; // Signal to NAV routine to continue flightplan

	static uint32_t last_record_ts = 0;
	static uint32_t last_replay_ts = 0;

	if (vh_seq_mode != VH_SEQ_STOP
			&& vh_input_mode == VH_IN_ODO
			&& vh_map_odometry()
			&& (vh_seq_mode == VH_SEQ_SINGLE
					|| (visualhoming_guidance_in_control() && odo_reqd))) {
		// Use ODOMETRY as guidance vector
		odo_reqd = 1;
		struct FloatVect2 *odo = vh_map_odometry();
		printf("odo->x = %+.2f,\todo->y = %+.2f\n", odo->x, odo->y);
		visualhoming_guidance_set_PD(odo->x, odo->y, velocity.x, velocity.y);
		// Detect arrival
		if (sqrt(odo->x * odo->x + odo->y * odo->y) < 0.2) {
			odo_reqd = 0;
		}
		// Reset snapshot timer
		last_replay_ts = current_ts;
	} else if (vh_seq_mode != VH_SEQ_STOP && target_snapshot) {
		// Use SNAPSHOT homing vector for guidance
		odo_reqd = 0;
		homingvector = vh_snapshot_homingvector(&current_snapshot,
				target_snapshot, &current_warped_snapshot,
				&target_rotated_snapshot);
		// Set guidance setpoints
		visualhoming_guidance_set_PD(vh_environment_radius * homingvector.x,
				vh_environment_radius * -homingvector.y,
				velocity.x, velocity.y);
		visualhoming_guidance_set_heading_error(-homingvector.sigma);
		// Waypoint sequencing in route mode
		if (vh_seq_mode == VH_SEQ_ROUTE) {
			if (visualhoming_guidance_in_control()) {
				// Inbound flight
				if (current_ts > last_replay_ts + VISUALHOMING_REPLAY_PERIOD) {
					if (vh_map_get_index() > 0) {
						vh_map_pop();
						odo_reqd = 1;
					}
					last_replay_ts = current_ts;
				}
				last_record_ts = current_ts;
			} else {
				// Outbound flight
				if (current_ts > last_record_ts + VISUALHOMING_RECORD_PERIOD) {
					vh_map_push(&current_snapshot);
					vh_odometry_reset(vh_map_odometry());
					last_record_ts = current_ts;
				}
				last_replay_ts = current_ts + 5e6;
			}
		}
	} else {
		// Maintain zero velocity
		visualhoming_guidance_set_PD(0, 0, velocity.x, velocity.y);
	}

	// Update timestamps
	previous_ts = current_ts;

	// Measure run-time of step
	step_time = get_sys_time_usec() - current_ts;
}

/* Static functions */
/**
 * Estimate velocity. Should be called every run with the current snapshot.
 * Velocity is in front,right coordinates.
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

static float homingvector_difference(struct homingvector_t trigger_vec_new) {
	static struct homingvector_t trigger_vec;
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
	if (!odo_reqd) {
		const struct snapshot_t *target_snapshot;
		target_snapshot = vh_map_peek();
		if (target_snapshot) {
			// Draw target snapshot
			vh_snapshot_to_horizon(target_snapshot, hor);
			for (int y = img->h / 5 * 4; y < img->h / 5 * 5; y++) {
				for (int x = 0; x < VISUALHOMING_HORIZON_RESOLUTION; x++) {
					PIXEL_UV(img, x, y) = 127;
					PIXEL_Y(img, x, y) = hor[x];
				}
			}
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
		}
	}
}

static void send_visualhoming(
		struct transport_tx *trans,
		struct link_device *dev) {
	int8_t input_mode = vh_input_mode;
	int8_t sequencer_mode = vh_seq_mode;

	struct FloatVect2 odo;
	if (vh_map_odometry()) {
		odo = *vh_map_odometry();
	} else {
		odo.x = 0;
		odo.y = 0;
	}

	int8_t in_control = visualhoming_guidance_in_control();
	struct EnuCoor_f *enu = stateGetPositionEnu_f();
	float psi = stateGetNedToBodyEulers_f()->psi;

	struct FloatEulers cmd = visualhoming_guidance_get_command();

	float dummy = 0;

	pprz_msg_send_VISUALHOMING(trans, dev, AC_ID, &input_mode, &sequencer_mode,
			&homingvector.x, &homingvector.y, &homingvector.sigma,
			&odo.x, &odo.y,
			&dummy, &dummy,
			&enu->x, &enu->y, &psi,
			&velocity.x, &velocity.y,
			&current_ts, &tel_dt, &step_time,
			&tel_angle_diff,
			&in_control,
			&cmd.phi, &cmd.theta, &cmd.psi);
}

