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
#include "visualhoming.h"
#include "visualhoming_core.h"

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

// Configuration
#ifndef VISUALHOMING_CAMERA
#define VISUALHOMING_CAMERA bottom_camera
#endif
#ifndef RADIUS_SAMPLING_NPOINTS
#define RADIUS_SAMPLING_NPOINTS 5
#endif
#ifndef VISUALHOMING_SEND_ABI_ID
#define VISUALHOMING_SEND_ABI_ID 1       ///< Default ID to send abi messages
#endif
PRINT_CONFIG_VAR(OPTICFLOW_SEND_ABI_ID)

// Default settings
#ifndef VISUALHOMING_CENTER_X
#define VISUALHOMING_CENTER_X 0.50
#endif
#ifndef VISUALHOMING_CENTER_Y
#define VISUALHOMING_CENTER_Y 0.50
#endif
#ifndef VISUALHOMING_RADIUS_TOP
#define VISUALHOMING_RADIUS_TOP 0.23
#endif
#ifndef VISUALHOMING_RADIUS_BOTTOM
#define VISUALHOMING_RADIUS_BOTTOM 0.27
#endif
#ifndef VISUALHOMING_ENV_RADIUS
#define VISUALHOMING_ENV_RADIUS 5.0
#endif
#ifndef VISUALHOMING_DEROTATE_GAIN
#define VISUALHOMING_DEROTATE_GAIN 0.08
#endif
#ifndef VISUALHOMING_USE_FRAME_TO_FRAME_VELOCITY
#define VISUALHOMING_USE_FRAME_TO_FRAME_VELOCITY 0
#endif

// Settings
int take_snapshot = 0;
int drop_snapshot = 0;
struct calibration_t calibration = {
		.center_x = VISUALHOMING_CENTER_X,
		.center_y = VISUALHOMING_CENTER_Y,
		.radius_top = VISUALHOMING_RADIUS_TOP,
		.radius_bottom = VISUALHOMING_RADIUS_BOTTOM };
float environment_radius = VISUALHOMING_ENV_RADIUS;
float derotate_gain = VISUALHOMING_DEROTATE_GAIN;
int use_frame_to_frame_velocity = VISUALHOMING_USE_FRAME_TO_FRAME_VELOCITY;

// Macro functions
#define PIXEL_UV(img,x,y) ( ((uint8_t*)((img)->buf))[2*(x) + 2*(y)*(img)->w] )
#define PIXEL_Y(img,x,y) ( ((uint8_t*)((img)->buf))[2*(x) + 1 + 2*(y)*(img)->w] )

// Cross-thread variables
static pthread_mutex_t video_mutex;
static struct snapshot_t *previous_snapshot = NULL;
static struct snapshot_t *current_snapshot = NULL;
static struct snapshot_t *target_snapshot = NULL;
static struct snapshot_t *current_warped_snapshot = NULL;
static struct snapshot_t *target_rotated_snapshot = NULL;
static struct FloatRMat attitude;

// Telemetry data
struct homingvector_t homingvector_telemetry;
struct homingvector_t velocity_telemetry;

// Function declarations
// Telemetry callback
static void send_visualhoming(
		struct transport_tx *trans,
		struct link_device *dev);
// Camera callback
static struct image_t* camera_callback(struct image_t *img);
// Helper functions
static void draw_calibration(struct image_t *img);
static void draw_snapshots(struct image_t *img, const float hor[]);
static void extract_horizon(float hor[], const struct image_t *img);
static struct point_t derotated_point(
		const struct image_t *img,
		float bearing,
		float elevation);

// Function definitions
// Module functions
void visualhoming_init(void) {
	printf("visualhoming_init\n");
	// Initialize attitude
	float_rmat_identity(&attitude);
	// Initialize visual homing core
	visualhoming_core_init();
	// Add callback to camera
	cv_add_to_device(&VISUALHOMING_CAMERA, camera_callback);
	// Register telemetry message
	homingvector_telemetry.x = 0.0;
	homingvector_telemetry.y = 0.0;
	homingvector_telemetry.sigma = 0.0;
	register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_VISUALHOMING,
			send_visualhoming);
}

void visualhoming_periodic(void) {
	struct FloatRMat *current_att;
	static struct homingvector_t vel_vec = { 0, 0, 0 };
	struct homingvector_t vec = { 0, 0, 0 };
//	static struct homingvector_t previous_vec = { 0, 0, 0 };
	struct homingvector_t measured_vel = { 0, 0, 0 };

	// Communicate current attitude to video thread.
	current_att = stateGetNedToBodyRMat_f();
	pthread_mutex_lock(&video_mutex);
	MAT33_COPY(attitude, *current_att);
	pthread_mutex_unlock(&video_mutex);

	/*
	 * Navigation
	 */
	if (current_snapshot == NULL) {
		return; // Nothing to do without video input :(.
	}
	pthread_mutex_lock(&video_mutex);
	// Estimate current velocity
	if (previous_snapshot != NULL) {
		// Estimate velocity
		const float GAIN = 0.20;
		measured_vel = homing_vector(previous_snapshot, current_snapshot, NULL,
		NULL);
		measured_vel.x *= VISUALHOMING_PERIODIC_FREQ * environment_radius;
		measured_vel.y *= VISUALHOMING_PERIODIC_FREQ * environment_radius;
		if (!isnan(measured_vel.x) && !isnan(measured_vel.y)) {
			vel_vec.x = GAIN * measured_vel.x + (1 - GAIN) * vel_vec.x;
			vel_vec.y = GAIN * measured_vel.y + (1 - GAIN) * vel_vec.y;
		}
		printf("VELOCITY: vx = %+.1f, vy = %+.1f\n", vel_vec.x, vel_vec.y);
		// Update previous snapshot
		snapshot_copy(previous_snapshot, current_snapshot);
	}
	// Estimate homing vector if req'd
	if (target_snapshot != NULL) {
		vec = homing_vector(current_snapshot, target_snapshot,
				&current_warped_snapshot, &target_rotated_snapshot);
		vec.x *= environment_radius;
		vec.y *= environment_radius;
		printf("dX = %+.2f\tdY = %+.2f\tsigma = %+4.0f\n", vec.x, vec.y,
				vec.sigma / M_PI * 180.0);

	}
	// Free current snapshot
	snapshot_free(current_snapshot);
	current_snapshot = NULL; // Signal to video thread to update current observation.
	pthread_mutex_unlock(&video_mutex);

	/*
	 * GPS-less guidance in MODULE mode.
	 */
	if (use_frame_to_frame_velocity) {
		visualhoming_guidance_set_PD(vec.x, -vec.y, vel_vec.x, -vel_vec.y);
	} else {
		visualhoming_guidance_set_pos_error(vec.x, -vec.y);
	}
	visualhoming_guidance_set_heading_error(vec.sigma);

	// Broadcast VELOCITY_ESTIMATE ABI message
//	uint32_t now_ts = get_sys_time_usec();
//	AbiSendMsgVELOCITY_ESTIMATE(VISUALHOMING_SEND_ABI_ID, now_ts, vel_vec.x,
//			-vel_vec.y, 0.0f, 0.0f);

//	// Calculate desired velocity
//	float vx_d, vy_d;
//	vx_d = guidance_h.gains.p * vec.x - guidance_h.gains.d * vel_vec.x;
//	vy_d = -(guidance_h.gains.p * vec.y - guidance_h.gains.d * vel_vec.y);
//	// TODO integrator
//	// Set guidance_opticflow_hover desired_vx, _vy.
//	opticflow_stab.desired_vx = vx_d;
//	opticflow_stab.desired_vy = vx_y;

	/*
	 * Guidance in GUIDED mode.
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
//	guidance_v_from_nav(true);

	/*
	 * Telemetry
	 */
	// Save homing vector for telemetry
	homingvector_telemetry = vec;
	velocity_telemetry = vel_vec;
}

void visualhoming_start(void) {
	printf("visualhoming_start\n");
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

/**
 * Camera callback function. Unwrap panoramic image as test
 * @param img camera image
 * @return pointer to unwrapped horizon image
 */
static struct image_t* camera_callback(struct image_t *img) {
	static float horizon[HORIZON_WIDTH];

	pthread_mutex_lock(&video_mutex);
	// Update current snapshot if required.
	// 'current_snapshot' will be freed and set to NULL by the navigation
	// thread when the image needs to be updated.
	if (current_snapshot == NULL) {
		extract_horizon(horizon, img);
		current_snapshot = snapshot_create(horizon);
		// Instantiate previous snapshot if req'd
		if (previous_snapshot == NULL) {
			previous_snapshot = snapshot_create(horizon);
		}
	}
	// Take or drop target snapshot
	if (take_snapshot) {
		if (target_snapshot != NULL) snapshot_free(target_snapshot);
		target_snapshot = snapshot_create(horizon); // TODO Double work, make more efficient.
		take_snapshot = 0;
	}
	if (drop_snapshot && target_snapshot != NULL) {
		snapshot_free(target_snapshot);
		target_snapshot = NULL;
		drop_snapshot = 0;
	}
	pthread_mutex_unlock(&video_mutex);

	// Draw snapshots or calibration pattern
	if (target_snapshot != NULL) {
		draw_snapshots(img, horizon);
	} else {
		draw_calibration(img);
	}

	return NULL;
}

// Helper functions
static void draw_calibration(struct image_t *img) {
	for (int i = 0; i < 360; i++) {
		struct point_t pt;
		pt = derotated_point(img, i / 180.0 * M_PI, -1.0);
		PIXEL_Y(img, pt.x, pt.y) = 0;
		pt = derotated_point(img, i / 180.0 * M_PI, 1.0);
		PIXEL_Y(img, pt.x, pt.y) = 0;
		pt.x = calibration.center_x * img->w
				+ 0.1 * img->h * cos(i / 180.0 * M_PI);
		pt.y = calibration.center_y * img->h
				+ 0.1 * img->h * sin(i / 180.0 * M_PI);
		PIXEL_Y(img, pt.x, pt.y) = 255;
	}
}

static void draw_snapshots(struct image_t *img, const float horizon[]) {
	float hor[HORIZON_WIDTH];
	// CAUTION: This function is called from the video thread.

	// Draw calibration
	draw_calibration(img);

	// Draw current horizon
	for (int y = 0; y < img->h / 5 * 1; y++) {
		for (int x = 0; x < HORIZON_WIDTH; x++) {
			PIXEL_UV(img, x, y) = 127;
			PIXEL_Y(img, x, y) = horizon[x];
		}
	}
	// Draw current snapshot
	if (current_snapshot != NULL) {
		snapshot_to_horizon(hor, current_snapshot);
		for (int y = img->h / 5 * 1; y < img->h / 5 * 2; y++) {
			for (int x = 0; x < HORIZON_WIDTH; x++) {
				PIXEL_UV(img, x, y) = 127;
				PIXEL_Y(img, x, y) = hor[x];
			}
		}
	}
	// Draw warped current snapshot
	if (current_warped_snapshot != NULL) {
		snapshot_to_horizon(hor, current_warped_snapshot);
		for (int y = img->h / 5 * 2; y < img->h / 5 * 3; y++) {
			for (int x = 0; x < HORIZON_WIDTH; x++) {
				PIXEL_UV(img, x, y) = 127;
				PIXEL_Y(img, x, y) = hor[x];
			}
		}
	}
	// Draw rotated target snapshot
	if (target_rotated_snapshot != NULL) {
		snapshot_to_horizon(hor, target_rotated_snapshot);
		for (int y = img->h / 5 * 3; y < img->h / 5 * 4; y++) {
			for (int x = 0; x < HORIZON_WIDTH; x++) {
				PIXEL_UV(img, x, y) = 127;
				PIXEL_Y(img, x, y) = hor[x];
			}
		}
	}
	// Draw target snapshot
	if (target_snapshot != NULL) {
		snapshot_to_horizon(hor, target_snapshot);
		for (int y = img->h / 5 * 4; y < img->h / 5 * 5; y++) {
			for (int x = 0; x < HORIZON_WIDTH; x++) {
				PIXEL_UV(img, x, y) = 127;
				PIXEL_Y(img, x, y) = hor[x];
			}
		}
	}
}

/**
 * extract_horizon: Extract one-dimensional horizon from 'img'.
 * @param[out] hor Array that holds horizon pixel values
 * @param hor_size Size of 'hor'
 * @param img Image from which the horizon is sampled
 * @param center Center of the panoramic image
 * @param radii Radii over which to average the horizon value
 * @param radii_size Size of 'radii'
 */
static void extract_horizon(float hor[], const struct image_t *img) {
	for (int b = 0; b < HORIZON_WIDTH; ++b) {
		float bearing;
		bearing = b / (float)HORIZON_WIDTH * 2 * M_PI;
		hor[b] = 0;
		for (int e = 0; e < RADIUS_SAMPLING_NPOINTS; ++e) {
			struct point_t p;
			float elevation;
			elevation = -1.0 + 2.0 * (e / (float)(RADIUS_SAMPLING_NPOINTS - 1));
			p = derotated_point(img, bearing, elevation);
			hor[b] += PIXEL_Y(img, p.x, p.y) / (float)RADIUS_SAMPLING_NPOINTS;
		}
	}
}

static struct point_t derotated_point(
		const struct image_t *img,
		float bearing,
		float elevation) {
	// ARDrone2 camera:		+x bottom, 	+y right
	// ARDrone2 attitude:	+x +R13,	+y -R23
	int x, y;
	struct point_t out;
	float radius;

	// Calculate sampling radius
	radius = (calibration.radius_top + calibration.radius_bottom) / 2.0
			+ elevation * (calibration.radius_top - calibration.radius_bottom)
					/ 2.0;
	radius += derotate_gain
			* (cos(bearing) * MAT33_ELMT(attitude, 0, 2)
					- sin(bearing) * MAT33_ELMT(attitude, 1, 2));

	// Calculate pixel coordinates
	x = img->w * calibration.center_x + radius * img->h * sin(bearing);
	y = img->h * calibration.center_y + radius * img->h * cos(bearing);

	// Check bounds
	if (x < 0) x = 0;
	if (x >= img->w) x = img->w - 1;
	if (y < 0) y = 0;
	if (y >= img->h) y = img->h - 1;

	out.x = x;
	out.y = y;
	return out;
}
