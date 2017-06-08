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

#include "visualhoming_video.h"

#include "math/pprz_algebra_float.h"
#include "state.h"

#include <pthread.h>
#include <string.h>

// Configuration
#ifndef VISUALHOMING_CAMERA
#define VISUALHOMING_CAMERA bottom_camera
#endif
#ifndef VISUALHOMING_RADIAL_SAMPLES
#define VISUALHOMING_RADIAL_SAMPLES 5
#endif

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
#ifndef VISUALHOMING_DEROTATE_GAIN
#define VISUALHOMING_DEROTATE_GAIN 0.08
#endif

// Settings
struct calibration_t calibration = {
		.center_x = VISUALHOMING_CENTER_X,
		.center_y = VISUALHOMING_CENTER_Y,
		.radius_top = VISUALHOMING_RADIUS_TOP,
		.radius_bottom = VISUALHOMING_RADIUS_BOTTOM };
float derotate_gain = VISUALHOMING_DEROTATE_GAIN;

// Static variables
static video_callback_t callback = 0;

// Cross-thread variables
static pthread_mutex_t video_mutex;
static horizon_t current_horizon;
static struct FloatRMat attitude;

// Static function declarations
static struct image_t* camera_callback(struct image_t *img);
static void extract_horizon(const struct image_t *img, horizon_t hor);
static struct point_t derotated_point(
		const struct image_t *img,
		float bearing,
		float elevation);
static void draw_calibration(struct image_t *img);

// External functions
void vh_video_init(void) {
	float_rmat_identity(&attitude);
	cv_add_to_device(&VISUALHOMING_CAMERA, camera_callback);
}

void vh_get_current_horizon(horizon_t hor) {
	// Note: also updates attitude from main thread!
	struct FloatRMat *current_att;
	current_att = stateGetNedToBodyRMat_f();

	pthread_mutex_lock(&video_mutex);
	memcpy(hor, current_horizon, sizeof(current_horizon));
	MAT33_COPY(attitude, *current_att);
	pthread_mutex_unlock(&video_mutex);
}

void vh_video_set_callback(video_callback_t cb) {
	callback = cb;
}

// Static functions
static struct image_t* camera_callback(struct image_t *img) {
	// Caution: runs on video thread
	pthread_mutex_lock(&video_mutex);
	extract_horizon(img, current_horizon);
	pthread_mutex_unlock(&video_mutex);

	draw_calibration(img);
	if (callback) callback(img);
	return img;
}

static void extract_horizon(const struct image_t *img, horizon_t hor) {
	for (int b = 0; b < VISUALHOMING_HORIZON_RESOLUTION; ++b) {
		float bearing;
		bearing = b / (float)VISUALHOMING_HORIZON_RESOLUTION * 2 * M_PI;
		float pixel_y = 0.0;
		for (int e = 0; e < VISUALHOMING_RADIAL_SAMPLES; ++e) {
			struct point_t p;
			float elevation;
			elevation = -1.0
					+ 2.0 * (e / (float)(VISUALHOMING_RADIAL_SAMPLES - 1));
			p = derotated_point(img, bearing, elevation);
			pixel_y += PIXEL_Y(img, p.x,
					p.y) / (float)VISUALHOMING_RADIAL_SAMPLES;
		}
		hor[b] = pixel_y;
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
