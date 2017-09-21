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

#ifndef VISUALHOMING_VIDEO_H
#define VISUALHOMING_VIDEO_H

#include "modules/computer_vision/cv.h"
#include "modules/computer_vision/lib/vision/image.h"

#ifndef VISUALHOMING_HORIZON_RESOLUTION
#define VISUALHOMING_HORIZON_RESOLUTION 128
#endif

#define PIXEL_UV(img,x,y) ( ((uint8_t*)((img)->buf))[2*(x) + 2*(y)*(img)->w] )
#define PIXEL_U(img,x,y) ( ((uint8_t*)((img)->buf))[4*(int)((x)/2) + 2*(y)*(img)->w] )
#define PIXEL_V(img,x,y) ( ((uint8_t*)((img)->buf))[4*(int)((x)/2) + 2*(y)*(img)->w + 2] )
#define PIXEL_Y(img,x,y) ( ((uint8_t*)((img)->buf))[2*(x) + 1 + 2*(y)*(img)->w] )

struct calibration_t {
	float center_x;
	float center_y;
	float radius_top;
	float radius_bottom;
};
extern struct calibration_t calibration;
extern float derotate_gain;

typedef float horizon_t[VISUALHOMING_HORIZON_RESOLUTION]; // float instead of uint8_t for easy manipulation in PFFFT.

void vh_video_init(void);
void vh_get_current_horizon(horizon_t hor);
uint32_t vh_get_current_timestamp(void);

typedef void (*video_callback_t)(struct image_t *img);
void vh_video_set_callback(video_callback_t cb);

#endif
