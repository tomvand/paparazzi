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
#define VISUALHOMING_HORIZON_RESOLUTION 64
#endif

typedef uint8_t horizon_t[VISUALHOMING_HORIZON_RESOLUTION];

void vh_video_init(void);
void vh_get_current_horizon(horizon_t hor);

typedef void (*video_callback_t)(struct image_t img);
void vh_video_set_callback(video_callback_t cb);

#endif
