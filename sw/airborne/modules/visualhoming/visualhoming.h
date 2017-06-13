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

#ifndef VISUALHOMING_H
#define VISUALHOMING_H

#include <stdbool.h>

// Settings
extern float vh_environment_radius;

/* Control modes */
enum visualhoming_mode_t {
	VH_MODE_STOP,
	VH_MODE_SNAPSHOT,
	VH_MODE_ODOMETRY,
	VH_RECORD_ROUTE,
	VH_FOLLOW_ROUTE,
	VH_MODE_NOCMD,
};

/* GCS buttons */
extern enum visualhoming_mode_t vh_mode_cmd;

/* Navigation functions for flightplan */
bool VisualHomingTakeSnapshot(void);
bool VisualHomingRecordOdometry(void);
bool NavVisualHoming(void);

/* Module functions */
void visualhoming_init(void);
void visualhoming_periodic(void);
void visualhoming_start(void);
void visualhoming_stop(void);

#endif
