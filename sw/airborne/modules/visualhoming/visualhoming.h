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
extern float vh_odometry_arrival_threshold;
extern float vh_snapshot_arrival_threshold;
extern float vh_snapshot_arrival_max_odo;
extern float vh_snapshot_initial_threshold;
extern float vh_snapshot_trigger_threshold;
extern int vh_snapshot_trigger_from_initial;

/* Control modes */
enum vh_sequencer_mode_t {
	VH_SEQ_STOP,
	VH_SEQ_SINGLE,
	VH_SEQ_ROUTE,
	VH_SEQ_NOCMD
};
enum vh_input_mode_t {
	VH_IN_SNAPSHOT,
	VH_IN_ODO,
	VH_IN_NOCMD
};

/* GCS buttons */
extern enum vh_sequencer_mode_t vh_gcs_seq_mode;
extern enum vh_input_mode_t vh_gcs_input_mode;

/* Navigation functions for flightplan */
bool VisualHomingSetSnapshotMode(void);
bool VisualHomingSetOdometryMode(void);

bool VisualHomingStop(void);
bool VisualHomingRecordSingle(void);
bool VisualHomingRecordRoute(void);

bool VisualHomingCompleted(void);

/* Module functions */
void visualhoming_init(void);
void visualhoming_periodic(void);
void visualhoming_start(void);
void visualhoming_stop(void);

#endif
