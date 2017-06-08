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

#include "visualhoming_statemachine.h"

#include "visualhoming_core.h"
#include "visualhoming_odometry.h"

#include <stdio.h>

// Settings
enum vh_state_t vh_state_cmd = VH_STATE_AUTO; // State set by user.

// State machine declaration
struct state_t {
	void (*enter)(void);
	struct state_t *(*run)(void);
};

static void continuous_run(void);

static enum vh_state_t state_standby_run(void);
static enum vh_state_t state_store_snapshot_run(void);
static void state_record_odometry_enter(void);
static enum vh_state_t state_record_odometry_run(void);
static enum vh_state_t state_follow_odometry_run(void);
static enum vh_state_t state_home_to_snapshot_run(void);

static const struct state_t states[] = {
		{ .run = state_standby_run },
		{ .run = state_store_snapshot_run },
		{ .run = state_record_odometry_run, .enter =
				state_record_odometry_enter },
		{ .run = state_follow_odometry_run },
		{ .run = state_home_to_snapshot_run }
};

// External functions
void vh_statemachine_run(void) {
	static struct state_t *current_state_ptr = states;

	// Run continuous task
	continuous_run();
	// Run current state
	enum vh_state_t next_state_idx = current_state_ptr->run();
	// User override of state switching logic
	if (vh_state_cmd != VH_STATE_AUTO) {
		next_state_idx = vh_state_cmd;
	}
	// Switch to next state
	struct state_t *next_state_ptr = states + next_state_idx;
	if (next_state_ptr != current_state_ptr) {
		if (next_state_ptr->enter) next_state_ptr->enter();
	}
	current_state_ptr = next_state_ptr;
}

// Static functions
static void continuous_run(void) {
	// TODO Get visual input
	// TODO Update velocity estimate
}

static enum vh_state_t state_standby_run(void) {
	// TODO Wait for user command to start
	return VH_STATE_STANDBY;
}

static enum vh_state_t state_store_snapshot_run(void) {
	// TODO Store current input as snapshot
	return VH_STATE_RECORD_ODOMETRY;
}

static void state_record_odometry_enter(void) {
	// TODO Reset this odometry vector
}
static enum vh_state_t state_record_odometry_run(void) {
	// TODO Update odometry with estimated velocity
	// TODO Decide when to create new snapshot
	return VH_STATE_RECORD_ODOMETRY;
}

static enum vh_state_t state_follow_odometry_run(void) {
	// TODO Set guidance towards odometry vector
	// TODO Update odometry with estimated velocity
	// TODO Detect arrival
	return VH_STATE_FOLLOW_ODOMETRY;
}

static enum vh_state_t state_home_to_snapshot_run(void) {
	// TODO Calculate homing vector
	// TODO Set guidance towards homing vector
	// TODO Detect arrival
	return VH_STATE_HOME_TO_SNAPSHOT;
}

