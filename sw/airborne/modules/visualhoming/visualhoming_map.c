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

#include "visualhoming_map.h"

#include <stdio.h>

// Map
static struct snapshot_t vh_waypoints[VISUALHOMING_MAX_WAYPOINTS];
static int16_t current_waypoint = -1; // Current homing target

// External functions
int vh_map_push(const struct snapshot_t *ss) {
	if (current_waypoint < VISUALHOMING_MAX_WAYPOINTS - 1) {
		current_waypoint++;
		vh_snapshot_copy(&vh_waypoints[current_waypoint], ss);
	} else {
		printf("[VISUALHOMING] Warning: map buffer is full!\n");
	}
	return current_waypoint;
}

const struct snapshot_t *vh_map_peek(void) {
	if (current_waypoint > 0) {
		return &vh_waypoints[current_waypoint];
	} else {
		return NULL;
	}
}

int vh_map_pop(void) {
	if (current_waypoint >= 0) {
		current_waypoint--;
	}
	return current_waypoint;
}

int vh_map_clear(void) {
	current_waypoint = -1;
	return current_waypoint;
}

int vh_map_get_index(void) {
	return current_waypoint;
}

// Telemetry callback
void send_visualhoming_map_update(
		struct transport_tx *trans,
		struct link_device *dev) {
	// Keep track of last transmitted waypoint
	static int16_t last_waypoint = -1;
	// Transmit new snapshot if waypoint was added or removed
	if (last_waypoint != current_waypoint) {
		if (last_waypoint < current_waypoint) {
			last_waypoint++;
		} else if (last_waypoint > current_waypoint) {
			last_waypoint--;
		}
		pprz_msg_send_VISUALHOMING_MAP_UPDATE(trans, dev, AC_ID,
				&last_waypoint, (uint8_t)sizeof(struct snapshot_t),
				(int8_t*)&vh_waypoints[last_waypoint]);
	}
}
