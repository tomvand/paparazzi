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
 * @file "modules/navtools/navtools.c"
 * @author Tom van Dijk
 * Misc functions for flightplans
 */

#include "modules/navtools/navtools.h"

// Internal grid data
static float grid_x0, grid_y0, grid_xX, grid_yX, grid_xY, grid_yY, grid_alt; // Grid axes
static int grid_width, grid_height; // Grid dimensions
static int grid_pos; // Current grid position

bool NavSetupGrid(
		uint8_t wp_origin,
		uint8_t wp_x,
		uint8_t wp_y,
		int width,
		int height) {
	grid_x0 = WaypointX(wp_origin);
	grid_y0 = WaypointY(wp_origin);
	grid_xX = WaypointX(wp_x);
	grid_yX = WaypointY(wp_x);
	grid_xY = WaypointX(wp_y);
	grid_yY = WaypointY(wp_y);
	grid_alt = WaypointAlt(wp_origin);
	grid_width = width;
	grid_height = height;
	grid_pos = 0;

	return FALSE;
}

bool NavGridPos(uint8_t wp_id, float x, float y) {
	struct EnuCoor_f enu;
	enu.x = grid_x0 + x * (grid_xX - grid_x0) + y * (grid_xY - grid_x0);
	enu.y = grid_y0 + x * (grid_yX - grid_y0) + y * (grid_yY - grid_y0);
	enu.z = grid_alt;
	waypoint_set_enu(wp_id, &enu);

	return FALSE;
}


bool NavGridNext(uint8_t wp_id) {
	float x = (float)(grid_pos % grid_width) / (grid_width - 1);
	float y = (int)(grid_pos / grid_width) / (float)(grid_height - 1);
	NavGridPos(wp_id, x, y);
	if (grid_pos + 1 < grid_width * grid_height) grid_pos++;

	return FALSE;
}


bool NavGridComplete() {
	return grid_pos >= grid_width * grid_height - 1;
}

