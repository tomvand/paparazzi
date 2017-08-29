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
 * @file "modules/navtools/navtools.h"
 * @author Tom van Dijk
 * Misc functions for flightplans
 */

#ifndef NAVTOOLS_H
#define NAVTOOLS_H

#include "subsystems/navigation/waypoints.h"

/**
 * Set up grid for flightplan
 *
 * @param wp_origin Grid origin
 * @param wp_x Grid X axis
 * @param wp_y Grid Y axis
 * @param width Number of x positions
 * @param height Number of y positions
 * @return
 */
bool NavSetupGrid(
		uint8_t wp_origin,
		uint8_t wp_x,
		uint8_t wp_y,
		int width,
		int height);

/**
 * Move to a specific grid position. Does not change current index.
 * @param x
 * @param y
 * @return
 */
bool NavGridPos(uint8_t wp_id, float x, float y);

/**
 * Advance waypoint to next position in grid
 * @param wp_id
 * @return
 */
bool NavGridNext(uint8_t wp_id);

/**
 * Returns true if all grid positions have been visited
 * @return
 */
bool NavGridComplete(void);

#endif

