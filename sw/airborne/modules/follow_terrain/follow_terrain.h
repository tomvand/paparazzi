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
 * @file "modules/follow_terrain/follow_terrain.h"
 * @author Tom van Dijk
 * Adjust a waypoint altitude using the sonar to follow terrain.
 */

#ifndef FOLLOW_TERRAIN_H
#define FOLLOW_TERRAIN_H

#include <stdint.h>

extern void follow_terrain_init(void);
extern int FollowTerrainInit(uint8_t wp); ///< Call from flight plan to set waypoint

#endif

