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
 * @file "modules/follow_terrain/follow_terrain.c"
 * @author Tom van Dijk
 * Adjust a waypoint altitude using the sonar to follow terrain.
 */

#include "modules/follow_terrain/follow_terrain.h"

#include "state.h"
#include "subsystems/abi.h"
#include "subsystems/navigation/waypoints.h"
#include "firmwares/rotorcraft/navigation.h"

#ifndef FOLLOW_TERRAIN_TARGET_AGL
#define FOLLOW_TERRAIN_TARGET_AGL 3.0
#endif
#ifndef FOLLOW_TERRAIN_KP
#define FOLLOW_TERRAIN_KP 0.30
#endif
#ifndef FOLLOW_TERRAIN_MAX_ERROR
#define FOLLOW_TERRAIN_MAX_ERROR 0.30
#endif

#define INVALID_WAYPOINT 255

struct follow_terrain_t {
  float target_agl;
  float kp;
  uint8_t wp;
};
static struct follow_terrain_t follow_terrain;


/** ABI binding for AGL.
 * Usually this is comes from sonar or gps.
 */
#ifndef FOLLOW_TERRAIN_AGL_ID
#define FOLLOW_TERRAIN_AGL_ID ABI_BROADCAST
#endif
static abi_event agl_ev;                 ///< The agl ABI event
static void agl_cb(uint8_t sender_id, float distance) {
  if(follow_terrain.wp != INVALID_WAYPOINT) {
    float delta = follow_terrain.target_agl - distance;
    if(delta > FOLLOW_TERRAIN_MAX_ERROR) delta = FOLLOW_TERRAIN_MAX_ERROR;
    if(delta < -FOLLOW_TERRAIN_MAX_ERROR) delta = -FOLLOW_TERRAIN_MAX_ERROR;
    float dist = sqrtf(get_dist2_to_waypoint(follow_terrain.wp));
    if(dist > ARRIVED_AT_WAYPOINT) {
      float alt = stateGetPositionEnu_f()->z + follow_terrain.kp * delta * dist;
      waypoint_set_alt(follow_terrain.wp, alt);
    }
  }
}

void follow_terrain_init(void) {
  follow_terrain.target_agl = FOLLOW_TERRAIN_TARGET_AGL;
  follow_terrain.kp = FOLLOW_TERRAIN_KP;
  follow_terrain.wp = INVALID_WAYPOINT;
  AbiBindMsgAGL(FOLLOW_TERRAIN_AGL_ID, &agl_ev, agl_cb);
}

int FollowTerrainInit(uint8_t wp) {
  follow_terrain.wp = wp;
  return FALSE; // Do not loop
}


