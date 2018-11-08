/*
 * Copyright (C) Tom van Dijk
 * Based on slamdunk_udp by kevindehecker
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
 * @file "modules/percevite/percevite.h"
 * @author Tom van Dijk
 * Obstacle detection and avoidance module for PercEvite
 */

#ifndef PERCEVITE_H
#define PERCEVITE_H

#include "firmwares/rotorcraft/navigation.h" // No idea where the bool type comes from :S
#include <stdint.h>

struct percevite_t {
  float timeout; ///< [s] Time since last message
  float time_since_velocity; ///< [s] Time since last velocity update
  uint8_t wp; ///< Waypoint moved by this module
  int request_clock_divider; ///< Count number of times the Go function is called
};
extern struct percevite_t percevite;

struct percevite_settings_t {
  uint8_t request_flags;
};
extern struct percevite_settings_t percevite_settings;

struct percevite_logging_t { ///< Export intermediate values for logging
  struct FloatVect3 velocity; ///< [m/s] Last velocity estimate in body frame
  struct FloatVect3 request; ///< [m] Waypoint requested from SLAMDunk in body frame
  uint8_t request_flags;
  struct FloatVect3 reply; ///< [m] Subgoal returned by SLAMDunk in body frame
  uint8_t reply_flags;
  uint8_t target_wp;  ///< Waypoint to move towards (Note: for logging only!)
};
extern struct percevite_logging_t percevite_logging;

extern void percevite_init(void);
extern void percevite_periodic(void);
extern void percevite_event(void);

extern bool PerceviteInit(uint8_t wp);
extern bool PerceviteGo(uint8_t target_wp);
extern bool PerceviteStay(uint8_t target_wp);

extern bool PerceviteOk(void);

#endif

