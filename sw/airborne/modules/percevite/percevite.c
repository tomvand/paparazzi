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
 * @file "modules/percevite/percevite.c"
 * @author Tom van Dijk
 * Obstacle detection and avoidance module for PercEvite
 */

#include "modules/percevite/percevite.h"
#include "modules/percevite/percevite_messages.h"

#include "pprzlink/pprz_transport.h"
#include "mcu_periph/udp.h"
#include "subsystems/datalink/downlink.h"

#include "firmwares/rotorcraft/guidance/guidance_h_ref.h"

#include "firmwares/rotorcraft/navigation.h"
#include "subsystems/navigation/waypoints.h"

#include "generated/modules.h"

#include <stdio.h>

// Reference deceleration rate in m/s2
#ifndef PERCEVITE_DECELERATION
#define PERCEVITE_DECELERATION 0.5
#endif

// Minimum distance to keep towards obstacle [m]
#ifndef PERCEVITE_MINIMUM_DISTANCE
#define PERCEVITE_MINIMUM_DISTANCE 3.0
#endif

// Assumed delay between image capture and receival of safe distance measurement [s]
#ifndef PERCEVITE_IMAGE_LAG
#define PERCEVITE_IMAGE_LAG 1.0
#endif

// Max allowed time between images [s]
#ifndef PERCEVITE_IMAGE_TIMEOUT
#define PERCEVITE_IMAGE_TIMEOUT 0.5
#endif

// Fraction of pixels for which a depth measurement must be available
#ifndef PERCEVITE_VALID_PIXELS_THRESHOLD
#define PERCEVITE_VALID_PIXELS_THRESHOLD 0.50
#endif

// Enable writing to guidance h max velocity
#ifndef PERCEVITE_SET_NAV_VMAX
#define PERCEVITE_SET_NAV_VMAX TRUE
#endif

struct percevite_t percevite = {
    .max_velocity = 0.0,
    .time_since_image = 0.0,
};

struct percevite_settings_t percevite_settings = {
    .deceleration = PERCEVITE_DECELERATION,
    .minimum_distance = PERCEVITE_MINIMUM_DISTANCE,
    .image_lag = PERCEVITE_IMAGE_LAG,
    .pixels_threshold = PERCEVITE_VALID_PIXELS_THRESHOLD,
};

static void slamdunk_init(void);
static void slamdunk_event(void);
static void slamdunk_parse_message(void);
static void slamdunk_send_message(union paparazzi_to_slamdunk_msg_t *msg);

void percevite_init(void) {
  slamdunk_init();
}

void percevite_periodic(void) {
  // Increase image timer
  percevite.time_since_image += PERCEVITE_PERIODIC_PERIOD;
  if(percevite.time_since_image > PERCEVITE_IMAGE_TIMEOUT) {
    printf("[percevite] WARNING: Image timeout exceeded!\n");
    percevite.max_velocity = 0.0;
    // Output to guidance H
#if PERCEVITE_SET_NAV_VMAX
    gh_set_max_speed(percevite.max_velocity);
#endif
  }
  // Send dummy message to slamdunk
  union paparazzi_to_slamdunk_msg_t msg = {
      .text = "Hello SLAMDunk!"
  };
  slamdunk_send_message(&msg);
}

void percevite_event(void) {
  slamdunk_event();
}

static void percevite_on_message(union slamdunk_to_paparazzi_msg_t *msg) {
  // Calculate max velocity
  float braking_distance = (0.10 * msg->safe_distance) - percevite_settings.minimum_distance;
  if(braking_distance > 0.0
      && msg->valid_pixels > 255 * percevite_settings.pixels_threshold) {
//    percevite.max_velocity = sqrtf(2.0 * braking_distance * percevite_settings.deceleration);
    percevite.max_velocity = sqrtf(2.0 * braking_distance * percevite_settings.deceleration
        + (percevite_settings.deceleration * percevite_settings.deceleration) * (percevite_settings.image_lag * percevite_settings.image_lag))
        - percevite_settings.deceleration * percevite_settings.image_lag;
    if(percevite.max_velocity > GUIDANCE_H_REF_MAX_SPEED) {
      percevite.max_velocity = GUIDANCE_H_REF_MAX_SPEED;
    }
  } else {
    percevite.max_velocity = 0.0;
  }
  // Print max velocity
  printf("[percevite] Max velocity = %.1fm/s\t(safe distance: %.1fm, valid pixels: %.0f%%)\n",
      percevite.max_velocity,
      0.10 * msg->safe_distance,
      msg->valid_pixels / 2.55);
  // Output to guidance H
#if PERCEVITE_SET_NAV_VMAX
  gh_set_max_speed(percevite.max_velocity);
#endif
  // Reset timeout
  percevite.time_since_image = 0.0;
}


/*
 * Navigation functions
 */
/*
 * Outline:
 * Navigation is performed by moving towards the percevite waypoint instead of
 * the real target. The percevite wp is set at a safe distance from the current
 * position in the direction of the target waypoint.
 * The percevite waypoint position is updated with each call to PerceviteGo or
 * PerceviteStay based on the last received safe distance.
 */

static uint8_t percevite_wp; ///< Waypoint that will be moved by the percevite module

///< Sets the heading towards the target_wp as in nav_set_heading_towards_waypoint,
/// but also returns a bool wether the aircraft is currently pointing in the right
/// direction +- a few degrees.
static bool aim_at_waypoint(uint8_t target_wp) {
  const float threshold = 0.20; // [rad]
  struct FloatVect2 target = {WaypointX(target_wp), WaypointY(target_wp)};
  struct FloatVect2 pos_diff;
  VECT2_DIFF(pos_diff, target, *stateGetPositionEnu_f());
  // don't change heading if closer than 0.5m to target
  if(VECT2_NORM2(pos_diff) < 0.25) {
    return TRUE; // Currently at waypoint, accept all headings
  }
  float target_heading = atan2f(pos_diff.x, pos_diff.y); // Note: ENU, CW from North
  nav_set_heading_rad(target_heading);
  // Compare to current heading
  float heading_error = target_heading - stateGetNedToBodyEulers_f()->psi; // ENU
  while(heading_error < -M_PI) heading_error += 2*M_PI;
  while(heading_error > M_PI) heading_error -= 2*M_PI;
  heading_error = fabsf(heading_error);
  return heading_error < threshold;
}

static void set_percevite_wp(uint8_t target_wp, float distance) {
  struct EnuCoor_f *pos = stateGetPositionEnu_f();
  struct EnuCoor_f wp_pos = { WaypointX(target_wp), WaypointY(target_wp), WaypointAlt(target_wp) };
  struct FloatVect2 move;
  VECT2_DIFF(move, wp_pos, *pos); // Vector from current pos to target_wp
  float distance_to_wp = float_vect2_norm(&move); // Distance between current position and target
  if(distance_to_wp < distance) {
    // Waypoint is withing safe distance, go there directly
    waypoint_copy(percevite_wp, target_wp);
  } else {
    // Move percevite_wp towards target_wp but keep it within the safe distance
    float_vect2_normalize(&move); // Unit vector towards target_wp
    VECT2_SMUL(move, move, distance); // Vector of length distance towards target_wp
    VECT2_SUM(wp_pos, *pos, move); // Write new wp x and y (pos + move). Keep Z of target wp.
    waypoint_set_enu(percevite_wp, &wp_pos);
  }
}

/**
 * Assign waypoint to PercEvite module. This waypoint will be used for PerceviteGo
 * and PerceviteStay commands. The waypoint will only be placed in regions that
 * are determined to be safe.
 * @param wp
 * @return
 */
bool PerceviteInit(uint8_t wp) {
  percevite_wp = wp;
  return FALSE; // No looping req'd
}

bool PerceviteGo(uint8_t target_wp) {
  NavSetWaypointHere(percevite_wp);
  if(aim_at_waypoint(target_wp)) {
    set_percevite_wp(percevite_wp, percevite.safe_distance);
  }
  NavGotoWaypoint(percevite_wp);
  return sqrtf(get_dist2_to_waypoint(target_wp)) > ARRIVED_AT_WAYPOINT; // Keep looping until arrived at target_wp
}

bool PerceviteStay(uint8_t target_wp) {
  PerceviteGo(target_wp);
  return TRUE; // Keep looping
}


/*
 * Communication with SLAMDunk
 */

struct slamdunk_t {
  struct link_device *device;      ///< The device which is uses for communication
  struct pprz_transport transport; ///< The transport layer (PPRZ)
  uint8_t msg_buf[128];            ///< Message buffer
  bool msg_available;              ///< If we received a message
};
static struct slamdunk_t slamdunk = {
    .device = (&((PERCEVITE_UDP).device)),
    .msg_available = false
};

static void slamdunk_init(void) {
  printf("[percevite] Initialize pprzlink (UDP) to SLAMDunk... ");
  pprz_transport_init(&slamdunk.transport);
  printf("ok\n");
}

static void slamdunk_event(void) {
  pprz_check_and_parse(slamdunk.device, &slamdunk.transport, slamdunk.msg_buf, &slamdunk.msg_available);
  if(slamdunk.msg_available) {
    slamdunk_parse_message();
    slamdunk.msg_available = FALSE;
  }
}

static void slamdunk_parse_message(void) {
  union slamdunk_to_paparazzi_msg_t *msg;
  switch(slamdunk.msg_buf[3]) { // For pprzlink v2.0
    case PPRZ_MSG_ID_PAYLOAD:
      msg = (union slamdunk_to_paparazzi_msg_t *)&slamdunk.msg_buf[5];
      percevite_on_message(msg);
      break;
    default:
      break;
  }
}

static void slamdunk_send_message(union paparazzi_to_slamdunk_msg_t *msg) {
  pprz_msg_send_PAYLOAD(&(slamdunk.transport.trans_tx), slamdunk.device,
      AC_ID, sizeof(*msg), &(msg->bytes));
}

