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
#include "subsystems/datalink/telemetry.h"

#include "firmwares/rotorcraft/navigation.h"
#include "subsystems/navigation/waypoints.h"

#include "subsystems/abi.h"

#include "generated/modules.h"

#include <stdio.h>

// Minimum distance to keep towards obstacle [m]
#ifndef PERCEVITE_MINIMUM_DISTANCE
#define PERCEVITE_MINIMUM_DISTANCE 2.0
#endif

// Fraction of pixels for which a depth measurement must be available
#ifndef PERCEVITE_VALID_PIXELS_THRESHOLD
#define PERCEVITE_VALID_PIXELS_THRESHOLD 0.50
#endif

// Send velocity estimate through ABI
#ifndef PERCEVITE_ESTIMATE_VELOCITY
#define PERCEVITE_ESTIMATE_VELOCITY FALSE
#endif

// Velocity estimate variance (default SD 50 cm/s = 0.25)
#ifndef PERCEVITE_VELOCITY_R
#define PERCEVITE_VELOCITY_R 0.25
#endif

// Maximum allowable difference with INS velocity [m/s]
#ifndef PERCEVITE_VELOCITY_MAX_ERROR
#define PERCEVITE_VELOCITY_MAX_ERROR 1.0
#endif

// Max allowed time between images [s]
#ifndef PERCEVITE_IMAGE_TIMEOUT
#define PERCEVITE_IMAGE_TIMEOUT 1.0
#endif

// Max allowed time between velocity updates [s]
#ifndef PERCEVITE_VELOCITY_TIMEOUT
#define PERCEVITE_VELOCITY_TIMEOUT 1.0
#endif

#define INVALID_WAYPOINT 255

struct percevite_t percevite = {
    .safe_region.distance = 0.0,
    .safe_region.seq = 0,
    .time_since_safe_distance = 0.0,
    .time_since_velocity = 0.0,
    .wp = INVALID_WAYPOINT, // Should cause an error when not initialized in flight plan!
};

struct percevite_settings_t percevite_settings = {
    .minimum_distance = PERCEVITE_MINIMUM_DISTANCE,
    .pixels_threshold = PERCEVITE_VALID_PIXELS_THRESHOLD,
};

struct percevite_logging_t percevite_logging = {
    .velocity = { 0.0, 0.0, 0.0 },
    .target_wp = INVALID_WAYPOINT,
    .raw_distance = 0.0,
};

static void slamdunk_init(void);
static void slamdunk_event(void);
static void slamdunk_parse_message(void);
static void slamdunk_send_message(union paparazzi_to_slamdunk_msg_t *msg);

static void send_percevite(struct transport_tx *trans, struct link_device *dev);

void percevite_init(void) {
  slamdunk_init();
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_PERCEVITE, send_percevite);
}

void percevite_periodic(void) {
  // Update timeouts
  percevite.time_since_safe_distance += PERCEVITE_PERIODIC_PERIOD;
  percevite.time_since_velocity += PERCEVITE_PERIODIC_PERIOD;
  if(percevite.time_since_safe_distance > PERCEVITE_IMAGE_TIMEOUT) {
    printf("[percevite] WARNING: Image timeout exceeded!\n");
    percevite.safe_region.distance = 0.0;
    percevite.safe_region.seq++;
  }
  if(percevite.time_since_velocity > PERCEVITE_VELOCITY_TIMEOUT) {
    printf("[percevite] WARNING: Velocity timeout exceeded!\n");
    percevite.safe_region.distance = 0.0;
    percevite.safe_region.seq++;
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

static void percevite_on_safe_distance(union slamdunk_to_paparazzi_msg_t *msg) {
  // Store safe distance for navigation
  if(msg->valid_pixels > percevite_settings.pixels_threshold * 255) {
    percevite.safe_region.distance = 0.10 * msg->safe_distance - percevite_settings.minimum_distance;
    if(percevite.safe_region.distance < 0) {
      percevite.safe_region.distance = 0.0;
    }
  } else {
    percevite.safe_region.distance = 0.0;
  }
  percevite.safe_region.seq++;
  printf("[percevite] Safe distance: %.1fm (ID: %d, Valid pixels: %.0f%%)\n",
      percevite.safe_region.distance, percevite.safe_region.seq, msg->valid_pixels / 2.55);
  // Reset timeout
  percevite.time_since_safe_distance = 0.0;
  // Logging
  percevite_logging.raw_distance = 0.10 * msg->safe_distance;
  percevite_logging.valid_pixels = msg->valid_pixels / 255.0;
}

static void percevite_on_velocity(union slamdunk_to_paparazzi_msg_t *msg) {
  // Velocity estimate
  printf("[percevite] Velocity: %.1f %.1f %.1f m/s\n",
      msg->vx, msg->vy, msg->vz);
  // Sanity check
  struct FloatVect3 vel_body;
  struct FloatRMat *R = stateGetNedToBodyRMat_f();
  struct NedCoor_f *vel_ltp = stateGetSpeedNed_f();
  MAT33_VECT3_MUL(vel_body, *R, *vel_ltp);
  if(fabsf(msg->vx - vel_body.x) < PERCEVITE_VELOCITY_MAX_ERROR &&
      fabsf(msg->vy - vel_body.y) < PERCEVITE_VELOCITY_MAX_ERROR &&
      fabsf(msg->vz - vel_body.z) < PERCEVITE_VELOCITY_MAX_ERROR) {
    // Velocity seems ok, send to INS
#if PERCEVITE_ESTIMATE_VELOCITY
    AbiSendMsgVELOCITY_ESTIMATE(VEL_PERCEVITE_ID, get_sys_time_usec(),
        msg->vx, msg->vy, msg->vz,
        PERCEVITE_VELOCITY_R, PERCEVITE_VELOCITY_R, PERCEVITE_VELOCITY_R);
#endif
  } else {
    // Sanity check failed
    printf("INS velocity: %.1f %.1f %.1f m/s\n", vel_body.x, vel_body.y, vel_body.z);
    printf("Velocity change too large! Ignoring...\n");
  }
  percevite_logging.velocity.x = msg->vx;
  percevite_logging.velocity.y = msg->vy;
  percevite_logging.velocity.z = msg->vz;
  percevite.time_since_velocity = 0.0; // Want to move this to sanity check ok, but this causes timeout during yaw
}

static void percevite_on_message(union slamdunk_to_paparazzi_msg_t *msg) {
  if(msg->flags & SD_MSG_FLAG_SAFE_DISTANCE) percevite_on_safe_distance(msg);
  if(msg->flags & SD_MSG_FLAG_VELOCITY) percevite_on_velocity(msg);
}

static void send_percevite(struct transport_tx *trans, struct link_device *dev) {
  uint8_t ok = PerceviteOk();
  pprz_msg_send_PERCEVITE(trans, dev, AC_ID,
      &ok,
      &percevite_logging.velocity.x, &percevite_logging.velocity.y, &percevite_logging.velocity.z,
      &percevite.time_since_velocity,
      &percevite.safe_region.distance, &percevite.safe_region.seq,
      &percevite_logging.raw_distance, &percevite_logging.valid_pixels,
      &percevite.time_since_safe_distance,
      &percevite.wp, &percevite_logging.target_wp);
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

/**
 * Sets the heading towards the target_wp as in nav_set_heading_towards_waypoint,
 * but also returns a bool wether the aircraft is currently pointing in the right
 * direction +- a few degrees.
 * @param target_wp
 * @return pointed at target
 */
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
  struct FloatVect2 diff;
  VECT2_DIFF(diff, wp_pos, *pos); // Vector from current pos to target_wp
  printf("[percevite] diff x = %.1f, y = %.1f\n", diff.x, diff.y);
  float distance_to_wp = float_vect2_norm(&diff); // Distance between current position and target
  if(distance_to_wp < distance) {
    // Waypoint is within safe distance, go there directly
    waypoint_copy(percevite.wp, target_wp);
  } else {
    // Move percevite_wp towards target_wp but keep it within the safe distance
    float_vect2_normalize(&diff); // Unit vector towards target_wp
    VECT2_SMUL(diff, diff, distance); // Vector of length distance towards target_wp
    VECT2_SUM(wp_pos, *pos, diff); // Write new wp x and y (pos + move). Keep Z of target wp.
    waypoint_set_enu(percevite.wp, &wp_pos);
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
  percevite.wp = wp;
  return FALSE; // No looping req'd
}

bool PerceviteGo(uint8_t target_wp) {
  static uint32_t last_seq = 0;
  percevite_logging.target_wp = target_wp;
  if(aim_at_waypoint(target_wp)) {
    if(percevite.time_since_safe_distance > PERCEVITE_IMAGE_TIMEOUT) {
      // Safe distance not available. Maintain current position.
      NavSetWaypointHere(percevite.wp);
    } else if (percevite.safe_region.seq > last_seq) {
      // New safe distance available while facing target_wp.
      last_seq = percevite.safe_region.seq;
      if(percevite.safe_region.distance > 0) {
        // Move waypoint forwards within safe distance.
        // Note: do not move waypoint if safe distance is 0, causes drift!
        set_percevite_wp(target_wp, percevite.safe_region.distance);
      }
    } // else: no new information, maintain safe waypoint
  } else {
    // Not facing target_wp. Maintain current position.
    NavSetWaypointHere(percevite.wp);
  }
  NavGotoWaypoint(percevite.wp);
  return sqrtf(get_dist2_to_waypoint(target_wp)) > ARRIVED_AT_WAYPOINT; // Keep looping until arrived at target_wp
}

bool PerceviteStay(uint8_t target_wp) {
  PerceviteGo(target_wp);
  return TRUE; // Keep looping
}

bool PerceviteOk(void) {
  return percevite.time_since_velocity < PERCEVITE_VELOCITY_TIMEOUT &&
      percevite.time_since_safe_distance < PERCEVITE_IMAGE_TIMEOUT;
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

