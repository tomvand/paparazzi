/*
 * Copyright (C) Tom van Dijk <tomvand@users.noreply.github.com>
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

/** @file "modules/visualhoming/visualhoming.c"
 * @author Tom van Dijk <tomvand@users.noreply.github.com>
 * Visual homing
 */

#include "modules/visualhoming/visualhoming.h"

#include "generated/flight_plan.h"
#include "firmwares/rotorcraft/navigation.h"
#include "subsystems/navigation/waypoints.h"
#include "subsystems/abi.h"
#include "subsystems/datalink/telemetry.h"
#include "math/pprz_algebra_int.h"

#ifndef RELATIVE_LOCALIZATION_ABI_ID
#define RELATIVE_LOCALIZATION_ABI_ID RELATIVE_LOCALIZATION_VISUALHOMING_ID
#endif

#ifndef HOMING_VECTOR_SCALE
#define HOMING_VECTOR_SCALE 5.0   // Multiplier from homing vector length to m
#endif

#ifndef HOMING_WAYPOINT
#define HOMING_WAYPOINT WP_HOMING_VISUAL
#endif

static abi_event relative_localization_ev;


static struct {
  int32_t vector_id;
  struct EnuCoor_f target;
  struct EnuCoor_f vector;
  struct EnuCoor_f ins;
  bool is_new_data;
} homing;

static struct {
  int32_t vector_id;
  struct EnuCoor_f target;
  struct EnuCoor_f vector;
  struct EnuCoor_f ins;
  bool is_new_data;
} telemetry;


static void send_visualhoming(struct transport_tx *trans, struct link_device *dev) {
  if (telemetry.is_new_data) {
    pprz_msg_send_VISUALHOMING(trans, dev, AC_ID,
        &telemetry.vector_id,
        &telemetry.target.x, &telemetry.target.y, &telemetry.target.z,
        &telemetry.vector.x, &telemetry.vector.y, &telemetry.vector.z,
        &telemetry.ins.x, &telemetry.ins.y, &telemetry.ins.z);
    telemetry.is_new_data = FALSE;
  }
}

static void relative_localization_cb(uint8_t sender_id __attribute__((unused)), int32_t id __attribute__((unused)),
    float x, float y, float z,
    float vx __attribute__((unused)), float vy __attribute__((unused)), float vz __attribute__((unused)))
{
  homing.vector_id = id;
  homing.vector.x = x;
  homing.vector.y = y;
  homing.vector.z = z;

  homing.ins = *stateGetPositionEnu_f();
  struct EnuCoor_f vec;
  VECT3_SMUL(vec, homing.vector, HOMING_VECTOR_SCALE);
  VECT3_SUM(homing.target, homing.ins, vec)

  homing.is_new_data = TRUE;
}

void visualhoming_init(void)
{
  AbiBindMsgRELATIVE_LOCALIZATION(RELATIVE_LOCALIZATION_ABI_ID, &relative_localization_ev, relative_localization_cb);
#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_VISUALHOMING, send_visualhoming);
#endif // PERIODIC_TELEMETRY
}

void visualhoming_periodic(void)
{
  // your periodic code here.
  // freq = 10.0 Hz
}


// Flight plan/navigation functions

bool NavHoming(void) {
  // Update guidance vector
  if (homing.is_new_data) {
    struct EnuCoor_i from_i;
    from_i.x = POS_BFP_OF_REAL(homing.ins.x);
    from_i.y = POS_BFP_OF_REAL(homing.ins.y);
    from_i.z = POS_BFP_OF_REAL(homing.ins.z);
    struct EnuCoor_i tgt_i;
    tgt_i.x = POS_BFP_OF_REAL(homing.target.x);
    tgt_i.y = POS_BFP_OF_REAL(homing.target.y);
    tgt_i.z = POS_BFP_OF_REAL(homing.target.z);
    waypoint_move_enu_i(HOMING_WAYPOINT, &tgt_i);
//    NavGotoWaypoint(HOMING_WAYPOINT);
    nav_route(&from_i, &tgt_i);
    homing.is_new_data = FALSE;
    // Copy data for telemetry
    telemetry.vector_id = homing.vector_id;
    telemetry.target = homing.target;
    telemetry.vector = homing.vector;
    telemetry.ins = homing.ins;
    telemetry.is_new_data = TRUE;
  }
  // TODO Arrival detection
  return TRUE;
}


