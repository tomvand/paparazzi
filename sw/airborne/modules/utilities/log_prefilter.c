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

/** @file "modules/utilities/log_prefilter.c"
 * @author Tom van Dijk <tomvand@users.noreply.github.com>
 * Low-pass filter IMU and CMD signals before logging at a lower sampling rate.
 */

#include "modules/utilities/log_prefilter.h"

#include "generated/modules.h"
#include "subsystems/datalink/telemetry.h"

#include "state.h"
#include "cmd_log.h"

#include "filters/low_pass_filter.h"

#include <math.h>


#ifndef LOG_PREFILTER_CUTOFF_HZ
#define LOG_PREFILTER_CUTOFF_HZ 30.0
#endif
#define LOG_PREFILTER_TAU (1.0 / (2 * M_PI * LOG_PREFILTER_CUTOFF_HZ))

#ifndef LOG_PREFILTER_ORDER
#define LOG_PREFILTER_ORDER 4
#endif

#if LOG_PREFILTER_ORDER == 2
typedef Butterworth2LowPass filter_t;
#define init_filter(...) init_butterworth_2_low_pass(__VA_ARGS__)
#define update_filter(...) update_butterworth_2_low_pass(__VA_ARGS__)
#define get_filter(...) get_butterworth_2_low_pass(__VA_ARGS__)
#elif LOG_PREFILTER_ORDER == 4
typedef Butterworth4LowPass filter_t;
#define init_filter(...) init_butterworth_4_low_pass(__VA_ARGS__)
#define update_filter(...) update_butterworth_4_low_pass(__VA_ARGS__)
#define get_filter(...) get_butterworth_4_low_pass(__VA_ARGS__)
#else
#error "Invalid prefilter order (2, 4 allowed)."
#endif // LOG_PREFILTER_ORDER


#define MODULES_PERIOD (1.0 / MODULES_FREQUENCY)


static struct {
  filter_t phi_est;
  filter_t theta_est;
  filter_t psi_est;
  filter_t acc_x;
  filter_t acc_y;
  filter_t acc_z;
  filter_t cmd_roll;
  filter_t cmd_pitch;
  filter_t cmd_yaw;
  filter_t cmd_thrust;
} filter;


#if PERIODIC_TELEMETRY
static void send_STATE_FILTERED(struct transport_tx *trans, struct link_device *dev) {
  int32_t phi_est = get_filter(&filter.phi_est);
  int32_t theta_est = get_filter(&filter.theta_est);
  int32_t psi_est = get_filter(&filter.psi_est);
  int32_t acc_x = get_filter(&filter.acc_x);
  int32_t acc_y = get_filter(&filter.acc_y);
  int32_t acc_z = get_filter(&filter.acc_z);
  pprz_msg_send_STATE_FILTERED(trans, dev, AC_ID,
      &phi_est, &theta_est, &psi_est,
      &acc_x, &acc_y, &acc_z);
}

static void send_CMD_FILTERED(struct transport_tx *trans, struct link_device *dev) {
  int16_t cmd_roll = (int16_t) get_filter(&filter.cmd_roll);
  int16_t cmd_pitch = (int16_t) get_filter(&filter.cmd_pitch);
  int16_t cmd_yaw = (int16_t) get_filter(&filter.cmd_yaw);
  int16_t cmd_thrust = (int16_t) get_filter(&filter.cmd_thrust);
  pprz_msg_send_CMD_FILTERED(trans, dev, AC_ID,
      &cmd_roll, &cmd_pitch, &cmd_yaw, &cmd_thrust);
}
#endif


void log_prefilter_init(void)
{
  struct Int32Eulers *att = stateGetNedToBodyEulers_i();
  struct Int32Vect3 *accel = stateGetAccelBody_i();
  init_filter(&filter.phi_est, LOG_PREFILTER_TAU, MODULES_PERIOD, att->phi);
  init_filter(&filter.theta_est, LOG_PREFILTER_TAU, MODULES_PERIOD, att->theta);
  init_filter(&filter.psi_est, LOG_PREFILTER_TAU, MODULES_PERIOD, att->psi);
  init_filter(&filter.acc_x, LOG_PREFILTER_TAU, MODULES_PERIOD, accel->x);
  init_filter(&filter.acc_y, LOG_PREFILTER_TAU, MODULES_PERIOD, accel->y);
  init_filter(&filter.acc_z, LOG_PREFILTER_TAU, MODULES_PERIOD, accel->z);
  init_filter(&filter.cmd_roll, LOG_PREFILTER_TAU, MODULES_PERIOD, cmd_log[COMMAND_ROLL]);
  init_filter(&filter.cmd_pitch, LOG_PREFILTER_TAU, MODULES_PERIOD, cmd_log[COMMAND_PITCH]);
  init_filter(&filter.cmd_yaw, LOG_PREFILTER_TAU, MODULES_PERIOD, cmd_log[COMMAND_YAW]);
  init_filter(&filter.cmd_thrust, LOG_PREFILTER_TAU, MODULES_PERIOD, cmd_log[COMMAND_THRUST]);
#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_STATE_FILTERED, send_STATE_FILTERED);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_CMD_FILTERED, send_CMD_FILTERED);
#endif
}

void log_prefilter_periodic(void)
{
  struct Int32Eulers *att = stateGetNedToBodyEulers_i();
  struct Int32Vect3 *accel = stateGetAccelBody_i();
  update_filter(&filter.phi_est, att->phi);
  update_filter(&filter.theta_est, att->theta);
  update_filter(&filter.psi_est, att->psi);
  update_filter(&filter.acc_x, accel->x);
  update_filter(&filter.acc_y, accel->y);
  update_filter(&filter.acc_z, accel->z);
  update_filter(&filter.cmd_roll, cmd_log[COMMAND_ROLL]);
  update_filter(&filter.cmd_pitch, cmd_log[COMMAND_PITCH]);
  update_filter(&filter.cmd_yaw, cmd_log[COMMAND_YAW]);
  update_filter(&filter.cmd_thrust, cmd_log[COMMAND_THRUST]);
}


