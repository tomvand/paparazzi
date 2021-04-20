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
 * @file "modules/pidtuner/pidtuner.c"
 * @author Tom van Dijk
 * tune pid gains via rc
 */

#include "modules/pidtuner/pidtuner.h"

#include "subsystems/radio_control.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_quat_int.h"
#include "firmwares/rotorcraft/guidance/guidance_v.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "subsystems/ahrs.h"
#include "mcu_periph/sys_time.h"

#include "generated/modules.h"
#include "generated/airframe.h"

#include <stdbool.h>


// -----------------------------------------------------------------------------
#ifndef CHIRP_START_AMPLITUDE
#define CHIRP_START_AMPLITUDE 50
#endif

static float chirp_start_time = 0.0;

static void __attribute__((unused)) chirp_trigger(void) {
  static bool trigger_prev = false;
    bool trigger = radio_control.values[RADIO_AUX1] > (9600 / 2);
    if (trigger && !trigger_prev) {
      sys_id_chirp_activate_handler(1);
      chirp_start_time = get_sys_time_float();
    } else if (!trigger && trigger_prev) {
      sys_id_chirp_activate_handler(0);
      chirp_start_time = 0.0;
    }
    trigger_prev = trigger;
}

static void __attribute__((unused)) chirp_init(void) {
  static bool chirp_initialized = false;
  if (chirp_initialized) return;

  if (chirp_amplitude == 0) {
    // Initialize chirp values
    chirp_amplitude = CHIRP_START_AMPLITUDE;
    chirp_noise_stdv_onaxis_ratio = 0.0;
    chirp_noise_stdv_offaxis = 0;
    sys_id_chirp_fstop_handler(75.0);
    sys_id_chirp_fstart_handler(5.0);
    chirp_length_s = 100.0;
    chirp_initialized = true;
  }
}

static void __attribute__((unused)) chirp_periodic(void) {
  if (chirp_active && chirp_start_time != 0.0) {
    float chirp_frequency = chirp_fstart_hz +
        (chirp_fstop_hz - chirp_fstart_hz) * (get_sys_time_float() - chirp_start_time) / chirp_length_s;
    chirp_amplitude = CHIRP_START_AMPLITUDE * chirp_frequency / chirp_fstart_hz;
  } else {
    chirp_amplitude = CHIRP_START_AMPLITUDE;
  }
}


// -----------------------------------------------------------------------------
static void __attribute__((unused)) pid_periodic(void) {
  // Gain 1 (left):   D action (PD zero)
  // Gain 2 (right):  Control gain
  float __attribute__((unused)) gain1 = radio_control.values[RADIO_GAIN1] / 9600.0 * 2;
  float __attribute__((unused)) gain2 = radio_control.values[RADIO_GAIN2] / 9600.0 * 2;

//  stabilization_gains.p.x = 180.0 *   1.0 * gain2;
//  stabilization_gains.p.y = 180.0 *   1.0 * gain2;
//  stabilization_gains.p.z = 635.0 *   1.0 * gain2;
//  stabilization_gains.i.x =  18.0 *   1.0 * gain2;
//  stabilization_gains.i.y =  18.0 *   1.0 * gain2;
//  stabilization_gains.i.z =   6.0 *   1.0 * gain2;
//  stabilization_gains.d.x = 104.0 * gain1 * gain2;
//  stabilization_gains.d.y = 104.0 * gain1 * gain2;
//  stabilization_gains.d.z = 587.0 * gain1 * gain2;

  att_ref_quat_i.model.omega.p = 800.0 * gain2;
  att_ref_quat_i.model.omega.q = 800.0 * gain2;
}


// -----------------------------------------------------------------------------
static void __attribute__((unused)) pidgain_set(void) {
  // Roll
  stabilization_gains.p.x = 280;
  stabilization_gains.i.x = 420;
  stabilization_gains.d.x = 0;
  stabilization_gains.dd.x = 2;
  // Pitch
  if (radio_control.values[RADIO_GAIN2] > 9600 / 3) {
    stabilization_gains.p.y = 280;
    stabilization_gains.i.y = 420;
    stabilization_gains.d.y = 0;
    stabilization_gains.dd.y = 3;
  }
  // Yaw
  if (radio_control.values[RADIO_GAIN2] > 9600 / 3 * 2) {
    stabilization_gains.p.z = 0;
    stabilization_gains.i.z = 266;
    stabilization_gains.d.z = 0;
    stabilization_gains.dd.z = 0;
  }
}

static void __attribute__((unused)) pidgain_restore(void) {
  // Roll
  stabilization_gains.p.x = STABILIZATION_ATTITUDE_PHI_PGAIN;
  stabilization_gains.i.x = STABILIZATION_ATTITUDE_PHI_IGAIN;
  stabilization_gains.d.x = STABILIZATION_ATTITUDE_PHI_DGAIN;
  stabilization_gains.dd.x = STABILIZATION_ATTITUDE_PHI_DDGAIN;
  // Pitch
  stabilization_gains.p.y = STABILIZATION_ATTITUDE_THETA_PGAIN;
  stabilization_gains.i.y = STABILIZATION_ATTITUDE_THETA_IGAIN;
  stabilization_gains.d.y = STABILIZATION_ATTITUDE_THETA_DGAIN;
  stabilization_gains.dd.y = STABILIZATION_ATTITUDE_THETA_DDGAIN;
  // Yaw
  stabilization_gains.p.z = STABILIZATION_ATTITUDE_PSI_PGAIN;
  stabilization_gains.i.z = STABILIZATION_ATTITUDE_PSI_IGAIN;
  stabilization_gains.d.z = STABILIZATION_ATTITUDE_PSI_DGAIN;
  stabilization_gains.dd.z = STABILIZATION_ATTITUDE_PSI_DDGAIN;
}

static void __attribute__((unused)) pidgain_trigger(void) {
  static bool trigger_prev = false;
  bool trigger = radio_control.values[RADIO_AUX1] > (9600 / 2);
  if (trigger && !trigger_prev) {
    pidgain_set();
  } else if (!trigger && trigger_prev) {
    pidgain_restore();
  }
  trigger_prev = trigger;
}


// -----------------------------------------------------------------------------
void pidtuner_periodic(void) {
//  pid_periodic();

  chirp_init();
  chirp_trigger();
  chirp_periodic();
}


