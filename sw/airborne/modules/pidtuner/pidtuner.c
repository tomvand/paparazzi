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

#include "generated/modules.h"

#include <stdbool.h>


static void chirp_trigger(void) {
  static bool trigger_prev = false;
    bool trigger = radio_control.values[RADIO_AUX1] > (9600 / 2);
    if (trigger && !trigger_prev) {
      sys_id_chirp_activate_handler(1);
    } else if (!trigger && trigger_prev) {
      sys_id_chirp_activate_handler(0);
    }
    trigger_prev = trigger;
}

static void chirp_init(void) {
  static bool chirp_initialized = false;
  if (chirp_initialized) return;

  if (chirp_amplitude == 0) {
    // Initialize chirp values
    chirp_amplitude = 200;
    chirp_noise_stdv_onaxis_ratio = 0.1;
    chirp_noise_stdv_offaxis = 200;
    sys_id_chirp_fstop_handler(20.0);
    sys_id_chirp_fstart_handler(1.0);
    chirp_length_s = 20.0;
    chirp_initialized = true;
  }
}


static void __attribute__((unused)) pid_periodic(void) {
  //  float gain1 = radio_control.values[RADIO_GAIN1];
  //  float gain2 = radio_control.values[RADIO_GAIN2];
  //
  //  // Gain Kp
  //  guidance_h.gains.p = (gain1 / 9600.0) * 2 * 650;
  //
  //  // Gain Kd
  //  guidance_h.gains.d = (gain2 / 9600.0) * 2 * 350;
}


void pidtuner_periodic(void) {
  chirp_init();
  chirp_trigger();
}


