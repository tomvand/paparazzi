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
#include "subsystems/ahrs.h"

void pidtuner_periodic(void) {
  float gain1 = radio_control.values[RADIO_GAIN1];
  float gain2 = radio_control.values[RADIO_GAIN2];

//  stabilization_gains.d.z = (gain1 / 9600.0) * 100 * 2;
//  stabilization_gains.d.y = (gain1 / 9600.0) * 70 * 2;

//  stabilization_gains.p.z = (gain2 / 9600.0) * 350 * 2;
//  stabilization_gains.i.y = (gain2 / 9600.0) * 10 * 2;

//  ahrs_fc.gravity_heuristic_factor = (gain2 / 9600.0) * 15 * 2;
}


