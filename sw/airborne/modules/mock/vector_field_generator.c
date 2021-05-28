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

/** @file "modules/mock/vector_field_generator.c"
 * @author Tom van Dijk <tomvand@users.noreply.github.com>
 * Generate fake vector fields for visual homing development
 */

#include "modules/mock/vector_field_generator.h"

#include "state.h"
#include "math/pprz_algebra.h"
#include "subsystems/abi.h"

#include <math.h>


static struct EnuCoor_f target;



void vector_field_generator_init(void)
{
  target.x = 0.f;
  target.y = 0.f;
  target.z = 0.f;
}

void vector_field_generator_periodic(void)
{
  // Get true vector to target
  struct EnuCoor_f rel_pos;
  struct EnuCoor_f *pos = stateGetPositionEnu_f();
  VECT3_DIFF(rel_pos, *pos, target);

  // Calculate fake homing vector
  struct EnuCoor_f vec;
  vec.x = -0.2 * rel_pos.x;
  vec.y = 0.2 * rel_pos.x - 0.5 * rel_pos.y;
  vec.z = -0.1 * rel_pos.z;
  float norm = sqrtf(VECT2_NORM2(vec));
  float norm_out = norm < 0.2 ? norm : 0.2;
  VECT3_SMUL(vec, vec, norm_out / norm);

  // Send to ABI
  AbiSendMsgRELATIVE_LOCALIZATION(
      RELATIVE_LOCALIZATION_VISUALHOMING_ID,
      0,
      vec.x, vec.y, vec.z,
      0, 0, 0
      );
}


void vector_field_generator_set_target(struct EnuCoor_f tgt) {
  target = tgt;
}


