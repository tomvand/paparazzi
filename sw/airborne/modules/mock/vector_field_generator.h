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

/** @file "modules/mock/vector_field_generator.h"
 * @author Tom van Dijk <tomvand@users.noreply.github.com>
 * Generate fake vector fields for visual homing development
 */

#ifndef VECTOR_FIELD_GENERATOR_H
#define VECTOR_FIELD_GENERATOR_H

#include "state.h"

extern void vector_field_generator_set_target(struct EnuCoor_f tgt);

extern void vector_field_generator_init(void);
extern void vector_field_generator_periodic(void);

#endif  // VECTOR_FIELD_GENERATOR_H
