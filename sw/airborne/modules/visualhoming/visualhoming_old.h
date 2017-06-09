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
 * @file "modules/visualhoming/visualhoming.h"
 * @author Tom van Dijk
 * Fourier-based local visual homing.
 */

#ifndef VISUALHOMING_H
#define VISUALHOMING_H

extern int take_snapshot;
extern int drop_snapshot;

extern float environment_radius;
extern int use_frame_to_frame_velocity;

extern void visualhoming_init(void);
extern void visualhoming_periodic(void);
extern void visualhoming_start(void);
extern void visualhoming_stop(void);

#endif

