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

/** @file "modules/test/loop_time.h"
 * @author Tom van Dijk <tomvand@users.noreply.github.com>
 * Measure loop timing, similar to sys_mon but with ap hooks
 */

#ifndef LOOP_TIME_H
#define LOOP_TIME_H

extern void periodic_start(void);
extern void periodic_end(void);
extern void modules_start(void);
extern void modules_end(void);
extern void event_start(void);
extern void event_end(void);

extern void loop_time_init(void);
extern void loop_time_report(void);

#endif  // LOOP_TIME_H
