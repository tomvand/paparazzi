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

/** @file "modules/utilities/log_prefilter.h"
 * @author Tom van Dijk <tomvand@users.noreply.github.com>
 * Low-pass filter IMU and CMD signals before logging at a lower sampling rate.
 */

#ifndef LOG_PREFILTER_H
#define LOG_PREFILTER_H

extern void log_prefilter_init(void);
extern void log_prefilter_periodic(void);

#endif  // LOG_PREFILTER_H
