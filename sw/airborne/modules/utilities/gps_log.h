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

/** @file "modules/utilities/gps_log.h"
 * @author Tom van Dijk <tomvand@users.noreply.github.com>
 * Receive and parse GPS datalink messages, and transmit telemetry messages (for onboard logging).
This module does not output any GPS data to ABI, so to the drone it's as if no GPS is received.
 */

#ifndef GPS_LOG_H
#define GPS_LOG_H

#include "stdint.h"

extern void gps_log_init(void);
extern void gps_log_parse_REMOTE_GPS_SMALL(uint8_t* buf);	// REMOTE_GPS_SMALL
extern void gps_log_parse_REMOTE_GPS_LOCAL(uint8_t* buf);	// REMOTE_GPS_LOCAL
extern void gps_log_parse_REMOTE_GPS(uint8_t* buf);	// REMOTE_GPS

#endif  // GPS_LOG_H
