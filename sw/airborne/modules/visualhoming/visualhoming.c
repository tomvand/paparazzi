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

/** @file "modules/visualhoming/visualhoming.c"
 * @author Tom van Dijk <tomvand@users.noreply.github.com>
 * Visual homing
 */


#include "modules/visualhoming/visualhoming.h"

#include "subsystems/datalink/downlink.h"


void visualhoming_init(void)
{
  // your init code here
}

void visualhoming_periodic(void)
{
  char msg[] = "Hello visual homing!";
  DOWNLINK_SEND_VISUALHOMING(DefaultChannel, DefaultDevice,
      sizeof(msg) - 1, msg);
}

void visualhoming_event(void)
{
  // your event code here
}


