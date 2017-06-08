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

#ifndef VISUALHOMING_STATEMACHINE_H
#define VISUALHOMING_STATEMACHINE_H

enum vh_state_t {
	VH_STATE_STANDBY,
	VH_STATE_STORE_SNAPSHOT,
	VH_STATE_RECORD_ODOMETRY,
	VH_STATE_FOLLOW_ODOMETRY,
	VH_STATE_HOME_TO_SNAPSHOT,
	VH_STATE_AUTO,
};

extern enum vh_state_t vh_state_cmd;

void vh_statemachine_run(void);

#endif
