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
 * @file "modules/imu_temp_calib/imu_temp_calib.h"
 * @author Tom van Dijk
 * IMU temperature bias calibration module
 */

#ifndef IMU_TEMP_CALIB_H
#define IMU_TEMP_CALIB_H

struct imu_temp_calib_t {
  bool send_telemetry;
  bool update_gyro;
  bool update_accel;
};
struct imu_temp_calib_t imu_temp_calib;

void imu_temp_calib_init();

#endif
