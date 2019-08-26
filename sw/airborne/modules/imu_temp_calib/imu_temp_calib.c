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
 * @file "modules/imu_temp_calib/imu_temp_calib.c"
 * @author Tom van Dijk
 * IMU temperature bias calibration module
 */

#include "modules/imu_temp_calib/imu_temp_calib.h"

#include "subsystems/abi.h"
#include "subsystems/imu.h"

#ifndef IMU_TEMP_CALIB_GYRO_ID
#define IMU_TEMP_CALIB_GYRO_ID ABI_BROADCAST
#endif

#ifndef IMU_TEMP_CALIB_ACCEL_ID
#define IMU_TEMP_CALIB_ACCEL_ID ABI_BROADCAST
#endif


abi_event ev_gyro_temp;
abi_event ev_accel_temp;


static void gyro_temp_cb(uint8_t sender_id, const float temp, struct Imu *imu) {

}

static void accel_temp_cb(uint8_t sender_id, const float temp, struct Imu *imu) {

}


void imu_temp_calib_init() {
  AbiBindMsgIMU_GYRO_TEMPERATURE(IMU_TEMP_CALIB_GYRO_ID, &ev_gyro_temp,
      gyro_temp_cb);
  AbiBindMsgIMU_ACCEL_TEMPERATURE(IMU_TEMP_CALIB_ACCEL_ID, &ev_accel_temp,
      accel_temp_cb);
}


