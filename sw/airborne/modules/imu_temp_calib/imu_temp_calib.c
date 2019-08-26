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


#ifndef IMU_TEMP_CALIB_GYRO_LUT_TEMP
#define IMU_TEMP_CALIB_GYRO_LUT_TEMP { 0, 0 }
#endif
#ifndef IMU_TEMP_CALIB_GYRO_LUT_P
#define IMU_TEMP_CALIB_GYRO_LUT_P { 0, 0 }
#endif
#ifndef IMU_TEMP_CALIB_GYRO_LUT_Q
#define IMU_TEMP_CALIB_GYRO_LUT_Q { 0, 0 }
#endif
#ifndef IMU_TEMP_CALIB_GYRO_LUT_R
#define IMU_TEMP_CALIB_GYRO_LUT_R { 0, 0 }
#endif


#ifndef IMU_TEMP_CALIB_ACCEL_LUT_TEMP
#define IMU_TEMP_CALIB_ACCEL_LUT_TEMP { 0, 0 }
#endif
#ifndef IMU_TEMP_CALIB_ACCEL_LUT_X
#define IMU_TEMP_CALIB_ACCEL_LUT_X { 0, 0 }
#endif
#ifndef IMU_TEMP_CALIB_ACCEL_LUT_Y
#define IMU_TEMP_CALIB_ACCEL_LUT_Y { 0, 0 }
#endif
#ifndef IMU_TEMP_CALIB_ACCEL_LUT_Z
#define IMU_TEMP_CALIB_ACCEL_LUT_Z { 0, 0 }
#endif


static abi_event ev_gyro_temp;
static abi_event ev_accel_temp;


static int32_t gyro_lut_temp[] = IMU_TEMP_CALIB_GYRO_LUT_TEMP;
static int32_t gyro_lut_p[] =    IMU_TEMP_CALIB_GYRO_LUT_P;
static int32_t gyro_lut_q[] =    IMU_TEMP_CALIB_GYRO_LUT_Q;
static int32_t gyro_lut_r[] =    IMU_TEMP_CALIB_GYRO_LUT_R;
static int gyro_lut_index = 0;

static int32_t accel_lut_temp[] = IMU_TEMP_CALIB_ACCEL_LUT_TEMP;
static int32_t accel_lut_x[] =    IMU_TEMP_CALIB_ACCEL_LUT_X;
static int32_t accel_lut_y[] =    IMU_TEMP_CALIB_ACCEL_LUT_Y;
static int32_t accel_lut_z[] =    IMU_TEMP_CALIB_ACCEL_LUT_Z;
static int accel_lut_index = 0;


/**
 * Fast LUT lookup
 * Performs a quick linear search starting at [previous_index]
 * only changes the LUT index if the neigbouring value is exceeded.
 * Assumes that lut[] is sorted and contains at least two elements.
 * Returns the last index smaller than the query value.
 */
static int lut_lookup(const int32_t *lut, const int lut_size,
    const float query, const int start_index) {
  //
  int i = start_index;
  for (; (query > lut[i + 1]) && (i < lut_size - 2); i++);
  for (; (query < lut[i]) && (i > 0); i--);
  return i;
}

static void gyro_temp_cb(uint8_t sender_id, const float temp, struct Imu *imu) {
  if (imu_temp_calib.update_gyro) {
    gyro_lut_index = lut_lookup(gyro_lut_temp,
        sizeof(gyro_lut_temp) / sizeof(gyro_lut_temp[0]), temp, gyro_lut_index);
    imu->gyro_neutral.p = IMU_GYRO_P_NEUTRAL + gyro_lut_p[gyro_lut_index];
    imu->gyro_neutral.q = IMU_GYRO_Q_NEUTRAL + gyro_lut_q[gyro_lut_index];
    imu->gyro_neutral.r = IMU_GYRO_R_NEUTRAL + gyro_lut_r[gyro_lut_index];
  }
}

static void accel_temp_cb(uint8_t sender_id, const float temp, struct Imu *imu) {
  if (imu_temp_calib.update_accel) {
    accel_lut_index = lut_lookup(accel_lut_temp,
        sizeof(accel_lut_temp) / sizeof(accel_lut_temp[0]), temp, accel_lut_index);
    imu->accel_neutral.x = IMU_ACCEL_X_NEUTRAL + accel_lut_x[accel_lut_index];
    imu->accel_neutral.y = IMU_ACCEL_Y_NEUTRAL + accel_lut_y[accel_lut_index];
    imu->accel_neutral.z = IMU_ACCEL_Z_NEUTRAL + accel_lut_z[accel_lut_index];
  }
}


void imu_temp_calib_init() {
  AbiBindMsgIMU_GYRO_TEMPERATURE(IMU_TEMP_CALIB_GYRO_ID, &ev_gyro_temp,
      gyro_temp_cb);
  AbiBindMsgIMU_ACCEL_TEMPERATURE(IMU_TEMP_CALIB_ACCEL_ID, &ev_accel_temp,
      accel_temp_cb);
}


