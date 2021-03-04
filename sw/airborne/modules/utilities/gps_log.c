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

/** @file "modules/utilities/gps_log.c"
 * @author Tom van Dijk <tomvand@users.noreply.github.com>
 * Receive and parse GPS datalink messages, and transmit telemetry messages (for onboard logging).
This module does not output any GPS data to ABI, so to the drone it's as if no GPS is received.

Code based on gps_datalink.
 */

#include "modules/utilities/gps_log.h"


#include "generated/flight_plan.h"
#include "subsystems/datalink/telemetry.h"
#include "math/pprz_geodetic_int.h"


static struct LtpDef_i ltp_def;


void gps_log_init(void)
{
  struct LlaCoor_i llh_nav0;
  llh_nav0.lat = NAV_LAT0;
  llh_nav0.lon = NAV_LON0;
  llh_nav0.alt = NAV_ALT0 + NAV_MSL0;
  ltp_def_from_lla_i(&ltp_def, &llh_nav0);
}


static void store_gps_log(struct EnuCoor_f *pos, float heading, uint8_t type) {

}


static void parse_gps_small(int16_t heading, uint32_t pos_xyz, uint32_t __attribute__((unused)) speed_xyz, uint32_t __attribute__((unused)) tow) {
  struct EnuCoor_i enu_pos;
  enu_pos.x = (int32_t)((pos_xyz >> 21) & 0x7FF); // bits 31-21 x position in cm
  if (enu_pos.x & 0x400) {
    enu_pos.x |= 0xFFFFF800;  // sign extend for twos complements
  }
  enu_pos.y = (int32_t)((pos_xyz >> 10) & 0x7FF); // bits 20-10 y position in cm
  if (enu_pos.y & 0x400) {
    enu_pos.y |= 0xFFFFF800;  // sign extend for twos complements
  }
  enu_pos.z = (int32_t)(pos_xyz & 0x3FF); // bits 9-0 z position in cm
  // Send for logging
  float heading_f = ANGLE_FLOAT_OF_BFP(heading);
  struct EnuCoor_f enu_pos_f;
  ENU_FLOAT_OF_BFP(enu_pos_f, enu_pos);
  store_gps_log(&enu_pos_f, heading_f, PPRZ_MSG_ID_REMOTE_GPS_SMALL);
}

void gps_log_parse_REMOTE_GPS_SMALL(uint8_t* buf)
{
  if (DL_REMOTE_GPS_SMALL_ac_id(buf) != AC_ID) { return; } // not for this aircraft
  parse_gps_small(DL_REMOTE_GPS_SMALL_heading(buf),
                  DL_REMOTE_GPS_SMALL_pos_xyz(buf),
                  DL_REMOTE_GPS_SMALL_speed_xyz(buf),
                  DL_REMOTE_GPS_SMALL_tow(buf));
}


static void parse_gps_local(float enu_x, float enu_y, float enu_z,
    float __attribute__((unused)) enu_xd, float __attribute__((unused)) enu_yd, float __attribute__((unused)) enu_zd,
    uint32_t __attribute__((unused)) tow, float course) {
  struct EnuCoor_f enu_pos_f = {enu_x, enu_y, enu_z};
  store_gps_log(&enu_pos_f, course, PPRZ_MSG_ID_REMOTE_GPS_LOCAL);
}

void gps_log_parse_REMOTE_GPS_LOCAL(uint8_t* buf)
{
  if (DL_REMOTE_GPS_LOCAL_ac_id(buf) != AC_ID) { return; } // not for this aircraft
  parse_gps_local(DL_REMOTE_GPS_LOCAL_enu_x(buf),
                  DL_REMOTE_GPS_LOCAL_enu_y(buf),
                  DL_REMOTE_GPS_LOCAL_enu_z(buf),
                  DL_REMOTE_GPS_LOCAL_enu_xd(buf),
                  DL_REMOTE_GPS_LOCAL_enu_yd(buf),
                  DL_REMOTE_GPS_LOCAL_enu_zd(buf),
                  DL_REMOTE_GPS_LOCAL_tow(buf),
                  DL_REMOTE_GPS_LOCAL_course(buf));
}


static void parse_gps(uint8_t __attribute__((unused)) numsv, int32_t __attribute__((unused)) ecef_x, int32_t __attribute__((unused)) ecef_y, int32_t __attribute__((unused)) ecef_z,
    int32_t lat, int32_t lon, int32_t alt, int32_t __attribute__((unused)) hmsl,
    int32_t __attribute__((unused)) ecef_xd, int32_t __attribute__((unused)) ecef_yd, int32_t __attribute__((unused)) ecef_zd,
    uint32_t __attribute__((unused)) tow, int32_t course) {
  struct LlaCoor_i lla_i = {lat, lon, alt};
  struct EnuCoor_i enu_i;
  enu_of_lla_pos_i(&enu_i, &ltp_def, &lla_i);
  struct EnuCoor_f enu_f;
  ENU_FLOAT_OF_BFP(enu_f, enu_i);
  float heading_f = ANGLE_FLOAT_OF_BFP(course);
  store_gps_log(&enu_f, heading_f, PPRZ_MSG_ID_REMOTE_GPS);
}

void gps_log_parse_REMOTE_GPS(uint8_t* buf)
{
  if (DL_REMOTE_GPS_ac_id(buf) != AC_ID) { return; } // not for this aircraft
  parse_gps(DL_REMOTE_GPS_numsv(buf),
            DL_REMOTE_GPS_ecef_x(buf),
            DL_REMOTE_GPS_ecef_y(buf),
            DL_REMOTE_GPS_ecef_z(buf),
            DL_REMOTE_GPS_lat(buf),
            DL_REMOTE_GPS_lon(buf),
            DL_REMOTE_GPS_alt(buf),
            DL_REMOTE_GPS_hmsl(buf),
            DL_REMOTE_GPS_ecef_xd(buf),
            DL_REMOTE_GPS_ecef_yd(buf),
            DL_REMOTE_GPS_ecef_zd(buf),
            DL_REMOTE_GPS_tow(buf),
            DL_REMOTE_GPS_course(buf));
}


