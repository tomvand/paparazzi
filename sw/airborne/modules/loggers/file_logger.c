/*
 * Copyright (C) 2014 Freek van Tienen <freek.v.tienen@gmail.com>
 *
 * This file is part of paparazzi.
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/** @file modules/loggers/file_logger.c
 *  @brief File logger for Linux based autopilots
 */

#include "file_logger.h"

#include "generated/modules.h"

#include <stdio.h>
#include "std.h"

#include "subsystems/imu.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "state.h"

/** Set the default File logger path to the USB drive */
#ifndef FILE_LOGGER_PATH
#define FILE_LOGGER_PATH /data/video/usb
#endif

/** Select variables to log */
#ifndef FILE_LOGGER_LOG_FLIGHTPLAN_BLOCK_STAGE
#define FILE_LOGGER_LOG_FLIGHTPLAN_BLOCK_STAGE FALSE
#endif
#ifndef FILE_LOGGER_LOG_LTP_POS
#define FILE_LOGGER_LOG_LTP_POS FALSE
#endif
#ifndef FILE_LOGGER_LOG_LTP_VEL
#define FILE_LOGGER_LOG_LTP_VEL FALSE
#endif
#ifndef FILE_LOGGER_LOG_BODY_VEL
#define FILE_LOGGER_LOG_BODY_VEL FALSE
#endif
#ifndef FILE_LOGGER_LOG_SONAR_BEBOP
#define FILE_LOGGER_LOG_SONAR_BEBOP FALSE
#endif
#ifndef FILE_LOGGER_LOG_PERCEVITE_VELOCITY_ESTIMATE
#define FILE_LOGGER_LOG_PERCEVITE_VELOCITY_ESTIMATE FALSE
#endif
#ifndef FILE_LOGGER_LOG_PERCEVITE_SAFE_DISTANCE
#define FILE_LOGGER_LOG_PERCEVITE_SAFE_DISTANCE FALSE
#endif
#ifndef FILE_LOGGER_LOG_PERCEVITE_WAYPOINTS
#define FILE_LOGGER_LOG_PERCEVITE_WAYPOINTS FALSE
#endif

#if FILE_LOGGER_LOG_SONAR_BEBOP
#include "modules/sonar/sonar_bebop.h"
#endif
#if FILE_LOGGER_LOG_FLIGHTPLAN_BLOCK_STAGE
#include "subsystems/navigation/common_flight_plan.h"
#endif
#if FILE_LOGGER_LOG_PERCEVITE_VELOCITY_ESTIMATE || \
  FILE_LOGGER_LOG_PERCEVITE_SAFE_DISTANCE || \
  FILE_LOGGER_LOG_PERCEVITE_WAYPOINTS
#include "modules/percevite/percevite.h"
#include "subsystems/navigation/waypoints.h"
#endif


/** The file pointer */
static FILE *file_logger = NULL;

/** Start the file logger and open a new file */
void file_logger_start(void)
{
  uint32_t counter = 0;
  char filename[512];

  // Check for available files
  sprintf(filename, "%s/%05d.csv", STRINGIFY(FILE_LOGGER_PATH), counter);
  while ((file_logger = fopen(filename, "r"))) {
    fclose(file_logger);

    counter++;
    sprintf(filename, "%s/%05d.csv", STRINGIFY(FILE_LOGGER_PATH), counter);
  }

  file_logger = fopen(filename, "w");

  if (file_logger != NULL) {
    fprintf(file_logger, "counter,time,");
#if FILE_LOGGER_LOG_FLIGHTPLAN_BLOCK_STAGE
    fprintf(file_logger, "block,stage,");
#endif
#if FILE_LOGGER_LOG_LTP_POS
    fprintf(file_logger, "pos_ltp_x,pos_ltp_y,pos_ltp_z,");
#endif
#if FILE_LOGGER_LOG_LTP_VEL
    fprintf(file_logger, "vel_ltp_x,vel_ltp_y,vel_ltp_z,");
#endif
#if FILE_LOGGER_LOG_BODY_VEL
    fprintf(file_logger, "vel_body_x,vel_body_y,vel_body_z,");
#endif
#if FILE_LOGGER_LOG_SONAR_BEBOP
    fprintf(file_logger, "sonar_bebop,");
#endif
#if FILE_LOGGER_LOG_PERCEVITE_VELOCITY_ESTIMATE || \
  FILE_LOGGER_LOG_PERCEVITE_SAFE_DISTANCE || \
  FILE_LOGGER_LOG_PERCEVITE_WAYPOINTS
    fprintf(file_logger, "percevite_ok,");
#endif
#if FILE_LOGGER_LOG_PERCEVITE_VELOCITY_ESTIMATE
    fprintf(file_logger, "percevite_vx,percevite_vy,percevite_vz,percevite_time_since_vel,");
#endif
#if FILE_LOGGER_LOG_PERCEVITE_SAFE_DISTANCE
    fprintf(file_logger, "percevite_safe_dist,percevite_safe_dist_seq,percevite_raw_dist,percevite_valid_px,");
#endif
#if FILE_LOGGER_LOG_PERCEVITE_WAYPOINTS
    fprintf(file_logger, "percevite_wp_id,percevite_wp_x,percevite_wp_y,percevite_wp_z,percevite_tgt_id,percevite_tgt_x,percevite_tgt_y,percevite_tgt_z,");
#endif
    fprintf(file_logger, "\n");
  }
}

/** Stop the logger an nicely close the file */
void file_logger_stop(void)
{
  if (file_logger != NULL) {
    fclose(file_logger);
    file_logger = NULL;
  }
}

/** Log the values to a csv file */
void file_logger_periodic(void)
{
  if (file_logger == NULL) {
    return;
  }
  static uint32_t counter;
  fprintf(file_logger, "%u,%f,", counter, counter * FILE_LOGGER_PERIODIC_PERIOD);
#if FILE_LOGGER_LOG_FLIGHTPLAN_BLOCK_STAGE
  {
    fprintf(file_logger, "%u,%u,", nav_block, nav_stage);
  }
#endif
#if FILE_LOGGER_LOG_LTP_POS
  {
    struct NedCoor_f *pos_ltp = stateGetPositionNed_f();
    fprintf(file_logger, "%f,%f,%f,", pos_ltp->x, pos_ltp->y, pos_ltp->z);
  }
#endif
#if FILE_LOGGER_LOG_LTP_VEL
  {
    struct NedCoor_f *vel_ltp = stateGetSpeedNed_f();
    fprintf(file_logger, "%f,%f,%f,", vel_ltp->x, vel_ltp->y, vel_ltp->z);
  }
#endif
#if FILE_LOGGER_LOG_BODY_VEL
  {
    struct FloatRMat *R = stateGetNedToBodyRMat_f();
    struct NedCoor_f *vel_ltp = stateGetSpeedNed_f();
    struct FloatVect3 vel_body;
    MAT33_VECT3_MUL(vel_body, *R, *vel_ltp);
    fprintf(file_logger, "%f,%f,%f,", vel_body.x, vel_body.y, vel_body.z);
  }
#endif
#if FILE_LOGGER_LOG_SONAR_BEBOP
  {
    fprintf(file_logger, "%f,", sonar_bebop.distance);
  }
#endif
#if FILE_LOGGER_LOG_PERCEVITE_VELOCITY_ESTIMATE || \
  FILE_LOGGER_LOG_PERCEVITE_SAFE_DISTANCE || \
  FILE_LOGGER_LOG_PERCEVITE_WAYPOINTS
  {
    fprintf(file_logger, "%d,", PerceviteOk());
  }
#endif
#if FILE_LOGGER_LOG_PERCEVITE_VELOCITY_ESTIMATE
  {
    fprintf(file_logger, "%f,%f,%f,%f,",
        percevite_logging.velocity.x,
        percevite_logging.velocity.y,
        percevite_logging.velocity.z,
        percevite.time_since_velocity);
  }
#endif
#if FILE_LOGGER_LOG_PERCEVITE_SAFE_DISTANCE
  {
    fprintf(file_logger, "%f,%u,%f,%f,",
        percevite.safe_region.distance, percevite.safe_region.seq,
        percevite_logging.raw_distance, percevite_logging.valid_pixels);
  }
#endif
#if FILE_LOGGER_LOG_PERCEVITE_WAYPOINTS
  {
    struct NedCoor_f wp = {0.0, 0.0, 0.0};
    if(percevite.wp < nb_waypoint) {
      wp.x = waypoint_get_y(percevite.wp); // Note: convert to NED
      wp.y = waypoint_get_x(percevite.wp);
      wp.z = -waypoint_get_alt(percevite.wp);
    }
    struct NedCoor_f tgt = {0.0, 0.0, 0.0};
    if(percevite_logging.target_wp < nb_waypoint) {
      tgt.x = waypoint_get_y(percevite_logging.target_wp); // Note: convert to NED
      tgt.y = waypoint_get_x(percevite_logging.target_wp);
      tgt.z = -waypoint_get_alt(percevite_logging.target_wp);
    }
    fprintf(file_logger, "%u,%f,%f,%f,%u,%f,%f,%f,",
        percevite.wp, wp.x, wp.y, wp.z,
        percevite_logging.target_wp, tgt.x, tgt.y, tgt.z);
  }
#endif
  fprintf(file_logger, "\n");
  counter++;
}
