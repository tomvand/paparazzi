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
 * @file "modules/percevite/percevite.c"
 * @author Tom van Dijk
 * Obstacle detection and avoidance module for PercEvite
 */

#include "modules/percevite/percevite.h"
#include "modules/percevite/percevite_messages.h"

#include "pprzlink/pprz_transport.h"
#include "mcu_periph/udp.h"
#include "subsystems/datalink/downlink.h"

#include "firmwares/rotorcraft/guidance/guidance_h_ref.h"

#include "generated/modules.h"

#include <stdio.h>

// Reference deceleration rate in m/s2
#ifndef PERCEVITE_DECELERATION
#define PERCEVITE_DECELERATION 0.5
#endif

// Minimum distance to keep towards obstacle [m]
#ifndef PERCEVITE_MINIMUM_DISTANCE
#define PERCEVITE_MINIMUM_DISTANCE 3.0
#endif

// Assumed delay between image capture and receival of safe distance measurement [s]
#ifndef PERCEVITE_IMAGE_LAG
#define PERCEVITE_IMAGE_LAG 1.0
#endif

// Max allowed time between images [s]
#ifndef PERCEVITE_IMAGE_TIMEOUT
#define PERCEVITE_IMAGE_TIMEOUT 0.5
#endif

// Fraction of pixels for which a depth measurement must be available
#ifndef PERCEVITE_VALID_PIXELS_THRESHOLD
#define PERCEVITE_VALID_PIXELS_THRESHOLD 0.50
#endif

// Enable writing to guidance h max velocity
#ifndef PERCEVITE_SET_NAV_VMAX
#define PERCEVITE_SET_NAV_VMAX TRUE
#endif

struct percevite_t percevite = {
    .max_velocity = 0.0,
    .time_since_image = 0.0,
};

struct percevite_settings_t percevite_settings = {
    .deceleration = PERCEVITE_DECELERATION,
    .minimum_distance = PERCEVITE_MINIMUM_DISTANCE,
    .image_lag = PERCEVITE_IMAGE_LAG,
    .pixels_threshold = PERCEVITE_VALID_PIXELS_THRESHOLD,
};

static void slamdunk_init(void);
static void slamdunk_event(void);
static void slamdunk_parse_message(void);
static void slamdunk_send_message(union paparazzi_to_slamdunk_msg_t *msg);

void percevite_init(void) {
  slamdunk_init();
}

void percevite_periodic(void) {
  // Increase image timer
  percevite.time_since_image += PERCEVITE_PERIODIC_PERIOD;
  if(percevite.time_since_image > PERCEVITE_IMAGE_TIMEOUT) {
    printf("[percevite] WARNING: Image timeout exceeded!\n");
    percevite.max_velocity = 0.0;
    // Output to guidance H
#if PERCEVITE_SET_NAV_VMAX
    gh_set_max_speed(percevite.max_velocity);
#endif
  }
  // Send dummy message to slamdunk
  union paparazzi_to_slamdunk_msg_t msg = {
      .text = "Hello SLAMDunk!"
  };
  slamdunk_send_message(&msg);
}

void percevite_event(void) {
  slamdunk_event();
}

static void percevite_on_message(union slamdunk_to_paparazzi_msg_t *msg) {
  // Calculate max velocity
  float braking_distance = (0.10 * msg->safe_distance) - percevite_settings.minimum_distance;
  if(braking_distance > 0.0
      && msg->valid_pixels > 255 * percevite_settings.pixels_threshold) {
//    percevite.max_velocity = sqrtf(2.0 * braking_distance * percevite_settings.deceleration);
    percevite.max_velocity = sqrtf(2.0 * braking_distance * percevite_settings.deceleration
        + (percevite_settings.deceleration * percevite_settings.deceleration) * (percevite_settings.image_lag * percevite_settings.image_lag))
        - percevite_settings.deceleration * percevite_settings.image_lag;
    if(percevite.max_velocity > GUIDANCE_H_REF_MAX_SPEED) {
      percevite.max_velocity = GUIDANCE_H_REF_MAX_SPEED;
    }
  } else {
    percevite.max_velocity = 0.0;
  }
  // Print max velocity
  printf("[percevite] Max velocity = %.1fm/s\t(safe distance: %.1fm, valid pixels: %.0f%%)\n",
      percevite.max_velocity,
      0.10 * msg->safe_distance,
      msg->valid_pixels / 2.55);
  // Output to guidance H
#if PERCEVITE_SET_NAV_VMAX
  gh_set_max_speed(percevite.max_velocity);
#endif
  // Reset timeout
  percevite.time_since_image = 0.0;
}


/*
 * Communication with SLAMDunk
 */

struct slamdunk_t {
  struct link_device *device;      ///< The device which is uses for communication
  struct pprz_transport transport; ///< The transport layer (PPRZ)
  uint8_t msg_buf[128];            ///< Message buffer
  bool msg_available;              ///< If we received a message
};
static struct slamdunk_t slamdunk = {
    .device = (&((PERCEVITE_UDP).device)),
    .msg_available = false
};

static void slamdunk_init(void) {
  printf("[percevite] Initialize pprzlink (UDP) to SLAMDunk... ");
  pprz_transport_init(&slamdunk.transport);
  printf("ok\n");
}

static void slamdunk_event(void) {
  pprz_check_and_parse(slamdunk.device, &slamdunk.transport, slamdunk.msg_buf, &slamdunk.msg_available);
  if(slamdunk.msg_available) {
    slamdunk_parse_message();
    slamdunk.msg_available = FALSE;
  }
}

static void slamdunk_parse_message(void) {
  union slamdunk_to_paparazzi_msg_t *msg;
  switch(slamdunk.msg_buf[3]) { // For pprzlink v2.0
    case PPRZ_MSG_ID_PAYLOAD:
      msg = (union slamdunk_to_paparazzi_msg_t *)&slamdunk.msg_buf[5];
      percevite_on_message(msg);
      break;
    default:
      break;
  }
}

static void slamdunk_send_message(union paparazzi_to_slamdunk_msg_t *msg) {
  pprz_msg_send_PAYLOAD(&(slamdunk.transport.trans_tx), slamdunk.device,
      AC_ID, sizeof(*msg), &(msg->bytes));
}

