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

#include <stdio.h>

struct slamdunk_t {
  struct link_device *device;           ///< The device which is uses for communication
  struct pprz_transport transport;      ///< The transport layer (PPRZ)
  uint8_t msg_buf[128]; ///< Message buffer
  bool msg_available;                 ///< If we received a message
};
static struct slamdunk_t slamdunk = {
    .device = (&((PERCEVITE_UDP).device)),
    .msg_available = false
};

static void parse_slamdunk_message(void){
  union slamdunk_to_paparazzi_msg_t *msg;
  printf("Message received!\n");
  switch(slamdunk.msg_buf[3]) { // For pprzlink v2.0
    case PPRZ_MSG_ID_PAYLOAD:
      printf("PAYLOAD received!\n");
      msg = (union slamdunk_to_paparazzi_msg_t *)&slamdunk.msg_buf[5];
      printf("%s\n", msg->text);
      break;
    default:
      break;
  }
}

void percevite_init(void) {
  printf("Initialize pprzlink (UDP) to SLAMDunk... ");
  pprz_transport_init(&slamdunk.transport);
  printf("ok\n");
}

void percevite_periodic(void) {
  union paparazzi_to_slamdunk_msg_t msg = {
      .text = "Hello SLAMDunk!"
  };
  pprz_msg_send_PAYLOAD(&(slamdunk.transport.trans_tx), slamdunk.device,
      AC_ID, sizeof(msg), &msg.bytes);
}

void percevite_event(void) {
  pprz_check_and_parse(slamdunk.device, &slamdunk.transport, slamdunk.msg_buf, &slamdunk.msg_available);
  if(slamdunk.msg_available) {
    parse_slamdunk_message();
    slamdunk.msg_available = FALSE;
  }
}


