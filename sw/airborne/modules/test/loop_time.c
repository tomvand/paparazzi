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

/** @file "modules/test/loop_time.c"
 * @author Tom van Dijk <tomvand@users.noreply.github.com>
 * Measure loop timing, similar to sys_mon but with ap hooks
 */

#include "modules/test/loop_time.h"

#include "mcu_periph/sys_time.h"
#include "subsystems/datalink/telemetry.h"


struct loop_timer {
  uint32_t timer;
  uint32_t total_time;
  uint32_t n;
  uint32_t max_time;
  uint32_t min_time;
};

static struct loop_timer periodic;
static struct loop_timer modules;
static struct loop_timer event;

static void loop_timer_start(struct loop_timer *t) {
  SysTimeTimerStart(t->timer);
}

static void loop_timer_end(struct loop_timer *t) {
  SysTimeTimerStop(t->timer);
  t->total_time += t->timer;
  t->n++;
  if (t->timer < t->min_time) t->min_time = t->timer;
  if (t->timer > t->max_time) t->max_time = t->timer;
}

static void loop_timer_reset(struct loop_timer *t) {
  t->total_time = 0;
  t->n = 0;
  t->max_time = 0;
  t->min_time = ~0;
}


void periodic_start(void) {
  loop_timer_start(&periodic);
}

void periodic_end(void) {
  loop_timer_end(&periodic);
}

void modules_start(void) {
  loop_timer_start(&modules);
}

void modules_end(void) {
  loop_timer_end(&modules);
}

void event_start(void) {
  loop_timer_start(&event);
}

void event_end(void) {
  loop_timer_end(&event);
}




void loop_time_init(void)
{
  loop_timer_reset(&periodic);
  loop_timer_reset(&modules);
  loop_timer_reset(&event);
}

void loop_time_report(void)
{
  // freq = 1.0 Hz
  uint16_t pmin = periodic.min_time;
  uint16_t pavg = periodic.total_time / periodic.n;
  uint16_t pmax = periodic.max_time;
  uint16_t mmin = modules.min_time;
  uint16_t mavg = modules.total_time / modules.n;
  uint16_t mmax = modules.max_time;
  uint16_t emin = event.min_time;
  uint16_t eavg = event.total_time / event.n;
  uint16_t emax = event.max_time;
  uint16_t mfreq = modules.n;
  uint8_t e_per_m = event.n / modules.n;

  DOWNLINK_SEND_LOOP_TIME(DefaultChannel, DefaultDevice,
      &pmin, &pavg, &pmax,
      &mmin, &mavg, &mmax,
      &emin, &eavg, &emax,
      &mfreq, &e_per_m);

  loop_time_init();
}


