/*
 * Copyright (C) Tom van Dijk
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/dead_reckoning/dead_reckoning.h"
 * @author Tom van Dijk
 * Estimate quadrotor position from IMU data.
 */

#ifndef DEAD_RECKONING_H
#define DEAD_RECKONING_H

extern float dr_mu_over_m;

extern void dr_init(void);
extern void dr_periodic(void);

#endif

