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

#include "math/pprz_algebra_float.h"

extern struct FloatVect2 dr_getBodyVel(void);
extern float dr_getHeading(void);

extern void dr_init(void);
extern void dr_periodic(void);

#endif

