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
 * @file "modules/visualhoming/visualhoming.h"
 * @author Tom van Dijk
 * This file contains all platform-independent code for local visual homing.
 */

#ifndef VISUALHOMING_CORE_H
#define VISUALHOMING_CORE_H

#include <stdint.h>

// Configuration
#ifndef HORIZON_WIDTH
#define HORIZON_WIDTH 128 /**< Number of pixels in horizon image */
#endif

#ifndef SNAPSHOT_K
#define SNAPSHOT_K 10 /**< Number of complex coefficients to store in snapshot */
#endif

#ifndef SNAPSHOT_N_IT
#define SNAPSHOT_N_IT 1 /**< Number of iterations for homing vector estimation */
#endif

struct snapshot_t;

struct homingvector_t {
	float x;
	float y;
	float sigma;
};

void visualhoming_core_init(void);
void visualhoming_core_close(void);

struct snapshot_t *snapshot_create(const float hor[]);
void snapshot_free(struct snapshot_t * ss);
void snapshot_copy(struct snapshot_t * dest, const struct snapshot_t * src);
struct homingvector_t homing_vector(
		const struct snapshot_t * current,
		const struct snapshot_t * target,
		struct snapshot_t **c_warped,
		struct snapshot_t **t_rotated);

// Test function
void snapshot_to_horizon(float hor[], const struct snapshot_t * ss);

#endif
