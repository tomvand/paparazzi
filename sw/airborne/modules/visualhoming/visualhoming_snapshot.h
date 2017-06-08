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

#ifndef VISUALHOMING_SNAPSHOT_H
#define VISUALHOMING_SNAPSHOT_H

#include "visualhoming_video.h"

// Configuration
#ifndef VISUALHOMING_SNAPSHOT_K
#define VISUALHOMING_SNAPSHOT_K 10
#endif

struct snapshot_t {
	int8_t ak[VISUALHOMING_SNAPSHOT_K];
	int8_t bk[VISUALHOMING_SNAPSHOT_K];
};

struct homingvector_t {
	float x;
	float y;
	float sigma;
};

void vh_snapshot_init(void);
void vh_snapshot_close(void);

void vh_snapshot_from_horizon(struct snapshot_t *ss, const horizon_t hor);
void vh_snapshot_to_horizon(const struct snapshot_t *ss, horizon_t hor);
void vh_snapshot_copy(struct snapshot_t *dst, const struct snapshot_t *src);

struct homingvector_t vh_snapshot_homingvector(
		const struct snapshot_t * current,
		const struct snapshot_t * target,
		struct snapshot_t **c_warped,
		struct snapshot_t **t_rotated);

#endif
