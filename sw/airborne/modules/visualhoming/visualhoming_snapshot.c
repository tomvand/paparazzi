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

#include "visualhoming_snapshot.h"

#include "ext/pffft/pffft.h"

#include "math/pprz_algebra_float.h"

#include <stdio.h>

#define COEFF_SCALE 0.25 // Max amplitude of coefficients (assuming [0..1] pixels)

// Configuration
#ifndef VISUALHOMING_SNAPSHOT_N_IT
#define VISUALHOMING_SNAPSHOT_N_IT 1 /**< Number of iterations for homing vector estimation */
#endif

// Macros for 1-based indexing of Fourier coefficients
#define SS_AK(ss, k) ( (ss)->ak[(k)-1] )
#define SS_BK(ss, k) ( (ss)->bk[(k)-1] )

// Macros for matrix operations
/* _mat1 += _mat2 */
#define MAT33_ADD(_mat1,_mat2) {     \
    MAT33_ELMT((_mat1),0,0) += MAT33_ELMT((_mat2),0,0);  \
    MAT33_ELMT((_mat1),0,1) += MAT33_ELMT((_mat2),0,1);  \
    MAT33_ELMT((_mat1),0,2) += MAT33_ELMT((_mat2),0,2);  \
    MAT33_ELMT((_mat1),1,0) += MAT33_ELMT((_mat2),1,0);  \
    MAT33_ELMT((_mat1),1,1) += MAT33_ELMT((_mat2),1,1);  \
    MAT33_ELMT((_mat1),1,2) += MAT33_ELMT((_mat2),1,2);  \
    MAT33_ELMT((_mat1),2,0) += MAT33_ELMT((_mat2),2,0);  \
    MAT33_ELMT((_mat1),2,1) += MAT33_ELMT((_mat2),2,1);  \
    MAT33_ELMT((_mat1),2,2) += MAT33_ELMT((_mat2),2,2);  \
  } /**< This macro seems to be missing in Paparazzi's math headers? */

#define MAT33_DET(_det, _m) {            \
    const float m00 = MAT33_ELMT((_m),1,1)*MAT33_ELMT((_m),2,2) - MAT33_ELMT((_m),1,2)*MAT33_ELMT((_m),2,1);    \
    const float m10 = MAT33_ELMT((_m),0,1)*MAT33_ELMT((_m),2,2) - MAT33_ELMT((_m),0,2)*MAT33_ELMT((_m),2,1);    \
    const float m20 = MAT33_ELMT((_m),0,1)*MAT33_ELMT((_m),1,2) - MAT33_ELMT((_m),0,2)*MAT33_ELMT((_m),1,1);    \
    _det = MAT33_ELMT((_m),0,0)*m00 - MAT33_ELMT((_m),1,0)*m10 + MAT33_ELMT((_m),2,0)*m20; \
  }

// Internal data types
struct spatial_gradient_t {
	float akx;
	float aky;
	float bkx;
	float bky;
};

// Static variables
static PFFFT_Setup *pffft_setup;

// Static function declarations
static float relative_orientation(
		const struct snapshot_t * current,
		const struct snapshot_t * target);
static void snapshot_rotate(
		const struct snapshot_t* ss,
		struct snapshot_t * ss_rotated,
		float angle);
static struct FloatVect3 relative_pose(
		const struct snapshot_t * current,
		const struct snapshot_t * target);
static void snapshot_warp(struct snapshot_t * ss, struct FloatVect3 pose);
static void spatial_gradient(
		struct spatial_gradient_t * grad,
		const struct snapshot_t * ss,
		int k);
static void get_Ak(
		struct FloatMat33 *ak,
		float akhs0,
		float bkhs0,
		struct spatial_gradient_t g,
		int k);
static void get_zk(
		struct FloatVect3 *zk,
		float ak,
		float bk,
		float akhs0,
		float bkhs0,
		struct spatial_gradient_t g,
		int k);

// External functions
void vh_snapshot_init(void) {
	pffft_setup = pffft_new_setup(VISUALHOMING_HORIZON_RESOLUTION, PFFFT_REAL);
	printf("Snapshot size: %zu bytes.\n", sizeof(struct snapshot_t));
}

void vh_snapshot_close(void) {
	pffft_destroy_setup(pffft_setup);
}

void vh_snapshot_from_horizon(struct snapshot_t *ss, const horizon_t hor) {
	float fourier_coeffs[VISUALHOMING_HORIZON_RESOLUTION];
	// Get (real) Fourier transform of horizon image
	pffft_transform_ordered(pffft_setup, hor, fourier_coeffs, NULL,
			PFFFT_FORWARD);
	// Store first SNAPSHOT_K components
	for (int k = 1; k <= VISUALHOMING_SNAPSHOT_K; k++) {
		float ak = fourier_coeffs[2 * k] / VISUALHOMING_HORIZON_RESOLUTION
				/ COEFF_SCALE;
		if (ak > INT8_MAX) {
			printf("SNAPSHOT OVERFLOW: value = %+.0f, max = %d!\n", ak,
			INT8_MAX);
			ak = INT8_MAX;
		}
		if (ak < INT8_MIN) {
			printf("SNAPSHOT UNDERFLOW: value = %+.0f, min = %d!\n", ak,
			INT8_MIN);
			ak = INT8_MIN;
		}
		float bk = fourier_coeffs[2 * k + 1] / VISUALHOMING_HORIZON_RESOLUTION
				/ COEFF_SCALE;
		if (bk > INT8_MAX) {
			printf("SNAPSHOT OVERFLOW: value = %+.0f, max = %d!\n", bk,
			INT8_MAX);
			bk = INT8_MAX;
		}
		if (bk < INT8_MIN) {
			printf("SNAPSHOT UNDERFLOW: value = %+.0f, min = %d!\n", bk,
			INT8_MIN);
			bk = INT8_MIN;
		}
		SS_AK(ss, k) = ak;
		SS_BK(ss, k) = -bk;
//		printf("K=%2d: %+3d %+3d\n", k, SS_AK(ss, k), SS_BK(ss, k));
	}
}

void vh_snapshot_to_horizon(const struct snapshot_t *ss, horizon_t hor) {
	float fourier_coeffs[VISUALHOMING_HORIZON_RESOLUTION];
	// Read Fourier coefficients from snapshot
	memset(fourier_coeffs, 0, sizeof(fourier_coeffs));
	fourier_coeffs[0] = 128;
	fourier_coeffs[1] = 0;
	for (int k = 1; k <= VISUALHOMING_SNAPSHOT_K; k++) {
		fourier_coeffs[2 * k] = SS_AK(ss, k) * COEFF_SCALE;
		fourier_coeffs[2 * k + 1] = -SS_BK(ss, k) * COEFF_SCALE;
	}
	// Perform inverse Fourier transform
	pffft_transform_ordered(pffft_setup, fourier_coeffs, hor, NULL,
			PFFFT_BACKWARD);
	// Fix under-/overflows
	for (int x = 0; x < VISUALHOMING_HORIZON_RESOLUTION; x++) {
		if (hor[x] < 0) hor[x] = 0;
		if (hor[x] > 255) hor[x] = 255;
	}
}

void vh_snapshot_copy(struct snapshot_t *dst, const struct snapshot_t *src) {
	memcpy(dst, src, sizeof(struct snapshot_t));
}

struct homingvector_t vh_snapshot_homingvector(
		const struct snapshot_t * current,
		const struct snapshot_t * target,
		struct snapshot_t **c_warped,
		struct snapshot_t **t_rotated) {
	// Temporary snapshot buffers
	static struct snapshot_t target_rotated;
	static struct snapshot_t current_warped;

	struct homingvector_t vec = {
			.x = 0,
			.y = 0,
			.sigma = 0 };

	vh_snapshot_copy(&current_warped, current);

	for (int i = 0; i < VISUALHOMING_SNAPSHOT_N_IT; i++) {
		float sigma;
		struct FloatVect3 pose;

		// Estimate and relative orientation and rotate target snapshot.
		sigma = relative_orientation(&current_warped, target);
		snapshot_rotate(target, &target_rotated, -sigma);
		// Estimate position
		pose = relative_pose(&current_warped, &target_rotated);
//		printf("Delta_sigma = %.0f\n", pose.z / M_PI * 180);
		// Update homing vector and warp current snapshot accordingly
		vec.x += pose.x;
		vec.y += pose.y;
		vec.sigma = sigma;
		snapshot_warp(&current_warped, pose);
	}

	// Output temporary snapshots if requested
	if (c_warped != NULL) {
		*c_warped = &current_warped;
	}
	if (t_rotated != NULL) {
		*t_rotated = &target_rotated;
	}

	return vec;
}

// Static functions
static float relative_orientation(
		const struct snapshot_t * current,
		const struct snapshot_t * target) {
	// See "Stürzl, W., & Mallot, H. A. (2006). Efficient visual homing based on
	// Fourier transformed panoramic images." section 2.4.
	float sigma = 0.0;
	float w = 0.0;
	for (int k = 1; k <= VISUALHOMING_SNAPSHOT_K; k++) {
		float ac, bc, at, bt;
		float magn_c, phase_c;
		float magn_t, phase_t;
		int nk;
		float sigma_k;
		float w_k;

		ac = SS_AK(current, k);
		bc = SS_BK(current, k);
		at = SS_AK(target, k);
		bt = SS_BK(target, k);

		magn_c = sqrtf(ac * ac + bc * bc);
		phase_c = atan2f(bc, ac);
		magn_t = sqrtf(at * at + bt * bt);
		phase_t = atan2f(bt, at);

		nk = roundf((phase_t - phase_c + k * sigma) / (2 * M_PI));
		sigma_k = (phase_c - phase_t + 2 * M_PI * nk) / k;
		w_k = magn_c * magn_t * k * k;

		sigma = (w * sigma + w_k * sigma_k) / (w + w_k);
		w = w + w_k;
	}

	return sigma;
}

static void snapshot_rotate(
		const struct snapshot_t* ss,
		struct snapshot_t * ss_rotated,
		float angle) {
	for (int k = 1; k <= VISUALHOMING_SNAPSHOT_K; k++) {
		SS_AK(ss_rotated, k) = SS_AK(ss, k) * cos(k * angle)
				+ SS_BK(ss, k) * sin(k * angle);
		SS_BK(ss_rotated, k) = SS_BK(ss, k) * cos(k * angle)
				- SS_AK(ss, k) * sin(k * angle);
	}
}

static struct FloatVect3 relative_pose(
		const struct snapshot_t * c,
		const struct snapshot_t * t) {
	// See "Stürzl, W., & Mallot, H. A. (2006). Efficient visual homing based on
	// Fourier transformed panoramic images." section 2.3.
	struct FloatMat33 A;
	struct FloatVect3 z;
	struct FloatVect3 out;

	FLOAT_MAT33_ZERO(A);
	FLOAT_VECT3_ZERO(z);

	for (int k = 1; k <= VISUALHOMING_SNAPSHOT_K; k++) {
		struct spatial_gradient_t gk;
		struct FloatMat33 Ak;
		struct FloatVect3 zk;

		memset(&gk, 0, sizeof(gk));
		FLOAT_MAT33_ZERO(Ak);
		FLOAT_VECT3_ZERO(zk);

		spatial_gradient(&gk, c, k);

		get_Ak(&Ak, SS_AK(t, k), SS_BK(t, k), gk, k);
		get_zk(&zk, SS_AK(c, k), SS_BK(c, k), SS_AK(t, k), SS_BK(t, k), gk, k);

		MAT33_ADD(A, Ak);
		VECT3_ADD(z, zk);
	}

	// XXX Debug
//	printf("z = [% 7.2f, % 7.2f, % 7.2f]\n", z.x, z.y, z.z);
//	float detA;
//	float detAinv;
//	MAT33_DET(detA, A);
//	printf("A = \n");
//	for (int r = 0; r < 3; r++) {
//		printf("[%10.2f%10.2f%10.2f]\n", MAT33_ELMT(A, r, 0),
//				MAT33_ELMT(A, r, 1), MAT33_ELMT(A, r, 2));
//	}
//	printf("det(A) = %.2e\n", detA);

	MAT33_INV(A, A);
	// XXX Debug
//	MAT33_DET(detAinv, A);
//	printf("det(Ainv) = %.2e\n", detAinv);
//	printf("1 = %.2f\n", detA * detAinv);

	VECT3_SMUL(z, z, -1.0);
//	printf("z = [% 7.2f, % 7.2f, % 7.2f]\n", z.x, z.y, z.z);
	MAT33_VECT3_MUL(out, A, z);

	return out;
}

static void snapshot_warp(struct snapshot_t * ss, struct FloatVect3 pose) {
	struct snapshot_t ss_old;
	struct spatial_gradient_t g;

	vh_snapshot_copy(&ss_old, ss);

	for (int k = 1; k <= VISUALHOMING_SNAPSHOT_K; k++) {
		spatial_gradient(&g, &ss_old, k);
		SS_AK(ss, k) += g.akx * pose.x + g.aky * pose.y;
		SS_BK(ss, k) += g.bkx * pose.x + g.bky * pose.y;
	}
}

static void spatial_gradient(
		struct spatial_gradient_t * g,
		const struct snapshot_t * ss,
		int k) {
	if (k == 1) {
		g->akx = 0.5 * ((k + 1) * SS_AK(ss, k + 1));
		g->aky = 0.5 * ((k + 1) * SS_BK(ss, k + 1));
		g->bkx = 0.5 * ((k + 1) * SS_BK(ss, k + 1));
		g->bky = 0.5 * (-(k + 1) * SS_AK(ss, k + 1));
	} else if (k == VISUALHOMING_SNAPSHOT_K) {
		g->akx = 0.5 * (-(k - 1) * SS_AK(ss, k - 1));
		g->aky = 0.5 * ((k - 1) * SS_BK(ss, k - 1));
		g->bkx = 0.5 * (-(k - 1) * SS_BK(ss, k - 1));
		g->bky = 0.5 * (-(k - 1) * SS_AK(ss, k - 1));
	} else {
		g->akx = 0.5
				* (-(k - 1) * SS_AK(ss, k - 1) + (k + 1) * SS_AK(ss, k + 1));
		g->aky = 0.5
				* ((k - 1) * SS_BK(ss, k - 1) + (k + 1) * SS_BK(ss, k + 1));
		g->bkx = 0.5
				* (-(k - 1) * SS_BK(ss, k - 1) + (k + 1) * SS_BK(ss, k + 1));
		g->bky = 0.5
				* (-(k - 1) * SS_AK(ss, k - 1) - (k + 1) * SS_AK(ss, k + 1));
	}
//	printf(
//			"\n@ k = %d:\nakx = % 7.2f\naky = % 7.2f\nbkx = % 7.2f\nbky = %7.2f\n\n",
//			k, g->akx, g->aky, g->bkx, g->bky);
}

static void get_Ak(
		struct FloatMat33 *ak,
		float akhs0,
		float bkhs0,
		struct spatial_gradient_t g,
		int k) {
	// Code based on MATLAB symbolic toolbox
	// 2017-03-03
	float t2 = g.akx * g.aky * 2.0;
	float t3 = g.bkx * g.bky * 2.0;
	float t4 = t2 + t3;
	float t5 = akhs0 * g.bkx;
	float t6 = t5 - g.akx * bkhs0;
	float t7 = k * t6 * 2.0;
	float t8 = akhs0 * g.bky;
	float t9 = t8 - g.aky * bkhs0;
	float t10 = k * t9 * 2.0;
	MAT33_ELMT(*ak, 0, 0) = (g.akx * g.akx) * 2.0 + (g.bkx * g.bkx) * 2.0;
	MAT33_ELMT(*ak, 0, 1) = t4;
	MAT33_ELMT(*ak, 0, 2) = t7;
	MAT33_ELMT(*ak, 1, 0) = t4;
	MAT33_ELMT(*ak, 1, 1) = (g.aky * g.aky) * 2.0 + (g.bky * g.bky) * 2.0;
	MAT33_ELMT(*ak, 1, 2) = t10;
	MAT33_ELMT(*ak, 2, 0) = t7;
	MAT33_ELMT(*ak, 2, 1) = t10;
	MAT33_ELMT(*ak, 2, 2) = (k * k) * (akhs0 * akhs0 + bkhs0 * bkhs0) * 2.0;
}

static void get_zk(
		struct FloatVect3 *zk,
		float ak,
		float bk,
		float akhs0,
		float bkhs0,
		struct spatial_gradient_t g,
		int k) {
	// Code based on MATLAB symbolic toolbox
	// 2017-03-03
	zk->x = ak * g.akx * 2.0 - akhs0 * g.akx * 2.0 + bk * g.bkx * 2.0
			- bkhs0 * g.bkx * 2.0;
	zk->y = ak * g.aky * 2.0 - akhs0 * g.aky * 2.0 + bk * g.bky * 2.0
			- bkhs0 * g.bky * 2.0;
	zk->z = k * (ak * bkhs0 - akhs0 * bk) * -2.0;
}

