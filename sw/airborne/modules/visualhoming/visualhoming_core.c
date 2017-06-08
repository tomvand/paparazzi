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
 * @file "modules/visualhoming/visualhoming_core.c"
 * @author Tom van Dijk
 * This file contains all platform-independent code for local visual homing.
 */

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <assert.h>

#include "visualhoming_core.h"

#include "math/pprz_algebra_float.h" /**< Replace with suitable header outside Paparazzi */

#include "ext/pffft/pffft.h"

#define SS_AK(ss, k) ( (ss)->data[2*((k)-1)] )
#define SS_BK(ss, k) ( (ss)->data[2*((k)-1)+1] )

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

struct spatial_gradient_t {
	float akx;
	float aky;
	float bkx;
	float bky;
};

static PFFFT_Setup *pffft_setup;

static float relative_orientation(
		const struct snapshot_t * current,
		const struct snapshot_t * target);
static struct FloatVect3 relative_pose(
		const struct snapshot_t * current,
		const struct snapshot_t * target);
static void snapshot_rotate(
		const struct snapshot_t* ss,
		struct snapshot_t * ss_rotated,
		float angle);
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

void visualhoming_core_init(void) {
	pffft_setup = pffft_new_setup(HORIZON_WIDTH, PFFFT_REAL);
}

void visualhoming_core_close(void) {
	pffft_destroy_setup(pffft_setup);
}

struct snapshot_t *snapshot_create(const float hor[]) {
	float fourier_coeffs[HORIZON_WIDTH];
	struct snapshot_t *ss = NULL;

	// Get (real) Fourier transform of horizon image.
	pffft_transform_ordered(pffft_setup, hor, fourier_coeffs, NULL,
			PFFFT_FORWARD);
	// Store first SNAPSHOT_K components.
	ss = (struct snapshot_t*)malloc(sizeof(struct snapshot_t));
	ss->dc[0] = fourier_coeffs[0] / HORIZON_WIDTH;
	ss->dc[1] = fourier_coeffs[1] / HORIZON_WIDTH;
	for (int k = 1; k <= SNAPSHOT_K; k++) {
		SS_AK(ss,k) = fourier_coeffs[2 * k] / HORIZON_WIDTH;
		SS_BK(ss,k) = -fourier_coeffs[2 * k + 1] / HORIZON_WIDTH; // Minus sign following definition in Sturzl and Mallot (2006)
	}

	// XXX show debug info
//	printf("\nSnapshot:\n");
//	printf("DC+Nq: %.2f%+.2fi\t", ss->dc[0], ss->dc[1]);
//	for (int k = 1; k <= SNAPSHOT_K; k++) {
//		printf("%d: %.2f%+.2fi\t", k, ss->data[2 * (k - 1)],
//				ss->data[2 * (k - 1) + 1]);
//	}
//	printf("\n");

	return ss;
}

void snapshot_free(struct snapshot_t *ss) {
	free(ss);
}

void snapshot_copy(struct snapshot_t * dest, const struct snapshot_t * src) {
	memcpy(dest, src, sizeof(struct snapshot_t));
}

struct homingvector_t homing_vector(
		const struct snapshot_t * current,
		const struct snapshot_t * target,
		struct snapshot_t **c_warped,
		struct snapshot_t **t_rotated) {
	// TODO remove temporary snapshot output
	static struct snapshot_t target_rotated;
	static struct snapshot_t current_warped;
	struct homingvector_t vec;

	vec.x = 0;
	vec.y = 0;
	vec.sigma = 0;

	snapshot_copy(&current_warped, current);

	for (int i = 0; i < SNAPSHOT_N_IT; i++) {
		float sigma;
		struct FloatVect3 pose;

		// Estimate and relative orientation and rotate target snapshot.
		sigma = relative_orientation(&current_warped, target);
		snapshot_rotate(target, &target_rotated, -sigma);
		// Estimate position
		pose = relative_pose(&current_warped, &target_rotated);
		printf("Delta_sigma = %.0f\n", pose.z / M_PI * 180);
		// Update homing vector and warp current snapshot accordingly
		vec.x += pose.x;
		vec.y += pose.y;
		vec.sigma = sigma;
		snapshot_warp(&current_warped, pose);
	}

	// XXX Debug output snapshots
	if (c_warped != NULL) {
		*c_warped = &current_warped;
	}
	if (t_rotated != NULL) {
		*t_rotated = &target_rotated;
	}

	return vec;
}

void snapshot_to_horizon(float hor[], const struct snapshot_t * ss) {
	float fourier_coeffs[HORIZON_WIDTH];

	// Read Fourier coefficients from snapshot
	memset(fourier_coeffs, 0, sizeof(fourier_coeffs));
	fourier_coeffs[0] = ss->dc[0];
	fourier_coeffs[1] = ss->dc[1];
	for (int k = 1; k <= SNAPSHOT_K; k++) {
		fourier_coeffs[2 * k] = SS_AK(ss, k);
		fourier_coeffs[2 * k + 1] = -SS_BK(ss, k);
	}
	// Perform inverse Fourier transform
	pffft_transform_ordered(pffft_setup, fourier_coeffs, hor, NULL,
			PFFFT_BACKWARD);
	// Fix under-/overflows
	for (int x = 0; x < HORIZON_WIDTH; x++) {
		if (hor[x] < 0) hor[x] = 0;
		if (hor[x] > 255) hor[x] = 255;
	}
}

static void snapshot_rotate(
		const struct snapshot_t* ss,
		struct snapshot_t * ss_rotated,
		float angle) {
	printf("Snapshot rotate angle = %.0f\n", angle / M_PI * 180);
	for (int k = 1; k <= SNAPSHOT_K; k++) {
		SS_AK(ss_rotated,k) = SS_AK(ss,k) * cos(k * angle)
				+ SS_BK(ss,k) * sin(k * angle);
		SS_BK(ss_rotated,k) = SS_BK(ss,k) * cos(k * angle)
				- SS_AK(ss,k) * sin(k * angle);
	}
	ss_rotated->dc[0] = ss->dc[0];
	ss_rotated->dc[1] = ss->dc[1]; // TODO remove
}

static void snapshot_warp(struct snapshot_t * ss, struct FloatVect3 pose) {
	struct snapshot_t ss_old;
	struct spatial_gradient_t g;

	snapshot_copy(&ss_old, ss);

	for (int k = 1; k <= SNAPSHOT_K; k++) {
		spatial_gradient(&g, &ss_old, k);
		SS_AK(ss,k) += g.akx * pose.x + g.aky * pose.y;
		SS_BK(ss,k) += g.bkx * pose.x + g.bky * pose.y;
	}
}

static float relative_orientation(
		const struct snapshot_t * current,
		const struct snapshot_t * target) {
	// See "Stürzl, W., & Mallot, H. A. (2006). Efficient visual homing based on
	// Fourier transformed panoramic images." section 2.4.
	float sigma = 0.0;
	float w = 0.0;
	for (int k = 1; k <= SNAPSHOT_K; k++) {
		float ac, bc, at, bt;
		float magn_c, phase_c;
		float magn_t, phase_t;
		int nk;
		float sigma_k;
		float w_k;

		ac = current->data[2 * (k - 1)];
		bc = current->data[2 * (k - 1) + 1];
		at = target->data[2 * (k - 1)];
		bt = target->data[2 * (k - 1) + 1];

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

	for (int k = 1; k <= SNAPSHOT_K; k++) {
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

static void spatial_gradient(
		struct spatial_gradient_t * g,
		const struct snapshot_t * ss,
		int k) {
	if (k == 1) {
		g->akx = 0.5 * ((k + 1) * SS_AK(ss, k + 1));
		g->aky = 0.5 * ((k + 1) * SS_BK(ss, k + 1));
		g->bkx = 0.5 * ((k + 1) * SS_BK(ss, k + 1));
		g->bky = 0.5 * (-(k + 1) * SS_AK(ss, k + 1));
	} else if (k == SNAPSHOT_K) {
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
	MAT33_ELMT(*ak,0,0) = (g.akx * g.akx) * 2.0 + (g.bkx * g.bkx) * 2.0;
	MAT33_ELMT(*ak,0,1) = t4;
	MAT33_ELMT(*ak,0,2) = t7;
	MAT33_ELMT(*ak,1,0) = t4;
	MAT33_ELMT(*ak,1,1) = (g.aky * g.aky) * 2.0 + (g.bky * g.bky) * 2.0;
	MAT33_ELMT(*ak,1,2) = t10;
	MAT33_ELMT(*ak,2,0) = t7;
	MAT33_ELMT(*ak,2,1) = t10;
	MAT33_ELMT(*ak,2,2) = (k * k) * (akhs0 * akhs0 + bkhs0 * bkhs0) * 2.0;
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
