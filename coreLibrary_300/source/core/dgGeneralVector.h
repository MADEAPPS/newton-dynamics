/* Copyright (c) <2003-2016> <Julio Jerez, Newton Game Dynamics>
*
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
*
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
*
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
*
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
*
* 3. This notice may not be removed or altered from any source distribution.
*/

#ifndef __dgGeneralVector__
#define __dgGeneralVector__

#include "dgStdafx.h"
#include "dgDebug.h"
#include "dgMemory.h"

template <class T>
DG_INLINE T dgSQRH(const T num, const T den)
{
	T r(num / den);
	return T(sqrt(T(1.0f) + r * r));
}

template <class T>
DG_INLINE T dgPythag(const T a, const T b)
{
	T absa(abs(a));
	T absb(abs(b));
	return (absa > absb) ? (absa * dgSQRH(absb, absa)) : ((absb == T(0.0f) ? T(0.0f) : (absb * dgSQRH(absa, absb))));
}

template <class T>
DG_INLINE T dgSign(const T a, const T b)
{
	return (b >= T(0.0f)) ? (a >= T(0.0f) ? a : -a) : (a >= T(0.0f) ? -a : a);
}


// return dot product
template<class T>
DG_INLINE T dgDotProduct(dgInt32 size, const T* const A, const T* const B)
{
	T val(0.0f);
	for (dgInt32 i = 0; i < size; i++) {
		val = val + A[i] * B[i];
	}
	return val;
}


template<class T>
DG_INLINE bool dgCholeskyFactorizationAddRow(dgInt32 size, dgInt32 n, T* const matrix)
{
	T* const rowN = &matrix[size * n];

	dgInt32 stride = 0;
	for (dgInt32 j = 0; j <= n; j++) {
		T s(0.0f);
		T* const rowJ = &matrix[stride];
		for (dgInt32 k = 0; k < j; k++) {
			s += rowN[k] * rowJ[k];
		}

		if (n == j) {
			T diag = rowN[n] - s;
			if (diag < T(dgFloat32(0.0f))) {
				// hack to prevent explosions when round error make the diagonal a small negative value
				if (diag < T(dgFloat32(-1.0e3f))) {
				dgAssert(0);
				return false;
			}
				diag = dgFloat32 (1.0e-12f);
			}
		
			rowN[n] = T(sqrt(diag));
		} else {
			rowN[j] = ((T(1.0f) / rowJ[j]) * (rowN[j] - s));
		}

		stride += size;
	}
	return true;
}

template<class T>
DG_INLINE void dgCholeskyRestore(dgInt32 size, dgInt32 from, dgInt32 to, T* const matrix, const T* const diagonal)
{
	dgInt32 stride = from * size;
	for (dgInt32 i = from; i < to; i++) {
		T* const row = &matrix[stride];
		row[i] = diagonal[i];
		for (dgInt32 j = 0; j < i; j++) {
			row[j] = matrix[size * j + i];
		}
		stride += size;
	}
}

template<class T>
DG_INLINE void dgCholeskySolve(dgInt32 size, dgInt32 n, const T* const choleskyMatrix, T* const x)
{
	dgInt32 stride = 0;
	for (dgInt32 i = 0; i < n; i++) {
		T acc(0.0f);
		const T* const row = &choleskyMatrix[stride];
		for (dgInt32 j = 0; j < i; j++) {
			acc = acc + row[j] * x[j];
		}
		x[i] = (x[i] - acc) / row[i];
		stride += size;
	}

	for (dgInt32 i = n - 1; i >= 0; i--) {
		T acc = 0.0f;
		for (dgInt32 j = i + 1; j < n; j++) {
			acc = acc + choleskyMatrix[size * j + i] * x[j];
		}
		x[i] = (x[i] - acc) / choleskyMatrix[size * i + i];
	}
}


// calculate delta_r = A * delta_x
template<class T>
DG_INLINE void dgCalculateDelta_r(dgInt32 size, dgInt32 n, const T* const matrix, const T* const delta_x, T* const delta_r)
{
	dgInt32 stride = n * size;
	for (dgInt32 i = n; i < size; i++) {
		delta_r[i] = dgDotProduct(size, &matrix[stride], delta_x);
		stride += size;
	}
}

template<class T>
DG_INLINE void dgCalculateDelta_x(dgInt32 size, T dir, dgInt32 n, const T* const matrix, T* const delta_x)
{
	const T* const row = &matrix[size * n];
	for (dgInt32 i = 0; i < n; i++) {
		delta_x[i] = -row[i] * dir;
	}
	dgCholeskySolve(size, n, matrix, delta_x);
	delta_x[n] = dir;
}


template<class T>
DG_INLINE void dgPermuteRows(dgInt32 size, dgInt32 i, dgInt32 j, T* const matrix, T* const x, T* const r, T* const low, T* const high, T* const diagonal, dgInt16* const permute)
{
	if (i != j) {
		T* const A = &matrix[size * i];
		T* const B = &matrix[size * j];
		for (dgInt32 k = 0; k < size; k++) {
			dgSwap(A[k], B[k]);
		}

		dgInt32 stride = 0;
		for (dgInt32 k = 0; k < size; k++) {
			dgSwap(matrix[stride + i], matrix[stride + j]);
			stride += size;
		}

		dgSwap(x[i], x[j]);
		dgSwap(r[i], r[j]);
		dgSwap(low[i], low[j]);
		dgSwap(high[i], high[j]);
		dgSwap(diagonal[i], diagonal[j]);
		dgSwap(permute[i], permute[j]);
	}
}

#endif


