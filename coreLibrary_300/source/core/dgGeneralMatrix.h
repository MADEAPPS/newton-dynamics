
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

#ifndef __dgGeneralMatrix__
#define __dgGeneralMatrix__

#include "dgStdafx.h"
#include "dgDebug.h"
#include "dgVector.h"
#include "dgGeneralVector.h"

#define DG_LCP_MAX_VALUE dgFloat32 (1.0e10f)

class dgSymmetricBiconjugateGradientSolve
{
	public:
	dgSymmetricBiconjugateGradientSolve();
	~dgSymmetricBiconjugateGradientSolve();

	dgFloat64 Solve(dgInt32 size, dgFloat64 tolerance, dgFloat64* const x, const dgFloat64* const b) const;

	protected:
	virtual void MatrixTimeVector(dgFloat64* const out, const dgFloat64* const v) const = 0;
	virtual bool InversePrecoditionerTimeVector(dgFloat64* const out, const dgFloat64* const v) const = 0;
	
	private:
	dgFloat64 DotProduct(dgInt32 size, const dgFloat64* const b, const dgFloat64* const c) const;
	void ScaleAdd(dgInt32 size, dgFloat64* const a, const dgFloat64* const b, dgFloat64 scale, const dgFloat64* const c) const;
	void Sub(dgInt32 size, dgFloat64* const a, const dgFloat64* const b, const dgFloat64* const c) const;
};


template<class T>
void dgMatrixTimeVector(dgInt32 size, const T* const matrix, const T* const v, T* const out)
{
	dgInt32 stride = 0;
	for (dgInt32 i = 0; i < size; i++) {
		out[i] = dgDotProduct(size, &matrix[stride], v);
		stride += size;
	}
}

template<class T>
void dgMatrixTimeMatrix(dgInt32 size, const T* const matrixA, const T* const matrixB, T* const out)
{
	for (dgInt32 i = 0; i < size; i++) {
		const T* const rowA = &matrixA[i * size];
		T* const rowOut = &out[i * size];
		for (dgInt32 j = 0; j < size; j++) {
			T acc = T(0.0f);
			for (dgInt32 k = 0; k < size; k++) {
				acc += rowA[k] * matrixB[j * size + k];
			}
			rowOut[j] = acc;
		}
	}
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
				diag = dgFloat32(1.0e-12f);
			}

			rowN[n] = T(sqrt(diag));
		}
		else {
			rowN[j] = ((T(1.0f) / rowJ[j]) * (rowN[j] - s));
		}

		stride += size;
	}
	return true;
}


template<class T>
void dgCholeskyFactorization(dgInt32 size, T* const psdMatrix)
{
	for (dgInt32 i = 0; i < size; i++) {
		dgCholeskyFactorizationAddRow(size, i, psdMatrix);
	}
}

template<class T>
void dgCholeskyInverse(dgInt32 size, T* const matrix, T* const invMatrix)
{

//	dgCholeskyFactorization(size, psdMatrix);


	memset(invMatrix, 0, sizeof(T)* size * size);
	for (dgInt32 i = 0; i < size; i++) {
		invMatrix[i * size + i] = T(dgFloat32(1.0f));
	}

	for (dgInt32 i = 0; i < size; i++) {
		T* const rowI = &matrix[i * size];
		T* const invRowI = &invMatrix[i * size];
		dgAssert (fabs (rowI[i]) > T(dgFloat32 (1.0e-6f)));
		T invDiag = T(dgFloat32(1.0f) / rowI[i]);
		for (dgInt32 j = 0; j < size; j++) {
			rowI[j] *= invDiag;
			invRowI[j] *= invDiag;
		}

		for (dgInt32 j = 0; j < i; j++) {
			T* const rowJ = &matrix[j * size];
			T* const invRowJ = &invMatrix[j * size];
			T scale = -rowJ[i];
			for (dgInt32 k = 0; k < size; k++) {
				rowJ[k] += rowI[k] * scale;
				invRowJ[k] += invRowI[k] * scale;
			}
		}

		for (dgInt32 j = i + 1; j < size; j++) {
			T* const rowJ = &matrix[j * size];
			T* const invRowJ = &invMatrix[j * size];
			T scale = -rowJ[i];
			for (dgInt32 k = 0; k < size; k++) {
				rowJ[k] += rowI[k] * scale;
				invRowJ[k] += invRowI[k] * scale;
			}
		}
	}
}


template<class T>
DG_INLINE void dgSolveCholesky(dgInt32 size, dgInt32 n, const T* const choleskyMatrix, T* const x)
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


template<class T>
void dgSolveCholesky(dgInt32 size, T* const choleskyMatrix, T* const x)
{
	dgSolveCholesky(size, size, choleskyMatrix, x);
}


template<class T>
bool dgSolveGaussian(dgInt32 size, T* const matrix, T* const b)
{
	for (dgInt32 i = 0; i < size - 1; i++) {
		const T* const rowI = &matrix[i * size];
		dgInt32 k = i;
		T maxVal (fabs(rowI[i]));
		for (dgInt32 j = i + 1; j < size - 1; j++) {
			T val (fabs(matrix[size * j + i]));
			if (val > maxVal) {
				k = j;
				maxVal = val;
			}
		}

		if (maxVal < T(1.0e-12f)) {
			return false;
		}

		if (k != i) {
			dgAssert(0);
//			dgGeneralMatrix<T, Rows, Columns>::SwapRows(i, k);
//			dgSwap(B[i], B[k]);
		}

		T den = T(1.0f) / rowI[i];
		for (dgInt32 k = i + 1; k < size; k++) {
			T* const rowK = &matrix[size * k];
			T factor(-rowK[i] * den);
			for (dgInt32 j = i + 1; j < size; j++) {
				rowK[j] += rowI[j] * factor;
			}
			rowK[i] = T(0.0f);
			b[k] += b[i] * factor;
		}
	}

	for (dgInt32 i = size - 1; i >= 0; i--) {
		T acc(0);
		T* const rowI = &matrix[i * size];
		for (dgInt32 j = i + 1; j < size; j++) {
			acc = acc + rowI[j] * b[j];
		}
		b[i] = (b[i] - acc) / rowI[i];
	}
	return true;
}


/*
template <class T>
DG_INLINE void dgHouseHolderReduction(const dgInt32 size, T* const matrix, T* const eigenValues, T* const offDiag)
{
	for (dgInt32 i = size - 1; i >= 1; i--) {
		const dgInt32 l = i - 1;
		T h(0.0f);
		T* const rowI = &matrix[i * size];

		if (l > 0) {
			T scale(0.0f);
			for (dgInt32 k = 0; k <= l; k++) {
				scale += T(abs(rowI[k]));
			}

			if (scale == T(0.0f)) {
				offDiag[i] = rowI[l];
			} else {
				for (dgInt32 k = 0; k <= l; k++) {
					rowI[k] /= scale;
					h += rowI[k] * rowI[k];
				}

				T f(rowI[l]);
				T g((f >= T(0.0f) ? -T(sqrt(h)) : T(sqrt(h))));
				offDiag[i] = scale * g;
				h -= f * g;
				rowI[l] = f - g;
				f = T(0.0f);

				for (dgInt32 j = 0; j <= l; j++) {
					g = T(0.0f);
					const T* const rowJ = &matrix[j * size];
					for (dgInt32 k = 0; k <= j; k++) {
						g += rowJ[k] * rowI[k];
					}
					for (dgInt32 k = j + 1; k <= l; k++) {
						g += matrix[k * size + j] * rowI[k];
					}
					offDiag[j] = g / h;
					f += offDiag[j] * rowI[j];
				}

				T hh(f / (h + h));
				for (dgInt32 j = 0; j <= l; j++) {
					T f(rowI[j]);
					T g(offDiag[j] - hh * f);
					offDiag[j] = g;
					T* const rowJ = &matrix[j * size];
					for (dgInt32 k = 0; k <= j; k++) {
						rowJ[k] -= (f * offDiag[k] + g * rowI[k]);
					}
				}
			}
		} else {
			offDiag[i] = rowI[l];
		}
		eigenValues[i] = h;
	}

	dgInt32 index = 0;
	for (dgInt32 i = 0; i < size; i++) {
		eigenValues[i] = matrix[index + i];
		index += size;
	}
}

*/

template <class T>
void dgEigenValues(const dgInt32 size, T* const matrix, T* const eigenValues)
{
	T* const offDiag = dgAlloca(T, size);
	for (dgInt32 i = size - 1; i > 0; i--) {
		T h(0.0f);
		T* const rowI = &matrix[i * size];

		if (i > 1) {
			T scale(0.0f);
			for (dgInt32 k = 0; k < i; k++) {
				scale += T(abs(rowI[k]));
			}

			if (scale == T(0.0f)) {
				offDiag[i] = rowI[i - 1];
			} else {
				for (dgInt32 k = 0; k < i; k++) {
					rowI[k] /= scale;
					h += rowI[k] * rowI[k];
				}

				T f(rowI[i - 1]);
				T g((f >= T(0.0f) ? -T(sqrt(h)) : T(sqrt(h))));
				offDiag[i] = scale * g;
				h -= f * g;
				rowI[i - 1] = f - g;
				f = T(0.0f);

				for (dgInt32 j = 0; j < i; j++) {
					g = T(0.0f);
					const T* const rowJ = &matrix[j * size];
					for (dgInt32 k = 0; k <= j; k++) {
						g += rowJ[k] * rowI[k];
					}
					for (dgInt32 k = j + 1; k < i; k++) {
						g += matrix[k * size + j] * rowI[k];
					}
					offDiag[j] = g / h;
					f += offDiag[j] * rowI[j];
				}

				T hh(f / (h + h));
				for (dgInt32 j = 0; j < i; j++) {
					T f(rowI[j]);
					T g(offDiag[j] - hh * f);
					offDiag[j] = g;
					T* const rowJ = &matrix[j * size];
					for (dgInt32 k = 0; k <= j; k++) {
						rowJ[k] -= (f * offDiag[k] + g * rowI[k]);
					}
				}
			}
		} else {
			offDiag[i] = rowI[i - 1];
		}
		eigenValues[i] = h;
	}

	dgInt32 index = size;
	eigenValues[0] = matrix[0];
	for (dgInt32 i = 1; i < size; i++) {
		eigenValues[i] = matrix[index + i];
		offDiag[i - 1] = offDiag[i];
		index += size;
	}

	for (dgInt32 i = 0; i < size; i++) {
		dgInt32 j;
		dgInt32 iter = 0;
		do {
			for (j = i; j < size - 1; j++) {
				T dd(abs(eigenValues[j]) + abs(eigenValues[j + 1]));
				if (abs(offDiag[j]) <= (T(1.e-6f) * dd)) {
					break;
				}
			}

			if (j != i) {
				iter++;
				if (iter == 10) {
					dgAssert(0);
					return;
				}

				T g((eigenValues[i + 1] - eigenValues[i]) / (T(2.0f) * offDiag[i]));
				T r(dgPythag(g, T(1.0f)));
				g = eigenValues[j] - eigenValues[i] + offDiag[i] / (g + dgSign(r, g));
				T s(1.0f);
				T c(1.0f);
				T p(0.0f);

				dgInt32 k;
				for (k = j - 1; k >= i; k--) {
					T f(s * offDiag[k]);
					T b(c * offDiag[k]);
					T r(dgPythag(f, g));
					offDiag[k + 1] = r;
					if (r == T(0.0f)) {
						eigenValues[k + 1] -= p;
						offDiag[j] = T(0.0f);
						break;
					}
					s = f / r;
					c = g / r;
					g = eigenValues[k + 1] - p;
					r = (eigenValues[k] - g) * s + T(2.0f) * c * b;
					p = s * r;
					eigenValues[k + 1] = g + p;
					g = c * r - b;
				}

				if (r == T(0.0f) && k >= i) {
					continue;
				}
				eigenValues[i] -= p;
				offDiag[i] = g;
				offDiag[j] = T(0.0f);
			}
		} while (j != i);
	}
}


// solve a general Linear complementary program (LCP)
// A * x = b + r
// subjected to constraints
// x(i) = low(i),  if r(i) >= 0  
// x(i) = high(i), if r(i) <= 0  
// low(i) <= x(i) <= high(i),  if r(i) == 0  
//
// return true is the system has a solution.
// in return 
// x is the solution,
// r is return in vector b
// note: although the system is called LCP, the solver is far more general than a strict LCP
// to solve a strict LCP, set the following
// low(i) = 0
// high(i) = infinity.
// this the same as enforcing the constraint: x(i) * r(i) = 0
template <class T>
void dgGaussSeidelLCP(const dgInt32 size, const T* const matrix, T* const x, const T* const b, const T* const low, const T* const high)
{
	const T* const me = matrix;
	T* const invDiag1 = dgAlloca(T, size);

	dgInt32 base = 0;
	for (dgInt32 i = 0; i < size; i++) {
		x[i] = dgClamp(x[i], low[i], high[i]);
		invDiag1[i] = T(1.0f) / me[base + i];
		base += size;
	}

	T tol(1.0e-6f);
	T tol2 = tol * tol;
	T tolerance(1.0e6f);

	const T* const invDiag = invDiag1;
	const dgInt32 maxIterCount = size * size * size * size + 100000;
	for (dgInt32 i = 0; (i < maxIterCount) && (tolerance > tol2); i++) {
		tolerance = T(dgFloat32(0.0f));
		dgInt32 base = 0;
		for (dgInt32 j = 0; j < size; j++) {
			const T* const row = &me[base];
			T r(b[j] - dgDotProduct(size, row, x));
			T f((r + row[j] * x[j]) * invDiag[j]);
			if (f > high[j]) {
				f = high[j];
			} else if (f < low[j]) {
				f = low[j];
			} else {
				tolerance += r * r;
			}
			x[j] = f;
			base += size;
		}
	}
	tolerance *= 1;
}

// solve a general Linear complementary program (LCP)
// A * x = b + r
// subjected to constraints
// x(i) = low(i),  if r(i) >= 0  
// x(i) = high(i), if r(i) <= 0  
// low(i) <= x(i) <= high(i),  if r(i) == 0  
//
// return true is the system has a solution.
// in return 
// x is the solution,
// note: although the system is called LCP, the solver is far more general than a strict LCP
// to solve a strict LCP, set the following
// low(i) = 0
// high(i) = infinity.
// this is the same as enforcing the constraint: x(i) * r(i) = 0
template <class T>
bool dgSolvePartitionDantzigLCP(dgInt32 size, T* const matrix, T* const x, T* const b, T* const low, T* const high)
{
dgAssert (0);
	T* const f = dgAlloca(T, size);
	dgInt16* const permute = dgAlloca(short, size);

	bool applyInitialGuess = false;
	for (dgInt32 i = 0; i < size; i++) {
		permute[i] = short(i);
		f[i] = dgClamp(x[i], low[i], high[i]);
		if ((low[i] > T(-DG_LCP_MAX_VALUE)) || (high[i] < T(DG_LCP_MAX_VALUE))) {
			low[i] -= f[i];
			high[i] -= f[i];
		}
		applyInitialGuess |= (f[i] != dgFloat32(0.0f));
	}

	if (applyInitialGuess) {
		dgMatrixTimeVector(size, matrix, f, x);
		for (dgInt32 i = 0; i < size; i++) {
			b[i] -= x[i];
			x[i] = b[i];
		}
	} else {
		for (dgInt32 i = 0; i < size; i++) {
			x[i] = b[i];
		}
	}

	dgInt32 unboundedSize = size;
	for (dgInt32 i = 0; i < unboundedSize; i++) {
		if ((low[i] <= T(-DG_LCP_MAX_VALUE)) && (high[i] >= T(DG_LCP_MAX_VALUE))) {
			dgCholeskyFactorizationAddRow(size, i, matrix);
		} else {
			dgInt32 j = unboundedSize - 1;
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
				dgSwap(b[i], b[j]);
				dgSwap(low[i], low[j]);
				dgSwap(high[i], high[j]);
				dgSwap(permute[i], permute[j]);
			}

			i--;
			unboundedSize--;
		}
	}

	bool ret = false;
	if (unboundedSize > 0) {
		dgSolveCholesky(size, unboundedSize, matrix, x);
		dgInt32 base = unboundedSize * size;
		for (dgInt32 i = unboundedSize; i < size; i++) {
			const T* const row = &matrix[base];
			T acc(dgFloat32(0.0f));
			for (dgInt32 j = 0; j < unboundedSize; j++) {
				acc += row[j] * x[j];
			}
			b[i] = acc - b[i];
			x[i] = T(dgFloat32(0.0f));
			base += size;
		}

		const dgInt32 boundedSize = size - unboundedSize;
		T* const l = dgAlloca(T, boundedSize);
		T* const h = dgAlloca(T, boundedSize);
		T* const c = dgAlloca(T, boundedSize);
		T* const u = dgAlloca(T, boundedSize);
		T* const a = dgAlloca(T, boundedSize * boundedSize);
		T* const r = dgAlloca(T, boundedSize * unboundedSize);

		for (dgInt32 i = 0; i < boundedSize; i++) {
			T* const g = &r[i * unboundedSize];
			const T* const row = &matrix[(unboundedSize + i) * size];
			for (dgInt32 j = 0; j < unboundedSize; j++) {
				g[j] = -row[j];
			}
			dgSolveCholesky(size, unboundedSize, matrix, g);

			T* const arow = &a[i * boundedSize];
			for (dgInt32 j = 0; j < boundedSize; j++) {
				T* const row = &matrix[(unboundedSize + j) * size];
				T acc = dgFloat32(0.0f);
				for (dgInt32 k = 0; k < unboundedSize; k++) {
					acc += row[k] * g[k];
				}
				arow[j] = acc + row[unboundedSize + i];
			}
			u[i] = T(dgFloat32(0.0f));
			c[i] = -b[i + unboundedSize];
			l[i] = low[i + unboundedSize];
			h[i] = high[i + unboundedSize];
		}

		if (dgSolveDantzigLCP(boundedSize, a, u, c, l, h)) {
			for (dgInt32 i = 0; i < boundedSize; i++) {
				const T s = u[i];
				x[unboundedSize + i] = s;
				const T* const g = &r[i * unboundedSize];
				for (dgInt32 j = 0; j < unboundedSize; j++) {
					x[j] += g[j] * s;
				}
			}
			ret = true;
		}
	} else {
		for (dgInt32 i = 0; i < size; i++) {
			x[i] = T(dgFloat32(0.0f));
		}
		ret = dgSolveDantzigLCP(size, matrix, x, b, low, high);
	}

	for (dgInt32 i = 0; i < size; i++) {
		b[i] = x[i];
	}
	for (dgInt32 i = 0; i < size; i++) {
		dgInt32 j = permute[i];
		x[j] = f[j] + b[i];
	}
	return ret;
}


#if 1

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


template<class T>
DG_INLINE void dgCalculateDelta_x(dgInt32 size, T dir, dgInt32 n, const T* const matrix, T* const delta_x)
{
	const T* const row = &matrix[size * n];
	for (dgInt32 i = 0; i < n; i++) {
		delta_x[i] = -row[i] * dir;
	}
	dgSolveCholesky(size, n, matrix, delta_x);
	delta_x[n] = dir;
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


// solve a general Linear complementary program (LCP)
// A * x = b + r
// subjected to constraints
// x(i) = low(i),  if r(i) >= 0  
// x(i) = high(i), if r(i) <= 0  
// low(i) <= x(i) <= high(i),  if r(i) == 0  
//
// return true is the system has a solution.
// in return 
// x is the solution,
// r is return in vector b
// note: although the system is called LCP, the solver is far more general than a strict LCP
// to solve a strict LCP, set the following
// low(i) = 0
// high(i) = infinity.
// this the same as enforcing the constraint: x(i) * r(i) = 0
template <class T>
bool dgSolveDantzigLCP(dgInt32 size, T* const matrix, T* const x, T* const b, T* const low, T* const high)
{
	T* const x0 = dgAlloca(T, size);
	T* const r0 = dgAlloca(T, size);
	T* const delta_r = dgAlloca(T, size);
	T* const diagonal = dgAlloca(T, size);
	dgInt16* const permute = dgAlloca(short, size);
	T* const delta_x = b;

	dgInt32 stride = 0;
	bool applyInitialGuess = false;
	for (dgInt32 i = 0; i < size; i++) {
		x[i] = dgClamp(x[i], low[i], high[i]);
		x0[i] = x[i];
		if ((low[i] > T(-DG_LCP_MAX_VALUE)) || (high[i] < T(DG_LCP_MAX_VALUE))) {
			low[i] -= x0[i];
			high[i] -= x0[i];
		}
		applyInitialGuess |= (x0[i] != dgFloat32(0.0f));
		permute[i] = short(i);
		diagonal[i] = matrix[stride + i];
		stride += size;
	}

	if (applyInitialGuess) {
		dgMatrixTimeVector(size, matrix, x0, r0);
		for (dgInt32 i = 0; i < size; i++) {
			r0[i] -= b[i];
			x0[i] = T(0.0f);
		}
	} else {
		for (dgInt32 i = 0; i < size; i++) {
			r0[i] = -b[i];
		}
	}

	dgInt32 index = 0;
	dgInt32 last = size;
	for (dgInt32 i = 0; i < last; i++) {
		if ((low[i] <= T(-DG_LCP_MAX_VALUE)) && (high[i] >= T(DG_LCP_MAX_VALUE))) {
			dgCholeskyFactorizationAddRow(size, index, matrix);
			index++;
		} else {
			dgPermuteRows(size, i, last - 1, matrix, x0, r0, low, high, diagonal, permute);
			i--;
			last--;
		}
	}

	if (index > 0) {
		dgSolveCholesky(size, index, matrix, r0);
		for (dgInt32 i = 0; i < index; i++) {
			x0[i] -= r0[i];
			r0[i] = T(dgFloat32(0.0f));
		}
		dgCalculateDelta_r(size, index, matrix, x0, delta_r);
		for (dgInt32 i = index; i < size; i++) {
			r0[i] += delta_r[i];
		}
	}
	
	for (dgInt32 i = 0; i < size; i++) {
		delta_x[i] = T(dgFloat32(0.0f));
		delta_r[i] = T(dgFloat32(0.0f));
	}

	const dgInt32 start = index;
	dgInt32 clampedIndex = size;
	dgInt32 count = size - index;
	while (count) {
		bool loop = true;
		bool calculateDelta_x = true;

		while (loop) {
			loop = false;
			T clamp_x(0.0f);
			dgInt32 swapIndex = -1;

			if (dgAbsf(r0[index]) > T(1.0e-12f)) {

				if (calculateDelta_x) {
					T dir(dgFloat32(1.0f));
					dgCalculateDelta_x(size, dir, index, matrix, delta_x);
				}

				calculateDelta_x = true;
				dgCalculateDelta_r(size, index, matrix, delta_x, delta_r);
				dgAssert(delta_r[index] != T(dgFloat32(0.0f)));
				dgAssert(dgAbsf(delta_x[index]) == T(1.0f));

				T s = -r0[index] / delta_r[index];
				dgAssert(dgAbsf(s) >= T(dgFloat32(0.0f)));

				for (dgInt32 i = start; i <= index; i++) {
					T x1 = x0[i] + s * delta_x[i];
					if (x1 > high[i]) {
						swapIndex = i;
						clamp_x = high[i];
						s = (high[i] - x0[i]) / delta_x[i];
					} else if (x1 < low[i]) {
						swapIndex = i;
						clamp_x = low[i];
						s = (low[i] - x0[i]) / delta_x[i];
					}
				}
				dgAssert(dgAbsf(s) >= T(dgFloat32(0.0f)));

				for (dgInt32 i = clampedIndex; (i < size) && (s > T(1.0e-12f)); i++) {
					T r1 = r0[i] + s * delta_r[i];
					if ((r1 * r0[i]) < T(dgFloat32(0.0f))) {
						dgAssert(dgAbsf(delta_r[i]) > T(dgFloat32(0.0f)));
						T s1 = -r0[i] / delta_r[i];
						dgAssert(dgAbsf(s1) >= T(dgFloat32(0.0f)));
						dgAssert(dgAbsf(s1) <= dgAbsf(s));
						if (dgAbsf(s1) < dgAbsf(s)) {
							s = s1;
							swapIndex = i;
						}
					}
				}

				for (dgInt32 i = 0; i < size; i++) {
					dgAssert((x0[i] + dgAbsf(x0[i]) * T(dgFloat32(1.0e-4f))) >= low[i]);
					dgAssert((x0[i] - dgAbsf(x0[i]) * T(dgFloat32(1.0e-4f))) <= high[i]);

					x0[i] += s * delta_x[i];
					r0[i] += s * delta_r[i];

					dgAssert((x0[i] + dgFloat32(1.0f)) >= low[i]);
					dgAssert((x0[i] - dgFloat32(1.0f)) <= high[i]);
				}
			}

			if (swapIndex == -1) {
				r0[index] = T(dgFloat32(0.0f));
				delta_r[index] = T(dgFloat32(0.0f));
				if (!dgCholeskyFactorizationAddRow(size, index, matrix)) {
					return false;
				}
				index++;
				count--;
				loop = false;
			} else if (swapIndex == index) {
				count--;
				clampedIndex--;
				x0[index] = clamp_x;
				dgPermuteRows(size, index, clampedIndex, matrix, x0, r0, low, high, diagonal, permute);
				loop = count ? true : false;
			} else if (swapIndex > index) {
				loop = true;
				r0[swapIndex] = T(dgFloat32(0.0f));
				dgAssert(swapIndex < size);
				dgAssert(clampedIndex <= size);
				if (swapIndex < clampedIndex) {
					count--;
					clampedIndex--;
					dgPermuteRows(size, clampedIndex, swapIndex, matrix, x0, r0, low, high, diagonal, permute);
					dgAssert(clampedIndex >= index);
				} else {
					count++;
					dgAssert(clampedIndex < size);
					dgPermuteRows(size, clampedIndex, swapIndex, matrix, x0, r0, low, high, diagonal, permute);
					clampedIndex++;
					dgAssert(clampedIndex <= size);
					dgAssert(clampedIndex >= index);
				}
				calculateDelta_x = false;

			} else {
				dgAssert(index > 0);
				x0[swapIndex] = clamp_x;
				delta_x[index] = T(dgFloat32(0.0f));

				dgAssert(swapIndex < index);
				dgCholeskyRestore(size, swapIndex, index, matrix, diagonal);
				dgPermuteRows(size, swapIndex, index - 1, matrix, x0, r0, low, high, diagonal, permute);
				dgPermuteRows(size, index - 1, index, matrix, x0, r0, low, high, diagonal, permute);
				dgPermuteRows(size, clampedIndex - 1, index, matrix, x0, r0, low, high, diagonal, permute);

				clampedIndex--;
				index--;
				for (dgInt32 i = swapIndex; i < index; i++) {
					dgCholeskyFactorizationAddRow(size, i, matrix);
				}
				loop = true;
			}
		}
	}

	for (dgInt32 i = 0; i < size; i++) {
		dgInt32 j = permute[i];
		x[j] += x0[i];
		b[j] = r0[i];
	}
	return true;
}



#else

template<class T>
DG_INLINE void dgPermuteRows(dgInt32 size, dgInt32 i, dgInt32 j, T* const matrix, T* const choleskyMatrix, T* const x, T* const r, T* const low, T* const high, dgInt16* const permute)
{
	if (i != j) {
		T* const A = &matrix[size * i];
		T* const B = &matrix[size * j];
		T* const invA = &choleskyMatrix[size * i];
		T* const invB = &choleskyMatrix[size * j];
		for (dgInt32 k = 0; k < size; k++) {
			dgSwap(A[k], B[k]);
			dgSwap(invA[k], invB[k]);
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
		dgSwap(permute[i], permute[j]);
	}
}

template<class T>
DG_INLINE void dgCalculateDelta_x(dgInt32 size, T dir, dgInt32 n, const T* const matrix, const T* const choleskyMatrix, T* const delta_x)
{
	const T* const row = &matrix[size * n];
	for (dgInt32 i = 0; i < n; i++) {
		delta_x[i] = -row[i] * dir;
	}
	dgSolveCholesky(size, n, choleskyMatrix, delta_x);
	delta_x[n] = dir;
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
//DG_INLINE void dgCholeskyUpdate(dgInt32 size, dgInt32 row, dgInt32 colum, T* const choleskyMatrix)
void dgCholeskyUpdate(dgInt32 size, dgInt32 row, dgInt32 colum, T* const choleskyMatrix)
{
	if (row != colum) {
		dgAssert (row < colum);
/*
		for (dgInt32 i = row; i < size; i ++) {
			T* const rowI = &choleskyMatrix[size * i];
			for (dgInt32 j = colum; j > i; j--) {
				T x1 = rowI[j];
				if (x1 != T(dgFloat32(0.0f))) {
					T x0 = rowI[i];
					T den = T (1.0f) / T (sqrt (x1 * x1 + x0 * x0));
					x0 *= den;
					x1 *= den;

					for (dgInt32 k = i; k < size; k ++) {
						T* const rowk = &choleskyMatrix[size * k];
						T a0 (rowk[i]);
						T a1 (rowk[j]);
						rowk[i] = a0 * x0 + a1 * x1;
						rowk[j] = a1 * x0 - a0 * x1;
					}
					rowI[j] = T (dgFloat32 (0.0f));
				}
			}
		}
*/

		T* const tmp = dgAlloca(T, size);
		T* const reflexion = dgAlloca(T, size);

		for (dgInt32 i = row; i < size; i ++) {
			T* const rowI = &choleskyMatrix[size * i];
			T mag (T (dgFloat32 (0.0f)));
			for (dgInt32 j = i + 1; j < size; j ++) {
				mag += rowI[j] * rowI[j];
				reflexion[j] = rowI[j];
			}
			if (mag > T(dgFloat32 (1.0e-10f))) {
				reflexion[i] = rowI[i] - T (sqrt(mag + rowI[i] * rowI[i]));

				T vMag2 (mag + reflexion[i] * reflexion[i]);
				T den (dgFloat32 (1.0f) / T (sqrt (vMag2)));
				for (dgInt32 j = i; j < size; j ++) {
					reflexion[j] *= den;
				}

				for (dgInt32 j = i; j < size; j ++) {
					T acc (0.0f);
					T* const rowJ = &choleskyMatrix[size * j];
					for (dgInt32 k = i; k < size; k ++) {
						acc += rowJ[k] * reflexion[k];
					}
					tmp[j] = acc * T (dgFloat32 (2.0f));
				}

				for (dgInt32 j = i + 1; j < size; j++) {
					T* const rowJ = &choleskyMatrix[size * j];
					for (dgInt32 k = i; k < size; k++) {
						rowJ[k] -= tmp[j] * reflexion[k];
					}
				}
				rowI[i] -= tmp[i] * reflexion[i];
			}
		
			for (dgInt32 k = i + 1; k < size; k++) {
				rowI[k] = T(dgFloat32(0.0f));
			}

			if (rowI[i] < T (dgFloat32 (1.0e-5f))) {
				for (dgInt32 k = i; k < size; k++) {
					choleskyMatrix[size * k + i] = -choleskyMatrix[size * k + i];
				}
			}
			dgAssert (rowI[i] > T (dgFloat32 (0.0f)));
		}
	}
}

template<class T>
bool dgCheckCholeskyUpdate(dgInt32 size, const T* const choleskyMatrix, const T* const matrixSource)
{
#ifdef _DEBUG
	T* const matrix = dgAlloca(T, size * size);
	memcpy (matrix, matrixSource, sizeof (T) * size * size);
	dgCholeskyFactorization(size, matrix);
	for (dgInt32 i = 0; i < size; i++) {
		const T* const rowA = &matrix[i * size];
		const T* const rowB = &choleskyMatrix[i * size];
		for (dgInt32 j = 0; j <= i; j++) {
//			dgAssert (dgAreEqual(rowA[j], rowB[j], T (dgFloat32 (1.0e-2f))));
		}
	}
#endif
	return true;
}


// solve a general Linear complementary program (LCP)
// A * x = b + r
// subjected to constraints
// x(i) = low(i),  if r(i) >= 0  
// x(i) = high(i), if r(i) <= 0  
// low(i) <= x(i) <= high(i),  if r(i) == 0  
//
// return true is the system has a solution.
// in return 
// x is the solution,
// r is return in vector b
// note: although the system is called LCP, the solver is far more general than a strict LCP
// to solve a strict LCP, set the following
// low(i) = 0
// high(i) = infinity.
// this the same as enforcing the constraint: x(i) * r(i) = 0
template <class T>
bool dgSolveDantzigLCP(dgInt32 size, T* const matrix, T* const choleskyMatrix, T* const x, T* const b, T* const low, T* const high)
{
	T* const x0 = dgAlloca(T, size);
	T* const r0 = dgAlloca(T, size);
	T* const delta_r = dgAlloca(T, size);
	dgInt16* const permute = dgAlloca(short, size);
	T* const delta_x = b;

	dgInt32 stride = 0;
	for (dgInt32 i = 0; i < size; i++) {
		x[i] = dgClamp(x[i], low[i], high[i]);
		x0[i] = x[i];
		r0[i] = -b[i];
		low[i] -= x0[i];
		high[i] -= x0[i];
		delta_x[i] = T(dgFloat32(0.0f));
		delta_r[i] = T(dgFloat32(0.0f));
		permute[i] = short(i);
		stride += size;
	}

	dgInt32 index = 0;
	dgInt32 count = size;
	dgInt32 clampedIndex = size;
	const dgInt32 start = 0;
	while (count) {
		bool loop = true;

		while (loop) {
			loop = false;
			T clamp_x(0.0f);
			dgInt32 swapIndex = -1;

			if (dgAbsf(r0[index]) > T(1.0e-12f)) {

				T dir(dgFloat32(1.0f));
				dgCalculateDelta_x(size, dir, index, matrix, choleskyMatrix, delta_x);
				dgCalculateDelta_r(size, index, matrix, delta_x, delta_r);
				dgAssert(delta_r[index] != T(dgFloat32(0.0f)));
				dgAssert(dgAbsf(delta_x[index]) == T(1.0f));

				T s = -r0[index] / delta_r[index];
				dgAssert(dgAbsf(s) >= T(dgFloat32(0.0f)));

				for (dgInt32 i = start; i <= index; i++) {
					T x1 = x0[i] + s * delta_x[i];
					if (x1 > high[i]) {
						swapIndex = i;
						clamp_x = high[i];
						s = (high[i] - x0[i]) / delta_x[i];
					} else if (x1 < low[i]) {
						swapIndex = i;
						clamp_x = low[i];
						s = (low[i] - x0[i]) / delta_x[i];
					}
				}
				dgAssert(dgAbsf(s) >= T(dgFloat32(0.0f)));

				for (dgInt32 i = clampedIndex; (i < size) && (s > T(1.0e-12f)); i++) {
					T r1 = r0[i] + s * delta_r[i];
					if ((r1 * r0[i]) < T(dgFloat32(0.0f))) {
						dgAssert(dgAbsf(delta_r[i]) > T(dgFloat32(0.0f)));
						T s1 = -r0[i] / delta_r[i];
						dgAssert(dgAbsf(s1) >= T(dgFloat32(0.0f)));
						dgAssert(dgAbsf(s1) <= dgAbsf(s));
						if (dgAbsf(s1) < dgAbsf(s)) {
							s = s1;
							swapIndex = i;
						}
					}
				}

				for (dgInt32 i = 0; i < size; i++) {
					dgAssert((x0[i] + dgAbsf(x0[i]) * T(dgFloat32(1.0e-4f))) >= low[i]);
					dgAssert((x0[i] - dgAbsf(x0[i]) * T(dgFloat32(1.0e-4f))) <= high[i]);

					x0[i] += s * delta_x[i];
					r0[i] += s * delta_r[i];

					dgAssert((x0[i] + dgFloat32(1.0f)) >= low[i]);
					dgAssert((x0[i] - dgFloat32(1.0f)) <= high[i]);
				}
			}

			if (swapIndex == -1) {
				r0[index] = T(dgFloat32(0.0f));
				delta_r[index] = T(dgFloat32(0.0f));
				index++;
				count--;
				loop = false;
			} else if (swapIndex == index) {
				count--;
				clampedIndex--;
				x0[index] = clamp_x;
				dgPermuteRows(size, index, clampedIndex, matrix, choleskyMatrix, x0, r0, low, high, permute);
				dgCholeskyUpdate(size, index, clampedIndex, choleskyMatrix);
				//dgAssert(dgCheckCholeskyUpdate(size, choleskyMatrix, matrix));

				loop = count ? true : false;
			} else if (swapIndex > index) {
				loop = true;
				r0[swapIndex] = T(dgFloat32(0.0f));
				dgAssert(swapIndex < size);
				dgAssert(clampedIndex <= size);
				if (swapIndex < clampedIndex) {
					count--;
					clampedIndex--;
					dgPermuteRows(size, clampedIndex, swapIndex, matrix, choleskyMatrix, x0, r0, low, high, permute);
					dgCholeskyUpdate(size, swapIndex, clampedIndex, choleskyMatrix);
					//dgAssert(dgCheckCholeskyUpdate(size, choleskyMatrix, matrix));
					dgAssert(clampedIndex >= index);
				} else {
					count++;
					dgAssert(clampedIndex < size);
					dgPermuteRows(size, clampedIndex, swapIndex, matrix, choleskyMatrix, x0, r0, low, high, permute);
					dgCholeskyUpdate(size, clampedIndex, swapIndex, choleskyMatrix);
					//dgAssert(dgCheckCholeskyUpdate(size, choleskyMatrix, matrix));
					clampedIndex++;
					dgAssert(clampedIndex <= size);
					dgAssert(clampedIndex >= index);
				}

			} else {
				dgAssert(index > 0);
				x0[swapIndex] = clamp_x;
				delta_x[index] = T(dgFloat32(0.0f));

				dgAssert(swapIndex < index);
				dgPermuteRows(size, swapIndex, index - 1, matrix, choleskyMatrix, x0, r0, low, high, permute);
				dgPermuteRows(size, index - 1, index, matrix, choleskyMatrix, x0, r0, low, high, permute);
				dgPermuteRows(size, clampedIndex - 1, index, matrix, choleskyMatrix, x0, r0, low, high, permute);
				dgCholeskyUpdate (size, swapIndex, clampedIndex - 1, choleskyMatrix);
				//dgAssert (dgCheckCholeskyUpdate(size, choleskyMatrix, matrix));

				clampedIndex--;
				index--;
				loop = true;
			}
		}
	}

	for (dgInt32 i = 0; i < size; i++) {
		dgInt32 j = permute[i];
		x[j] += x0[i];
		b[j] = r0[i];
	}
	return true;
}

template <class T>
bool dgSolveDantzigLCP(dgInt32 size, T* const matrix, T* const x, T* const b, T* const low, T* const high)
{
	T* const choleskyMatrix = dgAlloca(T, size * size);
	memcpy (choleskyMatrix, matrix, sizeof (T) * size * size);
	dgCholeskyFactorization(size, choleskyMatrix);
	for (dgInt32 i = 0; i < size; i ++) {
		T* const row = &choleskyMatrix[i * size];
		for (dgInt32 j = i + 1; j < size; j ++) {
			row[j] = T(dgFloat32 (0.0));
		}
	}
	return dgSolveDantzigLCP(size, matrix, choleskyMatrix, x, b, low, high);
}

#endif


#endif
