
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


bool dgCholeskyFactorization(dgInt32 size, dgFloat32* const psdMatrix, dgInt32 rowStride);

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


template<dgInt32 maxRows>
class dgOldSolverNetwon_1_5
{
public:
	dgOldSolverNetwon_1_5()
	{
	}

	void SetSize(dgInt32 size)
	{
		m_size = size;
	}

	dgInt32 GetSize() const
	{
		return m_size;
	}

	dgFloat32* GetX()
	{
		return m_x;
	}

	dgFloat32* GetB()
	{
		return m_b;
	}

	dgFloat32* GetLow()
	{
		return m_low;
	}

	dgFloat32* GetHigh()
	{
		return m_high;
	}

	dgInt32* GetFrictionIndex()
	{
		return m_frictionIndex;
	}

	dgFloat32* GetInvDiag()
	{
		return m_invDiag;
	}

	dgFloat32* GetMatrixRow(dgInt32 i)
	{
		return &m_matrix[i * m_size];
	}

	dgFloat32 Solve()
	{
		dgInt32 stride = 0;
		m_x[m_size] = dgFloat32(1.0f);
		dgFloat32 accelNorm = dgFloat32(0.0f);
		for (dgInt32 i = 0; i < m_size; i++) {
			dgVector error(m_b[i]);
			dgVector x(m_x[i]);
			const dgInt32 frictionIndex = m_frictionIndex[i];
			const dgVector low(m_low[i] * m_x[frictionIndex]);
			const dgVector high(m_high[i] * m_x[frictionIndex]);
			error = error.AndNot((x > high) | (x < low));
			accelNorm += error.GetScalar() * error.GetScalar();
			stride += m_size;
		}

		const dgFloat32 tol2 = dgFloat32(1.0e-5f);
		if (accelNorm > tol2) {
			dgFloat32 accelNorm0 = accelNorm;
			for (dgInt32 i = 0; (i < 5) && (accelNorm0 > tol2); i++) {
				stride = 0;
				accelNorm0 = dgFloat32(0.0f);
				for (dgInt32 j = 0; j < m_size; j++) {
					const dgFloat32* const row = &m_matrix[stride];
					dgFloat32 r = m_b[j];
					for (dgInt32 k = 0; k < m_size; k++) {
						r = r - row[k] * m_x[k];
					}
					const dgInt32 frictionIndex = m_frictionIndex[j];
					const dgVector low(m_low[j] * m_x[frictionIndex]);
					const dgVector high(m_high[j] * m_x[frictionIndex]);
					dgVector x((r + row[j] * m_x[j]) * m_invDiag[j]);

					dgVector a(r);
					a = a.AndNot((x > high) | (x < low));
					x = x.GetMax(low).GetMin(high);
					m_x[j] = x.GetScalar();

					accelNorm0 += a.GetScalar() * a.GetScalar();
					stride += m_size;
				}
			}

			if (accelNorm0 > tol2) {
				stride = 0;
				dgFloat32 mask[maxRows];
				for (dgInt32 i = 0; i < m_size; i++) {
					dgFloat32 r = dgFloat32(0.0f);
					const dgFloat32* const row = &m_matrix[stride];
					for (dgInt32 j = 0; j < m_size; j++) {
						r += row[j] * m_x[j];
					}
					m_b[i] -= r;
					m_delta_x[i] = m_b[i];
					mask[i] = dgFloat32(1.0f);
					const dgInt32 frictionIndex = m_frictionIndex[i];
					m_low[i] *= m_x[frictionIndex];
					m_high[i] *= m_x[frictionIndex];
					stride += m_size;
				}

				dgFloat32 beta = dgFloat32(1.0f);
				for (dgInt32 k = 0; (k < 20) && (beta > tol2); k++) {
					stride = 0;
					dgFloat32 num = dgFloat32(0.0f);
					dgFloat32 den = dgFloat32(0.0f);
					for (dgInt32 i = 0; i < m_size; i++) {
						const dgFloat32* const row = &m_matrix[stride];
						dgFloat32 r = dgFloat32(0.0f);
						for (dgInt32 j = 0; j < m_size; j++) {
							r += row[j] * m_delta_x[j];
						}
						stride += m_size;
						m_delta_r[i] = r;
						den += m_delta_x[i] * r;
						num += m_b[i] * m_b[i] * mask[i];
					}

					dgInt32 index = -1;
					dgFloat32 alpha = num / den;
					dgAssert(alpha > dgFloat32(0.0f));

					for (dgInt32 i = 0; (i < m_size) && (alpha > dgFloat32(0.0f)); i++) {

						if (m_delta_x[i]) {
							dgFloat32 x = m_x[i] + alpha * m_delta_x[i];
							if (x < m_low[i]) {
								index = i;
								alpha = (m_low[i] - m_x[i]) / m_delta_x[i];
							} else if (x > m_high[i]) {
								index = i;
								alpha = (m_high[i] - m_x[i]) / m_delta_x[i];
							}
							dgAssert(alpha >= dgFloat32(-1.0e-4f));
							if (alpha < dgFloat32(1.0e-6f)) {
								alpha = dgFloat32(0.0f);
							}
						}
					}

					beta = dgFloat32(0.0f);
					for (dgInt32 i = 0; i < m_size; i++) {
						m_x[i] += alpha * m_delta_x[i];
						m_b[i] -= alpha * m_delta_r[i];
						beta += m_b[i] * m_b[i] * mask[i];
					}

					if (index >= 0) {
						beta = dgFloat32(0.0f);
						mask[index] = dgFloat32(0.0f);
						for (dgInt32 i = 0; i < m_size; i++) {
							m_delta_x[i] = m_b[i] * mask[i];
							beta += m_b[i] * m_b[i] * mask[i];
							stride += m_size;
						}
					} else {
						alpha = beta / num;
						for (dgInt32 i = 0; i < m_size; i++) {
							m_delta_x[i] = m_b[i] * mask[i] + alpha * m_delta_x[i];
						}
					}
				}
			}
		}

		return accelNorm;
	}

	private:
	dgFloat32 m_x[maxRows + 4];
	dgFloat32 m_b[maxRows];
	dgFloat32 m_low[maxRows];
	dgFloat32 m_high[maxRows];
	dgFloat32 m_invDiag[maxRows];
	dgFloat32 m_delta_x[maxRows];
	dgFloat32 m_delta_r[maxRows];
	dgFloat32 m_matrix[maxRows * maxRows];
	dgInt32 m_frictionIndex[maxRows];
	dgInt32 m_size;
};



template<class T>
void dgMatrixTimeVector(dgInt32 size, const T* const matrix, const T* const v, T* const out)
{
	dgCheckAligment(v);
	dgCheckAligment(out);
	dgCheckAligment(matrix);
	dgInt32 stride = 0;
	for (dgInt32 i = 0; i < size; i++) {
		//dgGetRow(const T* const matrix, dgInt32 size, dgInt32 index)
		const T* const row = &matrix[stride];
		dgCheckAligment(row);
		out[i] = dgDotProduct(size, row, v);
		stride += size;
	}
}

template<class T>
void dgMatrixTimeMatrix(dgInt32 size, const T* const matrixA, const T* const matrixB, T* const out)
{
	dgCheckAligment(out);
	dgCheckAligment(matrixA);
	dgCheckAligment(matrixB);
	for (dgInt32 i = 0; i < size; i++) {
		const T* const rowA = &matrixA[i * size];
		dgCheckAligment(rowA);
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
			if (diag < T(dgFloat32(1.0e-6f))) {
				return false;
			}

			rowN[n] = T(sqrt(diag));
		} else {
			rowN[j] = (rowN[j] - s) / rowJ[j];
		}

		stride += size;
	}
	return true;
}


template<class T>
bool dgCholeskyFactorization(dgInt32 size, T* const psdMatrix)
{
	bool state = true;
	for (dgInt32 i = 0; (i < size) && state; i++) {
		state = state && dgCholeskyFactorizationAddRow(size, i, psdMatrix);
	}
	return state;
}


template<class T>
DG_INLINE void dgSolveCholesky(dgInt32 size, dgInt32 n, const T* const choleskyMatrix, T* const x)
{
	dgInt32 stride = 0;
	for (dgInt32 i = 0; i < n; i++) {
		T acc(0.0f);
		const T* const row = &choleskyMatrix[stride];
		dgCheckAligment(row);
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
		dgInt32 m = i;
		T maxVal (dgAbs(rowI[i]));
		for (dgInt32 j = i + 1; j < size - 1; j++) {
			T val (dgAbs(matrix[size * j + i]));
			if (val > maxVal) {
				m = j;
				maxVal = val;
			}
		}

		if (maxVal < T(1.0e-12f)) {
			return false;
		}

		if (m != i) {
			T* const rowK = &matrix[m * size];
			T* const rowJ = &matrix[i * size];
			for (dgInt32 j = 0; j < size; j++) {
				dgSwap(rowK[j], rowJ[j]);
			}
			dgSwap(b[i], b[m]);
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
				scale += T(dgAbs(rowI[k]));
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
void dgEigenValues(const dgInt32 size, const T* const choleskyMatrix, T* const eigenValues)
{
	T* const offDiag = dgAlloca(T, size);
	T* const matrix = dgAlloca(T, size * size);
	dgCheckAligment(offDiag);
	dgCheckAligment(matrix);

	memcpy(matrix, choleskyMatrix, sizeof(T) * size * size);

	for (dgInt32 i = size - 1; i > 0; i--) {
		T h(0.0f);
		T* const rowI = &matrix[i * size];

		if (i > 1) {
			T scale(0.0f);
			for (dgInt32 k = 0; k < i; k++) {
				scale += dgAbs(rowI[k]);
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
					T f1 (rowI[j]);
					T g1(offDiag[j] - hh * f1);
					offDiag[j] = g1;
					T* const rowJ = &matrix[j * size];
					for (dgInt32 k = 0; k <= j; k++) {
						rowJ[k] -= (f1 * offDiag[k] + g1 * rowI[k]);
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
				T dd(dgAbs(eigenValues[j]) + dgAbs(eigenValues[j + 1]));
				if (dgAbs(offDiag[j]) <= (T(1.e-6f) * dd)) {
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
					T d(dgPythag(f, g));
					offDiag[k + 1] = d;
					if (d == T(0.0f)) {
						eigenValues[k + 1] -= p;
						offDiag[j] = T(0.0f);
						break;
					}
					s = f / d;
					c = g / d;
					g = eigenValues[k + 1] - p;
					d = (eigenValues[k] - g) * s + T(2.0f) * c * b;
					p = s * d;
					eigenValues[k + 1] = g + p;
					g = c * d - b;
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

template <class T>
DG_INLINE void dgGaussSeidelLcpSor(const dgInt32 size, const T* const matrix, T* const x, const T* const b, const T* const low, const T* const high, T tol2, dgInt32 maxIterCount, dgInt16* const clipped, dgFloat32 sor)
{
	const T* const me = matrix;
	T* const invDiag1 = dgAlloca(T, size);
	dgCheckAligment(invDiag1);

	dgInt32 stride = 0;
	for (dgInt32 i = 0; i < size; i++) {
		x[i] = dgClamp(x[i], low[i], high[i]);
		invDiag1[i] = T(1.0f) / me[stride + i];
		stride += size;
	}

	T tolerance(tol2 * 2.0f);
	const T* const invDiag = invDiag1;
#ifdef _DEBUG 
	dgInt32 passes = 0;
#endif
	for (dgInt32 i = 0; (i < maxIterCount) && (tolerance > tol2); i++) {
		dgInt32 base = 0;
		tolerance = T(0.0f);
#ifdef _DEBUG 
		passes++;
#endif
		for (dgInt32 j = 0; j < size; j++) {
			const T* const row = &me[base];
			T r(b[j] - dgDotProduct(size, row, x));
			T f((r + row[j] * x[j]) * invDiag[j]);
			if (f > high[j]) {
				x[j] = high[j];
				clipped[j] = 1;
			} else if (f < low[j]) {
				x[j] = low[j];
				clipped[j] = 1;
			} else {
				clipped[j] = 0;
				tolerance += r * r;
				x[j] = x[j] + (f - x[j]) * sor;
			}
			base += size;
		}
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
void dgGaussSeidelLCP(const dgInt32 size, const T* const matrix, T* const x, const T* const b, const T* const low, const T* const high, T sor = T(1.3f))
{
	dgGaussSeidelLcpSor(size, matrix, x, b, low, high, T(1.0e-5f), size * size, sor);
}

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
DG_INLINE void dgCalculateDelta_x(dgInt32 size, dgInt32 n, const T* const matrix, const T* const choleskyMatrix, T* const delta_x)
{
	const T* const row = &matrix[size * n];
	for (dgInt32 i = 0; i < n; i++) {
		delta_x[i] = -row[i];
	}
	dgSolveCholesky(size, n, choleskyMatrix, delta_x);
	delta_x[n] = T(1.0f);
}

// calculate delta_r = A * delta_x
template<class T>
DG_INLINE void dgCalculateDelta_r(dgInt32 size, dgInt32 n, const T* const matrix, const T* const delta_x, T* const delta_r)
{
	dgInt32 stride = n * size;
	const dgInt32 size1 = n + 1;
	for (dgInt32 i = n; i < size; i++) {
		delta_r[i] = dgDotProduct(size1, &matrix[stride], delta_x);
		stride += size;
	}
}

/*
template<class T>
DG_INLINE void dgCholeskyUpdateGivensColums(dgInt32 size, dgInt32 row, dgInt32 colum, T* const choleskyMatrix)
{
	dgFloat64 a01 = choleskyMatrix[size * row + colum];
	choleskyMatrix[size * row + colum] = T(0.0f);
	if (dgAbs(a01) > dgFloat32(1.0e-14f)) {
		dgFloat64 a00 = choleskyMatrix[size * row + row];

		dgFloat64 den = a00 * a00 + a01 * a01;
		dgAssert(den > dgFloat64(0.0f));
		den = dgFloat64(1.0f) / sqrt(den);

		dgFloat64 b0 = a00 * den;
		dgFloat64 b1 = a01 * den;
		dgAssert((a01 * b0 - a00 * b1) < dgFloat64(1.0e-8f));
		choleskyMatrix[size * row + row] = T(a00 * b0 + a01 * b1);

		for (dgInt32 j = row + 1; j < size; j++) {
			a00 = choleskyMatrix[size * j + row];
			a01 = choleskyMatrix[size * j + colum];
			choleskyMatrix[size * j + row] = T(a00 * b0 + a01 * b1);
			choleskyMatrix[size * j + colum] = T(a00 * b1 - a01 * b0);
		}

		if (choleskyMatrix[size * row + row] < T(0.0f)) {
			dgAssert (0);
			for (dgInt32 j = row + 1; j < size; j++) {
				choleskyMatrix[size * j + row] *= T(-1.0f);
			}
		}

		if (choleskyMatrix[size * colum + colum] < T(0.0f)) {
			for (dgInt32 j = row + 1; j < size; j++) {
				choleskyMatrix[size * j + colum] *= T(-1.0f);
			}
		}
	}
}

template<class T>
DG_INLINE void dgCholeskyUpdateGivensRows(dgInt32 size, dgInt32 row, dgInt32 colum, T* const choleskyMatrix)
{
	for (dgInt32 i = row; i < colum; i++) {
		dgCholeskyUpdateGivensColums(size, i, colum, choleskyMatrix);
	}
}

// Givens rotations should be much faster for a single row update, but is far more unstable
template<class T>
DG_INLINE void dgCholeskyUpdateGivens(dgInt32 size, dgInt32 row, dgInt32 colum, T* const choleskyMatrix)
{
	for (dgInt32 i = colum; i > row; i--) {
		dgCholeskyUpdateGivensRows(size, row, i, choleskyMatrix);
	}
}
*/

/*
template<class T>
DG_INLINE void dgCholeskyUpdateHouseholder___(dgInt32 size, dgInt32 row, dgInt32 colum, T* const choleskyMatrix, T* const tmp, T* const reflexion, dgInt16* const activeColumns)
{
	if (row != colum) {
		dgAssert(row < colum);
		for (dgInt32 i = row; i < size; i++) {
			T* const rowI = &choleskyMatrix[size * i];
			activeColumns[0] = dgInt16(i);
			dgInt32 width = 1;
			for (dgInt32 j = i + 1; j < size; j++) {
				activeColumns[width] = dgInt16(j);
				width += dgAbs(rowI[j]) > dgFloat32(1.0e-14f) ? 1 : 0;
			}

			if (width > 1) {
				T mag2(dgFloat32(0.0f));
				for (dgInt32 j = 1; j < width; j++) {
					dgInt32 index = activeColumns[j];
					mag2 += rowI[index] * rowI[index];
					reflexion[index] = rowI[index];
				}
				reflexion[i] = rowI[i] - T(sqrt(mag2 + rowI[i] * rowI[i]));

				const T vMag2(mag2 + reflexion[i] * reflexion[i]);
				const T den(dgFloat32(2.0f) / vMag2);
				for (dgInt32 j = i; j < size; j++) {
					T acc(0.0f);
					T* const rowJ = &choleskyMatrix[size * j];
					for (dgInt32 k = 0; k < width; k++) {
						dgInt32 index = activeColumns[k];
						acc += rowJ[index] * reflexion[index];
					}
					tmp[j] = acc;
				}

				for (dgInt32 j = i + 1; j < size; j++) {
					T* const rowJ = &choleskyMatrix[size * j];
					const T a = tmp[j] * den;
					for (dgInt32 k = 0; k < width; k++) {
						dgInt32 index = activeColumns[k];
						rowJ[index] -= a * reflexion[index];
					}
				}
				rowI[i] -= tmp[i] * reflexion[i] * den;
			}

			for (dgInt32 k = i + 1; k < size; k++) {
				rowI[k] = T(0.0f);
			}

			if (rowI[i] < T(dgFloat32(0.0f))) {
				for (dgInt32 k = i; k < size; k++) {
					choleskyMatrix[size * k + i] = -choleskyMatrix[size * k + i];
				}
			}
		}
		for (dgInt32 i = row; i < size; i++) {
			choleskyMatrix[size * i + i] = dgMax(choleskyMatrix[size * i + i], T(dgFloat32(1.0e-6f)));
		}
	}
}
*/

template<class T>
DG_INLINE void dgCholeskyUpdateHouseholder(dgInt32 size, dgInt32 row, dgInt32 colum, T* const choleskyMatrix, T* const tmp, T* const reflexion)
{
//	T xxxx[6][6];
//	memcpy(xxxx, choleskyMatrix, sizeof(T) * size * size);
//	dgCholeskyUpdateHouseholder___(size, row, colum, &xxxx[0][0], tmp, reflexion, activeColumns___);

	dgAssert(row <= colum);
	for (dgInt32 i = row; i < colum; i++) {
		T* const rowI = &choleskyMatrix[size * i];
		T mag2(dgFloat32(0.0f));
		for (dgInt32 j = i + 1; j <= colum; j++) {
			mag2 += rowI[j] * rowI[j];
			reflexion[j] = rowI[j];
		}
		if (mag2 > T (1.0e-14f)) {
			reflexion[i] = rowI[i] - T(sqrt(mag2 + rowI[i] * rowI[i]));

			const T vMag2(mag2 + reflexion[i] * reflexion[i]);
			const T den(dgFloat32(2.0f) / vMag2);
			for (dgInt32 j = i; j <= colum; j++) {
				T acc(0.0f);
				T* const rowJ = &choleskyMatrix[size * j];
				for (dgInt32 k = i; k <= colum; k++) {
					acc += rowJ[k] * reflexion[k];
				}
				tmp[j] = acc;
			}

			for (dgInt32 j = i + 1; j < size; j++) {
				rowI[j] = T(0.0f);
				T* const rowJ = &choleskyMatrix[size * j];
				const T a = tmp[j] * den;
				for (dgInt32 k = i; k <= colum; k++) {
					rowJ[k] -= a * reflexion[k];
				}
			}
			rowI[i] -= tmp[i] * reflexion[i] * den;

			if (rowI[i] < T(dgFloat32(0.0f))) {
				for (dgInt32 k = i; k <= colum; k++) {
					choleskyMatrix[size * k + i] = -choleskyMatrix[size * k + i];
				}
			}
		}
	}

	if (row > colum) {
		for (dgInt32 i = row; i < size; i++) {
			choleskyMatrix[size * i + i] = dgMax(choleskyMatrix[size * i + i], T(dgFloat32(1.0e-6f)));
		}
	}
}

template<class T>
DG_INLINE void dgCholeskyUpdate(dgInt32 size, dgInt32 row, dgInt32 colum, T* const choleskyMatrix, T* const tmp, T* const reflexion)
{
#if 0
	// I have no idea why given rotation do not work.
	dgCholeskyUpdateGivens(size, row, colum, choleskyMatrix);
#else

	dgInt32 n = colum - row;
	dgInt32 n1 = n + 1;
	dgInt32 choleskyCost = (size * size * size - row * row * row) / 3;
	dgInt32 householdCost = n * (n + 1) / 2 + n1 * (n1 + 1) * (2 * (2 * n1 + 1) - 3 + 3 * (size - colum - 1)) / 6 - 1;
	if (householdCost < choleskyCost) {
		dgCholeskyUpdateHouseholder(size, row, colum, choleskyMatrix, tmp, reflexion);
	} else {
		dgCholeskyUpdateHouseholder(size, row, colum, choleskyMatrix, tmp, reflexion);
	}
#endif

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
//DG_INLINE void dgSolveDantzigLcpLow(dgInt32 size, T* const symmetricMatrixPSD, T* const lowerTriangularMatrix, T* const x, T* const b, T* const low, T* const high)
void dgSolveDantzigLcpLow(dgInt32 size, T* const symmetricMatrixPSD, T* const lowerTriangularMatrix, T* const x, T* const b, T* const low, T* const high)
{	
	T* const x0 = dgAlloca(T, size);
	T* const r0 = dgAlloca(T, size);
	T* const tmp0 = dgAlloca(T, size);
	T* const tmp1 = dgAlloca(T, size);
	T* const delta_r = dgAlloca(T, size);
	T* const delta_x = dgAlloca(T, size);
	dgInt16* const permute = dgAlloca(dgInt16, size);
	//dgInt16* const updateIndex = dgAlloca(dgInt16, size);

	dgCheckAligment(x);
	dgCheckAligment(b);
	dgCheckAligment(x0);
	dgCheckAligment(r0);
	dgCheckAligment(low);
	dgCheckAligment(high);
	dgCheckAligment(tmp0);
	dgCheckAligment(tmp1);
	dgCheckAligment(delta_r);
	dgCheckAligment(delta_x);
	dgCheckAligment(permute);
	dgCheckAligment(symmetricMatrixPSD);
	dgCheckAligment(lowerTriangularMatrix);

#if 0
	for (dgInt32 i = 0; i < size; i++) {
		permute[i] = dgInt16(i);
		r0[i] = b[i];
		x0[i] = dgFloat32(0.0f);
		delta_x[i] = r0[i];
		delta_r[i] = r0[i];
	}

	dgInt32 initialGuessCount = size;
	for (T error2 = T(1.0f); initialGuessCount && (error2 > T(1.0e-12f));) {
		dgSolveCholesky(size, initialGuessCount, lowerTriangularMatrix, delta_x);

		T alpha = T(1.0f);
		dgInt32 index = -1;
		for (dgInt32 i = 0; i < initialGuessCount; i++) {
			T x1 = x0[i] + alpha * delta_x[i];
			if (x1 < low[i]) {
				index = i;
				alpha = (low[i] - x0[i]) / delta_x[i];
			} else if (x1 > high[i]) {
				index = i;
				alpha = (high[i] - x0[i]) / delta_x[i];
			}
		}

		error2 = T(0.0f);
		for (dgInt32 i = 0; i < initialGuessCount; i++) {
			x0[i] += alpha * delta_x[i];
			r0[i] -= alpha * delta_r[i];
			delta_x[i] = r0[i];
			delta_r[i] = r0[i];
			error2 = r0[i] * r0[i];
		}

		if (index != -1) {
			initialGuessCount--;
			delta_x[index] = T(0.0f);
			dgSwap(delta_x[index], delta_x[initialGuessCount]);
			dgSwap(delta_r[index], delta_r[initialGuessCount]);
			dgPermuteRows(size, index, initialGuessCount, symmetricMatrixPSD, lowerTriangularMatrix, x0, r0, low, high, permute);
			dgCholeskyUpdate(size, index, initialGuessCount, lowerTriangularMatrix, tmp0, tmp1);
		}
	}

	if (initialGuessCount == size) {
		for (dgInt32 i = 0; i < size; i++) {
			x[i] = x0[i];
			b[i] = T(0.0f);
		}
		return;
	}

	for (dgInt32 i = 0; i < size; i++) {
		r0[i] = dgFloat32(0.0f);
		delta_x[i] = dgFloat32(0.0f);
		delta_r[i] = dgFloat32(0.0f);
	}

	dgInt32 clampedIndex = size;
	dgInt32 index = initialGuessCount;
	dgInt32 count = size - initialGuessCount;

	dgInt32 stride = index * size;
	for (dgInt32 i = index; i < size; i++) {
		dgInt32 j = permute[i];
		r0[i] = dgDotProduct(size, &symmetricMatrixPSD[stride], x0) - b[j];
		stride += size;
	}
#else


	for (dgInt32 i = 0; i < size; i++) {
		permute[i] = dgInt16(i);
		r0[i] = b[i];
		x0[i] = dgFloat32(0.0f);
		delta_x[i] = dgMax (dgAbs(b[i]), T (1.0f));
	}

	for (dgInt32 i = 0; i < size; i++) {
		dgInt32 min = i;
		for (dgInt32 j = i + 1; j < size; j++) {
			if (delta_x[j] < delta_x[min]) {
				min = j;
			}
		}
		if (min != i) {
			dgSwap(delta_x[i], delta_x[min]);
			dgPermuteRows(size, i, min, symmetricMatrixPSD, lowerTriangularMatrix, x0, r0, low, high, permute);
		}
	}

/*
	for (dgInt32 i = 0; i < size; i++) {
		permute[i] = dgInt16(i);
		r0[i] = b[i];
		x0[i] = dgFloat32(0.0f);
		delta_x[i] = r0[i];
		delta_r[i] = r0[i];
	}

	dgInt32 initialGuessCount = size;
	for (T error2 = T(1.0f); initialGuessCount && (error2 > T(1.0e-12f));) {
		dgSolveCholesky(size, initialGuessCount, lowerTriangularMatrix, delta_x);

		T alpha = T(1.0f);
		dgInt32 index = -1;
		for (dgInt32 i = 0; i < initialGuessCount; i++) {
			T x1 = x0[i] + alpha * delta_x[i];
			if (x1 < low[i]) {
				index = i;
				alpha = (low[i] - x0[i]) / delta_x[i];
			}
			else if (x1 > high[i]) {
				index = i;
				alpha = (high[i] - x0[i]) / delta_x[i];
			}
		}

		error2 = T(0.0f);
		for (dgInt32 i = 0; i < initialGuessCount; i++) {
			x0[i] += alpha * delta_x[i];
			r0[i] -= alpha * delta_r[i];
			delta_x[i] = r0[i];
			delta_r[i] = r0[i];
			error2 = r0[i] * r0[i];
		}

		if (index != -1) {
			initialGuessCount--;
			delta_x[index] = T(0.0f);
			dgSwap(delta_x[index], delta_x[initialGuessCount]);
			dgSwap(delta_r[index], delta_r[initialGuessCount]);
			dgPermuteRows(size, index, initialGuessCount, symmetricMatrixPSD, lowerTriangularMatrix, x0, r0, low, high, permute);
			dgCholeskyUpdate(size, index, initialGuessCount, lowerTriangularMatrix, tmp0, tmp1);
		}
	}

	if (initialGuessCount == size) {
		for (dgInt32 i = 0; i < size; i++) {
			x[i] = x0[i];
			b[i] = T(0.0f);
		}
		return;
	}

	for (dgInt32 i = 0; i < size; i++) {
		r0[i] = dgFloat32(0.0f);
		delta_x[i] = dgFloat32(0.0f);
		delta_r[i] = dgFloat32(0.0f);
	}

	dgInt32 clampedIndex = size;
	dgInt32 index = initialGuessCount;
	dgInt32 count = size - initialGuessCount;

	dgInt32 stride = index * size;
	for (dgInt32 i = index; i < size; i++) {
		dgInt32 j = permute[i];
		r0[i] = dgDotProduct(size, &symmetricMatrixPSD[stride], x0) - b[j];
		stride += size;
	}
*/
#endif

/*
	while (count) {
		bool loop = true;

		while (loop) {
			loop = false;
			T clamp_x(0.0f);
			dgInt32 swapIndex = -1;

			if (dgAbs(r0[index]) > T(1.0e-12f)) {
				dgCalculateDelta_x(size, index, symmetricMatrixPSD, lowerTriangularMatrix, delta_x);
				dgCalculateDelta_r(size, index, symmetricMatrixPSD, delta_x, delta_r);

				dgAssert(delta_r[index] != T(0.0f));
				dgAssert(dgAbs(delta_x[index]) == T(1.0f));
				delta_r[index] = (delta_r[index] == T(dgFloat32 (0.0f))) ? T(dgFloat32 (1.0e-12f)) : delta_r[index];

				T s = -r0[index] / delta_r[index];
				dgAssert(dgAbs(s) >= T(0.0f));

				for (dgInt32 i = 0; i <= index; i++) {
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
				dgAssert(dgAbs(s) >= T(0.0f));

				for (dgInt32 i = clampedIndex; (i < size) && (s > T(1.0e-12f)); i++) {
					T r1 = r0[i] + s * delta_r[i];
					if ((r1 * r0[i]) < T(0.0f)) {
						dgAssert(dgAbs(delta_r[i]) > T(0.0f));
						T s1 = -r0[i] / delta_r[i];
						dgAssert(dgAbs(s1) >= T(0.0f));
						dgAssert(dgAbs(s1) <= dgAbs(s));
						if (dgAbs(s1) < dgAbs(s)) {
							s = s1;
							swapIndex = i;
						}
					}
				}

				for (dgInt32 i = 0; i < size; i++) {
					x0[i] += s * delta_x[i];
					r0[i] += s * delta_r[i];
				}
			}

			if (swapIndex == -1) {
				r0[index] = T(0.0f);
				delta_r[index] = T(0.0f);
				index++;
				count--;
				loop = false;
			} else if (swapIndex == index) {
				count--;
				clampedIndex--;
				x0[index] = clamp_x;
				dgPermuteRows(size, index, clampedIndex, symmetricMatrixPSD, lowerTriangularMatrix, x0, r0, low, high, permute);
				dgCholeskyUpdate(size, index, clampedIndex, lowerTriangularMatrix, tmp0, tmp1);
				loop = count ? true : false;
			} else if (swapIndex > index) {
				loop = true;
				r0[swapIndex] = T(0.0f);
				dgAssert(swapIndex < size);
				dgAssert(clampedIndex <= size);
				if (swapIndex < clampedIndex) {
					count--;
					clampedIndex--;
					dgPermuteRows(size, clampedIndex, swapIndex, symmetricMatrixPSD, lowerTriangularMatrix, x0, r0, low, high, permute);
					dgCholeskyUpdate(size, swapIndex, clampedIndex, lowerTriangularMatrix, tmp0, tmp1);
					dgAssert(clampedIndex >= index);
				} else {
					count++;
					dgAssert(clampedIndex < size);
					dgPermuteRows(size, clampedIndex, swapIndex, symmetricMatrixPSD, lowerTriangularMatrix, x0, r0, low, high, permute);
					dgCholeskyUpdate(size, clampedIndex, swapIndex, lowerTriangularMatrix, tmp0, tmp1);
					clampedIndex++;
					dgAssert(clampedIndex <= size);
					dgAssert(clampedIndex >= index);
				}

			} else {
				dgAssert(index > 0);
				x0[swapIndex] = clamp_x;
				delta_x[index] = T(0.0f);

				dgAssert(swapIndex < index);
				dgPermuteRows(size, swapIndex, index - 1, symmetricMatrixPSD, lowerTriangularMatrix, x0, r0, low, high, permute);
				dgPermuteRows(size, index - 1, index, symmetricMatrixPSD, lowerTriangularMatrix, x0, r0, low, high, permute);
				dgPermuteRows(size, clampedIndex - 1, index, symmetricMatrixPSD, lowerTriangularMatrix, x0, r0, low, high, permute);
				dgCholeskyUpdate (size, swapIndex, clampedIndex - 1, lowerTriangularMatrix, tmp0, tmp1);

				clampedIndex--;
				index--;
				loop = true;
			}
		}
	}

	for (dgInt32 i = 0; i < size; i++) {
		dgInt32 j = permute[i];
		x[j] = x0[i];
		b[j] = r0[i];
	}
*/
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
bool dgSolveDantzigLCP(dgInt32 size, T* const symetricMatrix, T* const x, T* const b, T* const low, T* const high)
{
	T* const choleskyMatrix = dgAlloca(T, size * size);
	dgCheckAligment(choleskyMatrix);

	memcpy (choleskyMatrix, symetricMatrix, sizeof (T) * size * size);
	dgCholeskyFactorization(size, choleskyMatrix);
	for (dgInt32 i = 0; i < size; i ++) {
		T* const row = &choleskyMatrix[i * size];
		for (dgInt32 j = i + 1; j < size; j ++) {
			row[j] = T(dgFloat32 (0.0f));
		}
	}
	return dgSolveDantzigLCP(size, symetricMatrix, choleskyMatrix, x, b, low, high);
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
// b is zero
// note: although the system is called LCP, the solver is far more general than a strict LCP
// to solve a strict LCP, set the following
// low(i) = 0
// high(i) = infinity.
// this is the same as enforcing the constraint: x(i) * r(i) = 0
template <class T>
bool dgSolvePartitionDantzigLCP(dgInt32 size, T* const symmetricMatrixPSD , T* const x, T* const b, T* const low, T* const high)
{
	dgInt16* const permute = dgAlloca(dgInt16, size);
	dgCheckAligment(permute);

	for (dgInt32 i = 0; i < size; i++) {
		x[i] = b[i];
		permute[i] = dgInt16(i);
	}

	dgInt32 unboundedSize = size;
	for (dgInt32 i = 0; i < unboundedSize; i++) {
		if ((low[i] <= T(-DG_LCP_MAX_VALUE)) && (high[i] >= T(DG_LCP_MAX_VALUE))) {
			dgCholeskyFactorizationAddRow(size, i, symmetricMatrixPSD );
		} else {
			dgInt32 j = unboundedSize - 1;
			if (i != j) {
				T* const A = &symmetricMatrixPSD [size * i];
				T* const B = &symmetricMatrixPSD [size * j];
				for (dgInt32 k = 0; k < size; k++) {
					dgSwap(A[k], B[k]);
				}

				dgInt32 stride = 0;
				for (dgInt32 k = 0; k < size; k++) {
					dgSwap(symmetricMatrixPSD [stride + i], symmetricMatrixPSD [stride + j]);
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
		dgSolveCholesky(size, unboundedSize, symmetricMatrixPSD , x);
		dgInt32 base = unboundedSize * size;
		for (dgInt32 i = unboundedSize; i < size; i++) {
			b[i] = dgDotProduct(unboundedSize, &symmetricMatrixPSD[base], x) - b[i];
			base += size;
		}

		const dgInt32 boundedSize = size - unboundedSize;
		T* const l = dgAlloca(T, boundedSize);
		T* const h = dgAlloca(T, boundedSize);
		T* const c = dgAlloca(T, boundedSize);
		T* const u = dgAlloca(T, boundedSize);
		T* const a11 = dgAlloca(T, boundedSize * boundedSize);
		T* const a10 = dgAlloca(T, boundedSize * unboundedSize);

		dgCheckAligment(l);
		dgCheckAligment(h);
		dgCheckAligment(c);
		dgCheckAligment(u);
		dgCheckAligment(a10);
		dgCheckAligment(a11);

		for (dgInt32 i = 0; i < boundedSize; i++) {
			T* const g = &a10[i * unboundedSize];
			const T* const row = &symmetricMatrixPSD [(unboundedSize + i) * size];
			for (dgInt32 j = 0; j < unboundedSize; j++) {
				g[j] = -row[j];
			}
			dgSolveCholesky(size, unboundedSize, symmetricMatrixPSD, g);

			T* const arow = &a11[i * boundedSize];
			const T* const row2 = &symmetricMatrixPSD[(unboundedSize + i) * size];
			arow[i] = row2[unboundedSize + i] + dgDotProduct(unboundedSize, g, row2);
			for (dgInt32 j = i + 1; j < boundedSize; j++) {
				const T* const row1 = &symmetricMatrixPSD [(unboundedSize + j) * size];
				T elem = row1[unboundedSize + i] + dgDotProduct(unboundedSize, g, row1);
				arow[j] = elem;
				a11[j * boundedSize + i] = elem;
			}
			u[i] = T(0.0f);
			c[i] = -b[i + unboundedSize];
			l[i] = low[i + unboundedSize];
			h[i] = high[i + unboundedSize];
		}

		if (dgSolveDantzigLCP(boundedSize, a11, u, c, l, h)) {
			for (dgInt32 i = 0; i < boundedSize; i++) {
				const T s = u[i];
				x[unboundedSize + i] = s;
				const T* const g = &a10[i * unboundedSize];
				for (dgInt32 j = 0; j < unboundedSize; j++) {
					x[j] += g[j] * s;
				}
			}
			ret = true;
		}
	} else {
		for (dgInt32 i = 0; i < size; i++) {
			x[i] = T(0.0f);
		}
		ret = dgSolveDantzigLCP(size, symmetricMatrixPSD, x, b, low, high);
	}

	for (dgInt32 i = 0; i < size; i++) {
		b[i] = x[i];
	}
	for (dgInt32 i = 0; i < size; i++) {
		dgInt32 j = permute[i];
		x[j] = b[i];
		b[i] = T(0.0f);
	}
	return ret;
}


template <class T>
void dgSolveDantzigLCP(dgInt32 size, T* const symmetricMatrixPSD, T* const lowerTriangularMatrix, T* const x, T* const b, T* const low, T* const high)
{
/*
//	if (size < 64) {
	if (1) {
		dgSolveDantzigLcpLow(size, symmetricMatrixPSD, lowerTriangularMatrix, x, b, low, high);
	} else {
		// larger size lcp required a better initial guess
		T error2 = T(0.25f);
		dgGaussSeidelLcpSor(size, symmetricMatrixPSD, x, b, low, high, error2, size, T(1.3f));

		if (accel2 > error2) {
			dgGaussSeidelLcpSor(size, symmetricMatrixPSD, x, b, low, high, error2, dgMax(20, size), T(1.3f), updateIndex);
			return;
		} 

		dgInt16* const permute = dgAlloca(dgInt16, size);
		for (dgInt32 i = 0; i < size; i++) {
			permute[i] = dgInt16(i);
		}

		dgInt32 bilareralCount = 0;
		for (dgInt32 i = 0; i < size; i++) {
			if (!updateIndex[i]) {
				dgSwap(updateIndex[i], updateIndex[bilareralCount]);
				dgPermuteRows(size, i, bilareralCount, symmetricMatrixPSD, lowerTriangularMatrix, x, b, low, high, permute);
				bilareralCount ++;
			}
		}
		if (bilareralCount != size) {
			memcpy (lowerTriangularMatrix, symmetricMatrixPSD, sizeof (T) * size * size);
			dgCholeskyFactorization(size, lowerTriangularMatrix);
		}

		dgSolveDantzigLcpLow(size, symmetricMatrixPSD, lowerTriangularMatrix, x, b, low, high, bilareralCount);
		if (bilareralCount != size) {
			for (dgInt32 i = 0; i < size; i++) {
				low[i] = x[i];
				high[i] = b[i];
			}
			for (dgInt32 i = 0; i < size; i++) {
				dgInt32 j = permute[i];
				x[j] = low[i];
				b[j] = high[i];
			}
		}
	}
*/

	
	T tol2 = T(0.25f * 0.25f);
	dgInt32 passes = dgClamp(size, 12, 20);
	T* const r = dgAlloca(T, size);
	dgInt16* const clipped = dgAlloca(dgInt16, size);
	dgGaussSeidelLcpSor(size, symmetricMatrixPSD, x, b, low, high, tol2, passes, clipped, T(1.3f));

	T err2(0.0f);
	dgInt32 stride = 0;
	dgInt32 clippeCount = 0;
	for (dgInt32 i = 0; i < size; i++) {
		const T* const row = &symmetricMatrixPSD[stride];
		r[i] = b[i] - dgDotProduct(size, row, x);
		clippeCount += clipped[i];
		err2 += (1 - clipped[i]) * r[i] * r[i];
		stride += size;
	}

	if (err2 > tol2) {
		if ((clippeCount > 64) && (err2 > T(4.0f))) {
			dgGaussSeidelLcpSor(size, symmetricMatrixPSD, x, b, low, high, tol2, 20, clipped, T(1.3f));
		} else {
			T* const x0 = dgAlloca(T, size);
			for (dgInt32 i = 0; i < size; i++) {
				x0[i] = x[i];
				low[i] -= x[i];
				high[i] -= x[i];
			}
			//dgSolveDantzigLcpLow(size, symmetricMatrixPSD, lowerTriangularMatrix, x0, r, low, high);

			dgGaussSeidelLcpSor(size, symmetricMatrixPSD, x, b, low, high, tol2, 20, clipped, T(1.3f));
		}
	}
}


#endif
