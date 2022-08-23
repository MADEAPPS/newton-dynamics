
/* Copyright (c) <2003-2022> <Julio Jerez, Newton Game Dynamics>
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

#ifndef __ND_GENERAL_MATRIX_H__
#define __ND_GENERAL_MATRIX_H__

#include "ndCoreStdafx.h"
#include "ndTypes.h"
#include "ndUtils.h"
#include "ndGeneralVector.h"

#define D_LCP_MAX_VALUE ndFloat32 (1.0e15f)

//*************************************************************
//
// generic linear algebra functions
//
//*************************************************************
template<class T>
void ndMatrixTimeVector(ndInt32 size, const T* const matrix, const T* const v, T* const out)
{
	ndInt32 stride = 0;
	for (ndInt32 i = 0; i < size; ++i) 
	{
		const T* const row = &matrix[stride];
		out[i] = ndDotProduct(size, row, v);
		stride += size;
	}
}

template<class T>
void ndMatrixTimeMatrix(ndInt32 size, const T* const matrixA, const T* const matrixB, T* const out)
{
	for (ndInt32 i = 0; i < size; ++i) 
	{
		const T* const rowA = &matrixA[i * size];
		T* const rowOut = &out[i * size];
		for (ndInt32 j = 0; j < size; ++j) 
		{
			T acc = T(0.0f);
			for (ndInt32 k = 0; k < size; ++k) 
			{
				acc += rowA[k] * matrixB[k * size + j];
			}
			rowOut[j] = acc;
		}
	}
}

template<class T>
void ndCovarianceMatrix(ndInt32 size, T* const matrix, const T* const vectorA, const T* const vectorB)
{
	ndInt32 stride = 0;
	for (ndInt32 i = 0; i < size; ++i) 
	{
		T scale(vectorA[i]);
		T* const row = &matrix[stride];
		for (ndInt32 j = 0; j < size; ++j) 
		{
			row[j] = scale * vectorB[j];
		}
		stride += size;
	}
}

template<class T>
bool ndCholeskyFactorizationAddRow(ndInt32, ndInt32 stride, ndInt32 n, T* const matrix, T* const invDiagonalOut)
{
	T* const rowN = &matrix[stride * n];

	ndInt32 base = 0;
	for (ndInt32 j = 0; j <= n; ++j) 
	{
		T s(0.0f);
		T* const rowJ = &matrix[base];
		for (ndInt32 k = 0; k < j; ++k) 
		{
			s += rowN[k] * rowJ[k];
		}

		if (n == j) 
		{
			T diag = rowN[n] - s;
			if (diag < T(1.0e-6f)) 
			{
				return false;
			}

			rowN[n] = T(sqrt(diag));
			invDiagonalOut[n] = T(1.0f) / rowN[n];
		} 
		else 
		{
			rowJ[n] = T(0.0f);
			//rowN[j] = (rowN[j] - s) / rowJ[j];
			rowN[j] = invDiagonalOut[j] * (rowN[j] - s);
		}

		base += stride;
	}

	return true;
}

template<class T>
bool ndCholeskyFactorization(ndInt32 size, ndInt32 stride, T* const psdMatrix)
{
	bool state = true;
	T* const invDiagonal = ndAlloca(T, size);
	for (ndInt32 i = 0; (i < size) && state; ++i) 
	{
		state = state && ndCholeskyFactorizationAddRow(size, stride, i, psdMatrix, invDiagonal);
	}
	return state;
}

template<class T>
bool ndTestPSDmatrix(ndInt32 size, ndInt32 stride, T* const matrix)
{
	T* const copy = ndAlloca(T, size * size);
	ndInt32 row = 0;
	for (ndInt32 i = 0; i < size; ++i) 
	{
		memcpy(&copy[i * size], &matrix[row], size * sizeof (T));
		row += stride;
	}
	return ndCholeskyFactorization(size, size, copy);
}

template<class T>
void ndCholeskyApplyRegularizer (ndInt32 size, ndInt32 stride, T* const psdMatrix, T* const regularizer)
{
	bool isPsdMatrix = false;
	ndFloat32* const lowerTriangule = ndAlloca(ndFloat32, stride * stride);
	do 
	{
		memcpy(lowerTriangule, psdMatrix, sizeof(ndFloat32) * stride * stride);
		isPsdMatrix = ndCholeskyFactorization(size, stride, lowerTriangule);
		if (!isPsdMatrix) 
		{
			for (ndInt32 i = 0; i < size; ++i) 
			{
				regularizer[i] *= ndFloat32(4.0f);
				psdMatrix[i * stride + i] += regularizer[i];
			}
		}
	} while (!isPsdMatrix);
}

template<class T>
void ndSolveCholesky(ndInt32 size, ndInt32 stride, const T* const choleskyMatrix, T* const x, const T* const b)
{
	ndInt32 rowStart = 0;
	for (ndInt32 i = 0; i < size; ++i) 
	{
		T acc(0.0f);
		const T* const row = &choleskyMatrix[rowStart];
		for (ndInt32 j = 0; j < i; ++j) 
		{
			acc = acc + row[j] * x[j];
		}
		x[i] = (b[i] - acc) / row[i];
		rowStart += stride;
	}

	for (ndInt32 i = size - 1; i >= 0; i--) 
	{
		T acc = 0.0f;
		for (ndInt32 j = i + 1; j < size; ++j) 
		{
			acc = acc + choleskyMatrix[stride * j + i] * x[j];
		}
		x[i] = (x[i] - acc) / choleskyMatrix[stride * i + i];
	}
}

template<class T>
void ndSolveCholesky(ndInt32 size, T* const choleskyMatrix, T* const x)
{
	ndSolveCholesky(size, size, choleskyMatrix, x);
}

template<class T>
bool ndSolveGaussian(ndInt32 size, T* const matrix, T* const b)
{
	for (ndInt32 i = 0; i < size - 1; ++i) 
	{
		const T* const rowI = &matrix[i * size];
		ndInt32 m = i;
		T maxVal (ndAbs(rowI[i]));
		for (ndInt32 j = i + 1; j < size - 1; ++j) 
		{
			T val (ndAbs(matrix[size * j + i]));
			if (val > maxVal) 
			{
				m = j;
				maxVal = val;
			}
		}

		if (maxVal < T(1.0e-12f)) 
		{
			return false;
		}

		if (m != i) 
		{
			T* const rowK = &matrix[m * size];
			T* const rowJ = &matrix[i * size];
			for (ndInt32 j = 0; j < size; ++j) 
			{
				ndSwap(rowK[j], rowJ[j]);
			}
			ndSwap(b[i], b[m]);
		}

		T den = T(1.0f) / rowI[i];
		for (ndInt32 k = i + 1; k < size; ++k) 
		{
			T* const rowK = &matrix[size * k];
			T factor(-rowK[i] * den);
			for (ndInt32 j = i + 1; j < size; ++j) 
			{
				rowK[j] += rowI[j] * factor;
			}
			rowK[i] = T(0.0f);
			b[k] += b[i] * factor;
		}
	}

	for (ndInt32 i = size - 1; i >= 0; i--) 
	{
		T acc(0);
		T* const rowI = &matrix[i * size];
		for (ndInt32 j = i + 1; j < size; ++j) 
		{
			acc = acc + rowI[j] * b[j];
		}
		b[i] = (b[i] - acc) / rowI[i];
	}
	return true;
}

template <class T>
void ndEigenValues(const ndInt32 size, const ndInt32 stride, const T* const symmetricMatrix, T* const eigenValues)
{
	T* const offDiag = ndAlloca(T, size);
	T* const matrix = ndAlloca(T, size * stride);

	memcpy(matrix, symmetricMatrix, sizeof(T) * size * stride);
	for (ndInt32 i = size - 1; i > 0; i--) 
	{
		T h(0.0f);
		T* const rowI = &matrix[i * stride];

		if (i > 1) 
		{
			T scale(0.0f);
			for (ndInt32 k = 0; k < i; ++k) 
			{
				scale += ndAbs(rowI[k]);
			}

			if (scale == T(0.0f)) 
			{
				offDiag[i] = rowI[i - 1];
			} 
			else 
			{
				for (ndInt32 k = 0; k < i; ++k) 
				{
					rowI[k] /= scale;
					h += rowI[k] * rowI[k];
				}

				T f(rowI[i - 1]);
				T g((f >= T(0.0f) ? -T(sqrt(h)) : T(sqrt(h))));
				offDiag[i] = scale * g;
				h -= f * g;
				rowI[i - 1] = f - g;
				f = T(0.0f);

				for (ndInt32 j = 0; j < i; ++j) 
				{
					g = T(0.0f);
					const T* const rowJ = &matrix[j * stride];
					for (ndInt32 k = 0; k <= j; ++k) 
					{
						g += rowJ[k] * rowI[k];
					}
					for (ndInt32 k = j + 1; k < i; ++k) 
					{
						g += matrix[k * stride + j] * rowI[k];
					}
					offDiag[j] = g / h;
					f += offDiag[j] * rowI[j];
				}

				T hh(f / (h + h));
				for (ndInt32 j = 0; j < i; ++j) 
				{
					T f1 (rowI[j]);
					T g1(offDiag[j] - hh * f1);
					offDiag[j] = g1;
					T* const rowJ = &matrix[j * stride];
					for (ndInt32 k = 0; k <= j; ++k) 
					{
						rowJ[k] -= (f1 * offDiag[k] + g1 * rowI[k]);
					}
				}
			}
		} 
		else 
		{
			offDiag[i] = rowI[i - 1];
		}
		eigenValues[i] = h;
	}

	ndInt32 index = stride;
	eigenValues[0] = matrix[0];
	for (ndInt32 i = 1; i < size; ++i) 
	{
		eigenValues[i] = matrix[index + i];
		offDiag[i - 1] = offDiag[i];
		index += stride;
	}

	for (ndInt32 i = 0; i < size; ++i) 
	{
		ndInt32 j;
		ndInt32 iter = 0;
		do 
		{
			for (j = i; j < size - 1; ++j) 
			{
  				T dd(ndAbs(eigenValues[j]) + ndAbs(eigenValues[j + 1]));
				if (ndAbs(offDiag[j]) <= (T(1.e-6f) * dd)) 
				{
					break;
				}
			}

			if (j != i) 
			{
				iter++;
				if (iter == 10) 
				{
					ndAssert(0);
					return;
				}

				T g((eigenValues[i + 1] - eigenValues[i]) / (T(2.0f) * offDiag[i]));
				T r(ndPythag(g, T(1.0f)));
				g = eigenValues[j] - eigenValues[i] + offDiag[i] / (g + ndSign(r, g));
				T s(1.0f);
				T c(1.0f);
				T p(0.0f);

				ndInt32 k;
				for (k = j - 1; k >= i; k--)
				{
					T f(s * offDiag[k]);
					T b(c * offDiag[k]);
					T d(ndPythag(f, g));
					offDiag[k + 1] = d;
					if (d == T(0.0f)) 
					{
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

				if (r == T(0.0f) && k >= i) 
				{
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
T ndConditionNumber(const ndInt32 size, const ndInt32 stride, const T* const choleskyMatrix)
{
	T* const eigenValues = ndAlloca(T, size);
	ndEigenValues(size, stride, choleskyMatrix, eigenValues);

	T minVal = T(1.0e20f);
	T maxVal = T(-1.0e20f);
	for (ndInt32 i = 0; i < size; ++i) 
	{
		minVal = ndMin(minVal, eigenValues[i]);
		maxVal = ndMax(maxVal, eigenValues[i]);
	}
	T condition = T(ndAbs(maxVal) / ndAbs(minVal));
	return condition;
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
void ndGaussSeidelLcpSor(const ndInt32 size, const T* const matrix, T* const x, const T* const b, const T* const low, const T* const high, T tol2, ndInt32 maxIterCount, ndInt16* const clipped, T sor)
{
	const T* const me = matrix;
	T* const invDiag1 = ndAlloca(T, size);

	ndInt32 stride = 0;
	for (ndInt32 i = 0; i < size; ++i) 
	{
		x[i] = ndClamp(T(0.0f), low[i], high[i]);
		invDiag1[i] = T(1.0f) / me[stride + i];
		stride += size;
	}

	T tolerance(tol2 * 2.0f);
	const T* const invDiag = invDiag1;
#ifdef _DEBUG 
	ndInt32 passes = 0;
#endif
	for (ndInt32 i = 0; (i < maxIterCount) && (tolerance > tol2); ++i) 
	{
		ndInt32 base = 0;
		tolerance = T(0.0f);
#ifdef _DEBUG 
		passes++;
#endif
		for (ndInt32 j = 0; j < size; ++j) 
		{
			const T* const row = &me[base];
			T r(b[j] - ndDotProduct(size, row, x));
			T f((r + row[j] * x[j]) * invDiag[j]);
			if (f > high[j]) 
			{
				x[j] = high[j];
				clipped[j] = 1;
			} 
			else if (f < low[j]) 
			{
				x[j] = low[j];
				clipped[j] = 1;
			} 
			else 
			{
				clipped[j] = 0;
				tolerance += r * r;
				x[j] = x[j] + (f - x[j]) * sor;
			}
			base += size;
		}
	}
}

template <class T>
void ndGaussSeidelLcpSor(const ndInt32 size, const ndInt32 stride, const T* const matrix, T* const x, const T* const b, const ndInt32* const normalIndex, const T* const low, const T* const high, T tol2, ndInt32 maxIterCount, T sor)
{
	const T* const me = matrix;
	T* const invDiag1 = ndAlloca(T, size);
	T* const u = ndAlloca(T, size + 1);
	ndInt32* const index = ndAlloca(ndInt32, size);

	u[size] = T(1.0f);
	ndInt32 rowStart = 0;
	for (ndInt32 j = 0; j < size; ++j) 
	{
		u[j] = x[j];
		index[j] = normalIndex[j] ? j + normalIndex[j] : size;
	}

	for (ndInt32 j = 0; j < size; ++j) 
	{
		const T val = u[index[j]];
		const T l = low[j] * val;
		const T h = high[j] * val;
		u[j] = ndClamp(u[j], l, h);
		invDiag1[j] = T(1.0f) / me[rowStart + j];
		rowStart += stride;
	}

	T tolerance(tol2 * 2.0f);
	const T* const invDiag = invDiag1;
	const ndInt32 maxCount = ndMax(8, size);
	for (ndInt32 i = 0; (i < maxCount) && (tolerance > tol2); ++i) 
	{
		ndInt32 base = 0;
		tolerance = T(0.0f);
		for (ndInt32 j = 0; j < size; ++j) 
		{
			const T* const row = &me[base];
			T r(b[j] - ndDotProduct(size, row, u));
			T f((r + row[j] * u[j]) * invDiag[j]);

			const T val = u[index[j]];
			const T l = low[j] * val;
			const T h = high[j] * val;
			if (f > h) 
			{
				u[j] = h;
			}
			else if (f < l) 
			{
				u[j] = l;
			}
			else 
			{
				tolerance += r * r;
				u[j] = f;
			}
			base += stride;
		}
	}

#ifdef _DEBUG 
	ndInt32 passes = 0;
#endif
	for (ndInt32 i = 0; (i < maxIterCount) && (tolerance > tol2); ++i) 
	{
		ndInt32 base = 0;
		tolerance = T(0.0f);
#ifdef _DEBUG 
		passes++;
#endif
		for (ndInt32 j = 0; j < size; ++j) 
		{
			const T* const row = &me[base];
			T r(b[j] - ndDotProduct(size, row, u));
			T f((r + row[j] * u[j]) * invDiag[j]);
			f = u[j] + (f - u[j]) * sor;

			const T val = u[index[j]];
			const T l = low[j] * val;
			const T h = high[j] * val;
			if (f > h) 
			{
				u[j] = h;
			}
			else if (f < l) 
			{
				u[j] = l;
			}
			else 
			{
				tolerance += r * r;
				u[j] = f;
			}
			base += stride;
		}
	}

	for (ndInt32 j = 0; j < size; ++j) 
	{
		x[j] = u[j];
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
void ndGaussSeidelLCP(const ndInt32 size, const T* const matrix, T* const x, const T* const b, const T* const low, const T* const high, T sor = T(1.2f))
{
	ndInt16* const clipped = ndAlloca(ndInt16, size);
	ndGaussSeidelLcpSor(size, matrix, x, b, low, high, T(1.0e-3f), size * size, clipped, sor);
}

template<class T>
void ndPermuteRows(ndInt32 size, ndInt32 i, ndInt32 j, T* const matrix, T* const choleskyMatrix, T* const x, T* const r, T* const low, T* const high, ndInt16* const permute)
{
	if (i != j) 
	{
		T* const A = &matrix[size * i];
		T* const B = &matrix[size * j];
		T* const invA = &choleskyMatrix[size * i];
		T* const invB = &choleskyMatrix[size * j];
		for (ndInt32 k = 0; k < size; ++k) 
		{
			ndSwap(A[k], B[k]);
			ndSwap(invA[k], invB[k]);
		}

		ndInt32 stride = 0;
		for (ndInt32 k = 0; k < size; ++k) 
		{
			ndSwap(matrix[stride + i], matrix[stride + j]);
			stride += size;
		}

		ndSwap(x[i], x[j]);
		ndSwap(r[i], r[j]);
		ndSwap(low[i], low[j]);
		ndSwap(high[i], high[j]);
		ndSwap(permute[i], permute[j]);
	}
}

template<class T>
void ndCalculateDelta_x(ndInt32 size, ndInt32 n, const T* const matrix, const T* const choleskyMatrix, T* const delta_x)
{
	const T* const row = &matrix[size * n];
	for (ndInt32 i = 0; i < n; ++i) 
	{
		delta_x[i] = -row[i];
	}
	ndSolveCholesky(size, n, choleskyMatrix, delta_x, delta_x);
	delta_x[n] = T(1.0f);
}

// calculate delta_r = A * delta_x
template<class T>
void ndCalculateDelta_r(ndInt32 size, ndInt32 n, const T* const matrix, const T* const delta_x, T* const delta_r)
{
	ndInt32 stride = n * size;
	const ndInt32 size1 = n + 1;
	for (ndInt32 i = n; i < size; ++i) 
	{
		delta_r[i] = ndDotProduct(size1, &matrix[stride], delta_x);
		stride += size;
	}
}

template<class T>
void ndHouseholderReflection(ndInt32 size, ndInt32 row, ndInt32 colum, T* const choleskyMatrix, T* const tmp, T* const reflection)
{
	ndAssert(row <= colum);
	if (row < colum) 
	{
		for (ndInt32 i = row; i <= colum; ++i) 
		{
			T* const rowI = &choleskyMatrix[size * i];
			T mag2(0.0f);
			for (ndInt32 j = i + 1; j <= colum; ++j) 
			{
				mag2 += rowI[j] * rowI[j];
				reflection[j] = rowI[j];
			}
			if (mag2 > T(1.0e-14f)) 
			{
				reflection[i] = rowI[i] + ndSign(rowI[i]) * T(sqrt(mag2 + rowI[i] * rowI[i]));

				const T vMag2(mag2 + reflection[i] * reflection[i]);
				const T den = T(2.0f) / vMag2;
				for (ndInt32 j = i; j < size; ++j) 
				{
					T acc(0.0f);
					T* const rowJ = &choleskyMatrix[size * j];
					for (ndInt32 k = i; k <= colum; ++k) {
						acc += rowJ[k] * reflection[k];
					}
					tmp[j] = acc;
				}

				for (ndInt32 j = i + 1; j < size; ++j) 
				{
					rowI[j] = T(0.0f);
					T* const rowJ = &choleskyMatrix[size * j];
					const T a = tmp[j] * den;
					for (ndInt32 k = i; k <= colum; ++k) 
					{
						rowJ[k] -= a * reflection[k];
					}
				}
				rowI[i] -= tmp[i] * reflection[i] * den;
			}

			if (rowI[i] < T(0.0f)) 
			{
				for (ndInt32 k = i; k < size; ++k) 
				{
					choleskyMatrix[size * k + i] = -choleskyMatrix[size * k + i];
				}
			}
		}

		for (ndInt32 i = row; i < size; ++i) 
		{
			choleskyMatrix[size * i + i] = ndMax(choleskyMatrix[size * i + i], T(1.0e-6f));
		}
	}
}

template<class T>
void ndCholeskyUpdate(ndInt32 size, ndInt32 row, ndInt32 colum, T* const choleskyMatrix, T* const tmp, T* const reflexion, const T* const psdMatrix)
{
	const ndInt32 n0 = colum - row;
	const ndInt32 n1 = n0 + 1;
	const ndInt32 choleskyCost = size * size * size / 3;
	const ndInt32 householdCost = n0 * (n0 + 1) / 2 + n1 * (n1 + 1) * (2 * (2 * n1 + 1) - 3 + 3 * (size - colum - 1)) / 6 - 1;

	if (householdCost < choleskyCost) 
	{
		ndHouseholderReflection(size, row, colum, choleskyMatrix, tmp, reflexion);
	} 
	else 
	{
		memcpy (choleskyMatrix, psdMatrix, sizeof (T) * size * size);
		ndCholeskyFactorization(size, choleskyMatrix);
	}

//#if _DEBUG
#if 0
	T* const psdMatrixCopy = dAlloca(T, size * size);
	memcpy(psdMatrixCopy, psdMatrix, sizeof(T) * size * size);
	dCholeskyFactorization(size, psdMatrixCopy);

	for (dInt32 i = 0; i < size; ++i) 
	{
		for (dInt32 j = 0; j < size; ++j) 
		{
			T err = psdMatrixCopy[i*size + j] - choleskyMatrix[i*size + j];
			dAssert(dAbs(err) < T(1.0e-4f));
		}
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
void ndSolveDantzigLcpLow(ndInt32 size, T* const symmetricMatrixPSD, T* const x, T* const b, T* const low, T* const high)
{
	T* const x0 = ndAlloca(T, size);
	T* const r0 = ndAlloca(T, size);
	T* const tmp0 = ndAlloca(T, size);
	T* const tmp1 = ndAlloca(T, size);
	T* const delta_r = ndAlloca(T, size);
	T* const delta_x = ndAlloca(T, size);
	T* const lowerTriangularMatrix = ndAlloca(T, size * size);
	ndInt16* const permute = ndAlloca(ndInt16, size);

	for (ndInt32 i = 0; i < size; ++i) 
	{
		permute[i] = ndInt16(i);
		x0[i] = T(0.0f);
		x[i] = ndMax (b[i] * b[i], T (1.0f));
	}

	for (ndInt32 n = size - 1, i = size - 1; i >= 0; i--) 
	{
		if (x[i] > T(1.0)) 
		{
			ndPermuteRows(size, n, i, symmetricMatrixPSD, lowerTriangularMatrix, x, b, low, high, permute);
			n --;
		}
	}

	for (ndInt32 i = size - 1; (i >= 0) && (x[i] > T(1.0f)) ; i--) 
	{
		ndInt32 min = i;
		for (ndInt32 j = i - 1; (j >= 0) && (x[j] > T(1.0f)); j--) 
		{
			if (x[j] > x[min]) 
			{
				min = j;
			}
		}
		if (min != i) 
		{
			ndPermuteRows(size, i, min, symmetricMatrixPSD, lowerTriangularMatrix, x, b, low, high, permute);
		}
	}

	ndInt32 initialGuessCount = size;
	while (x[initialGuessCount - 1] >= T(16.0f)) 
	{
		initialGuessCount --;
	}
	
	memcpy(lowerTriangularMatrix, symmetricMatrixPSD, sizeof(T) * size * size);
#ifdef _DEBUG
	bool valid = ndCholeskyFactorization(size, lowerTriangularMatrix);
	ndAssert(valid);
#else
	ndCholeskyFactorization(size, lowerTriangularMatrix);
#endif
	for (ndInt32 j = 0; (j != -1) && initialGuessCount;) 
	{
		ndSolveCholesky(size, initialGuessCount, lowerTriangularMatrix, x0, b);

		j = -1;
		T alpha(1.0f);
		T value(0.0f);
		for (ndInt32 i = initialGuessCount - 1; i >= 0; i--) 
		{
			T x1 = alpha * x0[i];
			if (x1 < low[i]) 
			{
				j = i;
				value = low[i];
				alpha = low[i] / x0[i];
			} 
			else if (x1 > high[i]) 
			{
				j = i;
				value = high[i];
				alpha = high[i] / x0[i];
			}
		}

		if (j != -1) 
		{
			x0[j] = value;
			initialGuessCount--;
			ndPermuteRows(size, j, initialGuessCount, symmetricMatrixPSD, lowerTriangularMatrix, x0, b, low, high, permute);
			ndCholeskyUpdate(size, j, initialGuessCount, lowerTriangularMatrix, tmp0, tmp1, symmetricMatrixPSD);
		}
	}

	if (initialGuessCount == size) 
	{
		for (ndInt32 i = 0; i < size; ++i) 
		{
			ndInt32 j = permute[i];
			x[j] = x0[i];
			b[i] = T(0.0f);
		}
		return;
	}

	ndInt32 clampedIndex = size;
	ndInt32 index = initialGuessCount;
	ndInt32 count = size - initialGuessCount;
	ndInt32 stride = index * size;

	for (ndInt32 i = 0; i < size; ++i) 
	{
		r0[i] = T(0.0f);
		delta_x[i] = T(0.0f);
		delta_r[i] = T(0.0f);
	}
	
	for (ndInt32 i = index; i < size; ++i) 
	{
		r0[i] = ndDotProduct(size, &symmetricMatrixPSD[stride], x0) - b[i];
		stride += size;
	}


	while (count) 
	{
		bool loop = true;

		while (loop) 
		{
			loop = false;
			T clamp_x(0.0f);
			ndInt32 swapIndex = -1;

			if (ndAbs(r0[index]) > T(1.0e-12f)) 
			{
				ndCalculateDelta_x(size, index, symmetricMatrixPSD, lowerTriangularMatrix, delta_x);
				ndCalculateDelta_r(size, index, symmetricMatrixPSD, delta_x, delta_r);

				ndAssert(delta_r[index] != T(0.0f));
				ndAssert(ndAbs(delta_x[index]) == T(1.0f));
				delta_r[index] = (delta_r[index] == T(0.0f)) ? T(1.0e-12f) : delta_r[index];

				T scale = -r0[index] / delta_r[index];
				ndAssert(ndAbs(scale) >= T(0.0f));

				for (ndInt32 i = 0; i <= index; ++i) 
				{
					T x1 = x0[i] + scale * delta_x[i];
					if (x1 > high[i]) 
					{
						swapIndex = i;
						clamp_x = high[i];
						scale = (high[i] - x0[i]) / delta_x[i];
					} 
					else if (x1 < low[i]) 
					{
						swapIndex = i;
						clamp_x = low[i];
						scale = (low[i] - x0[i]) / delta_x[i];
					}
				}
				ndAssert(ndAbs(scale) >= T(0.0f));

				for (ndInt32 i = clampedIndex; (i < size) && (scale > T(1.0e-12f)); ++i) 
				{
					T r1 = r0[i] + scale * delta_r[i];
					if ((r1 * r0[i]) < T(0.0f)) 
					{
						ndAssert(ndAbs(delta_r[i]) > T(0.0f));
						T s1 = -r0[i] / delta_r[i];
						ndAssert(ndAbs(s1) >= T(0.0f));
						ndAssert(ndAbs(s1) <= ndAbs(scale));
						if (ndAbs(s1) < ndAbs(scale)) 
						{
							scale = s1;
							swapIndex = i;
						}
					}
				}

				if (ndAbs(scale) > T(1.0e-12f)) 
				{
					for (ndInt32 i = 0; i < size; ++i) 
					{
						x0[i] += scale * delta_x[i];
						r0[i] += scale * delta_r[i];
					}
				}
			}

			if (swapIndex == -1) 
			{
				r0[index] = T(0.0f);
				delta_r[index] = T(0.0f);
				index++;
				count--;
				loop = false;
			} 
			else if (swapIndex == index) 
			{
				count--;
				clampedIndex--;
				x0[index] = clamp_x;
				ndPermuteRows(size, index, clampedIndex, symmetricMatrixPSD, lowerTriangularMatrix, x0, r0, low, high, permute);
				ndCholeskyUpdate(size, index, clampedIndex, lowerTriangularMatrix, tmp0, tmp1, symmetricMatrixPSD);
				loop = count ? true : false;
			} 
			else if (swapIndex > index) 
			{
				loop = true;
				r0[swapIndex] = T(0.0f);
				ndAssert(swapIndex < size);
				ndAssert(clampedIndex <= size);
				if (swapIndex < clampedIndex) 
				{
					count--;
					clampedIndex--;
					ndPermuteRows(size, clampedIndex, swapIndex, symmetricMatrixPSD, lowerTriangularMatrix, x0, r0, low, high, permute);
					ndCholeskyUpdate(size, swapIndex, clampedIndex, lowerTriangularMatrix, tmp0, tmp1, symmetricMatrixPSD);
					ndAssert(clampedIndex >= index);
				} 
				else 
				{
					count++;
					ndAssert(clampedIndex < size);
					ndPermuteRows(size, clampedIndex, swapIndex, symmetricMatrixPSD, lowerTriangularMatrix, x0, r0, low, high, permute);
					ndCholeskyUpdate(size, clampedIndex, swapIndex, lowerTriangularMatrix, tmp0, tmp1, symmetricMatrixPSD);
					clampedIndex++;
					ndAssert(clampedIndex <= size);
					ndAssert(clampedIndex >= index);
				}
			} 
			else 
			{
				ndAssert(index > 0);
				x0[swapIndex] = clamp_x;
				delta_x[index] = T(0.0f);

				ndAssert(swapIndex < index);
				ndPermuteRows(size, swapIndex, index - 1, symmetricMatrixPSD, lowerTriangularMatrix, x0, r0, low, high, permute);
				ndPermuteRows(size, index - 1, index, symmetricMatrixPSD, lowerTriangularMatrix, x0, r0, low, high, permute);
				ndPermuteRows(size, clampedIndex - 1, index, symmetricMatrixPSD, lowerTriangularMatrix, x0, r0, low, high, permute);
				ndCholeskyUpdate (size, swapIndex, clampedIndex - 1, lowerTriangularMatrix, tmp0, tmp1, symmetricMatrixPSD);

				clampedIndex--;
				index--;
				loop = true;
			}
		}
	}

	for (ndInt32 i = 0; i < size; ++i) 
	{
		ndInt32 j = permute[i];
		x[j] = x0[i];
		b[j] = r0[i];
	}
}

/*
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
bool ndSolveDantzigLCP(ndInt32 size, T* const symetricMatrix, T* const x, T* const b, T* const low, T* const high)
{
	T* const choleskyMatrix = ndAlloca(T, size * size);
	dCheckAligment(choleskyMatrix);

	memcpy (choleskyMatrix, symetricMatrix, sizeof (T) * size * size);
	ndCholeskyFactorization(size, choleskyMatrix);
	for (ndInt32 i = 0; i < size; ++i) 
	{
		T* const row = &choleskyMatrix[i * size];
		for (ndInt32 j = i + 1; j < size; ++j) 
		{
			row[j] = T(0.0f);
		}
	}
	return ndSolveDantzigLCP(size, symetricMatrix, choleskyMatrix, x, b, low, high);
}
*/

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
bool ndSolvePartitionDantzigLCP(ndInt32 size, T* const symmetricMatrixPSD , T* const x, T* const b, T* const low, T* const high)
{
	ndInt16* const permute = ndAlloca(ndInt16, size);

	for (ndInt32 i = 0; i < size; ++i) 
	{
		x[i] = b[i];
		permute[i] = ndInt16(i);
	}

	ndInt32 unboundedSize = size;
	for (ndInt32 i = 0; i < unboundedSize; ++i) 
	{
		if ((low[i] <= T(-D_LCP_MAX_VALUE)) && (high[i] >= T(D_LCP_MAX_VALUE))) 
		{
			ndCholeskyFactorizationAddRow(size, i, symmetricMatrixPSD );
		} 
		else 
		{
			ndInt32 j = unboundedSize - 1;
			if (i != j) 
			{
				T* const A = &symmetricMatrixPSD [size * i];
				T* const B = &symmetricMatrixPSD [size * j];
				for (ndInt32 k = 0; k < size; ++k) 
				{
					ndSwap(A[k], B[k]);
				}

				ndInt32 stride = 0;
				for (ndInt32 k = 0; k < size; ++k) 
				{
					ndSwap(symmetricMatrixPSD [stride + i], symmetricMatrixPSD [stride + j]);
					stride += size;
				}
				ndSwap(x[i], x[j]);
				ndSwap(b[i], b[j]);
				ndSwap(low[i], low[j]);
				ndSwap(high[i], high[j]);
				ndSwap(permute[i], permute[j]);
			}

			i--;
			unboundedSize--;
		}
	}

	bool ret = false;
	if (unboundedSize > 0) 
	{
		ndSolveCholesky(size, unboundedSize, symmetricMatrixPSD , x);
		ndInt32 base = unboundedSize * size;
		for (ndInt32 i = unboundedSize; i < size; ++i) 
		{
			b[i] -= ndDotProduct(unboundedSize, &symmetricMatrixPSD[base], x);
			base += size;
		}

		const ndInt32 boundedSize = size - unboundedSize;
		T* const l = ndAlloca(T, boundedSize);
		T* const h = ndAlloca(T, boundedSize);
		T* const c = ndAlloca(T, boundedSize);
		T* const u = ndAlloca(T, boundedSize);
		T* const a11 = ndAlloca(T, boundedSize * boundedSize);
		T* const a10 = ndAlloca(T, boundedSize * unboundedSize);

		for (ndInt32 i = 0; i < boundedSize; ++i) 
		{
			T* const g = &a10[i * unboundedSize];
			const T* const row = &symmetricMatrixPSD [(unboundedSize + i) * size];
			for (ndInt32 j = 0; j < unboundedSize; ++j) 
			{
				g[j] = -row[j];
			}
			ndSolveCholesky(size, unboundedSize, symmetricMatrixPSD, g);

			T* const arow = &a11[i * boundedSize];
			const T* const row2 = &symmetricMatrixPSD[(unboundedSize + i) * size];
			arow[i] = row2[unboundedSize + i] + ndDotProduct(unboundedSize, g, row2);
			for (ndInt32 j = i + 1; j < boundedSize; ++j) 
			{
				const T* const row1 = &symmetricMatrixPSD [(unboundedSize + j) * size];
				T elem = row1[unboundedSize + i] + ndDotProduct(unboundedSize, g, row1);
				arow[j] = elem;
				a11[j * boundedSize + i] = elem;
			}
			u[i] = T(0.0f);
			c[i] = b[i + unboundedSize];
			l[i] = low[i + unboundedSize];
			h[i] = high[i + unboundedSize];
		}

		if (ndSolveDantzigLCP(boundedSize, a11, u, c, l, h)) 
		{
			for (ndInt32 i = 0; i < boundedSize; ++i) 
			{
				const T s = u[i];
				x[unboundedSize + i] = s;
				const T* const g = &a10[i * unboundedSize];
				for (ndInt32 j = 0; j < unboundedSize; ++j) 
				{
					x[j] += g[j] * s;
				}
			}
			ret = true;
		}
	} 
	else 
	{
		for (ndInt32 i = 0; i < size; ++i) 
		{
			x[i] = T(0.0f);
		}
		ret = ndSolveDantzigLCP(size, symmetricMatrixPSD, x, b, low, high);
	}

	for (ndInt32 i = 0; i < size; ++i) 
	{
		b[i] = x[i];
	}
	for (ndInt32 i = 0; i < size; ++i) 
	{
		ndInt32 j = permute[i];
		x[j] = b[i];
		b[i] = T(0.0f);
	}
	return ret;
}

template <class T>
void ndSolveDantzigLCP(ndInt32 size, T* const symmetricMatrixPSD, T* const x, T* const b, T* const low, T* const high)
{
	T tol2 = T(0.25f * 0.25f);
	ndInt32 passes = ndClamp(size, 12, 20);
	T* const r = ndAlloca(T, size);
	ndInt16* const clipped = ndAlloca(ndInt16, size);

	// find an approximation to the solution
	ndGaussSeidelLcpSor(size, symmetricMatrixPSD, x, b, low, high, tol2, passes, clipped, T(1.3f));

	T err2(0.0f);
	ndInt32 stride = 0;
	ndInt32 clippeCount = 0;
	for (ndInt32 i = 0; i < size; ++i) 
	{
		const T* const row = &symmetricMatrixPSD[stride];
		r[i] = b[i] - ndDotProduct(size, row, x);
		clippeCount += clipped[i];
		err2 += clipped[i] ? T(0.0f) : r[i] * r[i];
		stride += size;
	}

	if (err2 > tol2) 
	{
		// check for small lcp
		if ((clippeCount < 16) && ((clippeCount < 32) && (err2 < T(16.0f)))) 
		{
			// small lcp can be solved with direct method
			T* const x0 = ndAlloca(T, size);
			for (ndInt32 i = 0; i < size; ++i) 
			{
				low[i] -= x[i];
				high[i] -= x[i];
			}
			ndSolveDantzigLcpLow(size, symmetricMatrixPSD, x0, r, low, high);
			for (ndInt32 i = 0; i < size; ++i) 
			{
				x[i] += x0[i];
			}
		} 
		else 
		{
			// larger lcp are too hard for direct method, see if we can get better approximation
			ndGaussSeidelLcpSor(size, symmetricMatrixPSD, x, b, low, high, tol2, 20, clipped, T(1.3f));
		}
	}
}

#endif
