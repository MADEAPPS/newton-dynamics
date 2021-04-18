
/* Copyright (c) <2003-2019> <Julio Jerez, Newton Game Dynamics>
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

#ifndef __D_GENERAL_MATRIX_H__
#define __D_GENERAL_MATRIX_H__

#include "dCoreStdafx.h"
#include "dTypes.h"
#include "dGeneralVector.h"

#define D_LCP_MAX_VALUE dFloat32 (1.0e10f)


template<class T>
class dSymmetricConjugateGradientSolver
{
	public:
	dSymmetricConjugateGradientSolver();
	dSymmetricConjugateGradientSolver(T* const r0, T* const z0, T* const p0, T* const q0);
	~dSymmetricConjugateGradientSolver();

	void SetBuffers(T* const r0, T* const z0, T* const p0, T* const q0);
	T Solve(dInt32 size, T tolerance, T* const x, const T* const b);

	protected:
	virtual void MatrixTimeVector(T* const out, const T* const v) const = 0;
	virtual void InversePrecoditionerTimeVector(T* const out, const T* const v) const = 0;

	private:
	T SolveInternal(dInt32 size, T tolerance, T* const x, const T* const b) const;
	//T DotProduct(dInt32 size, const T* const b, const T* const c) const;
	//void Sub(dInt32 size, T* const a, const T* const b, const T* const c) const;
	//void ScaleAdd(dInt32 size, T* const a, const T* const b, T scale, const T* const c) const;

	T* m_r0;
	T* m_z0;
	T* m_p0;
	T* m_q0;
};

template<class T>
dSymmetricConjugateGradientSolver<T>::dSymmetricConjugateGradientSolver()
{
	SetBuffers(nullptr, nullptr, nullptr, nullptr);
}

template<class T>
dSymmetricConjugateGradientSolver<T>::dSymmetricConjugateGradientSolver(T* const r0, T* const z0, T* const p0, T* const q0)
{
	SetBuffers(r0, z0, p0, q0);
}

template<class T>
dSymmetricConjugateGradientSolver<T>::~dSymmetricConjugateGradientSolver() 
{
}

template<class T>
void dSymmetricConjugateGradientSolver<T>::SetBuffers(T* const r0, T* const z0, T* const p0, T* const q0)
{
	m_r0 = r0;
	m_z0 = z0;
	m_p0 = p0;
	m_q0 = q0;
}

template<class T>
T dSymmetricConjugateGradientSolver<T>::Solve(dInt32 size, T tolerance, T* const x, const T* const b)
{
	if (m_r0) 
	{
		return SolveInternal(size, tolerance, x, b);
	} 
	else 
	{
		T* const r0 = dAlloca(T, size);
		T* const z0 = dAlloca(T, size);
		T* const p0 = dAlloca(T, size);
		T* const q0 = dAlloca(T, size);
		SetBuffers(r0, z0, p0, q0);
		T error = SolveInternal(size, tolerance, x, b);
		SetBuffers(nullptr, nullptr, nullptr, nullptr);
		return error;
	}
}

template<class T>
T dSymmetricConjugateGradientSolver<T>::SolveInternal(dInt32 size, T tolerance, T* const x, const T* const b) const
{
	MatrixTimeVector(m_z0, x);
	dSub(size, m_r0, b, m_z0);
	InversePrecoditionerTimeVector(m_p0, m_r0);

	dInt32 iter = 0;
	T num = dDotProduct(size, m_r0, m_p0);
	T error2 = num;
	for (dInt32 j = 0; (j < size) && (error2 > tolerance); j++) 
	{
		MatrixTimeVector(m_z0, m_p0);
		T den = dDotProduct(size, m_p0, m_z0);

		dAssert(fabs(den) > T(0.0f));
		T alpha = num / den;

		dMulAdd(size, x, x, m_p0, alpha);
		if ((j % 50) != 49) 
		{
			dMulAdd(size, m_r0, m_r0, m_z0, -alpha);
		} else 
		{
			MatrixTimeVector(m_z0, x);
			dSub(size, m_r0, b, m_z0);
		}

		InversePrecoditionerTimeVector(m_q0, m_r0);

		T num1 = dDotProduct(size, m_r0, m_q0);
		T beta = num1 / num;
		dMulAdd(size, m_p0, m_q0, m_p0, beta);
		num = dDotProduct(size, m_r0, m_q0);
		iter++;
		error2 = num;
		if (j > 10) 
		{
			error2 = T(0.0f);
			for (dInt32 i = 0; i < size; i++) 
			{
				error2 = dMax(error2, m_r0[i] * m_r0[i]);
			}
		}
	}
	dAssert(iter <= size);
	return num;
}


//*************************************************************
//
// generic linear algebra functions
//
//*************************************************************
template<class T>
void dMatrixTimeVector(dInt32 size, const T* const matrix, const T* const v, T* const out)
{
	dInt32 stride = 0;
	for (dInt32 i = 0; i < size; i++) 
	{
		const T* const row = &matrix[stride];
		out[i] = dDotProduct(size, row, v);
		stride += size;
	}
}

template<class T>
void dMatrixTimeMatrix(dInt32 size, const T* const matrixA, const T* const matrixB, T* const out)
{
	for (dInt32 i = 0; i < size; i++) 
	{
		const T* const rowA = &matrixA[i * size];
		T* const rowOut = &out[i * size];
		for (dInt32 j = 0; j < size; j++) 
		{
			T acc = T(0.0f);
			for (dInt32 k = 0; k < size; k++) 
			{
				acc += rowA[k] * matrixB[k * size + j];
			}
			rowOut[j] = acc;
		}
	}
}

template<class T>
void dCovarianceMatrix(dInt32 size, T* const matrix, const T* const vectorA, const T* const vectorB)
{
	dInt32 stride = 0;
	for (dInt32 i = 0; i < size; i++) 
	{
		T* const row = &matrix[stride];
		T scale (vectorA[i]);
		for (dInt32 j = 0; j < size; j++) 
		{
			row[j] = scale * vectorA[j];
		}
		stride += size;
	}
}

template<class T>
bool dCholeskyFactorizationAddRow(dInt32, dInt32 stride, dInt32 n, T* const matrix, T* const invDiagonalOut)
{
	T* const rowN = &matrix[stride * n];

	dInt32 base = 0;
	for (dInt32 j = 0; j <= n; j++) 
	{
		T s(0.0f);
		T* const rowJ = &matrix[base];
		for (dInt32 k = 0; k < j; k++) 
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
bool dCholeskyFactorization(dInt32 size, dInt32 stride, T* const psdMatrix)
{
	bool state = true;
	T* const invDiagonal = dAlloca(T, size);
	for (dInt32 i = 0; (i < size) && state; i++) 
	{
		state = state && dCholeskyFactorizationAddRow(size, stride, i, psdMatrix, invDiagonal);
	}
	return state;
}

template<class T>
bool dTestPSDmatrix(dInt32 size, dInt32 stride, T* const matrix)
{
	T* const copy = dAlloca(T, size * size);
	dInt32 row = 0;
	for (dInt32 i = 0; i < size; i++) 
	{
		memcpy(&copy[i * size], &matrix[row], size * sizeof (T));
		row += stride;
	}
	return dCholeskyFactorization(size, size, copy);
}

template<class T>
void dCholeskyApplyRegularizer (dInt32 size, dInt32 stride, T* const psdMatrix, T* const regularizer)
{
	bool isPsdMatrix = false;
	dFloat32* const lowerTriangule = dAlloca(dFloat32, stride * stride);
	do 
	{
		memcpy(lowerTriangule, psdMatrix, sizeof(dFloat32) * stride * stride);
		isPsdMatrix = dCholeskyFactorization(size, stride, lowerTriangule);
		if (!isPsdMatrix) 
		{
			for (dInt32 i = 0; i < size; i++) 
			{
				regularizer[i] *= dFloat32(4.0f);
				psdMatrix[i * stride + i] += regularizer[i];
			}
		}
	} while (!isPsdMatrix);
}

template<class T>
void dSolveCholesky(dInt32 size, dInt32 stride, const T* const choleskyMatrix, T* const x, const T* const b)
{
	dInt32 rowStart = 0;
	for (dInt32 i = 0; i < size; i++) 
	{
		T acc(0.0f);
		const T* const row = &choleskyMatrix[rowStart];
		for (dInt32 j = 0; j < i; j++) 
		{
			acc = acc + row[j] * x[j];
		}
		x[i] = (b[i] - acc) / row[i];
		rowStart += stride;
	}

	for (dInt32 i = size - 1; i >= 0; i--) 
	{
		T acc = 0.0f;
		for (dInt32 j = i + 1; j < size; j++) 
		{
			acc = acc + choleskyMatrix[stride * j + i] * x[j];
		}
		x[i] = (x[i] - acc) / choleskyMatrix[stride * i + i];
	}
}

template<class T>
void dSolveCholesky(dInt32 size, T* const choleskyMatrix, T* const x)
{
	dSolveCholesky(size, size, choleskyMatrix, x);
}

template<class T>
bool dSolveGaussian(dInt32 size, T* const matrix, T* const b)
{
	for (dInt32 i = 0; i < size - 1; i++) 
	{
		const T* const rowI = &matrix[i * size];
		dInt32 m = i;
		T maxVal (dAbs(rowI[i]));
		for (dInt32 j = i + 1; j < size - 1; j++) 
		{
			T val (dAbs(matrix[size * j + i]));
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
			for (dInt32 j = 0; j < size; j++) 
			{
				dSwap(rowK[j], rowJ[j]);
			}
			dSwap(b[i], b[m]);
		}

		T den = T(1.0f) / rowI[i];
		for (dInt32 k = i + 1; k < size; k++) 
		{
			T* const rowK = &matrix[size * k];
			T factor(-rowK[i] * den);
			for (dInt32 j = i + 1; j < size; j++) 
			{
				rowK[j] += rowI[j] * factor;
			}
			rowK[i] = T(0.0f);
			b[k] += b[i] * factor;
		}
	}

	for (dInt32 i = size - 1; i >= 0; i--) 
	{
		T acc(0);
		T* const rowI = &matrix[i * size];
		for (dInt32 j = i + 1; j < size; j++) 
		{
			acc = acc + rowI[j] * b[j];
		}
		b[i] = (b[i] - acc) / rowI[i];
	}
	return true;
}

template <class T>
void dEigenValues(const dInt32 size, const dInt32 stride, const T* const symmetricMatrix, T* const eigenValues)
{
	T* const offDiag = dAlloca(T, size);
	T* const matrix = dAlloca(T, size * stride);

	memcpy(matrix, symmetricMatrix, sizeof(T) * size * stride);
	for (dInt32 i = size - 1; i > 0; i--) 
	{
		T h(0.0f);
		T* const rowI = &matrix[i * stride];

		if (i > 1) 
		{
			T scale(0.0f);
			for (dInt32 k = 0; k < i; k++) 
			{
				scale += dAbs(rowI[k]);
			}

			if (scale == T(0.0f)) 
			{
				offDiag[i] = rowI[i - 1];
			} 
			else 
			{
				for (dInt32 k = 0; k < i; k++) 
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

				for (dInt32 j = 0; j < i; j++) 
				{
					g = T(0.0f);
					const T* const rowJ = &matrix[j * stride];
					for (dInt32 k = 0; k <= j; k++) 
					{
						g += rowJ[k] * rowI[k];
					}
					for (dInt32 k = j + 1; k < i; k++) 
					{
						g += matrix[k * stride + j] * rowI[k];
					}
					offDiag[j] = g / h;
					f += offDiag[j] * rowI[j];
				}

				T hh(f / (h + h));
				for (dInt32 j = 0; j < i; j++) 
				{
					T f1 (rowI[j]);
					T g1(offDiag[j] - hh * f1);
					offDiag[j] = g1;
					T* const rowJ = &matrix[j * stride];
					for (dInt32 k = 0; k <= j; k++) 
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

	dInt32 index = stride;
	eigenValues[0] = matrix[0];
	for (dInt32 i = 1; i < size; i++) 
	{
		eigenValues[i] = matrix[index + i];
		offDiag[i - 1] = offDiag[i];
		index += stride;
	}

	for (dInt32 i = 0; i < size; i++) 
	{
		dInt32 j;
		dInt32 iter = 0;
		do 
		{
			for (j = i; j < size - 1; j++) 
			{
  				T dd(dAbs(eigenValues[j]) + dAbs(eigenValues[j + 1]));
				if (dAbs(offDiag[j]) <= (T(1.e-6f) * dd)) 
				{
					break;
				}
			}

			if (j != i) 
			{
				iter++;
				if (iter == 10) 
				{
					dAssert(0);
					return;
				}

				T g((eigenValues[i + 1] - eigenValues[i]) / (T(2.0f) * offDiag[i]));
				T r(dPythag(g, T(1.0f)));
				g = eigenValues[j] - eigenValues[i] + offDiag[i] / (g + dSign(r, g));
				T s(1.0f);
				T c(1.0f);
				T p(0.0f);

				dInt32 k;
				for (k = j - 1; k >= i; k--)
				{
					T f(s * offDiag[k]);
					T b(c * offDiag[k]);
					T d(dPythag(f, g));
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
T dConditionNumber(const dInt32 size, const dInt32 stride, const T* const choleskyMatrix)
{
	T* const eigenValues = dAlloca(T, size);
	dEigenValues(size, stride, choleskyMatrix, eigenValues);

	T minVal = T(1.0e20f);
	T maxVal = T(-1.0e20f);
	for (dInt32 i = 0; i < size; i++) 
	{
		minVal = dMin(minVal, eigenValues[i]);
		maxVal = dMax(maxVal, eigenValues[i]);
	}
	T condition = T(dAbs(maxVal) / dAbs(minVal));
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
void dGaussSeidelLcpSor(const dInt32 size, const T* const matrix, T* const x, const T* const b, const T* const low, const T* const high, T tol2, dInt32 maxIterCount, dInt16* const clipped, T sor)
{
	const T* const me = matrix;
	T* const invDiag1 = dAlloca(T, size);

	dInt32 stride = 0;
	for (dInt32 i = 0; i < size; i++) 
	{
		x[i] = dClamp(T(0.0f), low[i], high[i]);
		invDiag1[i] = T(1.0f) / me[stride + i];
		stride += size;
	}

	T tolerance(tol2 * 2.0f);
	const T* const invDiag = invDiag1;
#ifdef _DEBUG 
	dInt32 passes = 0;
#endif
	for (dInt32 i = 0; (i < maxIterCount) && (tolerance > tol2); i++) 
	{
		dInt32 base = 0;
		tolerance = T(0.0f);
#ifdef _DEBUG 
		passes++;
#endif
		for (dInt32 j = 0; j < size; j++) 
		{
			const T* const row = &me[base];
			T r(b[j] - dDotProduct(size, row, x));
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
void dGaussSeidelLcpSor(const dInt32 size, const dInt32 stride, const T* const matrix, T* const x, const T* const b, const dInt32* const normalIndex, const T* const low, const T* const high, T tol2, dInt32 maxIterCount, T sor)
{
	const T* const me = matrix;
	T* const invDiag1 = dAlloca(T, size);
	T* const u = dAlloca(T, size + 1);
	dInt32* const index = dAlloca(dInt32, size);

	u[size] = T(1.0f);
	dInt32 rowStart = 0;
	for (dInt32 j = 0; j < size; j++) 
	{
		u[j] = x[j];
		index[j] = normalIndex[j] ? j + normalIndex[j] : size;
	}

	for (dInt32 j = 0; j < size; j++) 
	{
		const T val = u[index[j]];
		const T l = low[j] * val;
		const T h = high[j] * val;
		u[j] = dClamp(u[j], l, h);
		invDiag1[j] = T(1.0f) / me[rowStart + j];
		rowStart += stride;
	}

	T tolerance(tol2 * 2.0f);
	const T* const invDiag = invDiag1;
	const dInt32 maxCount = dMax(8, size);
	for (dInt32 i = 0; (i < maxCount) && (tolerance > tol2); i++) 
	{
		dInt32 base = 0;
		tolerance = T(0.0f);
		for (dInt32 j = 0; j < size; j++) 
		{
			const T* const row = &me[base];
			T r(b[j] - dDotProduct(size, row, u));
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
	dInt32 passes = 0;
#endif
	for (dInt32 i = 0; (i < maxIterCount) && (tolerance > tol2); i++) 
	{
		dInt32 base = 0;
		tolerance = T(0.0f);
#ifdef _DEBUG 
		passes++;
#endif
		for (dInt32 j = 0; j < size; j++) 
		{
			const T* const row = &me[base];
			T r(b[j] - dDotProduct(size, row, u));
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

	for (dInt32 j = 0; j < size; j++) 
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
void dGaussSeidelLCP(const dInt32 size, const T* const matrix, T* const x, const T* const b, const T* const low, const T* const high, T sor = T(1.2f))
{
	dInt16* const clipped = dAlloca(dInt16, size);
	dGaussSeidelLcpSor(size, matrix, x, b, low, high, T(1.0e-3f), size * size, clipped, sor);
}

template<class T>
void dPermuteRows(dInt32 size, dInt32 i, dInt32 j, T* const matrix, T* const choleskyMatrix, T* const x, T* const r, T* const low, T* const high, dInt16* const permute)
{
	if (i != j) 
	{
		T* const A = &matrix[size * i];
		T* const B = &matrix[size * j];
		T* const invA = &choleskyMatrix[size * i];
		T* const invB = &choleskyMatrix[size * j];
		for (dInt32 k = 0; k < size; k++) 
		{
			dSwap(A[k], B[k]);
			dSwap(invA[k], invB[k]);
		}

		dInt32 stride = 0;
		for (dInt32 k = 0; k < size; k++) 
		{
			dSwap(matrix[stride + i], matrix[stride + j]);
			stride += size;
		}

		dSwap(x[i], x[j]);
		dSwap(r[i], r[j]);
		dSwap(low[i], low[j]);
		dSwap(high[i], high[j]);
		dSwap(permute[i], permute[j]);
	}
}

template<class T>
void dCalculateDelta_x(dInt32 size, dInt32 n, const T* const matrix, const T* const choleskyMatrix, T* const delta_x)
{
	const T* const row = &matrix[size * n];
	for (dInt32 i = 0; i < n; i++) 
	{
		delta_x[i] = -row[i];
	}
	dSolveCholesky(size, n, choleskyMatrix, delta_x, delta_x);
	delta_x[n] = T(1.0f);
}

// calculate delta_r = A * delta_x
template<class T>
void dCalculateDelta_r(dInt32 size, dInt32 n, const T* const matrix, const T* const delta_x, T* const delta_r)
{
	dInt32 stride = n * size;
	const dInt32 size1 = n + 1;
	for (dInt32 i = n; i < size; i++) 
	{
		delta_r[i] = dDotProduct(size1, &matrix[stride], delta_x);
		stride += size;
	}
}

template<class T>
void dHouseholderReflection(dInt32 size, dInt32 row, dInt32 colum, T* const choleskyMatrix, T* const tmp, T* const reflection)
{
	dAssert(row <= colum);
	if (row < colum) 
	{
		for (dInt32 i = row; i <= colum; i++) 
		{
			T* const rowI = &choleskyMatrix[size * i];
			T mag2(0.0f);
			for (dInt32 j = i + 1; j <= colum; j++) 
			{
				mag2 += rowI[j] * rowI[j];
				reflection[j] = rowI[j];
			}
			if (mag2 > T(1.0e-14f)) 
			{
				reflection[i] = rowI[i] + dSign(rowI[i]) * T(sqrt(mag2 + rowI[i] * rowI[i]));

				const T vMag2(mag2 + reflection[i] * reflection[i]);
				const T den = T(2.0f) / vMag2;
				for (dInt32 j = i; j < size; j++) 
				{
					T acc(0.0f);
					T* const rowJ = &choleskyMatrix[size * j];
					for (dInt32 k = i; k <= colum; k++) {
						acc += rowJ[k] * reflection[k];
					}
					tmp[j] = acc;
				}

				for (dInt32 j = i + 1; j < size; j++) 
				{
					rowI[j] = T(0.0f);
					T* const rowJ = &choleskyMatrix[size * j];
					const T a = tmp[j] * den;
					for (dInt32 k = i; k <= colum; k++) 
					{
						rowJ[k] -= a * reflection[k];
					}
				}
				rowI[i] -= tmp[i] * reflection[i] * den;
			}

			if (rowI[i] < T(0.0f)) 
			{
				for (dInt32 k = i; k < size; k++) 
				{
					choleskyMatrix[size * k + i] = -choleskyMatrix[size * k + i];
				}
			}
		}

		for (dInt32 i = row; i < size; i++) 
		{
			choleskyMatrix[size * i + i] = dMax(choleskyMatrix[size * i + i], T(1.0e-6f));
		}
	}
}

template<class T>
void dCholeskyUpdate(dInt32 size, dInt32 row, dInt32 colum, T* const choleskyMatrix, T* const tmp, T* const reflexion, const T* const psdMatrix)
{
	const dInt32 n0 = colum - row;
	const dInt32 n1 = n0 + 1;
	const dInt32 choleskyCost = size * size * size / 3;
	const dInt32 householdCost = n0 * (n0 + 1) / 2 + n1 * (n1 + 1) * (2 * (2 * n1 + 1) - 3 + 3 * (size - colum - 1)) / 6 - 1;

	if (householdCost < choleskyCost) 
	{
		dHouseholderReflection(size, row, colum, choleskyMatrix, tmp, reflexion);
	} 
	else 
	{
		memcpy (choleskyMatrix, psdMatrix, sizeof (T) * size * size);
		dCholeskyFactorization(size, choleskyMatrix);
	}

//#if _DEBUG
#if 0
	T* const psdMatrixCopy = dAlloca(T, size * size);
	memcpy(psdMatrixCopy, psdMatrix, sizeof(T) * size * size);
	dCholeskyFactorization(size, psdMatrixCopy);

	for (dInt32 i = 0; i < size; i++) 
	{
		for (dInt32 j = 0; j < size; j++) 
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
void dSolveDantzigLcpLow(dInt32 size, T* const symmetricMatrixPSD, T* const x, T* const b, T* const low, T* const high)
{
	T* const x0 = dAlloca(T, size);
	T* const r0 = dAlloca(T, size);
	T* const tmp0 = dAlloca(T, size);
	T* const tmp1 = dAlloca(T, size);
	T* const delta_r = dAlloca(T, size);
	T* const delta_x = dAlloca(T, size);
	T* const lowerTriangularMatrix = dAlloca(T, size * size);
	dInt16* const permute = dAlloca(dInt16, size);

	for (dInt32 i = 0; i < size; i++) 
	{
		permute[i] = dInt16(i);
		x0[i] = T(0.0f);
		x[i] = dMax (b[i] * b[i], T (1.0f));
	}

	for (dInt32 n = size - 1, i = size - 1; i >= 0; i--) 
	{
		if (x[i] > T(1.0)) 
		{
			dPermuteRows(size, n, i, symmetricMatrixPSD, lowerTriangularMatrix, x, b, low, high, permute);
			n --;
		}
	}

	for (dInt32 i = size - 1; (i >= 0) && (x[i] > T(1.0f)) ; i--) 
	{
		dInt32 min = i;
		for (dInt32 j = i - 1; (j >= 0) && (x[j] > T(1.0f)); j--) 
		{
			if (x[j] > x[min]) 
			{
				min = j;
			}
		}
		if (min != i) 
		{
			dPermuteRows(size, i, min, symmetricMatrixPSD, lowerTriangularMatrix, x, b, low, high, permute);
		}
	}

	dInt32 initialGuessCount = size;
	while (x[initialGuessCount - 1] >= T(16.0f)) 
	{
		initialGuessCount --;
	}
	
	memcpy(lowerTriangularMatrix, symmetricMatrixPSD, sizeof(T) * size * size);
#ifdef _DEBUG
	bool valid = dCholeskyFactorization(size, lowerTriangularMatrix);
	dAssert(valid);
#else
	dCholeskyFactorization(size, lowerTriangularMatrix);
#endif
	for (dInt32 j = 0; (j != -1) && initialGuessCount;) 
	{
		dSolveCholesky(size, initialGuessCount, lowerTriangularMatrix, x0, b);

		j = -1;
		T alpha(1.0f);
		T value(0.0f);
		for (dInt32 i = initialGuessCount - 1; i >= 0; i--) 
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
			dPermuteRows(size, j, initialGuessCount, symmetricMatrixPSD, lowerTriangularMatrix, x0, b, low, high, permute);
			dCholeskyUpdate(size, j, initialGuessCount, lowerTriangularMatrix, tmp0, tmp1, symmetricMatrixPSD);
		}
	}

	if (initialGuessCount == size) 
	{
		for (dInt32 i = 0; i < size; i++) 
		{
			dInt32 j = permute[i];
			x[j] = x0[i];
			b[i] = T(0.0f);
		}
		return;
	}

	dInt32 clampedIndex = size;
	dInt32 index = initialGuessCount;
	dInt32 count = size - initialGuessCount;
	dInt32 stride = index * size;

	for (dInt32 i = 0; i < size; i++) 
	{
		r0[i] = T(0.0f);
		delta_x[i] = T(0.0f);
		delta_r[i] = T(0.0f);
	}
	
	for (dInt32 i = index; i < size; i++) 
	{
		r0[i] = dDotProduct(size, &symmetricMatrixPSD[stride], x0) - b[i];
		stride += size;
	}


	while (count) 
	{
		bool loop = true;

		while (loop) 
		{
			loop = false;
			T clamp_x(0.0f);
			dInt32 swapIndex = -1;

			if (dAbs(r0[index]) > T(1.0e-12f)) 
			{
				dCalculateDelta_x(size, index, symmetricMatrixPSD, lowerTriangularMatrix, delta_x);
				dCalculateDelta_r(size, index, symmetricMatrixPSD, delta_x, delta_r);

				dAssert(delta_r[index] != T(0.0f));
				dAssert(dAbs(delta_x[index]) == T(1.0f));
				delta_r[index] = (delta_r[index] == T(0.0f)) ? T(1.0e-12f) : delta_r[index];

				T scale = -r0[index] / delta_r[index];
				dAssert(dAbs(scale) >= T(0.0f));

				for (dInt32 i = 0; i <= index; i++) 
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
				dAssert(dAbs(scale) >= T(0.0f));

				for (dInt32 i = clampedIndex; (i < size) && (scale > T(1.0e-12f)); i++) 
				{
					T r1 = r0[i] + scale * delta_r[i];
					if ((r1 * r0[i]) < T(0.0f)) 
					{
						dAssert(dAbs(delta_r[i]) > T(0.0f));
						T s1 = -r0[i] / delta_r[i];
						dAssert(dAbs(s1) >= T(0.0f));
						dAssert(dAbs(s1) <= dAbs(scale));
						if (dAbs(s1) < dAbs(scale)) 
						{
							scale = s1;
							swapIndex = i;
						}
					}
				}

				if (dAbs(scale) > T(1.0e-12f)) 
				{
					for (dInt32 i = 0; i < size; i++) 
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
				dPermuteRows(size, index, clampedIndex, symmetricMatrixPSD, lowerTriangularMatrix, x0, r0, low, high, permute);
				dCholeskyUpdate(size, index, clampedIndex, lowerTriangularMatrix, tmp0, tmp1, symmetricMatrixPSD);
				loop = count ? true : false;
			} 
			else if (swapIndex > index) 
			{
				loop = true;
				r0[swapIndex] = T(0.0f);
				dAssert(swapIndex < size);
				dAssert(clampedIndex <= size);
				if (swapIndex < clampedIndex) 
				{
					count--;
					clampedIndex--;
					dPermuteRows(size, clampedIndex, swapIndex, symmetricMatrixPSD, lowerTriangularMatrix, x0, r0, low, high, permute);
					dCholeskyUpdate(size, swapIndex, clampedIndex, lowerTriangularMatrix, tmp0, tmp1, symmetricMatrixPSD);
					dAssert(clampedIndex >= index);
				} 
				else 
				{
					count++;
					dAssert(clampedIndex < size);
					dPermuteRows(size, clampedIndex, swapIndex, symmetricMatrixPSD, lowerTriangularMatrix, x0, r0, low, high, permute);
					dCholeskyUpdate(size, clampedIndex, swapIndex, lowerTriangularMatrix, tmp0, tmp1, symmetricMatrixPSD);
					clampedIndex++;
					dAssert(clampedIndex <= size);
					dAssert(clampedIndex >= index);
				}
			} 
			else 
			{
				dAssert(index > 0);
				x0[swapIndex] = clamp_x;
				delta_x[index] = T(0.0f);

				dAssert(swapIndex < index);
				dPermuteRows(size, swapIndex, index - 1, symmetricMatrixPSD, lowerTriangularMatrix, x0, r0, low, high, permute);
				dPermuteRows(size, index - 1, index, symmetricMatrixPSD, lowerTriangularMatrix, x0, r0, low, high, permute);
				dPermuteRows(size, clampedIndex - 1, index, symmetricMatrixPSD, lowerTriangularMatrix, x0, r0, low, high, permute);
				dCholeskyUpdate (size, swapIndex, clampedIndex - 1, lowerTriangularMatrix, tmp0, tmp1, symmetricMatrixPSD);

				clampedIndex--;
				index--;
				loop = true;
			}
		}
	}

	for (dInt32 i = 0; i < size; i++) 
	{
		dInt32 j = permute[i];
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
bool dSolveDantzigLCP(dInt32 size, T* const symetricMatrix, T* const x, T* const b, T* const low, T* const high)
{
	T* const choleskyMatrix = dAlloca(T, size * size);
	dCheckAligment(choleskyMatrix);

	memcpy (choleskyMatrix, symetricMatrix, sizeof (T) * size * size);
	dCholeskyFactorization(size, choleskyMatrix);
	for (dInt32 i = 0; i < size; i ++) 
	{
		T* const row = &choleskyMatrix[i * size];
		for (dInt32 j = i + 1; j < size; j ++) 
		{
			row[j] = T(0.0f);
		}
	}
	return dSolveDantzigLCP(size, symetricMatrix, choleskyMatrix, x, b, low, high);
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
bool dSolvePartitionDantzigLCP(dInt32 size, T* const symmetricMatrixPSD , T* const x, T* const b, T* const low, T* const high)
{
	dInt16* const permute = dAlloca(dInt16, size);

	for (dInt32 i = 0; i < size; i++) 
	{
		x[i] = b[i];
		permute[i] = dInt16(i);
	}

	dInt32 unboundedSize = size;
	for (dInt32 i = 0; i < unboundedSize; i++) 
	{
		if ((low[i] <= T(-D_LCP_MAX_VALUE)) && (high[i] >= T(D_LCP_MAX_VALUE))) 
		{
			dCholeskyFactorizationAddRow(size, i, symmetricMatrixPSD );
		} 
		else 
		{
			dInt32 j = unboundedSize - 1;
			if (i != j) 
			{
				T* const A = &symmetricMatrixPSD [size * i];
				T* const B = &symmetricMatrixPSD [size * j];
				for (dInt32 k = 0; k < size; k++) 
				{
					dSwap(A[k], B[k]);
				}

				dInt32 stride = 0;
				for (dInt32 k = 0; k < size; k++) 
				{
					dSwap(symmetricMatrixPSD [stride + i], symmetricMatrixPSD [stride + j]);
					stride += size;
				}
				dSwap(x[i], x[j]);
				dSwap(b[i], b[j]);
				dSwap(low[i], low[j]);
				dSwap(high[i], high[j]);
				dSwap(permute[i], permute[j]);
			}

			i--;
			unboundedSize--;
		}
	}

	bool ret = false;
	if (unboundedSize > 0) 
	{
		dSolveCholesky(size, unboundedSize, symmetricMatrixPSD , x);
		dInt32 base = unboundedSize * size;
		for (dInt32 i = unboundedSize; i < size; i++) 
		{
			b[i] -= dDotProduct(unboundedSize, &symmetricMatrixPSD[base], x);
			base += size;
		}

		const dInt32 boundedSize = size - unboundedSize;
		T* const l = dAlloca(T, boundedSize);
		T* const h = dAlloca(T, boundedSize);
		T* const c = dAlloca(T, boundedSize);
		T* const u = dAlloca(T, boundedSize);
		T* const a11 = dAlloca(T, boundedSize * boundedSize);
		T* const a10 = dAlloca(T, boundedSize * unboundedSize);

		for (dInt32 i = 0; i < boundedSize; i++) 
		{
			T* const g = &a10[i * unboundedSize];
			const T* const row = &symmetricMatrixPSD [(unboundedSize + i) * size];
			for (dInt32 j = 0; j < unboundedSize; j++) 
			{
				g[j] = -row[j];
			}
			dSolveCholesky(size, unboundedSize, symmetricMatrixPSD, g);

			T* const arow = &a11[i * boundedSize];
			const T* const row2 = &symmetricMatrixPSD[(unboundedSize + i) * size];
			arow[i] = row2[unboundedSize + i] + dDotProduct(unboundedSize, g, row2);
			for (dInt32 j = i + 1; j < boundedSize; j++) 
			{
				const T* const row1 = &symmetricMatrixPSD [(unboundedSize + j) * size];
				T elem = row1[unboundedSize + i] + dDotProduct(unboundedSize, g, row1);
				arow[j] = elem;
				a11[j * boundedSize + i] = elem;
			}
			u[i] = T(0.0f);
			c[i] = b[i + unboundedSize];
			l[i] = low[i + unboundedSize];
			h[i] = high[i + unboundedSize];
		}

		if (dSolveDantzigLCP(boundedSize, a11, u, c, l, h)) 
		{
			for (dInt32 i = 0; i < boundedSize; i++) 
			{
				const T s = u[i];
				x[unboundedSize + i] = s;
				const T* const g = &a10[i * unboundedSize];
				for (dInt32 j = 0; j < unboundedSize; j++) 
				{
					x[j] += g[j] * s;
				}
			}
			ret = true;
		}
	} 
	else 
	{
		for (dInt32 i = 0; i < size; i++) 
		{
			x[i] = T(0.0f);
		}
		ret = dSolveDantzigLCP(size, symmetricMatrixPSD, x, b, low, high);
	}

	for (dInt32 i = 0; i < size; i++) 
	{
		b[i] = x[i];
	}
	for (dInt32 i = 0; i < size; i++) 
	{
		dInt32 j = permute[i];
		x[j] = b[i];
		b[i] = T(0.0f);
	}
	return ret;
}

template <class T>
void dSolveDantzigLCP(dInt32 size, T* const symmetricMatrixPSD, T* const x, T* const b, T* const low, T* const high)
{
	T tol2 = T(0.25f * 0.25f);
	dInt32 passes = dClamp(size, 12, 20);
	T* const r = dAlloca(T, size);
	dInt16* const clipped = dAlloca(dInt16, size);

	// find an approximation to the solution
	dGaussSeidelLcpSor(size, symmetricMatrixPSD, x, b, low, high, tol2, passes, clipped, T(1.3f));

	T err2(0.0f);
	dInt32 stride = 0;
	dInt32 clippeCount = 0;
	for (dInt32 i = 0; i < size; i++) 
	{
		const T* const row = &symmetricMatrixPSD[stride];
		r[i] = b[i] - dDotProduct(size, row, x);
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
			T* const x0 = dAlloca(T, size);
			for (dInt32 i = 0; i < size; i++) 
			{
				low[i] -= x[i];
				high[i] -= x[i];
			}
			dSolveDantzigLcpLow(size, symmetricMatrixPSD, x0, r, low, high);
			for (dInt32 i = 0; i < size; i++) 
			{
				x[i] += x0[i];
			}
		} 
		else 
		{
			// larger lcp are too hard for direct method, see if we can get better approximation
			dGaussSeidelLcpSor(size, symmetricMatrixPSD, x, b, low, high, tol2, 20, clipped, T(1.3f));
		}
	}
}

#endif
