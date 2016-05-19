/* Copyright (c) <2003-2011> <Julio Jerez, Newton Game Dynamics>
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

#ifndef __dgSPDMatrix__
#define __dgSPDMatrix__

#include "dgStdafx.h"
#include "dgDebug.h"
#include "dgGeneralMatrix.h"

#define DG_LCP_MAX_VALUE dgFloat32 (1.0e12f)

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


template<class T, dgInt32 Size>
class dgSPDMatrix: public dgSquareMatrix<T, Size>
{
	public:
	dgSPDMatrix() {}
	~dgSPDMatrix() {}

	bool CholeskyFactorization();
	void CholeskySolve(dgGeneralVector<T, Size> &x, const dgGeneralVector<T, Size> &b) const;
	bool TestPSD() const;
	
//	bool LDLtDecomposition();
//	bool CholeskyFactorization(dgInt32 n);

	protected:
	bool CholeskyFactorizationAddRow(dgInt32 n);
};


// ***********************************************************************************************
//
//   LinearSystem
//
// ***********************************************************************************************
/* 

template<class T, dgInt32 Size>
bool dgSPDMatrix<T, Size>::LDLtDecomposition()
{
	for (dgInt32 j = 0; j < dgGeneralMatrix<T, Size>::m_rowCount; j++) {
		T* const rowJ = &dgGeneralMatrix<T, Size>::m_rows[j].m_columns[0];
		for (dgInt32 k = 0; k < j; k++) {
			T* const rowK = &dgGeneralMatrix<T, Size>::m_rows[k].m_columns[0];
			T factor = rowK[j];
			for (dgInt32 i = j; i < dgGeneralMatrix<T, Size>::m_rowCount; i++) {
				rowJ[i] -= rowK[i] * factor;
			}
		}

		T factor = rowJ[j];
		if (factor <= T(0.0f)) {
			return false;
		}

		rowJ[j] = T(sqrt(factor));
		factor = T(1.0f / rowJ[j]);
		for (dgInt32 k = j + 1; k < dgGeneralMatrix<T, Size>::m_rowCount; k++) {
			rowJ[k] *= factor;
		}
	}
	return true;
}

*/

template<class T, dgInt32 Size>
void dgSPDMatrix<T, Size>::CholeskySolve(dgGeneralVector<T, Size>& x, const dgGeneralVector<T, Size> &b) const
{
	for (dgInt32 i = 0; i < Size; i++) {
		T acc (0.0f);
		const dgGeneralVector<T, Size>& row = dgGeneralMatrix<T, Size, Size>::m_rows[i];
		for (dgInt32 j = 0; j < i; j++) {
			acc = acc + row[j] * x[j];
		}
		x[i] = (b[i] - acc) / row[i];
	}

	for (dgInt32 i = Size - 1; i >= 0; i--) {
		T acc = 0.0f;
		for (dgInt32 j = i + 1; j < Size; j++) {
			acc = acc + dgGeneralMatrix<T, Size, Size>::m_rows[j][i] * x[j];
		}
		x[i] = (x[i] - acc) / dgGeneralMatrix<T, Size, Size>::m_rows[i][i];
	}
}


template<class T, dgInt32 Size>
bool dgSPDMatrix<T, Size>::CholeskyFactorizationAddRow(dgInt32 n)
{
	dgGeneralVector<T, Size>& rowN = dgGeneralMatrix<T, Size, Size>::m_rows[n];
	for (dgInt32 j = 0; j <= n; j++) {
		T s(0.0f);
		const dgGeneralVector<T, Size>& rowJ = dgGeneralMatrix<T, Size, Size>::m_rows[j];
		for (dgInt32 k = 0; k < j; k++) {
			s += rowN[k] * rowJ[k];
		}

		if (n == j) {
			T diag = rowN[n] - s;
			if (diag < T(0.0f)) {
				dgAssert(0);
				return false;
			}
			rowN[n] = T(sqrt(diag));
		} else {
			rowN[j] = (T(1.0f) / rowJ[j] * (rowN[j] - s));
		}
	}
	return true;
}


// calculate Cholesky factorization of the n first rows
template<class T, dgInt32 Size>
bool dgSPDMatrix<T, Size>::CholeskyFactorization()
{
	bool passed = true;
	for (dgInt32 i = 0; passed && (i < Size); i++) {
		passed = CholeskyFactorizationAddRow(i);
	}
	return passed;
}


template<class T, dgInt32 Size>
bool dgSPDMatrix<T, Size>::TestPSD() const
{
	dgSPDMatrix<T, Size> tmp(*this);
	return tmp.CholeskyDecomposition();
}


#if 0

// solve a general Linear complementary program 
// A * x = b + r
// subjected to constraints
// x(i) = low(i),  if r(i) >= 0  
// x(i) = high(i), if r(i) <= 0  
// low(i) <= x(i) <= high(i),  if r(i) == 0  
template<class T, dgInt32 Size>
class dgLCP: public dgSPDMatrix<T, Size>
{
	public:
	
	dgLCP() {}
	~dgLCP() {}

	dgGeneralVector<T, Size>& GetB() { return m_b;}
	dgGeneralVector<T, Size>& GetX() { return m_x_out;}
	dgGeneralVector<T, Size>& GetLowLimit() { return m_low;}
	dgGeneralVector<T, Size>& GetHightLimit() { return m_high;}
	const dgGeneralVector<T, Size>& GetR() { return m_r_out;}	

	// Using George B. Dantzig algorithm.  
	// Inspired from David Baraff interpretation of George B. Dantzig algorithm in his paper. 
	// Fast Contact Force Computation for non penetrating Rigid Bodies, http://www.cs.cmu.edu/~baraff/papers/sig94.pdf
	// but not quite exactly the same. 
	// on exit matrix is all input data is destroyed 
	// return true if the system has a solution
	bool SolveDantzig();
/*
	// Using Conjugate Gradient Descend method
	bool SolveConjugateGradient(T tol = T(1.0e-5f));

	// Using Projected Gauss Seidel
	bool GaussSeidelLCP(dgInt32 maxIterCount, T tol = T(1.0e-5f));

	static dgInt32 CalculateMemorySize(dgInt32 size)
	{
		return dgSPDMatrix<T, Size>::CalculateMemorySize(size) + 5 * dgGeneralVector<T, Size>::CalculateMemorySize(size) + 6 * (size * sizeof (T)+sizeof (dgVector));
	}

	private:
	DG_INLINE T PartialDotProduct(const T* const a, const T* const b, dgInt32 size) const;
	DG_INLINE void PartialMatrixTimeVector(const T* const v, T* const out, dgInt32 size) const;
	DG_INLINE void PartialScaleAdd(T* const a, const T* const b, T scale, const T* const c, dgInt32 size) const;
*/
	DG_INLINE void CalculateDelta_r (dgInt32 n);
	DG_INLINE void CalculateDelta_x(T dir, dgInt32 n);
	DG_INLINE void PermuteRows(dgInt32 i, dgInt32 j);
	DG_INLINE void CholeskyRestore(dgInt32 i, dgInt32 size);

	private:
	dgGeneralVector<T, Size> m_b;
	dgGeneralVector<T, Size> m_x_out;
	dgGeneralVector<T, Size> m_r_out;
	dgGeneralVector<T, Size> m_low;
	dgGeneralVector<T, Size> m_high;
	dgGeneralVector<T, Size> m_r;
	dgGeneralVector<T, Size> m_x;
	dgGeneralVector<T, Size> m_delta_r;
	dgGeneralVector<T, Size> m_delta_x;
	dgGeneralVector<T, Size> m_diagonal;
	dgGeneralVector<T, Size> m_tmp;
	dgInt16 m_permute[Size];
};


template<class T, dgInt32 Size>
bool dgLCP<T, Size>::GaussSeidelLCP(dgInt32 maxIterCount, T tol)
{
	const dgInt32 count = dgGeneralMatrix<T, Size>::GetRowCount();
	const dgSPDMatrix<T, Size>& me = *this;

	T* const x = &m_x[0];
	T* const invDiag1 = &m_tmp[2][0];
	const T* const b = &m_b[0];
	const T* const low = &m_low[0];
	const T* const high = &m_high[0];

	for (dgInt32 i = 0; i < count; i++) {
		x[i] = dgClamp(x[i], low[i], high[i]);
		invDiag1[i] = T(1.0f) / me[i][i];
	}

	T tolerance(1.0e6f);

	T tol2 = tol * tol;
	const T* const invDiag = invDiag1;
	for (dgInt32 i = 0; (i < maxIterCount) && (tolerance > tol2); i++) {
		tolerance = T(0.0f);
		for (dgInt32 j = 0; j < count; j++) {
			const dgGeneralVector<T, Size>& row = me[j];
			T a = b[j] - row.DotProduct(x);
			T x1 = (a + row[j] * x[j]) * invDiag[j];
			if (x1 > high[j]) {
				x1 = high[j];
			} else if (x1 < low[j]) {
				x1 = low[j];
			} else {
				tolerance += a * a;
			}
			x[j] = x1;
		}
	}
	return tolerance < tol2;
}


template<class T, dgInt32 Size>
bool dgLCP<T, Size>::SolveConjugateGradient(T tol)
{
     dgSPDMatrix<T, Size>& me = *this;
     const dgInt32 size = dgGeneralMatrix<T, Size>::m_rowCount;

static int xxx;
xxx ++;
dgLCP<T, Size> gauss(*this);
//gauss.GaussSeidelLCP(100000, T(1.0e-6f));
gauss.SolveDantzig();
dgTrace(("dtz %d :", xxx));
gauss.GetX().Trace();


     T* const r = &m_tmp[0][0];
     T* const x = &m_tmp[1][0];
     T* const delta_r = &m_tmp[2][0];
     T* const delta_x = &m_tmp[3][0];
     T* const MinvR0 = &m_tmp[4][0];
     T* const invDiag = &m_tmp[5][0];
     dgInt16* const permute = m_permute;

     T* const b = &m_b[0];
     T* const x_out = &m_x[0];
     T* const r_out = &m_r[0];
     T* const low = &m_low[0];
     T* const high = &m_high[0];

     for (dgInt32 i = 0; i < size; i++) {
           x[i] = dgClamp(x_out[i], low[i], high[i]);
           permute[i] = dgInt16(i);
           invDiag[i] = T(1.0f) / me[i][i];
     }

     PartialMatrixTimeVector(x, delta_r, size);
     for (dgInt32 i = 0; i < size; i++) {
           r[i] = b[i] - delta_r[i];
           delta_x[i] = invDiag[i] * r[i];
     }

     dgInt32 iter(size);
     dgInt32 index = size;

     const T tol2 = tol * tol;
     T num = PartialDotProduct(r, delta_x, index);
     while (iter && (num > tol2)) {
           iter--;
           PartialMatrixTimeVector(delta_x, delta_r, index);
           T den(PartialDotProduct(delta_r, delta_x, index));

           dgAssert(den > T(0.0f));
           T alpha = num / den;

           T clamp_x(0.0f);
           dgInt32 swapIndex = -1;
           for (dgInt32 i = 0; (i < index) && (alpha > T(1.0e-15f)); i++) {
                T x1 = x[i] + alpha * delta_x[i];
                if (x1 > high[i]) {
                     swapIndex = i;
                     clamp_x = high[i];
                     dgAssert((high[i] - x[i]) / delta_x[i] <= alpha);
                     alpha = (high[i] - x[i]) / delta_x[i];
                } else if (x1 < low[i]) {
                     swapIndex = i;
                     clamp_x = low[i];
                     dgAssert((low[i] - x[i]) / delta_x[i] <= alpha);
                     alpha = (low[i] - x[i]) / delta_x[i];
                }
           }

           if (alpha > T(1.0e-15f)) {
                CalculateDelta_r(delta_r, delta_x, index);
                for (dgInt32 i = index; (i < size) && (alpha > T(1.0e-15f)); i++) {
                     T r1 = r[i] - alpha * delta_r[i];
                     if ((r1 * r[i]) < T(0.0f)) {
                           dgAssert(T(fabs(delta_r[i]) > T(0.0f)));
                           T alpha1 = r[i] / delta_r[i];
                           dgAssert(alpha1 >= T(0.0f));
                           if (alpha1 < alpha) {
                                alpha = alpha1;
                                swapIndex = i;
                           }
                     }
                }
           }

           if (alpha > T(1.0e-15f)) {
                for (dgInt32 i = 0; i < size; i++) {
                     x[i] += alpha * delta_x[i];
                     r[i] -= alpha * delta_r[i];
                }
           }

           if (swapIndex == -1) {
                T num1(0.0f);
                for (dgInt32 i = 0; i < index; i++) {
                     MinvR0[i] = invDiag[i] * r[i];
                     num1 += MinvR0[i] * r[i];
                }
                T beta(num1 / num);
                PartialScaleAdd(delta_x, MinvR0, beta, delta_x, index);
                num = PartialDotProduct(r, MinvR0, index);
           } else if (swapIndex >= index) {
                dgAssert (0);
           } else {
                index--;
                iter = index;
                x[swapIndex] = clamp_x;
                if (swapIndex != index) {
                     dgGeneralMatrix<T, Size>::SwapRows(swapIndex, index);
                     dgGeneralMatrix<T, Size>::SwapColumns(swapIndex, index);
                     dgSwap(x[swapIndex], x[index]);
                     dgSwap(low[swapIndex], low[index]);
                     dgSwap(high[swapIndex], high[index]);
                     dgSwap(invDiag[swapIndex], invDiag[index]);
                     dgSwap(permute[swapIndex], permute[index]);
                }
                delta_x[index] = T(0.0f);

                PartialMatrixTimeVector(x, delta_r, index);
                for (dgInt32 i = 0; i < index; i++) {
                     r[i] = b[i] - delta_r[i];
                     delta_x[i] = invDiag[i] * r[i];
                }
                num = PartialDotProduct(r, delta_x, index);
           }
/*
           } else if (swapIndex < count) {
                count--;
                iter = count;
                x[swapIndex] = clamp_x;
                if (swapIndex != count) {
                     SwapRows(swapIndex, count);
                     SwapColumns(swapIndex, count);
                     dgSwap(x[swapIndex], x[count]);
                     dgSwap(low[swapIndex], low[count]);
                     dgSwap(high[swapIndex], high[count]);
                     dgSwap(invDiag[swapIndex], invDiag[count]);
                     dgSwap(permute[swapIndex], permute[count]);
                }

                delta_x[count] = T(0.0f);
                PartialMatrixTimeVector(x, delta_r, count);
                for (dgInt32 i = 0; i < count; i++) {
                     r[i] = b[i] - delta_r[i];
                     delta_x[i] = invDiag[i] * r[i];
                }
                num = PartialDotProduct(r, delta_x, count);
           }
           else {
                if (swapIndex != count) {
                     SwapRows(swapIndex, count);
                     SwapColumns(swapIndex, count);
                     dgSwap(x[swapIndex], x[count]);
                     dgSwap(low[swapIndex], low[count]);
                     dgSwap(high[swapIndex], high[count]);
                     dgSwap(invDiag[swapIndex], invDiag[count]);
                     dgSwap(permute[swapIndex], permute[count]);
                }
                count++;
                iter = count;
                PartialMatrixTimeVector(x, delta_r, count);
                for (dgInt32 i = 0; i < count; i++) {
                     r[i] = b[i] - delta_r[i];
                     delta_x[i] = invDiag[i] * r[i];
                }
                //              r[count] = T(0.0f);
                num = PartialDotProduct(r, delta_x, count);
           }
*/
     }
     for (dgInt32 i = 0; i < size; i++) {
           x_out[permute[i]] = x[i];
           r_out[permute[i]] = r[i];
     }

dgTrace(("cgr %d :", xxx));
m_x.Trace();
m_x.Copy(gauss.GetX());

     return num < tol2;
}
#endif



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
DG_INLINE void dgMatrixTimeVector(dgInt32 size, const T* const matrix, const T* const v, T* const out)
{
	dgInt32 stride = 0;
	for (dgInt32 i = 0; i < size; i++) {
		out[i] = dgDotProduct(size, &matrix[stride], v);
		stride += size;
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
			if (diag < T(0.0f)) {
				dgAssert(0);
				return false;
			}
			rowN[n] = T(sqrt(diag));
		} else {
			rowN[j] = (T(1.0f) / rowJ[j] * (rowN[j] - s));
		}

		stride += size;
	}

	return true;
}


template<class T>
DG_INLINE void dgCholeskyRestore(dgInt32 size, T* const matrix, const T* const diagonal, dgInt32 n, dgInt32 subSize)
{
	dgInt32 stride = n * size;
	for (dgInt32 i = n; i < subSize; i++) {
		T* const row = &matrix[stride];
		row[i] = diagonal[i];
		for (dgInt32 j = 0; j < i; j++) {
			row[j] = matrix[size * j + i];
		}
		stride += size;
	}
}


template<class T>
DG_INLINE void dgCholeskySolve(dgInt32 size, const T* const matrix, T* const x, dgInt32 n)
{
	dgInt32 stride = 0;
	for (dgInt32 i = 0; i < n; i++) {
		T acc(0.0f);
		const T* const row = &matrix[stride];
		for (dgInt32 j = 0; j < i; j++) {
			acc = acc + row[j] * x[j];
		}
		x[i] = (x[i] - acc) / row[i];
		stride += size;
	}

	for (dgInt32 i = n - 1; i >= 0; i--) {
		T acc = 0.0f;
		for (dgInt32 j = i + 1; j < n; j++) {
			acc = acc + matrix[size * j + i] * x[j];
		}
		x[i] = (x[i] - acc) / matrix[size * i + i];
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
		delta_x[i] = row[i] * dir;
	}
	dgCholeskySolve(size, matrix, delta_x, n);
	delta_x[n] = -dir;
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
// note: although the system is called LCP, the solve is far more general than a strict LCP
// to solve a strict LCP set the following
// low(i) = 0
// high(i) = infinity.
// this the same as enforcing the constrain: x(i) * r(i) = 0
template <class T>
bool dgSolveDantzigLCP(dgInt32 size, T* const matrix, T* const x, T* const b, T* const low, T* const high)
{
	T* const x0 = dgAlloca(T, size);
	T* const r0 = dgAlloca(T, size);
	T* const delta_r = dgAlloca(T, size);
	dgInt16* const permute = dgAlloca(short, size);

	T* const delta_x = b;
	T* const diagonal = x;

	dgInt32 stride = 0;
	for (dgInt32 i = 0; i < size; i++) {
		x0[i] = dgClamp(x[i], low[i], high[i]);
		permute[i] = short(i);
		diagonal[i] = matrix[stride + i];
		stride += size;
	}

	dgMatrixTimeVector(size, matrix, x0, r0);
	for (dgInt32 i = 0; i < size; i++) {
		r0[i] -= b[i];
	}

	dgInt32 index = 0;
	dgInt32 count = size;

	for (dgInt32 i = 0; i < count; i++) {
		if ((low[i] <= T(-DG_LCP_MAX_VALUE)) && (high[i] >= T(DG_LCP_MAX_VALUE))) {
			dgCholeskyFactorizationAddRow(size, index, matrix);
			index++;
		} else {
			dgPermuteRows(size, i, count - 1, matrix, x0, r0, low, high, diagonal, permute);
			i--;
			count--;
		}
	}

	if (index > 0) {
		dgCholeskySolve(size, matrix, r0, index);
		for (dgInt32 i = 0; i < index; i++) {
			x0[i] -= r0[i];
			r0[i] = T(0.0f);
		}
		dgCalculateDelta_r(size, index, matrix, x0, delta_r);
		for (dgInt32 i = index; i < size; i++) {
			r0[i] += delta_r[i];
		}
	}
	count = size - index;
	for (dgInt32 i = 0; i < size; i++) {
		delta_x[i] = T(0.0f);
		delta_r[i] = T(0.0f);
	}

	const dgInt32 start = index;
	dgInt32 clampedIndex = size;
	while (count) {
		bool loop = true;
		bool calculateDelta_x = true;

		while (loop) {
			loop = false;
			T clamp_x(0.0f);
			dgInt32 swapIndex = -1;

			if (dgAbsf(r0[index]) > T(1.0e-12f)) {

				if (calculateDelta_x) {
					T dir = dgSign(r0[index]);
					dgCalculateDelta_x(size, dir, index, matrix, delta_x);
				}

				calculateDelta_x = true;
				dgCalculateDelta_r(size, index, matrix, delta_x, delta_r);
				dgAssert(delta_r[index] != T(0.0f));
				dgAssert(dgAbsf(delta_x[index]) == T(1.0f));

				T s = -r0[index] / delta_r[index];
				dgAssert(s >= T(0.0f));

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
				dgAssert(s >= T(0.0f));

				for (dgInt32 i = clampedIndex; (i < size) && (s > T(1.0e-12f)); i++) {
					T r1 = r0[i] + s * delta_r[i];
					if ((r1 * r0[i]) < T(0.0f)) {
						dgAssert(dgAbsf(delta_r[i]) > T(0.0f));
						T s1 = -r0[i] / delta_r[i];
						dgAssert(s1 >= T(0.0f));
						if (s1 < s) {
							s = s1;
							swapIndex = i;
						}
					}
				}

				if (s > T(1.0e-12f)) {
					for (dgInt32 i = 0; i < size; i++) {
						dgAssert((x0[i] + T(1.0e-4f)) >= low[i]);
						dgAssert((x0[i] - T(1.0e-4f)) <= high[i]);
						x0[i] += s * delta_x[i];
						r0[i] += s * delta_r[i];
						dgAssert((x0[i] + T(1.0e-4f)) >= low[i]);
						dgAssert((x0[i] - T(1.0e-4f)) <= high[i]);
					}
				}
			}

			if (swapIndex == -1) {
				r0[index] = T(0.0f);
				delta_r[index] = T(0.0f);
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
				r0[swapIndex] = T(0.0f);
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
				delta_x[index] = T(0.0f);

				dgCholeskyRestore(size, matrix, diagonal, swapIndex, index);
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
		x[j] = x0[i];
		b[j] = r0[i];
	}

	return true;
}


#endif