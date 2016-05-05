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
//	dgSPDMatrix(const dgSPDMatrix<T, Size>& src);

	~dgSPDMatrix() {}

	void CholeskySolve(dgGeneralVector<T, Size> &x, const dgGeneralVector<T, Size> &b, dgInt32 n) const;
/*
	bool TestPSD() const;
	bool LDLtDecomposition();
	bool CholeskyFactorization();
	

	protected:
	DG_INLINE bool CholeskyFactorization(dgInt32 n);
	DG_INLINE bool CholeskyFactorizationAddRow(dgInt32 n);
	DG_INLINE void CholeskySolve(T* const x, const T* const b, dgInt32 n) const;
*/
};


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
//	dgLCP(const dgLCP& src);

	~dgLCP() {}

	dgGeneralVector<T, Size>& GetB() { return b;}
	dgGeneralVector<T, Size>& GetX() { return x_out;}
	dgGeneralVector<T, Size>& GetLowLimit() { return low;}
	dgGeneralVector<T, Size>& GetHightLimit() { return high;}
	const dgGeneralVector<T, Size>& GetR() { return r_out;}	

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
	DG_INLINE void PermuteRows(dgInt32 i, dgInt32 j);
	
	
	DG_INLINE void CholeskyRestore(T* const diagonal, dgInt32 i, dgInt32 size);
	DG_INLINE T PartialDotProduct(const T* const a, const T* const b, dgInt32 size) const;
	DG_INLINE void PartialMatrixTimeVector(const T* const v, T* const out, dgInt32 size) const;
	DG_INLINE void PartialScaleAdd(T* const a, const T* const b, T scale, const T* const c, dgInt32 size) const;
*/
	DG_INLINE void CalculateDelta_r (dgInt32 n);
	DG_INLINE void CalculateDelta_x(T dir, dgInt32 n);
	

//	dgGeneralVector<T, Size> m_b;
//	dgGeneralVector<T, Size> m_x;
//	dgGeneralVector<T, Size> m_r;
//	dgGeneralVector<T, Size> m_low;
//	dgGeneralVector<T, Size> m_high;

	private:
	dgGeneralVector<T, Size> b;
	dgGeneralVector<T, Size> x_out;
	dgGeneralVector<T, Size> r_out;
	dgGeneralVector<T, Size> low;
	dgGeneralVector<T, Size> high;
	dgGeneralVector<T, Size> r;
	dgGeneralVector<T, Size> x;
	dgGeneralVector<T, Size> delta_r;
	dgGeneralVector<T, Size> delta_x;
	dgGeneralVector<T, Size> diagonal;
	dgGeneralVector<T, Size> tmp;
	dgInt16 permute[Size];
};


#if 0
// ***********************************************************************************************
//
//   LinearSystem
//
// ***********************************************************************************************
/*
template<class T, dgInt32 Size>
dgSPDMatrix<T, Size>::dgSPDMatrix(dgMemoryAllocator* const allocator, dgInt32 size)
	:dgSquareMatrix<T, Size>(allocator, size)
{
}
*/



template<class T, dgInt32 Size>
dgSPDMatrix<T, Size>::~dgSPDMatrix()
{
}

template<class T, dgInt32 Size>
bool dgSPDMatrix<T, Size>::TestPSD() const
{
	dgSPDMatrix<T, Size> tmp(*this);
	return tmp.CholeskyDecomposition();
}


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


template<class T, dgInt32 Size>
bool dgSPDMatrix<T, Size>::CholeskyFactorization()
{
	return CholeskyFactorization(dgGeneralMatrix<T, Size>::m_rowCount);
}



template<class T, dgInt32 Size>
DG_INLINE bool dgSPDMatrix<T, Size>::CholeskyFactorizationAddRow(dgInt32 n)
{
dgAssert (0);
return false;
/*
	dgAssert(n <= dgGeneralMatrix<T, Size>::m_rowCount);
	dgGeneralMatrix<T, Size>& me = *this;
	T* const rowI = &me[n][0];
	for (dgInt32 j = 0; j <= n; j++) {
		T s(0.0f);
		T* const rowJ = &me[j][0];
		for (dgInt32 k = 0; k < j; k++) {
			s += rowI[k] * rowJ[k];
		}

		if (n == j) {
			T diag = rowI[n] - s;
			if (diag < T(0.0f)) {
				return false;
			}
			rowI[n] = T(sqrt(diag));
		} else {
			rowI[j] = (T(1.0f) / rowJ[j] * (rowI[j] - s));
		}
	}
	return true;
*/
}


// calculate Cholesky factorization of the n first rows
template<class T, dgInt32 Size>
DG_INLINE bool dgSPDMatrix<T, Size>::CholeskyFactorization(dgInt32 n)
{
	bool passed = true;
	for (dgInt32 i = 0; passed && (i < n); i++) {
		passed = CholeskyFactorizationAddRow(i);
	}
	return passed;
}


// calculate x = inv (A) * b 
template<class T, dgInt32 Size>
DG_INLINE void dgSPDMatrix<T, Size>::CholeskySolve(T* const x, const T* const b, dgInt32 n) const
{
	const dgGeneralMatrix<T, Size>& me = *this;
	for (dgInt32 i = 0; i < n; i++) {
		T acc = 0.0f;
		const T* const row = &me[i][0];
		for (dgInt32 j = 0; j < i; j++) {
			acc = acc + row[j] * x[j];
		}
		x[i] = (b[i] - acc) / row[i];
	}

	for (dgInt32 i = n - 1; i >= 0; i--) {
		T acc = 0.0f;
		for (dgInt32 j = i + 1; j < n; j++) {
			acc = acc + me[j][i] * x[j];
		}
		x[i] = (x[i] - acc) / me[i][i];
	}
}



template<class T, dgInt32 Size>
dgLCP<T, Size>::dgLCP(const dgLCP& src)
	:dgSPDMatrix<T, Size>(src)
	,m_b(src.m_b)
	,m_x(src.m_x)
	,m_r(src.m_r)
	,m_low(src.m_low)
	,m_high(src.m_high)
//	,m_permute((dgInt16*)dgGeneralMatrix<T, Size>::m_allocator->MallocLow(src.m_rowCount * sizeof (dgInt16)))
{
	dgAssert (0);
	memcpy (m_permute, src.m_permute, sizeof (m_permute)); 
//	dgInt32 n = dgInt32(sizeof(m_tmp) / sizeof (m_tmp[0]));
//	for (dgInt32 i = 0; i < n; i++) {
//		m_tmp[i] = (T*)dgGeneralMatrix<T, Size>::m_allocator->MallocLow(src.m_rowCount * sizeof (T));
//	}
}




template<class T, dgInt32 Size>
dgLCP<T, Size>::~dgLCP()
{
	dgAsser (0);
/*
	dgInt32 n = dgInt32(sizeof(m_tmp) / sizeof (m_tmp[0]));
	for (dgInt32 i = n - 1; i >= 0; i--) {
		dgGeneralMatrix<T, Size>::m_allocator->FreeLow(m_tmp[i]);
	}
	dgGeneralMatrix<T, Size>::m_allocator->FreeLow(m_permute);
*/
}


template<class T, dgInt32 Size>
DG_INLINE void dgLCP<T, Size>::PermuteRows(dgInt32 i, dgInt32 j)
{
	if (i != j) {
		dgGeneralMatrix<T, Size>::SwapRows(i, j);
		dgGeneralMatrix<T, Size>::SwapColumns(i, j);
		dgSwap(m_b[i], m_b[j]);
		dgSwap(m_low[i], m_low[j]);
		dgSwap(m_high[i], m_high[j]);
		dgSwap(m_permute[i], m_permute[j]);
		for (dgInt32 k = 0; k < 5; k++) {
			dgSwap(m_tmp[k][i], m_tmp[k][j]);
		}
	}
}






template<class T, dgInt32 Size>
DG_INLINE void dgLCP<T, Size>::CholeskyRestore(T* const diagonal, dgInt32 n, dgInt32 size)
{
	dgGeneralMatrix<T, Size>& me = *this;
	for (dgInt32 i = n; i < size; i++) {
		me[i][i] = diagonal[i];
		T* const row = &me[i][0];
		for (dgInt32 j = 0; j < i; j++) {
			row[j] = me[j][i];
		}
	}
}


template<class T, dgInt32 Size>
DG_INLINE T dgLCP<T, Size>::PartialDotProduct(const T* const a, const T* const b, dgInt32 size) const
{
	T acc(0.0f);
	for (dgInt32 i = 0; i < size; i++) {
		acc += a[i] * b[i];
	}
	return acc;
}

template<class T, dgInt32 Size>
DG_INLINE void dgLCP<T, Size>::PartialScaleAdd(T* const a, const T* const b, T scale, const T* const c, dgInt32 size) const
{
	for (dgInt32 i = 0; i < size; i++) {
		a[i] = b[i] + scale * c[i];
	}
}

template<class T, dgInt32 Size>
DG_INLINE void dgLCP<T, Size>::PartialMatrixTimeVector(const T* const v, T* const out, dgInt32 size) const
{
	const dgGeneralMatrix<T, Size>& me = *this;
	for (dgInt32 i = 0; i < size; i++) {
		out[i] = PartialDotProduct(&me[i][0], v, size);
	}
}



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

template<class T, dgInt32 Size>
void dgSPDMatrix<T, Size>::CholeskySolve(dgGeneralVector<T, Size>& x, const dgGeneralVector<T, Size> &b, dgInt32 n) const
{
//	CholeskySolve(&x[0], &b[0], dgGeneralMatrix<T, Size>::m_rowCount);
//	const dgGeneralMatrix<T, Size>& me = *this;
	for (dgInt32 i = 0; i < n; i++) {
		T acc = 0.0f;
		const T* const row = &m_rows[i][0];
		for (dgInt32 j = 0; j < i; j++) {
			acc = acc + row[j] * x[j];
		}
		x[i] = (b[i] - acc) / row[i];
	}

	for (dgInt32 i = n - 1; i >= 0; i--) {
		T acc = 0.0f;
		for (dgInt32 j = i + 1; j < n; j++) {
			acc = acc + m_rows[j][i] * x[j];
		}
		x[i] = (x[i] - acc) / m_rows[i][i];
	}
}



// calculate delta_r = A * delta_x
template<class T, dgInt32 Size>
DG_INLINE void dgLCP<T, Size>::CalculateDelta_r(dgInt32 n)
{
//	const dgInt32 size = dgGeneralMatrix<T, Size>::m_rowCount;
//	const dgSPDMatrix<T, Size>& me = *this;
	for (dgInt32 i = n; i < Size; i++) {
		delta_r[i] = m_rows[i].DotProduct(delta_x);
	}
}


// calculate delta_x = int (Acc) * unitStep 
// unitStep  in vector [-a[c][0] * dir, -a[c][1] * dir, -a[c][2] * dir, ....-a[c][n-1] * dir]
// on exit delta_x[n] = dir,  delta_x[n.. size] = 0.0f
template<class T, dgInt32 Size>
//DG_INLINE void dgLCP<T, Size>::CalculateDelta_x(T* const delta_x, T* const tmp, T dir, dgInt32 n) const
DG_INLINE void dgLCP<T, Size>::CalculateDelta_x(T dir, dgInt32 n)
{
//	const dgInt32 size = dgGeneralMatrix<T, Size>::m_rowCount;
//	const dgSPDMatrix<T, Size>& me = *this;
	const dgGeneralVector<T, Size>& row = m_rows[n];
	for (dgInt32 i = 0; i < n; i++) {
		//tmp[i] = -me[n][i] * dir;
		tmp[i] = -row[i] * dir;
	}
	CholeskySolve(delta_x, tmp, n);
	delta_x[n] = dir;
	for (dgInt32 i = n + 1; i < Size; i++) {
		delta_x[i] = T(0.0f);
	}
}


template<class T, dgInt32 Size>
bool dgLCP<T, Size>::SolveDantzig()
{
	//dgSPDMatrix<T, Size>& me = *this;
	//const dgInt32 size = dgGeneralMatrix<T, Size>::m_rowCount;

	//static int xxx;
	//xxx++;
	//dgLCP<T, Size> gauss(*this);
	//gauss.GaussSeidelLCP(100000, T(1.0e-6f));
	//dgTrace(("pgs %d :", xxx));
	//gauss.GetX().Trace();

	//T* const r = &m_tmp[0][0];
	//T* const x = &m_tmp[1][0];
	//T* const delta_r = &m_tmp[2][0];
	//T* const delta_x = &m_tmp[3][0];
	//T* const diagonal = &m_tmp[4][0];
	//T* const tmp = &m_tmp[5][0];
	//T* const b = &m_b[0];
	//T* const x_out = &m_x[0];
	//T* const r_out = &m_r[0];
	//T* const low = &m_low[0];
	//T* const high = &m_high[0];
	//dgInt16* const permute = m_permute;

	for (dgInt32 i = 0; i < Size; i++) {
		x[i] = dgClamp(x_out[i], low[i], high[i]);
		permute[i] = dgInt16(i);
		diagonal[i] = m_rows[i][i];
	}

	MatrixTimeVector(x, r);
	for (dgInt32 i = 0; i < Size; i++) {
		r[i] -= b[i];
	}

	dgInt32 index = 0;
	dgInt32 count = Size;
	dgInt32 clampedIndex = Size;

	while (count) {
		bool loop = true;
		bool calculateDelta_x = true;
		T dir = 0.0f;

		while (loop) {
			loop = false;
			T clamp_x(0.0f);
			dgInt32 swapIndex = -1;

			if (T(fabs(r[index]) > T(1.0e-12f))) {
				if (calculateDelta_x) {
					dir = (r[index] <= T(0.0f)) ? T(1.0f) : T(-1.0f);
					CalculateDelta_x(dir, index);
				}
				calculateDelta_x = true;
				CalculateDelta_r(index);
				dgAssert(delta_r[index] != T(0.0f));
				dgAssert(T(fabs(delta_x[index])) == T(1.0f));

				T s = -r[index] / delta_r[index];
				dgAssert(s >= T(0.0f));

dgAssert (0);
/*
				for (dgInt32 i = 0; i <= index; i++) {
					T x1 = x[i] + s * delta_x[i];
					if (x1 > high[i]) {
						swapIndex = i;
						clamp_x = high[i];
						s = (high[i] - x[i]) / delta_x[i];
					} else if (x1 < low[i]) {
						swapIndex = i;
						clamp_x = low[i];
						s = (low[i] - x[i]) / delta_x[i];
					}
				}
				dgAssert(s >= T(0.0f));
				dgAssert(s <= -r[index] / delta_r[index]);

				for (dgInt32 i = clampedIndex; (i < size) && (s > T(1.0e-12f)); i++) {
					T r1 = r[i] + s * delta_r[i];
					if ((r1 * r[i]) < T(0.0f)) {
						dgAssert(T(fabs(delta_r[i]) > T(0.0f)));
						T s1 = -r[i] / delta_r[i];
						dgAssert(s1 >= T(0.0f));
						if (s1 < s) {
							s = s1;
							swapIndex = i;
						}
					}
				}

				if (s > T(1.0e-12f)) {
					for (dgInt32 i = 0; i < size; i++) {
						dgAssert((x[i] + T(1.0e-4f)) >= low[i]);
						dgAssert((x[i] - T(1.0e-4f)) <= high[i]);
						x[i] += s * delta_x[i];
						r[i] += s * delta_r[i];
						dgAssert((x[i] + T(1.0e-4f)) >= low[i]);
						dgAssert((x[i] - T(1.0e-4f)) <= high[i]);
					}
				}
*/			
			}

dgAssert (0);
/*
			if (swapIndex == -1) {
				r[index] = T(0.0f);
				delta_r[index] = T(0.0f);
				if (!dgSPDMatrix<T, Size>::CholeskyFactorizationAddRow(index)) {
					return false;
				}
				index++;
				count--;
				loop = false;
			} else if (swapIndex == index) {
				count--;
				clampedIndex--;
				x[index] = clamp_x;
				delta_x[index] = T(0.0f);
				PermuteRows(index, clampedIndex);
				loop = count ? true : false;
			} else if (swapIndex > index) {
				loop = true;
				r[swapIndex] = T(0.0f);
				dgAssert(swapIndex < size);
				dgAssert(clampedIndex <= size);
				if (swapIndex < clampedIndex) {
					count--;
					clampedIndex--;
					PermuteRows(clampedIndex, swapIndex);
					dgAssert(clampedIndex >= index);
				} else {
					count++;
					dgAssert(clampedIndex < size);
					PermuteRows(clampedIndex, swapIndex);
					clampedIndex++;
					dgAssert(clampedIndex <= size);
					dgAssert(clampedIndex >= index);
				}
				calculateDelta_x = false;

			} else {
				x[swapIndex] = clamp_x;
				CholeskyRestore(diagonal, swapIndex, index);

				dgAssert(index > 0);
				PermuteRows(swapIndex, index - 1);
				PermuteRows(index - 1, index);

				clampedIndex--;
				PermuteRows(clampedIndex, index);

				index--;
				for (dgInt32 i = swapIndex; i < index; i++) {
#ifdef _DEBUG
					dgAssert(dgSPDMatrix<T, Size>::CholeskyFactorizationAddRow(i));
#else
					dgSPDMatrix<T, Size>::CholeskyFactorizationAddRow(i);
#endif
				}
				loop = true;
			}
*/
		}
	}

	for (dgInt32 i = 0; i < Size; i++) {
		dgInt32 j = permute[i];
		x_out[j] = x[i];
		r_out[j] = r[i];
	}

	//dgTrace(("lcp %d :", xxx));
	//m_x.Trace();
	//m_x.Copy(gauss.GetX());

	return true;
}



#endif