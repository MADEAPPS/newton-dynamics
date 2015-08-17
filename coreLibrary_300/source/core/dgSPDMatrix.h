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


template<class T>
class dgSPDMatrix: public dgGeneralMatrix<T>
{
	public:
	dgSPDMatrix(const dgSPDMatrix<T>& src);
	dgSPDMatrix(dgMemoryAllocator* const allocator, dgInt32 size);
	~dgSPDMatrix();

	bool TestPSD() const;
	bool LDLtDecomposition();
	bool CholeskyFactorization();
	void CholeskySolve(dgGeneralVector<T> &x, const dgGeneralVector<T> &b) const;

	protected:
	DG_INLINE bool CholeskyFactorization(dgInt32 n);
	DG_INLINE bool CholeskyFactorizationAddRow(dgInt32 n);
	DG_INLINE void CholeskySolve(T* const x, const T* const b, dgInt32 n) const;
};



// solve a general Linear complementary program using Danzig algorithm,  
// A * x = b + r
// subjected to conditions
// x(i) = low(i),  if r(i) >= 0  
// x(i) = high(i), if r(i) <= 0  
// low(i) <= x(i) <= high(i),  if r(i) = 0  
// on exit matrix is all input data is destroyed 
// return true if the system has a solution
// 
// Derived from http://www.cs.cmu.edu/~baraff/papers/sig94.pdf
// Fast Contact Force Computation for non penetrating Rigid Bodies, by David Baraff
// by but quite exactly the same. 
template<class T>
class dgDanzigLCP: public dgSPDMatrix<T>
{
	public:
	dgDanzigLCP(const dgDanzigLCP& src);
	dgDanzigLCP(dgMemoryAllocator* const allocator, dgInt32 size);
	~dgDanzigLCP();

	dgGeneralVector<T>& GetX() {return m_xOut;}
	dgGeneralVector<T>& GetB() {return m_bb;}
	dgGeneralVector<T>& GetLowLimit() {return m_lowLimit;}
	dgGeneralVector<T>& GetHightLimit(){return m_highLimit;}

	bool Solve();

	private:
	DG_INLINE T CalculateRow_r(dgInt32 n)  const;
	DG_INLINE void PermuteRows(dgInt32 i, dgInt32 j);
	DG_INLINE void CalculateDelta_r(dgInt32 n) const;
	DG_INLINE void CalculateDelta_x(T dir, dgInt32 n) const;
	DG_INLINE void CholeskyRestore(dgInt32 i, dgInt32 size);

	dgGeneralVector<T> m_bb;
	dgGeneralVector<T> m_xOut;
	dgGeneralVector<T> m_lowLimit;
	dgGeneralVector<T> m_highLimit;
	T* m_x;
	T* m_b;
	T* m_low;
	T* m_high;
	T* m_delta_x;
	T* m_delta_r;
	T* m_scratch;
	T* m_savedDiagonal;
	dgInt16* m_permute;
};


// ***********************************************************************************************
//
//   LinearSystem
//
// ***********************************************************************************************
template<class T>
dgSPDMatrix<T>::dgSPDMatrix(dgMemoryAllocator* const allocator, dgInt32 size)
	:dgGeneralMatrix<T>(allocator, size, size)
{
}

template<class T>
dgSPDMatrix<T>::dgSPDMatrix(const dgSPDMatrix<T>& src)
	:dgGeneralMatrix<T>(src)
{
}


template<class T>
dgSPDMatrix<T>::~dgSPDMatrix()
{
}

template<class T>
bool dgSPDMatrix<T>::TestPSD() const
{
	if (!TestSymetry()) {
		return false;
	}

	dgSPDMatrix<T> tmp(*this);
	return tmp.CholeskyDecomposition();
}

/*

*/

template<class T>
bool dgSPDMatrix<T>::LDLtDecomposition()
{
	for (dgInt32 j = 0; j < m_rowCount; j++) {
		T* const rowJ = &m_rows[j].m_columns[0];
		for (dgInt32 k = 0; k < j; k++) {
			T* const rowK = &m_rows[k].m_columns[0];
			T factor = rowK[j];
			for (dgInt32 i = j; i < m_rowCount; i++) {
				rowJ[i] -= rowK[i] * factor;
			}
		}

		T factor = rowJ[j];
		if (factor <= T(0.0f)) {
			return false;
		}

		rowJ[j] = T(sqrt(factor));
		factor = T(1.0f / rowJ[j]);
		for (dgInt32 k = j + 1; k < m_rowCount; k++) {
			rowJ[k] *= factor;
		}
	}
	return true;
}


template<class T>
bool dgSPDMatrix<T>::CholeskyFactorization()
{
	return CholeskyFactorization(m_rowCount);
}

template<class T>
void dgSPDMatrix<T>::CholeskySolve(dgGeneralVector<T> &x, const dgGeneralVector<T> &b) const
{
	CholeskySolve(&x[0], &b[0], m_rowCount);
}


template<class T>
DG_INLINE bool dgSPDMatrix<T>::CholeskyFactorizationAddRow(dgInt32 n)
{
	dgAssert(n <= m_rowCount);
	dgGeneralMatrix<T>& me = *this;
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
}


// calculate Cholesky factorization of the n first rows
template<class T>
DG_INLINE bool dgSPDMatrix<T>::CholeskyFactorization(dgInt32 n)
{
	bool passed = true;
	for (dgInt32 i = 0; pass && (i < n) ; i++) {
		passed = CholeskyFactorizationAddRow(i);
	}
	return passed;
}


// calculate x = inv (A) * b 
template<class T>
DG_INLINE void dgSPDMatrix<T>::CholeskySolve (T* const x, const T* const b, dgInt32 n) const
{
	const dgGeneralMatrix<T>& me = *this;
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

template<class T>
dgDanzigLCP<T>::dgDanzigLCP(const dgDanzigLCP& src)
	:dgSPDMatrix(src)
	,m_bb(src.m_)
	,m_xOut(src.m_xOut)
	,m_lowLimit(src.m_lowLimit)
	,m_highLimit(src.m_highLimit)
	,m_x(NULL)
	,m_b(NULL)
	,m_low(NULL)
	,m_high(NULL)
	,m_delta_x(NULL)
	,m_delta_r(NULL)
	,m_scratch(NULL)
	,m_savedDiagonal(NULL)
	,m_permute(NULL)
{
}

template<class T>
dgDanzigLCP<T>::dgDanzigLCP(dgMemoryAllocator* const allocator, dgInt32 size)
	:dgSPDMatrix(allocator, size)
	,m_bb(allocator, size)
	,m_xOut(allocator, size)
	,m_lowLimit(allocator, size)
	,m_highLimit(allocator, size)
	,m_x(NULL)
	,m_b(NULL)
	,m_low(NULL)
	,m_high(NULL)
	,m_delta_x(NULL)
	,m_delta_r(NULL)
	,m_scratch(NULL)
	,m_savedDiagonal(NULL)
	,m_permute(NULL)
{
}

template<class T>
dgDanzigLCP<T>::~dgDanzigLCP()
{
}

template<class T>
DG_INLINE void dgDanzigLCP<T>::PermuteRows(dgInt32 i, dgInt32 j)
{
	SwapRows(i, j);
	SwapColumns(i, j);
	dgSwap(m_x[i], m_x[j]);
	dgSwap(m_b[i], m_b[j]);
	dgSwap(m_low[i], m_low[j]);
	dgSwap(m_high[i], m_high[j]);
	dgSwap(m_savedDiagonal[i], m_savedDiagonal[j]);
	dgSwap(m_permute[i], m_permute[j]);
}

// calculate delta_x = int (Acc) * unitStep 
// unitStep  in vector [-a[c][0] * dir, -a[c][1] * dir, -a[c][2] * dir, ....-a[c][n-1] * dir]
// on exit delta_x[n] = dir,  delta_x[n.. size] = 0.0f
template<class T>
DG_INLINE void dgDanzigLCP<T>::CalculateDelta_x(T dir, dgInt32 n) const
{
	const dgInt32 size = m_rowCount;
	const dgSPDMatrix<T>& me = *this;
	for (dgInt32 i = 0; i < n; i++) {
		m_scratch[i] = -me[n][i] * dir;
	}
	CholeskySolve(m_delta_x, m_scratch, n);
	m_delta_x[n] = dir;
	for (dgInt32 i = n + 1; i < size; i++) {
		m_delta_x[i] = T(0.0f);
	}
}


// calculate delta_r = A * delta_x
template<class T>
DG_INLINE void dgDanzigLCP<T>::CalculateDelta_r(dgInt32 n) const
{
	const dgInt32 size = m_rowCount;
	const dgSPDMatrix<T>& me = *this;

	for (dgInt32 i = 0; i < n; i++) {
		m_delta_r[i] = me[n][i] * m_delta_x[i];
	}
	for (dgInt32 i = n; i < size; i++) {
		T acc(T(0.0f));
		const T* const row = &me[i][0];
		for (dgInt32 j = 0; j <= n; j++) {
			acc += row[j] * m_delta_x[j];
		}
		m_delta_r[i] = acc;
	}
}


template<class T>
DG_INLINE T dgDanzigLCP<T>::CalculateRow_r(dgInt32 index) const
{
	const dgInt32 size = m_rowCount;
	const dgGeneralMatrix<T>& me = *this;

	T acc = 0.0f;
	const T* const row = &me[index][0];
	for (dgInt32 j = 0; j < size; j++) {
		acc += row[j] * m_x[j];
	}
	return acc - m_b[index];
}

template<class T>
DG_INLINE void dgDanzigLCP<T>::CholeskyRestore(dgInt32 n, dgInt32 size)
{
	dgGeneralMatrix<T>& me = *this;
	for (dgInt32 i = n; i < size; i ++) {
		me[i][i] = m_savedDiagonal[i];
		T* const row = &me[i][0];
		for (dgInt32 j = 0; j < i; j ++) {
			row[j] = me[j][i];
		}
	}
}

template<class T>
bool dgDanzigLCP<T>::Solve()
{
static int xxx;
static int xxx1;
xxx ++;

	dgSPDMatrix<T>& me = *this;
	const dgInt32 size = m_rowCount;
	m_x = dAlloca (T, size);
	m_delta_x = dAlloca (T, size);
	m_delta_r = dAlloca (T, size);
	m_savedDiagonal = dAlloca (T, size);
	m_permute = dAlloca (dgInt16, size);
	
	m_b = &m_bb[0];
	m_scratch = &m_xOut[0];
	m_low = &m_lowLimit[0];
	m_high = &m_highLimit[0];

	for (dgInt32 i = 0; i < size; i++) {
		m_x[i] = T(0.0f);
		m_permute[i] = dgInt16(i);
		m_savedDiagonal[i] = me[i][i];
	}


	dgInt32 index = 0;
	dgInt32 count = size;
	dgInt32 clampedIndex = size - 1;
//Trace ();
//dgTrace(("\n"));
	while (count) {
xxx1 ++;
		bool loop = true;
		while (loop) {
			loop = false;
			T r = CalculateRow_r (index);
			if (fabs (r) < T(1.0e-6f)) {
				if (!CholeskyFactorizationAddRow(index)) {
					return false;
				}
				index ++;
				count --;
				break;
			} else {
				T dir = (r <= T(0.0f)) ? T(1.0f) : T(-1.0f);
				CalculateDelta_x (dir, index);
				CalculateDelta_r (index);

				dgAssert(m_delta_r[index] != T(0.0f));
				dgAssert(fabs(m_delta_x[index]) == T(1.0f));
		
				T clamp_x (0.0f) ;
				dgInt32 swapIndex = -1;
				T s = -r / m_delta_r[index];
				for (dgInt32 i = 0; i <= index; i ++) {
					if (m_delta_x[i] > T(0.0f)) {
						T s1 = (m_high[i] - m_x[i]) / m_delta_x[i];
						dgAssert (s1 >= T(0.0f));
						if (s1 < s) {
							s = s1;
							swapIndex = i;
							clamp_x = m_high[i];
						}
					} else if (m_delta_x[i] < T(0.0f)) {
						T s1 = (m_low[i] - m_x[i]) / m_delta_x[i];
						dgAssert (s1 >= T(0.0f));
						if (s1 < s) {
							s = s1;
							swapIndex = i;
							clamp_x = m_low[i];
						}
					}
				}

				for (dgInt32 i = 0; i < size; i ++) {
					dgAssert(m_x[i] >= m_low[i]);
					dgAssert(m_x[i] <= m_high[i]);
					m_x[i] += s * m_delta_x[i];
				}

				if (swapIndex == -1) {
					if (!CholeskyFactorizationAddRow(index)) {
						return false;
					}
					index++;
					count--;
					loop = false;
				} else if (swapIndex == index) {
					m_x[index] = clamp_x;
					PermuteRows(index, clampedIndex);
					count--;
					clampedIndex --;
					loop = count ? true : false;
				} else if (swapIndex < index) {
					m_x[swapIndex] = clamp_x;
					CholeskyRestore(swapIndex, index);
					index --;
					PermuteRows(swapIndex, index);
					for (dgInt32 i = swapIndex; i < index; i ++) {
						if(!CholeskyFactorizationAddRow(i)) {
							return false;
						}
					}
					PermuteRows(index, clampedIndex);
					clampedIndex--;
					loop = true;
				} else {
					dgAssert (0);
				}
			}

			#ifdef _DEBUG
				for (dgInt32 i = 0; i < size; i++) {
					dgAssert(m_x[i] >= m_low[i]);
					dgAssert(m_x[i] <= m_high[i]);
				}
			#endif 
		}
	}

	for (dgInt32 i = 0; i < size; i ++) {
		dgAssert(m_x[i] >= m_low[i]);
		dgAssert(m_x[i] <= m_high[i]);
		m_scratch[m_permute[i]] = m_x[i];
	}
	return true;
}


#endif


