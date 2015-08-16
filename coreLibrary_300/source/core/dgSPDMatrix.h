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
	dgSPDMatrix(dgInt32 size);
	dgSPDMatrix(const dgSPDMatrix<T>& src);
	dgSPDMatrix(dgMemoryAllocator* const allocator, dgInt32 size);
	~dgSPDMatrix();

	// solve a general Linear complementary program using Danzig algorithm,  
	// A * x = b + r
	// subjected to conditions
	// x(i) = low(i),  if r(i) >= 0  
	// x(i) = high(i), if r(i) <= 0  
	// low(i) <= x(i) <= high(i),  if r(i) = 0  
	// on exit matrix is all input data is destryed 
	// return true if the system has a solution
	// 
	// Derived from http://www.cs.cmu.edu/~baraff/papers/sig94.pdf
	// Fast Contact Force Computation for Nonpenetrating Rigid Bodies, by David Baraff
	// by but quite exactly the same. 
	bool DanzigLCP(dgGeneralVector<T>& x, dgGeneralVector<T>& b, dgGeneralVector<T>& low, dgGeneralVector<T>& high);

	bool TestPSD() const;
	bool LDLtDecomposition();

	bool CholeskyFactorization();
	void CholeskySolve(dgGeneralVector<T> &x, const dgGeneralVector<T> &b) const;

	private:
	bool CholeskyFactorization(dgInt32 n);
	bool CholeskyFactorizationAddRow(dgInt32 n);
	void CholeskySolve(T* const x, const T* const b, dgInt32 n) const;
	
	void CalculateDelta_x (T* const x, T dir, dgInt32 n) const;
	void CalculateDelta_r (T* const r, const T* const x, dgInt32 n) const;
	T CalculateRow_r (const T* const x, const T* const b, dgInt32 n) const;

//	void DownDateCholeskyDecomposition(dgInt32 column);
//	bool Solve(dgGeneralVector<T> &b);
//	void BackAndForwardSustitition(dgGeneralVector<T> &b);
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

/*
template<class T>
void dgSPDMatrix<T>::DownDateCholeskyDecomposition(dgInt32 column)
{
	dgAssert (0);
	dgGeneralVector<T>* const rowsBuffer = (dgGeneralVector<dgFloat32>*) dgStackAlloc(m_rowCount * sizeof (dgGeneralVector<T>));
	T* const buffer = (T *)dgStackAlloc((m_rowCount + 4) * (m_rowCount + 4) * sizeof (T));

	dgGeneralMatrix<T> tmp(m_rowCount, m_rowCount, buffer, rowsBuffer);
	dgGeneralMatrix<T>& me = *this;
	for (dgInt32 i = 0; i < m_rowCount; i++) {
		tmp[i][i] = me[i][i];
		for (dgInt32 j = i + 1; j < m_rowCount; j++) {
			tmp[i][j] = T(0.0f);
			tmp[j][i] = me[i][j];
		}
	}

	me.MatrixTimeMatrixTranspose(tmp, tmp);
	for (dgInt32 i = 0; i < m_rowCount; i++) {
		me[i][column] = T(0.0f);
		me[column][i] = T(0.0f);
	}
	me[column][column] = T(1.0f);

	CholeskyDecomposition();
}
*/

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
bool dgSPDMatrix<T>::CholeskyFactorizationAddRow(dgInt32 n)
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
bool dgSPDMatrix<T>::CholeskyFactorization(dgInt32 n)
{
	bool pass = true;
	for (dgInt32 i = 0; pass && (i < n) ; i++) {
		pass = CholeskyFactorizationAddRow(i);
		Trace();
	}
	return pass;
}


// calculate x = inv (A) * b 
template<class T>
void dgSPDMatrix<T>::CholeskySolve (T* const x, const T* const b, dgInt32 n) const
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
T dgSPDMatrix<T>::CalculateRow_r (const T* const x, const T* const b, dgInt32 n) const
{
	const dgInt32 size = m_rowCount;
	const dgGeneralMatrix<T>& me = *this;

	T acc = 0.0f;
	const T* const row = &me[n][0];
	for (dgInt32 j = 0; j < size; j++) {
		acc += row[j] * x[j];
	}
	return acc - b[n];
}

// calculate delta_r = A * delta_x
template<class T>
void dgSPDMatrix<T>::CalculateDelta_r (T* const delta_r, const T* const delta_x, dgInt32 n) const
{
	const dgInt32 size = m_rowCount;
	const dgSPDMatrix<T>& me = *this;

	for (dgInt32 i = 0; i < n; i++) {
		delta_r[i] = me[n][i] * delta_x[i];
	}
	for (dgInt32 i = n; i < size; i++) {
		T acc(T(0.0f));
		const T* const row = &me[i][0];
		for (dgInt32 j = 0; j <= n; j++) {
			acc += row[j] * delta_x[j];
		}
		delta_r[i] = acc;
	}
}

// calculate delta_x = int (Acc) * unitStep 
// unitStep  in vector [-a[c][0] * dir, -a[c][1] * dir, -a[c][2] * dir, ....-a[c][n-1] * dir]
// on exit delta_x[n] = dir,  delta_x[n.. size] = 0.0f
template<class T>
void dgSPDMatrix<T>::CalculateDelta_x (T* const x, T dir, dgInt32 n) const
{
	const dgInt32 size = m_rowCount;
	T* const tmp = dAlloca (T, size);
	const dgSPDMatrix<T>& me = *this;
	for (dgInt32 i = 0; i < n; i ++) {
		tmp[i] = - me[n][i] * dir;
	}
	CholeskySolve (x, tmp, n);
	x[n] = dir;
	for (dgInt32 i = n + 1; i < size; i ++) {
		x[i] = T(0.0f);
	}
}

template<class T>
bool dgSPDMatrix<T>::DanzigLCP(dgGeneralVector<T>& xOut, dgGeneralVector<T>& bb, dgGeneralVector<T>& lowLimit, dgGeneralVector<T>& highLimit)
{
	dgSPDMatrix<T>& me = *this;
	const dgInt32 size = m_rowCount;
	T* const x = dAlloca (T, size);
	T* const delta_x = dAlloca (T, size);
	T* const delta_r = dAlloca (T, size);
	T* const diagonal = dAlloca (T, size);
	dgInt16* const permute = dAlloca (dgInt16, size);
	
	T* const b = &bb[0];
	T* const low = &lowLimit[0];
	T* const high = &highLimit[0];
	for (dgInt32 i = 0; i < size; i++) {
		permute[i] = dgInt16(i);
		x[i] = T(0.0f);
		diagonal[i] = me[i][i];
	}

	dgInt32 index = 0;
	dgInt32 count = size;
	dgInt32 clampedIndex = size - 1;
	while (count) {
		bool loop = true;
		while (loop) {
			loop = false;
			T r = CalculateRow_r (x, b, index);
			if (fabs (r) < T(1.0e-6f)) {
				if (!CholeskyFactorizationAddRow(index)) {
					return false;
				}
				index ++;
				count --;
				break;
			} else {
				T dir = (r <= T(0.0f)) ? T(1.0f) : T(-1.0f);
				CalculateDelta_x (delta_x, dir, index);
				CalculateDelta_r (delta_r, delta_x, index);

				dgAssert(delta_r[index] != T(0.0f));
				dgAssert(fabs(delta_x[index]) == T(1.0f));
		
				T clamp_x (0.0f) ;
				dgInt32 swapIndex = -1;
				T s = -r / delta_r[index];
				for (dgInt32 j = 0; j <= index; j ++) {
					if (delta_x[j] > T(0.0f)) {
						T s1 = (high[j] - x[j]) / delta_x[j];
						dgAssert (s1 >= T(0.0f));
						if (s1 < s) {
							s = s1;
							swapIndex = j;
							clamp_x = high[j];
						}
					} else if (delta_x[j] < T(0.0f)) {
						T s1 = (low[j] - x[j]) / delta_x[j];
						dgAssert (s1 >= T(0.0f));
						if (s1 < s) {
							s = s1;
							swapIndex = j;
							clamp_x = low[j];
						}
					}
				}

				for (dgInt32 j = 0; j < size; j ++) {
					x[j] += s * delta_x[j];
				}

				if (swapIndex == -1) {
					if (!CholeskyFactorizationAddRow(index)) {
						return false;
					}
					index++;
					count--;
					loop = count ? true : false;
				} else if (swapIndex == index) {
					x[index] = clamp_x;
					SwapRows(index, clampedIndex);
					SwapColumns(index, clampedIndex);
					dgSwap(x[index], x[clampedIndex]);

					dgSwap (b[index], b[clampedIndex]);
					dgSwap (low[index], low[clampedIndex]);
					dgSwap (high[index], high[clampedIndex]);
					dgSwap(diagonal[index], diagonal[clampedIndex]);
					dgSwap(permute[index], permute[clampedIndex]);
					count--;
					clampedIndex --;
					loop = count ? true : false;
				} else if (swapIndex < index) {
					dgAssert (0);
				}
			}
		}
	}

	T* const out = &xOut[0];
	for (dgInt32 i = 0; i < size; i ++) {
		out[i] = x[permute[i]];
	}
	return true;
}
#endif


