
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

#ifndef __dgGeneralMatrix__
#define __dgGeneralMatrix__

#include "dgStdafx.h"
#include "dgDebug.h"
#include "dgGeneralVector.h"


template<class T>
class dgGeneralMatrix
{
	public:
	dgGeneralMatrix(const dgGeneralMatrix<T>& src);
	dgGeneralMatrix(dgMemoryAllocator* const allocator, dgInt32 row, dgInt32 column);
	~dgGeneralMatrix();

	void* operator new (size_t size);
	void operator delete (void* const ptr);
	void* operator new (size_t size, dgMemoryAllocator* const allocator);
	void operator delete (void* const ptr, dgMemoryAllocator* const allocator);

	dgGeneralVector<T>& operator[] (dgInt32 i);
	const dgGeneralVector<T>& operator[] (dgInt32 i) const;

	dgInt32 GetRowCount() const;
	dgInt32 GetColCount() const;

	void Clear(T val);
	void Identity();

	void SwapRows(dgInt32 i, dgInt32 j);
	void SwapColumns(dgInt32 i, dgInt32 j);
	
	// calculate out = v * A;
	void VectorTimeMatrix(const dgGeneralVector<T> &v, dgGeneralVector<T> &out) const;

	// calculate out = A * v;
	void MatrixTimeVector(const dgGeneralVector<T> &v, dgGeneralVector<T> &out) const;

	// calculate M = A * B;
	void MatrixTimeMatrix(const dgGeneralMatrix<T>& A, const dgGeneralMatrix<T>& B);

	// calculate M = A * transpose (B);
	void MatrixTimeMatrixTranspose(const dgGeneralMatrix<T>& A, const dgGeneralMatrix<T>& Bt);

	void Trace() const;

	static dgInt32 CalculateMemorySize (dgInt32 row, dgInt32 column)
	{
		return row * dgGeneralVector<T>::CalculateMemorySize (column) + row * sizeof (dgGeneralVector<T>*) + row * sizeof (dgGeneralVector<T>);
	}

	protected:
	DG_INLINE void VectorTimeMatrix(const T* const v, T* const out) const;
	DG_INLINE void MatrixTimeVector(const T* const v, T* const out) const;

	dgGeneralVector<T>** m_rows;
	dgMemoryAllocator* m_allocator;
	dgInt32 m_rowCount;
};

template<class T>
class dgSquareMatrix: public dgGeneralMatrix<T>
{
	public:
	dgSquareMatrix(const dgGeneralMatrix<T>& src);
	dgSquareMatrix(dgMemoryAllocator* const allocator, dgInt32 size);
	~dgSquareMatrix();

	static dgInt32 CalculateMemorySize(dgInt32 row)
	{
		return dgGeneralMatrix<T>::CalculateMemorySize(row, row);
	}


	bool SolveGaussian(dgGeneralVector<T> &x, const dgGeneralVector<T> &b);
};

// ***********************************************************************************************
//
//   LinearSystem
//
// ***********************************************************************************************
template<class T>
dgGeneralMatrix<T>::dgGeneralMatrix(dgMemoryAllocator* const allocator, dgInt32 row, dgInt32 column)
	:m_rows ((dgGeneralVector<T>**) allocator->MallocLow(row * sizeof (dgGeneralVector<T>*)))
	,m_allocator(allocator)
	,m_rowCount(row)
{
     dgAssert(row > 0);
     dgAssert(column > 0);
     for (dgInt32 i = 0; i < row; i++) {
           m_rows[i] = new (m_allocator) dgGeneralVector<T>(allocator, column);
     }
}


template<class T>
dgGeneralMatrix<T>::dgGeneralMatrix(const dgGeneralMatrix<T>& src)
	:m_rows((dgGeneralVector<T>**) src.m_allocator->MallocLow(src.m_rowCount * sizeof (dgGeneralVector<T>*)))
	,m_allocator(src.m_allocator)
	,m_rowCount(src.m_rowCount)
{
	for (dgInt32 i = 0; i < m_rowCount; i++) {
		m_rows[i] = new (m_allocator) dgGeneralVector<T>(src[i]);
	}
}


template<class T>
dgGeneralMatrix<T>::~dgGeneralMatrix()
{
	for (dgInt32 i = 0; i < m_rowCount; i++) {
		delete m_rows[i]; 
	}
    m_allocator->FreeLow(m_rows);
}

template<class T>
void* dgGeneralMatrix<T>::operator new (size_t size)
{
	dgAssert(0);
	return NULL;
}

template<class T>
void* dgGeneralMatrix<T>::operator new (size_t size, dgMemoryAllocator* const allocator)
{
	return allocator->MallocLow(size);
}

template<class T>
void dgGeneralMatrix<T>::operator delete (void* const ptr, dgMemoryAllocator* const allocator)
{
	dgAssert(0);
	//dgFree(ptr); 
}

template<class T>
void dgGeneralMatrix<T>::operator delete (void* const ptr)
{
	dgAssert(0);
}


template<class T>
dgInt32 dgGeneralMatrix<T>::GetRowCount() const
{
     return m_rowCount;
}

template<class T>
dgInt32 dgGeneralMatrix<T>::GetColCount() const
{
     return m_rows[0]->GetRowCount();
}


template<class T>
void dgGeneralMatrix<T>::Trace() const
{
     for (dgInt32 i = 0; i < m_rowCount; i++) {
           m_rows[i]->Trace();
     }
}

template<class T>
dgGeneralVector<T>& dgGeneralMatrix<T>::operator[] (dgInt32 i)
{
     dgAssert(i < m_rowCount);
     dgAssert(i >= 0);
     return *m_rows[i];
}

template<class T>
const dgGeneralVector<T>& dgGeneralMatrix<T>::operator[] (dgInt32 i) const
{
     dgAssert(i < m_rowCount);
     dgAssert(i >= 0);
     return *m_rows[i];
}

template<class T>
void dgGeneralMatrix<T>::Clear(T val)
{
     for (dgInt32 i = 0; i < m_rowCount; i++) {
           m_rows[i].Clear(val);
     }
}

template<class T>
void dgGeneralMatrix<T>::Identity()
{
     for (dgInt32 i = 0; i < m_rowCount; i++) {
           m_rows[i].Clear(T(0.0f));
           m_rows[i][i] = T(1.0f);
     }
}

template<class T>
void dgGeneralMatrix<T>::VectorTimeMatrix(const dgGeneralVector<T> &v, dgGeneralVector<T> &out) const
{
	dgAssert(&v != &out);
	dgAssert(m_rowCount == v.m_colCount);
	dgAssert(m_colCount == out.m_colCount);
	VectorTimeMatrix(v, out);
}


template<class T>
void dgGeneralMatrix<T>::VectorTimeMatrix(const T* const v, T* const out) const
{
	const dgGeneralMatrix<T>& me = *this;
	const dgInt32 colCount = GetColCount();
	for (dgInt32 i = 0; i < colCount; i++) {
		T acc = T(0.0f);
		for (dgInt32 j = 0; j < m_rowCount; j++) {
			acc = acc + me[j][i] * v[i];
		}
		out[i] = acc;
	}
}



template<class T>
void dgGeneralMatrix<T>::MatrixTimeVector(const dgGeneralVector<T> &v, dgGeneralVector<T> &out) const
{
     dgAssert(&v != &out);
     dgAssert(m_rowCount == out.m_colCount);
     dgAssert(GetColCount() == v.m_colCount);
	 MatrixTimeVector (&v[0], &out[0]);
}

template<class T>
DG_INLINE void dgGeneralMatrix<T>::MatrixTimeVector(const T* const v, T* const out) const
{
	const dgGeneralMatrix<T>& me = *this;
	for (dgInt32 i = 0; i < m_rowCount; i++) {
		out[i] = me[i].DotProduct(v);
	}
}


template<class T>
void dgGeneralMatrix<T>::MatrixTimeMatrix(const dgGeneralMatrix<T>& A, const dgGeneralMatrix<T>& B)
{
	dgAssert(this != &A);
	dgAssert(m_rowCount == A.m_rowCount);
	dgAssert(GetColCount() == B.GetColCount());
	dgAssert(A.GetColCount() == B.m_rowCount);

	const dgInt32 colCount = GetColCount();
	dgInt32 count = A.m_colCount;
	for (dgInt32 i = 0; i < m_rowCount; i++) {
		T* const out = &m_rows[i][0];
		T* const rowA = &A.m_rows[i][0];
		for (dgInt32 j = 0; j < colCount; j++) {
			T acc(0.0f);
			for (dgInt32 k = 0; k < count; k++) {
				acc = acc + rowA[k] * B.m_rows[k][j];
			}
			out[j] = acc;
		}
	}
}


template<class T>
void dgGeneralMatrix<T>::MatrixTimeMatrixTranspose(const dgGeneralMatrix<T>& A, const dgGeneralMatrix<T>& Bt)
{
     dgAssert(m_rowCount == A.m_rowCount);
     dgAssert(GetColCount() == Bt.m_rowCount);
     dgAssert(A.GetColCount() == Bt.GetColCount());

     dgAssert(this != &A);
     dgAssert(this != &Bt);

	 const dgInt32 colCount = GetColCount();
     dgInt32 count = A.m_colCount;
     for (dgInt32 i = 0; i < m_rowCount; i++) {
           T* const out = &m_rows[i][0];
           T* const rowA = &A.m_rows[i][0];
           for (dgInt32 j = 0; j < colCount; j++) {
                T acc(0.0f);
                T* const rowB = &Bt.m_rows[j][0];
                for (dgInt32 k = 0; k < count; k++) {
                     acc = acc + rowA[k] * rowB[k];
                }
                out[j] = acc;
           }
     }
}

template<class T>
void dgGeneralMatrix<T>::SwapRows(dgInt32 i, dgInt32 j)
{
     dgAssert(i >= 0);
     dgAssert(j >= 0);
     dgAssert(i < m_rowCount);
     dgAssert(j < m_rowCount);
	 if (j != i) {
		dgSwap(m_rows[i]->m_columns, m_rows[j]->m_columns);
	 }
}

template<class T>
void dgGeneralMatrix<T>::SwapColumns(dgInt32 i, dgInt32 j)
{
     dgAssert(i >= 0);
     dgAssert(j >= 0);
     dgAssert(i < GetColCount());
     dgAssert(j < GetColCount());
	 if (j != i) {
		 dgGeneralMatrix<T>& me = *this;
		 for (dgInt32 k = 0; k < m_rowCount; k++) {
			dgSwap(me[k][i], me[k][j]);
		 }
	 }
}

template<class T>
dgSquareMatrix<T>::dgSquareMatrix(const dgGeneralMatrix<T>& src)
	:dgGeneralMatrix<T>(src)
{
}

template<class T>
dgSquareMatrix<T>::dgSquareMatrix(dgMemoryAllocator* const allocator, dgInt32 size)
	:dgGeneralMatrix<T>(allocator, size, size)
{
}

template<class T>
dgSquareMatrix<T>::~dgSquareMatrix()
{
}


template<class T>
bool dgSquareMatrix<T>::SolveGaussian(dgGeneralVector<T> &x, const dgGeneralVector<T> &b)
{
	dgAssert(GetColCount() == GetRowCount());
	dgAssert(b.GetRowCount() == GetRowCount());
	dgAssert(x.GetRowCount() == GetRowCount());

	x.Copy(b);
	T* const B = &x[0];

	// convert to upper triangular matrix by applying gauss partial pivoting
	dgGeneralMatrix<T>& me = *this;
	for (dgInt32 i = 0; i < dgGeneralMatrix<T>::m_rowCount - 1; i++) {

		dgInt32 k = i;
		T maxVal (fabs(me[i][i]));
		for (dgInt32 j = i + 1; j < dgGeneralMatrix<T>::m_rowCount - 1; j++) {
			T val (fabs(me[j][i]));
			if (val > maxVal) {
				k = j;
				maxVal = val;
			}
		}

		if (maxVal < T(1.0e-12f)) {
			return false;
		}

		if (k != i) {
			dgGeneralMatrix<T>::SwapRows(i, k);
			dgSwap(B[i], B[k]);
		}

		T* const rowI = &me[i][0];
		T den = T(1.0f) / rowI[i];
		for (dgInt32 k = i + 1; k < dgGeneralMatrix<T>::m_rowCount; k++) {
			T* const rowK = &me[k][0];
			T factor(-rowK[i] * den);
			for (dgInt32 j = i + 1; j < dgGeneralMatrix<T>::m_rowCount; j++) {
				rowK[j] += rowI[j] * factor;
			}
			rowK[i] = T(0.0f);
			B[k] += B[i] * factor;
		}
	}

	for (dgInt32 i = dgGeneralMatrix<T>::m_rowCount - 1; i >= 0; i--) {
		T acc(0);
		T* const rowI = &me[i][0];
		for (dgInt32 j = i + 1; j < dgGeneralMatrix<T>::m_rowCount; j++) {
			acc = acc + rowI[j] * B[j];
		}
		B[i] = (B[i] - acc) / rowI[i];
	}
	//   Trace();
	//   x.Trace();
	return true;
}

#endif
