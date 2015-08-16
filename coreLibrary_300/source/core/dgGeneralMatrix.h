
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
	DG_CLASS_ALLOCATOR(allocator)
	dgGeneralMatrix(const dgGeneralMatrix<T>& src);
	dgGeneralMatrix(dgMemoryAllocator* const allocator, dgInt32 row, dgInt32 column);
	~dgGeneralMatrix();

	dgGeneralVector<T>& operator[] (dgInt32 i);
	const dgGeneralVector<T>& operator[] (dgInt32 i) const;

	dgInt32 GetRowCount() const;
	dgInt32 GetColCount() const;

	void Clear(T val);
	void Identity();

	void SwapRows(dgInt32 i, dgInt32 j);
	void SwapColumns(dgInt32 i, dgInt32 j);

	//dgGeneralMatrix Transpose ();
	//void Inverse (dgGeneralMatrix& inverseOut);
	void GaussianPivotStep(dgInt32 srcRow, dgInt32 pivotRow, dgInt32 pivotCol, T tol = T(1.0e-6f));

	// calculate out = V * A;
	void VectorTimeMatrix(const dgGeneralVector<T> &v, dgGeneralVector<T> &out);

	// calculate out = A * transpose (V);
	void MatrixTimeVectorTranspose(const dgGeneralVector<T> &v, dgGeneralVector<T> &out);

	// calculate M = A * B;
	void MatrixTimeMatrix(const dgGeneralMatrix<T>& A, const dgGeneralMatrix<T>& B);

	// calculate M = A * transpose (B);
	void MatrixTimeMatrixTranspose(const dgGeneralMatrix<T>& A, const dgGeneralMatrix<T>& Bt);

	bool Solve(dgGeneralVector<T> &x, const dgGeneralVector<T> &b);
	void Trace() const;

	protected:
	dgGeneralVector<T>** m_rows;
	dgMemoryAllocator* m_allocator;
	dgInt32 m_rowCount;
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
	:m_rows((dgGeneralVector<T>**) allocator->MallocLow(src.m_rowCount * sizeof (dgGeneralVector<T>*)))
	,m_allocator(src.m_allocator)
	,m_rowCount(src.m_rowCount)
{
	dgInt32 colums = GetColCount();
	for (dgInt32 i = 0; i < m_rowCount; i++) {
		m_rows[i] = new (m_allocator) dgGeneralVector<T>(m_allocator, colums);
		m_rows[i].Copy (src[i]);
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


//template<class T>
//void dgGeneralMatrix<T>::Transpose ()
//{
//   dgInt32 i;
//   dgInt32 j;
//
//   dgAssert (m_rowCount == 
//   dgGeneralMatrix<T>& me = *this;
//   for (i = 0; i < m_rowCount; i ++) {
//         for (j = i + 1; j < m_rowCount; j ++) {
//              T tmp (me[i][j]);
//              me[i][j] = me[j][i];
//              me[j][i] = tmp;
//         }
//   }
//}


template<class T>
void dgGeneralMatrix<T>::GaussianPivotStep(dgInt32 srcRow, dgInt32 pivotRow, dgInt32 pivotCol, T tol)
{
     dgGeneralMatrix<T>& me = *this;

     T num(me[pivotRow][pivotCol]);
     if (T(dgAbsf(num)) > tol) {
           T den(me[srcRow][pivotCol]);
           dgAssert(T(dgAbsf(den)) > T(0.0f));
           den = -num / den;
           me[pivotRow].LinearCombine(den, me[srcRow], me[pivotRow]);
     }
}


//template<class T>
//void dgGeneralMatrix<T>::Inverse (dgGeneralMatrix& inverseOut)
//{
//   dgAssert (m_colCount == m_rowCount);
//}


template<class T>
void dgGeneralMatrix<T>::VectorTimeMatrix(const dgGeneralVector<T> &v, dgGeneralVector<T> &out)
{
     dgAssert(&v != &out);
     dgAssert(m_rowCount == v.m_colCount);
     dgAssert(m_colCount == out.m_colCount);

     T const* outMem = &out[0];
     const T* const inMem = &v[0];
     const dgGeneralMatrix<T>& me = *this;
     for (dgInt32 i = 0; i < m_colCount; i++) {
           T acc = T(0.0f);
           for (dgInt32 j = 0; j < m_rowCount; j++) {
                acc = acc + inMem[j] * me[j][i];
           }
           outMem[i] = acc;
     }
}


template<class T>
void dgGeneralMatrix<T>::MatrixTimeVectorTranspose(const dgGeneralVector<T> &v, dgGeneralVector<T> &out)
{
     dgAssert(&v != &out);
     dgAssert(m_rowCount == out.m_colCount);
     dgAssert(m_colCount == v.m_colCount);

     for (dgInt32 i = 0; i < m_rowCount; i++) {
           out[i] = v.DotProduct(m_rows[i]);
     }
}

template<class T>
void dgGeneralMatrix<T>::MatrixTimeMatrix(const dgGeneralMatrix<T>& A, const dgGeneralMatrix<T>& B)
{
     dgAssert(m_rowCount == A.m_rowCount);
     dgAssert(m_colCount == B.m_colCount);
     dgAssert(A.m_colCount == B.m_rowCount);

     dgAssert(this != &A);

     dgInt32 count = A.m_colCount;
     for (dgInt32 i = 0; i < m_rowCount; i++) {
           T* const out = &m_rows[i][0];
           T* const rowA = &A.m_rows[i][0];
           for (dgInt32 j = 0; j < m_colCount; j++) {
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
     dgAssert(m_colCount == Bt.m_rowCount);
     dgAssert(A.m_colCount == Bt.m_colCount);

     dgAssert(this != &A);
     dgAssert(this != &Bt);

     dgInt32 count = A.m_colCount;
     for (dgInt32 i = 0; i < m_rowCount; i++) {
           T* const out = &m_rows[i][0];
           T* const rowA = &A.m_rows[i][0];
           for (dgInt32 j = 0; j < m_colCount; j++) {
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
bool dgGeneralMatrix<T>::Solve(dgGeneralVector<T> &x, const dgGeneralVector<T> &b)
{
     dgAssert(GetColCount() == GetRowCount());
     dgAssert(b.GetRowCount() == GetRowCount());
	 dgAssert(x.GetRowCount() == GetRowCount());

	 x.Copy(b);
     T* const B = &x[0];

     // convert to upper triangular matrix by applying gauss partial pivoting
	 dgGeneralMatrix<T>& me = *this;
     for (dgInt32 i = 0; i < m_rowCount - 1; i++) {
           T* const rowI = &me[i][0];
           T den(rowI[i]);

           if (T(dgAbsf(den)) < T(1.0e-12f)) {
			   dgInt32 pivot = -1;
                for (dgInt32 j = i + 1; j < m_rowCount - 1; j++) {
					if (T(dgAbsf(den)) > T(1.0e-12f)) {
						pivot = j;
						break;
					}
				}
				if (pivot == -1) {
					return false;
				}
				me[i] += me[pivot];
				den = rowI[i];
           }

		   den = T(1.0f) / den;
           for (dgInt32 k = i + 1; k < m_rowCount; k++) {
                T* const rowK = &me[k][0];
                T factor(-rowK[i] * den);
                for (dgInt32 j = i + 1; j < m_rowCount; j++) {
                    rowK[j] += rowI[j] * factor;
                }
				rowK[i] = T(0.0f);
                B[k] += B[i] * factor;
           }
     }

     B[m_rowCount - 1] = B[m_rowCount - 1] / me[m_rowCount - 1][m_rowCount - 1];
     for (dgInt32 i = m_rowCount - 2; i >= 0; i--) {
           T acc(0);
           T* const rowI = &me[i][0];
           for (dgInt32 j = i + 1; j < m_rowCount; j++) {
                acc = acc + rowI[j] * B[j];
           }
           B[i] = (B[i] - acc) / rowI[i];
     }
//	Trace();
//	x.Trace();
    return true;
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

#endif
