
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


template<class T, dgInt32 Rows, dgInt32 Columns>
class dgGeneralMatrix
{
	public:
	dgGeneralMatrix() {};
	~dgGeneralMatrix() {}

	dgGeneralVector<T, Columns>& operator[] (dgInt32 i);
	const dgGeneralVector<T, Columns>& operator[] (dgInt32 i) const;

	// calculate out = A * v;
	void MatrixTimeVector(const dgGeneralVector<T, Columns> &v, dgGeneralVector<T, Columns> &out) const;

	dgInt32 GetRowCount() const {return Rows;}
	dgInt32 GetColCount() const {return Columns;}

	void SwapRows(dgInt32 i, dgInt32 j);
	void SwapColumns(dgInt32 i, dgInt32 j);


/*
	void Clear(T val);
	void Identity();
	
	// calculate out = v * A;
	void VectorTimeMatrix(const dgGeneralVector<T, Rows> &v, dgGeneralVector<T, Rows> &out) const;


	// calculate M = A * transpose (B);
	void MatrixTimeMatrixTranspose(const dgGeneralMatrix<T, Rows, Columns>& A, const dgGeneralMatrix<T, Rows, Columns>& Bt);

	void Trace() const;

	static dgInt32 CalculateMemorySize (dgInt32 row, dgInt32 column)
	{
		return row * dgGeneralVector<T>::CalculateMemorySize (column) + row * sizeof (dgGeneralVector<T>*) + row * sizeof (dgGeneralVector<T>);
	}

	protected:
	DG_INLINE void VectorTimeMatrix(const T* const v, T* const out) const;
*/
	dgGeneralVector<T, Rows> m_rows[Columns];
};


template<class T, dgInt32 Size>
class dgSquareMatrix: public dgGeneralMatrix<T, Size, Size>
{
	public:
	dgSquareMatrix() {}
	~dgSquareMatrix() {}
/*
	static dgInt32 CalculateMemorySize(dgInt32 row)
	{
		return dgGeneralMatrix<T, Rows, Columns>::CalculateMemorySize(row, row);
	}

	bool SolveGaussian(dgGeneralVector<T, Size> &x, const dgGeneralVector<T, Size> &b);
*/
};


#if 0
// ***********************************************************************************************
//
//   LinearSystem
//
// ***********************************************************************************************
template<class T, dgInt32 Rows, dgInt32 Columns>
dgGeneralMatrix<T, Rows, Columns>::dgGeneralMatrix(const dgGeneralMatrix<T, Rows, Columns>& src)
{
}


template<class T, dgInt32 Rows, dgInt32 Columns>
dgGeneralMatrix<T, Rows, Columns>::~dgGeneralMatrix()
{
}

/*
template<class T, dgInt32 Rows, dgInt32 Columns>
void* dgGeneralMatrix<T, Rows, Columns>::operator new (size_t size)
{
	dgAssert(0);
	return NULL;
}

template<class T, dgInt32 Rows, dgInt32 Columns>
void* dgGeneralMatrix<T, Rows, Columns>::operator new (size_t size, dgMemoryAllocator* const allocator)
{
	return allocator->MallocLow(size);
}

template<class T>
void dgGeneralMatrix<T, Rows, Columns>::operator delete (void* const ptr, dgMemoryAllocator* const allocator)
{
	dgAssert(0);
	//dgFree(ptr); 
}

template<class T>
void dgGeneralMatrix<T, Rows, Columns>::operator delete (void* const ptr)
{
	dgAssert(0);
}
*/


template<class T, dgInt32 Rows, dgInt32 Columns>
void dgGeneralMatrix<T, Rows, Columns>::Trace() const
{
     for (dgInt32 i = 0; i < Rows; i++) {
           m_rows[i]->Trace();
     }
}


template<class T, dgInt32 Rows, dgInt32 Columns>
void dgGeneralMatrix<T, Rows, Columns>::Clear(T val)
{
     for (dgInt32 i = 0; i < m_rowCount; i++) {
           m_rows[i].Clear(val);
     }
}

template<class T, dgInt32 Rows, dgInt32 Columns>
void dgGeneralMatrix<T, Rows, Columns>::Identity()
{
     for (dgInt32 i = 0; i < m_rowCount; i++) {
           m_rows[i].Clear(T(0.0f));
           m_rows[i][i] = T(1.0f);
     }
}

template<class T, dgInt32 Rows, dgInt32 Columns>
void dgGeneralMatrix<T, Rows, Columns>::VectorTimeMatrix(const dgGeneralVector<T, Rows> &v, dgGeneralVector<T, Rows> &out) const
{
	dgAssert (0);
	dgAssert(&v != &out);
	dgAssert(GetRowCount() == v.GetRowCount());
	dgAssert(GetColCount() == out.GetRowCount());
	VectorTimeMatrix(v, out);
}


template<class T, dgInt32 Rows, dgInt32 Columns>
void dgGeneralMatrix<T, Rows, Columns>::VectorTimeMatrix(const T* const v, T* const out) const
{
	dgAssert (0);
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





template<class T, dgInt32 Rows, dgInt32 Columns>
void dgGeneralMatrix<T, Rows, Columns>::MatrixTimeMatrix(const dgGeneralMatrix<T, Rows, Columns>& A, const dgGeneralMatrix<T, Rows, Columns>& B)
{
	dgAssert (0);
/*
	dgAssert(this != &A);
	dgAssert(dgGeneralMatrix<T, Rows, Columns>::GetRowCount() == A.m_rowCount);
	dgAssert(dgGeneralMatrix<T, Rows, Columns>::GetColCount() == B.GetColCount());
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
*/
}


template<class T, dgInt32 Rows, dgInt32 Columns>
void dgGeneralMatrix<T, Rows, Columns>::MatrixTimeMatrixTranspose(const dgGeneralMatrix<T, Rows, Columns>& A, const dgGeneralMatrix<T, Rows, Columns>& Bt)
{
	dgAssert (0);
/*
     dgAssert(m_rowCount == A.m_rowCount);
     dgAssert(dgGeneralMatrix<T, Rows, Columns>::GetColCount() == Bt.m_rowCount);
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
*/
 }


template<class T, dgInt32 Size>
dgSquareMatrix<T, Size>::dgSquareMatrix(const dgGeneralMatrix<T, Size, Size>& src)
	:dgGeneralMatrix<T, Size, Size>(src)
{
}


template<class T, dgInt32 Size>
dgSquareMatrix<T, Size>::~dgSquareMatrix()
{
}


template<class T, dgInt32 Size>
bool dgSquareMatrix<T, Size>::SolveGaussian(dgGeneralVector<T, Size> &x, const dgGeneralVector<T, Size> &b)
{
	dgAssert (0);
	return false;
/*
	dgAssert(dgGeneralMatrix<T, Rows, Columns>::GetColCount() == dgGeneralMatrix<T, Rows, Columns>::GetRowCount());
	dgAssert(b.GetRowCount() == dgGeneralMatrix<T, Rows, Columns>::GetRowCount());
	dgAssert(x.GetRowCount() == dgGeneralMatrix<T, Rows, Columns>::GetRowCount());

	x.Copy(b);
	T* const B = &x[0];

	// convert to upper triangular matrix by applying gauss partial pivoting
	dgGeneralMatrix<T>& me = *this;
	for (dgInt32 i = 0; i < dgGeneralMatrix<T, Rows, Columns>::m_rowCount - 1; i++) {

		dgInt32 k = i;
		T maxVal (fabs(me[i][i]));
		for (dgInt32 j = i + 1; j < dgGeneralMatrix<T, Rows, Columns>::m_rowCount - 1; j++) {
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
			dgGeneralMatrix<T, Rows, Columns>::SwapRows(i, k);
			dgSwap(B[i], B[k]);
		}

		T* const rowI = &me[i][0];
		T den = T(1.0f) / rowI[i];
		for (dgInt32 k = i + 1; k < dgGeneralMatrix<T, Rows, Columns>::m_rowCount; k++) {
			T* const rowK = &me[k][0];
			T factor(-rowK[i] * den);
			for (dgInt32 j = i + 1; j < dgGeneralMatrix<T, Rows, Columns>::m_rowCount; j++) {
				rowK[j] += rowI[j] * factor;
			}
			rowK[i] = T(0.0f);
			B[k] += B[i] * factor;
		}
	}

	for (dgInt32 i = dgGeneralMatrix<T, Rows, Columns>::m_rowCount - 1; i >= 0; i--) {
		T acc(0);
		T* const rowI = &me[i][0];
		for (dgInt32 j = i + 1; j < dgGeneralMatrix<T, Rows, Columns>::m_rowCount; j++) {
			acc = acc + rowI[j] * B[j];
		}
		B[i] = (B[i] - acc) / rowI[i];
	}
	//   Trace();
	//   x.Trace();
	return true;
*/
}
#endif

template<class T, dgInt32 Rows, dgInt32 Columns>
dgGeneralVector<T, Columns>& dgGeneralMatrix<T, Rows, Columns>::operator[] (dgInt32 i)
{
     dgAssert(i < Rows);
     dgAssert(i >= 0);
     return m_rows[i];
}

template<class T, dgInt32 Rows, dgInt32 Columns>
const dgGeneralVector<T, Columns>& dgGeneralMatrix<T, Rows, Columns>::operator[] (dgInt32 i) const
{
     dgAssert(i < Rows);
     dgAssert(i >= 0);
     return m_rows[i];
}



template<class T, dgInt32 Rows, dgInt32 Columns>
void dgGeneralMatrix<T, Rows, Columns>::SwapRows(dgInt32 i, dgInt32 j)
{
	dgAssert(i >= 0);
	dgAssert(j >= 0);
	dgAssert(i < GetRowCount());
	dgAssert(j < GetRowCount());
	if (j != i) {
		//dgGeneralMatrix<T>& me = *this;
		dgGeneralVector<T, Columns>& A = m_rows[i];
		dgGeneralVector<T, Columns>& B = m_rows[j];
		for (dgInt32 k = 0; k < Columns; k++) {
			dgSwap(A[k], B[k]);
		}
	}
}

template<class T, dgInt32 Rows, dgInt32 Columns>
void dgGeneralMatrix<T, Rows, Columns>::SwapColumns(dgInt32 i, dgInt32 j)
{
	dgAssert(i >= 0);
	dgAssert(j >= 0);
	dgAssert(i < GetColCount());
	dgAssert(j < GetColCount());
	if (j != i) {
		//dgGeneralMatrix<T>& me = *this;
		for (dgInt32 k = 0; k < Rows; k++) {
			dgSwap(m_rows[k][i], m_rows[k][j]);
		}
	}
}


template<class T, dgInt32 Rows, dgInt32 Columns>
void dgGeneralMatrix<T, Rows, Columns>::MatrixTimeVector(const dgGeneralVector<T, Columns> &v, dgGeneralVector<T, Columns> &out) const
{
    dgAssert(&v != &out);
	dgAssert(GetRowCount() == v.GetRowCount());
	dgAssert(GetColCount() == out.GetRowCount());
	for (dgInt32 i = 0; i < Rows; i++) {
		out[i] = m_rows[i].DotProduct(v);
	}
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


#endif
