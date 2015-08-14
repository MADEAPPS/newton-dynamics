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
	dgSPDMatrix(dgInt32 size, T *elemBuffer, dgGeneralVector<T>* m_rowBuffer);
	dgSPDMatrix(const dgSPDMatrix<T>& src, T *elemBuffer, dgGeneralVector<T>* m_rowBuffer);
	~dgSPDMatrix();


	// solve a general Linear complementary program using Danzig algorithm,  
	// A * x = b + r
	// subjected to conditions
	// x = low, if w >= 0  
	// x = high, if w <= 0  
	// low <= x <= high,  if w = 0  
	void DanzigLCP(const dgGeneralVector<T>& x, const dgGeneralVector<T>& b, const dgGeneralVector<T>& low, const dgGeneralVector<T>& high);

	bool TestPSD() const;
	bool LDLtDecomposition();
	bool CholeskyDecomposition();
	void DownDateCholeskyDecomposition(dgInt32 column);

	bool Solve(dgGeneralVector<T> &b);
	void BackAndForwardSustitition(dgGeneralVector<T> &b);
};


// ***********************************************************************************************
//
//   LinearSystem
//
// ***********************************************************************************************
template<class T>
dgSPDMatrix<T>::dgSPDMatrix(dgInt32 size)
:dgGeneralMatrix<T>(size, size)
{
}

template<class T>
dgSPDMatrix<T>::dgSPDMatrix(const dgSPDMatrix<T>& src)
: dgGeneralMatrix<T>(src)
{
}


template<class T>
dgSPDMatrix<T>::dgSPDMatrix(
	dgInt32 size,
	T *elemBuffer,
	dgGeneralVector<T>* m_rowBuffer)
	:dgGeneralMatrix<T>(size, size, elemBuffer, m_rowBuffer)
{
	}

template<class T>
dgSPDMatrix<T>::dgSPDMatrix(
	const dgSPDMatrix<T>& src,
	T *elemBuffer,
	dgGeneralVector<T>* m_rowBuffer)
	: dgGeneralMatrix<T>(src, elemBuffer, m_rowBuffer)
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


template<class T>
void dgSPDMatrix<T>::BackAndForwardSustitition(dgGeneralVector<T> &b)
{
//	_BackAndForwardSustitition(b.m_columns, &m_rows[0].m_columns, sizeof (dgGeneralVector<T>), sizeof (T), m_rowCount);

	//dgSPDMatrix<dgFloat32> M (8);
	//M.CholeskyDecomposition();

	B[0] = B[0] / rows[0][0];
	for (dgInt32 i = 1; i < size; i++) {
		T acc = 0.0f;
		for (dgInt32 j = 0; j < i; j++) {
			acc = acc + rows[j][i] * B[j];
		}
		B[i] = (B[i] - acc) / rows[i][i];
	}

	B[size - 1] = B[size - 1] / rows[size - 1][size - 1];
	for (dgInt32 i = size - 2; i >= 0; i--) {
		T acc = 0.0f;
		T* const row = rows[i];
		for (dgInt32 j = i + 1; j < size; j++) {
			acc = acc + row[j] * B[j];
		}
		B[i] = (B[i] - acc) / rows[i][i];
	}
}


template<class T>
bool dgSPDMatrix<T>::Solve(dgGeneralVector<T> &b)
{
//	bool _SolveByCholeskyDecomposition(void *rightsideVector, void *rowPointers, dgInt32 rowStrideInBytes, dgInt32 typeSizeInBytes, dgInt32 size)

	dgUnsigned8* const rowArray = (dgUnsigned8*)rowPointers;
	if (typeSizeInBytes == dgInt32(sizeof (dgFloat32))) {
		dgInt32 i;
		dgFloat32 **rows;
		rows = (dgFloat32**)dgStackAlloc(size * sizeof (dgFloat32*));
		for (i = 0; i < size; i++) {
			rows[i] = *((dgFloat32 **)rowArray);
			rowArray += rowStrideInBytes;
		}


		if (CholeskyDecomposition(rows, size)) {
			BackAndForwardSustitition(rows, size, (dgFloat32*)rightsideVector);
			return true;
		}
	} else {
		dgAssert(0);
		dgAssert(typeSizeInBytes == dgInt32(sizeof (dgFloat64)));
	}
}


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
bool dgSPDMatrix<T>::CholeskyDecomposition()
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
void dgSPDMatrix<T>::DownDateCholeskyDecomposition(dgInt32 column)
{
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

template<class T>
void dgSPDMatrix<T>::DanzigLCP(const dgGeneralVector<T>& x, const dgGeneralVector<T>& b, const dgGeneralVector<T>& low, const dgGeneralVector<T>& high)
{
}
#endif


