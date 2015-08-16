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

#ifndef __dgGeneralVector__
#define __dgGeneralVector__

#include "dgStdafx.h"
#include "dgDebug.h"
#include "dgMemory.h"

template<class T> class dgSPDMatrix;
template<class T> class dgGeneralMatrix;

template<class T>
class dgGeneralVector
{
	friend class dgSPDMatrix<T>;
	friend class dgGeneralMatrix<T>;

public:
	DG_CLASS_ALLOCATOR(allocator)

	dgGeneralVector(const dgGeneralVector<T> &src);
	dgGeneralVector(dgMemoryAllocator* const allocator, dgInt32 size);

	~dgGeneralVector();

	T& operator[] (dgInt32 i);
	const T& operator[] (dgInt32 i) const;

	dgInt32 GetRowCount() const;

	void Clear(T val);
	void Copy(const dgGeneralVector<T> &src);
	T DotProduct(const dgGeneralVector &b) const;
	T Norm() const;
	T Norm2() const;

	void operator += (const dgGeneralVector &A);
	void operator -= (const dgGeneralVector &A);

	void Scale(T scale);
	void LinearCombine(T scale, const dgGeneralVector &A, const dgGeneralVector &B);

	void Trace() const;

protected:
	T* m_columns;
	dgMemoryAllocator* m_allocator;
	dgInt32 m_colCount;
	dgGeneralVector();
};


// ***********************************************************************************************
//
//   vector
//
// ***********************************************************************************************
template<class T>
dgGeneralVector<T>::dgGeneralVector()
	:m_columns(NULL)
	,m_allocator(NULL)
	,m_colCount(0)
{
}

template<class T>
dgGeneralVector<T>::dgGeneralVector(dgMemoryAllocator* const allocator, dgInt32 size)
	:m_columns((T*) allocator->MallocLow (size * sizeof (T)))
	,m_allocator(allocator)
	,m_colCount(size)
{
	dgAssert(size > 0);
	dgAssert((((dgUnsigned32)m_columns) & 0x0f) == 0);
}

template<class T>
dgGeneralVector<T>::dgGeneralVector(const dgGeneralVector<T> &src)
	:m_columns((T*)src.m_allocator->MallocLow(src.m_colCount * sizeof (T)))
	,m_allocator(src.m_allocator)
	,m_colCount(src.m_colCount)
{
	dgAssert((((dgUnsigned32)m_columns) & 0x0f) == 0);
	Copy(src);
}

template<class T>
dgGeneralVector<T>::~dgGeneralVector()
{
	if (m_allocator) {
		dgAssert(m_columns);
		m_allocator->FreeLow(m_columns);
	}
}

template<class T>
void dgGeneralVector<T>::Trace() const
{
	for (dgInt32 i = 0; i < m_colCount; i++) {
		dgTrace(("%f ", m_columns[i]));
	}
	dgTrace(("\n"));
}

template<class T>
T& dgGeneralVector<T>::operator[] (dgInt32 i)
{
	dgAssert(i < m_colCount);
	dgAssert(i >= 0);
	return m_columns[i];
}

template<class T>
const T& dgGeneralVector<T>::operator[] (dgInt32 i) const
{
	dgAssert(i < m_colCount);
	dgAssert(i >= 0);
	return m_columns[i];
}

template<class T>
dgInt32 dgGeneralVector<T>::GetRowCount() const
{
	return m_colCount;
}


// return dot product
template<class T>
T dgGeneralVector<T>::DotProduct(const dgGeneralVector<T> &A) const
{
	dgAssert(m_colCount == A.m_colCount);
	T val(0.0);
	for (dgInt32 i = 0; i < m_colCount; i++) {
		val = val + m_columns[i] * A.m_columns[i];
	}
	return val;
}

template<class T>
void dgGeneralVector<T>::Clear(T val)
{
	for (dgInt32 i = 0; i < m_colCount; i++) {
		m_columns[i] = val;
	}
}

template<class T>
void dgGeneralVector<T>::Copy(const dgGeneralVector<T> &src)
{
	dgAssert(m_colCount == src.m_colCount);
	for (dgInt32 i = 0; i < m_colCount; i++) {
		m_columns[i] = src.m_columns[i];
	}
}

template<class T>
void dgGeneralVector<T>::Scale(T scale)
{
	for (dgInt32 i = 0; i < m_colCount; i++) {
		m_columns[i] = m_columns[i] * scale;
	}
}


template<class T>
T dgGeneralVector<T>::Norm2() const
{
	T norm(0);
	for (dgInt32 i = 0; i < m_colCount; i++) {
		norm = dgMax(m_columns[i] * m_columns[i], norm);
	}
	return norm;
}

template<class T>
T dgGeneralVector<T>::Norm() const
{
	return T(sqrt(Norm2()));
}


template<class T>
void dgGeneralVector<T>::LinearCombine(T scale, const dgGeneralVector<T> &A, const dgGeneralVector<T> &B)
{
	dgAssert(A.m_colCount == m_colCount);
	dgAssert(B.m_colCount == m_colCount);
	for (dgInt32 i = 0; i < m_colCount; i++) {
		m_columns[i] = A.m_columns[i] * scale + B.m_columns[i];
	}
}


template<class T>
void dgGeneralVector<T>::operator+= (const dgGeneralVector<T> &A)
{
	dgAssert(A.m_colCount == m_colCount);
	for (dgInt32 i = 0; i < m_colCount; i++) {
		m_columns[i] += A.m_columns[i];
	}
}

template<class T>
void dgGeneralVector<T>::operator-= (const dgGeneralVector<T> &A)
{
	dgAssert(A.m_colCount == m_colCount);
	for (dgInt32 i = 0; i < m_colCount; i++) {
		m_columns[i] -= A.m_columns[i];
	}
}

#endif


