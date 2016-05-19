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

//template<class T, dgInt32 Size> class dgLCP;
//template<class T, dgInt32 Size> class dgSPDMatrix;
//template<class T, dgInt32 Size> class dgSquareMatrix;
//template<class T, dgInt32 Rows, dgInt32 Columns> class dgGeneralMatrix;

template<class T, dgInt32 Size>
class dgGeneralVector
{
	public:
	dgGeneralVector() {}
	~dgGeneralVector() {}

	T& operator[] (dgInt32 i);
	const T& operator[] (dgInt32 i) const;

	T DotProduct(const dgGeneralVector &b) const;
	dgInt32 GetRowCount() const {return Size;}
/*
	void operator += (const dgGeneralVector &a);
	void operator -= (const dgGeneralVector &a);

	T Norm() const;
	T Norm2() const;

	void Clear(T val);
	void Scale(T scale);
	void Copy(const dgGeneralVector &src);
	
	void LinearCombine(T scale, const dgGeneralVector &a, const dgGeneralVector &b);

	void Trace() const;

	static dgInt32 CalculateMemorySize(dgInt32 size)
	{
		return size * sizeof (T) + 16;
	}

	protected:
	dgGeneralVector();
	DG_INLINE void Add(const T* const a);
	DG_INLINE void Sub(const T* const a);
	DG_INLINE T DotProduct (const T* const b) const;
*/
	T m_columns[Size];

//	friend class dgLCP<T, Size>;
//	friend class dgSPDMatrix<T, Size>;
//	friend class dgSquareMatrix<T, Size>;
};


#if 0
// ***********************************************************************************************
//
//   vector
//
// ***********************************************************************************************
template<class T, dgInt32 Size>
dgGeneralVector<T, Size>::dgGeneralVector()
	:m_columns(NULL)
	,m_allocator(NULL)
	,m_colCount(0)
{
}


template<class T, dgInt32 Size>
dgGeneralVector<T, Size>::dgGeneralVector(dgMemoryAllocator* const allocator, dgInt32 size)
	:m_columns((T*) allocator->MallocLow (size * sizeof (T)))
	,m_allocator(allocator)
	,m_colCount(size)
{
	dgAssert(size > 0);
	dgAssert((((dgUnsigned32)m_columns) & 0x0f) == 0);
}


template<class T, dgInt32 Size>
dgGeneralVector<T, Size>::dgGeneralVector(const dgGeneralVector<T, Size> &src)
	:m_columns((T*)src.m_allocator->MallocLow(src.m_colCount * sizeof (T)))
	,m_allocator(src.m_allocator)
	,m_colCount(src.m_colCount)
{
	dgAssert((((dgUnsigned32)m_columns) & 0x0f) == 0);
	Copy(src);
}

template<class T, dgInt32 Size>
void dgGeneralVector<T, Size>::Trace() const
{
	for (dgInt32 i = 0; i < m_colCount; i++) {
		dgTrace(("%f ", m_columns[i]));
	}
	dgTrace(("\n"));
}

template<class T, dgInt32 Size>
DG_INLINE T dgGeneralVector<T, Size>::DotProduct (const T* const A) const
{
	T val(0.0f);
	const T* const me = m_columns;
	for (dgInt32 i = 0; i < m_colCount; i++) {
		val = val + me[i] * A[i];
	}
	return val;
}

template<class T, dgInt32 Size>
void dgGeneralVector<T, Size>::Clear(T val)
{
	memset (&m_columns[0], 0, m_colCount * sizeof (T));
}

template<class T, dgInt32 Size>
void dgGeneralVector<T, Size>::Copy(const dgGeneralVector<T, Size> &src)
{
	dgAssert(m_colCount == src.m_colCount);
	memcpy (&m_columns[0], &src[0], m_colCount * sizeof (T));
}

template<class T, dgInt32 Size>
void dgGeneralVector<T, Size>::Scale(T scale)
{
	for (dgInt32 i = 0; i < m_colCount; i++) {
		m_columns[i] *= scale;
	}
}


template<class T, dgInt32 Size>
T dgGeneralVector<T, Size>::Norm2() const
{
	T norm(0);
	for (dgInt32 i = 0; i < m_colCount; i++) {
		norm = dgMax(m_columns[i] * m_columns[i], norm);
	}
	return norm;
}

template<class T, dgInt32 Size>
T dgGeneralVector<T, Size>::Norm() const
{
	return T(sqrt(Norm2()));
}

template<class T, dgInt32 Size>
void dgGeneralVector<T, Size>::LinearCombine(T scale, const dgGeneralVector<T, Size> &A, const dgGeneralVector<T, Size> &B)
{
	dgAssert(A.m_colCount == m_colCount);
	dgAssert(B.m_colCount == m_colCount);
	for (dgInt32 i = 0; i < m_colCount; i++) {
		m_columns[i] = A.m_columns[i] * scale + B.m_columns[i];
	}
}

template<class T, dgInt32 Size>
void dgGeneralVector<T, Size>::operator+= (const dgGeneralVector<T, Size> &A)
{
	dgAssert(A.m_colCount == m_colCount);
	dgAssert(A.m_colCount == m_colCount);
	Add(&A[0]);
}

template<class T, dgInt32 Size>
void dgGeneralVector<T, Size>::operator-= (const dgGeneralVector<T, Size> &A)
{
	dgAssert(A.m_colCount == m_colCount);
	Sub(& A[0]);
}

template<class T, dgInt32 Size>
DG_INLINE void dgGeneralVector<T, Size>::Add(const T* const a)
{
	T* const dst = &m_columns[0];
	for (dgInt32 i = 0; i < m_colCount; i++) {
		dst[i] -= a[i];
	}
}

template<class T, dgInt32 Size>
DG_INLINE void dgGeneralVector<T, Size>::Sub(const T* const a)
{
	T* const dst = &m_columns[0];
	for (dgInt32 i = 0; i < m_colCount; i++) {
		dst[i] -= a[i];
	}
}

/*
template<class T, dgInt32 Size>
void* dgGeneralVector<T, Size>::operator new (size_t size)
{
	dgAssert(0);
	return NULL;
}

template<class T, dgInt32 Size>
void* dgGeneralVector<T, Size>::operator new (size_t size, dgMemoryAllocator* const allocator)
{
	return allocator->MallocLow(dgInt32 (size));
}

template<class T, dgInt32 Size>
void dgGeneralVector<T, Size>::operator delete (void* const ptr, dgMemoryAllocator* const allocator)
{
	dgGeneralVector<T>* const me = (dgGeneralVector<T>*) ptr;
	me->m_allocator->FreeLow(ptr);
}

template<class T, dgInt32 Size>
void dgGeneralVector<T, Size>::operator delete (void* const ptr)
{
	dgGeneralVector<T>* const me = (dgGeneralVector<T>*) ptr;
	me->m_allocator->FreeLow(ptr);
}
*/
#endif

template<class T, dgInt32 Size>
T& dgGeneralVector<T, Size>::operator[] (dgInt32 i)
{
	dgAssert(i < Size);
	dgAssert(i >= 0);
	return m_columns[i];
}

template<class T, dgInt32 Size>
const T& dgGeneralVector<T, Size>::operator[] (dgInt32 i) const
{
	dgAssert(i < Size);
	dgAssert(i >= 0);
	return m_columns[i];
}

// return dot product
template<class T, dgInt32 Size>
T dgGeneralVector<T, Size>::DotProduct(const dgGeneralVector<T, Size> &b) const
{
	T val(0.0f);
	dgAssert (Size == b.GetRowCount());
	for (dgInt32 i = 0; i < Size; i++) {
		val = val + m_columns[i] * b.m_columns[i];
	}
	return val;
}

#endif


