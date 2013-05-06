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

/****************************************************************************
*
*  Visual C++ 6.0 created by: Julio Jerez
*
****************************************************************************/
#ifndef __dgArray__
#define __dgArray__

#include "dgStdafx.h"

template<class T>
class dgArray
{
	public:
	dgArray (dgInt32 granulatitySize, dgMemoryAllocator* const allocator, dgInt32 aligmentInBytes = DG_MEMORY_GRANULARITY);
	~dgArray ();

	DG_CLASS_ALLOCATOR(allocator)

	
	T& operator[] (dgInt32 i);
	const T& operator[] (dgInt32 i) const;
	void Resize (dgInt32 size) const;

	dgInt32 GetElementSize() const;
	dgInt32 GetBytesCapacity () const;
	dgInt32 GetElementsCapacity () const; 

	bool ExpandCapacityIfNeessesary (dgInt32 index, dgInt32 stride) const;

	private:
	dgInt32 m_granulatity;
	dgInt32 m_aligmentInByte;
	mutable dgInt32 m_maxSize;
	mutable T *m_array;
	dgMemoryAllocator* m_allocator; 
};


template<class T>
dgArray<T>::dgArray (dgInt32 granulatitySize, dgMemoryAllocator* const allocator, dgInt32 aligmentInBytes)
 :m_granulatity(granulatitySize), m_aligmentInByte(aligmentInBytes), m_maxSize(0), m_array(NULL), m_allocator(allocator)
{
	if (m_aligmentInByte <= 0) {
		m_aligmentInByte = 8;
	}
	m_aligmentInByte = 1 << dgExp2(m_aligmentInByte);
}

template<class T>
dgArray<T>::~dgArray ()
{
	if (m_array) {
		m_allocator->FreeLow (m_array);
	}
}


template<class T>
const T& dgArray<T>::operator[] (dgInt32 i) const
{ 
	dgAssert (i >= 0);
	while (i >= m_maxSize) {
		Resize (i);
	}
	return m_array[i];
}


template<class T>
T& dgArray<T>::operator[] (dgInt32 i)
{
	dgAssert (i >= 0);
	while (i >= m_maxSize) {
		Resize (i);
	}
	return m_array[i];
}

template<class T>
dgInt32 dgArray<T>::GetElementSize() const
{
	return sizeof (T);
}

template<class T>
dgInt32 dgArray<T>::GetElementsCapacity () const
{
	return m_maxSize;
}


template<class T>
dgInt32 dgArray<T>::GetBytesCapacity () const
{
	return  m_maxSize * GetElementSize();
}



template<class T>
void dgArray<T>::Resize (dgInt32 size) const
{
	if (size >= m_maxSize) {
		size = size + m_granulatity - (size + m_granulatity) % m_granulatity;
		T* const newArray = (T*) m_allocator->MallocLow (GetElementSize() * size, m_aligmentInByte);
		if (m_array) {
			for (dgInt32 i = 0; i < m_maxSize; i ++) {
				newArray[i]	= m_array[i];
			}
			m_allocator->FreeLow (m_array);
		}
		m_array = newArray;
		m_maxSize = size;
	} else if (size < m_maxSize) {
		size = size + m_granulatity - (size + m_granulatity) % m_granulatity;
		T* const newArray = (T*) m_allocator->MallocLow (GetElementSize() * size, m_aligmentInByte);
		if (m_array) {
			for (dgInt32 i = 0; i < size; i ++) {
				newArray[i]	= m_array[i];
			}
			m_allocator->FreeLow (m_array);
		}
		m_array = newArray;
		m_maxSize = size;
	}
}

template<class T>
bool dgArray<T>::ExpandCapacityIfNeessesary (dgInt32 index, dgInt32 stride) const
{
	bool ret = false;
	dgInt32 size = (index + 1) * stride;
	while (size >= m_maxSize) {
		ret = true;
		Resize (m_maxSize * 2);
	}
	return ret;
}

#endif




