/* Copyright (c) <2003-2019> <Julio Jerez, Newton Game Dynamics>
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
#ifndef __D_ARRAY_H__
#define __D_ARRAY_H__

#include "dCoreStdafx.h"
#include "dTypes.h"
#include "dMemory.h"

template<class T>
class dArray
{
	public:
	dArray ();
	//dArray (dgMemoryAllocator* const allocator, dInt32 aligmentInBytes = DG_MEMORY_GRANULARITY);
	//dArray (const dArray& source, dInt32 itemsToCopy);
	//dArray(const dArray& source);
	~dArray ();

	dInt32 GetCount() const;

	void Clear();
	void Resize(dInt32 count);
	void Reserve(dInt32 count);

//	DG_CLASS_ALLOCATOR(allocator)
	
	T& operator[] (dInt32 i);
	const T& operator[] (dInt32 i) const;

	void PushBack(T& element);
	
/*	
	
	DG_INLINE void ResizeIfNecessary  (dInt32 index) const;

	dInt32 GetElementSize() const;
	dInt32 GetBytesCapacity () const;
	dInt32 GetElementsCapacity () const; 
	dgMemoryAllocator* GetAllocator() const;
	void SetAllocator(dgMemoryAllocator* const allocator);
*/
	protected:
	T* m_array;

	private:
	dInt32 m_size;
	dInt32 m_capacity;
//	dInt32 m_aligmentInBytes;
};

template<class T>
dArray<T>::dArray()
	:m_array(NULL)
	,m_size(0)
	,m_capacity(0)
{
}

/*
template<class T>
dArray<T>::dArray (dgMemoryAllocator* const allocator, dInt32 aligmentInBytes)
	:m_array(NULL)
	,m_maxSize(0)
	,m_aligmentInBytes(aligmentInBytes)
	,m_allocator(allocator)
{
	if (m_aligmentInBytes <= 0) {
		m_aligmentInBytes = DG_MEMORY_GRANULARITY;
	}
	m_aligmentInBytes = 1 << dgExp2(m_aligmentInBytes);
}

template<class T>
dArray<T>::dArray (const dArray& source, dInt32 itemsToCopy)
	:m_array(NULL)
	,m_maxSize(itemsToCopy)
	,m_aligmentInBytes(source.m_aligmentInBytes)
	,m_allocator(source.m_allocator)
{
	if (source.m_array) {
		m_array = (T*) m_allocator->MallocLow (sizeof (T) * itemsToCopy, m_aligmentInBytes);
		for (dInt32 i = 0; i < itemsToCopy; i++) {
			m_array[i] = source.m_array[i];
		}
	}
}
template<class T>
dArray<T>::dArray(const dArray& source)
	:m_array(NULL)
	,m_maxSize(source.m_maxSize)
	,m_aligmentInBytes(source.m_aligmentInBytes)
	,m_allocator(source.m_allocator)
{
	dgAssert(0);
	// use dArray<T>::dArray(const dArray& source, dInt32 itemsToCopy)
	if (source.m_array) {
		m_array = (T*)m_allocator->MallocLow(sizeof(T) * m_maxSize, m_aligmentInBytes);
		for (dInt32 i = 0; i < m_maxSize; i++) {
			m_array[i] = source.m_array[i];
		}
	}
}
*/

template<class T>
dArray<T>::~dArray ()
{
	if (m_array) 
	{
		dFree(m_array);
	}
}

template<class T>
const T& dArray<T>::operator[] (dInt32 i) const
{
	dAssert(i >= 0);
	while (i >= m_maxSize) 
	{
		Resize(i * 2);
	}
	return m_array[i];
}

template<class T>
T& dArray<T>::operator[] (dInt32 i)
{
	dAssert(i >= 0);
	while (i >= m_capacity) 
	{
		Resize(i * 2);
	}
	return m_array[i];
}

template<class T>
void dArray<T>::PushBack(T& element)
{
	dArray<T>& me = *this;
	me[m_size] = element;
	m_size++;
}


template<class T>
dInt32 dArray<T>::GetCount() const
{
	return m_size;
}

template<class T>
void dArray<T>::Clear()
{
	m_size = 0;
}

template<class T>
void dArray<T>::Reserve(dInt32 count)
{
	dAssert(0);
}

template<class T>
void dArray<T>::Resize(dInt32 size)
{
	if (size >= m_capacity) 
	{
		size = dMax(size, 16);
		T* const newArray = (T*)dMalloc(dInt32(sizeof(T) * size));
		if (m_array) 
		{
			for (dInt32 i = 0; i < m_capacity; i++)
			{
				newArray[i] = m_array[i];
			}
			dFree(m_array);
		}
		m_array = newArray;
		m_capacity = size;
	}
	else if (size < m_capacity) 
	{
		size = dMax(size, 16);
		T* const newArray = (T*)dMalloc(dInt32(sizeof(T) * size));
		if (m_array) 
		{
			for (dInt32 i = 0; i < size; i++) 
			{
				newArray[i] = m_array[i];
			}
			dFree(m_array);
		}
		m_array = newArray;
		m_capacity = size;
	}
}


/*
template<class T>
dInt32 dArray<T>::GetElementSize() const
{
	return sizeof (T);
}

template<class T>
dInt32 dArray<T>::GetElementsCapacity () const
{
	return m_maxSize;
}

template<class T>
dInt32 dArray<T>::GetBytesCapacity () const
{
	return  m_maxSize * GetElementSize();
}


template<class T>
dgMemoryAllocator* dArray<T>::GetAllocator() const
{
	return m_allocator;
}

template<class T>
void dArray<T>::SetAllocator(dgMemoryAllocator* const allocator)
{
	dgAssert (!m_allocator);
	m_allocator = allocator;
}


template<class T>
DG_INLINE void dArray<T>::ResizeIfNecessary  (dInt32 size) const
{
	while (size >= m_maxSize) {
		Resize (m_maxSize * 2);
	}
}
*/

#endif




