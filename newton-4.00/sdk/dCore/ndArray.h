/* Copyright (c) <2003-2021> <Julio Jerez, Newton Game Dynamics>
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
#ifndef __ND_ARRAY_H__
#define __ND_ARRAY_H__

#include "ndCoreStdafx.h"
#include "ndTypes.h"
#include "ndUtils.h"
#include "ndClassAlloc.h"

template<class T>
class ndArray: public ndClassAlloc
{
	public:
	ndArray();
	ndArray(dInt32 count);
	ndArray(const ndArray& source);

	~ndArray ();

	dInt32 GetCount() const;
	void SetCount(dInt32 count);

	void Clear();
	void Resize(dInt32 count);
	dInt32 GetCapacity() const;
	
	T& operator[] (dInt32 i);
	const T& operator[] (dInt32 i) const;

	void Swap(ndArray& other);
	void PushBack(const T& element);

	protected:
	T* m_array;
	dInt32 m_size;
	dInt32 m_capacity;
};

template<class T>
ndArray<T>::ndArray()
	:ndClassAlloc()
	,m_array(nullptr)
	,m_size(0)
	,m_capacity(0)
{
}

template<class T>
ndArray<T>::ndArray(dInt32 count)
	:ndClassAlloc()
	,m_array(nullptr)
	,m_size(0)
	,m_capacity(0)
{
	Resize(count);
}

template<class T>
ndArray<T>::ndArray(const ndArray& source)
	:ndClassAlloc()
	,m_array(nullptr)
	,m_size(0)
	,m_capacity(0)
{
	if (source.m_array)
	{
		Resize(source.m_capacity);
		SetCount(source.m_size);
		for (dInt32 i = 0; i < source.m_size; i++)
		{
			m_array[i] = source[i];
		}
	}
}

template<class T>
ndArray<T>::~ndArray ()
{
	if (m_array) 
	{
		ndMemory::Free(m_array);
	}
}

template<class T>
const T& ndArray<T>::operator[] (dInt32 i) const
{
	dAssert(i >= 0);
	dAssert(i < m_size);
	return m_array[i];
}

template<class T>
T& ndArray<T>::operator[] (dInt32 i)
{
	dAssert(i >= 0);
	dAssert(i < m_size);
	return m_array[i];
}

template<class T>
void ndArray<T>::PushBack(const T& element)
{
	dAssert(m_size <= m_capacity);
	if (m_size == m_capacity)
	{
		Resize(m_capacity * 2);
	}
	m_array[m_size] = element;
	m_size++;
}

template<class T>
dInt32 ndArray<T>::GetCount() const
{
	return m_size;
}

template<class T>
void ndArray<T>::SetCount(dInt32 count)
{
	m_size = count;
	while (m_size > m_capacity)
	{
		Resize(m_capacity * 2);
	}
}

template<class T>
dInt32 ndArray<T>::GetCapacity() const
{
	return m_capacity;
}

template<class T>
void ndArray<T>::Clear()
{
	m_size = 0;
}

template<class T>
void ndArray<T>::Resize(dInt32 size)
{
	if (size > m_capacity || (m_capacity == 0))
	{
		size = dMax(size, 16);
		T* const newArray = (T*)ndMemory::Malloc(dInt32(sizeof(T) * size));
		if (m_array) 
		{
			for (dInt32 i = 0; i < m_capacity; i++)
			{
				memcpy(&newArray[i], &m_array[i], sizeof(T));
			}
			ndMemory::Free(m_array);
		}
		m_array = newArray;
		m_capacity = size;
	}
	else if (size < m_capacity) 
	{
		size = dMax(size, 16);
		T* const newArray = (T*)ndMemory::Malloc(dInt32(sizeof(T) * size));
		if (m_array) 
		{
			for (dInt32 i = 0; i < size; i++) 
			{
				memcpy(&newArray[i], &m_array[i], sizeof(T));
			}
			ndMemory::Free(m_array);
		}
		m_size = size;
		m_capacity = size;
		m_array = newArray;
	}
}

template<class T>
void ndArray<T>::Swap(ndArray& other)
{
	dSwap(m_array, other.m_array);
	dSwap(m_size, other.m_size);
	dSwap(m_capacity, other.m_capacity);
}

#endif




