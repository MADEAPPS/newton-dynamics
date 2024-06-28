/* Copyright (c) <2003-2022> <Julio Jerez, Newton Game Dynamics>
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
#include "ndVector.h"
#include "ndClassAlloc.h"
#include "ndProbability.h"

/// Generic template vector.
/// note: this template vector is similar to std::vector but has some significant differences.
/// therefore is not meant to be a replacement of the sdt::vector, it is simple a tool for this engine 
template<class T>
class ndArray: public ndClassAlloc
{
	public:
	/// constructor, set count and capacity to zero, not memory is allocated.
	ndArray();
	/// constructor, set count and capacity, allocated space for count elements.
	ndArray(ndInt64 count);
	/// copy constructor, allocate and copy only m_size elements from source.
	ndArray(const ndArray& source);

	/// deallocate all memory, dos not call destructor on any of th elements.
	~ndArray ();

	/// return the size of the array.
	ndInt64 GetCount() const;

	/// Set a new size.
	/// if count is larger than m_size, the array resized by doubling its size, 
	/// all the data is simply copied to the new array and old array is deleted.
	void SetCount(ndInt64 count);

	//void Clear();
	/// Set a new size.
	/// the array resized to count, all the data is simply copied 
	/// to the new array and old array is deleted.
	void Resize(ndInt64 count);

	/// return the capacity of the array.
	ndInt64 GetCapacity() const;
	
	/// Get the i element for the array.
	/// behavior is undefined is i is larger of equal to the array size
	T& operator[] (ndInt64 i);

	/// Get the i element for the array.
	/// behavior is undefined is i is larger of equal to the array size
	const T& operator[] (ndInt64 i) const;

	/// Interchange all the information with other.
	/// other must be of the same type.
	void Swap(ndArray& other);

	/// Add element to the end of the buffer.
	/// size is incremented by one, and the array is resized if it reaches max capacity
	void PushBack(const T& element);

	/// Randomize the vector entries.
	void RandomShuffle(ndInt64 count);

	/// set all member to 0.
	/// useful for when making vectors of vectors (ex matrices)
	void ResetMembers();

	/// assign all members.
	/// useful for when making vectors of vectors (ex matrices)
	void SetMembers(ndInt64 size, void* const memory);

	private: 
	void CopyData(T* const dst, const T* const src, ndInt64 elements);

	protected:
	T* m_array;
	ndInt64 m_size;
	ndInt64 m_capacity;
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
ndArray<T>::ndArray(ndInt64 count)
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
		for (ndInt64 i = 0; i < source.m_size; ++i)
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
const T& ndArray<T>::operator[] (ndInt64 i) const
{
	ndAssert(i >= 0);
	ndAssert(i < m_size);
	return m_array[i];
}

template<class T>
T& ndArray<T>::operator[] (ndInt64 i)
{
	ndAssert(i >= 0);
	ndAssert(i < m_size);
	return m_array[i];
}

template<class T>
void ndArray<T>::PushBack(const T& element)
{
	ndAssert(m_size <= m_capacity);
	if (m_size == m_capacity)
	{
		Resize(m_capacity * 2);
	}
	m_array[m_size] = element;
	m_size++;
}

template<class T>
ndInt64 ndArray<T>::GetCount() const
{
	return m_size;
}

template<class T>
void ndArray<T>::SetCount(ndInt64 count)
{
	while (count > m_capacity)
	{
		Resize(m_capacity * 2);
	}
	m_size = count;
}

template<class T>
ndInt64 ndArray<T>::GetCapacity() const
{
	return m_capacity;
}

template<class T>
void ndArray<T>::CopyData(T* const dstPtr, const T* const srcPtr, ndInt64 elements)
{
	ndMemCpy(dstPtr, srcPtr, elements);
}

template<class T>
void ndArray<T>::Resize(ndInt64 newSize)
{
	// note: I know some tools will detect a warning here
	// because it is copying object from one array
	// to another without calling the copy constructor
	// and destructor, but the ndArray is designed for 
	// high performance memory resizing of structures,
	// if an application needs to use an array with
	// for general purpose classes, 
	// please use standart lib std::vector
	if (newSize > m_capacity || (m_capacity == 0))
	{
		newSize = ndMax(newSize, ndInt64(16));
		T* const newArray = (T*)ndMemory::Malloc(size_t(sizeof(T) * newSize));
		if (m_array) 
		{
			if (m_size)
			{
				CopyData(newArray, m_array, m_size);
			}
			ndMemory::Free(m_array);
		}
		m_array = newArray;
		m_capacity = newSize;
	}
	else if (newSize < m_capacity)
	{
		newSize = ndMax(newSize, ndInt64(16));
		T* const newArray = (T*)ndMemory::Malloc(size_t(sizeof(T) * newSize));
		if (m_array) 
		{
			CopyData(newArray, m_array, newSize);
			ndMemory::Free(m_array);
		}
		m_size = newSize;
		m_array = newArray;
		m_capacity = newSize;
	}
}

template<class T>
void ndArray<T>::Swap(ndArray& other)
{
	ndSwap(m_array, other.m_array);
	ndSwap(m_size, other.m_size);
	ndSwap(m_capacity, other.m_capacity);
}

template<class T>
void ndArray<T>::SetMembers(ndInt64 size, void* const memory)
{
	m_size = size;
	m_capacity = size + 1;
	m_array = (T*)memory;
}

template<class T>
void ndArray<T>::ResetMembers()
{
	m_size = 0;
	m_capacity = 0;
	m_array = nullptr;
}

template<class T>
void ndArray<T>::RandomShuffle(ndInt64 count)
{
	const ndInt64 size = ndMin (count, GetCount());
	for (ndInt64 i = size - 1; i >= 0; --i)
	{
		ndInt64 randomIndex = ndRandInt();
		ndInt64 j = randomIndex % size;
		ndSwap (m_array[i], m_array[j]);
	}
}

#endif




