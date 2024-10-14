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

#ifndef __ND_FIX_SIZE_ARRAY_H__
#define __ND_FIX_SIZE_ARRAY_H__

#include "ndCoreStdafx.h"

template<class T, ndInt32 maxSize>
class ndFixSizeArray: public ndClassAlloc
{
	public:
	ndFixSizeArray();

	ndInt32 GetCount() const;
	void SetCount(ndInt32 count);

	ndInt32 GetCapacity() const;

	T& operator[] (ndInt32 i);
	const T& operator[] (ndInt32 i) const;

	T Pop();
	void PushBack(const T& element);

	T m_array[maxSize];
	ndInt32 m_count;
};

template<class T, ndInt32 maxSize>
ndFixSizeArray<T, maxSize>::ndFixSizeArray()
	:ndClassAlloc()
	,m_count(0)
{
}

template<class T, ndInt32 maxSize>
ndInt32 ndFixSizeArray<T, maxSize>::GetCapacity() const
{
	return maxSize;
}

template<class T, ndInt32 maxSize>
ndInt32 ndFixSizeArray<T, maxSize>::GetCount() const
{
	return m_count;
}

template<class T, ndInt32 maxSize>
void ndFixSizeArray<T, maxSize>::SetCount(ndInt32 count)
{
	m_count = (count < 0) ? 0 : ((count > maxSize) ? maxSize : count);
}

template<class T, ndInt32 maxSize>
T& ndFixSizeArray<T, maxSize>::operator[] (ndInt32 i)
{
	ndAssert(i >= 0);
	ndAssert(i < maxSize);
	return m_array[i];
}

template<class T, ndInt32 maxSize>
const T& ndFixSizeArray<T, maxSize>::operator[] (ndInt32 i) const
{
	ndAssert(i >= 0);
	ndAssert(i < maxSize);
	return m_array[i];
}

template<class T, ndInt32 maxSize>
T ndFixSizeArray<T, maxSize>::Pop()
{
	ndAssert(m_count >= 1);
	m_count--;
	return (*this)[m_count];
}

template<class T, ndInt32 maxSize>
void ndFixSizeArray<T, maxSize>::PushBack(const T& element)
{
	ndAssert(m_count <= maxSize);
	(*this)[m_count] = element;
	m_count++;
}

#endif
