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

#ifndef __ND_FIX_SIZE_ARRAY_H__
#define __ND_FIX_SIZE_ARRAY_H__

#include "ndCoreStdafx.h"

template<class T, dInt32 maxSize>
class dFixSizeArray: public dClassAlloc
{
	public:
	dFixSizeArray();

	dInt32 GetCount() const;
	void SetCount(dInt32 count);

	dInt32 GetCapacity() const;

	T& operator[] (dInt32 i);
	const T& operator[] (dInt32 i) const;

	void PushBack(const T& element);

	T m_array[maxSize];
	dInt32 m_count;
};

template<class T, dInt32 maxSize>
dFixSizeArray<T, maxSize>::dFixSizeArray()
	:dClassAlloc()
	,m_count(0)
{
}

template<class T, dInt32 maxSize>
dInt32 dFixSizeArray<T, maxSize>::GetCapacity() const
{
	return maxSize;
}

template<class T, dInt32 maxSize>
dInt32 dFixSizeArray<T, maxSize>::GetCount() const
{
	return m_count;
}

template<class T, dInt32 maxSize>
void dFixSizeArray<T, maxSize>::SetCount(dInt32 count)
{
	m_count = (count < 0) ? 0 : ((count > maxSize) ? maxSize : count);
}

template<class T, dInt32 maxSize>
T& dFixSizeArray<T, maxSize>::operator[] (dInt32 i)
{
	dAssert(i >= 0);
	dAssert(i < maxSize);
	return m_array[i];
}

template<class T, dInt32 maxSize>
const T& dFixSizeArray<T, maxSize>::operator[] (dInt32 i) const
{
	dAssert(i >= 0);
	dAssert(i < maxSize);
	return m_array[i];
}

template<class T, dInt32 maxSize>
void dFixSizeArray<T, maxSize>::PushBack(const T& element)
{
	dAssert(m_count <= maxSize);
	(*this)[m_count] = element;
	m_count++;
}

#endif
