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

#ifndef __dStack__
#define __dStack__

#include "dCoreStdafx.h"
#include "dTypes.h"
#include "dDebug.h"
#include "dMemory.h"

class dStackBase
{
	protected:
	dStackBase (size_t size);
	~dStackBase ();

	const void *m_ptr;
};

inline dStackBase::dStackBase (size_t size)
	:m_ptr (dMemory::Malloc (size_t (size)))
{
}

inline dStackBase::~dStackBase ()
{
	dMemory::Free ((void*)m_ptr);
}

template<class T>
class dStack: public dStackBase
{
	public:
	dStack (size_t size);
	~dStack ();
	dInt32 GetSizeInBytes() const;
	dInt32 GetElementsCount() const;
	
	D_INLINE T& operator[] (dInt32 entry);
	D_INLINE const T& operator[] (dInt32 entry) const;

	private:
	size_t m_size;
};

template<class T>
dStack<T>::dStack (size_t size)
	:dStackBase (size * sizeof(T))
	,m_size(size)
{
}

template<class T>
dStack<T>::~dStack ()
{
}

template<class T>
dInt32 dStack<T>::GetElementsCount() const
{
	return dInt32 (m_size);
}

template<class T>
dInt32 dStack<T>::GetSizeInBytes() const
{
	return dInt32 (m_size * sizeof(T));
}

template<class T>
D_INLINE T& dStack<T>::operator[] (dInt32 entry) 
{
	dAssert (entry >= 0);
	dAssert ((size_t(entry) < m_size) || ((m_size == 0) && (entry == 0)));

	T* const mem = (T*) m_ptr;
	return mem[entry];
}

template<class T>
D_INLINE const T& dStack<T>::operator[] (dInt32 entry) const
{
	dAssert (entry >= 0);
	dAssert ((entry < m_size) || ((m_size == 0) && (entry == 0)));

	const T* const mem = (T*) m_ptr;
	return mem[entry];
}

#endif

