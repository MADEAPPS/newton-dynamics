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

#ifndef __ndStack__
#define __ndStack__

#include "ndCoreStdafx.h"
#include "ndTypes.h"
#include "ndDebug.h"
#include "ndMemory.h"
#include "ndClassAlloc.h"

class ndStackBase : public ndClassAlloc
{
	protected:
	ndStackBase (size_t size);
	~ndStackBase ();

	const void* m_ptr;
};

inline ndStackBase::ndStackBase (size_t size)
	:ndClassAlloc()
	,m_ptr (ndMemory::Malloc (size_t (size)))
{
}

inline ndStackBase::~ndStackBase ()
{
	ndMemory::Free ((void*)m_ptr);
}

template<class T>
class ndStack: public ndStackBase
{
	public:
	ndStack (size_t size);
	~ndStack ();
	dInt32 GetSizeInBytes() const;
	dInt32 GetElementsCount() const;
	
	inline T& operator[] (dInt32 entry);
	inline const T& operator[] (dInt32 entry) const;

	private:
	size_t m_size;
};

template<class T>
ndStack<T>::ndStack (size_t size)
	:ndStackBase (size * sizeof(T))
	,m_size(size)
{
}

template<class T>
ndStack<T>::~ndStack ()
{
}

template<class T>
dInt32 ndStack<T>::GetElementsCount() const
{
	return dInt32 (m_size);
}

template<class T>
dInt32 ndStack<T>::GetSizeInBytes() const
{
	return dInt32 (m_size * sizeof(T));
}

template<class T>
inline T& ndStack<T>::operator[] (dInt32 entry) 
{
	dAssert (entry >= 0);
	dAssert ((size_t(entry) < m_size) || ((m_size == 0) && (entry == 0)));

	T* const mem = (T*) m_ptr;
	return mem[entry];
}

template<class T>
inline const T& ndStack<T>::operator[] (dInt32 entry) const
{
	dAssert (entry >= 0);
	dAssert ((entry < m_size) || ((m_size == 0) && (entry == 0)));

	const T* const mem = (T*) m_ptr;
	return mem[entry];
}

#endif

