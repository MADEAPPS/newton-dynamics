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

#ifndef _D_CONTAINERS_ALLOC_H_
#define _D_CONTAINERS_ALLOC_H_

#include "dCoreStdafx.h"
#include "dClassAlloc.h"

template<class T>
class dContainersAlloc: public dClassAlloc
{
	public:
	dContainersAlloc()
	{
	}

	~dContainersAlloc() 
	{
	}

	static void FlushFreeList()
	{
	}
};

template<class T>
class dContainersFreeListAlloc
{
	class FreeList 
	{
		public:
		FreeList* m_next;
	};

	public:
	dContainersFreeListAlloc()
	{
	}

	~dContainersFreeListAlloc()
	{
	}

	void *operator new (size_t size)
	{
		FreeList** const freeList = GetFreeList();
		if (*freeList) 
		{
			m_size--;
			FreeList* const self = *freeList;
			*freeList = self->m_next;
			return self;
		}
		else
		{
			return dMalloc(size);
		}
	}

	void operator delete (void* ptr)
	{
		FreeList** const freeList = GetFreeList();
		FreeList* const self = (FreeList*)ptr;
		self->m_next = *freeList;
		*freeList = self;
		m_size++;
	}

	static void FlushFreeList()
	{
		FreeList** const freeList = GetFreeList();
		FreeList* first = *freeList;
		while (first)
		{
			FreeList* const self = first;
			first = first->m_next;
			dFree(self);
		}
		m_size = 0;
		*freeList = nullptr;
	}

	private:
	static FreeList** GetFreeList()
	{
		static FreeList* freeList = nullptr;
		return &freeList;
	}
	static dUnsigned32 m_size;
};

template<class T>
dUnsigned32 dContainersFreeListAlloc<T>::m_size = 0;


#endif
