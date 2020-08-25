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
};

/*
class dContainerNodeAllocator
{
	public:
	dContainerNodeAllocator(int nodeSize)
		:m_freeListNode(NULL)
		,m_nodeSize(nodeSize)
	{
	}

	virtual ~DCONTAINERS_API dContainerNodeAllocator()
	{
	}

	DCONTAINERS_API virtual void Flush () = 0;
	DCONTAINERS_API virtual void* Alloc() = 0;
	DCONTAINERS_API virtual void Free(void* const ptr) = 0;

	protected:
	class dFreeListNode
	{
		public:
		int m_count;
		dFreeListNode* m_next;
	};

	dFreeListNode* m_freeListNode;
	int m_nodeSize;
};


class dContainerFreeListAllocator: public dContainerNodeAllocator
{
	public:
	dContainerFreeListAllocator(int nodeSize)
		:dContainerNodeAllocator(nodeSize)
		,m_count(0)
	{
	}

	virtual ~DCONTAINERS_API dContainerFreeListAllocator()
	{
		Flush();
	}

	DCONTAINERS_API virtual void Flush();
	DCONTAINERS_API virtual void* Alloc();
	DCONTAINERS_API virtual void Free(void* const ptr);

	protected:
	int m_count;
};


class dContainerFixSizeAllocator: public dContainerNodeAllocator
{
	public:
	static DCONTAINERS_API dContainerFixSizeAllocator* Create (int nodeSize, int poolSize);
	DCONTAINERS_API ~dContainerFixSizeAllocator();
	DCONTAINERS_API void* Alloc();
	DCONTAINERS_API void Free(void* const ptr);
	DCONTAINERS_API void Flush ();

	private:
	dContainerFixSizeAllocator(int size, int poolSize);
	void Prefetch ();

	int m_poolSize;
};
*/

#endif
