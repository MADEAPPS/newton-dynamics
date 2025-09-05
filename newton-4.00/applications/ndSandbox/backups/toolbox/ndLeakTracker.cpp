/* Copyright (c) <2003-2022> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "ndSandboxStdafx.h"
#include "ndLeakTracker.h"

class ndLeakTrackerAllocator
{
	public:
	ndLeakTrackerAllocator()
	{
	}

	void* operator new (size_t size)
	{
		void* const mem = malloc(size);
		return mem;
	}

	void operator delete (void* ptr)
	{
		free(ptr);
	}
};

// keep track of all allocation, this should be disabled for production build 
class ApplicationMemoryLeakTracker: public ndTree<ndInt64, void*, ndLeakTrackerAllocator>
{
	public:
	ApplicationMemoryLeakTracker()
		:ndTree<ndInt64, void*, ndLeakTrackerAllocator>()
		,m_lock()
		,m_allocIndex(1)
		,m_startTracking(true)
	{
	}

	~ApplicationMemoryLeakTracker()
	{
		// need to make sure the free list are cleaned up after the last world was deleted
		ndFreeListAlloc::Flush();

		ndScopeSpinLock lock(m_lock);
		Iterator it(*this);
		for (it.Begin(); it; )
		{
			ndNode* const node = it.GetNode();
			it++;
			if (node->GetInfo() < 0)
			{
				Remove(node->GetKey());
			}
		}
		ndAssert(!GetCount());
	}

	void InsertPointer(void* const ptr)
	{
		ndScopeSpinLock lock (m_lock);
		ndInt64 allocIndex = m_startTracking ? m_allocIndex : -m_allocIndex;
		Insert(allocIndex, ptr);
		//ndAssert(m_allocIndex != 1486);
		m_allocIndex++;
	}

	void RemovePointer(void* const ptr)
	{
		ndScopeSpinLock lock(m_lock);
		ndAssert(Find(ptr));
		//ndAssert(Find(ptr)->GetInfo() != 2717);
		Remove(ptr);
	}

	static ApplicationMemoryLeakTracker& GetLeakTracker()
	{
		static ApplicationMemoryLeakTracker leakTracker;
		return leakTracker;
	}

	ndSpinLock m_lock;
	ndInt64 m_allocIndex;
	bool m_startTracking;
};

// memory allocation for Newton
static void* PhysicsAlloc(size_t sizeInBytes)
{
	void* const ptr = malloc(sizeInBytes);
	ndAssert(ptr);

	#ifdef ND_USE_LEAK_TRACKER
	ApplicationMemoryLeakTracker::GetLeakTracker().InsertPointer(ptr);
	#endif
	return ptr;
}

// memory free use by the engine
static void PhysicsFree(void* ptr)
{
	#ifdef ND_USE_LEAK_TRACKER
	ApplicationMemoryLeakTracker::GetLeakTracker().RemovePointer(ptr);
	#endif
	free(ptr);
}

// make sure new and delete are all directed to sdk memory callbacks
void* operator new (size_t size)
{
	static bool initialized = false;
	if (!initialized)
	{
		ndMemFreeCallback free;
		ndMemAllocCallback alloc;

		initialized = true;
		ndMemory::GetMemoryAllocators(alloc, free);
		if (alloc != PhysicsAlloc)
		{
			ndSetAllocators allocators;
			ApplicationMemoryLeakTracker& tracker = ApplicationMemoryLeakTracker::GetLeakTracker();
			tracker.m_startTracking = false;
		}
	}
	//void* const ptr = ndMemory::Malloc(size);
	ndIntPtr ptr;
	ptr.m_ptr = ndMemory::Malloc(size);
	ndAssert((ndUnsigned64(ptr.m_int) & (0x1f)) == 0);
	return ptr.m_ptr;
}

void operator delete (void* ptr) noexcept
{
	ndMemory::Free(ptr);
}

ndSetAllocators::ndSetAllocators()
{
	ndMemFreeCallback free;
	ndMemAllocCallback alloc;

	ApplicationMemoryLeakTracker& tracker = ApplicationMemoryLeakTracker::GetLeakTracker();
	tracker.m_startTracking = true;
	ndMemory::GetMemoryAllocators(alloc, free);
	if (alloc != PhysicsAlloc) 
	{
		ndMemory::SetMemoryAllocators(PhysicsAlloc, PhysicsFree);
	}
}
