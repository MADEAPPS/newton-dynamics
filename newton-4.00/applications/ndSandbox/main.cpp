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
#include "ndDemoEntityManager.h"

#ifdef _DEBUG
	#define ND_USE_LEAK_TRACKER
#endif

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

class ApplicationMemoryLeakTracket: public ndTree<ndUnsigned64, void*, ndLeakTrackerAllocator>
{
	public:
	ApplicationMemoryLeakTracket()
		:ndTree<ndUnsigned64, void*, ndLeakTrackerAllocator>()
		,m_lock()
		,m_allocIndex(0)
	{
	}

	~ApplicationMemoryLeakTracket()
	{
		ndAssert(!GetCount());
	}

	void InsertPointer(void* const ptr)
	{
		ndScopeSpinLock lock (m_lock);
		Insert(m_allocIndex, ptr);
		//ndAssert(m_allocIndex != 208);
		m_allocIndex++;
	}

	void RemovePointer(void* const ptr)
	{
		ndScopeSpinLock lock(m_lock);
		ndAssert(Find(ptr));
		Remove(ptr);
	}

	static ApplicationMemoryLeakTracket& GetLeakTracker()
	{
		static ApplicationMemoryLeakTracket leakTracker;
		return leakTracker;
	}

	ndSpinLock m_lock;
	ndUnsigned64 m_allocIndex;
};

// memory allocation for Newton
static void* PhysicsAlloc(size_t sizeInBytes)
{
	void* const ptr = malloc(sizeInBytes);
	ndAssert(ptr);

	#ifdef ND_USE_LEAK_TRACKER
	ApplicationMemoryLeakTracket::GetLeakTracker().InsertPointer(ptr);
	#endif
	
	return ptr;
}

// memory free use by the engine
static void PhysicsFree(void* ptr)
{
	#ifdef ND_USE_LEAK_TRACKER
	ApplicationMemoryLeakTracket::GetLeakTracker().RemovePointer(ptr);
	#endif

	free(ptr);
}

void* operator new (size_t size)
{
	void* const ptr = ndMemory::Malloc(size);
	ndAssert((ndUnsigned64(ptr) & (0x1f)) == 0);
	return ptr;
}

void operator delete (void* ptr) noexcept
{
	ndMemory::Free(ptr);
}

class ndSetAllocators
{
	public:
	ndSetAllocators()
	{
		ndMemory::SetMemoryAllocators(PhysicsAlloc, PhysicsFree);
	}
};
static ndSetAllocators setAllocators;

int main(int, char**)
{
	ndDemoEntityManager demos;
	demos.Run();
	return 0;
}

