/* Copyright (c) <2003-2021> <Newton Game Dynamics>
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
		,m_allocIndex(0)
	{
	}

	~ApplicationMemoryLeakTracket()
	{
		dAssert(!GetCount());
	}

	static ApplicationMemoryLeakTracket& GetLeakTracker()
	{
		static ApplicationMemoryLeakTracket leakTracker;
		return leakTracker;
	}

	ndUnsigned64 m_allocIndex;
};

// memory free use by the engine
static void PhysicsFree(void* ptr)
{
#ifdef _DEBUG
	ApplicationMemoryLeakTracket& leakTracker = ApplicationMemoryLeakTracket::GetLeakTracker();
	dAssert(leakTracker.Find(ptr));
	leakTracker.Remove(ptr);
#endif

	free(ptr);
}

// memory allocation for Newton
static void* PhysicsAlloc(size_t sizeInBytes)
{
	void* const ptr = malloc(sizeInBytes);
	dAssert(ptr);

#ifdef _DEBUG
	ApplicationMemoryLeakTracket& leakTracker = ApplicationMemoryLeakTracket::GetLeakTracker();
	leakTracker.Insert(leakTracker.m_allocIndex, ptr);
	leakTracker.m_allocIndex++;
#endif
	
	return ptr;
}

void* operator new (size_t size)
{
	void* const ptr = ndMemory::Malloc(size);
	dAssert((ndUnsigned64(ptr) & (0x1f)) == 0);
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

