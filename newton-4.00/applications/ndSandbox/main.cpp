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

// memory allocation for Newton
static void* PhysicsAlloc(size_t sizeInBytes)
{
	void* const ptr = malloc(sizeInBytes);
	dAssert(ptr);
	return ptr;
}

// memory free use by the engine
static void PhysicsFree(void* ptr)
{
	free(ptr);
}

void *operator new (size_t size)
{
	// this should not happens on this test
	// newton should never use global operator new and delete.
	//dAssert(0);
	void* const ptr = dMemory::Malloc(size);
	dAssert((dUnsigned64(ptr) & (32-1)) == 0);
	return ptr;
}

void operator delete (void* ptr) noexcept
{
	dMemory::Free(ptr);
}

class CheckMemoryLeaks
{
	public:
	CheckMemoryLeaks()
	{
		#if defined(_DEBUG) && defined(_MSC_VER)
			// Track all memory leaks at the operating system level.
			// make sure no Newton tool or utility leaves leaks behind.
			_CrtSetDbgFlag(_CRTDBG_LEAK_CHECK_DF | _CRTDBG_REPORT_FLAG);
			//_CrtSetBreakAlloc (180);
		#endif

		atexit(CheckMemoryLeaksCallback);
		// Set the memory allocation function before creation the newton world
		// this is the only function that can be called before the creation of the newton world.
		// it should be called once, and the the call is optional 
		dMemory::SetMemoryAllocators(PhysicsAlloc, PhysicsFree);
	}

	static void CheckMemoryLeaksCallback()
	{
		#if defined(_DEBUG) && defined(_MSC_VER)
			_CrtDumpMemoryLeaks();
		#endif
	}
};
static CheckMemoryLeaks checkLeaks;

int main(int, char**)
{
	ndDemoEntityManager demos;
	demos.Run();
	return 0;
}

