/* Copyright (c) <2003-2019> <Newton Game Dynamics>
*
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
*
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "testStdafx.h"

// memory allocation for Newton
static void* PhysicsAlloc(size_t sizeInBytes)
{
	return new char[sizeInBytes];
}

// memory free use by the engine
static void PhysicsFree(void* ptr)
{
	delete[](char*)ptr;
}


class CheckMemoryLeaks
{
	public:
	CheckMemoryLeaks()
	{
		atexit(CheckMemoryLeaksCallback);
		// Set the memory allocation function before creation the newton world
		// this is the only function that can be called before the creation of the newton world.
		// it should be called once, and the the call is optional 
		dSetMemoryAllocators(PhysicsAlloc, PhysicsFree);

	#if defined(_DEBUG) && defined(_MSC_VER)
		// Track all memory leaks at the operating system level.
		// make sure no Newton tool or utility leaves leaks behind.
		_CrtSetDbgFlag(_CRTDBG_LEAK_CHECK_DF | _CRTDBG_LEAK_CHECK_DF);
		//_CrtSetBreakAlloc (318776);
	#endif
	}

	static void CheckMemoryLeaksCallback()
	{
		#if defined(_DEBUG) && defined(_MSC_VER)
		_CrtDumpMemoryLeaks();
		#endif
	}
};
static CheckMemoryLeaks checkLeaks;


void CreateBodyList(dNewton& newton)
{
	dShapeInstance box(new dShapeBox(1.0f, 1.0f, 1.0f));
	for (int i = 0; i < 2000; i++)
	{
		dDynamicBody* const body = new dDynamicBody();

		body->SetCollisionShape(box);
		body->SetMassMatrix(10.0f, box);

		newton.AddBody(body);
	}
}

int main (int argc, const char * argv[]) 
{
	dNewton newton;

	newton.SetThreadCount(4);
	newton.SetSubSteps(2);

	CreateBodyList(newton);

	for (int i = 0; i < 10000; i ++)
	{
		newton.Update(1.0f / 60.0f);
		//newton.Sync();
	}

	return 0;
}

