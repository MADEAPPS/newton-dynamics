/* Copyright (c) <2018-2018> <Newton Game Dynamics>
*
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
*
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

// dllmain.cpp : Defines the entry point for the DLL application.

//#pragma warning (disable: 4100) //unreferenced formal parameter

#ifdef WIN32 
#include <windows.h>
#include <crtdbg.h>

class CheckMemoryLeaks
{
public:
	CheckMemoryLeaks()
	{
		#if defined(_DEBUG) && defined(_MSC_VER)
			// Track all memory leaks at the operating system level.
			// make sure no Newton tool or utility leaves leaks behind.
			_CrtSetDbgFlag(_CRTDBG_LEAK_CHECK_DF | _CRTDBG_REPORT_FLAG);
			//_CrtSetBreakAlloc(168);
		#endif

		atexit(CheckMemoryLeaksCallback);
		// Set the memory allocation function before creation the newton world
		// this is the only function that can be called before the creation of the newton world.
		// it should be called once, and the the call is optional 
		//dMemory::SetMemoryAllocators(PhysicsAlloc, PhysicsFree);
	}

	static void CheckMemoryLeaksCallback()
	{
#if defined(_DEBUG) && defined(_MSC_VER)
		_CrtDumpMemoryLeaks();
#endif
	}
};
//static CheckMemoryLeaks checkLeaks;


BOOL APIENTRY DllMain( HMODULE hModule,
                       DWORD  ul_reason_for_call,
                       LPVOID lpReserved
					 )
{
	switch (ul_reason_for_call)
	{
	case DLL_PROCESS_ATTACH:
	case DLL_THREAD_ATTACH:
	case DLL_THREAD_DETACH:
	case DLL_PROCESS_DETACH:
		break;
	}
	return TRUE;
}
#endif
