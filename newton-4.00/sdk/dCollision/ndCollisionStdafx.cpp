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

#include "dCoreStdafx.h"
#include "ndCollisionStdafx.h"

#if defined (_D_COLLISION_DLL) && !defined (_D_SINGLE_LIBRARY)

	void *operator new (size_t size)
	{
		// this should not happens on this test
		// newton should never use global operator new and delete.
		dAssert(0);
		return dMemory::Malloc(size);
	}

	void operator delete (void* ptr)
	{
		dAssert(0);
		dMemory::Free(ptr);
	}

	BOOL APIENTRY DllMain(HMODULE , DWORD  ul_reason_for_call, LPVOID )
	{
		switch (ul_reason_for_call)
		{
			case DLL_PROCESS_ATTACH:
			case DLL_THREAD_ATTACH:
				#if defined(_DEBUG) && defined(_MSC_VER)
					// Track all memory leaks at the operating system level.
					// make sure no Newton tool or utility leaves leaks behind.
					_CrtSetDbgFlag(_CRTDBG_LEAK_CHECK_DF | _CRTDBG_REPORT_FLAG);
					//_CrtSetBreakAlloc(3342281);
				#endif

			case DLL_THREAD_DETACH:
			case DLL_PROCESS_DETACH:
				break;
		}
		return TRUE;
	}
#endif
