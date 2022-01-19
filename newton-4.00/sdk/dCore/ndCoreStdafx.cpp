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

#include "ndCoreStdAfx.h"

#ifdef _D_CORE_DLL
	#include "ndTypes.h"
	#include "ndMemory.h"

	#ifndef D_USE_DEFAULT_NEW_AND_DELETE
	void *operator new (size_t size)
	{
		// this should not happens on this test
		// newton should never use global operator new and delete.
		return ndMemory::Malloc(size);
	}

	void operator delete (void* ptr) noexcept
	{
		ndMemory::Free(ptr);
	}
	#endif

#if (defined(WIN32) || defined(_WIN32))
	BOOL APIENTRY DllMain(HMODULE, DWORD  ul_reason_for_call, LPVOID)
	{
		switch (ul_reason_for_call)
		{
			case DLL_PROCESS_ATTACH:
			case DLL_THREAD_ATTACH:
			{
				#if defined(_DEBUG) && defined(_MSC_VER)
					// Track all memory leaks at the operating system level.
					// make sure no Newton tool or utility leaves leaks behind.
					ndUnsigned32 flags = _CrtSetDbgFlag(_CRTDBG_LEAK_CHECK_DF) & 0xffff;
					flags = flags | _CRTDBG_REPORT_FLAG;
					flags = flags | _CRTDBG_CHECK_EVERY_1024_DF;
					_CrtSetDbgFlag(flags);
					//_CrtSetBreakAlloc(3342281);
				#endif
			}

			case DLL_THREAD_DETACH:
			case DLL_PROCESS_DETACH:
				break;
		}
		return TRUE;
	}
#endif
#endif
