/* Copyright (c) <2003-2022> <Julio Jerez, Newton Game Dynamics>
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

#include "ndCoreStdafx.h"
#include "ndTypes.h"
#include "ndUtils.h"
#include "ndThread.h"
#include "ndProfiler.h"
#include "ndThreadSyncUtils.h"

#ifdef _MSC_VER
#pragma warning( push )
#pragma warning( disable : 4355)
#endif

ndThread::ndThread()
	:ndClassAlloc()
	,ndThreadName()
	,ndSemaphore()
#ifndef D_USE_THREAD_EMULATION
	,ndAtomic<bool>(true)
	,std::condition_variable()
	,std::thread(&ndThread::ThreadFunctionCallback, this)
#endif
{
	strcpy (m_name, "newtonWorker");
#ifndef D_USE_THREAD_EMULATION
	store(false);
#endif
}

#ifdef _MSC_VER
#pragma warning( pop )
#endif

ndThread::~ndThread()
{
}

void ndThread::SetName(const char* const name)
{
	strncpy(m_name, name, sizeof(m_name) - 1);
#if defined(_MSC_VER) && !defined (D_USE_THREAD_EMULATION)
	// a hideous way to set the thread name, bu this is how Microsoft does it
	const DWORD MS_VC_EXCEPTION = 0x406D1388;
	#pragma pack(push,8)  
		struct THREADNAME_INFO
		{
			DWORD dwType; // Must be 0x1000.  
			LPCSTR szName; // Pointer to name (in user addr space).  
			DWORD dwThreadID; // Thread ID (-1=caller thread).  
			DWORD dwFlags; // Reserved for future use, must be zero.  
		};
	#pragma pack(pop)  

	THREADNAME_INFO info;
	info.dwType = 0x1000;
	info.szName = name;
	info.dwThreadID = GetThreadId(std::thread::native_handle());
	info.dwFlags = 0;
	__try 
	{
		RaiseException(MS_VC_EXCEPTION, 0, sizeof(info) / sizeof(ULONG_PTR), (ULONG_PTR*)&info);
	}
	__except (EXCEPTION_EXECUTE_HANDLER)
	{
	}
#endif

	D_SET_TRACK_NAME(m_name);
}

void ndThread::Finish()
{
#ifndef D_USE_THREAD_EMULATION
	Terminate();
	join();
#endif
}

void ndThread::Signal()
{
#ifndef D_USE_THREAD_EMULATION
	ndSemaphore::Signal();
#endif
}

void ndThread::ThreadFunctionCallback()
{
#ifndef D_USE_THREAD_EMULATION
	// wait until constructor was fully initialized.
	while (load())
	{
		ndThreadYield();
	}

	D_SET_TRACK_NAME(m_name);
	ndFloatExceptions exception;

	while (!Wait())
	{
		ThreadFunction();
		Release();
	}
#endif
}



