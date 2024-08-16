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
#include "ndThreadSyncUtils.h"

#ifndef D_USE_THREAD_EMULATION
void ndSpinLock::Delay(ndInt32& exp)
{
	#if defined (__x86_64) || defined(__x86_64__) || defined(_M_IX86) || defined(_M_X64)
		// adding exponential pause delay
		for (ndInt32 i = 0; i < exp; ++i)
		{
			_mm_pause();
			_mm_pause();
			_mm_pause();
			_mm_pause();
		}
     #else
		ndInt32 x = 0;
		volatile ndInt32 count = 1;
		for (ndInt32 i = 0; i < exp * 2; ++i)
		{
			x += count;
		}
    #endif
	exp = ndMin(exp * 2, 64);
}
#endif

void ndThreadYield()
{
	std::this_thread::yield();
}

void ndThreadPause()
{
	//#if 0
	#if defined (__x86_64) || defined(__x86_64__) || defined(_M_IX86) || defined(_M_X64)
	for (ndInt32 i = 0; i < 4; ++i)
	{
		_mm_pause();
		_mm_pause();
		_mm_pause();
		_mm_pause();
	}
	#else
	std::this_thread::yield();
	#endif
}

ndFloatExceptions::ndFloatExceptions(ndUnsigned32 mask)
{
	#if defined (_MSC_VER)
		_clearfp();
		m_floatMask = _controlfp(0, 0);
		_controlfp(m_floatMask & ~mask, _MCW_EM);
	#endif
	
	#if (defined(_M_IX86) || defined(__x86_64__) || defined(_M_X64))
		m_simdMask = _mm_getcsr();
		_MM_SET_DENORMALS_ZERO_MODE(_MM_DENORMALS_ZERO_ON);
		_MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON);
	#endif

	#if defined (__APPLE__)
		//#pragma message ("warning!!! apple flush to zero not defined for x86 platforms")
	#endif

	//ndFloat32 a = ndFloat32(1.0f);
	//ndFloat32 b = ndFloat32(0.1f);
	//ndFloat32 c = ndFloat32(0.0f);
	//ndInt32 count = 0;
	//while (a != 0.0f)
	//{
	//	a = a * b;
	//	count++;
	//}
	//count++;
}

ndFloatExceptions::~ndFloatExceptions()
{
	#if (defined(_M_IX86) || defined(__x86_64__) || defined(_M_X64))
		_mm_setcsr(m_simdMask);
	#endif

	//#if (defined (WIN32) || defined(_WIN32))
	#if defined (_MSC_VER)
		_clearfp();
		_controlfp(m_floatMask, _MCW_EM);
	#endif
}
