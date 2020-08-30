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

#ifndef _D_SEMAPHORE_H_
#define _D_SEMAPHORE_H_

#include "dCoreStdafx.h"

class dSemaphore
{
	public:
	D_CORE_API dSemaphore();
	D_CORE_API ~dSemaphore();

	D_CORE_API dInt32 GetCount();
	D_CORE_API void Signal();
	D_CORE_API bool Wait();
	D_CORE_API void Terminate();

#ifndef D_USE_THREAD_EMULATION
	private:
	std::mutex m_mutex;
	std::condition_variable m_condition;
	dInt32 m_count;
	dAtomic<bool> m_terminate;
#endif
};

#endif
