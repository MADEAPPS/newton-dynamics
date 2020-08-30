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
#include "dSemaphore.h"

dSemaphore::dSemaphore()
#ifndef D_USE_THREAD_EMULATION
	:m_mutex()
	,m_condition()
	,m_count(0)
	,m_terminate(false)
#endif
{
}

dSemaphore::~dSemaphore()
{
}

dInt32 dSemaphore::GetCount()
{
#ifdef D_USE_THREAD_EMULATION
	return 0;
#else
	std::unique_lock<std::mutex> lock(m_mutex);
	return m_count;
#endif
}

bool dSemaphore::Wait()
{
#ifdef D_USE_THREAD_EMULATION
	return false;
#else
	std::unique_lock<std::mutex> lock(m_mutex);
	while (m_count == 0)
	{
		m_condition.wait(lock);
	}

	m_count--;
	return m_terminate.load();
#endif
}

void dSemaphore::Signal()
{
#ifndef D_USE_THREAD_EMULATION
	std::unique_lock<std::mutex> lock(m_mutex);
	m_count++;
	m_condition.notify_one();
#endif
}

void dSemaphore::Terminate()
{
#ifndef D_USE_THREAD_EMULATION
	std::unique_lock<std::mutex> lock(m_mutex);
	m_count++;
	m_terminate.store(true);
	m_condition.notify_one();
#endif
}

