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
	:m_mutex()
	,m_condition()
	,m_count(0)
	,m_terminate(false)
{
}

dSemaphore::~dSemaphore()
{
}

int dSemaphore::GetCount()
{
	std::unique_lock<std::mutex> lock(m_mutex);
	return m_count;
}

bool dSemaphore::Wait()
{
	std::unique_lock<std::mutex> lock(m_mutex);
	while (m_count == 0)
	{
		m_condition.wait(lock);
	}

	m_count--;
	return m_terminate.load();
}


void dSemaphore::Signal()
{
	std::unique_lock<std::mutex> lock(m_mutex);
	m_count++;
	m_condition.notify_one();
}

void dSemaphore::Terminate()
{
	std::unique_lock<std::mutex> lock(m_mutex);
	m_count++;
	m_terminate.store(true);
	m_condition.notify_one();
}

