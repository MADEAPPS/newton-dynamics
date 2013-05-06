/* Copyright (c) <2003-2011> <Julio Jerez, Newton Game Dynamics>
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

#include "dgStdafx.h"
#include "dgThread.h"
#include "dgAsyncThread.h"


dgAsyncThread::dgAsyncThread(const char* const name, dgInt32 id)
	:dgThread(name, id)
	,m_myMutex()
	,m_callerMutex()
{
	Init ();
}

dgAsyncThread::~dgAsyncThread(void)
{
	Terminate();
}


void dgAsyncThread::Terminate()
{
	if (StillBusy()) {
		dgInterlockedExchange(&m_terminate, 1);
		m_myMutex.Release();
		Close();
	}
} 


void dgAsyncThread::Execute (dgInt32 threadID)
{
	dgAssert (threadID == m_id);
	while (!m_terminate) {
		SuspendExecution(m_myMutex);
		m_callerMutex.Release();
		if (!m_terminate) {
			TickCallback(threadID);
		}
	}
}

void dgAsyncThread::Tick()
{
	m_myMutex.Release();
	SuspendExecution(m_callerMutex);
}

