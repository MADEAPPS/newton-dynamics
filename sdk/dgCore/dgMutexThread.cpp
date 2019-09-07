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

#include "dgStdafx.h"
#include "dgThread.h"
#include "dgMutexThread.h"


dgMutexThread::dgMutexThread(const char* const name, dgInt32 id)
	:dgThread(name, id)
	,m_mutex()
	,m_parentMutex()
{
	Init ();
}

dgMutexThread::~dgMutexThread(void)
{
	Terminate();
}

void dgMutexThread::Terminate()
{
	if (IsThreadActive()) {
		dgInterlockedExchange(&m_terminate, 1);
		m_mutex.Release();
		Close();
	}
} 

void dgMutexThread::Execute (dgInt32 threadID)
{
	// suspend this tread until the call thread decide to 
	dgAssert (threadID == m_id);
	while (!m_terminate) {
		// wait for the main thread to signal an update
		m_mutex.Wait();
		if (!m_terminate) {
			TickCallback(threadID);
		}
		m_parentMutex.Release();
	}
}

void dgMutexThread::Tick()
{
	// let the thread run until the update function return  
	m_mutex.Release();
	m_parentMutex.Wait();
}

dgAsyncThread::dgAsyncThread(const char* const name, dgInt32 id)
	:dgThread(name, id)
	,m_mutex()
	,m_inUpdate(0)
	,m_beginUpdate(0)
{
	Init();
}

dgAsyncThread::~dgAsyncThread(void)
{
	Terminate();
}

void dgAsyncThread::Terminate()
{
	if (IsThreadActive()) {
		dgInterlockedExchange(&m_terminate, 1);
		m_mutex.Release();
		Close();
	}
}

void dgAsyncThread::Sync()
{
	while (m_inUpdate) {
		dgThreadYield();
	}
}

void dgAsyncThread::Tick()
{
	// let the thread run until the update function return  
	Sync();
	m_beginUpdate = 0;
	m_mutex.Release();
	while (!m_beginUpdate) {
		dgThreadPause();
	}
}


void dgAsyncThread::Execute(dgInt32 threadID)
{
	dgAssert(threadID == m_id);
	while (!m_terminate) {
		m_mutex.Wait();
		if (!m_terminate) {
			dgInterlockedExchange(&m_inUpdate, 1);
			dgInterlockedExchange(&m_beginUpdate, 1);
			TickCallback(threadID);
			dgInterlockedExchange(&m_inUpdate, 0);
		}
	}
}