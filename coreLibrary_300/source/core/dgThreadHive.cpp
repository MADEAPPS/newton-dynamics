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
#include "dgTypes.h"
#include "dgMemory.h"
#include "dgThreadHive.h"




dgThreadHive::dgThreadBee::dgThreadBee()
	:dgThread()
	,m_isBusy(0)
	,m_ticks (0)
	,m_myMutex()
	,m_hive(NULL)
	,m_allocator(NULL)
	,m_getPerformanceCount(NULL)
{
}

dgThreadHive::dgThreadBee::~dgThreadBee()
{
	while (IsBusy());

	dgInterlockedExchange(&m_terminate, 1);
	m_myMutex.Release();
	Close();
}

void dgThreadHive::dgThreadBee::SetUp(dgMemoryAllocator* const allocator, const char* const name, dgInt32 id, dgThreadHive* const hive)
{
	m_allocator = allocator;
	m_hive = hive;
	Init (name, id);

	int priority = GetPriority();
	SetPriority(priority + 1);
}

bool dgThreadHive::dgThreadBee::IsBusy() const
{
	return m_isBusy ? true : false;
}

void dgThreadHive::dgThreadBee::SetPerfomanceCounter(OnGetPerformanceCountCallback callback)
{
	m_getPerformanceCount = callback;
}


void dgThreadHive::dgThreadBee::Execute (dgInt32 threadId)
{
	m_hive->OnBeginWorkerThread (threadId);

	while (!m_terminate) {
		dgInterlockedExchange(&m_isBusy, 0);
		SuspendExecution(m_myMutex);
		dgInterlockedExchange(&m_isBusy, 1);
		if (!m_terminate) {
			RunNextJobInQueue(threadId);
			m_hive->m_myMutex[threadId].Release();
		}
	}

	dgInterlockedExchange(&m_isBusy, 0);

	m_hive->OnEndWorkerThread (threadId);
}


void dgThreadHive::dgThreadBee::RunNextJobInQueue(dgInt32 threadId)
{
	dgUnsigned32 ticks = m_getPerformanceCount();

	bool isEmpty = false;
	do {
		dgThreadJob job;
		dgAssert (threadId == m_id);
		m_hive->m_jobsCriticalSection.Lock();
		isEmpty = m_hive->m_jobsPool.IsEmpty();
		if (!isEmpty) {
			job = m_hive->m_jobsPool.GetHead();
			m_hive->m_jobsPool.Pop();
		}
		m_hive->m_jobsCriticalSection.Unlock();

		if (!isEmpty) {
			job.m_callback (job.m_context0, job.m_context1, m_id);
		}
	} while (!isEmpty);

	
	m_ticks += (m_getPerformanceCount() - ticks);
}


dgThreadHive::dgThreadHive(dgMemoryAllocator* const allocator)
	:m_beesCount(0)
	,m_currentIdleBee(0)
	,m_workerBees(NULL)
	,m_myMasterThread(NULL)
	,m_allocator(allocator)
	,m_jobsCriticalSection()
	,m_globalCriticalSection()
    ,m_jobsPool(allocator)
{
}

dgThreadHive::~dgThreadHive()
{
	DestroyThreads();
}

void dgThreadHive::SetMatertThread (dgThread* const mastertThread)
{
	m_myMasterThread = mastertThread;
}

void dgThreadHive::DestroyThreads()
{
	if (m_beesCount) {
		delete[] m_workerBees;
		m_workerBees = NULL;
		m_beesCount = 0;
	}
}


void dgThreadHive::SetPerfomanceCounter(OnGetPerformanceCountCallback callback)
{
	for (dgInt32 i = 0; i < m_beesCount; i ++) {
		m_workerBees[i].SetPerfomanceCounter(callback);
	}
}


dgUnsigned32 dgThreadHive::GetPerfomanceTicks (dgUnsigned32 threadIndex) const
{
	return (m_beesCount && (threadIndex < dgUnsigned32(m_beesCount))) ? m_workerBees[threadIndex].m_ticks : 0;
}




dgInt32 dgThreadHive::GetThreadCount() const
{
	return m_beesCount ? m_beesCount : 1;
}

void dgThreadHive::ClearTimers()
{
	for (dgInt32 i = 0; i < m_beesCount; i ++) {
		m_workerBees[i].m_ticks = 0;
	}
}

dgInt32 dgThreadHive::GetMaxThreadCount() const
{
	return DG_MAX_THREADS_HIVE_COUNT;
}


void dgThreadHive::SetThreadsCount (dgInt32 threads)
{
	DestroyThreads();

	m_beesCount = dgMin (threads, DG_MAX_THREADS_HIVE_COUNT);
	if (m_beesCount == 1) {
		m_beesCount = 0;
	}

	if (m_beesCount) {
		m_workerBees = new (m_allocator) dgThreadBee[dgUnsigned32 (m_beesCount)];

		for (dgInt32 i = 0; i < m_beesCount; i ++) {
			char name[256];
			sprintf (name, "dgThreadBee%d", i);
			m_workerBees[i].SetUp(m_allocator, name, i, this);
		}
	}
}


void dgThreadHive::QueueJob (dgWorkerThreadTaskCallback callback, void* const context0, void* const context1)
{
	if (!m_beesCount) {
		callback (context0, context1, 0);
	} else {
		#ifdef DG_USE_THREAD_EMULATION
			callback (context0, context1, 0);
		#else 
			dgThreadJob job (context0, context1, callback);
			m_jobsPool.Push(job);
			if (m_jobsPool.IsFull()) {
				SynchronizationBarrier ();
			}
		#endif
	}
}


void dgThreadHive::OnBeginWorkerThread (dgInt32 threadId)
{
}

void dgThreadHive::OnEndWorkerThread (dgInt32 threadId)
{
}


void dgThreadHive::SynchronizationBarrier ()
{
	if (m_beesCount) {
		for (dgInt32 i = 0; i < m_beesCount; i ++) {
			m_workerBees[i].m_myMutex.Release();
		}

		m_myMasterThread->SuspendExecution(m_beesCount, m_myMutex);
		dgAssert (m_jobsPool.IsEmpty());
	}
}


