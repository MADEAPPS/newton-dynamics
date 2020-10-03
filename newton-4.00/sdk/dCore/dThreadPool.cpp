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
#include "dThreadPool.h"
#include "dProfiler.h"

#ifdef D_LOCK_FREE_THREADS_POOL
void dThreadPool::dThreadLockFreeUpdate::Execute()
{
	m_begin.store(true);
	while (m_begin.load())
	{
		dThreadPoolJob* const job = m_job.exchange(nullptr);
		if (job)
		{
			job->Execute();
			m_joindInqueue->fetch_add(-1);
		}
		else
		{
			std::this_thread::yield();
		}
	}
}
#endif

dThreadPool::dWorkerThread::dWorkerThread()
	:dClassAlloc()
	,dThread()
	,m_job(nullptr)
	,m_threadIndex(0)
{
}

dThreadPool::dWorkerThread::~dWorkerThread()
{
	Finish();
}

void dThreadPool::dWorkerThread::ThreadFunction()
{
	dAssert(m_job);
	D_SET_TRACK_NAME(m_name);
	m_job->Execute();
	m_owner->m_sync.Release();
}

void dThreadPool::dWorkerThread::ExecuteJob(dThreadPoolJob* const job)
{
	m_job = job;
	m_job->m_threadIndex = m_threadIndex;
	m_owner->m_sync.Tick();
	Signal();
}

dThreadPool::dThreadPool(const char* const baseName)
	:dSyncMutex()
	,dThread()
	,m_sync()
	,m_workers(nullptr)
	,m_count(0)
#ifdef D_LOCK_FREE_THREADS_POOL
	,m_joindInqueue(0)
#endif
{
	char name[256];
	strncpy(m_baseName, baseName, sizeof (m_baseName));
	sprintf(name, "%s_%d", m_baseName, 0);
	SetName(name);
#ifdef D_LOCK_FREE_THREADS_POOL
	for (dInt32 i = 0; i < D_MAX_THREADS_COUNT; i++)
	{
		m_lockFreeJobs[i].m_joindInqueue = &m_joindInqueue;
	}
#endif
}

dThreadPool::~dThreadPool()
{
	SetCount(0);
}

dInt32 dThreadPool::GetCount() const
{
	return m_count + 1;
}

void dThreadPool::SetCount(dInt32 count)
{
#ifdef D_USE_THREAD_EMULATION	
	count = 1;
#endif
	count = dClamp(count, 1, D_MAX_THREADS_COUNT) - 1;
	if (count != m_count)
	{
		if (m_workers)
		{
			m_count = 0;
			delete[] m_workers;
			m_workers = nullptr;
		}
		if (count)
		{
			m_count = count;
			m_workers = new dWorkerThread[count];
			for (dInt32 i = 0; i < count; i++)
			{
				char name[256];
				m_workers[i].m_owner = this;
				m_workers[i].m_threadIndex = i;
				sprintf(name, "%s_%d", m_baseName, i + 1);
				m_workers[i].SetName(name);
			}
		}
	}
}

void dThreadPool::ExecuteJobs(dThreadPoolJob** const jobs)
{
#ifdef D_LOCK_FREE_THREADS_POOL
	if (m_count > 0)
	{
		m_joindInqueue.fetch_add(m_count);
		for (dInt32 i = 0; i < m_count; i++)
		{
			jobs[i]->m_threadIndex = i;
			m_lockFreeJobs[i].m_job.store(jobs[i]);
		}

		jobs[m_count]->m_threadIndex = m_count;
		jobs[m_count]->Execute();
		while (m_joindInqueue.load())
		{
			std::this_thread::yield();
		}
	}
	else
	{
		jobs[0]->m_threadIndex = 0;
		jobs[0]->Execute();
	}
#else
	if (m_count > 0)
	{
		for (dInt32 i = 0; i < m_count; i++)
		{
			m_workers[i].ExecuteJob(jobs[i]);
		}

		jobs[m_count]->m_threadIndex = m_count;
		jobs[m_count]->Execute();
		m_sync.Sync();
	}
	else
	{
		jobs[0]->m_threadIndex = 0;
		jobs[0]->Execute();
	}
#endif
}

void dThreadPool::Begin()
{
#ifdef	D_LOCK_FREE_THREADS_POOL
	for (dInt32 i = 0; i < m_count; i++)
	{
		m_workers[i].ExecuteJob(&m_lockFreeJobs[i]);
	}
#endif
}

void dThreadPool::End()
{
#ifdef	D_LOCK_FREE_THREADS_POOL
	for (dInt32 i = 0; i < m_count; i++)
	{
		m_lockFreeJobs[i].m_begin.store(false);
	}
	m_sync.Sync();
#endif
}

void dThreadPool::TickOne()
{
	dSyncMutex::Tick();
	dSemaphore::Signal();
#ifdef D_USE_THREAD_EMULATION	
	ThreadFunction();
#endif
}

void dThreadPool::Release()
{
	dSyncMutex::Release();
}
