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

#include "ndCoreStdafx.h"
#include "ndTypes.h"
#include "ndUtils.h"
#include "ndThreadPool.h"
#include "ndProfiler.h"

void ndThreadPool::ndThreadLockFreeUpdate::Execute()
{
#ifndef	D_USE_THREAD_EMULATION
	m_begin.store(true);
	while (m_begin.load())
	{
		ndThreadPoolJob* const job = m_job.exchange(nullptr);
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
#endif
}

ndThreadPool::ndWorkerThread::ndWorkerThread()
	:ndThread()
	,m_job(nullptr)
	,m_threadIndex(0)
{
}

ndThreadPool::ndWorkerThread::~ndWorkerThread()
{
	Finish();
}

void ndThreadPool::ndWorkerThread::ThreadFunction()
{
	dAssert(m_job);
	D_SET_TRACK_NAME(m_name);
	m_job->Execute();
	m_owner->m_sync.Release();
}

void ndThreadPool::ndWorkerThread::ExecuteJob(ndThreadPoolJob* const job)
{
	m_job = job;
	m_job->m_threadIndex = m_threadIndex;
	m_owner->m_sync.Tick();
	Signal();
}

ndThreadPool::ndThreadPool(const char* const baseName)
	:ndSyncMutex()
	,ndThread()
	,m_sync()
	,m_workers(nullptr)
	,m_count(0)
	,m_joindInqueue(0)
{
	char name[256];
	strncpy(m_baseName, baseName, sizeof (m_baseName));
	sprintf(name, "%s_%d", m_baseName, 0);
	SetName(name);
	for (ndInt32 i = 0; i < D_MAX_THREADS_COUNT; i++)
	{
		m_lockFreeJobs[i].m_joindInqueue = &m_joindInqueue;
	}
}

ndThreadPool::~ndThreadPool()
{
	SetCount(0);
}

void ndThreadPool::SetCount(ndInt32 count)
{
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
			m_workers = new ndWorkerThread[count];
			for (ndInt32 i = 0; i < count; i++)
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

void ndThreadPool::ExecuteJobs(ndThreadPoolJob** const jobs)
{
#ifdef D_USE_THREAD_EMULATION	
	for (ndInt32 i = 0; i <= m_count; i++)
	{
		jobs[i]->m_threadIndex = i;
		m_lockFreeJobs[i].m_job = jobs[i];
		jobs[i]->Execute();
	}
#else
	if (m_count > 0)
	{
		m_joindInqueue.fetch_add(m_count);
		for (ndInt32 i = 0; i < m_count; i++)
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
#endif
}

void ndThreadPool::Begin()
{
	D_TRACKTIME();
	class ndDoNothing : public ndThreadPoolJob
	{
		virtual void Execute()
		{
			D_TRACKTIME();
		}
	};

	for (ndInt32 i = 0; i < m_count; i++)
	{
		m_workers[i].ExecuteJob(&m_lockFreeJobs[i]);
	}

	ndDoNothing extJob[D_MAX_THREADS_COUNT];
	ndThreadPoolJob* extJobPtr[D_MAX_THREADS_COUNT];
	for (ndInt32 i = 0; i < D_MAX_THREADS_COUNT; i++)
	{
		extJobPtr[i] = &extJob[i];
	}
	ExecuteJobs(extJobPtr);
}

void ndThreadPool::End()
{
	for (ndInt32 i = 0; i < m_count; i++)
	{
		m_lockFreeJobs[i].m_begin.store(false);
	}
	m_sync.Sync();
}

void ndThreadPool::TickOne()
{
	ndSyncMutex::Tick();
	ndSemaphore::Signal();
#ifdef D_USE_THREAD_EMULATION	
	ThreadFunction();
#endif
}

void ndThreadPool::Release()
{
	ndSyncMutex::Release();
}
