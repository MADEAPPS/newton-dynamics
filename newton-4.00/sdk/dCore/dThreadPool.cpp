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

dThreadPool::dThreadPool()
	:m_sync()
	,m_workers(nullptr)
	,m_count(0)
{
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
	count = dClamp(count, 1, D_MAX_THREADS_COUNT) - 1;
	if (count != m_count)
	{
		if (m_workers)
		{
			m_count = 0;
			delete[] m_workers;
			m_workers = nullptr;
		}
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
			sprintf(name, "newton Worker %d", i);
			m_workers[i].SetName(name);
			m_workers[i].Start();
		}
	}
}

void dThreadPool::ExecuteJobs(dThreadPoolJob** const jobs)
{
	for (dInt32 i = 0; i < m_count; i++)
	{
		m_workers[i].ExecuteJob(jobs[i]);
	}

	jobs[m_count]->m_threadIndex = m_count;
	jobs[m_count]->Execute();
	m_sync.Sync();
}
