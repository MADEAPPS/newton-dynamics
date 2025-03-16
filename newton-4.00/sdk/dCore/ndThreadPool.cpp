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
#include "ndProfiler.h"
#include "ndThreadPool.h"
#include "ndThreadSyncUtils.h"

ndThreadPool::ndWorker::ndWorker()
	:ndThread()
	,m_owner(nullptr)
	,m_task(nullptr)
	,m_threadIndex(0)
#ifdef D_USE_SYNC_SEMAPHORE
	,m_taskReady()
#else
	,m_taskReady(0)
#endif
	,m_begin(0)
	,m_stillLooping(0)
{
}

ndThreadPool::ndWorker::~ndWorker()
{
	Finish();
}

ndUnsigned8 ndThreadPool::ndWorker::IsTaskInProgress() const
{
	#ifdef D_USE_SYNC_SEMAPHORE
	return ndUnsigned8 (m_task ? 1 : 0);
	#else
	return m_taskReady;
	#endif
}

void ndThreadPool::ndWorker::ExecuteTask(ndTask* const task)
{
	m_task = task;
#ifdef D_USE_SYNC_SEMAPHORE
	m_taskReady.Signal();
#else
	m_taskReady = 1;
#endif
}

void ndThreadPool::ndWorker::TaskUpdate()
{
	m_begin = 1;
	ndInt32 iterations = 0;
	while (m_begin)
	{
		if (m_taskReady)
		{
			//D_TRACKTIME();
			if (m_task)
			{
				ndAssert(m_task->m_threadIndex == m_threadIndex);
				m_task->Execute();
			}
			iterations = 0;
			m_taskReady = 0;
		}
		else
		{
			if (iterations == 32)
			{
				// make sure that OS has the chance to task switch
				ndThreadYield();
				iterations = 0;
			}
			else
			{
				ndThreadPause();
			}
			iterations++;
		}
	}
}

void ndThreadPool::ndWorker::ThreadFunction()
{
#ifndef	D_USE_THREAD_EMULATION
	m_stillLooping = 1;

#ifdef D_USE_SYNC_SEMAPHORE
	while (!m_taskReady.Wait() && m_task)
	{
		//D_TRACKTIME();
		m_task->Execute();
		m_task = nullptr;
	}
#else

	m_owner->WorkerUpdate(m_threadIndex);
#endif
	m_stillLooping = 0;
#endif
}

ndThreadPool::ndThreadPool(const char* const baseName)
	:ndSyncMutex()
	,ndThread()
	,m_workers(nullptr)
	,m_count(0)
{
	char name[256];
	strncpy(m_baseName, baseName, sizeof (m_baseName));
	snprintf(name, sizeof (name), "%s_%d", m_baseName, 0);
	SetName(name);
}

ndThreadPool::~ndThreadPool()
{
	SetThreadCount(0);
}

ndInt32 ndThreadPool::GetMaxThreads()
{
	#ifdef D_USE_THREAD_EMULATION
		return D_MAX_THREADS_COUNT;
	#else
		return ndClamp(ndInt32(std::thread::hardware_concurrency() + 1) / 2, 1, D_MAX_THREADS_COUNT);
	#endif
}

void ndThreadPool::TaskUpdate(ndInt32 threadIndex)
{
	ndAssert(threadIndex >= 1);
	ndAssert(threadIndex <= m_count);
	m_workers[threadIndex - 1].TaskUpdate();
}

void ndThreadPool::WorkerUpdate(ndInt32 threadIndex)
{
	ndAssert(0);
	TaskUpdate(threadIndex);
}

ndInt32 ndThreadPool::GetThreadCount() const
{
	return m_count + 1;
}

void ndThreadPool::SetThreadCount(ndInt32 count)
{
#ifdef D_USE_THREAD_EMULATION
	m_count = ndClamp(count, 1, D_MAX_THREADS_COUNT) - 1;
#else
	ndInt32 maxThread = GetMaxThreads();
	count = ndClamp(count, 1, maxThread) - 1;
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
			m_workers = new ndWorker[size_t(count)];
			for (ndInt32 i = 0; i < count; ++i)
			{
				char name[256];
				m_workers[i].m_owner = this;
				m_workers[i].m_threadIndex = i + 1;
				snprintf(name, sizeof(name), "%s_%d", m_baseName, i + 1);
				m_workers[i].SetName(name);
			}
		}
	}
#endif
}

void ndThreadPool::Begin()
{
	D_TRACKTIME();
	#ifndef	D_USE_THREAD_EMULATION
	for (ndInt32 i = 0; i < m_count; ++i)
	{
		m_workers[i].Signal();
	}
	#endif

	auto BeginJobs = ndMakeObject::ndFunction([this](ndInt32, ndInt32)
	{
		D_TRACKTIME_NAMED(BeginJobs);
	});
	ParallelExecute(BeginJobs);
}

void ndThreadPool::End()
{
	#ifndef	D_USE_THREAD_EMULATION
	for (ndInt32 i = 0; i < m_count; ++i)
	{
		m_workers[i].ExecuteTask(nullptr);
		#if !defined(D_USE_SYNC_SEMAPHORE)
		m_workers[i].m_begin = 0;
		#endif
	}

	ndUnsigned8 stillLooping = 1;
	do 
	{
		ndUnsigned8 looping = 0;
		for (ndInt32 i = 0; i < m_count; ++i)
		{
			looping = ndUnsigned8(looping | m_workers[i].m_stillLooping);
		}
		stillLooping = ndUnsigned8(stillLooping & looping);
		if (m_count)
		{
			ndThreadYield();
		}
	} while (stillLooping);
	#endif
}

void ndThreadPool::Release()
{
	ndSyncMutex::Release();
}

void ndThreadPool::TickOne()
{
	ndSyncMutex::Tick();
	ndSemaphore::Signal();
#ifdef D_USE_THREAD_EMULATION	
	ThreadFunction();
#endif
}

void ndThreadPool::WaitForWorkers()
{
	ndInt32 iterations = 0;
	ndUnsigned8 jobsInProgress = 1;
	do
	{
		ndUnsigned8 inProgess = 0;
		for (ndInt32 i = 0; i < m_count; ++i)
		{
			inProgess = ndUnsigned8(inProgess | (m_workers[i].IsTaskInProgress()));
		}
		jobsInProgress = ndUnsigned8 (jobsInProgress & inProgess);
		if (jobsInProgress)
		{
			if ((iterations & -32) == -32)
			{
				ndThreadYield();
			}
			else
			{
				ndThreadPause();
			}
			iterations++;
		}
	} while (jobsInProgress);
	//if (iterations > 10000)
	//{
	//	ndExpandTraceMessage("xxx %d\n", iterations);
	//}
}

