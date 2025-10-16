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

#define	ND_THREAD_IDLE_ITERATIONS		32

ndThreadPool::ndWorker::ndWorker()
	:ndThread()
	,m_owner(nullptr)
	,m_task(nullptr)
	,m_threadIndex(0)
	,m_begin(0)
	,m_stillLooping(0)
{
}

ndThreadPool::ndWorker::~ndWorker()
{
	Finish();
}

void ndThreadPool::ndWorker::ExecuteTask(ndTask* const task)
{
	m_task = task;
}

void ndThreadPool::ndWorker::TaskUpdate()
{
	m_begin = 1;
	while (m_begin)
	{
		//D_TRACKTIME();
		if (m_task)
		{
			ndAssert(m_task->m_threadIndex == m_threadIndex);
			m_task->Execute();
			m_task = nullptr;
			m_owner->m_taskInProgress.fetch_add(-1);
		} 
		else
		{ 
			ndThreadYield();
		}
	}
}

void ndThreadPool::ndWorker::ThreadFunction()
{
#ifndef	D_USE_THREAD_EMULATION
	m_stillLooping = 1;
	m_owner->WorkerUpdate(m_threadIndex);
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

	auto BeginJobs = ndMakeObject::ndFunction([](ndInt32, ndInt32)
	{
		D_TRACKTIME_NAMED(BeginJobs);
	});
	ParallelExecute(BeginJobs, GetThreadCount(), 1);
}

void ndThreadPool::End()
{
	#ifndef	D_USE_THREAD_EMULATION
	for (ndInt32 i = 0; i < m_count; ++i)
	{
		m_workers[i].m_begin = 0;
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
		if (m_count && stillLooping)
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

ndInt32 ndThreadPool::OptimalGroupBatch(ndInt32 numberOfGroups) const
{
	// one of no more than two batches per hardware thread.
	ndInt32 threadCount = GetThreadCount() * 2;
	ndInt32 batchSize = ndMax(D_WORKER_BATCH_SIZE, numberOfGroups / threadCount);

	ndInt32 batchSizePowerOfTwo = D_WORKER_BATCH_SIZE;
	for (ndInt32 i = 0; (i < 5) && (batchSize > batchSizePowerOfTwo); ++i)
	{ 
		batchSizePowerOfTwo *= 2;
	}
	return batchSizePowerOfTwo;
}

void ndThreadPool::WaitForWorkers()
{
	for (ndInt32 test = 0; !m_taskInProgress.compare_exchange_weak(test, 0); test = 0)
	{
		ndThreadYield();
	}
}
