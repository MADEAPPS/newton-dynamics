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

#ifndef _ND_BRAIN_TREAD_POOL_H__
#define _ND_BRAIN_TREAD_POOL_H__

#include "ndBrainStdafx.h"

//#define D_USE_BRAIN_THREAD_EMULATION

class ndBrainThreadPool: public ndClassAlloc, public ndSyncMutex
{
	public: 
	class ndWorker;

	ndBrainThreadPool();
	~ndBrainThreadPool();

	ndInt32 GetThreadCount() const;
	static ndInt32 GetMaxThreads();
	void SetThreadCount(ndInt32 count);

	template <typename Function>
	void ParallelExecute(const Function& function);

	template <typename Function>
	void ParallelExecuteNew(const Function& function, ndInt32 numberOfJobs, ndInt32 numberOfJobsBatch = D_WORKER_BATCH_SIZE);

	private:
	void SubmmitTask(ndTask* const task, ndInt32 index);
	ndFixSizeArray<ndWorker*, D_MAX_THREADS_COUNT> m_workers;
};

template <typename Function>
class ndBrainTaskImplement: public ndTask
{
	public:
	ndBrainTaskImplement(ndInt32 threadIndex, ndBrainThreadPool* const threadPool, const Function& function)
		:ndTask(threadIndex)
		,m_function(function)
		,m_threadPool(threadPool)
		,m_threadCount(threadPool->GetThreadCount())
	{
	}

	~ndBrainTaskImplement()
	{
	}

	private:
	void Execute() const
	{
		m_function(m_threadIndex, m_threadCount);
	}

	Function m_function;
	ndBrainThreadPool* m_threadPool;
	const ndInt32 m_threadCount;
	friend class ndBrainThreadPool;
};

template <typename Function>
void ndBrainThreadPool::ParallelExecute(const Function& callback)
{
	const ndInt32 threadCount = GetThreadCount();
	ndBrainTaskImplement<Function>* const jobsArray = ndAlloca(ndBrainTaskImplement<Function>, threadCount);

	for (ndInt32 i = 0; i < threadCount; ++i)
	{
		ndBrainTaskImplement<Function>* const job = &jobsArray[i];
		new (job) ndBrainTaskImplement<Function>(i, this, callback);
	}

	if (m_workers.GetCount() > 0)
	{
		#ifdef D_USE_BRAIN_THREAD_EMULATION
		for (ndInt32 i = 0; i < threadCount; ++i)
		{
			ndBrainTaskImplement<Function>* const job = &jobsArray[i];
			callback(job->m_threadIndex, job->m_threadCount);
		}
		#else
		for (ndInt32 i = threadCount - 1; i > 0; --i)
		{
			ndBrainTaskImplement<Function>* const job = &jobsArray[i];
			SubmmitTask(job, i - 1);
		}
		ndBrainTaskImplement<Function>* const job = &jobsArray[0];
		callback(job->m_threadIndex, job->m_threadCount);
		Sync();
		#endif
	}
	else
	{
		ndBrainTaskImplement<Function>* const job = &jobsArray[0];
		callback(job->m_threadIndex, job->m_threadCount);
	}
}


template <typename Function>
class ndBrainTaskImplementNew : public ndTask
{
	public:
	ndBrainTaskImplementNew(
		ndBrainThreadPool* const threadPool,
		const Function& function,
		ndAtomic<ndInt32>& threadIterator,
		ndInt32 jobsCount,
		ndInt32 jobsStride,
		ndInt32 threadIndex)
		:ndTask(m_threadIndex)
		,m_function(function)
		,m_threadPool(threadPool)
		,m_threadIterator(threadIterator)
		,m_jobsCount(jobsCount)
		,m_jobsStride(jobsStride)
		//,m_threadIndex(threadIndex)
	{
	}

	private:
	void Execute() const
	{
		for (ndInt32 batchIndex = m_threadIterator.fetch_add(m_jobsStride); batchIndex < m_jobsCount; batchIndex = m_threadIterator.fetch_add(m_jobsStride))
		{
			//ndTrace(("t(%d) bat(%d) %x\n", m_threadIndex, batchIndex, &m_threadIterator));
			const ndInt32 count = ((batchIndex + m_jobsStride) < m_jobsCount) ? m_jobsStride : m_jobsCount - batchIndex;
			ndAssert(count <= m_jobsStride);
			for (ndInt32 j = 0; j < count; ++j)
			{
				m_function(batchIndex + j, 0);
			}
		}
	}

	Function m_function;
	ndBrainThreadPool* m_threadPool;
	ndAtomic<ndInt32>& m_threadIterator;
	const ndInt32 m_jobsCount;
	const ndInt32 m_jobsStride;
	//const ndInt32 m_threadIndex;
	friend class ndBrainThreadPool;
};

template <typename Function>
void ndBrainThreadPool::ParallelExecuteNew(const Function& function, ndInt32 numberOfJobs, ndInt32 numberOfJobsBatch)
{
	const ndInt32 threadCount = GetThreadCount();
	if (threadCount <= 1)
	{
		// in single threaded, just execute all jobs in the main thread
		for (ndInt32 i = 0; i < numberOfJobs; ++i)
		{
			function(i, 0);
		}
	}
	else
	{
		// calculate number of thread needed
		ndAssert(numberOfJobsBatch >= 1);
		const ndInt32 virtualThreadCount = numberOfJobs / numberOfJobsBatch;
		if (virtualThreadCount < 2)
		{
			// not enough jobs to use all cores, just dispact all job in main thread
			for (ndInt32 i = 0; i < numberOfJobs; ++i)
			{
				function(i, 0);
			}
		}
		else
		{
			// enough work to use more than one core. get number of cores needed using batch size
			ndAtomic<ndInt32> threadIterator(0);
			const ndInt32 numberOfThreads = ndMin(virtualThreadCount, threadCount);
			ndBrainTaskImplementNew<Function>* const jobsArray = ndAlloca(ndBrainTaskImplementNew<Function>, numberOfThreads);
			for (ndInt32 i = 0; i < numberOfThreads; ++i)
			{
				ndBrainTaskImplementNew<Function>* const job = &jobsArray[i];
				new (job) ndBrainTaskImplementNew<Function>(this, function, threadIterator, numberOfJobs, numberOfJobsBatch, i);
			}

			//ndTrace(("start batches\n"));
			for (ndInt32 i = numberOfThreads - 1; i > 0; --i)
			{
				ndInt32 threadSlot = i - 1;
				ndBrainTaskImplementNew<Function>* const job = &jobsArray[i];
				SubmmitTask(job, threadSlot);
			}

			for (ndInt32 batchIndex = threadIterator.fetch_add(numberOfJobsBatch); batchIndex < numberOfJobs; batchIndex = threadIterator.fetch_add(numberOfJobsBatch))
			{
				const ndInt32 count = ((batchIndex + numberOfJobsBatch) < numberOfJobs) ? numberOfJobsBatch : numberOfJobs - batchIndex;
				for (ndInt32 j = 0; j < count; ++j)
				{
					function(batchIndex + j, 0);
				}
			}

			Sync();
		}
	}
}
#endif 

