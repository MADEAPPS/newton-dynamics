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

#ifndef __ND_THREAD_POOL_H_
#define __ND_THREAD_POOL_H_

#include "ndCoreStdafx.h"
#include "ndTypes.h"
#include "ndArray.h"
#include "ndThread.h"
#include "ndSyncMutex.h"
#include "ndSemaphore.h"
#include "ndClassAlloc.h"

#define	D_MAX_THREADS_COUNT	32
#define D_WORKER_BATCH_SIZE	32

class ndThreadPool;

class ndStartEnd
{
	public:
	ndStartEnd(ndInt32 count, ndInt32 threadIndex, ndInt32 threads)
	{
		ndInt32 stride = count / threads;
		ndInt32 residual = count - stride * threads;
		m_start = stride * threadIndex;
		stride += (threadIndex < residual) ? 1 : 0;
		m_start += (threadIndex < residual) ? threadIndex : residual;
		m_end = m_start + stride;
	}

	ndInt32 m_start;
	ndInt32 m_end;
};

class ndTask
{
	public:
	ndTask(ndInt32 threadIndex)
		:m_threadIndex(threadIndex) 
	{
	}
	virtual ~ndTask(){}
	virtual void Execute() const = 0;

	const ndInt32 m_threadIndex;
};

class ndThreadPool: public ndSyncMutex, public ndThread
{
	class ndWorker: public ndThread
	{
		public:
		D_CORE_API ndWorker();
		D_CORE_API virtual ~ndWorker();
		D_CORE_API void ExecuteTask(ndTask* const task);
	
		private:
		void TaskUpdate();
		virtual void ThreadFunction();

		ndThreadPool* m_owner;
		ndTask* m_task;
		ndInt32 m_threadIndex;
		ndUnsigned8 m_begin;
		ndUnsigned8 m_stillLooping;
		friend class ndThreadPool;
	};

	public:
	D_CORE_API ndThreadPool(const char* const baseName);
	D_CORE_API virtual ~ndThreadPool();

	D_CORE_API ndInt32 GetThreadCount() const;
	D_CORE_API static ndInt32 GetMaxThreads();
	D_CORE_API void SetThreadCount(ndInt32 count);
	D_CORE_API virtual void WorkerUpdate(ndInt32 threadIndex);

	D_CORE_API virtual ndInt32 OptimalGroupBatch(ndInt32 numberOfGroups) const;

	D_CORE_API void TickOne();
	D_CORE_API void Begin();
	D_CORE_API void End();
	D_CORE_API void TaskUpdate(ndInt32 threadIndex);

	template <typename Function>
	void ParallelExecute(const Function& function, ndInt32 workGroupCount, ndInt32 groupsPerThreads = D_WORKER_BATCH_SIZE);

	private:
	D_CORE_API virtual void Release();
	D_CORE_API virtual void WaitForWorkers();

	ndWorker* m_workers;
	ndAtomic<ndInt32> m_taskInProgress;
	ndInt32 m_count;
	char m_baseName[32];
};

template <typename Type, typename ... Args>
class ndFunction
	:public ndFunction<decltype(&Type::operator())(Args...)>
{
};

template <typename Type>
class ndFunction<Type>
{
	public:
	ndFunction(const Type& obj)
		:m_object(obj)
	{
	}

	void operator()(ndInt32 threadIndex, ndInt32 threadCount) const
	{
		m_object.operator()(threadIndex, threadCount);
	}

	void operator()(ndInt32 groupId, ndInt32 threadIndex, ndInt32 threadCount) const
	{
		m_object.operator()(groupId, threadIndex, threadCount);
	}

	private:
	Type m_object;
};

namespace ndMakeObject
{
	template<typename Type> auto ndFunction(const Type & obj) -> decltype (::ndFunction<Type>(obj))
	{
		return ::ndFunction<Type>(obj);
	}
}

template <typename Function>
class ndTaskImplement : public ndTask
{
	public:
	ndTaskImplement(
		ndThreadPool* const threadPool,
		const Function& function,
		ndAtomic<ndInt32>& threadIterator,
		ndInt32 jobsCount,
		ndInt32 jobsStride,
		ndInt32 threadIndex)
		:ndTask(threadIndex)
		,m_function(function)
		,m_threadPool(threadPool)
		,m_threadIterator(threadIterator)
		,m_jobsCount(jobsCount)
		,m_jobsStride(jobsStride)
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
				m_function(batchIndex + j, m_threadIndex);
			}
		}
	}

	Function m_function;
	ndThreadPool* m_threadPool;
	ndAtomic<ndInt32>& m_threadIterator;
	const ndInt32 m_jobsCount;
	const ndInt32 m_jobsStride;
	friend class ndThreadPool;
};

template <typename Function>
void ndThreadPool::ParallelExecute(const Function& function, ndInt32 workGroupCount, ndInt32 groupsPerThreads)
{
	const ndInt32 threadCount = GetThreadCount();
	if (threadCount <= 1)
	{
		// in single threaded, just execute all jobs in the main thread
		for (ndInt32 i = 0; i < workGroupCount; ++i)
		{
			function(i, 0);
		}
	}
	else
	{
#ifdef	D_USE_THREAD_EMULATION
		// in emulation mode, threads execute all task in the main thread
		for (ndInt32 i = 0; i < workGroupCount; ++i)
		{
			function(i, 0);
		}
#else	

		// calculate number of thread needed
		ndAssert(groupsPerThreads >= 1);

		// enough work to use more than one core. get number of cores needed using batch size
		ndAtomic<ndInt32> threadIterator(0);
		const ndInt32 requiredThreads = (workGroupCount + groupsPerThreads - 1) / groupsPerThreads;
		const ndInt32 numberOfThreads = ndMax(ndMin(requiredThreads, threadCount) - 1, 0);
		ndTaskImplement<Function>* const jobsArray = ndAlloca(ndTaskImplement<Function>, numberOfThreads);
		for (ndInt32 i = 0; i < numberOfThreads; ++i)
		{
			ndTaskImplement<Function>* const job = &jobsArray[i];
			new (job) ndTaskImplement<Function>(this, function, threadIterator, workGroupCount, groupsPerThreads, i + 1);
		}
	
		//ndTrace(("start batches\n"));
		for (ndInt32 i = numberOfThreads - 1; i >= 0; --i)
		{
			ndTaskImplement<Function>* const job = &jobsArray[i];
			m_taskInProgress.fetch_add(1);
			m_workers[i].ExecuteTask(job);
		}
	
		for (ndInt32 batchIndex = threadIterator.fetch_add(groupsPerThreads); batchIndex < workGroupCount; batchIndex = threadIterator.fetch_add(groupsPerThreads))
		{
			const ndInt32 count = ((batchIndex + groupsPerThreads) < workGroupCount) ? groupsPerThreads : workGroupCount - batchIndex;
			for (ndInt32 j = 0; j < count; ++j)
			{
				function(batchIndex + j, 0);
			}
		}
		WaitForWorkers();
#endif
	}
}

#endif
