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

//#define	D_USE_SYNC_SEMAPHORE

//#define	D_MAX_THREADS_COUNT	16
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

		D_CORE_API ndUnsigned8 IsTaskInProgress() const;
		D_CORE_API void ExecuteTask(ndTask* const task);
	
		private:
		void TaskUpdate();
		virtual void ThreadFunction();

		ndThreadPool* m_owner;
		ndTask* m_task;
		ndInt32 m_threadIndex;
		#ifdef D_USE_SYNC_SEMAPHORE
		ndSemaphore m_taskReady;
		//std::binary_semaphore m_taskReady;
		#else
		ndUnsigned8 m_taskReady;
		#endif
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

	D_CORE_API void TickOne();
	D_CORE_API void Begin();
	D_CORE_API void End();

	template <typename Function>
	void ParallelExecute(const Function& ndFunction);

	D_CORE_API void TaskUpdate(ndInt32 threadIndex);

	private:
	D_CORE_API virtual void Release();
	D_CORE_API virtual void WaitForWorkers();

	ndWorker* m_workers;
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
	ndTaskImplement(ndInt32 threadIndex, ndThreadPool* const threadPool, const Function& ndFunction)
		:ndTask(threadIndex)
		,m_function(ndFunction)
		,m_threadPool(threadPool)
		,m_threadCount(threadPool->GetThreadCount())
	{
	}

	~ndTaskImplement()
	{
	}

	private:
	void Execute() const
	{
		m_function(m_threadIndex, m_threadCount);
	}

	Function m_function;
	ndThreadPool* m_threadPool;
	const ndInt32 m_threadCount;
	friend class ndThreadPool;
};

template <typename Function>
void ndThreadPool::ParallelExecute(const Function& callback)
{
	const ndInt32 threadCount = GetThreadCount();
	ndTaskImplement<Function>* const jobsArray = ndAlloca(ndTaskImplement<Function>, threadCount);

	for (ndInt32 i = 0; i < threadCount; ++i)
	{
		ndTaskImplement<Function>* const job = &jobsArray[i];
		new (job) ndTaskImplement<Function>(i, this, callback);
	}

	ndInt32 workerCount = threadCount - 1;
	if (workerCount > 0)
	{
		#ifdef	D_USE_THREAD_EMULATION
			for (ndInt32 i = 0; i < threadCount; ++i)
			{
				ndTaskImplement<Function>* const job = &jobsArray[i];
				callback(job->m_threadIndex, job->m_threadCount);
			}
		#else
			for (ndInt32 i = 0; i < workerCount; ++i)
			{
				ndTaskImplement<Function>* const job = &jobsArray[i + 1];
				m_workers[i].ExecuteTask(job);
			}
	
			ndTaskImplement<Function>* const job = &jobsArray[0];
			callback(job->m_threadIndex, job->m_threadCount);
			WaitForWorkers();
		#endif
	}
	else
	{
		ndTaskImplement<Function>* const job = &jobsArray[0];
		callback(job->m_threadIndex, job->m_threadCount);
	}
}

#endif
