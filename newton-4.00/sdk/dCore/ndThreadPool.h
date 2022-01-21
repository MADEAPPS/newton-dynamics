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

#ifndef __ND_THREAD_POOL_H_
#define __ND_THREAD_POOL_H_

#include "ndCoreStdafx.h"
#include "ndTypes.h"
#include "ndArray.h"
#include "ndThread.h"
#include "ndSyncMutex.h"
#include "ndSemaphore.h"
#include "ndClassAlloc.h"

#define	D_MAX_THREADS_COUNT	16

class ndThreadPool;

class ndStartEnd
{
	public:
	ndStartEnd(ndInt32 count, ndInt32 threadIndex, ndInt32 threads)
	{
		ndInt32 stride = count / threads;
		m_start = stride * threadIndex;
		m_end = (threadIndex != (threads - 1)) ? stride + m_start : count;
	}

	ndInt32 m_start;
	ndInt32 m_end;
};

class ndTask
{
	public:
	ndTask(){}
	virtual ~ndTask(){}
	virtual void Execute() const = 0;
	friend class ndThreadPool;
};

class ndThreadPoolJob_old
{
	public:
	ndThreadPoolJob_old() 
	{
	}

	virtual ~ndThreadPoolJob_old() 
	{
	}

	ndInt32 GetThreadId() const 
	{ 
		return m_threadIndex; 
	}

	ndInt32 GetThreadCount() const
	{
		return m_threadCount;
	}

	ndThreadPool* GetThreadPool() const
	{
		return m_threadPool;
	}

	void* GetContext() const
	{
		return m_context;
	}

	virtual void Execute() = 0;

	private:
	void* m_context;
	ndThreadPool* m_threadPool;
	ndInt32 m_threadCount;
	ndInt32 m_threadIndex;
	friend class ndThreadPool;
};

class ndThreadPool: public ndSyncMutex, public ndThread
{
	class ndWorkerThread: public ndThread
	{
		public:
		D_CORE_API ndWorkerThread();
		D_CORE_API virtual ~ndWorkerThread();

		private:
		virtual void ThreadFunction();

		ndThreadPool* m_owner;
		ndAtomic<bool> m_begin;
		ndAtomic<bool> m_stillLooping;
		ndAtomic<ndTask*> m_jobLambda;
		ndAtomic<ndThreadPoolJob_old*> m_jobOld;
		ndInt32 m_threadIndex;
		friend class ndThreadPool;
	};

	public:
	D_CORE_API ndThreadPool(const char* const baseName);
	D_CORE_API virtual ~ndThreadPool();

	ndInt32 GetThreadCount() const;
	D_CORE_API void SetThreadCount(ndInt32 count);

	D_CORE_API void TickOne();
	D_CORE_API void ExecuteJobs(ndThreadPoolJob_old** const jobs, void* const context);

	D_CORE_API void Begin();
	D_CORE_API void End();

	template <class T>
	void SubmitJobs(void* const context = nullptr);

	template <typename Function>
	void Execute(Function ndFunction);

	private:
	D_CORE_API virtual void Release();

	ndWorkerThread* m_workers;
	ndInt32 m_count;
	char m_baseName[32];
};

inline ndInt32 ndThreadPool::GetThreadCount() const
{
	return m_count + 1;
}

template <class T>
void ndThreadPool::SubmitJobs(void* const context)
{
	T* const extJob = dAlloca(T, D_MAX_THREADS_COUNT);
	ndThreadPoolJob_old* extJobPtr[D_MAX_THREADS_COUNT];

	const ndInt32 threadCount = GetThreadCount();
	for (ndInt32 i = 0; i < threadCount; i++)
	{
		new (&extJob[i]) T();
		extJobPtr[i] = &extJob[i];
	}
	ExecuteJobs(extJobPtr, context);
}

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

	template<typename... Args> typename
		std::result_of<Type(Args...)>::type operator()(Args... args)
	{
		m_object.operator()(args...);
	}

	template<typename... Args> typename
		std::result_of<const Type(Args...)>::type operator()(Args... args) const
	{
		m_object.operator()(args...);
	}

	private:
	Type m_object;
};

namespace ndMakeObject
{
	template<typename Type> auto ndFunction(const Type & obj)
	{
		return ::ndFunction<Type>(obj);
	}
}

template <typename Function>
class ndTaskImplement : public ndTask
{
	public:
	ndTaskImplement(ndInt32 threadIndex, ndThreadPool* const threadPool, Function ndFunction)
		:ndTask()
		,m_function(ndFunction)
		,m_threadPool(threadPool)
		,m_threadIndex(threadIndex)
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
	const ndInt32 m_threadIndex;
	const ndInt32 m_threadCount;
	friend class ndThreadPool;
};

template <typename Function>
void ndThreadPool::Execute(Function ndFunction)
{
	const ndInt32 threadCount = GetThreadCount();
	ndTaskImplement<Function>* const jobsArray = dAlloca(ndTaskImplement<Function>, threadCount);

	for (ndInt32 i = 0; i < threadCount; ++i)
	{
		ndTaskImplement<Function>* const job = &jobsArray[i];
		new (job) ndTaskImplement<Function>(i, this, ndFunction);
	}

	if (m_count > 0)
	{
		for (ndInt32 i = 0; i < m_count; ++i)
		{
			ndTaskImplement<Function>* const job = &jobsArray[i];
			m_workers[i].m_jobLambda.store(job);
		}

		ndTaskImplement<Function>* const job = &jobsArray[m_count];
		ndFunction(job->m_threadIndex, job->m_threadCount);

		bool jobsInProgress = true;
		do
		{
			bool inProgess = false;
			for (ndInt32 i = 0; i < m_count; ++i)
			{
				inProgess = inProgess | (m_workers[i].m_jobLambda.load() != nullptr);
			}
			jobsInProgress = jobsInProgress & inProgess;
			if (jobsInProgress)
			{
				std::this_thread::yield();
			}
		} while (jobsInProgress);
	}
	else
	{
		ndTaskImplement<Function>* const job = &jobsArray[0];
		ndFunction(job->m_threadIndex, job->m_threadCount);
	}
}

#endif
