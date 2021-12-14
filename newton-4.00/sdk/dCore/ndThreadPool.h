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

class ndThreadPoolJob
{
	public:
	ndThreadPoolJob() 
	{
	}

	virtual ~ndThreadPoolJob() 
	{
	}

	ndInt32 GetThreadId() const 
	{ 
		return m_threadIndex; 
	}

	virtual void Execute() = 0;

	private:
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
		void ExecuteJob(ndThreadPoolJob* const job);
		virtual void ThreadFunction();

		ndThreadPoolJob* m_job;
		ndThreadPool* m_owner;
		ndInt32 m_threadIndex;
		friend class ndThreadPool;
	};

	class ndThreadLockFreeUpdate: public ndThreadPoolJob
	{
		public:
		ndThreadLockFreeUpdate()
			:ndThreadPoolJob()
			,m_job(nullptr)
			,m_begin(false)
			,m_joindInqueue(nullptr)
		{
		}

		virtual void Execute();
		private:
		ndAtomic<ndThreadPoolJob*> m_job;
		ndAtomic<bool> m_begin;
		ndAtomic<ndInt32>* m_joindInqueue;
		friend class ndThreadPool;
	};

	public:
	D_CORE_API ndThreadPool(const char* const baseName);
	D_CORE_API virtual ~ndThreadPool();

	ndInt32 GetCount() const;
	D_CORE_API void SetCount(ndInt32 count);

	D_CORE_API void TickOne();
	D_CORE_API void ExecuteJobs(ndThreadPoolJob** const jobs);

	D_CORE_API void Begin();
	D_CORE_API void End();

	private:
	D_CORE_API virtual void Release();

	ndSyncMutex m_sync;
	ndWorkerThread* m_workers;
	ndInt32 m_count;
	char m_baseName[32];
	ndAtomic<ndInt32> m_joindInqueue;
	ndThreadLockFreeUpdate m_lockFreeJobs[D_MAX_THREADS_COUNT];
};

inline ndInt32 ndThreadPool::GetCount() const
{
	return m_count + 1;
}

#endif
