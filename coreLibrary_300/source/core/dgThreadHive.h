/* Copyright (c) <2003-2011> <Julio Jerez, Newton Game Dynamics>
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


#ifndef __DG_THREAD_HIVE_H__
#define __DG_THREAD_HIVE_H__

#include "dgThread.h"
#include "dgMemory.h"
#include "dgFastQueue.h"



//#define DG_THREAD_POOL_JOB_SIZE (512)
#define DG_THREAD_POOL_JOB_SIZE (1024 * 8)

typedef void (*dgWorkerThreadTaskCallback) (void* const context0, void* const context1, dgInt32 threadID);

class dgThreadHive  
{
	public:

	class dgThreadJob
	{
		public:
		dgThreadJob()
		{
		}

		dgThreadJob (void* const context0, void* const context1, dgWorkerThreadTaskCallback callback)
			:m_context0(context0)
			,m_context1(context1)
			,m_callback(callback)
		{
		}
		void* m_context0;
		void* m_context1;
		dgWorkerThreadTaskCallback m_callback;
	};


	class dgThreadBee: public dgThread
	{
		public:
		DG_CLASS_ALLOCATOR(allocator)

		dgThreadBee();
		~dgThreadBee();

		bool IsBusy() const;
		void SetPerfomanceCounter(OnGetPerformanceCountCallback callback);
		void SetUp(dgMemoryAllocator* const allocator, const char* const name, dgInt32 id, dgThreadHive* const hive);
		virtual void Execute (dgInt32 threadId);

		void RunNextJobInQueue(dgInt32 threadId);

		dgInt32 m_isBusy;

		dgUnsigned32 m_ticks;
		dgSemaphore m_myMutex;
		dgThreadHive* m_hive;
		dgMemoryAllocator* m_allocator; 
		OnGetPerformanceCountCallback m_getPerformanceCount;	
	};

	dgThreadHive(dgMemoryAllocator* const allocator);
	virtual ~dgThreadHive();

	virtual void OnBeginWorkerThread (dgInt32 threadId);
	virtual void OnEndWorkerThread (dgInt32 threadId);

	void SetMatertThread (dgThread* const mastertThread);

	void GlobalLock() const;
	void GlobalUnlock() const;

	void GetIndirectLock (dgThread::dgCriticalSection* const criticalSectionLock) const;
	void ReleaseIndirectLock (dgThread::dgCriticalSection* const criticalSectionLock) const;

	void ClearTimers();
	dgInt32 GetThreadCount() const;

	dgInt32 GetMaxThreadCount() const;
	void SetThreadsCount (dgInt32 count);

	void QueueJob (dgWorkerThreadTaskCallback callback, void* const context0, void* const context1);
	void SynchronizationBarrier ();

	void SetPerfomanceCounter(OnGetPerformanceCountCallback callback);
	dgUnsigned32 GetPerfomanceTicks (dgUnsigned32 threadIndex) const;

	private:
	void DestroyThreads();

	dgInt32 m_beesCount;
	dgInt32 m_currentIdleBee;
	dgThreadBee* m_workerBees;
	dgThread* m_myMasterThread;
	dgMemoryAllocator* m_allocator;
	dgThread::dgCriticalSection m_jobsCriticalSection;
	mutable dgThread::dgCriticalSection m_globalCriticalSection;
	dgThread::dgSemaphore m_myMutex[DG_MAX_THREADS_HIVE_COUNT];
	dgFastQueue<dgThreadJob, DG_THREAD_POOL_JOB_SIZE> m_jobsPool;
};


inline void dgThreadHive::GlobalLock() const
{
	GetIndirectLock(&m_globalCriticalSection);
}

inline void dgThreadHive::GlobalUnlock() const
{	
	ReleaseIndirectLock(&m_globalCriticalSection);
}


inline void dgThreadHive::GetIndirectLock (dgThread::dgCriticalSection* const criticalSectionLock) const
{
	if (m_beesCount) {	
		criticalSectionLock->Lock();
	}
}

inline void dgThreadHive::ReleaseIndirectLock (dgThread::dgCriticalSection* const criticalSectionLock) const
{
	if (m_beesCount) {	
		criticalSectionLock->Unlock();
	}
}

class dgThreadHiveScopeLock
{
	public:
	dgThreadHiveScopeLock(const dgThreadHive* const me, dgThread::dgCriticalSection* const criticalSectionLock)
		:m_me(me)
		,m_lock (criticalSectionLock)
	{
		me->GetIndirectLock (criticalSectionLock);
	}

	~dgThreadHiveScopeLock()
	{
		m_me->ReleaseIndirectLock (m_lock);
	}

	const dgThreadHive* m_me;
	dgThread::dgCriticalSection* m_lock; 
};



#endif
