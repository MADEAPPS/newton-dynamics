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

#include "ndBrainStdafx.h"
#include "ndBrainThreadPool.h"

class ndBrainThreadPool::ndWorker : public ndThread
{
	public:
	ndWorker(const char* const name, ndBrainThreadPool* const owner, ndInt32)
		:ndThread()
		,m_task(nullptr)
		,m_owner(owner)
	{
		SetName(name);
	}

	virtual ~ndWorker()
	{
		Finish();
	}

	void SubmmitTask(ndTask* const task)
	{
		ndAssert(!m_task);
		m_task = task;
		m_owner->Tick();
		Signal();
	}

	private:
	void ThreadFunction()
	{
		m_task->Execute();
		m_task = nullptr;
		m_owner->Release();
	}

	ndTask* m_task;
	ndBrainThreadPool* m_owner;
	friend class ndBrainThreadPool;
};

ndBrainThreadPool::ndBrainThreadPool()
	:ndClassAlloc()
	,ndSyncMutex()
	,m_workers()
{
	for (ndInt32 i = 0; i < D_MAX_THREADS_COUNT; ++i)
	{
		m_workers.PushBack(nullptr);
	}

	m_workers.SetCount(0);
	SetThreadCount(1);
}

ndBrainThreadPool::~ndBrainThreadPool()
{
	#ifndef D_USE_BRAIN_THREAD_EMULATION
	for (ndInt32 i = 0; i < m_workers.GetCount(); ++i)
	{
		ndAssert(m_workers[i]);
		delete m_workers[i];
	}
	#endif
}

ndInt32 ndBrainThreadPool::GetThreadCount() const
{
	return m_workers.GetCount() + 1;
}

ndInt32 ndBrainThreadPool::GetMaxThreads()
{
	#ifdef D_USE_BRAIN_THREAD_EMULATION
		return D_MAX_THREADS_COUNT;
	#else
		//return ndClamp(ndInt32(std::thread::hardware_concurrency() + 1) / 2, 1, D_MAX_THREADS_COUNT);
		ndInt32 threadCount = ndInt32(std::thread::hardware_concurrency());
		return ndClamp((threadCount + 1) / 2, 1, D_MAX_THREADS_COUNT);
	#endif
}

void ndBrainThreadPool::SubmmitTask(ndTask* const task, ndInt32 index)
{
	m_workers[index]->SubmmitTask(task);
}

void ndBrainThreadPool::SetThreadCount(ndInt32 count)
{
	#ifdef D_USE_BRAIN_THREAD_EMULATION
		m_workers.SetCount (ndClamp(count, 1, D_MAX_THREADS_COUNT) - 1);
	#else
		ndInt32 maxThread = GetMaxThreads();
		count = ndClamp(count, 1, maxThread) - 1;
		if (count > m_workers.GetCount())
		{
			for (ndInt32 i = m_workers.GetCount(); i < count; ++i)
			{
				char name[256];
				snprintf(name, sizeof (name), "ndBrain_%d", i + 1);
				ndWorker* const worker = new ndWorker(name, this, i + 1);
				m_workers.PushBack(worker);
			}
		}
		else if (count < m_workers.GetCount())
		{
			for (ndInt32 i = m_workers.GetCount() - 1; i >= count; --i)
			{
				delete m_workers[i];
				m_workers[i] = nullptr;
			}
			m_workers.SetCount(count);
		}
	#endif
}
