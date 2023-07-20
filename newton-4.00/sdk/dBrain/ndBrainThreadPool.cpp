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
	ndWorker();
	virtual ~ndWorker();

	private:
	virtual void ThreadFunction();

	ndBrainThreadPool* m_owner;
	ndAtomic<bool> m_begin;
	ndAtomic<bool> m_stillLooping;
	ndAtomic<ndTask*> m_task;
	ndInt32 m_threadIndex;
	friend class ndBrainThreadPool;
};

ndBrainThreadPool::ndBrainThreadPool()
	:ndClassAlloc()
	,m_workers()
{
	m_workers.SetCount(0);
}

ndBrainThreadPool::~ndBrainThreadPool()
{
	for (ndInt32 i = 0; i < m_workers.GetCount(); ++i)
	{
		ndAssert(0);
	}
	//Finish();
}

ndInt32 ndBrainThreadPool::GetThreadCount() const
{
	return m_workers.GetCount() + 1;
}

ndInt32 ndBrainThreadPool::GetMaxThreads()
{
	#ifdef D_USE_THREAD_EMULATION
		return D_MAX_THREADS_COUNT;
	#else
		return ndClamp(ndInt32(std::thread::hardware_concurrency() + 1) / 2, 1, D_MAX_THREADS_COUNT);
	#endif
}

void ndBrainThreadPool::SetThreadCount(ndInt32 count)
{
#ifdef D_USE_THREAD_EMULATION
	m_count = ndClamp(count, 1, D_MAX_THREADS_COUNT) - 1;
#else
	ndInt32 maxThread = GetMaxThreads();
	count = ndClamp(count, 1, maxThread) - 1;
	if (count != m_workers.GetCount())
	{
		ndAssert(0);
		//if (m_workers)
		//{
		//	m_count = 0;
		//	delete[] m_workers;
		//	m_workers = nullptr;
		//}
		//if (count)
		//{
		//	m_count = count;
		//	m_workers = new ndWorker[size_t(count)];
		//	for (ndInt32 i = 0; i < count; ++i)
		//	{
		//		char name[256];
		//		m_workers[i].m_owner = this;
		//		m_workers[i].m_threadIndex = i;
		//		sprintf(name, "%s_%d", m_baseName, i + 1);
		//		m_workers[i].SetName(name);
		//	}
		//}
	}
#endif

}
