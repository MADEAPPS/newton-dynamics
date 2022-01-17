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

#ifndef __ND_THREAD_BACKGROUNG_WORKER_H_
#define __ND_THREAD_BACKGROUNG_WORKER_H_

#include "ndCoreStdafx.h"
#include "ndTypes.h"
#include "ndList.h"
#include "ndSemaphore.h"
#include "ndClassAlloc.h"
#include "ndThreadPool.h"

class ndThreadBackgroundWorker;

class ndBackgroundJob
{
	public:
	enum ndJobState
	{
		m_jobInProccess,
		m_jobCompleted,
	};

	ndBackgroundJob()
		:m_jobState(m_jobCompleted)
		,m_threaPool(nullptr)
	{
	}

	ndJobState JobState() const
	{
		return m_jobState.load();
	}

	virtual ~ndBackgroundJob()
	{
	}


	ndThreadBackgroundWorker* GetThreadPool() const
	{
		return m_threaPool;
	}

	protected:
	virtual void Execute()
	{
	}

	private:
	ndAtomic<ndJobState> m_jobState;
	ndThreadBackgroundWorker* m_threaPool;
	friend class ndThreadBackgroundWorker;
};

class ndThreadBackgroundWorker : public ndList<ndBackgroundJob*, ndContainersFreeListAlloc<ndBackgroundJob*>>, public ndThreadPool
{
	public:
	D_CORE_API ndThreadBackgroundWorker();
	D_CORE_API ~ndThreadBackgroundWorker();

	D_CORE_API void Terminate();
	D_CORE_API void SendJob(ndBackgroundJob* const job);
	
	private:
	virtual void ThreadFunction();

	ndSpinLock m_lock;
	ndAtomic<bool> m_inLoop;
	ndAtomic<bool> m_teminate;
	ndSemaphore m_queueSemaphore;
};

#endif
