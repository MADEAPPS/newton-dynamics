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

#include "ndCoreStdafx.h"
#include "ndTypes.h"
#include "ndUtils.h"
#include "ndProfiler.h"
#include "ndThreadBackgroundWorker.h"

ndThreadBackgroundWorker::ndThreadBackgroundWorker()
	:ndThread()
	,ndList<ndBackgroundJob*, ndContainersFreeListAlloc<ndBackgroundJob*>>()
	,m_lock()
	,m_queueSemaphore()
	,m_teminate(false)
{
	SetName(__FUNCTION__);
	Signal();
}

ndThreadBackgroundWorker::~ndThreadBackgroundWorker()
{
	m_queueSemaphore.Terminate();
	Finish();
}

void ndThreadBackgroundWorker::Terminate()
{
	m_teminate = true;
}

void ndThreadBackgroundWorker::SendJob(ndBackgroundJob* const job)
{
	{
		ndScopeSpinLock lock(m_lock);
		job->m_jobState = ndBackgroundJob::m_jobInProccess;
		Append(job);
	}
	m_queueSemaphore.Signal();
}

void ndThreadBackgroundWorker::ThreadFunction()
{
	while (!m_queueSemaphore.Wait() && !m_teminate)
	{
		ndBackgroundJob* job;
		{
			ndScopeSpinLock lock(m_lock);
			ndNode* const node = GetFirst();
			job = node->GetInfo();
			Remove(node);
		}
		job->Execute();
		job->m_jobState = ndBackgroundJob::m_jobCompleted;
	}
}