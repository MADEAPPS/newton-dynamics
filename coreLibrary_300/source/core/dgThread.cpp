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

#include "dgStdafx.h"
#include "dgThread.h"



dgThread::dgThread ()
	:m_id(0)
	,m_terminate(0)
	,m_threadRunning(0)
{
	m_name[0] = 0;
}

dgThread::dgThread (const char* const name, dgInt32 id)
	:m_id(id)
	,m_terminate(0)
	,m_threadRunning(0)
{
	strncpy (m_name, name, sizeof (m_name) - 1);
}


void dgThread::Init (const char* const name, dgInt32 id)
{
	m_id = id;
	strncpy (m_name, name, sizeof (m_name) - 1);
	Init();
}


bool dgThread::StillBusy() const
{
	return m_threadRunning ? true : false;
}



#ifdef DG_USE_THREAD_EMULATION

dgThread::dgSemaphore::dgSemaphore ()
{
	m_sem = NULL;
}

dgThread::dgSemaphore::~dgSemaphore ()
{
}

void dgThread::dgSemaphore::Release ()
{
}

dgThread::dgCriticalSection::dgCriticalSection()
{
}

dgThread::dgCriticalSection::~dgCriticalSection()
{
}


dgThread::~dgThread ()
{
}

void dgThread::Init ()
{
}

void dgThread::Close ()
{
}

void dgThread::SuspendExecution (dgSemaphore& mutex)
{
}


void dgThread::SuspendExecution (dgInt32 count, dgSemaphore* const semArray)
{
}


int dgThread::GetPriority() const
{
	return 0;
}

void dgThread::SetPriority(int priority)
{
}


void* dgThread::dgThreadSystemCallback(void* threadData)
{
	return 0;
}

#else  


#if defined (_MACOSX_VER) || defined (IOS) || defined (__APPLE__)
	#define DG_SEMAPHORE_NAME "/semaphore"
#endif

dgThread::dgSemaphore::dgSemaphore ()
{
	#if defined (_MACOSX_VER) || defined (IOS) || defined (__APPLE__)
		char temp[256];
		static int semID = 1;
		m_nameId = semID ++;
		sprintf (temp, "%s%d", DG_SEMAPHORE_NAME, m_nameId);
        sem_unlink(temp);
		m_sem = sem_open(temp, O_CREAT, S_IRUSR | S_IWUSR, 0);
	#else
		sem_init (&m_sem, 0, 0);
	#endif
}

dgThread::dgSemaphore::~dgSemaphore ()
{
	#if defined (_MACOSX_VER) || defined (IOS) || defined (__APPLE__)
		char temp[256];
		sprintf (temp, "%s%d", DG_SEMAPHORE_NAME, m_nameId);
		sem_close(m_sem);
		sem_unlink(temp) ;
	#else 
		sem_destroy (&m_sem);
	#endif
}

void dgThread::dgSemaphore::Release ()
{
	#if defined (_MACOSX_VER) || defined (IOS) || defined (__APPLE__)
		sem_post (m_sem);
	#else 
		sem_post (&m_sem);
	#endif
}

void dgThread::dgSemaphore::Wait()
{
	#if defined (_MACOSX_VER) || defined (IOS) || defined (__APPLE__)
		sem_wait (m_sem);
	#else 
		sem_wait (&m_sem);
	#endif
}

dgThread::dgCriticalSection::dgCriticalSection()
	#ifdef DG_USE_MUTEX_CRITICAL_SECTION
		:m_mutex (PTHREAD_MUTEX_INITIALIZER)
	#else 
		:m_mutex(0)
	#endif
{
}

dgThread::dgCriticalSection::~dgCriticalSection()
{
	#ifdef DG_USE_MUTEX_CRITICAL_SECTION
		pthread_mutex_destroy(&m_mutex);
	#endif
};



dgThread::~dgThread ()
{
}

void dgThread::Init ()
{
	pthread_create (&m_handle, NULL, dgThreadSystemCallback, this);
}

void dgThread::Close ()
{
	pthread_join (m_handle, NULL);
}


dgInt32 dgThread::GetPriority() const
{
	dgInt32 policy;
	sched_param param;
	pthread_getschedparam (m_handle, &policy, &param);
	return param.sched_priority;
}

void dgThread::SetPriority(int priority)
{
	dgInt32 policy;
	sched_param param;

	pthread_getschedparam (m_handle, &policy, &param);

	param.sched_priority = priority;
	pthread_setschedparam (m_handle, policy, &param);
}


void dgThread::SuspendExecution (dgSemaphore& mutex)
{
	mutex.Wait();
}


void dgThread::SuspendExecution (dgInt32 count, dgSemaphore* const semArray)
{
	for (dgInt32 i = 0; i < count; i ++) {
		SuspendExecution (semArray[i]);
	}
}


void* dgThread::dgThreadSystemCallback(void* threadData)
{
	dgFloatExceptions exception;
	dgSetPrecisionDouble precision;

	dgThread* const me = (dgThread*) threadData;
	dgInterlockedExchange(&me->m_threadRunning, 1);
	me->Execute(me->m_id);
	dgInterlockedExchange(&me->m_threadRunning, 0);
	dgThreadYield();
	return 0;
}


#endif



