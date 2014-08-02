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

#ifndef __DG_THREAD_API_H__
#define __DG_THREAD_API_H__

// by default newton is run on a separate thread, optionally concurrent with the calling thread, it also uses a thread job pool for multi core systems.
// define DG_USE_THREAD_EMULATION on the command line for platform that do not support hardware multi threading or if multi threading is not stable 
//#define DG_USE_THREAD_EMULATION

//
//#define DG_USE_MUTEX_CRITICAL_SECTION

class dgThread
{
public:
	class dgCriticalSection
	{
		public:
		dgCriticalSection();
		~dgCriticalSection();
		void Lock();
		void Unlock();

		private:
		#if (defined (DG_USE_THREAD_EMULATION) || !defined (DG_USE_MUTEX_CRITICAL_SECTION))
			dgInt32 m_mutex;
		#else 
			pthread_mutex_t  m_mutex;
		#endif
	};

	class dgSemaphore
	{
		public:
		dgSemaphore ();
		~dgSemaphore ();
		void Wait();
		void Release();

		private:
		#ifdef DG_USE_THREAD_EMULATION
			dgInt32 m_sem;
		#else 
			#if defined (_MACOSX_VER) || defined (IOS) || defined (__APPLE__)
				sem_t* m_sem;
				dgInt32 m_nameId;
			#else
				sem_t m_sem;
			#endif
		#endif
		friend class dgThread;
	};

	dgThread ();
	dgThread (const char* const name, dgInt32 id);
	virtual ~dgThread ();

	virtual void Execute (dgInt32 threadId) = 0;
	
	bool StillBusy() const;
	void SuspendExecution (dgSemaphore& mutex);
	void SuspendExecution (dgInt32 count, dgSemaphore* const mutexes);

	dgInt32 GetPriority() const;
	void SetPriority(int priority);

	protected:
	void Init ();
	void Init (const char* const name, dgInt32 id);
	void Close ();

	static void* dgThreadSystemCallback(void* threadData);

	pthread_t m_handle;
	dgInt32 m_id;
	dgInt32 m_terminate;
	dgInt32 m_threadRunning;
	
	char m_name[32];
};


DG_INLINE void dgThread::dgCriticalSection::Lock()
{
	#ifndef DG_USE_THREAD_EMULATION 
		#ifdef DG_USE_MUTEX_CRITICAL_SECTION
			pthread_mutex_lock(&m_mutex);
		#else 
			while (dgInterlockedExchange(&m_mutex, 1)) {
				dgThreadYield();
			}
		
		#endif
	#endif
}

DG_INLINE void dgThread::dgCriticalSection::Unlock()
{
	#ifndef DG_USE_THREAD_EMULATION 
		#ifdef DG_USE_MUTEX_CRITICAL_SECTION
			pthread_mutex_unlock(&m_mutex);
		#else 
			dgInterlockedExchange(&m_mutex, 0);
		#endif
	#endif
}


#endif
