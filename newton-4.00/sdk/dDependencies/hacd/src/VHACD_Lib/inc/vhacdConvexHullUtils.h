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

#ifndef __ND_CONVEXHULL_3D_UTILS__
#define __ND_CONVEXHULL_3D_UTILS__

#include "vhacdVector.h"

namespace nd
{
	namespace VHACD 
	{
		#define VHACD_WORKERS_THREADS	8

		inline int dBitReversal(int v, int base)
		{
			int x = 0;
			int power = ndExp2(base) - 1;
			do
			{
				x += (v & 1) << power;
				v >>= 1;
				power--;
			} while (v);
			return x;
		}

		class Job
		{
			public:
			Job()
			{
			}

			Job(const Job&)
			{
			}

			Job& operator=(const Job& other) = default;

			virtual ~Job()
			{
			}

			virtual void Execute(int threadId) = 0;
		};

		class Semaphore
		{
			public:
			Semaphore();
			~Semaphore();
			bool Wait();
			void Signal();
			void Terminate();

			int m_count;
			std::mutex m_mutex;
			std::condition_variable m_condition;
			std::atomic<bool> m_terminate;
		};

		class Queue;
		class Thread : public Semaphore, public std::thread
		{
			public:
			Thread();
			~Thread();
			void ThreadFunction();

			int m_threadID;
			Queue* m_queue;
		};

		class Queue : public ndList<Job*>
		{
			public:
			Queue();
			~Queue();

			void Sync();
			Job* PopTask();
			void PushTask(Job* const job);

			std::mutex m_mutex;
			ndAtomic<ndInt32> m_jobs;
			Thread m_threads[VHACD_WORKERS_THREADS];
		};
	}
}
#endif
