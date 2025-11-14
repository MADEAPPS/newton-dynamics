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

#include "ndCoreStdafx.h"
#include "ndCollisionStdafx.h"

#include <string.h>
#include "vhacdConvexHullUtils.h"

namespace nd
{
	namespace VHACD
	{
		//#define Absolute(a)  ((a) >= 0.0 ? (a) : -(a))
		//double Determinant2x2(const double matrix[2][2], double* const error)
		//{
		//	double a00xa11 = matrix[0][0] * matrix[1][1];
		//	double a01xa10 = matrix[0][1] * matrix[1][0];
		//	*error = Absolute(a00xa11) + Absolute(a01xa10);
		//	return a00xa11 - a01xa10;
		//}
		//
		//double Determinant3x3(const double matrix[3][3], double* const error)
		//{
		//	double sign = double(-1.0f);
		//	double det = double(0.0f);
		//	double accError = double(0.0f);
		//	for (int i = 0; i < 3; ++i)
		//	{
		//		double cofactor[2][2];
		//		for (int j = 0; j < 2; ++j)
		//		{
		//			int k0 = 0;
		//			for (int k = 0; k < 3; ++k)
		//			{
		//				if (k != i)
		//				{
		//					cofactor[j][k0] = matrix[j][k];
		//					k0++;
		//				}
		//			}
		//		}
		//
		//		double parcialError;
		//		double minorDet = Determinant2x2(cofactor, &parcialError);
		//		accError += parcialError * Absolute(matrix[2][i]);
		//		det += sign * minorDet * matrix[2][i];
		//		sign *= double(-1.0f);
		//	}
		//
		//	*error = accError;
		//	return det;
		//}

		Semaphore::Semaphore()
			:m_count(0)
			,m_mutex()
			,m_condition()
			,m_terminate(false)
		{
		}

		Semaphore::~Semaphore()
		{
		}

		bool Semaphore::Wait()
		{
			std::unique_lock<std::mutex> lock(m_mutex);
			while (m_count == 0)
			{
				m_condition.wait(lock);
			}

			m_count--;
			return m_terminate.load();
		}

		void Semaphore::Signal()
		{
			std::unique_lock<std::mutex> lock(m_mutex);
			m_count++;
			m_condition.notify_one();
		}

		void Semaphore::Terminate()
		{
			std::unique_lock<std::mutex> lock(m_mutex);
			m_count++;
			m_terminate.store(true);
			m_condition.notify_one();
		}

		Queue::Queue()
			:ndList<Job*>()
			,m_mutex()
			,m_jobs(0)
		{
			for (int i = 0; i < VHACD_WORKERS_THREADS; ++i)
			{
				m_threads[i].m_threadID = i;
				m_threads[i].m_queue = this;
			}
		}

		Queue::~Queue()
		{
			for (int i = 0; i < VHACD_WORKERS_THREADS; ++i)
			{
				m_threads[i].Terminate();
				m_threads[i].join();
			}
		}

		void Queue::PushTask(Job* const job)
		{
			std::unique_lock<std::mutex> lock(m_mutex);
			Append(job);
			m_jobs.fetch_add(1);
			for (int i = 0; i < VHACD_WORKERS_THREADS; ++i)
			{
				m_threads[i].Signal();
			}
		}

		Job* Queue::PopTask()
		{
			std::unique_lock<std::mutex> lock(m_mutex);
			Job* job = nullptr;
			if (GetCount())
			{
				ndNode* const node = GetFirst();
				job = node->GetInfo();
				Remove(node);
			}
			return job;
		}

		void Queue::Sync()
		{
			while (m_jobs)
			{
				std::this_thread::yield();
			}
		}

		#ifdef _MSC_VER
		#pragma warning( push )
		#pragma warning( disable : 4355)
		#endif

		Thread::Thread()
			:Semaphore()
			,std::thread(&Thread::ThreadFunctionCallback, this)
			,m_queue(nullptr)
		{
		}

		#ifdef _MSC_VER
		#pragma warning( pop )
		#endif

		Thread::~Thread()
		{
		}

		void Thread::ThreadFunctionCallback()
		{
			while (!Wait())
			{
				Job* const job = m_queue->PopTask();
				if (job)
				{
					job->Execute(m_threadID);
					m_queue->m_jobs.fetch_add(-1);
				}
			}
		}
	}
}
