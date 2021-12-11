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

#ifndef __ND_SEMAPHORE_H_
#define __ND_SEMAPHORE_H_

#include "ndCoreStdafx.h"

/// Generic counting semaphore for thread synchronization 
class ndSemaphore
{
	public:
	/// Create and initialize counter to zero
	D_CORE_API ndSemaphore();

	/// Destroy semaphore
	D_CORE_API ~ndSemaphore();

	/// Returns counter counter value
	D_CORE_API dInt32 GetCount() const;

	/// Synchronize with another threads.
	/// \return returns false if member function Terminate has not been called. 
	/// \brief When internal variable m_counter is zero, this function blocks
	/// the calling thread until another thread call Signal function incrementing
	/// m_count by one. 
	/// \brief when counter is hight that zero, this function return immediately 
	/// decrementing the m_count by one.
	D_CORE_API bool Wait();

	/// Notify a thread blocked by member function Wait to wake and test m_counter again.
	/// Increment internal variable m_count by one and signal the thread to wakeup.
	D_CORE_API void Signal();

	/// Notify a waiting thread on member function Wait that is time to exit the thread loop.
	D_CORE_API void Terminate();

#ifndef D_USE_THREAD_EMULATION
	private:
	mutable std::mutex m_mutex;
	std::condition_variable m_condition;
	dInt32 m_count;
	ndAtomic<bool> m_terminate;
#endif
};

#endif
