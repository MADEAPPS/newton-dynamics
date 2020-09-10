/* Copyright (c) <2003-2019> <Julio Jerez, Newton Game Dynamics>
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

#ifndef _D_SYNC_MUTEX_H_
#define _D_SYNC_MUTEX_H_

#include "dCoreStdafx.h"

/// Generic counting mutex for synchronization of thread jobs
class dSyncMutex
{
	public:
	/// Create and initialize counter to zero
	D_CORE_API dSyncMutex();

	/// Destroy mutex
	D_CORE_API ~dSyncMutex();

	/// Synchronize with another worker threads.
	/// \brief When internal variable m_counter larger than zero, this function blocks
	/// the calling thread until another thread call member function Release.
	/// \brief When counter is zero, this function return immediately. 
	D_CORE_API void Sync();

	/// Increment internal variable m_count by one.
	D_CORE_API void Tick();

	/// Decrement internal variable m_count by one and signal the thread to wakeup.
	D_CORE_API void Release();

#ifndef D_USE_THREAD_EMULATION	
	private:
	std::mutex m_mutex;
	std::condition_variable m_condition;
	int m_count;
#endif
};

#endif
