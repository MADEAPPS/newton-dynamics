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

#ifndef __ND_THREAD_H_
#define __ND_THREAD_H_

#include "ndCoreStdafx.h"
#include "ndSemaphore.h"
#include "ndClassAlloc.h"

class ndThreadName
{
	public:
	ndThreadName()
	{
		strcpy_s(m_name, sizeof(m_name), "newtonWorker");
	}
	char m_name[32];
};

class ndThreadInterface: public ndClassAlloc, public ndSemaphore
{
	public:
	D_CORE_API ndThreadInterface();
	D_CORE_API virtual ~ndThreadInterface();

	virtual void Signal() = 0;
	virtual void Finish() = 0;
	virtual void Release() = 0;
	virtual void ThreadFunction() = 0;
	virtual void ThreadFunctionCallback() = 0;
	virtual void SetName(const char* const name) = 0;

	ndThreadName m_name;
};

/// Base class for for all multi thread functionality.
class ndThread
	:public ndThreadInterface
#ifndef D_USE_THREAD_EMULATION
	,public ndAtomic<bool>
	,public std::condition_variable
	,public std::thread
#endif
{
	public:
	/// Empty default constructor
	/// after creation all threads go to a wait state
	D_CORE_API ndThread();

	/// Empty, does not terminate the thread loop. 
	/// The thread loop is only terminated after calling Finish.
	D_CORE_API virtual ~ndThread();

	/// Set thread name. 
	/// Useful for when debugging or profiler and application. 
	D_CORE_API virtual void SetName(const char* const name) override;

	/// Set the thread, to execute one call to and go back to a wait state  
	D_CORE_API virtual void Signal() override;

	/// Force the thread loop to terminate.
	/// This function must be call explicitly when the application
	/// wants to terminate the thread because the destructor does not do it. 
	D_CORE_API virtual void Finish() override;

	/// Thread function to execute in a perpetual loop until the thread is terminated.
	/// Each time the thread owner calls function Signal, the loop execute one call to 
	/// this function and upon return, the thread goes back to wait for another signal  
	/// or to exit the loop. 
	//virtual virtual void ThreadFunction() = 0;

	protected:
	D_CORE_API virtual void Release() override;
	D_CORE_API virtual void ThreadFunctionCallback() override;
};

#endif
