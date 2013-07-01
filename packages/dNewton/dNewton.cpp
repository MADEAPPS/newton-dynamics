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

#include "dStdAfxNewton.h"
#include "dNewton.h"
#include "dNewtonBody.h"
#include "dNewtonCollision.h"

dNewton::ScopeLock::ScopeLock (unsigned* const lock)
	:m_atomicLock(lock)
{
	while (NewtonAtomicSwap((int*)m_atomicLock, 1)) {
		NewtonYield();
	}
}

dNewton::ScopeLock::~ScopeLock()
{
	NewtonAtomicSwap((int*)m_atomicLock, 0);	
}


dNewton::dNewton()
	:m_maxUpdatePerIterations(2)
{
	// create a newton world
	m_world = NewtonCreate();

	// for two way communication between low and high lever, link the world with this class for 
	NewtonWorldSetUserData(m_world, this);

	// set the simplified solver mode (faster but less accurate)
	NewtonSetSolverModel (m_world, 1);

	// by default runs on four micro threads
	NewtonSetThreadsCount(m_world, 4);

	// set the collision copy constructor callback
	NewtonWorldSetCollisionConstructorDestuctorCallback (m_world, OnCollisionCopyConstruct, OnCollisionDestructorCallback);

	// set th timer
	ResetTimer();
}

dNewton::~dNewton()
{
	NewtonWaitForUpdateToFinish (m_world);
	NewtonDestroy (m_world);
}

void dNewton::SetMaxUpdatesPerIterations (int update)
{
	m_maxUpdatePerIterations = update;
}

void dNewton::SetAllocationDrivers (CNewtonAllocMemory alloc, CNewtonFreeMemory free)
{
	NewtonSetMemorySystem (alloc, free);
}

void dNewton::OnCollisionDestructorCallback (const NewtonWorld* const newtonWorld, const NewtonCollision* const collision)
{
	dNewtonCollision* const srcColl = (dNewtonCollision*) NewtonCollisionGetUserData(collision);
	if (srcColl) {
		delete srcColl;
	}
}

void dNewton::OnCollisionCopyConstruct (const NewtonWorld* const world, NewtonCollision* const collision, const NewtonCollision* const sourceCollision)
{
	dNewtonCollision* const srcColl = (dNewtonCollision*) NewtonCollisionGetUserData(sourceCollision);
	dAssert (srcColl);
	srcColl->Clone(collision);
}


NewtonWorld* dNewton::GetNewton () const
{
	return m_world;
}

void* dNewton::operator new (size_t size)
{
	return NewtonAlloc(int (size));
}

void dNewton::operator delete (void* ptr)
{
	NewtonFree(ptr);
}

dNewtonBody* dNewton::GetFirstBody() const
{
	NewtonBody* const newtonBody = NewtonWorldGetFirstBody(m_world);
	return newtonBody ? (dNewtonBody*) NewtonBodyGetUserData(newtonBody) : NULL;
}

dNewtonBody* dNewton::GetNextBody(const dNewtonBody* const body) const
{
	NewtonBody* const newtonBody = NewtonWorldGetNextBody(m_world, body->GetNewtonBody());
	return newtonBody ? (dNewtonBody*) NewtonBodyGetUserData(newtonBody) : NULL;
}


void dNewton::ResetTimer()
{
	#ifdef _MSC_VER
		LARGE_INTEGER baseCount;
		LARGE_INTEGER frequency;
		QueryPerformanceFrequency(&frequency);
		QueryPerformanceCounter (&baseCount);
		m_baseCount = dLong (baseCount.QuadPart);
		m_frequency = dLong (frequency.QuadPart);
	#endif
/*
	#if (defined (_POSIX_VER) || defined (_POSIX_VER_64))
		timespec ts;
		clock_gettime(CLOCK_REALTIME, &ts); // Works on Linux
		//baseCount = ts.tv_nsec / 1000;
		baseCount = unsigned64 (ts.tv_sec) * 1000000 + ts.tv_nsec / 1000;
	#endif

	#ifdef _MACOSX_VER
		timeval tp;
		gettimeofday(&tp, NULL);
		unsigned64 microsecunds =  unsigned64 (tp.tv_sec) * 1000000 + tp.tv_usec;
		baseCount = microsecunds;
	#endif
*/

	m_microseconds = GetTimeInMicrosenconds();
}


dLong dNewton::GetTimeInMicrosenconds() const 
{
	#ifdef _MSC_VER
		LARGE_INTEGER count;
		QueryPerformanceCounter (&count);
		count.QuadPart -= m_baseCount;
		dLong ticks = count.QuadPart * LONGLONG (1000000) / m_frequency;
		return ticks;
	#endif
/*
	#if (defined (_POSIX_VER) || defined (_POSIX_VER_64))
		timespec ts;
		clock_gettime(CLOCK_REALTIME, &ts); // Works on Linux
		//return unsigned64 (ts.tv_nsec / 1000) - baseCount;
		
		return unsigned64 (ts.tv_sec) * 1000000 + ts.tv_nsec / 1000 - baseCount;
	#endif


	#ifdef _MACOSX_VER
		timeval tp;
		gettimeofday(&tp, NULL);
		unsigned64 microsecunds =  unsigned64 (tp.tv_sec) * 1000000 + tp.tv_usec;
		return microsecunds - baseCount;
	#endif
*/
}


dFloat dNewton::GetInteplationParam(dFloat timestepInSecunds) const
{
	dLong timeStep = GetTimeInMicrosenconds () - m_microseconds;
	dFloat param = (dFloat (timeStep) * 1.0e-6f) / timestepInSecunds;
	return dClamp (param, 0.0f, 1.0f);
}


void dNewton::Update (dFloat timestepInSecunds)
{
	dLong timestepMicrosecunds = dLong (double (timestepInSecunds) * 1000000.0f);
	dLong currentTime = GetTimeInMicrosenconds ();

	dLong nextTime = currentTime - m_microseconds;
	int loops = 0;
	while ((nextTime >= timestepMicrosecunds) && (loops < m_maxUpdatePerIterations)) {
		loops ++;
		NewtonUpdate (m_world, timestepInSecunds);
		nextTime -= timestepMicrosecunds;
		m_microseconds += timestepMicrosecunds;
	}
	if (loops >= m_maxUpdatePerIterations) {
		ResetTimer();
	}
	
}

void dNewton::UpdateAsync (dFloat timestepInSecunds)
{
	dLong timestepMicrosecunds = dLong (double (timestepInSecunds) * 1000000.0f);
	dLong currentTime = GetTimeInMicrosenconds ();

	dLong nextTime = currentTime - m_microseconds;
	int loops = 0;
	while ((nextTime >= timestepMicrosecunds) && (loops < m_maxUpdatePerIterations)) {
		loops ++;
		NewtonUpdateAsync (m_world, timestepInSecunds);
		nextTime -= timestepMicrosecunds;
		m_microseconds += timestepMicrosecunds;
	}
	if (loops >= m_maxUpdatePerIterations) {
		ResetTimer();
	}
}
