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


int dNewton::m_totalMemoryUsed = 0;
bool dNewton::m_memorySystemInitialized = false;


void* dNewton::DefualtAlloc (int sizeInBytes)
{
	m_totalMemoryUsed += sizeInBytes;
	return new char[sizeInBytes];
}

// memory free use by the engine
void dNewton::DefaultFree (void* ptr, int sizeInBytes)
{
	m_totalMemoryUsed -= sizeInBytes;
	delete[] (char*)ptr;
}


void dNewton::SetAllocationDrivers (CNewtonAllocMemory alloc, CNewtonFreeMemory free)
{
	if (!m_memorySystemInitialized) {
		m_memorySystemInitialized = true;
		NewtonSetMemorySystem (alloc, free);
	}
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



dNewton::dNewton()
{
	if (!m_memorySystemInitialized) {
		SetAllocationDrivers (DefualtAlloc, DefaultFree);
	}

	// create a newtop world
	m_world = NewtonCreate();

	// for two way comunication between low and high lever, link the world with this class for 
	NewtonWorldSetUserData(m_world, this);

	// set the simplified solver mode (faster but less accurate)
	NewtonSetSolverModel (m_world, 1);

	// by default runs on fout micro threads
	NewtonSetThreadsCount(m_world, 4);

	// set th timer
	ResetTimer();
}

dNewton::~dNewton()
{
	NewtonWaitForUpdateToFinish (m_world);
	NewtonDestroy (m_world);
}


void dNewton::Update (dFloat timestepInSecunds)
{
	dLong timestepMicrosecunds = dLong (double (timestepInSecunds) * 1000000.0f);
	dLong currentTime = GetTimeInMicrosenconds ();

	dLong nextTime = currentTime - m_microsecunds;
	int loops = 0;
	while ((nextTime >= timestepMicrosecunds) && (loops < CNEWTON_MAX_PHYSICS_LOOPS)) {
		loops ++;
		NewtonUpdate (m_world, timestepInSecunds);
		nextTime -= timestepMicrosecunds;
		m_microsecunds += timestepMicrosecunds;
	}
	if (loops >= CNEWTON_MAX_PHYSICS_LOOPS) {
		ResetTimer();
	}

}

void dNewton::UpdateAsync (dFloat timestepInSecunds)
{
	dLong timestepMicrosecunds = dLong (double (timestepInSecunds) * 1000000.0f);
	dLong currentTime = GetTimeInMicrosenconds ();

	dLong nextTime = currentTime - m_microsecunds;
	int loops = 0;
	while ((nextTime >= timestepMicrosecunds) && (loops < CNEWTON_MAX_PHYSICS_LOOPS)) {
		loops ++;
		NewtonUpdateAsync (m_world, timestepInSecunds);
		nextTime -= timestepMicrosecunds;
		m_microsecunds += timestepMicrosecunds;
	}
	if (loops >= CNEWTON_MAX_PHYSICS_LOOPS) {
		ResetTimer();
	}
}
