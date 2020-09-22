/* Copyright (c) <2003-2019> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "ndSandboxStdafx.h"
#include "ndOpenGlUtil.h"
#include "ndHighResolutionTimer.h"

#define LOCKED_FPS		dFloat32 (100.0f)
#define LOCKED_TIMESTEP int ((1000000.0f/LOCKED_FPS))

const dFloat32 TICKS2SEC = 1.0e-6f;


static dUnsigned64 m_prevTime = 0;


#ifdef _WIN32
	static LARGE_INTEGER frequency;
	static LARGE_INTEGER baseCount;
#else 
	static dUnsigned64 baseCount;
#endif



void dResetTimer()
{
	#if defined(_WIN32)
		QueryPerformanceFrequency(&frequency);
		QueryPerformanceCounter (&baseCount);
	#elif defined(__linux__)
		timespec ts;
		clock_gettime(CLOCK_REALTIME, &ts); // Works on Linux
		//baseCount = ts.tv_nsec / 1000;
		baseCount = dUnsigned64 (ts.tv_sec) * 1000000 + ts.tv_nsec / 1000;
	#elif defined(_MACOSX_VER)
		timeval tp;
		gettimeofday(&tp, nullptr);
		dUnsigned64 microsecunds =  dUnsigned64 (tp.tv_sec) * 1000000 + tp.tv_usec;
		baseCount = microsecunds;
	#endif
}


dUnsigned64 dGetTimeInMicrosenconds()
{
	#if defined(_WIN32)
		LARGE_INTEGER count;
		QueryPerformanceCounter (&count);
		count.QuadPart -= baseCount.QuadPart;
		dUnsigned64 ticks = dUnsigned64 (count.QuadPart * LONGLONG (1000000) / frequency.QuadPart);
		return ticks;
	#elif defined(__linux__)
		timespec ts;
		clock_gettime(CLOCK_REALTIME, &ts); // Works on Linux
		//return dUnsigned64 (ts.tv_nsec / 1000) - baseCount;
		return dUnsigned64 (ts.tv_sec) * 1000000 + ts.tv_nsec / 1000 - baseCount;
	#elif defined(_MACOSX_VER)
		timeval tp;
		gettimeofday(&tp, nullptr);
		dUnsigned64 microsecunds =  dUnsigned64 (tp.tv_sec) * 1000000 + tp.tv_usec;
		return microsecunds - baseCount;
	#endif
}


dFloat32 dGetElapsedSeconds()
{
	dFloat32 timeStep;
	dUnsigned64 miliseconds;

	miliseconds = dGetTimeInMicrosenconds();

	// optimal keep the fps below 120 fps
	timeStep = dFloat32 (miliseconds - m_prevTime) * TICKS2SEC;
	m_prevTime = miliseconds;

	return timeStep;
} 

