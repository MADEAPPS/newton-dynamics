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

#include "toolbox_stdafx.h"
#include "OpenGlUtil.h"
#include "dHighResolutionTimer.h"

#define LOCKED_FPS		dFloat (100.0f)
#define LOCKED_TIMESTEP int ((1000000.0f/LOCKED_FPS))

const dFloat TICKS2SEC = 1.0e-6f;


static unsigned64 m_prevTime = 0;


#ifdef _MSC_VER
	static LARGE_INTEGER frequency;
	static LARGE_INTEGER baseCount;
#else 
	static unsigned64 baseCount;
#endif



void dResetTimer()
{
	#ifdef _MSC_VER
		QueryPerformanceFrequency(&frequency);
		QueryPerformanceCounter (&baseCount);

	#endif

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
}


unsigned64 dGetTimeInMicrosenconds()
{
#ifdef _MSC_VER
		LARGE_INTEGER count;
		QueryPerformanceCounter (&count);
		count.QuadPart -= baseCount.QuadPart;
		unsigned64 ticks = unsigned64 (count.QuadPart * LONGLONG (1000000) / frequency.QuadPart);
		return ticks;

	#endif

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
}


dFloat dGetElapsedSeconds()
{
	dFloat timeStep;
	unsigned64 miliseconds;

	miliseconds = dGetTimeInMicrosenconds();

	// optimal keep the fps below 120 fps
	timeStep = dFloat (miliseconds - m_prevTime) * TICKS2SEC;
	m_prevTime = miliseconds;

	return timeStep;
} 

