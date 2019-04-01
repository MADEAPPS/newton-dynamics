/* Copyright (c) <2018-2018> <Newton Game Dynamics>
*
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
*
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "dProfiler.h"

#pragma warning (disable: 4100) //unreferenced formal parameter

//#if !defined (WIN32) || (_MSC_VER >= 1900)
#if 1


	long long dProfilerStartTrace(const char* const srcloc)
	{
		return long long (0);
	}

	void dProfilerEndTrace(long long threadId)
	{
	}

	void dProfilerSetTrackName(const char* const trackName)
	{
	}

	void dProfilerDeleteTrack()
	{
	}

#else

	long long dProfilerStartTrace(const char* const fileName)
	{
		return 0;
	}

	void dProfilerEndTrace(int long long)
	{
	}

	void dProfilerSetTrackName(const char* const trackName)
	{
	}

	void dProfilerDeleteTrack()
	{
	}

#endif
