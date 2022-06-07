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

#include "dTracyProfiler.h"

//#if 0
#if !defined (WIN32) || (_MSC_VER >= 1900)


	#include "Tracy.hpp"
	#include "common\TracySystem.hpp"
	#include "client\TracyProfiler.hpp"

	using namespace tracy;

	static bool profileOn = false;

	void dProfilerEnableProlingLow()
	{
		profileOn = !profileOn;
	}
	
	void dProfilerStartTraceLow(const dProfilerSourceLocation* const srcloc)
	{
		if (profileOn) 
		{
			TracyQueuePrepare(QueueType::ZoneBegin);
			MemWrite(&item->zoneBegin.time, Profiler::GetTime());
			MemWrite(&item->zoneBegin.srcloc, (uint64_t)srcloc);
			TracyQueueCommit(zoneBeginThread);
		}
	}

	void dProfilerEndTraceLow()
	{
		if (profileOn) 
		{
			TracyQueuePrepare(QueueType::ZoneEnd);
			MemWrite(&item->zoneEnd.time, Profiler::GetTime());
			TracyQueueCommit(zoneEndThread);
		}
	}

	void dProfilerSetTrackNameLow(const char* const trackName)
	{
		tracy::SetThreadName(trackName);
	}

#else

	void dProfilerEnableProlingLow(int mode)
	{
		mode = 0;
	}

	long long dProfilerStartTraceLow(const dProfilerSourceLocation* const)
	{
		return 0;
	}

	void dProfilerEndTraceLow(int long long)
	{
	}

	void dProfilerSetTrackNameLow(const char* const)
	{
	}

#endif
