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

#if !defined (WIN32) || (_MSC_VER >= 1900)

	#include "Tracy.hpp"


	int dProfilerStartTrace(const char* const fileName)
	{
		return 0;
	}

	void dProfilerEndTrace(int id)
	{
	}

	void dProfilerSetTrackName(const char* const trackName)
	{
	}

	void dProfilerDeleteTrack()
	{
	}

#else

	int dProfilerStartTrace(const char* const fileName)
	{
		return 0;
	}

	void dProfilerEndTrace(int id)
	{
	}

	void dProfilerSetTrackName(const char* const trackName)
	{
	}

	void dProfilerDeleteTrack()
	{
	}

#endif
