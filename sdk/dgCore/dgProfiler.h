/* Copyright (c) <2003-2016> <Julio Jerez, Newton Game Dynamics>
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

#ifndef __DG_PROFILER_H__
#define __DG_PROFILER_H__

#include "dgTypes.h"

#ifdef _DG_USE_PROFILER
/*
class dgProfile
{
	public:
	dgProfile(const char* const functionName)
		:m_entry(dProfilerStartTrace(functionName))
	{
	}

	~dgProfile()
	{
		dProfilerEndTrace(m_entry);
	}

	private:
	dgInt64 m_entry;
};
*/

#define DG_SET_TRACK_NAME(trackName) dProfilerSetTrackName(trackName)
//#define DG_TRACKTIME(name) dgProfile _profile##name(name);
//#define DG_TRACKTIME_NAMED(name) dgProfile _profile(name);

#define DG_TRACKTIME(name) dProfilerZoneScoped(name)
#define DG_TRACKTIME_NAMED(name) dProfilerZoneScoped(name)

#else

#define DG_TRACKTIME(name);
#define DG_TRACKTIME_NAMED(name);
#define DG_SET_TRACK_NAME(trackName);

#endif

#endif
