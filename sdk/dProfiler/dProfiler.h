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

#ifndef __D_PROFILER_H__
#define __D_PROFILER_H__


#ifdef D_PROFILER_EXPORTS
#define D_PROFILER_API __declspec(dllexport)
#else
#define D_PROFILER_API __declspec(dllimport)
#endif

struct dProfilerSourceLocation
{
	const char* name;
	const char* function;
	const char* file;
	long long line;
	long long color;
};

D_PROFILER_API long long dProfilerStartTrace(const dProfilerSourceLocation* const sourceLocation);
D_PROFILER_API void dProfilerEndTrace(long long id);
D_PROFILER_API void dProfilerSetTrackName(const char* const trackName);

class dgProfile
{
	public:
	dgProfile(const dProfilerSourceLocation* const location)
		:m_thread(dProfilerStartTrace(location))
	{
	}

	~dgProfile()
	{
		dProfilerEndTrace(m_thread);
	}

	private:
	long long m_thread;
};


#define dProfilerZoneScoped(name)					\
static const dProfilerSourceLocation __dprofiler_source_location { name, __FUNCTION__,  __FILE__, (long long)__LINE__, 0 }; \
dgProfile ___dgprofile_scoped_zone( &__dprofiler_source_location );

#endif