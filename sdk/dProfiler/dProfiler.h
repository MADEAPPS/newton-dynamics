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

#define D_PROFILER

struct dProfilerSourceLocation
{
	const char* name;
	const char* function;
	const char* file;
	long long line;
	long long color;
};


#ifdef D_PROFILER

D_PROFILER_API long long dProfilerStartTrace__(const dProfilerSourceLocation* const sourceLocation);
D_PROFILER_API void dProfilerEndTrace__(long long id);
D_PROFILER_API void dProfilerSetTrackName__(const char* const trackName);


class dgProfile
{
	public:
	dgProfile(const dProfilerSourceLocation* const location, bool is_active = true)
		:m_thread(0)
		,m_active (is_active)
	{
		if (m_active) {
			m_thread = dProfilerStartTrace__(location);
		}
	}

	~dgProfile()
	{
		if (m_active) {
			dProfilerEndTrace__(m_thread);
		}
	}

	private:
	long long m_thread;
	const bool m_active;
};

#define dProfilerZoneScoped(name)					\
static const dProfilerSourceLocation __dprofiler_source_location { NULL, __FUNCTION__,  __FILE__, (long long)__LINE__, 0 }; \
dgProfile ___dgprofile_scoped_zone( &__dprofiler_source_location );

#define dProfilerSetTrackName(trackName) dProfilerSetTrackName__(trackName) 

#else


#define dProfilerZoneScoped(name)
#define dProfilerSetTrackName(trackName)
#endif

#endif