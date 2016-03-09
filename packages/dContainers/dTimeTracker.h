
/* Copyright (c) <2009> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#ifndef __DTIME_TRACKER_H__
#define __DTIME_TRACKER_H__

#include "dContainersAlloc.h"
#include "dList.h"

#define D_TIME_TRACKER

#ifdef D_TIME_TRACKER
class dTimeTracker
{
	public:
	class dTrackerThread;
	class dTimeTrackerEntry
	{
		public:
		dTimeTrackerEntry(const char* const name);
		~dTimeTrackerEntry();

		private:
		const char* m_name;
		long long m_startTime;
		long long m_endTime;
		dTrackerThread* m_thread;
		dTimeTrackerEntry* m_parent;
	};

	class dTrackerThread
	{
		public:
		dTrackerThread (const char* const name);

		private:
		dList<dTimeTrackerEntry*> m_stack;
		const char* m_name;
		long long m_threadId;
		friend class dTimeTracker;
	};

	
	static dTimeTracker* GetIntance();
	void CreateThread (const char* const name);

	private:
	dTimeTracker ();
	dList<dTrackerThread> m_tracks;
};


#define dTimeTrackerCreateThread(name)							\
	dTimeTracker::GetIntance()->CreateThread (name);


#define dTimeTrackerTrackTime(name)							\
	dTimeTracker::dTimeTrackerEntry ___trackerEntyr___(name);	

#endif

#endif