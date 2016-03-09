
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
	class dTimeTrackerEntry;
/*
	class dTimeTrackerEntry
	{
		public:
		char* m_name;
		long long m_startTime;
		long long m_endTime;
		dTimeTrackerEntry* m_parent
	};
*/
	public:
	class dTrackerThread
	{
		public:
		dTrackerThread (const char* const name);

		private:
		const char* m_name;
		dTimeTrackerEntry* m_root;
		dTimeTrackerEntry* m_current;
	};

	
	static dTimeTracker* GetIntance();
//	dTimeTrackerEntry* CreateTrack();

	private:
	dTimeTracker ();
	dList<dTrackerThread*> m_tracks;
};


#define dTimeTrackerCreateThread(name)							\
	static dTimeTracker::dTrackerThread threadHeader (name);
#endif

#endif