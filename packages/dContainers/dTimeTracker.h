
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
#include "dTree.h"
#include "dCRC.h"

#define D_TIME_TRACKER

#ifdef D_TIME_TRACKER
class dTimeTracker
{
	public:
	class dTrackerThread;
	class dTimeTrackerEntry
	{
		public:
		dTimeTrackerEntry(dCRCTYPE nameCRC);
		~dTimeTrackerEntry();

		private:
		dCRCTYPE m_nameCRC;
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
	
	static dTimeTracker* GetInstance();
	void CreateThread (const char* const name);
	dCRCTYPE RegisterName (const char* const name);

	long long GetTimeInMicrosenconds();

	private:
	dTimeTracker ();
	dList<dTrackerThread> m_tracks;
	dTree<const char*, dCRCTYPE> m_dictionary;

	LARGE_INTEGER m_frequency;
	LARGE_INTEGER m_baseCount;
};


#define dTimeTrackerCreateThread(name)							\
	dTimeTracker::GetInstance()->CreateThread (name);


#define dTimeTrackerTrackTime(name)													\
	static dCRCTYPE __crcName__ = dTimeTracker::GetInstance()->RegisterName (name);	\
	dTimeTracker::dTimeTrackerEntry ___trackerEntyr___(__crcName__);	

#endif

#endif