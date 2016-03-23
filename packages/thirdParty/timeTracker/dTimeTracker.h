
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

#if defined (D_TIME_TRACKER) && defined(_MSC_VER)

#include "dContainersAlloc.h"
#include "dList.h"
#include "dTree.h"
#include "dCRC.h"


#ifdef TIMETRACKER_EXPORTS
	#ifdef _WIN32
		#define TIMETRACKER_API __declspec (dllexport)
	#else
		#define TIMETRACKER_API __attribute__ ((visibility("default")))
	#endif
#else
	#ifdef _WIN32
		#define TIMETRACKER_API __declspec (dllimport)
	#else
		#define TIMETRACKER_API
	#endif
#endif



class dTimeTracker
{
	public:
	TIMETRACKER_API static dTimeTracker* GetInstance();

	TIMETRACKER_API void Update ();
	TIMETRACKER_API void StartSection (int numberOfFrames);
	TIMETRACKER_API dCRCTYPE RegisterName (const char* const name);
	TIMETRACKER_API void RegisterThreadName (const char* const name);

	class dTrackEntry
	{
		public:
		dTrackEntry(){}
		TIMETRACKER_API dTrackEntry(dCRCTYPE nameCRC);
		TIMETRACKER_API ~dTrackEntry();

		long long m_startTime;
		long long m_endTime;
		dCRCTYPE m_nameCRC;
		DWORD m_threadId;
	};

	private:
	class dLabels
	{
		public:
		char m_name[128];
	};

	dTimeTracker ();
	~dTimeTracker ();

	void EndSection ();
	void FlushQueue ();
	void QueueEvents ();
	long long GetTimeInNanosenconds();
	
	dTree<dLabels, dCRCTYPE> m_dictionary;
	LARGE_INTEGER m_frequency;
	LARGE_INTEGER m_baseCount;
	CRITICAL_SECTION m_criticalSection; 
	DWORD m_processId;
	FILE* m_file;
	int m_numberOfFrames;
	int m_firstRecord;
	long m_bufferIndex;
	long m_bufferSize;
	dTrackEntry* m_buffer;

	int m_outQueueBufferindex;
	int m_outQueueBufferSize;
	dTrackEntry* m_outQueueBuffer;
};


#define dTimeTrackerStartSection(frames)													\
	dTimeTracker::GetInstance()->StartSection (frames);

#define dTimeTrackerUpdate()																\
	dTimeTracker::GetInstance()->Update ();

#define dTimeTrackerSetThreadName(name)														\
	dTimeTracker::GetInstance()->RegisterThreadName (name);

#define dTimeTrackerEvent(name)																\
	static dCRCTYPE __crcEventName__ = dTimeTracker::GetInstance()->RegisterName (name);	\
	dTimeTracker::dTrackEntry ___trackerEntry___(__crcEventName__);	

#else 

#define dTimeTrackerUpdate()
#define dTimeTrackerEvent(name)
#define dTimeTrackerSetThreadName(name)
#define dTimeTrackerStartSection(frames)

#endif

#endif