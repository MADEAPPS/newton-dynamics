
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
	class dTrackRecord
	{
		public:
		long long m_startTime;
		long long m_endTime;
		dCRCTYPE m_nameCRC;
		int m_size;
	};

	class dTrackerThread;
	class dTimeTrackerEntry
	{
		public:
		TIMETRACKER_API dTimeTrackerEntry(dCRCTYPE nameCRC);
		TIMETRACKER_API ~dTimeTrackerEntry();

		private:
		dTrackerThread* m_thread;
		int m_index;
	};

	class dTrackerThread
	{
		public:
		dTrackerThread (const char* const name);
		~dTrackerThread ();

		void Realloc();

		private:
		dList<dTimeTrackerEntry*> m_stack;
		const char* m_name;
		dTrackRecord* m_buffer;
		int m_index;
		int m_size;
		DWORD m_threadId;
		DWORD m_processId;
		friend class dTimeTracker;
	};
	
	TIMETRACKER_API static dTimeTracker* GetInstance();
	TIMETRACKER_API void StartSection (const char* const name, int frames);
	TIMETRACKER_API void EndSection ();

	TIMETRACKER_API void CreateTrack (const char* const name);
	TIMETRACKER_API dCRCTYPE RegisterName (const char* const name);

	private:
	class dLabels
	{
		public:
		char m_name[128];
	};
	dTimeTracker ();
	~dTimeTracker ();
	long long GetTimeInMicrosenconds();
	void WriteTrack(const dTrackerThread* const track, const dTrackRecord& record);

	dList<dTrackerThread> m_tracks;
	dTree<dLabels, dCRCTYPE> m_dictionary;
	LARGE_INTEGER m_frequency;
	LARGE_INTEGER m_baseCount;
	FILE* m_file;
	CRITICAL_SECTION m_criticalSection; 
	int m_frames;
	bool m_firstRecord;
};


#define dTimeTrackerStartSection(fileName,frames)									\
	dTimeTracker::GetInstance()->StartSection (name, frame);


#define dTimeTrackerCreateTrack(name)												\
	dTimeTracker::GetInstance()->CreateTrack (name);

#define dTimeTrackerTrackTime(name)													\
	static dCRCTYPE __crcName__ = dTimeTracker::GetInstance()->RegisterName (name);	\
	dTimeTracker::dTimeTrackerEntry ___trackerEntry___(__crcName__);	

#define dTimeTrackerSync()															\
{                                                                                   \
		dTimeTrackerTrackTime("timeTracker")										\
		dTimeTracker::GetInstance()->SavaData ();									\
}


#else 

#define dTimeTrackerCreateTrack(name)							
#define dTimeTrackerTrackTime(name)													

#endif

#endif