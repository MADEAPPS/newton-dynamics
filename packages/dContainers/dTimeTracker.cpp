
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


#include "dContainersStdAfx.h"
#include "dTimeTracker.h"

#ifdef D_TIME_TRACKER

dTimeTracker* dTimeTracker::GetInstance()
{
	static dTimeTracker instance;
	return &instance;
}


dTimeTracker::dTrackerThread::dTrackerThread (const char* const name)
	:m_name (name)
//	,m_root (NULL)
//	,m_current (NULL)
	,m_threadId (GetCurrentThreadId())
{
}




dTimeTracker::dTimeTrackerEntry::dTimeTrackerEntry(dCRCTYPE nameCRC)
	:m_nameCRC (nameCRC)
{
	dTimeTracker* const instance = dTimeTracker::GetInstance();
	m_startTime = instance->GetTimeInMicrosenconds ();

	long long threadId = GetCurrentThreadId();
	dList<dTimeTracker::dTrackerThread>::dListNode* tracker = instance->m_tracks.GetFirst();
	for (dList<dTimeTracker::dTrackerThread>::dListNode* threadPtr = instance->m_tracks.GetFirst(); threadPtr; threadPtr = threadPtr->GetNext()) {
		if (threadPtr->GetInfo().m_threadId == threadId) {
			tracker = threadPtr;
			break;
		}
	}
	
	dTrackerThread& thread = tracker->GetInfo();
	m_thread = &thread;
	thread.m_stack.Append (this);
}

dTimeTracker::dTimeTrackerEntry::~dTimeTrackerEntry()
{
	dTimeTracker* const instance = dTimeTracker::GetInstance();
	dAssert (this == m_thread->m_stack.GetLast()->GetInfo());
	m_thread->m_stack.Remove (m_thread->m_stack.GetLast());
	m_endTime = instance->GetTimeInMicrosenconds ();
}


dTimeTracker::dTimeTracker ()
{
	QueryPerformanceFrequency(&m_frequency);
	QueryPerformanceCounter (&m_baseCount);

}

void dTimeTracker::CreateThread (const char* const name)
{
	m_tracks.Append (dTrackerThread (name));
}

dCRCTYPE dTimeTracker::RegisterName (const char* const name)
{
	dCRCTYPE crc = dCRC64 (name);
	m_dictionary.Insert (name, crc);
	return crc;
}

long long dTimeTracker::GetTimeInMicrosenconds()
{
	#ifdef _MSC_VER
		LARGE_INTEGER count;
		QueryPerformanceCounter (&count);
		count.QuadPart -= m_baseCount.QuadPart;
		LONGLONG ticks = LONGLONG (count.QuadPart * LONGLONG (1000000) / m_frequency.QuadPart);
		return ticks;
	#endif
/*
	#if (defined (_POSIX_VER) || defined (_POSIX_VER_64))
		timespec ts;
		clock_gettime(CLOCK_REALTIME, &ts); // Works on Linux
		//return unsigned64 (ts.tv_nsec / 1000) - baseCount;
		
		return unsigned64 (ts.tv_sec) * 1000000 + ts.tv_nsec / 1000 - baseCount;
	#endif


	#ifdef _MACOSX_VER
		timeval tp;
		gettimeofday(&tp, NULL);
		unsigned64 microsecunds =  unsigned64 (tp.tv_sec) * 1000000 + tp.tv_usec;
		return microsecunds - baseCount;
	#endif
*/
}




#endif