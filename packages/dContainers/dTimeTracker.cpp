
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

dTimeTracker* dTimeTracker::GetIntance()
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

dTimeTracker::dTimeTrackerEntry::dTimeTrackerEntry(const char* name)
	:m_name (name)
{
	long long threadId = GetCurrentThreadId();

	dList<dTimeTracker::dTrackerThread>::dListNode* tracker = dTimeTracker::GetIntance()->m_tracks.GetFirst();
	for (dList<dTimeTracker::dTrackerThread>::dListNode* threadPtr = dTimeTracker::GetIntance()->m_tracks.GetFirst(); threadPtr; threadPtr = threadPtr->GetNext()) {
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
	dAssert (this == m_thread->m_stack.GetLast()->GetInfo());
	m_thread->m_stack.Remove (m_thread->m_stack.GetLast());
}


dTimeTracker::dTimeTracker ()
{
}

void dTimeTracker::CreateThread (const char* const name)
{
	m_tracks.Append (dTrackerThread (name));
}

#endif