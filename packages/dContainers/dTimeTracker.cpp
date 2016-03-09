
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
	,m_root (NULL)
	,m_current (NULL)
{
	dTimeTracker::GetIntance()->m_tracks.Append (this);
}



dTimeTracker::dTimeTracker ()
{
}

//dTimeTrackerEntry* dTimeTrackerEntry::CreateTrack();

#endif