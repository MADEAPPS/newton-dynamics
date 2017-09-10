
/* Copyright (c) <2003-2016> <Newton Game Dynamics>
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


#ifdef D_TIME_TRACKER

class dTimeTracker
{
	public:
	private:
	dTimeTracker ();
	~dTimeTracker ();
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