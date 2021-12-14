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

#ifndef __D_ANIMIMATION_SEQUENCE_h__
#define __D_ANIMIMATION_SEQUENCE_h__

#include "ndSandboxStdafx.h"
#include "ndAnimationKeyframesTrack.h"

class ndAnimationPose;

class ndAnimationSequence: public ndClassAlloc
{
	public:
	ndAnimationSequence();
	~ndAnimationSequence();

	ndAnimationKeyFramesTrack* AddTrack();

	ndFloat32 GetPeriod() const;
	void SetPeriod(ndFloat32 period);
	ndList<ndAnimationKeyFramesTrack>& GetTracks();
	
	const ndString& GetName() const;
	void SetName(const char* const name);
	void CalculatePose(ndAnimationPose& output, ndFloat32 t) const;
	
	ndString m_name;
	ndList<ndAnimationKeyFramesTrack> m_tracks;
	ndFloat32 m_period;
};

inline const ndString& ndAnimationSequence::GetName() const
{
	return m_name;
}

inline void ndAnimationSequence::SetName(const char* const name)
{
	m_name = name;
}


inline ndFloat32 ndAnimationSequence::GetPeriod() const
{ 
	return m_period; 
}

inline void ndAnimationSequence::SetPeriod(ndFloat32 period) 
{ 
	m_period = period; 
}

inline ndList<ndAnimationKeyFramesTrack>& ndAnimationSequence::GetTracks() 
{ 
	return m_tracks; 
}

#endif