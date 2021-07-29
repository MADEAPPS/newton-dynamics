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

class ndAnimationSequence: public dClassAlloc
{
	public:
	ndAnimationSequence();
	~ndAnimationSequence();

	ndAnimationKeyFramesTrack* AddTrack();

	void Load(const char* const fileName);
	void Save(const char* const fileName);

	dFloat32 GetPeriod() const;
	void SetPeriod(dFloat32 period);
	dList<ndAnimationKeyFramesTrack>& GetTracks();
	
	void CalculatePose(ndAnimationPose& output, dFloat32 t) const;
	
	dList<ndAnimationKeyFramesTrack> m_tracks;
	dFloat32 m_period;
};

inline dFloat32 ndAnimationSequence::GetPeriod() const
{ 
	return m_period; 
}

inline void ndAnimationSequence::SetPeriod(dFloat32 period) 
{ 
	m_period = period; 
}

inline dList<ndAnimationKeyFramesTrack>& ndAnimationSequence::GetTracks() 
{ 
	return m_tracks; 
}

#endif