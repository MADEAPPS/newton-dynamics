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

#ifndef __D_ANIMIMATION_TAKE_DATA_h__
#define __D_ANIMIMATION_TAKE_DATA_h__

#include "dAnimationKeyframesTrack.h"

class dAnimationSequence
{
	public:
	dAnimationSequence();
	~dAnimationSequence();

//	static dAnimationSequence* LoadAnimation(const dScene& scene, const char* const animName);
	dAnimimationKeyFramesTrack* AddTrack();

	void Save(const char* const fileName);

	dFloat GetPeriod() const { return m_period; }
	void SetPeriod(dFloat period) { m_period = period;}

	dList<dAnimimationKeyFramesTrack>& GetTracks() { return m_tracks; }
	
	void CalculatePose(dAnimationPose& output, dFloat t) const;
	
	dList<dAnimimationKeyFramesTrack> m_tracks;
	dFloat m_period;
};


#endif