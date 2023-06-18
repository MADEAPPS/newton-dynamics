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

#ifndef __ND_ANIMIMATION_SEQUENCE_h__
#define __ND_ANIMIMATION_SEQUENCE_h__

#include "ndAnimationKeyframesTrack.h"

class ndAnimationPose;

class ndAnimationSequence : public ndContainersFreeListAlloc<ndAnimationSequence>
{
	public:
	ndAnimationSequence();
	virtual ~ndAnimationSequence();

	ndFloat32 GetDuration() const;
	const ndString& GetName() const;
	void SetName(const char* const name);

	ndAnimationKeyFramesTrack* AddTrack();
	ndList<ndAnimationKeyFramesTrack>& GetTracks();
	ndAnimationKeyFramesTrack& GetTranslationTrack();

	virtual ndVector GetTranslation(ndFloat32 param) const;
	virtual void CalculatePose(ndAnimationPose& output, ndFloat32 param) const;

	protected:
	ndList<ndAnimationKeyFramesTrack> m_tracks;
	ndAnimationKeyFramesTrack m_translationTrack;
	ndString m_name;
	ndFloat32 m_duration;

	friend class ndFbxMeshLoader;
};

#endif