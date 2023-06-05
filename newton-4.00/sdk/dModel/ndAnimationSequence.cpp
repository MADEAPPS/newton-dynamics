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

#include "ndModelStdafx.h"
#include "ndAnimationPose.h"
#include "ndAnimationSequence.h"

ndAnimationSequence::ndAnimationSequence()
	:ndClassAlloc()
	,m_tracks()
	,m_translationTrack()
	,m_name("")
	,m_duration(ndFloat32 (0.0f))
{
}

ndAnimationSequence::~ndAnimationSequence()
{
}

const ndString& ndAnimationSequence::GetName() const
{
	return m_name;
}

ndFloat32 ndAnimationSequence::GetDuration() const
{
	return m_duration;
}

void ndAnimationSequence::SetName(const char* const name)
{
	m_name = name;
}

ndAnimationKeyFramesTrack& ndAnimationSequence::GetTranslationTrack()
{
	return m_translationTrack;
}

ndList<ndAnimationKeyFramesTrack>& ndAnimationSequence::GetTracks()
{
	return m_tracks;
}

ndAnimationKeyFramesTrack* ndAnimationSequence::AddTrack()
{
	ndList<ndAnimationKeyFramesTrack>::ndNode* const node = m_tracks.Append();
	return &node->GetInfo();
}

void ndAnimationSequence::CalculatePose(ndAnimationPose& output, ndFloat32 param) const
{
	if (output.GetCount())
	{
		ndInt32 index = 0;
		ndAnimKeyframe* const keyFrames = &output[0];
		for (ndList<ndAnimationKeyFramesTrack>::ndNode* srcNode = m_tracks.GetFirst(); srcNode; srcNode = srcNode->GetNext())
		{
			const ndAnimationKeyFramesTrack& track = srcNode->GetInfo();
			ndAnimKeyframe& keyFrame = keyFrames[index];
			track.InterpolatePosition(param, keyFrame.m_posit);
			track.InterpolateRotation(param, keyFrame.m_rotation);
			ndAssert(keyFrame.m_rotation.DotProduct(keyFrame.m_rotation).GetScalar() > 0.999f);
			ndAssert(keyFrame.m_rotation.DotProduct(keyFrame.m_rotation).GetScalar() < 1.001f);

			index++;
		}
	}
}
