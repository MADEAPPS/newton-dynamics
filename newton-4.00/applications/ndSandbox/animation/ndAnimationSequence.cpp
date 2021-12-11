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

#include "ndSandboxStdafx.h"
#include "ndAnimationPose.h"
#include "ndAnimationSequence.h"

ndAnimationSequence::ndAnimationSequence()
	:m_name("")
	,m_tracks()
	,m_period(dFloat32 (1.0f))
{
}

ndAnimationSequence::~ndAnimationSequence()
{
}

ndAnimationKeyFramesTrack* ndAnimationSequence::AddTrack()
{
	ndList<ndAnimationKeyFramesTrack>::ndNode* const node = m_tracks.Append();
	return &node->GetInfo();
}

void ndAnimationSequence::CalculatePose(ndAnimationPose& output, dFloat32 t) const
{
	if (output.GetCount())
	{
		dInt32 index = 0;
		ndAnimKeyframe* const keyFrames = &output[0];
		for (ndList<ndAnimationKeyFramesTrack>::ndNode* srcNode = m_tracks.GetFirst(); srcNode; srcNode = srcNode->GetNext())
		{
			const ndAnimationKeyFramesTrack& track = srcNode->GetInfo();
			ndAnimKeyframe& keyFrame = keyFrames[index];
			track.InterpolatePosition(t, m_period, keyFrame.m_posit);
			track.InterpolateRotation(t, m_period, keyFrame.m_rotation);
			dAssert(keyFrame.m_rotation.DotProduct(keyFrame.m_rotation).GetScalar() > 0.999f);
			dAssert(keyFrame.m_rotation.DotProduct(keyFrame.m_rotation).GetScalar() < 1.001f);

			index++;
		}
	}
}
