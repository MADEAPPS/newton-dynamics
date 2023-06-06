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
#include "ndAnimationSequencePlayer.h"

ndAnimationSequencePlayer::ndAnimationSequencePlayer(ndSharedPtr<ndAnimationSequence>& sequence)
	:ndAnimationBlendTreeNode(nullptr)
	,m_sequence(sequence)
	,m_veloc(ndVector::m_zero)
	,m_time(ndFloat32 (0.0f))
{
}

ndAnimationSequencePlayer::~ndAnimationSequencePlayer()
{
}

ndFloat32 ndAnimationSequencePlayer::GetTime() const
{
	return m_time;
}

void ndAnimationSequencePlayer::SetTime(ndFloat32 time)
{
	ndFloat32 duration = m_sequence->GetDuration();
	ndFloat32 scale = ndFloat32(2.0f) * ndPi / duration;

	ndFloat32 angle1 = time * scale;
	ndFloat32 angle0 = m_time * scale;
	ndFloat32 angle = angle0 + ndAnglesSub(angle1, angle0);
	m_time = ndMod (angle, ndFloat32(2.0f) * ndPi) / scale;
	ndAssert(m_time >= 0.0f);
}

ndSharedPtr<ndAnimationSequence>& ndAnimationSequencePlayer::GetSequence()
{
	return m_sequence;
}

void ndAnimationSequencePlayer::Evaluate(ndAnimationPose& output, ndVector& veloc)
{
	veloc = m_veloc;
	m_sequence->CalculatePose(output, m_time);
}


