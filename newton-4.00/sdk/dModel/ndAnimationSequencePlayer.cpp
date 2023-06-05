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
	ndFloat32 t1 = ndFloat32(2.0f) * time * ndPi / duration;
	ndFloat32 t0 = ndFloat32 (2.0f) * m_time * ndPi / duration;
	m_time = ndMod (t0 + ndAnglesSub(t1, t0), ndFloat32(2.0f) * ndPi) * duration / (ndFloat32(2.0f) * ndPi);
	ndAssert(m_time > 0.0f);
}

ndSharedPtr<ndAnimationSequence>& ndAnimationSequencePlayer::GetSequence()
{
	return m_sequence;
}

void ndAnimationSequencePlayer::Evaluate(ndAnimationPose& output)
{
	m_sequence->CalculatePose(output, m_time);
}


