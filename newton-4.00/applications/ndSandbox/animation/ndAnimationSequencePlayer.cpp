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
#include "ndAnimationSequencePlayer.h"

ndAnimationSequencePlayer::ndAnimationSequencePlayer(ndAnimationSequence* const sequence)
	:ndAnimationBlendTreeNode(nullptr)
	,m_sequence(sequence)
	,m_time(ndFloat32 (0.0f))
{
}

ndAnimationSequencePlayer::~ndAnimationSequencePlayer()
{
}

void ndAnimationSequencePlayer::SetFrame(ndFloat32 absoluteTime)
{
	m_time = dMod(absoluteTime, m_sequence->m_period);
}

void ndAnimationSequencePlayer::Evaluate(ndAnimationPose& output, ndFloat32 timestep)
{
	AdvanceFrame(timestep);
	m_sequence->CalculatePose(output, m_time);
}


