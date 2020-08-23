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

#include "dAnimationStdAfx.h"
#include "dAnimationPose.h"
#include "dAnimationSequence.h"
#include "dAnimationSequencePlayer.h"


dAnimationSequencePlayer::dAnimationSequencePlayer(dAnimationSequence* const sequence)
	:dAnimationBlendTreeNode(NULL)
	,m_sequence(sequence)
	,m_time(0.0f)
{
}

dAnimationSequencePlayer::~dAnimationSequencePlayer()
{
}


void dAnimationSequencePlayer::Evaluate(dAnimationPose& output, dFloat timestep)
{
	AdvanceFrame(timestep);
	m_sequence->CalculatePose(output, m_time);
}


