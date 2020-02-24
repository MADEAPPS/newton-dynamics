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

#ifndef __D_ANIM_TAKE_DATA_h__
#define __D_ANIM_TAKE_DATA_h__


#include "dAnimationBlendTreeNode.h"

class dAnimPose;
class dAnimationSequence;

class dAnimationSequencePlayer: public dAnimationBlendTreeNode
{
	public:
	dAnimationSequencePlayer(dAnimationSequence* const sequence);
	virtual ~dAnimationSequencePlayer();

	virtual void Evaluate(dAnimationPose& output, dFloat timestep);

	void SetFrame(dFloat timestep)
	{
		m_time = timestep;
	}

	void AdvanceFrame(dFloat timestep)
	{
		SetFrame (dMod(m_time + timestep, m_sequence->m_period));
	}

	dFloat GetFrame() const 
	{
		return m_time;
	}

	dAnimationSequence* GetSequence()
	{
		return m_sequence;
	}

	private:
	dAnimationSequence* m_sequence;
	dFloat m_time;
};


#endif