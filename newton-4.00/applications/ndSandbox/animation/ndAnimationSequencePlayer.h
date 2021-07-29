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


#include "ndAnimationBlendTreeNode.h"

class ndAnimPose;
class ndAnimationSequence;

class ndAnimationSequencePlayer: public ndAnimationBlendTreeNode
{
	public:
	ndAnimationSequencePlayer(ndAnimationSequence* const sequence);
	virtual ~ndAnimationSequencePlayer();

	virtual void Evaluate(ndAnimationPose& output, dFloat32 timestep);

	void SetFrame(dFloat32 timestep)
	{
		m_time = timestep;
	}

	void AdvanceFrame(dFloat32 timestep)
	{
		SetFrame (dMod(m_time + timestep, m_sequence->m_period));
	}

	dFloat32 GetFrame() const 
	{
		return m_time;
	}

	ndAnimationSequence* GetSequence()
	{
		return m_sequence;
	}

	private:
	ndAnimationSequence* m_sequence;
	dFloat32 m_time;
};


#endif