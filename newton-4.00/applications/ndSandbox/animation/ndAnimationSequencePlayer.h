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

	virtual void Evaluate(ndAnimationPose& output, ndFloat32 timestep);

	void SetFrame(ndFloat32 timestep);
	void AdvanceFrame(ndFloat32 timestep);

	ndFloat32 GetFrame() const 
	{
		return m_time;
	}

	ndAnimationSequence* GetSequence()
	{
		return m_sequence;
	}

	private:
	ndAnimationSequence* m_sequence;
	ndFloat32 m_time;
};

inline void ndAnimationSequencePlayer::AdvanceFrame(ndFloat32 timestep)
{
	SetFrame(m_time + timestep);
}
#endif