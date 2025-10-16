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

#ifndef __ND_ANIM_TAKE_DATA_h__
#define __ND_ANIM_TAKE_DATA_h__


#include "ndAnimationBlendTreeNode.h"

class ndAnimPose;
class ndAnimationSequence;

class ndAnimationSequencePlayer: public ndAnimationBlendTreeNode
{
	public:
	ndAnimationSequencePlayer(ndSharedPtr<ndAnimationSequence>& sequence);
	virtual ~ndAnimationSequencePlayer();

	virtual void Evaluate(ndAnimationPose& output, ndVector& veloc);

	ndFloat32 GetTime() const;
	void SetTime(ndFloat32 time);

	virtual void Update(ndFloat32 dt);

	ndSharedPtr<ndAnimationSequence>& GetSequence();

	private:
	ndSharedPtr<ndAnimationSequence> m_sequence;
	ndVector m_veloc;
	ndFloat32 m_time;
};

#endif