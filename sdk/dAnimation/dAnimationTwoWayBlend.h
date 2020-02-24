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

#ifndef __D_ANIMATION_TWO_WAY_h__
#define __D_ANIMATION_EFFECTOR_BLEND_TWO_WAY_h__

#include "dAnimationBlendTreeNode.h"


class dAnimationTwoWayBlend: public dAnimationBlendTreeNode
{
	public:
	dAnimationTwoWayBlend(dAnimationBlendTreeNode* const node0, dAnimationBlendTreeNode* const node1);
	virtual ~dAnimationTwoWayBlend();

	dFloat GetParam () const {return m_param;}
	void SetParam (dFloat param) {m_param = param;}

	void SetTimeDilation0 (dFloat dilation)
	{
		m_timeDilation0 = dilation;
	}

	void SetTimeDilation1(dFloat dilation)
	{
		m_timeDilation1 = dilation;
	}

	virtual void Evaluate(dAnimationPose& output, dFloat timestep);

	protected:
	dAnimationBlendTreeNode* m_node0;
	dAnimationBlendTreeNode* m_node1;
	dFloat m_timeDilation0;
	dFloat m_timeDilation1;
	dFloat m_param;
};


#endif