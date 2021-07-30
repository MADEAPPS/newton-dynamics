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
#define __D_ANIMATION_TWO_WAY_h__

#include "ndSandboxStdafx.h"
#include "ndAnimationBlendTreeNode.h"

class ndAnimationTwoWayBlend: public ndAnimationBlendTreeNode
{
	public:
	ndAnimationTwoWayBlend(ndAnimationBlendTreeNode* const node0, ndAnimationBlendTreeNode* const node1);
	virtual ~ndAnimationTwoWayBlend();

	dFloat32 GetParam() const;
	void SetParam(dFloat32 param);

	void SetTimeDilation0 (dFloat32 dilation)
	{
		m_timeDilation0 = dilation;
	}

	void SetTimeDilation1(dFloat32 dilation)
	{
		m_timeDilation1 = dilation;
	}

	virtual void Evaluate(ndAnimationPose& output, dFloat32 timestep);

	protected:
	ndAnimationBlendTreeNode* m_node0;
	ndAnimationBlendTreeNode* m_node1;
	dFloat32 m_timeDilation0;
	dFloat32 m_timeDilation1;
	dFloat32 m_param;
};

inline dFloat32 ndAnimationTwoWayBlend::GetParam() const
{
	return m_param; 
}

inline void ndAnimationTwoWayBlend::SetParam(dFloat32 param) 
{ 
	m_param = param; 
}

#endif