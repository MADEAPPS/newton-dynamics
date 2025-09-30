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

#ifndef __ND_ANIMATION_TWO_WAY_h__
#define __ND_ANIMATION_TWO_WAY_h__

#include "ndAnimationBlendTreeNode.h"

class ndAnimationTwoWayBlend: public ndAnimationBlendTreeNode
{
	public:
	ndAnimationTwoWayBlend(const ndSharedPtr<ndAnimationBlendTreeNode>& node0, const ndSharedPtr<ndAnimationBlendTreeNode>& node1);
	virtual ~ndAnimationTwoWayBlend();

	ndFloat32 GetParam() const;
	void SetParam(ndFloat32 param);

	void Update(ndFloat32 dt);
	void Evaluate(ndAnimationPose& output, ndVector& veloc);

	protected:
	ndSharedPtr<ndAnimationBlendTreeNode> m_node0;
	ndSharedPtr<ndAnimationBlendTreeNode> m_node1;
	ndFloat32 m_param;
};

inline ndFloat32 ndAnimationTwoWayBlend::GetParam() const
{
	return m_param; 
}

inline void ndAnimationTwoWayBlend::SetParam(ndFloat32 param) 
{ 
	m_param = param; 
}

#endif