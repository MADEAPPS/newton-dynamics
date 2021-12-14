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
#include "ndAnimationTwoWayBlend.h"

ndAnimationTwoWayBlend::ndAnimationTwoWayBlend(ndAnimationBlendTreeNode* const node0, ndAnimationBlendTreeNode* const node1)
	:ndAnimationBlendTreeNode(nullptr)
	,m_node0(node0)
	,m_node1(node1)
	,m_timeDilation0(1.0f)
	,m_timeDilation1(1.0f)
	,m_param(0.0f)
{
}

ndAnimationTwoWayBlend::~ndAnimationTwoWayBlend()
{
	delete m_node0;
	delete m_node1;
}

void ndAnimationTwoWayBlend::Evaluate(ndAnimationPose& output, ndFloat32 timestep)
{
	if (m_param < 0.001f) 
	{
		m_node0->Evaluate(output, timestep);
	} 
	else if (m_param > 0.999f) 
	{
		m_node1->Evaluate(output, timestep);
	} 
	else 
	{
		const int count = output.GetCount();
		ndAnimKeyframe* const buffer = dAlloca(ndAnimKeyframe, count + 32);
		ndAnimationLocalPose localPose(buffer);
		localPose.SetCount(count);
		m_node0->Evaluate(output, timestep * m_timeDilation0);
		m_node1->Evaluate(localPose, timestep * m_timeDilation1);

		ndAnimKeyframe* const dst = &output[0];
		const ndAnimKeyframe* const src = &localPose[0];
		for (ndInt32 i = 0; i < count; i ++) 
		{
			ndAnimKeyframe& dstFrame = dst[i];
			const ndAnimKeyframe& srcFrame = src[i];
			dstFrame.m_rotation = dstFrame.m_rotation.Slerp(srcFrame.m_rotation, m_param);
			dstFrame.m_posit = dstFrame.m_posit + (srcFrame.m_posit - dstFrame.m_posit).Scale(m_param);
		}
	}
}