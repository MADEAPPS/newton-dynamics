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
#include "dAnimationTwoWayBlend.h"

dAnimationTwoWayBlend::dAnimationTwoWayBlend(dAnimationBlendTreeNode* const node0, dAnimationBlendTreeNode* const node1)
	:dAnimationBlendTreeNode(NULL)
	,m_node0(node0)
	,m_node1(node1)
	,m_param(0.0f)
{
	m_param = 0.6f;
}

dAnimationTwoWayBlend::~dAnimationTwoWayBlend()
{
	delete m_node0;
	delete m_node1;
}

void dAnimationTwoWayBlend::Evaluate(dAnimationPose& output, dFloat timestep)
{
	if (m_param < 0.001f) {
		m_node0->Evaluate(output, timestep);
	} else if (m_param > 0.999f) {
		m_node1->Evaluate(output, timestep);
	} else {
		const int count = output.GetSize();
		dAnimKeyframe* const buffer = dAlloca(dAnimKeyframe, count);
		dAnimationLocalPose localPose(buffer);
		m_node0->Evaluate(output, timestep);
		m_node1->Evaluate(localPose, timestep);

		dAnimKeyframe* const dst = &output[0];
		const dAnimKeyframe* const src = &localPose[0];
		for (int i = 0; i < count; i ++) {
			dAnimKeyframe& dstFrame = dst[i];
			const dAnimKeyframe& srcFrame = src[i];
			dstFrame.m_rotation = dstFrame.m_rotation.Slerp(srcFrame.m_rotation, m_param);
			dstFrame.m_posit = dstFrame.m_posit + (srcFrame.m_posit - dstFrame.m_posit).Scale(m_param);
		}
	}
}