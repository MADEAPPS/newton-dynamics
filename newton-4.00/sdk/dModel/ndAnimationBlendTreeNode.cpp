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

#include "ndModelStdafx.h"
#include "ndAnimationPose.h"
#include "ndAnimationBlendTreeNode.h"

ndAnimationBlendTreeNode::ndAnimationBlendTreeNode(const ndSharedPtr<ndAnimationBlendTreeNode>& child)
	:ndContainersFreeListAlloc<ndAnimationBlendTreeNode>()
	,m_input(child)
{
}

ndAnimationBlendTreeNode::~ndAnimationBlendTreeNode()
{
	//if (m_input) 
	//{
	//	delete m_input;
	//}
}

void ndAnimationBlendTreeNode::SetTime(ndFloat32 dt)
{
	if (m_input)
	{
		m_input->SetTime(dt);
	}
}

void ndAnimationBlendTreeNode::Update(ndFloat32 dt)
{
	if (m_input)
	{
		m_input->Update(dt);
	}
}

void ndAnimationBlendTreeNode::Evaluate(ndAnimationPose& output, ndVector& veloc)
{
	veloc = ndVector::m_zero;
	if (m_input)
	{
		m_input->Evaluate(output, veloc);
	}
}
