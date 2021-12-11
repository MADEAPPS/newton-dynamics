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
#include "ndAnimationBlendTreeNode.h"

/*
ndAnimationBlendTreeNode::dAnimationPose::dAnimationPose()
	:dList<dAnimationTransform>()
{
	dAssert (0);
//	dList<dAnimIDRigEffector*>& effectorsList = character->GetEffectors();
//	for (dList<dAnimIDRigEffector*>::dListNode* node = effectorsList.GetFirst(); node; node = node->GetNext()) {
//		dAnimIDRigEffector* const effector = node->GetInfo();
//
//		const dMatrix& localMatrix = effector->GetLocalMatrix();
//
//		dAnimationTransform frame;
//		frame.m_effector = effector;
//		frame.m_posit = localMatrix.m_posit;
//		frame.m_rotation = dQuaternion(localMatrix);
//		Append(frame);
//	}
}

void ndAnimationBlendTreeNode::dAnimationPose::CopySource(const dAnimationPose& source)
{
	dAssert (0);
//	dListNode* destNode = GetFirst();
//	for (dListNode* sourceNode = source.GetFirst(); sourceNode; sourceNode = sourceNode->GetNext()) {
//		const dAnimationTransform& srcFrame = sourceNode->GetInfo();
//		dAnimationTransform& dstFrame = destNode->GetInfo();
//		dAssert(srcFrame.m_effector == dstFrame.m_effector);
//		dstFrame.m_rotation = srcFrame.m_rotation;
//		dstFrame.m_posit = srcFrame.m_posit;
//		destNode = destNode->GetNext();
//	}
}

void ndAnimationBlendTreeNode::dAnimationPose::SetTargetPose() const
{
	dAssert (0);
//	dMatrix rootMatrix(character->GetBasePoseMatrix());
//	for (dListNode* node = GetFirst(); node; node = node->GetNext()) {
//		const dAnimationTransform& frame = node->GetInfo();
//		dMatrix matrix(dMatrix(frame.m_rotation, frame.m_posit) * rootMatrix);
//		frame.m_effector->SetTargetPose(matrix);
//	}
}
*/

ndAnimationBlendTreeNode::ndAnimationBlendTreeNode(ndAnimationBlendTreeNode* const child)
	:ndClassAlloc()
//	,m_character(character)
	,m_input(child)
{
}

ndAnimationBlendTreeNode::~ndAnimationBlendTreeNode()
{
	if (m_input) 
	{
		delete m_input;
	}
}

