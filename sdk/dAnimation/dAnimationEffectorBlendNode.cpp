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
#include "dAnimationCharacterRig.h"
#include "dAnimationEffectorBlendNode.h"

dAnimationPose::dAnimationPose(dAnimationCharacterRig* const character)
	:dList<dAnimationTransform>()
{
//	NewtonBodyGetMatrix(character->Get hexaBody, &rootMatrix[0][0]);
//	rootMatrix = rootMatrix.Inverse();
//	for (int i = 0; i < legEffectorCount; i++) {
//		dEffectorTreeInterface::dEffectorTransform frame;
//		dCustomInverseDynamicsEffector* const effector = legEffectors[i];
//		dMatrix effectorMatrix(effector->GetBodyMatrix());
//		dMatrix poseMatrix(effectorMatrix * rootMatrix);
//		frame.m_effector = effector;
//		frame.m_posit = poseMatrix.m_posit;
//		frame.m_rotation = dQuaternion(poseMatrix);
//		idlePose->GetPose().Append(frame);
//		walkPoseGenerator->GetPose().Append(frame);
//		m_animTreeNode->GetPose().Append(frame);
//	}


	dMatrix rootMatrix (character->GetBasePoseMatrix());

	const dAnimationRigJoint* stackPool[128];

	int stack = 1;
	stackPool[0] = character;

	while (stack) {

		dMatrix matrix;
		stack--;
		const dAnimationRigJoint* const node = stackPool[stack];

		//NewtonBodyGetMatrix(newtonBody, &matrix[0][0]);
		//manager->OnUpdateTransform(node, matrix * parentMatrix);
		//parentMatrix = matrix.Inverse();


		const dList<dAnimationAcyclicJoint*>& children = node->GetChildren();
		for (dList<dAnimationAcyclicJoint*>::dListNode* child = children.GetFirst(); child; child = child->GetNext()) {
			stackPool[stack] = (dAnimationRigJoint*)child->GetInfo();
			stack++;
		}
	}
}

dAnimationEffectorBlendNode::dAnimationEffectorBlendNode(dAnimationCharacterRig* const character)
	:dCustomAlloc()
	,m_character(character)
{
}

dAnimationEffectorBlendNode::~dAnimationEffectorBlendNode()
{
}

