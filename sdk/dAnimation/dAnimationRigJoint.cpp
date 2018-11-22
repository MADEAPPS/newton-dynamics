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
#include "dAnimationRigJoint.h"
#include "dAnimationCharacterRigManager.h"


dAnimationRigJoint::dAnimationRigJoint(dAnimationRigJoint* const parent, NewtonBody* const body)
	:dAnimationAcyclicJoint(parent)
	,m_body(body)
{
}

dAnimationRigJoint::~dAnimationRigJoint()
{
}


void dAnimationRigJoint::UpdateLocalTransforms (dAnimationCharacterRigManager* const manager) const
{
	dMatrix parentMatrixPool[128];
	const dAnimationRigJoint* stackPool[128];
	
	int stack = 1;
	stackPool[0] = this;
	parentMatrixPool[0] = dGetIdentityMatrix();

	while (stack) {
		dMatrix matrix;
		stack--;

		dMatrix parentMatrix(parentMatrixPool[stack]);
		const dAnimationRigJoint* const node = stackPool[stack];
		NewtonBody* const newtonBody = node->GetNewtonBody();

		if (newtonBody) {
			NewtonBodyGetMatrix(newtonBody, &matrix[0][0]);
			//manager->OnUpdateTransform(node, matrix * parentMatrix * bone.m_bindMatrix);
			manager->OnUpdateTransform(node, matrix * parentMatrix);

			parentMatrix = matrix.Inverse();
			for (dList<dAnimationAcyclicJoint*>::dListNode* child = m_children.GetFirst(); child; child = child->GetNext()) {
				stackPool[stack] = (dAnimationRigJoint*) child->GetInfo();
				parentMatrixPool[stack] = parentMatrix;
				stack++;
			}
		}
	}
}
