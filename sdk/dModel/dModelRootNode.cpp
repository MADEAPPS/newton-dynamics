/* Copyright (c) <2003-2019> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "dModelStdAfx.h"
#include "dModelManager.h"
#include "dModelRootNode.h"

dModelRootNode::dModelRootNode(NewtonBody* const rootBody, const dMatrix& bindMatrix)
	:dModelNode(rootBody, bindMatrix, NULL)
	,m_localTransformMode(false)
{
}

dModelRootNode::~dModelRootNode()
{
}


void dModelRootNode::PostUpdate(dModelManager* const manager, dFloat timestep) const
{
	if (m_localTransformMode) {

		dMatrix parentMatrixPool[128];
		const dModelNode* stackPool[128];

		int stack = 1;
		stackPool[0] = this;
		parentMatrixPool[0] = dGetIdentityMatrix();

		while (stack) {
			dMatrix matrix;
			stack--;

			dMatrix parentMatrix(parentMatrixPool[stack]);
			const dModelNode* const bone = stackPool[stack];

			NewtonBodyGetMatrix(bone->GetBody(), &matrix[0][0]);
			manager->OnUpdateTransform(bone, matrix * parentMatrix * bone->GetBindMatrix());

			parentMatrix = matrix.Inverse();
			for (dList<dPointer<dModelNode>>::dListNode* ptrNode = bone->m_children.GetFirst(); ptrNode; ptrNode = ptrNode->GetNext()) {
				parentMatrixPool[stack] = parentMatrix;
				stackPool[stack] = ptrNode->GetInfo().GetData();
				stack++;
			}
		}
	}
}

