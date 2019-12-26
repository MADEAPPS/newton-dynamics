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
#include "dModelNode.h"
#include "dModelManager.h"

dModelNode::dModelNode(NewtonBody* const modelBody, const dMatrix& bindMatrix, dModelNode* const parent)
	:dCustomAlloc()
	,m_bindMatrix(bindMatrix)
	,m_userData(NULL)
	,m_body(modelBody)
	,m_parent(parent)
{
	if (m_parent) {
		m_parent->m_children.Append(this);
	}
}

dModelNode::~dModelNode()
{
	while (m_children.GetCount()) {
		delete m_children.GetFirst()->GetInfo();
		m_children.Remove(m_children.GetFirst());
	}
}

const dModelNode* dModelNode::GetRoot() const
{
	const dModelNode* root = this;
	while (root->GetParent()) {
		root = root->GetParent();
	}
	return root;
}

const dCustomJoint* dModelNode::GetJoint() const
{
	if (m_parent) {
		NewtonJoint* const joint = NewtonWorldFindJoint(GetBody(), m_parent->GetBody());
		return joint ? (dCustomJoint*) NewtonJointGetUserData(joint) : NULL;
	}
	return NULL;
}

void dModelNode::ForEachNode (Callback callback, void* const context) const
{
	const dModelNode* stackPool[128];

	int stack = 1;
	stackPool[0] = this;
	while (stack) {
		stack--;
		const dModelNode* const bone = stackPool[stack];
		(this->*callback) (bone, context);

		for (dModelChildrenList::dListNode* ptrNode = bone->m_children.GetFirst(); ptrNode; ptrNode = ptrNode->GetNext()) {
			stackPool[stack] = ptrNode->GetInfo();
			stack++;
		}
	}
}
