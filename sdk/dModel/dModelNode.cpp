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
	,m_body(modelBody)
	,m_parent(parent)
	,m_children()
{
	if (m_parent) {
		dModelChildrenList::dListNode* const node = m_parent->m_children.Append();
		node->GetInfo().SetData(this);
	}
}

dModelNode::~dModelNode()
{
}

const dModelNode* dModelNode::GetRoot() const
{
	const dModelNode* root = this;
	while (root->GetParent()) {
		root = root->GetParent();
	}
	return root;
}

