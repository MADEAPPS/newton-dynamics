/* Copyright (c) <2009> <Newton Game Dynamics>
*
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
*
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "dLSCstdafx.h"
#include "dDAG.h"
#include "dDAGTypeNode.h"
#include "dDAGDimensionNode.h"

dInitRtti(dDAGTypeNode);



dDAGTypeNode::dDAGTypeNode(dList<dDAG*>& allNodes, const dString& typeName)
	:dDAG(allNodes)
	,m_dimensions()
	,m_intrinsicType(dTreeAdressStmt::m_int)
{
	/*
	m_name = typeName;
	bool foundMatch = false;
	for (int i = 0; sizeof (m_maptable) / sizeof (m_maptable[0]); i ++) {
		if (m_name == m_maptable[i].m_name) {
			foundMatch = true;
			m_intrinsicType = m_maptable[i].m_type;
			break;
		}
	}
	dAssert (foundMatch);
*/
}

dDAGTypeNode::dDAGTypeNode(dList<dDAG*>& allNodes, const dDAGTypeNode& copySource)
	:dDAG(allNodes)
	,m_dimensions()
{
	m_name = copySource.m_name;
	for (dList<dDAGDimensionNode*>::dListNode* node = copySource.m_dimensions.GetFirst(); node; node = node->GetNext()) {
		dDAGDimensionNode* const dim = node->GetInfo();
		m_dimensions.Append ((dDAGDimensionNode*)dim->Clone(allNodes));
	}
}

dDAGTypeNode::~dDAGTypeNode(void)
{
}


dDAG* dDAGTypeNode::Clone (dList<dDAG*>& allNodes) const
{
	return new dDAGTypeNode(allNodes, *this);
}


void dDAGTypeNode::AddDimensions (dDAGDimensionNode* const dimList)
{
	for (dDAGDimensionNode* node = dimList; node; node = node->m_next) {
		dAssert (node->IsType(dDAGDimensionNode::GetRttiType()));
		m_dimensions.Append(node);
	}
}


void dDAGTypeNode::ConnectParent(dDAG* const parent)  
{
	m_parent = parent;
	for (dList<dDAGDimensionNode*>::dListNode* node = m_dimensions.GetFirst(); node; node = node->GetNext()) {
		dDAGDimensionNode* const dim = node->GetInfo();
		dim->ConnectParent(this);
	}
}