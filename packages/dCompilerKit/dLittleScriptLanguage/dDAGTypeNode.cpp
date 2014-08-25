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
	,m_type (dThreeAdressStmt::GetTypeID (typeName))
{
}

dDAGTypeNode::dDAGTypeNode(dList<dDAG*>& allNodes, const dDAGTypeNode& copySource)
	:dDAG(allNodes)
	,m_dimensions(copySource.m_dimensions)
	,m_type (copySource.m_type)
{
	m_name = copySource.m_name;
	for (dList<dDAGDimensionNode*>::dListNode* node = copySource.m_dimensions.GetFirst(); node; node = node->GetNext()) {
		dAssert (m_type.m_isPointer);
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


const dThreeAdressStmt::dArgType& dDAGTypeNode::GetArgType() const
{
	return m_type;
}
/*
bool dDAGTypeNode::IsPointer() const
{
	dAssert (0);
	return 0;
//	return m_isPointer;
}

dString dDAGTypeNode::GetIntrisicTypeString() const
{

	if (IsPointer()) {
		return dThreeAdressStmt::GetTypeString (GetIntrinsicType()) + dCIL::m_pointerDecoration;
	} else {
		return dThreeAdressStmt::GetTypeString (GetIntrinsicType());
	}
}
*/

void dDAGTypeNode::AddDimensions (dDAGDimensionNode* const dimList)
{
	m_type.m_isPointer = true;
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