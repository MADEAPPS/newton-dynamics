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

#include "dLSCstdafx.h"
#include "dDAG.h"
#include "dDAGTypeNode.h"
#include "dDAGDimensionNode.h"
#include "dDAGExpressionNodeConstant.h"


dInitRtti(dDAGDimensionNode);

dDAGDimensionNode::dDAGDimensionNode(dList<dDAG*>& allNodes, dDAGExpressionNode* const size)
	:dDAG(allNodes)
	,m_arraySize("fix_ArraySize")
	,m_dimExp (size)
	,m_next (NULL)
{
}

dDAGDimensionNode::dDAGDimensionNode(dList<dDAG*>& allNodes, const dDAGDimensionNode& copy)
	:dDAG(allNodes)
	,m_arraySize(copy.m_arraySize)
	,m_dimExp (NULL)
	,m_next (NULL)
{
	if (copy.m_dimExp) {
		m_dimExp = (dDAGExpressionNode *) copy.m_dimExp->Clone (allNodes);
	}
}

dDAGDimensionNode::~dDAGDimensionNode(void)
{
}

dDAG* dDAGDimensionNode::Clone (dList<dDAG*>& allNodes) const
{
	return new dDAGDimensionNode (allNodes, *this);
}

void dDAGDimensionNode::ConnectParent(dDAG* const parent)  
{
	m_parent = parent;
	if (m_dimExp) {
		m_dimExp->ConnectParent(this);
	}

	if (m_next) {
		m_next->ConnectParent(this);
	}
}

void dDAGDimensionNode::CompileCIL(dCIL& cil)  
{
	if (m_dimExp) {
		m_dimExp->CompileCIL(cil) ;
		m_result = LoadLocalVariable(cil, m_dimExp->m_result);
		dAssert (m_result.GetType().m_intrinsicType != dCILInstr::m_constFloat);
		if (m_result.GetType().m_intrinsicType == dCILInstr::m_constInt) {
			dCILInstrMove* const move = new dCILInstrMove (cil, cil.NewTemp(), dCILInstr::dArgType (dCILInstr::m_int), m_result.m_label, m_result.GetType());
			move->Trace();
			m_result = move->GetArg0();
		}
		dAssert (m_result.GetType().m_intrinsicType == dCILInstr::m_int);
	} else {
		dAssert (0);
	}
}