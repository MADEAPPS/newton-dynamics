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
#include "dDAGExpressionNodeVariable.h"
#include "dDAGExpressionClassVariable.h"

dInitRtti(dDAGExpressionClassVariable);

dDAGExpressionClassVariable::dDAGExpressionClassVariable (dList<dDAG*>& allNodes, dDAGExpressionNode* const expression)
	:dDAGExpressionNode(allNodes)
	,m_expression(expression)
	,m_variable (expression->FindLeftVariable()) 
	,m_iniatilized(false)
	,m_initialValue()
{
	dAssert (m_variable);
}


dDAGExpressionClassVariable::~dDAGExpressionClassVariable ()
{
	dAssert (0);
}


dCIL::dReturnValue dDAGExpressionClassVariable::Evalue(const dDAGFunctionNode* const function) 
{
dAssert (0);
return dCIL::dReturnValue();
/*
	dCIL::dReturnValue val;
	val.m_type = m_variable->m_type->m_intrinsicType;
	val.m_f = 0.0;
	if ((dDAG*)m_expression != (dDAG*)m_variable) {
		val = m_expression->Evalue(function);
	}
	m_iniatilized = true;
	m_initialValue = val;
	return val;
*/
}

void dDAGExpressionClassVariable::ConnectParent(dDAG* const parent)
{
dAssert (0);
	m_parent = parent;
	m_expression->ConnectParent(this);
}


