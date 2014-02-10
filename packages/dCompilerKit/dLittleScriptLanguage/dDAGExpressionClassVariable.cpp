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
#include "dDAGExpressionClassVariable.h"

dInitRtti(dDAGExpressionClassVariable);

dDAGExpressionClassVariable::dDAGExpressionClassVariable (dList<dDAG*>& allNodes, dDAGExpressionNode* const expression)
	:dDAGExpressionNode(allNodes)
	,m_expression(expression)
	,m_variable (expression->FindLeftVariable()) 
{
	dAssert (m_variable);
}


dDAGExpressionClassVariable::~dDAGExpressionClassVariable ()
{
	dAssert (0);
}


dDAGExpressionNode::dDAGEvaluation dDAGExpressionClassVariable::Evalue(dCIL& cil) 
{
	dDAGEvaluation val (m_expression->Evalue(cil));
	
	dAssert (0);

	return val;
}

void dDAGExpressionClassVariable::ConnectParent(dDAG* const parent)
{
	m_parent = parent;
	m_expression->ConnectParent(this);
}


