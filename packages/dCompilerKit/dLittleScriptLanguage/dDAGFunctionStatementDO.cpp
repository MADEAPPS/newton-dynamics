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
#include "dDAGExpressionNode.h"
#include "dDAGFunctionStatementDO.h"


dInitRtti(dDAGFunctionStatementDO);

dDAGFunctionStatementDO::dDAGFunctionStatementDO(dList<dDAG*>& allNodes, dDAGExpressionNode* const expression, dDAGFunctionStatement* const stmt)
	:dDAGFunctionStatementFlow(allNodes, stmt, expression)
	,m_expression(expression)
{
}


dDAGFunctionStatementDO::~dDAGFunctionStatementDO()
{
}

void dDAGFunctionStatementDO::ConnectParent(dDAG* const parent)
{
	dDAGFunctionStatementFlow::ConnectParent(parent);
	if (m_expression) {
		m_expression->ConnectParent(this);
	}
}


void dDAGFunctionStatementDO::CompileCIL(dCIL& cil)  
{
dAssert (0);
/*
	dCIL::dListNode* const blockTerminatorNode = cil.NewStatement();
	dCILInstr& blockTerminator = blockTerminatorNode->GetInfo();
	blockTerminator.m_instruction = dCILInstr::m_goto;
	blockTerminator.m_arg0.m_label = cil.NewLabel();
	DTRACE_INTRUCTION (&blockTerminator);

	dCIL::dListNode* const entryLabelNode = cil.NewStatement();
	blockTerminator.m_trueTargetJump = entryLabelNode;
	dCILInstr& entryLabel = entryLabelNode->GetInfo();
	entryLabel.m_instruction = dCILInstr::m_label;
	entryLabel.m_arg0 = blockTerminator.m_arg0;
	DTRACE_INTRUCTION (&entryLabel);
	CompileCILLoopBody(cil, entryLabelNode, NULL);
*/
}