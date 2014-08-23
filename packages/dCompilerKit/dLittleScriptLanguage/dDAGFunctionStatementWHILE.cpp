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
#include "dDAGFunctionStatementWHILE.h"


dInitRtti(dDAGFunctionStatementWHILE);

dDAGFunctionStatementWHILE::dDAGFunctionStatementWHILE(dList<dDAG*>& allNodes, dDAGExpressionNode* const expression, dDAGFunctionStatement* const stmt)
	:dDAGFunctionStatementFlow(allNodes, stmt, expression)
	,m_expression(expression)
{
}


dDAGFunctionStatementWHILE::~dDAGFunctionStatementWHILE()
{
}


void dDAGFunctionStatementWHILE::ConnectParent(dDAG* const parent)
{
	dDAGFunctionStatementFlow::ConnectParent(parent);
	if (m_expression) {
		m_expression->ConnectParent(this);
	}
}


void dDAGFunctionStatementWHILE::CompileCIL(dCIL& cil)  
{
	dCIL::dListNode* startExpressionTestNode = NULL;

	dString startLabel (cil.NewLabel());
//	dString exitLabel (cil.NewLabel());
	if (m_testExpression) {
		m_testExpression->CompileCIL(cil);

		startExpressionTestNode = cil.NewStatement();
		dThreeAdressStmt& stmt = startExpressionTestNode->GetInfo();
		stmt.m_instruction = dThreeAdressStmt::m_if;
		stmt.m_operator = dThreeAdressStmt::m_nothing;
		stmt.m_arg0 = m_testExpression->m_result;
		stmt.m_arg1.m_label = startLabel;
		stmt.m_arg2.m_label = "";
		//stmt.m_arg1 = tmpTest.m_arg0;
		//stmt.m_arg2.m_label = cil.NewLabel();
		DTRACE_INTRUCTION (&stmt);
	}

	dCIL::dListNode* const entryLabelNode = cil.NewStatement();
	if (startExpressionTestNode) {
		startExpressionTestNode->GetInfo().m_trueTargetJump = entryLabelNode;
	}
	dThreeAdressStmt& entryLabel = entryLabelNode->GetInfo();
	entryLabel.m_instruction = dThreeAdressStmt::m_label;
	entryLabel.m_arg0.m_label = startLabel;
	DTRACE_INTRUCTION (&entryLabel);


	dCIL::dListNode* const exitLabelStmtNode = CompileCILLoopBody(cil, entryLabelNode, NULL);

	if (startExpressionTestNode) {
		dThreeAdressStmt& stmt = startExpressionTestNode->GetInfo();
		stmt.m_falseTargetJump = exitLabelStmtNode;
		stmt.m_arg2 = exitLabelStmtNode->GetInfo().m_arg0; 
	}
}