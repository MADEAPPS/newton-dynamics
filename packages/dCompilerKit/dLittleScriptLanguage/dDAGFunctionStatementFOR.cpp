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
#include "dDAGFunctionStatementFOR.h"


dInitRtti(dDAGFunctionStatementFOR);

dDAGFunctionStatementFOR::dDAGFunctionStatementFOR(dList<dDAG*>& allNodes, dDAGFunctionStatement* const beginStmt, dDAGExpressionNode* const expression, dDAGFunctionStatement* const endStmt, dDAGFunctionStatement* const stmt)
	:dDAGFunctionStatementFlow(allNodes, stmt, expression)
	,m_initialStmt(beginStmt)
	,m_endStmt(endStmt)
{
}


dDAGFunctionStatementFOR::~dDAGFunctionStatementFOR()
{
}


void dDAGFunctionStatementFOR::ConnectParent(dDAG* const parent)  
{
	dDAGFunctionStatementFlow::ConnectParent(parent);

	if (m_initialStmt) {
		m_initialStmt->ConnectParent(this);
	}
	if (m_endStmt) {
		m_endStmt->ConnectParent(this);
	}
}

void dDAGFunctionStatementFOR::CompileCIL(dCIL& cil)  
{
	if (m_initialStmt) {
		m_initialStmt->CompileCIL(cil);
	}

	dCIL::dListNode* startExpressionTestNode = NULL;
	if (m_testExpression) {
		m_testExpression->CompileCIL(cil);

		dTreeAdressStmt& tmpTest = cil.NewStatement()->GetInfo();
		tmpTest.m_instruction = dTreeAdressStmt::m_assigment;
		tmpTest.m_operator = dTreeAdressStmt::m_nothing;
		tmpTest.m_arg0.m_label = cil.NewTemp();
		tmpTest.m_arg1.m_type = dTreeAdressStmt::m_intConst;
		tmpTest.m_arg1.m_label = "0"; 
		DTRACE_INTRUCTION (&tmpTest);

		startExpressionTestNode = cil.NewStatement();
		dTreeAdressStmt& stmt = startExpressionTestNode->GetInfo();
		stmt.m_instruction = dTreeAdressStmt::m_if;
		stmt.m_operator = dTreeAdressStmt::m_identical;
		stmt.m_arg0 = m_testExpression->m_result;
		stmt.m_arg1 = tmpTest.m_arg0;
		stmt.m_arg2.m_label = "loopExit"; 
		DTRACE_INTRUCTION (&stmt);
	}

	dCIL::dListNode* const exitLabelStmtNode = CompileCILLoopBody(cil, m_endStmt);

	if (startExpressionTestNode) {
		dTreeAdressStmt& stmt = startExpressionTestNode->GetInfo();
		stmt.m_jmpTarget = exitLabelStmtNode;
		stmt.m_arg2.m_label = exitLabelStmtNode->GetInfo().m_arg0.m_label; 
	}
}