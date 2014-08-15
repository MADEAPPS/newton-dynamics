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
#include "dDAGFunctionStatementIF.h"
#include "dDAGExpressionNode.h"

dInitRtti(dDAGFunctionStatementIF);

dDAGFunctionStatementIF::dDAGFunctionStatementIF(dList<dDAG*>& allNodes, 
	dDAGExpressionNode* const expression,
	 dDAGFunctionStatement* const thenStmt, 
	 dDAGFunctionStatement* const elseStmt)
	:dDAGFunctionStatement(allNodes)
	,m_expression(expression)
	,m_thenStmt (thenStmt)
	,m_elseStmt (elseStmt)
{
	dAssert (m_expression);
	dAssert (m_thenStmt);
}


dDAGFunctionStatementIF::~dDAGFunctionStatementIF()
{
}


void dDAGFunctionStatementIF::ConnectParent(dDAG* const parent)  
{
	m_parent = parent;
	m_expression->ConnectParent(this);
	m_thenStmt->ConnectParent(this);
	if (m_elseStmt) {
		m_elseStmt->ConnectParent(this);
	}
}

void dDAGFunctionStatementIF::CompileCIL(dCIL& cil)  
{
	m_expression->CompileCIL(cil);

	dTreeAdressStmt& stmt = cil.NewStatement()->GetInfo();
	stmt.m_instruction = dTreeAdressStmt::m_if;
	stmt.m_operator = dTreeAdressStmt::m_nothing;
	stmt.m_arg0 = m_expression->m_result;
	stmt.m_arg1.m_label = cil.NewLabel();
    stmt.m_arg2.m_label = cil.NewLabel();
	DTRACE_INTRUCTION (&stmt);

    stmt.m_trueTargetJump = cil.NewStatement();
	dTreeAdressStmt& trueLabel = stmt.m_trueTargetJump->GetInfo();
	trueLabel.m_instruction = dTreeAdressStmt::m_label;
	trueLabel.m_arg0.m_label = stmt.m_arg1.m_label;
	DTRACE_INTRUCTION (&trueLabel);
	m_thenStmt->CompileCIL(cil);

	if (!m_elseStmt) {
		dTreeAdressStmt& gotoStmt = cil.NewStatement()->GetInfo();
		gotoStmt.m_instruction = dTreeAdressStmt::m_goto;
		gotoStmt.m_arg0.m_label = stmt.m_arg2.m_label;
		DTRACE_INTRUCTION (&gotoStmt);
		
		stmt.m_falseTargetJump = cil.NewStatement();
		gotoStmt.m_trueTargetJump = stmt.m_falseTargetJump;
		dTreeAdressStmt& falseLabel = stmt.m_falseTargetJump->GetInfo();
		falseLabel.m_instruction = dTreeAdressStmt::m_label;
		falseLabel.m_arg0.m_label = stmt.m_arg2.m_label;
		DTRACE_INTRUCTION (&falseLabel);
	} else {
		dTreeAdressStmt& gotoExitStmt0 = cil.NewStatement()->GetInfo();
		gotoExitStmt0.m_instruction = dTreeAdressStmt::m_goto;
		gotoExitStmt0.m_arg0.m_label = cil.NewLabel();
		DTRACE_INTRUCTION (&gotoExitStmt0);

		stmt.m_falseTargetJump = cil.NewStatement();
		dTreeAdressStmt& falseLabel = stmt.m_falseTargetJump->GetInfo();
		falseLabel.m_instruction = dTreeAdressStmt::m_label;
		falseLabel.m_arg0.m_label = stmt.m_arg2.m_label;
		DTRACE_INTRUCTION (&falseLabel);

		m_elseStmt->CompileCIL(cil);

		dTreeAdressStmt& gotoExitStmt1 = cil.NewStatement()->GetInfo();
		gotoExitStmt1.m_instruction = dTreeAdressStmt::m_goto;
		gotoExitStmt1.m_arg0.m_label = gotoExitStmt0.m_arg0.m_label;
		DTRACE_INTRUCTION (&gotoExitStmt0);

		dCIL::dListNode* const exitNode = cil.NewStatement();
		gotoExitStmt0.m_trueTargetJump = exitNode;
		gotoExitStmt1.m_trueTargetJump = exitNode;

		dTreeAdressStmt& exitLabel = exitNode->GetInfo();
		exitLabel.m_instruction = dTreeAdressStmt::m_label;
		exitLabel.m_arg0.m_label = gotoExitStmt0.m_arg0.m_label;
		DTRACE_INTRUCTION (&exitLabel);
	}
}