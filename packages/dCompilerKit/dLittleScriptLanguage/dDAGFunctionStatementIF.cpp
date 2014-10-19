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

	dString label0 (cil.NewLabel());
	dString label1 (cil.NewLabel());
	dCILInstrConditional* const branch = new dCILInstrConditional (cil, dCILInstrConditional::m_ifnot, m_expression->m_result.m_label, m_expression->m_result.GetType(), label1, label0);
	branch->Trace();

	dCILInstrLabel* const target0 = new dCILInstrLabel (cil, label0);
	target0->Trace();

	m_thenStmt->CompileCIL(cil);
	if (!m_elseStmt) {
		dCILInstrGoto* const branchTarget1 = new dCILInstrGoto (cil, label1);
		branchTarget1->Trace();

		dCILInstrLabel* const target1 = new dCILInstrLabel (cil, label1);
		target1->Trace();

		branchTarget1->SetTarget (target1);
		branch->SetTargets (target1, target0);
	} else {
		dAssert (0);
/*
		dCILInstr& gotoExitStmt0 = cil.NewStatement()->GetInfo();
		gotoExitStmt0.m_instruction = dCILInstr::m_goto;
		gotoExitStmt0.m_arg0.m_label = cil.NewLabel();
		DTRACE_INTRUCTION (&gotoExitStmt0);

		stmt.m_falseTargetJump = cil.NewStatement();
		dCILInstr& falseLabel = stmt.m_falseTargetJump->GetInfo();
		falseLabel.m_instruction = dCILInstr::m_label;
		falseLabel.m_arg0.m_label = stmt.m_arg2.m_label;
		DTRACE_INTRUCTION (&falseLabel);

		m_elseStmt->CompileCIL(cil);

		dCILInstr& gotoExitStmt1 = cil.NewStatement()->GetInfo();
		gotoExitStmt1.m_instruction = dCILInstr::m_goto;
		gotoExitStmt1.m_arg0.m_label = gotoExitStmt0.m_arg0.m_label;
		DTRACE_INTRUCTION (&gotoExitStmt0);

		dCIL::dListNode* const exitNode = cil.NewStatement();
		gotoExitStmt0.m_trueTargetJump = exitNode;
		gotoExitStmt1.m_trueTargetJump = exitNode;

		dCILInstr& exitLabel = exitNode->GetInfo();
		exitLabel.m_instruction = dCILInstr::m_label;
		exitLabel.m_arg0.m_label = gotoExitStmt0.m_arg0.m_label;
		DTRACE_INTRUCTION (&exitLabel);
*/
	}
}