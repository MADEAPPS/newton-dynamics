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
#include "dDAGFunctionNode.h"
#include "dDAGExpressionNode.h"
#include "dDAGFunctionStatementFlow.h"


dInitRtti(dDAGFunctionStatementFlow);

dDAGFunctionStatementFlow::dDAGFunctionStatementFlow(dList<dDAG*>& allNodes, dDAGFunctionStatement* const loopBodyStmt, dDAGExpressionNode* const testExpression)
	:dDAGFunctionStatement(allNodes)
	,m_currentBreakLabel()
	,m_currentContinueLabel()
	,m_backPatchStart (NULL)
	,m_continueTarget (NULL)
	,m_testExpression (testExpression)
	,m_loopBodyStmt (loopBodyStmt)
{
}


dDAGFunctionStatementFlow::~dDAGFunctionStatementFlow()
{
}


void dDAGFunctionStatementFlow::ConnectParent(dDAG* const parent)
{
	m_parent = parent;
	if (m_loopBodyStmt) {
		m_loopBodyStmt->ConnectParent(this);
	}

	if (m_testExpression) {
		m_testExpression->ConnectParent(this);
	}
}



void dDAGFunctionStatementFlow::CompileCIL(dCIL& cil)  
{
	m_backPatchStart = cil.GetLast();
	m_currentBreakLabel = cil.NewLabel();
	//m_currentContinueLabel = cil.NewLabel();
	m_currentContinueLabel = "";
}

void dDAGFunctionStatementFlow::BackPatch (dCIL& cil)
{
	if (!m_backPatchStart) {
		dAssert (0);
		m_backPatchStart = cil.GetFirst();
	}
	
	dCIL::dListNode* node = m_backPatchStart;
	do {
		node = node->GetNext();
		if (node) {
			dThreeAdressStmt& stmt = node->GetInfo();
			if ((stmt.m_instruction == dThreeAdressStmt::m_goto) && (stmt.m_arg2.m_label == "break")) {
				dAssert (0);
/*
				dCIL::dListNode* const target = cil.NewStatement();
				dThreeAdressStmt& jmpTarget = target->GetInfo();
				jmpTarget.m_instruction = dThreeAdressStmt::m_label;
				jmpTarget.m_arg0.m_label = m_currentBreakLabel;
				DTRACE_INTRUCTION (&jmpTarget);
				for (; node; node = node->GetNext()) {
					dThreeAdressStmt& stmt = node->GetInfo();
					if ((stmt.m_instruction == dThreeAdressStmt::m_goto) && (stmt.m_arg2.m_label == "break")){
						stmt.m_jmpTarget = target;
						stmt.m_arg2.m_label = "";
					}
				}
*/
			}
		}
	} while (node);


	for (dCIL::dListNode* node = m_backPatchStart; node; node = node->GetNext()) {
		dThreeAdressStmt& stmt = node->GetInfo();
		if ((stmt.m_instruction == dThreeAdressStmt::m_goto) && (stmt.m_arg2.m_label == "continue")) {
			dAssert (0);
/*
			stmt.m_jmpTarget = m_continueTarget;
			stmt.m_arg2.m_label = "";
*/
		}
	}
	m_continueTarget = NULL;
}


void dDAGFunctionStatementFlow::OpenPreHeaderBlock(dCIL& cil)
{
	dDAGFunctionNode* const function = GetFunction();
	function->m_loopLayer ++ ;
	int layer = function->m_loopLayer;

	dThreeAdressStmt& stmt = cil.NewStatement()->GetInfo();
	stmt.m_instruction = dThreeAdressStmt::m_nop;
	stmt.m_arg0.m_label = m_loopHeadMetaData;
	stmt.m_arg1.m_label = dString (layer);
	//stmt.m_extraInformation = layer;
	DTRACE_INTRUCTION (&stmt);
}

void dDAGFunctionStatementFlow::ClosePreHeaderBlock(dCIL& cil)
{
	dDAGFunctionNode* const function = GetFunction();
	int layer = function->m_loopLayer;

	dThreeAdressStmt& stmt0 = cil.NewStatement()->GetInfo();
	stmt0.m_instruction = dThreeAdressStmt::m_nop;
	DTRACE_INTRUCTION (&stmt0);

	dThreeAdressStmt& stmt = cil.NewStatement()->GetInfo();
	stmt.m_instruction = dThreeAdressStmt::m_nop;
	stmt.m_arg0.m_label = m_loopTailMetaData;
	stmt.m_arg1.m_label = dString (layer);
	DTRACE_INTRUCTION (&stmt);
	function->m_loopLayer --;
}


dCIL::dListNode* dDAGFunctionStatementFlow::CompileCILLoopBody(dCIL& cil, dCIL::dListNode* const entryLabelNode, dDAGFunctionStatement* const posFixStmt)
{
	OpenPreHeaderBlock(cil);

	dAssert (entryLabelNode && (entryLabelNode->GetInfo().m_instruction == dThreeAdressStmt::m_label));

	dString exitLabel (cil.NewLabel());

	dDAGFunctionStatementFlow::CompileCIL(cil);
	if (m_loopBodyStmt) {
//		m_loopBodyStmt->CompileCIL(cil);
	}

	if (m_currentContinueLabel != "") {
		m_continueTarget = cil.NewStatement();
		dThreeAdressStmt& continueTargeStmt = m_continueTarget->GetInfo();
		continueTargeStmt.m_instruction = dThreeAdressStmt::m_label;
		continueTargeStmt.m_arg0.m_label = m_currentContinueLabel;
		DTRACE_INTRUCTION (&continueTargeStmt);
	}

	if (posFixStmt) {
		dAssert (0);
		posFixStmt->CompileCIL(cil);
	}

	dCIL::dListNode* expressionTestNode = NULL;
	if (m_testExpression) {
		m_testExpression->CompileCIL(cil);

		dThreeAdressStmt& entryLabel = entryLabelNode->GetInfo();

		expressionTestNode = cil.NewStatement();
		dThreeAdressStmt& stmt = expressionTestNode->GetInfo();
		stmt.m_instruction = dThreeAdressStmt::m_if;
		stmt.m_operator = dThreeAdressStmt::m_nothing;
		stmt.m_arg0 = m_testExpression->m_result;
		stmt.m_arg1 = entryLabel.m_arg0;
		stmt.m_trueTargetJump = entryLabelNode;
		stmt.m_arg2.m_label = exitLabel;
		DTRACE_INTRUCTION (&stmt);

	} else {
		dAssert (0);
/*
		dThreeAdressStmt& stmt = cil.NewStatement()->GetInfo();
		stmt.m_instruction = dThreeAdressStmt::m_goto;
		stmt.m_arg0.m_label = loopHeaderLabel; 
		stmt.m_jmpTarget = loopStartNode;
		DTRACE_INTRUCTION (&stmt);
*/
	}

	BackPatch (cil);
	ClosePreHeaderBlock(cil);

	dCIL::dListNode* const exitLabelStmtNode =  cil.NewStatement();
	if (expressionTestNode) {
		expressionTestNode->GetInfo().m_falseTargetJump = exitLabelStmtNode;
	}

	dThreeAdressStmt& exitLabelStmt = exitLabelStmtNode->GetInfo();
	exitLabelStmt.m_instruction = dThreeAdressStmt::m_label;
	exitLabelStmt.m_arg0.m_label = exitLabel;
	DTRACE_INTRUCTION (&exitLabelStmt);

	return exitLabelStmtNode;
}