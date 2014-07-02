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
dAssert (0);
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
dAssert (0);
/*
	if (!m_backPatchStart) {
		m_backPatchStart = cil.GetFirst();
	}
	
	dCIL::dListNode* node = m_backPatchStart;
	do {
		node = node->GetNext();
		if (node) {
			dTreeAdressStmt& stmt = node->GetInfo();
			if ((stmt.m_instruction == dTreeAdressStmt::m_goto) && (stmt.m_arg2.m_label == "break")) {
				dCIL::dListNode* const target = cil.NewStatement();
				dTreeAdressStmt& jmpTarget = target->GetInfo();
				jmpTarget.m_instruction = dTreeAdressStmt::m_label;
				jmpTarget.m_arg0.m_label = m_currentBreakLabel;
				DTRACE_INTRUCTION (&jmpTarget);
				for (; node; node = node->GetNext()) {
					dTreeAdressStmt& stmt = node->GetInfo();
					if ((stmt.m_instruction == dTreeAdressStmt::m_goto) && (stmt.m_arg2.m_label == "break")){
						stmt.m_jmpTarget = target;
						stmt.m_arg2.m_label = "";
					}
				}
			}
		}
	} while (node);


	for (dCIL::dListNode* node = m_backPatchStart; node; node = node->GetNext()) {
		dTreeAdressStmt& stmt = node->GetInfo();
		if ((stmt.m_instruction == dTreeAdressStmt::m_goto) && (stmt.m_arg2.m_label == "continue")) {
			stmt.m_jmpTarget = m_continueTarget;
			stmt.m_arg2.m_label = "";
		}
	}

	m_continueTarget = NULL;
*/
}


void dDAGFunctionStatementFlow::OpenPreHeaderBlock(dCIL& cil)
{
dAssert (0);
/*
	dDAGFunctionNode* const function = GetFunction();
	function->m_loopLayer ++ ;

	int layer = function->m_loopLayer;

	dTreeAdressStmt& stmt = cil.NewStatement()->GetInfo();
	stmt.m_instruction = dTreeAdressStmt::m_nop;
	stmt.m_arg2.m_label = D_LOOP_HEADER_SYMBOL;
	stmt.m_extraInformation = layer;
	DTRACE_INTRUCTION (&stmt);
*/
}

void dDAGFunctionStatementFlow::ClosePreHeaderBlock(dCIL& cil)
{
dAssert (0);
/*
	dDAGFunctionNode* const function = GetFunction();
	int layer = function->m_loopLayer;

	dTreeAdressStmt& stmt0 = cil.NewStatement()->GetInfo();
	stmt0.m_instruction = dTreeAdressStmt::m_nop;
	DTRACE_INTRUCTION (&stmt0);

	dTreeAdressStmt& stmt = cil.NewStatement()->GetInfo();
	stmt.m_instruction = dTreeAdressStmt::m_nop;
	stmt.m_arg2.m_label = D_LOOP_TAIL_SYMBOL;
	stmt.m_extraInformation = layer;
	DTRACE_INTRUCTION (&stmt);

	function->m_loopLayer --;
*/
}


dCIL::dListNode* dDAGFunctionStatementFlow::CompileCILLoopBody(dCIL& cil, dDAGFunctionStatement* const posFixStmt)
{
dAssert (0);
return NULL;
	/*

	OpenPreHeaderBlock(cil);

	dString loopHeaderLabel (cil.NewLabel());
	dCIL::dListNode* const loopStartNode =  cil.NewStatement();
	dTreeAdressStmt& loopStart = loopStartNode->GetInfo();
	loopStart.m_instruction = dTreeAdressStmt::m_label;
	loopStart.m_arg0.m_label = loopHeaderLabel;
	DTRACE_INTRUCTION (&loopStart);

	dDAGFunctionStatementFlow::CompileCIL(cil);
	if (m_loopBodyStmt) {
		m_loopBodyStmt->CompileCIL(cil);
	}

	if (m_currentContinueLabel != "") {
		m_continueTarget = cil.NewStatement();
		dTreeAdressStmt& continueTargeStmt = m_continueTarget->GetInfo();
		continueTargeStmt.m_instruction = dTreeAdressStmt::m_label;
		continueTargeStmt.m_arg0.m_label = m_currentContinueLabel;
		DTRACE_INTRUCTION (&continueTargeStmt);
	}


	if (posFixStmt) {
		posFixStmt->CompileCIL(cil);
	}

	if (m_testExpression) {
		m_testExpression->CompileCIL(cil);

		dTreeAdressStmt& tmpTest = cil.NewStatement()->GetInfo();
		tmpTest.m_instruction = dTreeAdressStmt::m_assigment;
		tmpTest.m_operator = dTreeAdressStmt::m_nothing;
		tmpTest.m_arg0.m_label = cil.NewTemp();
		tmpTest.m_arg1.m_type = dTreeAdressStmt::m_intConst;
		tmpTest.m_arg1.m_label = "0"; 
		DTRACE_INTRUCTION (&tmpTest);

		dCIL::dListNode* const loopExpressionTestNode = cil.NewStatement();
		dTreeAdressStmt& stmt = loopExpressionTestNode->GetInfo();
		stmt.m_instruction = dTreeAdressStmt::m_if;
		stmt.m_operator = dTreeAdressStmt::m_different;
		stmt.m_arg0 = m_testExpression->m_result;
		stmt.m_arg1 = tmpTest.m_arg0;
		stmt.m_arg2.m_label = loopHeaderLabel; 
		stmt.m_jmpTarget = loopStartNode;
		DTRACE_INTRUCTION (&stmt);
	} else {
		dTreeAdressStmt& stmt = cil.NewStatement()->GetInfo();
		stmt.m_instruction = dTreeAdressStmt::m_goto;
		stmt.m_arg0.m_label = loopHeaderLabel; 
		stmt.m_jmpTarget = loopStartNode;
		DTRACE_INTRUCTION (&stmt);
	}


	BackPatch (cil);
	ClosePreHeaderBlock(cil);


	dString exitLabel(cil.NewLabel());
	dCIL::dListNode* const exitLabelStmtNode =  cil.NewStatement();
	dTreeAdressStmt& exitLabelStmt = exitLabelStmtNode->GetInfo();
	exitLabelStmt.m_instruction = dTreeAdressStmt::m_label;
	exitLabelStmt.m_arg0.m_label = exitLabel;
	DTRACE_INTRUCTION (&exitLabelStmt);
	return exitLabelStmtNode;
*/
}