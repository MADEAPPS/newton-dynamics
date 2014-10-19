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
	//,m_currentBreakLabel()
	//,m_currentContinueLabel()
	//,m_backPatchStart (NULL)
	//,m_continueTarget (NULL)
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




/*
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
			dCILInstrGoto* const stmt = node->GetInfo()->GetAsGoto();
			//if ((stmt.m_instruction == dCILInstr::m_goto) && (stmt.m_arg2.m_label == "break")) {
			if (stmt && (stmt->GetArg0().m_label == "break")) {
				dAssert (0);
#if 0
				dCIL::dListNode* const target = cil.NewStatement();
				dCILInstr& jmpTarget = target->GetInfo();
				jmpTarget.m_instruction = dCILInstr::m_label;
				jmpTarget.m_arg0.m_label = m_currentBreakLabel;
				DTRACE_INTRUCTION (&jmpTarget);
				for (; node; node = node->GetNext()) {
					dCILInstr& stmt = node->GetInfo();
					if ((stmt.m_instruction == dCILInstr::m_goto) && (stmt.m_arg2.m_label == "break")){
						stmt.m_jmpTarget = target;
						stmt.m_arg2.m_label = "";
					}
				}
#endif
			}
		}
	} while (node);


	for (dCIL::dListNode* node = m_backPatchStart; node; node = node->GetNext()) {
		//dCILInstr& stmt = node->GetInfo();
		dCILInstrGoto* const stmt = node->GetInfo()->GetAsGoto();
		//if ((stmt.m_instruction == dCILInstr::m_goto) && (stmt.m_arg2.m_label == "continue")) {
		if (stmt && (stmt->GetArg0().m_label == "continue")) {
			dAssert (0);
#if 0
			stmt.m_jmpTarget = m_continueTarget;
			stmt.m_arg2.m_label = "";
#endif
		}
	}
	m_continueTarget = NULL;
}


void dDAGFunctionStatementFlow::OpenPreHeaderBlock(dCIL& cil)
{
	dDAGFunctionNode* const function = GetFunction();
	function->m_loopLayer ++ ;
	int layer = function->m_loopLayer;

	dCILInstrNop* const marker = new dCILInstrNop;
	marker->m_comment = m_loopHeadMetaData + dString (layer);
//	dCILInstr& stmt = cil.NewStatement()->GetInfo();
//	stmt.m_instruction = dCILInstr::m_nop;
//	stmt.m_arg0.m_label = m_loopHeadMetaData;
//	stmt.m_arg1.m_label = dString (layer);
//	DTRACE_INTRUCTION (&stmt);
	marker->Trace();
}

void dDAGFunctionStatementFlow::ClosePreHeaderBlock(dCIL& cil)
{
	dDAGFunctionNode* const function = GetFunction();
	int layer = function->m_loopLayer;

	//dCILInstr& stmt0 = cil.NewStatement()->GetInfo();
	//stmt0.m_instruction = dCILInstr::m_nop;
	//DTRACE_INTRUCTION (&stmt0);

	//dCILInstr& stmt = cil.NewStatement()->GetInfo();
	//stmt.m_instruction = dCILInstr::m_nop;
	//stmt.m_arg0.m_label = m_loopTailMetaData;
	//stmt.m_arg1.m_label = dString (layer);
	//DTRACE_INTRUCTION (&stmt);
	 
	dCILInstrNop* const marker = new dCILInstrNop;
	marker->m_comment = m_loopTailMetaData + dString(layer);
	function->m_loopLayer --;
}
*/

/*
dCIL::dListNode* dDAGFunctionStatementFlow::CompileCILLoopBody(dCIL& cil, dCILInstrConditional* const preConditinalTest, dCILInstrLabel* const loopStart, dDAGFunctionStatement* const posFixStmt)
{
dAssert (0);
return 0;

	//OpenPreHeaderBlock(cil);

	dAssert (entryLabelNode && (entryLabelNode->GetInfo()->GetAsLabel()));

	//dDAGFunctionStatementFlow::CompileCIL(cil);
	if (m_loopBodyStmt) {
		m_loopBodyStmt->CompileCIL(cil);
	}


//	if (m_currentContinueLabel != "") {
//		dAssert(0);
//		return NULL;
//		m_continueTarget = cil.NewStatement();
//		dCILInstr& continueTargeStmt = m_continueTarget->GetInfo();
//		continueTargeStmt.m_instruction = dCILInstr::m_label;
//		continueTargeStmt.m_arg0.m_label = m_currentContinueLabel;
//		DTRACE_INTRUCTION (&continueTargeStmt);
//	}


	if (posFixStmt) {
		dAssert (0);
		posFixStmt->CompileCIL(cil);
	}

	dString exitLabel (cil.NewLabel());
	dCILInstrConditional* conditional = NULL;
	if (m_testExpression) {
		m_testExpression->CompileCIL(cil);

		dCILInstrLabel* const entryLabel = entryLabelNode->GetInfo()->GetAsLabel();
		dAssert (entryLabel);
		
		conditional = new dCILInstrConditional (cil, dCILInstrConditional::m_if, m_testExpression->m_result.m_label, m_testExpression->m_result.GetType(), entryLabel->GetArg0().m_label, exitLabel);
		conditional->Trace();

	} else {
		dAssert (0);
#if 0
		dCILInstr& stmt = cil.NewStatement()->GetInfo();
		stmt.m_instruction = dCILInstr::m_goto;
		stmt.m_arg0.m_label = loopHeaderLabel; 
		stmt.m_jmpTarget = loopStartNode;
		DTRACE_INTRUCTION (&stmt);
#endif
	}

//	BackPatch (cil);
//	ClosePreHeaderBlock(cil);

	dCILInstrLabel* const exitLabelStmt = new dCILInstrLabel (cil, exitLabel);
	exitLabelStmt->Trace();
	if (conditional) {
		conditional->SetTargets (entryLabelNode->GetInfo()->GetAsLabel(), exitLabelStmt);
	}
	return exitLabelStmt->GetNode();
}
*/


void dDAGFunctionStatementFlow::CompileCIL(dCIL& cil)
{
	//	m_backPatchStart = cil.GetLast();
	//	m_currentBreakLabel = cil.NewLabel();
	//	m_currentContinueLabel = "";
	
	dString loopBeginLabel (cil.NewLabel());
	dCILInstrLabel* const entryLabel = new dCILInstrLabel(cil, loopBeginLabel);
	entryLabel->Trace();

	if (m_loopBodyStmt) {
		m_loopBodyStmt->CompileCIL(cil);
	}

	dDAGFunctionStatement* const posFixStmt = GetPostFixStatement();
	if (posFixStmt) {
		dAssert(0);
		posFixStmt->CompileCIL(cil);
	}

	if (m_testExpression) {
		m_testExpression->CompileCIL(cil);

		dCILInstrConditional* const conditional = new dCILInstrConditional(cil, dCILInstrConditional::m_if, m_testExpression->m_result.m_label, m_testExpression->m_result.GetType(), entryLabel->GetArg0().m_label, cil.NewLabel());
		conditional->Trace();

		dCILInstrLabel* const exitLabel = new dCILInstrLabel(cil, conditional->GetArg2().m_label);
		exitLabel->Trace();
		conditional->SetTargets (entryLabel, exitLabel);

	} else {
		dAssert (0);
	}

}
