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
	dCILInstrConditional* const conditional = new dCILInstrConditional (cil, dCILInstrConditional::m_ifnot, m_expression->m_result.m_label, m_expression->m_result.GetType(), label1, label0);
	conditional->Trace();

	dCILInstrLabel* const target0 = new dCILInstrLabel (cil, label0);
	target0->Trace();

	if (!m_elseStmt) {
		m_thenStmt->CompileCIL(cil);
	
		dCILInstrGoto* const branchTarget1 = new dCILInstrGoto (cil, label1);
		branchTarget1->Trace();

		dCILInstrLabel* const target1 = new dCILInstrLabel (cil, label1);
		target1->Trace();

		branchTarget1->SetTarget (target1);
		conditional->SetTargets (target1, target0);
	} else {
		m_elseStmt->CompileCIL(cil);

		dString label2 (cil.NewLabel());
		dCILInstrGoto* const branchTarget2 = new dCILInstrGoto(cil, label2);
		branchTarget2->Trace();

		dCILInstrLabel* const target1 = new dCILInstrLabel (cil, label1);
		target1->Trace();
		conditional->SetTargets(target1, target0);

		m_thenStmt->CompileCIL(cil);

		dCILInstrGoto* const branchTarget3 = new dCILInstrGoto(cil, label2);
		branchTarget3->Trace();

		dCILInstrLabel* const target3 = new dCILInstrLabel(cil, label2);
		target3->Trace();

		branchTarget2->SetTarget(target3);
		branchTarget3->SetTarget(target3);
	}

//cil.Trace();
}