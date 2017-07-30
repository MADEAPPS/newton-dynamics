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


dDAGFunctionStatement* const dDAGFunctionStatementWHILE::GetPostFixStatement() const
{
	return NULL;
}

void dDAGFunctionStatementWHILE::CompileCIL(dCIL& cil)  
{
	dCILInstrConditional* conditional = NULL;
	if (m_testExpression) {
		m_testExpression->CompileCIL(cil);
		conditional = new dCILInstrConditional (cil, dCILInstrConditional::m_ifnot, m_expression->m_result.m_label, m_expression->m_result.GetType(), "xxx", "xxx");
		conditional->Trace();
	} else {
		dAssert (0);
	}

	dDAGFunctionStatementFlow::CompileCIL(cil);

	if (conditional) {
		dCILInstrLabel* const exitLabel = cil.GetLast()->GetInfo()->GetAsLabel();
		dCILInstrLabel* const entryLabel = conditional->GetNode()->GetNext()->GetInfo()->GetAsLabel();
		dAssert (exitLabel);
		dAssert (entryLabel);
		conditional->SetLabels (exitLabel->GetArg0().m_label, entryLabel->GetArg0().m_label);
		conditional->SetTargets (exitLabel, entryLabel);
		//conditional->Trace();
	}

//cil.Trace();
}