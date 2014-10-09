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
#include "dDAGTypeNode.h"
#include "dDAGFunctionNode.h"
#include "dDAGScopeBlockNode.h"
#include "dDAGFunctionStatementReturn.h"

dInitRtti(dDAGFunctionStatementReturn);

dDAGFunctionStatementReturn::dDAGFunctionStatementReturn(dList<dDAG*>& allNodes, dDAGExpressionNode* const expression)
	:dDAGFunctionStatement(allNodes)
	,m_expression(expression)
{
}


dDAGFunctionStatementReturn::~dDAGFunctionStatementReturn()
{
}


void dDAGFunctionStatementReturn::ConnectParent(dDAG* const parent)
{
	m_parent = parent;
	if (m_expression) {
		m_expression->ConnectParent(this);
	}
}

void dDAGFunctionStatementReturn::CompileCIL(dCIL& cil)
{
	dCILInstr::dArg retValue;
	if (m_expression) {
		m_expression->CompileCIL(cil);
		retValue = m_expression->m_result;
	}
	

	for (dDAG* node = m_parent; node && (node->GetTypeId() != dDAGFunctionNode::GetRttiType()); node = node->m_parent) {
		if (node->IsType(dDAGScopeBlockNode::GetRttiType())) {
			dDAGScopeBlockNode* const scope = (dDAGScopeBlockNode*)node;
			for (dList<dString>::dListNode* node = scope->m_allocations.GetLast(); node; node = node->GetPrev()) {
				dAssert(0);
				//dThreeAdressStmt& allocationStmt = cil.NewStatement()->GetInfo();
				//allocationStmt.m_instruction = dThreeAdressStmt::m_release;
				//allocationStmt.m_arg0.m_label = node->GetInfo();
				//DTRACE_INTRUCTION (&allocationStmt);
			}
		}
	}


	dCILInstr::dArg arg1 (LoadLocalVariable(cil, retValue));
	dCILInstrReturn* const instr = new dCILInstrReturn (cil, arg1.m_label, arg1.GetType());
	instr->Trace();
}