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
	dDAGFunctionNode* const function = GetFunction();

	if (m_expression) {
		m_expression->CompileCIL(cil);
		
		dCIL::dReturnType returnTypeVal = dCIL::m_intRegister;
		dDAGTypeNode* const returnType = function->m_returnType;
		if (returnType->m_name == "void") {
			returnTypeVal = dCIL::m_void;
		} else if (returnType->m_name == "int") {
			returnTypeVal = dCIL::m_intRegister;
		} else if (returnType->m_name == "float") {
			dAssert (0);
		} else if (returnType->m_name == "double") {
			dAssert (0);
		} else {
			returnTypeVal = dCIL::m_intRegister;
		}

		if (returnTypeVal == dCIL::m_intRegister) {
			dTreeAdressStmt& stmt = cil.NewStatement()->GetInfo();
			stmt.m_instruction = dTreeAdressStmt::m_assigment;
			stmt.m_arg0.m_label = GetReturnVariableName();

			stmt.m_arg1 = m_expression->m_result;
			DTRACE_INTRUCTION (&stmt);
		} else if (returnTypeVal == dCIL::m_floatRegister) {
			dAssert (0);
		}
	}

	for (dDAG* node = m_parent; node && (node->GetTypeId() != dDAGFunctionNode::GetRttiType()); node = node->m_parent) {
		if (node->IsType(dDAGScopeBlockNode::GetRttiType())) {
			dDAGScopeBlockNode* const scope = (dDAGScopeBlockNode*)node;
			for (dList<dString>::dListNode* node = scope->m_allocations.GetLast(); node; node = node->GetPrev()) {
				dTreeAdressStmt& allocationStmt = cil.NewStatement()->GetInfo();
				allocationStmt.m_instruction = dTreeAdressStmt::m_release;
				allocationStmt.m_arg0.m_label = node->GetInfo();
				DTRACE_INTRUCTION (&allocationStmt);
			}
		}
	}

	dTreeAdressStmt& stmt = cil.NewStatement()->GetInfo();
	stmt.m_instruction = dTreeAdressStmt::m_goto;
	stmt.m_arg0.m_label = function->m_exitLabel;
	DTRACE_INTRUCTION (&stmt);
}