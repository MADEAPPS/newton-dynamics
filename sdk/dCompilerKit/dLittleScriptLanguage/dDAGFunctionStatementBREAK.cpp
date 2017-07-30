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
#include "dDAGScopeBlockNode.h"
#include "dDAGFunctionStatementFlow.h"
#include "dDAGFunctionStatementBREAK.h"

dInitRtti(dDAGFunctionStatementBREAK);

dDAGFunctionStatementBREAK::dDAGFunctionStatementBREAK(dList<dDAG*>& allNodes)
	:dDAGFunctionStatement(allNodes)
{
}


dDAGFunctionStatementBREAK::~dDAGFunctionStatementBREAK()
{
}

void dDAGFunctionStatementBREAK::ConnectParent(dDAG* const parent)
{
dAssert (0);
	m_parent = parent;
}

void dDAGFunctionStatementBREAK::CompileCIL(dCIL& cil)  
{
dAssert (0);
/*
	for (dDAG* node = m_parent; node && (node->GetTypeId() != dDAGFunctionNode::GetRttiType()); node = node->m_parent) {
		if (node->IsType(dDAGScopeBlockNode::GetRttiType())) {
			dDAGScopeBlockNode* const scope = (dDAGScopeBlockNode*)node;
			for (dList<dString>::dListNode* node = scope->m_allocations.GetLast(); node; node = node->GetPrev()) {
				dCILInstr& allocationStmt = cil.NewStatement()->GetInfo();
				allocationStmt.m_instruction = dCILInstr::m_release;
				allocationStmt.m_arg0.m_label = node->GetInfo();
				DTRACE_INTRUCTION (&allocationStmt);
			}
		}

		if (node->IsType(dDAGFunctionStatementFlow::GetRttiType())) {
			dDAGFunctionStatementFlow* const flowControl = (dDAGFunctionStatementFlow*) node;
			dCILInstr& stmt = cil.NewStatement()->GetInfo();
			stmt.m_instruction = dCILInstr::m_goto;
			stmt.m_arg0.m_label = flowControl->m_currentBreakLabel;
			stmt.m_arg2.m_label = "break";
			DTRACE_INTRUCTION (&stmt);
			return ;
		} 
	}

	// warning break instruction outside of a flow control statement
	dAssert (0);
*/
}
