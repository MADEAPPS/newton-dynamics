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
#include "dDAGFunctionStatementFlow.h"
#include "dDAGFunctionStatementCONTINUE.h"

dInitRtti(dDAGFunctionStatementCONTINUE);

dDAGFunctionStatementCONTINUE::dDAGFunctionStatementCONTINUE(dList<dDAG*>& allNodes)
	:dDAGFunctionStatement(allNodes)
{
}


dDAGFunctionStatementCONTINUE::~dDAGFunctionStatementCONTINUE()
{
}

void dDAGFunctionStatementCONTINUE::ConnectParent(dDAG* const parent)
{
	m_parent = parent;
}

void dDAGFunctionStatementCONTINUE::CompileCIL(dCIL& cil)  
{
	dAssert (0);
	/*

	for (dDAG* node = m_parent; node && (node->GetTypeId() != dDAGFunctionNode::GetRttiType()); node = node->m_parent) {
		if (node->IsType(dDAGFunctionStatementFlow::GetRttiType())) {
			dDAGFunctionStatementFlow* const flowControl = (dDAGFunctionStatementFlow*) node;
			dTreeAdressStmt& stmt = cil.NewStatement()->GetInfo();
			stmt.m_instruction = dTreeAdressStmt::m_goto;
			if (flowControl->m_currentContinueLabel == "") {
				flowControl->m_currentContinueLabel = cil.NewLabel();
			} 
			stmt.m_arg0.m_label = flowControl->m_currentContinueLabel;
			stmt.m_arg2.m_label = "continue";
			DTRACE_INTRUCTION (&stmt);
			return ;
		} 
	}

	// warning break instruction outside of a flow control statement
	dAssert (0);
*/
}
