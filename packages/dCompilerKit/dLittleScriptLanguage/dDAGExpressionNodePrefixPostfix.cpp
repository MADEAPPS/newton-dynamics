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
#include "dDAGExpressionNodePrefixPostfix.h"


dInitRtti(dDAGExpressionNodePrefixPostfix);

dDAGExpressionNodePrefixPostfix::dDAGExpressionNodePrefixPostfix(dList<dDAG*>& allNodes, dDAGExpressionNode* const expression, bool isPrefix, bool isIncrement)
	:dDAGExpressionNode(allNodes)
	,m_isPrefix (isPrefix)
	,m_isIncrement(isIncrement)
	,m_expression(expression)
{
}


dDAGExpressionNodePrefixPostfix::~dDAGExpressionNodePrefixPostfix(void)
{
}


void dDAGExpressionNodePrefixPostfix::ConnectParent(dDAG* const parent) 
{
	m_parent = parent;
	m_expression->ConnectParent(this) ;
}

void dDAGExpressionNodePrefixPostfix::CompileCIL(dCIL& cil)
{
	m_expression->CompileCIL(cil);

	if (m_isPrefix) {
		dTreeAdressStmt& stmt = cil.NewStatement()->GetInfo();
		stmt.m_instruction = dTreeAdressStmt::m_assigment;
		stmt.m_operator = m_isIncrement ? dTreeAdressStmt::m_add : dTreeAdressStmt::m_sub;
		stmt.m_arg0 = m_expression->m_result;
		stmt.m_arg1 = m_expression->m_result;
		stmt.m_arg2.m_type = dTreeAdressStmt::m_constInt;
		stmt.m_arg2.m_label = "1";
		DTRACE_INTRUCTION (&stmt);
		m_result = stmt.m_arg0;
	} else {
		dTreeAdressStmt& stmt1 = cil.NewStatement()->GetInfo();
		stmt1.m_instruction = dTreeAdressStmt::m_assigment;
		stmt1.m_arg0.m_label = cil.NewTemp();
		stmt1.m_arg1 = m_expression->m_result;
		DTRACE_INTRUCTION (&stmt1);

		dTreeAdressStmt& stmt = cil.NewStatement()->GetInfo();
		stmt.m_instruction = dTreeAdressStmt::m_assigment;
		stmt.m_operator = m_isIncrement ? dTreeAdressStmt::m_add : dTreeAdressStmt::m_sub;
		stmt.m_arg0 = m_expression->m_result;
		stmt.m_arg1 = m_expression->m_result;
		stmt.m_arg2.m_type = dTreeAdressStmt::m_constInt;
		stmt.m_arg2.m_label = "1";
		DTRACE_INTRUCTION (&stmt);

		m_result = stmt1.m_arg0;
	}
}

