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
	dThreeAdressStmt::dArg arg1 (LoadLocalVariable(cil, m_expression->m_result));

	dThreeAdressStmt& stmt = cil.NewStatement()->GetInfo();
	stmt.m_instruction = dThreeAdressStmt::m_assigment;
	stmt.m_operator = m_isIncrement ? dThreeAdressStmt::m_add : dThreeAdressStmt::m_sub;
	stmt.m_arg0.m_label = cil.NewTemp();
	stmt.m_arg0.SetType (arg1.GetType());
	stmt.m_arg1 = arg1;
	stmt.m_arg2.SetType (dThreeAdressStmt::m_constInt, false);
	stmt.m_arg2.m_label = "1";
	DTRACE_INTRUCTION (&stmt);
	m_result = stmt.m_arg0;

	dThreeAdressStmt& stmt1 = cil.NewStatement()->GetInfo();
	stmt1.m_instruction = dThreeAdressStmt::m_storeBase;
	stmt1.m_arg0 = m_expression->m_result;
	stmt1.m_arg1 = stmt.m_arg0;
	DTRACE_INTRUCTION (&stmt1);
}

