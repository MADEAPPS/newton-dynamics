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
	dCILInstr::dArg arg1 (LoadLocalVariable(cil, m_expression->m_result));

	dCILInstrMove* move = NULL;
	if (m_isPrefix) {
		dCILInstrIntergerLogical* const stmt = new dCILInstrIntergerLogical (cil, m_isIncrement ? dCILThreeArgInstr::m_add : dCILThreeArgInstr::m_sub, cil.NewTemp(), arg1.GetType(), arg1.m_label, arg1.GetType(), "1", dCILInstr::dArgType (dCILInstr::m_constInt));
		stmt->Trace();

		move = new dCILInstrMove (cil, arg1.m_label, arg1.GetType(), stmt->GetArg0().m_label, stmt->GetArg0().GetType());
		move->Trace();

	} else {
		move = new dCILInstrMove(cil, cil.NewTemp(), arg1.GetType(), arg1.m_label, arg1.GetType());
		move->Trace();

		dCILInstrIntergerLogical* const stmt = new dCILInstrIntergerLogical(cil, m_isIncrement ? dCILThreeArgInstr::m_add : dCILThreeArgInstr::m_sub, arg1.m_label, arg1.GetType(), arg1.m_label, arg1.GetType(), "1", dCILInstr::dArgType(dCILInstr::m_constInt));
		stmt->Trace();
	}

	m_result = move->GetArg0();
}

