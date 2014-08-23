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
#include "dDAGFunctionStatement.h"
#include "dDAGParameterNode.h"
#include "dDAGScopeBlockNode.h"
#include "dDAGExpressionNodeVariable.h"
#include "dDAGExpressionNodeBinaryOperator.h"

dInitRtti(dDAGScopeBlockNode);

dDAGScopeBlockNode::dDAGScopeBlockNode(dList<dDAG*>& allNodes)
	:dDAGFunctionStatement(allNodes)
	,m_allocations()
	,m_localVariables()
	,m_statementList()
	,m_scopeLayer(0)
{
}

dDAGScopeBlockNode::~dDAGScopeBlockNode()
{
}


void dDAGScopeBlockNode::AddStatement (dDAGFunctionStatement* const statement)
{
	m_statementList.Append(statement);
}

void dDAGScopeBlockNode::AddVariable (const dString& name, dThreeAdressStmt::dArgType type)
{
	dTree<dThreeAdressStmt::dArg, dString>::dTreeNode* const node = m_localVariables.Insert(name);
	dAssert (node);
	dThreeAdressStmt::dArg& arg = node->GetInfo();
	arg.m_type = type;
	arg.m_label = m_scopePrefix + dString (m_scopeLayer) + name, name;
}

dTree<dThreeAdressStmt::dArg, dString>::dTreeNode* dDAGScopeBlockNode::FindVariable(const dString& name) const
{
	return m_localVariables.Find(name);
}

void dDAGScopeBlockNode::ConnectParent(dDAG* const parent)  
{
	m_parent = parent;

	for (dDAG* node = m_parent; node; node = node->m_parent) {
		if (node->GetTypeId() == dDAGScopeBlockNode::GetRttiType()) {
			dDAGScopeBlockNode* const scope = (dDAGScopeBlockNode*) node;
			m_scopeLayer = scope->m_scopeLayer + 1;
			break;
		}
	}

	for (dList<dDAGFunctionStatement*>::dListNode* node = m_statementList.GetFirst(); node; node = node->GetNext()) {
		dDAGFunctionStatement* const stmt = node->GetInfo();
		stmt->ConnectParent(this);
	}
}



void dDAGScopeBlockNode::CompileCIL(dCIL& cil)  
{
	for (dList<dDAGFunctionStatement*>::dListNode* node = m_statementList.GetFirst(); node; node = node->GetNext()) {
		dDAGFunctionStatement* const stmt = node->GetInfo();
		stmt->CompileCIL(cil);
	}

	for (dList<dString>::dListNode* node = m_allocations.GetLast(); node; node = node->GetPrev()) {
		dAssert (0);
		dThreeAdressStmt& allocationStmt = cil.NewStatement()->GetInfo();
		allocationStmt.m_instruction = dThreeAdressStmt::m_release;
		allocationStmt.m_arg0.m_label = node->GetInfo();
		DTRACE_INTRUCTION (&allocationStmt);
	}
}


