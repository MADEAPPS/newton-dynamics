/* Copyright (c) <2003-2016> <Newton Game Dynamics>
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
#include "dDAGFunctionStatementCase.h"

dInitRtti(dDAGFunctionStatementCase);

dDAGFunctionStatementCase::dDAGFunctionStatementCase(dList<dDAG*>& allNodes, const char* const nameID, dDAGFunctionStatement* const childStatement)
	:dDAGFunctionStatement(allNodes)
	,m_nameId(nameID)
	,m_statementList()
{
	for (dDAGFunctionStatement* node = childStatement; node; node = (dDAGFunctionStatement*) node->m_next) {
		m_statementList.Append(node);
		dAssert (0);
//		node->AddRef();
	}
}

dDAGFunctionStatementCase::~dDAGFunctionStatementCase()
{
	for (dList<dDAGFunctionStatement*>::dListNode* node = m_statementList.GetFirst(); node; node = node->GetNext()) {
		dAssert (0);
//		dDAGFunctionStatement* const statement = node->GetInfo();
//		statement->Release();
	}
}


void dDAGFunctionStatementCase::ConnectParent(dDAG* const parent)
{
dAssert (0);
	m_parent = parent;
	for (dList<dDAGFunctionStatement*>::dListNode* node = m_statementList.GetFirst(); node; node = node->GetNext()) {
		dDAGFunctionStatement* const statement = node->GetInfo();
		statement->ConnectParent(this);
	}
}


void dDAGFunctionStatementCase::CompileCIL(dCIL& cil)
{
	for (dList<dDAGFunctionStatement*>::dListNode* node = m_statementList.GetFirst(); node; node = node->GetNext()) {
		dDAGFunctionStatement* const statement = node->GetInfo();
		statement->CompileCIL(cil);
	}
}
