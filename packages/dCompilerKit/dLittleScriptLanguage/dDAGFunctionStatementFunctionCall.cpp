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
#include "dDAGExpressionNodeFunctionCall.h"
#include "dDAGFunctionStatementFunctionCall.h"

dInitRtti(dDAGFunctionStatementFunctionCall);

dDAGFunctionStatementFunctionCall::dDAGFunctionStatementFunctionCall(dList<dDAG*>& allNodes, dDAGExpressionNodeFunctionCall* const function)
	:dDAGFunctionStatement(allNodes)
	,m_function(function)
{
	_ASSERTE (m_function);
	_ASSERTE (0);
//	if (m_function) {
//		m_function->AddRef();
//	}
}


dDAGFunctionStatementFunctionCall::~dDAGFunctionStatementFunctionCall()
{
	_ASSERTE (0);
//	if (m_function) {
//		m_function->Release();
//	}
}


void dDAGFunctionStatementFunctionCall::ConnectParent(dDAG* const parent)  
{
	m_parent = parent;
	if (m_function) {
		m_function->ConnectParent(this);
	}
}

void dDAGFunctionStatementFunctionCall::CompileCIL(dCIL& cil)  
{
	m_function->CompileCIL(cil);
}