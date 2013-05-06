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
#include "dDAGClassNode.h"
#include "dDAGFunctionNode.h"
#include "dDAGParameterNode.h"
#include "dDAGScopeBlockNode.h"


dRttiRootClassSupportImplement(dDAG);

dDAG::dDAG(dList<dDAG*>& allNodes)
	:m_name ("")
	,m_result()
	,m_next (NULL)
	,m_parent (NULL)
	,m_myListNode(NULL)
{
	m_myListNode = allNodes.Append(this);
}

dDAG::~dDAG(void)
{
}



dDAGScopeBlockNode* dDAG::GetScope() const
{
	for (const dDAG* node = this; node; node = node->m_parent) {
		if (node->GetTypeId() == dDAGScopeBlockNode::GetRttiType()) {
			return (dDAGScopeBlockNode*) node;
		}
	}
	return NULL;
}

dDAGFunctionNode* dDAG::GetFunction() const
{
	for (const dDAG* node = this; node; node = node->m_parent) {
		if (node->GetTypeId() == dDAGFunctionNode::GetRttiType()) {
			return (dDAGFunctionNode*) node;
		}
	}
	_ASSERTE (0);
	return NULL;
}

dDAGClassNode* dDAG::GetClass() const
{
	for (const dDAG* node = this; node; node = node->m_parent) {
		if (node->GetTypeId() == dDAGClassNode::GetRttiType()) {
			return (dDAGClassNode*) node;
		}
	}
	_ASSERTE (0);
	return NULL;
}

bool dDAG::RenameLocalVariable(dCIL& cil, dString& variable) const
{
	for (dDAGScopeBlockNode* scope = GetScope(); scope; scope = (dDAGScopeBlockNode*)scope->m_parent->GetScope()) {
		char text[256];
		sprintf (text, "%s%d%s", D_SCOPE_PREFIX, scope->m_scopeLayer, variable.GetStr());

		if (scope->m_localVariablesFilter.FindVariable(text)) {
			variable = text;
			return true;
		}
	}

	dDAGFunctionNode* const function = GetFunction();
	dDAGParameterNode* const functionVariable = function->FindArgumentVariable(variable.GetStr());
	if (functionVariable) {
		if (functionVariable->m_result.m_label == "") {
			dTreeAdressStmt& fntArg = cil.NewStatement()->GetInfo();
			fntArg.m_instruction = dTreeAdressStmt::m_paramLoad;
			fntArg.m_arg0.m_label = cil.NewTemp();
			fntArg.m_arg2.m_label = functionVariable->m_name;
			DTRACE_INTRUCTION (&fntArg);
			functionVariable->m_result.m_label = fntArg.m_arg0.m_label;
		} 
		variable = functionVariable->m_result.m_label;
		return true;
	}

	dDAGClassNode* const classNode = function->GetClass();
	if (classNode->FindVariable(variable.GetStr())) {
		_ASSERTE (0);
		return true;
	}

	_ASSERTE (0);
	return false;
}