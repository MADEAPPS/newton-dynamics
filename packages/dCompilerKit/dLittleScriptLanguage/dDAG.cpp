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


dString dDAG::m_scopePrefix ("scope");
dString dDAG::m_prototypeSeparator ("::");


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

dTree<dCILInstr::dArg, dString>::dTreeNode* dDAG::FindLocalVariable(const dString& name) const
{
	for (const dDAGScopeBlockNode* scope = GetScope(); scope; scope = scope->m_parent->GetScope()) {
		dTree<dCILInstr::dArg, dString>::dTreeNode* const variableNode = scope->m_localVariables.Find(name);	
		if (variableNode) {
			return variableNode;
		}
	}
	return NULL;
}


dCILInstr::dArg dDAG::LoadLocalVariable (dCIL& cil, const dCILInstr::dArg& arg) const
{
	if (arg.m_label.Find(m_scopePrefix) == 0) {
/*
		dThreeAdressStmt& loadVar = cil.NewStatement()->GetInfo();
		loadVar.m_instruction = dThreeAdressStmt::m_loadBase;
		loadVar.m_arg1 = arg;
		loadVar.m_arg0.m_label = cil.NewTemp();
		loadVar.m_arg0.SetType (loadVar.m_arg1);
		DTRACE_INTRUCTION (&loadVar);
		return loadVar.m_arg0;
*/
		dCILInstrLoad* const load = new dCILInstrLoad (cil, arg.m_label, arg);
		DTRACE_INTRUCTION (load);
		return load->GetResult();
	}
	return arg;
}


dDAGFunctionNode* dDAG::GetFunction() const
{
	for (const dDAG* node = this; node; node = node->m_parent) {
		if (node->GetTypeId() == dDAGFunctionNode::GetRttiType()) {
			return (dDAGFunctionNode*) node;
		}
	}
	dAssert (0);
	return NULL;
}

dDAGClassNode* dDAG::GetClass() const
{
	for (const dDAG* node = this; node; node = node->m_parent) {
		if (node->GetTypeId() == dDAGClassNode::GetRttiType()) {
			return (dDAGClassNode*) node;
		}
	}
	dAssert (0);
	return NULL;
}

/*
bool dDAG::RenameLocalVariable(dCIL& cil, dString& variable) const
{
	for (dDAGScopeBlockNode* scope = GetScope(); scope; scope = (dDAGScopeBlockNode*)scope->m_parent->GetScope()) {
		char text[256];
		sprintf (text, "%s%d%s", D_SCOPE_PREFIX, scope->m_scopeLayer, variable.GetStr());

		dTree<dThreeAdressStmt::dArg, dString>::dTreeNode* const varNode = scope->m_localVariables.Find(text);
		if (varNode) {
			variable = text;
			return true;
		}
	}

	dDAGFunctionNode* const function = GetFunction();
	dDAGParameterNode* const functionVariable = function->FindArgumentVariable(variable.GetStr());
	if (functionVariable) {
		if (functionVariable->m_result.m_label == "") {
			dAssert(0);
			dThreeAdressStmt& fntArg = cil.NewStatement()->GetInfo();
			fntArg.m_instruction = dThreeAdressStmt::m_loadBase;
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
		dAssert (0);
		return true;
	}

	dAssert (0);
	return false;
}
*/