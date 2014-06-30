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
#include "dDAGClassNode.h"
#include "dDAGFunctionNode.h"
#include "dDAGParameterNode.h"
#include "dDAGExpressionClassVariable.h"

dInitRtti(dDAGClassNode);

dDAGClassNode::dDAGClassNode(dList<dDAG*>& allNodes)
	:dDAG(allNodes)
	,m_isFinal(false)
	,m_isPublic(true)
	,m_baseClass (NULL)
	,m_functionList()
	,m_constructors()
	,m_variables()
{
}


dDAGClassNode::~dDAGClassNode(void)
{
}

void dDAGClassNode::FinalizeImplementation (const dString& visibility, const dString& name, dDAGClassNode* const baseClass)
{
//	dString visibilityString (visibility);
	m_isFinal = (visibility.Find("final") >= 0) ? true : false;
	m_isPublic = (visibility.Find("private") >= 0) ? false : true;
	m_name = name;
//	m_baseClass = baseClass;
//	_ASSERTE (!m_baseClass);
}


void dDAGClassNode::AddFunction (dDAGFunctionNode* const function)
{
	m_functionList.Append(function);
}

dDAGFunctionNode* dDAGClassNode::GetCurrentFunction ()
{
	return m_functionList.GetLast()->GetInfo();
}

dString dDAGClassNode::GetFunctionName (const dString& name, dList<dDAGExpressionNode*>& argumentList) const
{
	for (dList<dDAGFunctionNode*>::dListNode* functionNode = m_functionList.GetFirst(); functionNode; functionNode = functionNode->GetNext()) {
		dDAGFunctionNode* const function = functionNode->GetInfo();
		if (function->m_name == name) {
			// for now only static functions, later remember to add function signatures
			return dString (m_name + "::" + name);
		}
	}
	dAssert (0);
	return "xxx";
}

dString dDAGClassNode::GetFunctionName (const dString& name, dList<dDAGParameterNode*>& parameterNodeList) const
{
	for (dList<dDAGFunctionNode*>::dListNode* functionNode = m_functionList.GetFirst(); functionNode; functionNode = functionNode->GetNext()) {
		dDAGFunctionNode* const function = functionNode->GetInfo();
		if (function->m_name == name) {
			// for now only static functions, later remember to add function signatures
			return dString (m_name + "::" + name);
		}
	}

	dAssert (0);
	return "xxxx";
}

dDAGTypeNode* dDAGClassNode::GetFunctionReturnType (const dString& name, dList<dDAGExpressionNode*>& argumentList) const
{
	for (dList<dDAGFunctionNode*>::dListNode* functionNode = m_functionList.GetFirst(); functionNode; functionNode = functionNode->GetNext()) {
		dDAGFunctionNode* const function = functionNode->GetInfo();
		if (function->m_name == name) {
			return function->m_returnType;
		}
	}

	dAssert (0);
	return NULL;
}


void dDAGClassNode::AddVariable (dDAGExpressionClassVariable* const variable)
{
	m_variables.Append(variable);
}

dDAGParameterNode* dDAGClassNode::FindVariable (const dString& name) const
{
	dAssert (0);
/*
	for (dList<dDAGParameterNode*>::dListNode* node = m_variables.GetFirst(); node; node = node->GetNext()) {
		dDAGParameterNode* const variable = node->GetInfo();
		if (variable->m_name == name) {
			return variable;
		}
	}
*/
	return NULL;
}

void dDAGClassNode::ConnectParent(dDAG* const parent)  
{
	m_parent = parent;
	for (dList<dDAGExpressionClassVariable*>::dListNode* node = m_variables.GetFirst(); node; node = node->GetNext()) {
		dDAGExpressionClassVariable* const variable = node->GetInfo();
		variable->ConnectParent(this);
	}

	for (dList<dDAGFunctionNode*>::dListNode* node = m_functionList.GetFirst(); node; node = node->GetNext()) {
		dDAGFunctionNode* const function = node->GetInfo();
		function->ConnectParent(this);
	}
}

void dDAGClassNode::CompileCIL(dCIL& cil)  
{
	for (dList<dDAGExpressionClassVariable*>::dListNode* node = m_variables.GetFirst(); node; node = node->GetNext()) {
		dDAGExpressionClassVariable* const variable = node->GetInfo();
		variable->Evalue(cil);
	}

	for (dList<dDAGFunctionNode*>::dListNode* node = m_functionList.GetFirst(); node; node = node->GetNext()) {
		m_cilCodeList.Append (cil.NewStatement());
		dDAGFunctionNode* const function = node->GetInfo();

		dCIL::dListNode* const functionNode = cil.GetLast();
		function->CompileCIL(cil);

		//cil.Optimize(functionNode->GetNext(), function->m_argumentsCount, returnType);
		//cil.Optimize(functionNode->GetNext(), 0, returnType);
		function->BuildBasicBlocks (cil, functionNode->GetNext());
	}
}


void dDAGClassNode::TranslateToLLVM (dCIL& cil, llvm::Module* const module, llvm::LLVMContext &context)
{
	for (dList<dDAGFunctionNode*>::dListNode* node = m_functionList.GetFirst(); node; node = node->GetNext()) {
		dDAGFunctionNode* const function = node->GetInfo();
		function->TranslateToLLVM (cil, module, context);
	}
}