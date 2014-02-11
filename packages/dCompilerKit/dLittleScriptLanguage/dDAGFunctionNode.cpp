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
#include "dDAGScopeBlockNode.h"
#include "dDAGFunctionModifier.h"
#include "dDAGFunctionStatementReturn.h"

dInitRtti(dDAGFunctionNode);

dDAGFunctionNode::dDAGFunctionNode(dList<dDAG*>& allNodes, dDAGTypeNode* const type, const char* const name, const char* const visivility)
	:dDAG(allNodes)
	,m_isStatic(false)
	,m_isPublic(true)
	,m_isConstructor(false)
	,m_returnType (type)
	,m_loopLayer(0)
	,m_body(NULL)
	,m_modifier(NULL)
	,m_parameters() 
{
	m_name = name;

	m_isStatic = strstr (visivility, "static") ? true : false;
	m_isPublic = strstr (visivility, "public") ? true : false;

	if (!m_isStatic) {
		dDAGParameterNode* const operatorThis = new dDAGParameterNode (allNodes, "this", "");
		operatorThis->SetType(new dDAGTypeNode (allNodes, "this"));
		AddParameter(operatorThis);
	}

}


dDAGFunctionNode::~dDAGFunctionNode(void)
{
	dAssert (m_returnType);
}



void dDAGFunctionNode::AddParameter(dDAGParameterNode* const parameter)
{
	dAssert (parameter->IsType(dDAGParameterNode::GetRttiType()));
	m_parameters.Append(parameter);
}

void dDAGFunctionNode::SetBody(dDAGScopeBlockNode* const body)
{
	m_body = body;
}

void dDAGFunctionNode::SetModifier(dDAGFunctionModifier* const modifier)
{
	m_modifier = modifier;
	dAssert (0);
//	m_modifier->AddRef();
}


dDAGParameterNode* dDAGFunctionNode::FindArgumentVariable(const char* const name) const
{
	for (dList<dDAGParameterNode*>::dListNode* node = m_parameters.GetFirst(); node; node = node->GetNext()) {
		dDAGParameterNode* const variable = node->GetInfo();
		if (variable->m_name == name) {
			return variable;
		}
	}
	return NULL;
}

void dDAGFunctionNode::ConnectParent(dDAG* const parent)
{
	m_parent = parent;
	m_body->ConnectParent(this);
	m_returnType->ConnectParent(this);

	if (m_modifier) {
		m_modifier->ConnectParent(this);
	}

	for (dList<dDAGParameterNode*>::dListNode* node = m_parameters.GetFirst(); node; node = node->GetNext()) {
		dDAGParameterNode* const variable = node->GetInfo();
		variable->ConnectParent(this);
	}
}

void dDAGFunctionNode::CompileCIL(dCIL& cil)  
{
	dAssert (0);
/*
	dAssert (m_body);
	dDAGClassNode* const myClass = GetClass();

	cil.ResetTemporaries();
	dString returnVariable (cil.NewTemp());

	dCIL::dReturnType returnTypeVal = dCIL::m_intRegister;
	if (m_returnType->m_name == "void") {
		returnTypeVal = dCIL::m_void;
	} else if (m_returnType->m_name == "int") {
		returnTypeVal = dCIL::m_intRegister;
	} else if (m_returnType->m_name == "float") {
		dAssert (0);
	} else {
		//_ASSERTE (0);
		returnTypeVal = dCIL::m_intRegister;
	}

	m_exitLabel = cil.NewLabel();
	dString functionName (myClass->GetFunctionName (m_name.GetStr(), m_parameters));

	dCIL::dListNode* const functionNode = cil.NewStatement();
	dTreeAdressStmt& function = functionNode->GetInfo();
	function.m_instruction = dTreeAdressStmt::m_function;
	function.m_arg0.m_label = functionName;
	DTRACE_INTRUCTION (&function);

	dCIL::dListNode* const enterNode = cil.NewStatement();
	dTreeAdressStmt& enter = enterNode->GetInfo();
	enter.m_instruction = dTreeAdressStmt::m_enter;
	enter.m_extraInformation = 0;
	DTRACE_INTRUCTION (&enter);

	// emit the function arguments
	for (dList<dDAGParameterNode*>::dListNode* argNode = m_parameters.GetFirst(); argNode; argNode = argNode->GetNext()) {
		dDAGParameterNode* const arg = argNode->GetInfo();
		dTreeAdressStmt& fntArg = cil.NewStatement()->GetInfo();
		fntArg.m_instruction = dTreeAdressStmt::m_argument;
		fntArg.m_arg0.m_label = arg->m_name;
		fntArg.m_arg1.m_label = arg->m_name;
		arg->m_result.m_label = cil.NewTemp();
		fntArg.m_arg1 = arg->m_result;
		DTRACE_INTRUCTION (&fntArg);
	}

	if (!m_isStatic) {
		dList<dDAGParameterNode*>::dListNode* const argNode = m_parameters.GetFirst();
		dDAGParameterNode* const arg = argNode->GetInfo();
		m_opertatorThis = arg->m_result.m_label;
	}

	// load arguments to local variables
	for (dList<dDAGParameterNode*>::dListNode* argNode = m_parameters.GetFirst(); argNode; argNode = argNode->GetNext()) {
		dDAGParameterNode* const arg = argNode->GetInfo();
		dTreeAdressStmt& fntArg = cil.NewStatement()->GetInfo();
		fntArg.m_instruction = dTreeAdressStmt::m_loadBase;
		fntArg.m_arg0 = arg->m_result;
		fntArg.m_arg2.m_label = arg->m_name;
		DTRACE_INTRUCTION (&fntArg);
	}

	if (returnTypeVal == dCIL::m_intRegister) {
		dTreeAdressStmt& returnVariable = cil.NewStatement()->GetInfo();
		returnVariable.m_instruction = dTreeAdressStmt::m_assigment;
		returnVariable.m_arg0.m_label = GetReturnVariableName();
		returnVariable.m_arg1.m_type = dTreeAdressStmt::m_intConst;
		returnVariable.m_arg1.m_label = "0";
		DTRACE_INTRUCTION (&returnVariable);
	} else if (returnTypeVal == dCIL::m_floatRegister) {
		dAssert (0);
	}

	m_body->CompileCIL(cil);


	bool returnStmt = false;
	for (dCIL::dListNode* node = functionNode; node; node = node->GetNext()) {
		dTreeAdressStmt& stmt = node->GetInfo();
		if ((stmt.m_instruction == dTreeAdressStmt::m_goto) && (stmt.m_arg0.m_label == m_exitLabel))  {
			returnStmt = true;
			break;
		}
	}

	if (returnStmt) {
		dCIL::dListNode* const retLabelNode = cil.NewStatement();
		dTreeAdressStmt& retLabel = retLabelNode->GetInfo();
		retLabel.m_instruction = dTreeAdressStmt::m_label;
		retLabel.m_arg0.m_label = m_exitLabel;
		DTRACE_INTRUCTION (&retLabel);

		for (dCIL::dListNode* node = functionNode; node; node = node->GetNext()) {
			dTreeAdressStmt& stmt = node->GetInfo();
			if ((stmt.m_instruction == dTreeAdressStmt::m_goto) && (stmt.m_arg0.m_label == m_exitLabel))  {
				stmt.m_jmpTarget = retLabelNode;
			}
		}
	}

	dCIL::dListNode* const exitNode = cil.NewStatement();
	dTreeAdressStmt& exit = exitNode->GetInfo();
	exit.m_instruction = dTreeAdressStmt::m_leave;
	exit.m_extraInformation = 0;
	DTRACE_INTRUCTION (&exit);

	dCIL::dListNode* const retNode = cil.NewStatement();
	dTreeAdressStmt& ret = retNode->GetInfo();
	ret.m_instruction = dTreeAdressStmt::m_ret;
	ret.m_extraInformation = m_parameters.GetCount() * 4;
	DTRACE_INTRUCTION (&ret);
*/
}
