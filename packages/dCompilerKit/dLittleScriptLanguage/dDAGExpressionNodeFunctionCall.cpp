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
#include "dCIL.h"
#include "dDAG.h"
#include "dDAGTypeNode.h"
#include "dDAGClassNode.h"
#include "dDAGFunctionNode.h"
#include "dDAGExpressionNodeFunctionCall.h"


dInitRtti(dDAGExpressionNodeFunctionCall);

dDAGExpressionNodeFunctionCall::dDAGExpressionNodeFunctionCall(dList<dDAG*>& allNodes, const char* const identifier, dDAGExpressionNode* const argumentList)
	:dDAGExpressionNode(allNodes)
	,m_argumentList()
{
	m_name = identifier;

	dDAGExpressionNode* next;
	for (dDAGExpressionNode* param = argumentList; param; param = next) {
		next = (dDAGExpressionNode*) param->m_next;
		dAssert (param->IsType(dDAGExpressionNode::GetRttiType()));
		m_argumentList.Append(param);
	}
}


dDAGExpressionNodeFunctionCall::~dDAGExpressionNodeFunctionCall(void)
{
//	for (dList<dDAGExpressionNode*>::dListNode* node = m_argumentList.GetFirst(); node; node = node->GetNext()) {
//		dDAGExpressionNode* const parameter = node->GetInfo();
//		parameter->Release();
//	}
}

void dDAGExpressionNodeFunctionCall::ConnectParent(dDAG* const parent)
{
	m_parent = parent;
	for (dList<dDAGExpressionNode*>::dListNode* node = m_argumentList.GetFirst(); node; node = node->GetNext()) {
		dDAGExpressionNode* const parameter = node->GetInfo();
		parameter->ConnectParent(this);
	}
}


void dDAGExpressionNodeFunctionCall::CompileCIL(dCIL& cil)  
{
	dDAGClassNode* const myClass = GetClass();
	dAssert (myClass);
	//dDAGFunctionNode* const myFunction = myClass->GetCurrentFunction ();

	dString name (m_name);
	dList<dCILInstr::dArg> paramList;
	for (dList<dDAGExpressionNode*>::dListNode* node = m_argumentList.GetLast(); node; node = node->GetPrev()) {
		dDAGExpressionNode* const expNode = node->GetInfo();
		expNode->CompileCIL(cil);
		paramList.Append (LoadLocalVariable(cil, expNode->m_result));
	}


//	for (dList<dThreeAdressStmt::dArg>::dListNode* node = paramList.GetFirst(); node; node = node->GetNext()) {
//		dThreeAdressStmt& stmt = cil.NewStatement()->GetInfo();
//		stmt.m_instruction = dThreeAdressStmt::m_param;
//		stmt.m_arg0 = node->GetInfo();
//		DTRACE_INTRUCTION (&stmt);
//	}

	for (dList<dCILInstr::dArg>::dListNode* node = paramList.GetLast(); node; node = node->GetPrev()) {
		//name += m_prototypeSeparator + expNode->GetTypeName();
		name += m_prototypeSeparator +node->GetInfo().GetTypeName();
	}

	dDAGFunctionNode* const function = myClass->GetFunction (name);
	dAssert (function);
	if (function->m_isStatic) {
		name = myClass->m_name + m_prototypeSeparator + name;
	}

	dDAGTypeNode* const returnType = function->m_returnType;
	m_result.m_label = cil.NewTemp ();
	m_result.SetType (returnType->GetArgType());
	dCILInstrCall* const instr = new dCILInstrCall (cil, m_result.m_label, m_result.GetType(), name, paramList);
	instr->Trace();
}
