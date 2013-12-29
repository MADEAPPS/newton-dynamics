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
		_ASSERTE (param->IsType(dDAGExpressionNode::GetRttiType()));
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
//	dDAGFunctionNode* const function = GetFunction();

//	int argumentsCount = m_argumentList.GetCount();
	for (dList<dDAGExpressionNode*>::dListNode* node = m_argumentList.GetLast(); node; node = node->GetPrev()) {
		dDAGExpressionNode* const expNode = node->GetInfo();
		expNode->CompileCIL(cil);
		
//		argumentsCount --;		
//		if (argumentsCount < function->m_argumentsCount) {
//			dTreeAdressStmt& stmt = cil.NewStatement()->GetInfo();
//			stmt.m_instruction = dTreeAdressStmt::m_push;
//			stmt.m_arg0.m_label = GetTemporaryVariableName(argumentsCount + 1);
//			DTRACE_INTRUCTION (&stmt);
//
//			dTreeAdressStmt& stmt1 = cil.NewStatement()->GetInfo();
//			stmt1.m_instruction = dTreeAdressStmt::m_assigment;
//			stmt1.m_arg0.m_label = stmt.m_arg0.m_label;
//			stmt1.m_arg1 = expNode->m_result;
//			DTRACE_INTRUCTION (&stmt1);
//
//		} else {
			dTreeAdressStmt& stmt = cil.NewStatement()->GetInfo();
			stmt.m_instruction = dTreeAdressStmt::m_push;
			stmt.m_arg0 = expNode->m_result;
			DTRACE_INTRUCTION (&stmt);
//		}
	}

	dCIL::dReturnType returnTypeVal = dCIL::m_intRegister;
	dDAGTypeNode* const returnType = myClass->GetFunctionReturnType(m_name.GetStr(), m_argumentList);
	if (returnType->m_name == "void") {
		returnTypeVal = dCIL::m_void;
	} else if (returnType->m_name == "int") {
		returnTypeVal = dCIL::m_intRegister;
	} else {
		_ASSERTE (0);
	}

	m_result.m_label = cil.NewTemp ();		
	dTreeAdressStmt& call = cil.NewStatement()->GetInfo();
	call.m_instruction = dTreeAdressStmt::m_call;
	call.m_arg0.m_label = myClass->GetFunctionName (m_name.GetStr(), m_argumentList);
	call.m_extraInformation = returnTypeVal;
	DTRACE_INTRUCTION (&call);

//	argumentsCount = 0;
//	for (dList<dDAGExpressionNode*>::dListNode* node = m_argumentList.GetFirst(); node; node = node->GetNext()) {
//		if (argumentsCount < function->m_argumentsCount) {
//			dTreeAdressStmt& stmt = cil.NewStatement()->GetInfo();
//			stmt.m_instruction = dTreeAdressStmt::m_pop;
//			stmt.m_arg0.m_label = GetTemporaryVariableName(argumentsCount + 1);
//			DTRACE_INTRUCTION (&stmt);
//		}
//		argumentsCount ++;
//	}

	
	if (returnTypeVal == dCIL::m_intRegister) {
		dTreeAdressStmt& result = cil.NewStatement()->GetInfo();
		result.m_instruction = dTreeAdressStmt::m_assigment;
		result.m_arg0 = m_result;

		result.m_arg1.m_label = GetReturnVariableName();
		DTRACE_INTRUCTION (&result);
	} else if (returnTypeVal == dCIL::m_floatRegister) {
		_ASSERT (0);
	}
}
