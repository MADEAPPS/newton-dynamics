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
	for (dList<dDAGExpressionNode*>::dListNode* node = m_argumentList.GetLast(); node; node = node->GetPrev()) {
		dDAGExpressionNode* const expNode = node->GetInfo();
		expNode->CompileCIL(cil);

		dTreeAdressStmt::dArg arg1 (LoadLocalVariable(cil, expNode->m_result));
		dTreeAdressStmt& stmt = cil.NewStatement()->GetInfo();
		stmt.m_instruction = dTreeAdressStmt::m_param;
		stmt.m_arg0 = arg1;
		DTRACE_INTRUCTION (&stmt);

		dTreeAdressStmt::dArgType intrisicType = stmt.m_arg0.m_type;
		switch (intrisicType) 
		{
			case dTreeAdressStmt::m_constInt:
			{
				intrisicType = dTreeAdressStmt::m_int;
				break;
			}
			case dTreeAdressStmt::m_constFloat:
			{
				intrisicType = dTreeAdressStmt::m_float;
				break;
			}
			case dTreeAdressStmt::m_int:
			{
				break;
			}

			//case dTreeAdressStmt::m_void:
			//case dTreeAdressStmt::m_bool:
			//case dTreeAdressStmt::m_byte:
			//case dTreeAdressStmt::m_short:
			//case dTreeAdressStmt::m_long:
			//case dTreeAdressStmt::m_float:
			//case dTreeAdressStmt::m_double:
			//case dTreeAdressStmt::m_classPointer:
				default:	
					dAssert (0);
		}

		name += m_prototypeSeparator + dTreeAdressStmt::GetTypeString (intrisicType);
	}

	dDAGFunctionNode* const function = myClass->GetFunction (name);
	dAssert (function);
	if (function->m_isStatic) {
		name = myClass->m_name + m_prototypeSeparator + name;
	}

	dDAGTypeNode* const returnType = function->m_returnType;

	m_result.m_label = cil.NewTemp ();
	m_result.m_type = returnType->m_intrinsicType;
	dTreeAdressStmt& call = cil.NewStatement()->GetInfo();
	call.m_instruction = dTreeAdressStmt::m_call;
	call.m_arg0 = m_result;
	call.m_arg1.m_label = name;
	DTRACE_INTRUCTION (&call);
}
