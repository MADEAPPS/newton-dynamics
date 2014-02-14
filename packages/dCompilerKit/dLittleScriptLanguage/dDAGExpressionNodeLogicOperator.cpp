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
#include "dDAGExpressionNodeConstant.h"
#include "dDAGExpressionNodeLogicOperator.h"


dInitRtti(dDAGExpressionNodeLogicOperator);

dDAGExpressionNodeLogicOperator::dDAGExpressionNodeLogicOperator(
	dList<dDAG*>& allNodes,
	dLogicOperator binaryOperator, dDAGExpressionNode* const expressionA, dDAGExpressionNode* const expressionB)
	:dDAGExpressionNode(allNodes)
	,m_operator (binaryOperator)
	,m_expressionA (expressionA)
	,m_expressionB (expressionB)
{
//	_ASSERTE (0);
//	m_expressionA->AddRef();
//	m_expressionB->AddRef();
}


dDAGExpressionNodeLogicOperator::~dDAGExpressionNodeLogicOperator(void)
{
	dAssert (0);
//	m_expressionA->Release();
//	m_expressionB->Release();
}

void dDAGExpressionNodeLogicOperator::ConnectParent(dDAG* const parent)  
{
	m_parent = parent;
	m_expressionA->ConnectParent(this);
	m_expressionB->ConnectParent(this);
}

/*
dTreeAdressStmt::dArg dDAGExpressionNodeLogicOperator::SetArgument(dDAGExpressionNode* const expression)
{
	dTreeAdressStmt::dArg argument;

	argument.m_label = expression->m_result;

	if (expression->GetTypeId() == dDAGExpressionNodeConstant::GetRttiType()) {
		dDAGExpressionNodeConstant* const expressionConst = (dDAGExpressionNodeConstant*) expression;
		switch (expressionConst->m_type) 
		{
			case dDAGExpressionNodeConstant::m_intValue:
				argument.m_type = dTreeAdressStmt::m_intConst;
				break;

			default:
				_ASSERTE (0);
		}
	}
	return argument;
}
*/

void dDAGExpressionNodeLogicOperator::CompileCIL(dCIL& cil)  
{
dAssert(0);
/*
	dTreeAdressStmt& stmt = cil.NewStatement()->GetInfo();
	m_result.m_label = cil.NewTemp ();		

	const char* initialBool = "0";
	const char* complementBool = "1";
	dTreeAdressStmt::dOperator logicOperation = dTreeAdressStmt::m_identical;
	if (m_operator == m_or) {
		initialBool = "1";
		complementBool = "0";
		logicOperation = dTreeAdressStmt::m_different;
	}
	
	
	stmt.m_instruction = dTreeAdressStmt::m_assigment;
	stmt.m_arg0 = m_result;
	stmt.m_arg1.m_type = dTreeAdressStmt::m_intConst;
	stmt.m_arg1.m_label = dString (initialBool);
	DTRACE_INTRUCTION (&stmt);

	dString exitLabel (cil.NewLabel()) ;
	m_expressionA->CompileCIL(cil);
	dTreeAdressStmt& testExpressionA = cil.NewStatement()->GetInfo();
	testExpressionA.m_instruction = dTreeAdressStmt::m_if;
	testExpressionA.m_operator = logicOperation;
	testExpressionA.m_arg0 = m_expressionA->m_result;
	testExpressionA.m_arg1.m_type = dTreeAdressStmt::m_intConst;
	testExpressionA.m_arg1.m_label = "0";
	testExpressionA.m_arg2.m_label = exitLabel;
	DTRACE_INTRUCTION (&testExpressionA);

	m_expressionB->CompileCIL(cil);
	dTreeAdressStmt& testExpressionB = cil.NewStatement()->GetInfo();
	testExpressionB.m_instruction = dTreeAdressStmt::m_if;
	testExpressionB.m_operator = logicOperation;
	testExpressionB.m_arg0 = m_expressionB->m_result;
	testExpressionA.m_arg1.m_type = dTreeAdressStmt::m_intConst;
	testExpressionB.m_arg1.m_label = "0";
	testExpressionB.m_arg2.m_label = exitLabel;
	DTRACE_INTRUCTION (&testExpressionB);

	dTreeAdressStmt& stmt1 = cil.NewStatement()->GetInfo();
	stmt1.m_instruction = dTreeAdressStmt::m_assigment;
	stmt1.m_arg0 = m_result;
	stmt1.m_arg1.m_type = dTreeAdressStmt::m_intConst;
	stmt1.m_arg1.m_label = dString (complementBool);
	DTRACE_INTRUCTION (&stmt1);

	dCIL::dListNode* const jmpTarget = cil.NewStatement();
	dTreeAdressStmt& labelStmt = jmpTarget->GetInfo();
	labelStmt.m_instruction = dTreeAdressStmt::m_label;
	labelStmt.m_arg0.m_label = exitLabel;
	DTRACE_INTRUCTION (&labelStmt);

	testExpressionA.m_jmpTarget = jmpTarget;
	testExpressionB.m_jmpTarget = jmpTarget;
*/
}