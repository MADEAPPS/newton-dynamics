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
#include "dDAGExpressionNodeBinaryOperator.h"


dInitRtti(dDAGExpressionNodeBinaryOperator);

dDAGExpressionNodeBinaryOperator::dDAGExpressionNodeBinaryOperator(
	dList<dDAG*>& allNodes,
	dBinaryOperator binaryOperator, dDAGExpressionNode* const expressionA, dDAGExpressionNode* const expressionB)
	:dDAGExpressionNode(allNodes)
	,m_operator (binaryOperator)
	,m_expressionA (expressionA)
	,m_expressionB (expressionB)
{
}


dDAGExpressionNodeBinaryOperator::~dDAGExpressionNodeBinaryOperator(void)
{
}

void dDAGExpressionNodeBinaryOperator::ConnectParent(dDAG* const parent)  
{
	m_parent = parent;
	m_expressionA->ConnectParent(this);
	m_expressionB->ConnectParent(this);
}

void dDAGExpressionNodeBinaryOperator::CompileCIL(dCIL& cil)  
{
	m_expressionA->CompileCIL(cil);
	m_expressionB->CompileCIL(cil);

	dTreeAdressStmt& stmt = cil.NewStatement()->GetInfo();
	m_result.m_label = cil.NewTemp ();		

	stmt.m_instruction = dTreeAdressStmt::m_assigment;
	stmt.m_arg0 = m_result;
	stmt.m_arg1 = m_expressionA->m_result;
	stmt.m_arg2 = m_expressionB->m_result;

	switch (m_operator) 
	{
		case m_add:
		{
			stmt.m_operator = dTreeAdressStmt::m_add;
			break;
		}


		case m_sub:
		{
			stmt.m_operator = dTreeAdressStmt::m_sub;
			break;
		}

		case m_mul:
		{
			stmt.m_operator = dTreeAdressStmt::m_mul;
			break;
		}

		case m_div:
		{
			stmt.m_operator = dTreeAdressStmt::m_div;
			break;
		}

		case m_mod:
		{
			stmt.m_operator = dTreeAdressStmt::m_mod;
			break;
		}

		case m_identical:
		{
			stmt.m_operator = dTreeAdressStmt::m_identical;
			break;
		}

		case m_different:
		{
			stmt.m_operator = dTreeAdressStmt::m_different;
			break;
		}


		case m_less:
		{
			stmt.m_operator = dTreeAdressStmt::m_less;
			break;
		}

		case m_lessEqual:
		{
			stmt.m_operator = dTreeAdressStmt::m_lessEqual;
			break;
		}
		
		case m_greather:
		{
			stmt.m_operator = dTreeAdressStmt::m_greather;
			break;
		}
	
		case m_greatherEqual:
		{
			stmt.m_operator = dTreeAdressStmt::m_greatherEqual;
			break;
		}
		
		default:
			_ASSERTE (0);
	}

	DTRACE_INTRUCTION (&stmt);
}