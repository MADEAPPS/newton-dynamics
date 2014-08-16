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

void dDAGExpressionNodeBinaryOperator::PromoteTypes (dCIL::dReturnValue& typeA, dCIL::dReturnValue& typeB) const
{
dAssert (0);
/*
	if (typeA.m_type != typeB.m_type) {
		switch (typeA.m_type) 
		{
			case dCIL::m_int:
			{
				switch (typeB.m_type) 
				{
					case dCIL::m_double:
					{
						typeA.m_type = dCIL::m_double;
						typeA.m_f = typeA.m_i;
						break;
					}

					case dCIL::m_void:
					case dCIL::m_bool:
					case dCIL::m_byte:
					case dCIL::m_short:
					case dCIL::m_int:
					case dCIL::m_long:
					case dCIL::m_float:
					case dCIL::m_classPointer:
						dAssert (0);
						break;
				}

				break;
			}

			case dCIL::m_void:
			case dCIL::m_bool:
			case dCIL::m_byte:
			case dCIL::m_short:
			
			case dCIL::m_long:
			case dCIL::m_float:
			case dCIL::m_double:
			case dCIL::m_classPointer:
				dAssert (0);
				break;
		}
	}
*/
}

dCIL::dReturnValue dDAGExpressionNodeBinaryOperator::Evalue(dCIL& cil)
{
	dCIL::dReturnValue val;
dAssert (0);
/*
	dCIL::dReturnValue operandA (m_expressionA->Evalue(cil));
	dCIL::dReturnValue operandB (m_expressionB->Evalue(cil));
	PromoteTypes (operandA, operandB);

	val.m_type = operandA.m_type;
	switch (m_operator) 
	{
		case m_add:
		{
			dAssert (0);
			break;
		}

		case m_sub:
		{
			dAssert (0);
			break;
		}

		case m_mul:
		{
			switch (operandA.m_type) 
			{
				case dCIL::m_float:
				case dCIL::m_double:
				{
					val.m_f = operandA.m_f * operandB.m_f;
					break;
				}

				case dCIL::m_void:
				case dCIL::m_bool:
				case dCIL::m_byte:
				case dCIL::m_short:
				case dCIL::m_int:
				case dCIL::m_long:
				case dCIL::m_classPointer:
					dAssert (0);
					break;
			}

			
			break;
		}

		case m_div:
		{
			dAssert (0);
			break;
		}

		case m_mod:
		{
			dAssert (0);
			break;
		}

		case m_identical:
		{
			dAssert (0);
			break;
		}

		case m_different:
		{
			dAssert (0);
			break;
		}


		case m_less:
		{
			dAssert (0);
			break;
		}

		case m_lessEqual:
		{
			dAssert (0);
			break;
		}

		case m_greather:
		{
			dAssert (0);
			break;
		}

		case m_greatherEqual:
		{
			dAssert (0);
			break;
		}

		default:
			dAssert (0);
	}
*/
	return val;
}

void dDAGExpressionNodeBinaryOperator::CompileCIL(dCIL& cil)  
{
	m_expressionA->CompileCIL(cil);
	dTreeAdressStmt::dArg arg1 (LoadLocalVariable(cil, m_expressionA->m_result));

	m_expressionB->CompileCIL(cil);
	dTreeAdressStmt::dArg arg2 (LoadLocalVariable(cil, m_expressionB->m_result));

	dTreeAdressStmt& stmt = cil.NewStatement()->GetInfo();
	m_result.m_label = cil.NewTemp ();		
	stmt.m_instruction = dTreeAdressStmt::m_assigment;
	stmt.m_arg0 = m_result;
	stmt.m_arg0.m_type = m_expressionA->m_result.m_type;

	stmt.m_arg1 = arg1;
	stmt.m_arg2 = arg2;
	//dAssert (stmt.m_arg1.m_type == stmt.m_arg2.m_type);


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
			dAssert (0);
	}

	DTRACE_INTRUCTION (&stmt);
}