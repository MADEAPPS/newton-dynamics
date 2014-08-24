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

//void dDAGExpressionNodeBinaryOperator::PromoteTypes (dCIL::dReturnValue& typeA, dCIL::dReturnValue& typeB) const
dThreeAdressStmt::dArgType dDAGExpressionNodeBinaryOperator::PromoteTypes (const dThreeAdressStmt::dArgType typeA, const dThreeAdressStmt::dArgType typeB) const
{
dAssert (0);
return dThreeAdressStmt::dArgType();
/*
	dThreeAdressStmt::dArgType type = typeA;
	if (typeA != typeB) {
		switch (typeA) 
		{
			case dThreeAdressStmt::m_constInt:
			{
				switch (typeB) 
				{
					case dThreeAdressStmt::m_int:
						type = dThreeAdressStmt::m_int;
						break;
					default:;
						dAssert (0);
				}
				break;
			}

			case dThreeAdressStmt::m_int:
			{
				switch (typeB) 
				{
					case dThreeAdressStmt::m_constInt:
						type = dThreeAdressStmt::m_int;
						break;

					default:;
						dAssert (0);
				}
				break;
			}

			default:;
				dAssert (0);
		}
	}

	switch (type) 
	{
		case dThreeAdressStmt::m_constInt:
			type = dThreeAdressStmt::m_int;
			break;

		case dThreeAdressStmt::m_int:
			break;

		default:;
			dAssert (0);
	}

	return type;
*/
}

dCIL::dReturnValue dDAGExpressionNodeBinaryOperator::Evalue(const dDAGFunctionNode* const function)
{
dAssert (0);
return dCIL::dReturnValue();
/*
	dCIL::dReturnValue operandA (m_expressionA->Evalue(function));
	dCIL::dReturnValue operandB (m_expressionB->Evalue(function));
	PromoteTypes (operandA, operandB);

	dCIL::dReturnValue val (operandA);	
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
	return val;
*/
}

void dDAGExpressionNodeBinaryOperator::CompileCIL(dCIL& cil)  
{
dAssert (0);
/*
	m_expressionA->CompileCIL(cil);
	dThreeAdressStmt::dArg arg1 (LoadLocalVariable(cil, m_expressionA->m_result));

	m_expressionB->CompileCIL(cil);
	dThreeAdressStmt::dArg arg2 (LoadLocalVariable(cil, m_expressionB->m_result));

	dThreeAdressStmt& stmt = cil.NewStatement()->GetInfo();
	m_result.m_label = cil.NewTemp ();		
	stmt.m_instruction = dThreeAdressStmt::m_assigment;
	stmt.m_arg0 = m_result;
	//stmt.m_arg0.m_type = m_expressionA->m_result.m_type;
	stmt.m_arg0.m_type = PromoteTypes (m_expressionA->m_result.m_type, m_expressionB->m_result.m_type);

	stmt.m_arg1 = arg1;
	stmt.m_arg2 = arg2;
	//dAssert (stmt.m_arg1.m_type == stmt.m_arg2.m_type);


	switch (m_operator) 
	{
		case m_add:
		{
			stmt.m_operator = dThreeAdressStmt::m_add;
			break;
		}


		case m_sub:
		{
			stmt.m_operator = dThreeAdressStmt::m_sub;
			break;
		}

		case m_mul:
		{
			stmt.m_operator = dThreeAdressStmt::m_mul;
			break;
		}

		case m_div:
		{
			stmt.m_operator = dThreeAdressStmt::m_div;
			break;
		}

		case m_mod:
		{
			stmt.m_operator = dThreeAdressStmt::m_mod;
			break;
		}

		case m_identical:
		{
			stmt.m_operator = dThreeAdressStmt::m_identical;
			break;
		}

		case m_different:
		{
			stmt.m_operator = dThreeAdressStmt::m_different;
			break;
		}


		case m_less:
		{
			stmt.m_operator = dThreeAdressStmt::m_less;
			break;
		}

		case m_lessEqual:
		{
			stmt.m_operator = dThreeAdressStmt::m_lessEqual;
			break;
		}
		
		case m_greather:
		{
			stmt.m_operator = dThreeAdressStmt::m_greather;
			break;
		}
	
		case m_greatherEqual:
		{
			stmt.m_operator = dThreeAdressStmt::m_greatherEqual;
			break;
		}
		
		default:
			dAssert (0);
	}

	DTRACE_INTRUCTION (&stmt);
*/
}