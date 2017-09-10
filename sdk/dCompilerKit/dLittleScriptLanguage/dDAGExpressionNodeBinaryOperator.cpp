/* Copyright (c) <2003-2016> <Newton Game Dynamics>
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
dCILInstr::dArgType dDAGExpressionNodeBinaryOperator::PromoteTypes (const dCILInstr::dArgType typeA, const dCILInstr::dArgType typeB) const
{
	dCILInstr::dArgType type (typeA);
	dAssert (!typeA.m_isPointer);
	dAssert (!typeB.m_isPointer);
	if (typeA.m_intrinsicType != typeB.m_intrinsicType) {
		switch (typeA.m_intrinsicType) 
		{
			case dCILInstr::m_constInt:
			{
				switch (typeB.m_intrinsicType) 
				{
					case dCILInstr::m_int:
						type.m_intrinsicType = dCILInstr::m_int;
						break;
					default:;
						dAssert (0);
				}
				break;
			}

			case dCILInstr::m_int:
			{
				switch (typeB.m_intrinsicType) 
				{
					case dCILInstr::m_constInt:
						type.m_intrinsicType = dCILInstr::m_int;
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

	switch (type.m_intrinsicType) 
	{
		case dCILInstr::m_constInt:
			type.m_intrinsicType = dCILInstr::m_int;
			break;

		case dCILInstr::m_int:
			break;

		default:;
			dAssert (0);
	}

	return type;

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
	m_expressionA->CompileCIL(cil);

	dCILInstr::dArg arg1 (LoadLocalVariable(cil, m_expressionA->m_result));
	m_expressionB->CompileCIL(cil);
	dCILInstr::dArg arg2 (LoadLocalVariable(cil, m_expressionB->m_result));

	dCILThreeArgInstr::dOperator operation = dCILThreeArgInstr::m_add;
	switch (m_operator) 
	{
		case m_add:
		{
			operation = dCILThreeArgInstr::m_add;
			break;
		}

		case m_sub:
		{
			operation = dCILThreeArgInstr::m_sub;
			break;
		}

		case m_mul:
		{
			operation = dCILThreeArgInstr::m_mul;
			break;
		}

		case m_div:
		{
			operation = dCILThreeArgInstr::m_div;
			break;
		}

		case m_mod:
		{
			operation = dCILThreeArgInstr::m_mod;
			break;
		}

		case m_identical:
		{
			operation = dCILThreeArgInstr::m_identical;
			break;
		}

		case m_different:
		{
			operation = dCILThreeArgInstr::m_different;
			break;
		}


		case m_less:
		{
			operation = dCILThreeArgInstr::m_less;
			break;
		}

		case m_lessEqual:
		{
			operation = dCILThreeArgInstr::m_lessEqual;
			break;
		}
		
		case m_greather:
		{
			operation = dCILThreeArgInstr::m_greather;
			break;
		}
	
		case m_greatherEqual:
		{
			operation = dCILThreeArgInstr::m_greatherEqual;
			break;
		}
		
		default:
			dAssert (0);
	}

	m_result.m_label = cil.NewTemp ();		
	dCILInstrIntergerLogical* const instr = new dCILInstrIntergerLogical(cil, operation, m_result.m_label, PromoteTypes (m_expressionA->m_result, m_expressionB->m_result), arg1.m_label, arg1.GetType(), arg2.m_label, arg2.GetType()); 
	instr->Trace();
}