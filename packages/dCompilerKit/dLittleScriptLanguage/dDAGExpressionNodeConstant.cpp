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
#include "dDAGFunctionNode.h"
#include "dDAGExpressionNodeConstant.h"


dInitRtti(dDAGExpressionNodeConstant);
dInitRtti(dDAGExpressionNodeOperatorThisConstant);

dDAGExpressionNodeConstant::dDAGExpressionNodeConstant(dList<dDAG*>& allNodes, dCIL::dIntrisicType type, const char* const value)
	:dDAGExpressionNode(allNodes)
	,m_type(type)
{
	m_name = value;
}


dDAGExpressionNodeConstant::~dDAGExpressionNodeConstant(void)
{
}


void dDAGExpressionNodeConstant::ConnectParent(dDAG* const parent) 
{
	m_parent = parent;
}

dCIL::dReturnValue dDAGExpressionNodeConstant::Evalue(dCIL& cil)
{
	dCIL::dReturnValue val;

	val.m_type = m_type;
	switch (m_type) 
	{
		case dCIL::m_int:
		{
			val.m_i = m_name.ToInteger();
			break;
		}

		case dCIL::m_float:
		case dCIL::m_double:
		{
			val.m_f = m_name.ToFloat();
			break;
		}

		case dCIL::m_classPointer:
		default:
			dAssert (0);
	}
	return val;
}

void dDAGExpressionNodeConstant::CompileCIL(dCIL& cil)
{
dAssert (0);
/*
	dTreeAdressStmt& stmt = cil.NewStatement()->GetInfo();
	stmt.m_instruction = dTreeAdressStmt::m_assigment;
	stmt.m_operator = dTreeAdressStmt::m_nothing;
	stmt.m_arg0.m_label = cil.NewTemp();

	switch (m_type) 
	{
		case m_intValue:
		{
			stmt.m_arg1.m_type = dTreeAdressStmt::m_intConst;
			break;
		}

		case m_floatValue:
		{
			dAssert (0);
			stmt.m_arg1.m_type = dTreeAdressStmt::m_floatConst;
			break;
		}

		case m_classPointer:
		{
			stmt.m_arg1.m_type = dTreeAdressStmt::m_classPointer;
			break;
		}

		default:
			dAssert (0);
	}

	stmt.m_arg1.m_label = m_name; 
	DTRACE_INTRUCTION (&stmt);
	m_result = stmt.m_arg0;
*/
}


dDAGExpressionNodeOperatorThisConstant::dDAGExpressionNodeOperatorThisConstant (dList<dDAG*>& allNodes)
	:dDAGExpressionNodeConstant(allNodes, dCIL::m_classPointer, "this")
{
}


void dDAGExpressionNodeOperatorThisConstant::CompileCIL(dCIL& cil)
{
	dAssert(0);
	/*

	dDAGFunctionNode* const function = GetFunction();
	m_result.m_type = dTreeAdressStmt::m_classPointer;
	m_result.m_label = function->m_opertatorThis;
	*/
}