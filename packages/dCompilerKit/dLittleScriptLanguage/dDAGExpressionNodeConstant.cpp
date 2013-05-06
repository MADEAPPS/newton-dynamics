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


dInitRtti(dDAGExpressionNodeConstant);

dDAGExpressionNodeConstant::dDAGExpressionNodeConstant(dList<dDAG*>& allNodes, dType type, const char* const value)
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

void dDAGExpressionNodeConstant::CompileCIL(dCIL& cil)
{
	dTreeAdressStmt& stmt = cil.NewStatement()->GetInfo();
	stmt.m_instruction = dTreeAdressStmt::m_assigment;
	stmt.m_operator = dTreeAdressStmt::m_nothing;
	stmt.m_arg0.m_label = cil.NewTemp();
/*
	switch (m_type) 
	{
//		case m_intValue:
//			m_result.m_type = dTreeAdressStmt::m_intConst;
//			break;

		case m_floatValue:
			_ASSERTE (0)
			m_result.m_type = dTreeAdressStmt::m_floatConst;
			break;

		default:
			m_result.m_type = dTreeAdressStmt::m_intVar;

	}
*/
	if (m_type == m_floatValue) {
		_ASSERTE (0);
		m_result.m_type = dTreeAdressStmt::m_floatVar;
	}

	stmt.m_arg1.m_type = dTreeAdressStmt::m_intConst;
	stmt.m_arg1.m_label = m_name; 
	DTRACE_INTRUCTION (&stmt);

	m_result = stmt.m_arg0;
}